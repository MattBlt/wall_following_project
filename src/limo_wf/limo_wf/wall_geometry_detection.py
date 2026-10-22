from __future__ import annotations

import math
from typing import Sequence, Tuple, List, Optional
from dataclasses import dataclass, field

import numpy as np


SIDE_LEFT = 0
SIDE_RIGHT = 1


@dataclass
class DetectionParams:
    """Parametri del rilevamento. I default servono ai test: sul robot
    arrivano dal YAML tramite il nodo."""

    side: int = SIDE_RIGHT                          # lato seguito

    # settore laterale da cui estrarre i segmenti (centrato a +-90 gradi)
    sector_half_width: float = math.radians(60.0)   # side_sector_mask

    # segmentazione
    gap_threshold: float = 0.20                     # split_runs            [m]
    split_threshold: float = 0.05                   # split_segments        [m]
    merge_threshold: float = 0.025                   # merge_collinear       [m]
    min_points: int = 8                             # split_segments
    min_segment_length: float = 0.30                # scarta segmenti corti [m]

    # plausibilita' della parete seguita
    max_wall_distance: float = 2.0                  # select_wall           [m]
    max_wall_angle: float = math.radians(60.0)      # select_wall           [rad]

    # corridoio frontale: LIMO largo ~0.22 m
    front_half_width: float = 0.20                  # front_clearance       [m]


@dataclass
class Segment:
    """Retta fittata a un gruppo di punti consecutivi e collineari, con i dati necessari per il resto della logica"""
    points: np.ndarray              # (N, 2) i punti del segmento
    normal: np.ndarray              # versore normale del segmento, dall'origine verso esso
    offset: float                   # distanza dall'origine del segmento (>=0)
    angle: float                    # orientamento del segmento (-pi/2, pi/2)
    length: float                   # lunghezza del segmento
    residual: float                 # RMS degli scarti perpendicolari
    p_start: np.ndarray             # punto iniziale del segmento
    p_end: np.ndarray               # punto finale del segmento

    @property
    def n_points(self) -> int:
        return int(self.points.shape[0])


@dataclass
class WallEstimate:
    """Risultato di detect_wall: cosa il nodo mette nel WallInfo."""
    detected: bool
    distance: float = float('nan')
    angle: float = float('nan')
    front_distance: float = float('inf')
    segment_length: float = 0.0
    inlier_count: int = 0
    fit_residual: float = float('nan')
    wall: Optional[Segment] = None
    segments: List[Segment] = field(default_factory=list)


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float, 
                   range_min: float, range_max : float, 
                    laser_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
                    ) -> np.ndarray:
    """
        Range validi -> punti (N, 2) nel frame base_link, nell'ordine di scansione.
        
        'laser_pose' = (x, y, yaw) del frame del laser in base_link (TF statica).
        Le lettere inf, nan o fuori da [range_min, range_max] vengono scartate
    """

    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)

    valid = np.isfinite(r) & (r >= max(range_min, 1e-3)) & (r <=range_max)
    r, a = r[valid], a[valid]

    # conversione da coordinate polari -> cartesiane nel frame del laser
    x1, y1 = r * np.cos(a), r * np.sin(a)

    # trasformazione delle coordiante laser -> base_link
    lx, ly, lyaw = laser_pose
    c, s = math.cos(lyaw), math.sin (lyaw)
    return np. column_stack((c * x1 - s *y1 + lx,
                            s * x1 + c *y1 + ly))


def side_sector_mask(points: np.ndarray, side: int, half_width: float) -> np.ndarray:
    """Maschera dei punti il cui azimut cade nel settore laterale del lato seguito"""

    center = -math.pi / 2 if side == SIDE_RIGHT else math.pi / 2
    bearing = np.arctan2(points[:, 1], points[:, 0])
    diff = np.arctan2(np.sin(bearing - center), np.cos(bearing - center))
    return np.abs(diff) <= half_width


def split_runs(points: np.ndarray, gap_threshold: float) -> list[Tuple[int, int]]:
    """Intervalli [i, j] di punti consecutivi senza varchi più larghi della soglia"""

    if len(points) == 0:
        return []
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cuts = np.nonzero(steps > gap_threshold)[0] + 1
    bounds = np.concatenate(([0], cuts, [len(points)]))
    return [(int(bounds[k]), int(bounds[k + 1])) for k in range(len(bounds) - 1)]


def _max_chord_deviation(pts: np.ndarray) -> Tuple[int, float]:
    """Indice e valore dello scarto massimo dalla corda fra il primo e l'ultimo punto"""

    p0, p1 = pts[0], pts[-1]
    d = p1 -p0
    norm = math.hypot(d[0], d[1])
    if norm < 1e-9:
        dev = np.linalg.norm(pts - p0, axis=1)
    else:
        dev = np.abs(d[0] * (pts[:, 1] -p0[1]) - d[1] * (pts[:, 0] - p0[0])) / norm
    k = int(np.argmax(dev))
    return k, float(dev[k])


def split_segments(pts: np.ndarray, threshold: float, min_points: int
                   ) -> List[Tuple[int, int]]:
    """Iterative End Point Fit: intervalli [i, j) rettilinei entro la soglia"""

    min_points = max(min_points, 2)
    n = len(pts)
    if n < min_points:
        return []

    stack, pieces = [(0, n)], []
    while stack:
        i, j = stack.pop()
        if j -i < min_points:
            continue
        k, dev = _max_chord_deviation(pts[i:j])
        if dev > threshold and 0 < k < (j - i - 1):
            stack.append((i + k, j))
            stack.append((i, i + k + 1))
        else:
            pieces.append((i, j))
    pieces.sort()
    return pieces


def fit_line(pts: np.ndarray) -> Segment:
    """Retta ai minimi quadrati totali (minimizza gli scarti perpendicolari)"""

    if len(pts) < 2:
        raise ValueError(f'fit_line richiede almeno 2 punti, ricevuti {len(pts)}') 
    
    centroid = pts.mean(axis=0)
    centered = pts - centroid

    # direzione di minima dispersione = normale alla retta
    cov = centered.T @ centered / len(pts)
    _, eigvec = np.linalg.eigh(cov)
    normal = eigvec[:, 0]
    
    # convenzione: offset >=0, quindi la normale punta dal robot verso il muro
    offset = float(normal @ centroid)
    if offset < 0:
        normal, offset = -normal, -offset

    # tangente canonica (sempre "in avanti") -> angolo in [-pi/2, pi/2]
    eps = 1e-9
    tangent = np.array([-normal[1], normal[0]])
    if tangent[0] < -eps or (abs(tangent[0]) <= eps and tangent[1] < 0):
        tangent = -tangent
    angle = math.atan2(tangent[1], tangent[0])

    proj = centered @ tangent
    perp = centered @ normal

    return Segment(
        points=pts,
        normal=normal,
        offset=offset,
        angle=angle,
        length=float(proj.max() - proj.min()),
        residual=float(math.sqrt(np.mean(perp ** 2 ))),
        p_start=centroid + float(proj[0]) * tangent,
        p_end=centroid + float(proj[-1]) * tangent,
    )


def merge_collinear(pts: np.ndarray, pieces: List[Tuple[int, int]], threshold: float
                    ) -> List[Tuple[int, int]]:
    """Rifonde i pezzi adiacenti che una retta sola spiega entrambi.

    Il criterio e' l'RMS degli scarti dalla retta comune, non il punto
    peggiore: il massimo cresce con il numero di punti (come sqrt(2 ln N)),
    quindi una soglia fissa sarebbe troppo stretta sui segmenti lunghi.
    L'RMS invece vale circa il rumore del sensore, qualunque sia la lunghezza
    """

    merged: List[Tuple[int, int]] = []
    for piece in pieces:
        if merged and piece[0] <= merged[-1][1]:
            i, j = merged[-1][0], piece[1]
            if fit_line(pts[i:j]).residual <= threshold:
                merged[-1] = (i, j)
                continue
        merged.append(piece)
    return merged


def extract_segments(points: np.ndarray, params: DetectionParams) -> List[Segment]:
    """Tutti i segmenti rettilinei validi in un insieme ordinato di punti"""

    segments = []
    for i, j in split_runs(points, params.gap_threshold):
        run = points[i:j]
        pieces = split_segments(run, params.split_threshold, params.min_points)
        for a, b in merge_collinear(run, pieces, params.merge_threshold):
            seg = fit_line(run[a:b])
            if seg.n_points >= params.min_points and seg.length >= params.min_segment_length:
                segments.append(seg)
    return segments


def point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Distanza di un punto dal segmento finito [a, b]"""

    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom < 1e-12 else min(1.0, max(0.0, float((p - a) @ ab) / denom))
    return float(np.linalg.norm(p - (a + t * ab)))


def select_wall(segmants: Sequence[Segment], params: DetectionParams) -> Optional[Segment]:
    """La parete seguita: fra i segmenti plausibili, quello più vicino al robot"""

    origin = np.zeros(2)
    want_sign = -1.0 if params.side == SIDE_RIGHT else 1.0
    best, best_d = None, math.inf
    for seg in segmants:
        if seg.normal[1] * want_sign<= 0:
            continue
        if  abs(seg.angle) > params.max_wall_angle:
            continue
        if seg.offset > params.max_wall_distance:
            continue
        d = point_segment_distance(origin, seg.p_start, seg.p_end)
        if d < best_d:
            best, best_d = seg, d
    return best


def front_clearance(points: np.ndarray, half_width: float) -> float:
    """Distanza in x del primo ostacolo nel corridoio |y| <= half_width davanti al robot"""

    if len(points) == 0:
        return math.inf
    ahead = points[(points[:, 0] > 0.0) & (np.abs(points[:, 1]) <= half_width)]
    return float(ahead[:, 0].min()) if len(ahead) else math.inf


def detect_wall(points: np.ndarray, params: DetectionParams) -> WallEstimate:
    """Dai punti della scansione (in base_link) alla stima della parete seguita"""

    front = front_clearance(points, params.front_half_width)
    lateral = points[side_sector_mask(points, params.side, params.sector_half_width)]
    segments = extract_segments(lateral, params)
    wall = select_wall(segments, params)
    if wall is None:
        return WallEstimate(detected=False, front_distance=front, segments=segments)
    return WallEstimate(
        detected=True,
        distance=wall.offset,
        angle=wall.angle,
        front_distance=front,
        segment_length=wall.length,
        inlier_count=wall.n_points,
        fit_residual=wall.residual,
        wall=wall,
        segments=segments,
    )


def angle_diff(a: float, b: float) -> float:
    """Differenza fra orientamenti di rette, ridotta in (-pi/2, pi/2).
    
    Serve perché una retta è definta modulo pi: +89.9 e -89.9 gradi distano solo 0.2 gradi, non 179.8
    """

    return ((a - b + math.pi / 2) % math.pi) - math.pi / 2