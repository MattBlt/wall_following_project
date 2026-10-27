import math
import numpy as np
from limo_wf.wall_geometry_detection import DetectionParams, detect_wall, SIDE_LEFT

# corridoio: muro destro a y=-0.5, sinistro a y=+0.5, frontale a x=2
pts = []
for x in np.arange(-1.0, 2.001, 0.01):
    pts += [(x, -0.5), (x, 0.5)]
for y in np.arange(-0.5, 0.501, 0.01):
    pts.append((2.0, y))

pts = np.array(pts)
pts = pts[np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))]      # ordine di scansione
pts += np.random.default_rng(0).normal(0, 0.005, pts.shape)  # rumore 5 mm

e = detect_wall(pts, DetectionParams(side=SIDE_LEFT))
print(e.detected, round(e.distance, 3), round(math.degrees(e.angle), 2),
      round(e.front_distance, 3), e.inlier_count, round(e.fit_residual, 4))