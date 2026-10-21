#!/usr/bin/env bash
#
# Esegue una serie di prove del quadrato con seed diversi, per la
# ripetibilita' della deriva odometrica (criterio di uscita fase 0).
#
#   ./run_prove.sh 1 2 3 4 5
#
# Per ogni seed: sim pulita (headless), attesa del primo messaggio vero
# su /odom_noisy, controllo che esista un solo nodo odom_noise,
# registrazione, percorso, chiusura, esportazione in CSV.
#
# Le prove finiscono in analisi/dati/quadrato_<seed>/, il log in
# analisi/dati/quadrato_<seed>.log

set +u    # gli script di setup di ROS usano variabili non definite.
          # NIENTE 'set -m': con il job control un processo in background
          # che tocca il terminale viene sospeso (SIGTTIN), e il recorder
          # resterebbe vivo senza registrare nulla.

WS="${WS:-$HOME/ros2_ws/wall_following_project}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
TIMEOUT_AVVIO=90        # s per vedere il primo messaggio su /odom_noisy
TIMEOUT_PERCORSO=300    # s oltre i quali il percorso e' considerato piantato
GRAZIA=10               # s concessi a un processo per chiudersi da solo
NODO="limo_wf_sim/odom_noise.py"

cd "$WS" || { echo "workspace non trovato: $WS"; exit 1; }

source "$ROS_SETUP"
source "$WS/install/setup.bash"

# Ammazza tutto quello che una prova puo' lasciare in giro, e aspetta che
# sia sparito davvero.  Ferma anche il daemon di ros2: tiene in memoria il
# grafo della prova precedente, compresi i nodi gia' morti, e farebbe
# credere che la nuova simulazione sia pronta quando non lo e'.
pulisci() {
    pkill -9 gzserver                 >/dev/null 2>&1
    pkill -9 gzclient                 >/dev/null 2>&1
    pkill -9 -f "gz camera"           >/dev/null 2>&1
    pkill -9 -f "$NODO"               >/dev/null 2>&1
    pkill -9 -f twist_watchdog        >/dev/null 2>&1
    pkill -9 -f square_drive_test     >/dev/null 2>&1
    pkill -9 -f wall_detection        >/dev/null 2>&1
    pkill -9 -f robot_state_publisher >/dev/null 2>&1
    pkill -9 -f joint_state_publisher >/dev/null 2>&1
    local t=0
    while pgrep -f "$NODO" >/dev/null 2>&1; do
        sleep 1
        t=$((t + 1))
        [ "$t" -ge 10 ] && break
    done
    ros2 daemon stop >/dev/null 2>&1
    sleep 2
}

# SIGINT, attesa limitata, poi SIGKILL: senza limite 'ros2 launch' puo'
# non uscire mai e lo script resterebbe appeso.
ferma() {
    local pid=$1 t=0
    kill -INT "$pid" 2>/dev/null || return 0
    while kill -0 "$pid" 2>/dev/null; do
        sleep 1
        t=$((t + 1))
        if [ "$t" -ge "$GRAZIA" ]; then
            kill -9 "$pid" 2>/dev/null
            break
        fi
    done
    wait "$pid" 2>/dev/null
}

trap 'echo; echo "interrotto"; pulisci; exit 130' INT TERM

if [ "$#" -eq 0 ]; then
    echo "uso: $0 <seed> [<seed> ...]"
    exit 1
fi

for seed in "$@"; do
    nome="quadrato_$(printf '%02d' "$seed")"
    dest="analisi/dati/$nome"
    log="analisi/dati/$nome.log"

    if [ -e "$dest" ]; then
        echo "[$nome] esiste gia', salto"
        continue
    fi

    echo "=== $nome (noise_seed $seed) ==="
    pulisci
    : > "$log"

    ros2 launch limo_wf_bringup sim.launch.py \
        noise_seed:="$seed" headless:=True >>"$log" 2>&1 </dev/null &
    launch_pid=$!

    # Si aspetta un MESSAGGIO, non la semplice presenza del topic: un nodo
    # fantasma nel grafo fa comparire il topic, ma non pubblica niente.
    echo -n "    avvio"
    inizio=$SECONDS
    until timeout 5 ros2 topic echo --once /odom_noisy >/dev/null 2>&1 </dev/null; do
        echo -n "."
        sleep 1
        if [ $((SECONDS - inizio)) -ge "$TIMEOUT_AVVIO" ]; then
            echo " la sim non e' partita"
            tail -15 "$log"
            ferma "$launch_pid"; pulisci
            exit 1
        fi
    done
    sleep 4            # margine perche' il robot si assesti a terra
    echo " ok"

    # Guardia contro i nodi orfani: si contano i PROCESSI, che non mentono.
    n_proc=$(pgrep -c -f "$NODO" 2>/dev/null)
    n_proc=${n_proc:-0}
    if [ "$n_proc" -ne 1 ]; then
        echo "    $n_proc nodi odom_noise attivi invece di 1: prova annullata"
        pgrep -af "$NODO" 2>/dev/null
        ferma "$launch_pid"; pulisci
        exit 1
    fi

    ros2 bag record -o "$dest" /odom /odom_noisy >>"$log" 2>&1 </dev/null &
    bag_pid=$!

    t=0
    while [ ! -d "$dest" ]; do
        sleep 1
        t=$((t + 1))
        if [ "$t" -ge 10 ]; then
            echo "    il recorder non e' partito:"
            tail -10 "$log"
            ferma "$bag_pid"; ferma "$launch_pid"; pulisci
            exit 1
        fi
    done
    sleep 2

    echo "    percorso in corso..."
    if ! timeout "$TIMEOUT_PERCORSO" \
            ros2 run limo_wf_sim square_drive_test.py >>"$log" 2>&1 </dev/null; then
        echo "    percorso fallito o scaduto:"
        tail -8 "$log"
    fi
    sleep 1

    ferma "$bag_pid"
    ferma "$launch_pid"
    pulisci

    echo "    esportazione"
    ros2 run limo_wf_sim export_bag.py "$dest"
    echo
done

echo "tutte le prove completate."
echo "verifica:  wc -l analisi/dati/quadrato_*/*_odom*.csv"
echo "i due file di ogni prova devono avere lo stesso numero di righe."