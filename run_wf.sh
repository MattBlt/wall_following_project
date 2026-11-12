#!/usr/bin/env bash
#
# Esegue una serie di prove di wall following a velocita' nominali diverse,
# ripetute piu' volte, per la ripetibilita' del giro (fase 1).
#
#   ./run_wf.sh 0.2 0.4 0.6
#   RIPETIZIONI=1 GIRI=1 ./run_wf.sh 0.6      # prova corta
#
# Per ogni velocita' e ripetizione: sim pulita (headless), attesa del primo
# messaggio vero su /wall_info, controllo che esista un solo controllore,
# impostazione di v_nominal, registrazione, abilitazione per la durata
# stimata dei giri, chiusura, esportazione in CSV.
#
# Le prove finiscono in analisi/dati/wf_v<vvv>_<kk>/, il log in
# analisi/dati/wf_v<vvv>_<kk>.log   (es. wf_v040_01 = 0.40 m/s, prova 1)

set +u    # gli script di setup di ROS usano variabili non definite.
          # NIENTE 'set -m': con il job control un processo in background
          # che tocca il terminale viene sospeso (SIGTTIN), e il recorder
          # resterebbe vivo senza registrare nulla.

WS="${WS:-$HOME/ros2_ws/wall_following_project}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
TIMEOUT_AVVIO=90        # s per vedere il primo messaggio su /wall_info
GRAZIA=10               # s concessi a un processo per chiudersi da solo

RIPETIZIONI="${RIPETIZIONI:-3}"
GIRI="${GIRI:-3}"
GIRO_A_040=105          # s per un giro a v_nominal = 0.40 m/s
MARGINE=10              # s in piu' sulla durata stimata

CONTROLLORE=/wall_follow_controller
NODO="limo_wf/wall_follow_controller"     # percorso dell'eseguibile, per pgrep
TOPIC="/scan /wall_info /cmd_vel /odom /odom_noisy"
DATI=analisi/dati

cd "$WS" || { echo "workspace non trovato: $WS"; exit 1; }

source "$ROS_SETUP"
source "$WS/install/setup.bash"

mkdir -p "$DATI"

# Ammazza tutto quello che una prova puo' lasciare in giro, e aspetta che
# sia sparito davvero.  Ferma anche il daemon di ros2: tiene in memoria il
# grafo della prova precedente, compresi i nodi gia' morti.
pulisci() {
    pkill -9 gzserver                 >/dev/null 2>&1
    pkill -9 gzclient                 >/dev/null 2>&1
    pkill -9 -f "gz camera"           >/dev/null 2>&1
    pkill -9 -f "$NODO"               >/dev/null 2>&1
    pkill -9 -f odom_noise            >/dev/null 2>&1
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

# Vero se esiste esattamente un processo con quel nome
unico() {
    local n
    n=$(pgrep -c -f "$1" 2>/dev/null)
    [ "${n:-0}" -eq 1 ]
}

trap 'echo; echo "interrotto"; ferma "$bag_pid"; ferma "$launch_pid"; pulisci; exit 130' INT TERM

if [ "$#" -eq 0 ]; then
    echo "uso: $0 <v_nominal> [<v_nominal> ...]     es. $0 0.2 0.4 0.6"
    echo "     RIPETIZIONI=1 GIRI=1 $0 0.6     per una prova corta"
    exit 1
fi

for v in "$@"; do
    # double esplicito: 'ros2 param set ... 1' manderebbe un intero e il
    # parametro, dichiarato float, verrebbe rifiutato.
    v=$(awk -v v="$v" 'BEGIN { printf "%.2f", v }')

    for k in $(seq 1 "$RIPETIZIONI"); do

        # --- nome, percorsi, durata
        vc=$(awk -v v="$v" 'BEGIN { printf "%03d", v*100 + 0.5 }')
        nome=$(printf 'wf_v%s_%02d' "$vc" "$k")
        dest="$DATI/$nome"                 # la cartella la creera' il recorder
        log="$DATI/$nome.log"              # sciolto in dati/, come nella fase 0
        # il giro dura GIRO_A_040 s a 0.40 m/s e scala come 1/v
        durata=$(awk -v v="$v" -v g="$GIRI" -v t="$GIRO_A_040" -v m="$MARGINE" \
                 'BEGIN { printf "%d", g * t * 0.40 / v * 1.3 + m + 0.5 }')

        if [ -e "$dest" ]; then
            echo "[$nome] esiste gia', salto"
            continue
        fi

        # --- pulizia e lancio
        echo "=== $nome (v_nominal $v, ${durata} s) ==="
        pulisci
        : > "$log"
        {
            echo "data:     $(date '+%F %T')"
            echo "commit:   $(git rev-parse --short HEAD)"
            echo "v_nom:    $v m/s   durata: ${durata} s   giri attesi: $GIRI"
        } >>"$log"
        if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
            echo "ATTENZIONE: albero sporco, modifiche non committate" | tee -a "$log"
        fi
        ros2 launch limo_wf_bringup sim.launch.py \
            headless:=True noise_seed:="$k" >>"$log" 2>&1 </dev/null &
        launch_pid=$!

        # --- attesa del primo /wall_info
        # Si aspetta un MESSAGGIO, non la semplice presenza del topic: un nodo
        # fantasma nel grafo fa comparire il topic, ma non pubblica niente.
        echo -n "    avvio"
        inizio=$SECONDS
        until timeout 5 ros2 topic echo --once /wall_info >/dev/null 2>&1 </dev/null; do
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

        # --- guardia contro i nodi orfani: si contano i PROCESSI
        for p in "$NODO" wall_detection odom_noise; do
            if ! unico "$p"; then
                echo "    $p: non esattamente un processo, prova annullata"
                pgrep -af "$p" 2>/dev/null
                ferma "$launch_pid"; pulisci
                exit 1
            fi
        done

        # --- velocita' nominale (il nodo rilegge i parametri a caldo)
        if ! ros2 param set "$CONTROLLORE" v_nominal "$v" >>"$log" 2>&1 </dev/null; then
            echo "    impossibile impostare v_nominal:"
            tail -5 "$log"
            ferma "$launch_pid"; pulisci
            exit 1
        fi
        letto=$(ros2 param get "$CONTROLLORE" v_nominal 2>/dev/null </dev/null | awk '{ print $NF }')
        if ! awk -v a="$letto" -v b="$v" 'BEGIN { exit !(a == b) }'; then
            echo "    v_nominal letto = '$letto' invece di $v: prova annullata"
            ferma "$launch_pid"; pulisci
            exit 1
        fi
        echo "    v_nominal = $letto" | tee -a "$log"
        # --- registrazione
        # shellcheck disable=SC2086   # TOPIC va spezzato in piu' argomenti
        ros2 bag record -o "$dest" $TOPIC >>"$log" 2>&1 </dev/null &
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

        # --- configurazione usata davvero: la copia INSTALLATA, non quella in src/
        cp "$(ros2 pkg prefix limo_wf_bringup)"/share/limo_wf_bringup/config/wall_*.yaml "$dest/"

        # --- prova
        echo "    wall following per ${durata} s..."
        if ! ros2 param set "$CONTROLLORE" enabled true >>"$log" 2>&1 </dev/null; then
            echo "    impossibile abilitare il controllore:"
            tail -5 "$log"
            ferma "$bag_pid"; ferma "$launch_pid"; pulisci
            exit 1
        fi
        sleep "$durata"
        ros2 param set "$CONTROLLORE" enabled false >>"$log" 2>&1 </dev/null
        sleep 2            # il robot si ferma dentro il bag

        ferma "$bag_pid"
        ferma "$launch_pid"
        pulisci

        echo "    esportazione"
        ros2 run limo_wf_sim export_bag.py "$dest"
        echo
    done
done
echo "tutte le prove completate. Analisi:"
echo "  cd analisi && env -u PYTHONPATH uv run criterio_uscita.py dati/wf_v*/ --giri $GIRI --msg ../src/limo_wf_msgs/msg/WallInfo.msg"