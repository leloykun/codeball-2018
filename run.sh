SECONDS=0

GAME_MODE=${1:-play}
IP_ADDRESS=${2:-127.0.0.1}
DURATION=${3:-18000}
P1_STRATEGY=${4:-cpp-cgdk/versions/CesistaStrategy_v47}
P2_STRATEGY=${5:-cpp-cgdk/build/MyStrategy}
P1_NAME=${6:-"Cesista's Strategy"}
P2_NAME=${7:-"Current Strategy"}
P1_PORT=${8:-31001}
P2_PORT=${9:-31002}
P1_KEY=${10:-0000000000000000}
P2_KEY=${11:-0000000000000000}

echo $GAME_MODE
if [[ $GAME_MODE == "auto" ]]
then
  codeball2018-linux/codeball2018 --p1-name "$P1_NAME" --p2-name "$P2_NAME" \
  --p1 tcp-$P1_PORT --p2 tcp-$P2_PORT --results-file results/run_result.txt \
  --duration $DURATION --no-countdown --nitro true &
  sleep 1; $P1_STRATEGY $IP_ADDRESS $P1_PORT $P1_KEY &
  sleep 1; $P2_STRATEGY $IP_ADDRESS $P2_PORT $P2_KEY
elif [[ $GAME_MODE == "play" ]]
then
  codeball2018-linux/codeball2018 --p1-name "Player" --p2-name "$P1_NAME" \
  --p1 keyboard --p2 tcp-$P1_PORT --results-file results/run_result.txt \
  --duration $DURATION --nitro true &
  sleep 1; $P1_STRATEGY $IP_ADDRESS $P1_PORT $P1_KEY
elif [[ $GAME_MODE == "helper" ]]
then
  codeball2018-linux/codeball2018 --p1-name "Player" --p2-name "Helper" \
  --p1 keyboard --p2 helper --results-file results/run_result.txt \
  --duration $DURATION --nitro true
elif [[ $GAME_MODE == "empty" ]]
then
  codeball2018-linux/codeball2018 --p1-name "Player" --p2-name "" \
  --p1 keyboard --p2 empty --results-file results/run_result.txt \
  --duration $DURATION --nitro true
else
  echo "ERROR: Invalid game mode!"
  exit 1
fi

wait
echo DONE!

echo "RESULTS:"
readarray -t RESFILE < codeball2018-linux/results/run_result.txt
IFS=':' read -ra P1_res <<< "${RESFILE[0]}"
IFS=':' read -ra P2_res <<< "${RESFILE[1]}"
let P1_scores=$((0 + P1_res[1]))
let P2_scores=$((0 + P2_res[1]))
if ((${RESFILE[0]:0:1} == "1" && ${RESFILE[1]:0:1} == "2"))
then
  echo "$P1_NAME WINS!"
elif ((${RESFILE[0]:0:1} == "2" && ${RESFILE[1]:0:1} == "1"))
then
  echo "$P2_NAME WINS!"
else
  echo "DRAW!"
fi
echo "SCORES: $P1_NAME $P1_scores - $P2_NAME $P2_scores"

echo "time taken: $SECONDS secs"

# Cleanup
rm -- codeball2018-linux/results/run_result.txt
