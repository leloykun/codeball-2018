SECONDS=0
echo "BENCHMARKING STARTED..."

NUM_CORES=$(grep -c ^processor /proc/cpuinfo)

BATCHES=${1:-1}
IP_ADDRESS=${2:-127.0.0.1}
DURATION=${3:-18000}
P1_STRATEGY=${4:-cpp-cgdk/versions/CesistaStrategy_v47}
P2_STRATEGY=${5:-cpp-cgdk/build/MyStrategy}
P1_NAME=${6:-"Cesista's Strategy"}
P2_NAME=${7:-"Current Strategy"}

echo "$BATCHES batches on $NUM_CORES cores"


rm benchmark_results.txt
# Prepare results directory
if [ ! -f codeball2018-linux/results ]; then
  echo "Creating results directory"
  mkdir codeball2018-linux/results
fi  
rm -- codeball2018-linux/results/result*.txt


let P1_wins=0
let P2_wins=0
let P1_scores=0
let P2_scores=0
for ((batch = 0; batch < $BATCHES; batch++)) do
  echo "BATCH $batch STARTED..."
  for ((core = 0; core < $NUM_CORES; core++)) do
    let game=$NUM_CORES*$batch+core
    let a=2*$game
    let b=2*$game+1
    let port_1=3100+$a
    let port_2=3100+$b
    codeball2018-linux/codeball2018 \
      --duration $DURATION \
      --p1-name "$P1_NAME" --p1 tcp-$port_1 \
      --p2-name "$P2_NAME" --p2 tcp-$port_2 \
      --results-file results/result_$game.txt --no-countdown \
      --noshow --nitro true &
    sleep 0.5; $P1_STRATEGY $IP_ADDRESS $port_1 $a &
    sleep 0.5; $P2_STRATEGY $IP_ADDRESS $port_2 $b &
    # '
    done
  wait

  # Aggregate results
  echo "RAW GAME RESULTS:"
  let batch_P1_wins=0
  let batch_P2_wins=0
  let batch_P1_scores=0
  let batch_P2_scores=0
  for ((game = 0; game < $(((batch+1)*NUM_CORES)); game++)) do
    readarray -t RESFILE < codeball2018-linux/results/result_$game.txt
    echo ${RESFILE[@]}

    IFS=':' read -ra P1_res <<< "${RESFILE[0]}"
    IFS=':' read -ra P2_res <<< "${RESFILE[1]}"
    # echo ${P1_res[@]}
    # echo ${P2_res[@]}

    let batch_P1_scores=$((batch_P1_scores+P1_res[1]))
    let batch_P2_scores=$((batch_P2_scores+P2_res[1]))

    if ((${RESFILE[0]:0:1} == "1" && ${RESFILE[1]:0:1} == "2"))
    then
      let batch_P1_wins=$batch_P1_wins+1
    elif ((${RESFILE[0]:0:1} == "2" && ${RESFILE[1]:0:1} == "1"))
    then
      let batch_P2_wins=$batch_P2_wins+1
    fi
  done
  let P1_wins=$P1_wins+$batch_P1_wins
  let P2_wins=$P2_wins+$batch_P2_wins
  let P1_scores=$P1_scores+$batch_P1_scores
  let P2_scores=$P2_scores+$batch_P2_scores

  # Display batch results
  echo "BATCH $batch RESULTS:"                                             | tee -a benchmark_results.txt
  echo "P1 || $P1_NAME || WINS: $batch_P1_wins || SCORE: $batch_P1_scores" | tee -a benchmark_results.txt
  echo "P2 || $P2_NAME || WINS: $batch_P2_wins || SCORE: $batch_P2_scores" | tee -a benchmark_results.txt

  echo "BATCH $batch DONE."
  echo "time taken: $SECONDS secs"
  done

# Display overall results
echo "OVERALL RESULTS:"                                      | tee -a benchmark_results.txt
echo "P1 || $P1_NAME || WINS: $P1_wins || SCORE: $P1_scores" | tee -a benchmark_results.txt
echo "P2 || $P2_NAME || WINS: $P2_wins || SCORE: $P2_scores" | tee -a benchmark_results.txt

echo "BENCHMARKING DONE."
echo "time taken: $SECONDS secs"


# Cleanup
rm -- codeball2018-linux/results/result*.txt
