SECONDS=0

rm -- codeball2018-linux/result*.txt
rm -- codeball2018-linux/game*.log

NUM_CORES=4

BATCHES=${1:-1}
DURATION=${2:-18000}
VERSION_P1=${3:-35_nodebug}
VERSION_P2=${4:-36_nodebug}

for ((batch = 0; batch < $BATCHES; batch++)) do
  for ((core = 0; core < $NUM_CORES; core++)) do
      let game=$NUM_CORES*$batch+core
      let a=2*$game
      let b=2*$game+1
      let port_1=3100+$a
      let port_2=3100+$b
      # echo $batch $core $game $a $b $port_1 $port_2
      # printf "1:1:OK\n2:0:OK\n7186238716238\n" > codeball2018-linux/result_$game.txt
      # : '
      codeball2018-linux/codeball2018 \
        --duration $DURATION \
        --p1-name V$VERSION_P1 --p1 tcp-$port_1 \
        --p2-name V$VERSION_P2 --p2 tcp-$port_2 \
        --results-file result_$game.txt --no-countdown \
        --noshow --log-file game_$game.log --nitro true &
      sleep 0.5; cpp-cgdk/versions/MyStrategy_v$VERSION_P1 127.0.0.1 $port_1 $a &
      sleep 0.5; cpp-cgdk/versions/MyStrategy_v$VERSION_P2 127.0.0.1 $port_2 $b &
      # '
    done
    wait
    echo "DONE BATCH $batch"
  done

echo "RESULTS:"
let P1_wins=0
let P2_wins=0
let P1_scores=0
let P2_scores=0
for ((game = 0; game < $((BATCHES*NUM_CORES)); game++)) do
    readarray -t RESFILE < codeball2018-linux/result_$game.txt
    echo ${RESFILE[@]}

    IFS=':' read -ra P1_res <<< "${RESFILE[0]}"
    IFS=':' read -ra P2_res <<< "${RESFILE[1]}"
    # echo ${P1_res[@]}
    # echo ${P2_res[@]}

    let P1_scores=$((P1_scores+P1_res[1]))
    let P2_scores=$((P2_scores+P2_res[1]))

    if ((${RESFILE[0]:0:1} == "1" && ${RESFILE[1]:0:1} == "2"))
    then
      let P1_wins=$P1_wins+1
    elif ((${RESFILE[0]:0:1} == "2" && ${RESFILE[1]:0:1} == "1"))
    then
      let P2_wins=$P2_wins+1
    fi
  done
echo "VERSION $VERSION_P1 || WINS: $P1_wins || SCORE: $P1_scores"
echo "VERSION $VERSION_P2 || WINS: $P2_wins || SCORE: $P2_scores"
echo "$P1_wins $P2_wins || $P1_scores $P2_scores" > run_results.txt
echo "time taken: $SECONDS secs"
