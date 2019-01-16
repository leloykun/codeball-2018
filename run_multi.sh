NUM_GAMES=${1:-4}
DURATION=${2:-18000}
VERSION_P1=${3:-33}
VERSION_P2=${4:-34}

for ((i = 1; i <= $NUM_GAMES; i++))
  do
    let a=2*$i-1
    let b=2*$i
    echo $i $a $b
    codeball2018-linux/codeball2018 \
      --duration $DURATION \
      --p1-name V$VERSION_P1 --p1 tcp-3100$a \
      --p2-name V$VERSION_P2 --p2 tcp-3100$b \
      --results-file res$i.txt --no-countdown \
      --noshow --log-file game$i.log --nitro true &
    sleep 1; cpp-cgdk/versions/MyStrategy_v$VERSION_P1 127.0.0.1 3100$a $a &
    sleep 1; cpp-cgdk/versions/MyStrategy_v$VERSION_P2 127.0.0.1 3100$b $b &
  done
: '
codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p1 tcp-31001 \
  --p2-name PREV-STRAT --p2 tcp-31002 --results-file res1.txt --no-countdown \
  --noshow --log-file game1.log --nitro true &
sleep 1; cpp-cgdk/versions/MyStrategy_v33 127.0.0.1 31001 1 &
sleep 1; cpp-cgdk/versions/MyStrategy_v34 127.0.0.1 31002 2 &

codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p1 tcp-31003 \
  --p2-name PREV-STRAT --p2 tcp-31004 --results-file res2.txt --no-countdown \
  --noshow --log-file game2.log --nitro true &
sleep 1; cpp-cgdk/versions/MyStrategy_v33 127.0.0.1 31003 3 &
sleep 1; cpp-cgdk/versions/MyStrategy_v34 127.0.0.1 31004 4 &

codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p1 tcp-31005 \
  --p2-name PREV-STRAT --p2 tcp-31006 --results-file res3.txt --no-countdown \
  --noshow --log-file game3.log --nitro true &
sleep 1; cpp-cgdk/versions/MyStrategy_v33 127.0.0.1 31005 5 &
sleep 1; cpp-cgdk/versions/MyStrategy_v34 127.0.0.1 31006 6 &

codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p1 tcp-31007 \
  --p2-name PREV-STRAT --p2 tcp-31008 --results-file res4.txt --no-countdown \
  --noshow --log-file game4.log --nitro true &
sleep 1; cpp-cgdk/versions/MyStrategy_v33 127.0.0.1 31007 7 &
sleep 1; cpp-cgdk/versions/MyStrategy_v34 127.0.0.1 31008 8

wait
echo DONE!

echo RESULTS:
cat codeball2018-linux/res1.txt
cat codeball2018-linux/res2.txt
cat codeball2018-linux/res3.txt
cat codeball2018-linux/res4.txt

'
