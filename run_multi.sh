for i in {1..7..2}
  do
    j=i+1
    echo $i $j
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
