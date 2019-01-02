codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p1 tcp-31003 --p2-name PREV-STRAT --p2 tcp-31004 --results-file res1.txt --until-first-goal --no-countdown --noshow &
sleep 1; cpp-cgdk/build/MyStrategy 127.0.0.1 31003 3 &
sleep 1; cpp-cgdk/versions/MyStrategy_v18 127.0.0.1 31004 4 &

codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p1 tcp-31005 --p2-name PREV-STRAT --p2 tcp-31006 --results-file res2.txt --until-first-goal --no-countdown --noshow &
sleep 1; cpp-cgdk/build/MyStrategy 127.0.0.1 31005 5 &
sleep 1; cpp-cgdk/versions/MyStrategy_v18 127.0.0.1 31006 6

wait
echo DONE!

echo RESULTS:
cat codeball2018-linux/res1.txt
cat codeball2018-linux/res2.txt
