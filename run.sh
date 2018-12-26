codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --p2 tcp-31002 --results-file res.txt --duration 7200 &
# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --p2 tcp-31002 --results-file res.txt &
sleep 1; cpp-cgdk/build/MyStrategy &
sleep 1; cpp-cgdk/versions/MyStrategy_v7 127.0.0.1 31002 0000000000000000

wait
echo DONE!

echo RESULTS:
cat codeball2018-linux/res.txt
