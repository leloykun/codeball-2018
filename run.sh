codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --p2 tcp-31002 --results-file res.txt --duration 7200 --no-countdown &
sleep 1; cpp-cgdk/build/MyStrategy &
sleep 1; cpp-cgdk/versions/MyStrategy_v19_nodebug 127.0.0.1 31002 0000000000000000
# sleep 1; python3 python.3-cgdk/Runner.py 127.0.0.1 31002 0000000000000000

wait
echo DONE!

echo RESULTS:
cat codeball2018-linux/res.txt
