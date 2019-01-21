SECONDS=0

make -C cpp-cgdk/build || exit 1
echo DONE MAKING!

SOURCE_1=${1:-build/MyStrategy}
SOURCE_2=${2:-versions/MyStrategy_v42}

codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT \
  --p2 tcp-31002 --results-file res.txt --duration 7200 --no-countdown \
  --nitro true &
# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name EMPTY --p2 empty --results-file res.txt --duration 7200 --no-countdown &
# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name HELPER --p2 helper --results-file res.txt --duration 7200 --no-countdown &
sleep 1; cpp-cgdk/$SOURCE_1 &
sleep 1; cpp-cgdk/$SOURCE_2 127.0.0.1 31002 0000000000000000
# sleep 1; python3 python.3-cgdk/Runner.py 127.0.0.1 31002 0000000000000000

wait
echo DONE!

echo RESULTS:
cat codeball2018-linux/res.txt

echo "time taken: $SECONDS secs"
