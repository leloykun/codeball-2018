# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --results-file res.txt --duration 7200 &
codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --p2 tcp-31002 --results-file res.txt --duration 7200 --vsync --team-size 2 &
# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --p2 tcp-31002 --results-file res.txt --duration 7200 --vsync --team-size 2 --noshow &
# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p1-dump temp.json --p2-name EMPTY --p2 empty --results-file res.txt --duration 7200 --vsync --team-size 2 &
# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --p2 tcp-31002 --results-file res.txt &
# codeball2018-linux/codeball2018 --p1-name CUR-STRAT --p2-name PREV-STRAT --p2 tcp-31002 --results-file res.txt --log-file logs/long_game_2.log --noshow --duration 50000 &
sleep 1; cpp-cgdk/build/MyStrategy &
# sleep 1; cpp-cgdk/versions/MyStrategy_v10 127.0.0.1 31002 0000000000000000
sleep 1; cpp-cgdk/versions/MyStrategy_v14_nodebug 127.0.0.1 31002 0000000000000000
# sleep 1; python3 python.3-cgdk/Runner.py 127.0.0.1 31002 0000000000000000

wait
echo DONE!

echo RESULTS:
cat codeball2018-linux/res.txt
