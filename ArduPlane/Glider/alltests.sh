#!/bin/bash

rm -rf logs
./runtest.py --mission 1
./runtest.py --mission 2
./runtest.py --mission 3
./runtest.py --mission 4
./runtest.py --mission 5 --speed-scheduling
./runtest.py --mission 6 --speed-scheduling
./runtest.py --mission 7 --speed-scheduling
./runtest.py --mission 8 --speed-scheduling
./runtest.py --mission 9 --speed-scheduling

for i in $(seq 1 9); do
    (./graphs/filter.sh test_runs/mission$i.bin && ./graphs/graph_logs.py test_runs/mission$i-glide.bin) &
done
wait
./graphs/make_index.sh test_runs/mission*glide*bin > test_runs/index.html
./tocsv.sh test_runs/mission?-glide.bin
