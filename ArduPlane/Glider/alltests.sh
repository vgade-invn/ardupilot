#!/bin/bash

rm -rf logs
./runtest.py mission1 --location KEDW1 --param-file glider-pullup.parm
./runtest.py mission2 --location KEDW2 --param-file glider-pullup.parm
./runtest.py mission3 --location KEDW3 --param-file glider-pullup.parm
./runtest.py mission4 --location KEDW4 --param-file glider-pullup.parm
./runtest.py mission5 --location KEDW5 --speed-scheduling --param-file glider-pullup.parm
./runtest.py mission6 --location KEDW6 --speed-scheduling --param-file glider-pullup.parm
./runtest.py mission7 --location KEDW7 --speed-scheduling --param-file glider-pullup.parm
./runtest.py mission8 --location KEDW8 --speed-scheduling --param-file glider-pullup.parm
./runtest.py mission9 --location KEDW9 --speed-scheduling --param-file glider-pullup.parm
for i in $(seq 1 9); do
    (./graphs/filter.sh test_runs/mission$i.bin && ./graphs/graph_logs.py test_runs/mission$i-glide.bin) &
done
wait
./graphs/make_index.sh test_runs/mission*glide*bin > test_runs/index.html
