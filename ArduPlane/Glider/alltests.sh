#!/bin/bash

rm -rf logs
./runtest.py mission1 --location KEDW2
./runtest.py mission2 --location KEDW2
./runtest.py mission3 --location KEDW2
./runtest.py mission4 --location KEDW2
./runtest.py mission5 --location KEDW3 --speed-scheduling
./runtest.py mission6 --location KEDW4 --speed-scheduling
./runtest.py mission7 --location KEDW5 --speed-scheduling
./runtest.py mission8 --location KEDW6 --speed-scheduling
./runtest.py mission9 --location KEDW6 --speed-scheduling
./graphs/filter.sh test_runs/mission?.bin
./graphs/graph_logs.py test_runs/mission?-glide.bin
