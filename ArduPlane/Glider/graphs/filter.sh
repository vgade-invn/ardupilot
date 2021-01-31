#!/bin/bash

for f in $*; do
    bname=$(basename $f .bin)
    dname=$(dirname $f)
    newname=$dname/$bname-glide.bin
    echo Creating $newname
    mavlogdump.py -q -o "$newname" "$f" --condition='in_mission(GPS,BARO,RCOU)' --nottypes 'NK*,XK*' &
done
wait
