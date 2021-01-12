#!/bin/bash

# a plane following a plane

# assume we start the script from the root directory
ROOTDIR=$PWD
PLANE=$ROOTDIR/build/sitl/bin/arduplane

[ -x "$PLANE" ] || {
    ./waf configure --board sitl --debug
    ./waf plane
}

# setup for either TCP or multicast
#UARTA="tcp:0"
UARTA="mcast:"

PLANE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/plane.parm"

mkdir -p swarm/plane1 swarm/plane2
(cd swarm/plane1 && $PLANE --model plane --uartA $UARTA --defaults $PLANE_DEFAULTS) &

# create default parameter file for the follower
cat <<EOF > swarm/plane2/follow.parm
SYSID_THISMAV 2
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID 1
FOLL_DIST_MAX 1000
EOF

(cd swarm/plane2 && $PLANE --model plane --uartA $UARTA --instance 1 --defaults $PLANE_DEFAULTS,follow.parm) &
wait
