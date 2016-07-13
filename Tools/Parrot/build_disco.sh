#!/bin/sh
# script to build for Disco with waf

# minimal submodule list for Disco build
SUBMODULES="modules/waf modules/mavlink"

for m in $SUBMODULES; do
    [ -f $m/.git ] || {
        echo "Initialising submodules"
        git submodule init || exit 1
    }
    echo "Checking submodule $m"
    git submodule update $m
done

echo "Running configure"
./modules/waf/waf-light configure --board disco --notests --no-submodule-update || {
    echo "Configure failed"
    exit 1
}
echo "Running build"
./modules/waf/waf-light plane --no-submodule-update
