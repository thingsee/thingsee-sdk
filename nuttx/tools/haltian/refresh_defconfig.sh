#!/bin/bash

set -e

cat <<EOF
This script updates defconfig on every important configuration in this folder.
To update it uses nuttx/tools/refresh.sh script, which does it in an interactive
mode.
Run the script from the board's folder (nuttx/configs/xxx) that you want to
update.

This script depends on 'realpath' package
EOF

just_dirname() {
    basename $(realpath $1)
}

BASE_DIR=$(pwd)

BOARD=$(just_dirname $BASE_DIR)
for DEFCONFIG in $(find \( -type l -o -type f \) -a -name defconfig)
do
    CONFIG=$BOARD/$(just_dirname $(dirname $DEFCONFIG))
    while true
    do
        read -p "Process $CONFIG? [Y/n]?" -n 1 -r
        if [[ $REPLY =~ ^[Yy]$ || $REPLY = "" ]]
        then
            (cd $BASE_DIR/../.. && tools/refresh.sh $CONFIG)
            break
        elif [[ $REPLY =~ ^[Nn]$ ]]
        then
            echo Skipping ${CONFIG}...
            break
        else
            echo Your input not understood. Retry
        fi
    done
done
