#!/bin/bash
#
# Script for building Thingsee
#
set -e

# Default configuration
conf=retail
debug=
[ -n "$1" ] && conf=$1
[ -n "$2" ] && debug=$2

make -j distclean

cd tools
./configure.sh haltian-tsone/${conf}
cd ..

if [ "x${debug}" ==  "x" ]; then
	echo NO DEBUG
else
	for dvar in CONFIG_STM32_DISABLE_IDLE_SLEEP_DURING_DEBUG CONFIG_STM32_KEEP_CORE_CLOCK_ENABLED_IN_IDLE_MODES
	do
		perl -p -i.bak -e "s/# `echo $dvar` is not set//" .config
		echo ${dvar}=y >> .config
	done
fi

CPUNO=$(grep -c processor /proc/cpuinfo)
make -j${CPUNO} 2>&1 |tee /tmp/build-log

if [[ ${PIPESTATUS[0]} -eq 0 ]]; then
	echo ""
	echo "OK"
	echo ""
	grep "warning:" /tmp/build-log
	exit 0
else
	echo ""
	echo "FAIL"
	echo ""
	grep "error:" /tmp/build-log
	grep "warning:" /tmp/build-log
	exit 1
fi
