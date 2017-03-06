#!/bin/bash

gdb_cfg="attach.gdb"
firmware_image="nuttx"
extra_windows=

function usage() {
	echo -e >&2 "\
Usage: $0 [-i] [-f firmware_image] [-w]\n\
-i     Initialize target instead of attaching to it\n\
-f     Use different firmware file than default \"nuttx\"\n\
-w     Open extra xterm windows for OpenOCD and telnet"
	exit 1
}

while [ $# -gt 0 ]; do
	case "$1" in
	-i) gdb_cfg="run.gdb";;
	-f) firmware_image="$2"; shift;;
	-w) extra_windows=1;;
	*) usage;;
	esac
	shift
done

# Olimex ARM-USB-OCD-H
function is_olimex_h() {
	lsusb -d 15ba:002b >/dev/null 2>&1
}

# SWD selection for Olimex
if [ "x$SWD" = "x1" ]; then
        olimex_swd_cfg="-f interface/ftdi/olimex-arm-jtag-swd.cfg"
else
        olimex_swd_cfg=""
fi

# Segger J-Link devices. At least idProduct 0101 to 0105 work with OpenOCD.
function is_jlink() {
	lsusb -d 1366: >/dev/null 2>&1
}

# Configuration for OpenOCD / target CPU & JTAG interface selection
if is_olimex_h; then
	openocd_cfg="-f interface/olimex-arm-usb-ocd-h.cfg $olimex_swd_cfg -f target/stm32l1x_dual_bank.cfg"
elif is_jlink; then
	openocd_cfg="-f interface/jlink.cfg -f target/stm32l1x_dual_bank.cfg"
else
	openocd_cfg="-f interface/olimex-arm-usb-ocd.cfg $olimex_swd_cfg -f target/stm32l1x_dual_bank.cfg"
fi

if [ -z "$extra_windows" ]; then
	arm-none-eabi-gdb -ex "target remote | openocd $openocd_cfg -f tools/haltian/debug/gdb-pipe.cfg" \
		-x tools/haltian/debug/$gdb_cfg $firmware_image
else
	openocd_pid=$(pidof 'openocd')
	if [ -z "$openocd_pid" ]; then
		xterm -geometry 100x24+0+0 -e openocd $openocd_cfg &
		sleep 0.5
	fi
	xterm -geometry 100x24+20+400 -e arm-none-eabi-gdb -ex "target extended-remote :3333" \
		-x tools/haltian/debug/$gdb_cfg $firmware_image &
	telnet localhost 4444
fi
