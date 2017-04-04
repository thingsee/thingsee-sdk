#!/bin/bash
#
# Simple JTAG flashing tool for ThingseeOne (OpenOCD + GDB)
#
# Copyright 2014 Haltian Oy.
#
# Author: Jussi Kivilinna
#

# Base address for STM32F7 MCU 96-bit UID
MCU_UID_BASE_ADDR="0x1FF0F420"

# Log directory
LOGDIR="$HOME/logs/nuttx_ocd_gdb_flash"

# Logfile for flash output.
LOGFILE="$LOGDIR/log.txt"

# Default firmware file to use for flashing. Must be in ELF format.
if [ "x$1" = "x" ]; then
	FIRMWARE_IMAGE="nuttx"
else
	FIRMWARE_IMAGE="$1"
fi

# Path to GDB
GDB="arm-none-eabi-gdb"

# Logfile for GDB communications
GDB_LOGFILE="$LOGDIR/log.gdb.txt"

# Path to OpenOCD
OPENOCD="openocd"

# Configuration for OpenOCD / target CPU & JTAG interface selection
if [ "x$OPENOCD_CFG" = "x" ]; then
  openocd_cfg="-f board/stm32f7discovery.cfg"
else
  openocd_cfg="$OPENOCD_CFG"
fi

# OpenOCD logfile
OPENOCD_LOGFILE="$LOGDIR/log.openocd.txt"

#
# Flashing functions
#

gdb_execute() {
	# log command
	echo "$*" >> "$GDB_LOGFILE"

	# feed command to gdb
	echo "$*" >&$gdb_in
}

__gdb_wait_for_done() {
	local timeout=""
	if [ "x$1" != "x" ]; then
		timeout="-t $1"
	fi

	gdb_done=""

	# Wait command to complete.
	while read -u$gdb_out $timeout gdb_line; do
		# Every command is terminated by '\n(gdb)\n'
		if [ "x$gdb_line" = "x(gdb)" ]; then
			return 0
		fi

		# Done line often contains useful information
		local done_line="${gdb_line:0:6}"
		if [ "x$done_line" = "x^done," ]; then
			gdb_done="$gdb_line"
		fi
	done

	# timeout
	return 1
}

do_print() {
	if [ "x$1" = "xprint" ]; then
		echo "$2"
	fi
}

gdb_wait_for_done() {
	local wait_timedout=0
	local print="$1"
	local timeout_max="$2"

	# Default to 5 second timeout.
	if [ "x$timeout_max" = "x" ]; then
		timeout_max="5"
	fi

	# Timeout after <timeout_max> seconds.
	if __gdb_wait_for_done "$timeout_max"; then
		do_print "$print" "OK."
		return 0
	else
		do_print "$print" "FAILED."
		return 1
	fi
}

gdb_wait_for_flash_done() {
	local wait_timedout=1
	local wait_max=$((60 * 1000)) #msec
	timedout=0

	while ! __gdb_wait_for_done "0.1"; do
		echo -n "."

		# Check for timeout
		wait_max=$((wait_max - 100))
		if [ "$wait_max" -eq "0" ]; then
			# timeout
			echo " FAILED."
			return 1
		fi
	done

	echo " OK."
	return 0
}

__gdb_wait_for_stop() {
	local timeout=""
	if [ "x$1" != "x" ]; then
		timeout="-t $1"
	fi

	# Wait command to complete.
	while read -u$gdb_out $timeout gdb_line; do
		local cut_line="${gdb_line:0:9}"

		# When execution stopped, '\n*stopped,' is generated.
		if [ "x$cut_line" = "x*stopped," ]; then
			return 0
		fi
	done

	# timeout
	return 1
}

gdb_wait_for_stop() {
	local wait_timedout=1
	local timeout="$1"

	if [ "x$1" = "x" ]; then
		timeout="120"
	fi

	local wait_max=$((timeout * 1000)) #msec
	timedout=0

	while ! __gdb_wait_for_stop "0.1"; do
		echo -n "."

		# Check for timeout
		wait_max=$((wait_max - 100))
		if [ "$wait_max" -eq "0" ]; then
			# timeout
			echo " FAILED."
			return 1
		fi
	done

	# After '*stopped,' we also get '(gdb)'
	if ! gdb_wait_for_done "noprint" "$wait_max"; then
		# timeout
		echo " FAILED."
		return 1
	fi

	echo " OK."
	return 0
}

parse_read_memory_to_ascii() {
	local gdb_line="$1"
	local line=""

	#
	# Input format is:
	#  ^done,memory=[{begin="0x<addr>",offset="0x<len>",end="0x<addr>",contents="<hex>"}]
	#

	# Convert input to normal output
	local output="$(sed 's/^.*contents=\"\([0-9,a-f,A-F]*\)\".*$/\1/' <<< $gdb_line | xxd -p -r)"

	while read line; do
		echo "SELFTESTS: $line"
	done <<< "$output"
}

parse_read_memory_32bit_integer() {
	local gdb_line="$1"
	local line=""

	#
	# Input format is:
	#  ^done,memory=[{begin="0x<addr>",offset="0x<len>",end="0x<begin+4>",contents="<eight hex-chars>"}]
	#

	# Convert input to hex bytes
	local output="$(sed -e 's/^.*contents=\"\([0-9,a-f,A-F]*\)\".*$/\1/' <<< $gdb_line)"

	# Convert little-endian hex presentation of 32-bit integer to decimal value
	local mul=1
	local pos=0
	local end=${#output}

	if [ $end -ne 8 ]; then
		return 1 # error reading memory; did not receive 32-bits
	fi

	# Read two hex-chars per loop from variable and update 'value'.
	value=0
	for ((pos = 0; pos < end; pos += 2)); do
		local byte_hex=${output:$((pos)):2}
		value=$((16#$byte_hex * mul + value))
		mul=$((mul * 256))
	done

	return 0
}

gdb_cleanup() {
	echo -n "Aborting flash software execution..."
	kill $gdb_PID
	sleep 1
	kill -9 $gdb_PID
	echo " Done."
}

read_mcu_uid_from_device() {
	local reg_base_addr="$1"

	# Read first 32-bits of UID
	gdb_execute "-data-read-memory-bytes $reg_base_addr+0x00 4"
	if ! gdb_wait_for_done "noprint"; then
		gdb_cleanup
		exit 1
	fi

	# Parse UID
	local value=""
	if ! parse_read_memory_32bit_integer "$gdb_done"; then
		gdb_cleanup
		exit 1
	fi
	local mcu_uid0=$(printf "%08x" "$value")

	# Read second 32-bits of UID
	gdb_execute "-data-read-memory-bytes $reg_base_addr+0x04 4"
	if ! gdb_wait_for_done "noprint"; then
		gdb_cleanup
		exit 1
	fi

	# Parse UID
	local value=""
	if ! parse_read_memory_32bit_integer "$gdb_done"; then
		gdb_cleanup
		exit 1
	fi
	local mcu_uid1=$(printf "%08x" "$value")

	# Read third 32-bits of UID
	gdb_execute "-data-read-memory-bytes $reg_base_addr+0x08 4"
	if ! gdb_wait_for_done "noprint"; then
		gdb_cleanup
		exit 1
	fi

	# Parse UID
	local value=""
	if ! parse_read_memory_32bit_integer "$gdb_done"; then
		gdb_cleanup
		exit 1
	fi
	local mcu_uid2=$(printf "%08x" "$value")

	MCU_UID="$mcu_uid0:$mcu_uid1:$mcu_uid2"
}

do_gdb_run() {
	local openocd_args="$openocd_cfg -c \"gdb_port pipe; log_output $OPENOCD_LOGFILE\" -c init"
	local gdb_remote="target extended-remote | $OPENOCD $openocd_args"
	local gdb_PID=""
	local gdb_in=""
	local gdb_out=""
	local gdb_line=""
	local gdb_done=""

	local do_flashing="$1"

	local run_type=""
	if [ "x$do_flashing" = "xflash" ]; then
		run_type="Flashing '$FIRMWARE_IMAGE'"
	fi

	# Log start
	local start_time=$(date)
	(
		echo ""
		echo "========================================================================================"
		echo " START; $start_time; TYPE: $do_flashing"
		echo "========================================================================================"
		echo ""
	) | tee -a $GDB_LOGFILE

	echo "===== $run_type ..."
	echo "=====       Date: $start_time."

	# Start GDB in GDB/MI mode
	echo -n "Starting flash software..."
	coproc "$GDB" --interpreter=mi2 "$FIRMWARE_IMAGE"
	gdb_out=${COPROC[0]}
	gdb_in=${COPROC[1]}
	gdb_PID=$COPROC_PID

	# Wait start-up to complete.
	gdb_wait_for_done
	echo " OK."

	# Setup output log-file for GDB.
	gdb_execute "set logging file $GDB_LOGFILE"
	if ! gdb_wait_for_done "noprint"; then
		gdb_cleanup
		exit 1
	fi

	# Start logging.
	gdb_execute "set logging on"
	if ! gdb_wait_for_done "noprint"; then
		gdb_cleanup
		exit 1
	fi

	# Make connection to remote target (launches OpenOCD)
	echo -n "Initializing JTAG adapter..."
	gdb_execute "$gdb_remote"
	if ! gdb_wait_for_done "print"; then
		gdb_cleanup
		exit 1
	fi

	# Read device 96-bit MCU UID.
	if [ "x$MCU_UID_BASE_ADDR" != "x" ]; then
		local MCU_UID=""

		read_mcu_uid_from_device "$MCU_UID_BASE_ADDR"

		echo "===== MCU UID: $MCU_UID" | tee -a $GDB_LOGFILE
	fi

	if [ "x$do_flashing" = "xflash" ]; then
		# Reset hardware and initialize for faster flashing
		echo -n "Resetting target device for flashing..."
		gdb_execute "monitor reset init"
		if ! gdb_wait_for_done "print"; then
			gdb_cleanup
			exit 1
		fi

		# Load/Flash firmware to device
		echo -n "Flashing firmware to device..."
		gdb_execute "-target-download"
		if ! gdb_wait_for_flash_done; then
			gdb_cleanup
			exit 1
		fi

		# Decrease adapter speed before reset
		gdb_execute "monitor adapter_khz 333"
		if ! gdb_wait_for_done "noprint"; then
			gdb_cleanup
			exit 1
		fi
	fi

	# Quit GDB
	gdb_execute "-gdb-exit"

	# Wait GDB to quit
	wait $gdb_PID

	# Log end
	(
		echo ""
		echo "========================================================================================"
		echo " END; $(date); TYPE: $do_flashing"
		echo "========================================================================================"
		echo ""
	) | tee -a $GDB_LOGFILE
}

clean_stalled_gdb() {
	# Poor man's solution to cleaning up previously stalled flashing tools
	(
		killall "$GDB"
		killall "$OPENOCD"
		sleep 0.25
		killall -9 "$GDB"
		killall -9 "$OPENOCD"
	) > /dev/null 2>&1
}

do_flash() {
	clean_stalled_gdb

	do_gdb_run flash 2>&1 | tee -a "$LOGFILE"
}

#
# Helper functions
#

print_banner() {
	echo ""
	echo "======================================="
	echo ""
}

print_banner_with_serial_number() {
	echo ""
	echo "======================================="
	echo ""
}

# Check if first argument is integer number
isnum() {
	( [ "x$1" != "x" ] && [ "$1" -eq "0" ] || [ "$1" -ne "0" ] ) > /dev/null 2>&1
}

wait_for_enter() {
	local input=""

	echo ""
	echo "++++"
	echo "Press Enter..."
	read input
}

#
# Menu actions
#

# Perform device flashing
do_menu_flash() {
	do_flash
}

menu_loop() {
	local input=""
	local selected_quit="0"
	local last_menu_option=""

	while [ "$selected_quit" = "0" ]; do
		print_banner_with_serial_number

		# Show options
		echo "1. Flash device"
		echo "2. Quit."
		last_menu_option="2" # Quit is last entry.
		default_menu_option="1"

		# Input
		while read input; do
			if [ "x$input" = "x" ]; then
				input=$default_menu_option
				break
			fi

			if ! isnum "$input" || \
			   [ "$input" -lt "1" ] || \
			   [ "$input" -gt "$last_menu_option" ]; then
				# Input is not valid menu option
				continue
			fi

			break
		done

		case "$input" in
		1)
			do_menu_flash
			;;
		2)
			selected_quit=1
			break
			;;
		esac
	done
}


#
# Main script, run menu loop
#

if ! [ -d "$LOGDIR" ]; then
	if ! mkdir -p "$LOGDIR"; then
		echo "Failed to create log-directory '$LOGDIR'. Exiting..."
		exit 1
	fi
fi

do_flash
