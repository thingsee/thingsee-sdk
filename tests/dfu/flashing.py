import gdb
import sys
import subprocess
import time
from syslog import syslog
from gdbsupport import *

MAX_BOOT_TIME_SEC = 15

timeout = None

def setup_function(function):
    global timeout
    timeout = None

def teardown_function(function):
    global timeout
    cancel_timeout(timeout)

def go_dfu():
    entrypoint_symbol = get_user_entrypoint()
    cancel_timeout(gdb_execute('mon reset init'))
    cancel_timeout(gdb_execute('tbreak ' + entrypoint_symbol))
    timeout = gdb_execute('continue', MAX_BOOT_TIME_SEC)
    assert timeout is not None
    cancel_timeout(timeout)

    cancel_timeout(gdb_execute('tbreak up_reset_to_system_bootloader'))
    timeout = gdb_execute('jump up_reset_to_system_bootloader', 2)
    assert timeout is not None
    cancel_timeout(timeout)

    cancel_timeout(gdb_execute('mon resume'))
    return True

def has_dfu_device():
    return subprocess.call('dfu-util -l | grep "Found DFU:"', shell=True) == 0

def wait_dfu_device(exists):
    syslog("Trying to detect DFU device " + ("presense" if exists else "absence") + " for 5 seconds")
    started_at = time.time()
    while (time.time() - started_at < 5):
        if (exists and has_dfu_device()):
            return True
        elif (not exists and not has_dfu_device()):
            return True
        time.sleep(1)
    return False

def flash_dfu_device():
    syslog("Flashing DFU device")
    return subprocess.call(['dfu-util', '-a0', '-D', 'nuttx.dfu', '-s', ':leave']) == 0

def test_flash_two_times():
    syslog("First try----------------------")
    assert go_dfu()
    assert wait_dfu_device(True)
    assert flash_dfu_device()
    assert wait_dfu_device(False)
    syslog("Second try----------------------")
    assert go_dfu()
    assert wait_dfu_device(True)
    assert flash_dfu_device()
    assert wait_dfu_device(False)

