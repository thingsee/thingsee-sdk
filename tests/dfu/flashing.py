import gdb
import sys
import subprocess
import time
from syslog import syslog

def gdb_execute(command):
    syslog("Executing gdb command: " + command)
    return gdb.execute(command, False, False)

def go_dfu():
    symbol = gdb.lookup_global_symbol('up_reset_to_system_bootloader').value().address
    assert(symbol != None)
    address = str(symbol).split()[0]
    syslog("up_reset_to_system_bootloader address is " + address)
    gdb_execute('mon reset init')
    gdb_execute('mon reg pc ' + address)
    gdb_execute('mon resume')
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

