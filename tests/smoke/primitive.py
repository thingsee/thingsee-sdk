import gdb
import subprocess
import pytest
import os
from syslog import syslog

MAX_BOOT_TIME_SEC = 15
timeout = None

def setup_function(function):
    global timeout
    timeout = None

def teardown_function(function):
    global timeout
    cancel_timeout(timeout)

def cancel_timeout(timeout):
    if (timeout != None):
        timeout.kill()
        timeout.wait()

def gdb_execute(command):
    return gdb.execute(command, False, False)

# if timeout did not trigger, the function will return child process
# object - it can be polled or terminated
# if timeout happened, the function will return None
def gdb_continue(timeout):
    fork = subprocess.Popen('sleep ' + str(timeout) + ' && kill -INT ' + str(os.getpid()), shell=True)
    gdb_execute('cont')
    fork.poll()
    if (fork.returncode != None):
        fork.wait()
        fork = None
    return fork

# .config is a Makefile, so let us use make to parse it
def get_user_entrypoint():
    return subprocess.check_output(['make', '-f', 'smoke/get_user_entrypoint.mk'])

def test_user_entrypoint():
    global timeout
    entrypoint_symbol = get_user_entrypoint()
    syslog('Doing reset init for target')
    gdb_execute('mon reset init')
    syslog("Setting tbreak at " + entrypoint_symbol)
    gdb_execute('tbreak ' + entrypoint_symbol)
    syslog("Continuing...")
    timeout = gdb_continue(MAX_BOOT_TIME_SEC)
    assert timeout != None
    cancel_timeout(timeout)
    timeout = None

def test_life_expectancy():
    global timeout
    syslog('Doing reset init for target')
    gdb_execute('mon reset init')
    syslog("Setting tbreak at os_start")
    gdb_execute('tbreak os_start')
    syslog("Continuing...")
    timeout = gdb_continue(MAX_BOOT_TIME_SEC)
    assert timeout != None
    cancel_timeout(timeout)
    timeout = None
    # Now the RAM sections are loaded/initialized and we can modify global variables
    # TODO: "unused functions" functionality is pending
    #test_runtime_disable_deepsleep = gdb.parse_and_eval('test_runtime_disable_deepsleep')
    #test_runtime_disable_deepsleep()
    # The device wont enter deep sleep mode, check it is not restarted within 1 minute
    gdb_execute('tbreak os_start')
    syslog("Waiting for 60 seconds without resets...")
    timeout = gdb_continue(60)
    assert timeout == None

