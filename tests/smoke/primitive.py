import gdb
import subprocess
import pytest
import os
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

def test_user_entrypoint():
    global timeout
    entrypoint_symbol = get_user_entrypoint()
    cancel_timeout(gdb_execute('mon reset init'))
    cancel_timeout(gdb_execute('tbreak ' + entrypoint_symbol))
    timeout = gdb_execute('continue', MAX_BOOT_TIME_SEC)
    assert timeout is not None
    cancel_timeout(timeout)
    timeout = None

def test_life_expectancy():
    global timeout
    cancel_timeout(gdb_execute('mon reset init'))
    cancel_timeout(gdb_execute('tbreak os_start'))
    timeout = gdb_execute('continue', MAX_BOOT_TIME_SEC)
    assert timeout is not None
    cancel_timeout(timeout)
    timeout = None
    # Now the RAM sections are loaded/initialized and we can modify global variables
    # TODO: "unused functions" functionality is pending
    #test_runtime_disable_deepsleep = gdb.parse_and_eval('test_runtime_disable_deepsleep')
    #test_runtime_disable_deepsleep()
    # The device wont enter deep sleep mode, check it is not restarted within 1 minute
    cancel_timeout(gdb_execute('tbreak os_start'))
    syslog("Waiting for 60 seconds without resets...")
    timeout = gdb_execute('continue', 60)
    assert timeout is None

