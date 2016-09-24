#!python

import gdb
from syslog import syslog
import subprocess
import os

# if timeout did not trigger, the function will return child process
# object - it can be polled or terminated
# if timeout happened, the function will return None
def gdb_execute(command, timeout=5):
    fork = subprocess.Popen('sleep ' + str(timeout) + ' && kill -INT ' + str(os.getpid()), shell=True)
    syslog("Executing gdb command: \'" + command + "\' with timeout " + str(timeout) + " seconds")
    gdb.execute(command)
    fork.poll()
    if (fork.returncode != None):
        fork.wait()
        fork = None
    return fork

def cancel_timeout(timeout):
    if (timeout != None):
        timeout.kill()
        timeout.wait()

# .config is a Makefile, so let us use make to parse it
def get_user_entrypoint():
    return subprocess.check_output(['make', '-f', 'smoke/get_user_entrypoint.mk'])
