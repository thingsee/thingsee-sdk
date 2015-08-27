source tools/debug/attach.gdb
mon reset halt
tbreak os_start
cont
