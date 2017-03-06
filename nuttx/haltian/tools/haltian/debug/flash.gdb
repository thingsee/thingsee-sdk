target extended-remote :3333
mon reset init
load
mon adapter_khz 300
mon reset halt
#cont
