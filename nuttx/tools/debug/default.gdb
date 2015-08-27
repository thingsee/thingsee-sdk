target extended-remote :3333
define hook-step
  mon cortex_m maskisr on
end
define hookpost-step
  mon cortex_m maskisr off
end
#mon reset halt
mon reset init
#mon poll off
mon cortex_m maskisr auto
set mem inaccessible-by-default off
mon adapter_khz 1000
mon target_request debugmsgs charmsg
mon gdb_sync
source tools/debug/Thingsee
#cont
