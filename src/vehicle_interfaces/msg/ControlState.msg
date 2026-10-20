# Guard-owned session and transition epoch; heartbeat uses the guard steady clock.
string guard_session
string manager_session
uint64 epoch
uint64 heartbeat
bool auto_mode
bool emergency_stop
# A replaced manager cannot prove its predecessor's actions ended.
bool navigation_reset_required
