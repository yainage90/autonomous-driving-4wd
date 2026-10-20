# All freshness decisions use the actuator's own steady clock and heartbeat.
string actuator_session
uint64 generation
string guard_session
uint64 sequence
uint64 heartbeat
geometry_msgs/Twist velocity
