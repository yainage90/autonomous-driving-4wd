string actuator_session
uint64 generation
string guard_session
uint64 heartbeat
bool armed
bool emergency_stop
bool paused
bool stationary
string reason
uint64 accepted_sequence
# Receiver-local steady clock diagnostics; never interpret as ROS timestamps.
int64 steady_ns
int64 last_valid_ns
int64 expired_ns
int64 zero_applied_ns
int64 max_update_gap_ns
float64 left_output_rps
float64 right_output_rps
