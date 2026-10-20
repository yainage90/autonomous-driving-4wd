# Current snapshot; recover service success means accepted, not completed.
string manager_session
string navigation_state
# none, required, recovering
string recovery_state
string fault_reason
# Unix wall clock, preserved until recovery succeeds.
int64 fault_time_ns
string attempt_id
string goal_id
# not_sent, acceptance_unknown, active, terminal, rejected
string action_state
bool stop_confirmed
bool data_ready
bool control_ready
string blocker
# none, in_progress, rejected, failed, succeeded
string last_result
string last_message
