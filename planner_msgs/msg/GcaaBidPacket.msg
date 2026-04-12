# Per-robot bid row for current global frontier set (consensus-based GCAA).
# All robots publish to the same topic; robot_id distinguishes senders.
std_msgs/Header header
uint32 robot_id
int32[] frontier_ids
float64[] bids
