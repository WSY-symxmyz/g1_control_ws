std_msgs/Header header
string[] joint_names
# When true, deltas are added to the controller's latest target positions.
# When false, positions are interpreted as absolute joint targets.
bool is_delta
float64[] positions
float64[] deltas
float64[] velocities
float64[] kp
float64[] kd
builtin_interfaces/Duration duration
bool hold
