# Observation of existing Pure Pursuit calculations. No control input.
std_msgs/Header header
string request_id
# Distance to nearest path POINT (not perpendicular distance to a segment), metres.
float64 path_error
float64 goal_distance
# Absolute wrapped yaw error, radians.
float64 goal_yaw_error
# Internal target translational speed before acceleration limiting, m/s.
float64 target_speed
