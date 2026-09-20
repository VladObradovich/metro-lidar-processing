uint8 UNKNOWN=0
uint8 OBSTACLE=1
uint8 NO_OBSTACLE_DETECTED=2
# Observation header: heartbeat must not advance its timestamp.
std_msgs/Header header
string source_instance_id
uint64 session_id
uint64 frame_sequence
uint8 state
string reason
ObstacleCandidate[] confirmed_objects
float64 distance_m
bool distance_valid
bool evaluation_region_valid
float64 evaluated_range_m
# Local time since last analysis received plus its processing age; not sensor age.
float64 result_age_ms
bool stale
