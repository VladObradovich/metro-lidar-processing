uint8 UNKNOWN=0
uint8 OBSTACLE=1
uint8 NO_OBSTACLE_DETECTED=2
uint8 CALIBRATION_TRUST_UNKNOWN=0
uint8 CALIBRATION_TRUST_ASSUMED=1
uint8 CALIBRATION_TRUST_VERIFIED=2
# Observation header: heartbeat must not advance its timestamp.
std_msgs/Header header
string source_instance_id
uint64 session_id
uint64 frame_sequence
uint8 state
string reason
# Trust used for this observation. ASSUMED may never prove a clear path.
uint8 calibration_trust
ObstacleCandidate[] confirmed_objects
float64 distance_m
bool distance_valid
bool evaluation_region_valid
float64 evaluated_range_m
# Local time since last analysis received plus its processing age; not sensor age.
float64 result_age_ms
bool stale
