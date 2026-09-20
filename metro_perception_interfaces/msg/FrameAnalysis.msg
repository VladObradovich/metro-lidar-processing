uint8 OK=0
uint8 NOT_IMPLEMENTED=1
uint8 BAD_INPUT=2
uint8 TF_UNAVAILABLE=3
uint8 INVALID_GEOMETRY=4
std_msgs/Header header
string source_instance_id
uint64 session_id
uint64 frame_sequence
uint8 processing_status
string reason
ObstacleCandidate[] candidates
CorridorSegment[] corridor
bool evaluation_region_valid
float64 evaluated_range_m
float64 processing_age_ms
uint64 received_frames
uint64 processed_frames
uint64 rejected_frames
uint64 overwritten_frames

# A02 diagnostics; valid crop does not mean valid corridor/free space.
bool transform_applied
bool calibration_verified
uint64 geometry_point_count
uint64 detection_point_count
uint64 invalid_point_count
uint64 blind_point_count
uint64 outside_roi_point_count
# Explicit operating assumption; does not imply measured extrinsics.
bool calibration_assumed
