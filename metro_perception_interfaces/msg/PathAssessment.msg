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
# All candidates of the fresh, successfully analysed frame this decision is based on,
# including candidates without a usable distance. Per frame; confirmation is in `tracks`.
ObstacleCandidate[] reported_objects
# Corridor of the same frame, in header.frame_id. Empty on analysis failure or stale output.
CorridorSegment[] corridor
# Candidates followed over frames; OBSTACLE comes from confirmed tracks. Empty when stale.
ObstacleTrack[] tracks
# Distance of the nearest confirmed track; NaN whenever distance_valid is false.
float64 distance_m
bool distance_valid
bool evaluation_region_valid
float64 evaluated_range_m
# Local time since last analysis received plus its processing age; not sensor age.
float64 result_age_ms
bool stale
# FrameAnalysis messages rejected by the source/session/sequence gate since start.
uint64 rejected_analyses
string last_rejection_reason
