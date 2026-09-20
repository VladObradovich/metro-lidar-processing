# ID is local to one frame, not a persistent track.
uint64 candidate_id
vision_msgs/BoundingBox3D bbox
geometry_msgs/Point nearest_point
float64 distance_m
bool distance_valid
uint32 support_points
uint32 support_rings
bool support_rings_valid
float32 geometry_quality
string[] reasons
