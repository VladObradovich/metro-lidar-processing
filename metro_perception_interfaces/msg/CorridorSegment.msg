# All geometry is in FrameAnalysis.header.frame_id.
geometry_msgs/Point start
geometry_msgs/Point end
float64 width_m
float64 height_m
# Normalized ax + by + cz + d = 0; only usable if geometry_valid.
float64[4] ground_plane
bool geometry_valid
bool coverage_valid
string[] reasons
