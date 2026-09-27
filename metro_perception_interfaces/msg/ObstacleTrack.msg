# A candidate followed over frames (G4). Confirmed tracks decide OBSTACLE.
uint64 track_id
bool confirmed
# Not measured in this frame: position and distance are predicted from the ego motion.
bool coasting
# Hits within the confirmation window, and frames since the track started.
uint32 hits
uint32 age_frames
# Last measured (or predicted) box and longitudinal distance, in the analysis frame.
vision_msgs/BoundingBox3D bbox
float64 distance_m
# Evidence seen over the track's life: "MOTION" and/or "GAUGE".
string[] reasons
