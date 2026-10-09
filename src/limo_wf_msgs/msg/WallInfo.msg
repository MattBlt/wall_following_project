# Stato della parete seguita, nel frame del robot.

std_msgs/Header header
bool wall_detected
float32 distance
float32 angle

uint8 SIDE_LEFT = 0
uint8 SIDE_RIGHT = 1
uint8 side

float32 segment_length
uint16 inlier_count
float32 fit_residual

float32 front_distance