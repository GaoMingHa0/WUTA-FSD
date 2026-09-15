# YOLO adapter contract. Pixels refer to the rectified LEFT image.
# Color order: UNKNOWN, BLUE, YELLOW, ORANGE (Cone constants).
uint32 detection_id
float64[4] bbox_xyxy
float32[4] color_probabilities
float32 confidence
# Optional stereo position in the array header's optical frame, metres.
# Reference: cone centre/axis at the observed centroid height, NOT bbox centre.
bool position_valid
geometry_msgs/Point position
float64[9] position_covariance
