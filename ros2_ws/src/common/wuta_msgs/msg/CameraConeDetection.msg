# YOLO adapter contract. Pixels refer to the rectified LEFT image.
# Color order: RED=0, YELLOW=1, BLUE=2, ORANGE=3, UNKNOWN=4.
# Model class IDs are used directly; no red/orange remapping.
uint32 detection_id
float64[4] bbox_xyxy
float32[5] color_probabilities
float32 confidence
# Optional stereo position in the array header's optical frame, metres.
# Reference: cone centre/axis at the observed centroid height, NOT bbox centre.
bool position_valid
geometry_msgs/Point position
float64[9] position_covariance
