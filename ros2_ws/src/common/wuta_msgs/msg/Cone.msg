# Single cone detection
# Color constants
uint8 COLOR_UNKNOWN = 4
uint8 COLOR_BLUE    = 2
uint8 COLOR_YELLOW  = 1
uint8 COLOR_ORANGE  = 3
uint8 COLOR_RED     = 0

geometry_msgs/Point position   # 3D position in sensor/map frame
uint8 color 4                   # Cone color (see constants above)
float32 confidence              # Detection confidence [0.0, 1.0]
