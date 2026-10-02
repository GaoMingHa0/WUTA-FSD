# Global cone map in map frame, organized by color
std_msgs/Header header       # frame_id: "map"

Cone[] red_cones             # Left boundary (red)
Cone[] blue_cones            # Right boundary (blue)
Cone[] yellow_cones          # Yellow markers (not a boundary)
Cone[] orange_cones          # Start/finish line
Cone[] unknown_cones         # Unclassified

bool is_closed               # True when track loop is closed
