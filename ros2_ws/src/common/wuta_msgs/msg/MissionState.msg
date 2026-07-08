# Mission and system state for Mission Manager
# State constants
uint8 IDLE         = 0   # System initializing
uint8 READY        = 1   # All sensors online, waiting for start
uint8 INSPECTION   = 2   # Vehicle inspection: sensor check + VCU CAN test
uint8 EXPLORE      = 3   # First lap: KISS-ICP + building cone map
uint8 MAPPING_DONE = 4   # Map complete, switching to race mode
uint8 RACE         = 5   # High-speed: NDT map matching
uint8 FINISH       = 6   # Mission complete
uint8 EMERGENCY    = 7   # E-stop / fault

# Mission mode constants (which event)
uint8 MISSION_TRACKDRIVE    = 0
uint8 MISSION_SKIDPAD       = 1
uint8 MISSION_ACCELERATION  = 2

# Localization mode constants
uint8 LOC_KISS_ICP = 0
uint8 LOC_NDT      = 1

std_msgs/Header header
uint8 state
uint8 mission_mode
uint8 localization_mode
string description
