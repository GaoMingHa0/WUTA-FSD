# FSD状态机自定义消息

# State
uint8 IDLE         = 0   # 系统启动
uint8 READY        = 1   # 一切就绪
uint8 INSPECTION   = 2   # 车检测试
uint8 EXPLORE      = 3   # 建图探索
uint8 MAPPING_DONE = 4   # （仅限trackdrive）建图完成
uint8 RACE         = 5   # 比赛
uint8 FINISH       = 6   # 任务完成
uint8 EMERGENCY    = 7   # RES急停/设备故障

# Mission mode
uint8 MISSION_TRACKDRIVE    = 0
uint8 MISSION_SKIDPAD       = 1
uint8 MISSION_ACCELERATION  = 2
uint8 MISSION_INSPECTION    = 3   # 车检测试（台架：慢速转驱动 + 正弦波转转向）
uint8 MISSION_EBS_TEST      = 4   # EBS 测试

# Localization mode
uint8 LOC_KISS_ICP = 0
uint8 LOC_NDT      = 1

std_msgs/Header header
uint8 state
uint8 mission_mode
uint8 localization_mode
string description
