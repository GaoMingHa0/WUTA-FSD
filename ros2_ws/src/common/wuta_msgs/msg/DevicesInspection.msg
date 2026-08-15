# 开机传感器自检结果（mission_manager → can_interface）
bool ok              # 全部通过
string[] failures    # 失败设备名列表，如 ["lidar"] / ["lidar","imu"]
