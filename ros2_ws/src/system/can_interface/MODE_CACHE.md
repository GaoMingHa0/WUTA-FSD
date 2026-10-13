# ZLG 模式缓存恢复

节点默认读取 `/run/zlgcan_bridge/<can.device>.mode`，无需改总启动器。
打开 CAN 接口后，启动恢复阶段先处理实时帧；如果已经收到有效长度的模式帧，
禁止缓存覆盖。否则尝试读取当前存活桥会话的快照并应用一次，随后仅接收实时帧。
模式字符串发布和周期保活沿用现有机制，不读取或恢复任何 RES GO/急停事件。

参数（代码默认值亦生效）：

- `mode_cache.enabled=true`
- `mode_cache.directory=/run/zlgcan_bridge`
- `mode_cache.bootstrap_wait_sec=2.0`：无缓存时最多尝试的窗口，非阻塞。
- `mode_cache.max_age_sec=0.0`：0 表示同一存活桥会话不限制模式年龄。

缓存须匹配接口、模式 CAN ID、系统 boot ID、存活桥的 PID/启动时间；
损坏文件、旧进程、僵尸进程、未来时间戳和超龄记录均拒绝。
缓存缺失或读取失败不影响实时接收。CAN 接口打开失败时不恢复模式。
成功恢复输出 `Restored VCU mode ... from bridge cache`；失败窗口结束后
输出跳过原因。快照只代表最后观测模式，不能用于判定 VCU 在线。

更新后需要重新启动桥和节点；旧桥进程不产生快照。
当前桥只缓存固定 0x501，节点配置其他模式 ID 时不会应用这个快照。

无硬件测试在主仓库运行：

```bash
/usr/bin/python3 -m unittest tools.test_mode_cache
source /opt/ros/humble/setup.bash
source WUTA-FSD/ros2_ws/install/setup.bash
/usr/bin/python3 -m unittest tools.test_mode_cache_ros
```

ROS 测试替换 CAN socket，使用隔离域 173，禁用所有周期发送；不接触实际 CAN 总线。
