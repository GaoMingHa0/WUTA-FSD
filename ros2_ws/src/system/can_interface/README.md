# CAN接口当前运行模式

2026-10-23。CAN业务编码和ROS输入事件由本包负责，ZLG桥只搬运原始帧；任务状态由mission_manager、驾驶Command由controller唯一生产。

| 模式 | 行为 |
| --- | --- |
| 正常（两个开关均false） | 沿原0x210控制和0x301心跳协议发送，接收VCU模式与RES GO/急停 |
| 制动台架（brake_bench_mode=true） | 原0x210控制和0x301心跳；纵向仅允许[-1,0]，转向强制0，Command/活动任务状态超0.5秒或自检不通过时中性；100ms控制周期 |
| 人工记录（receive_only=true） | 禁止控制、心跳、回调和析构发送，保留真实模式/RES接收；与台架互斥 |

## 台架当前行为

当前`parseResFrame`中调用`prepareBrakeBench`的入口已临时注释，真实GO直接交给原任务机；controller原5秒中性等待、10秒50%制动、中性释放仍执行。FSD不再发送0x603前置帧。该函数源码保留供后续恢复，相关`brake_bench_prepare` CTest显式DISABLED，不能把它当作当前启用路径。正常模式和人工记录不受此临时停用影响。

## 协议和保护

0x210/DLC8中Signal1/Signal2为大端，纵向中性32767(7F FF)，50%制动开度-0.5编码16383(3F FF)，转向回正为7F FF；Signal3来自DevicesInspection，Signal4来自FINISH。0x301/DLC1为上线心跳。0x1E4首字节13为真实GO变化沿，10为急停，11为RES上线；模式缓存仅恢复模式，不恢复GO或急停。

台架正常析构发送一次中性，根启动器停止时先停controller并保留CAN0.7秒；总线断开或进程强杀不能保证帧送达。电机侧对象字典、机械制动力及VCU定标不能仅由0x210开度推断。`test_brake_bench_can.cpp`和`test_receive_only_can.cpp`用内存设备验证真实回调/编码，无物理CAN操作；所有ROS测试使用独立非零ROS domain和ROS_LOCALHOST_ONLY=1。

正式协议见`docs_for_can`；部署、消息消费者重建、CLI M键及恢复流程见宿主`docs/DEPLOYMENT.md`和`docs/ROS_INTERFACE.md`。旧0x603前置的字节格式属于历史用户指定序列，不代表完整电机初始化或50%电机实际转矩。
