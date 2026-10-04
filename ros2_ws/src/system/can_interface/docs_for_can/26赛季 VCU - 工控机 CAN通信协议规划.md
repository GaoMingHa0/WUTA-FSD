# 26赛季 VCU \- 工控机 CAN通信协议规划

# VCU状态机流程

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NzJhZTI2MTkzMGI1MmRmOWUyMmZlY2UxYTJhOGVmMWFfODMwMTdlZmJjMzBlZWQ3ZDczZTI1NmIyZjAzM2QwNTJfSUQ6NzY3MzQ1MDIxNjEzMzQ0Njg0NF8xNzg3MTMwNjA3OjE3ODcyMTcwMDdfVjM)



# **接口约定**

### **工控机→ VCU，转发到 CAN 总线**

|无人工控机发至VCU的报文信息|||||
|---|---|---|---|---|
|**CAN报文ID**|0x210||||
|**报文长度DLC**|8 Bytes||||
|信号|含义|大小范围|位置|备注|
|Signal1|纵向控制|10\~65525<br>10\~32767：越靠近0制动力越大<br>32767\~65525：越大驱动力越大|Byte1\-Byte2|<br>|
|Signal2|横向控制|10\~65525<br>以32767为中心<br>10方向为左<br>65525方向为右|Byte3\-Byte4||
|Signal3|工控机上线|1：上线<br>0：未上线|Byte5|代表无人这边设备自检有无问题。如果有问题就切EMERGENCY|
|Signal4|无人任务已完成|1：无人任务已完成<br>0：无人任务未完成|Byte6|所有项目FINISH后发送|
|Signal5|空报文|空|Byte7\-Byte8||

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTc2NDQyNGMxMjFlNmJlYWZjNDIzZDJiNDMzZmYyZDJfMDA1MjBjMDQ3YTQ5NDJmOTFkMTVlYjBkMDFlMmVlOWNfSUQ6NzY3MzQ1MDk1NDIxMDAzNjkzMF8xNzg3MTMwNjA3OjE3ODcyMTcwMDdfVjM)



### **VCU → 工控机 ，CAN 总线解析**

|**VCU发到无人工控机的信息**||||
|---|---|---|---|
|**CAN报文ID**|0x501|||
|**报文长度DLC**|8 Byte|||
||值|含义|备注|
|Byte1|1|操控性测试||
||2|直线加速测试||
||3|高速循迹测试||
||4|八字绕环测试||
||5|EBS测试||
||6|车检测试||
|Byte2\-8|空报文|空||

> **协议变更**：取消原 Byte1「VCU 状态」定义（0~12 状态码表作废），原 Byte2「测试模式」提升到 Byte1。
> RES Go / RES 急停不再经 0x501 下发，改由 RES 遥控器的 **0x1E4**（下表）承载；上位机恢复为「选模式 + 等 RES 发车放行」——选中任务模式后仍需 RES 发车按钮（0x1E4 Byte1=0x13），车检（mode 6）同样需要。

### **RES 遥控器 → 工控机，CAN 总线解析**

|**RES 发到无人工控机的信息**||||
|---|---|---|---|
|**CAN报文ID**|0x1E4（十进制 484）|||
|**波特率 / 帧格式**|500 kbps / 标准帧|||
|**报文长度DLC**|3 Byte（实测桥侧记录为 8 Byte，故解析只读 Byte1、不校验 DLC）|||
||值|含义|备注|
|Byte1|0x11|遥控器上线|持续电平（实测 33.3Hz 周期广播，中位 30ms）|
||0x13|发车按钮被按下|**瞬时脉冲**（实测 0.15\~0.51s），按一下即发车|
||0x10|按下急停|持续电平（实测按下后保持 9.6\~475s），锁存|
||0x00|遥控器未上线|未定义值，工控机忽略并告警|
|Byte2\-3|空报文|空||

> **上位机行为**（`can_interface` 节点）：
> - `0x13` → **仅在变化沿**发布一次 `/system/start_command=true`（不重放、不锁存）。`mission_manager` 是常驻订阅者，漏收只可能是它当时没在跑，需重按。
> - `0x10` → 锁存发布 `/system/emergency=true`（与设备自检失败同一通路）。
> - `0x11` → 纯状态，只记日志（实测 33Hz 持续电平）。
> - 已选模式但未收到 `0x13` 时不会启动（门控），故「只选模式」不再放行。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTJhN2Q3YTJiYzg0NjQyMGZlYTg0MGRhM2Y0MDkzODRfYmM2NzBhY2FjMzhhYzgyYmM2NGUyYjk1YmVmODYxYjNfSUQ6NzY3NDYwOTc4NjU1MzQ2OTkzMF8xNzg3MTMwNjA3OjE3ODcyMTcwMDdfVjM)



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzBhNWFhODE1MmFiYjg4NmVjZTM3MWIwMDg2MzRjMDhfMjNmZWUzODQzMGM0NWMzZGFmYTcyOTM1ZWVmYTk4MjZfSUQ6NzY3MzQ4MzkyNjM3NTIyMjU2OF8xNzg3MTMwNjA3OjE3ODcyMTcwMDdfVjM)







## 

## 

