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
|Byte1|0|静默|等待ASMS激活且Message2\>1|
||1|有人：等待高压||
||2|有人：等待控制器就绪||
||3|有人：等待有人开始驾驶||
||4|有人：有人驾驶状态||
||5|有人：有人状态错误||
||6|无人：等待高压|AS\_OFF|
||7|无人：等待控制器就绪|AS\_OFF|
||8|无人：启动冷却|AS\_READY|
||9|无人：无人系统待命|AS\_READY|
||10|无人：无人驾驶状态|AS\_DRIVING|
||11|无人：任务完成|AS\_FINISHED|
||12|无人：EMERGENCY|AS\_EMERGENCY|
|Byte2|1|操控性测试||
||2|直线加速测试||
||3|高速循迹测试||
||4|八字绕环测试||
||5|EBS测试||
||6|车检测试||
|Byte3\-8|空报文|空||

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTJhN2Q3YTJiYzg0NjQyMGZlYTg0MGRhM2Y0MDkzODRfYmM2NzBhY2FjMzhhYzgyYmM2NGUyYjk1YmVmODYxYjNfSUQ6NzY3NDYwOTc4NjU1MzQ2OTkzMF8xNzg3MTMwNjA3OjE3ODcyMTcwMDdfVjM)



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzBhNWFhODE1MmFiYjg4NmVjZTM3MWIwMDg2MzRjMDhfMjNmZWUzODQzMGM0NWMzZGFmYTcyOTM1ZWVmYTk4MjZfSUQ6NzY3MzQ4MzkyNjM3NTIyMjU2OF8xNzg3MTMwNjA3OjE3ODcyMTcwMDdfVjM)







## 

## 

