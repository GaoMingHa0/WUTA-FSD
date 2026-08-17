# 26赛季 VCU \- 工控机 CAN通信协议规划

# VCU状态机流程

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzE0Y2M0OWY1YjNiMDgyZTljMjZiZjc0YThkOWYwZjRfZmRjMTY4YmVhYzUxYWQxZjc4YzlhOTExNDRiYjY5MTVfSUQ6NzY3MzQ1MDIxNjEzMzQ0Njg0NF8xNzg2Nzc0NzMyOjE3ODY4NjExMzJfVjM)



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

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTNlOWVlYzQzMDA4NTQwNTA1ZDkyMDhhNjI0NzIwZDBfODM4ODkzMjg4NGMxNmRkZGYwZDAzNWQzMDkxNzA2YjVfSUQ6NzY3MzQ1MDk1NDIxMDAzNjkzMF8xNzg2Nzc0NzMyOjE3ODY4NjExMzJfVjM)



### **VCU → 工控机 ，CAN 总线解析**

|无人赛项任务AMI信号|||
|---|---|---|
|**CAN报文ID**|0x501||
|**报文长度DLC**|8 Byte||
|Byte1 <br>|值|含义|
||1|操控性测试|
||2|直线加速测试|
||3|高速循迹测试|
||4|八字绕环测试|
||5|EBS测试|
||6|车检测试|

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTc1MjMyNGFmOWY1NmM5ZmIwZTIwMzUwODZhMDI4ZDFfOGM5MjgyY2M1NGI4NzQ4M2I0NGM3YjhmZmE1N2M2MDJfSUQ6NzY3MzQ4MzkyNjM3NTIyMjU2OF8xNzg2Nzc0NzMyOjE3ODY4NjExMzJfVjM)







## 

## 

