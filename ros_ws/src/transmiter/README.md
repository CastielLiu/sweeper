# transmiter package
	接收环境识别信息将其编码为CAN协议矩阵,并以can_msgs/FrameArray类型发布

## Nodes

### Node transmiter_node 信息转发节点
1. 订阅环境识别模块发布的识别信息，打包为can消息发布出去，用于can驱动节点向总线发布数据
2. 订阅can驱动节点发布的数据(can分析仪读取总线消息后发布的消息)，解码后以可读消息类型发布出去

* **Subscriber**
1. "sweeper/area_info" [transmiter::AreasInfo]
	识别区域信息
	
2. "sweeper/env_info" [transmiter::EnvInfo]
	识别环境信息
	
* **Publisher**
1. "to_can_topic" [can_msgs::FrameArray]
	发布can数据帧，供给can驱动节点订阅后发送到总线

* **Parameters**
1. "to_can_topic" [string]　default: "to_can_topic"
	向外发布can数据帧的ros话题　
	
2. "from_can_topic" [string ] default: "from_can_topic"
	订阅can分析仪发布的can数据帧

3. "interval" [float] default: 5.0
    发布区域信息的时间间隔

### Node logger_node 测试文件记录节点
1. 订阅环境识别原始消息以及编码后的can消息，将其保存于txt文档
2. 文档内容包括 时间戳	[区域ID,垃圾等级,行人有无,植被类型,]...	16进制can消息(按照0-7排布)
3. 节点自带数据检验功能, 将can消息解码后与原始数据对比, 判断正确性


