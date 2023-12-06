# Msg_FilteredPerceptionResult

```c
uint32_t	msg_id			// 消息编号，固定值：8
uint32_t	msg_size		// Msg_FilteredPerceptionResult消息所占字节大小，固定值
uint64_t	timestamp		// 本地时间戳，icgm上电时刻+397系统时间
uint32_t	frame_id		// 数据帧编号，从0开始每发一次增1
uint32_t	check_sum		// 整个MSG的checksum值
PrcedObj					prced_obj		// 只包含车辆，行人，自行车
PrcedCone					prced_cone		// 雪糕筒类型障碍物
PrcedWarningTriangle		prced_warning_Triangle		// 三脚架类型障碍物
PrcedLine					prced_line_lane
PrcedLine_Lane_Road_Edge	prced_line_lane_road_edge	// 包含前后两条，但目前定义有疑问，道路边缘形成的原因有很多（如curb, guardrail, concretebarrier, fence, wall）
PrcedLine_Lane_Cone			prced_line_lane_cone	// 1-left_front_line:2-right_front_line;3-left_rear_line; 4-right_rear_line
PrcedLine_Lane_HPP			prced_line_lane_hpp	
RoadCrossWalkLine			road_cross_walk_line	
RoadMarkingStopline			road_marking_stopline	
PrcedTSR					prced_tsr	
IHBC						ihbc	
WorkCondition				work_condition	
ImageFail					image_fail	
Trig						trig	
uint8_t		vcu_delay_status	// 车身can信号链路的延迟状态
uint8_t		radar_delay_status	// 雷达数据信号数据传输链路的延迟状态
uint8_t		swc_delay_status	// 车身控制信号传输链路的延迟状态
uint8_t		j3a_delay_status	// 前视数据的传输链路的延迟状态，150ms，3个周期作为故障检测状态跳转触发条件
uint8_t		j3b_delay_status	// 周视融合输出的传输链路的延迟状态，150ms，3个周期作为故障检测状态跳转触发条件
uint8_t		fusion_status		// 融合模块的状态位;返回1,正常;返回值不为1,模块不正常;200ms，四个周期作为故障检测状态跳转触发条件
uint8_t		filter_status		// 过滤模块的状态位;返回1,正常;返回值不为1,模块不正常;200ms，四个周期作为故障检测状态跳转触发条件
uint8_t		rsv					// 保留位
```







## PrcedObj

```c
uint8_t		PrcedObj_Res6				// reserve
uint8_t		PrcedObj_Source				// 目标来源
uint8_t 	PrcedObj_LostAge			// 目标丢失帧数
uint8_t 	PrcedObj_Types				// 目标类型
uint8_t 	PrcedObj_MtnSts				// 目标运动状态
uint8_t 	PrcedObj_MtnCategory		// 目标运动类别
uint8_t 	PrcedObj_LaneInfo			// 目标所在车道信息
uint8_t 	PrcedObj_BrakeLight			// 刹车灯状态
uint8_t 	PrcedObj_TurnLight			// 转向灯状态
uint8_t 	PrcedObj_CIPVFlag			// closest in path vehicle
uint16_t 	PrcedObj_ObjID				// 目标id
uint32_t 	PrcedObj_Age				// 目标持续帧数
float32_t 	PrcedObj_HeadingAngle		// 目标航向角度
float32_t 	PrcedObj_YawRate			// 目标横摆角度
float32_t 	PrcedObj_PosX				// 目标和自车相对纵向距离
float32_t 	PrcedObj_PosY				// 目标横向相对距离
float32_t 	PrcedObj_LongSpd			// 目标绝对纵向速度
float32_t 	PrcedObj_LatSpd			// 目标绝对横向速度
float32_t 	PrcedObj_Ax				// 目标绝对加速度
float32_t 	PrcedObj_Ay				// 目标绝对横向加速度
float32_t 	PrcedObj_LongRelSpd		// 目标和自车相对纵向速度
float32_t 	PrcedObj_LatRelSpd		// 目标横向相对速度
float32_t 	PrcedObj_RelAx			// 目标绝对加速度
float32_t 	PrcedObj_RelAy			// 目标绝对横向加速度
float32_t 	PrcedObj_Height			// 目标高度
float32_t 	PrcedObj_Length			// 目标长度
float32_t 	PrcedObj_Width			// 目标宽度
float32_t 	PrcedObj_ObjConf			// 目标可靠度
float32_t 	PrcedObj_ObjClassConf		// 目标类型可靠度
float32_t 	PrcedObj_TTC				// ttc时间
float32_t 	PrcedObj_HeadingAngleVar	// 目标朝向角度方差
float32_t 	PrcedObj_YawRateVar		// 目标横摆角速度方差
float32_t 	PrcedObj_PosXVar			// 目标和自车相对纵向距离方差
float32_t 	PrcedObj_PosYVar			// 目标横向相对距离方差
float32_t 	PrcedObj_LongSpdVar		// 目标绝对纵向速度方差
float32_t 	PrcedObj_LatSpdVar		// 目标绝对横向速度方差
float32_t 	PrcedObj_AxVar			// 目标绝对加速度方差
float32_t 	PrcedObj_AyVar			// 目标绝对横向加速度方差
float32_t 	PrcedObj_LongRelSpdVar	// 目标和自车相对纵向速度方差
float32_t 	PrcedObj_LatRelSpdVar		// 目标横向相对速度方差
float32_t 	PrcedObj_RelAxVar			// 目标相对加速度方差
float32_t 	PrcedObj_RelAyVar			// 目标相对横向加速度方差
uint64_t 	PrcedObj_Res1				// reserve
uint64_t 	PrcedObj_Res2				// reserve
uint64_t 	PrcedObj_Res3				// reserve
uint64_t 	PrcedObj_Res4				// reserve
uint64_t 	PrcedObj_Res5				// reserve
```



| type                 | value                                                        |
| :------------------- | :----------------------------------------------------------- |
| PrcedObj_Source      | 0:undefined 4:HisRadar 5:Radar_only 8:HisVision 10:Vision_only 12:HisRadarVision 13:RadarVision |
| PrcedObj_Types       | 0:undefined 1:car 2:bus 3:truck 4:pedstrain 5:bicyle 6:motorbike 7:animal |
| PrcedObj_MtnSts      | 0:undefined 1:moving 2:stationary 3:stopped 4:moving slowly  |
| PrcedObj_MtnCategory | 0:unknown 1:forward 2:oncoming 3:FW_cut-in 4:FW_cut-out 5:OC_cut-in 6:OC_cut-out 7:crossing |
| PrcedObj_LaneInfo    | 0:unknown 1:InPath 2:left 3:right 4:next left 5:next right 6:CutHostVehicleLeftPath 7:CutHostVehicleRightPath |
| PrcedObj_BrakeLight  | 0:unknown 1:on 2:off                                         |
| PrcedObj_TurnLight   | 0:unknown 1:left 2:right 3:double                            |
| PrcedObj_CIPVFlag    | 0:unknown 1:CIPV 2:not CIPV                                  |







## PrcedLine

```c
uint8_t		Status					// 车道线性质状态
uint8_t		Crossing				// 车辆跨线标志位，至少三个周期
uint8_t		Lanemark_Type			// 车道线类型
uint8_t		LaneColor				// 车道线颜色
uint8_t		DECEL_Type				// 减速线类型
uint8_t		exist_vir_to_real
uint8_t		exist_real_to_vir
uint8_t		rsv9
float32_t	Confidence				// 主车道线置信度	/百分比
float32_t	Lanemark_Type_Conf		// 车道线类型置信度	
float32_t	C0						// 车道线C0	/m
float32_t	C1						// 车道线C1	/rad
float32_t	C2						// 车道线C2	/INV(m)
float32_t	C3						// 车道线C3	/INV(m2)
float32_t	Start_X					// 车道线起始距离	/m
float32_t	End_X					// 车道线终止距离	/m
float32_t	MarkerWidth				// 车道线宽度	/m
float32_t	rsv10
float32_t	rsv11
float32_t	vir_to_real_x
float32_t	vir_to_real_y
float32_t	real_to_vir_x
float32_t	real_to_virl_y
uint8_t		rsv1
uint8_t		rsv2
uint8_t		rsv3
uint8_t		rsv4
float32_t	rsv5
float32_t	rsv6
float32_t	rsv7
float32_t	rsv8
```



| type          | value                                                        |
| :------------ | :----------------------------------------------------------- |
| Status        | 0:Unavalible 1:Predicted 2:Detected                          |
| Crossing      | 0:not crossing 1:crossing                                    |
| Lanemark_Type | 0:undefined 1:Solid 2:Dashed 3:DLMSolidDashed 4:DLMDashedSolid 5:DLMDashed 6:DLMSolid 7:DLMUndefined |
| LaneColor     | 0:undefined 1:White 2:Yellow 3:bule 4:green                  |
| DECEL_Type    | 0:NOTDECEL 1:SOLID 2:DASHED 3:UNDECIDED 4:INVALID            |







## PrcedLine_Lane_Road_Edge

```c
uint8_t		LRE_Status			// 道路边缘性质状态：

uint8_t		LRE_TYPE			// 道路边缘类型：
uint8_t		rsv9	
float32_t	LRE_Confidence		// 道路边缘线置信度
float32_t	LRE_C0				// 道路边缘C0
float32_t	LRE_C1				// 道路边缘C1
float32_t	LRE_C2				// 道路边缘C2
float32_t	LRE_C3				// 道路边缘C3
float32_t	LRE_Start_X			// 道路边缘起始距离
float32_t	LRE_End_X			// 道路边缘终止距离
float32_t	LRE_Height			// 道路边缘高度
float32_t	dtlc	
float32_t	ttlc	
uint8_t		rsv1	
uint8_t		rsv2	
uint8_t		rsv3	
uint8_t		rsv4	
float32_t	rsv5	
float32_t	rsv6	
float32_t	rsv7	
float32_t	rsv8	
```



| type       | value                                                        |
| ---------- | ------------------------------------------------------------ |
| LRE_Status | 0:Unavalible 1:Predicted 2:Detected                          |
| LRE_TYPE   | 0:undefined 1:Road edge 2:Curb 路肩 3:Barrier 护栏  4:ConesPoles 雪糕筒边缘 5:ParkedCars 路边车辆 |







## PrcedCone

```c
uint8_t		PrcedCone_curr_lane			// 雪糕桶在当前车道
uint8_t		PrcedCone_Status			// 雪糕筒是否存在的状态
uint8_t		PrcedCone_Conf
uint8_t		PrcedCone_Age
uint16_t	PrcedCone_ObjID
uint16_t	rsv9
float32_t	PrcedCone_PosX
float32_t	PrcedCone_PosY
float32_t	PrcedCone_ettc				// 强化距离碰撞时间(ettc，enhanced time to collision)
float32_t	PrcedCone_ttc				// 与雪糕桶的碰撞时间
float32_t	PrcedCone_heading
uint8_t		rsv1
uint8_t		rsv2
uint8_t		rsv3
uint8_t		rsv4
float32_t	rsv5
float32_t	rsv6
float32_t	rsv7
float32_t	rsv8
```







## PrcedWarningTriangle

```c
uint16_t	PrcedWarningTriangle_ObjID
uint8_t		PrcedWarningTriangle_Status
uint8_t		PrcedWarningTriangle_Conf
uint8_t		PrcedWarningTriangle_Age
uint8_t		PrcedWarningTriangle_curr_lane
uint8_t		rsv9
float32_t	PrcedWarningTriangle_PosX
float32_t	PrcedWarningTriangle_PosY
float32_t	PrcedWarningTriangle_ettc
float32_t	PrcedWarningTriangle_ttc
float32_t	PrcedWarningTriangle_heading
uint8_t		rsv1
uint8_t		rsv2
uint8_t		rsv3
uint8_t		rsv4
float32_t	rsv5
float32_t	rsv6
float32_t	rsv7
float32_t	rsv8
```







## PrcedLine_Lane_Cone

``` c
uint8_t		Status						// 有效位
uint8_t		TYPE	 
uint8_t		rsv9	
float32_t	Confidence				// 马路边缘线置信度
float32_t	C0						// 马路边缘C0
float32_t	C1						// 马路边缘C1
float32_t	C2						// 马路边缘C2
float32_t	C3						// 马路边缘C3
float32_t	Start_X					// 马路边缘起始距离
float32_t	End_X					// 马路边缘终止距离
float32_t	dtlc	
float32_t	ttlc	
uint8_t		rsv1	
uint8_t		rsv2	
uint8_t		rsv3	
uint8_t		rsv4	
float32_t	rsv5	
float32_t	rsv6	
float32_t	rsv7	
float32_t	rsv8	
uint8_t		rsv10	
```



| type | value                                                        |
| ---- | ------------------------------------------------------------ |
| TYPE | 0: undefined 1: Road edge 2: Curb 3: Barrier 4: ConesPoles 5: ParkedCars |







## PrcedLine_Lane_HPP

```c
uint8_t		LHPP_Valid						// HPP有效位 0 invalid 1valid
uint8_t		LHPP_Is_Construction_Area		// Indicates whether this is a construction area scene.0 invalid 1valid
uint8_t		LHPP_Is_Highway_Merge_Left		// Indication for highway merge left to the host.0 invalid 1valid
uint8_t		LHPP_Is_Highway_Merge_Right		// Indication for highway merge right to the host.0 invalid 1valid
uint8_t		LHPP_Is_Highway_Exit_Left		// Indication for highway exit left to the host.0 invalid 1valid
uint8_t		LHPP_Is_Highway_Exit_Right		// Indication for highway exit right to the host.0 invalid 1valid
uint8_t		LHPP_Path_Pred_Source			// HPP预测来源
uint8_t		rsv9	
float32_t	LHPP_Confidence	HPP				// 置信度
float32_t	LHPP_C0							// HPP车道线C0
float32_t	LHPP_C1							// HPP车道线C1
float32_t	LHPP_C2							// HPP车道线C2
float32_t	LHPP_C3							// HPP车道线C3
float32_t	LHPP_Start						// HPP车道线起始距离
float32_t	LHPP_End						// HPP车道线终止距离
float32_t	LHPP_Path_Pred_Host_Width		// 车道宽度
uint8_t		rsv1	
uint8_t		rsv2	
uint8_t		rsv3	
uint8_t		rsv4	
float32_t	rsv5	
float32_t	rsv6	
float32_t	rsv7	
float32_t	rsv8	
uint8_t		rsv10	
```



| type                  | value                                                        |
| --------------------- | ------------------------------------------------------------ |
| LHPP_Path_Pred_Source | 0: 无效 1:两侧车道线 2:单侧车道线 3:单侧马路牙子 4:单侧车道线和马路牙子 5:前车历史轨迹 6:地图自车目标轨迹线 7:其他（包含其他目标物如施工区域/锥桶/路边停止车辆等边界拟合线） |







## RoadCrossWalkLine

```c
uint8_t		LINE_RCWL_isRelevant		// Specify whether the stop line is relevant to the host vehicle.
uint8_t		LINE_RCWL_Status			// 基于图像还是基于运动预测进行人行道线测量
uint8_t		LINE_RCWL_id				// StopLine's id
uint8_t		LINE_RCWL_Type				// Type of stop line
uint8_t		LINE_RCWL_ColorType			// Color of stop line
uint8_t		LINE_RCWL_curr_lane	
uint8_t		rsv9	
float32_t	LINE_RCWL_lateralDist		// The lateral distance to the center of the road marking
float32_t	LINE_RCWL_longitudinalDist	// The longitudinal distance to the center of the road marking
float32_t	LINE_RCWL_confidence		// Probability that the object is actually a real stop line
float32_t	LINE_RCWL_length	
float32_t	LINE_RCWL_width	
float32_t	LINE_RCWL_height	
float32_t	LINE_RCWL_ettc	
float32_t	LINE_RCWL_ttc	
float32_t	LINE_RCWL_angle				// Stop line angle in real world in radians. positive angle or negative angle
uint8_t		rsv1	
uint8_t		rsv2	
uint8_t		rsv3	
uint8_t		rsv4	
float32_t	rsv5	
float32_t	rsv6	
float32_t	rsv7	
float32_t	rsv8	
```



| type                 | value                                                        |
| -------------------- | ------------------------------------------------------------ |
| LINE_RCWL_isRelevant | 0:not relevant 1: relevant                                   |
| LINE_RCWL_Status     | 0:Unknown 1:In_Image 2:Predicted                             |
| LINE_RCWL_Type       | 0=before traffic lights crossroads (single solid)<br/>1= stop to give way line (double solid)<br/>2= decelarate to give way line (double dashed)<br/>3= before pedestrion crosing<br/>4= end of tuning waiting area(right) <br/>5= end of tuning waiting area(left) <br/>6= direct-type entrance marking(dashed)<br/>7= direct-type exit marking(dashed)<br/>8= reserved<br/>9= reserved" |
| LINE_RCWL_ColorType  | 0=Green_Blue 1=White 2=Yellow_Orange_Red                     |







## RoadMarkingStopline

```c
uint8_t		LINE_RdSL_isRelevant		// Specify whether the stop line is relevant to the host vehicle.
uint8_t		LINE_RdSL_Status			// 基于图像还是基于运动预测进行人行道线测量
uint8_t		LINE_RdSL_id				// StopLine's id
uint8_t		LINE_RdSL_Type				// Type of stop line
uint8_t		LINE_RdSL_ColorType			// Color of stop line
uint8_t		LINE_RdSL_curr_lane	
uint8_t		rsv9	
float32_t	LINE_RdSL_lateralDist		// The lateral distance to the center of the road marking
float32_t	LINE_RdSL_longitudinalDist	// The longitudinal distance to the center of the road marking
float32_t	LINE_RdSL_confidence		// Probability that the object is actually a real stop line
float32_t	LINE_RdSL_length	
float32_t	LINE_RdSL_width	
float32_t	LINE_RdSL_height	
float32_t	LINE_RdSL_ettc	
float32_t	LINE_RdSL_ttc	
float32_t	LINE_RdSL_angle				// Stop line angle in real world in radians. positive angle or negative angle
uint8_t		rsv1	
uint8_t		rsv2	
uint8_t		rsv3	
uint8_t		rsv4	
float32_t	rsv5	
float32_t	rsv6	
float32_t	rsv7	
float32_t	rsv8	
```



| type                 | value                                                        |
| -------------------- | ------------------------------------------------------------ |
| LINE_RdSL_isRelevant | 0:not relevant 1: relevant                                   |
| LINE_RdSL_Status     | 0:Unknown 1:In_Image 2:Predicted                             |
| LINE_RdSL_Type       | 0=before traffic lights crossroads (single solid)<br/>1= stop to give way line (double solid)<br/>2= decelarate to give way line (double dashed)<br/>3= before pedestrion crosing<br/>4= end of tuning waiting area(right) <br/>5= end of tuning waiting area(left) <br/>6= direct-type entrance marking(dashed)<br/>7= direct-type exit marking(dashed)<br/>8= reserved<br/>9= reserved" |
| LINE_RdSL_ColorType  | 0=Green_Blue 1=White 2=Yellow_Orange_Red                     |







## PrcedTSR

Processed Traffic Sign Recognition

```c
uint8_t		PrcedTSR_ID	
uint8_t		PrcedTSR_SupplementalClass1		// 标志牌类型：36-限速标志牌 37—解除限速
uint8_t		PrcedTSR_SupplementalClass2		// 限速值
uint8_t		PrcedTSR_RelevantDecision
uint8_t		PrcedTSR_FilterType
uint8_t		rsv9	
uint16_t	PrcedTSR_Class					// Mono3.0交通标志对照表-release rev0.3_20210325
float32_t	PrcedTSR_PosX	
float32_t	PrcedTSR_PosY	
float32_t	PrcedTSR_PosZ	
float32_t	PrcedTSR_Conf	
uint8_t		rsv1	
uint8_t		rsv2	
uint8_t		rsv3	
uint8_t		rsv4	
float32_t	rsv5	
float32_t	rsv6	
float32_t	rsv7	
float32_t	rsv8	
uint8_t		rsv10	
```



| type                      | value                                                        |
| ------------------------- | ------------------------------------------------------------ |
| PrcedTSR_RelevantDecision | 0=RELEVANT_SIGN<br/>1=HIGHWAY_EXIT_SIGN<br/>2=LANE_ASSIGNMENT_SIGN<br/>3=PARRALEL_ROAD_SIGN<br/>4=SIGN_ON_TURN<br/>5=FAR_IRRELEVANT_SIGN<br/>6=INTERNAL_SIGN_CONTRADICTION<br/>7=ERROR_SIGN_CODE<br/>8= CIPV_IN_FRONT<br/>9= CONTRADICT_ARROW_SIGN<br/>10=OTHER_FILTER_REASON" |
| PrcedTSR_FilterType       | 0=NO_SLI_FILTER,<br/>1=IRRELEVANT_SIGN,<br/>2=TRUCK_FILTER,<br/>3=EMBEDDED_FILTER,<br/>4=MINIMUM_FILTER,<br/>5=ROAD_NUMBER_FILTER" |







## IHBC

智能大灯	Intelligent high beam control system

```c
uint8_t		IHBC_light_on			// 用于更新AHBC信息的视觉框架的索引
uint8_t		IHBC_object_num			// 用于更新AHBC信息的摄像机图像的索引
uint8_t		rsv1	
float32_t	IHBC_cal_lux_up			// 计算AHBC信息时使用的车辆状态数据的索引
float32_t	rsv4	
float32_t	rsv5	
```







## WorkCondition

IHBC工作环境

```c
uint8_t		WorkCondition_weathertype
uint8_t		WorkCondition_scenetype
uint8_t		WorkCondition_timetype
uint8_t		WorkCondition_lighttype
uint8_t		rsv1
uint8_t		rsv2
uint8_t		rsv3
uint8_t		rsv6
float32_t	rsv4
float32_t	rsv5
```







## ImageFail

```c
uint8_t		ImageFail_type
uint8_t		ImageFail_level
uint8_t		ImageFail_score
uint8_t		rsv1
uint8_t		rsv2
uint8_t		rsv3
uint8_t		rsv6
float32_t	rsv4
float32_t	rsv5
```







## Trig

```c
uint64_t	timestamp			// 触发时间，格式为年月日时分秒，例如211118090623
uint32_t	check_sum			// 整个MSG的checksum值
uint8_t		src					// 触发源信息，1/2/3/4 分别表示J3A/J3B/821和397规控触发
uint8_t		event				// 触发事件代号，用一个整型表示即可，详见功能需求文档相关章节，触发场景相关内容
uint8_t		rsv1	
```



