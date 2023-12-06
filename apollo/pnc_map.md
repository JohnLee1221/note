# 1 Overview

## 1.1 什么是pnc map

PncMap是Planning and Control Map的缩写，即用于规划控制使用的地图，Planning模块收到Routing发送的导航轨迹信息后，将其转为可以用于规划控制地图。PncMap功能实现位于Planning模块的reference_line_provider中，为参考线的生成提供基础数据。

pnc_map在Apollo中属于相对独立的一块内容，作为hd_map与planning的中间层，在生成reference_line即规划参考线的时候会使用到。换句话说，pnc_map就是解析routing结果，再由每个路由段转换为reference_line的数据形式，这是它的主要功能。实现这一模块的目的也是为了让规划模块的独立性更高，避免与高精地图的数据格式所耦合，导致可迁移性变差。



## 1.2 pnc map的作用 

***pnc map的作用就是将routing的结果，进行一系列的操作处理，找到一个局部的(固定长度)，可以供规划使用的，车辆能够正常驶入的局部地图（RouteSegments），后续用于生成规划中的ReferenceLine结构***



# 2 Data Structure

首先，需要了解pnc_map中用到的几个地图数据结构。因为pnc_map中一个重要功能就是处理routing数据，因此首先分析下routing的proto文件

## 2.1 proto的数据类型

### VehicleState

***modules/common/vehicle_state/proto/vehicle_state.proto***

```protobuf
import "modules/common_msgs/chassis_msgs/chassis.proto";
import "modules/common_msgs/localization_msgs/pose.proto";

message VehicleState {
  optional double x = 1 [default =0.0];            // 车辆世界ENU坐标系x坐标
  optional double y = 2 [default =0.0];            // 车辆世界ENU坐标系y坐标
  optional double z = 3 [default =0.0];            // 车辆世界ENU坐标系z坐标
  optional double timestamp = 4 [default =0.0];    // 时间戳信息
  optional double roll = 5 [default =0.0];         // 车辆姿态相对于世界坐标系x轴旋转角度
  optional double pitch = 6 [default =0.0];        // 车辆姿态相对于世界坐标系y轴旋转角度
  optional double yaw = 7 [default =0.0];          // 车辆姿态相对于世界坐标系z轴旋转角度
  optional double heading = 8 [default =0.0];      // 车辆速度方向
  optional double kappa = 9 [default =0.0];        // 车辆半径倒数1/R
  optional double linear_velocity = 10 [default =0.0];      // 车辆线速度
  optional double angular_velocity = 11 [default =0.0];     // 车辆角速度
  optional double linear_acceleration = 12 [default =0.0];  // 车辆线加速度
  optional apollo.canbus.Chassis.GearPosition gear = 13;    // 车辆齿轮状态，包含前进、倒车。停车、低速等状态
  optional apollo.canbus.Chassis.DrivingMode driving_mode = 14;  // 驾驶状态，包含手动驾驶、自动驾驶、转向、刹车与油门等状态
  optional apollo.localization.Pose pose = 15;     // 车辆姿态，包含坐标，局部到世界坐标系变换矩阵，线速度(矢量)，线加速度(矢量)等信息。
  optional double steering_percentage = 16;
}
```



### PointENU

***modules/common_msgs/basic_msgs/geometry.proto***

描述了在hd_map上的一个点

```protobuf
message PointENU {
  optional double x = 1 [default = nan];  // East from the origin, in meters.
  optional double y = 2 [default = nan];  // North from the origin, in meters.
  optional double z = 3 [default = 0.0];  // Up from the WGS-84 ellipsoid, in
                                          // meters.
}
```



### SLPoint

***modules/common_msgs/basic_msgs/pnc_point.proto***

描述了Frenet坐标系上的一个点

```protobuf
message SLPoint {
  optional double s = 1;
  optional double l = 2;
}
```



### LaneWaypoint

***modules/common_msgs/routing_msgs/routing.proto***

表示hd_map道路中的一个点

包含了道路id，s值，全局坐标信息

```protobuf
// 为路由模块的输入，车辆在全局地图中必须经过的点
// PointENU为东北天坐标系，apollo中被简化为二维横轴墨卡托坐标系
message LaneWaypoint {
  optional string id = 1;
  optional double s = 2;
  optional apollo.common.PointENU pose = 3;
  // When the developer selects a point on the dreamview route editing
  // the direction can be specified by dragging the mouse
  // dreamview calculates the heading based on this to support construct lane way point with heading
  optional double heading = 4;
}
```



### RoutingRequest

***modules/common_msgs/routing_msgs/routing.proto***

RoutingRequest为dreamview上发出的routing请求点，主要关注***waypoint***这个数据类型，表示routing算法导航出的路径需经过这些***waypoint***点。

**LaneWayPoint为地图道路上的一个点，包含了道路id，s值，全局坐标信息。**

```protobuf
message RoutingRequest {
  optional apollo.common.Header header = 1;
  // at least two points. The first is start point, the end is final point.
  // The routing must go through each point in waypoint.
  repeated LaneWaypoint waypoint = 2;
  repeated LaneSegment blacklisted_lane = 3;
  repeated string blacklisted_road = 4;
  optional bool broadcast = 5 [default = true];
  optional apollo.hdmap.ParkingSpace parking_space = 6 [deprecated = true];
  optional ParkingInfo parking_info = 7;
  optional DeadEndInfo dead_end_info = 8;
}
```



### LaneSegment

***modules/common_msgs/routing_msgs/routing.proto***

***LaneSegment***就是最基础的道路片断数据类型，包含了道路id，起始点终点s值信息

```protobuf
message LaneSegment {
  // id存储的为全局唯一的车道id，即定义在map_lane.proto中的Lane的id
  optional string id = 1;
  optional double start_s = 2;
  optional double end_s = 3;
}
```



### Passage

***modules/common_msgs/routing_msgs/routing.proto***

核心数据类型**LaneSegment**

**Passage**表示由多个道路片段组成的一个车道

```protobuf
message Passage {
  repeated LaneSegment segment = 1;
  // 当前passage是否可以道道下一个passage
  optional bool can_exit = 2;
  // 当前passage的换道类型
  optional ChangeLaneType change_lane_type = 3 [default = FORWARD];
}
```



### RoadSegment

***modules/common_msgs/routing_msgs/routing.proto***

核心数据**Passage**

**RoadSegment**表示由道路中多个车道（一条道路可能包含多条并行的通路）的集合组成

```protobuf
message RoadSegment {
  // 道路id
  optional string id = 1;
  repeated Passage passage = 2;
}
```



### RoutingResponse

***modules/common_msgs/routing_msgs/routing.proto***

RoutingResponse表示的是routing的结果信息

核心数据类型**RoadSegment**，描述routing规划得出的hd_map中多段道路集合

```protobuf
message RoutingResponse {
  optional apollo.common.Header header = 1;
  repeated RoadSegment road = 2;
  optional Measurement measurement = 3;
  optional RoutingRequest routing_request = 4;
  optional bytes map_version = 5;
  optional apollo.common.StatusPb status = 6;
}
```



## 2.2 pnc_map中的数据类型

### LaneWaypoint

***modules/map/pnc_map/path.h***

描述了pnc_map中车道上的点

与routing.proto中的LaneWaypoint有一些区别，它是pcn_map中直接拿取的道路点，包含道路信息以及s和l信息

```c++
struct LaneWaypoint {
  LaneInfoConstPtr lane = nullptr;
  double s = 0.0;
  double l = 0.0;
  std::string DebugString() const;
};
```



### LaneSegment

***modules/map/pnc_map/path.h***

是pcn_map中最基础的道路数据类型，和routing.proto中的LaneSegment类似，但包含了更多的信息。

```c++
struct LaneSegment {
  LaneInfoConstPtr lane = nullptr;	//车道中心线原始信息，包含道路点信息等
  double start_s = 0.0;
  double end_s = 0.0;
  double Length() const { return end_s - start_s; }
  static void Join(std::vector<LaneSegment>* segments);
  std::string DebugString() const;
};
```



### RouteSegments

***modules/map/pnc_map/route_segments.h***

**RouteSegments**与protobuf中的**Passage**类似，由多条相连的车道组成，它是由通道区域扩展而来，但保留了一些属性

在pnc_map中会将routing结果的**Passage**信息转换为**RouteSegments**，这两者其实可以看做同一东西

其包含了pnc_map的主要功能及输出结果，这个类中也有许多对路由段进行处理的函数，为之后更好地生成ReferenceLine做准备

```c++
class RouteSegments : public std::vector<LaneSegment> {
 private:
  LaneWaypoint route_end_waypoint_;
  bool can_exit_ = false;
  // 指示车辆是否在当前的RouteSegment.
  bool is_on_segment_ = false;
  // 指示当前路由段是否是车辆的邻居routeSegment.
  bool is_neighbor_ = false;
  routing::ChangeLaneType next_action_ = routing::FORWARD;
  routing::ChangeLaneType previous_action_ = routing::FORWARD;
  std::string id_;
  bool stop_for_destination_ = false;
};
```



## 2.3 Data中的关系

Routing模块将道路结构分解成多个**RoadSegment**，每个**RoadSegment**又包含多个通道**Passage**，每个**Passage**又由多个**LaneSegment**组成

![image-20230925100650360](D:\Work_Station\Documents\note\apollo\images\image-20230925100650360.png)

如图中的例子

* Routing规划的道路信息，其中红色点为起始点，橙色点为终点

* **RoutingResponse**给我们的**road**结果是：

  ```c+
  road = [RoadSegment1, RoadSegment2, RoadSegment3];
  ```

  ```c++
  RoadSegment1 = [Passage1, Passage2];
  RoadSegment2 = [Passage1, Passage2];
  RoadSegment3 = [Passage2];
  ```
  
  ```c++
  // RoadSegment1中的
  Passage1 = [LaneSegment1, LaneSegment2, LaneSegment3];
  Passage2 = [LaneSegment1, LaneSegment2, LaneSegment3];
  
  // ...
  ```

​		当我们为**RoadSegment**，**Passage**，**LaneSegment**编号时，就可以将Routing模块输出的地图道路信息中每个车道中的每个**LaneSegment**快速的索引出		来。如图中**（1, 1, 1）**代表左下角红圈所在的**LaneSegment**

* **Passage**中的`can_exit` ：代表能否从当前通道去下一个通道。用于判断将来是否需要变道。例如Passage1中can_exit为false， Passage2中为true。

* Passage中的`change_lane_type` ：表当前通道是直行（FORWARD），左转（LEFT）还是右转（RIGHT）。 图中Passage2为RIGHT， 因为需要右转才能到终点。Passage1为FORWARD。



**RoadSegment**，**Passage**，**LaneSegment**图解：

![image-20230925100712639](D:\Work_Station\Documents\note\apollo\images\image-20230925100712639.png)



# 3 PncMap processing flow

## 3.1 PncMap类的使用

pnc_map封装在reference_line_provider下面，主要供其在由routing模块给出的结果到Reference_Line的过程做准备。

**ReferenceLineProvider**如何使用的：

***modules/planning/reference_line/reference_line_provider.h***

```c++
// 声明一个pnc_map_的独占只能指针
class ReferenceLineProvider {
  // ...
 private:
  std::unique_ptr<hdmap::PncMap> pnc_map_;
  // ...
};
```



***modules/planning/reference_line/reference_line_provider.cc***

```c++
// 构造函数的时候初始化pnc_map_
ReferenceLineProvider::ReferenceLineProvider(
  // ...
  if (!FLAGS_use_navigation_mode) {
    pnc_map_ = std::make_unique<hdmap::PncMap>(base_map);
    relative_map_ = nullptr;
  } else {
    pnc_map_ = nullptr;
    relative_map_ = relative_map;
  }
  // ...
}
    
// 更新未来的路由信息
std::vector<routing::LaneWaypoint> ReferenceLineProvider::FutureRouteWaypoints() {
  if (!FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    return pnc_map_->FutureRouteWaypoints();
  }
  // ...
}
    
// 获取pnc_map_中的道路片段
bool ReferenceLineProvider::CreateRouteSegments(
    const common::VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> *segments) {
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!pnc_map_->GetRouteSegments(vehicle_state, segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }
  // ...
}
    
// 更新RoutingResponce提供的信息
bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (pnc_map_->IsNewRouting(routing)) {
      is_new_routing = true;
      if (!pnc_map_->UpdateRoutingResponse(routing)) {
        AERROR << "Failed to update routing in pnc map";
        return false;
    }
  }
}
    
// 扩展pnc_map_中道路的信息
bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,
                                                RouteSegments *segments,
                                                ReferenceLine *reference_line) {
  // ...
  if (!pnc_map_->ExtendSegments(*prev_segment, future_start_s, future_end_s,
                                &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);
  }
  // ...
}
```



## 3.2 PncMap内部方法

**PncMap**主要的功能流程分如下：

* 处理routing结果：这部分接收routing模块的路径查询响应，然后进行一定的处理，最后存储到map相关的数据结构，供后继使用

  ```c++
  bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing);
  ```

  通过传入的**routing::RoutingResponse**类型的routing消息，更新如下数据：

  * range_lane_ids_

    路由范围，随车辆状态不断更新车辆当前位置到终点的lane的范围

  * route_indices_

    存储完整的routing信息

  * all_lane_ids_

    存储routing轨迹所经过的所有车道id

  * adc_route_index_

    动态更新车辆的routing范围

    

* 更新**RouteSegments**结构

  ```c++
  bool PncMap::GetRouteSegments(const VehicleState &vehicle_state, std::list<RouteSegments> *const route_segments);
  
  bool PncMap::GetRouteSegments(const VehicleState &vehicle_state, const double backward_length,
                                const double forward_length, std::list<RouteSegments> *const route_segments);
  ```

在这之后，reference_line_provider 里会将**RouteSegments**转换为**hdmap::Path**结构，再转成**ReferenceLine**结构，继而进行平滑







https://blog.csdn.net/zhizhengguan/article/details/129029781

https://blog.csdn.net/lzw0107/article/details/107814610
