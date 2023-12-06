# apollo_ros api

## 1 ros api

### 1.1 sim_pose

```c++
geometry_msgs::PoseStamped sim_pose;
sim_pose.header.frame_id = map_frame_id_;
sim_pose.header.stamp = ros::Time::now();;
sim_pose.pose.position.x = viz_point.x();
sim_pose.pose.position.y = viz_point.y();
sim_pose.pose.position.z = 0.0;
sim_pose.pose.orientation = GetQuaternionFromRPY(0.0, 0.0, point.path_point().theta());
```

![image-20231026173701929](D:\Work_Station\Documents\note\apollo\apollo_ros_7.0\images\image-20231026173701929.png)



## 2 apollo api

### 2.1 localization

***modules/apollo_msgs/proto/apollo_msgs/proto/localization/localization.proto***

***modules/apollo_simulator/src/sim_control.cpp***

```c++
LocalizationEstimate localization;					// 376
...
AdapterManager::PublishLocalization(localization);	// 424
```

```c++
struct PointENU {
  	double x;
  	double y;
    double z;
};

struct Quaternion {
    double qx;
    double qy;
    double qz;
    double qw;
};

struct Point3D {
    double x;
    double y;
    double z;
};

struct LocalizationEstimate {
    double heading;
	PointENU position;
    Quaternion orientation;
    Point3D linear_velocity;
    Point3D linear_acceleration;
    Point3D angular_velocity;
    Point3D linear_accelaration_vrf;
    Point3D anguler_velocity_vrf;
};
```









## 3 51sim

### 3.1 Publish

#### 3.1.1 SimVehicleStateInfo

51sim发布自车的状态信息

仿真启动时，一直发布自车状态信息

```c++
struct SimPublishVehicleStateInfo {
  // coordinates
  double x;
  double y;
  double z = 0.0;
  // direction on the x-y plane
  double heading;
  // curvature on the x-y planning
  double kappa;
  // linear velocity
  double speed;  // in [m/s]
  // linear acceleration
  double a;
};
```



#### 3.1.2 SimObstacleInfo

51sim发布障碍物信息

当添加一个障碍物时，会自动发布障碍物信息

```c++
struct SimPubObstacleInfo {
  // coordinates
  double x;
  double y;
  double z = 0.0;
  // obstacle linear velocity
  double speed;  // in [m/s]
  // obstacle direction on the x-y plane
  double heading;
  // obstacle length width height
  double length;
  double width;
  double height;
};
```



### 3.2 Subscribe

#### 3.2.1 SimLocalizationInfo

51sim订阅定位信息

目前没有控制模块，基于理想下的车辆控制，车辆会订阅系统给的定位信息，仿真器中直接同步车辆位姿状态

```c++
struct SimSubLocalizationInfo {
  double x;
  double y;
  double z = 0;
  double heading;
};
```



#### 3.2.2 SimSubRoutingPath

```c++
visualization_msgs::MarkerArray routing_path;
```



#### 3.2.3 SimSubReferenceLine

```c++
nav_msgs::Path reference_line;
```



#### 3.2.4 SimSubPlanningTrajectory

```c++
nav_msgs::Path palnning_trajectory;
```

















