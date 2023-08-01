### adapter_manager.cc

67



### cotrol.cc

91

101

108



### planning_node.cc

67









### palnning.launch

 <node pkg="apollo_planning" type="raw_path_node" name="row_path" args = "

 --flagfile=$(find apollo_planning)/conf/planning.conf

 --adapter_config_path=$(find apollo_planning)/conf/adapter.conf

 --rtk_trajectory_filename=$(find apollo_planning)/data/garage.csv"

 output="screen"/>



**apollo_simulator.h**

```c++
// 146
  // ===========================================lzh====================================================start
  void SubCallback(const apollo_simulator::integratedNavigationMsg& msg) {
    special_pose_.position.x = msg.Lattitude;
    special_pose_.position.y = msg.Longitude;
    special_pose_.position.z = msg.Heading;
    ROS_INFO_STREAM("------- Rreceiving rtk data -------");
    ROS_INFO_STREAM("------- Lattitude: " << msg.Lattitude << "\t"
        << "Longitude: " << msg.Longitude << "\t" << "Heading: "
        << msg.Heading << "-------");
  }
  // 计时器Timer周期间隔回调函数
  void RtkPoseCallback(const ros::TimerEvent &event) {
    ros::NodeHandle nh;
    sub_localization_ = nh.subscribe("/integratedNavigation/data", 100, &Simulator::SubCallback, this);
    TimerCallbackSimulation(event);
  }
  // geometry_msgs::Pose类型，position中的x y z代表的是空间上的位置，这里临时代表横坐标、纵坐标和航向角(航向角为弧度制)
  geometry_msgs::Pose special_pose_;        // 定义一个包含横坐标，纵坐标和航向的特殊位置点
  ros::Subscriber sub_localization_;        // 添加一个订阅定位的话题
  ros::Publisher pub_chassis_twist_;        // 发布底盘控制话题
  // ===========================================lzh====================================================end
```





**apollo_simulator.cpp** 

```c++
// 73
  // ===========================================lzh====================================================start
  /* Timer */
  // timer_simulation_ = nh.createTimer(ros::Duration(dt), &Simulator::TimerCallbackSimulation, this);
  
  // 通过计时器周期性调用RtkPoseCallback的函数
  timer_simulation_ = nh.createTimer(ros::Duration(dt), &Simulator::RtkPoseCallback, this);
  // ===========================================lzh====================================================end

// 243
    // ===========================================lzh====================================================start
    // linear_vel和steer_angle是从apollo_control获取到的控制量，把这个两个量发送到底盘
    // geometry_msgs::Twist数据类型中的 linear.x 默认单位是 m/s
    // geometry_msgs::Twist数据类型中的 angular.z 默认单位是 弧度/s，这里代表的是角度
    ros::NodeHandle nh;
    geometry_msgs::Twist chassis_signal;
    chassis_signal.linear.x = linear_vel;                     // 线速度
    chassis_signal.angular.z = steer_angle * 180.0 / M_PI;    // steer_angle从弧度转换成角度
    ROS_INFO_STREAM("-------------- publish linear velocity:\t"
                    << chassis_signal.linear.x << "m/s");
    ROS_INFO_STREAM("-------------- publish steer angle:\t\t"
                    << chassis_signal.angular.z << "degree");
    pub_chassis_twist_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_chassis_twist_.publish(chassis_signal);
    // ===========================================lzh====================================================end

// 287
  // ===========================================lzh====================================================start
  // current_pose_.position.x = vehicle_model_ptr_->getX();
  // current_pose_.position.y = vehicle_model_ptr_->getY();
  // closest_pos_z_ = 0.0;
  // current_pose_.position.z = closest_pos_z_;
  // double roll = 0.0;
  // double pitch = 0.0;
  // double yaw = vehicle_model_ptr_->getYaw();
  // current_twist_.linear.x = vehicle_model_ptr_->getVx();
  // current_twist_.angular.z = vehicle_model_ptr_->getWz();

  // 更改current_pose_.position.x
  //     current_pose_.position.y的赋值，从vehicle_model_ptr_改变成rtk
  current_pose_.position.x = special_pose_.position.x;      // 获取到rtk定位信息横坐标
  current_pose_.position.y = special_pose_.position.y;      // 获取到rtk定位信息纵坐标
  closest_pos_z_ = 0.0;
  current_pose_.position.z = closest_pos_z_;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = special_pose_.position.z;                    // 获取到rtk的航向角
  current_twist_.linear.x = 2;
  current_twist_.angular.z = 0;
  std::cout << "========>   current_pose_.position.x : " << current_pose_.position.x << std::endl;
  std::cout << "========>   current_pose_.position.y : " << current_pose_.position.y << std::endl;
  std::cout << "========>   yaw                      : " << yaw << std::endl;
  std::cout << "========>   current_twist_.linear.x  : " << current_twist_.linear.x << std::endl;
  std::cout << "========>   current_twist_.angular.z : " << current_twist_.angular.z << std::endl;
  // ===========================================lzh====================================================end
```



![image-20230626173558108](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230626173558108.png)





![image-20230628105707377](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230628105707377.png)