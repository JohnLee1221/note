```shell
//初始位置话题
/initialpose

//数据类型	geometry_msgs/PoseWithCovarianceStamped

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
```

* position：为坐标信息，对2d来说只有x和y值。

* orientation为四元数格式，参考相关链接：四元数与欧拉角之间的转换

​		对2d平面的移动机器人感官上易于理解的就是朝向信息，即欧拉角中绕z轴旋转的偏航角。

​		俯仰角和滚转角为0，故x和y均为0，即只有w和z值。

​		若航向角为alpha，则w = cos（alpha/2），z = sin（alpha/2）。




```
header:
  seq: 3
  stamp:
    secs: 1683697637
    nsecs: 354891332
  frame_id: "map"
pose:
  pose:
    position:
      x: -6.73139762878418
      y: -2.428220748901367
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.9750576095884136
      w: 0.22195192719084178
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
```

