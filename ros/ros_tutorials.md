



## 1 ros支持的基本数据类型

![image-20230529163627713](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230529163627713.png)

http://wiki.ros.org/msg



## 2 创建并编译一个ros包

```shell
# 初始化一个工作空间
mkdir -p test_ws/src
cd test_ws/src
catkin_init_workspace

# 创建一个ros包
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
cd test_ws/src
catkin_create_pkg test roscpp std_msgs

# 编译工作空间
cd test_ws
catkin_make
catkin_make --pkg package1	# 指定编译package1包
catkin_make --exclude package1 # 指定跳过编译package1包
```



## 3 分布式设置

export ROS_MASTER_URI=http://xxx:11311



## 4 ros topic

### rostopic list -v

在rostopic list中使用verbose选项

```shell
rostopic list -v
```

```shell
Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher
 * /rosout [rosgraph_msgs/Log] 2 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
```



### rostopic pub

`rostopic pub`可以把数据发布到当前某个正在广播的话题上。

```shell
rostopic pub [topic] [msg_type] [args]
```

![image-20230510223953586](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230510223953586.png)



* 发布一次话题数据

```shell
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

* 按照1hz频率发布数据

```shell
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```



**发布一个** *geometry_msgs/PoseWithCovarianceStamped* **类型的数据** 

> 查看数据类型：
>
> > ![image-20230510233302996](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproamytyproaimage-20230510233302996.png)

> 打印一组数据：
>
> > <img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230510233536404.png" alt="image-20230510233536404" style="zoom:80%;" />

> 发布一次话题：
>
> > <img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230510233640623.png" alt="image-20230510233640623" style="zoom:80%;" />





### rostopic hz

`rostopic hz`报告数据发布的速率。pose

```shell
rostopic hz /turtle1/pose
```

```
rostopic pub -r 1 /initialpose geometry_msgs/PoseWithCovarianceStamped -- "header:
  seq: 3
  stamp:
    secs: 1683731901
    nsecs: 758897644
  frame_id: 'map'
pose:
  pose:
    position:
      x: -7.089181423187256
      y: -2.2342958450317383
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.979904614645722
      w: 0.19946665434608124
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"
```





## 5 srv

### 5.1 自定义srv

1. 在package下面新建一个srv文件夹，并创建一个.srv文件

```shell
// 以test_ws为下的test为例子
cd ~/test_ws/src/test/
mkdir srv
cd srv
touch AddTwoInts.srv
```



2. 编辑.srv文件

```yaml
int64 a
int64 b
---
int64 sum
```



3. 配置文件

* package.xml

  打开如下两条语句

  ```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```

* CMakeLists.txt

  ```cmake
  # find_package中添加message_generation
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
  )
  
  # 打开add_service_files标签，并添加对应的AddTwoInts.srv文件
  add_service_files(
    FILES				#默认的，不需要动
    AddTwoInts.srv
  )
  
  # 打开generate_message标签
  generate_messages(
    DEPENDENCIES		#默认，不动
    std_msgs
  )
  ```



4. 编译并查看

   ```shell
   cd ~/test_ws/src
   catkin_make
   
   source devel/setup.bash
   rossrv show test/AddTwoInts
   
   # 生成的头文件路径
   cd ~/test_ws/devel/include/test
   ```

   

### 5.2 srv的命令

```shell
# rossrv show <service_type>
rossrv show test/AddTwoInts
```



## 6 ros service

```shell
rosservice list         输出活跃服务的信息
rosservice call         用给定的参数调用服务
rosservice type         输出服务的类型
rosservice find         按服务的类型查找服务
rosservice uri          输出服务的ROSRPC uri
```



```shell
# 服务列表
rosservice list

# /add_two_ints服务的信息
rosservice info /add_two_ints

# 查看/add_two_ints中服务的具体类型
rosservice type /add_two_ints | rossrv show
```



终端请求一个服务

```shell
# /add_two_ints中test/AddTwoInts服务的具体类型

# int32 a
# int32 b
# ---
# int32 sum

# 请求一个服务
rosservice call /add_two_ints 1 99
```



