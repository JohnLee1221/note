

# ros tutorials

## 1 ros支持的基本数据类型

![image-20230529163627713](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230529163627713.png)

http://wiki.ros.org/msg



## 2 创建一个ros包

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
```



## 3 package.xml文件

```xml
<package format="2">
  <name>foo_core</name>															<!-- 名称  -->
  <version>1.2.4</version>														<!-- 版本 -->
  <description>
    This package provides foo capability.
  </description>																<!-- 描述 -->
  <maintainer email="ivana@willowgarage.com">Ivana Bildbotz</maintainer>		<!-- 维护者 -->
  <license>BSD</license>

  <url>http://ros.org/wiki/foo_core</url>
  <author>Ivana Bildbotz</author>

  <buildtool_depend>catkin</buildtool_depend>									<!-- 构建工具 -->

  <depend>roscpp</depend>
  <depend>std_msgs</depend>

  <!-- 依赖列表 -->
  <build_depend>message_generation</build_depend>

  <!-- 运行时依赖 -->
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>rospy</exec_depend>

  <!-- 测试依赖 -->
  <test_depend>python-mock</test_depend>

  <doc_depend>doxygen</doc_depend>
</package>
```

### depend

等价于 <build_depend> + <build_exprot_depend> + <exec_depend>

### <build_depend> 

​	构建依赖项指定了构建这个包需要哪些包。当构建时需要这些包中的任何文件时，就会出现这种情况。这可以是在编译时包括这些包的头文件，针对这些包的库进行链接，或者在构建时需要任何其他资源（特别是当这些包在 CMake 中被 find_package()-ed 时）。在交叉编译的情况下，构建依赖是针对目标架构的。

### <build_export_depend>

​	构建输出依赖指定了哪些包需要针对这个包来构建库。当你把它们的头文件包含在这个包的公共头文件中时就是这种情况（特别是当这些包在 CMake 的 catkin_package() 中被声明为 (CATKIN_)DEPENDS 时）。

### <exec_depend>

​	执行依赖指定了哪些包需要运行此包中的代码。当你依赖这个包中的共享库时就是这种情况（特别是当这些包在CMake的catkin_package()中被声明为(CATKIN_)DEPENDS时）。

### <test_depend>

​	测试依赖仅指定单元测试的额外依赖。它们不应该与任何已经提到的构建或运行的依赖关系重复。

### <buildtool_depend>

​	构建工具依赖项指定该包需要的构建系统工具，以构建自身。一般来说，唯一需要的构建工具是 catkin。在交叉编译的情况下，编译工具的依赖性是针对编译所处的架构。

### <doc_depend>

​	文档工具依赖项指定了该软件包生成文档所需的文档工具。



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





## 5 使用srv

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



```shell
# /add_two_ints中test/AddTwoInts服务的具体类型

# int32 a
# int32 b
# ---
# int32 sum

# 请求一个服务
rosservice call /add_two_ints 1 99
```



## 7 ros param

```shell
# rosparam list           列出参数名
rosparam list

# rosparam set            设置参数
rosparam set 1000

# rosparam get            获取参数
rosparam get /turtlesim/background_g

# rosparam dump           向文件中转储参数
# rosparam dump [file_name] [namespace]
rosparam dump params.yaml

# rosparam load           从文件中加载参数
rosparam load params.yaml

rosparam delete         删除参数
```



## 8 ros launch

