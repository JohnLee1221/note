# 1 tf_tree

## 1.1 rqt_tf_tree

```shell
rosrun rqt_tf_tree rqt_tf_tree
```

<img src="D:\Work_Station\Documents\note\ros\images\image-20230925102736556.png" alt="image-20230925102736556" style="zoom: 33%;" />



## 1.2 view_frames

view_frames创建由tf2通过ROS广播的帧的图表

```shell
rosrun tf2_tools view_frames.py
```

显示

```shell
evince frames.pdf
```

<img src="D:\Work_Station\Documents\note\ros\images\image-20230925102750962.png" alt="image-20230925102750962" style="zoom: 33%;" />

# 2 tf_echo

tf_echo报告通过ROS广播的任意两帧之间的转换

```shell
rosrun tf tf_echo [reference_frame] [target_frame]
```



```shell
rosrun tf tf_echo turtle1 turtle2
```



# 3 rviz and tf2

在turtle_tf2配置文件中使用rviz的-d选项

```shell
rosrun rviz rviz -d `rospack find turtle_tf2`/rviz/turtle_rviz.rviz
```

<img src="D:\Work_Station\Documents\note\ros\images\image-20230925102806609.png" alt="image-20230925102806609" style="zoom:50%;" />
