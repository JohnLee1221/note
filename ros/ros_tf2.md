# 1 tf_tree

## 1.1 rqt_tf_tree

```shell
rosrun rqt_tf_tree rqt_tf_tree
```

![image-20230627220726896](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230627220726896.png)



## 1.2 view_frames

view_frames创建由tf2通过ROS广播的帧的图表

```shell
rosrun tf2_tools view_frames.py
```

显示

```shell
evince frames.pdf
```

![image-20230627221122389](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230627221122389.png)

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

![image-20230627221804864](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230627221804864.png)
