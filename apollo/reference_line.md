# 1	reference line介绍

## 1.1 reference line的作用

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100735480.png" alt="image-20230925100735480" style="zoom: 50%;" />

参考线贯穿整个planning模块，是规划模块的基础



## 1.2 reference line在planning中的流程

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100801714.png" alt="image-20230925100801714" style="zoom:50%;" />

* **Routing模块**

  根据道路的拓扑结构和起始点与终点，搜索出一跳轨迹，通过AStar算法

  是一个宏观而且静态的模块，只要没有道路堵塞和障碍物，Routing只会规划一次

* **reference_line模块**

  以Routing生成的路线为基础，根据车辆的实时位置来计算参考线

  参考线会考虑车辆周围的动态信息和交通规则

* **trajectory**

  基于参考线，进行决策规划和轨迹优化，最终输出一条更为具体，更为局部的的轨迹

  除了包含参考线的信息外，还会包含别的信息，速度、方向、加速度等

  

## 1.3 为什么要生成reference line

​		***routing给的路径不够平滑***

***		导航路径太长，减少不必要的规划***



# 2	reference line模块

## 2.1 ReferenceLine数据结构

***apollo/modules/planning/reference_line/reference_line.h***

```c++
class ReferenceLine {
/**
 *...
 **/
 private:
  struct SpeedLimit {                               	// 限速的结构类型
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  std::vector<SpeedLimit> speed_limit_;					// 多段限速
  std::vector<ReferencePoint> reference_points_;		// 参考点
  hdmap::Path map_path_;                              	// hdmap中的参考线
  uint32_t priority_ = 0;                             	// 优先级
};
```

**ReferencePoint**

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100818925.png" alt="image-20230925100818925" style="zoom:50%;" />

在`ReferencePoint` 中`kappa` 和 `dkappa` 的跳变，都会影响方向盘的抖动，进而影响乘坐的舒适



## 2.2 reference line流程

参考线通过pnc地图和定位信息获取地图中的参考线

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100835539.png" alt="image-20230925100835539" style="zoom: 50%;" />

参考线平滑提供三种平滑算法，基于不同对象，选择不同的平滑算法

* 离散点平滑（apollo中默认）
* 螺旋线平滑
* 样条曲线平滑

## 2.3 reference line生成

![image-20230925100859057](D:\Work_Station\Documents\note\apollo\images\image-20230925100859057.png)



## 2.4 reference line平滑

### 2.4.1 平滑算法配置

三种平滑算法：`离散点参考线平滑` 、`螺旋线参考线平滑` 、`样条曲线参考线平滑` 

其中**离散点参考线平滑**又分为 `CosThetaSmooth` 和 `FemPosSmooth` 

![image-20230925100912291](D:\Work_Station\Documents\note\apollo\images\image-20230925100912291.png)

通过配置文件配置来选择不同的平滑器

***apollo/modules/planning/conf***

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100925681.png" alt="image-20230925100925681" style="zoom:50%;" />



***apollo/modules/planning/common/planning_gflags.cc***

```c++
DEFINE_string(smoother_config_filename,
              "/apollo/modules/planning/conf/qp_spline_smoother_config.pb.txt",
              "The configuration file for qp_spline smoother");
```



***apollo/modules/planning/reference_line/reference_line_provider.cc*** 

```c++
if (smoother_config_.has_qp_spline()) {
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_spiral()) {
    smoother_.reset(new SpiralReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_discrete_points()) {
    smoother_.reset(new DiscretePointsReferenceLineSmoother(smoother_config_));
  } else {
    ACHECK(false) << "unknown smoother config "
                  << smoother_config_.DebugString();
  }
```



### 2.4.2 平滑算法流程

***modules/planning/reference_line/reference_line_provider.cc***

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100942615.png" alt="image-20230925100942615" style="zoom:50%;" />

* **step1** ：设置中间点 `anchor points`

  ***modules/planning/reference_line/reference_line_provider.cc***

  <img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100959575.png" alt="image-20230925100959575" style="zoom:50%;" />

​		平滑的过程中，只有首尾两个点是强约束点

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925101012763.png" alt="image-20230925101012763" style="zoom:50%;" />

* **step2** ：配置选择平滑算法

  ***modules/planning/reference_line/discrete_points_reference_line_smoother.cc***

  ![image-20230925101032357](D:\Work_Station\Documents\note\apollo\images\image-20230925101032357.png)

* **step3** ：FemPosSmooth

  ![image-20230925101057395](D:\Work_Station\Documents\note\apollo\images\image-20230925101057395.png)

* **step4** ：求解器Solve函数

  ***modules/planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.cc***

  ![image-20230925101120277](D:\Work_Station\Documents\note\apollo\images\image-20230925101120277.png)

​		Apollo默认是使用无曲率约束的求解器，速度更快



### 2.4.3 平滑问题求解

![image-20230925101136950](D:\Work_Station\Documents\note\apollo\images\image-20230925101136950.png)

![image-20230925101150156](D:\Work_Station\Documents\note\apollo\images\image-20230925101150156.png)

![image-20230925101205368](D:\Work_Station\Documents\note\apollo\images\image-20230925101205368.png)



# 3 reference line仿真

这个工具直接生成轨迹

![image-20230925101222699](D:\Work_Station\Documents\note\apollo\images\image-20230925101222699.png)