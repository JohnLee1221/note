# Fields2cover architecture

## 1 Overview

<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230518131004887.png" alt="image-20230518131004887"  />

## 2 Introduction

​		由于农用车的非自主性，必须在田地中预留一个被称为头顶的区域供车辆转向。最基本的方法是在**田地周围分配一个恒定宽度的区域**。这种策略将大量的空间分配给产量低的区域。根据田间地头的排列方式，有些地头区域与作业方向平行，因此它们不需要转弯。通过**只在转弯的田间边缘建造地头区域**，可以最大限度地减少为其保留的面积。

**headland**: 地头

**swath**：作业幅宽

<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230518135729554.png" alt="image-20230518135729554" style="zoom: 67%;" />

​		在内场中生成斜线，内场是减去地头后的剩余区域。在二维平面场中，可以用一条参考线作为生成作业线的参考，每条作业线都会基于这个参考线平行生成。这条线可以为了方便而选择，也可以通过诸如蛮力或元启发式的算法来选择。Oksanen描述了一种驱动角搜索策略，它比蛮力搜索需要更少的迭代，但它不能保证找到全局最小值。目标函数，如转弯的数量或扫描长度的总和，被用来确定扫描生成中的最优性。

覆盖田野所需的距离和时间受各条沟的顺序影响。路线是指要覆盖的条形图的顺序

* Boustrophedon order，牛耕式，即从田野的一边走到另一边，按顺序覆盖田地；
* Snake order，蛇形模式，在每个转弯处跳过一条沟，并通过未覆盖的沟返回；
* Spiral order：螺旋形模式，是蛇形模式的一个变种，它以蛇形模式的固定大小的群组来排序条带。



## 3 Modules

### 3.1 Headland Generator





### 3.2 Swath Generator





### 3.3 Route Planner





### 3.4 Path Planner

