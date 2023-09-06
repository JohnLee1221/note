# Fields2Cover functional specification

<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230518131004887.png" alt="image-20230518131004887" style="zoom:80%;" />



## 1 types

![image-20230625112250279](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230625112250279.png)





### 1.1 Geometry

简单的几何点和几何线的类

整个基础类型的基类，内部存储了一个智能指针，管理所有数据类型

```c++
// OGRwkbGeometryType
template <class T, OGRwkbGeometryType R>
struct Geometry {
// ...
 protected:
  std::shared_ptr<T> data; 
}；
```

**OGRwkbGeometryType** 是一个***枚举类*** ，定义了一种几何类型的标识符，包括点、线、多边形等。

```c++
typedef enum
{
    wkbUnknown = 0,         /**< unknown type, non-standard */
    wkbPoint = 1,           /**< 0-dimensional geometric object, standard WKB */
    wkbLineString = 2,      /**< 1-dimensional geometric object with linear
                             *   interpolation between Points, standard WKB */
    wkbPolygon = 3,         /**< planar 2-dimensional geometric object defined
                             *   by 1 exterior boundary and 0 or more interior
                             *   boundaries, standard WKB */
    wkbMultiPoint = 4,      /**< GeometryCollection of Points, standard WKB */
    wkbMultiLineString = 5, /**< GeometryCollection of LineStrings, standard WKB */
    wkbMultiPolygon = 6,    /**< GeometryCollection of Polygons, standard WKB */
    wkbGeometryCollection = 7, /**< geometric object that is a collection of 1
                                    or more geometric objects, standard WKB */
    wkbCircularString = 8,  /**< one or more circular arc segments connected end to end,
                             *   ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbCompoundCurve = 9,   /**< sequence of contiguous curves, ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbCurvePolygon = 10,   /**< planar surface, defined by 1 exterior boundary
                             *   and zero or more interior boundaries, that are curves.
                             *    ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbMultiCurve = 11,     /**< GeometryCollection of Curves, ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbMultiSurface = 12,   /**< GeometryCollection of Surfaces, ISO SQL/MM Part 3. GDAL &gt;= 2.0 */
    wkbCurve = 13,          /**< Curve (abstract type). ISO SQL/MM Part 3. GDAL &gt;= 2.1 */
    wkbSurface = 14,        /**< Surface (abstract type). ISO SQL/MM Part 3. GDAL &gt;= 2.1 */
    wkbPolyhedralSurface = 15,/**< a contiguous collection of polygons, which share common boundary segments,
                               *   ISO SQL/MM Part 3. Reserved in GDAL &gt;= 2.1 but not yet implemented */
    wkbTIN = 16,              /**< a PolyhedralSurface consisting only of Triangle patches
                               *    ISO SQL/MM Part 3. Reserved in GDAL &gt;= 2.1 but not yet implemented */
    wkbTriangle = 17,         /**< a Triangle. ISO SQL/MM Part 3. Reserved in GDAL &gt;= 2.1 but not yet implemented */
} OGRwkbGeometryType;
```



#### 1.1.1 Point

描述一个点的类

```c++
struct Point : public Geometry<OGRPoint, wkbPoint>;

// 构造
F2CPoint p1 (1.2, 3.4);
F2CPoint p2 (9.8, 7.6, 5.4);
F2CPoint p3 (OGRPoint(11, 22));
F2CPoint p4;
```



#### 1.1.2 Geometries

描述几何体，几何图形的一个类

```c++
// OGRwkbGeometryType
template <class SAMETYPE, class T, OGRwkbGeometryType R, class CHILDRENTYPE>
struct Geometries : public Geometry<T, R>;
```

##### MultiPoint

WKB标准的，几何集合点

```c++
struct MultiPoint :
  public Geometries<MultiPoint, OGRMultiPoint, wkbMultiPoint, Point>;

// 构造
F2CMultiPoint points {F2CPoint(1, 2), F2CPoint(3, 4)};
```

##### LinearRing

闭合环的类，通过一个Point集合进行初始化

```c++
struct LinearRing : 
  public Geometries<LinearRing, OGRLinearRing, wkbLinearRing, Point>;

// 构造
F2CLinearRing ring{F2CPoint(1, 1), F2CPoint(3, 1), F2CPoint(2,2), F2CPoint(1,2), F2CPoint(1,1)};
```

##### LineString

描述一个线段的类

```c++
struct LineString : public Geometries<LineString, OGRLineString, wkbLineString, Point>;

// 构造
F2CLineString line1;
line1.addPoint(3,0);
line1.addPoint(p5);
F2CLineString line2({F2CPoint(1, 0), F2CPoint(1, 1), F2CPoint(0, 1)});
```

##### MultiLineString

`LineString` 的集合

```c++
struct MultiLineString :
    public Geometries<MultiLineString, OGRMultiLineString, wkbMultiLineString, LineString>;

// 构造
F2CMultiLineString lines;
lines.addGeometry(line1);
lines.addGeometry(line2);
```

##### Cell

表示多边形的类

```c++
struct Cell : public Geometries<Cell, OGRPolygon, wkbPolygon, LinearRing>;
```

##### Cells

一个不重叠的多个 `Cell` 类

```c++
struct Cells : public Geometries<Cells, OGRMultiPolygon, wkbMultiPolygon, Cell>;
```



### 1.2 Path

描述路径的类，包含道路状态 `PathState` （Point、angle、velocity等等）和任务时间，误差精度等

```c++
enum class PathSectionType {
  SWATH = 1,
  TURN = 2,
};

enum class PathDirection {
  FORWARD = 1,
  BACKWARD = -1,
  // STOP = 0,
};

struct PathState {
  Point point;
  double angle;
  double velocity;
  double duration;
  PathDirection dir;
  PathSectionType type;
};

struct Path {
 public:
  std::vector<PathState> states;
  double task_time {0.0};
  double measure_error {0.1};
};
```



### 1.3 Robot

描述车辆的状态，车辆的基本参数等

```c++
struct Robot {
 public:
  double op_width {0.0};
  double robot_width {0.0};  // Distance between wheels
  std::optional<Point> start_point;
  std::optional<Point> end_point;
  double cruise_speed {1.0};
  double max_icc {1.0};  // [1/m]
  double linear_curv_change {2.0};  // [1/m^2]
  std::optional<double> max_accel;
  std::optional<double> min_accel;
  std::optional<double> max_vel;
  std::optional<double> min_vel;
  std::optional<double> max_u_front;
  std::optional<double> min_u_front;
  std::optional<double> max_u_rot;
  std::optional<double> min_u_rot;
};
```



### 1.4 Route

路径信息类

```c++
struct Route {
 public:
  std::vector<Swaths> v_swaths;
  std::vector<MultiPoint> connections;
  RouteType type;
};
```



### 1.5 Swath

作业幅宽

```c++
struct Swath {
 private:
  int id_;
  LineString path_;
  double width_ {0.0};
  bool creation_dir_ {true};
};

// 实例化
F2CSwath swath1(F2CLineString({F2CPoint(0.0, 1.0), F2CPoint(4.0, 1.0)}), width);
```



### 1.6 Swaths

多个作业幅宽

```c++
struct Swaths {
 private:
  std::vector<Swath> data;
};

//
F2CSwaths swaths({swath1, swath2, swath3})
```



### 1.7 Strip

包含`Cell` 类的一个类

```c++
struct Strip {
  f2c::types::Cell cell;
  std::string name;
};
```





### 1.8 OptimizationParams

一些优化参数的结构体

```c++
struct OptimizationParams {
 public:
  double best_angle;
  double headland_width;
  uint16_t max_turns;
  double cost_if_no_crop;
  double greening_subsidy;
  double increment;
  double penalty_extra_swath;
};
```



### 1.9 Field

田地信息，包含参考点和多边形

```c++
struct Field {
  std::string id {""};
  std::string coord_sys {""};
  Point ref_point;
  Cells field;
};
```

#### *Field* -> *Point* 

Method to get the raw point data in the ***Field*** data type

获取Field数据类型中的原始点位数据方法

```c++
F2CCells cells = fields_[0].field;
size_t cs_size = cells.size();
for (size_t i = 0; i < cs_size; ++i) {
    F2CCell cell = cells.getCell(i);
    size_t c_size = cell.size();
    for (size_t j = 0; j < c_size; ++j) {
        F2CLinearRing linear_ring;
        linear_ring = cell.getGeometry(i);
        size_t lr_size = linear_ring.size();
        for (size_t k = 0; k < lr_size; ++k) {
            F2CPoint point;
            point = linear_ring.getGeometry(k);
            std::cout << point.getX() << "\t" << point.getY() << "\t" << point.getZ() << std::endl; 
        }
    }
}
```





## 2 headland_generator

田间地头生成器模块



<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230625132034964.png" alt="image-20230625132034964" style="zoom:67%;" />

### 2.1 HeadlandGeneratorBase

生成田间地头的基类



### 2.2 ConstHL

constant headland

一个可以生成每个边界都是等宽的田间地头类

继承自HeadlandGeneratorBase基类



## 3 swath_generator

作业幅宽生成器模块

<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230625130008363.png" alt="image-20230625130008363" style="zoom:67%;" />



### 3.1 SwathGeneratorBase

作业幅宽生成器的基类



### 3.2 BruteForce

继承自SwathGeneratorBase基类

重写generateBestSwaths方法







## 4 bjectives

代价函数模块

![image-20230625112649634](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230625112649634.png)



### 4.1 BaseObjective

计算带有符号的代价值，是一个实现多态的父类，整个模块的入口



### 4.2 SGObjective

Swath Generator Objective，继承自BaseObjective基类

SwathGeneratorBase类的方法依赖SGObjective对象 

SGObjective自身是一个基类，通过派生类实现computeCost的方法



> #### FieldCoverage
>
> * SGObjective的派生类
>
> * 计算swath在田地的占比百分比，其中swaths都是相同的宽度
>
> #### NSwath
>
> * SGObjective的派生类 Number of Swath
>
> * 计算swath的数量
>
> * 这个成本函数假设转弯速度比穿过swaths更慢。更少的区域意味着更少的转弯和更快的路径。
>
> #### Overlaps
>
> * SGObjective的派生类
>
> * 计算swath之间的重合区域与整个field的比例
>
> #### SwathLength
>
> * SGObjective的派生类
>
> * 计算swath的总长度



### 4.3 RPObjective

Route Planner Objective，继承自BaseObjective基类

路由规划器的基类



> #### DirectDistPathObj
>
> * RPObjective的派生类
> * 路径的代价函数，路线中各点之间的直线距离
>
> #### CompleteTurnPathObj
>
> * RPObjective的派生类
> * 计算从一个点转到另一个点的代价
> * 这是最接近执行路径计划的实际成本的结果，因为它实际上计算了所有的转弯



### 4.4 PPObjective

Path Planner Objective，继承自BaseObjective基类

计算path的长度代价函数基类



> #### PathLength
>
> * PPObjective的派生类
> * 空实现



### 4.5 HGObjective

Headland Generator Objective，继承自BaseObjective基类

田间地头生成器代价函数的基类



> #### RemArea
>
> * HGObjective的派生类
> * 重写了isMinimizing的函数实现





## 5 route_planning

路由规划类

![image-20230625124830024](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230625124830024.png)



### 5.1 SingleCellSwathsOrderBase

路由方式的基类，路由规划通过派生类继承重新方法

实现了通过蛇形、牛耕式、行星式等规划方法



### 5.2 BoustrophedonOrder

牛耕式路由方法

继承自SingleCellSwathsOrderBase基类



### 5.3 SnakeOrder

蛇形路由方法

继承自SingleCellSwathsOrderBase基类



### 5.4 SpiralOrder

行星式路由方法

继承自SingleCellSwathsOrderBase基类



### 5.5 CustomOrder

自定义方式，预留自定义接口类

继承自SingleCellSwathsOrderBase基类



## 6 path_palnning

转弯规划器的基类模块

![image-20230625112942196](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230625112942196.png)



### 6.1 PathPlanning

使用TurningBase类方法连接路径的路径规划类

使用Turning算法将一个条形图按顺序连接到下一个条形图

Pathplanning关联到TuringBase类的对象



### 6.2 TuringBase

提供了转弯方法的接口

`createSimpleTurn`方法通过继承实现



### 6.2 DubinsCurves

Dubins曲线规划器

继承自TuringBase基类



### 6.3 DubinsCurvesCC

Dubins' curves planner with continuous curves

包含连接曲线的Dubins曲线规划器

继承自TuringBase基类



### 6.4 ReedsSheppCurves

Reeds-Shepp曲线规划器

继承自TuringBase基类



### 6.5 ReedsSheppCurvesHC

Reeds-Shepp's curves planner with continuous curves

包含连接曲线的Reeds-Shepp曲线规划器

继承自TuringBase基类







