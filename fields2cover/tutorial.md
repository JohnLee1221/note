# Fields2Cover tutorial

<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230518131004887.png" alt="image-20230518131004887"  />

## 1 basic type

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



## 2 Objectives

代价函数模块

![image-20230525125328768](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230525125328768.png)



### 2.1 SGObjective



#### 2.1.1 FieldCoverage

计算swath在田地的占比



#### 2.1.2 NSwath

计算swath的数量



#### 2.1.3 Overlaps

计算swath之间的重合区域与整个field的比例



#### 2.1.4 SwathLength

计算swath的总长度













