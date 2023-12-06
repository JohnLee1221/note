# 1. 作用

产生路径边界。根据场景进行判断，然后将生成的边界信息存入，std::vector candidate_path_boundaries中。

<img src="D:\Work_Station\Documents\note\apollo\apollo_ros_7.0\tasks\images\image-20231103160831206.png" alt="image-20231103160831206" style="zoom:67%;" />

# 2. 数据结构

主要的数据结构是PathBoundary类

其主要作用是：储存路径边界的相对于参考线SL坐标点序列；储存frenet系起始纵坐标，储存阻塞障碍物id。

**modules\planning\common\path_boundary.cc/.h**