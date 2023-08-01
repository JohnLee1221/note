# 适配器模块

定义了一个适配器，基于这个模块，可以直接添加通讯接口

## 1 文件结构

头文件：

<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230706145036334.png" alt="image-20230706145036334" style="zoom:80%;" />

源文件：

<img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230706145255674.png" alt="image-20230706145255674" style="zoom:80%;" />

## 2 文件说明

### 2.1 adapter_gflags

声明并定义gflag的全局变量

**adapter_gflags.h** 

```c++
DECLARE_string(raw_trajectory_topic);
```

**adapter_gflags.cc**

```c++
// 参数名称：raw_trajectory_topic
// 默认值：/apollo/raw_trajectory
// 描述信息：raw trajectory topic name
DEFINE_string(raw_trajectory_topic, "/apollo/raw_trajectory",
              "raw trajectory topic name");
```



```c++
// 得到全局变量
FLAGS_raw_trajectory_topic
// 等同于std::string FLAGS_raw_trajectory_topic = "/apollo/raw_trajectory"
```



### 2.2 adapter

实现了一个Adapter的类模板

```c++
template <typename D>
class Adapter;
```





