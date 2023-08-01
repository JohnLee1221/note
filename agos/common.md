# common module



##  1 architecture

```shell
.
├── CMakeLists.txt
├── conf
│   ├── adapter.conf
│   └── test.conf
├── include
│   └── common
│       ├── adapter_gflag.h
│       ├── adapter.hpp
│       ├── adapter_manager.hpp
│       ├── adapter_message.h
│       ├── clock.hpp
│       ├── file.hpp
│       └── macro.h
├── package.xml
├── README.md
├── src
│   └── adapter_gflag.cpp
└── test
    ├── adapter_manager_pub.cpp
    ├── adapter_manager_sub.cpp
    ├── adapter_test.cpp
    └── timer_test.cpp
```



## 2 develop

### 2.1 DECLARE_SINGLETON

***agos/modules/common/include/common/macro.h***

定义了3个宏，实现单例模式的宏定义

* DISALLOW_COPY_AND_ASSIGN
* DISALLOW_IMPLICIT_CONSTRUCTORS
* DECLARE_SINGLETON



### 2.2 File parse system

***agos/modules/common/include/common/file.hpp***

提供了几个函数用来解析和转换文件：

```c++
// 转换proto文件到ASCII文件
bool Proto2ASCIIFile(const Message& message, const std::string& file_name);

// 转换ASCII文件到proto文件
bool ASCIIFile2Proto(const std::string& file_name, Message* message);

// 转换二进制文件到proto文件
bool BinaryFile2Proto(const std::string& file_name, Message *message);

// 转换文件到proto的函数，内部会调用BinaryFile2Proto和ASCIIFile2Proto
bool File2Proto(const std::string& file_name, Message *message);
```



提供额外的功能函数：

```c++
// 验证文件名`original`是不是以`pattern`字符结尾
bool EndWith(const std::string& original, const std::string& pattern);

// 查询路径是否存在
bool DirectoryExists(const std::string& dir_path);

// 删除所有文件函数
bool RemoveAllFiles(const std::string &dir_path)
```



### 2.3 Clock

实现了一个时钟功能，主要是获取系统的时间戳

```c++
// 时间戳类型为Timestamp
using Timestamp = std::chrono::time_point<std::chrono::system_clock, Duration>;

// 获取时间戳函数
static Timestamp Now()；
```



提供了一些转换函数

```c++
template <typename PrecisionDuration>
int64_t AsInt64(const Duration& duration);

template <typename PrecisionDuration>
int64_t AsInt64(const Timestamp& timestamp);

// 间隔时间转换成秒
inline double ToSecond(const Duration& duration);
// 时间戳转换成秒
inline double ToSecond(const Timestamp& timestamp)
```



用法

```c++
double timestamp = agos::common::ToSecond(agos::common::Clock::Now());
```



### 2.4 Adapter

实现了适配器类类，主要功能是创建了两个队列： `data_queue_`  、 `observed_queue_` ，并封装了一些方法

* **std::list<std::shared_ptr<D>> data_queue_**

  数据队列，用于存储通讯接收到的数据，队列大小通过配置文件配置

* **std::list<std::shared_ptr<D>> observed_queue_**

  观测数据队列，获取当前 `data_queue_` 中的数据

```c++
// Adapter构造函数
Adapter(const std::string &adapter_name, const std::string &topic_name,
          size_t message_num) : adapter_name_(adapter_name),
          topic_name_(topic_name), message_num_(message_num);

// Adapter中的Ros类型的回到函数，适配Subscriber<std_msgs::String>的定义
void ROSCallback(const std_msgs::String::ConstPtr &message);
```

**Adapter**类中的方法不直接调用方法，上层通过适配管理器封装了Adapter类的方法



### 2.5 AdapterManager

适配管理器类，基于ros的通讯协议封装了Adapter类，在代码中以**懒汉模式**实现AdapterManager的实例

> ros是单线程节点，多进程间通讯。一个节点中只会持有一个**AdapterManager**的单例对象并初始化一次，所以不会出现线程安全问题，如果节点中起多线程，可能会出现不安全。



* **std::vector<std::function<void()>> observes_；**

​		内部定义了一个任务队列observes_，每个元素是一个 `std::function<void()>` 类型的函数对象



```c++
// 初始化函数
static void Init(int argc, char **argv);

static void Init(int argc, char **argv,
                 const std::string& adapter_config_filename);

// AdapterManagerConfig是在adapter_config.proto中定义的message AdapterManagerConfig
// 在这个初始化函数中加载并遍历初始化不同的Adapter对象
static void Init(int argc, char **argv, 
                 const AdapterManagerConfig& configs);
```



```c++
// 观测函数，遍历并调用任务队列中的任务函数
static void Observe();
```



```c++
// 一次发布适配器，类似于ros中的一次发布
static void OncePublishAdapter(const DataType& data);

// 循环发布适配器，按照一定频率发布
static void OnPublishAdapter(DataType& data, std::function<void(DataType&)> callback)；
```



```c++
// 接收观测到Adapter中数据列表中最新数据，并把数据传递到callback回调函数中执行处理
static void ReceiveLatestObserved(std::function<void(const DataType&)> callback)；
    
// 接收观测到Adapter中数据列表中的最老数据，并把数据传递到callback回调函数中执行处理
static void ReceiveOldestObserved(std::function<void(const DataType&)> callback);
```



```c++
// 调用ros::waitForShutdown()函数
static void WaitForShutdown();
```



**AdapterManager用法**

* step1

  在工程目录下添加自己要用到的适配器配置文件

  ***agos/modules/.../conf/adapter.conf***

  ```shell
  config {
    type: LOCALIZATION			#要生成的适配器类型
    mode: PUBLISH_ONLY			#通讯类型
    message_history_limit: 10		#数据队列大小
    rate: 1						#通讯频率
  }
  is_ros: true					#是否为ros通讯
  node_name: "test_node"			#节点名称
  
  ```

* step 2

  定义话题名称

  ***modules/common/include/common/adapter_gflag.h***

  ```c++
  DECLARE_string(adapter_config_path);
  
  DECLARE_string(localization_topic);
  DECLARE_string(planning_topic);
  DECLARE_string(control_topic);
  DECLARE_string(chassis_topic);
  
  ```

  ***modules/common/src/adapter_gflag.cpp***

  ```c++
  DEFINE_string(adapter_config_path, "", "the file path of adapter config file");
  
  DEFINE_string(chassis_topic,        "/agos/chassis",        "chassis topic name");
  DEFINE_string(localization_topic,   "/agos/localization",   "localization topic name");
  DEFINE_string(planning_topic,       "/agos/planning",       "planning topic name");
  DEFINE_string(control_topic,        "/agos/control",        "control topic name");
  ```

* step 3

  设置要添加的适配器类型

  ***modules/common/include/common/adapter_message.h***

  ```c++
  using ChassisAdapterManager = AdapterManager<Adapter<::agos::chassis::Chassis>>;
  ```

* step 4

  代码调用

  可以参考测试代码

  ***modules/common/test/adapter_manager_pub.cpp***

  ***modules/common/test/adapter_manager_sub.cpp***

  