# 适配器模块

定义了一个适配器，基于这个模块，可以直接添加通讯接口

## 1 文件结构

头文件：

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100246541.png" alt="image-20230925100246541" style="zoom: 67%;" />

源文件：

<img src="D:\Work_Station\Documents\note\apollo\images\image-20230925100312135.png" alt="image-20230925100312135" style="zoom: 80%;" />

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



### 2.2 Adapter类

实现了一个Adapter的类模板

```c++
template <typename D>
class Adapter : public AdapterBase;
```



几个重要的成员函数

#### 2.2.1 EnqueueData

```c++
void EnqueueData(DataPtr data) {
  // ...
}
```

`std::list<DataPtr> data_queue_`

维护一个固定长度的list : `data_queue_`，每调用一次，把data入列一次

#### 2.2.2 FireCallbacks

```c++
  void FireCallbacks(const D& data) {
    for (const auto& callback : receive_callbacks_) {
      callback(data);
    }
  }
```

循环调用一次回调函数列表 ： `std::vector<Callback> receive_callbacks_` 中的回调函数

#### 2.2.3 RosCallback

```c++
  void RosCallback(const std_msgs::String::ConstPtr& message) {
    D data;
    if (!data.ParseFromString(message->data)) {
      AERROR << "Failed to parse proto-message.";
      return;
    }
    last_receive_time_ = apollo::common::time::Clock::NowInSeconds();
    auto data_ptr = boost::make_shared<D const>(data);
    EnqueueData(data_ptr);
    FireCallbacks(*data_ptr);
  }
```

ros通讯接口的回调函数，对接收到的数据`message` 入列到数据列表，调用FireCallbacks

#### 2.2.4 Observe() 和 GetLatestObserved()

```c++
  void Observe() override {
    std::lock_guard<std::mutex> lock(mutex_);
    observed_queue_ = data_queue_;
  }

  const D& GetLatestObserved() const {
    std::lock_guard<std::mutex> lock(mutex_);
    DCHECK(!observed_queue_.empty())
        << "The view of data queue is empty. No data is received yet or you "
           "forgot to call Observe()"
        << ":" << topic_name_;
    return *observed_queue_.front();
  }
```

* Observe函数是把当前的数据队列赋值给观察队列
* GetLatestObserved函数是获取观察队列队首的数据

这两个要搭配使用，先调用`Observe()`函数，再调用`GetLatestObserved()`函数



## 2.3 AdapterManager类

封装了Adapter类和ros的通讯接口

```c++
class AdapterManager {
  //..
  DECLARE_SINGLETON(AdapterManager);
}
```



几个重要的成员函数

#### 2.3.1 InternalEnable##name

```c++
  void InternalEnable##name(const std::string &topic_name,                     \
                            const AdapterConfig &config) {                     \
    name##_.reset(                                                             \
        new name##Adapter(#name, topic_name, config.message_history_limit())); \
    if (config.mode() != AdapterConfig::PUBLISH_ONLY && IsRos()) {             \
      name##subscriber_ =                                                      \
          node_handle_->subscribe(topic_name, config.message_history_limit(),  \
                                  &name##Adapter::RosCallback, name##_.get()); \
    }                                                                          \
    if (config.mode() != AdapterConfig::RECEIVE_ONLY && IsRos()) {             \
      name##publisher_ = node_handle_->advertise<std_msgs::String>(            \
          topic_name, config.message_history_limit(), config.latch());         \
    }                                                                          \
                                                                               \
    observers_.push_back([this]() { name##_->Observe(); });                    \
    name##config_ = config;                                                    \
  } 
```

根据配置文件，启动订阅节点或者发布节点，其中默认调用Adapter::Observe()函数





