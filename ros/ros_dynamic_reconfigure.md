## dynamic_reconfigure

`dynamic_reconfigure`的重点是提供一种标准方法来将节点参数的子集公开给外部重新配置。客户端程序，例如 GUI，可以向节点查询一组可重新配置的参数，包括它们的名称、类型和范围，并向用户呈现定制的界面。



`dynamic_reconfigure::Server` 是 ROS 中的一个类，用于实现动态重新配置参数的机制。它允许在运行时通过重新配置服务（reconfigure service）来修改节点的参数，而无需停止和重新启动节点。

`dynamic_reconfigure::Server` 的通信机制如下：

1. 创建 `dynamic_reconfigure::Server` 对象时，需要指定一个配置类型。配置类型是一个结构体，用于定义参数的名称、类型和默认值等信息。
2. 当 `dynamic_reconfigure::Server` 对象被创建时，它会自动创建一个名为 `~set_parameters` 的服务，用于接收重新配置请求。
3. 当调用 `~set_parameters` 服务时，`dynamic_reconfigure::Server` 会将请求中的参数值更新到配置对象中。
4. 配置对象的值发生变化时，会触发回调函数。可以通过在创建 `dynamic_reconfigure::Server` 时指定回调函数来定义处理参数更新的逻辑。
5. 回调函数可以读取更新后的参数值，并在节点中应用这些新的参数



### 1 api

* **`dynamic_reconfigure::Server<T>`**

  `Server`类的构造

* **`dynamic_reconfigure::Server<T>::CallbackType`**

  可调用对象的构造

* **`setCallback`**

  回调



### 2 cfg文件生成代码

ros通过配置xml和CMakeLists.txt中的`dynamic_reconfigure`，可以基于.cfg文件生成对应的头文件

**Test.cfg**

```python
#!/usr/bin/env python
PACKAGE = "dynamic_param"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("int_param",    int_t,    0, "An Integer parameter", 50,  0, 100)
gen.add("double_param", double_t, 0, "A double parameter",    .5, 0,   1)
gen.add("str_param",    str_t,    0, "A string parameter",  "Hello World")
gen.add("bool_param",   bool_t,   0, "A Boolean parameter",  True)

size_enum = gen.enum([ gen.const("Small",      int_t, 0, "A small constant"),
                       gen.const("Medium",     int_t, 1, "A medium constant"),
                       gen.const("Large",      int_t, 2, "A large constant"),
                       gen.const("ExtraLarge", int_t, 3, "An extra large constant")],
                     "An enum to set size")

gen.add("size", int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)

exit(gen.generate(PACKAGE, "dynamic_param", "Test"))
```



**exit(gen.generate(PACKAGE, "dynamic_param", "Test"))**

>`PACKAGE` : 对应文件中第一行 `PACKAGE = "dynamic_param"`
>
>`dynamic_param` : 和`PACKAGE`名称对应一致，在生成代码中对应的是namespace
>
>`Test`: 会生成一个TestConfig.h文件，生成代码中对应的是`TestConfig`类



生成**TestConfig.h**

```c++
// 简化
namespace dynamic_param {
struct TestConfig {
  int int_param;
  double double_param;
  std::string str_param;
  bool bool_param;
  enum size {
    Small,
    Medium,
    Large,
    ExtraLarge
  };
};
}  // dynamic_param
```



### 3 配置dynamic_reconfigure

* package.xml

  ```xml
  <depend>dynamic_reconfigure</depend>
  ```

* CMakeLists.txt

  ```cmake
  find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
  )
  
  generate_dynamic_reconfigure_options(
    cfg/Test.cfg
  )
  
  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
  )
  ```



### 4 实现

1. 新建Test.cfg

   ```shell
   cd ~/test_ws/src/parameters/
   mkdir cfg
   cd cfg && touch Test.cfg
   # Test.cfg内容可参考上述Test.cfg
   ```

2. 配置参考5.3

3. dynamic_param.cpp

   ```shell
   cd ~/test_ws/src/parameters/src
   touch dynamic_param.cpp
   ```

   代码

   ```c++
   #include <ros/ros.h>
   #include <dynamic_reconfigure/server.h>
   
   #include "parameters/TestConfig.h"
   
   void Callback(dynamic_param::TestConfig& config, uint32_t level) {
     ROS_INFO("Reconfigure Request: %d %f %s %s %d", config.int_param,
              config.double_param, config.str_param.c_str(), 
              config.bool_param ? "True" : "False", 
              config.size);
   }
   
   int main(int argc, char** argv) {
     ros::init(argc, argv, "dynamic_param");
     dynamic_reconfigure::Server<dynamic_param::TestConfig> server;
     dynamic_reconfigure::Server<dynamic_param::TestConfig>::CallbackType f;
     f = boost::bind(&Callback, _1, _2);
     server.setCallback(f);
   
     ros::spin();
     return 0;
   }
   ```

4. 运行

   ```shell
   roscore
   ```

   ```shell
   rosrun parameters dynamic_prarm_node
   ```

   ```shell
   rosrun rqt_reconfigure rqt_reconfigure
   ```

5. 图示

   <img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproamytyproaimage-20230531165821671.png" alt="image-20230531165821671" style="zoom: 50%;" />