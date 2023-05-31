required from ‘bool std::any_of(_IIter, _IIter, _Predicate) [with _IIter = __gnu_cxx::__normal_iterator<const rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry*, std::vector<rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry> >; _Predicate = rclcpp::wait_set_policies::DynamicStorage::storage_has_entity(const EntityT&, const SequenceOfEntitiesT&) [with EntityT = rclcpp::Waitable; SequenceOfEntitiesT = std::vector<rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry>]::<lambda(const int&)>]’
/opt/ros/foxy/include/rclcpp/wait_set_policies/dynamic_storage.hpp:223:23:   required from ‘static bool rclcpp::wait_set_policies::DynamicStorage::storage_has_entity(const EntityT&, const SequenceOfEntitiesT&) [with EntityT = rclcpp::Waitable; SequenceOfEntitiesT = std::vector<rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry>]’
/opt/ros/foxy/include/rclcpp/wait_set_policies/dynamic_storage.hpp:351:55:   required from here
/usr/include/c++/9/bits/predefined_ops.h:283:11: error: no match for call to ‘(rclcpp::wait_set_policies::DynamicStorage::storage_has_entity(const EntityT&, const SequenceOfEntitiesT&) [with EntityT = rclcpp::Waitable; SequenceOfEntitiesT = std::vector<rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry>]::<lambda(const int&)>) (const rclcpp::wait_set_policies::DynamicStorage::WeakWaitableEntry&)’
  283 |  { return bool(_M_pred(*__it)); }



\# Default to C++14

if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 14)

endif()



如何定义功能的接口？

你们测试用例是用什么实现的？

了解控制理论吗？常用的控制算法能描述一下吗？

你会基于CANoe做一些开发和配置吗？

会使用capl语言编程，包括编写一些测试用例吗？



制定测试计划，和case



有没有在mcu端做过can配置





 <node pkg="apollo_planning" type="raw_path_node" name="row_path" args = "

  --flagfile=$(find apollo_planning)/conf/planning.conf

  --adapter_config_path=$(find apollo_planning)/conf/adapter.conf

  --rtk_trajectory_filename=$(find apollo_planning)/data/garage.csv"

 output="screen"/>



梁艺潇

1.算法模型的开发及Demo实车验证，具体是实现了哪些功能或者说是要完成什么场景下的算法开发？

2.介绍一下你们MBD模型开发的流程。

3.你们控制算法是基于matlab做开发的吗？

4.你对CAN总线的数据帧格式熟悉吗？可以分为标准帧和扩展帧，两者各有什么特点？

5.如果给你一份dbc文件和CAN设备，如何解析车辆底盘信号？

6.会基于CANoe做相应的底盘系统测试和仿真吗？

7.如果一个系统中，传感器的误差抵消不掉，如何从控制的角度减少这种误差的累加？



**1.描述一下拷贝构造和移动构造，和使用场景？**

**2.c++中虚函数和纯虚函数有什么区别**

**3.如何通过指针的形式调用一个成员函数，类似于回调，有哪些方法？**

**4.了解设计模式吗？列举一两个。**

**5.简单介绍一下lambda表达式**





研究

arxml---模型---代码



鲁棒性验证

乘用车 横纵向 优化

线控底盘 智能车控 IGBP工作	

车辆状态改进





讲一讲Apollo的规划控制模块的框架

这个项目目前已经实现实车功能

设计很多框架，对算法的实现也是你做的吗

os平台 数据采集 系统诊断 ota平台



具体是高速场景？

目前开发，实现到哪一个阶段？

决策，比如说换到策略，如何触发？

voerlap 是根据定位去处理洗一个路口？

speed_ref 二次规划器？

具体开发部署有做过吗？

功能验证，你们是如何验证的？



李晓亚

你的avp和apa已经到实车验证了，还是仿真验证了？

说一下A star和混合a星的区别

高精地图是做的语义定义吗？



