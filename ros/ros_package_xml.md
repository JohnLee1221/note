## package.xml文件

```xml
<package format="2">
  <name>foo_core</name>															<!-- 名称  -->
  <version>1.2.4</version>														<!-- 版本 -->
  <description>
    This package provides foo capability.
  </description>																<!-- 描述 -->
  <maintainer email="ivana@willowgarage.com">Ivana Bildbotz</maintainer>		<!-- 维护者 -->
  <license>BSD</license>

  <url>http://ros.org/wiki/foo_core</url>
  <author>Ivana Bildbotz</author>

  <buildtool_depend>catkin</buildtool_depend>									<!-- 构建工具 -->

  <depend>roscpp</depend>
  <depend>std_msgs</depend>

  <!-- 依赖列表 -->
  <build_depend>message_generation</build_depend>

  <!-- 运行时依赖 -->
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>rospy</exec_depend>

  <!-- 测试依赖 -->
  <test_depend>python-mock</test_depend>

  <doc_depend>doxygen</doc_depend>
</package>
```





### < depend> 

等价于 <build_depend> + <build_exprot_depend> + <exec_depend>



### <build_depend> 

​	构建依赖项指定了构建这个包需要哪些包。当构建时需要这些包中的任何文件时，就会出现这种情况。这可以是在编译时包括这些包的头文件，针对这些包的库进行链接，或者在构建时需要任何其他资源（特别是当这些包在 CMake 中被 find_package()-ed 时）。在交叉编译的情况下，构建依赖是针对目标架构的。



### <build_export_depend>

​	构建输出依赖指定了哪些包需要针对这个包来构建库。当你把它们的头文件包含在这个包的公共头文件中时就是这种情况（特别是当这些包在 CMake 的 catkin_package() 中被声明为 (CATKIN_)DEPENDS 时）。



### <exec_depend>

​	执行依赖指定了哪些包需要运行此包中的代码。当你依赖这个包中的共享库时就是这种情况（特别是当这些包在CMake的catkin_package()中被声明为(CATKIN_)DEPENDS时）。



### <test_depend>

​	测试依赖仅指定单元测试的额外依赖。它们不应该与任何已经提到的构建或运行的依赖关系重复。



### <buildtool_depend>

​	构建工具依赖项指定该包需要的构建系统工具，以构建自身。一般来说，唯一需要的构建工具是 catkin。在交叉编译的情况下，编译工具的依赖性是针对编译所处的架构。



### <doc_depend>

​	文档工具依赖项指定了该软件包生成文档所需的文档工具。