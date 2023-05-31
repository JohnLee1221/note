# g++

```shell
g++ -g test.cpp -o demo					# debug模式
g++ test.cpp -o demo -ltest				# 链接库

# g++使用opencv库
g++ png.cpp -o demo `pkg-config --cflags --libs opencv4`

# g++ 使用python库
g++ test.cpp -o demo -I/usr/include/python3.8 -lpython3.8

# g++ 指定自定义库libtest.so,libtmp.so
g++ test.cpp -o demo -L/home/test/ -ltest -ltmp
```





# cmake

```shell
cmake -DCMAKE_BUILD_TYPE=Debug ..
```





#       CMakeLists.txt

## 常用命令

**CMAKE_MINIMUM_REQUIRED()**

指定cmake最小版本要求

```cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.4.1)
```



**ADD_COMPILE_OPTIONS()**

添加编译选项

```cmake
ADD_COMPILE_OPTIONS(-std=c++11 -Wall -Won-error=old-style-cast -Wno-error=useless-cast -Wold-style-cast -Wuseless-cast -Wno-error=useless-cast)
```



**PROJECT()**

设置项目名称

自动引入两个变量：***DEMO_SOURCE_DIR***   ***DEMO_BINARY_DIR***，等价cmake自动定义的两个变量：***PROJECT_SOURCE_DIR***    ***PROJECT_BINARY_DIR***

```cmake
PROJECT(DEMO)
```



**MESSAGE()**

输出消息

```cmake
MESSAGE(STATUS "THIS IS A DEMO PROGRAM")
```



**OPTION()**

给变量赋值

```cmake
OPTION(VAR_NAME "comment" VAR_VALUE)		#comment是变量注释
```



### SET()

设置变量

```cmake
# 常用
SET(SRC_LIST demo.cpp)
set()
```



### ADD_LIBRARY()

ADD_LIBRARY(common1 STATIC ${PROJECT_SOURCE_DIR}/src/common1.cpp)#指定生成静态库--libcommon1.a

ADD_LIBRARY(common2 SHARED ${PROJECT_SOURCE_DIR}/src/common2.cpp)#指定生成动态库--libcommon2.so

### SET_TARGET_PROPERTIES()

\#对库的名称重新定义

SET_TARGET_PROPERTIES(common1 PROPERTIES OUTPUT_NAME "common")

SET_TARGET_PROPERTIES(common2 PROPERTIES OUTPUT_NAME "common")

SET(LIBRARY_OUTPUT_PATH ${SRC_LIST}/lib)#设置库文件生成路径

### ADD_EXECUTABLE()

\#指定可执行文件的源文件

ADD_EXECUTABLE(demo ${SRC_LIST})

### INCLUDE_DIRECTOIES()

\#定义头文件的包含目录

INCLUDE_DIRECTOIES(${PROJECT_SOURCE_DIR}/include}

SET(EXECUTABLE_OUTPUT_PATH ${SRC_LIST}/bin)#设置执行文件生成路径

### FIND_LIBRARY()

\#指定库目录

FIND_LIBRARY(DEMO_LIB common HINTS ${PROJECT_SOURCE_DIR}/lib)#在指定目录下查找指定库，并把绝对路径放置到变量里

### TARGET_LINK_LIBRARIES()

TARGET_LINK_LIBRARIES(demo ${DEMO_LIB})#把目标文件与库文件进行链接

### AUX_SOURCE_DIRECTORY()

AUX_SOURCE_DIRECTORY(${PROJECT_SOOURCE_DIR}/ SRC_LIST}#搜索项目主目录下的所有源文件

ADD_EXECUTABLE(demo ${SRC_LIST})



### FIND_PACKAGE()

```cmake
find_package(<PackageName> [version] [EXACT] [QUIET] [MODULE]
             [REQUIRED] [[COMPONENTS] [components...]]
             [OPTIONAL_COMPONENTS components...]
             [NO_POLICY_SCOPE])
```

```cmake
# 查找CURL库
find_package(CURL)
add_executable(curltest curltest.cc)
if(CURL_FOUND)
    target_include_directories(clib PRIVATE ${CURL_INCLUDE_DIR})
    target_link_libraries(curltest ${CURL_LIBRARY})
else(CURL_FOUND)
    message(FATAL_ERROR ”CURL library not found”)
endif(CURL_FOUND)
```

对于系统预定义的 `Find<LibaryName>.cmake` 模块，使用方法一般如上例所示。

每一个模块都会定义以下几个变量 

```
<LibaryName>_FOUND
<LibaryName>_INCLUDE_DIR or <LibaryName>_INCLUDES
<LibaryName>_LIBRARY or <LibaryName>_LIBRARIES
```

你可以通过`<LibaryName>_FOUND` 来判断模块是否被找到，如果没有找到，按照工程的需要关闭 某些特性、给出提醒或者中止编译，上面的例子就是报出致命错误并终止构建。 如果`<LibaryName>_FOUND` 为真，则将`<LibaryName>_INCLUDE_DIR` 加入 INCLUDE_DIRECTORIES



```
install (
  TARGETS hello_world_service hello_world_client
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
)
```



```cmake
cmake_minimum_required(VERSION 3.5)
project(arm_core)

set(BUILD_TEST TRUE)
set(LIB_NAME libarmcore)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

include_directories(
  include/
)

add_library(libarmcore SHARED
   #pose
   src/pose/Pose.cpp
)

target_link_libraries(libarmcore
  jsoncpp
  yaml-cpp
  spdlog
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS libarmcore
  EXPORT libarmcore
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# add lib.config
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        libarmcoreConfig.cmake
        VERSION 1.0
        COMPATIBILITY AnyNewerVersion 
        )

install(EXPORT libarmcore
        FILE libarmcoreConfig.cmake
        DESTINATION lib/cmake/libarmcore
        )
```

------





## 预定义变量

**PROJECT_SOURCE_DIR**

工程的根目录

**PROJECT_BINARY_DIR**

运行 cmake 命令的目录，通常是 ${PROJECT_SOURCE_DIR}/build

**PROJECT_NAME**

返回通过 project 命令定义的项目名称

**CMAKE_CURRENT_SOURCE_DIR**

当前处理的 CMakeLists.txt 所在的路径

**CMAKE_CURRENT_BINARY_DIR**

target 编译目录

**CMAKE_CURRENT_LIST_DIR**

CMakeLists.txt 的完整路径

**CMAKE_CURRENT_LIST_LINE**

当前所在的行

**CMAKE_MODULE_PATH**

定义自己的 cmake 模块所在的路径，SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)，然后可以用INCLUDE命令来调用自己的模块

**EXECUTABLE_OUTPUT_PATH**

重新定义目标二进制可执行文件的存放位置

**LIBRARY_OUTPUT_PATH**

重新定义目标链接库文件的存放位置





## CMAKE_PREFIX_PATH

`CMAKE_PREFIX_PATH`是一个分号分隔的路径列表，用来指明软件/库安装路径前缀，以供`find_package()`，`find_program()`，`find_library()`，`find_file()`和`find_path()`命令搜索使用，这样就方便搜索可执行文件、头文件及库文件等。初始为空，由用户设定。

```shell
cmake .. -DCMAKE_PREFIX_PATH=/usr/local/lib
```

```cmake
list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib")
```



## CMAKE_MODULE_PATH

`CMAKE_MODULE_PATH`是以分号分隔的列表，供`include()`或 `find_package()`使用。初始为空，由用户设定。



## CMAKE_INSTALL_PREFIX

```bash
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
```

```cmake
SET(CMAKE_INSTALL_PREFIX /usr/local)
INSTALL(TARGETS demo DESTINATION bin)		#将demo安装在/usr/local/bin目录下
```

------













