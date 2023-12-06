# Protobuf

## 1 library install && remove

### 1.1 apt install

```shell
sudo apt install libprotobuf-dev libprotoc-dev
```



### 1.2 manual install

指定版本：v3.6.1

从github克隆特定版本的protobuf源码

```shell
# 克隆版本为v3.6.1的protobuf源码
git clone -b v3.6.1 https://github.com/protocolbuffers/protobuf.git
cd protobuf
# 克隆protobuf的子模块，主要是gtest
git submodule update --init --recursive
```

编译protobuf源码并安装

```shell
./configure --prefix=/usr
make -j8
sudo make install
sudo ldconfig
```



 查看是否安装成功

```shell
protoc --version
```

![image-20231129170115623](D:\Work_Station\Documents\note\develop_utils\images\image-20231129170115623.png)



### 1.3 apt remove

```shell
sudo apt remove libprotobuf-dev libprotoc-dev
```

### 1.4 manual remove

```shell
# 先查看位置
which protoc
# protoc: /usr/bin/protoc

# 然后删除执行文件
sudo rm -rf /usr/bin/protoc

# 删除所有proto 的链接库和头文件
sudo rm -rf /usr/include/google/protobuf #头文件
sudo rm -rf /usr/local/include/google/protobuf #头文件
sudo rm -rf /usr/lib/libproto* #库文件
sudo rm -rf /usr/local/lib/libproto* # 库文件
```



## 2 .proto文件

### 2.1 数据类型c++对照表

<img src="D:\Work_Station\Documents\note\develop_utils\images\image-20231129170252326.png" alt="image-20231129170252326" style="zoom: 67%;" />

### 2.2 常见关键字

* **required**：proto3中删除

```tex
顾名思义，就是必须的意思，数据发送方和接收方都必须处理这个字段，
```

* **repeated**：proto3中没有变化

```tex
字面意思大概是重复的意思，其实protobuf处理这个字段的时候，也是optional字段一样，另外加了一个count计数变量，用于标明这个字段有多少个，这样发送方发送的时候，同时发送了count计数变量和这个字段的起始地址，接收方在接受到数据之后，按照count来解析对应的数据即可。
```
* **optional**：proto3中兼容

```tex
字面意思是可选的意思，具体protobuf里面怎么处理这个字段呢，就是protobuf处理的时候另外加了一个bool的变量，用来标记这个optional字段是否有值，发送方在发送的时候，如果这个字段有值，那么就给bool变量标记为true，否则就标记为false，接收方在收到这个字段的同时，也会收到发送方同时发送的bool变量，拿着bool变量就知道这个字段是否有值了，这就是option的意思。
```

* **message**

```tex
等同于c++中的class或者struct
```

* **package**

```tex
等同于c++中的namespace
```

* **map**

```c++
等同于c++中的map
```



### 2.3 .proto语法

如下是c++的数据类型：

```c++
namespace test_proto {
struct PhoneNum {
    std::string number;
};

struct SchoolInfo {
    int id;
    std::string school_name;
    std::vector<double> scores;
};
    
struct Person {
   	std::string name;
    int age;
    SchoolInfo school_info;
    std::vector<PhoneNum> phone_num;
};
    
enum Color
{
    Red,	// 可以不给初始值, 默认为0
    Green,
    Yellow,
    Blue
};
}	// namespace test_proto
```

对应生成.proto类型：

```protobuf
syntax = "proto3";			// 指定proto版本

package test_proto;

// message中等号后面的值为tag，没有实际意义，但不能重复
message PhoneNum {
  string number = 1;
}							// 不用“；”号

message SchoolInfo {
  int32 id = 1;
  string school_name = 2;
  repeated double scores = 3;
}

// 枚举类中等号后面的值为枚举元素的值
enum Color
{
    Red = 0;		// 第一个元素必须为0
    Green = 3;		// 第一个元素以外的元素值可以随意指定
    Yellow = 6;
    Blue = 9;
}

message Person {
  string name = 1;
  int32 age = 2;
  SchoolInfo school_info = 3;
  repeated PhoneNum phone_num = 4; 
  Color color = 5;
}
```



## 3 生成 .cc和 .h文件

### 3.1 g++

`.proto`文件编辑好之后就可以使用`protoc`工具将其转换为C++文件了。

```shell
protoc -I path/xxx.proto文件 --cpp_out=输出路径(存储生成的c++文件)
```

> 在 protoc 命令中，-I 参数后面可以跟随一个或多个路径，用于告诉编译器在哪些路径下查找导入的文件或依赖的文件，使用绝对路径或相对路径都是没问题的。



如果有多个路径，可以使用多个 -I 参数或在一个参数中使用冒号（:）分隔不同的路径。如果只有一个路径-I 参数可以省略。

protoc -I path1 -I path2 或 protoc -I path1:path2 都表示告诉编译器在 path1 和 path2 路径下查找导入的文件或依赖的文件。

```shell
protoc -I path1/xxx.proto -I path2/xxx.proto --cpp_out=.
# 或者
protoc -I path1/xxx.proto:path2/xxx.proto --cpp_out=.
```



### 3.2 cmake工程

在大型工程中，会有很多个`.proto`文件，通过cmake把所有`.proto`文件统一生成代码并编译成库

<img src="D:\Work_Station\Documents\note\develop_utils\images\image-20231129172744286.png" alt="image-20231129172744286" style="zoom:67%;" />

**CMakeLists.txt** 

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(apollo_msgs)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Debug")

# find protobuf library
find_package(Protobuf REQUIRED)

# 遍历并查找当前目录下的所有.proto文件并生成代码
set(PROTO_LIB_NAME apollo_msgs)
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()
set(PROTOBUF_PROTOC_EXECUTABLE protoc)
set(PROTO_HRDS)
set(PROTO_SRCS)
SUBDIRLIST(SUBDIRS ${PROJECT_SOURCE_DIR}/proto/${PROJECT_NAME}/proto)
foreach(subdir ${SUBDIRS})
  # include_directories(${PROJECT_SOURCE_DIR}/proto)
  file(GLOB PROTO_FILES "${PROJECT_SOURCE_DIR}/proto/${PROJECT_NAME}/proto/${subdir}/*.proto")
  message(STATUS "proto files: ${PROTO_FILES}")
  # set(proto_out_path ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/proto/${subdir})
  set(proto_out_path ${PROJECT_SOURCE_DIR}/include)
  file(REMOVE ${proto_out_path})
  file(MAKE_DIRECTORY ${proto_out_path})
  foreach(proto_file ${PROTO_FILES})
    get_filename_component(msg_path ${proto_file} ABSOLUTE)
    get_filename_component(msg_name ${proto_file} NAME_WE)
    # list(APPEND PROTO_HRDS ${proto_out_path}/${msg_name}.pb.h)
    # list(APPEND PROTO_SRCS ${proto_out_path}/${msg_name}.pb.cc)
    list(APPEND PROTO_HRDS ${proto_out_path}/${PROJECT_NAME}/proto/${subdir}/${msg_name}.pb.h)
    list(APPEND PROTO_SRCS ${proto_out_path}/${PROJECT_NAME}/proto/${subdir}/${msg_name}.pb.cc)
    execute_process(
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} 
      # -I=${PROJECT_SOURCE_DIR}/proto/${subdir}
      -I=${PROJECT_SOURCE_DIR}/proto
      --cpp_out=${proto_out_path} ${msg_path}
      # WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endforeach()
  message(STATUS "proto srcs: ${PROTO_SRCS}")
endforeach()

# 生成动态库
add_library(${PROTO_LIB_NAME} SHARED
	${PROTO_HRDS}
	${PROTO_SRCS}
)
target_link_libraries(${PROTO_LIB_NAME}
	protobuf
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION /home/lzh/genesys/lib/${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

# 安装库文件
install(TARGETS ${PROTO_LIB_NAME}
	ARCHIVE DESTINATION /home/.../lib
	LIBRARY DESTINATION /home/.../lib
)

```



## 4 API

### 4.1 **基础操作**

通过`protoc`命令对`.proto`文件的转换，得到的头文件中有一个类，这个类的名字和` .proto`文件中`message`关键字后边指定的名字相同，`.proto`文件中`message`消息体的成员就是生成的类的私有成员。

 可以调用生成的类提供的公共成员函数，访问生成的类的私有成员，这些函数有如下规律：

* **void clear_prop()** // 清除字段数据

* **const T& prop()** // 获取字段值

* **void set_prop(val)** // 字段赋值

* **T* mutable_prop()** // 返回字段指针

  得到类私有成员的地址, 通过这块地址读/写当前私有成员变量的值

  * **string类型**

    ```c++
    ::std::string* mutable_xxx();
    ```

  * **message类型**

    ```c++
    ::xxx_Message* mutable_xxx();
    ```

  * **`repeated + message类型` 或者 `repeated + string`**

    ```c++
    // message类型
    ::xxx_Message* mutable_xxx(int index);
    // string类型
    ::std::string* mutable_xxx(int index);
    ```

  * **其他基础类型没有mutable_xxx()接口**

* **T* add_prop()** // 向repeated field增加一个数据

  * 数组中元素的个数: **xxx_size()**
  * 添加一块内存, 存储新的元素数据: **add_xxx()** 、**add_xxx(参数)**

* **bool has_prop()** // 判断字段是否赋值，用于区分默认值和null

​	用于判断这个对象有没有被赋值，返回`ture`或者`false`



### 4.2 **序列化操作**

* 序列化

```c++
// 将类对象中的数据序列化为字符串, c++ 风格的字符串, 参数是一个传出参数
bool SerializeToString(std::string* output) const;

// 将类对象中的数据序列化为字符串, c 风格的字符串, 参数 data 是一个传出参数
bool SerializeToArray(void* data, int size) const;
```

* 反序列化

```c++
bool ParseFromString(const std::string& data);

bool ParseFromArray(const void* data, int size);
```



### 4.3 message操作

```c++
//使用另外一个message的值来覆盖本message
void CopyFrom(Message from);

//相同字段会被覆盖，repeated字段会追加
void MergeFrom(Message from);

// 将message中所有数据都清空，用于复用message
void Clear();

// 交换两个message数据
void Swap(Message * message1, Message * message2);

//交换两个message中指定的fields
void SwapFields(Message * message1, Message * message2, std::vector< const FieldDescriptor * > & fields);

//protobuf支持从特定格式的明文字符串中解析出message，可用于配置加载
bool TextFormat::Parser::ParseFromString(string, Message * output);
```



### 4.4 **debug操作**

```c++
//将message转化成格式化的字符串，用于debug
string DebugString();

//将message转化成精简的格式化的字符串，用于debug
string ShortDebugString();

//获取message占用的空间大小
size_t SpaceUsedLong(const Message & message);
```



### 4.5 JSON转换

```c++
Status MessageToJsonString(const Message & message, string * output);

Status JsonStringToMessage(StringPiece input, Message * message, JsonParseOptions & options);
```



## 5 使用protobuf库

### 5.1 find_package

```cmake
find_package(Protobuf REQUIRED)

include_directories()

target_link_libraries(xxx
protobuf
)
```





