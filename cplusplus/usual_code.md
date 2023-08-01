

## 1 时间戳函数

```c++
#include <time.h>
#include <iomanip>

// 设置输出为年月日时分秒格式的时间戳
std::string GetTimeStamp() {
  time_t current_time = time(nullptr);
  char current_time_char[256];
  strftime(current_time_char, sizeof(current_time_char), "%Y%m%d%H%M%S",	//年月日时分秒
           localtime(&current_time));
  std::string current_time_str = current_time_char;
  return current_time_str;
}
```





## 2 生成log文件

设计一个***Data2Log*** 的仿函数，输出Data数据类型的数据

![](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproamytyproaimage-20230717110125990.png)

* **Data2Log()** && **Data2Log(const std::string& name)**

构造函数，定义一个log文件名字

*  **void Init()** 

初始化函数，打开文件并添加log文件抬头

* **std::string GetTimeStamp()** 

时间戳函数

* **void operator()(const Data& data)**

重载`()`的函数，在此函数中输出传入的Data类型的数据



**代码：**

```c++
#include <iostream>
#include <fstream>
#include <iomanip>

struct Data {
  int number;
  std::string str;
};

class Data2Log {
 public:
  Data2Log() : file_name_(GetTimeStamp() + ".log") {
    Init();
  }
  Data2Log(const std::string& name) : file_name_(name + ".log") {
    Init();
  }

  void operator()(const Data& data) {
    if (op_file_.is_open()) {
      // 设置输出数据
      op_file_ << std::setprecision(5) << data.number << "\t" << data.str << std::endl;
    } else {
      std::cout << "Open file is failure" << std::endl;
    }
  }

 private:
  void Init() {
    op_file_.open(file_name_.c_str());
    // 设置log文件抬头
    op_file_ << "number" << "\t" << "string" << std::endl;
  }
  std::string GetTimeStamp() {
    time_t current_time = time(nullptr);
    char current_time_char[256];
    strftime(current_time_char, sizeof(current_time_char), "%Y%m%d%H%M%S",	//年月日时分秒
            localtime(&current_time));
    std::string current_time_str = current_time_char;
    return current_time_str;
  }

 private:
  std::string file_name_;
  std::ofstream op_file_;
};
```

**调用**：

```c++
int main(int argc, char **argv) {
  Data test_data;
  Data2Log log_file("log_class");
  for (size_t i = 1; i < 100; ++i) {
    test_data.number = i;
    test_data.str = "test-" + std::to_string(i);
    log_file(test_data);
  }

  return 0;
}
```





## 3 画表格

* 把matplotlibcpp.h文件放置在目录下

* 代码

  ```c++
  #include "matplotlibcpp.h"
  using plt = matplotlibcpp;
  
  void Plot() {
    std::vector<int> vec = 
    		{ 1.2, 2.0, 3, 4, 5, 6, 3, 2, 3, 2, 0, 2, 1, 4 };
    plt::plot(vec);
    plt::show();   
  }
  ```
  
* 编译时要链接python库

  ```shell
  # g++编译
  g++ plot.cpp -o demo -I/usr/include/python3.8/ -lpython3.8
  ```

* 效果图示

  <img src="https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230531145735164.png" alt="image-20230531145735164" style="zoom:50%;" />





## 4 查询数据类型

* **typeid(T)**	**typeid(value).name()**

  ```c++
  #include <iostream>
  #include <typeinfo>
  
  template<typename T>
  void foo() {
      if (typeid(T) == typeid(int)) {
          std::cout <<  T 是 int 类型 << std::endl;
      }
  }
  
  int main() {
    int value = 42;
  
    std::cout << "The type of value is: " << typeid(value).name() << std::endl;
  
    return 0;
  }
  ```



* **std::iterator_traits** 

  ```c++
  #include <iostream>
  #include <vector>
  
  using MyVector = std::vector<int>;
  
  int main() {
    using Iterator = typename MyVector::iterator;
    using TypeValue = typename std::iterator_traits<Iterator>::value_type;
    if (std::is_same<TypeValue, int>::value) {
      std::cout << "type is same : int" << std::endl;
    } else {
      std::cout << "type is different" << std::endl;
    }
  
    return 0;
  }
  ```




* ***type_traits***

  `std::enable_if`

  `std::enable_if_t`

  `std::is_base_of`

  `std::is_same`

  

  ```c++
  #include <type_traits>
  
  class ClassA {};
  class ClassB {};
  class ClassC : public ClassA {};
  
  // IsDerived函数在编译阶段就可以匹配MessageType是否是ClassA的派生类
  template <typename MessageType>
  bool IsDerived(typename std::enable_if<std::is_base_of<ClassA, MessageType>::value, MessageType>::type *message = nullptr) {
  	return true;
  }
  
  // 无名参数列表
  template <typename MessageType, typename = std::enable_if_t<std::is_base_of<ClassA, MessageType>::value>>
  bool IsDerived() {
  	return true;
  }
  ```

  

​		使用***type_traits***来设计一个编译阶段可以验证的，实现验证是否为派生类的重载函数模板

```c++
#include <type_traits>
#include <iostream>

class Base {};
class ClassA {};
class ClassB : public Base {};

template <typename MessageType>
bool IsDerived(typename std::enable_if<std::is_base_of<Base, MessageType>::value, MessageType>::type *message = nullptr) {
  return true;
}

template <typename MessageType>
bool IsDerived(typename std::enable_if<!std::is_base_of<Base, MessageType>::value, MessageType>::type *message = nullptr) {
  return false;
}

int main() {
  if (IsDerived<ClassA>()) {
    std::cout << "This class is a derivative of the Base class" << std::endl;
  } else {
    std::cout << "This class is not a derivative of the Base class!!!" << std::endl;
  }

  return 0;
}
```

