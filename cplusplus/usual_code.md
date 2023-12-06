

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

![image-20230925101252526](D:\Work_Station\Documents\note\cplusplus\images\image-20230925101252526.png)

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

**matplotlibcpp库**

### 3.1 语法

`matplotlibcpp` 是一个 C++ 封装库，用于在 C++ 程序中绘制类似于 Python 中使用 Matplotlib 绘图的图形。以下是一些常用的 `matplotlibcpp` 库的语法：

1. **包含头文件**：

   ```c++
   #include <matplotlibcpp.h>
   namespace plt = matplotlibcpp;
   ```

2. **绘制曲线**：

   ```c++
   std::vector<double> x = {1, 2, 3, 4, 5};
   std::vector<double> y = {2, 4, 6, 8, 10};
   plt::plot(x, y, "r-"); // 绘制红色实线
   plt::plot(x, y, "g--"); // 绘制绿色虚线
   ```

3. **绘制点图：**

   ```c++
   std::vector<double> x = {1, 2, 3, 4, 5};
   std::vector<double> y1 = {2, 4, 6, 8, 10};
   std::vector<double> y2 = {3, 6, 7, 10, 15};
   plt::plot(x, "o");    		// 绘制x集合点
   plt::plot(x, y1, "ro"); 	// 绘制红色点
   plt::plot(x, y2, "go"); 	// 绘制绿色点
   ```

4. **绘制散点图**：

   ```c++
   std::vector<double> x = {1, 2, 3, 4, 5};
   std::vector<double> y = {2, 4, 6, 8, 10};
   plt::scatter(x, y); // 绘制散点图
   ```

5. **设置标题和标签**：

   ```c++
   plt::title("Scatter Plot");
   plt::xlabel("X Axis");
   plt::ylabel("Y Axis");
   ```

6. **显示图形**：

   ```c++
   plt::show();
   ```

7. **保存图形为文件**：

   ```c++
   plt::save("plot.png"); // 保存为 PNG 图片
   ```

8. **设置坐标范围**：

   ```c++
   plt::xlim(0, 6); // 设置 X 轴范围为 [0, 6]
   plt::ylim(0, 12); // 设置 Y 轴范围为 [0, 12]
   ```

9. **设置线宽和点的大小**：

   在 `matplotlibcpp` 的 `plot` 函数中，你提供的信息是正确的，各关键字参数的使用如下：

   - `color`：用于指定线条的颜色，例如 `"red"` 表示红色，`"blue"` 表示蓝色，等等。
   - `marker`：用于指定标记点的样式，如 `"o"` 表示圆圈，`"s"` 表示方块，等等。
   - `linestyle`：用于指定线条的样式，如 `"-"` 表示实线，`"--"` 表示虚线，等等。
   - `linewidth`：用于指定线条的宽度，例如 `2.0` 表示宽度为 2.0。
   - `markersize`：用于指定标记点的大小，例如 `10` 表示大小为 10。
   - `label`：用于设置曲线的标签，以便用于图例。
   - `alpha`：用于设置图表元素的透明度，例如 `0.5` 表示半透明。

   ```c++
   std::vector<double> x = {1, 2, 3, 4, 5};
   std::vector<double> y = {2, 4, 6, 8, 10};
   
   // 设置线宽
   std::map<std::string, std::string> line_style;
   line_style.insert(std::make_pair<std::string, std::string>("color", "green"));
   line_style.insert(std::make_pair<std::string, std::string>("linewidth", "5"));
   plt::plot(x, line_style);		// 绿色实线，线宽5
   
   std::map<std::string, std::string> point_style;
   point_style["color"] = "red";
   point_style["marker"] = "o";
   point_style["markersize"] = "5";
   plt::plot(y, point_style);		// 红色点线，点大小为5
   ```

   <img src="D:\Work_Station\Documents\note\cplusplus\images\image-20230925101309870.png" alt="image-20230925101309870" style="zoom: 33%;" />

10. **绘制动图**：

    这个画图会占用线程，如果想边执行程序边画图，最好开多线程，否则程序会卡在画图这，不会往后执行

    ```c++
    #include <cmath>
    #include <vector>
    #include "matplotlibcpp.h"
    namespace plt = matplotlibcpp;
    
    int main()
    {
      int n = 1000;
      std::vector<double> x;
      std::vector<double> sin_line;
      std::vector<double> log_line;
      for(int i = 0; i < n; i++) {
        x.push_back(i * i);
        sin_line.push_back(sin( 2 * M_PI * i / 360.0));
        log_line.push_back(log(i));
        if (i % 10 == 0) {
    	  plt::clf();										// 清空之前的plot数据
    	  plt::named_plot("sin(x)", x, sin_line);			// 绘制一个sin(x)函数
    	  plt::named_plot("log(x)", x, log_line);			// 绘制一个log(x)函数
                
    	  plt::xlim(0, 1000000);							// 设置x轴的范围
    	  plt::title("Sample figure");
    
    	  plt::legend();
    	  plt::pause(0.01);								// 图片显示时间，与legend()配合使用
    	}
      }
      plt::show();
      return 0;
    }
    ```

    <img src="D:\Work_Station\Documents\note\cplusplus\images\image-20230925101336668.png" alt="image-20230925101336668" style="zoom: 25%;" />

    <img src="D:\Work_Station\Documents\note\cplusplus\images\image-20230925101403334.png" alt="image-20230925101403334" style="zoom:25%;" />

11. **绘制多个图形**：

    ```c++
    plt::subplot(2, 1, 1); // 创建一个 2 行 1 列的子图，激活第一个子图
    plt::plot(x, y, "r-");
    plt::subplot(2, 1, 2); // 激活第二个子图
    plt::scatter(x, y);
    ```

12. **创建多个窗口**：

    ```c++
    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {2, 4, 6, 8, 10};
    plt::figure();		// 创建一个窗口
    plt::plot(x);
    plt::figure();		// 创建另一个窗口
    plt::plot(y, "o");
    ```

    

### 3.2 示例

* 把matplotlibcpp.h文件放置在目录下

* 代码

  ```c++
  #include "matplotlibcpp.h"
  
  namespace plt = matplotlibcpp;
  
  int main() {
    std::vector<double> v1 = { 1.2, 2.0, 3, 4, 5, 6, 3, 2, 3, 2, 0, 2, 1, 4 };
    std::vector<int> v2 = { 1, 2, 3, 4, 5, 6};
    std::vector<int> v3 = { 6, 5, 4, 3, 2, 1};
      
    plt::figure();
    
    // 创建第一个子图，位于 2x1 网格中的第一个位置
    plt::subplot(2, 1, 1);		// 添加子图
    plt::plot(v1);				// 默认画线
    plt::title("Plot 1");
      
    // 创建第二个子图，位于 2x1 网格中的第二个位置
    plt::subplot(2, 1, 2);		// 添加子图
    plt::plot(v2, v3, "o");		// "o"是画点
    plt::title("Plot 2");
    
    plt::save("test");			// 保存为test.png
    plt::show();
    plt::close();
  }
  ```
  
* 编译时要链接python库

  ```shell
  # g++编译
  g++ plot.cpp -o demo -I /usr/include/python3.8/ -lpython3.8
  ```

* 效果图示

  <img src="D:\Work_Station\Documents\note\cplusplus\images\image-20230925101416689.png" alt="image-20230925101416689" style="zoom:50%;" />





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

  

​	使用***type_traits***来设计一个编译阶段可以验证的，实现验证是否为派生类的重载函数模板

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



## 5 读取csv文件

读取一个csv文件，把里面的数据转换成double类型的数据

**test.csv**

```
x,y
0.0,0.0
0.5,0.0
1.0,0.0
1.5,0.0
2.0,0.0
2.5,0.0
3.0,0.0
3.5,0.0
4.0,0.0

```



**main.cpp**

```c++
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <cmath>

constexpr auto kFileDir = "/.../data/test.csv";

// 字符串解析，按照","分割
std::vector<std::string> StringParse(const std::string& message) {
  std::istringstream ss(message);
  std::vector<std::string> segment;
  std::string token;
  while (std::getline(ss, token, ',')) {
    segment.push_back(token);
  }
  return segment;
}

// 读取文档并
void ReadFile(const std::string& filename) {
  std::vector<double> messages;
  if (!messages.empty()) {
    messages.clear();
  }

  std::ifstream file_in(filename.c_str());
  if (!file_in.is_open()) {
    std::cout << "Can not open xxx file: " << filename << std::endl;
    std::exit(1);
  }

  std::string line;
  // 跳过头行
  getline(file_in, line);

  while (true) {
    getline(file_in, line);
    if (line == "") {
      break;
    }
    auto tokens = StringParse(line);
    // 验证数据的完整性
    if (tokens.size() < 2) {
      std::cout << "parse line failed; the data dimension does not match."
          		<< line << std::endl;
      continue;
    }

    std::cout << std::stod(tokens[0]) << "\t"
              << std::stod(tokens[1]) << std::endl;
  }

  file_in.close();
}

int main() {
  ReadFile(kFileDir);

  return 0;
}

```

