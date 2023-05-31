# usual code



## 时间戳函数

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





## ofstream

生成一个data.txt文件

```c++
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

static int count = 0;

std::ofstream Data2File() {
  std::string file_name = "data.txt";
  double data;
  
  std::ofstream file_out;
  fout.open(file_name.c_str());
  
  file_out << "num" << std::endl;
  for (int i = 0; i < 50; ++i) {
    // std::setprecision	设置输出参数精度
    file_out << std::setprecision(15) << data << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  file_out.close();
  std::move(file_out);
}
```





## 画表格

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
