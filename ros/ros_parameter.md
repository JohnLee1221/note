## 1 ros parameter

全局变量的概念



## 2 rosparam

```shell
# rosparam list           列出参数名
rosparam list

# rosparam set            设置参数
rosparam set 1000

# rosparam get            获取参数
rosparam get /turtlesim/background_g

# rosparam dump           向文件中转储参数
# rosparam dump [file_name] [namespace]
rosparam dump params.yaml

# rosparam load           从文件中加载参数
rosparam load params.yaml

rosparam delete         	# 删除参数
```



## 3 parameter c++ API

* **` param `**

  参数索引，`rosparam list`看不到

* **`setParam`**

  参数设置

* **`getParam`**

  获取参数

* **`hasParam`**

  判断参数是否存在

* **`searchParam`**

  搜索参数



```c++
int main(int argc, char **argv) {
  ros::init(argc, argv, "parameters");
  ros::NodeHandle nh;
    
  int i;
  std::string str;
  nh.param("my_num", i, 100);
  nh.param<std::string>("my_string", str, "hello ros");

  std::vector<int> vec = {1, 3, 4, 5};
  nh.setParam("my_vec", vec);
  nh.getParam("my_vec", vec);

  if (nh.hasParam("my_vec")) {
    ROS_INFO("my_vec parameter is OK");
  }
    
  std::string param_name;
  if (nh.searchParam("b", param_name)) {
    int i = 0;
    nh.getParam(param_name, i);
  } else {
    ROS_INFO("No param 'b' found in an upward search");
  }
}
```



## 4 parameter yaml

```c++
// TODO:update
```





















