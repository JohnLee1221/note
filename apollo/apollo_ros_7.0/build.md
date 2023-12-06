## install

### apt install

```shell
sudo apt install libgoogle-glog-dev libgflags-dev libgtest-dev coinor-libipopt-dev libomp-dev libcolpack-dev libadolc-dev libtar-dev
```



### absl

```shell
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j8
sudo make install
```



### qp-oases

```shell
git clone https://github.com/startcode/qp-oases.git
cd qp-oases
mkdir build && cd build
cmake ..
make -j8 CPPFLAGS="-Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE -D__NO_COPYRIGHT__"
sudo make install
```



### osqp

```shell
wget https://github.com/oxfordcontrol/osqp/archive/v0.5.0.tar.gz
tar xzf v0.5.0.tar.gz
cd osqp-0.5.0
wget https://github.com/oxfordcontrol/qdldl/archive/v0.1.4.tar.gz
tar xzf v0.1.4.tar.gz --strip-components=1 -C ./lin_sys/direct/qdldl/qdldl_sources
rm -rf v0.1.4.tar.gz
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" -DCMAKE_BUILD_TYPE=Release
make -j8 && make install 
cd ../.. && rm -rf v0.5.0.tar.gz
ldconfig
```



### protobuf

v3.6.1

```shell
sudo apt-get install autoconf automake libtool curl make g++ unzip
```

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
```



**手动删除protobuf**

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



### build error

* no runtime destination for executable target



![image-20231010133925200](D:\Work_Station\Documents\note\apollo\apollo_ros_7.0\images\image-20231010133925200.png)

​	apollo.ros-7.0.0/modules/apollo_planning/CMakeLists.txt

```cmake
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
```



![image-20231010134023056](D:\Work_Station\Documents\note\apollo\apollo_ros_7.0\images\image-20231010134023056.png)



* //usr/lib/x86_64-linux-gnu/libapr-1.so.0：对‘uuid_generate@UUID_1.0’未定义的引用

  ![image-20231011095948976](D:\Work_Station\Documents\note\apollo\apollo_ros_7.0\images\image-20231011095948976.png)

​	

```shell
ldd //usr/lib/x86_64-linux-gnu/libapr-1.so.0
```

​	![image-20231011100338345](D:\Work_Station\Documents\note\apollo\apollo_ros_7.0\images\image-20231011100338345.png)

​	libuuid.so.1这个库链接有问题，重新添加到CMakeLists.txt中的target_link_libraraies

![image-20231011101326098](D:\Work_Station\Documents\note\apollo\apollo_ros_7.0\images\image-20231011101326098.png)



### docker ssh

*  mobaxterm无法远程到docker中，rviz可视化

  问题很奇怪，对docker添加username就会出错，修改dockerfile

  在hosts端启动x server

  ```shell
  # xhost 是用来控制X server访问权限的。
  # 通常当你从hostA登陆到hostB上运行hostB上的应用程序时，做为应用程序来说，hostA是client,
  # 但是作为图形来说，是在hostA上显示的，需要使用hostA的Xserver,所以hostA是server.(执行xhost +后，得到提示“access control disabled, clients can connect from any host”)
  # 因此在登陆到hostB前，需要在hostA上运行xhost +来使其它用户能够访问hostA的Xserver.
  
  # xhost + 是使所有用户都能访问Xserver.
  # xhost + ip使ip上的用户能够访问Xserver.
  # xhost + nis:user@domain使domain上的nis用户user能够访问
  # xhost + inet:user@domain使domain上的inet用户能够访问。
  
  sudo vim /etc/profile
  xhost +
  ```
  
  

  

  **docker/build/apollo.ros-7.0.0.dockerfile**

  ```dockerfile
  RUN cp /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
  	&& echo 'Asia/Shanghai' > /etc/timezone \
  	&& locale-gen zh_CN.UTF-8 \
  	&& echo "export LC_ALL=zh_CN.UTF-8">> /etc/profile \
  	&& rm -rf /var/lib/apt/lists/*
  	
  # WORKDIR /
  RUN mkdir log \
  	apollo_ros_ws \
  	apollo_ros_ws/build \
  	apollo_ros_ws/devel \
  	apollo_ros_ws/install \
  	apollo_ros_ws/logs \
  	&& echo "source /opt/ros/noetic/setup.bash" >> .bashrc    
  ```
  
  
  
  **docker/scripts/docker_start.sh**
  
  ```dockerfile
  # 需要在apollo.ros-7.0.0所在目录下执行：./docker/scripts/docker_start.sh
  
  docker run -it -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority \
  	--net=host --name root \
  	-v $PWD:/apollo_ros_ws/src/apollo.ros-7.0.0	\
  	--rm apollo.ros:7.0.0 /bin/bash
  	
  ```




* 无法ssh到docker

  修改dockerfile

  ```cobol
  	&& echo "root:root" | chpasswd \
  	&& echo "Port 22" >> /etc/ssh/ssh_config \
  	&& echo "PasswordAuthentication yes" >> /etc/ssh/ssh_config \
  	&& echo "PermitRootLogin yes" >> /etc/ssh/sshd_config \
  ```

```shell
docker run -it -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority \
 	-p 2000:22 --name root \
	-v $PWD:/apollo_ros_ws/src/apollo.ros-7.0.0	\
	--rm apollo.ros:7.0.0 /bin/bash -c "service ssh start && /bin/bash"
```





*docker/scripts/docker_start.sh*

1. mobaxterm可视化，必须name是root，网络模式是host模式

2. host端可视化，而且可以通过ssh到docker内部，必须bridge模式(默认模式)

   **host端修改**

   https://blog.csdn.net/level_code/article/details/125534607?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169871777816800184151874%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169871777816800184151874&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-125534607-null-null.142^v96^pc_search_result_base9&utm_term=docker%20qxcbconnection%3A%20could%20not%20connect%20to%20display%3A0&spm=1018.2226.3001.4187

```shell
#!/bin/bash

# 需要在apollo.ros-7.0.0所在目录下执行：./docker/scripts/docker_start.sh

# # primal env
# docker run --privileged -ti --rm  --network host --env="DISPLAY" --name apollo_ros \
# 	-v /home/$USER/.Xauthority:/home/apollo_ros/.Xauthority:rw \
# 	-v $PWD:/home/apollo_ros/apollo_ros_ws/src/apollo.ros-7.0.0 \
# 	--rm apollo.ros:7.0.0 /bin/bash

# # mobaxterm env
# docker run -it -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority \
# 	--network host --name root \
# 	-v $PWD:/apollo_ros_ws/src/apollo.ros-7.0.0	\
# 	--rm apollo.ros:7.0.0 /bin/bash -c "service ssh start && /bin/bash"

# host env
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --env="QT_X11_NO_MITSHM=1"	\
	--name root \
	-v $PWD:/apollo_ros_ws/src/apollo.ros-7.0.0	\
	--rm apollo.ros:7.0.0 /bin/bash -c "service ssh start && /bin/bash"
```



