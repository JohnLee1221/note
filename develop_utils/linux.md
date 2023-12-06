# Linux 日常记录

## find

```shell
find . -name "xxx*"
# f 普通文件	l 符号链接		d 目录	c 字符设备		s 套接字
find . -type f -iname "xxx*"
   
find . -type d -iname "xxx*"
```



## du

```shell
du -lh                           //查看文件夹大小 
df -lh                           //查看系统使用大小
```



## tar

```shell
tar -zcvf test.tar.gz test      			//压缩 支持语法
tar -zxvf /usr/local/test.tar.gz          	//解压 再续语法
```



## dpkg

```shell
sudo ldconfig -p | grep ***    			//查找库文件
dpkg -L ***                             //查找头文件
```



## xdg-open

```shell
xdg-open meinv.jpg
```



## chmod

```shell
sudo chmod -R 777 ./test                              	//文件夹权限 r4 w2 x1
sudo chown -R nvidia:nvidia ./test                		//文件夹属
```



## fsck

fsck命令的全称是file system check，用于检查与修复 Linux 档案系统

```shell
fsck -y /dev/sda5
```



## 增加用户

![img_v2_7d537140-ba38-498c-97dc-295ceb885efg](D:\Work_Station\Documents\note\develop_utils\images\img_v2_7d537140-ba38-498c-97dc-295ceb885efg.jpg)



## 进程查看

```shell
ps aux | grep ***      //查看***进程
ps -ef | grep "xxx"

top -Hp pid             //查看线程
```



## 生成core文件

```shell
ulimit -a
ulimit -c unlimited
sudo sysctl -w kernel.core_pattern=core.%p
gdb ./test core...
```



## 查找动态库的关键字

```shell
strings xxx.so | grep ***
```



## ssh linux ~ windows

 ```shell
 # ssh 拷贝windows主机的test.txt文件到linux主目录
 scp -r 28376@192.168.1.180:D:\\Work_Station\\900_temp_files\\Autonomous-Driving\\temp\\test.txt ~
 
 # ssh 拷贝linux主机的test文件夹到windows的temp目录
 scp -r test/ 28376@192.168.1.180:D:\\Work_Station\\900_temp_files\\Autonomous-Driving\\temp
 ```



## wifi设置

```shell
sudo nmcli device wifi rescan										#扫描wifi
nmcli c
nmcli device wifi list												#显示wifi
sudo nmcli device wifi connect john_wifi password 123456abc			#连接wifi
sudo nmcli c del 6140b040-c7bf-4af5-b252-2431fc9718a4				#删除wifi
```



## 环境变量

```shell
env						# 所有环境变量
env | grep ***			# 查找指定环境变量

export LD_LIBRARY_PATH=/.../.../:$LD_LIBRARY_PATH		#设置链接库路径的环境变量
export LIBRARY_PATH=/.../.../:$LIBRARY_PATH			#设置链接静态库的路径

export PYTHONPATH=/.../.../:$PYTHONPATH

export PATH=PATH:HOME/bin										#在PATH中找到可执行文件程序的路径。

export C_INCLUDE_PATH=/.../.../:$C_INCLUDE_PATH					#gcc找到头文件的路径
export CPLUS_INCLUDE_PATH=/.../.../:$CPLUS_INCLUDE_PATH			#g++找到头文件的路径
```

