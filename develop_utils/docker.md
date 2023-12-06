## docker安装

https://blog.csdn.net/m0_60827485/article/details/125197867?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168895864116800188595961%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=168895864116800188595961&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-125197867-null-null.142^v88^insert_down38v5,239^v2^insert_chatgpt&utm_term=ubuntu%2020docker%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4187



## 给docker增加root权限

```shell
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

If you initially ran Docker CLI commands using `sudo` before adding your user to the `docker` group, you may see the following error:

```shell
WARNING: Error loading config file: /home/user/.docker/config.json -
stat /home/user/.docker/config.json: permission denied
```

This error indicates that the permission settings for the `~/.docker/` directory are incorrect, due to having used the `sudo` command earlier.

To fix this problem, either remove the `~/.docker/` directory (it's recreated automatically, but any custom settings are lost), or change its ownership and permissions using the following commands:

```shell
sudo chown "$USER":"$USER" /home/"$USER"/.docker -R
sudo chmod g+rwx "$HOME/.docker" -R
```





## 彻底卸载docker

https://m.php.cn/faq/520283.html



## ssh到docker

```shell
stu@XXX:~$ docker run -it --gpus all --name ymz --shm-size="2g" -p 8088:99 -v /data/ymz:/data/ymz pytorch/pytorch:1.9.1-cuda11.1-cudnn8-devel /bin/bash
```



## image常用操作

```shell
# 查看本地image列表
docker images  
docker image ls
# 获取远端镜像
docker pull
# 删除镜像[注意此镜像如果正在使用，或者有关联的镜像，则需要先处理完] 
docker image rm imageid
docker rmi -f imageid 强制删除镜像
docker rmi -f $(docker image ls)     删除所有镜像
# 运行镜像
docker run image
# 发布镜像
docker push
```



## container

```shell
# 根据镜像创建容器
docker run -d --name -p 9090:8080 my-tomcat tomcat
# 查看运行中的container
docker ps
# 查看所有的container[包含退出的]
docker ps -a
# 删除container
docker rm containerid
docker rm -f $(docker ps -a) 	 删除所有container
# 进入到一个container中
docker exec -it container bash
# 根据container生成image
docker
# 查看某个container的日志docker logs container
# 查看容器资源使用情况
docker stats
# 查看容器详情信息
docker inspect container
# 停止/启动容器
docker stop/start container
```

