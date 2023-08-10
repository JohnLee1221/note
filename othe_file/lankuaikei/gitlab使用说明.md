# 1 进入GitLab

## 1.1 登录

首先确保电脑连上公司的网络，在公司的局域网下，连上公司wifi即可

打开http://172.16.2.211:9990/，会显示如下界面，并登录

![image-20230803093311451](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803093311451.png)

## 1.2 界面介绍

* **默认界面**

  * Your projects

    当前持有的项目列表

  ![image-20230803093709204](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803093709204.png)

* **个人界面**

  按照如图所示，依次点击`step 1`  `step 2` 可以看到`step3`为当前用户的预览

  ![image-20230803093958173](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803093958173.png)

  * Overview

    用户状态总览

  * Activity

    活动消息

  * Groups

    当前所在团队

    

* **团队界面**

  依次点击`step 4` `step 5` 进入团队界面

  ![image-20230803094557941](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803094557941.png)

  点击`step 6`  `step 7` 可以查看当前团队成员信息

  ![image-20230803095057884](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803095057884.png)



## 1.3 进入项目仓库

* 通过默认界面，直接点击进入项目
* 点击`step 8` `step 9`通过团队持有的项目列表进入项目
* ![image-20230803095521541](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803095521541.png)



# 2 仓库介绍

* **AGOS仓库**

  当前开发仓库名为**AGOS**，为**Agricultural Operating System**的缩写

  ![image-20230803100714169](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproamytyproaimage-20230803100714169.png)



* **分支介绍**

  * main

    主分支，不做合并

  * integration

    集成分支，主分支的子分支

    当所有开发完成后，合并到当前分支，

  * develop

    开发分支，集成分支的子分支

    每个开发者开发一个feature时，从develop分支切出一个子分支进行开发，开发完成后合并到develop分支上

    develop只能合并到integration分支

    

    

  ![image-20230803101001227](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803101001227.png)



# 3 拉取仓库

通过配置ssh-key，可以把仓库和本地电脑通过ssh密钥进行绑定，后续拉取和推送代码可以直接推送，不用输入用户名和密码

## 3.1 配置ssh-key

* 打开git bash

* 输入**ssh-keygen.exe**

  ```shell
  ssh-keygen.exe
  ```

  过程中直接enter，可以生成一个ssh key

  ![image-20230803102350042](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803102350042.png)

* 进入目录

  ```shell
  cd ~/.ssh
  cat id_rsa.pub
  ```

  ![image-20230803102648897](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803102648897.png)

* gitlab添加ssk key

  在step5中粘贴刚才git bash中显示的key

  ![image-20230803103704594](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803103704594.png)



* 添加完成

  刷新当前界面

  ![image-20230803103231870](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803103231870.png)



## 3.2 拉取仓库代码

* 进入仓库界面，通过`step 7`  点击 `Clone with SSH`选项的按钮

![image-20230803103832458](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803103832458.png)



* 在你所处的目录下，打开git bash，拉取代码

  ![image-20230803104311256](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230803104311256.png)

  