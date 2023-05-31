# Ubuntu系统

## 1 虚拟机配置

![image-20230531130355371](https://images-1318119468.cos.ap-shanghai.myqcloud.com/mytyproaimage-20230531130355371.png)





## 2 ubuntu 预装软件卸载

```shell
sudo apt-get autoremove libreoffice-common
sudo apt-get -y purge firefox
sudo apt-get -y purge thunderbird totem rhythmbox simple-scan gnome-mahjongg aisleriot gnome-mines cheese transmission-common gnome-sudoku
```





## 3 常见问题

* is not in the sudoers file

  ```shell
  su root
  vim /etc/sudoers
  *** ALL=(ALL) ALL
  ```



* 打不开终端

  ```shell
  cd /etc/default
  sudo vim locale # 把文件中的“en_US"改为 “en_US.UTF-8”
  
  sudo locale-gen --purge
  reboot
  ```

