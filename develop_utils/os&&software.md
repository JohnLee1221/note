# 系统与软件

## windows设置

**取消右键折叠菜单**

```shell
# powershell
reg add "HKCU\Software\Classes\CLSID\{86ca1aa0-34aa-4e8b-a509-50c905bae2a2}\InprocServer32" /f /ve
```



**隐藏“快速访问”**

```shell
\HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows\CurrentVersion\Explorer
右侧新建>>DWORD(32位）值
重命名为HubMode
1     隐藏
0     显示
```



**访问共享盘**

windows -> \\192.168.1.44
ubuntu -> smb://192.168.1.44





## 办公软件

* **chrome**

* **drawio**

* **typora**

* **sumatraPDF**

* **office**

* **xmind**



## 开发软件

### vscode

字体设置

* win

  ![image-20230925095720917](D:\Work_Station\Documents\note\develop_utils\images\image-20230925095720917.png)

  ```txt
  Editor:Font Size
  14
  Editor:Font Family
  Consolas, 'Courier New', monospace
  Editor:Table Size
  2
  ```

* ubuntu

  ```shell
  sudo apt install fonts-firacode
  ```

  ![image-20230925095757275](D:\Work_Station\Documents\note\develop_utils\images\image-20230925095757275.png)

  ```txt
  Editor:Font Size
  14
  Editor:Font Family
  Fira Code, Consolas, 'Courier New', monospace
  Editor:Table Size
  2
  ```

  

```shell
ctrl+shift+p   命令面板
ctrl+r             打开最近的文件
ctrl+shift+l     全选相同名字
ctrl+shift+k    删除行
alt+uparrow   移动行
shift+alt+downarrow 复制一行到下面
shift+alt+F     格式规范化
F2    分析全选和替换
```

* **git**

* **clash**

  https://github.com/Fndroid/clash_for_windows_pkg/releases

```txt
clash
https://s.suying666.info/link/vFaGNkqrbTqZJx5J?sub=1

https://s.suying666.info/link/uPpmh23TAYLeuDYX?clash=1

Clash 进阶转换
https://api.wcc.best/sub?target=clash&url=https%3A%2F%2Fs.suying666.info%2Flink%2FvFaGNkqrbTqZJx5J%3Fclash%3D1&insert=false&config=https%3A%2F%2Fcdn.jsdelivr.net%2Fgh%2FSleepyHeeead%2Fsubconverter-config%40master%2Fremote-config%2Fspecial%2Fbasic.ini&emoji=true&list=false&tfo=false&scv=false&fdn=false&sort=false&new_name=true

https://api.wcc.best/sub?target=clash&url=https%3A%2F%2Fs.suying666.info%2Flink%2FuPpmh23TAYLeuDYX%3Fclash%3D1&insert=false&config=https%3A%2F%2Fcdn.jsdelivr.net%2Fgh%2FSleepyHeeead%2Fsubconverter-config%40master%2Fremote-config%2Fspecial%2Fbasic.ini&emoji=true&list=false&tfo=false&scv=false&fdn=false&sort=false&new_name=true
```

* **vitrual box**

* **mobaxterm**

```shell
切换全屏模式：F11
显示/隐藏侧边栏：Ctrl + Shift + B
窗口分离：Ctrl + Shift + D
向前导航：Ctrl + Tab/Ctrl + Alt + RightArrow
向后导航：Ctrl + Shift + Tab/Ctrl + Alt + LeftArrow
```

* **youdao**

* **everything**

* **Notepad**





## 安装

### carsim_2019_install

1.关闭杀毒软件和防火墙；

2.点击打开carsim2019/Setup_CarSim_2019.0_r76356.exe

3.安装位置为：D:\Program Files (x86)\CarSim2019.0_Prog

4....一直默认下一步...

5.finish时候不打开默认的pdf文档

6.复制安装包中的复制目录下所有文件，到安装目录

7.双击SolidSQUADLoaderEnabler.reg文件，安装

8.电脑重启

9.打开CarSim 2019.0，继续

10.选择Continue with the selected database，继续

11.选择Specity the License File，继续

12.Browse，打开D:\Program Files (x86)\CarSim2019.0_Prog\MSCLIC_SSQ.dat,激活完成



### Matlab_2020a_install

1. 鼠标右击软件压缩包，选择“解压到Matlab R2020a“。
   <img src="https://img-blog.csdnimg.cn/4d5b167d7edd47a7ac43c2f61922eae1.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
2. 打开解压后的文件夹，鼠标右击“MATLAB R2020a”选择“解压到MATLAB R2020a”。
   <img src="https://img-blog.csdnimg.cn/b6c62a794e2a4444b0293284fb0b386b.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
3. 打开解压后的文件夹，鼠标右击“setup.exe”选择“以管理员身份运行”。
   <img src="https://img-blog.csdnimg.cn/cf988015bb8e42dc824922de7b56bd84.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
4. 点击“是”，然后点击“下一步”。
   <img src="https://img-blog.csdnimg.cn/4e5bcaa9fc0148b78fa8b20237f4338c.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
5. 输入密钥：09806-07443-53955-64350-21751-41297，然后点击“下一步”。<img src="https://img-blog.csdnimg.cn/22fcd6864c2940828e8e45a78dac0839.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
6. 点击“浏览”。
   <img src="https://img-blog.csdnimg.cn/c813b46cf5cf4d2ab8255b1f7ae8d5af.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
7. 选择解压后的文件夹里的“Crack”文件夹下的“license_standalone.lic”，然后点击“打开”。
   <img src="https://img-blog.csdnimg.cn/eda444a24b344f7c99e9867a8fe0b900.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
8. 点击“下一步”。
   <img src="https://img-blog.csdnimg.cn/642a54f97d56420c858f3ed9223fbdda.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
9. 点击“浏览”选择软件的安装位置，然后点击“下一步”。
   <img src="https://img-blog.csdnimg.cn/9c8f97f0203b4c979425f753655cafc4.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
10. 点击“下一步”。
    <img src="https://img-blog.csdnimg.cn/526b31dc221f4e24bbc48d705880100b.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
11. 勾选“将快捷方式添加到桌面”，然后点击“下一步”。
    <img src="https://img-blog.csdnimg.cn/ba526a2107544b6d93ac9369b339a58e.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
12. 点击“开始安装”。
    <img src="https://img-blog.csdnimg.cn/427e82211e4947e6a742ca0c1c21a99c.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
13. 软件正在安装，请耐心等待，谢谢。
    <img src="https://img-blog.csdnimg.cn/b672fd04dec9498895b49dd76c944a0a.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
14. 点击“关闭”。
    <img src="https://img-blog.csdnimg.cn/09c30334241b468cb6d344ea24033b51.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
15. 打开解压后的文件夹目录下的“Crack”文件夹，鼠标右击“bin”选择“复制”。
    <img src="https://img-blog.csdnimg.cn/32094643c777453e86c5bbe642fc736c.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
16. 打开软件的安装位置，如图所示，将其粘贴进去。
    <img src="https://img-blog.csdnimg.cn/d4e7c50cb665437da4c2d65378346455.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
17. 点击“替换目标中的文件”。
    <img src="https://img-blog.csdnimg.cn/28c3e948ec674c5b937a51c6ebd68460.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
18. 打开该目录下的“bin”文件夹，鼠标右击“matlab”选择“发送到-桌面快捷方式”。
    <img src="https://img-blog.csdnimg.cn/39e4fc34be9d4a30aa4380194d49d675.png#pic_center" alt="在这里插入图片描述" style="zoom:50%;" />
19. 双击桌面图标。
20. 安装完成。

