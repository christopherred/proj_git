## 项目作者讲解

下面内容具体可见`/doc`文件夹技术文档

### 一、传感器

#### 1.1 GPS定位：RTK

配置主从流动站：

##### UM982部分

用对应专业软件`UPrecise`接译GPS信息，配置代码在`UPrecise`软件对应框中打，指令含义在UM982传感器手册中可见，包括主从站配置

##### P900部分

复用开关打到config模式，UM982用串口配置，P900可以用上位机配置，软件打开输出功率就是空中功率，当下小型室外0.1W够用，大场景就加功率，可以加到1W，但这样P900会很烫，或许一开始就会因为过热断掉通讯，尽量做好散热

配置完复用开关切到DTU模式，这样UM982就串上P900了，加功率后信号质量会很好，但也发热严重，这里要在板子上进行改进

##### 总结与提问

上方为从站配置，主站差不多流程
关于天线，有两个天线就可以算夹角了，计算航向角
主站在边上，从站在边上，上面需要接天线（主站1个，从站2个），P900天线无论主从站都要接，==小车从站如果只要单点定位，那么只接main就行，如果大空间做航向，那么main和s……都要接，两个间距要够，0.5m即以上==（天线这段没听清，也不懂）

#### 1.2 雷达：思岚

雷达两种用法，转串口版本和转网口版本，转串口尝试了多次不稳定，转网口版本尝试了，还比较稳定，目前用的是转网口版

雷达接线也是供电和数据分开的，供电线仅仅供电，不传输任何数据

网口接线比较复杂：供电线、网口、雷达默认IP（192.168.0.7）

电脑与雷达直接链接，不经过路由器，因此需要对电脑手动分配IP：

`网络与Internet`->`以太网`->`192.168.0.x，子网掩码255.255.255.0，网关192.168.0.x，首选DNS 192.168.0.x`

配不成功用图形化界面，配置完成打开powershell `ping 192.168.0.7`

要用思岚科技软件打开去连接雷达，选TCP服务器 192.168.0.7，端口20108

雷达状态OK后开启就可以看到点云信息

如果要用多个雷达，一个网络内IP不可重复，用`USR-TCP232-M4,E45-V2.3.0.78.exe`改IP，改完也可以用powershell ping一下看能不能连接

### 二、连接小机器人

#### 2.1 概述与挂载

IP（JiMoke）：192.168.1.27

password: robot1011

远程小机器人->内部文件夹：

data：用来存放公用数据和数据集

ROS2：ROS2的小机器人workspace

doc：机器人整体设计笔记

开机实际为一个映射过程，主要需要用sudo权限写一个文件`/etc/fstab`静态文件系统信息，内容主要写：

```bash
//192.168.1.235/robot/mnt/nas cifs usrname=robot password=ZDHxpf1011, uid……（权限相关的设置，默认为最大权限）
```

作用是把`//192.168.1.235/robot/mnt/nas`挂载到`/mnt/nas`下面，用cifs协议，然后定义用户名与密码

如果挂载失败，需要手动输入代码`sudo mount -a #将所有东西手动挂载`

copynas是一个将主要代码文件夹copy到实际用的bash脚本，可以`cat /home/copynas`看下内容，或者可以用`async`这个同步命令

#### 2.2 NoMachine

本地化小车上的小屏幕，看起来方便，企业版特性支持两机同时远程，更稳定连接，需要过一个月重装一次重复白嫖

#### 2.3 小车启动运行

而后进入`ROS2/order.txt`看下小车目前支持的命令集合

##### 2.3.1 `ros2 launch  sensors_driver          start.launch.py`

所有sensor启动，没有装docker，直接用的ROS2 env，如果要同事运行ROS1，或许需要docker了

怎么看控制台启动后输出的日志

（1）雷达 正常输出
SLLidarS/N ：5HAA……（要能够正确显示）
SLLidar health status: OK.
current scan mode: Standard, sample rate: 16kHz, max_distance:30.0m, scan_frequency: 10.0Hz

（2）IMU ahrs_driver_node-1正常输出
只要Received response: *#OK，少数几个failed也没问题，偶尔一两帧是允许的

##### 2.3.2 `ros2 launch  omni_chassis_driver     actual.launch.py   use_global_controller:=true`

启动底盘

##### 2.3.3 `ros2 launch  joy_control             start.launch.py`

启动手柄控制

##### 2.3.4 `ros2 launch  cartographer cartographer.launch.py`

建图

##### 2.3.5 `ros2 launch  cartographer_pose cartographer_pose.launch.py`

包含位姿建图

#### 2.4 传感器相关代码文件

用rviz2可以看雷达效果，关于传感器：

```markdown
fdilink_ahrs_ROS2: IMU
realsense: 相机
sensors_driver: 所有传感器的一个驱动
sllidar_ros2: 雷达
UM982_driver: GPS，可以自己去写，当前不完善，实际就是收到的一串数据怎么去解析，实际用法开源了，具体在：https://github.com/sunshinehappy/UM982Driver，用法是全的，照着改就行了
```

雷达配置有部分在`sensor_driver`里面，用launch文件就可以看到，nmea_driver_node话题是标准读GPS的，们没有用到，但我们用的RTK

还有航向角heading可以用，如果要用，要自己写自己完善，并且要用BIT 982定位系统说明文档种的PVTSLNB com21 发话题订阅

#### 2.5 底盘相关代码文件

电机承接自threading.Thread，是一个线程

驱动代码的`_current`和`_torques`数据是问电机商家客服问到的数据（16V以下），这部分特性是电机固有的

然后下方用插值给电流和力矩的表达式

再下方打开串口，涉及一个超时重传机制，下面读力矩、设置速度都用了相同的机制，这里很容易解释，上面这些涉及串口的都是阻塞的，占用CPU干这件事，干完CPU才会释放，因而设一个异步函数`asyn_set_speed`，在里面设置一个标志位，对应标志位下干对应的活，最后都用run挂载起来

但是实际用起来调用的是start函数，这是一个继承设置的原因，用了start会自动进入run函数，run才是我们写的线程主函数，而后就是用ros2进行指令包装，包装成电机指令了，这部分GPT就可以写的很好

关于base_controller，更多为电机动力学，正负符号设置，选择开关在`chassis/chassis_driver/omni_chassis_driver/launch/actual.launch.py`内部有`use_global_controller:=true`等选择开关

而后还启动了tf，之后rviz2界面就会有完整的维护了，map要建图后才会有

`joy_controller`为手柄控制，主要设定按键与行为绑定，有01量，也有连续量，关于joy_node有专门的工具集

#### 2.6 建图相关代码文件

cartographer.launch.py可以建图，这里可以更新激光雷达使用数量，而后重新映射内部的名字

tf：odom->map，具体位置要用cartographer_pose cartographer_pose.launch.py，可以获得odom pose以及速度等，得到的信息是最全的，不过这里实现是取tf数据用差分计算的速度，问题在于长时间会漂移，最理想的建立方法应该是如下的闭环过程：

```markdown
tf<-建图<-雷达 /sensors/lidar  ->
	    <-IMU /sensors/imu    ->EKF->/sensors/filterodom 
		<-RTK /sensors/odom   ->
EKF->/sensors/filterodom -> tf
tf->pose+twist(差分)->与RTK pose比对，修正信息->tf
```

也就是tf提取得到的pose信息可以和RTK传感器那里得到的进行比对形成一个闭环，数据在不停地基于上一帧进行一个滤波，并不是从前到后，而是要闭环。

EKF结果送到建图，建图结果用来修正EKF

目前是断开的，开环的，没做进去，即目前确实EKF传odom进建图形成闭环

另外还有一个可视化界面没有完成，rosmanager与startgui.sh，一个在线的web控制页面，数据采集有些问题
