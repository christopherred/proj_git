# 文件夹ros2_ws

文件夹ros2_ws为ROS2下的工作空间文件夹，位于"~\Robot\robot_code\ros2_ws"，里面主要存放与小车、机械臂等相关的代码，本文档对其中的内容进行简要解释，以方便大家了解和使用。

进入src文件夹，里面有6个子文件夹：

- chassis_controller
- chassis_driver
- JAKA
- navigation
- sensors_driver
- streamlit

## **1.chassis_controller**

底盘控制器，目前有两种控制器：

- constraint_following_controller
- mpc_controller

### <span data-type="text" style="font-size: 19px;">1.1 constraint_following_controller</span>

约束跟随伺服控制器，理论部分可在[约束跟随伺服控制器](约束跟随伺服控制器.md)中查看，该控制器是在MATLAB中的simulink下实现的。

订阅的话题有：/odom、/chassis/actual/cmd_vel

其中/odom提供了小车的位置信息，作为 $q$ 输入到控制器中。/odom和/chassis/actual/cmd_vel都提供了小车的速度信息，即线速度和角速度，理论上二者是相同的，但是由于滑动摩擦等原因，可能会导致不同，所以设置了一个手动切换的开关，速度信息作为 $\dot q$ 输入到控制器中。

发布的话题有：/chassis/command/cmd_vel

该话题为控制信号，提供了小车要求的速度信息，将其转换为三个轮子的角速度后，用于控制小车。

### <span data-type="text" style="font-size: 19px;">1.2 mpc_controller</span>

文件夹中包含mpc.py、mpc_controller.py、traj_publisher.py三个文件

mpc.py是一个基础的MPC框架，根据给定的车辆状态和目标路径，使用 MPC 优化车辆的运动控制输入（线速度和角速度），以最小化代价函数，同时满足车辆运动学约束和速度限制。对优化问题进行求解，得到最优控制输入系列和最优控制系列，可被mpc_controller.py调用进行求解。

mpc_controller.py是控制器的主体部分，订阅轨迹话题/designed_path与位置信息话题/odom，进行调用mpc_controller.py进行优化问题的求解，然后发布话题/chassis/command/cmd_vel实现对小车的控制。

traj_publisher.py是轨迹发布的程序，将 $x$ 坐标、$y$ 坐标与角度 $\theta$ 构成的轨迹进行发布，供MPC控制器使用。

## **2.chassis_driver**

底盘驱动，包含仿真驱动与实物驱动：

- omni_chassis_driver
- diff_chassis_driver
- joy_control

### <span data-type="text" style="font-size: 19px;">2.1 omni_chassis_driver</span>

全向轮底盘驱动，即三轮小车的底盘驱动，其中主要的几个文件夹如下：

- vrep_model
- omni_base_controller
- omni_base_driver
- omni_simulation_base_driver
- launch

#### <span data-type="text" style="font-size: 16px;">2.1.1</span> <span data-type="text" style="font-size: 16px;">vrep_model</span>

vrep_model中存放vrep的场景文件，包含小车、地图、激光雷达等。

#### <span data-type="text" style="font-size: 16px;">2.1.2</span> <span data-type="text" style="font-size: 16px;">omni_base_controller</span>

omni_base_controller中存放小车底盘控制信号的转换函数，主要功能是通过正向运动学和逆向运动学方程，实现小车整体的线速度、角速度与三个轮子角速度的转换。

在对小车的控制过程中，如果使用手柄对实物小车进行控制，那么只是在局部坐标系$X_RY_R$下进行；如果在 Vrep 中进行仿真，则是在全局坐标系$XY$下运行。所以对于运动学方程，要将其改写出局部坐标系$X_RY_R$与全局坐标系$XY$下的两个代码文件。

base_controller_global.py是全局坐标系下的，base_controller_local.py是局部坐标系下的。

正向运动学方程将三个轮子的角速度、小车的偏航角转换为小车在全局坐标系下的线速度、角速度，订阅的话题是/chassis/actual/wheelspeed，发布的话题是/chassis/actual/cmd_vel

逆向运动学方程将小车在全局坐标系下的线速度、角速度、偏航角转换为三个轮子的角速度，订阅的话题是/chassis/command/cmd_vel，发布的话题是/chassis/command/wheelspeed

更具体的内容可以在[Omni机器人数学建模](Omni机器人数学建模.md)中查看。

#### <span data-type="text" style="font-size: 16px;">2.1.3</span> <span data-type="text" style="font-size: 16px;">omni_base_driver</span>

实物小车的底盘驱动

#### <span data-type="text" style="font-size: 16px;">2.1.4</span> <span data-type="text" style="font-size: 16px;">omni_simulation_base_driver</span>

vrep仿真中的底盘驱动，由于API的不同以及传感器的添加，目前有三个版本，分别是共享内存、ZMQ、添加激光雷达的共享内存，其本质内容都是相同的，正在使用的是ZMQ版本。

订阅的话题有/chassis/command/wheelspeed，然后将该速度传入三个轮子，控制小车运动。

发布的话题有/chassis/actual/wheelspeed，读取三个轮子实际的速度，进行发布；/chassis/actual/torque，读取三个轮子实际的转矩，进行发布；/odom，读取小车的位置信息，进行发布。

#### <span data-type="text" style="font-size: 16px;">2.1.5</span> <span data-type="text" style="font-size: 16px;">launch</span>

launch文件的作用是同时启动多个 ROS 节点，简化节点的管理和启动过程。但是在实际运行的过程中，并不是所有的节点都需要启动，例如进行实物控制时，要运行base_controller_local.py；进行仿真实验时，要运行base_controller_global.py，这就要求在编写launch文件时添加一个开关进行切换，代码如下所示：

```python
def generate_launch_description():
    # 定义参数，用于切换控制器
    use_global_controller = LaunchConfiguration('use_global_controller')

    # 声明参数
    declare_use_global_controller = DeclareLaunchArgument(
        'use_global_controller',  # 参数名称
        default_value='true',     # 默认值，可以设置为 'false'
        description='Use global base controller if true, otherwise use local base controller'
    )

    # 全局控制器节点
    base_controller_global_node = Node(
        package='omni_chassis_driver',
        executable='base_controller_global',
        name='base_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'wheel_radius': 0.0375}, {'wheel_spacing': 0.175}],
        condition=IfCondition(use_global_controller),
    )

    # 局部控制器节点
    base_controller_local_node = Node(
        package='omni_chassis_driver',
        executable='base_controller_local',
        name='base_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'wheel_radius': 0.0375}, {'wheel_spacing': 0.175}],
        condition=UnlessCondition(use_global_controller),
    )

    return LaunchDescription([
        # 声明参数
        declare_use_global_controller,

        # 加载全局或局部控制器节点
        base_controller_global_node,
        base_controller_local_node,
    ])
```

启用全局控制器（默认）：

ros2 launch omni_chassis_driver chassis.launch.py

启用局部控制器：

ros2 launch omni_chassis_driver chassis.launch.py use_global_controller:=false

同理，对于 base_driver_node 和 simulation_base_driver_node也可以用相同的设置方法添加开关。

需要注意的是，在编写launch文件之前，要对setup.py文件进行修改，将包名等信息都配置好：

```python
'console_scripts': [
             "omni_base_controller_global_node  = omni_base_controller.base_controller_global:main",
             "omni_base_controller_local_node  = omni_base_controller.base_controller_local:main",
             "omni_base_driver_node      = omni_base_driver.base_driver:main",
             "omni_simulation_base_driver_node = omni_simulation_base_driver.simulation_base_driver_zmq:main",
        ],
```

### <span data-type="text" style="font-size: 19px;">2.2 diff_chassis_driver</span>

‍

### <span data-type="text" style="font-size: 19px;">2.3 joy_control</span>

‍
