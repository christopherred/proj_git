# Topic设计

# 1. 底盘及基础控制（chassis_driver文件夹）

![底盘及基础控制](assets/底盘及基础控制-20241209143958-fhg5q06.svg)​

- ​`joy_node`​：（ros标准pkg）遥控器驱动程序
- ​`joy_controller`​：对`joy`​信号进行处理，根据推杆解析出速度控制量，根据按键发送切换控制模式指令
- ​`command_switcher_node`​：根据`joy_controller`​的指令，对控制模式进行在**急停**，**遥控器控制**，**键盘控制**，**自动控制**四种模式之间进行切换
- ​`base_controller_node`​：根据`cmd_vel`​，对轮子电机的期望速度进行计算，并根据实际电机速度计算实际车身速度
- ​`base_driver_node`​：将电机的期望速度下发，并收集实际速度和实际扭矩

‍
