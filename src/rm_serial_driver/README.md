# rm_serial_driver
RoboMaster 视觉系统与电控系统的串口通讯模块


## 使用指南

安装依赖 `sudo apt install ros-humble-serial-driver`

更改 [serial_driver.yaml](config/serial_driver.yaml) 中的参数以匹配与电控通讯的串口

启动串口模块 `ros2 launch rm_serial_driver serial_driver.launch.py`

## 发送和接收

详情请参考 [packet.hpp](include/rm_serial_driver/packet.hpp)

从电控端需要接受

- 机器人的自身颜色 `robot_color` 以判断对应的识别目标颜色
- 云台姿态 `pitch` 和 `yaw`, 单位和方向请参考 https://www.ros.org/reps/rep-0103.html
- 当前云台瞄准的位置 `aim_x, aim_y, aim_z`，用于发布可视化 Marker

视觉端发送 
- 开火指令`is_fire`
- 目标装甲板的位置`x`、`y`、`z`
- 目标的yaw轴角速度`v_yaw`
- 云台的yaw轴与pitch轴期望转动角度`yaw`、`pitch`

## 电控端的处理

TBD
