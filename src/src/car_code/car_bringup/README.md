# ROS2 机器人底盘驱动包 README 文档

## 包名称
**car_bringup**

## 功能描述
该包是机器人底盘驱动的核心包，提供不同型号机器人的底层控制接口，包括运动控制、IMU数据发布、RGB灯光控制、蜂鸣器控制等功能。支持多种机器人型号(X1、X3、R2)的驱动程序。

## 节点列表

### 1. Mcnamu_driver_X3 (X3型机器人驱动节点)
- **节点名称**: `driver_node`
- **可执行文件**: `Mcnamu_driver_X3`
- **功能**: X3型机器人底盘驱动控制

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/edition` | `std_msgs/msg/Float32` | 发布版本信息 |
| `/voltage` | `std_msgs/msg/Float32` | 发布电池电压信息 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布关节状态信息 |
| `/vel_raw` | `geometry_msgs/msg/Twist` | 发布原始速度信息 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 发布IMU原始数据 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 发布磁力计数据 |

#### 订阅接口
| 话题名称 | 消息类型 | 回调函数 | 描述 |
|:---------|:---------|:---------|:-----|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `cmd_vel_callback` | 订阅运动控制命令 |
| `/RGBLight` | `std_msgs/msg/Int32` | `RGBLightcallback` | 订阅RGB灯光控制 |
| `/Buzzer` | `std_msgs/msg/Bool` | `Buzzercallback` | 订阅蜂鸣器控制 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `car_type` | string | "X3" | 机器人型号 |
| `imu_link` | string | "imu_link" | IMU坐标系名称 |
| `Prefix` | string | "" | 话题前缀 |
| `xlinear_limit` | double | 1.0 | X轴线速度限制 |
| `ylinear_limit` | double | 1.0 | Y轴线速度限制 |
| `angular_limit` | double | 5.0 | 角速度限制 |

### 2. Mcnamu_driver_x1 (X1型机器人驱动节点)
- **节点名称**: `driver_node`
- **可执行文件**: `Mcnamu_driver_x1`
- **功能**: X1型机器人底盘驱动控制

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/edition` | `std_msgs/msg/Float32` | 发布版本信息 |
| `/voltage` | `std_msgs/msg/Float32` | 发布电池电压信息 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布关节状态信息 |
| `/vel_raw` | `geometry_msgs/msg/Twist` | 发布原始速度信息 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 发布IMU原始数据 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 发布磁力计数据 |

#### 订阅接口
| 话题名称 | 消息类型 | 回调函数 | 描述 |
|:---------|:---------|:---------|:-----|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `cmd_vel_callback` | 订阅运动控制命令 |
| `/RGBLight` | `std_msgs/msg/Int32` | `RGBLightcallback` | 订阅RGB灯光控制 |
| `/Buzzer` | `std_msgs/msg/Bool` | `Buzzercallback` | 订阅蜂鸣器控制 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `car_type` | string | "X1" | 机器人型号 |
| `imu_link` | string | "imu_link" | IMU坐标系名称 |
| `nav_use_rotvel` | bool | false | 导航是否使用旋转速度 |

### 3. Ackman_driver_R2 (R2型机器人驱动节点)
- **节点名称**: `driver_node`
- **可执行文件**: `Ackman_driver_R2`
- **功能**: R2型阿克曼转向机器人底盘驱动控制

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/edition` | `std_msgs/msg/Float32` | 发布版本信息 |
| `/voltage` | `std_msgs/msg/Float32` | 发布电池电压信息 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布关节状态信息 |
| `/vel_raw` | `geometry_msgs/msg/Twist` | 发布原始速度信息 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 发布IMU原始数据 |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 发布磁力计数据 |

#### 订阅接口
| 话题名称 | 消息类型 | 回调函数 | 描述 |
|:---------|:---------|:---------|:-----|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `cmd_vel_callback` | 订阅运动控制命令 |
| `/RGBLight` | `std_msgs/msg/Int32` | `RGBLightcallback` | 订阅RGB灯光控制 |
| `/Buzzer` | `std_msgs/msg/Bool` | `Buzzercallback` | 订阅蜂鸣器控制 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `car_type` | string | "R2" | 机器人型号 |
| `imu_link` | string | "imu_link" | IMU坐标系名称 |
| `nav_use_rotvel` | bool | false | 导航是否使用旋转速度 |

## 巡逻功能节点

### 4. patrol_4ROS (四轮机器人巡逻节点)
- **节点名称**: `patrol_node`
- **可执行文件**: `patrol_4ROS`
- **功能**: 基于激光雷达的自主巡逻功能

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 发布运动控制命令 |

#### 订阅接口
| 话题名称 | 消息类型 | 回调函数 | 描述 |
|:---------|:---------|:---------|:-----|
| `/scan` | `sensor_msgs/msg/LaserScan` | `LaserScanCallback` | 订阅激光雷达数据 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `odom_frame` | string | "odom" | 里程计坐标系名称 |
| `base_frame` | string | "base_footprint" | 机器人基座坐标系名称 |

### 5. patrol_a1_X3 (A1雷达X3机器人巡逻节点)
- **节点名称**: `patrol_node`
- **可执行文件**: `patrol_a1_X3`
- **功能**: 基于A1激光雷达的X3机器人巡逻

### 6. patrol_a1_R2 (A1雷达R2机器人巡逻节点)
- **节点名称**: `patrol_node`
- **可执行文件**: `patrol_a1_R2`
- **功能**: 基于A1激光雷达的R2机器人巡逻

### 7. patrol_4ROS_R2 (四轮雷达R2机器人巡逻节点)
- **节点名称**: `patrol_node`
- **可执行文件**: `patrol_4ROS_R2`
- **功能**: 基于四轮激光雷达的R2机器人巡逻

## 校准功能节点

### 8. calibrate_linear_X3 (X3线性校准节点)
- **可执行文件**: `calibrate_linear_X3`
- **功能**: X3机器人线性运动校准

### 9. calibrate_angular_X3 (X3角度校准节点)
- **可执行文件**: `calibrate_angular_X3`
- **功能**: X3机器人角度运动校准

### 10. calibrate_linear_R2 (R2线性校准节点)
- **可执行文件**: `calibrate_linear_R2`
- **功能**: R2机器人线性运动校准

### 11. calibrate_angular_R2 (R2角度校准节点)
- **可执行文件**: `calibrate_angular_R2`
- **功能**: R2机器人角度运动校准

## 启动文件

### car_bringup_R2_launch.py
启动R2机器人完整系统，包含：
- R2机器人驱动节点
- 机器人状态发布节点
- IMU滤波节点
- EKF定位节点
- 手柄控制节点

## 使用说明

### 1. 启动机器人驱动
```bash
# 启动X3机器人
ros2 run car_bringup Mcnamu_driver_X3

# 启动X1机器人
ros2 run car_bringup Mcnamu_driver_x1

# 启动R2机器人
ros2 run car_bringup Ackman_driver_R2
```

### 2. 启动完整系统
```bash
# 启动R2机器人完整系统
ros2 launch car_bringup car_bringup_R2_launch.py
```

### 3. 控制机器人运动
```bash
# 发布运动控制命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 4. 控制RGB灯光
```bash
# 控制RGB灯光效果 (0-6不同效果)
ros2 topic pub /RGBLight std_msgs/msg/Int32 "data: 1"
```

### 5. 控制蜂鸣器
```bash
# 打开蜂鸣器
ros2 topic pub /Buzzer std_msgs/msg/Bool "data: true"
```

## 依赖项
- ROS2 (机器人操作系统)
- Rosmaster_Lib (机器人底层控制库)
- tf2_ros (坐标变换库)
- sensor_msgs (传感器消息类型)
- geometry_msgs (几何消息类型)
- PyKDL (运动学和动力学库)

## 注意事项
1. 确保机器人硬件连接正确
2. 根据实际机器人型号选择对应的驱动节点
3. 校准功能需要在开阔平坦的环境中进行
4. 巡逻功能需要激光雷达正常工作
5. 注意电池电压监控，低电压时及时充电 