# ROS2 颜色追踪与识别包 README 文档

## 包名称
**car_astra**

## 功能描述
该包主要用于基于深度摄像头的颜色目标追踪和识别功能。包含颜色识别、HSV颜色空间处理、目标追踪等功能，适用于机器人视觉导航和目标跟随应用。

## 节点列表

### 1. colorHSV (颜色识别节点)
- **节点名称**: `coloridentify`
- **可执行文件**: `colorHSV`
- **功能**: 实时颜色识别和HSV参数调整

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/Current_point` | `yahboomcar_msgs/msg/Position` | 发布检测到的颜色目标位置信息 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 发布运动控制命令 |

#### 订阅接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| 摄像头数据 | `sensor_msgs/msg/Image` | 通过OpenCV VideoCapture获取 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `Hmin` | int | 0 | HSV色彩空间H分量最小值 |
| `Smin` | int | 85 | HSV色彩空间S分量最小值 |
| `Vmin` | int | 126 | HSV色彩空间V分量最小值 |
| `Hmax` | int | 9 | HSV色彩空间H分量最大值 |
| `Smax` | int | 253 | HSV色彩空间S分量最大值 |
| `Vmax` | int | 253 | HSV色彩空间V分量最大值 |
| `refresh` | bool | false | 是否刷新HSV参数 |

### 2. colorTracker (颜色追踪节点)
- **节点名称**: `color_tracker`
- **可执行文件**: `colorTracker`
- **功能**: 基于深度信息的颜色目标追踪和跟随

#### 发布接口
| 话题名称 | 消息类型 | 描述 |
|:---------|:---------|:-----|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 发布机器人运动控制命令 |

#### 订阅接口
| 话题名称 | 消息类型 | 回调函数 | 描述 |
|:---------|:---------|:---------|:-----|
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `depth_img_Callback` | 订阅深度图像数据 |
| `/JoyState` | `std_msgs/msg/Bool` | `JoyStateCallback` | 订阅手柄状态 |
| `/Current_point` | `yahboomcar_msgs/msg/Position` | `positionCallback` | 订阅目标位置信息 |

#### 参数配置
| 参数名称 | 类型 | 默认值 | 描述 |
|:---------|:-----|:-------|:-----|
| `linear_Kp` | double | 3.0 | 线性PID控制器比例系数 |
| `linear_Ki` | double | 0.0 | 线性PID控制器积分系数 |
| `linear_Kd` | double | 1.0 | 线性PID控制器微分系数 |
| `angular_Kp` | double | 0.5 | 角度PID控制器比例系数 |
| `angular_Ki` | double | 0.0 | 角度PID控制器积分系数 |
| `angular_Kd` | double | 2.0 | 角度PID控制器微分系数 |
| `scale` | int | 1000 | 距离缩放因子 |
| `minDistance` | double | 1.0 | 最小跟随距离(米) |

## 启动文件

### colorTracker_X3.launch.py
启动颜色追踪系统，包含：
- 机器人底盘驱动节点
- 颜色识别节点

## 使用说明

### 1. 颜色识别模式
```bash
# 启动颜色识别节点
ros2 run car_astra colorHSV
```

**操作说明**:
- 按 `i` 键进入识别模式
- 按 `r` 键重置参数
- 按空格键进入追踪模式
- 按 `q` 键退出程序

### 2. 颜色追踪模式
```bash
# 启动完整的颜色追踪系统
ros2 launch car_astra colorTracker_X3.launch.py
```

### 3. HSV参数调整
- 在识别模式下，可以通过鼠标选择区域来自动获取HSV参数
- 参数会自动保存到 `colorHSV.text` 文件中
- 可以通过ROS2参数服务器动态调整HSV参数

## 依赖项
- OpenCV (计算机视觉库)
- PCL (点云处理库)
- ROS2 (机器人操作系统)
- yahboomcar_msgs (自定义消息类型)
- cv_bridge (OpenCV与ROS图像转换)

## 注意事项
1. 需要连接深度摄像头(如Astra系列)
2. 确保光照条件良好，避免颜色识别误差
3. 根据实际环境调整HSV参数范围
4. 追踪模式下请确保安全距离，避免碰撞 