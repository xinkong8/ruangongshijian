# robot_pose_publisher_ros2

This is ros2 verison of [robot_pose_publisher](https://github.com/GT-RAIL/robot_pose_publisher)


# ROS2 节点 README 文档

## 节点名称
**robot_pose_publisher**

## 功能描述
该节点的主要功能是基于TF（变换框架）获取机器人在地图坐标系（`/map`）下的位姿，并将其发布为`geometry_msgs/msg/PoseStamped`或`geometry_msgs/msg/Pose`消息。节点通过定时器定期查询TF变换，获取机器人在地图坐标系中的位置和姿态，并将这些信息封装到相应的消息类型中进行发布，供其他节点使用。

## 订阅接口
该节点没有显式的订阅接口，因为它依赖于TF（变换框架）来获取机器人位姿信息。节点通过`tf2_ros::TransformListener`监听TF变换，从`/map`到`/base_link`的变换被用于计算机器人位姿。

## 发布接口
| 话题名称      | 消息类型                                      | 发布内容描述                     |
|:--------------|:----------------------------------------------|:-------------------------------|
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` 或 `geometry_msgs/msg/Pose` | 发布机器人在地图坐标系下的位姿数据 |

## 其他信息

### 代码中的问题
1. **异常处理不够完善**：
   - 在`timer_callback`函数中，如果TF查询失败，异常被捕获但没有记录具体的错误信息，这在调试时可能会导致问题难以定位。

2. **参数验证不足**：
   - 节点中使用了参数`map_frame`、`base_frame`和`is_stamped`，但在代码中没有对这些参数进行充分的验证，可能导致节点在运行时出现意外行为。

3. **资源管理问题**：
   - 在节点构造函数中，创建了多个共享指针（如`tf_buffer_`、`tf_listener_`、`publisher_stamp`、`publisher_`），但在节点销毁时没有明确释放这些资源，可能导致资源泄漏。

4. **日志输出不够详细**：
   - 节点在运行过程中缺乏足够的日志输出，无法方便地监控节点的状态和数据流动。

### 修改建议
1. **完善异常处理**：
   - 在异常处理中添加具体的错误信息，方便调试和问题定位。
   - 修改代码：
     ```cpp
     void timer_callback()
     {
         geometry_msgs::msg::TransformStamped transformStamped;
         try
         {
             transformStamped = tf_buffer_->lookupTransform(map_frame, base_frame, this->now());
         }
         catch (tf2::TransformException &ex)
         {
             RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
             return;
         }
         // 其他逻辑...
     }
     ```

2. **添加参数验证**：
   - 在设置参数后，添加验证逻辑，确保参数值合理。
   - 示例修改：
     ```cpp
     RobotPosePublisher() : Node("robot_pose_publisher")
     {
         // 参数声明和获取...
         if (map_frame.empty() || base_frame.empty())
         {
             RCLCPP_ERROR(this->get_logger(), "Invalid frame parameters");
             throw std::runtime_error("Invalid frame parameters");
         }
         // 其他逻辑...
     }
     ```

3. **优化资源管理**：
   - 在节点析构函数中明确释放共享资源，确保没有资源泄漏。
   - 修改代码：
     ```cpp
     ~RobotPosePublisher()
     {
         tf_buffer_.reset();
         tf_listener_.reset();
         publisher_stamp.reset();
         publisher_.reset();
     }
     ```

4. **添加详细日志输出**：
   - 在关键步骤添加日志输出，方便监控节点运行状态。
   - 示例修改：
     ```cpp
     void timer_callback()
     {
         // 查询TF变换...
         RCLCPP_INFO(this->get_logger(), "Published robot pose: (%.2f, %.2f, %.2f)", 
             pose_stamped.pose.position.x, 
             pose_stamped.pose.position.y, 
             pose_stamped.pose.position.z);
         // 发布消息...
     }
     ```

### 使用说明
1. **运行节点**：
   - 确保ROS2环境已正确配置，并安装了必要的依赖项。
   - 使用以下命令运行节点：
     ```bash
     ros2 run <package_name> robot_pose_publisher
     ```

2. **验证节点功能**：
   - 使用`ros2 topic echo /robot_pose`命令查看发布的机器人位姿数据。
   - 确保TF变换框架中存在从`/map`到`/base_link`的变换，否则节点将无法正常工作。

3. **参数调整**：
   - 根据实际应用场景，可以调整节点中的参数，如`map_frame`、`base_frame`和`is_stamped`，通过命令行或配置文件进行设置。

通过以上修改和优化，可以提高节点的稳定性和可维护性，使其更好地适应实际应用需求。