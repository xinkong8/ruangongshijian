from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的共享路径
    yahboomcar_description_path = get_package_share_path('yahboomcar_description')
    # 定义 URDF 文件路径
    urdf_path = PathJoinSubstitution(
        [yahboomcar_description_path, 'urdf', 'yahboomcar_R2.urdf']
    )
    # 定义 Gazebo 启动文件路径
    gazebo_launch_path = PathJoinSubstitution(
        [get_package_share_path('gazebo_ros'), 'launch', 'gazebo.launch.py']
    )

    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
    )

    # 启动机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(str(urdf_path.perform(None)), 'r').read()
        }],
    )

    # 将机器人模型加载到 Gazebo 中
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'yahboomcar',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
