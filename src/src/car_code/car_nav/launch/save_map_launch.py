from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    # 获取 yahboomcar_nav 包的共享路径，并将其转换为字符串类型
    package_share_path = str(get_package_share_path('yahboomcar_nav'))
    # 获取 yahboomcar_nav 目录的绝对路径，通过共享路径向上回溯到 src 目录下的 yahboomcar_nav 目录
    package_path = os.path.abspath(os.path.join(
        package_share_path, "../../../../src/yahboomcar_nav"))
    # 定义地图名称
    map_name = "yahboomcar"
    # 拼接默认的地图保存路径，将包路径、maps 文件夹和地图名称组合起来
    default_map_path = os.path.join(package_path, 'maps', map_name)

    # 声明一个启动参数，名称为 'map_path'，默认值为上面拼接的默认地图路径
    # 该参数用于指定地图保存的路径，description 描述了该参数的作用
    map_arg = DeclareLaunchArgument(name='map_path', default_value=str(default_map_path),
                                    description='The path of the map')

    # 创建一个地图保存节点
    map_saver_node = Node(
        # 指定节点所属的包为 nav2_map_server
        package='nav2_map_server',
        # 指定要执行的可执行文件为 map_saver_cli，用于保存地图
        executable='map_saver_cli',
        # 传递给节点的命令行参数
        # '-f' 用于指定地图保存的文件路径，从启动参数 'map_path' 中获取
        # '--ros-args -p save_map_timeout:=10000' 用于设置保存地图的超时时间为 10000 毫秒
        arguments=[
            '-f', LaunchConfiguration('map_path'), '--ros-args', '-p', 'save_map_timeout:=10000'],
    )

    # 返回一个 LaunchDescription 对象，该对象包含了声明的启动参数和要启动的地图保存节点
    return LaunchDescription([
        map_arg,
        map_saver_node
    ])
