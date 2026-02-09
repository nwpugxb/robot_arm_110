from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 这一步非常关键：加载 robot_description, kinematics, limits 等所有配置
    moveit_config = (
        MoveItConfigsBuilder("so100", package_name="so_arm101_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="hello_moveit",
            executable="hello_moveit_node",
            name="hello_moveit_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(), # 必须把配置传给节点
                {"use_sim_time": True},  # 如果你在仿真，这行很重要；如果是真机，设为 False
            ],
        )
    ])
