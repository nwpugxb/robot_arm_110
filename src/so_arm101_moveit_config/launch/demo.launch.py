from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("so100", package_name="so_arm101_moveit_config")
        .robot_description(file_path="config/so100.urdf.xacro")
        # 核心：显式指定轨迹执行配置文件
        .trajectory_execution(file_path="config/moveit_controllers.yaml") 
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)    
