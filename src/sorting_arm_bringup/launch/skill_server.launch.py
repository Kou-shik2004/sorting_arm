import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, relative_path):
    absolute_path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(absolute_path, 'r', encoding='utf-8') as yaml_file:
        return yaml.safe_load(yaml_file)


def generate_launch_description():
    skills_config_arg = DeclareLaunchArgument(
        'skills_config',
        default_value=os.path.join(
            get_package_share_directory('sorting_arm_skills'), 'config', 'skills.yaml'),
    )

    # skill_server_node builds its own MoveGroupInterface robot model; without this
    # it only sees the URDF's limits, not sorting_arm_moveit's joint_limits.yaml overrides.
    robot_description_planning = {
        'robot_description_planning': load_yaml('sorting_arm_moveit', 'config/joint_limits.yaml')
    }

    skill_server_node = Node(
        package='sorting_arm_skills',
        executable='skill_server_node',
        output='screen',
        parameters=[LaunchConfiguration('skills_config'), robot_description_planning],
    )

    return LaunchDescription([skills_config_arg, skill_server_node])
