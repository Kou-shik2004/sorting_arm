import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    skills_config_arg = DeclareLaunchArgument(
        'skills_config',
        default_value=os.path.join(
            get_package_share_directory('sorting_arm_skills'), 'config', 'skills.yaml'),
    )
    # skill_server_node now builds an MTC task: task.loadRobotModel + PipelinePlanner
    # (OMPL) + ComputeIK (kinematics) all read these params off the node, so it needs
    # the same full MoveIt config move_group gets (URDF, SRDF, kinematics, planning
    # pipelines, joint limits), not just the joint-limits override it used before.
    moveit_config = MoveItConfigsBuilder(
        'sorting_arm', package_name='sorting_arm_moveit'
    ).to_moveit_configs()

    skill_server_node = Node(
        package='sorting_arm_skills',
        executable='skill_server_node',
        output='screen',
        parameters=[moveit_config.to_dict(), LaunchConfiguration('skills_config')],
    )

    return LaunchDescription([skills_config_arg, skill_server_node])
