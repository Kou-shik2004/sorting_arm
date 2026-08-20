from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'sorting_arm', package_name='sorting_arm_moveit'
    ).to_moveit_configs()
    # The skill server runs a MoveIt Task Constructor task and calls task.execute();
    # that goes through move_group's ExecuteTaskSolutionCapability action server, so
    # load it here (D31). generate_move_group_launch reads this as the capabilities
    # param default.
    moveit_config.move_group_capabilities['capabilities'] = (
        'move_group/ExecuteTaskSolutionCapability'
    )
    # The gripper close runs through move_group now (task.execute), and GripperCommand
    # only reports success once the fingers stall on the cube — an unpredictable time.
    # The default duration window (1.2x + 0.5s) aborts that stall, so give it wide margin.
    moveit_config.trajectory_execution.update({
        'trajectory_execution.allowed_execution_duration_scaling': 5.0,
        'trajectory_execution.allowed_goal_duration_margin': 5.0,
    })
    return generate_move_group_launch(moveit_config)
