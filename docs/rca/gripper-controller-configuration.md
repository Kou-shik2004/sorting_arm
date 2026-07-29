# Gripper controller configuration mismatch

On 27 Jul 2026, the generated gripper configuration used a trajectory-controller
schema while MoveIt expected a `GripperCommand` action. Correcting the controller
required aligning its plugin, parameter schema, hardware interfaces, and runtime
dependencies with the installed ROS 2 Jazzy controller contract.

This report records the failure mode, corrections, verification evidence, and
remaining compatibility work.

## The failure being fixed

`src/sorting_arm_moveit/config/moveit_controllers.yaml` tells MoveIt to drive the
gripper over a `GripperCommand` action at namespace `gripper_cmd`. The generated
`src/sorting_arm_bringup/config/ros2_controllers.yaml` used a
`JointTrajectoryController`, which serves `follow_joint_trajectory` rather than
`gripper_cmd`.

Nothing about that combination fails at build or launch time. The package compiles, `move_group` starts, RViz plans a path to `gripper_close`, and only then does execution stall against an action server that was never advertised. That is the worst shape a config defect can take, which is why it is the one the P1 gate checks for directly.

## Edit 1 — controller type

`src/sorting_arm_bringup/config/ros2_controllers.yaml`

```yaml
gripper_controller:
  type: position_controllers/GripperActionController   # was joint_trajectory_controller/JointTrajectoryController
```

The plugin string keeps the `position_controllers/` prefix even though the package
that registers it is `gripper_controllers`. The installed
`/opt/ros/jazzy/share/gripper_controllers/ros_control_plugins.xml` declares
`position_controllers/GripperActionController` under
`<library path="gripper_action_controller">`; a plugin namespace and its package name
do not have to match.

## Edit 2 — singular `joint:`

`src/sorting_arm_bringup/config/ros2_controllers.yaml`

```yaml
gripper_controller:
  ros__parameters:
    joint: robotiq_85_left_knuckle_joint
```

`GripperActionController` declares `joint` as a string, not `joints` as a list. The generated parameter header `/opt/ros/jazzy/include/gripper_controllers/gripper_action_controller_parameters.hpp:259` is the authority; its other parameters are `action_monitor_rate`, `goal_tolerance`, `max_effort`, `allow_stalling`, `stall_velocity_threshold` and `stall_timeout`. It declares no `command_interfaces`, no `state_interfaces` and no `allow_nonzero_velocity_at_trajectory_end`, so those three keys were dropped rather than carried over.

The two YAML files in this package genuinely disagree on schema for the same gripper, and both are right:

| File | Key | Consumer |
|---|---|---|
| `ros2_controllers.yaml` | `joint:` string | the controller's own parameter declaration |
| `moveit_controllers.yaml` | `joints:` list | `moveit_simple_controller_manager` |

The schemas intentionally differ because the files have different consumers. Making
their key shapes match would violate one of those consumers' contracts.

## Edit 3 — velocity state interface

`src/sorting_arm_description/urdf/sorting_arm.ros2_control.xacro`

```xml
<joint name="robotiq_85_left_knuckle_joint">
    <command_interface name="position"/>
    <state_interface name="position">...</state_interface>
    <state_interface name="velocity"/>
</joint>
```

This one was not in the plan. `gripper_action_controller_impl.hpp:331-337` claims **two** state interfaces:

```cpp
return {
  controller_interface::interface_configuration_type::INDIVIDUAL,
  {params_.joint + "/" + hardware_interface::HW_IF_POSITION,
   params_.joint + "/" + hardware_interface::HW_IF_VELOCITY}};
```

The Setup Assistant's Screen 8 offers one command/state pair for the whole robot, so it had given the driver joint `position` state only. Edits 1 and 2 alone would therefore have swapped one silent failure for another: the controller loads, then refuses to activate because `robotiq_85_left_knuckle_joint/velocity` is not available, and `list_controllers` reports it `inactive`.

The installed controller implementation establishes that velocity state is required,
not an optional simulator detail.

**Re-running Screen 8 reverts this.** The Setup Assistant regenerates `sorting_arm.ros2_control.xacro` from the `control_xacro` block in `.setup_assistant`, which records `position` only. Any future MSA pass needs this interface added back by hand.

## Edit 4 — runtime dependencies

`src/sorting_arm_bringup/package.xml` had `joint_trajectory_controller` commented out
and omitted `gripper_controllers`, while `ros2_controllers.yaml` names plugins from
both packages. These are runtime dependencies loaded by `controller_manager` for
simulation and mock-hardware launches.

No Docker change was needed: `.docker/packages/ros.txt` already lists `ros-jazzy-joint-trajectory-controller` and `ros-jazzy-gripper-controllers`, so the image was ahead of the manifest.

## Gate result

```
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
gripper_controller      position_controllers/GripperActionController           active

/gripper_controller/gripper_cmd
```

`xacro … | check_urdf -` parses, root link `world`.

## Open item — the plugin is deprecated

`controller_manager` emits this on activation:

```
[WARN] [gripper_controller]: [Deprecated]: the `position_controllers/GripperActionController` and
`effort_controllers::GripperActionController` controllers are replaced by
'parallel_gripper_controllers/GripperActionController' controller
```

The configured plugin works, but the installed replacement registers as
`parallel_gripper_action_controller/GripperActionController` and uses a different
parameter set. Migration remains a deliberate compatibility task because the current
plugin satisfies the controller activation gate.

## Not fixed, on purpose

- `.setup_assistant` declares `control_xacro` twice with identical content. Tool artifact, regenerating does not clear it. Harmless to `MoveItConfigsBuilder`, but a strict YAML loader rejects duplicate keys.
- `package.xml` lists `moveit_ros_move_group`, `tf2_ros` and `xacro` twice each. Also tool output, also harmless.
- `joint_limits.yaml` velocity and acceleration scaling stay at `0.1`. That is a tunable, not a defect.
