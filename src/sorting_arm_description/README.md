# sorting_arm_description

URDF/xacro and meshes for the UR5e arm and Robotiq 2F-85 gripper, plus the
kinematic and physical parameters both the mock-hardware and Gazebo paths build
from.

This package has no launch files of its own. To see the robot, use
`sorting_arm_bringup`'s `display.launch.xml` (RViz only) or `moveit.launch.xml`
(mock hardware, MoveIt planning, no Gazebo).

## Parameters you might change

| File | Controls |
|---|---|
| `config/initial_positions.yaml` | The joint state the robot starts in under mock hardware and Gazebo. Keep it matched to the SRDF `home` state in `sorting_arm_moveit`. |
| `config/physical_parameters.yaml` | Link masses and inertial radii used by the xacro macros. |
| `config/joint_limits.yaml` | Velocity and acceleration scaling for the arm's joints. |
| `config/default_kinematics.yaml` | UR5e DH parameters, only needed if you're calibrating against a different physical arm. |
| `urdf/sorting_arm.urdf.xacro` | The top-level robot description. `sorting_arm.mock.urdf.xacro` and `sorting_arm.sim.urdf.xacro` layer mock-hardware and Gazebo specifics on top of it. |

## Validating the description on its own

```bash
xacro urdf/sorting_arm.urdf.xacro | check_urdf -
```

Confirms the xacro parses and the root link is `world`, without needing anything
else running.
