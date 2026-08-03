# Gripper motion used the wrong abstraction

## Summary

On 29 Jul 2026, a minimal program tried to close the Robotiq gripper by planning its one commanded
joint through MoveIt. Gazebo reported that joint a microscopic distance below its lower boundary,
so Jazzy MoveIt rejected the start state before planning.

The first proposed correction was to loosen the MoveIt lower limit. That was a symptom patch. The
root cause was using the arm-planning abstraction for a one-DOF gripper even though architectural
decision D3 already required direct `GripperCommand` control.

## Original failure

The program created a `MoveGroupInterface` for the `gripper` group, copied the current state into
the planning request, and selected the named `gripper_close` target:

```cpp
auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");
gripper.setStartStateToCurrentState();
gripper.setNamedTarget("gripper_close");
gripper.plan(plan);
```

MoveIt accepted the request and then aborted it:

```text
[moveit.ros.check_start_state_bounds]: Joint 'robotiq_85_left_knuckle_joint'
from the starting state is outside bounds by: [-4.35514e-11 ]
should be in the range [0 ], [0.8 ].

PlanningRequestAdapter 'CheckStartStateBounds' failed, because
'Start state out of bounds.'.
```

All three controllers were active and `/gripper_controller/gripper_cmd` existed. The failure
happened before controller execution: Gazebo reported the joint a microscopic distance below its
lower boundary, and Jazzy MoveIt rejected that value while validating the `gripper` planning group.

## Rejected symptom patches

The first proposed fix was to override the MoveIt lower limit with a small negative margin. That
would make this one state pass validation, but it changes a valid physical limit to accommodate
numerical residue. It does not correct the abstraction that put the value through
OMPL, so the proposal was rejected.

Changing the Gazebo initial position to `0.7929` is not the root fix either. That
value starts the gripper almost fully closed and avoids the lower boundary only until
the first open command returns the joint to zero.

## Root cause

The program used the arm-planning abstraction for the gripper despite architectural decision D3:

- the six-joint arm needs MoveIt for collision-aware planning, IK and trajectory generation;
- the Robotiq controller exposes one commanded joint and only needs a requested opening value and
  maximum effort;
- a gripper close needs the `GripperCommand` result fields `stalled` and `reached_goal` for grasp
  verification, which an OMPL plan does not provide.

The controller configuration was not the cause. `gripper_controller` was active, its required
interfaces were claimed, and its action server was available. That separate work remains documented
in [Gripper controller configuration mismatch](gripper-controller-configuration.md).

## Resolution

Use `MoveGroupInterface` for arm motion and `control_msgs/action/GripperCommand`
directly for gripper motion. The client uses `0.0` for open and `0.8` for close,
matching the SRDF named-state values, and populates maximum effort `50.0`, matching
the URDF joint limit. It does not create a gripper `MoveGroupInterface`, parse either
description, or calculate a target from object width. Keep the SRDF gripper group for
semantic modelling and RViz configuration; do not make it the runtime command path.

Before proposing a configuration change for an implementation failure, check the applicable
architectural decision and confirm that the failing program uses the intended abstraction. Do not
loosen URDF or MoveIt joint limits to hide simulator floating-point residue.
