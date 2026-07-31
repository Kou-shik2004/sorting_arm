# Open RCA: Gripper grasp is unstable in simulation

## Overview

The simulated Robotiq 2F-85 does not yet grasp `red_box_1` reliably. The same `/pick`
workflow can succeed, abort during `close_gripper`, or lift the cube and then drop it during
`retreat`.

This RCA is open. It records what has been proved, what has already been ruled out, and the next
controlled change. Do not restart the investigation from the original gripper parameters. The
detailed working log remains in `ai/debug/gripper-grasp.md`.

The current evidence points to two separate problems:

1. The original client-side close ladder raced the controller and left old goals running. That
   defect is understood and corrected.
2. The later fingertip friction change made `mu=100000` active in the generated SDF. The newest
   runtime timing shows that contact now takes almost the full client timeout to settle. This is
   the leading cause of the current shaking and inconsistent close result, but the proposed
   `mu=5` correction still needs Koushik's runtime confirmation.

## Impact

`/home` succeeds consistently. The failure is limited to `/pick` after the robot starts the grasp
sequence.

Observed outcomes are:

- a successful action followed by the cube falling during the lift;
- an abort after the fingers shake against the cube and `close_gripper` times out; or
- a successful close that finishes only just before the 10-second client timeout.

Because success changes between identical runs, the current branch is not ready to merge into
`main`.

## Reproduction

Koushik uses the same sequence for every observation:

1. Launch the simulation with `gui:=true`.
2. Start `skill_server_node` with `use_sim_time:=true` and the installed `skills.yaml`.
3. Call `/sync_objects`.
4. send the robot to `/home`.
5. Send `/pick` for `red_box_1` with feedback enabled.

The successful `/home` action is not part of this incident. Pre-grasp planning can reject several
IK candidates, but the latest supplied log selected candidate 4 of 5 after a complete Cartesian
descent preflight. Those rejected candidates did not cause the latest contact failure.

## Investigation timeline

### Iteration 0: use simulation time in the manually started skill server

The manually started `skill_server_node` used wall time while Gazebo and MoveIt used simulation
time. That caused retreat retiming to report that no current robot state was available.

Starting the node with `-p use_sim_time:=true` corrected the clock mismatch. A later successful
retreat confirmed that the retiming error was gone.

### Iteration 1: replace the close ladder with one controller-owned goal

The original `GripperCommander::close()` sent a 16-step position ladder. A one-second client
timeout was mistaken for object contact, timed-out goals were not cancelled, and callers ignored
the controller's `stalled` and `reached_goal` result fields.

The close path now sends one `GripperCommand` goal and reads the controller's own stall verdict.
`allow_stalling: true` makes real contact return `SUCCEEDED` with `stalled: true`. A client timeout
cancels its goal so no old command survives its caller.

This removed the goal ladder, but the first run still shook on contact for the full 10-second
client timeout. The controller's default `stall_velocity_threshold` of `0.001` rad/s never saw the
moving joint remain quiet for the one-second stall window.

### Iteration 1b: widen the controller's quiet-velocity band

`stall_velocity_threshold` changed from its `0.001` rad/s default to `0.02` rad/s. Koushik's next
run produced one gripper goal, returned `SUCCEEDED` with `stalled: true` in about one second, and
completed retreat. This confirmed the action and timing corrections.

The cube still fell during retreat.

### Iteration 2: lower the grasp and activate fingertip friction

Forward kinematics and fingertip mesh bounds showed that the pads sit almost entirely above the
`tcp` frame. With `grasp_offset_m: -0.02`, only 21 mm of the cube's 40 mm face was between the
pads, and the contact patch centre was 9.5 mm above the cube's centre of mass.

`grasp_offset_m` changed to `-0.036`. The derived pose puts 37 mm of the cube face between the
pads, centres the contact patch within 1.5 mm of the centre of mass, and leaves about 2.9 mm of
closed-pad clearance above the table.

The same iteration removed invalid `<surface>` elements from the URDF collision elements and
added supported `<gazebo reference="...finger_tip_link">` friction blocks. This made the old
`mu=100000` values real instead of inert.

The two changes were applied together, so the deeper grasp and new friction value did not receive
independent runtime tests.

## Latest evidence

The installed simulation description contains the Iteration 2 values:

- `grasp_offset_m: -0.036`;
- `stall_velocity_threshold: 0.02`; and
- generated SDF fingertip friction values `<mu>100000</mu>` and `<mu2>100000</mu2>` on both pads.

In the latest successful log, `gripper_controller` accepted the close goal at
`1785531988.171`. MoveIt received the retreat Cartesian-path request at `1785531997.873`.
The gap is about 9.70 seconds. Close and verification therefore consumed almost the complete
10-second client timeout. In other runs, visible shaking continues long enough for the client to
cancel the goal and abort `/pick`.

Iteration 1b had already shown an approximately one-second stall result before `mu=100000` became
active. The generated SDF proves that the new value reaches the physics engine. The timing
regression began after that value became active. This is direct evidence that the extreme pad
friction is destabilizing Bullet contact and making the result depend on small timing differences.

The latest successful action still dropped the cube during retreat. That does not disprove the
deeper grasp measurement because the unstable `mu=100000` contact was active in the same run.
The deeper grasp has not yet been observed with a stable, realistic pad friction value.

## Causes ruled out

Do not repeat these checks without new contradictory evidence:

- The UR5 reference's gripper parameters are not a valid direct comparison. Its close is
  fire-and-forget, while this project waits for and verifies a `GripperCommand` result.
- `max_effort` is inert on this position-only controller interface.
- The 50 N m knuckle effort limit is not the constraint. The position adapter already drives the
  joint motor into its configured limit at contact.
- Raising `close_position` or `position_proportional_gain` does not add useful squeeze here. The
  motor is already saturated against the cube.
- Cube mass and friction already match the working reference class of values.
- Mimic-joint wiring, Bullet Featherstone, and simulation step size already match the established
  controller contract.
- MoveIt, IK, grasp-pose generation, and approach execution can reach the object. Candidate search
  failures before a later valid candidate are separate from the contact instability.
- MoveIt's attached collision object does not physically carry the Gazebo cube. `attach()` only
  updates the planning scene, so a real simulated pad contact must hold the object during retreat.

## Current root-cause hypothesis and next correction

The current contact instability comes from the fingertip friction coefficient of `100000`. It is
many orders of magnitude beyond the cube's coefficient of `5` and gives the contact solver a poor
numerical condition. The resulting shake keeps joint velocity above the stall threshold for a
variable length of time.

The next change is deliberately limited to one variable:

- keep the valid Gazebo friction blocks;
- change both pads' `mu1` and `mu2` from `100000.0` to `5.0`;
- keep `grasp_offset_m: -0.036`;
- keep `stall_velocity_threshold: 0.02`; and
- keep the single-goal close implementation and its 10-second safety timeout.

This preserves the measured full-face grasp while matching the cube's already adequate friction.
It also isolates the friction correction from every previously proved controller change.

## Required evidence before closing this RCA

Static validation must show that the installed simulation xacro parses and that both generated SDF
pad collisions contain `mu=5` and `mu2=5`.

Koushik then runs one standard GUI workflow. The RCA can close only when that run shows all of the
following:

1. descent reaches the object;
2. the fingers do not sustain visible contact shaking;
3. close returns `SUCCEEDED` with `stalled: true` near the one-second stall window, not near the
   10-second client timeout;
4. the cube remains between the pads throughout retreat; and
5. `/pick` returns `ok: true`.

Until that runtime evidence exists, `mu=5` is a planned correction, not a confirmed resolution.
