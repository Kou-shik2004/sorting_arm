# Closed RCA: Gripper grasp was unstable in simulation

**Closed at Iteration 13.** `/pick` returns `ok: true` through `attach`/`retreat`/`verify_hold`
against `red_box_1`, confirmed on two consecutive Gate C runs (2026-08-01). The
investigation ran Iterations 0–13 across two sessions. This file is a restructured
version of that record: the verdict, every ruled-out cause, and every mechanism worth
carrying forward are kept in full; the iteration-by-iteration narrative, duplicated
source quotes, and superseded status snapshots are compressed into the table below. The
full original narrative is in git history at this file's pre-restructure revision
(`3586ad8`, merged as `bfb9279` on `main`).

Do not restart the investigation from the original gripper parameters. Read "Ruled out"
before repeating any check.

## Overview

The simulated Robotiq 2F-85 did not grasp `red_box_1` reliably for most of this
investigation. The same `/pick` sequence could succeed, abort during `close_gripper`, or
lift the cube and drop it during retreat, with no code change between identical-looking
runs. `/home` succeeded consistently throughout; the failure was always downstream of the
grasp sequence starting.

## Verdict — three combined causes, none sufficient alone

1. **The close target moved off the mechanical hard stop** (Iteration 12).
   `close_position: 0.8` is the knuckle's URDF upper limit; a 40 mm cube stops the joint at
   `0.4484 rad`. Commanding `0.8` against that left a permanent `0.35 rad` position error,
   and `gz_ros2_control`'s position→velocity adapter (`velocity = position_proportional_gain
   × error × update_rate`, gain `0.1`, rate `100 Hz`, never overridden) turned that into a
   continuous `3.52 rad/s` demand on a joint rated `0.5 rad/s`. Every iteration from 0
   through 11 shows the same shaking/chattering symptom because of this; nothing before
   Iteration 12 touched it. Fix: `close(object_width_m)` now targets the knuckle angle for
   `object_width_m − squeeze_depth_m` — a derived window, `1.10–5.52 mm` past contact —
   never the hard stop.
2. **The physics engine moved to `dart`** (Iteration 12, a platform decision, not a fix
   under test — see `docs/project/decisions.md` D24). `dart` is Gazebo Harmonic's
   default/primary engine; `bullet_featherstone` is documented upstream as preliminary.
   Measured consequence: a free-air raw-goal run (Gate B) showed `0.000 mrad` fingertip
   symmetry under `dart`, against `bullet_featherstone`'s photographed `3.6°–5.4°`
   mismatches in earlier iterations — a real, measured improvement, though not the one
   this RCA set out to prove.
3. **Verification stopped trusting the controller's own verdict** (Iteration 13).
   `GripperActionController` never resolved a terminal result on either Gate C run — a
   periodic `+0.15 rad/s` velocity reading, recurring every 0.3–1.4 sim-seconds during
   sustained rigid contact under `dart`, kept resetting the controller's 1.0 s
   quiet-window requirement. `close()` now measures the jaw directly (gap + fingertip
   symmetry from `/joint_states`) on a `"gripper result timed out"` failure instead of
   treating the timeout as proof nothing was caught, because the client's own cancel
   triggers `set_hold_position()`, which freezes the target wherever the joint already is
   — not at open.

**Acceptance evidence**, from the controller log and `/pick`'s own result, never RViz:

| # | Point | Evidence |
|---|---|---|
| 1 | Both pads close symmetrically | `0.000016 rad` fingertip residual on the passing run |
| 2 | Pads touch the box and maintain pressure | measured gap `39.76 mm` vs. the `40 mm` box, held through `attach`/`retreat` with no goal or cancel sent after capture |
| 3 | `GripperActionController` returns its own terminal successful result | **not met as originally written** — see Iteration 13: `GraspOutcome.ok` was redefined from "the action completed" to "a usable verdict exists, from the controller or from direct measurement"; direct jaw measurement stands in for this point |
| 4 | `/pick` proceeds past `close_gripper` | confirmed both Gate C runs |
| 5 | Arm retreats while holding the box | `verify_hold` passes post-retreat |
| 6 | `/pick` returns `ok: true` | confirmed |
| 7 | Every number from the controller log / `/joint_states`, never RViz | confirmed |

## Iteration table

| # | Hypothesis / change | Runtime result | Verdict |
|---|---|---|---|
| 0 | `skill_server_node` ran on wall time while Gazebo/MoveIt used sim time | fixed a retiming clock mismatch | Right, real defect, unrelated to the grasp |
| 1 | Replace the 16-step client-side close ladder with one controller-owned goal, read native `stalled`/`reached_goal` | one goal now sent with `allow_stalling`; first run still shook for the full 10 s timeout | Right and prerequisite — cleared the racing-goal defect so later measurements became interpretable |
| 1b | Widen `stall_velocity_threshold` `0.001`→`0.02` | one clean `stalled: true` in ~1 s; retreat still dropped the cube | Confirms the timing fix; does not fix the grasp |
| 2 | Deepen `grasp_offset_m` `-0.02`→`-0.036` (pad-face geometry) and activate `mu=100000` fingertip friction together | both changes shipped in one run, untested independently | `-0.036` sound and durable; friction untestable at this depth |
| 3 | Lower fingertip friction `100000`→`5` | both runs still timed out in `close_gripper`; visible shaking continued after abort | Did not resolve — friction was never actually being exercised (see Iteration 7) |
| 4 | Use ROS time instead of wall time for the client's result deadline | 3 of 4 runs timed out; 1 stalled weakly and dropped the cube in retreat | Wall-time mismatch ruled out; confirmed the running controller had the expected parameters loaded |
| 5 | Match upstream `robotiq_description` exactly (`continuous` followers, no limits, inline `<surface>` friction) | reproduced the identical failure signature | Wrong — and reintroduced the unlimited-follower defect Iteration 7 had to undo |
| 6 | Bag `/joint_states` through a full close | position stayed frozen; *reported* velocity spiked to `0.05–0.37 rad/s` roughly 60 times in 5 s with no corresponding motion — traced to `gz_ros2_control::GazeboSimSystem::read()` copying `gz-sim`'s unfiltered `JointVelocity` value | Right observation (the velocity readback is noisy); wrong response attempted next (tuning the threshold instead of not trusting it) |
| 7 | Decode what the recorded stall angles meant physically | stall angles (`0.175–0.177 rad`) convert to a `68.6 mm` jaw gap — **15.1 mm of open air**, never contact, on every run on record | **The central finding of the whole RCA up to this point.** Fix: restore follower joint limits (`revolute`, real effort/velocity), move fingertip friction to `<gazebo>` blocks (the inline `<surface>` was silently dropped by URDF→SDF conversion), use a taller box for table clearance. Also surfaced a real-time-factor collapse to `0.09` under the reconstrained followers |
| 8 | Stop the second mimic enforcer — remove the five follower joints from `<ros2_control>` entirely (state-interface presence alone still let `gz_ros2_control` drive them against the SDF-native constraint) | `close_gripper` no longer times out or shakes; `/pick` aborts at `verify_grasp`: "reached close_position uncontested: no object captured" | Right — the actuation problem is gone; a different, better failure (targeting) is exposed |
| 9 | Revert Iteration 7's box-geometry experiment (`0.04×0.04×0.06`→back to `0.04³`, matching `blue_box_2`/`red_box_2` exactly) | pre-grasp and descent succeed; `close_gripper` times out at 10 s with **no stall**, no result | Right — fixes a real synced-dimension-vs-spawned-box mismatch; a new failure (table-clearance chatter) surfaces |
| 10 | Open `grasp_offset_m` `-0.036`→`-0.028` for more table clearance | **worse** — all 5 pre-grasp candidates failed (planner timeout / preflight coverage) | Wrong, reverted. Re-running the reverted `-0.036` reproduced Iteration 9's result exactly, confirming it wasn't a fluke |
| 11 | Ladder-close to measured width + margin; verify capture by measured jaw gap instead of `stalled` | the 50 mm free-space approach step itself never completed — the driven knuckle oscillated at a 57.3 mm minimum, in open air | Right idea, wrong entry point — exposed a prior, still-unexplained oscillation-in-free-air defect before the ladder logic ever ran |
| 12 | **Close to `object_width_m − squeeze_depth_m`, never the hard stop; switch physics engine to `dart` (platform decision)** | Gate B (free-air raw goal): clean settle, `0.000 mrad` symmetry. Gate C (full `/pick`): pads photographed flush and centred, genuine `39.8 mm`/`40 mm` contact — but the action still timed out, with no shaking, chattering, or wobble at all | **Right — the dominant cause.** The physical grasp is now solid; what's left is a controller-verdict problem, not a physical one |
| 13 | **Fall back to direct `/joint_states` jaw measurement when `send_goal` reports `"gripper result timed out"`**, since a timeout doesn't reopen the jaw | `/pick` returns `ok: true` through `verify_hold`, reproduced on a second run | **Right — the closing cause.** Combined with 12, this closes the RCA |

**In practice, two changes closed this RCA — Iteration 12 and Iteration 13.** Every
iteration before them shows the same dominant symptom in some form (shaking, chattering,
or wobbling during `close_gripper`) because of one thing true in every one of them and
only changed at Iteration 12: the close target was the mechanical hard stop.

## Mechanisms worth keeping

**`GripperActionController::update()` — `stalled` means "stopped watching," not "stopped
moving."** Verified directly against the installed source
(`/opt/ros/jazzy/include/gripper_action_controller/gripper_controllers/gripper_action_controller_impl.hpp`):

```cpp
controller_interface::return_type GripperActionController<HardwareInterface>::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  command_struct_rt_ = *(command_.readFromRT());
  const double current_position = joint_position_state_interface_->get().get_optional().value();
  const double current_velocity = joint_velocity_state_interface_->get().get_optional().value();
  const double error_position = command_struct_rt_.position_ - current_position;
  const double error_velocity = -current_velocity;

  check_for_success(get_node()->now(), error_position, current_position, current_velocity);

  // Hardware interface adapter: Generate and send commands
  computed_command_ = hw_iface_adapter_.updateCommand(
    command_struct_rt_.position_, 0.0, error_position, error_velocity,
    command_struct_rt_.max_effort_);
  return controller_interface::return_type::OK;
}
```

`check_for_success()` marks the goal `SUCCEEDED` (`reached_goal`), `SUCCEEDED` with
`stalled: true` (only because `allow_stalling: true`), or `ABORTED` — and in every case
clears the active goal, so no more *results* are ever reported for it. It never touches
`command_struct_rt_.position_`, and `updateCommand()` three lines below runs
unconditionally, every cycle, regardless of whether an active goal exists. Only two things
ever change that stored target: a new goal being accepted, or a cancellation, which calls
`set_hold_position()` — explicitly freezing the target at the current position. A
stalled-or-reached `SUCCEEDED` result does neither. The joint keeps being driven toward
whatever the target was, indefinitely, with no client watching, until a cancel or a new
goal changes it. This is why every `stalled: true` verdict logged before Iteration 12 was
read as "settled contact, safe to proceed" when it only ever meant "the controller stopped
watching."

**`gz_ros2_control`'s position→velocity adapter is a proportional loop with no upper
bound.** `velocity_sp = position_proportional_gain × position_error × update_rate`. The
never-overridden default gain is `0.1`, `update_rate: 100` — gain `10` on the driven
joint. The mimic drive for follower joints runs the equivalent loop at gain `100`
(`velocity_sp = -(position_follower - position_driven × multiplier) × update_rate`, no
separate proportional term) — ten times stiffer than the joint it tracks. Neither loop
caps its output at the joint's own velocity rating; a large position error simply asks for
whatever multiple of that rating the arithmetic produces.

**Harmonic hands a mimic joint to two governors where Fortress hands it to one.** sdformat
14 (Gazebo Harmonic) converts a URDF `<mimic>` tag into a real SDF `<axis><mimic>`
constraint; Fortress-era sdformat does not generate one at all. `bullet_featherstone`
exports the constraint feature (`JointFeatures::SetJointMimicConstraint`, confirmed against
the installed plugin); `dart` exports zero mimic-constraint symbols (checked with both
`strings` and `nm -D`, `337` hits for `bullet_featherstone`, `0` for `dartsim`).
`gz_ros2_control` separately auto-detects the same URDF tag via `hardware_interface`'s own
parser and runs its own control-cycle mimic drive on any joint listed in `<ros2_control>`
at all — state-interface-only does not exempt it. Under Harmonic + `bullet_featherstone`, a
follower listed in `<ros2_control>` therefore gets two independent enforcers unless it is
removed from `<ros2_control>` entirely (Iteration 8) or the native constraint is absent
(true under `dart`, which is why the followers can safely stay in `<ros2_control>` again
under the Iteration 12 engine switch). This is also why `arm_stack`'s
Humble/Fortress/`ign_ros2_control` `<ros2_control>` shape does not transfer: it only ever
had one enforcer to begin with.

**The jaw-gap closed form is the measurement basis for the whole fix**, now living in
`helpers.hpp`/`.cpp` (`jaw_gap_m`/`knuckle_angle_for_gap_m`) instead of only in this file:

```
gap(th) = 2 × (c + a·cos(th) − b·sin(th)),  a = 0.0371575, b = 0.04342168, c = 0.03060114 − 0.02526
```

Endpoint-checked against the spec: `gap(0) = 85.0 mm` (rated open), `gap(0.8) ≈ 0.15 mm`
(fully closed). The inverse is exact via the `R·cos(th+phi)` identity
(`R = hypot(a,b)`, `phi = atan2(b,a)`), rejecting an out-of-range gap rather than clamping
it. At the 40 mm cube's contact angle (`th = 0.4484 rad`), local sensitivity is
`dgap/dth ≈ -0.1105 m/rad` — 1 mm of gap is 9.05 mrad of knuckle. That sensitivity is what
bounds `squeeze_depth_m`'s usable window (`1.10–5.52 mm`, mid-point `0.003 m`) and derives
`capture_tolerance_m` (`0.005 m`, ≈0.55 mm of the allowed 5 mrad symmetry residual, the
remainder budgeted for contact penetration and solver slop).

**The `dart`-side periodic velocity artifact is real, measured, and still unexplained.**
During sustained rigid contact, the driven joint's reported velocity spikes to roughly
`+0.15 rad/s` every 0.3–1.4 sim-seconds, each spike resetting the controller's 1.0 s
quiet-window timer. On the passing Gate C run the longest quiet stretch measured `0.98 s`
against the `1.0 s` requirement — missed by 20 ms. Converting each `/joint_states` sample's
own sim-time stamp (not wall-clock arrival time) gives an effective real-time factor of
`0.848` for that window — far healthier than Iteration 7's `0.09` collapse under
`bullet_featherstone`, but still below 1.0. This is a materially different signature from
Iteration 6's `bullet_featherstone` finding (chaotic, both-signed, position frozen); this
one is single-signed and semi-regular, consistent with a periodic contact-solver
re-evaluation under `dart`'s LCP-based solver, though the exact mechanism isn't pinned down
further — nothing currently bridged (no contact-force topic) localises it more precisely
than "engine-side, tied to sustained rigid contact."

## Ruled out — do not repeat without new contradictory evidence

- **The UR5 reference's gripper parameters are not a valid comparison.** Its close is
  fire-and-forget; this project waits for and verifies a `GripperCommand` result.
- **`max_effort` is inert** on this position-only controller interface — `move_group` logs
  "will command a max effort of: 0" every run.
- **Raising the knuckle's 50 N·m effort limit or `position_proportional_gain`** adds no
  usable squeeze — the motor already saturates against the object whenever contact stops
  it short of the commanded target.
- **Cube mass and friction** already matched the working reference class of values before
  either was ever the active line of investigation.
- **Mimic-joint wiring, the physics engine, and the simulation step size** each matched the
  established controller contract at multiple points in this investigation; each match
  reproduced the same failure, ruling out the match itself as a sufficient fix.
- **MoveIt, IK, grasp-pose generation, and approach execution** can reach the object;
  pre-grasp candidate rejections before a later valid candidate are separate from the
  contact instability.
- **MoveIt's `attach()` does not physically carry the Gazebo object** — it only updates the
  planning scene. A real simulated pad contact has to hold the object through retreat.
- **One pad contacting the box asymmetrically** (versus something else near the descent
  path) was directly ruled out by watching the Gazebo viewport during a real `/pick` run:
  the arm never got within contact range of anything at the time.
- **Fingertip friction (`100000`, then `5`) as the fix for chatter** — the pads were
  15.1 mm from the cube in every run that claimed to test either value (Iteration 7);
  neither was actually exercised under real contact until the Iteration 12/13 passing runs.
- **Matching upstream `robotiq_description`'s follower-joint shape exactly** reproduced the
  identical failure signature (Iteration 5) and separately introduced an
  unconstrained-linkage regression that Iteration 7 had to correct.
- **Copying `arm_stack`'s `<ros2_control>` shape onto this stack** does not reproduce its
  result — see "Harmonic hands a mimic joint to two governors" above. `arm_stack` also
  never verifies its grasp at all; its `close_slowly()` treats any step timeout as contact
  and never cancels the last goal, so it isn't a valid target to copy for a project whose
  contract is verification.
- **Opening `grasp_offset_m` for more table clearance without new targeting evidence**
  (Iteration 10) made reachability worse, by a mechanism this RCA never explained; reverted
  and confirmed reproducible at the original value.
- **Tuning `stall_velocity_threshold` to any single value** cannot separate settled contact
  from engine noise — Iteration 6 measured spurious velocity spikes the same order of
  magnitude as genuine free-swing motion (`0.37 rad/s`); Iteration 13 confirmed this still
  holds under `dart`.

## Open

- **The `dart`-side periodic `+0.15 rad/s` velocity artifact** during sustained contact —
  measured, unexplained beyond "engine-side, tied to rigid contact." It no longer blocks
  `/pick` (verification no longer depends on the controller resolving it), but it would
  block any future consumer of this controller's own terminal result.
- **The SRDF's `gripper_close` named state** (`sorting_arm.srdf`) still specifies `0.8` — a
  MoveIt named state the skills close path has never used and still does not. Left as-is;
  not part of this RCA's scope.

## Reproduction

1. Launch the simulation (`sim.launch.xml gui:=true`, or `app.launch.xml` for the whole
   cell in one command — see `sorting_arm_bringup`).
2. Start `skill_server_node` with `use_sim_time:=true` and the installed `skills.yaml`
   (already true under `app.launch.xml`).
3. `/sync_objects`, then `/home`, then `/pick` for `red_box_1` with feedback enabled.

## Related

<<<<<<< Updated upstream
- `docs/project/decisions.md` D24 — the `dart` physics engine choice.
- `docs/rca/gripper-controller-configuration.md` — `GripperActionController`'s full
  declared parameter set, verified against the installed generated header.
=======
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

### Iteration 3: lower active fingertip friction

Both supported Gazebo fingertip friction blocks changed from `mu1=mu2=100000` to `mu1=mu2=5`.
This kept the valid friction configuration while matching the cube's friction coefficient.
`colcon build` passed; only the pre-existing upstream `tl_expected` deprecation warning appeared.

Koushik ran the standard GUI workflow twice. Both runs reached the object, completed descent, and
then aborted with `close_gripper: gripper result timed out`. The second run visibly continued
squeezing and shaking the cube after `/pick` had aborted. This change did not resolve the issue.

## Latest evidence

The pre-Iteration-3 simulation description contained the Iteration 2 values:

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

After the Iteration 3 source change, both manual picks timed out in `close_gripper` despite
`mu=5`. The fingers therefore still fail to remain below the configured stall velocity threshold
for a complete stall window, or the running controller does not have the expected parameter.
The supplied logs are from `skill_server_node`; they do not include controller velocity samples,
loaded controller parameters, or cancel processing, so they cannot decide which condition is
true.

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

## Root-cause status after Iteration 3

Extreme friction was a plausible contributor but is not sufficient root cause. Lowering both
fingertip coefficients to `5` left the same two close-action timeouts.

The remaining evidence supported two unseparated possibilities:

1. Contact kept the left-knuckle velocity above `stall_velocity_threshold: 0.02` for the full
   10-second client timeout.
2. The running gripper controller did not load the expected stall settings.

The client timeout path also sent `async_cancel_goal()` and returned without waiting for cancel
acknowledgement or terminal action result. That matched, but did not yet prove, Koushik's
observation that the gripper continued squeezing after `/pick` aborted.

## Iteration 4: use ROS time for the client result deadline

### Change

`GripperCommander::send_goal()` now measures `gripper.result_timeout_s` with the skill node's ROS
clock instead of one wall-clock `std::future::wait_for()`. Short future waits only let the worker
observe the ROS-time deadline. Logging was added for gripper goal send, acceptance, result,
timeout, and the Pick close phase. No controller parameter, physics, collision, or cancellation
behaviour changed.

`colcon build` passed. The only stderr was the existing upstream `tl_expected` deprecation
warning.

### Runtime results

Koushik ran four standard workflows with `use_sim_time: true`.

- One close returned `SUCCEEDED` with `stalled: true` after 8.25 ROS seconds at position
  `0.174928`, but the box was held weakly at the fingertip and fell during retreat.
- Three closes timed out after about 10.006 ROS seconds.
- The running controller dump confirms `allow_stalling: true`, `stall_timeout: 1.0`,
  `stall_velocity_threshold: 0.02`, and `use_sim_time: true`.
- Each timeout reached the controller's cancel callback immediately. The current client still
  does not wait for the cancellation acknowledgement or terminal action result, but an old close
  goal is not the explanation for these three observed timeouts.

The wall-time mismatch is therefore ruled out. Changing the client's clock did not make grasping
reliable: one run completed weakly and three runs still aborted in `close_gripper`.

### Collision configuration check

Static collision configuration is correct: the box collision, gripper collision meshes, and
fingertip friction blocks are enabled, with no collision mask excluding pad-to-box contact. This
does not identify the runtime collision pair. The 2.9 mm closed-pad/table clearance remains a
possible table contact and needs runtime evidence.

The saved left-knuckle trace is invalid for this purpose. It ends at simulation time `98.110`,
before the close phase, and contains only near-zero knuckle positions. It cannot show stall
velocity, the contact pair, or the observed one-sided close.

## Current root-cause status

The controller loaded the expected settings, so that earlier possibility is closed. The
remaining unresolved condition is physical contact behaviour:

1. Contact keeps the left-knuckle velocity above `stall_velocity_threshold: 0.02` for the full
   10-second ROS-time result window, or contact is with an unintended physical pair.

## Iteration 5 setup: match upstream `robotiq_description` exactly

An uncommitted working-tree edit to `robotiq_2f_85_macro.urdf.xacro`, present before this
session, reverts both fingertip-friction attempts from Iterations 2 and 3 and the joint-type
change made alongside them. Diffed against the real upstream package
(`/opt/ros/jazzy/share/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro`, the same package
`ur5-pick-and-place-ros2` depends on): the working tree now matches it exactly except for vendored
mesh paths. The four mimic-follower joints return to `type="continuous"` with no `<limit>`, and
the fingertip collision reverts to the inline `mu1=mu2=100000` `<surface>` block that Iteration 2
already showed is dropped during URDF-to-SDF conversion — confirmed again here by generating SDF
from the current file and finding no friction element on either fingertip collision. `grasp_offset_m:
-0.036` and every controller/timeout parameter from Iterations 1b through 4 are unchanged.

This is the one fingertip-friction configuration not yet tested: neither of our two invented
values (`100000`, then `5.0`), but the reference's own (inert, i.e. bare SDF default). `check_urdf`
passes; the description package is symlink-installed, so the change is already live. Not yet run.
Evidence to capture and the acceptance criteria are unchanged from the section below; see
`ai/debug/gripper-grasp.md` Iteration 5 for the exact capture commands.

### Gripper configuration audit against the reference

Every URDF-level gripper value (joint types, limits, mimic tags, fingertip friction, `ros2_control`
interface split, box/table geometry, physics engine, `update_rate`) now matches the working
reference exactly, confirmed against the real upstream `robotiq_description` package rather than
only the vendored copy under `ai/ref_repos`.

Three `gripper_controller` values remain intentionally different from the reference:
`allow_stalling: true` vs. its `false`, `stall_velocity_threshold: 0.02` vs. its `0.001` default,
and our `close_position: 0.8` vs. its `0.95`. The reference never reads the action result and
never needs the controller to resolve the goal at all, so its values are not a valid target to
copy; ours exist because this project's `close_gripper` phase reads `stalled` as the grasp
verdict. Closing this gap would regress grasp verification, not the physical grasp, so it stays
open by design.

**Status:** superseded by Iteration 6 below — the run happened and did not resolve the issue, but
it produced the evidence this RCA had been missing since Iteration 3.

## Iteration 6: the required trace, and a new root cause

Koushik ran Iteration 5's configuration with a `ros2 bag record` of `/joint_states` and `/rosout`
spanning the whole `/pick` call, plus the skill node's and simulation's own logs. `/pick` aborted
the same way as Iterations 3 and 4: `close_gripper: gripper result timed out`. Matching the
reference's actual fingertip friction and joint types did not fix the grasp.

The bag, extracted against `robotiq_85_left_knuckle_joint`, shows why. The close goal was accepted
at `1785543358.665` (ROS time); the client's 10-second deadline expired and cancelled it at
`1785543368.671`, matching both the skill node's and the controller's own logged timestamps.
Within that window:

- The first ~1.7 seconds are free swing and initial contact: velocity reaches 0.37 rad/s
  approaching the object, then rings between +0.25 and −0.32 rad/s as the pad makes and rebounds
  off first contact — ordinary impact behaviour.
- From roughly 360.8s to 365.8s, the joint's *position* is essentially frozen — it holds within a
  0.003 rad band, consistent with the pad genuinely at rest against the object. Its *reported
  velocity*, however, does not agree: it spikes to 0.05–0.37 rad/s roughly every 0.1–0.3 seconds
  throughout this window, with no corresponding position change. Checked directly by computing
  finite-difference velocity from the position samples themselves: at one representative sample
  the controller sees a reported velocity of +0.231 rad/s while the position moved by
  approximately 0.00002 rad over the surrounding ~80ms — a two-thousand-fold discrepancy. The same
  pattern recurs roughly sixty times across the five-second window. The velocity *state interface*
  for this joint is producing spurious readings uncorrelated with the joint's actual motion.
- Every one of those spurious spikes exceeds `stall_velocity_threshold: 0.02` and resets
  `GripperActionController::check_for_success()`'s internal movement timer, restarting the
  required 1.0-second quiet window from zero. The longest continuous run below threshold anywhere
  in the ten-second attempt was 0.82 seconds, broken 0.18 seconds short of the requirement it
  needed.
- At the moment the client's timeout cancelled the goal (`368.671`), velocity swings to
  approximately −0.25 rad/s within roughly 100ms and position drops from 0.185 to 0.155 rad before
  chattering unpredictably. This matches `async_cancel_goal()` invoking the controller's
  `set_hold_position()` mid-contact — a mechanism Iteration 1 identified as a plausible side effect
  from reading the controller's source, now directly confirmed in a trace, timestamp-matched to the
  cancellation.

The velocity readback was traced to `gz_ros2_control::GazeboSimSystem::read()`, which copies
`gz-sim`'s `JointVelocity` entity-component-system value for the joint with no filtering. The noise
originates in what `gz-sim`'s physics system populates that component with from
`bullet_featherstone` each step, not in this project's code or in `gz_ros2_control` itself.

This closes the friction and joint-geometry line of investigation: Iteration 5 matched the working
reference exactly on both axes and produced the same failure signature, so the defect is downstream
of contact, in what the controller is told the joint's velocity is rather than in what causes or
strengthens contact.

It also rules out further `stall_velocity_threshold` tuning by evidence rather than assumption: the
spurious spikes (up to 0.37 rad/s) are the same order of magnitude as genuine motion measured in
the same trace (free swing measured 0.37 rad/s; real continued closing measured 0.01–0.05 rad/s).
No single threshold can separate a settled joint from a noisy reading without also masking real
motion.

## Current root-cause status

Two independent, evidence-confirmed defects remain, neither fixed yet:

1. `GripperActionController`'s stall detection depends on a `JointVelocity` state interface that
   `gz-sim`/`bullet_featherstone` populates with spurious high-magnitude spikes while the joint is
   physically at rest. This makes the controller's 1.0-second quiet-window requirement a low-odds
   race against noise rather than a reliable detector of a settled grasp, which explains the
   run-to-run inconsistency observed since Iteration 1.
2. `GripperCommander::send_goal()`'s timeout path calls `async_cancel_goal()` without qualification,
   which drives the controller into `set_hold_position()` regardless of how close the joint was to
   a genuine stall. Iteration 6's trace shows this actively destabilizing an already-mostly-settled
   contact at the exact cancellation timestamp.

## Iteration 7: the jaws never reached the cube

Decoding what the Iteration 4 and Iteration 6 "successful" stalls meant physically, rather than
only in the controller's own terms, found a defect upstream of everything investigated so far.

### The measurement

`GripperCommander::close()` reads `result.position`, the `robotiq_85_left_knuckle_joint` angle at
the controller's declared stall. The two stalls on record were `0.174928` rad (Iteration 4) and
`0.176945` rad (a same-day rerun of the pre-Iteration-7 configuration). The macro's own link
origins and the fingertip pad's collision-mesh bounds give a closed-form jaw gap:

```
gap(th) = 2 * (0.03060114 + 0.0371575*cos(th) - 0.04342168*sin(th) - 0.02526)
```

This checks against the gripper's own published spec: `gap(0) = 85.0 mm` (rated open) and
`gap(0.8) = 0.1 mm` (fully closed). At the recorded stall angles, `gap(0.176945) = 68.6 mm`. The
cube is 40 mm. An independent per-vertex distance check between every gripper collision mesh and
the cube box confirms it: at the angle the joint parks, the nearest gripper surface to the cube is
15.1 mm away, and the lowest point of the gripper sits 9.3 mm above the table. No part of the
gripper had touched the cube or the table in any run this RCA has on record. Every `stalled: true`
verdict since Iteration 1 was a stall on air, and the friction, grasp-depth, and threshold changes
in Iterations 2–6 were each tuning a contact that never happened.

### The mechanism

The commit immediately before this iteration (`8e76832`, matching upstream `robotiq_description`
exactly) changed the four mimic-follower joints (`left_inner_knuckle`, `right_inner_knuckle`,
`left_finger_tip`, `right_finger_tip`) from `type="revolute"` with `effort="50" velocity="0.5"`
limits to `type="continuous"` with no `<limit>` element. The generated SDF confirmed the
consequence directly: four of the gripper's six joints reached `bullet_featherstone` with no
effort limit, no velocity limit, and infinite travel. A linkage built from unconstrained follower
joints has no torque authority to hold itself rigid against contact or its own dynamics.

Re-extracting the Iteration 6 bag for all six gripper joints (prior iterations examined only the
driven knuckle) shows this directly. Mimic *tracking* is not broken — error stays under roughly
0.02–0.05 rad through the whole close window. But during the window the followers hold a
steady-state offset from the mimic law with a consistent sign: the inner knuckles run slightly
over-closed, the fingertips run under-closed by a larger margin. A static offset under constant
drive is a force imbalance, not tracking lag — lag decays once motion stops, this does not. This
is the physical form of the asymmetric closing behaviour observed by eye during these runs.

Two standing assumptions are corrected by this iteration. `max_effort` cannot be raised regardless
of target geometry: `position_controllers/GripperActionController` on a position command interface
discards it entirely (`move_group` logs "will command a max effort of: 0" on every run), so the
squeeze force is standing position error against the joint's own 50 N·m limit, already at its
achievable maximum whenever contact stops the jaw short of the commanded target. Separately, the
close phase was measured running at a real-time factor near 0.09 — one same-day run spent 111.8
seconds of wall clock inside a 10-second ROS deadline that never fired, while the arm's own motions
in the same log ran near real time. Physics specifically thrashes while the gripper is commanded,
consistent with an unconstrained four-bar linkage recomputing contact every step.

### The fix

The four follower joints are restored to `revolute` with the same limits this repository had
before `8e76832`, which are also `ai/ref_repos/arm_stack`'s values for the same joints — the one
reference repo whose own grasp is visibly solid. Every `<mimic>` tag is unchanged. The two inline
`<surface>` blocks nested in the fingertip `<collision>` elements — confirmed again this iteration
to be silently dropped during URDF→SDF conversion — are replaced with the supported
`<gazebo reference="...finger_tip_link"><mu1>5.0</mu1><mu2>5.0</mu2></gazebo>` form. This is pad
friction's first real test in this RCA: the Iteration 3 run that appeared to test `mu=5` tested
nothing, because the pads were never within 15 mm of the cube.

The box (`red_box_1`, `blue_box_1`) changes from a 40 mm cube to `0.04 x 0.04 x 0.06` m, matching
`arm_stack`'s proven geometry, with mass unchanged at 0.05 kg and inertia corrected to the true
`m(a²+b²)/12` values (the previous `1.5e-5` on all three axes was wrong for any box shape). The
taller box buys two things at once: more pad face for contact, and more clearance between the
closed pad tip and the table — the 40 mm cube left only 2.9 mm of table clearance at grip depth,
a plausible second failure mode once the linkage could finally close. `grasp_offset_m` is
re-derived for the new height (`-0.036` → `-0.048`).

`GripperCommander::send_goal()` gained a feedback callback logging jaw gap, ROS-elapsed time, and
wall-elapsed time throughout the close, plus the same figures on the terminal result and on a
timeout. This makes both the physical jaw gap and the real-time-factor collapse visible from the
skill node's own log without needing a bag.

One option evaluated and rejected: `arm_stack` also gives all six gripper joints, including the
mimic followers, a command interface in `ros2_control`. That shape is not legal under this
project's ROS 2 Jazzy — `hardware_interface` enforces "Activated mimic joints cannot have command
interfaces" (confirmed directly in the installed library). `arm_stack` runs Humble, where that
check does not exist. The followers stay state-interface-only; the joint-limit correction is the
portable part of `arm_stack`'s shape and is what this iteration ships.

**Build status:** `colcon build --symlink-install` on the three affected packages — clean, no
warnings, no errors. `check_urdf` on the regenerated model passes with root link `world`.
`gz sdf -p` on both the robot and the world confirms every joint limit and the fingertip friction
values reach the generated model as intended.

**Runtime result:** worse, not better. `close_gripper` never resolved — canceled at the 10-second
ROS deadline (32.62 s wall). Koushik observed continuous shaking, not a settle-then-stall. The
jaw-gap logging did its job even in failure: the recorded left-knuckle trace never gets past
`0.31` rad (55 mm jaw) against a 40 mm box, so this was not a near-miss.

The bag Koushik recorded across the close window, extracted for all six gripper joints, explains
why. Mimic tracking error — under ~0.02–0.05 rad throughout the Iteration 6 (pre-fix) trace — now
reaches 0.149 rad and is still growing at the end of the window, not settling. `gz_ros2_control`
runs its own internal mimic-joint drive independent of anything declared in this project's
`<ros2_control>` XML — confirmed by a startup log line present in every run
(`gz_ros_control: Joint '...' is mimicking joint '...'`) and by the plugin's own source
(`gz_system.cpp:342-367` in the vendored `gz_ros2_control` copy, read to understand the plugin's
mechanics, not as a design reference): it loops over every joint declared inside `<ros2_control>`,
auto-detects the URDF-level `<mimic>` relation via `hardware_interface`'s own parser, and drives
that joint's command from the mimicked joint's state every control cycle. This runs in addition to
`bullet_featherstone`'s native SDF `<mimic>` constraint, which sdformat generates automatically
from the same URDF tag. Before this iteration, the follower joints had no effort limit, so the
native physics constraint could satisfy itself without resistance regardless of what
`gz_ros2_control` commanded. Restoring real effort gave both enforcers genuine authority, and they
now fight instead of agreeing — the bag's unbounded, growing divergence is that fight, not contact
chatter (nothing in the trace correlates with proximity to the box; the jaw never gets within 15 mm
of contact range).

One planned instrument failed to produce data and is now understood, not just absent:
`position_controllers/GripperActionController` never publishes action feedback during a goal —
confirmed by inspecting the installed controller implementation directly, zero matches for any
feedback-publishing call. The `feedback_callback` hook this iteration added to
`GripperCommander::send_goal()` is structurally unable to fire on this controller. The
terminal-result and timeout logging (jaw gap, both clocks) are unaffected and were how this
iteration's failure was read without needing the bag first.

## Iteration 8: stop the second mimic enforcer, and why `arm_stack` never had one

Koushik asked directly why `ai/ref_repos/arm_stack` grasps cleanly with the same UR5e and the same
Robotiq 2F-85 while this project has not. The answer is not the gripper model, friction, grasp
depth, or controller tuning — all searched and closed in Iterations 2–7. It is the simulator
generation each project runs against.

`arm_stack` is ROS 2 Humble, Gazebo Fortress, `ign_ros2_control` (its own `README.md`,
`SETUP.md`). Fortress-era sdformat does not convert a URDF `<mimic>` tag into an SDF `<axis>`
constraint at all, so `arm_stack`'s follower joints reach physics with exactly one enforcer:
`ign_ros2_control`'s own control-cycle velocity drive. Its `2f_85.ros2_control.xacro` lists all
five followers under `sim_ignition` deliberately without command interfaces — mimic parameters
only, one governor, no conflict.

This project is ROS 2 Jazzy, Gazebo Harmonic, `gz_ros2_control`. Generating this project's own
URDF through `gz sdf -p` (confirmed this iteration) shows sdformat 14 *does* convert the tag: all
five followers carry a real `<axis><mimic>` block in the generated SDF, and
`bullet_featherstone`'s installed plugin exports `JointFeatures::SetJointMimicConstraint` — the
constraint is solved inside the articulated-body solver every 1 ms physics step. Every joint this
project ever listed in `<ros2_control>`, even state-only, was picked up by
`gz_ros2_control`'s own auto-detection of that same URDF tag and given a second, independent
velocity drive on top. Copying `arm_stack`'s `<ros2_control>` shape onto this stack does not
reproduce its result, because Harmonic hands the same joint to two governors where Fortress hands
it to one.

The two governors are not evenly matched. `gz_ros2_control`'s mimic drive (confirmed against the
installed `/opt/ros/jazzy/lib/libgz_hardware_plugins.so`, whose only mimic string is `'is
mimicking joint '` — the exact line in every launch log) writes
`velocity_sp = -(position_follower - position_driven * multiplier) * update_rate` every control
cycle. At `update_rate: 100`, that is a velocity loop of gain 100. The driven knuckle's own
position command interface runs `target_vel = -position_proportional_gain * error * update_rate`;
`position_proportional_gain` is a node parameter this project never overrides
(`grep -rn position_proportional_gain src/` is empty), so the plugin default `0.1` applies and the
driven joint's own loop runs at gain 10. The followers were driven ten times stiffer than the
joint they are supposed to track, at 100 Hz, on top of a 1000 Hz solver-level constraint doing the
same job. Before Iteration 7 the followers had no effort limit, so the physics constraint mostly
yielded to the stiffer velocity drive for free — the residual disagreement was still enough to
drag the driven knuckle under `stall_velocity_threshold` and produce every false "successful" stall
this RCA has on record. Iteration 7 gave the followers real torque authority, so neither enforcer
yields anymore, and the two fight — the growing, unbounded mimic divergence and visible shake
recorded at the end of Iteration 7 is that fight, not contact.

### The fix

`sorting_arm.gazebo_ros2_control.xacro`'s `<ros2_control>` block now lists only the six arm joints
and the driven `robotiq_85_left_knuckle_joint`. The five follower `<joint>` blocks are removed
entirely, not just their interfaces — evidence from this project's own prior state ruled out the
lighter option: the followers were already state-interface-only before this iteration, and
`gz_ros2_control` still logged all five `is mimicking` lines and still drove them, because its
mimic list is built from the URDF tag regardless of whether the joint is actuated. Only absence
from `<ros2_control>` removes a joint from the hardware plugin's mimic list. The URDF `<mimic>`
tags in `robotiq_2f_85_macro.urdf.xacro` and the resulting SDF constraint are untouched — the
constraint remains, `gz_ros2_control`'s competing drive does not.

`GripperCommander::send_goal()`'s feedback-callback hook and its throttled logging, confirmed dead
in Iteration 7 (`GripperActionController` never publishes action feedback), are removed. The
terminal-result and timeout logging, including jaw gap, are unchanged and remain the primary
runtime signal.

**Build status:** `colcon build --symlink-install --packages-select sorting_arm_skills
sorting_arm_description sorting_arm_bringup` — clean, no warnings, no errors. `check_urdf` on the
regenerated model passes, root link `world`. `gz sdf -p` confirms all five `<mimic>` blocks remain
in the generated SDF and the regenerated `<ros2_control>` block carries exactly seven joints (six
arm, one gripper) — the five followers are gone from it.

**Runtime result — Koushik's run:** the double-mimic fix itself worked. `close_gripper` no longer
times out or shakes; the goal completed cleanly and `/pick` returned `ABORTED` at `verify_grasp`
with `reached close_position uncontested: no object captured` — the controller closed the jaw all
the way to `close_position` without ever stalling on the object. This is a different, and better,
failure mode than every prior iteration: the actuation problem this iteration targeted is gone.
What's left is a targeting problem, addressed in Iteration 9 below.

## Iteration 9: the grasp target didn't match the box it was aimed at

Descent visibly plunged far deeper than intended, and the jaw closed on nothing. `grasp_pose`
computes `tcp_z = object_centre_z + half_height_m + grasp_offset_m`; `grasp_offset_m: -0.048` was
derived in Iteration 7 specifically for the 0.06 m tall `red_box_1`/`blue_box_1` geometry that
iteration introduced. `half_height_m` comes from whatever was last synced into the planning scene
for that object id via `/sync_objects`, not read live from the world file — if the synced
dimension and the box actually spawned in `sorting_cell.sdf` disagree, `grasp_pose` targets a
height that belongs to neither, which is consistent with both symptoms observed: a target close
enough to the table to look like an overly deep descent, and a target far enough from the box's
true centre to close on nothing.

**Fix:** full revert of Iteration 7's box-geometry experiment, per Koushik's direction to make
`red_box_1`/`blue_box_1` match `blue_box_2`/`red_box_2` exactly. `sorting_cell.sdf`: both boxes
back to `0.04 x 0.04 x 0.04`, pose z `0.530` → `0.520`, inertia back to the correct uniform-cube
value `1.5e-5` on all three axes — byte-for-byte identical to `blue_box_2`/`red_box_2`, confirmed
with `git diff HEAD` showing an empty diff on this file after the edit. `grasp_offset_m` reverts
with it, `-0.048` → `-0.036` in `skills.yaml`, `pick_server.cpp`, and `place_server.cpp` — the
value Iteration 4 derived and documented for exactly this 40 mm geometry (37 mm of the 40 mm face
between the pads). Iteration 7's other changes (follower joint limits, fingertip friction via
`<gazebo reference="...">`, the mimic fix above) are unaffected; this reverts geometry only.

At the reverted geometry, `tcp_z = 0.520 + 0.02 - 0.036 = 0.504`, 4 mm above the table surface
(`0.500`). This is the same close table clearance Iteration 6 flagged as a risk before Iteration 7
tried to fix it by making the box taller — that risk was never actually the failure mechanism (the
dimension mismatch was), so it returns unaddressed here. If retreat or descent now clips the table
at this clearance, that is the next thing to fix, not a sign this revert was wrong.

**Build status:** `colcon build --symlink-install --packages-select sorting_arm_skills
sorting_arm_description sorting_arm_bringup` — clean, no warnings, no errors.

**Runtime result — Koushik's run:** pre-grasp and descent both succeeded, matching the analysis
above. `close_gripper` timed out at 10 s with no stall and no completed result — a different
failure signature from Iteration 8's clean "uncontested" abort. No jaw-gap number is available (it
is only logged on a completed goal); the timeout-without-stall pattern matches the pad/table
chatter `skills.yaml`'s own `allow_stalling` comment describes, and is consistent with the ~2 mm
pad-tip/table clearance computed above — the risk this iteration knowingly left unaddressed.

## Iteration 10 (reverted): grasp_offset_m cannot simply be opened up for clearance

**Attempted fix:** `grasp_offset_m: -0.036` → `-0.028`, recomputed from the same pad-face model
Iteration 7 derived (pad face ≈ 57 mm, pad bottom ≈ 2 mm below tcp): at `-0.028`, tcp = 0.512,
pad bottom 10 mm above the table (the margin Iteration 7 called safe), face coverage drops from
38 mm to 30 mm of the 40 mm face — still substantial on paper.

**Runtime result:** worse, not better. `/pick` never reached `descend` — all 5 pre-grasp
candidates failed, split across two different failure modes (OMPL/RRTConnect timing out at the
5 s planner limit on some, the Cartesian descent preflight failing its coverage-fraction check on
others). Moving the target 8 mm *further* from the table was expected to ease reachability, not
break it, so the mechanism is not understood from this evidence alone.

**Reverted.** `grasp_offset_m` back to `-0.036` in all three files; `git diff HEAD` on
`pick_server.cpp` and `place_server.cpp` is empty, confirming an exact revert. Clean rebuild.

Per this project's rule against guesses or clamps that hide a cause, this offset is not to be
retried at another value blind. The next step needs either a rejected candidate's actual target
pose and IK failure reason from the `move_group` log, or a planning-scene dump, before touching
`grasp_offset_m` again. The likelier root cause for the original chatter, not yet investigated: a
disagreement between MoveIt's planning-scene collision margins and Gazebo's contact model — the
Cartesian descent preflight at `-0.036` reported the path collision-free, and the physical chatter
says it was not. That gap, not the offset value, is what this RCA needs next.

Re-run after the Iteration 10 revert, same session: `-0.036` reproduces exactly the Iteration 9
result — pre-grasp and descent both succeed, `close_gripper` times out at 10.01 s wall with no
stall, no result. Confirms Iteration 9's failure is real and repeatable, and that the Iteration 10
detour changed nothing about it either way. This RCA is closed for this session at that state: the
double-mimic fix (8) and the box-geometry match (9) are both correct and runtime-confirmed; the
remaining, unsolved problem is the table-clearance chatter on `close_gripper`, diagnosis proposed
above (MoveIt/Gazebo collision-margin disagreement) but not yet investigated.

## Required evidence before closing this RCA

1. The five `Joint '...' is mimicking joint '...'` launch-log lines must be gone — direct proof
   the second enforcer is off. No `Mimic joint '...' not found in <ros2_control> tag` exception on
   load. **Confirmed this session** — Iteration 8's runtime result showed no shaking and no
   timeout, consistent with a single mimic enforcer.
2. `ros2 control list_hardware_interfaces` — only the driven knuckle carries a gripper command
   interface; the five followers no longer appear.
3. `/joint_states` and TF — the followers drop out of `/joint_states` (`joint_state_broadcaster`
   sources from hardware state interfaces), but `robot_state_publisher` computes mimic-joint
   transforms from the URDF `<mimic>` relation and the driving joint's own position
   (`MimicMap`, `robot_state_publisher.hpp:166`), so TF and the fingertip frames must still move.
   If they do not, this assumption is wrong and needs revising, not patching.
4. Descent no longer looks visibly too deep, and `close_gripper` resolves with a jaw gap in the
   neighbourhood of 40 mm, not 55–70 mm and not a table strike.
5. descent reaches the object and the fingers close without sustained visible shaking;
6. `close_gripper` resolves near the one-second stall window, not the ten-second client timeout;
7. the cube remains between the pads throughout retreat; and
8. `/pick` returns `ok: true`.

If the jaw gap closes correctly but the object still slips free, that is the first genuine
friction/grip-force observation this RCA will have made, and the two lines of investigation closed
after Iteration 6 (fingertip friction, `JointVelocity` noise as a verification signal) may need to
reopen on that new footing rather than be treated as settled. If the 4 mm table clearance turns out
to be too tight, `grasp_offset_m` is the lever, and Iteration 4's derivation in this file is the
reference for re-deriving it.

## Iteration 11: controller completion is not a usable contact verdict

The repeated ten-second close timeout is not evidence that the pads reach the table. It is the
expected result of asking `position_controllers/GripperActionController` to finish a goal that is
blocked by a held object.

For a target of 0.8 rad, a 40 mm cube stops the driven knuckle near 0.448 rad. The controller can
finish only when the position error is below its 0.01 rad tolerance, or when reported velocity stays
below its stall threshold for the full stall timeout. Neither condition held in the Iteration 6 bag:
the position remained essentially constant while reported velocity repeatedly rose to 0.231 rad/s;
the longest quiet interval was 0.82 s. The controller therefore produces no terminal result, and the
client's ten-second ROS-time deadline aborts the pick.

This also explains the visible shaking. `gz_ros2_control` turns a position command into a velocity
setpoint. With the default proportional gain of 0.1 and update rate of 100 Hz, the blocked 0.352 rad
error asks for about 3.5 rad/s even though the driven joint is rated for 0.5 rad/s. Changing friction,
box dimensions, or grasp depth cannot make that completion condition true.

The geometry was re-derived from the fingertip collision mesh. At `grasp_offset_m: -0.036`, the pads
cover about 36.2 mm of the cube's 40 mm face. Their lower edge remains above the table: 14.7 mm with
the jaw open, 3.8 mm at cube contact, and 1.2 mm fully closed. The grasp offset, cube dimensions,
fingertip friction, and single-mimic configuration are not changed by this iteration.

Cancellation was a separate release mechanism. The controller's `cancel_callback` calls
`set_hold_position()`, replacing the squeeze target with the current position and removing position
error. The old timeout path therefore released the object before retreat. Sending a new goal instead
preempts the previous goal without that hold-position rewrite.

### Change

`GripperCommander::close()` now uses five phases:

1. It approaches the expected object width plus a 10 mm margin. This free-space goal must complete.
2. It sends 0.05 rad ladder steps. Each target is based on the last measured controller position, so
   the initial position error is never more than 0.05 rad. At the configured gain and update rate,
   that bounds the requested velocity to the joint's 0.5 rad/s rating.
3. A step with no terminal result in 0.8 ROS seconds is contact. The next squeeze goal preempts that
   step; it is left active and is never cancelled while the object is held.
4. After a 0.5 ROS-second settle period, `/joint_states` supplies the left-knuckle angle. The existing
   `jaw_gap_m()` converts it to a physical gap, and capture requires that gap to be within 6 mm of the
   object `BOX_X` width supplied by `SceneManager`.
5. If a step reaches `close_position` normally, the jaw closed on air and the pick reports no object.

The inverse `knuckle_angle_for_gap_m()` is a closed-form inverse of the existing jaw-gap equation.
Invalid widths and out-of-range angles fail instead of being clamped. The old
`stall_velocity_threshold: 0.02` override is removed; capture no longer uses stall velocity as its
verdict, while `allow_stalling: true` remains so an unexpectedly blocked approach returns its native
result.

### Verification still required

This iteration has compilation and static-validation evidence only until Koushik runs the cell. The
runtime acceptance evidence is: a completed approach; falling gap logs; one ladder timeout at a
40–48 mm gap; a measured gap within 6 mm of 40 mm; no sustained shaking; the cube held through
retreat; and `/pick` returning `ok: true`.

### Runtime result: approach did not reach its free-space target

Koushik ran the Iteration 11 build on 2026-08-01. All three controllers were active. `SyncObjects`
accepted `red_box_1` at `(0.40, 0.12, 0.520)` with `BOX` dimensions `(0.04, 0.04, 0.04)`, and
`/home`, pre-grasp, and descent succeeded. The new close path did start: it logged an approach target
of 50.0 mm at 0.357 rad. It never reached that target, so `send_goal()` correctly timed out after
10.000 ROS seconds and cancelled the free-space approach. No ladder step, squeeze, or measured-gap
capture verdict ran.

The recorded bag contains 1,000 left-knuckle samples during that ten-second approach. Its position
started at 0 rad, peaked at 0.287364 rad, and ended at 0.238795 rad. The existing `jaw_gap_m()`
conversion gives a smallest observed gap of 57.336 mm, not the 49.963 mm requested by the 0.357 rad
goal. Since the cube is 40 mm wide, at least 17.336 mm of total jaw gap remained. Koushik also
observed clear gaps between both fingertip pads and the box, so this was not object contact.

This invalidates Iteration 11's first contract: the 50 mm approach is not completing in free space.
The new code has exposed that failure without treating it as capture or entering the ladder. The
remaining cause is unsolved. Do not retune grasp geometry, friction, or ladder parameters from this
run; next investigation must explain why the driven knuckle oscillates below an unobstructed command.

If those checks pass except for the retreat hold, friction and grip force become an open investigation
again. Until then, do not retune them: this iteration changes the verdict and cancellation mechanisms
that prevented a valid grasp from reaching retreat.

### Follow-up: the free-space approach broke the velocity bound

The Iteration 11 ladder bounded each later target change to 0.05 rad, but the first approach still
sent one direct command from the measured 0 rad open position to 0.357 rad. That command violated
the same velocity invariant the ladder was meant to enforce. Jazzy `gz_ros2_control` converts a
position error to a velocity command using the configured proportional gain and controller update
rate. With the installed default gain of 0.1 and this project's 100 Hz update rate, the first command
requested `0.1 * 0.357 * 100 = 3.57 rad/s` from a joint limited to 0.5 rad/s. Even at the recorded
0.287 rad peak, the remaining error requested about 0.70 rad/s. The ten-second timeout was therefore
downstream evidence, not the first failure.

`GripperCommander::close()` now starts from a fresh measured knuckle position and uses at most
`close_step_rad` for the whole approach. A `must_reach` step is accepted only when the action result
is `SUCCEEDED`, `reached_goal=true`, and `stalled=false`. Its active timeout goal is cancelled and the
approach fails. A `stalled=true` result also fails, but it is already terminal in Jazzy; after a
terminal result, `rclcpp_action` removes the goal handle and cannot cancel it. After the 50 mm target
is reached, `contact_allowed` steps retain the Iteration 11 behavior: no result before
`step_timeout_s` means contact, that goal remains active, and the bounded squeeze goal preempts it.
Every next target uses the previous controller result position. Final approach acceptance uses
`reached_goal`, not exact equality between measured floating-point positions.

Each approach and close step now logs measured start, target, delta, calculated target gap, and the
terminal `stalled`/`reached_goal` fields when a result exists. Geometry, friction, mimic wiring,
controller type, gain, timeouts, squeeze size, and measured-gap capture verdict are unchanged.

Runtime behavior remains unverified. Koushik's first run of this follow-up never entered
`GripperCommander::close()`. `open_gripper` succeeded, candidate 1 found a pre-grasp plan but its
Cartesian descent covered only 96% against the required 99%, and candidates 2–5 each reached the
existing five-second OMPL timeout. `/pick` aborted at `descend_preflight`; no `close_gripper started`
or `must_reach` log exists. This is the same pre-grasp failure class recorded in Iteration 10, before
the bounded-approach change. The gripper change therefore has no runtime verdict from this attempt.

Diagnosis-only logging now records both target poses, each planned candidate's terminal arm joint
positions, exact Cartesian fraction and point count, native MoveIt codes, and every candidate outcome
in the final action detail. It changes no planner, target, collision, timeout, or gripper behavior.
The saved `skills.log` and `sim.log` under `ai/debug/iteration_12/runtime` contain the evidence above;
`runtime.log` is empty because the action command and its `tee` pipeline were entered as separate
shell commands, so it is not evidence.

A second run passed the complete descent preflight with candidate 2 and reached `close_gripper`, but
the first approach step still did not run. The new code rejected the fresh knuckle sample because it
required `position >= open_position` exactly. The open action had succeeded at 0.000060 rad; a live
`/joint_states` sample immediately after the failure reported `-2.480390615e-10 rad`. That tiny
negative value is Gazebo boundary residue around the zero-radian lower limit, not an unsafe gripper
state. The strict check was added by this follow-up and was not part of the accepted contract.

Measured positions are now accepted down to `open_position - close_step_rad`, the lowest state from
which one bounded step still produces a legal target at or above `open_position`. The measured value
is neither clamped nor replaced, so the next target remains based on the real result position and its
delta remains at most `close_step_rad`. Non-finite values, values more than one step below open, and
values above `close_position` still fail with the measured value in the error detail.

The next runtime run still expects approach targets near 0.05, 0.10, and later 0.05 rad increments
through 0.357 rad; every logged delta at or below 0.05 rad; every `must_reach` result reporting
`reached_goal=true` and `stalled=false`; contact only after the 50 mm approach; a settled gap within
6 mm of the 40 mm cube; no sustained shaking; the cube held through retreat; and `/pick` returning
`ok: true`. If a bounded `must_reach` step still fails before contact, stop this slice. The next
diagnosis is to restore follower state interfaces with `mimic="false"` so true linkage motion can be
observed without restoring `gz_ros2_control`'s second mimic drive.

### Bounded runtime result: free-space linkage failure remains

The next run reached the bounded approach and confirmed its command invariant. Every requested delta
was exactly 0.05 rad. Four steps returned `SUCCEEDED`, `reached_goal=true`, and `stalled=false` at
measured positions 0.040735, 0.081096, 0.121614, and 0.161944 rad. Each result remained roughly
0.0093–0.0097 rad below its target, which is inside the installed gripper controller's default
0.01 rad `goal_tolerance`. The fifth target was 0.211944 rad, a calculated 65.1 mm jaw gap, but it
did not return before the 0.8 ROS-second free-space deadline. The skill cancelled that active goal
and failed as designed.

This is before the 50 mm approach boundary and leaves 25.1 mm more total gap than the 40 mm cube.
The screenshot also shows both pads clear of the cube. Contact cannot explain the failure. The
bounded command fixed the oversized first request, but it did not fix the underlying free-space
linkage motion. This triggers the recorded stop condition: do not raise the timeout, retry silently,
or retune geometry, friction, gain, or step size from this result.

The five native-mimic follower joints are now restored to the Gazebo `<ros2_control>` resource as
state-only joints with the joint-level attribute `mimic="false"`. Installed `ros2_control` 4.45.2
uses that attribute to exclude them from `HardwareInfo::mimic_joints`. Installed
`gz_ros2_control` 1.2.19 therefore reads their real Gazebo position and velocity but does not run
its separate mimic velocity-command loop. They have no command interfaces, so they are passive to
`gz_ros2_control`; Harmonic's native SDF mimic constraint remains the only linkage governor.

This is diagnostic observability, not a claimed grasp fix. The next runtime evidence must contain
all six gripper joint positions during the same bounded approach. Compare each follower against its
URDF mimic relation and locate the first divergence or velocity-limit event at the fifth step. The
launch log must not contain any `Joint '...' is mimicking joint '...'` line for the five followers.
Runtime status remains unresolved until that trace is captured.

### Final runtime record: investigation stopped

Koushik requested that this investigation stop without another fix or diagnosis. The repository is
left with the bounded approach and the diagnostic follower state interfaces described above. This
RCA remains unresolved; `/pick` does not succeed.

The diagnostic wiring was confirmed at runtime before `/sync_objects`, `/home`, and `/pick`. All
three controllers were active. The driven `robotiq_85_left_knuckle_joint` was the only gripper joint
with a command interface. Position and velocity state interfaces were present for the driven joint
and all five followers. One idle `/joint_states` sample reported the driven joint at
`0.000070364 rad`; follower positions were between `-0.000070058` and `0.000068665 rad`, with signs
matching their URDF mimic multipliers around the open position. Follower effort entries were `NaN`
because no follower effort interfaces were declared.

Both simulator launches loaded all six gripper joints and logged no `is mimicking joint` line. This
confirms that `mimic="false"` kept the five followers out of `gz_ros2_control`'s second mimic loop
while their physical states remained observable.

Two recorded `/pick` attempts reproduced the same failure on different arm candidates. In the first,
candidate 2 passed a 100% Cartesian descent preflight. In the second, candidate 1 passed the same
check. Both executed pre-grasp and descent, completed four bounded 0.05 rad approach steps, then
timed out and cancelled the fifth step at a calculated 65.1 mm gap. Both actions aborted in
`close_gripper` with `free-space approach step timed out before reaching its target`.

Evidence is stored under `ai/debug/iteration_13/runtime`: `skills.log`, `skills2.log`, `sim.log`,
`sim3.log`, `runtime.log`, and two MCAP bags named `joints` and `joints2`. The bags contain 22,236
and 20,501 `/joint_states` messages respectively, together with `/clock`. No further interpretation
or change was made after Koushik's stop request.
>>>>>>> Stashed changes
