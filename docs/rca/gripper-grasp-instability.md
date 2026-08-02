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

- `docs/project/decisions.md` D24 — the `dart` physics engine choice.
- `docs/rca/gripper-controller-configuration.md` — `GripperActionController`'s full
  declared parameter set, verified against the installed generated header.

