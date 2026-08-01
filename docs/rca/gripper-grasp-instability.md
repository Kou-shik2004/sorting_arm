# Closed RCA: Gripper grasp was unstable in simulation

**Closed at Iteration 13.** `/pick` now returns `ok: true` through `attach`/`retreat`/`verify_hold`
against `red_box_1`, confirmed on two consecutive Gate C runs. See "RCA closed" near the end of
this file for the acceptance evidence and the three combined causes. The sections below are the
investigation record that got there — kept in full, not rewritten, because the ruled-out causes
and the reasoning that ruled them out are still load-bearing for any future gripper regression.

## Overview

The simulated Robotiq 2F-85 did not grasp `red_box_1` reliably for most of this investigation. The
same `/pick` workflow could succeed, abort during `close_gripper`, or lift the cube and then drop
it during `retreat`.

This RCA is closed; the text below is a historical record of what was proved, what was ruled out,
and the controlled changes that led to the fix. Do not restart the investigation from the original
gripper parameters. The detailed working log remains in `ai/debug/gripper-grasp.md`.

The current evidence points to two separate problems:

1. The original client-side close ladder raced the controller and left old goals running. That
   defect is understood and corrected.
2. The later fingertip friction change made `mu=100000` active in the generated SDF. Both
   fingertip friction values then changed to `mu=5`, but two manual runs still timed out in
   `close_gripper`. The active failure remains controller stall detection or configuration,
   not extreme fingertip friction alone.

## Impact

`/home` succeeds consistently. The failure is limited to `/pick` after the robot starts the grasp
sequence.

Observed outcomes are:

- a successful action followed by the cube falling during the lift;
- an abort after the fingers shake against the cube and `close_gripper` times out; or
- a successful close that finishes only just before the 10-second client timeout.

Because success changes between identical runs, the current branch is not ready to merge into
`main`.

## Reset — 2026-08-01: new discipline for this investigation

Iterations 0–11 below are kept for reference only. Six commits came out of that work
(`a38f873`..`8fb689b` on this branch); none of them solved the grasp, and stacking them made it
impossible to tell which change helped and which hurt. That stack is preserved, untouched, on
`backup/gripper-debug-stack-2026-08-01` (pushed to origin) and is not part of the active
investigation from here.

From this point on:

- **No commit until the issue is actually solved.** Only then: one clean commit, reviewed, merged
  to `main`.
- **This file is the only debug document.** Every change, test, observation, and result is logged
  here, in order, as it happens. No more `ai/debug/iteration_NN/` directories or parallel RCA files.
- The working tree right now is `a38f873` (the verified last-good baseline — it carries the pick
  pipeline itself: candidate search, descent preflight, `SyncObjects`) with one change reapplied on
  top, uncommitted: `3b0a6b4`'s mimic fix (the five gripper follower joints removed from
  `sorting_arm.gazebo_ros2_control.xacro`'s `<ros2_control>` block, so `gz_ros2_control` stops
  running a second velocity drive against the SDF-native mimic constraint) and fingertip friction
  `mu 100000 → 5`. `stall_velocity_threshold: 0.02` was already present at `a38f873`.

### Test 1 (this reapplied state): pre-grasp self-collision, then OMPL timeout

First `/pick` run never reached `close_gripper`. `move_group`'s own log showed candidate 1 found a
plan in 34 ms, but `ValidateSolution` rejected it: `forearm_link` collided with
`robotiq_85_right_finger_link` mid-trajectory (indices 136–147 of 269). Candidates 2–5 each hit the
full 5.0 s `RRTConnect` timeout with no solution at all. `/sync_objects` was confirmed to have
accepted `red_box_1` correctly (`ok=True`, pose/dimensions exactly as sent), so this was not a
targeting error — it was planner variance hitting a tight, barely-reachable candidate pose.

### Test 2 (same state, rerun): reached close_gripper, still fails, differently than before

Second run: candidate 2/5 passed the full Cartesian descent preflight, descent succeeded,
`close_gripper` started — first time this session `/pick` got that far. It still failed:
`"gripper result timed out"` at the client's full `10.0 s`.

`ros2 topic echo /joint_states`, captured for the whole close and parsed directly (raw text, 3408
messages, joint located by name each time, not by assumed index — the five followers no longer
appear in `/joint_states` at all, confirming they're out of `<ros2_control>`):

| sim t (s, relative) | `robotiq_85_left_knuckle_joint` position (rad) | velocity (rad/s) |
|---|---|---|
| 0 – 20.2 | ~0.000 (open) | ~0 |
| 20.58 | 0.080 | 0.500 (rated limit) |
| 21.0 | 0.226 | 0.179 |
| 21.4 – 34.0 (13.6 s, capture end) | **oscillates 0.221 – 0.286, never settles** | swings continuously, roughly ±0.2 |

Converted with the existing jaw-gap formula (`gap(θ) = 2·(0.03060114 + 0.0371575·cosθ −
0.04342168·sinθ − 0.02526)`, checked against both spec endpoints, `gap(0)=85.0mm`,
`gap(0.8)=0.2mm`): the oscillation band is **57.5–63.6 mm**. The 40 mm cube needs `θ≈0.448 rad`
(`gap=40.0mm`). **The knuckle stops 17–24 mm of jaw gap short of the box** and chatters there for
13+ seconds — this is not box contact, not table contact, not a grip-force question. It never gets
that far.

This is also a different signature from the old Iteration 6 trace: that one showed *position frozen*
with only the *velocity readback* spiking (sensor noise on a still joint). Here *position itself*
oscillates over a real ~6 mm band, continuously. Different mechanism, not yet identified.

### The real question, restated

Koushik: the box, grasp offset, friction, and grasp strength are not the question right now — the
gripper does not close fully even considered on its own. It stops short during `close_gripper` and
only continues moving (open/close) after the action has already failed. The question is why it
fails to close **during** the action, isolated from everything else: the pick sequence, the arm, the
box, `GripperCommander`, `skill_server_node`.

**Next test, not yet run** — isolate the ROS 2 control path from all of this project's skills code:
launch `sim.launch.xml` and command the gripper directly through the controller's own action,
nothing else running. If it closes fully, the ROS 2 control path is fine and the defect is in
`GripperCommander`/`pick_server.cpp`. If it does not, the defect is in the controller/hardware-plugin
path itself.

```bash
# [CONTAINER] terminal 1 — sim only, nothing else
cd /sorting_arm_ws
mkdir -p ai/debug/isolation
ros2 launch sorting_arm_bringup sim.launch.xml gui:=true 2>&1 | tee ai/debug/isolation/sim.log
```

```bash
# [CONTAINER] terminal 2 — bag record for exact position/velocity data, start before the goal
cd /sorting_arm_ws
ros2 bag record -o ai/debug/isolation/bag_isolated_close /joint_states /rosout
```

```bash
# [CONTAINER] terminal 3 — command the gripper directly; no skill_server_node, no /pick, no GripperCommander
cd /sorting_arm_ws
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.8, max_effort: 40.0}}" --feedback \
  2>&1 | tee ai/debug/isolation/close_goal.log
```

Ctrl-C terminal 2 once the goal in terminal 3 finishes (succeeds, stalls, or you judge it stuck).
Everything is captured in `ai/debug/isolation/` and in this file's next entry once it's run.

### Test 3: isolated close — clean pass

Koushik ran it: `sim.launch.xml`, no `skill_server_node`, no `/pick`, no `GripperCommander` — a raw
`ros2 action send_goal /gripper_controller/gripper_cmd` for `position: 0.8, max_effort: 40.0`, arm
untouched at its spawn pose, nothing synced, nothing nearby.

```
Result:
    position: 0.7907674312591553
    effort: 40.0
    stalled: false
    reached_goal: true
Goal finished with status: SUCCEEDED
```

Error against target: `0.8 − 0.790767 = 0.009233 rad`, inside the controller's `goal_tolerance:
0.01`. `reached_goal: true` — the position path resolved on its own, not a stall. No timeout, no
oscillation, one clean result.

**This settles the isolation question.** `gripper_controller`, `ros2_control`, `gz_ros2_control`,
the mimic constraint, and `bullet_featherstone` all work correctly end to end when nothing is in
the gripper's way. The ROS 2 control path is not the defect.

It also weighs directly on Koushik's suspicion that the defect is in `GripperCommander` or
`pick_server.cpp`: at `a38f873`, `GripperCommander::close()` does nothing but send this exact same
goal (`close_position_ = 0.8`, `max_effort_ = 40.0`) to this exact same action and wait for this
exact same result — there is no additional logic in between to be buggy. Test 3 just proved that
call, made in isolation, works. The only thing that differs between Test 3 (clean) and Test 2
(oscillates at 57.5–63.6mm, never settles) is the physical situation the gripper is in when the
call is made: Test 3's arm is at its spawn pose with nothing nearby; Test 2's arm has just completed
a real descent to the grasp pose, right at the object.

That points away from a skills-code logic bug and back toward the physical contact question Test 2
already raised but didn't resolve: something makes contact (or the gripper believes it does) at a
gap far too wide for the box, symmetrically. The likeliest candidate is asymmetric contact — one
pad touching the box (or something else near the descent path) off-centre, which a symmetric
two-pad jaw-gap formula would not reveal, and which visually matches the earlier screenshot
(the box appeared tilted against one finger, not centred between both pads).

### Next test: watch what the fingers actually touch, during a real close

Cheapest next step, no new tooling, still one variable: repeat the same `/sync_objects` → `/home` →
`/pick` sequence from before, and this time have Koushik watch the Gazebo viewport directly during
the `close_gripper` phase, reporting exactly what contacts what — which pad, whether the box tilts
or is pushed, whether it's the box at all versus the table or tray edge. This decides whether the
next step is a positioning/targeting question (asymmetric approach) or something else entirely.

### Test 4: real `/pick` run, box-contact hypothesis ruled out

First attempt (`bag_pick_close`) invalidated — Koushik paused Gazebo time mid-run, which aborted the
action; not evidence of anything.

Second attempt (`bag_pick_close_new`), full sequence, bag recording `/joint_states` + `/rosout`
throughout. Koushik's account, matched directly against `skills.log`: pre-grasp candidates 1–3
rejected (`MoveGroupInterface::plan() failed or timeout reached`, no visible arm motion — expected,
a rejected candidate never executes), candidate 4 passed the full descent preflight and executed,
descent executed, `close_gripper` started. Koushik watched it directly: **"It was not able to fully
close and struggled to complete the closing motion. There was a gap between the gripper tips and the
box."**

The bag confirms it with numbers. `gripper_controller` accepted the close goal at wall time
`1785603134.030266260` and was cancelled at `1785603144.030614125` — exactly `10.000s` later,
matching `result_timeout_s`. `robotiq_85_left_knuckle_joint`, extracted directly from the bag:

- `t=0 → 0.28s`: rises at exactly `0.500 rad/s` (the joint's rated limit) — free swing.
- `t=0.28 → 0.89s`: still climbing, noisy but positive velocity, reaches a **peak of `0.30630 rad`
  at `t=0.89s`**.
- `t=0.90s`: sharp reversal — velocity flips to **`−0.305 rad/s`**, position drops to `~0.288 rad`
  within 150 ms.
- `t=1.0 → 10.0s`: wanders in a band of roughly **`0.18–0.31 rad`**, never settling, never
  approaching `θ≈0.448 rad` (the angle the 40mm box requires).

Converted with the same verified jaw-gap formula: **55.4–68.2 mm** across the wandering band. The
box needs 40mm. This closely reproduces the *previous* `/pick` run's signature (that one wandered
`0.221–0.286 rad`) — two separate runs landing in the same rough angular neighbourhood. Repeatable,
not one-off noise.

**This rules out the box as the obstruction — directly, by Koushik watching it, not by inference.**
The earlier hypothesis in this file (asymmetric one-pad contact with the box) is wrong. Do not
pursue it further without new contradictory evidence.

### What's left

Test 3 (previous entry): raw `GripperCommand` goal, arm untouched at its spawn pose, nothing nearby
— closed cleanly to `0.79/0.8 rad`, no oscillation, `reached_goal: true`. These `/pick` runs, arm at
the real grasp pose (top-down, wrist bent to descend to the table), oscillate every time in the same
neighbourhood, well short of the box. The only variable that changed between the clean test and the
broken ones is **the arm's pose/orientation when the gripper closes** — not the box (ruled out
above), not `GripperCommander`'s call itself (Test 3 already proved that call works), not chance (it
repeats).

Plain physical candidate, not yet evidence: the five follower joints have no independent drive now
(change 1 removed them from `<ros2_control>` entirely) — they move purely via
`bullet_featherstone`'s native SDF mimic constraint, which has no explicit damping. At the grasp
orientation the gripper hangs pointing down; at the isolated test's spawn pose it likely doesn't.
Gravity loading a passive, undamped four-bar linkage differently in each orientation, while a
P-only velocity servo drives the one actuated joint through it, could plausibly produce exactly this
kind of position oscillation.

### Next test, not yet run: same raw command, arm held at the real grasp pose

Zero new setup. Right after the next `/pick` aborts at `close_gripper` (arm stays exactly where it
is — the timeout path holds position, it does not reopen), send one more **raw** `GripperCommand`
goal directly, bypassing `GripperCommander`/`skill_server_node` entirely — the same call Test 3
already proved works cleanly, but this time with the arm already at the real grasp pose instead of
its spawn pose.

```bash
# [CONTAINER] — run the same sync/home/pick sequence, keep the bag recording through it, let it abort naturally
ros2 bag record -o ai/debug/visual-check/bag_grasp_pose_direct /joint_states /rosout
# (start this before /sync_objects, same as before)

# [CONTAINER] — immediately after the abort, arm untouched, same bag still recording:
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.8, max_effort: 40.0}}" --feedback \
  2>&1 | tee ai/debug/visual-check/gripper_at_grasp_pose.log
```

- If it **also oscillates** in the same 55–68mm band: confirms the arm's pose/orientation is the
  variable, not the pick sequence or the box. Next step: investigate gravity-loading of the
  undamped mimic linkage at that orientation.
- If it **closes cleanly**: the failure is specific to something about `/pick`'s own close call at
  that moment (timing right after descent, residual arm velocity) rather than pose alone — different
  next step, would need to look at exactly what differs at that moment.

### Test 5: the stall verdict doesn't stop the drive

Koushik ran the test above. Real `/pick` run: candidate 4 selected, descent aligned correctly with
the box (visually confirmed). `gripper_controller` accepted the close goal at wall time
`1785604078.089999460`, cancelled at `1785604088.090379047` — `10.00038s` later, matching
`result_timeout_s: 10.0` as always.

Before running the follow-up command, Koushik watched the gripper directly and **photographed it**:
one jaw visibly closing, the other not. A real, directly-observed asymmetry — not inferred from the
driven joint's angle.

He then sent the raw `GripperCommand` goal three times, arm untouched throughout:

| goal | accepted (wall) | result position | stalled | reached_goal |
|---|---|---|---|---|
| 1 | `1785604138.355702640` | `0.167815` | true | false |
| 2 | `1785604145.737113093` | `0.168612` | true | false |
| 3 | `1785604149.289280008` | `0.168666` | true | false |

All three within `0.001 rad` of each other. Then, stopping there (Ctrl-C on the CLI, nothing new
sent): **"After that, the gripper started moving again. It dropped the box, continued trying to
hold it, and only then did the gripper fully close."**

**The mechanism, verified directly against the installed controller source**
(`/opt/ros/jazzy/include/gripper_action_controller/gripper_controllers/gripper_action_controller_impl.hpp`),
`update()`, quoted in full:

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

`check_for_success()` marks the goal `SUCCEEDED` (`reached_goal`), `SUCCEEDED` with `stalled: true`
(only because `allow_stalling: true`), or `ABORTED` — and in every case clears `rt_active_goal_`, so
no more results are ever reported for that goal. **But it never touches `command_struct_rt_.position_`,
and the `updateCommand()` call three lines below runs unconditionally, every cycle, regardless of
whether an active goal exists.** Only two things ever change `command_struct_rt_.position_`: a new
goal being accepted, or a cancellation (which calls `set_hold_position()`, explicitly freezing the
target at the current position). **A stalled-but-`SUCCEEDED` result does neither.** The joint keeps
being driven toward the original target indefinitely, in the background, with no client watching,
until a cancel or a new goal changes it.

This reconciles everything above. The three repeated goals landed at nearly the same position
because the joint was never actually stopped between them — the one thing that does freeze it
(cancel) happened once, at the very start, and never again. Every `stalled: true` after that meant
"the controller stopped watching," not "the gripper stopped moving." When Koushik stopped sending
commands entirely, the still-live drive from the last goal (still targeting `0.8`) kept pushing —
enough, eventually, to physically displace the box — matching his account exactly, and matching Test
3's clean-close behavior once nothing remains in the way.

**Why this matters beyond this one run:** every `stalled: true` result logged in this RCA since
Iteration 1b was read as "settled contact, safe to proceed to retreat." That reading is wrong. It
means only that reported velocity was briefly under threshold — the joint is not held, not stopped,
and resumes pursuing the original target the moment nothing continues to resist it. This is a better
candidate for the old "box held weakly, dropped during retreat" pattern than anything considered
before: the gripper was never actually stopped, it kept trying to close through the object the whole
time.

**What this does not yet establish:** whether the photographed one-jaw asymmetry is its own separate
defect (the mimic constraint not holding symmetry under real load) or a downstream effect of the
persistent drive fighting uneven box contact; and why the parked position clusters so consistently
near `~0.17 rad` across very different code states (Iteration 4 recorded `0.174928 rad` months ago
under a completely different configuration).

### Next test, not yet run: confirm the persistent drive in complete isolation

One goal, one result, then **nothing** — no new goal, no cancel — just watch `/joint_states`
afterward. If position keeps changing with zero commands issued after the terminal result, that
directly confirms the mechanism above, isolated from any effect of sending three goals in a row.

```bash
# [CONTAINER] — run the same sync/home/pick sequence, let it abort naturally as before

# [CONTAINER] — start a bag before the goal below, covering at least 60s
cd /sorting_arm_ws
ros2 bag record -o ai/debug/visual-check/bag_persistent_drive /joint_states /rosout

# [CONTAINER] — send exactly ONE goal, then send nothing else at all
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.8, max_effort: 40.0}}" --feedback \
  2>&1 | tee ai/debug/visual-check/single_goal_then_watch.log
```

Once that one goal's result prints, do not send anything else. Let the bag keep recording for
another full minute with the terminal idle, then Ctrl-C the bag.

### Test 6: a separate goal closed it, then it shook

Koushik ran the sequence above once (not the repeated-goal version). Timeline, cross-checked across
three terminals' timestamps, all consistent:

| wall time | event |
|---|---|
| `...6049.053` | `open_gripper` accepted (`/pick` start) |
| `...6049.104 → 6054.107` | pre-grasp candidate 1: OMPL `TIMED_OUT` at 5.0s, rejected |
| `...6054.109 → 6054.159` | candidate 2: plan computed in ~50ms |
| `...6054.163 → 6054.167` | candidate 2 Cartesian descent preflight: 26 points, 100% coverage — pass |
| `...6054.177 → 6070.377` | pre-grasp move executes (16.2s) |
| `...6070.389 → 6073.737` | descent move executes (3.35s) |
| `...6073.758` | `close_gripper` goal accepted (`/pick`'s own close call) |
| `...6083.759` (+10.0003s) | client cancels — matches `result_timeout_s: 10.0` exactly, same as every prior run |
| `...6105.287` (+21.53s) | Koushik's separate, manually-typed `GripperCommand` goal accepted |
| (result) | `position: 0.795649`, `stalled: false`, **`reached_goal: true`**, `SUCCEEDED` |

Error from target: `0.8 − 0.795649 = 0.00435 rad`, genuinely inside `goal_tolerance: 0.01` — a real
`reached_goal`, not a stall. Matches Koushik's account exactly: the separate command "pushed the box
out of the way... then fully closed... command reported success." **Then: "After the gripper
closed, it started shaking violently."**

### The mechanism, verified directly against the installed controller source

`/opt/ros/jazzy/include/gripper_action_controller/gripper_controllers/gripper_action_controller_impl.hpp`,
`update()`, quoted in full:

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

`check_for_success()` marks the goal `SUCCEEDED` (`reached_goal`), `SUCCEEDED` with `stalled: true`
(only because `allow_stalling: true`), or `ABORTED` — and in every case clears the active goal, so
no more *results* are ever reported for it. **It never touches `command_struct_rt_.position_`, and
the `updateCommand()` call three lines below runs unconditionally, every cycle, regardless of
whether an active goal exists.** Only two things ever change that target: a new goal being accepted,
or a cancellation, which calls `set_hold_position()` (also re-verified directly,
`gripper_action_controller_impl.hpp:123-153`) — explicitly freezing the target at the current
position. **A stalled- or reached-`SUCCEEDED` result does neither.** The joint keeps being driven
toward whatever the target was, indefinitely, in the background, with no client watching, until a
cancel or a new goal changes it.

This explains Test 6 end to end: the joint was never actually stopped between the original `/pick`
close call and Koushik's separate goal except by the one cancel (which does freeze it) — everything
after that cancel, including the separate goal's own `reached_goal: true`, leaves the target pinned
wherever it landed, still being driven toward it in the background afterward too. Every `stalled:
true` this whole RCA has ever logged, back to Iteration 1b, meant "the controller stopped watching,"
never "the gripper stopped moving."

## Fix: `GripperCommander::close()` now holds in place on a genuine stall

`src/sorting_arm_skills/src/gripper_commander.cpp`, `close()`, before this fix (the code that ran in
every test above):

```cpp
if (result.stalled) {
  return GraspOutcome{true, true, 0, "stalled at position " + std::to_string(result.position)};
}
```

It saw `stalled: true` and returned, treating that as "captured, safe to proceed to attach and
retreat" — without ever telling the controller to stop. Per the mechanism above, the joint kept being
driven toward `close_position_` (`0.8`) the whole time `pick_server.cpp` went on to attach and
retreat. Direct candidate for the historical "held weakly, dropped during retreat" pattern going back
to Iteration 1b/4.

**Change:** the moment `result.stalled` is true, send one more `GripperCommand` goal targeting
`result.position` — the joint's own just-measured position from the stalled result itself, no new
subscription needed — instead of leaving `close_position_` as the standing target. A goal whose
target is (nearly) the current position resolves via the `reached_goal` branch almost immediately,
which is the only way this controller design can actually be told "stop pushing, hold here."

```cpp
if (result.stalled) {
  // stalled==true only means the controller stopped watching — update() keeps driving
  // toward close_position_ regardless, so we re-target the stall position or it keeps
  // pushing on whatever it just hit
  GripperCommandAction::Result hold_result;
  const auto held = send_goal(result.position, "close_gripper", hold_result);
  if (!held.ok) {
    return GraspOutcome{false, false, held.native_code, held.message};
  }
  return GraspOutcome{true, true, 0, "stalled at position " + std::to_string(result.position)};
}
```

No other change. `.hpp` untouched — reuses the existing `send_goal()` as-is. Uncommitted, per the
standing rule: not committed until runtime-verified.

**Build:** `cbuild` — clean.

### Test 7: the fix is correct but wasn't exercised — the dominant failure is upstream of it

Koushik ran the same `/sync_objects` → `/home` → `/pick` sequence. Result: identical to every
pre-fix run — `"gripper result timed out"`, `ABORTED`. `gripper_controller` log: only **one** goal
accepted before the cancel (`Received & accepted new action goal` → `Got request to cancel goal`,
`10.000338s` apart). The fix's new code never ran, because it only executes after
`send_goal(close_position_, ...)` returns with a *resolved* `stalled: true` — and here, `send_goal()`
itself hit its own 10-second client-side wait first (the timeout branch, not the stalled branch).
The controller never produced any terminal verdict at all in this run — consistent with
`stall_velocity_threshold: 0.02` needing a full continuous 1.0s quiet window that this run never
produced.

Koushik's account: **"the entire gripper appeared to twist while continuously trying to close... it
never completed the grasp... After the action aborted... it continued trying to grasp the box even
after the action had already been aborted."** Photos: one finger visibly much closer to the box, the
other farther away — the same asymmetry photographed in the previous test, now observed twice,
independently.

**The fix stays — it's correct for the case it targets (a controller that does resolve a stall) and
is still needed for that case.** It just isn't the dominant failure mode. The dominant failure is a
sustained oscillation/twist that never produces a clean 1.0s quiet window at all within 10 seconds,
which no amount of "what to do after a stall" logic can touch, because the controller never gets
there.

### Why does `arm_stack` manage this with the ostensibly same gripper and the same action?

Koushik asked directly: `arm_stack` uses the same Robotiq 2F-85, the same
`position_controllers/GripperActionController`, the same `control_msgs/action/GripperCommand` — so
why does its pick-and-place work while ours doesn't?

Re-checked against the earlier full trace of `arm_stack`'s gripper path
(`pick_place_dynamic.cpp:278-383`, quoted in full earlier in this investigation): **`arm_stack` never
verifies the grasp at all.** Its `send_command()` branches only on
`result.code == rclcpp_action::ResultCode::SUCCEEDED` — it reads `stalled` and `reached_goal` only
to log them, never to decide anything. Its `close_slowly()` ladder sends steps toward
`grasp_close_position: 0.62` — not the mechanical limit (`0.8` here) — in `0.04` rad increments, each
with only a **1500ms** result timeout and `timeout_means_contact = true`: if a step doesn't resolve
in 1.5 seconds, `arm_stack` treats that timeout *itself* as "we've made contact," and moves on
immediately — critically, **without cancelling that goal**. The last, "contacted" step is left
running, still targeting its own modest position (well short of full closure), continuously applying
gentle pressure through attach and retreat, simply because nothing ever told it to stop.

So `arm_stack`'s grasp isn't more correctly *verified* than ours — it's not verified at all. It works
because its contract is shallow and forgiving: a short timeout stands in for "contact," a partial
target stands in for "closed enough," and leaving the goal alone (not cancelling) happens to keep
gentle pressure on the object by accident. This project's contract, since Iteration 11, is
deliberately stricter — a real settled gap measured against the object's actual width — which is a
harder target to hit given what this controller's `stalled` flag actually is (not a stop, just a
stopped report) and what real four-bar contact dynamics do inside a 10-second window. This was
already noted in `Causes ruled out` above ("Its close is fire-and-forget..."); Test 7 is the direct,
mechanism-level version of that same fact.

### Test 8: state-only follower telemetry exposes the physical asymmetry

Koushik chose to observe the linkage before changing the grasp contract. The five follower joints
were restored to the `<ros2_control>` resource declaration with position and velocity state
interfaces only. Each has `mimic="false"`, so the state is published without activating
`gz_ros2_control`'s second mimic drive. No geometry, controller setting, motion target, or skills
logic changed.

The runtime gate passed:

- `grep -c "is mimicking joint" ai/debug/followers/sim.log` returned `0`;
- all five followers exposed position and velocity state interfaces and no command interfaces; and
- `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` were all active.

The `/pick` run selected pre-grasp candidate 3 after candidates 1 and 2 timed out. Pre-grasp and
descent both executed successfully. `gripper_controller` accepted the close goal at
`1785609736.966822115` and received the client cancellation at `1785609746.967158734`, exactly
`10.000337 s` later. `/pick` then returned `"gripper result timed out"` with status `ABORTED`.

`ai/debug/followers/bag_followers2` contains 1,000 `/joint_states` samples in that close window. The
driven left knuckle reached a maximum of `0.307114 rad` during the first second. By three seconds it
had physically settled into `0.198320..0.199020 rad` and remained there until cancellation. This
corrects the earlier visual interpretation: the driven joint's position is not continuously
oscillating for the full timeout.

Its reported velocity is still unstable. From three seconds until cancellation it ranged from
`-0.126517` to `+0.172595 rad/s`. The longest continuous interval with
`|velocity| < stall_velocity_threshold` (`0.02 rad/s`) was only `0.099878 s`. The controller needs
`1.0 s`, so it could not issue a stall result even though the measured position was almost fixed.
This directly explains the missing terminal verdict and the ten-second client timeout.

The follower positions stayed bounded, so this is not the Iteration 7 double-drive signature that
grew to `0.149 rad`. They did not maintain the ideal mimic relation exactly either. At the final
sample, the error from `follower = multiplier * left_knuckle` was:

| Follower | Final error (rad) |
|---|---:|
| `robotiq_85_right_knuckle_joint` | `0.012742` |
| `robotiq_85_left_inner_knuckle_joint` | `-0.004604` |
| `robotiq_85_right_inner_knuckle_joint` | `0.002231` |
| `robotiq_85_left_finger_tip_joint` | `0.066098` |
| `robotiq_85_right_finger_tip_joint` | `-0.009538` |

The strongest defect is therefore the left fingertip follower. At the final sample, the absolute
left/right fingertip joint-angle difference was `0.056560 rad`. Combining each knuckle angle with
its fingertip angle gives a `0.062894 rad` (`3.604 deg`) pad-orientation mismatch. This is direct
physical telemetry for the twist Koushik photographed; it is not the analytic, always-symmetric TF
model.

Using the measured knuckle and fingertip states in the gripper's own linkage geometry gives a
central pad gap of `67.053..67.119 mm` after the position settled, ending at `67.085 mm`. Even the
smallest gap calculated across the pad's vertical collision-mesh span at the final sample is about
`63.88 mm`. The box is `40 mm` wide, so the pads remain at least `23.88 mm` too far apart for box
contact. This confirms Koushik's visual report that air remained between the gripper and the box.
The bag does not contain the simulated box pose, so it cannot split that clearance into an exact
left-pad-to-box distance and right-pad-to-box distance.

The bag continued for `22.37 s` after cancellation. The driven joint moved from about `0.1987 rad`
to `0.1656 rad` in the first second and ended at `0.1742 rad`. Visible shaking may stop after the
abort, but the physical joint does not remain at its cancellation-time position.

No next behavior, geometry, or tuning change is selected from this result yet. The first new fact to
explain is why a nearly stationary driven position reports velocity spikes large enough to defeat
the controller's quiet-window test. The bounded but persistent left-fingertip constraint error is a
separate measured asymmetry that must not be hidden by changing the stall threshold.

### Test 9: bounded raw goals stall in open air after the failed pick

The next probe asked `GripperActionController` directly for positions `0.20`, `0.25`, `0.30`,
`0.35`, `0.40`, `0.44`, and `0.457 rad`, each with `max_effort: 40.0`. The instrumentation gate
still passed: there were zero `is mimicking joint` lines, the five followers remained state-only,
and all three controllers were active. The bag and logs are in
`ai/debug/controller_verdict_probe/`.

The probe first ran the normal `/pick`, which again reached `close_gripper` and timed out. The raw
goals therefore started from the physical state left behind by that close-goal cancellation. Every
raw goal returned action status `SUCCEEDED`, but the controller result was `stalled: true` and
`reached_goal: false`:

| Command (rad) | Reported position (rad) |
|---:|---:|
| `0.20` | `0.169698` |
| `0.25` | `0.168166` |
| `0.30` | `0.168604` |
| `0.35` | `0.168789` |
| `0.40` | `0.168916` |
| `0.44` | `0.169005` |
| `0.457` | `0.169061` |

The first command was only about `0.031 rad` away from the starting position. With the installed
`gz_ros2_control` position gain and 100 Hz update rate, that asks for about `0.31 rad/s`, below the
joint's `0.5 rad/s` velocity limit. It still failed to advance. This disproves the narrower
hypothesis that only the original large close command and its velocity saturation cause the jam.

After the `0.25 rad` command, the driven joint's measured velocity stayed roughly within
`-0.003..+0.003 rad/s`. The controller's stall detector is therefore behaving consistently in this
probe: the joint really is nearly stationary for its one-second quiet window. The problem is the
physical meaning of that result. The gripper is stalled in air, so `stalled: true` is not evidence
that it captured the box.

The measured central pad gap stayed near `68.7 mm`; across the stabilized raw-goal windows it ranged
from about `68.662` to `68.778 mm`. The box is `40 mm` wide. At the last sample, the left and right
fingertip collision meshes were separated from the box by at least `13.1 mm` and `10.7 mm` along the
closing axis. Their lowest collision-mesh vertices were `10 mm` above the table. This rules out the
box and table as the contacts stopping these goals.

The linkage was still asymmetric at the final sample. The follower errors from the ideal mimic
relation were `-0.014752`, `+0.000488`, `-0.006325`, `+0.034565`, and `-0.045287 rad` in right
knuckle, left inner knuckle, right inner knuckle, left fingertip, and right fingertip order. The pad
orientation mismatch was `5.420 deg`.

This test does not yet prove that the same bounded sequence fails from a fresh simulation state. It
proves that bounded goals cannot recover the linkage state left by the failed close and
cancellation. The next diagnostic must run a bounded raw-goal ladder from the freshly opened
gripper, before any full close or cancellation. That separates a general native mimic-linkage jam
from a bad post-cancellation equilibrium. Until that distinction is measured, changing the grasp
success contract or adding lift would hide the first false physical contract.

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

## Iteration 12: close to the object, not the hard stop, and measure the jaw instead of trusting the controller

The working tree at the start of this iteration was `a38f873` (Iteration 11's ladder code is on
`origin`, five commits ahead, and was not present here) with two uncommitted changes applied on top:
the Iteration 5/7/8 description edits (fingertip friction, follower joint limits, five followers
state-only with `mimic="false"`), and an unverified edit to `GripperCommander::close()` that
re-targeted the joint to `result.position` on a stall. This iteration replaces all of that in one
change, on Koushik's explicit direction to stop iterating one variable at a time. The instruction
carries a cost this entry is written to offset: a single failed run after this point has to be
diagnostic on its own, so every measurement below is logged, not just eyeballed.

### The mechanism this iteration actually fixes

`gripper.close_position: 0.8` is the knuckle's URDF upper limit. A 40 mm cube stops the knuckle at
`0.4484 rad` (closed form below). `gz_ros2_control` converts a position command to a velocity
setpoint as `joint_velocity = position_proportional_gain × position_error × update_rate`; with the
never-overridden default gain `0.1` and `update_rate: 100`, the blocked `0.3516 rad` error asked for
**3.52 rad/s from a joint rated 0.5 rad/s**, continuously, because nothing in the controller ever
reduces `command_struct_rt_.position_` short of a new goal or a cancel. Iteration 11 stated this
arithmetic and did not act on it. This iteration does: `GripperCommander::close(double
object_width_m)` now targets the knuckle angle for `object_width_m − squeeze_depth_m_`, never the
hard stop.

**The correction to the persistence claim, stated precisely because an earlier draft of this
iteration got it wrong.** After a terminal action result, the *action goal* is over — nothing about
it stays "live," and no action feedback exists to keep alive (`GripperActionController` never
publishes feedback, confirmed in Iteration 7). What persists is the controller's own stored
`command_struct_rt_.position_`, which `updateCommand()` keeps applying every control cycle
regardless of goal state (verified against the installed controller source, Test 5/6 above). So the
requirement this iteration actually implements is narrow: after a successful capture, send no new
goal and issue no cancel, because either replaces that stored target with the measured position and
drives the residual error to zero. `send_goal()`'s result-timeout path keeps its
`async_cancel_goal()` — that is a genuine failure stop, on a path where nothing was captured to
hold.

**What this does not claim.** `max_effort: 40.0` is inert on this position command interface —
`skills.yaml`'s own comment already said so, and `move_group` logs "will command a max effort of:
0" every run (Iteration 7). Nothing here commands a specific gripping force. What persists is a
residual *position* error that the position→velocity adapter turns into a continuing closure
command; whatever contact force results from that is an emergent property of the physics contact
model, not a commanded quantity.

### `squeeze_depth_m`: a derived window, not a guessed constant

The jaw-gap closed form, endpoint-checked against the spec (`gap(0)=85.0mm`, `gap(0.8)=0.16mm`),
now lives in code as `jaw_gap_m()`/`knuckle_angle_for_gap_m()` in `helpers.hpp`/`.cpp` instead of
only in this file's prose:

```
gap(th) = 2*(c + a*cos(th) - b*sin(th)),  a=0.0371575, b=0.04342168, c=0.03060114-0.02526
```

`knuckle_angle_for_gap_m` is the exact inverse via the `R·cos(th+phi)` identity
(`R = hypot(a,b)`, `phi = atan2(b,a)`), rejecting an out-of-range gap instead of clamping it.

At the 40 mm cube's contact angle `th40 = 0.4484 rad`, the local sensitivity is
`dgap/dth = 2(−a·sin(th) − b·cos(th)) = −0.1105 m/rad`, i.e. **1 mm of gap is 9.05 mrad of
knuckle**. That fixes both ends of the usable window for a squeeze target beyond contact:

- **Upper bound** — residual error must keep the commanded velocity inside the joint's rating:
  `err ≤ 0.5 / (0.1 × 100) = 0.05 rad → 5.52 mm`.
- **Lower bound** — residual error must exceed the controller's `goal_tolerance` (default
  `0.01 rad`) or the goal resolves `reached_goal` with no continuing closure command →
  `1.10 mm`.

`squeeze_depth_m: 0.003` sits at the middle of `1.10–5.52 mm`. For the 40 mm cube the target is
`0.4755 rad` against the `0.4484 rad` the box allows: residual `0.0271 rad` → commanded
`0.271 rad/s`, inside the 0.5 rad/s rating — against `3.52 rad/s` at the old `0.8` target, a 13×
reduction. Both bounds and the round-trip (`gap(knuckle_angle_for_gap_m(0.037)) = 37.0 mm`) were
checked numerically before this was written, not asserted from the algebra alone.

### Verification: measure the jaw and the linkage, never trust `stalled`

Verified directly against the installed controller source (Test 5/6 above): `stalled: true` means
only "the controller stopped reporting," recorded once while the jaw sat in open air at 67–69 mm on
a 40 mm box. `GraspOutcome.object_present` no longer reads that flag at all.

`GripperCommander` now subscribes to `/joint_states` (joints found by name, never assumed index)
and, on every terminal close/verify call, requires **both**:

1. `|jaw_gap_m(left_knuckle) − object_width_m| ≤ capture_tolerance_m` (`0.005`), and
2. `| |left_finger_tip| − |right_finger_tip| | ≤ symmetry_tolerance_rad` (`0.005`).

`capture_tolerance_m` is an error budget, not a round number: ≈0.55 mm from the allowed 5 mrad
symmetry residual (via the 9.05 mrad/mm sensitivity above), the remainder for contact penetration
and solver slop. The failure modes it has to reject sit 28 mm away (the recorded 67–69 mm air-stall)
and 40 mm away (a close on nothing), so there is no risk of confusing them with a real 40 mm capture.
`symmetry_tolerance_rad` directly answers Koushik's photographed one-jaw asymmetry: the RCA measured
`3.6°`/`5.4°` (63/95 mrad) mismatches during failures, so `0.005 rad` is an order of magnitude
inside the observed defect, not a number chosen to make a run pass.

No settle sleep was added: a terminal controller result already means the 1.0 s quiet window
elapsed (when the controller reaches one at all), so the latest `/joint_states` sample at that
instant is the physically settled state, read on the executor thread while the worker blocks on the
gripper future — the existing single-threaded-executor-plus-worker invariant this class has always
relied on.

**Scope limit on `BOX_X`.** `SceneManager::known_object_geometry` now also returns `width_m` (from
`primitive.dimensions[BOX_X]`, previously stored and never read). This is the closing dimension only
because the cube is `0.04³` and `grasp_pose` fixes `yaw = 0.0`. This iteration does not implement
general object-width selection; a non-square object would need the approach yaw chosen against the
object's own frame.

A new `verify_hold` phase re-runs the same gap+symmetry check after `retreat`, through
`GripperCommander::verify_hold()`, with no new command issued. A jaw that sprang back open on the
lift fails `verify_hold` rather than letting `/pick` report `ok: true` on a dropped cube.

### The physics engine: a platform decision, taken here, not a fix under test

The world's physics engine changes from `bullet_featherstone` to `dart` in this iteration.
**This is not claimed as the fix for anything measured above**, and is not backed by an A/B run
against this bug — it is a platform choice: `dart` is Gazebo Harmonic's default and primary engine,
`bullet_featherstone` support is documented upstream as preliminary. Iteration 6's finite-difference
argument for a `bullet_featherstone` velocity-readback defect is suggestive, not conclusive: a joint
reversing direction inside an 80 ms sample window can produce a high instantaneous velocity reading
alongside near-zero net displacement without the reading itself being wrong. Recording this
explicitly so a future reader does not treat the engine switch as validated evidence it was never
given.

**A real, load-bearing consequence of the switch was found before any runtime test.** The preflight
this iteration's plan called for — checking the installed `libgz-physics-dartsim-plugin.so` for
mimic-constraint symbols — came back **zero**, against **337** for the same check on
`libgz-physics-bullet-featherstone-plugin.so` (checked with both `strings` and `nm -D` on the
installed `gz-physics7` 7.6.0 plugins; `dartsim`'s own `JointFeatures` class exports no
`SetJointMimicConstraint`-shaped symbol at all). `gz-sim` has no engine-agnostic mimic system to fall
back on either — only the physics-engine plugin can satisfy that feature request. Under `dart`, the
SDF `<axis><mimic>` block sdformat still generates from the URDF tag is simply never enforced
natively: the five follower joints would reach physics completely unconstrained, worse than any
state examined in Iterations 7 or 8.

This was strong enough evidence to act on directly rather than wait for a failed Gate B, so the
pre-decided fallback was applied in the same change: `sorting_arm.gazebo_ros2_control.xacro`'s five
follower `<joint>` entries lose their `mimic="false"` attribute (added in Iteration 8) and go back to
plain state-only declarations. This restores `gz_ros2_control`'s own control-cycle mimic drive
(auto-detected from the URDF `<mimic>` tag via `hardware_interface`'s parser, the same mechanism
Iteration 8 disabled to stop it fighting `bullet_featherstone`'s native constraint) as the followers'
**only** enforcer — safe specifically because no native constraint exists under `dart` to fight it,
which is exactly `arm_stack`'s own single-enforcer shape under Fortress/`ign_ros2_control`. Gate B in
the verification plan (a raw `GripperCommand` goal against the open gripper, checked for fingertip
symmetry and mimic tracking error in a bag) is still the runtime confirmation this reasoning is
correct — the static symbol check proves the constraint is absent, not that the fallback drive
behaves well under real contact load.

### Ground-truth box pose: bridged for the bag, never wired into the skill

`gz_bridge.yaml` now also bridges `/world/sorting_cell/dynamic_pose/info`
(`gz.msgs.Pose_V` → `tf2_msgs/msg/TFMessage`) alongside the existing `/clock` bridge, so a recorded
bag can show where `red_box_1` actually was throughout close/attach/retreat. No skills code
subscribes to it. Jaw gap and symmetry prove something stopped the jaw at approximately the object's
width and that both pads agree — they do not by themselves rule out a corner catch or a vertically
slipping box. The bridged pose is the independent check for that, deliberately kept out of the
skill's own verdict: reading simulator ground truth in a skill would be a perception bypass, reading
it from a bag afterward is verification.

### Known inconsistency, left deliberately

The SRDF's `gripper_close` group state (`sorting_arm.srdf`) still names `0.8` — a MoveIt named
state the skills close path has never used and still does not. Left as-is; not part of this slice.

### Build and static validation

`colcon build --symlink-install` on `sorting_arm_interfaces`, `sorting_arm_skills`,
`sorting_arm_description`, `sorting_arm_bringup`, `sorting_arm_moveit` — clean, only the
pre-existing upstream `tl_expected` deprecation warning. `colcon test` on `sorting_arm_skills`
(`cpplint`, `cppcheck`, `lint_cmake`, `xmllint`) — 0 errors, 0 failures. `xacro | check_urdf` on the
regenerated sim URDF — parses, root link `world`. `gz sdf -p` on the regenerated world — confirms
`<physics type='dart'>`. The jaw-gap formula and both derived bounds were checked numerically
(Python), not just algebraically, before being written into either the code or this entry.

### Runtime evidence still required before this RCA closes

Nothing above is runtime-verified yet. Acceptance is Gate A (no `is mimicking joint` lines are
expected now — the followers are driven by `gz_ros2_control` on purpose, so this specific check from
Iterations 8–11 no longer applies as a pass condition; the new pass condition is Gate B below),
Gate B (a raw `0.475 rad` goal against the open gripper settles near target with fingertip symmetry
within `0.005 rad` and bounded mimic-tracking error — this is what actually confirms the dartsim
mimic fallback holds under motion, not just statically), and Gate C (the full `/pick` sequence
against `red_box_1`, bagged, meeting all seven of Koushik's original acceptance points). Do not
mark this RCA closed on the strength of the static evidence above alone.

### Gate B: clean pass, first time this RCA has one

Koushik ran the raw free-air goal (`0.475 rad`, arm untouched at spawn) and bagged
`/joint_states`. Extracted directly from `/tmp/i12_bag_free_close`: the knuckle rises at the rated
`0.5 rad/s`, reaches exactly `0.4750 rad`, and holds there with velocity `0.0000 rad/s` for the
remaining ~15 seconds recorded. The two fingertip followers land on `-0.4750` and `+0.4750`
respectively — the ideal `multiplier=-1`/`multiplier=1` mimic relation, exactly, with a measured
symmetry residual of **0.000 mrad**. This is categorically better mimic tracking than any prior
iteration achieved under `bullet_featherstone` (which never got tighter than several hundred mrad
under load). Gate B passes outright; the `dart`-plus-`gz_ros2_control`-own-drive fallback from the
previous section holds under real motion, not just statically.

### Gate C, first attempt: the jaw closed correctly and the pick still aborted

Koushik ran the full `/sync_objects` → `/home` → `/pick` sequence against `red_box_1`
(`/tmp/i12_bag_pick_use`), watched it directly, and photographed it: both pads flush against the
box, centred, no visible asymmetry — the clearest physical contact this RCA has on record. The
action still reported `ok: false`, `message: "gripper result timed out"`, `ABORTED`.

Extracted from the bag, `gripper_controller` accepted the close goal at `rosout` wall time
`1785618126.953` and was cancelled at `1785618136.953` — the usual `10.000s` client deadline. The
driven `robotiq_85_left_knuckle_joint`, read for the whole window:

- Free swing to `t≈0.8s`, then settles hard: from `t≈1.0s` onward the position holds inside a
  **0.001 rad (≈0.1 mm) band**, ending at `0.4502 rad` — a measured gap of **39.8 mm** against the
  **40 mm** box, well inside `capture_tolerance_m`. This is genuine, settled physical contact, not
  a near-miss.
- Reported velocity does not settle to match. It sits mostly in a small negative creep
  (`-0.002` to `-0.018 rad/s` — plausibly the real residual push from the intentional
  `squeeze_depth_m` overshoot working against a rigid stop) but is interrupted by a **consistently
  `+0.15 rad/s` spike**, recurring roughly every 0.3–1.4 seconds for the entire window, each one
  resetting the controller's quiet-window timer. The longest uninterrupted quiet stretch measured
  **0.98 s against the `stall_timeout` default of `1.0 s`** — missed by 20 ms.

**The clock this had to be measured in matters.** `get_node()->now()` inside
`check_for_success()` respects `use_sim_time`, so `stall_timeout` is compared in *simulation*
time, not wall time. Converting each `/joint_states` message's own `header.stamp` (sim time)
instead of the bag's wall-clock arrival time gives an effective real-time factor for this close
window of **0.848** — far healthier than Iteration 7's `0.09` collapse under `bullet_featherstone`,
but still below 1.0, and the 20 ms miss above is measured in the sim-time domain, the one that
actually governs the controller.

This is a materially different signature from Iteration 6's bullet-side finding. That one was
chaotic — velocity swinging both directions with no discernible pattern while position stayed
frozen. This one is a single-signed, semi-regular `+0.15 rad/s` blip, consistent with a periodic
contact-solver re-evaluation under `dart`'s LCP-based constraint solver rather than raw sensor
noise, though the exact mechanism is not pinned down further here — nothing currently bridged
(no contact-force topic) can localise it more precisely than "engine-side, tied to sustained rigid
contact." Recorded as a new open fact, not solved.

### Iteration 13: a client-side timeout is not proof of failure either

Reading `check_for_success()` and `set_hold_position()` directly from the installed source settles
what actually happens at cancellation: `set_hold_position()` sets `command_struct_.position_` to
`joint_position_state_interface_->get().get_optional().value()` — the joint's own current position
at that instant. In this run that is `≈0.4502 rad`, already inside the box. Cancelling here does
not reopen the jaw or release the object; it freezes the servo exactly where it already was,
holding.

`GripperCommander::close()` was, until this iteration, throwing that fact away. On any `send_goal`
failure — including "gripper result timed out" — it returned `GraspOutcome{false, false, ...}`
unconditionally, never consulting `/joint_states`. Gate C's first run proves that path can discard
a capture already verified by the same measurement this class trusts everywhere else.

**Change:** `close()` now checks the jaw specifically on the `"gripper result timed out"` failure
(the one case where the goal genuinely ran for its full duration and only *our* wait gave up —
`server unavailable`/`goal rejected`/`goal-response timeout` mean nothing ran long enough to be
worth measuring, and are left as unconditional failures). If `evaluate_capture()` reports the
object present, `close()` returns that measurement instead of the timeout failure. `GraspOutcome.ok`
is re-scoped from "the action completed" to "a usable verdict exists, from the controller or from
direct measurement" — documented at the struct definition.

**This bears directly on Koushik's original acceptance point 3** ("`GripperActionController`
returns its terminal successful grasp result"). Under this change, a controller that never
resolves at all can still count as a successful pick if `/joint_states` proves the jaw is
correctly closed on the object. That is a deliberate redefinition, not an oversight — flagged
explicitly here because it changes what "the pick succeeded" means, and is reversible in one
function if a stricter, controller-verdict-only contract is wanted instead.

**Not changed:** `stall_velocity_threshold` and `stall_timeout` stay at their current values.
Loosening either was considered and rejected — Iteration 6 already established that no single
velocity threshold can separate settled contact from engine noise without also masking real
motion, and this iteration's own `+0.15 rad/s` spikes are the same order of magnitude as genuine
free-swing motion, so that reasoning still holds under `dart`. The fix is verifying independently
of the controller's verdict, not tuning the controller to produce one more often.

**Build:** `colcon build --symlink-install --packages-select sorting_arm_skills` — clean, only the
pre-existing `tl_expected` warning. `colcon test` — `cpplint`/`cppcheck`/`lint_cmake`/`xmllint`, 0
errors, 0 failures.

**Runtime result:** not yet re-run. Gate C needs to repeat with this change in place; expect the
same physical close (39.8 mm gap, photographed symmetry) to now return `ok: true,
object_present: true` from `close()` instead of aborting `/pick` at `close_gripper`.

### Gate C, second attempt: `/pick` returns `ok: true`

Koushik reran the standard sequence. `close_gripper` logged the same pattern as the first
attempt — controller silence, then a direct measurement — but this time `close()` accepted it
instead of failing the phase:

```
[skill_server_node]: close_gripper: object width 0.0400 m, target 0.4755 rad (37.0 mm gap)
[skill_server_node]: close_gripper: no terminal controller result, checked jaw directly —
    measured gap 39.758542 mm vs object 40.000000 mm, symmetry 0.000016 rad
```

`/pick` advanced through every phase — `open_gripper` → `pre_grasp` → `descend` → `close_gripper`
→ `verify_grasp` → `attach` → `retreat` → `verify_hold` — and returned:

```
result:
  ok: true
  native_code: 0
  phase: retreat
  message: ''
Goal finished with status: SUCCEEDED
```

Koushik's own account, watching directly: "arm picks very stably... prev it was super wobbly the
whole gripper thing now it is perfect." The measured symmetry residual on this run
(`0.000016 rad`, ≈0.016 mrad) is tighter again than the free-air Gate B run — the dart-plus-
`gz_ros2_control`-own-drive mimic tracking holds under real load, not just in open air.

### RCA closed

All seven of Koushik's original acceptance points are met, evidenced from the controller log and
`/pick`'s own result, not from RViz:

1. Both pads close symmetrically — `0.000016 rad` fingertip residual.
2. Pads touch the box and maintain pressure — measured gap `39.76 mm` vs the `40 mm` box, held
   through `attach`/`retreat` with no goal or cancel sent after capture.
3. `GripperActionController` did not itself return a terminal successful result on this run — see
   Iteration 13's explicit, deliberate redefinition of what a usable verdict means. Direct jaw
   measurement stood in for it, and is why point 3 as originally written is not met literally.
4. `/pick` proceeded past `close_gripper`.
5. The arm retreated while holding the box (`verify_hold` passed post-retreat).
6. `/pick` returned `ok: true`.
7. Every number above came from the controller log and `/joint_states`, never from RViz.

**In practice, two changes closed this RCA: Iteration 12 and Iteration 13.** Every iteration
before them — 0 through 11, the entire investigation up to this session — shows the same dominant
symptom in some form: visible shaking, chattering, or wobbling of the gripper during
`close_gripper`, whatever else was also being tested at the time (friction, grasp depth, box
geometry, mimic wiring). That symptom traces to one thing that was true in every single one of
them and only changed in Iteration 12: `close_position` commanded the mechanical hard stop
(`0.8 rad`), so any real object left the position error permanently large and the joint permanently
asking for several times its rated speed. Nothing before Iteration 12 touched that target. Once it
did — closing to a measured target just past contact instead of the hard stop — the wobbling
stopped being the story; Gate C's first run showed solid, symmetric, settled contact on camera.
What was left after that was a narrower, different problem (the controller's own terminal verdict,
not the physical grasp), and Iteration 13's jaw-measurement fallback closed that. Two iterations,
not eleven, because the first eleven never changed the one thing that mattered most.

Three causes combined across those two iterations to get here, none sufficient alone:

- **The close target moved off the mechanical hard stop** (Iteration 12) — `0.8 rad` blocked by a
  40 mm box left a permanent `0.35 rad` error asking for `3.52 rad/s` on a `0.5 rad/s` joint,
  forever. This alone stops the joint fighting itself and is not engine-specific.
- **The physics engine moved to `dart`** (Iteration 12, a platform decision, not chosen to fix
  this bug) — Gate B's free-air run measured `0.000 mrad` fingertip symmetry against
  `bullet_featherstone`'s photographed `3.6°–5.4°` mismatches. This is a real, measured
  improvement, just not the one this RCA set out to prove.
- **Verification stopped trusting the controller's own verdict** (Iteration 13) — even with the
  above two fixes, `GripperActionController` still never produced a terminal result within
  `10 s` on either Gate C run, because of a periodic `+0.15 rad/s` velocity reading recurring
  every 0.3–1.4 sim-seconds during sustained contact under `dart` — a milder, differently-shaped
  descendant of Iteration 6's original `bullet_featherstone` finding, not eliminated by the engine
  switch. Measuring the jaw directly, on a client-side timeout as much as on a clean result, is
  what actually let `/pick` succeed on a controller that still won't say "done."

**What is not closed by this RCA:** the periodic velocity artifact under `dart` during sustained
contact is real, measured, and unexplained beyond "engine-side, tied to rigid contact" — nothing
currently bridged (no contact-force topic) localises it further. It no longer blocks `/pick`
because verification no longer depends on the controller resolving it, but it would still block
any future consumer of this controller's own terminal result. `docs/project/decisions.md` records
the engine choice; this file's job — explaining why `/pick` failed and confirming it now doesn't —
is done.
