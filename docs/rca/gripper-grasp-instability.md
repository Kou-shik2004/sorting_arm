# RCA: Gripper grasp was unstable in simulation

## Problem

The simulated Robotiq 2F-85 gripper did not grasp cubes reliably across most of this
investigation. The same `/pick` sequence could succeed, abort during `close_gripper`,
or lift the cube and drop it during retreat, with no code change between
identical-looking runs. `/home` succeeded consistently throughout; the failure was
always downstream of the grasp sequence starting.

## Expected Behavior

A `/pick` action closes the gripper on a cube and holds it through retreat, with every
one of these provable from the controller log and `/joint_states`, never from
watching RViz:

1. Both pads close symmetrically.
2. The pads touch the cube and hold pressure through attach and retreat.
3. The controller (or a direct measurement standing in for it) returns a usable
   verdict, not a timeout.
4. `/pick` proceeds past `close_gripper`.
5. The arm retreats while still holding the cube.
6. `/pick` returns `ok: true`.

## Root Cause

Three causes combined, and none of them alone explained the failure:

1. **The close target sat on the mechanical hard stop.** `close_position: 0.8` is the
   knuckle's URDF upper limit, but a 40 mm cube stops the joint at `0.4484 rad`.
   Commanding `0.8` against that left a permanent `0.35 rad` position error.
   `gz_ros2_control`'s position-to-velocity adapter has no upper bound
   (`velocity = position_proportional_gain × error × update_rate`, gain `0.1`, rate
   `100 Hz`), so that error turned into a continuous `3.52 rad/s` demand on a joint
   rated `0.5 rad/s`, the shaking and chattering seen on every earlier run.

2. **The physics engine mattered more than expected.** Gazebo Harmonic converts a
   URDF `<mimic>` tag into a real SDF mimic constraint; `bullet_featherstone` exports
   that constraint, `dart` does not. `gz_ros2_control` runs its own mimic-drive loop
   regardless, at ten times the driven joint's stiffness. Under `bullet_featherstone`,
   a follower joint listed in `<ros2_control>` ends up governed twice. Switching to
   `dart`, Gazebo Harmonic's default engine and not a workaround, removed the second
   governor and measurably tightened fingertip symmetry, from photographed
   `3.6-5.4°` mismatches down to `0.000 mrad`.

3. **The controller's own "stalled" verdict didn't mean what it looked like.**
   `GripperActionController::update()`, verified against the installed Jazzy header,
   clears the active goal on `stalled` or `reached_goal`, but keeps driving the joint
   toward the same target every cycle regardless. Only a cancel calls
   `set_hold_position()` and actually freezes the target. Every `stalled: true` read
   before this was found meant "the controller stopped watching," not "the gripper
   stopped moving." A separate, real artifact made this worse: during sustained
   contact under `dart`, the driven joint's reported velocity spiked to roughly
   `+0.15 rad/s` every 0.3-1.4 seconds, and each spike reset the controller's
   quiet-window timer. That spike is measured and still not fully explained. It looks
   consistent with a periodic contact-solver re-evaluation, but nothing currently
   bridged localises it further than "engine-side, tied to sustained rigid contact."

## Fix

Two changes closed the investigation, and closing it meant accepting that the
controller's own verdict couldn't be trusted on its own:

- **Close to a derived target, never the hard stop.** `close(object_width_m)` now
  targets the knuckle angle for `object_width_m - squeeze_depth_m`, a window
  `1.10-5.52 mm` past contact, instead of the joint's mechanical limit.
- **Fall back to direct measurement when the controller times out.** When
  `send_goal` reports `"gripper result timed out"`, the code measures the jaw
  directly from `/joint_states`, gap and fingertip symmetry, instead of treating the
  timeout as proof nothing was caught. A timeout doesn't reopen the jaw: a cancel only
  freezes the target wherever it already was.

Two later runtime follow-ups adjusted the design further without touching either
principle above:

- The one-shot close was replaced with sequential absolute `GripperCommand` goals,
  each starting from the freshly measured position and advancing by at most
  `0.03 rad`. A controller-reported stall is now only an obstruction *candidate*.
  Pick immediately holds at the measured position, runs a `0.02 m` validation lift,
  and sends one small retention probe before MoveIt attachment.
- After a cancelled close froze the gripper mid-contact and a later `Home` failed
  collision checking against the still-attached box, the direct-close design went
  back to trusting the controller's native `stalled` result, with
  `stall_velocity_threshold=0.02` and a `stall_timeout=0.2` quiet window. This is a
  controller-completion correction, not a return to jaw measurement.

## Verification

- Free-air raw-goal test (Gate B): `0.000 mrad` fingertip symmetry under `dart`,
  against earlier `3.6-5.4°` mismatches under `bullet_featherstone`.
- Full `/pick` test (Gate C), two consecutive runs (2026-08-01): pads photographed
  flush and centred, measured gap `39.76 mm` against the `40 mm` box, held through
  attach and retreat with no goal or cancel sent after capture. `/pick` returned
  `ok: true` through `attach`/`retreat`/`verify_hold` both times.
- After the bounded-close follow-up, I ran the full `sequence_demo` and the demo and
  grasp completed cleanly. This is one runtime observation, not deterministic-test
  evidence: an isolated empty-close result and a recorded six-joint trace were never
  separately captured.
- After the direct-close follow-up, I confirmed a successful full sequence after
  restarting the application. An isolated empty-close result remains unreported
  there too.

## Prevention

- The active close path requires the controller's own terminal `stalled=true`
  result; the `0.02 rad/s` quiet band and `0.2 s` window are a runtime gate, not a
  tuning knob. Don't reintroduce jaw measurement or timeout-as-contact logic without
  a reason this file didn't already rule out.
- The `dart`-side periodic velocity artifact during sustained contact is still open.
  If a future failure looks like this one, check the controller's raw velocity
  readback before assuming a new physical cause.

## Lessons Learned

A controller's `stalled: true` is not proof the joint stopped moving; it only proves
the controller stopped watching. That single wrong assumption produced most of the
dead ends below, and none of them were fixed by tuning:

| Tried | What happened | Why it didn't work |
|---|---|---|
| Widen `stall_velocity_threshold` | One clean `stalled: true`, but the cube still dropped in retreat | Fixed a timing symptom, not the grasp itself |
| Lower fingertip friction `100000 → 5` | No change | The pads were 15.1 mm from the cube on every run that "tested" this; friction was never actually exercised |
| Match upstream `robotiq_description` exactly (unlimited follower joints, inline surface friction) | Reproduced the identical failure signature | Reintroduced a defect an earlier fix had already removed |
| Copy a reference UR stack's `<ros2_control>` shape (Humble/Fortress, `ign_ros2_control`) | Did not reproduce that stack's result | That stack only ever had one mimic-joint governor to begin with, and its own gripper close never verified a grasp at all |
| Open `grasp_offset_m` for more table clearance | Made pre-grasp planning worse, all five candidates failed | Reverted; reproducible at the original value |
| Raise the knuckle's effort limit or proportional gain | No usable improvement | The motor already saturates against the object once contact stops it short of the target |

Two broader lessons:

- A physics-engine switch chosen for an unrelated reason, Gazebo Harmonic's
  supported default, turned out to be the dominant fix for a problem that looked
  purely like a controller bug.
- Tuning a single threshold cannot separate settled contact from engine noise when
  both are the same order of magnitude. Measure the noise floor before tuning
  against it.

## References

- `docs/project/decisions.md` D24: the `dart` physics engine choice.
- Reproduction: launch the simulation (`sim.launch.xml gui:=true` or
  `app.launch.xml`), start `skill_server_node` with `use_sim_time:=true`, then
  `/sync_objects`, `/home`, and `/pick` for a cube with feedback enabled.
