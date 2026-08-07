# Tutorial: from clone to a sorted cube

One path, start to finish, with nothing to decide along the way. If you want the
options - GPU selection, `--dev`, individual launch files - see
[daily-use.md](./daily-use.md) once this has worked once.

The only requirement is Docker. There is nothing else to install, and nothing ROS-
related runs on your host at any point.

## 1. Clone and run

```bash
git clone https://github.com/Kou-shik2004/sorting_arm.git
cd sorting_arm
./scripts/demo
```

One command. It builds the Docker image, starts the container, builds the ROS
workspace inside it, and launches the full application, in that order, doing only
what hasn't been done yet.

## 2. Wait through the first run

The first run is the slow one. In order, you're watching:

1. The Docker image build - installing ROS 2 Jazzy, MoveIt, Gazebo's bridge
   packages, and this project's own toolchain. Several minutes, once.
2. The ROS workspace build inside the container - `colcon build`, compiling every
   package under `src/`. A minute or two.
3. The application launch - Gazebo and RViz starting, controllers coming up, the
   BehaviorTree beginning its one sorting cycle.

Every run after this one skips straight to step 3, because the image and the
workspace are both already built.

## 3. What you should see

Two windows open: Gazebo, showing the UR5e arm over a table with four cubes (two red,
two blue) and two trays, and RViz, showing the same scene from MoveIt's side. Once the
three controllers (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`)
report active, the arm moves to its observation pose and the cycle starts: look, pick
a cube by colour, place it in the matching tray, return, look again. It stops on its
own once all four cubes are sorted.

## 4. Stop

`Ctrl-C` in the terminal stops the launch - Gazebo and RViz close, the sorting cycle
ends - but the container itself keeps running underneath, so the next `./scripts/demo`
skips straight back to step 3. To run another cycle right away:

```bash
./scripts/demo
```

To tear the container down entirely:

```bash
./scripts/stop
```

## Where to go next

- [daily-use.md](./daily-use.md) - the individual scripts, when a rebuild is actually
  needed, and troubleshooting
- [devcontainer.md](./devcontainer.md) - attaching VS Code to the same container
- The README's [Running with or without a
  GPU](../../README.md#running-with-or-without-a-gpu) and [If the simulation is
  lagging](../../README.md#if-the-simulation-is-lagging) - if step 3 was slower than
  you'd like
- [dockerfile.md](./dockerfile.md) and [compose.md](./compose.md) - what's actually in
  the image and how the container is configured, for anyone changing either
