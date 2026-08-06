# sorting_arm

A simulated UR5e arm and Robotiq 2F-85 gripper that sorts cubes by colour: ROS 2
Jazzy, MoveIt 2, `ros2_control`, Gazebo Harmonic, BehaviorTree.CPP, C++20.

Built for a Linux desktop or WSL, with Docker as the only host dependency.

## Run it

```bash
git clone <this repository>
cd sorting_arm_ws
./scripts/demo
```

Builds the image and the workspace on first run, then launches the full
application: Gazebo, a camera-based detector, and a BehaviorTree that picks up
four cubes and sorts them by colour into red and blue trays. `Ctrl-C` stops the
demo; the container keeps running.

## Everything else

```bash
./scripts/build   # build the image only
./scripts/start   # bring the container up, detached, idle
./scripts/enter   # shell into it
```

Once you're in with `./scripts/enter`, the workspace is already sourced.
`cbuild`, `ctest_all`, and `cdeps` are shell functions for building, testing, and
installing `rosdep`-declared dependencies.

## Launch files

`sorting_arm_bringup` holds every entry point, staged from a bare URDF up to the
full application:

| File | Brings up |
|---|---|
| `display.launch.xml` | URDF + RViz, no MoveIt, no Gazebo |
| `moveit.launch.xml` | Mock hardware + MoveIt planning, no Gazebo |
| `sim.launch.xml` | Gazebo + MoveIt |
| `skills.launch.xml` | Gazebo + the manipulation server, no executive |
| `perception.launch.xml` | Just the cube detector |
| `camera_validation.launch.xml` | Gazebo + MoveIt + the detector, so you can watch detection while jogging the arm |
| `app.launch.xml` | Everything, the one `./scripts/demo` runs |

Each package's own README covers its launch files and config in more detail; see
[docs/architecture.md](docs/architecture.md) for how the packages fit together.

## Documentation

- [docs/architecture.md](docs/architecture.md) - the package boundaries and the
  detect-pick-place-home cycle
- [docs/container.md](docs/container.md) - the image, the compose files, and how
  to add your own persistent state
- [docs/rca/](docs/rca/) - short write-ups of real failures and how they were
  fixed

## License

MIT, see [LICENSE](LICENSE).
