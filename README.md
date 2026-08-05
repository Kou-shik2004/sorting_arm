# sorting_arm

Simulated UR5e and Robotiq 2F-85 sorting cell — ROS 2 Jazzy, MoveIt 2,
`ros2_control`, Gazebo Harmonic, BehaviorTree.CPP.

## Run the demo

Requires Docker only — no ROS install, no GPU.

```bash
git clone <this repository>
cd sorting_arm_ws
docker compose -f .docker/deploy/compose.yaml up --build
```

Then open `http://localhost:6080` in a browser. The arm sorts four cubes into
their colored bins; the container keeps running after the cycle finishes, so
`Ctrl-C` when you're done watching.

See [docs/container/deployment.md](docs/container/deployment.md) for how the
image works and why it's built the way it is.

## Developing

The project is built and run inside a container; see
[docs/container/README.md](docs/container/README.md) for the development
image, the devcontainer, and day-to-day commands.
