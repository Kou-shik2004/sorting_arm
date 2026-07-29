# sorting_arm

`sorting_arm` is a simulated manipulation cell built around a UR5e, a Robotiq
2F-85, ROS 2 Jazzy, MoveIt 2, `ros2_control`, and Gazebo Harmonic. The project is
being developed toward a reproducible sorting cycle with explicit manipulation
skills, task orchestration, failure handling, and wrist-camera perception.

## Current state

The robot description, MoveIt configuration, controllers, Gazebo world, unified
bringup, and development container are established. The manipulation skills layer
is currently experimental; reusable Pick, Place, and Home actions, the executive,
CI, and perception remain under implementation.

## Development environment

Build and start the development container:

```bash
docker compose up --build -d
docker compose exec dev bash
```

The development service intentionally stays alive for interactive work. Starting
it does not yet launch the sorting application.

## Project documentation

- [Project overview](docs/project/overview.md)
- [Architecture](docs/project/architecture.md)
- [Architectural decisions](docs/project/decisions.md)
- [Development workflow](docs/project/development-workflow.md)
- [Context management](docs/project/context-management.md)
- [Implementation roadmap](docs/plan/roadmap.md)
- [Continuous integration plan](docs/plan/continuous-integration.md)
