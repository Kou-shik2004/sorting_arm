# Continuous integration (CI)

## Why this project needs CI

CI means GitHub checks the project automatically.

Code can work in Koushik's development container and still fail on another machine.
For example, the container may already contain a missing dependency or an old build
file may hide a bad CMake install rule.

CI starts with a clean checkout and performs the same checks every time. This helps
us find problems before they are merged.

A green CI result currently means:

- the declared dependencies are available;
- the source passes the configured format and syntax checks;
- all four ROS packages build using clean output directories;
- the robot xacro files produce valid URDF;
- the required `world`, `tcp`, and MoveIt `arm` definitions are present; and
- every test registered with `colcon` passes.

A green result does **not** mean the robot moves correctly in Gazebo. The current CI
does not start Gazebo, MoveIt, controllers, RViz, or the sorting application.

## What we have now

| Level | Status | What it checks |
|---|---|---|
| Level 1 — workspace | Implemented | Builds and checks the ROS workspace in a small CI image. |
| Level 2 — development image | Implemented | Builds the real development image and checks its basic setup. |
| Level 3 — simulation | Future | Will run one small simulation without a graphical window. |

Levels 1 and 2 are separate because they answer different questions. Level 1 checks
the project source. Level 2 checks the larger image used for development.

## Level 1 — workspace checks

The workflow is `.github/workflows/ci.yml`.

### When it runs

- `push` runs it after code is pushed to GitHub.
- `pull_request` runs it for a pull request.
- `workflow_dispatch` lets Koushik run it manually from GitHub.

The manual run has a `clean_image` option:

- `type: boolean` makes it a true-or-false checkbox.
- `required: false` means the checkbox is optional. The workflow can start without
  Koushik selecting it.
- `default: false` means Docker normally uses cached image layers. This makes most
  runs faster.
- When `clean_image` is `true`, Docker does not use cached layers. This checks that
  the image can still build completely from scratch.

Here, `required: false` only describes the manual checkbox. It is not the same as
making CI a required pull-request check in GitHub.

### Why the permission is read-only

The workflow uses:

```yaml
permissions:
  contents: read
```

The job only needs to read the repository so it can build and test the project. It
does not need to change files, create releases, or push code.

Read-only permission limits what the workflow token can do. If a future step has a
security problem, that token cannot write to the repository.

The checkout step also uses:

```yaml
persist-credentials: false
```

The later steps do not need the GitHub token, so checkout does not leave it in the
local Git configuration.

### How the job works

1. `actions/checkout@v6` downloads the repository.
2. `docker/setup-buildx-action@v4` prepares the Docker builder and layer cache.
3. `docker/build-push-action@v7` builds `.docker/Dockerfile.ci`.
4. The workflow loads the image into the runner and runs `.docker/ci.sh`.

The job uses Ubuntu 24.04 because ROS 2 Jazzy uses Ubuntu 24.04.

The 60-minute timeout stops a build that becomes stuck.

The image is loaded only for this job. `push: false` prevents the workflow from
publishing it to a registry.

The image tag contains `${{ github.sha }}`, which is the current Git commit ID. This
stops the job from confusing the image with one built for another commit.

Docker image layers are cached to make later runs faster. The cache uses the
`sorting-arm-ci` name, so it is not mixed with the development-image cache. A cache
upload failure does not fail CI because the cache only saves time. It does not prove
that the project works.

### Why the source is read-only

The workflow gives the source to the container as:

```text
${GITHUB_WORKSPACE}:/workspace:ro
```

The `:ro` part means read-only. The checks can read the source but cannot change it.

Build files, installed files, logs, expanded URDF, and test results go into a new
temporary directory. This makes the job check the original source instead of
changing a file while checking it.

`docker run --rm` removes the container when the job finishes. If `.docker/ci.sh`
returns an error, the workflow fails.

## The small CI image

`.docker/Dockerfile.ci` starts from `ros:jazzy-ros-base-noble`.

This base image has the ROS 2 tools needed to build the project, but it does not have
the full desktop and simulation software. Level 1 does not run Gazebo or RViz, so
adding those tools would only make the image larger and slower.

The image installs:

- command-line tools from `.docker/packages/ci.txt`; and
- ROS packages from `.docker/packages/ci-ros.txt`.

The image runs `.docker/ci.sh` automatically when its container starts.

The normal development image remains separate. It uses `jazzy-desktop-full` because
Koushik needs graphical and simulation tools during development.

## What `.docker/ci.sh` checks

The script performs these checks in order:

1. It runs `rosdep check` for the build and test dependencies declared in each
   `package.xml`. This catches a declared dependency that is missing from the image.
2. It checks C and C++ formatting, Bash syntax, CMake style, Python syntax, and XML
   structure. These checks find simple source errors before the build.
3. It builds all four ROS packages in `Release` mode using new build, install, and
   log directories.
4. It expands the plain, mock-hardware, simulation, and MoveIt xacro entry files.
   `check_urdf` checks each resulting robot model.
5. It checks that the plain robot contains the `world` and `tcp` links.
6. It checks that the MoveIt `arm` group goes from `base_link` to `tcp`.
7. It runs every test registered with `colcon`.
8. It prints the detailed test result even when a test fails.

The script keeps the original test result before printing the report. Printing the
report must not hide a failed test. The report command also writes its own log into
the temporary directory because the source directory is read-only.

The script returns status 2 when it is used incorrectly, such as when the workspace
has no `src/` directory or root `.clang-format` file.

### Why CI does not use `--symlink-install`

During development, `--symlink-install` lets some installed files point back to the
source tree. This is useful because source changes can appear without copying the
files again.

CI uses a normal copied install. This checks that every package's CMake install rules
actually install the required files. A missing install rule should fail CI instead
of silently reading the file from the source tree.

## Level 2 — development-image checks

The workflow is `.github/workflows/development-image.yml`.

The development image is much larger than the Level 1 image. Rebuilding it for every
C++ change would waste time without checking anything new.

It runs when a push or pull request changes development-environment files. These
include the development Dockerfile, package lists, scripts, underlay list,
devcontainer files, Compose file, `.dockerignore`, package manifests, and the
workflow itself.

Koushik can also run it manually. Its `clean_image` checkbox works like the Level 1
checkbox.

This workflow:

1. checks that the devcontainer and Compose configuration can be read;
2. builds the real `.docker/Dockerfile`;
3. checks that the default user is the non-root user `kratos`;
4. checks that the entrypoint loads ROS 2 Jazzy;
5. checks that `behaviortree_ros2` and `btcpp_ros2_interfaces` come from
   `/opt/underlay/install`;
6. checks all dependencies declared in the current `package.xml` files; and
7. runs the same `.docker/ci.sh` checks used by Level 1.

It has read-only repository permission, does not keep checkout credentials, and does
not publish the image.

Its timeout is 120 minutes because the full development image takes longer to build.
Its Docker cache is separate from the Level 1 cache.

This workflow does not start Docker Compose, Gazebo, MoveIt, controllers, RViz, GPU
access, or display forwarding. It checks the image, not the running sorting cell.

## Level 3 — future simulation check

Level 3 will run one small simulation without a graphical window. It will be added
only after the project has a reliable way to:

- tell when Gazebo, controllers, MoveIt, and the application are ready;
- run one small action with a result we can check;
- stop after a fixed maximum time;
- save useful logs when it fails; and
- shut down every started process.

Simulation is slower and can change with startup and physics timing. We should not
make it a required check until the same scenario works reliably on repeated local
runs.

## What is not included yet

- `industrial_ci` is not used. The project supports only ROS 2 Jazzy on Ubuntu
  24.04, so the direct Dockerfile and script are easier to understand.
- There is no operating-system or compiler matrix because the project currently
  supports one environment.
- There is no GUI or GPU job because the current checks do not open graphical tools.
- There is no mock-hardware MoveIt job because its action tests do not exist yet.
- There is no Gazebo job because the small, reliable simulation check does not exist
  yet.
- There is no runtime image because the final launch command and runtime dependency
  list are not stable yet.
- Test-result files are not uploaded because the repository currently has no
  registered tests. The available result is printed in the workflow log.

## What has been checked so far

Local checks completed on 2026-07-30:

- dependency, format, syntax, xacro, URDF, frame, and MoveIt group checks passed;
- all four packages built in `Release` mode using clean output directories;
- `colcon` ran every registered test and printed the detailed result;
- an invalid workspace returned status 2 as expected; and
- both workflow files passed `actionlint` 1.7.12.

The repository currently has zero registered tests. The current result is therefore
zero failed tests out of zero tests. It does not prove application behavior.

On 2026-07-30, Koushik built the Level 1 image on the host without cached layers.
The first container run found a deleted source file that was still listed in
`sorting_arm_skills/CMakeLists.txt`. After removing that old target, the clean
workspace build passed on the host. The next command then found that
`colcon test-result` was trying to create a log inside the read-only source mount.
After the script was fixed, the host container run completed successfully: all four
packages built and `colcon` reported zero errors and zero failures. The repository
still has zero registered tests. The GitHub-hosted workflows are still pending.

## What Koushik still needs to check

1. Push the changes and confirm that the Level 1 GitHub workflow passes.
2. Confirm that the `.docker/ci.sh` change starts the development-image workflow.
3. Manually run the development-image workflow with `clean_image` selected.
4. Confirm that an intentional failure produces a red Level 1 check and a useful
   error log.

After these checks pass, Koushik can decide whether Level 1 should be a required
GitHub branch-protection check. A required check blocks a pull request from merging
when CI fails.
