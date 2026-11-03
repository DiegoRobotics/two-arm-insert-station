# two-arm-insert-station
Two-arm heat-set insert station: a Yahboom JetCobot (myCobot 280) handles placement while an AR4 with a tool changer presses them in. Built on ROS2 Humble + MoveIt, simulated in RViz.

## Repository layout

| Path | Contents |
| --- | --- |
| `JetCobot-Placer/ros_ws/` | ROS 2 workspace for the JetCobot placer arm |
| `JetCobot-Placer/ros_ws/src/jetcobot_placer/` | Placement logic (Python, `ament_python`) |
| `JetCobot-Placer/ros_ws/src/jetcobot_description/` | JetCobot URDF + meshes, copied from Yahboom (see its `ORIGIN.md`; not Apache-2.0) |
| `deps.repos` | External repositories (AR4 driver) to import with `vcs` |

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble
- `python3-colcon-common-extensions`, `python3-rosdep`, `python3-vcstool`

## Build the JetCobot workspace

```bash
cd JetCobot-Placer/ros_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

View the arm in RViz:

```bash
ros2 launch jetcobot_description display.launch.py
```

## External dependencies

`deps.repos` lists the AR4 driver. Import it into the `src/` folder of the
workspace that needs it, so it lands in `src/external/` (ignored by git):

```bash
cd <workspace>            # e.g. a ros_ws folder
vcs import src < <repo-root>/deps.repos
```

The AR4 driver branch in `deps.repos` must match the ROS distro of the
machine running the AR4 (`humble` for Humble, `main` for Jazzy).

## License

Apache-2.0 (see `LICENSE`), except `jetcobot_description`, whose files are
Yahboom's and carry no declared license.
