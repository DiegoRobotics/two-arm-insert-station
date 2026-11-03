# two-arm-insert-station
Two-arm heat-set insert station: a Yahboom JetCobot (myCobot 280) handles placement while an AR4 with a tool changer presses them in. Built on ROS 2 Humble + MoveIt, simulated in RViz (the AR4 half also builds on Jazzy).

## Repository layout

| Path | Contents |
| --- | --- |
| `JetCobot-Placer/ros_ws/` | ROS 2 workspace for the JetCobot placer arm |
| `JetCobot-Placer/ros_ws/src/jetcobot_placer/` | Placement logic (Python, `ament_python`) |
| `JetCobot-Placer/ros_ws/src/jetcobot_description/` | JetCobot URDF + meshes, copied from Yahboom (see its `ORIGIN.md`; not Apache-2.0) |
| `AR4-ToolChanger/` | AR4 MK3 tool changer: digital twin, tool-change sequence, Annin HMI 6.0 export and bridge ([README](AR4-ToolChanger/README.md)) |
| `AR4-ToolChanger/ros_ws/src/ar4tc_toolchanger/` | Tool-changer ROS 2 package (Python, `ament_python`) |
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

## Build the AR4 tool changer

See [AR4-ToolChanger/README.md](AR4-ToolChanger/README.md). In short, inside Ubuntu (22.04 Humble or 24.04 Jazzy):

```bash
bash AR4-ToolChanger/setup/setup_wsl_ros_humble.sh   # once; Jazzy: setup_wsl_ros.sh
bash AR4-ToolChanger/setup/build.sh                  # sync to ~/ar4tc_ws, build, test
ros2 launch ar4tc_toolchanger toolchange_demo.launch.py
```

The setup scripts clone the AR4 driver themselves at a pinned commit, so `deps.repos`
is not needed for this path.

## External dependencies

`deps.repos` lists the AR4 driver. Import it into the `src/` folder of the
workspace that needs it, so it lands in `src/external/` (ignored by git):

```bash
cd <workspace>            # e.g. a ros_ws folder
vcs import src < <repo-root>/deps.repos
```

`deps.repos` pins the AR4 driver to the same commit as
`AR4-ToolChanger/setup/setup_wsl_ros_humble.sh` (`b9adb2a`, `humble` branch).
For Jazzy use `4b88db1` (`main` branch), as in `setup_wsl_ros.sh`; keep the two
files in sync when bumping.

## License

Apache-2.0 (see `LICENSE`), except `jetcobot_description`, whose files are
Yahboom's and carry no declared license.
