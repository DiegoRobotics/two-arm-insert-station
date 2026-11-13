# workcell_description

The table layout for the RViz simulation. It has two scenes:

| Scene | Launch | What it shows |
|---|---|---|
| Vial inspection (current) | `vial_inspection.launch.py` | SolidWorks `Layout_V3`, configuration `Vial_Inspection`: myCobot + gripper/camera, two vial trays, a static AR4 |
| Old placeholder layout | `display.launch.py` | Boxes only, from the earlier insert-station idea |

Tested on ROS 2 **Jazzy** (Linux Mint 22.3 / Ubuntu 24.04).

## Build the vial inspection scene

The scene uses two robot model packages from outside this workspace:
- `jetcobot_description`, from `JetCobot-Placer/ros_ws` in this repo;
- `annin_ar4_description`, from the AR4 driver repo, which is cloned into `src/external/` (git ignores that folder).

Run these from the repo root (`two-arm-insert-station/`).

**1. Build the JetCobot workspace first**

```bash
source /opt/ros/jazzy/setup.bash
cd JetCobot-Placer/ros_ws
colcon build --packages-select jetcobot_description
source install/setup.bash
cd ../..
```

**2. Clone the AR4 driver into this workspace** (one time only)

```bash
git clone https://github.com/ycheng517/ar4_ros_driver.git Workcell/ros_ws/src/external/ar4_ros_driver
git -C Workcell/ros_ws/src/external/ar4_ros_driver checkout 4b88db1c924515f7ce343105d46a637ecf2d7659
```

Use the commit that matches your ROS distro (the same pins as `deps.repos`):
- Jazzy: `4b88db1c924515f7ce343105d46a637ecf2d7659`, used and tested here;
- Humble: `b9adb2a65be5d4bc7c7f5e25c421238528aeeae5`, not tested with this scene.

**3. Build this workspace**

Use the same terminal, so the JetCobot workspace is still sourced.

```bash
cd Workcell/ros_ws
colcon build --packages-up-to workcell_description
source install/setup.bash
```

`--packages-up-to` builds only `annin_ar4_description` from the AR4 driver repo, not the whole driver.

**4. Launch**

```bash
ros2 launch workcell_description vial_inspection.launch.py
```

In a new terminal, source all three again, in this order:

```bash
source /opt/ros/jazzy/setup.bash
source ~/two-arm-insert-station/JetCobot-Placer/ros_ws/install/setup.bash
source ~/two-arm-insert-station/Workcell/ros_ws/install/setup.bash
```

**Check:** RViz shows the scene, and the slider window lists only `1_Joint` … `6_Joint`, the myCobot joints. The AR4 is frozen, so it has no sliders.

## Notes on the vial inspection scene

- **World frame = the SolidWorks layout frame:**
  - the origin is the front-left corner of the plate's **bottom** face, so the plate top is at Z = 6.35 mm;
  - +X runs along the long edge, +Y to the back, +Z up;
  - every handoff/CAD number applies as-is (mm → m).
- **Meshes:** the static meshes in `meshes/vial_inspection/` were exported from SolidWorks already in that frame (mm, scale 0.001).
- **myCobot sign flip:** J2, J3 and J4 in the URDF have the opposite sign to the CAD model. For example, CAD [0, 40, −60, −60, 0, 0] deg = URDF [0, −40, 60, 60, 0, 0].
- **AR4 model** (`urdf/ar4_static.xacro`):
  - frozen in the CAD pose [0, 55, 52, 0, −17, 0] deg;
  - the base enclosure is removed, so the J1 turret floats ~20 mm above the table;
  - meshes from `annin_ar4_description` (MIT).
- **Accuracy vs. the CAD:** the myCobot flange is within 2.7 mm at home and at the CAD pose. The AR4 tool face is within ~11 mm.
