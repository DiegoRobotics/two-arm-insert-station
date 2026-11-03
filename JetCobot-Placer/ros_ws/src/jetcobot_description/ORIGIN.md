# Origin

Copied from Yahboom's JetCobot image (~/jetcobot_colcon_ws/src/jetcobot_description).
URDF and meshes are Yahboom's work. Yahboom did not declare a license
(package.xml says "TODO: License declaration"); this repo's Apache-2.0
license does NOT apply to these files.

Changes from the original:
- removed urdf/.ipynb_checkpoints (unrelated DOFBOT autosave).
- package.xml: added exec_depends needed by launch/display.launch.py; filled in
  description and maintainer; license marked Unknown (not declared by Yahboom).
- launch/display.launch.py: fixed "omnicar_description" copy-paste in the
  `model` argument description; removed unused `default_urdf_path` variable.
