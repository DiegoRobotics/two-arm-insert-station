from glob import glob

from setuptools import setup

package_name = "ar4tc_toolchanger"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/srdf", glob("srdf/*.xacro")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gael",
    maintainer_email="gael10838@gmail.com",
    description="AR4 MK3 simulated tool changer",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "toolchange_demo = ar4tc_toolchanger.toolchange_demo:main",
            "goto_pose = ar4tc_toolchanger.goto_pose:main",
            "goto_pose_hmi = ar4tc_toolchanger.goto_pose_hmi:main",
            "hold_joints_hmi = ar4tc_toolchanger.hold_joints_hmi:main",
            "ik_benchmark = ar4tc_toolchanger.ik_benchmark:main",
            "annin_bridge = ar4tc_toolchanger.annin_bridge:main",
            "annin_emulator = ar4tc_toolchanger.annin_emulator:main",
            "export_hmi = ar4tc_toolchanger.export_hmi:main",
            "hmi_check = ar4tc_toolchanger.hmi_check:main",
        ],
    },
)
