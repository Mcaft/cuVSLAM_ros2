from glob import glob
import os

from setuptools import setup

package_name = "cuvslam_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hcfly",
    maintainer_email="hcfly@vision02.local",
    description="cuVSLAM VIO for RealSense D435i, ROS 2 bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cuvslam_bridge_node = cuvslam_bridge.bridge_node:main",
        ],
    },
)
