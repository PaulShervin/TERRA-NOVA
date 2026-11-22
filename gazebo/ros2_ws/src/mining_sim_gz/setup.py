import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mining_sim_gz"


def tree(dst, src):
    """(install dir, [files]) pairs for a whole directory tree, keeping the sub-folder layout (models/<name>/meshes/...)."""
    out = {}
    for path in glob(os.path.join(src, "**", "*"), recursive=True):
        if os.path.isfile(path):
            out.setdefault(os.path.join("share", package_name, dst, os.path.relpath(os.path.dirname(path), src)).rstrip("/."), []).append(path)
    return list(out.items())


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        *tree("worlds", "worlds"),
        *tree("models", "models"),
        *tree("config", "config"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="user@example.com",
    description="Gazebo port of the industrial / cave mapping simulator",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_noise_node = mining_sim_gz.odom_noise_node:main",
            "scan_slam_node = mining_sim_gz.scan_slam_node:main",
            "explorer_node = mining_sim_gz.explorer_node:main",
            "segmentation_node = mining_sim_gz.segmentation_node:main",
        ],
    },
)
