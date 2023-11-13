#!/usr/bin/env python3

from setuptools import find_packages, setup

package_name = "gazebo_video_monitor_utils"

setup(
    name=package_name,
    version="0.8.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Nick Lamprianidis",
    author_email="info@nlamprian.me",
    maintainer="Nick Lamprianidis",
    maintainer_email="info@nlamprian.me",
    description=(
        "Contains utility scripts that are meant to interact with the gazebo video monitor plugins."
    ),
    license="GPLv3",
    entry_points={
        "console_scripts": [f"wait_for_model = {package_name}.wait_for_model:main"]
    },
)
