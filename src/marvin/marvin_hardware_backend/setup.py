from glob import glob
import os

from setuptools import find_packages, setup


package_name = "marvin_hardware_backend"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"marvin_sdk": ["libMarvinSDK.so"]},
    include_package_data=True,
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml", "VENDOR_SDK_SHA256SUMS"],
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Asahel",
    maintainer_email="lichengmeng2001@foxmail.com",
    description="ROS 2 safety and transport boundary for Marvin hardware.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "marvin_hardware_bridge = "
            "marvin_hardware_backend.marvin_hardware_bridge:main",
            "marvin_backend_probe = "
            "marvin_hardware_backend.backend_probe:main",
            "marvin_feedback_probe = "
            "marvin_hardware_backend.feedback_probe:main",
            "marvin_hold_position = "
            "marvin_hardware_backend.hold_position_probe:main",
            "marvin_small_motion = "
            "marvin_hardware_backend.small_motion_probe:main",
        ],
    },
)
