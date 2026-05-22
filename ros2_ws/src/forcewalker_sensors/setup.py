from setuptools import find_packages, setup

package_name = "forcewalker_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/teensy_force_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Force Walker",
    maintainer_email="soar@example.com",
    description="Force Walker Teensy force and IMU serial bridge.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "teensy_force_bridge = forcewalker_sensors.teensy_force_bridge:main",
        ],
    },
)
