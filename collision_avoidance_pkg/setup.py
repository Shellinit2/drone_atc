from setuptools import find_packages, setup

package_name = "collision_avoidance_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ShellInit2",
    maintainer_email="shellinit2@itdoesnt@exist",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "manager = collision_avoidance_pkg.collision_manager_node:main",
            "pilot_0 = collision_avoidance_pkg.offboard_control_node:main_0",
            "pilot_1 = collision_avoidance_pkg.offboard_control_node:main_1",
        ],
    },
)
