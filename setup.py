from setuptools import setup

package_name = "turtle_navigator"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gopikrishnan K",
    maintainer_email="gopikrishnank99@gmail.com",
    description="Turtlesim autonomous navigation with ROS2",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "move_turtle = turtle_navigator.move_turtle:main",
        ],
    },
)
