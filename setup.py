from setuptools import setup

package_name = "napari_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "napari"],
    zip_safe=True,
    maintainer="Alan Liddell",
    maintainer_email="alan.c.liddell@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "viewer = napari_ros.viewer:main",
            "stream_viewer = napari_ros.stream_viewer:main",
        ],
    },
)
