from setuptools import setup

package_name = "robot_app"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="lucas.zheng@warriorlife.net",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_estimator = robot_app.pose_estimator:main",
            "imu_tuner = robot_app.imu_publisher:main",
            "imu = robot_app.imu:main",
            "odom = robot_app.odom:main",
            "true_pose = robot_app.true_pose:main",
            "X150_status_pub = robot_app.X150_status_pub:main",
            "range_bearing_pub = robot_app.range_bearing_pub:main",
        ],
    },
)
