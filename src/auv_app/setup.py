from setuptools import setup

package_name = "auv_app"

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
            "pose_estimator = auv_app.pose_estimator:main",
            "imu_corrector = auv_app.imu_corrector:main",
        ],
    },
)
