import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory("auv_bringup")
    pkg_project_gazebo = get_package_share_directory("auv_gazebo")
    pkg_project_description = get_package_share_directory("auv_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Load the urdf, xacro file from "description" package
    path_to_xacro = os.path.join(pkg_project_description, "model", "auv.xacro")
    path_to_urdf = os.path.join(pkg_project_description, "model", "auv.urdf")
    # Convert xacro to urdf
    os.system("xacro " + str(path_to_xacro) + " > " + str(path_to_urdf))
    # Get robot description
    with open(path_to_urdf, "r") as infp:
        robot_description = infp.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution([pkg_project_gazebo, "worlds", "world.sdf"])
        }.items(),
    )

    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="both",
        arguments=["-file", path_to_urdf, "-x", "0", "-y", "0", "-z", "3"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_project_bringup, "config", "rover.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,
            spawn_robot,
            # DeclareLaunchArgument(
            #     "rviz", default_value="true", description="Open RViz."
            # ),
            bridge,
            robot_state_publisher,
            # rviz,
        ]
    )
