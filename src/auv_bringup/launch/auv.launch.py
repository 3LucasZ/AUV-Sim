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
    # --Param--
    init_pose = ["0", "0", "1", "0", "0", "0"]  # xyz-rpy

    # --Prelim--
    # Get important package directory paths
    pkg_project_bringup = get_package_share_directory("auv_bringup")
    pkg_project_gazebo = get_package_share_directory("auv_gazebo")
    pkg_project_description = get_package_share_directory("auv_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # --Gazebo--
    # Get the path to the xacro and future urdf files from "description" package
    path_to_xacro = os.path.join(pkg_project_description, "model", "auv.xacro")
    path_to_urdf = os.path.join(pkg_project_description, "model", "auv.urdf")
    # Convert xacro to urdf
    os.system("xacro " + str(path_to_xacro) + " > " + str(path_to_urdf))
    # Get robot description
    with open(path_to_urdf, "r") as infp:
        robot_description = infp.read()
    # Configure gz
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution([pkg_project_gazebo, "worlds", "world.sdf"])
        }.items(),
    )
    # Spawn urdf in gz
    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="both",
        arguments=[
            "-file",
            path_to_urdf,
            "-x",
            init_pose[0],
            "-y",
            init_pose[1],
            "-z",
            init_pose[2],
        ],
    )
    # Robot state publisher
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

    # --Bridge--
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

    # --RViz--
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_project_bringup, "config", "default.rviz")],
    )

    # --Nodes--
    imuCorrector = Node(package="auv_app", executable="imu_corrector", output="screen")
    poseEstimator = Node(
        package="auv_app", executable="pose_estimator", output="screen"
    )
    simPosePublisher = Node(package="auv_app", executable="sim_pose_publisher")

    # --Post--
    return LaunchDescription(
        [
            gz_sim,
            spawn_robot,
            robot_state_publisher,
            bridge,
            rviz2,
            # imuCorrector,
            # poseEstimator,
            simPosePublisher,
        ]
    )
