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
    world_filename = "rover_world"  # auto sufpend .sdf
    model_filename = (
        "rover"  # auto sufpent .xacro, .urdf, _bridge.yaml, _ekf.yaml, .rviz
    )

    # --Prelim--
    # Get important package directory paths
    pkg_bringup = get_package_share_directory("robot_bringup")
    pkg_gazebo = get_package_share_directory("robot_gazebo")
    pkg_description = get_package_share_directory("robot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    passthrough_config_path = "/home/ubuntu/dev_ws/src/robot_bringup/config"

    # --Gazebo--
    # Get the path to the xacro and future urdf files from "description" package
    path_to_xacro = os.path.join(pkg_description, "model", model_filename + ".xacro")
    path_to_urdf = os.path.join(pkg_description, "model", model_filename + ".urdf")
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
            "gz_args": PathJoinSubstitution(
                [pkg_gazebo, "worlds", world_filename + ".sdf"]
            )
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

    # GZ -> Ros Robot tf2 state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
        arguments=[path_to_urdf],
    )

    # --Bridge--
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_bringup, "config", model_filename + "_bridge.yaml"
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
        arguments=[
            "-d",
            os.path.join(passthrough_config_path, model_filename + ".rviz"),
        ],
    )

    # --Basic Nodes--
    imu = Node(package="robot_app", executable="imu", output="screen")
    odom = Node(package="robot_app", executable="odom", output="screen")
    true_pose = Node(package="robot_app", executable="true_pose", output="screen")
    # --Intermediate Nodes--
    imuTuner = Node(package="robot_app", executable="imu_tuner", output="screen")
    poseEstimator = Node(
        package="robot_app", executable="pose_estimator", output="screen"
    )

    # --Advance Nodes--
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                pkg_bringup,
                "config",
                model_filename + "_ekf.yaml",
            ),
        ],
    )
    # --Post--
    return LaunchDescription(
        [
            gz_sim,
            spawn_robot,
            # robot_state_publisher,
            bridge,
            rviz2,
            imu,
            odom,
            true_pose,
            # imuTuner,
            # poseEstimator,
            # simPosePublisher,
            ekf,
        ]
    )
