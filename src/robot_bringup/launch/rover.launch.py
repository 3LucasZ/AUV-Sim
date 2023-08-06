import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node


def generate_launch_description():
    # --Param--
    world_filename = "odometer"  # auto push_back .sdf
    init_pose = ["0", "0", "1", "0", "0", "0"]  # xyz-rpy
    model_filename = (
        "bluerov2_heavy"  # auto sufpent .xacro, .urdf, _bridge.yaml, _ekf.yaml, .rviz
    )
    dummy_filename = "dummy"
    ign_verbose_level = 1

    # --Prelim--
    # Get important package directory paths
    pkg_bringup = get_package_share_directory("robot_bringup")
    pkg_gazebo = get_package_share_directory("robot_gazebo")
    pkg_description = get_package_share_directory("robot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    passthrough_config_path = "/home/ubuntu/dev_ws/src/robot_bringup/config"
    # Get the path to the xacro and future urdf files from "description" package
    path_to_xacro = os.path.join(pkg_description, "model", model_filename + ".xacro")
    path_to_urdf = os.path.join(pkg_description, "model", model_filename + ".urdf")
    # Convert xacro to urdf
    os.system("xacro " + str(path_to_xacro) + " > " + str(path_to_urdf))
    # Get robot description
    with open(path_to_urdf, "r") as infp:
        robot_description = infp.read()
    # Get the path to dummy
    path_to_dummy = os.path.join(pkg_description, "model", dummy_filename + ".sdf")

    # --Start GZ sim--
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

    # --Start GZ sim (ALTERNATE WAY)--
    # ign gazebo -v 4 -r visualize_lidar.sdf
    path_to_world = os.path.join(pkg_gazebo, "worlds", world_filename + ".sdf")
    gz_sim_alt = ExecuteProcess(
        cmd=["ign", "gazebo", "-v", str(ign_verbose_level), "-r", path_to_world],
        output="screen",
    )

    # --Spawn URDF in the world--
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
    spawn_dummy = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="both",
        arguments=[
            "-file",
            path_to_dummy,
            "-x",
            init_pose[0],
            "-y",
            init_pose[1],
            "-z",
            init_pose[2],
        ],
    )

    # --Start tate Publisher--
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

    # --Start Bridge--
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

    # --Start RViz--
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(passthrough_config_path, model_filename + ".rviz"),
        ],
    )

    # --Start Basic Nodes--
    imu = Node(package="robot_app", executable="imu", output="screen")
    odom = Node(package="robot_app", executable="odom", output="screen")
    true_pose = Node(package="robot_app", executable="true_pose", output="screen")

    # --Start Intermediate Nodes--
    imuTuner = Node(package="robot_app", executable="imu_tuner", output="screen")
    poseEstimator = Node(
        package="robot_app", executable="pose_estimator", output="screen"
    )

    # --Start Advance Nodes--
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
            gz_sim_alt,
            # spawn_robot,
            # spawn_dummy,
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
