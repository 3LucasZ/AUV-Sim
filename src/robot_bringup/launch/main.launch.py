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
    world_name = "rover_world"
    init_pose = ["0", "0", "2", "0", "0", "0"]  # xyz-rpy -> z is most important!
    # auto push_back .xacro, .urdf, _bridge.yaml, _ekf.yaml, .rviz
    model_name = "rover"
    model_nested = False  # Weird one, remember this!
    convert = True  # set convert to true if xacro -> urdf -> sdf
    ign_verbose_level = 2  # usually 2 or 3 is good
    # spawn_model, world, gz_simt MUST be enabled at bare minimum
    bridge_enabled = True
    robot_state_publisher_enabled = False
    rviz2_enabled = False

    # --Prelim--
    # Get important package directory paths
    pkg_bringup = get_package_share_directory("robot_bringup")
    pkg_gazebo = get_package_share_directory("robot_gazebo")
    pkg_description = get_package_share_directory("robot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    passthrough_config_path = "/home/ubuntu/dev_ws/src/robot_bringup/config"
    # Get the path to the xacro and future urdf files from "description" package
    model_file = model_name + "/model" if model_nested else model_name
    path_to_xacro = os.path.join(pkg_description, "models", model_file + ".xacro")
    path_to_urdf = os.path.join(pkg_description, "models", model_file + ".urdf")
    path_to_sdf = os.path.join(pkg_description, "models", model_file + ".sdf")
    # Convert xacro to urdf
    if convert:
        os.system("xacro " + str(path_to_xacro) + " > " + str(path_to_urdf))
    # Get robot description
    if convert:
        with open(path_to_urdf, "r") as infp:
            robot_description = infp.read()
    # Get path to world
    path_to_world = os.path.join(pkg_gazebo, "worlds", world_name + ".sdf")
    # init run list
    run = []

    # --Start GZ sim--
    # ign gazebo -v 4 -r visualize_lidar.sdf
    gz_sim = ExecuteProcess(
        cmd=[
            "ign",
            "gazebo",
            "-v",
            str(ign_verbose_level),
            path_to_world,
        ],  # use -r option to run on start
        output="screen",
    )
    run.append(gz_sim)

    # --Spawn model in the world--
    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="both",
        arguments=[
            "-file",
            path_to_urdf if convert else path_to_sdf,
            "-x",
            init_pose[0],
            "-y",
            init_pose[1],
            "-z",
            init_pose[2],
        ],
    )
    run.append(spawn_robot)

    # --Start state Publisher--
    # GZ -> Ros, Robot tf2 state publisher
    if convert:
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
        if robot_state_publisher_enabled:
            run.append(robot_state_publisher)

    # --Start Bridge--
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_bringup, "config", model_name + "_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )
    if bridge_enabled:
        run.append(bridge)

    # --Start RViz--
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(passthrough_config_path, model_name + ".rviz"),
        ],
    )
    if rviz2_enabled:
        run.append(rviz2)

    # --Start Basic Nodes--
    imu = Node(package="robot_app", executable="imu", output="screen")
    odom = Node(package="robot_app", executable="odom", output="screen")
    true_pose = Node(package="robot_app", executable="true_pose", output="screen")
    X150_status_pub = Node(
        package="robot_app", executable="X150_status_pub", output="screen"
    )
    run.append(X150_status_pub)
    range_bearing_pub = Node(
        package="robot_app", executable="range_bearing_pub", output="screen"
    )
    run.append(range_bearing_pub)

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
                model_name + "_ekf.yaml",
            ),
        ],
    )

    # --Post--
    return LaunchDescription(run)
