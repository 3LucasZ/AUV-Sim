from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command


def generate_launch_description():
    # settings
    world_filename = "rover_world"
    model_filename = "sam_bot"
    init_pose = ["0", "0", "0.1", "0", "0", "0"]  # xyz-rpy

    # Get important package directory paths
    pkg_bringup = get_package_share_directory("robot_bringup")
    pkg_description = get_package_share_directory("robot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_gazebo = get_package_share_directory("robot_gazebo")
    passthrough_config_path = "/home/ubuntu/dev_ws/src/robot_bringup/config"

    # prepare urdf
    path_to_xacro = os.path.join(pkg_description, "model", model_filename + ".xacro")
    path_to_urdf = os.path.join(pkg_description, "model", model_filename + ".urdf")
    os.system("xacro " + str(path_to_xacro) + " > " + str(path_to_urdf))
    with open(path_to_urdf, "r") as infp:
        robot_description = infp.read()

    # robot state publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # joint state publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # rviz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(passthrough_config_path, model_filename + ".rviz"),
        ],
    )

    # spawn urdf in gz
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

    # gz
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

    # EKF
    robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_bringup, "config", model_filename + "_ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    odom = Node(package="robot_app", executable="odom", output="screen")

    # ret
    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            gz_sim,
            spawn_robot,
            bridge,
            robot_localization_node,
            rviz2,
            odom,
        ]
    )
