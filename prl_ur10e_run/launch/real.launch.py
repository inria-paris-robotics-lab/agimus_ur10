# import logging
# logging.root.setLevel(logging.DEBUG)
############################################################################################################
# Description: This file is used to connect the real workbench with the ros2 environment. 
#              The launch file starts the following nodes:
#               - Controller Manager
#               - Controller Spawners
#               - Dashboard Client
#               - Robot State Helper
#               - URScript Interface
#               - RViz
#               - Joint State Publisher
#              The launch file also includes the following launch files:            
#               - ur10e_controllers.launch.py
#               - ur10e_gripper_controllers.launch.py
#               - sensors.launch.py
# Arguments:
#               - robot_ip: IP address of the robot
#               - activate_joint_controller: Activate wanted joint controller.
#               - launch_rviz: Launch RViz
#               - launch_dashboard_client: Launch Dashboard Client
#               - launch_urscript_interface: Launch URScript Interface
#               - left_kinematics_file: Left robot kinematics file
#               - right_kinematics_file: Right robot kinematics file
#               - update_rate_config_file: Update rate configuration file
#               - launch_moveit: Launch MoveIt
#               - activate_cameras: Activate cameras
# Usage:
#               $ ros2 launch prl_ur10e_run real.launch.py robot_ip:=<ip>
############################################################################################################
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
import yaml


def launch_setup(context):
    # Setup file
    controllers_file = PathJoinSubstitution([FindPackageShare("prl_ur10e_control"), "config", "ur10e_controller.yaml"])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("ur_description"), "rviz", "view_robot.rviz"])
    kinematics_file = LaunchConfiguration("kinematics_file")
    config_file = os.path.join(get_package_share_directory('prl_ur10e_robot_configuration'), 'config', 'standard_setup.yaml')
    config_path = Path(config_file) 
    with config_path.open('r') as setup_file:
        config = yaml.safe_load(setup_file)
    robot_ip = config.get('arm')['network']['ip']
    config_controller_path = os.path.join(get_package_share_directory('prl_ur10e_robot_configuration'), 'config', 'controller_setup.yaml')
    with open(config_controller_path, 'r') as setup_file:
        config_controller = yaml.safe_load(setup_file)
    all_controllers = config_controller.get('controllers')
    activate_controllers = all_controllers.get('active_controllers', [])
    loaded_controllers = all_controllers.get('inactive_controllers', [])
    # Generals Arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    launch_urscript_interface = LaunchConfiguration("launch_urscript_interface")
    activate_cameras = LaunchConfiguration("activate_cameras")
    launch_moveit = LaunchConfiguration("launch_moveit")

    ###### Calibration ######
    calib = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('ur_calibration'),
            'launch',
            'calibration_correction.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'target_filename': kinematics_file,
        }.items(),
    )

    ###### Controllers ######
    # Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file),
        ],
        output="screen",
    )
    
    # Spawn controllers
    active_controllers = ",".join(["io_and_status_controller"]) + "," + ",".join(activate_controllers)
    
    inactive_controllers = ",".join(loaded_controllers)

    print("Active controllers: ", active_controllers)
    print("Inactive controllers: ", inactive_controllers)
    controller_spawners = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('prl_ur10e_control'),
            'launch',
            'ur10e_controllers.launch.py',
            ])
        ]),
        launch_arguments={
            'active_controller': active_controllers,
            'loaded_controllers': inactive_controllers,
        }.items(),
    )

    ###### UR Driver Side ######

    # Dashboard client node, it enables us to make everything like we have the robot dashboard
    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(launch_dashboard_client),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    # The robot_state_helper node can be used to start the robot, release the brakes, and (re-)start the program through an action call.
    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        parameters=[
            {"headless_mode": True},
        ],
    )

    # The URScript interface node is used to send URScript commands to the robot controller directly 
    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        name="urscript_interface",
        output="screen",
        condition=IfCondition(launch_urscript_interface),
    )

    ###### RViz ######
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    ###### Joint state Publisher ######

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # Find the xacro executable
            " ", 
            PathJoinSubstitution([FindPackageShare("prl_ur10e_description"), "urdf", "ur10e_complete_setup.urdf.xacro"]),
            " ",
            "gz_sim:=",
            "false",
            " ",
            "real_robot:=",
            "true",
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }
    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ###### Gripper ######

    gripper_controller = config.get('arm')['gripper_controller']

    gripper_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('prl_ur10e_control'),
                'launch',
                'ur10e_gripper_controllers.launch.py',
            ])
        ]),
        launch_arguments=[
            ('gripper_controller', gripper_controller),
            ('prefix', ''),
        ],
    )

    ###### Sensors ######
    # Launch the force/torque driver node
    # bota_driver_node = Node(
    #     package='bota_driver',
    #     executable='bota_driver_node',
    #     output='screen',
    #     parameters=[
    #         {'config_file': os.path.join(
    #             get_package_share_directory('prl_ur10e_robot_configuration'),
    #             'config',
    #             'ft_sensor_config.json'
    #         )},
    #         {'output_rate': 500},
    #         {'bota_driver_node_name' : "bota_ft_sensor"},
    #         {'bota_ft_sensor_link_name':"bota_ft_sensor"}
    #     ]
    # )

    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #         FindPackageShare('prl_ur5_control'),
    #         'launch',
    #         'sensors.launch.py',
    #         ])
    #     ]),
    #     condition=IfCondition(activate_cameras),
    # )

    ###### MoveIt ######
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("prl_ur10e_moveit"),
                "launch",
                "start_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
        condition=IfCondition(launch_moveit),
    )

    return [
        calib,
        control_node,
        controller_spawners,
        gripper_controller,
        dashboard_client_node,
        urscript_interface,
        rsp,
        rviz_node,
        # bota_driver_node,
        # camera_launch,
        moveit_launch,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("prl_ur10e_robot_configuration"),
                        "config",
                        "kinematics",
                    ]
                ),
                "/default_kinematics.yaml",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_cameras",
            default_value="false",
            description="Activate cameras?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="false",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_urscript_interface",
            default_value="false",
            description="Launch URScript Interface?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_robot_driver"),
                        "config",
                    ]
                ),
                "/ur10e_update_rate.yaml",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="launch_moveit",
            default_value="true",
            description="Launch MoveIt ?",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])