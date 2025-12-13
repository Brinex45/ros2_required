from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    arm_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("arm_description"), "urdf", "arm.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": arm_description_content}

    arm_controllers = PathJoinSubstitution(
        [
            FindPackageShare("arm_bringup"),
            "config",
            "arm_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_bringup"), "rviz", "arm_urdf.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[arm_controllers],
        output="both",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager","--activate"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joints_controller", "--controller-manager", "/controller_manager","--activate"],
        output="screen",
    )

    delayed_arm_controller = TimerAction(
        period=2.0,
        actions=[arm_controller_spawner]
    )

    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )


    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delayed_arm_controller,
        delayed_rviz,
    ]

    return LaunchDescription(nodes)
