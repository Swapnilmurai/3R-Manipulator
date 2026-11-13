from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("three_r_description")
    xacro_file = PathJoinSubstitution([share, "urdf", "three_r.urdf.xacro"])
    rviz_file = PathJoinSubstitution([share, "rviz", "three_r.rviz"])
    robot_description = {"robot_description": Command(["xacro ", xacro_file])}
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_file], output="screen"),
    ])
