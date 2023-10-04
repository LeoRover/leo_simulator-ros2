from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # Spawn a robot inside a simulation
    leo_rover = Node(
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "leo_rover",
            "-z",
            "1.65",
        ],
    )

    return LaunchDescription(
        [
            leo_rover,
        ]
    )
