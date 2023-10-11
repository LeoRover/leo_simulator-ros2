# Copyright (c) [2023] [Jan Hernas]

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import xacro
from launch_ros.actions import Node


def spawn_robot(context: LaunchContext, namespace: LaunchConfiguration):
    pkg_project_description = get_package_share_directory('leo_description')
    robot_name = context.perform_substitution(namespace)

    robot_desc = xacro.process(
        os.path.join(
            pkg_project_description,
            'urdf',
            'leo_sim.urdf.xacro',
        ),
        mappings={'robot_ns': robot_name},
    )

    if robot_name == '':
        robot_state_publisher_node_name = 'robot_state_publisher'
        robot_gazebo_name = 'leo_rover'
        topic_bridge_node_name = 'topic_bridge'
        image_bridge_node_name = 'image_bridge'
    else:
        robot_state_publisher_node_name = robot_name + '_robot_state_publisher'
        robot_gazebo_name = 'leo_rover_' + robot_name
        topic_bridge_node_name = robot_name + '_topic_bridge'
        image_bridge_node_name = robot_name + '_image_bridge'

    # Launch robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_state_publisher_node_name,
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
        remappings=[
            ('/robot_description', robot_name + '/robot_description'),
        ],
    )
    # Spawn a robot inside a simulation
    leo_rover = Node(
        package='ros_gz_sim',
        executable='create',
        name='ros_gz_sim_create',
        output='both',
        arguments=[
            '-topic',
            robot_name + '/robot_description',
            '-name',
            robot_gazebo_name,
            '-z',
            '1.65',
        ],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=topic_bridge_node_name,
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            robot_name + '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            robot_name + '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            robot_name + '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            robot_name + '/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            robot_name
            + '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            'world/leo_marsyard/model/'
            + robot_gazebo_name
            + '/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        parameters=[
            {
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }
        ],
        remappings=[
            (
                'world/leo_marsyard/model/' + robot_gazebo_name + '/joint_state',
                robot_name + '/joint_states',
            ),
        ],
        output='screen',
    )

    # Camera image bridge
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name=image_bridge_node_name,
        arguments=[robot_name + '/camera/image_raw'],
        output='screen',
    )
    return [
        robot_state_publisher,
        leo_rover,
        topic_bridge,
        image_bridge,
    ]


def generate_launch_description():
    name_argument = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Robot namespace',
    )

    namespace = LaunchConfiguration('robot_name')

    return LaunchDescription(
        [name_argument, OpaqueFunction(function=spawn_robot, args=[namespace])]
    )
