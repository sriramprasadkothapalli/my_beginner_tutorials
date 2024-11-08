from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency', default_value='500',
        description='Frequency in milliseconds to publish messages')

    publish_frequency = LaunchConfiguration('publish_frequency')

    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
        parameters=[{'publish_frequency': publish_frequency}]
    )

    return LaunchDescription([
        publish_frequency_arg,
        talker_node
    ])
