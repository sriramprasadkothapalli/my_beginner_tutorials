from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare the bag_record argument, default is to enable bag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='true',
        description='Flag to enable/disable bag recording'
    )

    # Use LaunchConfiguration to obtain the value of 'record_bag'
    record_bag = LaunchConfiguration('record_bag')

    # Bag record process setup
    def launch_setup(context, *args, **kwargs):
        record_bag_value = record_bag.perform(context) == 'true'
        bag_record_process = []

        if record_bag_value:
            # Add ExecuteProcess to run ros2 bag record command for all topics
            bag_record_process.append(
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-o', 'talker_record'],
                    output='screen'
                )
            )

        return bag_record_process

    # Define the Talker node
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
        output='screen'
    )

    return LaunchDescription([
        record_bag_arg,
        OpaqueFunction(function=launch_setup),
        talker_node
    ])
