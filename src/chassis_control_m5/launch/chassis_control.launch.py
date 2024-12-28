from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument ,TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='body')
    rate_arg = DeclareLaunchArgument('rate', default_value='200.0')
    enable_on_start_arg = DeclareLaunchArgument('enable_on_start', default_value='true')
    disable_on_end_arg = DeclareLaunchArgument('disable_on_end', default_value='true')
    can_socket_interface_arg = DeclareLaunchArgument('can_socket_interface', default_value='can0')
    can_bitrate_arg = DeclareLaunchArgument('can_bitrate', default_value='1000000')



        # Define the ros2can_bridge node
    ros2can_bridge_node = Node(
        package='ros2socketcan_bridge',
        executable='ros2can_bridge',
        output='screen',
        parameters=[
            {'can_socket_interface': LaunchConfiguration('can_socket_interface')},
            {'can_bitrate': LaunchConfiguration('can_bitrate')},
        ]
    )
    
    # Define the chassis control node with a 1-second delay
    chassis_control_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='chassis_control_m5',
                executable='chassis_control_m5_node',
                output='screen',
                parameters=[
                    {'frame_id': LaunchConfiguration('frame_id')},
                    {'rate': LaunchConfiguration('rate')},
                    {'enable_on_start': LaunchConfiguration('enable_on_start')},
                    {'disable_on_end': LaunchConfiguration('disable_on_end')},
                    os.path.join(FindPackageShare('chassis_control_m5').find('chassis_control_m5'), 'config', 'chassis_driver_config.yaml')
                ]
            )
        ]
    )

    # Create and return the launch description
    return LaunchDescription([
        frame_id_arg,
        rate_arg,
        enable_on_start_arg,
        disable_on_end_arg,
        can_socket_interface_arg,
        can_bitrate_arg,
        ros2can_bridge_node,
        chassis_control_node
        
    ])
