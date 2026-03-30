from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare Launch Configurations (User Inputs)
    is_pro_arg = DeclareLaunchArgument(
        'is_pro',
        default_value='false',
        description='Set to true to use the Pro model hand mapping'
    )
    
    retargeting_arg = DeclareLaunchArgument(
        'manus_retargeting',
        default_value='false',
        description='Set to true to enable kinematic retargeting logic'
    )

    debug_arg = DeclareLaunchArgument(
        'debug_print',
        default_value='false',
        description='Set to true to enable debug terminal printing'
    )

    # 2. Define the Node
    bridge_node = Node(
        package='manus_omni_bridge',
        executable='bridge_node', # This must match the add_executable name in CMakeLists.txt
        name='manus_to_omni_bridge',
        output='screen',          # Ensures your RCLCPP_INFO logs show up in the terminal
        parameters=[{
            'is_pro': LaunchConfiguration('is_pro'),
            'manus_retargeting': LaunchConfiguration('manus_retargeting'),
            'debug_print': LaunchConfiguration('debug_print')
        }]
    )

    # 3. Return the Launch Description
    return LaunchDescription([
        is_pro_arg,
        retargeting_arg,
        debug_arg,
        bridge_node
    ])