import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to the Agibot Launch File
    agibot_launch_dir = os.path.join(
        get_package_share_directory('omnihand_node'), 'launch')
    
    agibot_hand = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agibot_launch_dir, 'omnihand_pro_2025_Manus.launch.py')
        )
    )

    # 2. Manus Data Publisher (Manus SDK)
    manus_publisher = Node(
        package='manus_ros2',
        executable='manus_data_publisher',
        name='manus_data_publisher',
        output='screen'
    )

    # 3. Your Bridge Node
    omni_bridge = Node(
        package='manus_omni_bridge',
        executable='bridge_node',
        name='manus_to_omni_bridge',
        output='screen'
    )

    return LaunchDescription([
        #agibot_hand,
        #manus_publisher,
        omni_bridge
    ])