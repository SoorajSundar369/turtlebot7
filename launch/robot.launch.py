import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Path to your URDF file
    urdf_file = os.path.join(
        get_package_share_directory('fire_robot'),
        'description',
        'robot.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. The Robot State Publisher (Loads the URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 2. Your ESP32 Bridge (Motors & Odometry)
        Node(
            package='fire_robot',
            executable='fire_bridge',
            name='fire_bridge',
            output='screen'
        ),
        
        # 3. The RPLidar Driver
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', # MAKE SURE THIS IS CORRECT
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])
