import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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

        # 2. Joint State Publisher
        # Publishes /joint_states so robot_state_publisher can complete
        # the TF chain out to right_wheel / left_wheel.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # 3. Sim Bridge (no hardware needed -- fake odometry from /cmd_vel)
        Node(
            package='fire_robot',
            executable='fire_bridge',
            name='fire_bridge',
            output='screen'
        ),

        # 4. The RPLidar Driver -- DISABLED until real hardware is connected.
        # rplidar_composition 2.1.4 crashes with a buffer overflow (SIGABRT)
        # if it can't open the serial port, rather than failing gracefully,
        # so leave this out entirely for hardware-free testing.
        
        Node(
             package='rplidar_ros',
             executable='rplidar_composition',
             name='rplidar_node',
             output='screen',
             parameters=[{
                 'serial_port': '/dev/ttyUSB0',
                 'frame_id': 'laser_frame',
                 'angle_compensate': True,
                 'scan_mode': 'Standard'
             }]
         ),
    ])
