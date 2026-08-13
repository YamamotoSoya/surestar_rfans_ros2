# claude: ROS2 launch (replaces vendor node_manager.launch).
#         Two-node pipeline: driver_node (UDP -> RfansPacket) and
#         calculation_node (RfansPacket -> PointCloud2).
#         model/rps/use_double_echo/data_level go to BOTH nodes — in ROS1 the
#         calculation node read the driver's params via the global server,
#         which does not exist in ROS2.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = LaunchConfiguration('model')
    device_ip = LaunchConfiguration('device_ip')
    device_port = LaunchConfiguration('device_port')
    rps = LaunchConfiguration('rps')
    data_level = LaunchConfiguration('data_level')
    use_double_echo = LaunchConfiguration('use_double_echo')
    frame_id = LaunchConfiguration('frame_id')
    angle_duration = LaunchConfiguration('angle_duration')
    pcap = LaunchConfiguration('pcap')
    read_once = LaunchConfiguration('read_once')
    read_fast = LaunchConfiguration('read_fast')
    repeat_delay = LaunchConfiguration('repeat_delay')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='R-Fans-16'),
        DeclareLaunchArgument('device_ip', default_value='192.168.0.3'),
        DeclareLaunchArgument('device_port', default_value='2014'),
        DeclareLaunchArgument('rps', default_value='10'),
        DeclareLaunchArgument('data_level', default_value='3'),
        DeclareLaunchArgument('use_double_echo', default_value='false'),
        DeclareLaunchArgument('frame_id', default_value='rfans'),
        DeclareLaunchArgument('angle_duration', default_value='360.0'),
        DeclareLaunchArgument('pcap', default_value='',
                              description='pcap file for device-less replay'),
        DeclareLaunchArgument('read_once', default_value='false'),
        DeclareLaunchArgument('read_fast', default_value='false'),
        DeclareLaunchArgument('repeat_delay', default_value='0.0'),

        Node(
            package='surestar_rfans_ros2',
            executable='driver_node',
            name='rfans_driver',
            output='screen',
            parameters=[{
                'model': model,
                'advertise_name': 'rfans_packets',
                'control_name': 'rfans_control',
                'device_ip': device_ip,
                'device_port': device_port,
                'rps': rps,
                'data_level': data_level,
                'use_double_echo': use_double_echo,
                'pcap': pcap,
                'read_once': read_once,
                'read_fast': read_fast,
                'repeat_delay': repeat_delay,
            }],
        ),
        Node(
            package='surestar_rfans_ros2',
            executable='calculation_node',
            name='calculation_node',
            output='screen',
            parameters=[{
                'model': model,
                'advertise_name': 'rfans_points',
                'subscribe_name': 'rfans_packets',
                'frame_id': frame_id,
                'device_ip': device_ip,
                'rps': rps,
                'data_level': data_level,
                'use_double_echo': use_double_echo,
                'angle_duration': angle_duration,
                'use_gps': False,
            }],
        ),
    ])
