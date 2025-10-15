
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():

    # Launch configuration variables
    rtk_ip = LaunchConfiguration('rtk_ip')
    rtk_port = LaunchConfiguration('rtk_port')
    apply_dual_antenna_offset = LaunchConfiguration('apply_dual_antenna_offset')
    gps_main_frame_id = LaunchConfiguration('gps_main_frame_id')
    gps_aux_frame_id = LaunchConfiguration('gps_aux_frame_id')

    # Declare the launch arguments
    declare_rtk_ip = DeclareLaunchArgument(
        'rtk_ip', default_value='0.0.0.0', description='IP address for gnss receiver communication configuration'
    )

    declare_rtk_port = DeclareLaunchArgument(
        'rtk_port', default_value='28001', description='Port for gnss receiver communication configuration)'
    )

    declare_apply_dual_antenna_offset = DeclareLaunchArgument(
        'apply_dual_antenna_offset', default_value='False', description='Whether the gsof driver node should apply a heading offset due to the \
                                                                        antennas not being in line with the robot. Leave False if this is done \
                                                                        internally by the receiver or via a yaw offset in the frame_id. \
                                                                        If true, the node will use the frame_ids below to offset the heading \
                                                                        output'
    )

    declare_gps_main_frame_id = DeclareLaunchArgument(
        'gps_main_frame_id', default_value='gps_main', description='Frame id of main gps antenna'
    )

    declare_gps_aux_frame_id = DeclareLaunchArgument(
        'gps_aux_frame_id', default_value='gps_aux', description='Frame id of aux gps antenna'
    )

    driver_node = Node(
        package='trimble_gnss_driver',
        executable='gsof_driver',
        output='screen',
        parameters=[
            {'rtk_ip': rtk_ip},
            {'rtk_port': rtk_port},
            {'output_frame_id': gps_main_frame_id},
            {'apply_dual_antenna_offset': apply_dual_antenna_offset},
            {'gps_main_frame_id': gps_main_frame_id},
            {'gps_aux_frame_id': gps_aux_frame_id},
        ],
        remappings=[
            ('fix', 'gps/fix'),
            ('yaw', 'yaw'),
            ('attitude', 'attitude'),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_rtk_ip)
    ld.add_action(declare_rtk_port)
    ld.add_action(declare_apply_dual_antenna_offset)
    ld.add_action(declare_gps_main_frame_id)
    ld.add_action(declare_gps_aux_frame_id)

    ld.add_action(driver_node)

    return ld