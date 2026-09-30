import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    leader_port_arg = DeclareLaunchArgument(
        'leader_port', default_value='/dev/ttyACM0')
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='None',
        description='UR5e robot IP address, used for the direct gripper socket connection')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true')

    urdf_xacro = os.path.join(
        get_package_share_directory('ur5e_description'), 'urdf', 'ur5e_gripper.urdf.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro, ' ur_type:=ur5e use_fake_hardware:=true robot_ip:=127.0.0.1']),
        value_type=str
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }]
    )

    teleop_node = Node(
        package='so100_control',
        executable='ur5e_teleop_node',
        name='ur5e_teleop_node',
        output='screen',
        parameters=[{
            'leader_port': LaunchConfiguration('leader_port'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'loop_hz': 30.0,
            'max_jump': 0.5,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        leader_port_arg,
        robot_ip_arg,
        use_rviz_arg,
        rsp_node,
        teleop_node,
        delayed_rviz,
    ])
