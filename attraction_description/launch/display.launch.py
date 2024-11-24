from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('attraction_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'spot_arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'spot.rviz') 
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node to publish the initial joint states
    joint_state_publisher_initial_node = Node(
        package='attraction_description',
        executable='home_states',
        name='publish_initial_joint_states',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    motion_planner_node = Node(
        package='motion_planner',
        executable='KDL',
        name='motion_planner_node',
        parameters=[
            {'robot_description': robot_urdf}
        ],
        output='screen'
    )
#
    test1_node = Node(
        package='motion_planner',
        executable='SpotServer',
        name='test1',
        parameters=[
            {'robot_description': robot_urdf}
        ],
        output='screen'
    )

    stair_visualizer_node = Node(
        package='motion_planner',
        executable='EnvironmentSetup',
        name='stair_visualizer',
        parameters=[
            {'robot_description': robot_urdf}
        ],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node,
        stair_visualizer_node,
        test1_node
    ])
