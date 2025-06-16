from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import xacro
import os

def generate_launch_description():
    share_dir = get_package_share_directory('robot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'AutobotGDB.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'autobot.rviz') 
    bridge_config_file = os.path.join(share_dir, 'config', 'bridge.yaml')
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='False'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')
    show_rviz = LaunchConfiguration('rviz')

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
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
    rviz2_node = Node(
        condition=IfCondition(show_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
        )
    # Path to your custom world file
    world_file = os.path.join(share_dir, 'worlds', 'home.sdf')
    # Gazebo Simulation
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    world_plugin = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'empty', '--plugin', 'gz-sim-sensors-system']
    )

     # Create the spawn entity node
    spawn_entity = ExecuteProcess(
        cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'AutobotGDB',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )

    # Bridge to transfer joint states
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
        }],
        output='screen'
    )

    lidar_publisher = Node(
        package='robot_description',
        executable='fake_lidar_pub',
        name='fake_lidar_pub',
        output='screen',
    )

    lidar_subscriber = Node(
        package='robot_description',
        executable='fake_lidar_sub',
        name='fake_lidar_sub',
        output='screen',
    )
                
    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
                ':',
                share_dir,
                ':',
                os.path.join(share_dir, 'meshes'),
                ':',
                os.path.join(share_dir, '..')
            ]
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ':',
                share_dir,
                ':',
                os.path.join(share_dir, 'meshes'),
                ':',
                os.path.join(share_dir, '..')
            ]
        ),
        gui_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        gazebo,
        spawn_entity,
        #lidar_publisher,
        #lidar_subscriber,
        bridge,        
    ])