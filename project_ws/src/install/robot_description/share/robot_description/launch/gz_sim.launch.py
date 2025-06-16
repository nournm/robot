import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'AutobotGDB.urdf.xacro'])
    
    # Get the package share directory path
    pkg_share_path = get_package_share_directory('robot_description')

    # Generate robot_description from xacro
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    return LaunchDescription([
        # Set GZ_SIM_RESOURCE_PATH so Gazebo can find meshes
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ':',
                pkg_share_path,
                ':',
                os.path.join(pkg_share_path, 'meshes'),
                ':',
                os.path.join(pkg_share_path, '..')
            ]
        ),
        
        # Alternative: Set IGN_GAZEBO_RESOURCE_PATH for older versions
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
                ':',
                pkg_share_path,
                ':',
                os.path.join(pkg_share_path, 'meshes'),
                ':',
                os.path.join(pkg_share_path, '..')
            ]
        ),
        
        # Start Gazebo (Harmonic/Garden)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py'],
            output='screen'
        ),
        
        # Publish robot_state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'': robot_description}]
        ),
        
        # Spawn the robot in Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'AutobotGDB',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ])