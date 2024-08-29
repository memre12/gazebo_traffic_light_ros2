import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration for main vehicle
    verbose = LaunchConfiguration('verbose')

    # Configuration for centilevered traffic light
    pkg_traffic_light = FindPackageShare(package='traffic_light_urdf').find('traffic_light_urdf')
    urdf_file_traffic_light = os.path.join(pkg_traffic_light, 'urdf', 'standalone_model.urdf.xacro')


    # Gazebo simulator launch
    gazebo_simulator = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
    pathWorldFile = PathJoinSubstitution([FindPackageShare('traffic_light_gazebo'), 'worlds', LaunchConfiguration('world')])
    availableWorlds = os.listdir(os.path.join(get_package_share_directory('traffic_light_gazebo'), 'worlds'))

    # Declare launch arguments
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false', description='Enable verbose mode for Gazebo')
    world_arg = DeclareLaunchArgument('world', default_value='teknofest.world', description=f'Choose an available world to launch in Gazebo: {availableWorlds}')

    # Include Gazebo launch
    gazeboLaunch = IncludeLaunchDescription(gazebo_simulator, launch_arguments={'world': pathWorldFile, 'verbose': verbose}.items())


    # Traffic Light Configuration
    def create_traffic_light_nodes(index, urdf_file, x, y, z, yaw):
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'traffic_light_state_publisher_{index}',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': Command([f'xacro {urdf_file}']),
                }]
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'traffic_light_spawn_entity_{index}',
                arguments=[
                    '-entity', f'traffic_light_{index}',
                    '-topic', 'robot_description',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-Y', str(yaw)
                ],
                output='screen'
            )
        ]

    traffic_light_nodes = []

    # Create centilevered traffic lights
    traffic_light_nodes += create_traffic_light_nodes(1, urdf_file_traffic_light, 0, 0, 0, 0)

    return LaunchDescription([
        verbose_arg,
        world_arg,
        gazeboLaunch,
     ] + traffic_light_nodes)
