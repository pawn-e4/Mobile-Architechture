from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to your package
    pkg_share = get_package_share_directory('smart_car')

    # Default world file (you can change this to your map)
    default_world = os.path.join(pkg_share, 'worlds', 'smalltown.world')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='SDF world file to load'
    )

    # Include gazebo_ros bringup (gazebo server + client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'smart_car',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Publish robot description from URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'smartcar.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
