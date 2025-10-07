from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('smart_car')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')

    # Absolute path to your world file
    world_file = os.path.expanduser('~/assignment/src/smart_car/world/smalltown.world')
    if not os.path.exists(world_file):
        print(f"ERROR: World file does not exist: {world_file}")
    else:
        print(f"World file found: {world_file}")
    # Launch argument for world (optional, but safe)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Absolute path to the world file'
    )

    # Log the world path at runtime (debug)
    log_world = LogInfo(msg=['Loading world: ', world_file])

    # Include Gazebo launch (server + client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()  # use absolute path
    )

    # Read URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'smartcar.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'smart_car',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        log_world,
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
