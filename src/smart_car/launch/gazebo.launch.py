from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('smart_car')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')

    # World file (absolute)
    world_file = os.path.expanduser('~/assignment/src/smart_car/world/smalltown.world')
    if not os.path.exists(world_file):
        print(f"ERROR: World file does not exist: {world_file}")
    else:
        print(f"World file found: {world_file}")

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Absolute path to the world file'
    )
    log_world = LogInfo(msg=['Loading world: ', world_file])

    # Gazebo (server + client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Load URDF text
    urdf_file = os.path.join(pkg_share, 'urdf', 'smartcar.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Joint state publisher (so RViz shows wheels)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    # If you prefer sliders, swap to joint_state_publisher_gui:
    # joint_state_publisher = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    # )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'smart_car', '-topic', 'robot_description'],
        output='screen'
    )

    # RViz (start a few seconds later so TF/robot_description are ready)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'smartcar.rviz')
    rviz = TimerAction(
        period=5.0,  # seconds
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file]
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        log_world,
        gazebo,
        joint_state_publisher,
        robot_state_publisher,
        spawn_robot,
        rviz
    ])
