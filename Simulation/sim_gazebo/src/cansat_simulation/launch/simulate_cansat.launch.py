import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('cansat_simulation')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'cansat.urdf')

    # --- THIS IS THE NEW PART ---
    # Set the GAZEBO_PLUGIN_PATH environment variable to include the path to your plugins
    # This is crucial for Gazebo to find your custom plugins
    install_dir = get_package_share_directory('cansat_simulation')
    gazebo_plugin_path = os.path.join(install_dir, 'lib')

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=f'{os.environ.get("GAZEBO_PLUGIN_PATH", "")}:{gazebo_plugin_path}'
    )
    # ----------------------------

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'cansat',
           '-x', '0', '-y', '0', '-z', '1000',
           '-Vz', '-10.0'], # Use -Vz for Z-velocity
        output='screen'
    )

    controller_node = Node(
        package='cansat_simulation',
        executable='controller_node',
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_plugin_path,  # Add the environment variable action
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_node,
    ])