import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('amr_bringup')

    # Gazebo
    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sim_world.launch.py')
        )
    )

    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'slam.launch.py')
        )
    )

    # Teleop (matches original project)
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'teleop.launch.py')
        )
    )

    # RViz
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    rviz_config = os.path.join(tb3_gazebo_dir, 'rviz', 'tb3_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        sim_world,
        slam,
        teleop,
        rviz_node,
    ])
