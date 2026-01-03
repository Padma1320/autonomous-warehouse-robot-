import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get TurtleBot3 Gazebo package
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # House world launch
    house_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py')
        )
    )
    
    return LaunchDescription([
        house_world,
    ])
