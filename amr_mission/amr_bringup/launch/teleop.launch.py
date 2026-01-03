from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[
            # This fixes the Twist vs TwistStamped issue
            ('cmd_vel', 'cmd_vel'),  # Let teleop publish to cmd_vel
        ],
        parameters=[
            {'use_stamped_vel': True},  # This makes teleop use TwistStamped!
        ]
    )
    
    return LaunchDescription([
        teleop_node
    ])
