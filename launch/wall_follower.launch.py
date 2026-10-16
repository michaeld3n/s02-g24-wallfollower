"""Start the controller; the simulator or robot is started separately."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    defaults = {
        'params_file': os.path.join(get_package_share_directory('wall_follower'),
                                    'config', 'wall_follower.yaml'),
        'use_sim_time': 'false',
        'start_enabled': 'false',
        'use_stamped_cmd_vel': 'true',
        'namespace': '',
        'scan_topic': 'scan',
        'cmd_vel_topic': 'cmd_vel',
        'state_topic': 'state',
        'enable_service': 'set_enabled',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    node = Node(
        package='wall_follower', executable='wall_follower_node',
        name='wall_follower', namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            name: ParameterValue(LaunchConfiguration(name), value_type=bool)
            for name in ('use_sim_time', 'start_enabled', 'use_stamped_cmd_vel')
        }],
        remappings=[(topic, LaunchConfiguration(argument)) for topic, argument in (
            ('scan', 'scan_topic'), ('cmd_vel', 'cmd_vel_topic'),
            ('state', 'state_topic'), ('set_enabled', 'enable_service'))],
    )
    return LaunchDescription(arguments + [node])
