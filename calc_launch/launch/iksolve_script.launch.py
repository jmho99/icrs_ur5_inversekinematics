import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topic_ik = LaunchConfiguration('topic_ik')
    service_ik = LaunchConfiguration('service_ik')
    action_ik = LaunchConfiguration('action_ik')

    topic_calc = launch_ros.actions.Node(
        package = 'calc_ur5ik_topic',
        namespace = topic_ik,
        executable = 'ur5ik',

    )
    return launch.LaunchDescription([
        topic_calc
    ])