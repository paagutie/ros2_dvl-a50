import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    dvl_sensor= launch_ros.actions.Node(
        package='rov_io', node_executable='dvl-A50.py', output='screen')


    return launch.LaunchDescription([
        dvl_sensor,
    ])


       
