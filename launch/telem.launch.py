from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gece_es',
            executable='process_telemetry',
            name='telem',
            output='screen',
            remappings=[
                ('/fmu/out/vehicle_attitude', '/px4_1/fmu/out/vehicle_attitude'),
                ('/fmu/out/vehicle_global_position', '/px4_1/fmu/out/vehicle_global_position')
            ]
        )
    ])