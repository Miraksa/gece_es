from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    total_teams = 4  # Set the number of teams here
    nodes = []

    for team_number in range(1, total_teams + 1):
        namespace = f'device{team_number}'

        telemetry_remappings = [
            (f'/team{team}/telemetry', f'/{namespace}/team{team}/telemetry')
            for team in range(1, total_teams + 1)
        ]

        node = Node(
            package='gece_es',
            executable='process_telemetry',
            name=f'team{team_number}',
            namespace=namespace,
            parameters=[{'team_number': team_number}, {'total_teams': total_teams}],
            output='screen',
            remappings=[
                ('/fmu/out/vehicle_attitude', f'/px4_{team_number}/fmu/out/vehicle_attitude'),
                ('/fmu/out/vehicle_global_position', f'/px4_{team_number}/fmu/out/vehicle_global_position'),
                ('/fmu/out/vehicle_gps_position', f'/px4_{team_number}/fmu/out/vehicle_gps_position')
            ] + telemetry_remappings
        )
        nodes.append(node)

    return LaunchDescription(nodes)
