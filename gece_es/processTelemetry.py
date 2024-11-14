import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, VehicleAttitude, SensorGps
from bohoso.msg import TeamTelemetry

import math
import json
import requests

class ProcessTelemetry(Node):
    def __init__(self): 
        super().__init__('process_telemetry')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.quaternion_conversion = QuaternionConversion()
        self.declare_parameter('team_number', 1)
        self.declare_parameter('total_teams', 4)

        self.team_number = self.get_parameter('team_number').get_parameter_value().integer_value
        self.total_teams = self.get_parameter('total_teams').get_parameter_value().integer_value

        # Team telemetry publisher
        self.team_publishers = {}
        for team_number in range(1, self.total_teams + 1):
            topic_name = f'/team{team_number}/telemetry'
            self.team_publishers[team_number] = self.create_publisher(
                TeamTelemetry,
                topic_name,
                qos_profile
            )
        
        self.global_position_subscriber_ = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_position_callback, qos_profile)
        self.attitude_subscriber_ = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.sensor_gps_subscriber_ = self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.sensor_gps_callback, qos_profile)

        self.global_position = VehicleGlobalPosition()
        self.attitude = VehicleAttitude()
        self.sensor_gps = SensorGps()

        self.team_telemetry_data = {}
        for team_number in range(1, self.total_teams + 1):
            self.team_telemetry_data[team_number] = TeamTelemetry()

        self.timer_ = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        roll, pitch, yaw = self.quaternion_conversion.to_euler(self.attitude.q)

        telemetry_data = {
            "team_number": self.team_number,
            "uav_latitude": self.sensor_gps.latitude_deg,
            "uav_longitude": self.sensor_gps.longitude_deg,
            "uav_altitude": self.sensor_gps.altitude_ellipsoid_m,
            "uav_elevation": pitch * 180 / math.pi,
            "uav_orientation": yaw * 180 / math.pi,
            "uav_banking": roll * 180 / math.pi,
            "uav_speed": self.sensor_gps.vel_m_s,
            "uav_battery": None,
            "uav_autonomous": None,
            "uav_locking": None,
            "target_center_x": None,
            "target_center_y": None,
            "target_width": None,
            "target_height": None,
            "gps_time": {
                "hour": None,
                "minute": None,
                "seconds": None,
                "milliseconds": None
            }
        }

        json_data = json.dumps(telemetry_data)

        try:
            response = requests.post("http://localhost:3000/api/send_telemetry_data", data=json_data, headers={'Content-Type': 'application/json'})
            
            if response.status_code == 200:  
                teams_data = self.separate_by_team(response.json())
                
                for team_number, telemetry_list in teams_data.items():
                    for telemetry_data in telemetry_list:
                        self.publish_team_telemetry(team_number, telemetry_data)

            else:
                print(f"Failed to fetch data: {response.status_code}")

        except requests.exceptions.RequestException as e:
            print(f"Failed to send data: {e}")        



    def global_position_callback(self, msg):
        self.global_position = msg

    def attitude_callback(self, msg):
        self.attitude = msg
        
    def sensor_gps_callback(self, msg):
        self.sensor_gps = msg

    def separate_by_team(self, data):
        teams = {}
        
        for entry in data["locationInfo"]:
            if "team_number" not in entry:
                self.get_logger().warn("Skipping entry without 'team_number'")
                continue
            team_number = entry["team_number"]
            if team_number not in teams:
                teams[team_number] = []
            teams[team_number].append(entry)

        return teams
    
    def publish_team_telemetry(self, team_number, telemetry_data):
        if team_number in self.team_publishers:
            msg = self.team_telemetry_data[team_number]
            msg.team_number = int(telemetry_data["team_number"])
            msg.uav_latitude = float(telemetry_data["uav_latitude"])
            msg.uav_longitude = float(telemetry_data["uav_longitude"])
            msg.uav_altitude = float(telemetry_data["uav_altitude"])
            msg.uav_elevation = float(telemetry_data["uav_elevation"])
            msg.uav_orientation = int(telemetry_data["uav_orientation"])
            msg.uav_banking = float(telemetry_data["uav_banking"])
            msg.uav_speed = float(telemetry_data["uav_speed"])

            self.team_publishers[team_number].publish(msg)
            self.get_logger().info(f"Published telemetry for team {team_number}")
            
        else:
            self.get_logger().warn(f"No publisher for team {team_number}")


class QuaternionConversion:
    @staticmethod
    def to_euler(quaternion):
        w, x, y, z = quaternion
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
        return roll, pitch, yaw

    @staticmethod
    def to_quaternion(euler):
        roll, pitch, yaw = euler
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return w, x, y, z

        
def main(args=None):
    rclpy.init(args=args)
    process = ProcessTelemetry()
    rclpy.spin(process)
    rclpy.shutdown()

if __name__ == "__main__": 
    main()
