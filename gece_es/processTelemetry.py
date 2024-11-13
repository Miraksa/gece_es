import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, VehicleAttitude, SensorGps
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

        self.global_position_subscriber_ = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_position_callback, qos_profile)
        self.attitude_subscriber_ = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.sensor_gps_subscriber_ = self.create_subscription(SensorGps, '/fmu/out/vehicle_gps', self.sensor_gps_callback, qos_profile)

        self.global_position = VehicleGlobalPosition()
        self.attitude = VehicleAttitude()
        self.sensor_gps = SensorGps()

        self.timer_ = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        roll, pitch, yaw = self.quaternion_conversion.to_euler(self.attitude.q)

        telemetry_data = {
            "team_number": 2,
            "uav_latitude": self.global_position.lat,
            "uav_longitude": self.global_position.lon,
            "uav_altitude": self.global_position.alt_ellipsoid,
            "uav_pitch": pitch * 180 / math.pi,
            "uav_yaw": yaw * 180 / math.pi,
            "uav_roll": roll * 180 / math.pi,
            "uav_speed": self.sensor_gps.vel_m_s,
            "uav_battery": 50,
            "uav_autonomous": 0,
            "uav_locking": 0,
            "target_center_x": 0,
            "target_center_y": 0,
            "target_width": 0,
            "target_height": 0,
            "gps_time": {
                "hour": 0,
                "minute": 0,
                "seconds": 0,
                "milliseconds": 0
            }
        }

        json_data = json.dumps(telemetry_data)

        try:
            response = requests.post("http://localhost:3000/api/send_telemetry_data", data=json_data, headers={'Content-Type': 'application/json'})
            self.get_logger().info(f"Data sent to /api/send_telemetry_data: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send data: {e}")

    def global_position_callback(self, msg):
        self.global_position = msg

    def attitude_callback(self, msg):
        self.attitude = msg
        
    def sensor_gps_callback(self, msg):
        self.sensor_gps = msg

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
