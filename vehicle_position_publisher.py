import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import time

class VehiclePositionPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_position_publisher')
        self.publisher_ = self.create_publisher(Pose, 'vehicle_position', 10)
        self.timer = self.create_timer(0.1, self.publish_position)
        self.x = 130.0
        self.y = 109.0
        self.z = 0.24
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.v = 11

        # Setup a timer to trigger shutdown after 10 seconds
        self.shutdown_timer = self.create_timer(10, self.shutdown_callback)

    def publish_position(self):
        msg = Pose()
        msg.position = Point(x=self.x, y=self.y, z=self.z)
        msg.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Position (%s, %s, %s) Orientation (roll: %s, pitch: %s, yaw: %s)' % (self.x, self.y, self.z, self.roll, self.pitch, self.yaw))
        self.x += round(self.v / 10, 1)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def shutdown_callback(self):
        self.get_logger().info('Shutting down...')
        self.destroy_timer(self.timer)
        self.destroy_timer(self.shutdown_timer)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    time.sleep(5)
    vehicle_position_publisher = VehiclePositionPublisher()
    rclpy.spin(vehicle_position_publisher)
    # The shutdown will be triggered by the timer

if __name__ == '__main__':
    main()



