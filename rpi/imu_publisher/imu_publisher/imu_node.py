"""
IMU Publisher Node - Reads MAVLink IMU data from Pixhawk and publishes to ROS 2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pymavlink import mavutil
import math


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Connect to MAVLink
        self.get_logger().info(f'Connecting to Pixhawk on {self.serial_port} at {self.baud_rate} baud...')
        try:
            self.mav = mavutil.mavlink_connection(self.serial_port, baud=self.baud_rate)
            self.get_logger().info('Waiting for heartbeat...')
            self.mav.wait_heartbeat()
            self.get_logger().info(f'Heartbeat received from system {self.mav.target_system}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Pixhawk: {e}')
            raise
        
        # Timer to poll MAVLink messages
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        
        # Storage for latest data
        self.attitude = None
        self.imu_raw = None
        
    def timer_callback(self):
        # Read available MAVLink messages
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == 'ATTITUDE':
                self.attitude = msg
            elif msg_type == 'SCALED_IMU':
                self.imu_raw = msg
        
        # Publish if we have data
        if self.attitude is not None:
            self.publish_imu()
            self.publish_tf()
    
    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Convert Euler angles to quaternion
        # NED to ENU conversion: negate pitch and yaw
        roll = self.attitude.roll
        pitch = -self.attitude.pitch
        yaw = -self.attitude.yaw
        
        # Euler to quaternion conversion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = self.attitude.rollspeed
        imu_msg.angular_velocity.y = self.attitude.pitchspeed
        imu_msg.angular_velocity.z = self.attitude.yawspeed
        
        # Linear acceleration from SCALED_IMU (convert from milli-g to m/s^2)
        if self.imu_raw is not None:
            imu_msg.linear_acceleration.x = self.imu_raw.xacc * 9.81 / 1000.0
            imu_msg.linear_acceleration.y = self.imu_raw.yacc * 9.81 / 1000.0
            imu_msg.linear_acceleration.z = self.imu_raw.zacc * 9.81 / 1000.0
        
        self.imu_pub.publish(imu_msg)
    
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.frame_id
        
        # No translation, just rotation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Use the same quaternion as IMU
        # NED to ENU conversion: negate pitch and yaw
        roll = self.attitude.roll
        pitch = -self.attitude.pitch
        yaw = -self.attitude.yaw
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
