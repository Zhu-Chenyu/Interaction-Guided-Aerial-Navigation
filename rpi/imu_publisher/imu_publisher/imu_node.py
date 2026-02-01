"""
IMU Publisher Node - Bidirectional MAVLink bridge.

1. Reads IMU/attitude data from Pixhawk and publishes to ROS 2
2. Listens for OptiTrack TF data and sends VISION_POSITION_ESTIMATE to Pixhawk
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from pymavlink import mavutil
import math
import time


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('vision_world_frame', 'optitrack')
        self.declare_parameter('vision_drone_frame', 'imu_link')
        self.declare_parameter('enable_vision_bridge', True)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.vision_world_frame = self.get_parameter('vision_world_frame').value
        self.vision_drone_frame = self.get_parameter('vision_drone_frame').value
        self.enable_vision_bridge = self.get_parameter('enable_vision_bridge').value

        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF2 buffer and listener for OptiTrack data
        if self.enable_vision_bridge:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info(
                f'Vision bridge enabled: {self.vision_world_frame} -> {self.vision_drone_frame}'
            )

        # Connect to MAVLink (use 'common' dialect for ODOMETRY support)
        self.get_logger().info(
            f'Connecting to Pixhawk on {self.serial_port} at {self.baud_rate} baud...'
        )
        try:
            self.mav = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baud_rate,
                dialect='common',
                source_system=1
            )
            self.get_logger().info('Waiting for heartbeat...')
            self.mav.wait_heartbeat()
            self.get_logger().info(f'Heartbeat received from system {self.mav.target_system}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Pixhawk: {e}')
            raise

        # Timer to poll MAVLink messages (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Separate timer for vision position (30Hz)
        if self.enable_vision_bridge:
            self.vision_timer = self.create_timer(0.033, self.vision_callback)

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
            # Only publish TF if vision bridge is disabled (OptiTrack provides TF otherwise)
            if not self.enable_vision_bridge:
                self.publish_tf()

    def vision_callback(self):
        """Look up OptiTrack TF and send to Pixhawk."""
        if not self.enable_vision_bridge:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.vision_world_frame,
                self.vision_drone_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            # TF not available yet, log occasionally
            if not hasattr(self, '_tf_warn_count'):
                self._tf_warn_count = 0
            self._tf_warn_count += 1
            if self._tf_warn_count % 100 == 1:
                self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # Extract position directly from OptiTrack
        # OptiTrack Z-up: X=forward, Y=right, Z=up
        # PX4 NED: X=north/forward, Y=east/right, Z=down
        x_opti = transform.transform.translation.x
        y_opti = transform.transform.translation.y
        z_opti = transform.transform.translation.z

        # Convert OptiTrack Z-up to NED:
        x_ned = x_opti
        y_ned = y_opti
        z_ned = -z_opti

        # Extract quaternion for yaw
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Extract yaw from quaternion (rotation around Z)
        # OptiTrack Z-up and NED both have yaw around Z, but opposite sign convention
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_opti = math.atan2(siny_cosp, cosy_cosp)

        # NED yaw: positive = clockwise from north (when viewed from above)
        # OptiTrack Z-up: positive = counter-clockwise (when viewed from above)
        # So negate the yaw
        yaw_ned = -yaw_opti

        # Timestamp in microseconds
        usec = int(time.time() * 1e6)

        # Send position + yaw (roll=0, pitch=0 - let IMU handle those)
        self.mav.mav.vision_position_estimate_send(
            usec,
            x_ned, y_ned, z_ned,
            0.0, 0.0, yaw_ned
        )

        # Log periodically
        if not hasattr(self, '_vision_send_count'):
            self._vision_send_count = 0
        self._vision_send_count += 1
        if self._vision_send_count % 90 == 1:
            self.get_logger().info(
                f'VISION #{self._vision_send_count}: '
                f'pos=({x_ned:.2f}, {y_ned:.2f}, {z_ned:.2f}) yaw={math.degrees(yaw_ned):.1f}°'
            )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

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
