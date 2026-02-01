"""
Vision Bridge Node - Receives TF from OptiTrack and sends to Pixhawk via MAVLink.

Subscribes to /tf topic, looks up the drone's pose in the world frame,
and sends VISION_POSITION_ESTIMATE messages to the Pixhawk.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from pymavlink import mavutil
import math
import time


class VisionBridge(Node):
    def __init__(self):
        super().__init__('vision_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('world_frame', 'optitrack')
        self.declare_parameter('drone_frame', 'imu_link')
        self.declare_parameter('rate_hz', 30.0)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.world_frame = self.get_parameter('world_frame').value
        self.drone_frame = self.get_parameter('drone_frame').value
        rate_hz = self.get_parameter('rate_hz').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Connect to MAVLink
        self.get_logger().info(
            f'Connecting to Pixhawk on {self.serial_port} at {self.baud_rate} baud...'
        )
        try:
            self.mav = mavutil.mavlink_connection(self.serial_port, baud=self.baud_rate)
            self.get_logger().info('Waiting for heartbeat...')
            self.mav.wait_heartbeat()
            self.get_logger().info(f'Heartbeat received from system {self.mav.target_system}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Pixhawk: {e}')
            raise

        # Timer for sending vision position
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

        self.get_logger().info(
            f'Vision bridge ready. Listening for TF: {self.world_frame} -> {self.drone_frame}'
        )

    def timer_callback(self):
        try:
            # Look up transform from world to drone
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.drone_frame,
                rclpy.time.Time()  # Get latest available
            )
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return

        # Extract position (ENU from ROS)
        x_enu = transform.transform.translation.x
        y_enu = transform.transform.translation.y
        z_enu = transform.transform.translation.z

        # Convert ENU to NED for MAVLink
        x_ned = y_enu   # ENU East -> NED North
        y_ned = x_enu   # ENU North -> NED East
        z_ned = -z_enu  # ENU Up -> NED Down

        # Extract quaternion
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Convert quaternion to Euler angles (ENU)
        roll_enu, pitch_enu, yaw_enu = self.quaternion_to_euler(qx, qy, qz, qw)

        # Convert ENU to NED Euler angles
        roll_ned = roll_enu
        pitch_ned = -pitch_enu
        yaw_ned = -yaw_enu + math.pi / 2  # Rotate 90 degrees for ENU->NED yaw

        # Normalize yaw to [-pi, pi]
        while yaw_ned > math.pi:
            yaw_ned -= 2 * math.pi
        while yaw_ned < -math.pi:
            yaw_ned += 2 * math.pi

        # Timestamp in microseconds
        usec = int(time.time() * 1e6)

        # Send VISION_POSITION_ESTIMATE
        self.mav.mav.vision_position_estimate_send(
            usec,
            x_ned, y_ned, z_ned,
            roll_ned, pitch_ned, yaw_ned
        )

        self.get_logger().debug(
            f'Sent vision pos: x={x_ned:.2f} y={y_ned:.2f} z={z_ned:.2f}'
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
