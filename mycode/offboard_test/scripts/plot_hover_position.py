#!/usr/bin/env python3
"""
Plot Hover Position: Records and plots drone x, y, z position during hover.
Shows deviation from setpoint in three subplots.

Automatically starts recording when the drone enters hover mode (armed + offboard)
and stops recording when hover ends.

Usage:
    1. Run this script: ros2 run offboard_test plot_hover_position.py
    2. Run your offboard test - recording starts automatically in hover mode
    3. Plots are saved automatically to src/images when hover ends

Subscribes to:
    - /fmu/out/vehicle_local_position (actual position from PX4)
    - /fmu/in/trajectory_setpoint (commanded setpoint)
    - /fmu/out/vehicle_status (to detect hover mode)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import signal
import sys
import os
from datetime import datetime

from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint, VehicleStatus


class HoverPlotter(Node):
    # PX4 VehicleStatus constants
    ARMING_STATE_ARMED = 2
    NAVIGATION_STATE_OFFBOARD = 14
    
    def __init__(self):
        super().__init__('hover_plotter')
        
        # Data storage
        self.times = deque()
        self.pos_x = deque()
        self.pos_y = deque()
        self.pos_z = deque()
        self.roll = deque()   # Roll angle (radians)
        self.pitch = deque()  # Pitch angle (radians)
        self.setpoint_x = deque()
        self.setpoint_y = deque()
        self.setpoint_z = deque()
        
        self.start_time = None
        self.latest_setpoint = [0.0, 0.0, 0.0]
        self.latest_position = [0.0, 0.0, 0.0]
        
        # State tracking for auto-detection
        self.is_armed = False
        self.is_offboard = False
        self.waiting_for_hover = False  # Waiting for drone to reach hover altitude
        self.is_recording = False
        self.was_recording = False  # Track if we were recording (to detect hover end)
        self.plot_saved = False  # Prevent multiple saves
        self.hover_setpoint_z = None  # Track z setpoint during hover
        
        # Output directory (project's src/images) - use absolute path
        self.output_dir = '/home/chenyu-zhu/winter_project/code/src/images'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # QoS for PX4 output topics (most use transient_local)
        qos_px4_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for setpoint (input topic, uses volatile)
        qos_setpoint = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers - use VehicleOdometry for position
        self.pos_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.position_callback,
            qos_px4_out
        )
        
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.setpoint_callback,
            qos_setpoint
        )
        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_px4_out
        )
        
        self.get_logger().info('Hover plotter started. Waiting for hover mode (armed + offboard)...')
        self.get_logger().info(f'Plots will be saved to: {self.output_dir}')
        
    def status_callback(self, msg):
        """Monitor vehicle status to detect hover mode and disarm."""
        was_armed = self.is_armed
        
        self.is_armed = (msg.arming_state == self.ARMING_STATE_ARMED)
        self.is_offboard = (msg.nav_state == self.NAVIGATION_STATE_OFFBOARD)
        
        in_offboard = self.is_armed and self.is_offboard
        
        # Enter waiting state when offboard mode starts (takeoff begins)
        if in_offboard and not self.waiting_for_hover and not self.is_recording and not self.was_recording:
            self.waiting_for_hover = True
            self.get_logger().info('🚁 Offboard mode detected! Waiting for hover altitude...')
        
        # Detect flight end - drone disarmed after we were recording
        if self.was_recording and was_armed and not self.is_armed:
            self.is_recording = False
            self.get_logger().info('✅ Drone disarmed - flight complete. Generating plots...')
            self.plot_data()
    
    def position_callback(self, msg):
        # VehicleOdometry uses position array [x, y, z] and quaternion q [w, x, y, z]
        x, y, z = msg.position[0], msg.position[1], msg.position[2]
        self.latest_position = [x, y, z]
        
        # Extract roll and pitch from quaternion (NED frame)
        qw, qx, qy, qz = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        # Roll (rotation around x-axis)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll_rad = np.arctan2(sinr_cosp, cosr_cosp)
        # Pitch (rotation around y-axis)
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = np.clip(sinp, -1.0, 1.0)  # Clamp to avoid numerical issues
        pitch_rad = np.arcsin(sinp)
        
        # Check if we've reached hover altitude (waiting for hover -> start recording)
        if self.waiting_for_hover and not self.is_recording:
            z_error = abs(z - self.latest_setpoint[2])
            # Start recording when within 10cm of hover setpoint
            if z_error < 0.10 and self.latest_setpoint[2] != 0.0:
                self.waiting_for_hover = False
                self.is_recording = True
                self.was_recording = True
                self.hover_setpoint_z = self.latest_setpoint[2]
                self.start_time = self.get_clock().now()
                self.get_logger().info(f'✨ Reached hover altitude ({z:.3f}m)! Started recording...')
                self.get_logger().info(f'Hover setpoint z: {self.hover_setpoint_z:.3f}m')
        
        # Only record during actual hover
        if not self.is_recording:
            return
            
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.times.append(t)
        self.pos_x.append(x)
        self.pos_y.append(y)
        self.pos_z.append(z)
        self.roll.append(roll_rad)
        self.pitch.append(pitch_rad)
        self.setpoint_x.append(self.latest_setpoint[0])
        self.setpoint_y.append(self.latest_setpoint[1])
        self.setpoint_z.append(self.latest_setpoint[2])
        
        # Log periodically
        if len(self.times) % 100 == 0:
            self.get_logger().info(
                f'Recording hover: {len(self.times)} samples. '
                f'Pos: ({x:.3f}, {y:.3f}, {z:.3f}), Roll: {np.degrees(roll_rad):.1f}°, Pitch: {np.degrees(pitch_rad):.1f}°'
            )
    
    def setpoint_callback(self, msg):
        if np.isnan(msg.position[0]):
            return
            
        new_z = msg.position[2]
        
        # Detect landing when recording: z setpoint moving toward ground
        if self.is_recording and self.hover_setpoint_z is not None:
            # If setpoint is more than 5cm higher (closer to ground) than hover setpoint
            if (new_z - self.hover_setpoint_z) > 0.05:
                self.is_recording = False
                self.get_logger().info('🛬 Landing detected! Hover ended. Generating plots...')
                self.plot_data()
        
        self.latest_setpoint = [msg.position[0], msg.position[1], msg.position[2]]
    
    def plot_data(self):
        # Prevent multiple saves
        if self.plot_saved:
            return
        self.plot_saved = True
        
        if len(self.times) < 10:
            self.get_logger().warn('Not enough data to plot!')
            return
            
        times = np.array(self.times)
        pos_x = np.array(self.pos_x)
        pos_y = np.array(self.pos_y)
        pos_z = np.array(self.pos_z)
        sp_x = np.array(self.setpoint_x)
        sp_y = np.array(self.setpoint_y)
        sp_z = np.array(self.setpoint_z)
        
        # Calculate errors
        err_x = pos_x - sp_x
        err_y = pos_y - sp_y
        err_z = pos_z - sp_z
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Create figure with 3 subplots
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Hover Position Tracking (NED Frame)', fontsize=14, fontweight='bold')
        
        # X position
        axes[0].plot(times, pos_x, 'b-', label='Actual X', linewidth=1.5)
        axes[0].plot(times, sp_x, 'r--', label='Setpoint X', linewidth=1.5)
        axes[0].fill_between(times, sp_x, pos_x, alpha=0.3, color='blue')
        axes[0].set_ylabel('X Position (m)')
        axes[0].legend(loc='upper right')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title(f'X Error: mean={np.mean(err_x):.4f}m, std={np.std(err_x):.4f}m, max={np.max(np.abs(err_x)):.4f}m')
        
        # Y position
        axes[1].plot(times, pos_y, 'g-', label='Actual Y', linewidth=1.5)
        axes[1].plot(times, sp_y, 'r--', label='Setpoint Y', linewidth=1.5)
        axes[1].fill_between(times, sp_y, pos_y, alpha=0.3, color='green')
        axes[1].set_ylabel('Y Position (m)')
        axes[1].legend(loc='upper right')
        axes[1].grid(True, alpha=0.3)
        axes[1].set_title(f'Y Error: mean={np.mean(err_y):.4f}m, std={np.std(err_y):.4f}m, max={np.max(np.abs(err_y)):.4f}m')
        
        # Z position
        axes[2].plot(times, pos_z, 'm-', label='Actual Z', linewidth=1.5)
        axes[2].plot(times, sp_z, 'r--', label='Setpoint Z', linewidth=1.5)
        axes[2].fill_between(times, sp_z, pos_z, alpha=0.3, color='magenta')
        axes[2].set_ylabel('Z Position (m)')
        axes[2].set_xlabel('Time (s)')
        axes[2].legend(loc='upper right')
        axes[2].grid(True, alpha=0.3)
        axes[2].set_title(f'Z Error: mean={np.mean(err_z):.4f}m, std={np.std(err_z):.4f}m, max={np.max(np.abs(err_z)):.4f}m')
        
        plt.tight_layout()
        
        # Save to src/images directory
        position_plot_path = os.path.join(self.output_dir, f'hover_position_{timestamp}.png')
        plt.savefig(position_plot_path, dpi=150)
        self.get_logger().info(f'Position plot saved to: {position_plot_path}')
        
        # Also create error-only plot
        fig2, axes2 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig2.suptitle('Position Error from Setpoint (NED Frame)', fontsize=14, fontweight='bold')
        
        axes2[0].plot(times, err_x * 100, 'b-', linewidth=1.5)  # Convert to cm
        axes2[0].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes2[0].set_ylabel('X Error (cm)')
        axes2[0].grid(True, alpha=0.3)
        
        axes2[1].plot(times, err_y * 100, 'g-', linewidth=1.5)
        axes2[1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes2[1].set_ylabel('Y Error (cm)')
        axes2[1].grid(True, alpha=0.3)
        
        axes2[2].plot(times, err_z * 100, 'm-', linewidth=1.5)
        axes2[2].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes2[2].set_ylabel('Z Error (cm)')
        axes2[2].set_xlabel('Time (s)')
        axes2[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        error_plot_path = os.path.join(self.output_dir, f'hover_error_{timestamp}.png')
        plt.savefig(error_plot_path, dpi=150)
        self.get_logger().info(f'Error plot saved to: {error_plot_path}')
        
        # Print summary statistics
        print('\n' + '='*60)
        print('HOVER POSITION TRACKING SUMMARY')
        print('='*60)
        print(f'Total samples: {len(times)}')
        print(f'Duration: {times[-1]:.2f} seconds')
        print()
        print(f'X Position Error:')
        print(f'  Mean:   {np.mean(err_x)*100:+.2f} cm')
        print(f'  Std:    {np.std(err_x)*100:.2f} cm')
        print(f'  Max:    {np.max(np.abs(err_x))*100:.2f} cm')
        print()
        print(f'Y Position Error:')
        print(f'  Mean:   {np.mean(err_y)*100:+.2f} cm')
        print(f'  Std:    {np.std(err_y)*100:.2f} cm')
        print(f'  Max:    {np.max(np.abs(err_y))*100:.2f} cm')
        print()
        print(f'Z Position Error:')
        print(f'  Mean:   {np.mean(err_z)*100:+.2f} cm')
        print(f'  Std:    {np.std(err_z)*100:.2f} cm')
        print(f'  Max:    {np.max(np.abs(err_z))*100:.2f} cm')
        print()
        print(f'3D Position RMSE: {np.sqrt(np.mean(err_x**2 + err_y**2 + err_z**2))*100:.2f} cm')
        print('='*60)
        
        # Convert roll/pitch to degrees
        roll_deg = np.array(self.roll) * 180.0 / np.pi
        pitch_deg = np.array(self.pitch) * 180.0 / np.pi
        
        # Plot 2: X Error and Pitch vs Time (2 subplots)
        fig2, axes2 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig2.suptitle('Pitch and X Position Error vs Time (NED Frame)', fontsize=14, fontweight='bold')
        
        # X Error subplot
        axes2[0].plot(times, err_x * 100, 'b-', linewidth=1.5)
        axes2[0].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes2[0].set_ylabel('X Error (cm)')
        axes2[0].grid(True, alpha=0.3)
        axes2[0].legend(['X Error', 'Zero'], loc='upper right')
        
        # Pitch subplot
        axes2[1].plot(times, pitch_deg, 'orange', linewidth=1.5)
        axes2[1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes2[1].set_ylabel('Pitch (degrees)')
        axes2[1].set_xlabel('Time (s)')
        axes2[1].grid(True, alpha=0.3)
        axes2[1].legend(['Pitch', 'Zero'], loc='upper right')
        
        # Add correlation coefficient
        corr_pitch_x = np.corrcoef(pitch_deg, err_x)[0, 1]
        fig2.text(0.02, 0.98, f'Correlation(Pitch, X Error): {corr_pitch_x:.3f}', 
                  transform=fig2.transFigure, fontsize=11,
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        pitch_x_plot_path = os.path.join(self.output_dir, f'pitch_x_error_{timestamp}.png')
        plt.savefig(pitch_x_plot_path, dpi=150)
        self.get_logger().info(f'Pitch vs X Error plot saved to: {pitch_x_plot_path}')
        
        # Plot 3: Y Error and Roll vs Time (2 subplots)
        fig3, axes3 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig3.suptitle('Roll and Y Position Error vs Time (NED Frame)', fontsize=14, fontweight='bold')
        
        # Y Error subplot
        axes3[0].plot(times, err_y * 100, 'g-', linewidth=1.5)
        axes3[0].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes3[0].set_ylabel('Y Error (cm)')
        axes3[0].grid(True, alpha=0.3)
        axes3[0].legend(['Y Error', 'Zero'], loc='upper right')
        
        # Roll subplot
        axes3[1].plot(times, roll_deg, 'purple', linewidth=1.5)
        axes3[1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes3[1].set_ylabel('Roll (degrees)')
        axes3[1].set_xlabel('Time (s)')
        axes3[1].grid(True, alpha=0.3)
        axes3[1].legend(['Roll', 'Zero'], loc='upper right')
        
        # Add correlation coefficient
        corr_roll_y = np.corrcoef(roll_deg, err_y)[0, 1]
        fig3.text(0.02, 0.98, f'Correlation(Roll, Y Error): {corr_roll_y:.3f}', 
                  transform=fig3.transFigure, fontsize=11,
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        roll_y_plot_path = os.path.join(self.output_dir, f'roll_y_error_{timestamp}.png')
        plt.savefig(roll_y_plot_path, dpi=150)
        self.get_logger().info(f'Roll vs Y Error plot saved to: {roll_y_plot_path}')
        
        # Close figures to free memory (don't show - we're running headless)
        plt.close('all')


def main():
    rclpy.init()
    node = HoverPlotter()
    
    def signal_handler(sig, frame):
        print('\n\nShutting down and plotting...')
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
