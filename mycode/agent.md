# Winter Project - Agent Context

Quick reference for the drone visualization project.

> [!IMPORTANT]
> **Keep this file updated!** When making any configuration changes to the system, update this document to reflect the current state.

## Network Configuration

| Device | IP Address | Credentials |
|--------|------------|-------------|
| **Raspberry Pi 5** | `192.168.20.232` | `ubuntu` / `shelby` |
| **WiFi (NUMSR2_5G)** | Gateway: `192.168.20.1` | Password: `Robotics1!` |

## SSH Access

```bash
# Passwordless (SSH key installed)
ssh -i ~/.ssh/rpi_key ubuntu@192.168.20.232

# Or with password
ssh ubuntu@192.168.20.232
# Password: shelby
```

> [!TIP]
> **For agents**: Establish one SSH session and run multiple commands within it, rather than starting a new `ssh` connection for each command. This avoids repeated connection overhead.

## Auto-Start Service (RPi)

The IMU publisher starts automatically on boot via systemd:
```bash
# Check status
sudo systemctl status imu-publisher

# Restart if needed
sudo systemctl restart imu-publisher

# View logs
journalctl -u imu-publisher -f
```

## Troubleshooting

> [!WARNING]
> **Topics not visible after moving RPi?** If the RPi was moved to a different network (or powered on elsewhere), DDS discovery may have stale network bindings. Restart the service:
> ```bash
> ssh ubuntu@192.168.20.232 "sudo systemctl restart imu-publisher"
> ```

## Hardware Connections

- **Pixhawk** → RPi via UART (`/dev/ttyAMA0`, **921600 baud**, TELEM2 port)
- **RPi** → Laptop via WiFi (NUMSR2_5G network)

## Software Stack

| Component | RPi | Laptop |
|-----------|-----|--------|
| **OS** | Ubuntu 24.04 LTS (aarch64) | Linux |
| **ROS 2** | Jazzy | Kilted |
| **MAVLink** | pymavlink installed | N/A |

## ROS 2 Multi-Machine Setup

Both machines use `ROS_DOMAIN_ID=42` (already in `~/.bashrc` on both RPi and laptop).
New terminals will have this set automatically.

## Key Directories

- **RPi ROS workspace**: `~/ros2_ws/`
- **Laptop workspace**: `/home/chenyu-zhu/winter_project/code/src/mycode/`

## Topics

| Topic | Message Type | Publisher | Description |
|-------|--------------|-----------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | RPi | IMU orientation and acceleration |
| `/tf` | `tf2_msgs/TFMessage` | RPi | Transform: world → imu_link |

## Quick Commands

```bash
# On RPi: Start IMU publisher
ros2 run imu_publisher imu_node

# On Laptop: Check topics from RPi
ros2 topic list
ros2 topic echo /imu/data

# On Laptop: Launch RViz
rviz2
```
