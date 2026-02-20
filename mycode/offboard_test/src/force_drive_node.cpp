/**
 * @file force_drive_node.cpp
 * @brief Drive the drone in the direction of estimated external force with obstacle avoidance.
 *
 * Integrates a Kalman filter to estimate external forces directly (no inter-node communication).
 * Uses OptiTrack TF for position and PX4 VehicleOdometry for attitude.
 * LiDAR-based obstacle avoidance adds repulsive forces from nearby obstacles.
 *
 * State machine:
 * 1. WAITING_FOR_POSE: Wait for valid OptiTrack data
 * 2. SENDING_SETPOINTS: Stream setpoints for 2s before arming (PX4 requirement)
 * 3. ARMING: Request offboard mode and arm
 * 4. TAKING_OFF: Commands position to target altitude, waits until reached
 * 5. POSITION_HOLD: Holds current position until external force detected
 * 6. VELOCITY: Drives drone in force direction, decelerates when force removed
 *
 * Obstacle avoidance (when enabled):
 * - Considers obstacles in a hemicircle facing F_ext direction
 * - Each obstacle generates repulsive force F = k/d^2 toward the drone
 * - F_cmd = F_ext + F_rep (superposition)
 * - Obstacle forces are NOT fed back into the Kalman filter
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ForceDriveNode : public rclcpp::Node
{
public:
    ForceDriveNode() : Node("force_drive_node")
    {
        // Parameters
        declare_parameter<double>("force_deadzone", 1.0);
        declare_parameter<double>("velocity_gain", 0.5);
        declare_parameter<double>("max_velocity", 1.0);
        declare_parameter<double>("decel_rate", 0.5);
        declare_parameter<double>("hold_speed_threshold", 0.05);
        declare_parameter<double>("takeoff_altitude", 0.2);
        declare_parameter<double>("altitude_threshold", 0.05);
        declare_parameter<double>("drone_mass", 1.56);
        declare_parameter<std::string>("world_frame", "optitrack");
        declare_parameter<std::string>("drone_frame", "imu_link");

        // Obstacle avoidance parameters
        declare_parameter<bool>("obstacle_avoidance_enabled", true);
        declare_parameter<double>("repulsion_gain", 0.5);
        declare_parameter<double>("min_obstacle_distance", 0.15);
        declare_parameter<double>("max_obstacle_distance", 2.0);
        declare_parameter<double>("hemicircle_radius_base", 0.5);
        declare_parameter<double>("hemicircle_radius_gain", 0.3);
        declare_parameter<double>("max_repulsion_force", 5.0);
        declare_parameter<int>("scan_downsample_factor", 4);

        force_deadzone_ = get_parameter("force_deadzone").as_double();
        velocity_gain_ = get_parameter("velocity_gain").as_double();
        max_velocity_ = get_parameter("max_velocity").as_double();
        decel_rate_ = get_parameter("decel_rate").as_double();
        hold_speed_threshold_ = get_parameter("hold_speed_threshold").as_double();
        takeoff_altitude_ = get_parameter("takeoff_altitude").as_double();
        altitude_threshold_ = get_parameter("altitude_threshold").as_double();
        mass_ = get_parameter("drone_mass").as_double();
        world_frame_ = get_parameter("world_frame").as_string();
        drone_frame_ = get_parameter("drone_frame").as_string();

        obstacle_avoidance_enabled_ = get_parameter("obstacle_avoidance_enabled").as_bool();
        repulsion_gain_ = get_parameter("repulsion_gain").as_double();
        min_obstacle_distance_ = get_parameter("min_obstacle_distance").as_double();
        max_obstacle_distance_ = get_parameter("max_obstacle_distance").as_double();
        hemicircle_radius_base_ = get_parameter("hemicircle_radius_base").as_double();
        hemicircle_radius_gain_ = get_parameter("hemicircle_radius_gain").as_double();
        max_repulsion_force_ = get_parameter("max_repulsion_force").as_double();
        scan_downsample_factor_ = get_parameter("scan_downsample_factor").as_int();
        if (scan_downsample_factor_ < 1) scan_downsample_factor_ = 1;

        // Initialize Kalman filter
        init_kalman_filter();

        // TF for position
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // QoS for PX4 publishers (best effort, volatile)
        rclcpp::QoS qos_pub(10);
        qos_pub.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_pub.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_pub.history(rclcpp::HistoryPolicy::KeepLast);

        // QoS for PX4 subscribers (best effort, transient local)
        rclcpp::QoS qos_sub(10);
        qos_sub.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_sub.durability(rclcpp::DurabilityPolicy::TransientLocal);
        qos_sub.history(rclcpp::HistoryPolicy::KeepLast);

        // Publishers
        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos_pub);
        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_pub);
        vehicle_command_pub_ = this->create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", qos_pub);

        // Visualization publishers
        obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/obstacle_markers", 10);
        force_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/force_marker", 10);

        // Subscribers
        vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos_sub,
            [this](const VehicleStatus::SharedPtr msg) {
                arming_state_ = msg->arming_state;
                nav_state_ = msg->nav_state;
            });

        odom_sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_sub,
            [this](const VehicleOdometry::SharedPtr msg) {
                latest_odom_ = *msg;
                have_odom_ = true;
            });

        // LaserScan subscriber (best effort, volatile for sensor data)
        rclcpp::QoS qos_scan(10);
        qos_scan.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_scan.durability(rclcpp::DurabilityPolicy::Volatile);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_scan,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                latest_scan_ = *msg;
                have_scan_ = true;
            });

        // Control loop at 50 Hz
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ForceDriveNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Force drive node started (with integrated Kalman filter)");
        RCLCPP_INFO(this->get_logger(),
            "  takeoff_alt=%.2f m, deadzone=%.2f N, gain=%.2f (m/s)/N, max_vel=%.2f m/s",
            takeoff_altitude_, force_deadzone_, velocity_gain_, max_velocity_);
        RCLCPP_INFO(this->get_logger(),
            "  obstacle_avoidance=%s, repulsion_gain=%.2f, hemicircle_base=%.2f m",
            obstacle_avoidance_enabled_ ? "ON" : "OFF", repulsion_gain_, hemicircle_radius_base_);
    }

private:
    enum class Mode {
        WAITING_FOR_POSE,
        SENDING_SETPOINTS,
        ARMING,
        TAKING_OFF,
        POSITION_HOLD,
        VELOCITY
    };

    static double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void init_kalman_filter()
    {
        // State: [px, vx, py, vy, Fx, Fy]
        const double dt = 0.02;  // 50 Hz

        kf_F_ = Eigen::Matrix<double, 6, 6>::Identity();
        kf_F_(0, 1) = dt;
        kf_F_(2, 3) = dt;
        kf_F_(1, 4) = dt / mass_;
        kf_F_(3, 5) = dt / mass_;

        kf_H_ = Eigen::Matrix<double, 2, 6>::Zero();
        kf_H_(0, 0) = 1.0;
        kf_H_(1, 2) = 1.0;

        // Process noise
        kf_Q_ = Eigen::Matrix<double, 6, 6>::Zero();
        kf_Q_(0, 0) = 0.01;   // pos
        kf_Q_(1, 1) = 0.1;    // vel
        kf_Q_(2, 2) = 0.01;   // pos
        kf_Q_(3, 3) = 0.1;    // vel
        kf_Q_(4, 4) = 1.0;    // force
        kf_Q_(5, 5) = 1.0;    // force

        // Measurement noise
        kf_R_ = Eigen::Matrix<double, 2, 2>::Zero();
        kf_R_(0, 0) = 0.001;
        kf_R_(1, 1) = 0.001;

        // Initial state and covariance
        kf_x_ = Eigen::Matrix<double, 6, 1>::Zero();
        kf_P_ = Eigen::Matrix<double, 6, 6>::Identity();
        kf_P_(4, 4) = 10.0;
        kf_P_(5, 5) = 10.0;
    }

    void update_force_estimate(double px_ned, double py_ned, double roll, double pitch, double yaw)
    {
        const double dt = 0.02;
        const double g = 9.81;
        // T such that vertical component T*cos(roll)*cos(pitch) = mass*g
        double cos_rp = std::cos(roll) * std::cos(pitch);
        if (std::abs(cos_rp) < 0.1) cos_rp = std::copysign(0.1, cos_rp);
        double T = mass_ * g / cos_rp;

        // Control input: thrust contribution to horizontal velocity (NED)
        // Full rotation: R_body_to_NED[:,2] gives the body-z axis in NED.
        // Thrust [0,0,-T] in body maps to NED horizontal:
        //   fx_NED = -T/m * (cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll))
        //   fy_NED = -T/m * (sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll))
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        Eigen::Matrix<double, 6, 1> Bu = Eigen::Matrix<double, 6, 1>::Zero();
        Bu(1) = -T / mass_ * (cos_yaw * std::sin(pitch) * std::cos(roll) + sin_yaw * std::sin(roll)) * dt;
        Bu(3) = -T / mass_ * (sin_yaw * std::sin(pitch) * std::cos(roll) - cos_yaw * std::sin(roll)) * dt;

        // Predict
        Eigen::Matrix<double, 6, 1> x_pred = kf_F_ * kf_x_ + Bu;
        Eigen::Matrix<double, 6, 6> P_pred = kf_F_ * kf_P_ * kf_F_.transpose() + kf_Q_;

        // Update
        Eigen::Matrix<double, 2, 1> z;
        z(0) = px_ned;
        z(1) = py_ned;

        Eigen::Matrix<double, 2, 1> y = z - kf_H_ * x_pred;
        Eigen::Matrix<double, 2, 2> S = kf_H_ * P_pred * kf_H_.transpose() + kf_R_;
        Eigen::Matrix<double, 6, 2> K = P_pred * kf_H_.transpose() * S.inverse();

        kf_x_ = x_pred + K * y;
        kf_P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * kf_H_) * P_pred;

        // Extract force estimate
        estimated_force_x_ = kf_x_(4);
        estimated_force_y_ = kf_x_(5);
    }

    bool get_current_pose_ned(double &x, double &y, double &z, double &yaw)
    {
        try {
            auto transform = tf_buffer_->lookupTransform(world_frame_, drone_frame_, tf2::TimePointZero);
            // OptiTrack world FLU (X-forward, Y-left, Z-up) -> NED
            x = transform.transform.translation.x;   // forward = north
            y = -transform.transform.translation.y;  // -left = east
            z = -transform.transform.translation.z;  // -up = down

            // Yaw from PX4 EKF output to match PX4's internal state exactly.
            // This avoids yaw conflicts between the setpoint and PX4's estimate.
            if (have_odom_) {
                double qw = latest_odom_.q[0];
                double qx = latest_odom_.q[1];
                double qy = latest_odom_.q[2];
                double qz = latest_odom_.q[3];
                yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz));
            } else {
                // Fallback to TF-derived yaw before PX4 odom is available
                // OptiTrack FLU -> NED quaternion: q_ned = (w, x, -y, -z)
                // NED yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2)) with substituted signs
                auto& q = transform.transform.rotation;
                yaw = std::atan2(
                    -2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            }
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    void get_attitude(double &roll, double &pitch)
    {
        roll = 0.0;
        pitch = 0.0;
        if (have_odom_) {
            double qw = latest_odom_.q[0];
            double qx = latest_odom_.q[1];
            double qy = latest_odom_.q[2];
            double qz = latest_odom_.q[3];

            double sinr_cosp = 2.0 * (qw * qx + qy * qz);
            double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
            roll = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2.0 * (qw * qy - qz * qx);
            if (std::abs(sinp) >= 1.0) {
                pitch = std::copysign(M_PI / 2.0, sinp);
            } else {
                pitch = std::asin(sinp);
            }
        }
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                VehicleCommand::ARMING_ACTION_ARM);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    /**
     * Compute repulsive force from LiDAR obstacles.
     *
     * Works in body frame for scan processing, converts result to NED.
     * Only considers obstacles in a hemicircle facing the F_ext direction.
     * Each obstacle contributes k/d^2 repulsion toward the drone.
     *
     * Architecture constraint: this output must NOT feed back into update_force_estimate().
     */
    void compute_obstacle_repulsion(double fx_ned, double fy_ned, double yaw_ned,
                                     double &frep_x_ned, double &frep_y_ned)
    {
        frep_x_ned = 0.0;
        frep_y_ned = 0.0;
        active_obstacle_points_.clear();

        if (!have_scan_) return;

        // Check scan staleness
        double scan_age = (this->now() - rclcpp::Time(latest_scan_.header.stamp)).seconds();
        if (scan_age > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Scan data stale (%.1fs old), obstacle avoidance disabled", scan_age);
            return;
        }

        // Convert F_ext from NED to base_link frame.
        // Step 1: NED -> FRD body (yaw rotation)
        double cos_yaw = std::cos(yaw_ned);
        double sin_yaw = std::sin(yaw_ned);
        double fx_frd =  cos_yaw * fx_ned + sin_yaw * fy_ned;
        double fy_frd = -sin_yaw * fx_ned + cos_yaw * fy_ned;
        // Step 2: FRD -> base_link FLU (x_base = x_frd forward, y_base = -y_frd left)
        double fx_base =  fx_frd;
        double fy_base = -fy_frd;
        double f_ext_mag = std::sqrt(fx_base * fx_base + fy_base * fy_base);

        if (f_ext_mag < force_deadzone_) return;

        // Hemicircle parameters (angle in base_link frame)
        double f_ext_angle = std::atan2(fy_base, fx_base);
        hemicircle_radius_ = std::min(
            hemicircle_radius_base_ + hemicircle_radius_gain_ * f_ext_mag,
            max_obstacle_distance_);
        hemicircle_angle_ = f_ext_angle;

        // Accumulate repulsive forces in body frame
        double frep_x_body = 0.0;
        double frep_y_body = 0.0;

        const auto& scan = latest_scan_;
        int n = static_cast<int>(scan.ranges.size());

        for (int i = 0; i < n; i += scan_downsample_factor_) {
            double range = scan.ranges[i];

            // Skip invalid ranges
            if (std::isnan(range) || std::isinf(range)) continue;
            if (range < scan.range_min || range > scan.range_max) continue;

            // Clamp close ranges to avoid singularity
            if (range < min_obstacle_distance_) range = min_obstacle_distance_;

            // Skip obstacles beyond hemicircle radius
            if (range > hemicircle_radius_) continue;

            // Scan point angle: convert from laser frame to body frame
            double angle_laser = scan.angle_min + i * scan.angle_increment;
            double angle_body = normalize_angle(angle_laser);

            // Hemicircle check: only consider obstacles within ±90° of movement direction
            double angle_diff = normalize_angle(angle_body - f_ext_angle);
            if (std::abs(angle_diff) > M_PI / 2.0) continue;

            // Repulsive force: k/d^2 pointing from obstacle toward drone
            double obs_x = range * std::cos(angle_body);
            double obs_y = range * std::sin(angle_body);
            double rep_mag = repulsion_gain_ / (range*range);

            frep_x_body += rep_mag * (-obs_x / range);
            frep_y_body += rep_mag * (-obs_y / range);

            // Store for visualization
            active_obstacle_points_.push_back({obs_x, obs_y});
        }

        // Clamp total repulsive force magnitude
        double frep_mag = std::sqrt(frep_x_body * frep_x_body + frep_y_body * frep_y_body);
        if (frep_mag > max_repulsion_force_) {
            double scale = max_repulsion_force_ / frep_mag;
            frep_x_body *= scale;
            frep_y_body *= scale;
        }

        // Store body-frame repulsion for visualization
        frep_body_x_ = frep_x_body;
        frep_body_y_ = frep_y_body;

        // Convert F_rep from base_link FLU to NED
        // FLU->FRD: (fx, -fy), then FRD->NED via R(yaw)
        // Combined: fx_ned = cos*fx + sin*fy, fy_ned = sin*fx - cos*fy
        frep_x_ned = cos_yaw * frep_x_body + sin_yaw * frep_y_body;
        frep_y_ned = sin_yaw * frep_x_body - cos_yaw * frep_y_body;
    }

    void publish_obstacle_markers(double fcmd_x_body, double fcmd_y_body)
    {
        visualization_msgs::msg::MarkerArray markers;
        auto stamp = this->now();

        // Marker 0: Hemicircle boundary (LINE_STRIP in base_link frame)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = stamp;
            m.ns = "obstacle_avoidance";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.02;  // line width
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);

            const int arc_points = 30;
            for (int i = 0; i <= arc_points; i++) {
                double angle = hemicircle_angle_ - M_PI / 2.0
                             + M_PI * static_cast<double>(i) / arc_points;
                geometry_msgs::msg::Point p;
                p.x = hemicircle_radius_ * std::cos(angle);
                p.y = hemicircle_radius_ * std::sin(angle);
                p.z = 0.0;
                m.points.push_back(p);
            }
            markers.markers.push_back(m);
        }

        // Marker 1: Active obstacle points (SPHERE_LIST in base_link frame)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = stamp;
            m.ns = "obstacle_avoidance";
            m.id = 1;
            m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.03; m.scale.y = 0.03; m.scale.z = 0.03;
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);

            for (const auto& pt : active_obstacle_points_) {
                geometry_msgs::msg::Point p;
                p.x = pt[0]; p.y = pt[1]; p.z = 0.0;
                m.points.push_back(p);
            }
            markers.markers.push_back(m);
        }

        // Marker 2: F_rep arrow (body frame, red)
        {
            double frep_mag = std::sqrt(frep_body_x_ * frep_body_x_ + frep_body_y_ * frep_body_y_);
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = stamp;
            m.ns = "obstacle_avoidance";
            m.id = 2;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);

            if (frep_mag > 0.01) {
                m.action = visualization_msgs::msg::Marker::ADD;
                geometry_msgs::msg::Point start, end;
                start.x = 0; start.y = 0; start.z = 0;
                end.x = frep_body_x_ * 0.1;
                end.y = frep_body_y_ * 0.1;
                end.z = 0;
                m.points.push_back(start);
                m.points.push_back(end);
                m.scale.x = 0.03;
                m.scale.y = 0.05;
                m.scale.z = 0.0;
                m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.9;
            } else {
                m.action = visualization_msgs::msg::Marker::DELETE;
            }
            markers.markers.push_back(m);
        }

        // Marker 3: F_cmd arrow (body frame, cyan)
        {
            double fcmd_mag = std::sqrt(fcmd_x_body * fcmd_x_body + fcmd_y_body * fcmd_y_body);
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = stamp;
            m.ns = "obstacle_avoidance";
            m.id = 3;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.lifetime = rclcpp::Duration::from_seconds(0.2);

            if (fcmd_mag > 0.01) {
                m.action = visualization_msgs::msg::Marker::ADD;
                geometry_msgs::msg::Point start, end;
                start.x = 0; start.y = 0; start.z = 0;
                end.x = fcmd_x_body * 0.1;
                end.y = fcmd_y_body * 0.1;
                end.z = 0;
                m.points.push_back(start);
                m.points.push_back(end);
                m.scale.x = 0.03;
                m.scale.y = 0.05;
                m.scale.z = 0.0;
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.9;
            } else {
                m.action = visualization_msgs::msg::Marker::DELETE;
            }
            markers.markers.push_back(m);
        }

        obstacle_marker_pub_->publish(markers);
    }

    void publish_force_marker(double fx_ned, double fy_ned)
    {
        try {
            auto tf = tf_buffer_->lookupTransform(world_frame_, drone_frame_, tf2::TimePointZero);

            // Convert NED force to OptiTrack world FLU: x_ot = x_ned, y_ot = -y_ned
            double fx_ot = fx_ned;   // NED north -> OptiTrack forward (same)
            double fy_ot = -fy_ned;  // NED east -> OptiTrack left (negate)
            double force_mag = std::sqrt(fx_ot * fx_ot + fy_ot * fy_ot);

            visualization_msgs::msg::MarkerArray markers;
            auto stamp = this->now();
            auto lifetime = rclcpp::Duration::from_seconds(0.5);

            constexpr double min_force = 0.3;
            if (force_mag < min_force) {
                // Show "Steady" text, delete arrow
                visualization_msgs::msg::Marker arrow;
                arrow.header.stamp = stamp;
                arrow.header.frame_id = world_frame_;
                arrow.ns = "force_estimate";
                arrow.id = 0;
                arrow.action = visualization_msgs::msg::Marker::DELETE;
                markers.markers.push_back(arrow);

                visualization_msgs::msg::Marker text;
                text.header.stamp = stamp;
                text.header.frame_id = world_frame_;
                text.ns = "force_estimate";
                text.id = 1;
                text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text.action = visualization_msgs::msg::Marker::ADD;
                text.lifetime = lifetime;
                text.scale.z = 0.15;
                text.color.r = 1.0; text.color.g = 1.0; text.color.b = 0.0; text.color.a = 1.0;
                text.text = "Steady";
                text.pose.position.x = tf.transform.translation.x;
                text.pose.position.y = tf.transform.translation.y;
                text.pose.position.z = tf.transform.translation.z + 0.3;
                markers.markers.push_back(text);
            } else {
                // Show arrow, delete text
                visualization_msgs::msg::Marker text;
                text.header.stamp = stamp;
                text.header.frame_id = world_frame_;
                text.ns = "force_estimate";
                text.id = 1;
                text.action = visualization_msgs::msg::Marker::DELETE;
                markers.markers.push_back(text);

                visualization_msgs::msg::Marker arrow;
                arrow.header.stamp = stamp;
                arrow.header.frame_id = world_frame_;
                arrow.ns = "force_estimate";
                arrow.id = 0;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.lifetime = lifetime;

                geometry_msgs::msg::Point start;
                start.x = tf.transform.translation.x;
                start.y = tf.transform.translation.y;
                start.z = tf.transform.translation.z;

                constexpr double arrow_scale = 0.1;
                double arrow_len = force_mag * arrow_scale;

                geometry_msgs::msg::Point end;
                end.x = start.x + fx_ot * arrow_scale;
                end.y = start.y + fy_ot * arrow_scale;
                end.z = start.z;

                arrow.points.push_back(start);
                arrow.points.push_back(end);

                double head_len = std::min(0.05, arrow_len * 0.3);
                arrow.scale.x = 0.03;
                arrow.scale.y = 0.06;
                arrow.scale.z = head_len;

                arrow.color.r = 0.0;
                arrow.color.g = 1.0;
                arrow.color.b = 0.0;
                arrow.color.a = 0.8;

                markers.markers.push_back(arrow);
            }

            force_marker_pub_->publish(markers);
        } catch (const tf2::TransformException &) {
            // Skip if TF not available
        }
    }

    void clear_obstacle_markers()
    {
        visualization_msgs::msg::MarkerArray markers;
        for (int id = 0; id < 4; id++) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = this->now();
            m.ns = "obstacle_avoidance";
            m.id = id;
            m.action = visualization_msgs::msg::Marker::DELETE;
            markers.markers.push_back(m);
        }
        obstacle_marker_pub_->publish(markers);
    }

    void control_loop()
    {
        const double dt = 0.02;
        auto now = this->now();

        // Get current pose
        double cur_x = 0, cur_y = 0, cur_z = 0, cur_yaw = 0;
        bool have_pose = get_current_pose_ned(cur_x, cur_y, cur_z, cur_yaw);

        // Get attitude and update force estimate (only when KF is initialized)
        double roll, pitch;
        get_attitude(roll, pitch);

        if (have_pose && kf_initialized_) {
            update_force_estimate(cur_x, cur_y, roll, pitch, cur_yaw);
        } else if (have_pose && !kf_initialized_) {
            // Initialize KF state with current position
            kf_x_(0) = cur_x;
            kf_x_(2) = cur_y;
            kf_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Kalman filter initialized at (%.3f, %.3f)", cur_x, cur_y);
        }

        double fx = estimated_force_x_;
        double fy = estimated_force_y_;
        double force_mag = std::sqrt(fx * fx + fy * fy);

        switch (mode_) {
            case Mode::WAITING_FOR_POSE:
            {
                if (have_pose) {
                    home_x_ = cur_x;
                    home_y_ = cur_y;
                    home_z_ = cur_z;
                    home_yaw_ = cur_yaw;
                    target_z_ = home_z_ - takeoff_altitude_;

                    RCLCPP_INFO(this->get_logger(),
                        "Pose acquired at NED (%.3f, %.3f, %.3f), target z=%.3f",
                        home_x_, home_y_, home_z_, target_z_);

                    mode_ = Mode::SENDING_SETPOINTS;
                    setpoint_start_time_ = now;
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Waiting for pose...");
                }
                break;
            }

            case Mode::SENDING_SETPOINTS:
            {
                // Keep tracking yaw as PX4's EKF converges to vision data
                if (have_pose) home_yaw_ = cur_yaw;

                publish_position_control_mode();
                publish_position_setpoint(home_x_, home_y_, home_z_, home_yaw_);

                if ((now - setpoint_start_time_).seconds() > 2.0) {
                    RCLCPP_INFO(this->get_logger(), "Requesting offboard mode + arm...");
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    arm();
                    mode_ = Mode::ARMING;
                    command_sent_time_ = now;
                }
                break;
            }

            case Mode::ARMING:
            {
                // Keep tracking yaw as PX4's EKF converges to vision data
                if (have_pose) home_yaw_ = cur_yaw;

                publish_position_control_mode();
                publish_position_setpoint(home_x_, home_y_, home_z_, home_yaw_);

                if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED &&
                    nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    RCLCPP_INFO(this->get_logger(), "Armed in offboard mode! Taking off...");
                    mode_ = Mode::TAKING_OFF;
                } else if ((now - command_sent_time_).seconds() > 1.0) {
                    if (nav_state_ != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Requesting offboard mode...");
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    }
                    if (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
                        arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
                        RCLCPP_INFO(this->get_logger(), "In offboard, sending arm...");
                        arm();
                    }
                    command_sent_time_ = now;
                }
                break;
            }

            case Mode::TAKING_OFF:
            {
                publish_position_control_mode();
                publish_position_setpoint(home_x_, home_y_, target_z_, home_yaw_);

                if (have_pose && std::abs(cur_z - target_z_) < altitude_threshold_) {
                    RCLCPP_INFO(this->get_logger(),
                        "Takeoff complete at z=%.3f, ready for force compliance", cur_z);
                    hold_x_ = cur_x;
                    hold_y_ = cur_y;
                    hold_z_ = target_z_;
                    hold_yaw_ = cur_yaw;

                    // Reset KF: clear takeoff transients so phantom forces don't trigger velocity mode
                    kf_x_(0) = cur_x;
                    kf_x_(1) = 0.0;  // vx
                    kf_x_(2) = cur_y;
                    kf_x_(3) = 0.0;  // vy
                    kf_x_(4) = 0.0;  // Fx
                    kf_x_(5) = 0.0;  // Fy
                    kf_P_ = Eigen::Matrix<double, 6, 6>::Identity();
                    kf_P_(4, 4) = 10.0;
                    kf_P_(5, 5) = 10.0;
                    estimated_force_x_ = 0.0;
                    estimated_force_y_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "KF reset for hover");

                    mode_ = Mode::POSITION_HOLD;
                }
                break;
            }

            case Mode::POSITION_HOLD:
            {
                // Transition to velocity mode only on external force
                if (force_mag > force_deadzone_) {
                    RCLCPP_INFO(this->get_logger(),
                        "Force detected (%.2f N), switching to velocity mode", force_mag);
                    mode_ = Mode::VELOCITY;
                } else {
                    publish_position_control_mode();
                    publish_position_setpoint(hold_x_, hold_y_, hold_z_, hold_yaw_);
                    clear_obstacle_markers();
                    break;
                }
            }
            [[fallthrough]];

            case Mode::VELOCITY:
            {
                if (force_mag > force_deadzone_) {
                    // Compute obstacle repulsion (does NOT affect KF estimates)
                    double frep_x = 0.0, frep_y = 0.0;
                    if (obstacle_avoidance_enabled_ && have_scan_) {
                        compute_obstacle_repulsion(fx, fy, cur_yaw, frep_x, frep_y);
                    }

                    // Superpose: F_cmd = F_ext + F_rep
                    double fcmd_x = fx + frep_x;
                    double fcmd_y = fy + frep_y;
                    double fcmd_mag = std::sqrt(fcmd_x * fcmd_x + fcmd_y * fcmd_y);

                    if (fcmd_mag > 0.01) {
                        double dir_x = fcmd_x / fcmd_mag;
                        double dir_y = fcmd_y / fcmd_mag;
                        double effective_force = std::max(fcmd_mag - force_deadzone_, 0.0);
                        double target_speed = std::min(effective_force * velocity_gain_, max_velocity_);

                        cmd_vx_ = dir_x * target_speed;
                        cmd_vy_ = dir_y * target_speed;
                    } else {
                        // F_ext and F_rep cancel out - stop
                        cmd_vx_ = 0.0;
                        cmd_vy_ = 0.0;
                    }

                    last_frep_x_ = frep_x;
                    last_frep_y_ = frep_y;

                    publish_velocity_control_mode();
                    publish_velocity_setpoint(cmd_vx_, cmd_vy_, hold_z_, hold_yaw_);

                    // Publish visualization (compute F_cmd in base_link frame for markers)
                    if (obstacle_avoidance_enabled_) {
                        double cos_yaw = std::cos(cur_yaw);
                        double sin_yaw = std::sin(cur_yaw);
                        // NED -> FRD -> base_link FLU (negate y)
                        double fcmd_body_x = cos_yaw * fcmd_x + sin_yaw * fcmd_y;
                        double fcmd_body_y = sin_yaw * fcmd_x - cos_yaw * fcmd_y;
                        publish_obstacle_markers(fcmd_body_x, fcmd_body_y);
                    }

                } else {
                    double cmd_speed = std::sqrt(cmd_vx_ * cmd_vx_ + cmd_vy_ * cmd_vy_);

                    if (cmd_speed > hold_speed_threshold_) {
                        double new_speed = std::max(cmd_speed - decel_rate_ * dt, 0.0);
                        double scale = new_speed / cmd_speed;
                        cmd_vx_ *= scale;
                        cmd_vy_ *= scale;

                        publish_velocity_control_mode();
                        publish_velocity_setpoint(cmd_vx_, cmd_vy_, hold_z_, hold_yaw_);

                    } else {
                        cmd_vx_ = 0.0;
                        cmd_vy_ = 0.0;

                        if (have_pose) {
                            hold_x_ = cur_x;
                            hold_y_ = cur_y;
                            hold_yaw_ = cur_yaw;
                            mode_ = Mode::POSITION_HOLD;
                            RCLCPP_INFO(this->get_logger(),
                                "Position hold at NED (%.3f, %.3f, %.3f)",
                                hold_x_, hold_y_, hold_z_);
                        }

                        publish_position_control_mode();
                        publish_position_setpoint(hold_x_, hold_y_, hold_z_, hold_yaw_);
                    }

                    last_frep_x_ = 0.0;
                    last_frep_y_ = 0.0;
                    clear_obstacle_markers();
                }
                break;
            }
        }

        // Publish force marker when KF is running
        if (kf_initialized_) {
            publish_force_marker(fx, fy);
        }

        // Log at ~2 Hz
        static int log_counter = 0;
        if (++log_counter >= 25) {
            log_counter = 0;
            const char* mode_str;
            switch (mode_) {
                case Mode::WAITING_FOR_POSE: mode_str = "WAIT_POSE"; break;
                case Mode::SENDING_SETPOINTS: mode_str = "SETPOINTS"; break;
                case Mode::ARMING: mode_str = "ARMING"; break;
                case Mode::TAKING_OFF: mode_str = "TAKEOFF"; break;
                case Mode::POSITION_HOLD: mode_str = "HOLD"; break;
                case Mode::VELOCITY: mode_str = "VEL"; break;
            }

            if (mode_ == Mode::VELOCITY) {
                double speed = std::sqrt(cmd_vx_ * cmd_vx_ + cmd_vy_ * cmd_vy_);
                double frep_mag = std::sqrt(last_frep_x_ * last_frep_x_ + last_frep_y_ * last_frep_y_);
                RCLCPP_INFO(this->get_logger(),
                    "[%s] vel=(%.2f, %.2f) spd=%.2f | Fext=(%.2f, %.2f) %.2fN | Frep=(%.2f, %.2f) %.2fN",
                    mode_str, cmd_vx_, cmd_vy_, speed,
                    fx, fy, force_mag,
                    last_frep_x_, last_frep_y_, frep_mag);
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "[%s] pos=(%.3f, %.3f, %.3f) | force=(%.2f, %.2f) mag=%.2f N",
                    mode_str, hold_x_, hold_y_, hold_z_, fx, fy, force_mag);
            }
        }
    }

    void publish_velocity_control_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_position_control_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_velocity_setpoint(double vx, double vy, double z, double yaw)
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position[0] = std::nanf("");
        msg.position[1] = std::nanf("");
        msg.position[2] = z;
        msg.velocity[0] = vx;
        msg.velocity[1] = vy;
        msg.velocity[2] = std::nanf("");
        msg.acceleration[0] = std::nanf("");
        msg.acceleration[1] = std::nanf("");
        msg.acceleration[2] = std::nanf("");
        msg.jerk[0] = std::nanf("");
        msg.jerk[1] = std::nanf("");
        msg.jerk[2] = std::nanf("");
        msg.yaw = yaw;
        msg.yawspeed = std::nanf("");
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_position_setpoint(double x, double y, double z, double yaw)
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position[0] = x;
        msg.position[1] = y;
        msg.position[2] = z;
        msg.velocity[0] = std::nanf("");
        msg.velocity[1] = std::nanf("");
        msg.velocity[2] = std::nanf("");
        msg.acceleration[0] = std::nanf("");
        msg.acceleration[1] = std::nanf("");
        msg.acceleration[2] = std::nanf("");
        msg.jerk[0] = std::nanf("");
        msg.jerk[1] = std::nanf("");
        msg.jerk[2] = std::nanf("");
        msg.yaw = yaw;
        msg.yawspeed = std::nanf("");
        trajectory_setpoint_pub_->publish(msg);
    }

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string world_frame_;
    std::string drone_frame_;

    // Subscribers
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr force_marker_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Kalman filter matrices
    Eigen::Matrix<double, 6, 6> kf_F_;
    Eigen::Matrix<double, 2, 6> kf_H_;
    Eigen::Matrix<double, 6, 6> kf_Q_;
    Eigen::Matrix<double, 2, 2> kf_R_;
    Eigen::Matrix<double, 6, 1> kf_x_;
    Eigen::Matrix<double, 6, 6> kf_P_;
    bool kf_initialized_ = false;

    // PX4 data
    VehicleOdometry latest_odom_;
    bool have_odom_ = false;

    // LiDAR data
    sensor_msgs::msg::LaserScan latest_scan_;
    bool have_scan_ = false;

    // Force estimate
    double estimated_force_x_ = 0.0;
    double estimated_force_y_ = 0.0;

    // Parameters
    double force_deadzone_;
    double velocity_gain_;
    double max_velocity_;
    double decel_rate_;
    double hold_speed_threshold_;
    double takeoff_altitude_;
    double altitude_threshold_;
    double mass_;

    // Obstacle avoidance parameters
    bool obstacle_avoidance_enabled_;
    double repulsion_gain_;
    double min_obstacle_distance_;
    double max_obstacle_distance_;
    double hemicircle_radius_base_;
    double hemicircle_radius_gain_;
    double max_repulsion_force_;
    int scan_downsample_factor_;

    // Obstacle avoidance state
    double last_frep_x_ = 0.0;
    double last_frep_y_ = 0.0;
    double frep_body_x_ = 0.0;
    double frep_body_y_ = 0.0;
    double hemicircle_radius_ = 0.0;
    double hemicircle_angle_ = 0.0;
    std::vector<std::array<double, 2>> active_obstacle_points_;

    // PX4 state
    uint8_t arming_state_ = VehicleStatus::ARMING_STATE_DISARMED;
    uint8_t nav_state_ = VehicleStatus::NAVIGATION_STATE_MANUAL;

    // State machine
    Mode mode_ = Mode::WAITING_FOR_POSE;
    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;

    // Home position
    double home_x_ = 0.0;
    double home_y_ = 0.0;
    double home_z_ = 0.0;
    double home_yaw_ = 0.0;
    double target_z_ = 0.0;

    // Position hold state
    double hold_x_ = 0.0;
    double hold_y_ = 0.0;
    double hold_z_ = 0.0;
    double hold_yaw_ = 0.0;

    // Timing
    rclcpp::Time setpoint_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time command_sent_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceDriveNode>());
    rclcpp::shutdown();
    return 0;
}
