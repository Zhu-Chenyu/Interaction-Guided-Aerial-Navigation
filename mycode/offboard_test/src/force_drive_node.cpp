/**
 * @file force_drive_node.cpp
 * @brief Drive the drone in the direction of estimated external force.
 *
 * Integrates a Kalman filter to estimate external forces directly (no inter-node communication).
 * Uses OptiTrack TF for position and PX4 VehicleOdometry for attitude.
 *
 * State machine:
 * 1. WAITING_FOR_POSE: Wait for valid OptiTrack data
 * 2. SENDING_SETPOINTS: Stream setpoints for 2s before arming (PX4 requirement)
 * 3. ARMING: Request offboard mode and arm
 * 4. TAKING_OFF: Commands position to target altitude, waits until reached
 * 5. POSITION_HOLD: Holds current position until external force detected
 * 6. VELOCITY: Drives drone in force direction, decelerates when force removed
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

#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ForceDriveNode : public rclcpp::Node
{
public:
    ForceDriveNode() : Node("force_drive_node")
    {
        // Parameters
        this->declare_parameter<double>("force_deadzone", 1.0);
        this->declare_parameter<double>("velocity_gain", 0.5);
        this->declare_parameter<double>("max_velocity", 1.0);
        this->declare_parameter<double>("decel_rate", 0.5);
        this->declare_parameter<double>("hold_speed_threshold", 0.05);
        this->declare_parameter<double>("takeoff_altitude", 0.2);
        this->declare_parameter<double>("altitude_threshold", 0.05);
        this->declare_parameter<double>("drone_mass", 1.56);
        this->declare_parameter<std::string>("world_frame", "optitrack");
        this->declare_parameter<std::string>("drone_frame", "imu_link");

        force_deadzone_ = this->get_parameter("force_deadzone").as_double();
        velocity_gain_ = this->get_parameter("velocity_gain").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        decel_rate_ = this->get_parameter("decel_rate").as_double();
        hold_speed_threshold_ = this->get_parameter("hold_speed_threshold").as_double();
        takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
        altitude_threshold_ = this->get_parameter("altitude_threshold").as_double();
        mass_ = this->get_parameter("drone_mass").as_double();
        world_frame_ = this->get_parameter("world_frame").as_string();
        drone_frame_ = this->get_parameter("drone_frame").as_string();

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

        // Control loop at 50 Hz
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ForceDriveNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Force drive node started (with integrated Kalman filter)");
        RCLCPP_INFO(this->get_logger(),
            "  takeoff_alt=%.2f m, deadzone=%.2f N, gain=%.2f (m/s)/N, max_vel=%.2f m/s",
            takeoff_altitude_, force_deadzone_, velocity_gain_, max_velocity_);
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

    void update_force_estimate(double px_ned, double py_ned, double roll, double pitch)
    {
        const double dt = 0.02;
        const double g = 9.81;
        double T = mass_ * g;  // Assume hover thrust

        // Control input: thrust contribution to velocity
        Eigen::Matrix<double, 6, 1> Bu = Eigen::Matrix<double, 6, 1>::Zero();
        Bu(1) = T * std::sin(pitch) / mass_ * dt;
        Bu(3) = -T * std::sin(roll) * std::cos(pitch) / mass_ * dt;

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
            x = transform.transform.translation.x;
            y = -transform.transform.translation.y;
            z = -transform.transform.translation.z;

            auto& q = transform.transform.rotation;
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            yaw = -std::atan2(siny_cosp, cosy_cosp);
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

    void control_loop()
    {
        const double dt = 0.02;
        auto now = this->now();

        // Get current pose
        double cur_x, cur_y, cur_z, cur_yaw;
        bool have_pose = get_current_pose_ned(cur_x, cur_y, cur_z, cur_yaw);

        // Get attitude and update force estimate
        double roll, pitch;
        get_attitude(roll, pitch);

        if (have_pose && kf_initialized_) {
            update_force_estimate(cur_x, cur_y, roll, pitch);
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
                    mode_ = Mode::POSITION_HOLD;
                }
                break;
            }

            case Mode::POSITION_HOLD:
            {
                if (force_mag > force_deadzone_) {
                    RCLCPP_INFO(this->get_logger(),
                        "Force detected (%.2f N), switching to velocity mode", force_mag);
                    mode_ = Mode::VELOCITY;
                } else {
                    publish_position_control_mode();
                    publish_position_setpoint(hold_x_, hold_y_, hold_z_, hold_yaw_);
                    break;
                }
            }
            [[fallthrough]];

            case Mode::VELOCITY:
            {
                if (force_mag > force_deadzone_) {
                    // Drive IN SAME direction as force (compliant - follow the push)
                    double dir_x = fx / force_mag;
                    double dir_y = fy / force_mag;
                    double effective_force = force_mag - force_deadzone_;
                    double target_speed = std::min(effective_force * velocity_gain_, max_velocity_);

                    cmd_vx_ = dir_x * target_speed;
                    cmd_vy_ = dir_y * target_speed;

                    publish_velocity_control_mode();
                    publish_velocity_setpoint(cmd_vx_, cmd_vy_, hold_z_);

                } else {
                    double cmd_speed = std::sqrt(cmd_vx_ * cmd_vx_ + cmd_vy_ * cmd_vy_);

                    if (cmd_speed > hold_speed_threshold_) {
                        double new_speed = std::max(cmd_speed - decel_rate_ * dt, 0.0);
                        double scale = new_speed / cmd_speed;
                        cmd_vx_ *= scale;
                        cmd_vy_ *= scale;

                        publish_velocity_control_mode();
                        publish_velocity_setpoint(cmd_vx_, cmd_vy_, hold_z_);

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
                }
                break;
            }
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
                RCLCPP_INFO(this->get_logger(),
                    "[%s] vel=(%.2f, %.2f) spd=%.2f | force=(%.2f, %.2f) mag=%.2f N",
                    mode_str, cmd_vx_, cmd_vy_, speed, fx, fy, force_mag);
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

    void publish_velocity_setpoint(double vx, double vy, double z)
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
        msg.yaw = std::nanf("");
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

    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

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
