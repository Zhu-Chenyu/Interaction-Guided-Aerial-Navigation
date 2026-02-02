/**
 * @file offboard_test_node.cpp
 * @brief Offboard control test for PX4 with OptiTrack position estimation
 *
 * This node:
 * 1. Waits for OptiTrack pose (via TF)
 * 2. Validates pose stability for safety (3 seconds)
 * 3. Publishes VehicleOdometry to PX4 for position estimation (50Hz)
 * 4. Sends offboard control mode continuously
 * 5. Arms the drone after validation
 * 6. Commands takeoff to target height
 * 7. Hovers for specified duration
 * 8. Lands and disarms
 *
 * SAFETY: Requires stable OptiTrack tracking before arming.
 * WARNING: Test in simulation first! This is DANGEROUS on real hardware.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardTestNode : public rclcpp::Node
{
public:
    OffboardTestNode() : Node("offboard_test_node")
    {
        // Parameters
        this->declare_parameter<std::string>("world_frame", "optitrack");
        this->declare_parameter<std::string>("drone_frame", "imu_link");
        this->declare_parameter<double>("takeoff_height", 1.0);   // meters (NED: negative)
        this->declare_parameter<double>("hover_duration", 5.0);   // seconds
        this->declare_parameter<double>("validation_duration", 3.0);  // seconds to validate pose
        this->declare_parameter<double>("max_position_variance", 0.01);  // m, max allowed variance

        world_frame_ = this->get_parameter("world_frame").as_string();
        drone_frame_ = this->get_parameter("drone_frame").as_string();
        takeoff_height_ = this->get_parameter("takeoff_height").as_double();
        hover_duration_ = this->get_parameter("hover_duration").as_double();
        validation_duration_ = this->get_parameter("validation_duration").as_double();
        max_position_variance_ = this->get_parameter("max_position_variance").as_double();

        // TF setup for OptiTrack pose
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // QoS for PX4 publishers — must match PX4's subscriber QoS (best effort, volatile)
        rclcpp::QoS qos_pub(10);
        qos_pub.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_pub.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_pub.history(rclcpp::HistoryPolicy::KeepLast);

        // QoS for PX4 subscribers (best effort, transient_local to match PX4's /fmu/out/*)
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
            std::bind(&OffboardTestNode::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_command_ack_sub_ = this->create_subscription<VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", qos_sub,
            std::bind(&OffboardTestNode::command_ack_callback, this, std::placeholders::_1));

        // Main control loop at 50 Hz
        timer_ = this->create_wall_timer(
            20ms, std::bind(&OffboardTestNode::control_loop, this));

        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Offboard test node started");
        RCLCPP_INFO(this->get_logger(), "Using frames: world='%s', drone='%s'", 
                    world_frame_.c_str(), drone_frame_.c_str());
        RCLCPP_WARN(this->get_logger(), "WARNING: Make sure you have a way to regain control!");
        RCLCPP_INFO(this->get_logger(), "Will validate pose for %.1f seconds before arming", 
                    validation_duration_);
    }

private:
    // State machine states
    enum class State {
        WAITING_FOR_POSE,      // Wait for OptiTrack
        VALIDATING_POSE,       // Validate pose stability before proceeding
        SENDING_SETPOINTS,     // Stream OCM + setpoints for 2s before arming
        ARMING,                // Send arm + offboard commands, wait for confirmation
        TAKING_OFF,            // Command takeoff
        HOVERING,              // Hover in place
        LANDING,               // Land
        DISARMING,             // Turn on the safety switch
        DONE                   // Finished
    };

    void control_loop()
    {
        auto now = this->now();

        // Try to get current pose from OptiTrack
        bool have_pose = get_current_pose();

        // Vision odometry is published by vision_bridge_node
        if (have_pose) {
            consecutive_pose_count_++;
        } else {
            consecutive_pose_count_ = 0;
            // If we lose pose during flight, this is dangerous
            if (state_ != State::WAITING_FOR_POSE && state_ != State::DONE) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "LOST OPTITRACK POSE! State: %d", static_cast<int>(state_));
            }
        }

        // Publish offboard control mode from SENDING_SETPOINTS onward.
        // PX4 needs OCM streaming before it accepts offboard mode switch.
        // Modern PX4 allows arming while OCM is streaming.
        if (state_ == State::SENDING_SETPOINTS || state_ == State::ARMING ||
            state_ == State::TAKING_OFF || state_ == State::HOVERING ||
            state_ == State::LANDING || state_ == State::DISARMING) {
            publish_offboard_control_mode();
        }

        switch (state_) {
            case State::WAITING_FOR_POSE:
                if (have_pose) {
                    RCLCPP_INFO(this->get_logger(), "Got pose from OptiTrack, starting validation");
                    state_ = State::VALIDATING_POSE;
                    validation_start_time_ = now;
                    // Initialize validation tracking
                    validation_pose_count_ = 0;
                    validation_sum_x_ = 0.0;
                    validation_sum_y_ = 0.0;
                    validation_sum_z_ = 0.0;
                    validation_sum_x2_ = 0.0;
                    validation_sum_y2_ = 0.0;
                    validation_sum_z2_ = 0.0;
                }
                break;

            case State::VALIDATING_POSE:
                if (!have_pose) {
                    RCLCPP_WARN(this->get_logger(), "Lost pose during validation, restarting");
                    state_ = State::WAITING_FOR_POSE;
                    break;
                }
                
                // Accumulate statistics
                validation_pose_count_++;
                validation_sum_x_ += current_x_;
                validation_sum_y_ += current_y_;
                validation_sum_z_ += current_z_;
                validation_sum_x2_ += current_x_ * current_x_;
                validation_sum_y2_ += current_y_ * current_y_;
                validation_sum_z2_ += current_z_ * current_z_;
                
                if ((now - validation_start_time_).seconds() > validation_duration_) {
                    // Calculate variance
                    double n = static_cast<double>(validation_pose_count_);
                    double var_x = (validation_sum_x2_ / n) - std::pow(validation_sum_x_ / n, 2);
                    double var_y = (validation_sum_y2_ / n) - std::pow(validation_sum_y_ / n, 2);
                    double var_z = (validation_sum_z2_ / n) - std::pow(validation_sum_z_ / n, 2);
                    double max_var = std::max({var_x, var_y, var_z});
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "Validation complete: variance (x=%.6f, y=%.6f, z=%.6f), max=%.6f",
                        var_x, var_y, var_z, max_var);
                    
                    if (max_var <= max_position_variance_) {
                        RCLCPP_INFO(this->get_logger(), 
                            "Pose is stable! Starting setpoint stream");
                        // Record initial position and yaw as home
                        home_x_ = current_x_;
                        home_y_ = current_y_;
                        home_z_ = current_z_;
                        home_yaw_ = current_yaw_;
                        RCLCPP_INFO(this->get_logger(), "Home yaw: %.1f degrees",
                                    home_yaw_ * 180.0 / M_PI);
                        state_ = State::SENDING_SETPOINTS;
                        setpoint_start_time_ = now;
                    } else {
                        RCLCPP_WARN(this->get_logger(), 
                            "Pose variance too high (%.6f > %.6f), restarting validation",
                            max_var, max_position_variance_);
                        state_ = State::WAITING_FOR_POSE;
                    }
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Validating pose: %.1f/%.1f seconds, pos=(%.3f, %.3f, %.3f)",
                        (now - validation_start_time_).seconds(), validation_duration_,
                        current_x_, current_y_, current_z_);
                }
                break;

            case State::SENDING_SETPOINTS:
                // Stream OCM + trajectory setpoints so PX4 is ready for offboard.
                publish_trajectory_setpoint(home_x_, home_y_, home_z_, home_yaw_);
                if ((now - setpoint_start_time_).seconds() > 2.0) {
                    // Same order as official PX4 example: offboard mode first, then arm
                    RCLCPP_INFO(this->get_logger(), "Requesting offboard mode + arm...");
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    arm();
                    state_ = State::ARMING;
                    command_sent_time_ = now;
                }
                break;

            case State::ARMING:
                publish_trajectory_setpoint(home_x_, home_y_, home_z_, home_yaw_);

                if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED &&
                    nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    RCLCPP_INFO(this->get_logger(),
                        "Armed in offboard mode! Taking off to %.1f m...", takeoff_height_);
                    state_ = State::TAKING_OFF;
                    takeoff_start_time_ = now;
                } else if ((now - command_sent_time_).seconds() > 1.0) {
                    // Request offboard mode if not yet confirmed
                    if (nav_state_ != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Requesting offboard mode (nav=%d)...", nav_state_);
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    }
                    // Retry arm if in offboard but not yet armed
                    if (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
                        arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
                        RCLCPP_INFO(this->get_logger(), "In offboard mode, sending arm command...");
                        arm();
                    }
                    command_sent_time_ = now;
                }
                break;

            case State::TAKING_OFF:
                {
                    // Set target altitude directly; PX4 position controller handles the trajectory
                    double target_z = home_z_ - takeoff_height_;
                    publish_trajectory_setpoint(home_x_, home_y_, target_z, home_yaw_);

                    // Check if we've reached target altitude (within 0.05m)
                    if (std::abs(current_z_ - target_z) < 0.05) {
                        RCLCPP_INFO(this->get_logger(), "Reached altitude, hovering for %.1f seconds...", hover_duration_);
                        state_ = State::HOVERING;
                        hover_start_time_ = now;
                    }
                }
                break;

            case State::HOVERING:
                publish_trajectory_setpoint(home_x_, home_y_, home_z_ - takeoff_height_, home_yaw_);
                if ((now - hover_start_time_).seconds() > hover_duration_) {
                    RCLCPP_INFO(this->get_logger(), "Hover complete, landing...");
                    state_ = State::LANDING;
                    landing_start_time_ = now;
                }
                break;

            case State::LANDING:
                {
                    double t = (now - landing_start_time_).seconds();
                    double descent_rate = 0.3;  // m/s
                    double current_alt = takeoff_height_ - t * descent_rate;
                    if (current_alt < 0.1) {
                        current_alt = 0.0;
                    }
                    double target_z = home_z_ - current_alt;
                    publish_trajectory_setpoint(home_x_, home_y_, target_z, home_yaw_);

                    if (current_alt <= 0.0) {
                        RCLCPP_INFO(this->get_logger(), "Landed, disarming...");
                        disarm();
                        state_ = State::DISARMING;
                        command_sent_time_ = now;
                    }
                }
                break;

            case State::DISARMING:
                publish_trajectory_setpoint(home_x_, home_y_, home_z_, home_yaw_);
                if (arming_state_ == VehicleStatus::ARMING_STATE_DISARMED) {
                    RCLCPP_INFO(this->get_logger(), "Disarmed. Test complete!");
                    state_ = State::DONE;
                } else if ((now - command_sent_time_).seconds() > 2.0) {
                    disarm();
                    command_sent_time_ = now;
                }
                break;

            case State::DONE:
                // Stop publishing, we're done
                timer_->cancel();
                rclcpp::shutdown();
                break;
        }
    }

    bool get_current_pose()
    {
        try {
            auto transform = tf_buffer_->lookupTransform(world_frame_, drone_frame_, tf2::TimePointZero);

            // Convert from OptiTrack to PX4 NED (negate Y and Z)
            current_x_ = transform.transform.translation.x;
            current_y_ = -transform.transform.translation.y;
            current_z_ = -transform.transform.translation.z;

            // Extract yaw from OptiTrack quaternion (Z-up), then negate for NED
            auto& q = transform.transform.rotation;
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            current_yaw_ = -std::atan2(siny_cosp, cosy_cosp);  // negate for NED

            return true;
        } catch (const tf2::TransformException& ex) {
            if (state_ == State::WAITING_FOR_POSE) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Waiting for TF: %s -> %s", world_frame_.c_str(), drone_frame_.c_str());
            }
            return false;
        }
    }

    void publish_offboard_control_mode()
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

    void publish_trajectory_setpoint(double x, double y, double z, double yaw)
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

    void disarm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                VehicleCommand::ARMING_ACTION_DISARM);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent");
    }

    void vehicle_status_callback(const VehicleStatus::SharedPtr msg)
    {
        arming_state_ = msg->arming_state;
        nav_state_ = msg->nav_state;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "PX4 status: arming=%d, nav=%d", arming_state_, nav_state_);
    }

    void command_ack_callback(const VehicleCommandAck::SharedPtr msg)
    {
        if (msg->result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
            RCLCPP_DEBUG(this->get_logger(), "Command %d accepted", msg->command);
        } else {
            RCLCPP_WARN(this->get_logger(), "Command %d failed with result %d",
                        msg->command, msg->result);
        }
    }

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string world_frame_;
    std::string drone_frame_;

    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Subscribers
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    State state_ = State::WAITING_FOR_POSE;
    uint8_t arming_state_ = VehicleStatus::ARMING_STATE_DISARMED;
    uint8_t nav_state_ = VehicleStatus::NAVIGATION_STATE_MANUAL;

    // Position tracking (NED for setpoints)
    double current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
    double current_yaw_ = 0.0;
    double home_x_ = 0.0, home_y_ = 0.0, home_z_ = 0.0;
    double home_yaw_ = 0.0;
    
    // Pose validation
    int consecutive_pose_count_ = 0;
    int validation_pose_count_ = 0;
    double validation_sum_x_ = 0.0, validation_sum_y_ = 0.0, validation_sum_z_ = 0.0;
    double validation_sum_x2_ = 0.0, validation_sum_y2_ = 0.0, validation_sum_z2_ = 0.0;

    // Parameters
    double takeoff_height_;
    double hover_duration_;
    double validation_duration_;
    double max_position_variance_;

    // Timing
    rclcpp::Time start_time_;
    rclcpp::Time validation_start_time_;
    rclcpp::Time setpoint_start_time_;
    rclcpp::Time command_sent_time_;
    rclcpp::Time takeoff_start_time_;
    rclcpp::Time hover_start_time_;
    rclcpp::Time landing_start_time_;
};

int main(int argc, char* argv[])
{
    std::cout << "========================================" << std::endl;
    std::cout << "   PX4 Offboard Control Test Node      " << std::endl;
    std::cout << "   with OptiTrack Position Estimation  " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "WARNING: This will ARM and FLY the drone!" << std::endl;
    std::cout << "Make sure you have emergency stop ready!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Pre-flight checklist:" << std::endl;
    std::cout << "  1. OptiTrack is tracking the drone" << std::endl;
    std::cout << "  2. PX4 EKF2_EV_CTRL = 7" << std::endl;
    std::cout << "  3. PX4 EKF2_HGT_REF = 3" << std::endl;
    std::cout << "  4. Emergency stop is ready" << std::endl;
    std::cout << "========================================" << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardTestNode>());
    rclcpp::shutdown();
    return 0;
}
