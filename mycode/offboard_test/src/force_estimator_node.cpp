/**
 * @file force_estimator_node.cpp
 * @brief Kalman filter based horizontal external force estimator for a drone.
 *
 * Uses OptiTrack position, PX4 attitude (VehicleOdometry), and PX4 thrust
 * (VehicleThrustSetpoint) to estimate horizontal external forces in the NED frame.
 *
 * State vector: [px, vx, py, vy, F_ext_x, F_ext_y]
 * Measurement:  [px, py] from OptiTrack TF
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include <Eigen/Dense>

using namespace std::chrono_literals;

class ForceEstimatorNode : public rclcpp::Node
{
public:
    ForceEstimatorNode() : Node("force_estimator_node")
    {
        // Parameters
        this->declare_parameter<double>("drone_mass", 1.56);
        this->declare_parameter<double>("hover_thrust_normalized", 0.5);
        this->declare_parameter<double>("process_noise_pos", 0.01);
        this->declare_parameter<double>("process_noise_vel", 0.1);
        this->declare_parameter<double>("process_noise_force", 1.0);
        this->declare_parameter<double>("measurement_noise", 0.001);
        this->declare_parameter<std::string>("world_frame", "optitrack");
        this->declare_parameter<std::string>("drone_frame", "imu_link");

        mass_ = this->get_parameter("drone_mass").as_double();
        hover_thrust_normalized_ = this->get_parameter("hover_thrust_normalized").as_double();
        double q_pos = this->get_parameter("process_noise_pos").as_double();
        double q_vel = this->get_parameter("process_noise_vel").as_double();
        double q_force = this->get_parameter("process_noise_force").as_double();
        double r_meas = this->get_parameter("measurement_noise").as_double();
        world_frame_ = this->get_parameter("world_frame").as_string();
        drone_frame_ = this->get_parameter("drone_frame").as_string();

        // Derived constants
        constexpr double g = 9.81;
        thrust_scale_ = mass_ * g / hover_thrust_normalized_;

        // Kalman filter matrices
        const double dt = 0.02;  // 50 Hz

        F_ = Eigen::Matrix<double, 6, 6>::Identity();
        F_(0, 1) = dt;
        F_(2, 3) = dt;
        F_(1, 4) = dt / mass_;
        F_(3, 5) = dt / mass_;

        H_ = Eigen::Matrix<double, 2, 6>::Zero();
        H_(0, 0) = 1.0;
        H_(1, 2) = 1.0;

        Q_ = Eigen::Matrix<double, 6, 6>::Zero();
        Q_(0, 0) = q_pos;
        Q_(1, 1) = q_vel;
        Q_(2, 2) = q_pos;
        Q_(3, 3) = q_vel;
        Q_(4, 4) = q_force;
        Q_(5, 5) = q_force;

        R_ = Eigen::Matrix<double, 2, 2>::Zero();
        R_(0, 0) = r_meas;
        R_(1, 1) = r_meas;

        x_ = Eigen::Matrix<double, 6, 1>::Zero();
        P_ = Eigen::Matrix<double, 6, 6>::Identity();
        P_(0, 0) = 1.0;
        P_(1, 1) = 1.0;
        P_(2, 2) = 1.0;
        P_(3, 3) = 1.0;
        P_(4, 4) = 10.0;
        P_(5, 5) = 10.0;

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // QoS for PX4 subscribers (best effort, transient local)
        rclcpp::QoS qos_sub(10);
        qos_sub.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_sub.durability(rclcpp::DurabilityPolicy::TransientLocal);
        qos_sub.history(rclcpp::HistoryPolicy::KeepLast);

        // Subscribers
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_sub,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                latest_odom_ = *msg;
                have_odom_ = true;
            });

        thrust_sub_ = this->create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/in/vehicle_thrust_setpoint", qos_sub,
            [this](const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg) {
                latest_thrust_ = *msg;
                have_thrust_ = true;
            });

        // Publishers - use transient_local so late-joining RViz gets the markers
        rclcpp::QoS qos_tl(10);
        qos_tl.transient_local();
        // Force estimate: use best_effort to match typical sensor data pattern
        rclcpp::QoS qos_force(10);
        qos_force.best_effort();
        qos_force.durability_volatile();
        force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/force_estimate", qos_force);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/force_marker", qos_tl);

        // Main loop at 50 Hz
        timer_ = this->create_wall_timer(20ms, std::bind(&ForceEstimatorNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "Force estimator started (mass=%.2f kg, thrust_scale=%.2f)",
                    mass_, thrust_scale_);
    }

private:
    void tick()
    {
        // 1. Look up OptiTrack TF for position
        double px_ned, py_ned;
        if (!lookup_position(px_ned, py_ned)) {
            return;
        }

        // Initialize state on first valid reading
        if (!initialized_) {
            x_(0) = px_ned;
            x_(2) = py_ned;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "KF initialized at (%.3f, %.3f) NED", px_ned, py_ned);
            return;
        }

        // 2. Get roll/pitch from VehicleOdometry quaternion
        double roll = 0.0, pitch = 0.0;
        if (have_odom_) {
            // q is [w, x, y, z] Hamiltonian, body-to-world passive rotation
            double qw = latest_odom_.q[0];
            double qx = latest_odom_.q[1];
            double qy = latest_odom_.q[2];
            double qz = latest_odom_.q[3];

            // Roll (phi) - rotation about X
            double sinr_cosp = 2.0 * (qw * qx + qy * qz);
            double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
            roll = std::atan2(sinr_cosp, cosr_cosp);

            // Pitch (theta) - rotation about Y
            double sinp = 2.0 * (qw * qy - qz * qx);
            if (std::abs(sinp) >= 1.0) {
                pitch = std::copysign(M_PI / 2.0, sinp);
            } else {
                pitch = std::asin(sinp);
            }
        }

        // 3. Compute thrust in Newtons
        // Since PX4 doesn't publish thrust setpoint, assume hover thrust (T ≈ m*g)
        // This is a reasonable approximation during hover when drone maintains altitude
        double T = mass_ * 9.81;  // Hover thrust = weight
        if (have_thrust_) {
            // If we do have thrust data, use it
            T = std::abs(latest_thrust_.xyz[2]) * thrust_scale_;
        }

        // 4. Predict step
        const double dt = 0.02;
        Eigen::Matrix<double, 6, 1> Bu = Eigen::Matrix<double, 6, 1>::Zero();
        Bu(1) = T * std::sin(pitch) / mass_ * dt;
        Bu(3) = -T * std::sin(roll) * std::cos(pitch) / mass_ * dt;

        Eigen::Matrix<double, 6, 1> x_pred = F_ * x_ + Bu;
        Eigen::Matrix<double, 6, 6> P_pred = F_ * P_ * F_.transpose() + Q_;

        // 5. Update step
        Eigen::Matrix<double, 2, 1> z;
        z(0) = px_ned;
        z(1) = py_ned;

        Eigen::Matrix<double, 2, 1> y = z - H_ * x_pred;
        Eigen::Matrix<double, 2, 2> S = H_ * P_pred * H_.transpose() + R_;
        Eigen::Matrix<double, 6, 2> K = P_pred * H_.transpose() * S.inverse();

        x_ = x_pred + K * y;
        P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H_) * P_pred;

        // 6. Publish force estimate
        geometry_msgs::msg::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = this->now();
        wrench_msg.header.frame_id = "NED";
        wrench_msg.wrench.force.x = x_(4);
        wrench_msg.wrench.force.y = x_(5);
        wrench_msg.wrench.force.z = 0.0;
        force_pub_->publish(wrench_msg);

        // 6b. Publish arrow marker in optitrack frame
        try {
            auto tf = tf_buffer_->lookupTransform(
                world_frame_, drone_frame_, tf2::TimePointZero);

            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = world_frame_;
            marker.ns = "force_estimate";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::ARROW;

            visualization_msgs::msg::Marker text;
            text.header.stamp = this->now();
            text.header.frame_id = world_frame_;
            text.ns = "force_estimate";
            text.id = 1;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.scale.z = 1.0;
            text.color.r = 1.0;
            text.color.g = 0.0;
            text.color.b = 0.0;
            text.color.a = 1.0;
            text.text = "Steady";

            // Convert force from NED to OptiTrack frame (negate Y)
            double fx_ot = -x_(4);
            double fy_ot = x_(5);
            double force_mag = std::sqrt(fx_ot * fx_ot + fy_ot * fy_ot);

            // Hide arrow when force is negligible
            constexpr double min_force = 0.3;  // N
            if (force_mag < min_force) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_pub_->publish(marker);
                text.action = visualization_msgs::msg::Marker::ADD;
                text.pose.position.x = tf.transform.translation.x;
                text.pose.position.y = tf.transform.translation.y;
                text.pose.position.z = tf.transform.translation.z + 0.3;
                marker_pub_->publish(text);
            } else {
                // Delete text marker
                text.action = visualization_msgs::msg::Marker::DELETE;
                marker_pub_->publish(text);

                // Build and publish arrow marker
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Start point: drone position
                geometry_msgs::msg::Point start;
                start.x = tf.transform.translation.x;
                start.y = tf.transform.translation.y;
                start.z = tf.transform.translation.z;

                // Scale: 0.1 m per 1 N
                constexpr double arrow_scale = 0.1;
                double arrow_len = force_mag * arrow_scale;

                // End point: drone position + scaled force vector
                geometry_msgs::msg::Point end;
                end.x = start.x + fx_ot * arrow_scale;
                end.y = start.y + fy_ot * arrow_scale;
                end.z = start.z;

                marker.points.push_back(start);
                marker.points.push_back(end);

                // Arrow thickness (head must be shorter than total arrow length)
                double head_len = std::min(0.05, arrow_len * 0.3);
                marker.scale.x = 0.03;   // shaft diameter
                marker.scale.y = 0.06;   // head diameter
                marker.scale.z = head_len;

                // Green with some transparency
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;

                marker_pub_->publish(marker);
            }
        } catch (const tf2::TransformException &) {
            // Silently skip marker if TF not available
        }

        // 7. Log at 10 Hz
        // int freq = 10;
        // double mag = std::sqrt(x_(4) * x_(4) + x_(5) * x_(5));
        // double dir = std::atan2(x_(5), x_(4));
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000 / freq,
        //     "F_ext: mag=%.3f N, dir=%.1f deg, (Fx=%.3f, Fy=%.3f) | T=%.2f N, roll=%.1f, pitch=%.1f",
        //     mag, dir * 180.0 / M_PI, x_(4), x_(5), T,
        //     roll * 180.0 / M_PI, pitch * 180.0 / M_PI);
    }

    bool lookup_position(double &px_ned, double &py_ned)
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                world_frame_, drone_frame_, tf2::TimePointZero);

            // Convert OptiTrack to NED: negate Y (same as offboard_test_node)
            px_ned = transform.transform.translation.x;
            py_ned = -transform.transform.translation.y;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for TF: %s -> %s", world_frame_.c_str(), drone_frame_.c_str());
            return false;
        }
    }

    // Parameters
    double mass_;
    double hover_thrust_normalized_;
    double thrust_scale_;
    std::string world_frame_;
    std::string drone_frame_;

    // Kalman filter
    Eigen::Matrix<double, 6, 6> F_;
    Eigen::Matrix<double, 2, 6> H_;
    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix<double, 2, 2> R_;
    Eigen::Matrix<double, 6, 1> x_;
    Eigen::Matrix<double, 6, 6> P_;
    bool initialized_ = false;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscriptions
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_sub_;
    px4_msgs::msg::VehicleOdometry latest_odom_;
    px4_msgs::msg::VehicleThrustSetpoint latest_thrust_;
    bool have_odom_ = false;
    bool have_thrust_ = false;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
