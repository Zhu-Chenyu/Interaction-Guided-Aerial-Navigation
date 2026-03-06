/**
 * @file vision_bridge_node.cpp
 * @brief Bridges OptiTrack TF data to PX4 VehicleOdometry via DDS
 *
 * Reads the OptiTrack transform (optitrack -> imu_link) and publishes
 * VehicleOdometry to /fmu/in/vehicle_visual_odometry at 50 Hz.
 * This node does NOT arm, take off, or send any control commands.
 */

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;

class VisionBridgeNode : public rclcpp::Node
{
public:
    VisionBridgeNode() : Node("vision_bridge")
    {
        this->declare_parameter<std::string>("world_frame", "optitrack");
        this->declare_parameter<std::string>("drone_frame", "imu_link");

        world_frame_ = this->get_parameter("world_frame").as_string();
        drone_frame_ = this->get_parameter("drone_frame").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        rclcpp::QoS qos_px4(10);
        qos_px4.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_px4.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_px4.history(rclcpp::HistoryPolicy::KeepLast);

        odom_pub_ = this->create_publisher<VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", qos_px4);

        timer_ = this->create_wall_timer(20ms, std::bind(&VisionBridgeNode::publish, this));

        RCLCPP_INFO(this->get_logger(), "Vision bridge started: %s -> %s -> PX4",
                    world_frame_.c_str(), drone_frame_.c_str());
    }

private:
    void publish()
    {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(world_frame_, drone_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for TF: %s -> %s", world_frame_.c_str(), drone_frame_.c_str());
            return;
        }

        auto now = this->get_clock()->now();

        VehicleOdometry msg{};
        msg.timestamp = now.nanoseconds() / 1000;
        msg.timestamp_sample = msg.timestamp;

        msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;

        // Position: OptiTrack world FLU (X-forward, Y-left, Z-up) -> NED
        double pos_x =  tf.transform.translation.x;  // forward = north
        double pos_y = -tf.transform.translation.y;  // -left   = east
        double pos_z = -tf.transform.translation.z;  // -up     = down
        msg.position[0] = pos_x;
        msg.position[1] = pos_y;
        msg.position[2] = pos_z;

        // Quaternion: OptiTrack world FLU + body FLU -> PX4 world NED + body FRD
        // R_world = diag(1,-1,-1) maps FLU->NED
        // R_body  = diag(1,-1,-1) maps FLU->FRD
        // Combined effect on quaternion: q_ned = (w, x, -y, -z)
        double w = tf.transform.rotation.w;
        double x = tf.transform.rotation.x;
        double y = tf.transform.rotation.y;
        double z = tf.transform.rotation.z;
        msg.q[0] =  w;
        msg.q[1] =  x;
        msg.q[2] = -y;
        msg.q[3] = -z;

        // Velocity: finite-difference of NED position at 50 Hz.
        // Gives PX4's EKF a direct velocity measurement so it doesn't have to
        // differentiate position internally (which adds a cycle of lag).
        if (have_prev_) {
            double dt = (now - prev_stamp_).seconds();
            if (dt > 0.005 && dt < 0.1) {  // sanity: reject stale or duplicate TFs
                msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
                msg.velocity[0] = (pos_x - prev_x_) / dt;
                msg.velocity[1] = (pos_y - prev_y_) / dt;
                msg.velocity[2] = (pos_z - prev_z_) / dt;
                msg.velocity_variance[0] = 0.01f;  // ~0.1 m/s std-dev
                msg.velocity_variance[1] = 0.01f;
                msg.velocity_variance[2] = 0.01f;
            } else {
                msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
                msg.velocity[0] = std::nanf("");
                msg.velocity[1] = std::nanf("");
                msg.velocity[2] = std::nanf("");
                msg.velocity_variance[0] = std::nanf("");
                msg.velocity_variance[1] = std::nanf("");
                msg.velocity_variance[2] = std::nanf("");
            }
        } else {
            msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
            msg.velocity[0] = std::nanf("");
            msg.velocity[1] = std::nanf("");
            msg.velocity[2] = std::nanf("");
            msg.velocity_variance[0] = std::nanf("");
            msg.velocity_variance[1] = std::nanf("");
            msg.velocity_variance[2] = std::nanf("");
        }

        prev_x_ = pos_x;  prev_y_ = pos_y;  prev_z_ = pos_z;
        prev_stamp_ = now;
        have_prev_ = true;

        msg.angular_velocity[0] = std::nanf("");
        msg.angular_velocity[1] = std::nanf("");
        msg.angular_velocity[2] = std::nanf("");

        // OptiTrack variance (sub-mm accuracy)
        msg.position_variance[0] = 0.001f;
        msg.position_variance[1] = 0.001f;
        msg.position_variance[2] = 0.001f;
        msg.orientation_variance[0] = 0.001f;
        msg.orientation_variance[1] = 0.001f;
        msg.orientation_variance[2] = 0.001f;

        msg.reset_counter = 0;
        msg.quality = 100;

        odom_pub_->publish(msg);
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string world_frame_;
    std::string drone_frame_;

    // Previous position for finite-difference velocity
    double prev_x_ = 0.0;
    double prev_y_ = 0.0;
    double prev_z_ = 0.0;
    rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
    bool have_prev_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
