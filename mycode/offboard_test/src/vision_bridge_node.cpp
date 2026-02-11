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

        VehicleOdometry msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.timestamp_sample = msg.timestamp;

        msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;

        // Position: OptiTrack (X-back, Y-right, Z-up) -> NED (X-north, Y-east, Z-down)
        msg.position[0] = -tf.transform.translation.x;  // -backward = forward/north
        msg.position[1] = tf.transform.translation.y;   // right = east
        msg.position[2] = -tf.transform.translation.z;  // -up = down

        // Quaternion: OptiTrack world BRU + body LUF -> PX4 world NED + body FRD
        // R_world = diag(-1, 1, -1) maps BRU->NED
        // R_body = [[0,0,1],[−1,0,0],[0,−1,0]] maps LUF->FRD
        // q_ned = R_world * q_ot * R_body^T
        double w = tf.transform.rotation.w;
        double x = tf.transform.rotation.x;
        double y = tf.transform.rotation.y;
        double z = tf.transform.rotation.z;
        msg.q[0] =  0.5 * ( w + x - y - z);
        msg.q[1] =  0.5 * ( w - x - y + z);
        msg.q[2] =  0.5 * ( w - x + y - z);
        msg.q[3] = -0.5 * ( w + x + y + z);

        // No velocity from mocap
        msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        msg.velocity[0] = std::nanf("");
        msg.velocity[1] = std::nanf("");
        msg.velocity[2] = std::nanf("");
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
        msg.velocity_variance[0] = std::nanf("");
        msg.velocity_variance[1] = std::nanf("");
        msg.velocity_variance[2] = std::nanf("");

        msg.reset_counter = 0;
        msg.quality = 100;

        odom_pub_->publish(msg);

        // Debug: publish NED data converted back to OptiTrack frame as a TF.
        // If conversion is correct, "imu_link_ned_check" overlaps "imu_link" in RViz.
        // Inverse position: x_ot = -x_ned, y_ot = y_ned, z_ot = -z_ned
        // Inverse quaternion: q_ot = R_world^T * q_ned * R_body  (same formula since R^T=R for these)
        // geometry_msgs::msg::TransformStamped debug_tf;
        // debug_tf.header.stamp = tf.header.stamp;
        // debug_tf.header.frame_id = world_frame_;
        // debug_tf.child_frame_id = "imu_link_ned_check";
        // debug_tf.transform.translation.x = -msg.position[0];
        // debug_tf.transform.translation.y = msg.position[1];
        // debug_tf.transform.translation.z = -msg.position[2];
        // double nw = msg.q[0], nx = msg.q[1], ny = msg.q[2], nz = msg.q[3];
        // debug_tf.transform.rotation.w =  0.5 * ( nw + nx - ny - nz);
        // debug_tf.transform.rotation.x =  0.5 * ( nw - nx - ny + nz);
        // debug_tf.transform.rotation.y =  0.5 * ( nw - nx + ny - nz);
        // debug_tf.transform.rotation.z = -0.5 * ( nw + nx + ny + nz);
        // tf_broadcaster_->sendTransform(debug_tf);
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string world_frame_;
    std::string drone_frame_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
