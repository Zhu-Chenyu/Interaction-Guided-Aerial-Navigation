/**
 * @file arena_markers_node.cpp
 * @brief Publishes RViz markers for the arena corner columns
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class ArenaMarkersNode : public rclcpp::Node
{
public:
    ArenaMarkersNode() : Node("arena_markers_node")
    {
        // Parameters for arena dimensions (in meters)
        this->declare_parameter<double>("column_width", 0.1);   // 100mm column width
        this->declare_parameter<double>("column_height", 2.92); // 2920mm height
        this->declare_parameter<std::string>("frame_id", "optitrack");

        // Arena corner positions from origin (in meters)
        // OptiTrack frame: X points FRONT, Y points LEFT
        this->declare_parameter<double>("x_back", -1.69);    // -1690mm (behind origin)
        this->declare_parameter<double>("x_front", 1.69);    // +1690mm (in front of origin)
        this->declare_parameter<double>("y_left", 2.15);     // +2150mm (left of origin)
        this->declare_parameter<double>("y_right", -3.27);   // -3270mm (right of origin)

        column_width_ = this->get_parameter("column_width").as_double();
        column_height_ = this->get_parameter("column_height").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        x_back_ = this->get_parameter("x_back").as_double();
        x_front_ = this->get_parameter("x_front").as_double();
        y_left_ = this->get_parameter("y_left").as_double();
        y_right_ = this->get_parameter("y_right").as_double();

        // Publisher with transient_local durability so late-joining subscribers get the markers
        rclcpp::QoS qos(10);
        qos.transient_local();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/arena_markers", qos);

        // Publish immediately on startup
        publish_markers();

        // Publish at 10 Hz for reliability (markers are static but frequent publish helps discovery)
        timer_ = this->create_wall_timer(100ms, std::bind(&ArenaMarkersNode::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "Arena markers node started");
        RCLCPP_INFO(this->get_logger(), "Arena: X=[%.2f, %.2f] Y=[%.2f, %.2f]",
                    x_back_, x_front_, y_right_, y_left_);
    }

private:
    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Corner positions (X, Y) - X=front/back, Y=left/right
        std::vector<std::pair<double, double>> corners = {
            {x_front_, y_left_},   // Front-left
            {x_front_, y_right_},  // Front-right
            {x_back_, y_left_},    // Back-left
            {x_back_, y_right_}    // Back-right
        };

        for (size_t i = 0; i < corners.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = frame_id_;
            marker.ns = "arena_columns";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Position: center of column at half height
            marker.pose.position.x = corners[i].first;
            marker.pose.position.y = corners[i].second;
            marker.pose.position.z = column_height_ / 2.0;  // Center vertically

            // No rotation
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;

            // Scale: column dimensions
            marker.scale.x = column_width_;
            marker.scale.y = column_width_;
            marker.scale.z = column_height_;

            // Purple color
            marker.color.r = 0.5;
            marker.color.g = 0.0;
            marker.color.b = 0.8;
            marker.color.a = 0.9;

            marker.lifetime = rclcpp::Duration(0, 0);  // Persistent

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    // Parameters
    double column_width_;
    double column_height_;
    std::string frame_id_;
    double x_back_, x_front_, y_left_, y_right_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArenaMarkersNode>());
    rclcpp::shutdown();
    return 0;
}
