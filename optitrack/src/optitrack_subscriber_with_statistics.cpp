// this is modified code from ROS2 Tutorial: Enabling topic statistics (C++)
// https://docs.ros.org/en/jazzy/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

//this node will receive optitrack data from the topic "optitrack_data" and calculate the age of the data — the difference in time between the moment before the message is published to when the subscriber node receives it
//this node publishes the message statistics on a different topic than the topic it's subscribed to
class MinimalSubscriberWithTopicStatistics : public rclcpp::Node
{
public:
  MinimalSubscriberWithTopicStatistics()
  : Node("OptiTrack_Subscriber_Node_With_Statistics") 
  { // configure node
    // manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    // configure the message collection window and publish statistics period (default 1s)
    options.topic_stats_options.publish_period = std::chrono::seconds(1);

    // configure the topic name (default '/statistics')
    // options.topic_stats_options.publish_topic = "/whatever_you_want"

    // prints out the received message
    auto callback = [this](const geometry_msgs::msg::PoseStamped & msg) {
        this->optitrack_topic_callback(msg);
    };

    auto optitrack_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();
    // initialize the subscription
    optitrack_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "optitrack_data", optitrack_qos, callback, options);
  }

private:
  void optitrack_topic_callback(const geometry_msgs::msg::PoseStamped & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '(x, y, z): (%3.2f, %3.2f, %3.2f), (qx, qy, qz, qw): (%3.2f, %3.2f, %3.2f, %3.2f)'", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,  msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr optitrack_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithTopicStatistics>());
  rclcpp::shutdown();
  return 0;
}
