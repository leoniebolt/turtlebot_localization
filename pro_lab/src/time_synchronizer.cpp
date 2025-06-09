#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
using std::placeholders::_1;
using std::placeholders::_2;
class FilterNode : public rclcpp::Node
{
public:
   FilterNode(): Node("filter_node")
   {
       odom_sub_.subscribe(this, "/odom");
       imu_sub_.subscribe(this, "/imu");
        sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>(odom_sub_, imu_sub_, 10);
        sync_->registerCallback(std::bind(&FilterNode::syncedCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Waiting for messages...");
   }
private:
   void syncedCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                       const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
   {
       RCLCPP_INFO(this->get_logger(), "Received synchronized messages:");
       RCLCPP_INFO(this->get_logger(), "  Odometry timestamp: %f", odom_msg->header.stamp.sec + 1e-9 * odom_msg->header.stamp.nanosec);
       RCLCPP_INFO(this->get_logger(), "  IMU timestamp     : %f", imu_msg->header.stamp.sec + 1e-9 * imu_msg->header.stamp.nanosec);
        /* Pipeline implementation */
   }
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
   message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
   std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>> sync_;
};
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<FilterNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}