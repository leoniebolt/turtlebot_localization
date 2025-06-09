// kf_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <Eigen/Dense>
#include <cmath>

class KFNode : public rclcpp::Node {
public:
    KFNode() : Node("kf_node"), initialized_(false) {
        mu_.setZero();
        bel_mu_.setZero();
        sigma_ = Eigen::Matrix3d::Identity() * 0.1;
        bel_sigma_ = Eigen::Matrix3d::Identity();

        A_ = Eigen::Matrix3d::Identity();
        B_ = Eigen::Matrix3d::Identity();
        C_ = Eigen::Matrix3d::Identity();

        R_ = Eigen::Matrix3d::Zero();
        R_.diagonal() << 0.05, 0.05, 0.1;

        Q_ = Eigen::Matrix3d::Zero();
        Q_.diagonal() << 0.1, 0.1, 0.2;

        u_.setZero();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&KFNode::odom_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&KFNode::imu_callback, this, std::placeholders::_1));

        kf_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/kf_marker", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Kalman Filter Node gestartet.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (!initialized_) {
            mu_(0, 0) = msg->pose.pose.position.x;
            mu_(1, 0) = msg->pose.pose.position.y;

            auto q = msg->pose.pose.orientation;
            double yaw = std::atan2(2.0 * (q.z * q.w + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            mu_(2, 0) = yaw;

            initialized_ = true;
            return;
        }

        kalman_predict(dt);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double omega = msg->angular_velocity.z;
        u_(2, 0) = omega;

        double dt = 0.01;

        Eigen::Vector3d z = bel_mu_;
        z(2, 0) += omega * dt;

        kalman_update(z);
        publish_kf_pose();
        publish_marker();
    }

    void kalman_predict(double dt) {
        double theta = mu_(2, 0);
        double vx = u_(0, 0);
        double vy = u_(1, 0);
        double omega = u_(2, 0);

        double dx = dt * (vx * std::cos(theta) - vy * std::sin(theta));
        double dy = dt * (vx * std::sin(theta) + vy * std::cos(theta));
        double dtheta = dt * omega;

        bel_mu_(0, 0) = mu_(0, 0) + dx;
        bel_mu_(1, 0) = mu_(1, 0) + dy;
        bel_mu_(2, 0) = mu_(2, 0) + dtheta;

        bel_sigma_ = sigma_ + R_;
    }

    void kalman_update(const Eigen::Vector3d& z) {
        Eigen::Matrix3d S = C_ * bel_sigma_ * C_.transpose() + Q_;
        Eigen::Matrix3d K = bel_sigma_ * C_.transpose() * S.inverse();
        Eigen::Vector3d y = z - C_ * bel_mu_;
        mu_ = bel_mu_ + K * y;
        sigma_ = (Eigen::Matrix3d::Identity() - K * C_) * bel_sigma_;
    }

    void publish_kf_pose() {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x = mu_(0, 0);
        msg.pose.pose.position.y = mu_(1, 0);
        msg.pose.pose.orientation.z = std::sin(mu_(2, 0) / 2.0);
        msg.pose.pose.orientation.w = std::cos(mu_(2, 0) / 2.0);

        for (int i = 0; i < 36; i++) msg.pose.covariance[i] = 0.0;
        msg.pose.covariance[0] = sigma_(0, 0);
        msg.pose.covariance[7] = sigma_(1, 1);
        msg.pose.covariance[35] = sigma_(2, 2);

        kf_pub_->publish(msg);
    }

    void publish_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "kf";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = mu_(0, 0);
        marker.pose.position.y = mu_(1, 0);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.z = std::sin(mu_(2, 0) / 2.0);
        marker.pose.orientation.w = std::cos(mu_(2, 0) / 2.0);

        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr kf_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    Eigen::Vector3d mu_, bel_mu_, u_;
    Eigen::Matrix3d sigma_, bel_sigma_, A_, B_, C_, R_, Q_;

    rclcpp::Time last_time_;
    bool initialized_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KFNode>());
    rclcpp::shutdown();
    return 0;
}
