#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode(double wheel_radius, double wheel_base) : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0)
    {

        wheel_radius_ = wheel_radius;
        wheel_base_ = wheel_base;

        // 发布里程计
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        last_time_ = this->now();
    }

    void updateOdometry(double left_wheel_speed, double right_wheel_speed)
    {
        left_wheel_speed_ = left_wheel_speed;
        right_wheel_speed_ = right_wheel_speed;
        // 时间计算
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (abs(left_wheel_speed) > 10 || abs(right_wheel_speed) > 10)
        {
            std::cout << "left_wheel_speed: " << left_wheel_speed;
            std::cout << " right_wheel_speed: " << right_wheel_speed << std::endl;
            left_wheel_speed_ = 0;
            right_wheel_speed_ = 0;
            return;
        }

        // 差速运动模型
        double v = wheel_radius_ * (left_wheel_speed_ + right_wheel_speed_) / 2.0;
        double omega = wheel_radius_ * (right_wheel_speed_ - left_wheel_speed_) / wheel_base_;

        // 更新位置
        double delta_x = v * std::cos(theta_) * dt;
        double delta_y = v * std::sin(theta_) * dt;
        double delta_theta = omega * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // 发布 /odom
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        // 位置
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

        // 速度
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = omega;

        odom_pub_->publish(odom_msg);

        // 发布 TF
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_footprint";
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.z = std::sin(theta_ / 2.0);
        transform.transform.rotation.w = std::cos(theta_ / 2.0);

        tf_broadcaster_->sendTransform(transform);
    }

private:
    // ROS2 组件
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 参数
    double wheel_radius_;
    double wheel_base_;

    // 状态变量
    double x_, y_, theta_;
    double left_wheel_speed_ = 0.0;
    double right_wheel_speed_ = 0.0;
    rclcpp::Time last_time_ = this->now();
};
