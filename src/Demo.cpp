#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class DemoNode : public rclcpp::Node {
public:
    DemoNode() : Node("demo_node") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DemoNode::odom_callback, this, _1)
        );
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&DemoNode::timer_callback, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        q_x_ = msg->pose.pose.orientation.x;
        q_y_ = msg->pose.pose.orientation.y;
        q_z_ = msg->pose.pose.orientation.z;
        q_w_ = msg->pose.pose.orientation.w;
    }

    void timer_callback() {
        geometry_msgs::msg::Twist cmd;
        tf2::Quaternion q(q_x_, q_y_, q_z_, q_w_);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Distance and heading calculations
        double dx = target_x_ - x_;
        double dy = target_y_ - y_;
        double distance = sqrt(dx * dx + dy * dy);
        double target_angle = atan2(dy, dx);
        double angle_error = wrap_angle(target_angle - yaw);

        // Simple proportional controller
        if (distance > 0.1) {
            cmd.linear.x = 0.2;
            cmd.angular.z = angle_error;
        }
        else {
            // Move to next waypoint
            if (count_ < waypoints_.size()) {
                target_x_ = waypoints_[count_].first;
                target_y_ = waypoints_[count_].second;
                count_++;
            }
            else {
                cmd.linear.x = 0;
                cmd.angular.z = 0;
            }
        }
        cmd_pub_->publish(cmd);
    }

    double wrap_angle(double a) {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0, y_ = 0, q_x_ = 0, q_y_ = 0, q_z_ = 0, q_w_ = 0;
    double target_x_ = 0, target_y_ = 0;
    size_t count_ = 0;
    std::vector<std::pair<double, double>> waypoints_ = { {1.0,0},{1.0,1.0},{0,1.0},{0,0} };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoNode>());
    rclcpp::shutdown();
    return 0;
}
