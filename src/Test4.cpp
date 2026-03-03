#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class Test4Node : public rclcpp::Node {
public:
    Test4Node() : Node("test4_node") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Test4Node::odom_callback, this, std::placeholders::_1)
        );
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&Test4Node::timer_callback, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
    }

    void timer_callback() {
        geometry_msgs::msg::Twist cmd;
        double dx = target_x_ - x_;
        double dy = target_y_ - y_;
        double distance = sqrt(dx * dx + dy * dy);

        if (distance > 0.05) {
            cmd.linear.x = 0.1;
        }
        else {
            cmd.linear.x = 0;
        }
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0, y_ = 0;
    double target_x_ = 2.0, target_y_ = 0;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Test4Node>());
    rclcpp::shutdown();
    return 0;
}
