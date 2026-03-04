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
    std::vector<std::pair<double,double>> waypoints_ = {
        {0.914, 1.829}, // waypoint before wall
        {0.914, 2.438}, // waypoint after wall
        {0.914, 3.353} // target port
    };


private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        // Convert quaternion to yaw
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        theta_ = std::atan2(siny_cosp, cosy_cosp);

    }

    void timer_callback() {
        if (current_waypoint_ >= waypoints_.size()) {
            return; // finished all waypoints
        }

        double target_x = waypoints_[current_waypoint_].first;
        double target_y = waypoints_[current_waypoint_].second;

        geometry_msgs::msg::Twist cmd;
        double dx = target_x_ - x_;
        double dy = target_y_ - y_;
        double distance = sqrt(dx * dx + dy * dy);

        double target_angle = std::atan2(dy, dx);
        double angle_error = target_angle - theta_;

        // Normalize angle
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        

        if (distance > 0.1) {
            cmd.linear.x = 0.1;
            cmd.angular.z = 1.5 * angle_error;
        }
        else {
            current_waypoint_++;
        }
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cmd.angular.z = 1.5 * angle_error;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Test4Node>());
    rclcpp::shutdown();
    return 0;
}
