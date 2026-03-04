#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DemoNode::scan_callback, this, _1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&DemoNode::timer_callback, this)
        );

        // Initialize first waypoint
        target_x_ = waypoints_[0].first;
        target_y_ = waypoints_[0].second;
        count_ = 1;
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

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        int center = msg->ranges.size() / 2;

        front_distance_ = msg->ranges[center];

        if (std::isnan(front_distance_) || std::isinf(front_distance_)) {
            front_distance_ = 10.0;
        }
    }

    void timer_callback() {

        geometry_msgs::msg::Twist cmd;

        tf2::Quaternion q(q_x_, q_y_, q_z_, q_w_);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 🚨 OBSTACLE AVOIDANCE FIRST
        if (front_distance_ < 0.5) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.6;   // turn left
            cmd_pub_->publish(cmd);
            return;
        }

        // Waypoint navigation
        double dx = target_x_ - x_;
        double dy = target_y_ - y_;
        double distance = sqrt(dx * dx + dy * dy);

        double target_angle = atan2(dy, dx);
        double angle_error = wrap_angle(target_angle - yaw);

        if (distance > 0.1) {
            cmd.linear.x = 0.2;
            cmd.angular.z = angle_error;
        }
        else {
            if (count_ < waypoints_.size()) {
                target_x_ = waypoints_[count_].first;
                target_y_ = waypoints_[count_].second;
                count_++;
            }
            else {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
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
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0, y_ = 0;
    double q_x_ = 0, q_y_ = 0, q_z_ = 0, q_w_ = 0;

    double target_x_ = 0, target_y_ = 0;
    double front_distance_ = 10.0;

    size_t count_ = 0;

    std::vector<std::pair<double, double>> waypoints_ = {
        {0.914, 1.829},
        {0.914, 2.438},
        {0.914, 3.353}
    };
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoNode>());
    rclcpp::shutdown();
    return 0;
}
