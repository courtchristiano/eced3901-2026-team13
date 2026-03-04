#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Test5Node : public rclcpp::Node {
public:
    Test5Node() : Node("test5_node") {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Test5Node::laser_callback, this, std::placeholders::_1)
        );
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&Test5Node::timer_callback, this));
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        min_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }

    void timer_callback() {
        geometry_msgs::msg::Twist cmd;
        if (min_distance_ < 0.5) {
            cmd.linear.x = 0;
            cmd.angular.z = 0.5;  // Turn away from obstacle
        }
        else {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0;
        }
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double min_distance_ = 10.0;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Test5Node>());
    rclcpp::shutdown();
    return 0;
}
