
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

struct Waypoint
{
    double x;
    double y;
    double theta;  // radians
};

class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("demo2_node")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&DemoNode::odom_callback, this, std::placeholders::_1));
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&DemoNode::lidar_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            100ms, std::bind(&DemoNode::control_loop, this));

        // Waypoints: forward → around wall → port
        waypoints_ = {
            {0.0, 1.83, 1.5708},       // Pre-wall (~6ft), facing +Y
            {-0.30, 2.13, 3.1416},     // Around wall (1ft left)
            {-0.30, 2.74, 1.5708},     // Past wall, face forward
            {0.0, 3.96, 1.5708}        // Port (12ft)
        };
        current_wp_ = 0;

        front_distance_ = 10.0; // initial LIDAR reading
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_wp_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    float front_distance_; // meters

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        current_yaw_ = yaw;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int center_index = msg->ranges.size() / 2;
        int range_check = 5;
        float sum = 0.0;
        int count = 0;

        for (int i = center_index - range_check; i <= center_index + range_check; i++)
        {
            if (i >= 0 && i < (int)msg->ranges.size())
            {
                if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
                {
                    sum += msg->ranges[i];
                    count++;
                }
            }
        }

        front_distance_ = (count > 0) ? sum / count : msg->range_max;
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void control_loop()
    {
        if (current_wp_ >= waypoints_.size())
        {
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            return;
        }

        auto target = waypoints_[current_wp_];

        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        geometry_msgs::msg::Twist cmd;

        double position_tolerance = 0.10; // meters
        double angle_tolerance = 0.05;    // radians

        // --- Obstacle avoidance ---
        if (front_distance_ < 0.3) // obstacle too close
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5; // turn left
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Turning...");
        }
        else if (distance > position_tolerance)
        {
            // --- Move toward waypoint first ---
            if (distance > 0.05) // move forward small distance before adjusting theta
            {
                double target_angle = std::atan2(dy, dx);
                double angle_error = normalize_angle(target_angle - current_yaw_);

                if (std::fabs(angle_error) > 0.1)
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.5 * angle_error;
                }
                else
                {
                    cmd.linear.x = 0.15;
                    cmd.angular.z = 0.2 * angle_error;
                }
            }
        }
        else
        {
            // --- Final orientation at waypoint ---
            double final_angle_error = normalize_angle(target.theta - current_yaw_);
            if (std::fabs(final_angle_error) > angle_tolerance)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.5 * final_angle_error;
            }
            else
            {
                current_wp_++;
                RCLCPP_INFO(this->get_logger(), "Waypoint reached (%zu/%zu).", current_wp_, waypoints_.size());
            }
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoNode>());
    rclcpp::shutdown();
    return 0;
}
