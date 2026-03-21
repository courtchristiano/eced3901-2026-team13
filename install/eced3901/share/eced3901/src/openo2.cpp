

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SquareRoutine : public rclcpp::Node
{
public:
    SquareRoutine() : Node("Square_Routine")
    {
        // Subscribers and publisher
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&SquareRoutine::odom_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&SquareRoutine::lidar_callback, this, _1));

        timer_ = this->create_wall_timer(100ms, std::bind(&SquareRoutine::timer_callback, this));

        // State initialization
        count_ = 0;
        last_state_complete = true;
        current_action_ = Action::IDLE;

        lifeboat_detected = false;
        lifeboat_routine_active = false;
        lifeboat_detection_dist = 0.5; // meters
        max_lifeboat_points = 5;       // max points to consider as lifeboat
    }

private:
    // -------------------- ENUM --------------------
    enum class Action { IDLE, MOVE, TURN, LIFEBOAT_TURN1, LIFEBOAT_REV, LIFEBOAT_FORWARD, LIFEBOAT_TURN2 };

    // -------------------- CALLBACKS --------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_now = msg->pose.pose.position.x;
        y_now = msg->pose.pose.position.y;
        q_x = msg->pose.pose.orientation.x;
        q_y = msg->pose.pose.orientation.y;
        q_z = msg->pose.pose.orientation.z;
        q_w = msg->pose.pose.orientation.w;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int total = msg->ranges.size();
        int front_start = total / 4;
        int front_end = 3 * total / 4;

        std::vector<float> front_points;

        // Only consider front half of LIDAR for walls/lifeboat
        for (int i = front_start; i <= front_end; ++i)
        {
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
                front_points.push_back(msg->ranges[i]);
        }

        // Wall avoidance distances (median front, left, right)
        front_wall_dist = median(msg, total / 2);
        left_wall_dist  = median(msg, 3 * total / 4);
        right_wall_dist = median(msg, total / 4);

        // ----- SMART LIFBOAT DETECTION -----
        int close_points = 0;
        for (auto r : front_points)
            if (r < lifeboat_detection_dist)
                close_points++;

        // Trigger lifeboat routine only if small cluster
        if (close_points > 0 && close_points <= max_lifeboat_points && !lifeboat_routine_active)
        {
            lifeboat_detected = true;
            lifeboat_distance_to_back = *std::min_element(front_points.begin(), front_points.end());
            RCLCPP_INFO(this->get_logger(), "Lifeboat detected! Min distance: %.2f m", lifeboat_distance_to_back);
        }
    }

    // -------------------- TIMER --------------------
    void timer_callback()
    {
        geometry_msgs::msg::Twist cmd;

        // Update yaw
        tf2::Quaternion q(q_x, q_y, q_z, q_w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        th_now = yaw;

        d_now = std::hypot(x_now - x_init, y_now - y_init);

        // Trigger lifeboat routine if detected
        if (lifeboat_detected)
        {
            start_lifeboat_routine();
            lifeboat_detected = false;
        }

        // Handle lifeboat routine first
        if (lifeboat_routine_active)
        {
            handle_lifeboat_routine();
            return;
        }

        // ----- NORMAL MOTION -----
        if (current_action_ == Action::MOVE)
        {
            cmd.linear.x = x_vel;
            cmd.angular.z = 0.0;

            double error = d_aim - d_now;

            // Stop for front wall
            if (front_wall_dist < 0.32)
            {
                cmd.linear.x = 0.0;
                current_action_ = Action::IDLE;
                last_state_complete = true;
                sequence_statemachine();
            }
            // Wall avoidance
            else if (left_wall_dist < 0.32)
                cmd.angular.z = -0.2;
            else if (right_wall_dist < 0.32)
                cmd.angular.z = 0.2;

            if (error < 0.01)
            {
                cmd.linear.x = 0.0;
                current_action_ = Action::IDLE;
                last_state_complete = true;
                sequence_statemachine();
            }
        }
        else if (current_action_ == Action::TURN)
        {
            double angle_turned = wrap_angle(th_now - th_init);
            double remaining = th_aim - angle_turned;
            cmd.linear.x = 0.0;

            if (std::fabs(remaining) > 0.01)
                cmd.angular.z = (remaining > 0 ? th_vel : -th_vel);
            else
            {
                cmd.angular.z = 0.0;
                current_action_ = Action::IDLE;
                last_state_complete = true;
            }
        }

        cmd_pub_->publish(cmd);
        sequence_statemachine();
    }

    // -------------------- SEQUENCE --------------------
    void sequence_statemachine()
    {
        if (!last_state_complete) return;
        last_state_complete = false;

        switch (count_)
        {
            case 0: move_distance(3.048); break;
            case 1: correct_with_wall(0.57);
            default: break;
        }
    }

    void move_distance(double distance)
    {
        d_aim = distance;
        x_init = x_now;
        y_init = y_now;
        count_++;
        current_action_ = Action::MOVE;
    }

    void turn_angle(double angle)
    {
        th_aim = angle;
        th_init = th_now;
        count_++;
        current_action_ = Action::TURN;
    }

    void correct_with_wall(double target)
    {
        double error = front_wall_dist - target;
        if (std::fabs(error) > 0.02)
        {
            x_vel = (error > 0 ? 0.05 : -0.05);
            move_distance(std::fabs(error));
            x_vel = 0.1;
        }
        else
        {
            count_++;
            last_state_complete = true;
        }
    }

    double wrap_angle(double a)
    {
        a = fmod(a + M_PI, 2*M_PI);
        if (a < 0) a += 2*M_PI;
        return a - M_PI;
    }

    // -------------------- LIFBOAT ROUTINE --------------------
    void start_lifeboat_routine()
    {
        lifeboat_routine_active = true;
        current_action_ = Action::LIFEBOAT_TURN1;
        th_init = th_now;
        th_aim = M_PI;  // turn 180°
        lifeboat_reverse_dist = lifeboat_distance_to_back;
        RCLCPP_INFO(this->get_logger(), "Starting Lifeboat routine");
    }

    void handle_lifeboat_routine()
    {
        geometry_msgs::msg::Twist cmd;

        if (current_action_ == Action::LIFEBOAT_TURN1)
        {
            double angle_turned = wrap_angle(th_now - th_init);
            double remaining = th_aim - angle_turned;

            if (std::fabs(remaining) > 0.01)
                cmd.angular.z = (remaining > 0 ? th_vel : -th_vel);
            else
            {
                cmd.angular.z = 0.0;
                current_action_ = Action::LIFEBOAT_REV;
                x_init = x_now;
                y_init = y_now;
                d_aim = lifeboat_reverse_dist;
            }
        }
        else if (current_action_ == Action::LIFEBOAT_REV)
        {
            double error = d_aim - std::hypot(x_now - x_init, y_now - y_init);
            if (error > 0.01)
                cmd.linear.x = -x_vel;
            else
            {
                cmd.linear.x = 0.0;
                current_action_ = Action::LIFEBOAT_FORWARD;
                x_init = x_now;
                y_init = y_now;
                d_aim = 0.2; // small forward step
            }
        }
        else if (current_action_ == Action::LIFEBOAT_FORWARD)
        {
            double error = d_aim - std::hypot(x_now - x_init, y_now - y_init);
            if (error > 0.01)
                cmd.linear.x = x_vel;
            else
            {
                cmd.linear.x = 0.0;
                current_action_ = Action::LIFEBOAT_TURN2;
                th_init = th_now;
                th_aim = 0; // back to original heading
            }
        }
        else if (current_action_ == Action::LIFEBOAT_TURN2)
        {
            double angle_turned = wrap_angle(th_now - th_init);
            double remaining = th_aim - angle_turned;

            if (std::fabs(remaining) > 0.01)
                cmd.angular.z = (remaining > 0 ? th_vel : -th_vel);
            else
            {
                cmd.angular.z = 0.0;
                lifeboat_routine_active = false;
                current_action_ = Action::IDLE;
                correct_with_wall(0.32);
                last_state_complete = true;
                RCLCPP_INFO(this->get_logger(), "Lifeboat routine complete, resuming path");
            }
        }

        cmd_pub_->publish(cmd);
    }

    float median(const sensor_msgs::msg::LaserScan::SharedPtr msg, int index)
    {
        int window = 3;
        std::vector<float> vals;
        int start = std::max(0, index - window);
        int end = std::min((int)msg->ranges.size() - 1, index + window);
        for (int i = start; i <= end; ++i)
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
                vals.push_back(msg->ranges[i]);
        if (vals.empty()) return 10.0f;
        std::sort(vals.begin(), vals.end());
        return vals[vals.size() / 2];
    }

    // -------------------- MEMBERS --------------------
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_now=0, y_now=0, x_init=0, y_init=0;
    double th_now=0, th_init=0;
    double q_x=0, q_y=0, q_z=0, q_w=0;
    double d_now=0, d_aim=0, th_aim=0;

    double x_vel=0.1, th_vel=0.2;
    size_t count_;
    bool last_state_complete;

    Action current_action_;

    double front_wall_dist=0.0, left_wall_dist=0.0, right_wall_dist=0.0;

    // Lifeboat detection
    bool lifeboat_detected;
    bool lifeboat_routine_active;
    double lifeboat_reverse_dist;
    double lifeboat_distance_to_back;
    double lifeboat_detection_dist;
    int max_lifeboat_points;
};

// -------------------- MAIN --------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareRoutine>());
    rclcpp::shutdown();
    return 0;
}
