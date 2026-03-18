#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//added in lidar messages
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SquareRoutine : public rclcpp::Node
{
public:
    SquareRoutine() : Node("Square_Routine")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&SquareRoutine::timer_callback, this));

        // Do NOT start move here — sequence_statemachine() will handle it
        count_ = 0;
        last_state_complete = 1; // ready for first step
        current_action_ = Action::IDLE;

        //adding in lidar sub

	rclcpp::QoS qos(rclcpp::KeepLast(10));
	qos.best_effort();

	lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",qos,std::bind(&SquareRoutine::lidar_callback, this, _1));
}

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    	//int front_index = msg->ranges.size() / 2; // straight ahead
    	//front_wall_dist = msg->ranges[front_index];
    	
    		int total = msg->ranges.size();
    		int front_index = total/2; //straight ahead
    		int left_index = (3*total)/4;
    		int right_index = total /4;
    		

		//int front_index = msg->ranges.size();
		//int window = 3;
		//std::vector<float> front_window;

		front_wall_dist = get_median(front_index, msg);
		left_wall_dist  = get_median(left_index, msg);
    		right_wall_dist = get_median(right_index, msg);
    		
    		RCLCPP_INFO(this->get_logger(), "F: %.2f L: %.2f R: %.2f", front_wall_dist, left_wall_dist, right_wall_dist);

	}
	
double get_median(int center_index, const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int window = 3;
    std::vector<float> vals;

    int start = std::max(0, center_index - window);
    int end   = std::min((int)msg->ranges.size() - 1, center_index + window);

    for (int i = start; i <= end; ++i) {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
            vals.push_back(msg->ranges[i]);
        }
    }

    if (vals.empty()) return 10.0; // fallback (no wall)

    std::sort(vals.begin(), vals.end());
    return vals[vals.size() / 2];
}

    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_now = msg->pose.pose.position.x;
        y_now = msg->pose.pose.position.y;
        q_x = msg->pose.pose.orientation.x;
        q_y = msg->pose.pose.orientation.y;
        q_z = msg->pose.pose.orientation.z;
        q_w = msg->pose.pose.orientation.w;
    }

    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        // Get current yaw
        tf2::Quaternion q(q_x, q_y, q_z, q_w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        th_now = yaw;

        // Distance travelled
        d_now = std::hypot(x_now - x_init, y_now - y_init);

       if (current_action_ == Action::MOVE)
{
    msg.linear.x = x_vel;

    double dist_error = d_aim - d_now;
    double threshold = 0.32;
    double k = 1.5;

    // Stop condition
    if (dist_error < 0.01 || front_wall_dist < threshold)
    {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        current_action_ = Action::IDLE;
        last_state_complete = 1;
    }
    else
    {
        double error = 0.0;

        if (left_wall_dist < threshold)
            error -= (threshold - left_wall_dist);

        if (right_wall_dist < threshold)
            error += (threshold - right_wall_dist);

        msg.angular.z = k * error;
    }
}
        else if (current_action_ == Action::TURN)
        {
            double angle_turned = wrap_angle(th_now - th_init);
            double remaining = th_aim - angle_turned;
            if (std::fabs(remaining) > 0.01) // ~0.5 deg tolerance
            {
                msg.linear.x = 0.0;
                msg.angular.z = (remaining > 0 ? th_vel : -th_vel);
            }
            else
            {
                msg.angular.z = 0.0;
                current_action_ = Action::IDLE;
                last_state_complete = 1;
            }
        }

        publisher_->publish(msg);
        sequence_statemachine();
    }

    void sequence_statemachine()
    {
        if (last_state_complete)
        {
            last_state_complete = 0;

            switch (count_)
            {
            case 0: move_distance(1.2192); RCLCPP_INFO(this->get_logger(), "Case 0"); break; // 4 ft forward
            case 1: correct_with_wall(0.32); RCLCPP_INFO(this->get_logger(), "Case 1"); break;
            case 2: turn_angle(M_PI / 2); RCLCPP_INFO(this->get_logger(), "Case 2"); break;  // turn left 90°
            case 3: move_distance(0.3068); RCLCPP_INFO(this->get_logger(), "Case 3"); break; // 1 ft forward
            case 4: correct_with_wall(0.32); RCLCPP_INFO(this->get_logger(), "Case 4"); break;
            case 5: turn_angle(-M_PI / 2); RCLCPP_INFO(this->get_logger(), "Case 5"); break; // turn right 90°
            case 6: move_distance(1.2192); RCLCPP_INFO(this->get_logger(), "Case 6"); break; // 4 ft forward
            case 7: correct_with_wall(0.32); RCLCPP_INFO(this->get_logger(), "Case 7"); break;
            case 8: turn_angle(-M_PI / 2); RCLCPP_INFO(this->get_logger(), "Case 8"); break; // turn right 90°
            case 9: move_distance(0.35); RCLCPP_INFO(this->get_logger(), "Case 9"); break; // 1 ft forward
            case 10: correct_with_wall(0.32); RCLCPP_INFO(this->get_logger(), "Case 10"); break;
            case 11: turn_angle(M_PI / 2); RCLCPP_INFO(this->get_logger(), "Case 11"); break;  // turn left 90°
            case 12: move_distance(1.2192); RCLCPP_INFO(this->get_logger(), "Case 12"); break; // 4 ft final leg
            case 13: correct_with_wall(0.32); RCLCPP_INFO(this->get_logger(), "Case 13"); break;
            default: break; // done
            }
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

    double wrap_angle(double angle)
    {
        angle = fmod(angle + M_PI, 2 * M_PI);
        if (angle < 0) angle += 2 * M_PI;
        return angle - M_PI;
    }

    //added function to correct pose
    void correct_with_wall(double target_dist) {
    	RCLCPP_INFO(this->get_logger(), "Front wall: %f", front_wall_dist);
    	double error = front_wall_dist - target_dist; // positive if too far, negative if too close

    	if (std::fabs(error) > 0.02) { // 2 cm tolerance
        	x_vel = (error > 0) ? 0.05 : -0.05; // move forward if too far, backward if too close
        	move_distance(error);        // use signed error
        	x_vel = 0.1;

    	}
    	else{
    		count_++;
    		last_state_complete = 1;
    	}
}

    // ROS members
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // Robot state
    double x_now = 0, y_now = 0, x_init = 0, y_init = 0;
    double th_now = 0, th_init = 0;
    double d_now = 0, d_aim = 0, th_aim = 0;
    double q_x = 0, q_y = 0, q_z = 0, q_w = 0;

    double x_vel = 0.1;  // m/s
    double th_vel = 0.2; // rad/s

    size_t count_ = 0;
    int last_state_complete = 1;

    enum class Action { IDLE, MOVE, TURN };
    Action current_action_ = Action::IDLE;

    //adding front wall dist variable
    double front_wall_dist = 0.0;
    //adding side wall variables
    double left_wall_dist = 0.0;
    double right_wall_dist = 0.0;
};

// Main
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareRoutine>());
    rclcpp::shutdown();
    return 0;
}




