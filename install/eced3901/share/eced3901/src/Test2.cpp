//Test 2: Rotation Accuracy Test
//Code prints robots estimated pose
//Measure real pose to compare




// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>		// Timer functions
#include <functional>		// Arithmetic, comparisons, and logical operations
#include <memory>		// Dynamic memory management
#include <string>		// String functions
#include <cmath>

// ROS Client Library for C++
#include "rclcpp/rclcpp.hpp"

// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RotationTest : public rclcpp::Node {
public:
	RotationTest() : Node("Rotation_Test") {
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RotationTest::topic_callback, this, _1);
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		timer_ = this->create_wall_timer(100ms, std::bind(&RotationTest::timer_callback, this));

		start_rotation(M_PI / 2.0); //rotate 90 deg
	}

private:
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		q_x = msg->pose.pose.orientation.x;
		q_y = msg->pose.pose.orientation.y;
		q_z = msg->pose.pose.orientation.z;
		q_w = msg->pose.pose.orientation.w;

		tf2::Quaternion q(q_x, q_y, q_z, q_w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		th_now = yaw;
		//RCLCPP_INFO(this->get_logger(), "Odom Acquired.");
	}
	void timer_callback()
	{
		geometry_msgs::msg::Twist cmd;

		if (!rotation_complete)
		{
			double error = wrap_angle(th_aim - th_now);

			if (std::abs(error) > 0.02) // ~1 degree tolerance
			{
				cmd.linear.x = 0.0;
				cmd.angular.z = 1.5 * error; // P-controller for heading
			}
			else
			{
				cmd.linear.x = 0.0;
				cmd.angular.z = 0.0;
				rotation_complete = true;

				RCLCPP_INFO(this->get_logger(), "Rotation complete!");
				RCLCPP_INFO(this->get_logger(), "Final Heading: %f rad", th_now);
				RCLCPP_INFO(this->get_logger(), "Heading Error: %f rad", error);
			}

			publisher_->publish(cmd);
		}
	}

	void start_rotation(double angle)
	{
		th_aim = wrap_angle(th_now + angle); // target heading relative to current
		rotation_complete = false;
	}

	double wrap_angle(double angle)
	{
		angle = fmod(angle + M_PI, 2.0 * M_PI);
		if (angle <= 0.0)
			angle += 2.0 * M_PI;
		return angle - M_PI;
	}

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	double q_x = 0, q_y = 0, q_z = 0, q_w = 0;
	double th_now = 0, th_aim = 0;
	bool rotation_complete = true;


};


	
//------------------------------------------------------------------------------------
// Main code execution
int main(int argc, char* argv[])
{
	// Initialize ROS2
	rclcpp::init(argc, argv);

	// Start node and callbacks
	rclcpp::spin(std::make_shared<SquareRoutine>());

	// Stop node 
	rclcpp::shutdown();
	return 0;
}
