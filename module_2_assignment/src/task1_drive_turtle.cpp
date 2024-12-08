#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node {
public:
    // Constructor: sets up the robot driver node
    RobotDriver(float b, std::string motion_type, float linear_velocity) 
        : Node("task1_robot_driver"),
          _theta(0.0), _b(b), _motion_type(motion_type), _linear_velocity(linear_velocity) {
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        auto cmdVelTopic = this->get_parameter("cmd_vel_topic").as_string();

        _publisher = this->create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic, 10);
        _timer = this->create_wall_timer(500ms, std::bind(&RobotDriver::timerCallback, this));
    }

private:
    // Member variables for the robot state
    float _theta;                // For tracking theta (used in logarithmic motion)
    float _b;                    // For the parameter of motion (radius for circular/logarithmic)
    std::string _motion_type;    // Type of motion ("circular" or "logarithmic")
    float _linear_velocity;      // Linear velocity of the robot

    // Timer callback: sends velocity commands
    void timerCallback() {
        auto message = geometry_msgs::msg::Twist();

        if (_motion_type == "circular") {
            float radius = _b;
            float angular_velocity = _linear_velocity / radius;

            message.linear.x = _linear_velocity;
            message.angular.z = angular_velocity; 
        }
        else if (_motion_type == "logarithmic") {
            float a = 0.1;
            float radius = a * std::exp(_b * _theta);
            float angular_velocity = _linear_velocity / radius; 
            _theta += angular_velocity * 0.1;
            message.linear.x = _linear_velocity; 
            message.angular.z = angular_velocity;
        }
        
        RCLCPP_INFO(this->get_logger(), "Driving Turtle");
        _publisher->publish(message);
    }

    // ROS 2 publisher and timer
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task1_robot_driver");

    // Declare parameters
    node->declare_parameter<float>("b", 4.0);
    node->declare_parameter<std::string>("motion_type", "circular");
    node->declare_parameter<float>("linear_velocity", 0.5);

    // Get parameters
    float b = node->get_parameter("b").as_double();
    std::string motion_type = node->get_parameter("motion_type").as_string();
    float linear_velocity = node->get_parameter("linear_velocity").as_double();

    rclcpp::spin(std::make_shared<RobotDriver>(b, motion_type, linear_velocity));
    rclcpp::shutdown();
    return 0;
}




