#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TurtleDriver : public rclcpp::Node {
public:
    TurtleDriver() : Node("turtle_driver"), _count(0), _direction(1) {
        // Declare parameters with default values
        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
        this->declare_parameter<double>("linear_speed", 1.0);
        this->declare_parameter<double>("travel_distance", 0.3);
        this->declare_parameter<int>("publish_frequency", 2); // 2 Hz

        // Get parameter values
        auto cmdVelTopic = this->get_parameter("cmd_vel_topic").as_string();
        _linearSpeed = this->get_parameter("linear_speed").as_double();
        _travelDistance = this->get_parameter("travel_distance").as_double();
        int publishFreq = this->get_parameter("publish_frequency").as_int();

        // Create publisher and timer
        _publisher = this->create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic, 10);
        
        // Create timer based on publish frequency
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / publishFreq)), 
            std::bind(&TurtleDriver::timerCallback, this)
        );
    }

private:
    void timerCallback() {
        auto message = geometry_msgs::msg::Twist();
        
        // Move back and forth
        message.linear.x = _linearSpeed * _direction; // Linear velocity with direction
        
        // Alternate direction when reaching limits
        if (std::abs(_count * 0.5) > _travelDistance) {
            _direction *= -1;
        }
        
        _count++;
        
        RCLCPP_INFO(this->get_logger(), 
            "Driving Turtle: Linear Velocity = %f, Direction = %d", 
            message.linear.x, _direction
        );
        _publisher->publish(message);
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    size_t _count;
    int _direction;
    double _linearSpeed;
    double _travelDistance;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleDriver>());
    rclcpp::shutdown();
    return 0;
}