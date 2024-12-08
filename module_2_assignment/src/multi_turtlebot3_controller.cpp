#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node {
public:
    RobotDriver() : Node("turtlebot3_driver"), 
                    _current_direction(1), 
                    _distance_traveled(0.0) {
        std::string default_topic = this->get_namespace();
        
        if (!default_topic.empty() && default_topic[0] == '/') {
            default_topic = default_topic.substr(1);
        }
        default_topic += "/cmd_vel";

        // Declare and get parameters with namespace-aware default
        this->declare_parameter<std::string>("cmd_vel_topic", default_topic);
        this->declare_parameter<double>("linear_speed", 0.5);
        this->declare_parameter<double>("max_distance", 2.0);

        _cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        _linear_speed = this->get_parameter("linear_speed").as_double();
        _max_distance = this->get_parameter("max_distance").as_double();


        _publisher = this->create_publisher<geometry_msgs::msg::Twist>(_cmd_vel_topic, 10);
        
        _timer = this->create_wall_timer(
            100ms, 
            std::bind(&RobotDriver::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Robot driver initialized on topic: %s", _cmd_vel_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Namespace: %s", this->get_namespace());
    }

private:
    void timerCallback() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = _linear_speed * _current_direction;
        message.linear.z = 0.0;

        _publisher->publish(message);
        
        _distance_traveled += std::abs(_linear_speed * 0.1);
        
        if (_distance_traveled >= _max_distance) {
            _current_direction *= -1;  // Reverse direction
            _distance_traveled = 0.0;
            RCLCPP_INFO(this->get_logger(), "Changing direction on %s", _cmd_vel_topic.c_str());
        }
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    
    int _current_direction;
    double _distance_traveled;
    
    std::string _cmd_vel_topic;
    double _linear_speed;
    double _max_distance;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}