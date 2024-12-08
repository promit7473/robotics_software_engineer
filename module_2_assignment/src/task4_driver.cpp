#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class TurtleDriver : public rclcpp::Node {
public:
    // Constructor: sets up the turtle driver node with parameters
    TurtleDriver() : Node("turtle_driver") {
        // Declare parameters with default values
        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
        this->declare_parameter<double>("linear_speed", 1.0);
        this->declare_parameter<double>("angular_speed", 1.0);

        // Get parameter values
        _cmdVelTopic = this->get_parameter("cmd_vel_topic").as_string();
        _linearSpeed = this->get_parameter("linear_speed").as_double();
        _angularSpeed = this->get_parameter("angular_speed").as_double();

        // Create publisher
        _publisher = this->create_publisher<geometry_msgs::msg::Twist>(_cmdVelTopic, 10);

        // Create timer
        _timer = this->create_wall_timer(
            500ms, 
            std::bind(&TurtleDriver::timerCallback, this)
        );

        // Add parameter callback
        _paramCallback = this->add_on_set_parameters_callback(
            std::bind(&TurtleDriver::parametersCallback, this, std::placeholders::_1)
        );
    }

private:
    // Timer callback: sends velocity commands
    void timerCallback() {
        auto message = geometry_msgs::msg::Twist();
        
        // Use current parameter values
        message.linear.x = _linearSpeed;
        message.angular.z = _angularSpeed;
        
        RCLCPP_INFO(
            this->get_logger(), 
            "Driving Turtle - Linear: %.2f, Angular: %.2f", 
            _linearSpeed, 
            _angularSpeed
        );
        
        _publisher->publish(message);
    }

    // Parameter callback: handle dynamic parameter changes
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    ) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "linear_speed") {
                _linearSpeed = param.as_double();
            } else if (param.get_name() == "angular_speed") {
                _angularSpeed = param.as_double();
            }
        }

        return result;
    }

    // Member variables
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    OnSetParametersCallbackHandle::SharedPtr _paramCallback;

    // Parameters
    std::string _cmdVelTopic;
    double _linearSpeed;
    double _angularSpeed;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleDriver>());
    rclcpp::shutdown();
    return 0;
}