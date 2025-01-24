#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

enum class RobotState {
    MovingStraight,
    TurningLeft,
    TurningRight,
    OutOfMaze
};

class MazeSolver : public rclcpp::Node {
public:
    MazeSolver()
        : Node("maze_solver"), robot_state_(RobotState::MovingStraight),
          current_velocity_(0.0), current_acceleration_(0.0), last_time_(this->now()) {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeSolver::lidar_callback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&MazeSolver::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Maze Solver Node Initialized");
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) {
        float right_obstacle = *std::min_element(lidar_msg->ranges.begin() + 260,
                                                 lidar_msg->ranges.begin() + 280);
        float front_obstacle = *std::min_element(lidar_msg->ranges.begin() + 340,
                                                 lidar_msg->ranges.begin() + 360);
        float left_obstacle = *std::min_element(lidar_msg->ranges.begin() + 80,
                                                lidar_msg->ranges.begin() + 100);

        RCLCPP_INFO(this->get_logger(), "LIDAR - Front: %.2f, Right: %.2f, Left: %.2f",
                    front_obstacle, right_obstacle, left_obstacle);

        if (front_obstacle < front_threshold_) {
            // If an obstacle is ahead, decide to turn left or right
            robot_state_ = (left_obstacle > right_obstacle) ? RobotState::TurningRight
                                                            : RobotState::TurningLeft;
        } else if (front_obstacle > open_threshold_ &&
                   right_obstacle > open_threshold_ &&
                   left_obstacle > open_threshold_) {
            // If there are no walls, the robot is out of the maze
            robot_state_ = RobotState::OutOfMaze;
        } else {
            // Move straight if no obstacle is in front
            robot_state_ = RobotState::MovingStraight;
        }

        geometry_msgs::msg::Twist command;
        switch (robot_state_) {
        case RobotState::MovingStraight:
            command.linear.x = linear_velocity_;
            command.angular.z = 0.0;
            break;
        case RobotState::TurningLeft:
            command.linear.x = 0.0;
            command.angular.z = -angular_velocity_;
            break;
        case RobotState::TurningRight:
            command.linear.x = 0.0;
            command.angular.z = angular_velocity_;
            break;
        case RobotState::OutOfMaze:
            command.linear.x = 0.0;
            command.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Robot is out of the maze!");
            break;
        }

        cmd_vel_publisher_->publish(command);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double linear_acceleration = imu_msg->linear_acceleration.x;

        // Filter out small noise in acceleration
        if (std::abs(linear_acceleration) < noise_threshold_) {
            linear_acceleration = 0.0;
        }

        // Integrate acceleration to calculate velocity
        current_velocity_ += linear_acceleration * dt;
        current_acceleration_ = linear_acceleration;

        // Prevent velocity drift (e.g., if it's very small, set it to 0)
        if (std::abs(current_velocity_) < velocity_threshold_) {
            current_velocity_ = 0.0;
        }

        RCLCPP_INFO(this->get_logger(),
                    "IMU - Velocity: %.2f m/s, Acceleration: %.2f m/s², Δt: %.3f s",
                    current_velocity_, current_acceleration_, dt);
    }

    // Parameters
    float front_threshold_ = 1.2f;
    float open_threshold_ = 3.0f;
    float linear_velocity_ = 0.7f;
    float angular_velocity_ = 1.0f;
    float noise_threshold_ = 0.05f;  // Ignore small acceleration noise
    float velocity_threshold_ = 0.01f;  // Consider velocity zero if it's below this

    // State variables
    RobotState robot_state_;
    double current_velocity_;
    double current_acceleration_;
    rclcpp::Time last_time_; // Store the last time for dt calculation

    // ROS 2 Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeSolver>());
    rclcpp::shutdown();
    return 0;
}