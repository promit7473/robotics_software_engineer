#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

enum class RobotState {
    MovingStraight,
    TurningLeft,
    TurningRight,
    OutOfMaze
};

class MazeSolver1 : public rclcpp::Node {
public:
    MazeSolver1()
        : Node("maze_solver1"),
          robot_state_(RobotState::MovingStraight),
          current_velocity_(0.0),
          last_time_(this->now()) {
        // Publisher for velocity commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscription for LIDAR data
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeSolver1::lidarCallback, this, std::placeholders::_1));

        // Subscription for IMU data
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&MazeSolver1::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Maze Solver Node Initialized");
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) {
        // Get minimum distances for front and right sectors
        float right_obstacle = *std::min_element(lidar_msg->ranges.begin() + 260,
                                                 lidar_msg->ranges.begin() + 280);
        float front_obstacle = *std::min_element(lidar_msg->ranges.begin() + 340,
                                                 lidar_msg->ranges.begin() + 360);

        RCLCPP_INFO(this->get_logger(), "LIDAR - Front: %.2f, Right: %.2f", front_obstacle, right_obstacle);

        geometry_msgs::msg::Twist velocity_command;

        // Determine robot state based on sensor input
        if (front_obstacle < FRONT_THRESHOLD_) {
            robot_state_ = RobotState::TurningRight;  // Obstacle ahead, turn right
        } else if (right_obstacle < DESIRED_DISTANCE_) {
            robot_state_ = RobotState::TurningLeft;   // Too close to wall, turn left
        } else if (right_obstacle > DESIRED_DISTANCE_) {
            robot_state_ = RobotState::TurningRight;  // Too far from wall, turn right
        } else {
            robot_state_ = RobotState::MovingStraight;  // Maintain straight path
        }

        // Execute velocity commands based on robot state
        switch (robot_state_) {
        case RobotState::MovingStraight:
            velocity_command.linear.x = LINEAR_VELOCITY_;
            velocity_command.angular.z = 0.0;
            break;
        case RobotState::TurningLeft:
            velocity_command.linear.x = TURN_SPEED_;
            velocity_command.angular.z = ANGULAR_VELOCITY_;
            break;
        case RobotState::TurningRight:
            velocity_command.linear.x = TURN_SPEED_;
            velocity_command.angular.z = -ANGULAR_VELOCITY_;
            break;
        case RobotState::OutOfMaze:
            velocity_command.linear.x = 0.0;
            velocity_command.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Robot is out of the maze!");
            break;
        }

        // Publish velocity command
        cmd_vel_publisher_->publish(velocity_command);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double linear_acceleration = imu_msg->linear_acceleration.x;

        // Integrate acceleration to calculate velocity
        current_velocity_ += linear_acceleration * dt;

        RCLCPP_INFO(this->get_logger(),
                    "IMU - Velocity: %.2f m/s, Δt: %.3f s", current_velocity_, dt);
    }

    // Constants for movement and sensor thresholds
    static constexpr float FRONT_THRESHOLD_ = 1.0f;       // Distance to front obstacle
    static constexpr float DESIRED_DISTANCE_ = 1.1f;      // Ideal wall-following distance
    static constexpr float LINEAR_VELOCITY_ = 0.5f;       // Forward speed
    static constexpr float ANGULAR_VELOCITY_ = 0.3f;      // Turn rate
    static constexpr float TURN_SPEED_ = 0.2f;            // Speed during turns

    RobotState robot_state_;
    double current_velocity_;
    rclcpp::Time last_time_;

    // ROS 2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeSolver1>());
    rclcpp::shutdown();
    return 0;
}
