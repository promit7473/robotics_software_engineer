// Purpose:
// - Receives Image messages, applies Canny edge detection, calculates the error between midpoint and center,
//   and publishes Twist messages to control robot motion.
// - Demonstrates image processing and motion control integration in a ROS2 environment.
// Author: Robotisim

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraSubscriber : public rclcpp::Node {
public:
    CameraSubscriber()
        : Node("CameraSubscriberNode"), angular_velocity_(0.3) {
        declare_parameter<int>("lower_threshold", 200);
        declare_parameter<int>("upper_threshold", 250);

        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CameraSubscriber::CameraCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "\n------ Node Started -----\n");
    }

private:
    void CameraCallback(const sensor_msgs::msg::Image::SharedPtr image_message) {
        auto twist_message = geometry_msgs::msg::Twist();
        cv_bridge::CvImagePtr cv_image_ptr;

        try {
            cv_image_ptr = cv_bridge::toCvCopy(image_message, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray_image, canny_image;
        cv::cvtColor(cv_image_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        int upper_threshold = get_parameter("upper_threshold").as_int();
        int lower_threshold = get_parameter("lower_threshold").as_int();
        cv::Canny(gray_image, canny_image, lower_threshold, upper_threshold);

        // Process Canny image to find the line's midpoint
        const int row = 150;
        const int column = 0;
        cv::Mat roi = canny_image(cv::Range(row, row + 240), cv::Range(column, column + 640));

        std::vector<int> edges;
        for (int i = 0; i < roi.cols; ++i) {
            if (roi.at<uchar>(160, i) == 255) {
                edges.push_back(i);
            }
        }

        if (!edges.empty()) {
            int mid_area = edges.back() - edges.front();
            int line_midpoint = edges.front() + mid_area / 2;
            int robot_midpoint = roi.cols / 2;

            // Calculate error and adjust robot's direction
            double error = robot_midpoint - line_midpoint;
            twist_message.linear.x = 0.1;
            twist_message.angular.z = (error < 0) ? -angular_velocity_ : angular_velocity_;

            cmd_vel_publisher_->publish(twist_message);

            // Visualization
            cv::circle(roi, cv::Point(line_midpoint, 160), 2, cv::Scalar(255, 255, 255), -1);
            cv::circle(roi, cv::Point(robot_midpoint, 160), 5, cv::Scalar(255, 255, 255), -1);
            cv::imshow("Processed Image", roi);
            cv::waitKey(1);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    double angular_velocity_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}
