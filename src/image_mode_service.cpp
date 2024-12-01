#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class ImageModeServiceNode : public rclcpp::Node {
public:
    ImageModeServiceNode() : Node("image_mode_service_node"), mode_("color") {
        
        // Declare and get the input topic and output topic from parameters
        this->declare_parameter<std::string>("input_image_topic", "/image_raw");
        std::string input_topic = this->get_parameter("input_image_topic").as_string();

        this->declare_parameter<std::string>("output_topic", "/processed_image");
        std::string output_topic = this->get_parameter("output_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing processed Image on: %s", output_topic.c_str());

        // Subscription to the image topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10, std::bind(&ImageModeServiceNode::imageCallback, this, std::placeholders::_1));

        // Publisher for the processed image
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

        // Service to handle mode switching
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "/set_mode", std::bind(&ImageModeServiceNode::handleSetMode, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Image Mode Service Node started.");
    }

private:
    void handleSetMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        mode_ = request->data ? "gray" : "color";
        response->success = true;
        response->message = "Mode set to " + mode_;
        RCLCPP_INFO(this->get_logger(), "Mode changed to: %s", mode_.c_str());
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image message to OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat processed_image;

            // Convert to grayscale if mode is "gray"
            if (mode_ == "gray") {
                cv::cvtColor(image, processed_image, cv::COLOR_BGR2GRAY);
            } else {
                processed_image = image.clone();
            }

            cv::waitKey(1);  // Refresh the window

            // Publish the processed image
            auto processed_msg = cv_bridge::CvImage(msg->header, mode_ == "gray" ? "mono8" : "bgr8", processed_image).toImageMsg();
            image_publisher_->publish(*processed_msg);

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    std::string mode_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageModeServiceNode>();

    // Spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
