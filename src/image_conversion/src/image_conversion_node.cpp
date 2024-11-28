#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <opencv2/opencv.hpp>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode()
        : Node("image_conversion_node"), mode_(1) {
        this->declare_parameter<std::string>("input_topic", "/usb_cam/image_raw");
        this->declare_parameter<std::string>("output_topic", "/image_converted");

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10, std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        mode_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_mode", std::bind(&ImageConversionNode::handleModeChange, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "ImageConversionNode initialized.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat input_image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()));

            cv::Mat output_image;
            if (mode_ == 1) { // Grayscale mode
                cv::cvtColor(input_image, output_image, cv::COLOR_BGR2GRAY);
                output_image = output_image.reshape(0, msg->height);
            } else { // Color mode
                output_image = input_image;
            }

            auto converted_msg = std::make_shared<sensor_msgs::msg::Image>();
            converted_msg->header = msg->header;
            converted_msg->height = output_image.rows;
            converted_msg->width = output_image.cols;
            converted_msg->encoding = (mode_ == 1) ? "mono8" : msg->encoding;
            converted_msg->step = output_image.step;
            converted_msg->data.assign(output_image.data, output_image.data + output_image.total() * output_image.elemSize());

            image_publisher_->publish(*converted_msg);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }

    void handleModeChange(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        mode_ = request->data ? 1 : 2;
        response->success = true;
        response->message = (mode_ == 1) ? "Mode set to Grayscale" : "Mode set to Color";
        RCLCPP_INFO(this->get_logger(), "Mode changed to: %s", response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
    std::string input_topic_, output_topic_;
    int mode_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConversionNode>());
    rclcpp::shutdown();
    return 0;
}