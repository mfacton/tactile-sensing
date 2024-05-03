#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ExamplePublisher : public rclcpp::Node
{
public:
    ExamplePublisher() : Node("example_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/example_topic", 10);

        // Publish a message every second
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ExamplePublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExamplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

