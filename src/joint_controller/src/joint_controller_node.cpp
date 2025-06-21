#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SimplePublisherNode : public rclcpp::Node
{
    public:
        SimplePublisherNode()
        : Node("simple_publisher_node")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&SimplePublisherNode::timer_callback, this)
            );
        }

    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, ROS2!";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
