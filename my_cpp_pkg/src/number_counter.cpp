#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callbackNumberPublisher, this, _1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");

        server_ = this->create_service<example_interfaces::srv::SetBool>(
                  "reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Reset Counter Service has been started.");
    }

private:
    void callbackNumberPublisher(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        auto newMsg = example_interfaces::msg::Int64();
        counter_ += msg->data;
        newMsg.data = counter_;
        publisher_->publish(newMsg);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                              const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->set__message("The counter has been set to 0.");
            RCLCPP_INFO(this->get_logger(), "%s",response->message.c_str());
            return;
        }
        response->success = false;
        response->set__message("The counter has not been set to 0!");
        RCLCPP_WARN(this->get_logger(), "%s",response->message.c_str());
    }
    int64_t counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
