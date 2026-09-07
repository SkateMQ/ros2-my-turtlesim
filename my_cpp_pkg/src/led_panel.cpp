#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"
#include "my_robot_interfaces/srv/set_led_panel_state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        this->declare_parameter("led_panel_state", std::vector<bool>({false, false, false}));
        this->declare_parameter("timer_period", 8.0);

        led_panel_state_ = this->get_parameter("led_panel_state").as_bool_array();

        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedPanelState>("led_panel_state", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(this->get_parameter("timer_period").as_double()), 
                                        std::bind(&LedPanelNode::publishLedPanelState, this));
        RCLCPP_INFO(this->get_logger(), "Led Panel has been started.");
        server_ = this->create_service<my_robot_interfaces::srv::SetLedPanelState>(
            "set_led", std::bind(&LedPanelNode::callbackSetLedPanelState, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Set Led Service has been started.");   
    }

private:
    void publishLedPanelState()
    {
        auto msg = my_robot_interfaces::msg::LedPanelState();
        std::copy_n(led_panel_state_.begin(), 3, msg.led_panel_state.begin());
        publisher_->publish(msg);        
    }
    void callbackSetLedPanelState(my_robot_interfaces::srv::SetLedPanelState::Request::SharedPtr request,
                                  my_robot_interfaces::srv::SetLedPanelState::Response::SharedPtr response)
    {
        if (request->led_number < 1 || request->led_number > static_cast<int32_t>(led_panel_state_.size()))
        {
            response->message = "Invalid LED number: " + std::to_string(request->led_number) +
                                ". Valid range is 1 to " + std::to_string(led_panel_state_.size());
            response->success = false;
            RCLCPP_WARN(this->get_logger(), 
                        "Invalid LED number: %d. Valid range is 1 to %ld", 
                        request->led_number, led_panel_state_.size());
            return;
        }
        
        led_panel_state_[request->led_number - 1] = request->state;
        this->publishLedPanelState();
        response->success = true;
        response->message = "Set LED " + std::to_string(request->led_number) + " to state " + std::to_string(request->state);
    }
    std::vector<bool> led_panel_state_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<my_robot_interfaces::srv::SetLedPanelState>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
