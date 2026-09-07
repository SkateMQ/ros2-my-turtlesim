#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led_panel_state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery"), time_counter_(0)
    {
        client_ = this->create_client<my_robot_interfaces::srv::SetLedPanelState>("set_led");
        timer_ = this->create_wall_timer(1s, std::bind(&BatteryNode::simulateBatteryLife, this));   
    }

    void callSetLed(int32_t led_number, bool state)
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLedPanelState::Request>();
        request->led_number = led_number;
        request->state = state;

        client_->async_send_request(request, std::bind(&BatteryNode::callbackCallSetLed, this, _1));   
    }

private:
    void callbackCallSetLed(rclcpp::Client<my_robot_interfaces::srv::SetLedPanelState>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), response->message.c_str());
            return;   
        }
        RCLCPP_WARN(this->get_logger(), response->message.c_str());
    }

    void simulateBatteryLife()
    {
        time_counter_++;
        if (time_counter_ == 4)
        {
            this->callSetLed(3, true);
            RCLCPP_WARN(this->get_logger(), "Battery is empty!");
        } else if (time_counter_ == 10)
        {
            time_counter_ = 0;
            this->callSetLed(3, false);
            RCLCPP_INFO(this->get_logger(), "\033[32mBattery fully charged (100%%)!\033[0m");
        }
    }

    rclcpp::Client<my_robot_interfaces::srv::SetLedPanelState>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int time_counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
