#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station")
    {
        this->declare_parameter("robot_name", "R2D2");
        this->declare_parameter("timer_period", 1.0);
        robot_name_ = this->get_parameter("robot_name").as_string();
        double timer_period = this->get_parameter("timer_period").as_double();

        // param_callback_handle_ = this->add_post_set_parameters_callback(
        //                         std::bind(&RobotNewsStationNode::parametersCallback, this, _1));
        
        param_callback_handle_ = this->add_post_set_parameters_callback(
                                [this](auto parameters) {
                                    parametersCallback(parameters);
                                });
        
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), 
                                        std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
    }

private:
    void parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        for(const auto & param:parameters) {
            if(param.get_name() == "robot_name"){
                robot_name_ = param.as_string();
            } else if (param.get_name() == "timer_period")
            {
                timer_->cancel();
                timer_ = this->create_wall_timer(std::chrono::duration<double>(param.as_double()), 
                                        std::bind(&RobotNewsStationNode::publishNews, this));
            }
        }
}

    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the robot news station.");
        publisher_->publish(msg); 
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
