#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/delete_dead_turtle.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/srv/eat_turtle.hpp"

class EatTurtleNode : public rclcpp::Node
{
public:
    EatTurtleNode() : Node("eat_turtle")
    {
        delete_client_ = this->create_client<my_robot_interfaces::srv::DeleteDeadTurtle>("delete_dead_turtle");
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        eat_turtle_server_ = this->create_service<my_robot_interfaces::srv::EatTurtle>(
                                                "eat_turtle", 
                                                [this](const std::shared_ptr<my_robot_interfaces::srv::EatTurtle::Request> request, 
                                                const std::shared_ptr<my_robot_interfaces::srv::EatTurtle::Response> response){
                                                    callbackEatTurtle(request, response);
                                                }
                                            );
    }

private:
    void callbackEatTurtle(const my_robot_interfaces::srv::EatTurtle::Request::SharedPtr request,
                              const my_robot_interfaces::srv::EatTurtle::Response::SharedPtr response)
    {
        auto delete_request = std::make_shared<my_robot_interfaces::srv::DeleteDeadTurtle::Request>();
        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();

        delete_request->set__turtle_name(request->name);
        kill_request->set__name(request->name);
        
        delete_client_->async_send_request(delete_request);               
        kill_client_->async_send_request(kill_request);
        
        response->set__success(true);
    }

    rclcpp::Client<my_robot_interfaces::srv::DeleteDeadTurtle>::SharedPtr delete_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Service<my_robot_interfaces::srv::EatTurtle>::SharedPtr eat_turtle_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EatTurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
