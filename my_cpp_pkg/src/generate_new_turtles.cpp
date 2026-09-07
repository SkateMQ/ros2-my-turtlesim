#include <random>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_robot_interfaces/msg/new_turtle_info.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class GenerateNewTurtlesNode : public rclcpp::Node
{
public:
    GenerateNewTurtlesNode() : Node("generate_new_turtles"),
                               gen_(std::random_device{}()),
                               dist_(0.0f, 11.0f),
                               suffix_count_(1)
    {
        this->declare_parameter("spawn_period", 2.0);
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        spawn_timer_ = this->create_wall_timer(
                        std::chrono::duration<double>(this->get_parameter("spawn_period").as_double()), 
                        [this](){callSpawn();});

        new_turtle_pub_ = this->create_publisher<my_robot_interfaces::msg::NewTurtleInfo>("new_turtle_info", 10);
        available_name_sub_ = this->create_subscription<example_interfaces::msg::String>(
                                    "available_name", 10, 
                                    [this](const example_interfaces::msg::String::SharedPtr msg){
                                        callbackAvailableName(msg);
                                    });
    }

private:
    void callSpawn()
    {
        while (!spawn_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        // generate random position
        request->set__x(dist_(gen_));
        request->set__y(dist_(gen_));
        request->set__theta(dist_(gen_));
        // recycle name using
        if (available_name_vec_.empty())
        {
            request->set__name("LiuZichen" + std::to_string(suffix_count_));
            suffix_count_++;
        } else {
            request->set__name(available_name_vec_.back());
            available_name_vec_.pop_back();
        }
        
        spawn_client_->async_send_request(request, 
                                        [this, request](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future){
                                            callbackCallSpawn(future, request);
                                        });      
    }

    void callbackCallSpawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future,
                           const turtlesim::srv::Spawn::Request::SharedPtr request)
    {
        auto response = future.get();
        auto msg = my_robot_interfaces::msg::NewTurtleInfo();
        msg.set__name(response->name);
        msg.set__x(request->x);
        msg.set__y(request->y);
        new_turtle_pub_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), 
        //             "[%s] has been created, posion: x: %f, y: %f", 
        //             response->name.c_str(), request->x, request->y);
    }

    void callbackAvailableName(const example_interfaces::msg::String::SharedPtr msg)
    {
        available_name_vec_.push_back(msg->data);
    }

    std::mt19937 gen_;
    std::uniform_real_distribution<float> dist_;
    
    std::vector<std::string> available_name_vec_;
    int suffix_count_;
    std::unordered_map<std::string, std::pair<float, float>> alive_turtle_map_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;

    rclcpp::Publisher<my_robot_interfaces::msg::NewTurtleInfo>::SharedPtr new_turtle_pub_;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr available_name_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenerateNewTurtlesNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
