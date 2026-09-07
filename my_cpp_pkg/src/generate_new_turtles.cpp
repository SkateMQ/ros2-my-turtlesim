#include <random>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_robot_interfaces/srv/delete_dead_turtle.hpp"
#include "my_robot_interfaces/srv/find_nearest_turtle.hpp"

using namespace std::chrono_literals;

class GenerateNewTurtlesNode : public rclcpp::Node
{
public:
    GenerateNewTurtlesNode() : Node("generate_new_turtles"),
                               gen_(std::random_device{}()),
                               dist_(0.0f, 11.0f),
                               suffix_count_(1)
    {
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        spawn_timer_ = this->create_wall_timer(2s, [this](){callSpawn();});
        delete_server_ = this->create_service<my_robot_interfaces::srv::DeleteDeadTurtle>(
                                    "delete_dead_turtle", 
                                    [this](const std::shared_ptr<my_robot_interfaces::srv::DeleteDeadTurtle::Request> request, 
                                        const std::shared_ptr<my_robot_interfaces::srv::DeleteDeadTurtle::Response> response) {
                                        callbackDeleteTurtle(request, response);
                                    });

        find_server_ = this->create_service<my_robot_interfaces::srv::FindNearestTurtle>(
                                    "find_nearest_turtle", 
                                    [this](const std::shared_ptr<my_robot_interfaces::srv::FindNearestTurtle::Request> request, 
                                        const std::shared_ptr<my_robot_interfaces::srv::FindNearestTurtle::Response> response) {
                                            callbackFindNearestTurtle(request, response);
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
        alive_turtle_map_[response->name] = {request->x, request->y};
        RCLCPP_INFO(this->get_logger(), 
                    "[%s] has been created, posion: x: %f, y: %f", 
                    response->name.c_str(), request->x, request->y);
        // for (auto it : alive_turtle_map_)
        // {
        //     RCLCPP_INFO(this->get_logger(), 
        //             "{name: %s, x: %f, y: %f}   ", 
        //             it.first.c_str(), it.second.first, it.second.second);
        // }
    }

    void callbackDeleteTurtle(const my_robot_interfaces::srv::DeleteDeadTurtle::Request::SharedPtr request,
                              const my_robot_interfaces::srv::DeleteDeadTurtle::Response::SharedPtr response)
    {
        std::string name = request->turtle_name;

        RCLCPP_INFO(this->get_logger(), "Call DeleteTurtle");

        // for (auto it : alive_turtle_map_)
        // {
        //     RCLCPP_INFO(this->get_logger(), 
        //             "{name: %s, x: %f, y: %f}   ", 
        //             it.first.c_str(), it.second.first, it.second.second);
        // }

        if (alive_turtle_map_.find(name) == alive_turtle_map_.end())
        {
            response->set__success(false);
            RCLCPP_ERROR(this->get_logger(), "Counldn't find turtle [%s]", name.c_str());
            return;
        }

        size_t deletedCount = alive_turtle_map_.erase(name);
        if (deletedCount > 0)
        {
            response->set__success(true);
            available_name_vec_.push_back(name);
            return;
        }
        
        response->set__success(false);
        RCLCPP_ERROR(this->get_logger(), "Counldn't delete turtle [%s] from record", name.c_str());
    }

    void callbackFindNearestTurtle(const my_robot_interfaces::srv::FindNearestTurtle::Request::SharedPtr request,
                              const my_robot_interfaces::srv::FindNearestTurtle::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Call FindNearestTurtle");
        if (alive_turtle_map_.empty())
        {
            response->set__there_are_turtles(false);
            RCLCPP_WARN(this->get_logger(), "There is no turtle now!");
            return;
        }

        response->set__there_are_turtles(true);
        response->set__turtle_name(alive_turtle_map_.begin()->first);   
        float min_distance = std::numeric_limits<float>::max();
        for (auto it : alive_turtle_map_)
        {
            float x_diff = request->x - it.second.first;
            float y_diff = request->y - it.second.second;
            if (x_diff * x_diff + y_diff * y_diff < min_distance)
            {
                min_distance = x_diff * x_diff + y_diff * y_diff;
                response->set__turtle_name(it.first);
                response->set__x(it.second.first);
                response->set__y(it.second.second);
            }
        }
    }

    std::mt19937 gen_;
    std::uniform_real_distribution<float> dist_;
    
    std::vector<std::string> available_name_vec_;
    int suffix_count_;
    std::unordered_map<std::string, std::pair<float, float>> alive_turtle_map_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;

    rclcpp::Service<my_robot_interfaces::srv::DeleteDeadTurtle>::SharedPtr delete_server_;
    rclcpp::Service<my_robot_interfaces::srv::FindNearestTurtle>::SharedPtr find_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenerateNewTurtlesNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
