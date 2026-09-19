#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/new_turtle_info.hpp"
#include "turtlesim/srv/kill.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

struct PidController
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float intergral = 0.0f;
    float prev_error = 0.0f;
};


class HuntTurtlesNode : public rclcpp::Node
{
public:
    HuntTurtlesNode() : Node("hunt_turtles"), curr_x_(5.544445f), curr_y_(5.544445f), curr_theta_(0.0f), 
                                            is_hunting_(false)
    {
        this->declare_parameter("linear_kp", 1.0);
        this->declare_parameter("linear_ki", 0.0);
        this->declare_parameter("linear_kd", 0.0);

        this->declare_parameter("angular_kp", 1.0);
        this->declare_parameter("angular_ki", 0.0);
        this->declare_parameter("angular_kd", 0.0);

        this->declare_parameter("eat_error", 0.0001);

        this->declare_parameter("control_period", 0.015);

        min_eat_error_ = this->get_parameter("eat_error").as_double();

        linear_pid_.kp = (float)this->get_parameter("linear_kp").as_double();
        linear_pid_.ki = (float)this->get_parameter("linear_ki").as_double();
        linear_pid_.kd = (float)this->get_parameter("linear_kd").as_double();

        angular_pid_.kp = (float)this->get_parameter("angular_kp").as_double();
        angular_pid_.ki = (float)this->get_parameter("angular_ki").as_double();
        angular_pid_.kd = (float)this->get_parameter("angular_kd").as_double();

        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
                            "/turtle1/pose", 10,
                            [this](const turtlesim::msg::Pose::SharedPtr msg){
                                callbackPose(msg);
                            });
        new_turtle_sub_ = this->create_subscription<my_robot_interfaces::msg::NewTurtleInfo>(
                            "new_turtle_info", 10, 
                            [this](const my_robot_interfaces::msg::NewTurtleInfo::SharedPtr msg){
                                callbackNewTurtle(msg);
                            });

        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        available_name_pub_ = this->create_publisher<example_interfaces::msg::String>("available_name", 10);

        moving_control_timer_ = this->create_wall_timer(
                                std::chrono::duration<double>(this->get_parameter("control_period").as_double()), 
                                [this](){ movingControl(); });
    }

private:
    void movingControl()
    {
        if (is_hunting_)
        {
            float linear_err = computeLinearErr();
            // RCLCPP_INFO(this->get_logger(), "linear_err: %f", linear_err);
            eatIfNear(linear_err);
            float angular_err = computeAngularErr();

            float linear_movement = computePid(linear_err, linear_pid_);
            float angular_movement = computePid(angular_err, angular_pid_);
            
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.set__x(linear_movement);
            msg.angular.set__z(angular_movement);
            cmd_vel_pub_->publish(msg);
        }
        
    }

    float computePid(float err, PidController pid)
    {
        float period = this->get_parameter("control_period").as_double();
        pid.intergral += period * err;
        float derivative = (err - pid.prev_error) / period;
        float outcome = pid.kp * err + pid.ki * pid.intergral + pid.kd * derivative;
        pid.prev_error = err;
        return outcome;
    }

    float computeLinearErr()
    {
        // RCLCPP_INFO(this->get_logger(), "curr_prey: %s, des_x_: %f, des_y_: %f, curr_x_: %f, curr_y_: %f", 
        //                                 curr_prey_.c_str(), des_x_, des_y_, curr_x_, curr_y_);
        float x_err = des_x_ - curr_x_;
        float y_err = des_y_ - curr_y_;

        return sqrtf(x_err * x_err + y_err * y_err);
    }

    float computeAngularErr()
    {
        float des_theta = atan2(des_y_ - curr_y_, des_x_ - curr_x_);
        if (des_theta - curr_theta_ > M_PIf)
        {
            return des_theta - curr_theta_ - M_PIf * 2.0f;
        }
        if (des_theta - curr_theta_ < -M_PIf)
        {
            return des_theta - curr_theta_ + M_PIf * 2.0f;
        }
        
        return des_theta - curr_theta_;
    }

    void eatIfNear(float linear_err)
    {
        if (linear_err < min_eat_error_)
        {
            auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
            kill_request->set__name(curr_prey_);

            kill_client_->async_send_request(kill_request, 
                                            [this](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future){
                                                callbackKill(future);
                                            });
        }
    }

    void callbackKill(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        future.get();

        // for (auto it : alive_turtle_map_)
        // {
        //     RCLCPP_INFO(this->get_logger(), 
        //             "{name: %s, x: %f, y: %f}   ", 
        //             it.first.c_str(), it.second.first, it.second.second);
        // }

        if (alive_turtle_map_.find(curr_prey_) == alive_turtle_map_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Counldn't find turtle [%s]", curr_prey_.c_str());
            return;
        }

        size_t deletedCount = alive_turtle_map_.erase(curr_prey_);
        if (deletedCount > 0)
        {
            auto msg = example_interfaces::msg::String();
            msg.set__data(curr_prey_);
            available_name_pub_->publish(msg);

            is_hunting_ = false;
            findNearestTurtle();
            clearPid();
            return;
        }
        
        RCLCPP_ERROR(this->get_logger(), "Counldn't delete turtle [%s] from record", curr_prey_.c_str());
    }

    void findNearestTurtle()
    {
        if (alive_turtle_map_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "There is no turtle now!");
            return;
        }

        float min_distance_squa = std::numeric_limits<float>::max();
        for (auto it : alive_turtle_map_)
        {
            float x_diff = curr_x_ - it.second.first;
            float y_diff = curr_y_ - it.second.second;
            if (x_diff * x_diff + y_diff * y_diff < min_distance_squa)
            {
                min_distance_squa = x_diff * x_diff + y_diff * y_diff;
                curr_prey_ = it.first;
                des_x_ = it.second.first;
                des_y_ = it.second.second;
            }
        }
        is_hunting_ = true;
    }

    void callbackPose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        curr_x_ = msg->x;
        curr_y_ = msg->y;
        curr_theta_ = msg->theta;
        // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", curr_x_, curr_y_, curr_theta_);
    }

    void callbackNewTurtle(const my_robot_interfaces::msg::NewTurtleInfo::SharedPtr msg)
    {
        alive_turtle_map_[msg->name] = {msg->x, msg->y};
        if (alive_turtle_map_.size() == 1)
        {
            RCLCPP_WARN(this->get_logger(), "There is only one prey.");
            curr_prey_ = msg->name;
            des_x_ = msg->x;
            des_y_ = msg->y;
            is_hunting_ = true;
            clearPid();
            return;
        }

        float des_distance_squa = (curr_x_ - des_x_) * (curr_x_ - des_x_) + (curr_y_ - des_y_) * (curr_y_ - des_y_);
        float curr_distance_squa = (curr_x_ - msg->x) * (curr_x_ - msg->x) + (curr_y_ - msg->y) * (curr_y_ - msg->y);

        if (curr_distance_squa < des_distance_squa)
        {
            // RCLCPP_WARN(this->get_logger(), "Target has changed from [%s] to [%s].", curr_prey_.c_str(), msg->name.c_str());
            // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", curr_x_, curr_y_, curr_theta_);
            curr_prey_ = msg->name;
            des_x_ = msg->x;
            des_y_ = msg->y;
            is_hunting_ = true;
            clearPid();
        }
        // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", curr_x_, curr_y_, curr_theta_);
        // RCLCPP_INFO(this->get_logger(), "des_x: %f, des_y: %f", des_x_, des_y_);
    }

    void clearPid()
    {
        linear_pid_.intergral = 0.0f;
        linear_pid_.prev_error = 0.0f;

        angular_pid_.intergral = 0.0f;
        angular_pid_.prev_error = 0.0f;
    }

    float curr_x_;
    float curr_y_;
    float curr_theta_;

    std::string curr_prey_;
    float des_x_;
    float des_y_;

    float min_eat_error_;

    bool is_hunting_;

    PidController linear_pid_;
    PidController angular_pid_;
    
    std::unordered_map<std::string, std::pair<float, float>> alive_turtle_map_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<my_robot_interfaces::msg::NewTurtleInfo>::SharedPtr new_turtle_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr available_name_pub_;


    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;

    rclcpp::TimerBase::SharedPtr moving_control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HuntTurtlesNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
