#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/srv/find_nearest_turtle.hpp"
#include "my_robot_interfaces/srv/eat_turtle.hpp"

using namespace std::chrono_literals;

struct PidController
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float intergral = 0.0f;
    float prev_error = 0.0f;
};


class MoveTurtleNode : public rclcpp::Node
{
public:
    MoveTurtleNode() : Node("move_turtle"), curr_x(5.544445f), curr_y(5.544445f), curr_theta(0.0f), 
                                            is_hunting(false)
    {
        this->declare_parameter("linear_kp", 1.0);
        this->declare_parameter("linear_ki", 0.0);
        this->declare_parameter("linear_kd", 0.0);

        this->declare_parameter("angular_kp", 1.0);
        this->declare_parameter("angular_ki", 0.0);
        this->declare_parameter("angular_kd", 0.0);

        this->declare_parameter("eat_error", 0.0001);

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
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        find_turtle_client_ = this->create_client<my_robot_interfaces::srv::FindNearestTurtle>("find_nearest_turtle");
        eat_turtle_client_ = this->create_client<my_robot_interfaces::srv::EatTurtle>("eat_turtle");

        this->declare_parameter("control_period", 0.2);
        moving_control_timer_ = this->create_wall_timer(
                                std::chrono::duration<double>(this->get_parameter("control_period").as_double()), 
                                [this](){ movingControl(); });
    }

private:
    void movingControl()
    {
        if (is_hunting)
        {
            float linear_err = computeLinearErr();
            RCLCPP_INFO(this->get_logger(), "linear_err: %f", linear_err);
            eatIfNear(linear_err);
            float angular_err = computeAngularErr();

            float linear_movement = computePid(linear_err, linear_pid_);
            float angular_movement = computePid(angular_err, angular_pid_);
            
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.set__x(linear_movement);
            msg.angular.set__z(angular_movement);
            cmd_vel_pub_->publish(msg);
        } else {
            auto find_req = std::make_shared<my_robot_interfaces::srv::FindNearestTurtle::Request>();
            find_req->set__x(curr_x);
            find_req->set__y(curr_y);

            find_turtle_client_->async_send_request(
                                find_req, 
                                [this](rclcpp::Client<my_robot_interfaces::srv::FindNearestTurtle>::SharedFuture future){
                                    callbackFindNearestTurtle(future);
                                });
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
        RCLCPP_INFO(this->get_logger(), "des_x: %f, des_y: %f, curr_x: %f, curr_y: %f", des_x, des_y, curr_x, curr_y);
        float x_err = des_x - curr_x;
        float y_err = des_y - curr_y;

        return sqrtf(x_err * x_err + y_err * y_err);
    }

    float computeAngularErr()
    {
        float des_theta = atan2(des_y - curr_y, des_x - curr_x);
        if (des_theta - curr_theta > M_PIf)
        {
            return des_theta - curr_theta - M_PIf * 2.0f;
        }
        if (des_theta - curr_theta < -M_PIf)
        {
            return des_theta - curr_theta + M_PIf * 2.0f;
        }
        
        return des_theta - curr_theta;
    }

    void callbackFindNearestTurtle(rclcpp::Client<my_robot_interfaces::srv::FindNearestTurtle>::SharedFuture future)
    {
        auto response = future.get();
        if (response->there_are_turtles)
        {
            des_x = response->x;
            des_y = response->y;
            curr_prey = response->turtle_name;
            is_hunting = true;
        }
    }

    void eatIfNear(float linear_err)
    {
        if (linear_err < min_eat_error_)
        {
            auto request = std::make_shared<my_robot_interfaces::srv::EatTurtle::Request>();
            request->set__name(curr_prey);

            eat_turtle_client_->async_send_request(request,
                                                   [this](rclcpp::Client<my_robot_interfaces::srv::EatTurtle>::SharedFuture future){
                                                    callbackEatIfNear(future);
                                                   });
        }
    }

    void callbackEatIfNear(rclcpp::Client<my_robot_interfaces::srv::EatTurtle>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success)
        {
            is_hunting = false;
            linear_pid_.intergral = 0.0f;
            linear_pid_.prev_error = 0.0f;

            angular_pid_.intergral = 0.0f;
            angular_pid_.prev_error = 0.0f;
        }
    }

    void callbackPose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        curr_x = msg->x;
        curr_y = msg->y;
        curr_theta = msg->theta;
        // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", curr_x, curr_y, curr_theta);
    }

    float curr_x;
    float curr_y;
    float curr_theta;

    float des_x;
    float des_y;

    float min_eat_error_;

    bool is_hunting;
    std::string curr_prey;

    PidController linear_pid_;
    PidController angular_pid_;
    
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<my_robot_interfaces::srv::FindNearestTurtle>::SharedPtr find_turtle_client_;
    rclcpp::Client<my_robot_interfaces::srv::EatTurtle>::SharedPtr eat_turtle_client_;

    rclcpp::TimerBase::SharedPtr moving_control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveTurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
