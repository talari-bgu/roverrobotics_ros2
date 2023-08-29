#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>

// LOAs:
// 0 - manual
// 1 - assisted teleoperation
// 2 - autonomous navigation

class CommandController : public rclcpp::Node {
public:
    CommandController() : Node("command_controller") {
        // Initialize subscribers and publishers
        nav2_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav2", 10, std::bind(&CommandController::nav2Callback, this, std::placeholders::_1));
        teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10, std::bind(&CommandController::teleopCallback, this, std::placeholders::_1));
        toggle_sub_ = this->create_subscription<std_msgs::msg::Int32>("loa_mode", 10, std::bind(&CommandController::toggleCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        current_mode_ = 2;
    }

    void nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_nav2_cmd_ = *msg;
        if (current_mode_ == 2) {
            cmd_pub_->publish(last_nav2_cmd_);
        }
    }

    void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_teleop_cmd_ = *msg;
        if (current_mode_ == 1 || current_mode_ == 0) {
            cmd_pub_->publish(last_teleop_cmd_);
        }
    }

    void toggleCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd_vel_stop_;
        // cmd_vel_for_stop_.linear.x = 0;
        // cmd_vel_for_stop_.angular.z = 0;
        cmd_pub_->publish(cmd_vel_stop_);
        
        current_mode_ = msg->data;

        if (current_mode_ == 0){
            
        }


        std::cout << "Called" << std::endl;
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr toggle_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    geometry_msgs::msg::Twist last_nav2_cmd_;
    geometry_msgs::msg::Twist last_teleop_cmd_;
    int current_mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
