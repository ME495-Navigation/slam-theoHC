#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>

class turtle_control : public rclcpp::Node{
    public:
        turtle_control();
    
    private:
        rclcpp::TimerBase::SharedPtr control_timer;
};