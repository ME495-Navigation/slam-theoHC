#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlelib/diff_drive.hpp>

class turtle_control : public rclcpp::Node{
    public:
        turtle_control();
    
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
        rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensorDataSub;

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void sensorDataCallback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg);

        rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelCommander;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;

        turtlelib::DiffDrive robotState;
};