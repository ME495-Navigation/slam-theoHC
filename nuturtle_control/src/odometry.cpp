#include <nuturtle_control/odometry.hpp>
#include <string>
#include "tf2/LinearMath/Quaternion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

odometry::odometry()
: Node("odometry")
{
  // Name of the body frame of the robot
  declare_parameter<std::string>("body_id", "base_footprint");
  // Name of the frame that the robots position is computed relative to
  declare_parameter<std::string>("odom_id", "odom");
  // Name of the left wheel joint
  declare_parameter<std::string>("wheel_left");
  // Name of the right wheel joint
  declare_parameter<std::string>("wheel_right");
  // Radius of the wheels of the robot
  declare_parameter<double>("wheel_radius");
  // Distance between the wheels of the robot
  declare_parameter<double>("track_width");

  rclcpp::Parameter filler_param;
  if(!(get_parameter("wheel_left", filler_param) &&
    get_parameter("wheel_right", filler_param) &&
    get_parameter("wheel_radius", filler_param) &&
    get_parameter("track_width", filler_param)))
  {
    RCLCPP_ERROR(get_logger(), "Wheel link names and robot parameters must be set");
    rclcpp::shutdown();
  }

  // Subscribes to the joint states of the robot of interest
  jsSub = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10,
                 std::bind(&odometry::odomCallback, this, std::placeholders::_1));
  // Publishes the computed odometry
  odomPub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Publishes the computed offset of the robot relative to the odom frame
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Service to set the initial pose of the robot
  initPoseServ = this->create_service<nuturtle_control::srv::OdomConfig>(
        "initial_pose",
        std::bind(&odometry::initPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

  robotState = turtlelib::DiffDrive(get_parameter("wheel_radius").as_double(),
                                    get_parameter("track_width").as_double());
}

void odometry::odomCallback(const sensor_msgs::msg::JointState msg)
{
// .at(), especially because anybody can publish a joint_state message so it actually is possible
    // that your code encounters msg.position.size() == 0 for example
    double left_wheel_pos = msg.position[0]; 
  double right_wheel_pos = msg.position[1];

    //Set the wheel positions if we haven't to avoid jumps on the first callback
  if(!hasSetWheels) {
    robotState.set_wheels(left_wheel_pos, right_wheel_pos);
    hasSetWheels = true;
  }

  robotState.forwardK(left_wheel_pos, right_wheel_pos);

    //Publish TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = get_parameter("odom_id").as_string();
  t.child_frame_id = get_parameter("body_id").as_string();
  turtlelib::Transform2D pose = robotState.get_pose();
  t.transform.translation.x = pose.translation().x;
  t.transform.translation.y = pose.translation().y;
  t.transform.translation.z = 0.0;
  double theta = pose.rotation();
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  t.transform.rotation = tf2::toMsg(q);

  tf_broadcaster->sendTransform(t);

  turtlelib::Twist2D vel = robotState.get_twist();

    //Publish Odometry Message
  nav_msgs::msg::Odometry odomMsg;
  odomMsg.header.stamp = t.header.stamp;
  odomMsg.header.frame_id = get_parameter("odom_id").as_string();
  odomMsg.child_frame_id = get_parameter("body_id").as_string();
  odomMsg.pose.pose.position.x = pose.translation().x;
  odomMsg.pose.pose.position.y = pose.translation().y;
  odomMsg.pose.pose.position.z = 0.0;
  odomMsg.pose.pose.orientation = tf2::toMsg(q);
  odomMsg.twist.twist.linear.x = vel.x;
  odomMsg.twist.twist.linear.y = 0.0;
  odomMsg.twist.twist.angular.z = vel.omega;
  odomPub->publish(odomMsg);
}

void odometry::initPoseCallback(
  const std::shared_ptr<nuturtle_control::srv::OdomConfig::Request> request,
  std::shared_ptr<nuturtle_control::srv::OdomConfig::Response>)
{
  turtlelib::Transform2D new_pose(request->theta, request->x, request->y);
  robotState.set_pose(new_pose);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = get_parameter("odom_id").as_string();
  t.child_frame_id = get_parameter("body_id").as_string();
  turtlelib::Transform2D pose = robotState.get_pose();
  t.transform.translation.x = pose.translation().x;
  t.transform.translation.y = pose.translation().y;
  t.transform.translation.z = 0.0;
  double theta = pose.rotation();
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  t.transform.rotation = tf2::toMsg(q);

  tf_broadcaster->sendTransform(t);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odometry>());
  rclcpp::shutdown();
  return 0;
}
