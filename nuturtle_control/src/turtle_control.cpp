#include <nuturtle_control/turtle_control.hpp>

turtle_control::turtle_control()
: Node("turtle_control")
{
  declare_parameter<double>("wheel_radius");
  declare_parameter<double>("track_width");
  declare_parameter<double>("encoder_ticks_per_rad");
  declare_parameter<double>("motor_cmd_per_rad_sec");

  rclcpp::Parameter filler_param;

  if(!(get_parameter("wheel_radius", filler_param) &&
    get_parameter("track_width", filler_param) &&
    get_parameter("encoder_ticks_per_rad", filler_param)))
  {
    RCLCPP_ERROR(get_logger(), "Parameters 'wheel_radius' and 'track_width' must be set");
    rclcpp::shutdown();
  }

  double radius = get_parameter("wheel_radius").as_double();
  double track = get_parameter("track_width").as_double();

  robotState = turtlelib::DiffDrive(radius, track);

  cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10,
    std::bind(&turtle_control::cmdVelCallback, this, std::placeholders::_1));
  sensorDataSub = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
                "sensor_data", 10,
    std::bind(&turtle_control::sensorDataCallback, this, std::placeholders::_1));

  wheelCommander = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
  jointStatePub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void turtle_control::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if(msg->linear.y != 0.0) {
    RCLCPP_WARN(get_logger(), "Non-zero y component in cmd_vel ignored");
  }
  double v = msg->linear.x;
  double w = msg->angular.z;

  turtlelib::Twist2D body_vel(w, v, 0.0);
  turtlelib::Vector2D wheel_vels = robotState.inverseK(body_vel);

  auto wheelCmdMsg = nuturtlebot_msgs::msg::WheelCommands();
  wheelCmdMsg.left_velocity = wheel_vels.x / get_parameter("motor_cmd_per_rad_sec").as_double();
  wheelCmdMsg.right_velocity = wheel_vels.y / get_parameter("motor_cmd_per_rad_sec").as_double();

  wheelCommander->publish(wheelCmdMsg);
}

void turtle_control::sensorDataCallback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
{
  sensor_msgs::msg::JointState jointStateMsg;

  double ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();

  jointStateMsg.name = {"wheel_left_joint", "wheel_right_joint"};
  jointStateMsg.position = {(double)msg->left_encoder / ticks_per_rad,
    (double)msg->right_encoder / (double)ticks_per_rad};                                                                                //publishing 0 for now since I don't know the tick-radian conversion

  jointStateMsg.header.stamp = this->now();

  jointStatePub->publish(jointStateMsg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}
