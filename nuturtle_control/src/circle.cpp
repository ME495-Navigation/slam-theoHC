#include <nuturtle_control/circle.hpp>
#include <cmath>

circle::circle()
: rclcpp::Node("circle")
{
  declare_parameter<int>("frequency", 100);

  velPub = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);
  startServ = this->create_service<std_srvs::srv::Empty>(
        "~/start_circle",
        std::bind(&circle::startCallback, this, std::placeholders::_1, std::placeholders::_2));
  reverseServ = this->create_service<std_srvs::srv::Empty>(
        "~/reverse_circle",
        std::bind(&circle::reverseCallback, this, std::placeholders::_1, std::placeholders::_2));

  controlTimer = this->create_timer(
        std::chrono::milliseconds(1000 / get_parameter("frequency").as_int()),
        std::bind(&circle::controlCallback, this));
}

void circle::controlCallback()
{
  if(moving) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = angularVel * radius * (forward ? 1 : -1);
    msg.angular.z = angularVel * (clockwise ? -1 : 1);
    velPub->publish(msg);
  } else {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velPub->publish(msg);
  }
}

void circle::reverseCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  clockwise = !clockwise;
  forward = !forward;
}

void circle::startCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  moving = !moving;
}

void circle::circleCallback(
  const std::shared_ptr<nuturtle_control::srv::TurtCircle::Request> request,
  std::shared_ptr<nuturtle_control::srv::TurtCircle::Response>)
{
  clockwise = request->angular_velocity < 0;
  radius = request->radius;
  angularVel = request->angular_velocity;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<circle>());
  rclcpp::shutdown();
  return 0;
}
