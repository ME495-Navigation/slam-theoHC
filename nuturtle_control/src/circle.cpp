#include <nuturtle_control/circle.hpp>
#include <cmath>

circle::circle()
: rclcpp::Node("circle")
{
  // Frequency at which commands are published in Hz
  declare_parameter<int>("frequency", 100);

  // Publishes twists to cmd_vel corresponding to circular motion
  velPub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Toggles whether or not the robot should moves. Starts stationary
  startServ = this->create_service<std_srvs::srv::Empty>(
        "stop",
        std::bind(&circle::startCallback, this, std::placeholders::_1, std::placeholders::_2));
  // Reverses the direction of motion. Starts forward.
  reverseServ = this->create_service<std_srvs::srv::Empty>(
        "reverse",
        std::bind(&circle::reverseCallback, this, std::placeholders::_1, std::placeholders::_2));
  // Allows the user to change the radius and angular velocity of the circle.
  circleServ = this->create_service<nuturtle_control::srv::TurtCircle>(
        "circle",
        std::bind(&circle::circleCallback, this, std::placeholders::_1, std::placeholders::_2));
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
