#include <nuturtle_control/turtle_control.hpp>

turtle_control::turtle_control(): Node("turtle_control"){
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}
