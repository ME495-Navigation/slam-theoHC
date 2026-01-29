#include <nuturtle_control/turtle_control.hpp>

turtle_control::turtle_control(): Node("turtle_control"){
	double radius = get_parameter("wheel_radius").as_double();
	double track = get_parameter("track_width").as_double();

	robotState = turtlelib::DiffDrive(radius, track);
}

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<turtle_control>());
	rclcpp::shutdown();
	return 0;
}
