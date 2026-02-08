#include <nusim/nusim.hpp>
#include <algorithm>
#include <numbers>

nusimulator::nusimulator()
: Node("nusimulator"), count(0)
{
  declare_parameter("rate", 10);
  declare_parameter("x0", 0.0);
  declare_parameter("y0", 0.0);
  declare_parameter("theta0", 0.0);

  declare_parameter("arena_x_length", 5.0);
  declare_parameter("arena_y_length", 5.0);

  declare_parameter("obstacles.x", std::vector<double>());
  declare_parameter("obstacles.y", std::vector<double>());
  declare_parameter("obstacles.r", 3.0);

  declare_parameter<double>("wheel_radius");
  declare_parameter<double>("track_width");
  declare_parameter<double>("encoder_ticks_per_rad");
  declare_parameter<double>("motor_cmd_max");
  declare_parameter<double>("motor_cmd_per_rad_sec");

  double x = get_parameter("x0").as_double();
  double y = get_parameter("y0").as_double();
  double theta = get_parameter("theta0").as_double();

  robotState = turtlelib::DiffDrive(get_parameter("wheel_radius").as_double(),
                                  get_parameter("track_width").as_double());

  robotState.set_pose({theta, x, y});

  tf_broadcaster =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  int timerrate = get_parameter("rate").as_int();

  std::chrono::milliseconds timer_period = std::chrono::milliseconds(timerrate);

  auto PeristentQoS = rclcpp::QoS(10).transient_local();

        //PUBLISHERS
  timesteppub = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

      //Publishers for visualization
  wallpub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls",
        PeristentQoS);
  obstaclepub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles",
        PeristentQoS);
      //Publishers for robot state
  sensordatapub = this->create_publisher<nuturtlebot_msgs::msg::SensorData>("~/sensor_data", 10);

        //SUBSCRIBERS
  wheelcmdsub = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("~/wheel_commands",
        10, std::bind(&nusimulator::wheelcmd_callback, this, std::placeholders::_1));

        //TIMERS
  simtick = this->create_wall_timer(timer_period, std::bind(&nusimulator::timer_callback, this));

        //SERVICES
  resetsrv = this->create_service<std_srvs::srv::Empty>("~/reset",
        std::bind(&nusimulator::reset_callback,
                this, std::placeholders::_1, std::placeholders::_2));

  publish_real_walls();
  publish_obstacle();
}

void nusimulator::timer_callback()
{
    //Publish current timestep
  auto timemsg = std_msgs::msg::UInt64();
  timemsg.data = count++;
  this->timesteppub->publish(timemsg);

    //Update robot state
  double dt = 1000.0 / get_parameter("rate").as_double();

  turtlelib::Vector2D wheel_positions = robotState.get_wheels();
  wheel_positions.x += turtlelib::normalize_angle(left_wheel_vel * dt);
  wheel_positions.y += turtlelib::normalize_angle(right_wheel_vel * dt);

  robotState.forwardK(wheel_positions);

  double encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
  auto sensormsg = nuturtlebot_msgs::msg::SensorData();
  sensormsg.stamp = this->get_clock()->now();

  sensormsg.left_encoder = static_cast<int>((std::numbers::pi + wheel_positions.x) * encoder_ticks_per_rad);
  sensormsg.right_encoder = static_cast<int>((std::numbers::pi + wheel_positions.y) * encoder_ticks_per_rad);
  sensordatapub->publish(sensormsg);

    //Publish transform
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "nusim/world";
  t.child_frame_id = "red/base_footprint";

  turtlelib::Transform2D robotPose = robotState.get_pose();
  double x = robotPose.translation().x;
  double y = robotPose.translation().y;
  double theta = robotPose.rotation();
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  t.transform.translation.x = x;
  t.transform.translation.y = y;

  tf_broadcaster->sendTransform(t);
}

void nusimulator::wheelcmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
{
  int max_motor_cmd = get_parameter("motor_cmd_max").as_int();
  int left_cmd = std::clamp(msg->left_velocity, -max_motor_cmd, max_motor_cmd);
  int right_cmd = std::clamp(msg->right_velocity, -max_motor_cmd, max_motor_cmd);
  
  left_wheel_vel = left_cmd * get_parameter("motor_cmd_per_rad_sec").as_double();
  right_wheel_vel = right_cmd * get_parameter("motor_cmd_per_rad_sec").as_double();
}

void nusimulator::reset_callback(
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
        //Reset count
  count = 0;

        //Reset robot position
  double x = get_parameter("x0").as_double();
  double y = get_parameter("y0").as_double();
  double theta = get_parameter("theta0").as_double();

  robotState.set_pose({theta, x, y});
}

void nusimulator::publish_obstacle()
{
  auto obsts = visualization_msgs::msg::MarkerArray();

  std::vector<double> xspots = get_parameter("obstacles.x").as_double_array();
  std::vector<double> yspots = get_parameter("obstacles.y").as_double_array();
  double rad = get_parameter("obstacles.r").as_double();

  if(xspots.size() != yspots.size()) {
    RCLCPP_ERROR(get_logger(),
                                "obstacles.x and obstacles.y must be the same size");
    rclcpp::shutdown();
  }

  for(size_t i = 0; i < xspots.size(); i++) {
                //setup
    auto obst = visualization_msgs::msg::Marker();
    obst.header.stamp = this->get_clock()->now();
    obst.header.frame_id = "nusim/world";
    obst.id = i;
    obst.type = obst.CYLINDER;
    obst.action = 0;
                //color
    obst.color.r = 1;
    obst.color.b = 0;
    obst.color.g = 0;
    obst.color.a = 1;
                //scale
    obst.scale.x = 2 * rad;
    obst.scale.y = 2 * rad;
    obst.scale.z = .25;
                //position
    obst.pose.position.x = xspots.at(i);
    obst.pose.position.y = yspots.at(i);
                //add to array
    obsts.markers.push_back(obst);
  }

  obstaclepub->publish(obsts);
}

void nusimulator::publish_real_walls()
{
  auto walls = visualization_msgs::msg::MarkerArray();

  walls.markers.push_back(genWallPair(false));
  walls.markers.push_back(genWallPair(true));

  wallpub->publish(walls);
}

visualization_msgs::msg::Marker nusimulator::genWallPair(bool side)
{
  double xlen = get_parameter("arena_x_length").as_double();
  double ylen = get_parameter("arena_y_length").as_double();

                //wall setup [horizontal ones]
  auto walls = visualization_msgs::msg::Marker();
  walls.header.stamp = this->get_clock()->now();
  walls.header.frame_id = "nusim/world";
  walls.id = side ? 0 : 1;
  walls.type = walls.CUBE_LIST;
  walls.action = 0;
                //color
  walls.color.r = 1;
  walls.color.b = 0;
  walls.color.g = 0;
  walls.color.a = 1;
                //scale
  walls.scale.x = side ? xlen + 2 * wall_thickness : wall_thickness;
  walls.scale.y = side ? wall_thickness : ylen;
  walls.scale.z = .25;
                //position
  auto wallpoint1 = geometry_msgs::msg::Point();
  wallpoint1.x = side ? 0 : wall_thickness / 2 + ylen / 2;
  wallpoint1.y = side ? wall_thickness / 2 + ylen / 2 : 0;
  auto wallpoint2 = geometry_msgs::msg::Point();
  wallpoint2.x = side ? 0 : -(wall_thickness / 2 + ylen / 2);
  wallpoint2.y = side ? -(wall_thickness / 2 + ylen / 2) : 0;
  walls.points.emplace_back(wallpoint1);
  walls.points.emplace_back(wallpoint2);

  return walls;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimulator>());
  rclcpp::shutdown();
  return 0;
}
