#include <nusim/nusim.hpp>
#include <algorithm>
#include <random>
#include "std_msgs/msg/color_rgba.hpp"

nusimulator::nusimulator()
: Node("nusimulator"), robotState(0.0, 0.0), left_wheel_vel(0.0), right_wheel_vel(0.0), reported_wheel_positions(0.0, 0.0), count(0)
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
  declare_parameter<double>("collision_radius", 0.1);

  declare_parameter("max_range", 5.0f);

  declare_parameter("input_noise", 0.0f);
  declare_parameter("slip_fraction", 0.0f);

  declare_parameter("basic_sensor_variance", 0.0f);

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

      //Publishers for visualization of walls and obstacles
  wallpub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls",
        PeristentQoS);
  obstaclepub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles",
        PeristentQoS);
  
  fake_obstaclepub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor",
        PeristentQoS);

      // Publishes the wheel position of the simulated robot
  sensordatapub = this->create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
      // Publishes the joint states of the simulated robot
  jointpub = this->create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

        //SUBSCRIBERS
        // Subscriber to wheel commands for simulated robot
  wheelcmdsub = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>("red/wheel_cmd",
        10, std::bind(&nusimulator::wheelcmd_callback, this, std::placeholders::_1));

        //TIMERS
  simtick = this->create_wall_timer(timer_period, std::bind(&nusimulator::sim_tick_callback, this));
  
  fake_sensor_tick = this->create_wall_timer(200ms, std::bind(&nusimulator::collision_check, this));

        //SERVICES
        // Resets the simulation
  resetsrv = this->create_service<std_srvs::srv::Empty>("~/reset",
        std::bind(&nusimulator::reset_callback,
                this, std::placeholders::_1, std::placeholders::_2));
  publish_real_walls();
  publish_obstacle();
}

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

void nusimulator::sim_tick_callback()
{
    //Publish current timestep
  auto timemsg = std_msgs::msg::UInt64();
  timemsg.data = count++;
  this->timesteppub->publish(timemsg);

    //Update robot state
  double dt = (double) get_parameter("rate").as_int() / 1000.0;

  double slip_range = get_parameter("slip_fraction").as_double();
  std::uniform_real_distribution d(-slip_range, slip_range);
  turtlelib::Vector2D slip = {d(get_random()), d(get_random())};

  turtlelib::Vector2D clean_wheel_vels = {left_wheel_vel * dt, right_wheel_vel * dt};
  turtlelib::Vector2D noise_wheel_vels = {left_wheel_vel * dt * (1 + slip.x), right_wheel_vel * dt * (1 + slip.y)};

  turtlelib::Vector2D wheel_positions = robotState.get_wheels();
  reported_wheel_positions = turtlelib::normalize_angle(reported_wheel_positions + clean_wheel_vels);
  
  turtlelib::Vector2D update_wheel_positions = turtlelib::normalize_angle(wheel_positions + noise_wheel_vels);; //wheel_positions + wheel_vels * (turtlelib::Vector2D(1, 1) + slip);

  robotState.forwardK(update_wheel_positions);

    //Publish simulated sensor data
  double encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
  auto sensormsg = nuturtlebot_msgs::msg::SensorData();
  sensormsg.stamp = this->get_clock()->now();

  sensormsg.left_encoder = static_cast<int>((reported_wheel_positions.x) * encoder_ticks_per_rad);
  sensormsg.right_encoder = static_cast<int>((reported_wheel_positions.y) * encoder_ticks_per_rad);
  sensordatapub->publish(sensormsg);

    //Publish joint states for red robot
  auto jointmsg = sensor_msgs::msg::JointState();
  jointmsg.header.stamp = this->get_clock()->now();
  jointmsg.name.push_back("wheel_left_joint");
  jointmsg.name.push_back("wheel_right_joint");
  jointmsg.position.push_back(reported_wheel_positions.x);
  jointmsg.position.push_back(reported_wheel_positions.y);
  jointpub->publish(jointmsg);

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

void nusimulator::collision_check()
{
  double robot_radius = get_parameter("collision_radius").as_double();
  double obstacle_radius = get_parameter("obstacles.r").as_double();
  double collision_radius = robot_radius + obstacle_radius;

  std::vector<double> xspots = get_parameter("obstacles.x").as_double_array();
  std::vector<double> yspots = get_parameter("obstacles.y").as_double_array();

  turtlelib::Vector2D robotPos = robotState.get_pose().translation();

  for(size_t i = 0; i < xspots.size(); i++) {
    turtlelib::Vector2D offset = robotPos - turtlelib::Vector2D(xspots.at(i), yspots.at(i));
    double dist = turtlelib::magnitude(offset);
    if(dist < collision_radius) {
      turtlelib::Vector2D correction = turtlelib::normalize(offset) * (collision_radius);
      robotState.set_pose({robotPos + correction, robotState.get_pose().rotation()});
    }
  }
}

void nusimulator::wheelcmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
{
  int max_motor_cmd = (int) get_parameter("motor_cmd_max").as_double();
  int left_cmd = std::clamp(msg->left_velocity, -max_motor_cmd, max_motor_cmd);
  int right_cmd = std::clamp(msg->right_velocity, -max_motor_cmd, max_motor_cmd);

  double input_variance = get_parameter("input_noise").as_double();

  std::normal_distribution<> d(0, input_variance);
  double noise_L = d(get_random());
  double noise_R = d(get_random());
  left_wheel_vel = left_cmd * get_parameter("motor_cmd_per_rad_sec").as_double() + noise_L;
  right_wheel_vel = right_cmd * get_parameter("motor_cmd_per_rad_sec").as_double() + noise_R;
}

void nusimulator::fake_sensor_tick_callback()
{
  std::vector<double> xspots = get_parameter("obstacles.x").as_double_array();
  std::vector<double> yspots = get_parameter("obstacles.y").as_double_array();

  turtlelib::Vector2D robotPos = robotState.get_pose().translation();

  auto obsts = visualization_msgs::msg::MarkerArray();

  for(size_t i = 0; i < xspots.size(); i++) {
    turtlelib::Vector2D offset = turtlelib::Vector2D(xspots.at(i), yspots.at(i)) - robotPos;
    double dist = turtlelib::magnitude(offset);

    auto color = std_msgs::msg::ColorRGBA();
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0;

    int action = dist < get_parameter("max_range").as_double() ? 0 : 2; // add/modify if in range, delete if out of range

    std::normal_distribution<double> d(0, get_parameter("basic_sensor_variance").as_double());
    turtlelib::Vector2D noise = {d(get_random()), d(get_random())};

    offset += noise;

    auto obst = create_obstacle_marker(offset.x, offset.y, color, action, "red/base_footprint", "fake_sensor");
    obst.id = i;
    obsts.markers.push_back(obst);
  }
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

// ##### begin_citation [23] #####
visualization_msgs::msg::Marker nusimulator::create_obstacle_marker(double x, double y,
  std_msgs::msg::ColorRGBA color, int action, std::string frame_id, std::string ns)
{
  double rad = get_parameter("obstacles.r").as_double();
  auto obst = visualization_msgs::msg::Marker();
  obst.header.stamp = this->get_clock()->now();
  obst.header.frame_id = frame_id;
  obst.type = obst.CYLINDER;
  obst.action = action;
  obst.color = color;
  obst.scale.x = 2 * rad;
  obst.scale.y = 2 * rad;
  obst.scale.z = .25;
  obst.pose.position.x = x;
  obst.pose.position.y = y;
  obst.ns = ns;
  return obst;
}
// ##### end_citation [23] #####

void nusimulator::publish_obstacle()
{
  auto obsts = visualization_msgs::msg::MarkerArray();

  std::vector<double> xspots = get_parameter("obstacles.x").as_double_array();
  std::vector<double> yspots = get_parameter("obstacles.y").as_double_array();

  if(xspots.size() != yspots.size()) {
    RCLCPP_ERROR(get_logger(),
                                "obstacles.x and obstacles.y must be the same size");
    rclcpp::shutdown();
  }

  for(size_t i = 0; i < xspots.size(); i++) {
    auto color = std_msgs::msg::ColorRGBA();
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;

    auto obst = create_obstacle_marker(xspots.at(i), yspots.at(i), color, 0, "nusim/world");
    obst.id = i;
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
