#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class nusimulator : public rclcpp::Node
{
public:
  nusimulator()
  : Node("nusimulator"), count(0)
  {
    declare_parameter("rate", 10);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);

    declare_parameter("arena_x_length", 5.0);
    declare_parameter("arena_y_length", 5.0);

    declare_parameter("obstacle.x", std::vector<double>());
    declare_parameter("obstacle.y", std::vector<double>());
    declare_parameter("obstacle.r", 3.0);


    x = get_parameter("x0").as_double();
    y = get_parameter("y0").as_double();
    theta = get_parameter("theta0").as_double();

    tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    int timerrate = get_parameter("rate").as_int();

    std::chrono::milliseconds timer_period = std::chrono::milliseconds(timerrate);

    auto PeristentQoS = rclcpp::QoS(10).transient_local();

    timesteppub = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    wallpub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", PeristentQoS);
    obstaclepub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", PeristentQoS);
    simtick = this->create_wall_timer(timer_period, std::bind(&nusimulator::timer_callback, this));
    resetsrv = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&nusimulator::reset_callback, 
        this,std::placeholders::_1, std::placeholders::_2));
    
    publish_real_walls();
    
  }

private:
    rclcpp::TimerBase::SharedPtr simtick;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timesteppub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetsrv; 
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wallpub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstaclepub;

    double x;
    double y;
    double theta;

    int count;

    const float wall_thickness = .1;

    void timer_callback(){
        //Publish current timestep
        auto timemsg = std_msgs::msg::UInt64();
        timemsg.data = count++;
        this->timesteppub->publish(timemsg);

        //Publish transform
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "nusim/world";
        t.child_frame_id = "red/base_footprint";

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(t);
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response){
        //Reset count
        (void)request;
        (void)response;
        count = 0;

        //Reset robot position
        x = get_parameter("x0").as_double();
        y = get_parameter("y0").as_double();
        theta = get_parameter("theta0").as_double();
    }

    void publish_obstacle(){
        auto walls = visualization_msgs::msg::MarkerArray();
        
        std::vector<double> xspots = get_parameter("obstacle.x").as_double_array();
        std::vector<double> yspots = get_parameter("obstacle.x").as_double_array();
        double rad = get_parameter("obstacle.r").as_double();

        if(xspots.size() != yspots.size()){
            RCLCPP_ERROR(get_logger(),
                "obstacles.x and obstacles.y must be the same size");
            rclcpp::shutdown();
        }

        for(int i = 0; i < xspots.size(); i++){
            auto obst = visualization_msgs::msg::Marker();
            obst.header.stamp = this->get_clock()->now();
            obst.header.frame_id = "nusim/world";
            obst.id = i;
            obst.type = obst.CYLINDER;
            obst.action = 0;
        }
    }

    void publish_real_walls(){
        auto walls = visualization_msgs::msg::MarkerArray();

        walls.markers.emplace_back(genWallPair(false));
        walls.markers.emplace_back(genWallPair(true));

        wallpub->publish(walls);
    }

    visualization_msgs::msg::Marker genWallPair(bool side){
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
        wallpoint1.x = side ? 0 : wall_thickness/2 + ylen/2;
        wallpoint1.y = side ? wall_thickness/2 + ylen/2 : 0;
        auto wallpoint2 = geometry_msgs::msg::Point();
        wallpoint2.x = side ? 0 : -(wall_thickness/2 + ylen/2);
        wallpoint2.y = side ? -(wall_thickness/2 + ylen/2) : 0;
        walls.points.emplace_back(wallpoint1);
        walls.points.emplace_back(wallpoint2);

        return walls;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimulator>());
  rclcpp::shutdown();
  return 0;
}