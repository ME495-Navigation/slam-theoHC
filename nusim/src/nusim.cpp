#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class nusimulator : public rclcpp::Node
{
public:
  nusimulator()
  : Node("nusimulator"), count_(0), count(0)
  {
    declare_parameter("rate", 10);

    int timerrate = get_parameter("rate").as_int();

    std::chrono::milliseconds timer_period = std::chrono::milliseconds(timerrate);

    timesteppub = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    simtick = this->create_wall_timer(timer_period, std::bind(&nusimulator::timer_callback, this));
    resetsrv = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&nusimulator::reset_callback, 
        this,std::placeholders::_1, std::placeholders::_2));
  }

private:
    rclcpp::TimerBase::SharedPtr simtick;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timesteppub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetsrv; 
    size_t count_;

    int count;

    void timer_callback(){
        auto timemsg = std_msgs::msg::UInt64();
        timemsg.data = count++;
        this->timesteppub->publish(timemsg);
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response){
        (void)request;
        (void)response;
        count = 0;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimulator>());
  rclcpp::shutdown();
  return 0;
}