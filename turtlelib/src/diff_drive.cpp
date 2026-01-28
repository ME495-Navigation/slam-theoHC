#include <turtlelib/diff_drive.hpp>
#include <stdexcept>

namespace turtlelib{

    turtlelib::DiffDrive::DiffDrive(double wheel_radius, double wheel_base)
        : wheel1_angle(0.0),
      wheel2_angle(0.0),
      pose(),
      wheel_radius(wheel_radius),
      wheel_base(wheel_base)  // Initializes to identity Transform2D
        {
    }


    // void turtlelib::DiffDrive::forwardK(double new_wheel1, double new_wheel2){
    // }


    Vector2D DiffDrive::inverseK(const Twist2D vel) const{
        if(vel.y != 0){
            throw std::logic_error("DiffDrive::inverseK: y component of velocity must be zero for differential drive robot.");
        }
        double theta1_dot = (vel.x - (wheel_base / 2.0) * vel.omega) / wheel_radius;
        double theta2_dot = (vel.x + (wheel_base / 2.0) * vel.omega) / wheel_radius;
        return Vector2D(theta1_dot, theta2_dot);
    }

    Transform2D DiffDrive::get_pose() const{
        return pose;
    }

    void DiffDrive::set_pose(const Transform2D &new_pose){
        pose = new_pose;
    }

}