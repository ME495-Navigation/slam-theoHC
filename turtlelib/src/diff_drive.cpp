#include <turtlelib/diff_drive.hpp>
#include <stdexcept>

namespace turtlelib{

    turtlelib::DiffDrive::DiffDrive(double wheel_radius, double wheel_base)
    // ######### begin_citation[14] #########
        : wheel1_angle(0.0),
      wheel2_angle(0.0),
      pose(),
      wheel_radius(wheel_radius),
      wheel_base(wheel_base)  // Initializes to identity Transform2D
    // ######### end_citation[14] #########
        {
    }

    void turtlelib::DiffDrive::forwardK(double new_wheel1, double new_wheel2){
        double wheel1_diff = normalize_angle(new_wheel1 - wheel1_angle);
        double wheel2_diff = normalize_angle(new_wheel2 - wheel2_angle);

        // ######### begin_citation[15] #########
        double dtheta = (wheel_radius / wheel_base) * (wheel2_diff - wheel1_diff);
        double dx = (wheel_radius / 2.0) * (wheel1_diff + wheel2_diff);
        // ######### end_citation[15] #########
        double dy = 0.0;
        Twist2D body_twist(dtheta, dx, dy);
        twist = body_twist;
        Transform2D delta_transform = integrate_twist(body_twist);
        pose *= delta_transform;

        wheel1_angle = new_wheel1;
        wheel2_angle = new_wheel2;
    }

    Vector2D DiffDrive::inverseK(const Twist2D vel) const{
        if(vel.y != 0){
            throw std::logic_error("DiffDrive::inverseK: y component of velocity must be zero for differential drive robot.");
        }
        double theta1_dot = (vel.x - (wheel_base / 2.0) * vel.omega) / wheel_radius;
        double theta2_dot = (vel.x + (wheel_base / 2.0) * vel.omega) / wheel_radius;
        return Vector2D(theta1_dot, theta2_dot); // Vector2D is redundant here
    }

    Transform2D DiffDrive::get_pose() const{
        return pose;
    }

    void DiffDrive::set_pose(const Transform2D &new_pose){
        pose = new_pose;
    }

    void turtlelib::DiffDrive::set_wheels(double wheel1, double wheel2){
        wheel1_angle = wheel1;
        wheel2_angle = wheel2;
    }
}
