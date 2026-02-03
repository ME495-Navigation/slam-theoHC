#pragma once
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/se2d.hpp>

/// \file
/// \brief Differential Drive Kinematics

namespace turtlelib{

    class DiffDrive{
    public:
        /// \brief Default constructor for DiffDrive
        DiffDrive() = default;  

        /// \brief Construct a DiffDrive object
        /// \param wheel_radius - radius of the wheels
        /// \param wheel_base - distance between the wheels
        DiffDrive(double wheel_radius, double wheel_base);

        /// \brief Update the robot's pose based on wheel movements
        /// \param new_wheel1 - new angle of wheel 1 (radians)
        /// \param new_wheel2 - new angle of wheel 2 (radians)
        void forwardK(double new_wheel1, double new_wheel2);

        /// \brief Compute wheel velocities from a given body twist
        /// \param vel - the desired body twist
        /// \return a Vector2D where x is wheel 1 velocity and y is wheel 2 velocity (in rad/s)
        Vector2D inverseK(const Twist2D vel) const;

        /// \brief Get the current pose of the robot
        /// \return the current Transform2D representing the robot's pose
        Transform2D get_pose() const;

        /// \brief Set the robot's pose
        /// \param new_pose - the new Transform2D to set as the robot's pose. Does not update wheel state.
        void set_pose(const Transform2D & new_pose);

        /// \brief Set the wheel angles
        /// \param wheel1 - angle of wheel 1 (radians)
        /// \param wheel2 - angle of wheel 2 (radians)
        void set_wheels(double wheel1, double wheel2);

    private:
        double wheel1_angle;
        double wheel2_angle;

        Transform2D pose;
        double wheel_radius;
        double wheel_base;
    };
}