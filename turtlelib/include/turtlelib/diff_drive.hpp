#pragma once
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/se2d.hpp>

/// \file
/// \brief Differential Drive Kinematics

namespace turtlelib{

    class DiffDrive{
    public:
        
    private:
        Transform2D pose;
        double wheel_radius;
        double wheel_base;

        double wheel1_angle;
        double wheel2_angle;
    };
}