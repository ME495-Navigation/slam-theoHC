#pragma once
#include <turtlelib/se2d.hpp>
#include <vector>

/// \file
/// \brief Helper for visualizing geometry as a .svg

namespace turtlelib{
    /// \brief A 
    class Svg{
        public:
        float xsize = 8.5;
        float ysize = 11;

        void write();

        void addPoint(Point2D newpoint);

        private:
        std::vector<Point2D> points;
    };
}