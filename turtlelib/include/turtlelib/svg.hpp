#pragma once
#include <turtlelib/se2d.hpp>
#include <vector>
#include <string>

/// \file
/// \brief Helper for visualizing geometry as a .svg

namespace turtlelib{

    struct dispPoint{
        Point2D point;
        std::string color;
        int frameID;
    };

    struct dispFrame{
        Transform2D transform;
        std::string name;
    };

    /// \brief A class for constructing an svg-based visualization of turtlelib geometry functions.
    class Svg{
        public:
        /// \brief Default constructor which adds a frame at the origin.
        Svg();

        float xsize = 8.5;
        float ysize = 11;
        float dpi = 96;

        /// \brief Writes all stored geometry as an svg to cout
        void write();

        /// \brief Add a Point2D object to the geometric representation
        /// \param newpoint the point to be added
        /// \param color the name of the color of the point
        /// \param frameID the index of the frame in which this point is represented (e.g. the first frame added)
        void addPoint(Point2D newpoint, std::string color, unsigned int frameID);

        /// @brief Add a frame object to the geometric representation
        /// @param transform The transform representing the frame, relative to the origin
        /// @param name The display name of the frame
        void addFrame(Transform2D transform, std::string name);

        private:
        std::vector<dispPoint> points;

        std::vector<dispFrame> frames;

        Transform2D CenterOfPage = Transform2D(.5 * Vector2D(xsize, ysize));

        void drawFrame(dispFrame frame);

        void drawPoint(dispPoint point);
    };
}