#include <turtlelib/svg.hpp>
#include <turtlelib/se2d.hpp>

int main(){
    turtlelib::Svg svger = turtlelib::Svg();

    svger.addPoint(turtlelib::Point2D(0, 1), "purple", 0);

    svger.addFrame(turtlelib::Transform2D(turtlelib::Vector2D(2, 2), turtlelib::deg2rad(45)), "b");

    svger.addPoint(turtlelib::Point2D(0, 2), "purple", 1);

    svger.write();
}