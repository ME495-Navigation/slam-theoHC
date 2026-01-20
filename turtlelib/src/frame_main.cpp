#include <turtlelib/svg.hpp>
#include <turtlelib/se2d.hpp>

int main(){
    turtlelib::Svg svger = turtlelib::Svg();

    svger.addPoint(turtlelib::Point2D(0, 0), "purple", 0);

    svger.write();
}