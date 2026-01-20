#include <turtlelib/svg.hpp>
#include <turtlelib/se2d.hpp>

int main(){
    turtlelib::Svg svger = turtlelib::Svg();

    svger.addPoint(turtlelib::Point2D(3, 4));

    svger.write();
}