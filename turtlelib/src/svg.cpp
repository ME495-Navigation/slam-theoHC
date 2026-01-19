#include <turtlelib/svg.hpp>
#include <iostream>

void turtlelib::Svg::write(){
    std::cout << std::format("<svg width=\"{}in\" height=\"{}in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">", xsize, ysize);

    
}

void turtlelib::Svg::addPoint(Point2D newpoint){
    points.push_back(newpoint);
}