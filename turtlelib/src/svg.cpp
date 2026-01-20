#include <turtlelib/svg.hpp>
#include <iostream>

void turtlelib::Svg::write(){

    std::cout << std::format("<svg width=\"{}in\" height=\"{}in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">", xsize, ysize);

    //Draw points
    for(Point2D point : points){
        std::cout << std::format("<circle cx=\"{}\" cy=\"{}\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\"/>", point.x, point.y);
    }

    std::cout << "</svg>";
}

void turtlelib::Svg::addPoint(Point2D newpoint){
    points.push_back(newpoint);
}