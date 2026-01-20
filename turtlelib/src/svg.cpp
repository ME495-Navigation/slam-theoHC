#include <turtlelib/svg.hpp>
#include <iostream>

turtlelib::Svg::Svg(){
    addFrame(Transform2D(), "a");
}

void turtlelib::Svg::write()
{

    std::cout << std::format("<svg width=\"{}in\" height=\"{}in\" viewBox=\"0 0 {} {}\" xmlns=\"http://www.w3.org/2000/svg\">\n",
        xsize, ysize, xsize * dpi, ysize * dpi);

    std::cout << 
        "<defs>\n"
            "<marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n"
                "<path transform=\"scale(0.2) translate(6,0)\" \n"
                "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" \n"
                "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\n"
            "</marker>\n"
        "</defs>\n";

    //Draw points
    for(dispPoint point : points){
        drawPoint(point);
    }

    for(dispFrame frame : frames){
        drawFrame(frame);
    }

    std::cout << "</svg>";
}

void turtlelib::Svg::addPoint(Point2D newpoint, std::string color, unsigned int frameID){
    if(frameID >= frames.size()) {
        return;
    }

    dispPoint point = dispPoint();
    
    point.point = newpoint;
    point.color = color;
    point.frameID = frameID;

    points.push_back(point);
}



void turtlelib::Svg::addFrame(Transform2D frame, std::string name){
    dispFrame newframe = dispFrame();
    newframe.transform = frame;
    newframe.name = name;

    frames.push_back(newframe);
}

void turtlelib::Svg::drawFrame(dispFrame frame){
    Transform2D drawtrans = CenterOfPage * frame.transform;

    Point2D origin = drawtrans(Point2D(0,0));
    Point2D xhead = drawtrans(Point2D(1,0));
    Point2D yhead = drawtrans(Point2D(0,-1));

    std::cout << "<g>";

    std::cout << std::format(
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"red\" stroke-width=\"2\" marker-start=\"url(#Arrow1Sstart)\"/>",
        xhead.x * dpi, xhead.y * dpi, origin.x * dpi, origin.y * dpi);

    std::cout << std::format(
        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"green\" stroke-width=\"2\" marker-start=\"url(#Arrow1Sstart)\"/>",
        yhead.x * dpi, yhead.y * dpi, origin.x * dpi, origin.y * dpi);
    
    std::cout << std::format(
        "<text x=\"{}\" y=\"{}\" fill=\"black\" font-size=\"12\">{{{}}}</text>",
        origin.x * dpi + 5, origin.y * dpi - 5, frame.name);

    std::cout << "</g>";
}

void turtlelib::Svg::drawPoint(dispPoint point){
    point.point.y *= -1;
    
    Transform2D drawtrans = CenterOfPage * frames.at(point.frameID).transform;

    Point2D cpoint = drawtrans(point.point);

    std::cout << std::format("<circle cx=\"{}\" cy=\"{}\" r=\"3\" stroke=\"{}\" fill=\"{}\" stroke-width=\"1\"/>", 
        cpoint.x * dpi, cpoint.y * dpi, point.color, point.color);
}
