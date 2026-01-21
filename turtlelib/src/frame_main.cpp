#include <turtlelib/svg.hpp>
#include <turtlelib/se2d.hpp>
#include <iostream>

int main(){
    std::cout << "Enter two transforms in the format {theta, x, y}\n";

    turtlelib::Transform2D tab, tbc;
    std::cin >> tab >> tbc;
    std::cout << "after tab: fail=" << std::cin.fail() << "\n";

    std::cout << std::format("Transform A to B: {}",  tab) << "\n";
    std::cout << std::format("Transform B to A: {}",  tab.inv()) << "\n";
    std::cout << std::format("Transform B to C: {}",  tbc) << "\n";
    std::cout << std::format("Transform C to B: {}",  tbc.inv()) << "\n";

    std::cout << "Enter a point in frame A (format: (x, y) or x y): \n";
    turtlelib::Point2D pa;
    std::cin >> pa;

    turtlelib::Point2D pb = tab.inv()(pa);
    turtlelib::Point2D pc = tbc.inv()(pb);

    std::cout << std::format("Point in frame A: {}", pa) << "\n";
    std::cout << std::format("Point in frame B: {}", pb) << "\n";
    std::cout << std::format("Point in frame C: {}", pc) << "\n";

    std::cout << "Enter a vector in frame B (format: [x, y] or x y): \n";
    turtlelib::Vector2D vb;
    std::cin >> vb;

    std::cout << std::format("Vector in frame B: {}", vb) << "\n";
    turtlelib::Vector2D vbhat = turtlelib::normalize(vb);
    std::cout << std::format("Normalized vector in frame B: {}", vbhat) << "\n";
    turtlelib::Vector2D va = tab(vb);
    turtlelib::Vector2D vc = tbc.inv()(vb);
    std::cout << std::format("Vector in frame A: {}", va) << "\n";
    std::cout << std::format("Vector in frame C: {}", vc) << "\n";

    turtlelib::Svg svgvis;
    svgvis.addFrame(tab, "b");
    svgvis.addFrame(tbc, "c", 1);

    svgvis.addPoint(pa, "purple", 0);
    svgvis.addPoint(pb, "brown", 1);
    svgvis.addPoint(pc, "orange", 2);

    svgvis.addVector(va, "purple", 0);
    svgvis.addVector(vb, "black", 1);
    svgvis.addVector(vbhat, "brown", 1);
    svgvis.addVector(vc, "orange", 2);

    svgvis.write("exercises/B6_frames.svg");
}