#include <turtlelib/angle.hpp>
#include <iostream>
#include <cstdio>
#include <string>

// ####### begin_citation [3] #######
void reset_cin () {    
    // clear all error status bits
    std::cin.clear(); 
    // clear input buffer
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}
// ####### end_citation [3] #######

int main(){
    while(true){
        std::cout << "Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)\n";

        float angle;
        std::cin >> angle;
        if(std::cin.fail()){
            if(std::cin.peek() == EOF) break;
            reset_cin();
            std::cout << "UP Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n";
            continue;
        }

        std::string unit;
        std::cin >> unit;

        float otherAngle;
        std::string otherUnit;

        if(unit == "deg"){
            otherUnit = "rad";
            otherAngle = turtlelib::normalize_angle(turtlelib::deg2rad(angle));
        }
        else if(unit == "rad"){
            otherUnit = "deg";
            otherAngle = turtlelib::rad2deg(turtlelib::normalize_angle(angle));
        }
        else{
            std::cout << "DOWN Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n";
            continue;
        }

        std::printf("%f %s is %f %s\n", angle, unit.c_str(), otherAngle, otherUnit.c_str());

    }
    return 0;
}