# turtlelib

### Author: Theo Coulson

This package contains several c++ headers for managing a differential drive robot.

- `angle.hpp` contains degree/radian conversion and angle normalization, as well as a constexpr utility for approxmiate float equality.

- `geometry2d.hpp` contains definitions and operators for the `Point2D` and `Vector2D` structs. Formatters and stream i/o are supported.

- `se2d.hpp` contains definitions for the `Twist2D` struct and `Transform2D` class. Stream input and formatters are provided for both, as are common arithmetic operations.

- `svg.hpp` contains the `svg` class which enables the display of turtlelib point, vector, and transform structures using the svg format.

- `diff_drive.hpp` contains the `diff_drive` clas, which provides fk and ik functionality for a differential drive robot.