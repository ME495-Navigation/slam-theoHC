#include <turtlelib/diff_drive.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

TEST_CASE("DiffDrive inverseK with pure linear twist", "[inverseK]"){
    turtlelib::DiffDrive robot(0.05, 0.2); // wheel radius 0.05m, wheel base 0.2m

    turtlelib::Twist2D vel = {0.0, 1.0, 0.0}; // pure linear velocity along x-axis

    turtlelib::Vector2D wheel_vel = robot.inverseK(vel);

    double expected_wheel_vel = vel.x / 0.05; // v = r * omega => omega = v / r

    REQUIRE(wheel_vel.x == Catch::Approx(expected_wheel_vel));
    REQUIRE(wheel_vel.y == Catch::Approx(expected_wheel_vel));
}

TEST_CASE("DiffDrive inverseK with pure rotational twist", "[inverseK]"){
    turtlelib::DiffDrive robot(0.05, 0.2); // wheel radius 0.05m, wheel base 0.2m

    turtlelib::Twist2D vel = turtlelib::Twist2D(turtlelib::deg2rad(90.0), 0.0, 0.0); // pure rotational velocity

    turtlelib::Vector2D wheel_vel = robot.inverseK(vel);

    double expected_wheel1_vel = -(0.2 / 2.0 * vel.omega) / 0.05;
    double expected_wheel2_vel = (0.2 / 2.0 * vel.omega) / 0.05;

    REQUIRE(wheel_vel.x == Catch::Approx(expected_wheel1_vel));
    REQUIRE(wheel_vel.y == Catch::Approx(expected_wheel2_vel));
}

TEST_CASE("DiffDrive inverseK with valid twist about a circle", "[inverseK]"){
    turtlelib::DiffDrive robot(0.05, 0.2); // wheel radius 0.05m, wheel base 0.2m

    turtlelib::Twist2D vel = turtlelib::Twist2D(turtlelib::deg2rad(30.0), 0.1, 0); // linear velocity 0.1 m/s, angular velocity 30 deg/s

    turtlelib::Vector2D wheel_vel = robot.inverseK(vel);

    double expected_wheel1_vel = (vel.x - (0.2 / 2.0) * vel.omega) / 0.05;
    double expected_wheel2_vel = (vel.x + (0.2 / 2.0) * vel.omega) / 0.05;

    REQUIRE(wheel_vel.x == Catch::Approx(expected_wheel1_vel));
    REQUIRE(wheel_vel.y == Catch::Approx(expected_wheel2_vel));
}

TEST_CASE("DiffDrive inverseK with invalid twist (non-zero y)", "[inverseK]"){
    turtlelib::DiffDrive robot(0.05, 0.2); // wheel radius 0.05m, wheel base 0.2m

    turtlelib::Twist2D vel = turtlelib::Twist2D(0.0, 1.0, 0.0); // non-zero y component

    REQUIRE_THROWS_AS(robot.inverseK(vel), std::logic_error);
}