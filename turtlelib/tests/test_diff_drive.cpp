#include <turtlelib/diff_drive.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

// ######### begin_citation[16] #########
TEST_CASE("DiffDrive forwardK with straight movement", "[forwardK]"){
    turtlelib::DiffDrive robot(0.05, 0.2); // wheel radius 0.05m, wheel base 0.2m

    double wheel_rotation = 1.0; // radians

    robot.forwardK(wheel_rotation, wheel_rotation);

    turtlelib::Transform2D pose = robot.get_pose();

    double expected_distance = 0.05 * wheel_rotation; // distance = radius * angle

    REQUIRE(pose.translation().x == Catch::Approx(expected_distance));
    REQUIRE(pose.translation().y == Catch::Approx(0.0));
    REQUIRE(pose.rotation() == Catch::Approx(0.0));
}

TEST_CASE("DiffDrive forwardK with in-place rotation", "[forwardK]"){
    turtlelib::DiffDrive robot(0.05, 0.2); // wheel radius 0.05m, wheel base 0.2m

    double wheel1_rotation = 1.0; // radians
    double wheel2_rotation = -1.0; // radians

    robot.forwardK(wheel1_rotation, wheel2_rotation);

    turtlelib::Transform2D pose = robot.get_pose();

    double expected_rotation = -(0.05 / 0.1); //1 radian of wheel about a .1 radius circle.

    REQUIRE(pose.translation().x == Catch::Approx(0.0));
    REQUIRE(pose.translation().y == Catch::Approx(0.0));
    REQUIRE(pose.rotation() == Catch::Approx(expected_rotation));
}

TEST_CASE("DiffDrive forwardK along an arc", "[forwardK]"){
    turtlelib::DiffDrive robot(0.05, 0.2); // wheel radius 0.05m, wheel base 0.2m

    double wheel1_rotation = 1.0; // radians
    double wheel2_rotation = 2.0; // radians

    robot.forwardK(wheel1_rotation, wheel2_rotation);

    turtlelib::Transform2D pose = robot.get_pose();

    double dtheta = (0.05 / 0.2) * (wheel2_rotation - wheel1_rotation);
    double dx = (0.05 / 2.0) * (wheel1_rotation + wheel2_rotation);
    double dy = 0.0;

    turtlelib::Twist2D body_twist(dtheta, dx, dy);
    turtlelib::Transform2D expected_pose = integrate_twist(body_twist);

    REQUIRE(pose.translation().x == Catch::Approx(expected_pose.translation().x));
    REQUIRE(pose.translation().y == Catch::Approx(expected_pose.translation().y));
    REQUIRE(pose.rotation() == Catch::Approx(expected_pose.rotation()));
}

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

    turtlelib::Twist2D vel = turtlelib::Twist2D(0.0, 0.0, 1.0); // non-zero y component

    REQUIRE_THROWS_AS(robot.inverseK(vel), std::logic_error);
}
// ######### end_citation[16] #########