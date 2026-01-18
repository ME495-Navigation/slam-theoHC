#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <turtlelib/se2d.hpp>
#include <turtlelib/angle.hpp>

TEST_CASE("Twist2D input with brackets", "[twist]") // Gregory, Aiosa
{
    std::stringstream ss;
    ss.str("<90 deg, 1.0, 2.0>"); // Test degrees input

    turtlelib::Twist2D tw;

    ss >> tw;

    REQUIRE(tw.omega == Catch::Approx(turtlelib::deg2rad(90.0)));
    REQUIRE(tw.x == Catch::Approx(1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));

    ss.clear();
    ss.str("<1.57 rad, 3.0, -4.0>"); // Test radians input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(1.57));
    REQUIRE(tw.x == Catch::Approx(3.0));
    REQUIRE(tw.y == Catch::Approx(-4.0));

    ss.clear();
    ss.str("<0.5, -1.0, 2.0>"); // Test no unit input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(0.5));
    REQUIRE(tw.x == Catch::Approx(-1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));
}

TEST_CASE("Twist2D input without brackets", "[twist]") // Gregory, Aiosa
{
    std::stringstream ss;
    ss.str("90 deg 1.0 2.0"); // Test degrees input

    turtlelib::Twist2D tw;

    ss >> tw;

    REQUIRE(tw.omega == Catch::Approx(turtlelib::deg2rad(90.0)));
    REQUIRE(tw.x == Catch::Approx(1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));

    ss.clear();
    ss.str("1.57 rad 3.0 -4.0"); // Test radians input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(1.57));
    REQUIRE(tw.x == Catch::Approx(3.0));
    REQUIRE(tw.y == Catch::Approx(-4.0));

    ss.clear();
    ss.str("0.5 -1.0 2.0"); // Test no unit input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(0.5));
    REQUIRE(tw.x == Catch::Approx(-1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));
}

TEST_CASE("Transform identity input", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf;

    REQUIRE(tf.rotation() == Catch::Approx(0.0));
    REQUIRE(tf.translation().x == Catch::Approx(0.0));
    REQUIRE(tf.translation().y == Catch::Approx(0.0));
}

TEST_CASE("Pure transformation input", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {2.0, 3.0};
    tf = turtlelib::Transform2D(trans);

    REQUIRE(tf.rotation() == Catch::Approx(0.0));
    REQUIRE(tf.translation().x == Catch::Approx(2.0));
    REQUIRE(tf.translation().y == Catch::Approx(3.0));
}

TEST_CASE("Pure rotation input", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf;
    tf = turtlelib::Transform2D(turtlelib::deg2rad(90.0));

    REQUIRE(tf.rotation() == Catch::Approx(turtlelib::deg2rad(90.0)));
    REQUIRE(tf.translation().x == Catch::Approx(0.0));
    REQUIRE(tf.translation().y == Catch::Approx(0.0));
}

TEST_CASE("Transform with rotation and translation input", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {1.0, -1.0};
    tf = turtlelib::Transform2D(trans, turtlelib::deg2rad(45.0));

    REQUIRE(tf.rotation() == Catch::Approx(turtlelib::deg2rad(45.0)));
    REQUIRE(tf.translation().x == Catch::Approx(1.0));
    REQUIRE(tf.translation().y == Catch::Approx(-1.0));
}

TEST_CASE("Apply a transformation to a point", "[transform]") // Gregory, Aiosa
{
    turtlelib::Point2D p = {1.0, 2.0};
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {1.0, -1.0};
    tf = turtlelib::Transform2D(trans, turtlelib::deg2rad(45.0));

    p = tf(p);

    REQUIRE(p.x == Catch::Approx(0.2928932188));
    REQUIRE(p.y == Catch::Approx(1.1213203436));
}

TEST_CASE("Apply a transformation to a vector", "[transform]") // Gregory, Aiosa
{
    turtlelib::Vector2D v = {1.0, 3.0};
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {4.0, -1.0};
    tf = turtlelib::Transform2D(trans, turtlelib::deg2rad(60.0));

    v = tf(v);

    REQUIRE(v.x == Catch::Approx(-2.0980762114));
    REQUIRE(v.y == Catch::Approx(2.366));
}

TEST_CASE("Apply a transformation to a twist", "[transform]") // Gregory, Aiosa
{
    turtlelib::Twist2D tw = {4, 3, 2};
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {3.0, -4.0};
    tf = turtlelib::Transform2D(trans, turtlelib::deg2rad(30.0));

    tw = tf(tw);

    REQUIRE(tw.omega == Catch::Approx(4.0));
    REQUIRE(tw.x == Catch::Approx(-14.402));
    REQUIRE(tw.y == Catch::Approx(-8.768));
}

TEST_CASE("Transform inversion", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {3.0, -4.0};
    tf = turtlelib::Transform2D(trans, turtlelib::deg2rad(30.0));

    tf = tf.inv();

    REQUIRE(tf.rotation() == Catch::Approx(turtlelib::deg2rad(-30.0)));
    REQUIRE(tf.translation().x == Catch::Approx(-0.5980762114));
    REQUIRE(tf.translation().y == Catch::Approx(4.9641016151));
}

TEST_CASE("Transform multiplication", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf1;
    turtlelib::Vector2D trans1 = {3.0, -4.0};
    tf1 = turtlelib::Transform2D(trans1, turtlelib::deg2rad(45.0));

    turtlelib::Transform2D tf2;
    turtlelib::Vector2D trans2 = {2.0, 2.0};
    tf2 = turtlelib::Transform2D(trans2, turtlelib::deg2rad(15.0));

    tf1 *= tf2;

    REQUIRE(tf1.rotation() == Catch::Approx(turtlelib::deg2rad(60.0)));
    REQUIRE(tf1.translation().x == Catch::Approx(3.0));
    REQUIRE(tf1.translation().y == Catch::Approx(-1.1715728753));
}

TEST_CASE("Grabbing the translational component of the transform", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {3.0, -4.0};
    tf = turtlelib::Transform2D(trans, turtlelib::deg2rad(45.0));

    REQUIRE(tf.translation().x == Catch::Approx(3.0));
    REQUIRE(tf.translation().y == Catch::Approx(-4.0));
}

TEST_CASE("Grabbing the rotational component of the transform", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf;
    turtlelib::Vector2D trans = {3.0, -4.0};
    tf = turtlelib::Transform2D(trans, turtlelib::deg2rad(45.0));

    REQUIRE(tf.rotation() == Catch::Approx(turtlelib::deg2rad(45.0)));
}

TEST_CASE("Transform2D input with brackets", "[transform]") // Gregory, Aiosa
{
    std::stringstream ss;
    ss.str("{90 deg, 1.0, 2.0}"); // Test degrees input

    turtlelib::Transform2D tf;

    ss >> tf;

    REQUIRE(tf.rotation() == Catch::Approx(turtlelib::deg2rad(90.0)));
    REQUIRE(tf.translation().x == Catch::Approx(1.0));
    REQUIRE(tf.translation().y == Catch::Approx(2.0));

    ss.clear();
    ss.str("{1.57 rad, 3.0, -4.0}"); // Test radians input

    ss >> tf;
    REQUIRE(tf.rotation() == Catch::Approx(1.57));
    REQUIRE(tf.translation().x == Catch::Approx(3.0));
    REQUIRE(tf.translation().y == Catch::Approx(-4.0));

    ss.clear();
    ss.str("{0.5, -1.0, 2.0}"); // Test no unit input

    ss >> tf;
    REQUIRE(tf.rotation() == Catch::Approx(0.5));
    REQUIRE(tf.translation().x == Catch::Approx(-1.0));
    REQUIRE(tf.translation().y == Catch::Approx(2.0));
}

TEST_CASE("Transform2D input without brackets", "[transform]") // Gregory, Aiosa
{
    std::stringstream ss;
    ss.str("90 deg 1.0 2.0"); // Test degrees input

    turtlelib::Transform2D tf;

    ss >> tf;

    REQUIRE(tf.rotation() == Catch::Approx(turtlelib::deg2rad(90.0)));
    REQUIRE(tf.translation().x == Catch::Approx(1.0));
    REQUIRE(tf.translation().y == Catch::Approx(2.0));

    ss.clear();
    ss.str("1.57 rad 3.0 -4.0"); // Test radians input

    ss >> tf;
    REQUIRE(tf.rotation() == Catch::Approx(1.57));
    REQUIRE(tf.translation().x == Catch::Approx(3.0));
    REQUIRE(tf.translation().y == Catch::Approx(-4.0));

    ss.clear();
    ss.str("0.5 -1.0 2.0"); // Test no unit input

    ss >> tf;
    REQUIRE(tf.rotation() == Catch::Approx(0.5));
    REQUIRE(tf.translation().x == Catch::Approx(-1.0));
    REQUIRE(tf.translation().y == Catch::Approx(2.0));
}

TEST_CASE("Multiply to transforms", "[transform]") // Gregory, Aiosa
{
    turtlelib::Transform2D tf1;
    turtlelib::Vector2D trans1 = {3.0, -4.0};
    tf1 = turtlelib::Transform2D(trans1, turtlelib::deg2rad(45.0));

    turtlelib::Transform2D tf2;
    turtlelib::Vector2D trans2 = {2.0, 2.0};
    tf2 = turtlelib::Transform2D(trans2, turtlelib::deg2rad(15.0));

    turtlelib::Transform2D result;
    result = tf1 * tf2;

    REQUIRE(result.rotation() == Catch::Approx(turtlelib::deg2rad(60.0)));
    REQUIRE(result.translation().x == Catch::Approx(3.0));
    REQUIRE(result.translation().y == Catch::Approx(-1.1715728753));
}