#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <turtlelib/geometry2d.hpp>
#include <sstream>

TEST_CASE("Points can be read from streams", "[operator>>]"){
    std::string PointString1 = "(1.2, 4.5)";
    std::stringstream stst1(PointString1);

    turtlelib::Point2D testPoint1;
    stst1 >> testPoint1;

    std::string PointString2 = "5.3, 8.9";
    std::stringstream stst2(PointString2);

    turtlelib::Point2D testPoint2;
    stst2 >> testPoint2;

    REQUIRE(testPoint1.x == 1.2);
    REQUIRE(testPoint1.y == 4.5);
    REQUIRE(testPoint2.x == 5.3);
    REQUIRE(testPoint2.y == 8.9);
}

TEST_CASE("Vectors can be read from streams", "[operator>>]"){
    std::string PointString1 = "[1.2, 4.5]";
    std::stringstream stst1(PointString1);

    turtlelib::Vector2D testVec1;
    stst1 >> testVec1;

    std::string PointString2 = "5.3, 8.9";
    std::stringstream stst2(PointString2);

    turtlelib::Vector2D testVec2;
    stst2 >> testVec2;

    REQUIRE(testVec1.x == 1.2);
    REQUIRE(testVec1.y == 4.5);
    REQUIRE(testVec2.x == 5.3);
    REQUIRE(testVec2.y == 8.9);
}

TEST_CASE("Vectors can be written to streams", "[operator<<]"){
    turtlelib::Vector2D testVec1(1, 2);
    std::string str = "";
    std::stringstream stst(str);
    stst << testVec1;

    REQUIRE(stst.str() == "[1, 2]");
}

TEST_CASE("Geometry primitive arithmetic", "[plus and minus]"){
    turtlelib::Point2D pointOne(1, 2);

    turtlelib::Point2D pointTwo(.5, .75);

    turtlelib::Vector2D outVec = pointOne - pointTwo;

    turtlelib::Point2D outPoint = pointOne + outVec;

    REQUIRE_THAT(outVec.x, Catch::Matchers::WithinAbs(.5, .001));
    REQUIRE_THAT(outVec.y, Catch::Matchers::WithinAbs(1.25, .001));

    REQUIRE_THAT(outPoint.x, Catch::Matchers::WithinAbs(1.5, .001));
    REQUIRE_THAT(outPoint.y, Catch::Matchers::WithinAbs(3.25, .001));

}

TEST_CASE("Normalization of vectors", "[normalize()]"){
    turtlelib::Vector2D testVec1(2, 0);
    testVec1 = turtlelib::normalize(testVec1);

    REQUIRE(testVec1.x == 1.0);
    REQUIRE(testVec1.y == 0.0);
}

TEST_CASE("Formatting to a string", "[Formatter]"){
    turtlelib::Vector2D testVec1(2, 0);

    std::string outstr = std::format("Vector: {}", testVec1);

    REQUIRE(outstr == "Vector: [2, 0]");
}

TEST_CASE("Formatting to a string", "[Formatter]"){
    turtlelib::Vector2D testVec1(2, 0);

    std::string outstr = std::format("Vector: {}", testVec1);

    REQUIRE(outstr == "Point: [2, 0]");
}