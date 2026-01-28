#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_approx.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/angle.hpp>
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

TEST_CASE("Formatting a vector to a string", "[Formatter]"){
    turtlelib::Vector2D testVec1(2, 0);

    std::string outstr = std::format("Vector: {}", testVec1);

    REQUIRE(outstr == "Vector: [2, 0]");
}

TEST_CASE("Formatting a point to a string", "[Formatter]"){
    turtlelib::Point2D testVec1(2, 0);

    std::string outstr = std::format("Point: {}", testVec1);

    REQUIRE(outstr == "Point: (2, 0)");
}

TEST_CASE("Scalar multiplication of vectors", "[operator*]"){
    turtlelib::Vector2D testVec1(1, 2);

    turtlelib::Vector2D outVec = 3.0 * testVec1;

    REQUIRE(outVec.x == 3.0);
    REQUIRE(outVec.y == 6.0);

    outVec = testVec1 * 4.0;

    REQUIRE(outVec.x == 4.0);
    REQUIRE(outVec.y == 8.0);
}

TEST_CASE("Dot product of vectors", "[operator*]"){
    turtlelib::Vector2D vecA(1, 2);
    turtlelib::Vector2D vecB(3, 4);

    double dotprod = vecA * vecB;

    REQUIRE(dotprod == Catch::Approx(11.0));
}

TEST_CASE("Angle between vectors", "[angle()]"){
    turtlelib::Vector2D vecA(1, 0);
    turtlelib::Vector2D vecB(1, 1);
    turtlelib::Vector2D vecC(0, 1);

    double angleAB = turtlelib::angle(vecA, vecB);
    double angleAC = turtlelib::angle(vecA, vecC);

    REQUIRE(angleAB == Catch::Approx(turtlelib::deg2rad(45.0)));
    REQUIRE(angleAC == Catch::Approx(turtlelib::deg2rad(90.0)));
}

TEST_CASE("In-place operations on vectors", "[in-place operators]"){
    turtlelib::Vector2D vecA(1, 2);
    turtlelib::Vector2D vecB(3, 4);

    vecA += vecB;

    REQUIRE(vecA.x == 4.0);
    REQUIRE(vecA.y == 6.0);

    vecA *= 2.0;

    REQUIRE(vecA.x == 8.0);
    REQUIRE(vecA.y == 12.0);

    vecA -= vecB;

    REQUIRE(vecA.x == 5.0);
    REQUIRE(vecA.y == 8.0);
}

TEST_CASE("Add vector to point in-place", "[in-place operators]"){
    turtlelib::Point2D pointA(1, 2);
    turtlelib::Vector2D vecA(3, 4);

    pointA += vecA;

    REQUIRE(pointA.x == 4.0);
    REQUIRE(pointA.y == 6.0);
}

TEST_CASE("Conversion between Point2D and Vector2D", "[conversion operators]"){
    turtlelib::Point2D pointA(1, 2);

    turtlelib::Vector2D vecA;
    vecA = pointA;

    REQUIRE(vecA.x == 1.0);
    REQUIRE(vecA.y == 2.0);

    turtlelib::Point2D pointB;
    pointB = vecA;

    REQUIRE(pointB.x == 1.0);
    REQUIRE(pointB.y == 2.0);
}

TEST_CASE("Magnitude of a vector", "[magnitude()]"){
    turtlelib::Vector2D vecA(3, 4);

    double mag = turtlelib::magnitude(vecA);

    REQUIRE(mag == Catch::Approx(5.0));
}