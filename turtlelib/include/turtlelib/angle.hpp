#ifndef TURTLELIB_ANGLE_HPP_INCLUDE_GUARD
#define TURTLELIB_ANGLE_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles
#include <cmath>
#include <numbers>

namespace turtlelib
{
    constexpr double pi = std::numbers::pi;
    /// \brief Approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 A number to compare
    /// \param d2 A second number to compare
    /// \param epsilon Absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        return std::abs(d1 - d2) < epsilon;
    }

    /// \brief Convert degrees to radians
    /// \param deg Angle in degrees
    /// \returns The equivalent angle in radians
    constexpr double deg2rad(double deg)
    {
        return (deg / 180) * pi;
    }

    /// \brief Convert radians to degrees
    /// \param rad  Angle in radians
    /// \returns The equivalent angle in degrees
    constexpr double rad2deg(double rad)
    {
        return (rad / pi) * 180;
    }

    /// \brief Wrap an angle to (-PI, PI]
    /// \param rad Angle in radians
    /// \return An equivalent angle the range (-PI, PI]
    constexpr double normalize_angle(double rad)
    {
        // ####### begin_citation [5] #######
        double r = std::fmod(rad, 2 * pi);

        if (r <= -pi) r += 2 * pi;
        if (r >  pi)  r -= 2 * pi;
        return r;
        // ####### end_citation [5] #######
    }

    /// static_assertions test compile time assumptions.
    /// These tests can provide assurance that your code is correct at compile time!
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0, 0.1, .2), "epsilon setting failed");
    static_assert(!almost_equal(0, 0.1), "nowhere near equal");
    static_assert(!almost_equal(-1.0, 1.0), "not even the same number");
    static_assert(almost_equal(0.0, -0.0), "zeros are the same");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(180.0), pi), "deg2rad failed");
    static_assert(almost_equal(deg2rad(-180.0), -1*pi), "deg2rad failed");
    static_assert(almost_equal(deg2rad(90), 0.5*pi), "deg2rad failed");
    static_assert(almost_equal(deg2rad(270), 1.5*pi), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(pi), 180), "rad2deg failed");
    static_assert(almost_equal(rad2deg(2*pi), 360), "rad2deg failed");
    static_assert(almost_equal(rad2deg(-pi/2), -90), "rad2deg failed");
    static_assert(almost_equal(rad2deg(-pi/4), -45), "rad2deg failed");
    /// TASK: Write at least 4 additional tests for rad2deg. Include at least one negative angle
    /// HINT: It helps to use angles where there is a simple known formula (e.g., 30 degrees)

    static_assert(almost_equal(normalize_angle(0.0), 0.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(pi), pi), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-pi), pi), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-pi/4), -pi/4), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(3*pi/2), -pi/2), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-5*pi/2), -pi/2), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-7*pi), pi), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-8*pi), 0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(7*pi), pi), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(10*pi), 0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(pi/12), pi/12), "norm_angle failed");
}
#endif
