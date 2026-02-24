#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives and other mathematical objects


// Note: <iosfwd> contains forward definitions for iostream objects
// allowing implementation of custom iostream operators without
// requiring the inclusion of <iostream>, which is a big header file
#include <iosfwd>
#include <format>

namespace turtlelib
{
    struct Vector2D; //Gotta forward declare this

    /// \brief a 2-Dimensional Point
    struct Point2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief Construct a Point2D
        /// \param x - x coordinate
        /// \param y - y coordinate
        Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}

        /// \brief allow us to make a point from a vector
        /// \param in The vector to convert
        /// \return The resulting point
        Point2D operator=(const Vector2D & in);

        /// \brief in-place addition of a vector to a point
        /// \param in The vector to add
        /// \return The updated point
        Point2D operator+=(const Vector2D & in);
    };

    /// \brief Input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   "(x, y)" or "x y"  (Not including the "").
    /// \param is An istream from which to read
    /// \param p [out] The Point2D object that will store the input
    /// \returns A reference to is. An error flag is set on the stream if the input cannot be parsed.
    /// The way input with istreams works is (more or less):
    /// What the user types is stored in a buffer until the user enters a newline (by pressing enter).
    /// The iostream methods then process the data in this buffer character-by-character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we expected an int but 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    std::istream & operator>>(std::istream & is, Point2D & p);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief Construct a Vector2D
        /// \param x The x component
        /// \param y The y component
        Vector2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}

        /// \brief allow us to make a vector from a point
        /// \param in The point to convert
        /// \return The resulting vector
        Vector2D operator=(const Point2D & in);

        /// \brief in-place addition of another vector
        /// \param in The vector to add
        /// \return The updated vector
        Vector2D operator+=(const Vector2D & in);

        /// \brief in-place scalar multiplication
        /// \param a The scalar to multiply by
        /// \return The updated vector
        Vector2D operator*=(const double & a);

        /// \brief in-place vector subtraction
        /// \param in The vector to subtract
        /// \return The updated vector
        Vector2D operator-=(const Vector2D & in);
    };

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this operator is not implemented in terms of -=
    /// because subtracting two Point2D yields a Vector2D not a Point2D
    Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Scalar multiplication of a vector
    /// \param a Scalar multiple
    /// \param vec Vector
    /// \return The scaled vector
    Vector2D operator*(const double & a, const Vector2D & vec);

    /// \brief Scalar multiplication of a vector
    /// \param a Scalar multiple
    /// \param vec Vector
    /// \return The scaled vector
    Vector2D operator*(const Vector2D & vec, const double & a);

    /// \brief Vector addition
    /// \param a First vector
    /// \param b Second vector
    /// \return The elementwise sum of a and b
    Vector2D operator+(const Vector2D & a, const Vector2D & b);

    /// \brief Vector subtraction
    /// \param a First vector
    /// \param b Second vector
    /// \return The elementwise difference of a and b
    Vector2D operator-(const Vector2D & a, const Vector2D & b);

    /// \brief Vector inner product
    /// \param a First vector
    /// \param b Second vector
    /// \return The dot product of a and b
    double operator*(const Vector2D & a, const Vector2D & b);

    /// \brief Vector inner product
    /// \param a First vector
    /// \param b Second vector
    /// \return The dot product of a and b
    double dot(const Vector2D & a, const Vector2D & b);

    /// \brief Vector equality
    /// \param a First vector
    /// \param b Second vector
    /// \return True if both vector components are equal
    bool operator==(const Vector2D & a, const Vector2D & b);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp);

    /// \brief output a 2 dimensional vector as [xcomponent, ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    /// Note: Here to demonstrate the old method of custom output
    /// std::format is very recent so std::ostream is commonly used.
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   "[x, y]" or "x y" (not including the "")
    /// \param is An istream from which to read
    /// \param v [out] - output vector
    /// \returns a reference to the istream, with any error flags set if
    /// a parsing error occurs
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief Compute the magnitude of a vector
    /// \param v The vector
    /// \return The magnitude of the vector
    double magnitude(const Vector2D & v);

    /// \brief Return a unit vector in the direction of v
    /// \param in The vector to normalize
    /// \return The normalized vector.
    /// \throws std::invalid_input if in is the zero vector
    Vector2D normalize(Vector2D in);

    /// \brief Compute the angle between two vectors
    /// \param a First vector
    /// \param b Second vector
    /// \return The angle between the two vectors in radians
    double angle(const Vector2D & a, const Vector2D & b);

    /// \brief Treat the components of the vector as angles and wrap them to [-pi, pi)
    /// \param in The vector to normalize
    /// \return The normalized vector.
    constexpr Vector2D normalize_angle(Vector2D in);
}
/// \brief A Formatter for 2D points
/// The output is "(x, y)"
/// All floating-point format specifiers are honored and applied to both x and y.
template<class CharT>
class std::formatter<turtlelib::Point2D, CharT> : public std::formatter<double, CharT>
{
    using super = formatter<double, CharT>;

    public:

    auto format(const turtlelib::Point2D& obj, std::format_context& ctx) const {
        auto out = ctx.out();

        out = std::format_to(out, "(");
        out = super::format(obj.x, ctx);
        out = std::format_to(out, ", ");
        out = super::format(obj.y, ctx);
        out = std::format_to(out, ")");

        return out;
    }
};

/// \brief A formatter for Vector2D
/// All double format-spec specifiers apply to each number in the vector
/// The vector is output as [x, y]
template<class CharT>
class std::formatter<turtlelib::Vector2D, CharT> : public std::formatter<double, CharT>
{
    using super = formatter<double, CharT>;

    public:

    auto format(const turtlelib::Vector2D& obj, std::format_context& ctx) const {
        auto out = ctx.out();

        out = std::format_to(out, "[");
        out = super::format(obj.x, ctx);
        out = std::format_to(out, ", ");
        out = super::format(obj.y, ctx);
        out = std::format_to(out, "]");

        return out;
    }
};
#endif
