#ifndef TURTLELIB_SE2_INCLUDE_GUARD_HPP
#define TURTLELIB_SE2_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#pragma once
#include <iosfwd>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/angle.hpp>
#include <format>

namespace turtlelib
{

    /// \brief represent a 2-Dimensional twist
    struct Twist2D
    {
        /// \brief the angular velocity
        double omega = 0.0;

        /// \brief the linear x velocity
        double x = 0.0;

        /// \brief the linear y velocity
        double y = 0.0;

        /// \brief Construct a Twist2D
        /// \param omega The angular velocity
        /// \param x The linear x velocity
        /// \param y The linear y velocity
        Twist2D(double omega = 0.0, double x = 0.0, double y = 0.0) : omega(omega), x(x), y(y) {}

        /// \brief in-place scalar multiplication
        /// \param a The scalar to multiply by
        /// \return The updated twist
        Twist2D operator*=(const double & a);
    };

    /// \brief Scalar multiplication of a twist
    /// \param a Scalar multiple
    /// \param tw Twist
    /// \return The scaled twist
    Twist2D operator*(const double & a, const Twist2D & tw);

    /// \brief Scalar multiplication of a twist
    /// \param a Scalar multiple
    /// \param tw Twist
    /// \return The scaled twist
    Twist2D operator*(const Twist2D & tw, const double & a);

    /// \brief read the Twist2D in the format "<w [<unit>], x, y>" or as "w [<unit>] x y"
    /// The "" are not part of the input.
    /// The [<unit>] is optional and can be any string without spaces that starts with an r
    /// (for rad/s) and any string without spaces that starts with a d for deg/s)
    /// If the unit is omitted, assume rad/s.
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        Transform2D(double radians, double x, double y);

        /// \brief apply a transformation to a 2D Point
        /// \param p the point to transform
        /// \return a point in the new coordinate system
        Point2D operator()(Point2D p) const;

        /// \brief apply a transformation to a 2D Vector
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const {return this->offset;}

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const {return this->rot;}
        
    protected:
        Vector2D offset;
        double rot;
    };

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as:
    ///  "theta [<unit>] dx dy" (i.e., three numbers separated by whitespace, angle assumed to be radians)
    //   "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  [<unit>] is optional and can be any string without spaces that starts with a d for deg or r for rad
    ///  If [<unit>] is omitted, assume the unit is radians
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    inline Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs *= rhs;
        return lhs;
    }

    Transform2D integrate_twist(const Twist2D & tw);

}

/// \brief print the Twist2D as "<w [<unit>], x, y>"
/// An R at the beginning of the format-spec makes [<unit>] rad/s
/// A  D at the beginning of the format-spec makes [<unit>] deg/s
/// No R or D means no unit is printed but the w is taken to be in rad/s
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers inserted
/// into the string.
template<class CharT>
class std::formatter<turtlelib::Twist2D, CharT> : public std::formatter<double, CharT>
{
    public:
    using super = formatter<double, CharT>;

    template<class ParseContext>
    constexpr auto parse(ParseContext& ctx){
        auto it = ctx.begin();

        if(it != ctx.end() && (*it == CharT('D') || *it == CharT('R'))){
            if(*it == CharT('D')) {option = 'D';}
            if(*it == CharT('R')) {option = 'R';}

            it++;
        }

        ctx.advance_to(it);

        return super::parse(ctx);
    }

    template<class FormatContext>
    auto format(const turtlelib::Twist2D & obj, FormatContext& ctx) const{
        auto out = ctx.out();

        out = std::format_to(out, "<");
        if(option == 'R'){
            out = super::format(obj.omega, ctx);
            out = std::format_to(out, " rad/s, ");
        }
        else if(option == 'D'){
            out = super::format(turtlelib::rad2deg(obj.omega), ctx);
            out = std::format_to(out, " deg/s, ");
        }
        else{
            out = super::format(obj.omega, ctx);
            out = std::format_to(out, ", ");
        }
        
        out = super::format(obj.x, ctx);
        out = std::format_to(out, ", ");
        out = super::format(obj.y, ctx);
        out = std::format_to(out, ">");

        return out;
    }

    protected:
    char option = '\0';
};

/// \brief A formatter for Transform2D
/// Creates a string representation of a Transform2D
/// as "{<angle> [<unit>], <x> <y>}"
/// An R at the beginning of the format-spec makes [<unit>] rad
/// A D  at the beginning of the format-spec makes [<unit>] deg
/// No R or D means no unit is printed but the angle is in radians.
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers that
/// are put into the string
template<class CharT>
class std::formatter<turtlelib::Transform2D, CharT> : public std::formatter<turtlelib::Twist2D, CharT>
{
    using super = formatter<double, CharT>;

    public:
    template<class FormatContext>
    auto format(const turtlelib::Transform2D& obj, FormatContext& ctx) const{
        auto out = ctx.out();

        out = std::format_to(out, "{{");
        if(this->option == 'R'){
            out = super::format(obj.rotation(), ctx);
            out = std::format_to(out, " rad, ");
        }
        else if(this->option == 'D'){
            out = super::format(turtlelib::rad2deg(obj.rotation()), ctx);
            out = std::format_to(out, " deg, ");
        }
        else{
            out = super::format(obj.rotation(), ctx);
            out = std::format_to(out, ", ");
        }
        
        out = super::format(obj.translation().x, ctx);
        out = std::format_to(out, ", ");
        out = super::format(obj.translation().y, ctx);
        out = std::format_to(out, "}}");

        return out;
    }
};

#endif