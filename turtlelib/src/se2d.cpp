#include <turtlelib/se2d.hpp>
#include <turtlelib/angle.hpp>
#include <iostream>

namespace turtlelib{

//Helper macro for the stream operator
#define REMOVE_COMMA if(is.peek() == ',') is.get();
#define REMOVE_SPACE if(is.peek() == ' ') is.get();

std::istream & operator>>(std::istream & is, Twist2D & tw){
    bool HasDelimiter = false;
    if(is.peek() == '<'){
        HasDelimiter = true;
        is.get();
    }

    is >> tw.omega;
    REMOVE_SPACE

    std::string unitstr;

    if(is.peek() == 'd'){
        tw.omega = deg2rad(tw.omega);
        is >> unitstr;
    }else if(is.peek() == 'r'){
        is >> unitstr;
    }

    REMOVE_COMMA

    is >> tw.x;

    REMOVE_COMMA

    is >> tw.y;

    if(is.peek() == '>'){
            if(HasDelimiter){
                is.get();
            }else{
                // ####### begin_citation [4] #######
                is.setstate(std::ios_base::failbit);
                // ####### end_citation [4] #######
            }
    }else if(HasDelimiter){
        is.setstate(std::ios_base::failbit);
    }

    return is;
}

std::istream & operator>>(std::istream & is, Transform2D & tf){
    is >> std::ws;
    bool HasDelimiter = false;
    if(is.peek() == '{'){
        HasDelimiter = true;
        is.get();
    }

    double rot;

    is >> rot;
    tf.setrot(rot);

    REMOVE_SPACE

    std::string unitstr;
    if(is.peek() == 'd'){
        tf.setrot(deg2rad(rot));
        is >> unitstr;
    }else if(is.peek() == 'r'){
        is >> unitstr;
    }

    REMOVE_COMMA

    Vector2D offset;

    is >> offset.x;

    REMOVE_COMMA

    is >> offset.y;
    tf.setoffset(offset);

    if(is.peek() == '}'){
            if(HasDelimiter){
                is.get();
            }else{
                // ####### begin_citation [4] #######
                is.setstate(std::ios_base::failbit);
                // ####### end_citation [4] #######
            }
    }else if(HasDelimiter){
        is.setstate(std::ios_base::failbit);
    }

    return is;
}

Transform2D::Transform2D(){
    offset = Vector2D(0, 0);
    rot = 0;
}

Transform2D::Transform2D(Vector2D offset){
    this->offset = offset;
    rot = 0;
}

Transform2D::Transform2D(double radians){
    offset = Vector2D(0, 0);
    rot = radians;
}

Transform2D::Transform2D(Vector2D trans, double radians){
    this->offset = trans;
    rot = radians;
}

Transform2D::Transform2D(double radians, double x, double y){
    offset = Vector2D(x, y);
    rot = radians;
}

Point2D Transform2D::operator()(Point2D p) const{
    Point2D out;

    out.x = p.x * cos(rot) - p.y * sin(rot) + offset.x;
    out.y = p.x * sin(rot) + p.y * cos(rot) + offset.y;

    return out;
}

Vector2D Transform2D::operator()(Vector2D v) const{
    Vector2D out;

    out.x = v.x * cos(rot) - v.y * sin(rot);
    out.y = v.x * sin(rot) + v.y * cos(rot);

    return out;
}

Twist2D Transform2D::operator()(Twist2D v) const{
    double newx = v.x * cos(rot) - v.y * sin(rot);
    double newy = v.x * sin(rot) + v.y * cos(rot);

    v.x = newx + v.omega * (offset.y);
    v.y = newy + v.omega * (-offset.x);;

    return v;
}

Transform2D Transform2D::inv() const{
    Transform2D newTrans = Transform2D(-rot);

    newTrans.offset = -1 * newTrans(offset);

    return newTrans;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs){
    offset = offset + (*this)(rhs.offset);

    rot += rhs.rot;

    return *this;
}

Twist2D Twist2D::operator*=(const double & a){
    omega *= a;
    x *= a;
    y *= a;

    return *this;
}

Twist2D operator*(const double & a, const Twist2D & tw){
    Twist2D out = tw;
    out *= a;
    return out;
}

Twist2D operator*(const Twist2D & tw, const double & a){
    return a * tw;
}


Transform2D integrate_twist(const Twist2D & tw){
    const double w = tw.omega;

    // Near-zero rotation: pure translation
    if (std::abs(w) < 1e-9)
    {
        return Transform2D(0.0, tw.x, tw.y);
    }

    const double sinw = std::sin(w);
    const double cosw = std::cos(w);

    const double x =
        ( tw.x * sinw + tw.y * (1.0 - cosw) ) / w;

    const double y =
        ( tw.y * sinw - tw.x * (1.0 - cosw) ) / w;

    return Transform2D(w, x, y);
}

} // namespace turtlelib