#include <turtlelib/geometry2d.hpp>
#include <turtlelib/angle.hpp>
#include <istream>
#include <cmath>

namespace turtlelib{

    std::istream & readPoint(std::istream & is, Point2D & p, char openDelim = '(', char closeDelim = ')'){
        bool hasParens = false;
        is >> std::ws;
        if(is.peek() == openDelim){
            is.get();
            hasParens = true;
        }

        is >> p.x;
        is >> std::ws;
        if(is.peek() == ','){
            is.get();
        }

        is >> p.y;

        is >> std::ws;
        if(is.peek() == closeDelim){
            if(hasParens){
                is.get();
            }else{
                // ####### begin_citation [4] #######
                is.setstate(std::ios_base::failbit);
                // ####### end_citation [4] #######
            }
        }else if(hasParens){
            is.setstate(std::ios_base::failbit);
        }

        return is;
    }
    
    std::istream & operator>>(std::istream & is, Point2D & p){
        return readPoint(is, p);
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail){
        Vector2D outvec;

        outvec.x = head.x - tail.x;
        outvec.y = head.y - tail.y;

        return outvec;
    }

    Vector2D operator-(const Vector2D & a, const Vector2D & b){
        return a + (-1 * b);
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        Point2D outpoint;

        outpoint.x = tail.x + disp.x;

        outpoint.y = tail.y + disp.y;

        return outpoint;
    }

    std::istream & operator>>(std::istream & is, Vector2D & p){
        Point2D dummyval;

        readPoint(is, dummyval, '[', ']');

        p = dummyval;

        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        return os << '[' << v.x << ", " << v.y << ']';
    }

    Vector2D normalize(Vector2D in){
        double len = magnitude(in);
        
        in.x /= len;
        in.y /= len;

        return in;
    }
    
    Vector2D operator*(const double & a, const Vector2D & vec){
        return Vector2D(a * vec.x, a * vec.y);
    }

    Vector2D operator*(const Vector2D & vec, const double & a){
        return a * vec;
    }

    Vector2D operator+(const Vector2D & a, const Vector2D & b){
        return Vector2D(a.x + b.x, a.y + b.y);
    }

    double operator*(const Vector2D & a, const Vector2D & b){
        return a.x * b.x + a.y * b.y;
    }

    bool operator==(const Vector2D & a, const Vector2D & b){
        return a.x == b.x && a.y == b.y;
    }

    Vector2D Vector2D::operator=(const Point2D & in){
        x = in.x;
        y = in.y;
        return *this;
    }

    Point2D Point2D::operator=(const Vector2D & in){
        x = in.x;
        y = in.y;
        return *this;
    }

    Point2D Point2D::operator+=(const Vector2D & in){
        x += in.x;
        y += in.y;
        return *this;
    }

    Vector2D Vector2D::operator+=(const Vector2D & in){
        x += in.x;
        y += in.y;
        return *this;
    }

    Vector2D Vector2D::operator*=(const double & a){
        x *= a;
        y *= a;
        return *this;
    };

    Vector2D Vector2D::operator-=(const Vector2D & in){
        x -= in.x;
        y -= in.y;
        return *this;
    }

    double angle(const Vector2D & a, const Vector2D & b){
        double dotprod = dot(a, b);
        double mags = magnitude(a) * magnitude(b);

        return std::acos(dotprod / mags);
    }

    double magnitude(const Vector2D & v){
        return std::pow(v * v, 0.5);
    }

    double dot(const Vector2D & a, const Vector2D & b){
        return a * b;
    }
}