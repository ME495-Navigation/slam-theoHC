#include <turtlelib/geometry2d.hpp>
#include <istream>
#include <cmath>

namespace turtlelib{

    std::istream & readPoint(std::istream & is, Point2D & p, char openDelim = '(', char closeDelim = ')'){
        bool hasParens = false;
        if(is.peek() == openDelim){
            is.get();
            hasParens = true;
        }

        is >> p.x;

        if(is.peek() == ','){
            is.get();
        }

        is >> p.y;

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

    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        Point2D outpoint;

        outpoint.x = tail.x + disp.x;

        outpoint.y = tail.y + disp.y;

        return outpoint;
    }

    std::istream & operator>>(std::istream & is, Vector2D & p){
        Point2D dummyval;
        readPoint(is, dummyval, '[', ']');
        p.x = dummyval.x;
        p.y = dummyval.y;

        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        return os << '[' << v.x << ", " << v.y << ']';
    }

    Vector2D normalize(Vector2D in){
        float len = std::pow(in * in, 0.5);
        
        in.x *= 1/len;
        in.y *= 1/len;

        return in;
    }
    
    Vector2D operator*(const double & a, const Vector2D & vec){
        return Vector2D(a * vec.x, a * vec.y);
    }

    Vector2D operator+(const Vector2D & a, const Vector2D & b){
        return Vector2D(a.x + b.x, a.y + b.y);
    }

    double operator*(const Vector2D & a, const Vector2D & b){
        return a.x * b.x + a.y + b.y;
    }
}