/***************************************************************************
 *            io/geometry2d.hpp
 *
 *  Copyright  2009-20  Davide Bresolin, Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

/*! \file io/geometry2d.hpp
 *  \brief 2D geometric objects exclusively for drawing.
 */

#ifndef ARIADNE_GEOMETRY2D_HPP
#define ARIADNE_GEOMETRY2D_HPP

typedef unsigned int Nat;

namespace Ariadne {

template<class T >
class List;

template<class R, class A> inline R numeric_cast(const A&);

struct Vector2d {
    double x,y; Vector2d(double x_, double y_) : x(x_), y(y_) { }
    template<class X, class Y> Vector2d(const X& x_, const Y& y_) : x(numeric_cast<double>(x_)), y(numeric_cast<double>(y_)) { }
};
inline Vector2d operator-(const Vector2d& v) { return Vector2d(-v.x,-v.y); }
inline Vector2d operator+(const Vector2d& v1, const Vector2d& v2) { return Vector2d(v1.x+v2.x,v1.y+v2.y); }
inline Vector2d operator-(const Vector2d& v1, const Vector2d& v2) { return Vector2d(v1.x-v2.x,v1.y-v2.y); }
inline Vector2d operator*(const double& s1, const Vector2d& v2) { return Vector2d(s1*v2.x,s1*v2.y); }
inline OutputStream& operator<<(OutputStream& os, const Vector2d& v) {
    return os << "["<<v.x<<","<<v.y<<"]"; }

struct Point2d {
    double x,y; Point2d() : x(), y() { } Point2d(double x_, double y_) : x(x_), y(y_) { }
    double& operator[](SizeType i) { return (&x)[i]; } double const& operator[](SizeType i) const { return (&x)[i]; }
    template<class X, class Y> Point2d(const X& x_, const Y& y_) : x(numeric_cast<double>(x_)), y(numeric_cast<double>(y_)) { }
};
inline Bool operator==(Point2d& pt1, const Point2d& pt2) { return pt1.x==pt2.x && pt1.y==pt2.y; }
inline Point2d& operator+=(Point2d& pt, const Vector2d& v) { pt.x+=v.x; pt.y+=v.y; return pt; }
inline Point2d& operator-=(Point2d& pt, const Vector2d& v) { pt.x-=v.x; pt.y-=v.y; return pt; }
inline OutputStream& operator<<(OutputStream& os, const Point2d& pt) {
    return os << "("<<pt.x<<","<<pt.y<<")"; }

struct Box2d {
    double xl,xu,yl,yu; Box2d() { }
    Box2d(double xl_, double xu_, double yl_, double yu_) : xl(xl_), xu(xu_), yl(yl_), yu(yu_) { }
};
inline OutputStream& operator<<(OutputStream& os, const Box2d& bx) {
    return os << "["<<bx.xl<<","<<bx.xu<<"]x["<<bx.yl<<","<<bx.yu<<"]"; }

struct Polytope2d
    : public Drawable2dInterface
{
    List<Point2d> boundary;
  public:
    Polytope2d(List<Point2d> bd_) : boundary(bd_) { }
    SizeType number_of_vertices() const { return boundary.size(); }
    List<Point2d>& vertices() { return boundary; }
    List<Point2d>const& vertices() const { return boundary; }
    Point2d const& vertex(SizeType i) const { return boundary[i]; }

    virtual Polytope2d* clone() const { return new Polytope2d(*this); }
    virtual DimensionType dimension() const { return 2u; }

    virtual Void draw(CanvasInterface& canvas, const Projection2d& p) const {
        if(boundary.size()==1) { canvas.dot(boundary[0].x,boundary[0].y); return; }
        canvas.move_to(boundary[0].x,boundary[0].y);
        for(SizeType i=1; i!=boundary.size(); ++i) {
            canvas.line_to(boundary[i].x,boundary[i].y);
        }
        canvas.line_to(boundary[0].x,boundary[0].y);
        canvas.fill();
    }

    Polytope2d operator+(const Vector2d& v) {
        Polytope2d r(*this); for(SizeType i=0; i!=r.boundary.size(); ++i) { r.boundary[i]+=v; } return r;
    }

    friend OutputStream& operator<<(OutputStream& os, const Polytope2d& p) {
        return p._write(os);
    }

  private:
    virtual OutputStream& _write(OutputStream& os) const { return os << "Polytope2d(boundary="<<boundary<<")"; }
};


} // namespace Ariadne


#endif // ARIADNE_GEOMETRY2D_HPP
