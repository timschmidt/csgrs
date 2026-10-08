/***************************************************************************
 *            io/graphics_interface.hpp
 *
 *  Copyright  2009-20  Pieter Collins, Mirko Albanese
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

/*! \file io/graphics_interface.hpp
 *  \brief Base graphics interface from which all plotting and drawing classes are inherited.
 */

#ifndef ARIADNE_GRAPHICS_INTERFACE_HPP
#define ARIADNE_GRAPHICS_INTERFACE_HPP

#include "../config.hpp"
#include "utility/declarations.hpp"
#include "numeric/numeric.hpp"
#include "numeric/float_bounds.hpp"

namespace Ariadne {

template<class R, class A> inline R numeric_cast(const A&);
template<class T, class CMP> class Set;

struct Vector2d;
struct Point2d;
struct Box2d;

typedef Box<Interval<FloatDPApproximation>> GraphicsBoundingBoxType;
struct Colour;

class Drawable2dInterface;
class LabelledDrawable2dInterface;
class FigureInterface;
class CanvasInterface;

struct Projection2d;
struct Projection3d;
struct Variables2d;
struct Variables3d;

enum class ProjType{ no_proj, x1_proj, x2_proj };

struct Projection2d {
    DimensionType n, i, j;
    Projection2d(DimensionType nn, DimensionType ii, DimensionType jj) : n(nn), i(ii), j(jj) { }
    DimensionType argument_size() const { return n; }
    DimensionType x_coordinate() const { return i; }
    DimensionType y_coordinate() const { return j; }
    friend OutputStream& operator<<(OutputStream& os, const Projection2d& p) {
        return os << "P<R"<<p.n<<";R2>[x"<<p.i<<",x"<<p.j<<"]"; }
};

struct Projection3d {
    DimensionType n, i, j, k;
    Projection3d(DimensionType nn, DimensionType ii, DimensionType jj, DimensionType kk) : n(nn), i(ii), j(jj), k(kk) { }
    DimensionType argument_size() const { return n; }
    DimensionType x_coordinate() const { return i; }
    DimensionType y_coordinate() const { return j; }
    DimensionType z_coordinate() const { return k; }
    friend OutputStream& operator<<(OutputStream& os, const Projection3d& p) {
        return os << "P<R"<<p.n<<";R3>[x"<<p.i<<",x"<<p.j<<",x"<<p.k<<"]"; }
};

//! \ingroup GraphicsModule
//! \brief Base interface for plotting and drawing classes.
class FigureInterface {
  public:
    virtual ~FigureInterface() = default;
    virtual FigureInterface& set_projection_map(const Projection2d& prj) = 0;
    virtual FigureInterface& set_bounding_box(const GraphicsBoundingBoxType& bx) = 0;
    virtual FigureInterface& set_projection(DimensionType as, DimensionType ix, DimensionType iy) = 0;
    virtual FigureInterface& set_line_style(Bool) = 0;
    virtual FigureInterface& set_line_width(Dbl) = 0;
    virtual FigureInterface& set_dot_radius(Dbl) = 0;
    virtual FigureInterface& set_line_colour(Colour) = 0;
    virtual FigureInterface& set_fill_opacity(Dbl) = 0;
    virtual FigureInterface& set_fill_colour(Colour) = 0;
    virtual Bool get_line_style() const = 0;
    virtual Dbl get_line_width() const = 0;
    virtual Colour get_line_colour() const = 0;
    virtual Bool get_fill_style() const = 0;
    virtual Dbl get_fill_opacity() const = 0;
    virtual Colour get_fill_colour() const = 0;
    virtual FigureInterface& draw(const Drawable2dInterface&) = 0;
};
inline Void draw(FigureInterface& fig, const Drawable2dInterface& shape) { fig.draw(shape); }

class Figure;
Void draw(Figure& fig, const Drawable2dInterface& shape);
Figure& operator<<(Figure& fig, const Drawable2dInterface& shape);

class LabelledFigure;
Void draw(LabelledFigure& fig, const LabelledDrawable2dInterface& shape);
LabelledFigure& operator<<(LabelledFigure& fig, const LabelledDrawable2dInterface& shape);

//! \ingroup GraphicsModule
//! \brief Interface to two-dimensional drawing canvas with the ability to draw polyhedra.
class CanvasInterface {
  public:
    //! \brief Destructor
    virtual ~CanvasInterface() = default;

    virtual Void initialise(StringType x, StringType y, StringType z, double xl, double xu, double yl, double yu, double lz, double uz) = 0;
    virtual Void initialise(StringType x, StringType y, double lx, double ux, double ly, double uy) = 0;
    virtual Void finalise() = 0;

    virtual Void write(const char* filename) const = 0;

    //! \brief Move the current initial point for a line to the point \a (x,y).
    virtual Void move_to(double x, double y) = 0;
    //! \brief Create a line segment from the current point to the point \a (x,y).
    virtual Void line_to(double x, double y) = 0;
    //! \brief Draw a circle with centre \a (x,y) and radius \a r.
    virtual Void circle(double x, double y, double r) = 0;
    //! \brief Draw a dot with centre \a (x,y).
    virtual Void dot(double x, double y) = 0;
    //! \brief Draw the working line.
    virtual Void stroke() = 0;
    //! \brief Draw and fill the working line.
    virtual Void fill() = 0;

    //! \brief Draw the lines from the \a boundary and fill the corresponding closed polytope
    virtual Void fill_boundary(List<Point2d> const& boundary) = 0;

    //! Set the width of the lines bounding shapes, in pixels.
    virtual Void set_line_width(double lw) = 0;
    //! \brief Set the colour of subsequent line to be drawn. The \a r, \a g and \a b
    //! arguments are intensities of red, green and blue on a scale of 0.0 to 1.0.
    virtual Void set_line_colour(double r, double g, double b) = 0;
    //! \brief  Set the colour of the next regions to be filled.
    virtual Void set_fill_opacity(double fo) = 0;
    //! \brief Set the colour of subsequent regions to be filled.
    virtual Void set_fill_colour(double r, double g, double b) = 0;

    //! \brief Set the colour palette.
    virtual Void set_colour_palette() = 0;
    //! \brief Draw and fill a 3d image.
    virtual Void fill_3d() = 0;
      //! \brief Set a heatmap projection.
    virtual Void set_heat_map(Bool b) = 0;
  public:
    template<class X, class Y> Void move_to(X x, Y y) { this->move_to(numeric_cast<double>(x),numeric_cast<double>(y)); }
    template<class X, class Y> Void line_to(X x, Y y) { this->line_to(numeric_cast<double>(x),numeric_cast<double>(y)); }
};


//! \ingroup GraphicsModule
//! \brief Base interface for drawable objects
class Drawable2dInterface {
  public:
    //! brief Virtual destructor.
    virtual ~Drawable2dInterface() = default;
    //! brief Make a dynamically-allocated copy.
    virtual Drawable2dInterface* clone() const = 0;
    //! brief Draw the object on the canvas \a c using line segments and fill/stroke commands.
    virtual Void draw(CanvasInterface& c, const Projection2d& p) const = 0;
    //! brief The dimension of the object in Euclidean space
    virtual DimensionType dimension() const = 0;
};

class Drawable2d3dInterface : public Drawable2dInterface {
  public:
    //! brief Virtual destructor.
    virtual ~Drawable2d3dInterface() = default;
    virtual Drawable2d3dInterface* clone2d3d() const = 0;
    //! brief Draw the object on the canvas \a c using line segments and fill/stroke commands.
    virtual Void draw(CanvasInterface& c, const Projection3d& p) const = 0;
    using Drawable2dInterface::draw;
};

//! \ingroup GraphicsModule
//! \brief Base interface for drawable objects
class LabelledDrawable2dInterface {
  public:
    //! brief Virtual destructor.
    virtual ~LabelledDrawable2dInterface() = default;
    //! brief Make a dynamically-allocated copy.
    virtual LabelledDrawable2dInterface* clone() const = 0;
    //! brief Draw the projection of object onto variables \a p on the canvas \a c .
    virtual Void draw(CanvasInterface& c, const Variables2d& p) const = 0;
};

class LabelledDrawable2d3dInterface : public LabelledDrawable2dInterface {
  public:
    //! brief Virtual destructor.
    virtual ~LabelledDrawable2d3dInterface() = default;
    //! brief Make a dynamically-allocated copy.
    virtual LabelledDrawable2d3dInterface* clone2d3d() const = 0;
    virtual LabelledDrawable2dInterface* clone() const = 0;

    //! brief Draw the projection of object onto variables \a p on the canvas \a c .
    virtual Void draw(CanvasInterface& c, const Variables3d& p) const = 0;
    using LabelledDrawable2dInterface::draw;
};

} // namespace Ariadne


#endif // ARIADNE_GRAPHICS_INTERFACE_HPP
