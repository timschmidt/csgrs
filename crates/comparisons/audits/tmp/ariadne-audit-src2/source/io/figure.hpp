/***************************************************************************
 *            io/figure.hpp
 *
 *  Copyright  2008-21  Pieter Collins, Mirko Albanese, Luca Geretti
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

/*! \file io/figure.hpp
 *  \brief Figure classes for drawing and outputting shapes in Euclidean space.
 */

#ifndef ARIADNE_FIGURE_HPP
#define ARIADNE_FIGURE_HPP

#include "config.hpp"

#include <iosfwd>
#include <string>
#include <vector>

#include "utility/typedefs.hpp"
#include "utility/declarations.hpp"
#include "symbolic/variable.hpp"
#include "colour.hpp"
#include "graphics_base.hpp"
#include "graphics_backend_interface.hpp"

namespace Ariadne {

typedef ApproximateBoxType GraphicsBoundingBoxType;

struct LineStyle { explicit LineStyle(Bool ls) : _style(ls) { } operator Bool() const { return this->_style; } private: Bool _style; };
struct LineWidth { explicit LineWidth(Dbl lw) : _width(lw) { } operator Dbl() const { return this->_width; } private: Dbl _width; };
struct LineColour : Colour { LineColour(const Colour& lc) : Colour(lc) { } LineColour(Dbl r, Dbl g, Dbl b) : Colour(r,g,b) { } };
struct FillStyle { explicit FillStyle(Bool fs) : _style(fs) { } operator Bool() const { return this->_style; } private: Bool _style; };
struct FillOpacity { explicit FillOpacity(Dbl fo) : _opacity(fo) { } operator Dbl() const { return this->_opacity; } private: Dbl _opacity; };
struct FillColour : Colour { FillColour(const Colour& fc) : Colour(fc) { } FillColour(Dbl r, Dbl g, Dbl b) : Colour(r,g,b) { } };
struct Animated { explicit Animated(Bool b) : _anim(b) { } operator Bool() const { return this-> _anim; } private: Bool _anim; };

inline LineStyle line_style(Bool s) { return LineStyle(s); }
inline LineWidth line_width(Dbl w) { return LineWidth(w); }
inline LineColour line_colour(const Colour& c) { return LineColour(c); }
inline LineColour line_colour(Dbl r, Dbl g, Dbl b) { return LineColour(Colour(r,g,b)); }
inline FillStyle fill_style(Bool s) { return FillStyle(s); }
inline FillOpacity fill_opacity(Dbl o) { return FillOpacity(o); }
inline FillColour fill_colour(const Colour& c) { return FillColour(c); }
inline FillColour fill_colour(Dbl r, Dbl g, Dbl b) { return FillColour(Colour(r,g,b)); }
inline Animated set_animated(Bool b) { return Animated(b); }

struct GraphicsProperties {
    GraphicsProperties()
        : dot_radius(1.0), line_style(true), line_width(1.0), line_colour(black), fill_style(true), fill_colour(orange), is_projected(false), is_3d(false), is_animated(false) { }
    GraphicsProperties(Bool ls, Dbl lw, Dbl dr, Colour lc, Bool fs, Colour fc)
        : dot_radius(dr), line_style(ls), line_width(lw), line_colour(lc), fill_style(fs), fill_colour(fc), is_3d(false), is_animated(false) { }
    Dbl dot_radius;
    Bool line_style;
    Dbl line_width;
    Colour line_colour;
    Bool fill_style;
    Colour fill_colour;

    Bool is_projected;
    Bool is_3d;
    Bool is_animated;

    GraphicsProperties& set_dot_radius(Dbl);
    GraphicsProperties& set_line_style(Bool);
    GraphicsProperties& set_line_width(Dbl);
    GraphicsProperties& set_line_colour(Colour);
    GraphicsProperties& set_fill_style(Bool);
    GraphicsProperties& set_fill_colour(Colour);

    GraphicsProperties& set_line_colour(Dbl, Dbl, Dbl);
    GraphicsProperties& set_fill_colour(Dbl, Dbl, Dbl);
    GraphicsProperties& set_fill_opacity(Dbl);

    GraphicsProperties& set_animated(Bool);


    friend OutputStream& operator<<(OutputStream& os, GraphicsProperties const& gp);
};

Void set_properties(CanvasInterface& canvas, const GraphicsProperties& properties);

// TODO: Move to interval.hpp as templated IntervalData classes
template<> class Interval<Double> {
    Double _l, _u;
  public:
    Interval(Double l, Double u) : _l(l), _u(u) { }
    Double lower_bound() const { return _l; }
    Double upper_bound() const { return _u; }
};

template<> class Interval<ApproximateDouble> {
    ApproximateDouble _l, _u;
  public:
    typedef ApproximateDouble UpperBoundType;
    Interval(ApproximateDouble l, ApproximateDouble u) : _l(l), _u(u) { }
    template<class UB> Interval(Interval<UB> const& ivl) : _l(ivl.lower_bound()), _u(ivl.upper_bound()) { }
    ApproximateDouble lower_bound() const { return _l; }
    ApproximateDouble upper_bound() const { return _u; }
};
using ApproximateDoubleInterval = Interval<ApproximateDouble>;
using ApproximateDoubleVariableInterval = VariableInterval<ApproximateDouble>;
using DoubleVariableInterval = VariableInterval<Double>;

struct Variables2d {
    Identifier _x,_y;
    Variables2d(const RealVariable& x, const RealVariable& y);
    RealVariable x() const;
    RealVariable y() const;
};

struct Variables3d {
  Identifier _x,_y,_z;
  Variables3d(const RealVariable& x, const RealVariable& y, const RealVariable& z);
  RealVariable x() const;
  RealVariable y() const;
  RealVariable z() const;
};

struct Axes2d {
    Axes2d(const ApproximateDoubleVariableInterval x, const ApproximateDoubleVariableInterval& y);
    Axes2d(ApproximateDouble xl, const RealVariable& x, ApproximateDouble xu, ApproximateDouble yl, const RealVariable& y, ApproximateDouble yu);
    Variables2d variables;
    Map<RealVariable,ApproximateDoubleInterval> bounds;
};

struct Axes3d {
  Axes3d(const ApproximateDoubleVariableInterval x, const ApproximateDoubleVariableInterval y, const ApproximateDoubleVariableInterval z);
  Axes3d(ApproximateDouble xl, const RealVariable x, ApproximateDouble xu, ApproximateDouble yl, const RealVariable y, ApproximateDouble yu, ApproximateDouble zl, const RealVariable z, ApproximateDouble zu);
  Variables3d variables3d;
  Map<RealVariable, ApproximateDoubleInterval> bounds;
};

//! \brief Class for plotting figures.
class Figure
    : public FigureInterface
{
  public:
    ~Figure();
    //! Construct a figure projecting \a bbx onto the \a proj coordinates
    Figure(const GraphicsBoundingBoxType& bbx, const Projection2d& proj);
    //! Construct a figure projecting \a bbx onto the (\a ix, \a iy) coordinates
    Figure(const GraphicsBoundingBoxType& bbx, DimensionType ix, DimensionType iy);
    //! Construct a figure projecting \a bbx onto the (\a ix, \a iy) coordinates
    Figure(const GraphicsBoundingBoxType& bbx, Pair<DimensionType,DimensionType> ixy);

    Figure(const GraphicsBoundingBoxType& bbx, const Projection3d& proj);

    Figure(const GraphicsBoundingBoxType& bbx, DimensionType ix, DimensionType iy, DimensionType iz);

    Figure& set_projection_map(const Projection2d&);
    Figure& set_projection_map(const Projection3d&);
    
    Projection2d get_2d_projection_map() const;
    Projection3d get_3d_projection_map() const;

    Figure& set_bounding_box(const GraphicsBoundingBoxType&);
    
    GraphicsBoundingBoxType get_bounding_box() const;
    //! Set the displayed coordinates to (\a i, \a j)
    Figure& set_projection(DimensionType i, DimensionType j);
    Figure& set_projection(DimensionType as, DimensionType ix, DimensionType iy);
    Figure& set_projection(DimensionType as, DimensionType ix, DimensionType iy, DimensionType iz);
    
    //! Set the radiues to draw points (dots).
    Figure& set_dot_radius(Dbl);
    Figure& set_line_style(Bool);
    //! Set the width to draw lines. A width of 0 means no lines are drawn.
    Figure& set_line_width(Dbl);
    //! Set the colour draw lines
    Figure& set_line_colour(Colour);
    Figure& set_fill_style(Bool);
    //! Set the colour to fill shapes.
    Figure& set_fill_colour(Colour);

    Figure& set_line_colour(Dbl, Dbl, Dbl);
    Figure& set_fill_colour(Dbl, Dbl, Dbl);
    //! Set the opacity of shapes. An opacity of 0 means no fill.
    Figure& set_fill_opacity(Dbl);

    Bool get_line_style() const;
    Dbl get_line_width() const;
    Dbl get_dot_radius() const;
    Colour get_line_colour() const;
    Bool get_fill_style() const;
    Dbl get_fill_opacity() const;
    Colour get_fill_colour() const;

  

    GraphicsProperties& properties();
    GraphicsProperties const& properties() const;

    //! Add a set to draw onto the figure.
    Figure& draw(const Drawable2dInterface& shape);
    Figure& draw(const ApproximateBoxType& box);
    Figure& draw(const RealBox& box);

    Figure& draw(const Drawable2d3dInterface& shape);

    //! Clear the figure.
    Figure& clear();
    //! Set animation
    Figure& set_animated(Bool b);
    //! Display the figure.
    Void display() const;

    //! Write to \a filename.
    Void write(const Char* filename) const;

    //! Write out to file, using width \a nx pixels, and height \a ny pixels
    Void write(const Char* filename, Nat nx, Nat ny) const;

  public:
    struct Data;
  public:
    Void _paint_all(CanvasInterface& canvas) const;
    Void _paint3d(CanvasInterface& canvas) const;    
  
  private:
    Data* _data;
};


Void draw(Figure& fig, const ApproximateBoxType& box);
inline Figure& operator<<(Figure& fig, const ApproximateBoxType& box) { fig.draw(box); return fig; }

inline Figure& operator<<(Figure& g, const LineStyle& ls) { g.properties().set_line_style(ls); return g; }
inline Figure& operator<<(Figure& g, const LineWidth& lw) { g.properties().set_line_width(lw); return g; }
inline Figure& operator<<(Figure& g, const LineColour& lc) { g.properties().set_line_colour(lc); return g; }
inline Figure& operator<<(Figure& g, const FillStyle& fs) { g.properties().set_fill_style(fs); return g; }
inline Figure& operator<<(Figure& g, const FillOpacity& fo) { g.properties().set_fill_opacity(fo); return g; }
inline Figure& operator<<(Figure& g, const FillColour& fc) { g.properties().set_fill_colour(fc); return g; }
inline Figure& operator<<(Figure&g, const Animated& animation) {g.properties().set_animated(animation); return g; }

template<class S> class LabelledSet;

//! \brief Class for plotting figures.
class LabelledFigure {
  public:
    ~LabelledFigure();

    //! Construct a figure drawing the given coordinates in the given bounds.
    LabelledFigure(const Axes2d& axes);

    LabelledFigure(const Axes3d& axes);

    LabelledFigure(const Variables2d& vars, VariablesBox<ApproximateIntervalType> const& bnds);

    LabelledFigure(const Variables3d& vars, VariablesBox<ApproximateIntervalType> const& bnds);

    Void set_axes(const Axes2d& axes);
    Void set_axes(const Axes3d& axes);
//    Void set_bounds(const RealVariable& x, const ApproximateDouble& l, const ApproximateDouble& u);
    Void set_bounds(const RealVariable& x, const ApproximateDoubleInterval& ivl);
    Void set_bounds(const Map<RealVariable,ApproximateDoubleInterval>& b);

    Void set_bounding_box(VariablesBox<ApproximateIntervalType> const& bnds);

    GraphicsProperties& properties();
    GraphicsProperties const& properties() const;

    //! Add a set to draw onto the figure.
    LabelledFigure& draw(const LabelledDrawable2dInterface& shape);
    LabelledFigure& draw(const LabelledDrawable2d3dInterface& shape);

    //! Clear the figure.
    LabelledFigure& clear();

    LabelledFigure& set_animated(Bool b);

    //! Display the figure.
    Void display() const;
    //! Write out to file, using width \a nx pixels, and height \a ny pixels
    Void write(const Char* filename, Nat nx, Nat ny) const;
    //! Write to \a filename.
    Void write(const Char* filename) const;
  
  public:
    struct Data;
  public:
    Void _paint_all(CanvasInterface& canvas) const; // Writes all shapes to the canvas
    Void _paint3d(CanvasInterface& canvas) const;
  private:
    Data* _data;
};

inline LabelledFigure& operator<<(LabelledFigure& g, const LineStyle& ls) { g.properties().set_line_style(ls); return g; }
inline LabelledFigure& operator<<(LabelledFigure& g, const LineWidth& lw) { g.properties().set_line_width(lw); return g; }
inline LabelledFigure& operator<<(LabelledFigure& g, const LineColour& lc) { g.properties().set_line_colour(lc); return g; }
inline LabelledFigure& operator<<(LabelledFigure& g, const FillStyle& fs) { g.properties().set_fill_style(fs); return g; }
inline LabelledFigure& operator<<(LabelledFigure& g, const FillOpacity& fo) { g.properties().set_fill_opacity(fo); return g; }
inline LabelledFigure& operator<<(LabelledFigure& g, const FillColour& fc) { g.properties().set_fill_colour(fc); return g; }
inline LabelledFigure& operator<<(LabelledFigure&g, const Animated& animation) {g.properties().set_animated(animation); return g; }


template<class S> class LabelledSet;
template<class S> class LabelledDrawableWrapper : public LabelledDrawable2dInterface {
    LabelledSet<S> _lset;
  public:
    LabelledDrawableWrapper(LabelledSet<S> lset) : _lset(lset) { }
    virtual LabelledDrawableWrapper<S>* clone() const { return new LabelledDrawableWrapper<S>(*this); }
    virtual Void draw(CanvasInterface& cnvs, Variables2d const& vars) const {
        Projection2d prj(this->_lset.euclidean_set().dimension(),this->_lset.space()[vars.x()],this->_lset.space()[vars.y()]);
        this->_lset.euclidean_set().draw(cnvs,prj);
    }
};
template<class S> inline LabelledFigure& operator<<(LabelledFigure& g, const LabelledSet<S>& lset) {
    return g << LabelledDrawableWrapper<S>(lset);
}

Projection2d projection(const RealSpace& space, const Variables2d& variables);

inline Void draw(Figure& g) { }

template<class SET, class... CSETS> inline Void
draw(Figure& g, const Colour& fc, const SET& set, CSETS const& ... csets) {
    g.set_fill_colour(fc); draw(g,set); draw(g,csets...); }

template<class... CSETS> Void
plot(const char* filename, const Projection2d& pr, const ApproximateBoxType& bbox, CSETS const&... csets) {
    Figure g(bbox,pr); draw(g,csets...);  g.write(filename); }

template<class... CSETS> Void
plot(const char* filename, const ApproximateBoxType& bbox, CSETS const&... csets) {
    plot(filename, Projection2d(2u,0,1), bbox, csets...); }

Void plot(const char* filename, const Projection2d& pr, const ApproximateBoxType& bbox, List<Pair<Colour,Drawable2dInterface const&>> const& csets);

} // namespace Ariadne

#endif // ARIADNE_FIGURE_HPP

