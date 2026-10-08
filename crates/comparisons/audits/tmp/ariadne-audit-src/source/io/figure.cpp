/***************************************************************************
 *            io/figure.cpp
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

#include "utility/standard.hpp"
#include "config.hpp"

#include "utility/macros.hpp"
#include "helper/stlio.hpp"
#include "numeric/numeric.hpp"
#include "function/function.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"
#include "symbolic/variable.hpp"
#include "symbolic/space.hpp"
#include "symbolic/expression_set.hpp"
#include "io/geometry2d.hpp"
#include "io/figure.hpp"
#include "conclog/logging.hpp"
#include "io/progress_indicator.hpp"
#include "io/graphics_manager.hpp"

using namespace ConcLog;

namespace Ariadne {

static const Int DEFAULT_WIDTH = 800;
static const Int DEFAULT_HEIGHT = 800;

Colour::Colour()
    : Colour("transparant", 1.0, 1.0, 1.0, 0.0) { }
Colour::Colour(Dbl rd, Dbl gr, Dbl bl, Dbl op)
    : Colour("",rd,gr,bl,op) { }
Colour::Colour(const char* nm, Dbl rd, Dbl gr, Dbl bl, Dbl op)
    : name(nm), red(rd), green(gr), blue(bl), opacity(op) { }
OutputStream& operator<<(OutputStream& os, const Colour& c) {
    return os << "Colour( name=" << c.name << ", r=" << c.red << ", g=" << c.green << ", b=" << c.blue << ", op=" << c.opacity << " )"; }


const Colour transparent=Colour();

const Colour white=Colour("white",1.0,1.0,1.0);
const Colour black=Colour("black",0.0,0.0,0.0);
const Colour red=Colour("red",1.0,0.0,0.0);
const Colour green=Colour("green",0.0,1.0,0.0);
const Colour blue=Colour("blue",0.0,0.0,1.0);
const Colour yellow=Colour("yellow",1.0,1.0,0.0);
const Colour cyan=Colour("cyan",0.0,1.0,1.0);
const Colour magenta=Colour("magenta",1.0,0.0,1.0);
const Colour orange=Colour("orange",1.0,0.75,0.5);
const Colour grey=Colour("grey",0.5,0.5,0.5);
const Colour lightgrey=Colour("lightgrey",0.75,0.75,0.75);
const Colour darkgrey=Colour("darkgrey",0.25,0.25,0.25);

GraphicsProperties& GraphicsProperties::set_dot_radius(Dbl dr) { this->dot_radius=dr; return *this; }
GraphicsProperties& GraphicsProperties::set_line_style(Bool ls) { this->line_style=ls; return *this; }
GraphicsProperties& GraphicsProperties::set_line_width(Dbl lw) { this->line_width=lw; return *this; }
GraphicsProperties& GraphicsProperties::set_line_colour(Colour lc) { this->line_colour=lc; return *this; }
GraphicsProperties& GraphicsProperties::set_line_colour(Dbl r, Dbl g, Dbl b) {
    this->set_line_colour(Colour(r,g,b)); return *this; }
GraphicsProperties& GraphicsProperties::set_fill_style(Bool fs) { this->fill_style=fs; return *this; }
GraphicsProperties& GraphicsProperties::set_fill_opacity(Dbl fo) { this->fill_colour.opacity=fo; return *this; }
GraphicsProperties& GraphicsProperties::set_fill_colour(Colour fc) { this->fill_colour=fc; return *this; }
GraphicsProperties& GraphicsProperties::set_fill_colour(Dbl r, Dbl g, Dbl b) {
    this->set_fill_colour(Colour(r,g,b,this->fill_colour.opacity)); return *this; }
GraphicsProperties& GraphicsProperties::set_animated(Bool b) { this->is_animated=b; return *this; }

OutputStream& operator<<(OutputStream& os, GraphicsProperties const& gp) {
    return os << "GraphicsProperties(" << "dot_radius=" << gp.dot_radius
        << ", line_style=" << gp.line_style<<", line_width=" << gp.line_width << ", line_colour=" << gp.line_colour
        << ", fill_style=" << gp.fill_style << ", fill_colour=" << gp.fill_colour << "is_animated" << gp.is_animated << ")"; }

Variables2d::Variables2d(const RealVariable& x, const RealVariable& y) : _x(x.name()), _y(y.name()) { }

Variables3d::Variables3d(const RealVariable& x, const RealVariable& y, const RealVariable& z) : _x(x.name()), _y(y.name()), _z(z.name()) { }

RealVariable Variables2d::x() const { return RealVariable(_x); }

RealVariable Variables2d::y() const { return RealVariable(_y); }

RealVariable Variables3d::x() const { return RealVariable(_x); }

RealVariable Variables3d::y() const { return RealVariable(_y); }

RealVariable Variables3d::z() const { return RealVariable(_z); }

Axes2d::Axes2d(const ApproximateDoubleVariableInterval x, const ApproximateDoubleVariableInterval& y)
        : variables(x.variable(),y.variable()), bounds() {
    bounds.insert(x.variable(),x.interval());
    bounds.insert(y.variable(),y.interval());
}

Axes2d::Axes2d(ApproximateDouble xl, const RealVariable& x, ApproximateDouble xu, ApproximateDouble yl, const RealVariable& y, ApproximateDouble yu)
        : variables(x,y), bounds() {
    bounds.insert(x,ApproximateDoubleInterval(xl,xu));
    bounds.insert(y,ApproximateDoubleInterval(yl,yu));
}

Axes3d::Axes3d(const ApproximateDoubleVariableInterval x, const ApproximateDoubleVariableInterval y, const ApproximateDoubleVariableInterval z)
        : variables3d(x.variable(), y.variable(), z.variable()), bounds() {
    bounds.insert(x.variable(), x.interval());
    bounds.insert(y.variable(), y.interval());
    bounds.insert(z.variable(), z.interval());
}

Axes3d::Axes3d(ApproximateDouble xl, const RealVariable x, ApproximateDouble xu, ApproximateDouble yl, const RealVariable y, ApproximateDouble yu, ApproximateDouble zl, const RealVariable z, ApproximateDouble zu)
        : variables3d(x, y, z), bounds() {
    bounds.insert(x, ApproximateDoubleInterval(xl, xu));
    bounds.insert(y, ApproximateDoubleInterval(yl, yu));
    bounds.insert(z, ApproximateDoubleInterval(zl, zu));
}

Bool valid_axis_variables(const RealSpace& space, const Variables2d& variables) {
    return ( (variables.x().name()==TimeVariable().name()) || space.contains(variables.x()) ) && space.contains(variables.y());
}

Projection2d projection(const RealSpace& space, const Variables2d& variables) {
    ARIADNE_ASSERT(valid_axis_variables(space,variables));
    Nat x_index = (variables.x()==TimeVariable() && !space.contains(variables.x())) ? space.dimension() : space.index(variables.x());
    Nat y_index = space.index(variables.y());
    return Projection2d(space.dimension(),x_index,y_index);
}


Void draw(Figure& fig, const Drawable2dInterface& shape) {
    fig.draw(shape);
}

Void draw(Figure& fig, const Drawable2d3dInterface& shape) {
    fig.draw(shape);
}

Void draw(Figure& fig, FloatDPApproximateBox const& box) {
    fig.draw(box);
}

struct GraphicsObject {
    GraphicsObject(const GraphicsProperties& gp, const Drawable2dInterface& sh)
        : properties(gp), shape_ptr(sh.clone()), shape3d_ptr(nullptr) { }
    GraphicsObject(const GraphicsProperties& gp, const Drawable2d3dInterface& sh)
        : properties(gp), shape_ptr(sh.clone()), shape3d_ptr(sh.clone2d3d()) { }
    GraphicsProperties properties;
    std::shared_ptr<const Drawable2dInterface> shape_ptr;
    std::shared_ptr<const Drawable2d3dInterface> shape3d_ptr;
};

struct LabelledGraphicsObject {
    LabelledGraphicsObject(const GraphicsProperties& gp, const LabelledDrawable2dInterface& sh)
        : properties(gp), shape_ptr(sh.clone()), shape3d_ptr(nullptr) { }
    LabelledGraphicsObject(const GraphicsProperties& gp, const LabelledDrawable2d3dInterface& sh)
        : properties(gp), shape_ptr(sh.clone()), shape3d_ptr(sh.clone2d3d()) { }
    GraphicsProperties properties;
    std::shared_ptr<const LabelledDrawable2dInterface> shape_ptr;
    std::shared_ptr<const LabelledDrawable2d3dInterface> shape3d_ptr;
};

struct Figure::Data
{
    Data() : properties(), projection(2,0,1), projection3d(3,0,1,2),bounding_box(0), variables(RealVariable(""),RealVariable("")), variables3d(RealVariable(""), RealVariable(""), RealVariable("")) { }
    Data(ApproximateBoxType bbx, Projection2d prj) : properties(), projection(prj), projection3d(3, 0, 1, 2),bounding_box(bbx),  variables(RealVariable(""),RealVariable("")), variables3d(RealVariable(""), RealVariable(""), RealVariable("")) { }
    Data(ApproximateBoxType bbx, Projection3d prj) : properties(), projection(2, 0, 1), projection3d(prj), bounding_box(bbx), variables(RealVariable(""), RealVariable("")), variables3d(RealVariable(""), RealVariable(""), RealVariable("")) { }

    GraphicsProperties properties;

    Projection2d projection;
    Projection3d projection3d;
    ApproximateBoxType bounding_box;
    List<GraphicsObject> objects;

    Variables2d variables;
    Variables3d variables3d;
    Map<RealVariable,ApproximateDoubleInterval> bounds;
    List<LabelledGraphicsObject> labelled_objects;


};

Figure::~Figure()
{
    delete this->_data;
}

Figure::Figure(const GraphicsBoundingBoxType& bbx, const Projection2d& proj)
    : _data(new Data(bbx,proj))
{
    ARIADNE_ASSERT_MSG(proj.argument_size() == bbx.dimension(), "Coordinate projection "<<proj<<" must take same number of arguments as the dimension of the bounding box "<<bbx);
    if(proj.argument_size() > 2){ this->_data->properties.is_projected = true; }
}

Figure::Figure(const GraphicsBoundingBoxType& bbx, DimensionType ix, DimensionType iy)
    : Figure(bbx,Projection2d(bbx.dimension(),ix,iy))
{
    ARIADNE_ASSERT_MSG(bbx.dimension()>=2,"The bounding box must be at least 2-dimensional.");
}

Figure::Figure(const GraphicsBoundingBoxType& bbx, Pair<DimensionType,DimensionType> ixy)
    : Figure(bbx,Projection2d(bbx.dimension(),ixy.first,ixy.second))
{
    ARIADNE_ASSERT_MSG(bbx.dimension()>=2,"The bounding box must be at least 2-dimensional.");
}

Figure::Figure(const GraphicsBoundingBoxType& bbx, const Projection3d& proj)
    : _data(new Data(bbx, proj))
{
    ARIADNE_ASSERT_MSG(proj.argument_size() == bbx.dimension(), "Coordinate projection "<<proj<<" must take same number of arguments as the dimension of the bounding box "<<bbx);
}

Figure::Figure(const GraphicsBoundingBoxType& bbx, DimensionType ix, DimensionType iy, DimensionType iz)
    : Figure(bbx, Projection3d(bbx.dimension(), ix, iy, iz))
{
    ARIADNE_ASSERT_MSG(bbx.dimension()>=3,"The bounding box must be at least 3-dimensional.");
}

Figure& Figure::draw(const Drawable2dInterface& shape)
{
    this->_data->objects.push_back(GraphicsObject(this->_data->properties,shape)); return *this;
}

Figure& Figure::draw(const Drawable2d3dInterface& shape)
{
    if (this->_data->bounding_box.size() == 3) { this->_data->properties.is_3d = true; }

    this->_data->objects.push_back(GraphicsObject(this->_data->properties,shape)); return *this;
}

Figure& Figure::set_projection(DimensionType as, DimensionType ix, DimensionType iy)
{
    ARIADNE_ASSERT_MSG(as==this->get_bounding_box().dimension(),"The bounding box must have the same argument size as the projection.");
    this->_data->projection=Projection2d(as,ix,iy); return *this;
}

Figure& Figure::set_projection(DimensionType as, DimensionType ix, DimensionType iy, DimensionType iz)
{
    ARIADNE_ASSERT_MSG(as==this->get_bounding_box().dimension(),"The bounding box must have the same argument size as the projection.");
    this->_data->projection3d=Projection3d(as, ix,iy,iz); return *this;
}

Figure& Figure::set_projection_map(const Projection2d& p)
{
    ARIADNE_ASSERT_MSG(p.argument_size()==this->get_bounding_box().dimension(),"The bounding box must have the same argument size as the projection.");
    this->_data->projection=p; return *this;
}

Figure& Figure::set_projection_map(const Projection3d& p)
{
    ARIADNE_ASSERT_MSG(p.argument_size()==this->get_bounding_box().dimension(),"The bounding box must have the same argument size as the projection.");
    this->_data->projection3d=p; return *this;
}

Figure& Figure::set_bounding_box(const ApproximateBoxType& bx)
{
    this->_data->bounding_box=bx; return *this;
}

Projection2d Figure::get_2d_projection_map() const
{
    return this->_data->projection;
}

Projection3d Figure::get_3d_projection_map() const
{
    return this->_data->projection3d;
}

ApproximateBoxType Figure::get_bounding_box() const
{
    return this->_data->bounding_box;
}

GraphicsProperties& Figure::properties() {
    return this->_data->properties;
}

const GraphicsProperties& Figure::properties() const {
    return this->_data->properties;
}

Figure& Figure::set_dot_radius(Dbl dr)
{
    this->_data->properties.dot_radius=dr; return *this;
}

Figure& Figure::set_line_style(Bool ls)
{
    this->_data->properties.line_style=ls; return *this;
}

Figure& Figure::set_line_width(Dbl lw)
{
    this->_data->properties.line_width=lw; return *this;
}

Figure& Figure::set_line_colour(Colour lc)
{
    this->_data->properties.line_colour=lc; return *this;
}

Figure& Figure::set_line_colour(Dbl r, Dbl g, Dbl b)
{
    this->set_line_colour(Colour(r,g,b)); return *this;
}

Figure& Figure::set_fill_style(Bool fs)
{
    this->_data->properties.fill_style=fs; return *this;
}

Figure& Figure::set_fill_opacity(Dbl fo)
{
    this->_data->properties.fill_colour.opacity=fo; return *this;
}

Figure& Figure::set_fill_colour(Colour fc)
{
    this->_data->properties.fill_colour=fc; return *this;
}

Figure& Figure::set_fill_colour(Dbl r, Dbl g, Dbl b)
{
    this->set_fill_colour(Colour(r,g,b,this->_data->properties.fill_colour.opacity)); return *this;
}

Bool Figure::get_line_style() const
{
    return this->_data->properties.line_style;
}

Dbl Figure::get_line_width() const
{
    return this->_data->properties.line_width;
}

Dbl Figure::get_dot_radius() const
{
    return this->_data->properties.dot_radius;
}

Colour Figure::get_line_colour() const
{
    return this->_data->properties.line_colour;
}


Bool Figure::get_fill_style() const
{
    return this->_data->properties.fill_style;
}

Dbl Figure::get_fill_opacity() const
{
    return this->_data->properties.fill_colour.opacity;
}

Colour Figure::get_fill_colour() const
{
    return this->_data->properties.fill_colour;
}

Figure& Figure::draw(ApproximateBoxType const& box)
{
    ApproximateBoxSetType box_set(box);
    Drawable2dInterface const& shape=box_set;
    this->draw(shape); return *this;
}

Figure& Figure::draw(RealBox const& box)
{
    return this->draw(ApproximateBoxType(box,DoublePrecision()));
}

Figure& Figure::clear() {
    this->_data->objects.clear(); return *this;
}

Figure& Figure::set_animated(Bool b){
    this->_data->properties.is_animated = b; return *this;
}

Figure& operator<<(Figure& fig, const Drawable2dInterface& shape) {
    fig.draw(shape); return fig;
}

Void set_properties(CanvasInterface& canvas, const GraphicsProperties& properties) {
    const Colour& line_colour=properties.line_colour;
    const Colour& fill_colour=properties.fill_colour;
    if (properties.line_style==false) { canvas.set_line_width(0); }
    else { canvas.set_line_width(properties.line_width); }
    if (properties.fill_style==false) { canvas.set_fill_opacity(0); }
    else { canvas.set_fill_opacity(properties.fill_colour.opacity); }
    canvas.set_fill_colour(fill_colour.red, fill_colour.green, fill_colour.blue);
    canvas.set_line_colour(line_colour.red, line_colour.green, line_colour.blue);
}

Void Figure::_paint3d(CanvasInterface& canvas) const
{
    DimensionType dimension=this->_data->projection.argument_size();
    ApproximateBoxType bounding_box=this->_data->bounding_box;
    const Projection3d projection=this->_data->projection3d;
    if(bounding_box.dimension()==0) {
        bounding_box=ExactBoxType(dimension,ExactIntervalType(-1,1));
    }
    ARIADNE_ASSERT_MSG(bounding_box.dimension()==projection.argument_size(),"bounding_box="<<bounding_box<<", projection="<<projection);
    ARIADNE_ASSERT(bounding_box.dimension()>projection.x_coordinate());
    ARIADNE_ASSERT(bounding_box.dimension()>projection.y_coordinate());
    ARIADNE_ASSERT(bounding_box.dimension()>projection.z_coordinate());
    Dbl xl=numeric_cast<Dbl>(bounding_box[projection.x_coordinate()].lower_bound());
    Dbl xu=numeric_cast<Dbl>(bounding_box[projection.x_coordinate()].upper_bound());
    Dbl yl=numeric_cast<Dbl>(bounding_box[projection.y_coordinate()].lower_bound());
    Dbl yu=numeric_cast<Dbl>(bounding_box[projection.y_coordinate()].upper_bound());
    Dbl zl=numeric_cast<Dbl>(bounding_box[projection.z_coordinate()].lower_bound());
    Dbl zu=numeric_cast<Dbl>(bounding_box[projection.z_coordinate()].upper_bound());
    String tx=String("x")+to_str(projection.x_coordinate());
    String ty=String("x")+to_str(projection.y_coordinate());
    String tz=String("x")+to_str(projection.z_coordinate());
    canvas.initialise(tx, ty, tz, xl, xu, yl, yu, zl, zu);  
    canvas.set_colour_palette(); 
    for(const GraphicsObject& object : this->_data->objects) {
        if(object.shape3d_ptr != nullptr)
        {
            const Drawable2d3dInterface& shape=object.shape3d_ptr.operator*();
            set_properties(canvas, object.properties);
            shape.draw(canvas, this->_data->projection3d); 
        }else{
            ARIADNE_ERROR("ERROR: Cannot draw a 2D object in a 3D graphic");
        }
    }      
}

Void Figure::_paint_all(CanvasInterface& canvas) const
{
    DimensionType dimension=this->_data->projection.argument_size();

    ApproximateBoxType bounding_box=this->_data->bounding_box;
    const Projection2d projection=this->_data->projection;

    // Don't attempt to compute a bounding box, as this relies on
    // a drawable object having one. Instead, the bounding box must be
    // specified explicitly
    if(bounding_box.dimension()==0) {
        bounding_box=ExactBoxType(dimension,ExactIntervalType(-1,1));
    }

    // Check projection and bounding box have same values.
    ARIADNE_ASSERT_MSG(bounding_box.dimension()==projection.argument_size(),"bounding_box="<<bounding_box<<", projection="<<projection);
    ARIADNE_ASSERT(bounding_box.dimension()>projection.x_coordinate());
    ARIADNE_ASSERT(bounding_box.dimension()>projection.y_coordinate());

    // Project the bounding box onto the canvas
    Dbl xl=numeric_cast<Dbl>(bounding_box[projection.x_coordinate()].lower_bound());
    Dbl xu=numeric_cast<Dbl>(bounding_box[projection.x_coordinate()].upper_bound());
    Dbl yl=numeric_cast<Dbl>(bounding_box[projection.y_coordinate()].lower_bound());
    Dbl yu=numeric_cast<Dbl>(bounding_box[projection.y_coordinate()].upper_bound());

    String tx=String("x")+to_str(projection.x_coordinate());
    String ty=String("x")+to_str(projection.y_coordinate());

    canvas.initialise(tx,ty,xl,xu,yl,yu);
    if(this->_data->properties.is_projected) { canvas.set_colour_palette(); }

    // Draw shapes
    for(const GraphicsObject& object : this->_data->objects) {

        const Drawable2dInterface& shape=object.shape_ptr.operator*();
        if(shape.dimension()==0) { break; } // The dimension may be equal to two for certain empty sets.
        ARIADNE_ASSERT(dimension==shape.dimension());
        set_properties(canvas, object.properties);
        shape.draw(canvas,this->_data->projection);
    }
}

Void
Figure::write(const Char* cfilename) const
{
    this->write(cfilename, DEFAULT_WIDTH, DEFAULT_HEIGHT);
}
  
Void
Figure::write(const Char* cfilename, Nat drawing_width, Nat drawing_height) const
{
    #if not(defined(HAVE_CAIRO_H)) and not(defined(HAVE_GNUPLOT_H))
        ARIADNE_ERROR("No facilities for displaying graphics are available.");
    #else
        SharedPointer<CanvasInterface> canvas=GraphicsManager::instance().backend().make_canvas(cfilename,drawing_width,drawing_height, this->_data->properties.is_animated);
        
        if(this->_data->properties.is_3d && this->_data->properties.is_projected == false){
            this->_paint3d(*canvas);
        }else{
            this->_paint_all(*canvas);
        }

        canvas->write(cfilename);
    #endif
}

struct LabelledFigure::Data
{
    Data(Axes2d axes)
        : properties(), variables(axes.variables), variables3d(axes.variables.x(),axes.variables.y(), RealVariable(Identifier("z"))), bounds(axes.bounds) { }
    Data(const Variables2d& vars, const Map<RealVariable,ApproximateDoubleInterval>& bnds)
        : properties(), variables(vars), variables3d(vars.x(), vars.y(), RealVariable(Identifier("z"))) ,bounds(bnds) { }
    Data(Axes3d axes)
        : properties(), variables(axes.variables3d.x(), axes.variables3d.y()) ,variables3d(axes.variables3d), bounds(axes.bounds) { }
    Data(const Variables3d& vars, const Map<RealVariable,ApproximateDoubleInterval>& bnds)
        : properties(), variables(vars.x(), vars.y()) ,variables3d(vars), bounds(bnds) { }

    GraphicsProperties properties;

    Variables2d variables;
    Variables3d variables3d;
    Map<RealVariable,ApproximateDoubleInterval> bounds;
    List<LabelledGraphicsObject> objects;

};

LabelledFigure::~LabelledFigure() {
    delete this->_data;
}

LabelledFigure::LabelledFigure(const Axes2d& axes)
    : _data(new Data(axes)) {
}

LabelledFigure::LabelledFigure(const Axes3d& axes) 
    : _data(new Data(axes)) {
        this->_data->properties.is_3d = true;
}

LabelledFigure::LabelledFigure(const Variables2d& vars, const VariablesBox<ApproximateIntervalType>& bbx)
    : _data(new Data(vars,VariablesBox<ApproximateDoubleInterval>(bbx).bounds())) {
    ARIADNE_ASSERT_MSG(bbx.bounds().size()>=2,"The box must be at least two-dimensional.")
}

LabelledFigure::LabelledFigure(const Variables3d& vars, const VariablesBox<ApproximateIntervalType>& bbx)
    : _data(new Data(vars, VariablesBox<ApproximateDoubleInterval>(bbx).bounds())){
    this->_data->properties.is_3d = true;
    ARIADNE_ASSERT_MSG(bbx.bounds().size()>=3,"The box must be at least three-dimensional.")
}

Void LabelledFigure::set_axes(const Axes2d& axes) {
    this->_data->properties.is_3d = false;
    _data->bounds=axes.bounds; _data->variables=axes.variables;
}

Void LabelledFigure::set_axes(const Axes3d& axes) {
    this->_data->properties.is_3d = true;
    _data->bounds=axes.bounds; _data->variables3d=axes.variables3d;
}

Void LabelledFigure::set_bounds(const RealVariable& x, const ApproximateDoubleInterval& ivl) {
    _data->bounds.insert(x,ivl);
}
Void LabelledFigure::set_bounds(const Map<RealVariable,ApproximateDoubleInterval>& b) {
    _data->bounds=b;
}

Void LabelledFigure::set_bounding_box(const VariablesBox<ApproximateIntervalType>& bx) {
    _data->bounds=bx.bounds();
}

GraphicsProperties& LabelledFigure::properties() {
    return this->_data->properties;
}

GraphicsProperties const& LabelledFigure::properties() const {
    return this->_data->properties;
}

LabelledFigure& LabelledFigure::draw(const LabelledDrawable2dInterface& shape)
{
    this->_data->objects.push_back(LabelledGraphicsObject(this->_data->properties,shape)); return *this;
}

LabelledFigure& LabelledFigure::draw(const LabelledDrawable2d3dInterface& shape)
{
    if(this->_data->bounds.size() == 3) { this->_data->properties.is_3d = true; }
    this->_data->objects.push_back(LabelledGraphicsObject(this->_data->properties,shape)); return *this;
}

LabelledFigure& LabelledFigure::clear() {
    this->_data->objects.clear(); return *this;
}

LabelledFigure& LabelledFigure::set_animated(Bool b){
    this->_data->properties.is_animated = b; return *this;
}

LabelledFigure& operator<<(LabelledFigure& fig, const LabelledDrawable2dInterface& shape) {
    fig.draw(shape); return fig;
}

Void LabelledFigure::_paint3d(CanvasInterface& canvas) const
{
    CONCLOG_SCOPE_CREATE;
    auto const& bounds = this->_data->bounds;
    RealVariable const& x=this->_data->variables.x();
    RealVariable const& y=this->_data->variables.y();
    RealVariable const& z=this->_data->variables3d.z();

    String tx=x.name();
    String ty=y.name();
    String tz=z.name();

    Dbl xl=numeric_cast<Dbl>(bounds[x].lower_bound());
    Dbl xu=numeric_cast<Dbl>(bounds[x].upper_bound());
    Dbl yl=numeric_cast<Dbl>(bounds[y].lower_bound());
    Dbl yu=numeric_cast<Dbl>(bounds[y].upper_bound());
    Dbl zl=numeric_cast<Dbl>(bounds[z].lower_bound());
    Dbl zu=numeric_cast<Dbl>(bounds[z].upper_bound());   

    SizeType total_objects = this->_data->objects.size();
    SizeType processed_objects = 0;
    ProgressIndicator indicator(total_objects);

    canvas.initialise(tx, ty, tz, xl, xu, yl, yu, zl, zu);
    canvas.set_colour_palette();

    for(const LabelledGraphicsObject& object : this->_data->objects) {
        if(object.shape3d_ptr != nullptr)
        {
            const LabelledDrawable2d3dInterface& shape=object.shape3d_ptr.operator*();
            set_properties(canvas, object.properties);
            shape.draw(canvas, this->_data->variables3d);
        }
        else{
            ARIADNE_ERROR("ERROR: Cannot draw a 2D object in a 3D graphic");
            break;
        }
        indicator.update_current(processed_objects++);
        CONCLOG_SCOPE_PRINTHOLD("[" << indicator.symbol() << "] " << indicator.percentage() << "% ");
    }

}

Void LabelledFigure::_paint_all(CanvasInterface& canvas) const
{
    CONCLOG_SCOPE_CREATE;
    auto const& bounds = this->_data->bounds;
    RealVariable const& x=this->_data->variables.x();
    RealVariable const& y=this->_data->variables.y();
    
    ARIADNE_ASSERT(x.name()!="" && y.name()!="");

    String tx=x.name();
    String ty=y.name();

    Dbl xl=numeric_cast<Dbl>(bounds[x].lower_bound());
    Dbl xu=numeric_cast<Dbl>(bounds[x].upper_bound());
    Dbl yl=numeric_cast<Dbl>(bounds[y].lower_bound());
    Dbl yu=numeric_cast<Dbl>(bounds[y].upper_bound());

    canvas.initialise(tx,ty,xl,xu,yl,yu);
    if(this->_data->properties.is_projected) { canvas.set_colour_palette(); }
    // Draw shapes
    SizeType total_objects = this->_data->objects.size();
    SizeType processed_objects = 0;
    ProgressIndicator indicator(total_objects);
    CONCLOG_PRINTLN("Writing " << total_objects << " object" << (total_objects > 1 ? "s..." : "..."));
    for(const LabelledGraphicsObject& object : this->_data->objects) {
        const LabelledDrawable2dInterface& shape=object.shape_ptr.operator*();
        set_properties(canvas, object.properties);
        shape.draw(canvas,this->_data->variables);
        indicator.update_current(processed_objects++);
        CONCLOG_SCOPE_PRINTHOLD("[" << indicator.symbol() << "] " << indicator.percentage() << "% ");
    }
    canvas.finalise();
}

Void
LabelledFigure::write(const Char* cfilename) const
{
    this->write(cfilename, DEFAULT_WIDTH, DEFAULT_HEIGHT);
}

Void
LabelledFigure::write(const Char* cfilename, Nat drawing_width, Nat drawing_height) const
{
    #if not(defined(HAVE_CAIRO_H)) and not(defined(HAVE_GNUPLOT_H))
        ARIADNE_ERROR("No facilities for displaying graphics are available.");
    #else
        SharedPointer<CanvasInterface> canvas=GraphicsManager::instance().backend().make_canvas(cfilename,drawing_width,drawing_height, this->_data->properties.is_animated);

        if(this->_data->properties.is_3d && this->_data->properties.is_projected == false){
            this->_paint3d(*canvas);
        }else{
            this->_paint_all(*canvas);
        }

        canvas->write(cfilename);
    #endif
}

Void plot(const char* filename, const Projection2d& pr, const ApproximateBoxType& bbox, List<Pair<Colour,Drawable2dInterface const&>> const& csets) {
    Figure fig(bbox,pr);
    for (auto cset : csets) {
        fig.set_fill_colour(cset.first); fig << cset.second;
    }
    fig.write(filename);
}

} // namespace Ariadne


