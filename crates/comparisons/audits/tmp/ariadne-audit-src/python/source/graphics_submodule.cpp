/***************************************************************************
 *            graphics_submodule.cpp
 *
 *  Copyright  2008-20  Pieter Collins
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

#include "pybind11.hpp"
#include "utilities.hpp"

#include "config.hpp"

#include "io/graphics_interface.hpp"
#include "io/graphics_manager.hpp"
#include "io/drawer.hpp"
#include "io/gnuplot.hpp"
#include "io/cairo.hpp"
#include "io/figure.hpp"
#include "io/geometry2d.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"
#include "symbolic/space.hpp"
#include "function/function.hpp"

using namespace Ariadne;

Void export_point2d(pybind11::module& module) {
    pybind11::class_<Point2d> point2d_class(module, "Point2d");
    point2d_class.def(pybind11::init<double, double>());
    point2d_class.def_readwrite("x", &Point2d::x);
    point2d_class.def_readwrite("y", &Point2d::y);
    point2d_class.def("__repr__", &__cstr__<Point2d>);
}

Void export_colour(pybind11::module& module)
{
    pybind11::class_<Colour> colour_class(module,"Colour");
    colour_class.def(pybind11::init<double,double,double>());

    module.attr("transparent") = transparent;
    module.attr("white") = white;
    module.attr("black") = black;
    module.attr("red") = red;
    module.attr("green") = green;
    module.attr("blue") = blue;
    module.attr("yellow") = yellow;
    module.attr("cyan") = cyan;
    module.attr("magenta") = magenta;
    module.attr("orange") = orange;
    module.attr("grey") = grey;
    module.attr("lightgrey") = lightgrey;
    module.attr("darkgrey") = darkgrey;
}

Void export_figure(pybind11::module& module)
{
    pybind11::class_<Projection2d> planar_projection_map_class(module,"Projection2d");
    planar_projection_map_class.def(pybind11::init<DimensionType,DimensionType,DimensionType>());
    planar_projection_map_class.def_readwrite("n",&Projection2d::n);
    planar_projection_map_class.def_readwrite("i",&Projection2d::i);
    planar_projection_map_class.def_readwrite("j",&Projection2d::j);

    pybind11::class_<Variables2d> variables2d_class(module,"Variables2d");
    variables2d_class.def(pybind11::init<RealVariable,RealVariable>());
    variables2d_class.def("x", &Variables2d::x);
    variables2d_class.def("y", &Variables2d::y);

    module.def("projection",&projection);

    pybind11::class_<Axes2d> axes2d_class(module,"Axes2d");
    axes2d_class.def(pybind11::init<double,RealVariable,double,double,RealVariable,double>());
    axes2d_class.def(pybind11::init<Real,RealVariable,Real,Real,RealVariable,Real>());

    static constexpr auto reference_internal = pybind11::return_value_policy::reference_internal ;

    pybind11::class_<Figure> figure_class(module,"Figure");
    figure_class.def(pybind11::init<GraphicsBoundingBoxType,Projection2d>());
    figure_class.def("set_projection_map",(Figure&(Figure::*)(const Projection2d&)) &Figure::set_projection_map, reference_internal);
    figure_class.def("set_projection",(Figure&(Figure::*)(DimensionType,DimensionType,DimensionType)) &Figure::set_projection, reference_internal);
    figure_class.def("set_bounding_box",&Figure::set_bounding_box, reference_internal);
    figure_class.def("set_dot_radius", (Figure&(Figure::*)(double)) &Figure::set_dot_radius, reference_internal);
    figure_class.def("set_line_style", (Figure&(Figure::*)(Bool)) &Figure::set_line_style, reference_internal);
    figure_class.def("set_line_width", (Figure&(Figure::*)(double)) &Figure::set_line_width, reference_internal);
    figure_class.def("set_line_colour", (Figure&(Figure::*)(double,double,double)) &Figure::set_line_colour, reference_internal);
    figure_class.def("set_fill_style", (Figure&(Figure::*)(Bool)) &Figure::set_fill_style, reference_internal);
    figure_class.def("set_fill_colour", (Figure&(Figure::*)(double,double,double)) &Figure::set_fill_colour, reference_internal);
    figure_class.def("set_fill_opacity", (Figure&(Figure::*)(double)) &Figure::set_fill_opacity, reference_internal);
    figure_class.def("draw",(Figure&(Figure::*)(const Drawable2dInterface&))&Figure::draw, reference_internal);
    figure_class.def("draw",(Figure&(Figure::*)(const RealBox&))&Figure::draw, reference_internal);
    figure_class.def("draw",(Figure&(Figure::*)(const ApproximateBoxType&))&Figure::draw, reference_internal);
    figure_class.def("clear",&Figure::clear, reference_internal);
    figure_class.def("write",(Void(Figure::*)(const Char*)const)&Figure::write);
    figure_class.def("write",(Void(Figure::*)(const Char*, Nat, Nat)const)&Figure::write);

}

Void export_graphics_properties(pybind11::module& module)
{
    static constexpr auto reference_internal = pybind11::return_value_policy::reference_internal;

    pybind11::class_<GraphicsProperties> graphics_properties_class(module,"GraphicsProperties");
    graphics_properties_class.def(pybind11::init<>());
    graphics_properties_class.def("set_dot_radius", (GraphicsProperties&(GraphicsProperties::*)(double)) &GraphicsProperties::set_dot_radius, reference_internal);
    graphics_properties_class.def("set_line_style", (GraphicsProperties&(GraphicsProperties::*)(Bool)) &GraphicsProperties::set_line_style, reference_internal);
    graphics_properties_class.def("set_line_width", (GraphicsProperties&(GraphicsProperties::*)(double)) &GraphicsProperties::set_line_width, reference_internal);
    graphics_properties_class.def("set_line_colour", (GraphicsProperties&(GraphicsProperties::*)(double,double,double)) &GraphicsProperties::set_line_colour, reference_internal);
    graphics_properties_class.def("set_line_colour", (GraphicsProperties&(GraphicsProperties::*)(Colour)) &GraphicsProperties::set_line_colour, reference_internal);
    graphics_properties_class.def("set_fill_style", (GraphicsProperties&(GraphicsProperties::*)(Bool)) &GraphicsProperties::set_fill_style, reference_internal);
    graphics_properties_class.def("set_fill_colour", (GraphicsProperties&(GraphicsProperties::*)(double,double,double)) &GraphicsProperties::set_fill_colour, reference_internal);
    graphics_properties_class.def("set_fill_colour", (GraphicsProperties&(GraphicsProperties::*)(Colour)) &GraphicsProperties::set_fill_colour, reference_internal);
    graphics_properties_class.def("set_fill_opacity", (GraphicsProperties&(GraphicsProperties::*)(double)) &GraphicsProperties::set_fill_opacity, reference_internal);
}

Void export_labelled_figure(pybind11::module& module)
{
    static constexpr auto reference_internal = pybind11::return_value_policy::reference_internal;

    pybind11::class_<LabelledFigure> labelled_figure_class(module,"LabelledFigure");
    labelled_figure_class.def(pybind11::init<Axes2d>());

    labelled_figure_class.def("set_axes",(Void(LabelledFigure::*)(const Axes2d&)) &LabelledFigure::set_axes, reference_internal);
    labelled_figure_class.def("properties",(GraphicsProperties&(LabelledFigure::*)())&LabelledFigure::properties, reference_internal);

    labelled_figure_class.def("draw",(LabelledFigure&(LabelledFigure::*)(const LabelledDrawable2dInterface&))&LabelledFigure::draw, reference_internal);
    labelled_figure_class.def("clear",&LabelledFigure::clear, reference_internal);
    labelled_figure_class.def("write",(Void(LabelledFigure::*)(const Char*)const)&LabelledFigure::write);
    labelled_figure_class.def("write",(Void(LabelledFigure::*)(const Char*,Nat,Nat)const)&LabelledFigure::write);
}

Void export_plot(pybind11::module& module)
{
    module.def("plot",(Void(*)(const char*,Projection2d const&,ApproximateBoxType const&,List<Pair<Colour,Drawable2dInterface const&>> const&)) &plot);
}

Void export_backend(pybind11::module& module) {
    pybind11::class_<GraphicsBackend> backend_class(module,"GraphicsBackend");

    #ifdef HAVE_GNUPLOT_H
        pybind11::class_<GnuplotGraphicsBackend> gnuplot_backend_class(module,"GnuplotGraphicsBackend");
        gnuplot_backend_class.def(pybind11::init<>());
        pybind11::implicitly_convertible<GnuplotGraphicsBackend,GraphicsBackend>();
        backend_class.def(pybind11::init<GnuplotGraphicsBackend>());
    #endif
    #ifdef HAVE_CAIRO_H
        pybind11::class_<CairoGraphicsBackend> cairo_backend_class(module,"CairoGraphicsBackend");
        cairo_backend_class.def(pybind11::init<>());
        pybind11::implicitly_convertible<CairoGraphicsBackend,GraphicsBackend>();
        backend_class.def(pybind11::init<CairoGraphicsBackend>());
    #endif
}

Void export_drawer(pybind11::module& module) {
    pybind11::class_<Drawer> drawer_class(module,"Drawer");

    pybind11::class_<AffineDrawer> affine_drawer_class(module,"AffineDrawer");
    affine_drawer_class.def(pybind11::init<Nat>());
    pybind11::implicitly_convertible<AffineDrawer,Drawer>();
    drawer_class.def(pybind11::init<AffineDrawer>());

    pybind11::class_<GridDrawer> grid_drawer_class(module,"GridDrawer");
    grid_drawer_class.def(pybind11::init<Nat>());
    pybind11::implicitly_convertible<GridDrawer,Drawer>();
    drawer_class.def(pybind11::init<GridDrawer>());

    pybind11::class_<BoxDrawer> box_drawer_class(module,"BoxDrawer");
    box_drawer_class.def(pybind11::init<>());
    pybind11::implicitly_convertible<BoxDrawer,Drawer>();
    drawer_class.def(pybind11::init<BoxDrawer>());
}

Void export_graphics_manager(pybind11::module& module)
{
    auto const& reference = pybind11::return_value_policy::reference;

    pybind11::class_<GraphicsManager> graphics_manager_class(module,"GraphicsManager");
    graphics_manager_class.def_static("instance", &GraphicsManager::instance, reference);
    graphics_manager_class.def("set_backend", &GraphicsManager::set_backend);
    graphics_manager_class.def("set_drawer", &GraphicsManager::set_drawer);
}

Void graphics_submodule(pybind11::module& module) {
    export_point2d(module);
    export_colour(module);
    export_figure(module);
    export_graphics_properties(module);
    export_labelled_figure(module);
    export_plot(module);
    export_backend(module);
    export_drawer(module);
    export_graphics_manager(module);
}

