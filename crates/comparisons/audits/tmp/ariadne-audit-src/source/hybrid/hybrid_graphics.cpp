/***************************************************************************
 *            hybrid/hybrid_graphics.cpp
 *
 *  Copyright  2011-20  Pieter Collins
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

#include "function/functional.hpp"
#include "config.hpp"

#include "utility/macros.hpp"
#include "helper/stlio.hpp"
#include "numeric/numeric.hpp"
#include "symbolic/space.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"
#include "io/geometry2d.hpp"
#include "io/figure.hpp"
#include "io/graphics_manager.hpp"
#include "hybrid/discrete_location.hpp"
#include "geometry/function_set.hpp"
#include "symbolic/expression_set.hpp"
#include "hybrid/hybrid_graphics.hpp"

namespace Ariadne {

static const Nat HYBRID_DEFAULT_WIDTH = 800;
static const Nat HYBRID_DEFAULT_HEIGHT = 800;

static const Nat HYBRID_LEFT_MARGIN = 0;
static const Nat HYBRID_BOTTOM_MARGIN = 0;
static const Nat HYBRID_TOP_MARGIN = 0;
static const Nat HYBRID_RIGHT_MARGIN = 0;


HybridFigure::~HybridFigure() {
}


HybridFigure::HybridFigure()
    : variables(RealVariable("x"),RealVariable("y"))
{
}

Void
HybridFigure::write(const char* cfilename) const
{
    this->write(cfilename, HYBRID_DEFAULT_WIDTH, HYBRID_DEFAULT_HEIGHT);
}


Void
HybridFigure::write(const char* cfilename, Nat drawing_width, Nat drawing_height) const
{
    #if not(defined(HAVE_CAIRO_H)) and not(defined(HAVE_GNUPLOT_H))
        ARIADNE_ERROR("No facilities for displaying graphics are available.");
    #else
        const Nat canvas_width = drawing_width+HYBRID_LEFT_MARGIN+HYBRID_RIGHT_MARGIN;
        const Nat canvas_height = drawing_height+HYBRID_BOTTOM_MARGIN+HYBRID_TOP_MARGIN;

        SharedPointer<CanvasInterface> canvas=GraphicsManager::instance().backend().make_canvas(cfilename, canvas_width, canvas_height, this->properties.is_animated);

        this->_paint_all(*canvas);
        canvas->write(cfilename);
    #endif
}

Void HybridFigure::_paint_all(CanvasInterface& canvas) const
{
    // Project the bounding box onto the canvas
    double xl=numeric_cast<double>(bounds[variables.x()].lower_bound());
    double xu=numeric_cast<double>(bounds[variables.x()].upper_bound());
    double yl=numeric_cast<double>(bounds[variables.y()].lower_bound());
    double yu=numeric_cast<double>(bounds[variables.y()].upper_bound());

    canvas.initialise(variables.x().name(),variables.y().name(),xl,xu,yl,yu);

    // Draw shapes
    for(SizeType i=0; i!=objects.size(); ++i) {
        const HybridDrawableInterface& shape=*objects[i].shape_ptr;
        set_properties(canvas, objects[i].properties);
        shape.draw(canvas,this->locations,this->variables);
    }

    canvas.finalise();
}



} // namespace Ariadne


