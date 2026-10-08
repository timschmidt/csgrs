/***************************************************************************
 *            io/cairo.cpp
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
#include "io/geometry2d.hpp"
#include "io/figure.hpp"
#include "io/cairo.hpp"
#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {

#ifdef HAVE_CAIRO_H

static const Int LEFT_MARGIN = 160;
static const Int BOTTOM_MARGIN = 40;
static const Int TOP_MARGIN = 10;
static const Int RIGHT_MARGIN = 10;

SharedPointer<CanvasInterface> CairoGraphicsBackend::make_canvas(const char* cfilename, Nat drawing_width, Nat drawing_height, Bool is_animated) const {
    return std::make_shared<CairoCanvas>(ImageSize2d(drawing_width,drawing_height));
}

CairoCanvas::~CairoCanvas()
{
    cairo_surface_destroy(cairo_get_target(cr));
    cairo_destroy(cr);
}

CairoCanvas::CairoCanvas(const ImageSize2d& size)
    : cr(0), lw(1.0), dr(1.0), lc(0.0,0.0,0.0), fc(1.0,1.0,1.0, 1.0)
{
    const Int canvas_width = static_cast<Int>(size.nx)+LEFT_MARGIN+RIGHT_MARGIN;
    const Int canvas_height = static_cast<Int>(size.ny)+BOTTOM_MARGIN+TOP_MARGIN;

    cairo_surface_t* surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, canvas_width, canvas_height);
    cr = cairo_create (surface);
    
}

Void CairoCanvas::stroke()
{
    cairo_save(cr);

    // Set user and device space identical so that the line width is interpreted as pixels
    cairo_identity_matrix(cr);

    cairo_set_source_rgb(cr, lc.red,lc.green,lc.blue);
    cairo_set_line_width(cr, lw);
    cairo_stroke (this->cr);

    cairo_restore(cr);
}

Void CairoCanvas::fill() {
    cairo_set_source_rgba(cr,fc.red,fc.green,fc.blue,fc.opacity);
    cairo_fill_preserve (cr);
    this->stroke();
}

ImageSize2d CairoCanvas::size_in_pixels() const {
    return ImageSize2d(cairo_image_surface_get_width(cairo_get_target(cr))-(LEFT_MARGIN+RIGHT_MARGIN),
                       cairo_image_surface_get_height(cairo_get_target(cr))-(BOTTOM_MARGIN+TOP_MARGIN));
}

Void CairoCanvas::move_to(double x, double y) { cairo_move_to (cr, x, y); }
Void CairoCanvas::line_to(double x, double y) { cairo_line_to (cr, x, y); }
Void CairoCanvas::circle(double x, double y, double r) { cairo_arc (cr, x, y, r, 0, 2*M_PI); }
Void CairoCanvas::dot(double x, double y) { cairo_arc (cr, x, y, dr/1000, 0, 2*M_PI); }
Void CairoCanvas::set_dot_radius(double radius) { this->dr=radius; }
Void CairoCanvas::set_line_width(double width) { this->lw=width; }
Void CairoCanvas::set_line_colour(double r, double g, double b) { lc.red=r; lc.green=g; lc.blue=b; }
Void CairoCanvas::set_fill_opacity(double o) { fc.opacity=o; }
Void CairoCanvas::set_fill_colour(double r, double g, double b) { fc.red=r; fc.green=g; fc.blue=b; }


// TODO: Use generic canvas routines; move cairo-specific functionality
// into CairoCanvas class.
Void CairoCanvas::initialise(StringType x, StringType y, StringType z, double xl, double xu, double yl, double yu, double lz, double uz) {
    ARIADNE_NOT_IMPLEMENTED;
}
Void CairoCanvas::initialise(StringType text_x, StringType text_y, double xl, double xu, double yl, double yu)
{
    CairoCanvas& cairo_canvas=*this;
    cairo_t *crp=cairo_canvas.cr;

    const ImageSize2d drawing_size = cairo_canvas.size_in_pixels();
    const Int drawing_width = static_cast<Int>(drawing_size.nx);
    const Int drawing_height = static_cast<Int>(drawing_size.ny);

    //const Int canvas_width = cairo_image_surface_get_width(cairo_get_target(cr));
    //const Int canvas_height = cairo_image_surface_get_height(cairo_get_target(cr));

    const Int left_margin = LEFT_MARGIN;
    //const Int right_margin = RIGHT_MARGIN;
    //const Int bottom_margin = BOTTOM_MARGIN;
    const Int top_margin = TOP_MARGIN;

    // clear background
    cairo_set_source_rgb (crp, 1,1,1);
    cairo_paint (crp);

    // Set text font
    cairo_select_font_face (crp, "roman",CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size (crp, 30);

    // Set text colour
    cairo_set_source_rgb (crp, 0., 0., 0.);

    // Get axis label text
    StringType text_xl=to_str(xl);
    StringType text_xu=to_str(xu);
    StringType text_yl=to_str(yl);
    StringType text_yu=to_str(yu);

    // Write axis labels
    cairo_text_extents_t te;
    cairo_text_extents (crp, text_xl.c_str(), &te);
    cairo_move_to(crp, left_margin-2, top_margin+drawing_height+4+te.height);
    cairo_show_text (crp, text_xl.c_str());
    cairo_text_extents (crp, text_xu.c_str(), &te);
    cairo_move_to(crp, left_margin+drawing_width-te.width-4, top_margin+drawing_height+4+te.height);
    cairo_show_text (crp, text_xu.c_str());
    cairo_text_extents (crp, text_x.c_str(), &te);
    cairo_move_to(crp, left_margin+drawing_width/2-te.width/2-3, top_margin+drawing_height+4+te.height);
    cairo_show_text (crp, text_x.c_str());

    cairo_text_extents (crp, text_yl.c_str(), &te);
    cairo_move_to(crp, left_margin-te.width-6, top_margin+drawing_height+2);
    cairo_show_text (crp, text_yl.c_str());
    cairo_text_extents (crp, text_yu.c_str(), &te);
    cairo_move_to(crp, left_margin-te.width-6, top_margin+te.height+2);
    cairo_show_text (crp, text_yu.c_str());
    cairo_text_extents (crp, text_y.c_str(), &te);
    cairo_move_to(crp, left_margin-te.width-6, top_margin+drawing_height/2+te.height+2);
    cairo_show_text (crp, text_y.c_str());


    // Save unclipped state and canvas coordinates
    cairo_save (crp);

    // Set clipping region
    cairo_move_to (crp, left_margin, top_margin+drawing_height);
    cairo_line_to (crp, left_margin+drawing_width, top_margin+drawing_height);
    cairo_line_to (crp, left_margin+drawing_width, top_margin);
    cairo_line_to (crp, left_margin, top_margin);
    cairo_line_to (crp, left_margin, top_margin+drawing_height);

    // Fill clipping region with a very light colour to indicate where figure
    // should be drawn
    cairo_set_source_rgb(crp, 0.97,0.97,0.97);
    cairo_fill_preserve (crp);
    cairo_clip (crp);
    cairo_new_path (crp);

    //std::cerr<<"cw="<<canvas_width<<" lm="<<left_margin<<" dw="<<drawing_width<<" rm="<<right_margin<<" xl="<<xl<<" xu="<<xu<<"\n";
    //std::cerr<<"ch="<<canvas_height<<"tm="<<top_margin<<" dw="<<drawing_height<<" bm="<<bottom_margin<<" yl="<<yl<<" yu="<<yu<<"\n";

    // compute device to user coordinate transformation
    double ctr0=left_margin;
    double ctr1=top_margin;
    double sc0=(drawing_width)/(xu-xl);
    double sc1=-(drawing_height)/(yu-yl);
    double utr0=(-xl);
    double utr1=(-yu);

    // Scale to user coordinates
    cairo_translate(crp, ctr0, ctr1);
    cairo_scale (crp, sc0,sc1);
    cairo_translate(crp, utr0, utr1);
}

Void CairoCanvas::write(const char* cfilename) const {
    StringType filename(cfilename);
    if(filename.rfind(".") != StringType::npos) {
    } else {
        filename=filename+".png";
    }
    cairo_surface_t* surface = cairo_get_target (cr);
    cairo_surface_write_to_png (surface, filename.c_str());
}

Void CairoCanvas::finalise()
{
    cairo_t *crp=this->cr;

    // Restore canvas coordinates and unclipped state
    cairo_restore (crp);

    const ImageSize2d drawing_size = this->size_in_pixels();
    const Int drawing_width = static_cast<Int>(drawing_size.nx);
    const Int drawing_height = static_cast<Int>(drawing_size.ny);

    const Int left_margin = LEFT_MARGIN;
    const Int top_margin = TOP_MARGIN;

    cairo_set_line_width (crp, 2.0);
    cairo_set_source_rgb (crp, 0.0, 0.0, 0.0);
    cairo_move_to (crp, left_margin, top_margin+drawing_height);
    cairo_line_to (crp, left_margin+drawing_width, top_margin+drawing_height);
    cairo_line_to (crp, left_margin+drawing_width, top_margin);
    cairo_line_to (crp, left_margin, top_margin);
    cairo_line_to (crp, left_margin, top_margin+drawing_height);
    cairo_stroke (crp);
}

Void CairoCanvas::set_colour_palette() {  }
Void CairoCanvas::fill_3d() {  }
Void CairoCanvas::set_heat_map(Bool b) {  }

#endif

} // namespace Ariadne


