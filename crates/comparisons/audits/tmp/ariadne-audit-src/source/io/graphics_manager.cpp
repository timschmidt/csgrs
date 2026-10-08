/***************************************************************************
 *            io/graphics_manager.cpp
 *
 *  Copyright  2008-21  Mirko Albanese, Luca Geretti
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

#include "config.hpp"
#include "utility/handle.hpp"
#include "io/graphics_backend_interface.hpp"
#include "io/graphics_manager.hpp"
#include "io/figure.hpp"
#include "io/geometry2d.hpp"
#include "io/cairo.hpp"
#include "io/gnuplot.hpp"
#include "io/drawer.hpp"
#include "io/null_graphics.hpp"

namespace Ariadne {

GraphicsBackend default_backend() {
    #if defined HAVE_CAIRO_H
        return CairoGraphicsBackend();
    #elif defined HAVE_GNUPLOT_H
        return GnuplotGraphicsBackend();
    #else
        return NullGraphicsBackend();
    #endif
}

GraphicsManager::GraphicsManager() : _backend(default_backend()), _drawer(AffineDrawer(0)) {
}

GraphicsManager& GraphicsManager::instance() {
    static GraphicsManager instance;
    return instance;
}

GraphicsBackend const& GraphicsManager::backend() const {
    return _backend;
}

Void GraphicsManager::set_backend(GraphicsBackend const& backend) {
    _backend = backend;
}

Drawer const& GraphicsManager::drawer() const {
    return _drawer;
}

Void GraphicsManager::set_drawer(Drawer const& drawer) {
    _drawer = drawer;
}

} // namespace Ariadne


