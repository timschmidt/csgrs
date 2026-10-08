/***************************************************************************
 *            io/graphics_base.hpp
 *
 *  Copyright  2009-21  Luca Geretti
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

/*! \file io/graphics_base.hpp
 *  \brief Base graphics implementation.
 */

#ifndef ARIADNE_GRAPHICS_BASE_HPP
#define ARIADNE_GRAPHICS_BASE_HPP

#include "graphics_interface.hpp"
#include "geometry2d.hpp"
#include "betterthreads/using.hpp"

namespace Ariadne {

using Mutex = std::mutex;
template<class T> using LockGuard = std::lock_guard<T>;

//! \brief Base for canvas classes
class CanvasBase : public CanvasInterface {
  public:
    Void fill_boundary(List<Point2d> const& boundary) override final {
        LockGuard<Mutex> lock(_mux);
        if(boundary.size()==1) { this->dot(boundary[0].x,boundary[0].y); }

        this->move_to(boundary[0].x,boundary[0].y);
        for(SizeType i=1; i!=boundary.size(); ++i) {
            this->line_to(boundary[i].x,boundary[i].y);
        }
        this->line_to(boundary[0].x,boundary[0].y);
        this->fill();
    }
  private:
    Mutex _mux;
};

} // namespace Ariadne


#endif // ARIADNE_GRAPHICS_BASE_HPP
