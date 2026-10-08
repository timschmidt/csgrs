/***************************************************************************
 *            io/graphics_manager.hpp
 *
 *  Copyright  2009-21  Luca Geretti, Mirko Albanese
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

/*! \file io/graphics_manager.hpp
 *  \brief Graphics singleton class for managing graphics and holding preferences.
 */

#ifndef ARIADNE_GRAPHICS_MANAGER_HPP
#define ARIADNE_GRAPHICS_MANAGER_HPP

#include "graphics_backend_interface.hpp"
#include "drawer_interface.hpp"

namespace Ariadne {

//! \ingroup GraphicsModule
//! \brief Manager for graphics preferences and global setup
class GraphicsManager {
  private:
    GraphicsManager();
  public:
    //! \brief Disable copy construction
    GraphicsManager(GraphicsManager const&) = delete;
    //! \brief Disable copy assignment
    Void operator=(GraphicsManager const&) = delete;

    //! \brief Return the static instance, lazily instantiated
    static GraphicsManager& instance();

    //! \brief The backend to use
    GraphicsBackend const& backend() const;
    //! \brief Set the backend
    Void set_backend(GraphicsBackend const& backend);
    //! \brief The drawer to use
    Drawer const& drawer() const;
    //! \brief Set the drawer
    Void set_drawer(Drawer const& drawer);

  private:
    GraphicsBackend _backend;
    Drawer _drawer;
};

} // namespace Ariadne


#endif // ARIADNE_GRAPHICS_MANAGER_HPP
