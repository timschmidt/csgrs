/***************************************************************************
 *            algebra_module.cpp
 *
 *  Copyright  2007-24  Pieter Collins
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

void foundations_submodule(pybind11::module&);
void numeric_submodule(pybind11::module&);
void linear_algebra_submodule(pybind11::module&);
void differentiation_submodule(pybind11::module&);

PYBIND11_MODULE(pyariadne_algebra, module) {
    foundations_submodule(module);
    numeric_submodule(module);
    linear_algebra_submodule(module);
    differentiation_submodule(module);
}
