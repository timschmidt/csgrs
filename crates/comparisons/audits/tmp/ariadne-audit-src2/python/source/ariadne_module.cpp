/***************************************************************************
 *            ariadne_module.cpp
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

void foundations_submodule(pybind11::module& module);
void numeric_submodule(pybind11::module& module);
void linear_algebra_submodule(pybind11::module& module);
void optimization_submodule(pybind11::module& module);
void differentiation_submodule(pybind11::module& module);
void function_submodule(pybind11::module& module);
void calculus_submodule(pybind11::module& module);
void geometry_submodule(pybind11::module& module);
void io_submodule(pybind11::module& module);
void concurrency_submodule(pybind11::module& module);
void solver_submodule(pybind11::module& module);
void storage_submodule(pybind11::module& module);
void symbolic_submodule(pybind11::module& module);
void system_submodule(pybind11::module& module);
void evolution_submodule(pybind11::module& module);
void graphics_submodule(pybind11::module& module);
void hybrid_submodule(pybind11::module& module);

PYBIND11_MODULE(pyariadne, module) {
    foundations_submodule(module);
    numeric_submodule(module);
    linear_algebra_submodule(module);
    differentiation_submodule(module);
    function_submodule(module);
    calculus_submodule(module);
    geometry_submodule(module);
    io_submodule(module);
    concurrency_submodule(module);
    solver_submodule(module);
    optimization_submodule(module);
    storage_submodule(module);
    symbolic_submodule(module);
    system_submodule(module);
    evolution_submodule(module);
    graphics_submodule(module);
    hybrid_submodule(module);
}
