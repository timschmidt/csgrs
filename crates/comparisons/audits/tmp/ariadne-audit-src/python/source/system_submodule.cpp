/***************************************************************************
 *
 *            system_submodule.cpp
 *
 *  Copyright  2009-20  Pieter Collins
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

#include <iostream>
#include <iomanip>
#include <functional>

#include "utility/tribool.hpp"
#include "numeric/numeric.hpp"
#include "function/function.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/space.hpp"
#include "dynamics/iterated_map.hpp"
#include "dynamics/vector_field.hpp"


using namespace Ariadne;

template<class V, class E> decltype(auto) to_list(Map<V,E>const& map) {
    using A=decltype(declval<V>()=declval<E>());
    List<A> lst; for (auto ve : map) { lst.append(ve.first=ve.second); } return lst;
}

Void export_map(pybind11::module& module)
{
    pybind11::class_<IteratedMap> iterated_map_class(module,"IteratedMap");
    iterated_map_class.def(pybind11::init([](Map<PrimedRealVariable,RealExpression> map){return IteratedMap(to_list(map));}));
    iterated_map_class.def(pybind11::init([](Map<PrimedRealVariable,RealExpression> map, Map<LetRealVariable,RealExpression> aux){return IteratedMap(to_list(map),to_list(aux));}));
//    iterated_map_class.def(pybind11::init<List<PrimedRealAssignment> const&>());
//    iterated_map_class.def(pybind11::init<List<PrimedRealAssignment> const&,List<RealAssignment> const&>());
    iterated_map_class.def("state_space", &IteratedMap::state_space);
    iterated_map_class.def("auxiliary_space", &IteratedMap::auxiliary_space);
    iterated_map_class.def("update_function", &IteratedMap::update_function);
    iterated_map_class.def("auxiliary_function", &IteratedMap::auxiliary_function);
    iterated_map_class.def("function", &IteratedMap::function);
    iterated_map_class.def("__str__",&__cstr__<IteratedMap>);
}

Void export_vector_field(pybind11::module& module)
{
    pybind11::class_<VectorField> vector_field_class(module,"VectorField");
    vector_field_class.def(pybind11::init([](Map<DottedRealVariable,RealExpression> dyn){return VectorField(to_list(dyn));}));
    vector_field_class.def(pybind11::init([](Map<DottedRealVariable,RealExpression> dyn, Map<LetRealVariable,RealExpression> aux){return VectorField(to_list(dyn),to_list(aux));}));
//    vector_field_class.def(pybind11::init<List<DottedRealAssignment> const&>());
//    vector_field_class.def(pybind11::init<List<DottedRealAssignment> const&,List<RealAssignment> const&>());
    vector_field_class.def("state_space", &VectorField::state_space);
    vector_field_class.def("auxiliary_space", &VectorField::auxiliary_space);
    vector_field_class.def("dynamic_function", &VectorField::dynamic_function);
    vector_field_class.def("auxiliary_function", &VectorField::auxiliary_function);
    vector_field_class.def("function", &VectorField::function);
    vector_field_class.def("__str__",&__cstr__<VectorField>);
}




Void system_submodule(pybind11::module& module) {
    export_map(module);
    export_vector_field(module);
}

