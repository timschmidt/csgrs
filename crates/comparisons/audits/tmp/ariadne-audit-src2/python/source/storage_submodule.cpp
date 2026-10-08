/***************************************************************************
 *            storage_submodule.cpp
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

#include "numeric/numeric.hpp"
#include "geometry/function_set.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/list_set.hpp"

using namespace Ariadne;


Void export_grid(pybind11::module& module)
{
    pybind11::class_< Grid > grid_class(module,"Grid");
    grid_class.def(pybind11::init<Nat>());
    grid_class.def(pybind11::init<Nat,FloatDP>());
    grid_class.def(pybind11::init<Vector<FloatDP>,Vector<FloatDP>>());
    grid_class.def("dimension", &Grid::dimension);
    grid_class.def("origin", &Grid::origin);
    grid_class.def("lengths", &Grid::lengths);
    grid_class.def("__str__",&__cstr__<Grid>);
}

Void export_grid_cell(pybind11::module& module)
{
    pybind11::class_<GridCell> grid_cell_class(module,"GridCell");
    grid_cell_class.def("dimension", &GridCell::dimension);
    grid_cell_class.def("depth", &GridCell::depth);
    grid_cell_class.def("split", (Pair<GridCell,GridCell>(GridCell::*)()const) &GridCell::split);
    grid_cell_class.def("split", (GridCell(GridCell::*)(Bool)const) &GridCell::split);
    grid_cell_class.def("box", &GridCell::box);
    grid_cell_class.def("__str__",&__cstr__<GridCell>);

    module.def("smallest_enclosing_primary_cell", &GridCell::smallest_enclosing_primary_cell);
}


Void export_grid_tree_set(pybind11::module& module) {

    pybind11::class_<GridTreeSubpaving> grid_tree_subpaving_class(module,"GridTreeSubset");

    pybind11::class_<GridTreePaving, pybind11::bases<GridTreeSubpaving,Drawable2dInterface> > grid_tree_paving_class(module,"GridTreePaving");
    grid_tree_paving_class.def(pybind11::init<GridTreePaving>());
    grid_tree_paving_class.def(pybind11::init<Nat>());
    grid_tree_paving_class.def(pybind11::init<Grid>());
    grid_tree_paving_class.def("bounding_box", &GridTreePaving::bounding_box);
    grid_tree_paving_class.def("is_empty", &GridTreePaving::is_empty);
    grid_tree_paving_class.def("size", &GridTreePaving::size);
    grid_tree_paving_class.def("dimension", &GridTreePaving::dimension);
    grid_tree_paving_class.def("clear", &GridTreePaving::clear);
    grid_tree_paving_class.def("mince", &GridTreePaving::mince);
    grid_tree_paving_class.def("recombine", &GridTreePaving::recombine);
    grid_tree_paving_class.def("grid", &GridTreePaving::grid);
    grid_tree_paving_class.def("measure", &GridTreePaving::measure);
    grid_tree_paving_class.def("adjoin", (Void(GridTreePaving::*)(const GridCell&))(&GridTreePaving::adjoin));
    grid_tree_paving_class.def("adjoin", (Void(GridTreePaving::*)(const GridTreeSubpaving&))(&GridTreePaving::adjoin));
    grid_tree_paving_class.def("restrict", (Void(GridTreePaving::*)(const GridTreeSubpaving&))(&GridTreePaving::restrict));
    grid_tree_paving_class.def("remove", (Void(GridTreePaving::*)(const GridTreeSubpaving&))(&GridTreePaving::remove));
    grid_tree_paving_class.def("adjoin_over_approximation", (Void(GridTreePaving::*)(const ExactBoxType&,const Nat)) &GridTreePaving::adjoin_over_approximation);
    grid_tree_paving_class.def("adjoin_outer_approximation", (Void(GridTreePaving::*)(const EffectiveEuclideanCompactSetInterface&,const Nat)) &GridTreePaving::adjoin_outer_approximation);
    grid_tree_paving_class.def("adjoin_inner_approximation", (Void(GridTreePaving::*)(const EffectiveEuclideanOpenSetInterface&,const Nat,const Nat)) &GridTreePaving::adjoin_inner_approximation);
    grid_tree_paving_class.def("__len__", &GridTreePaving::size);
    grid_tree_paving_class.def("__iter__", [](GridTreePaving const& g){return pybind11::make_iterator(g.begin(), g.end());});
    grid_tree_paving_class.def("__str__",&__cstr__<GridTreePaving>);

    module.def("union",(GridTreePaving(*)(const GridTreeSubpaving&,const GridTreeSubpaving&))(&join));
    module.def("difference",(GridTreePaving(*)(const GridTreeSubpaving&,const GridTreeSubpaving&))(&difference));
    module.def("intersection",(GridTreePaving(*)(const GridTreeSubpaving&,const GridTreeSubpaving&))(&intersection));
    module.def("intersect",(Bool(*)(const GridTreeSubpaving&,const GridTreeSubpaving&))(&intersect));
    module.def("subpaving",(Bool(*)(const GridTreeSubpaving&,const GridTreeSubpaving&))(&subset));
    module.def("intersect",(Bool(*)(const GridCell&,const GridTreeSubpaving&))(&intersect));
    module.def("subpaving",(Bool(*)(const GridCell&,const GridTreeSubpaving&))(&subset));

    module.def("outer_approximation",(GridTreePaving(*)(const EffectiveEuclideanCompactSetInterface&,const Grid&,const Nat)) &outer_approximation);
    module.def("inner_approximation",(GridTreePaving(*)(const EffectiveEuclideanOpenSetInterface&,const Grid&,const Nat,const Nat)) &inner_approximation);

}


Void storage_submodule(pybind11::module& module)
{
    export_grid(module);
    export_grid_cell(module);
    export_grid_tree_set(module);
}

