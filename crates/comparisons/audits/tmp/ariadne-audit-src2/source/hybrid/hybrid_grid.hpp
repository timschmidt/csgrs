/***************************************************************************
 *            hybrid/hybrid_grid.hpp
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

/*! \file hybrid/hybrid_grid.hpp
 *  \brief Grids in hybrid spaces.
 */

#ifndef ARIADNE_HYBRID_GRID_HPP
#define ARIADNE_HYBRID_GRID_HPP

#include <iostream>

#include "symbolic/space.hpp"
#include "hybrid/hybrid_space.hpp"
#include "hybrid/hybrid_scalings.hpp"

#include "geometry/grid.hpp"

namespace Ariadne {

typedef Vector<FloatDP> ExactFloatVector;
class HybridGridTreePaving;

//! \ingroup HybridModule
//! \brief A grid in a hybrid space
class HybridGrid
{
    // NOTE: The use of the system is to allow the "Grid" of a compositional
    // hybrid automaton to be computed on-the-fly since it might not be
    // feasible to compute the reachable states.
    HybridSpace _space;
    HybridScalings _scalings;
  public:
    //! Test whether the grid has location \a q.
    Bool has_location(const DiscreteLocation& q) const {
        return this->_space.has_location(q); }
    //! The grid in location \a loc.
    Grid operator[](const DiscreteLocation& loc) const;
    //! The underlying hybrid space.
    const HybridSpace& space() const { return this->_space; }
    //! The underlying real space in location \a loc.
    RealSpace space(const DiscreteLocation& loc) const { return this->_space[loc]; }
    //! The variable scalings used.
    HybridScalings& scalings() { return this->_scalings; }
    //! The grid in location \a loc.
    Grid grid(const DiscreteLocation& loc) const { return this->operator[](loc); }
  public:
    //! The grid corresponding to unit resolution on each dimension.
    HybridGrid(const HybridSpace& hsp) : _space(hsp), _scalings(SimpleHybridScalings()) { }
    //! The grid corresponding to scalings each variable in each location according to the policy \a hsc.
    HybridGrid(const HybridSpace& hsp, const HybridScalings& hsc)
        : _space(hsp), _scalings(hsc) { }
    HybridGrid* clone() const { return new HybridGrid(*this); }
    //!
    friend OutputStream& operator<<(OutputStream& os, const HybridGrid& hgrid);
};

inline Grid HybridGrid::operator[](const DiscreteLocation& loc) const
{
    return this->_scalings.grid(loc,this->_space[loc]);
}

inline OutputStream& operator<<(OutputStream& os, const HybridGrid& hgrid) {
    return os << "HybridGrid( space="<<hgrid._space << ", scalings=" << hgrid._scalings << " )";
}

}

#endif // ARIADNE_HYBRID_GRID_HPP

