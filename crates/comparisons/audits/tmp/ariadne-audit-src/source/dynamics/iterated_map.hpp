/***************************************************************************
 *            dynamics/iterated_map.hpp
 *
 *  Copyright  2004-20  Alberto Casagrande, Pieter Collins
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

/*! \file dynamics/iterated_map.hpp
 *  \brief Main class for discrete-time continuous-space systems.
 */

#ifndef ARIADNE_ITERATED_MAP_HPP
#define ARIADNE_ITERATED_MAP_HPP

#include <memory>

#include "function/function.hpp"
#include "geometry/set_interface.hpp"
#include "geometry/grid.hpp"

#include "symbolic/variable.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/space.hpp"

namespace Ariadne {

class Enclosure;
class LabelledEnclosure;
class Storage;
class LabelledStorage;

class IteratedMapEvolver;

//! \brief An iterated function system.
class IteratedMap
{
  public:
    //! \brief The type used to represent time.
    typedef Integer TimeType;
    //! \brief The type used to represent real numbers.
    typedef Real RealType;
    //! \brief The type used to evolve the system
    typedef IteratedMapEvolver EvolverType;
    typedef LabelledEnclosure EnclosureType;
    //! \brief The type used to define global pavings of reach and evolve sets.
    typedef LabelledStorage StorageType;
    //! \brief The state space
    typedef EuclideanSpace StateSpaceType;
  public:
    IteratedMap(const List<PrimedRealAssignment>&);
    IteratedMap(const List<PrimedRealAssignment>&, List<RealAssignment> const&);

    const EffectiveVectorMultivariateFunction& update_function() const { return this->_update_function; }
    const EffectiveVectorMultivariateFunction& auxiliary_function() const { return this->_auxiliary_function; }
    const EffectiveVectorMultivariateFunction& auxiliary_mapping() const { return this->_auxiliary_function; }

    RealSpace state_space() const;
    RealSpace auxiliary_space() const;
    RealSpace state_auxiliary_space() const;

    IteratedMap* clone() const { return new IteratedMap(*this); }
    ~IteratedMap() = default;
    DimensionType dimension() const { return this->_update_function.result_size(); }
    const EffectiveVectorMultivariateFunction& function() const { return _update_function; }

    friend OutputStream& operator<<(OutputStream& os, IteratedMap const& map);
  private:
    List<PrimedRealAssignment> _updates;
    List<RealAssignment> _auxiliary;
    EffectiveVectorMultivariateFunction _update_function;
    EffectiveVectorMultivariateFunction _auxiliary_function;
};


} // namespace Ariadne

#endif // ARIADNE_ITERATED_MAP_HPP
