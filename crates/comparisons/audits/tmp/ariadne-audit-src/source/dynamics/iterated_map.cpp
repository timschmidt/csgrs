/***************************************************************************
 *            dynamics/iterated_map.cpp
 *
 *  Copyright  2008-20  Alberto Casagrande, Pieter Collins
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

#include "function/functional.hpp"
#include "config.hpp"

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "utility/tuple.hpp"
#include "helper/stlio.hpp"
#include "algebra/vector.hpp"
#include "function/function.hpp"
#include "function/constraint.hpp"
#include "symbolic/assignment.hpp"
#include "dynamics/enclosure.hpp"
#include "dynamics/orbit.hpp"

#include "conclog/logging.hpp"

#include "dynamics/iterated_map.hpp"

using namespace ConcLog;

namespace Ariadne {

EffectiveVectorMultivariateFunction make_auxiliary_function(
    Space<Real> const& state_space,
    List<RealAssignment> const& algebraic);

EffectiveVectorMultivariateFunction make_reset_function(
    Space<Real> const& space,
    List<RealAssignment> const& algebraic,
    List<PrimedRealAssignment> const& differential);


IteratedMap::IteratedMap(const List<PrimedRealAssignment>& updates)
    : IteratedMap(updates,List<RealAssignment>()) { }

IteratedMap::IteratedMap(const List<PrimedRealAssignment>& updates, List<RealAssignment> const& auxiliary)
    : _updates(updates), _auxiliary(auxiliary)
    , _update_function(make_reset_function(left_hand_sides(updates),auxiliary,updates))
    , _auxiliary_function(make_auxiliary_function(left_hand_sides(updates),auxiliary))
{
}

RealSpace IteratedMap::state_space() const {
    return RealSpace(left_hand_sides(this->_updates));
}

RealSpace IteratedMap::auxiliary_space() const {
    return RealSpace(left_hand_sides(this->_auxiliary));
}

RealSpace IteratedMap::state_auxiliary_space() const {
    return join(state_space(),auxiliary_space());
}

OutputStream& operator<<(OutputStream& os, const IteratedMap& map) {
    os << "IteratedMap( update_function = " << map.update_function() << ", "
          "auxiliary_function = " << map.auxiliary_function() << ", "
          "updates = " << map._updates << ", "
          "auxiliary = " << map._auxiliary << ")";
    return os;
}

}  // namespace Ariadne

