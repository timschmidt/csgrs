/***************************************************************************
 *            hybrid/discrete_location.hpp
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

/*! \file hybrid/discrete_location.hpp
 *  \brief Class representing a discrete location of a hybrid automaton.
 */

#ifndef ARIADNE_DISCRETE_LOCATION_HPP
#define ARIADNE_DISCRETE_LOCATION_HPP

#include <string>
#include "utility/container.hpp"
#include "symbolic/valuation.hpp"

namespace Ariadne {

//! \ingroup SystemModule
//! \brief Type of a  discrete location of a hybrid system.
class DiscreteLocation
    : public StringValuation
{
  public:
    //! \brief Construct a location which does not set any discrete variables.
    DiscreteLocation() : StringValuation() { }
    //! \brief Construct the location q|n.
    DiscreteLocation(Int n);
    DiscreteLocation(StringVariable var, String val) : DiscreteLocation({var|val}) { }
    DiscreteLocation(Pair<StringVariable,String> svarstr) : StringValuation({svarstr}) { }
    DiscreteLocation(const StringValuation& val) : StringValuation(val) { }
    DiscreteLocation(const Map<Identifier,String>& sm) : StringValuation(sm) { }
    DiscreteLocation(const Map<StringVariable,String>& sm) : StringValuation(sm) { }
    DiscreteLocation(const InitializerList<Pair<StringVariable,String>>& salst) : StringValuation(salst) { }
    Void adjoin(const DiscreteLocation& loc) { this->_values.adjoin(loc._values); }
};

//! \relates DiscreteLocation \brief Combine the values of variables of two locations.
DiscreteLocation join(const DiscreteLocation& loc1, const DiscreteLocation& loc2);
//! \relates DiscreteLocation \brief Equality test.
//! Throws an IndistinguishableLocationsError if the valuations are not identical but have no variable with distinct values.
Bool operator==(const DiscreteLocation& loc1, const DiscreteLocation& loc2);
//! \relates DiscreteLocation \brief Inequality test.
Bool operator!=(const DiscreteLocation& loc1, const DiscreteLocation& loc2);
//! \relates DiscreteLocation \brief A total order on DiscreteLocation, allowing comparison of non-distinuishable valuations.
Bool operator<(const DiscreteLocation& loc1, const DiscreteLocation& loc2);

//! \relates DiscreteLocation \brief Test if two locations are distinguishable i.e. specify a common variable which takes different values.
Bool are_distinguishable(const DiscreteLocation& location1, const DiscreteLocation& location2);

//! \relates DiscreteLocation \brief Test if \a location1 and \a location2 are the same i.e. define the same values.
Bool are_same(const DiscreteLocation& location1, const DiscreteLocation& location2);

//! \relates DiscreteLocation \brief Test if \a partial_location is defined by a restricted set of variables of \a full_location.
Bool is_restriction(const DiscreteLocation& partial_location, const DiscreteLocation& full_location);

//! \relates DiscreteLocation \brief Retrict the variables defined in \a location to those of \a variables.
DiscreteLocation restrict(const DiscreteLocation& location, const Set<Identifier>& variables);

} //namespace Ariadne

#endif /* ARIADNE_DISCRETE_LOCATION_HPP */
