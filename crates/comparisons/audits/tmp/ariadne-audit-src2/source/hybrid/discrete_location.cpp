/***************************************************************************
 *            hybrid/discrete_location.cpp
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

#include "discrete_location.hpp"
#include "hybrid_automaton_interface.hpp"

namespace Ariadne {

DiscreteLocation join(const DiscreteLocation& loc1, const DiscreteLocation& loc2) {
    return DiscreteLocation(join(loc1.values(),loc2.values()));
}

DiscreteLocation::DiscreteLocation(Int n) : DiscreteLocation(StringVariable("q")|to_str(n)) {
}

Bool operator==(const DiscreteLocation& q1, const DiscreteLocation& q2) {
    Bool identical=true;
    const Map<Identifier,String>& q1sm=q1.values();
    const Map<Identifier,String>& q2sm=q2.values();
    Map<Identifier,String>::ConstIterator q1iter=q1sm.begin();
    Map<Identifier,String>::ConstIterator q2iter=q2sm.begin();

    while(q1iter!=q1sm.end() && q2iter!=q2sm.end()) {
        if(q1iter->first==q2iter->first) {
            if(q1iter->second != q2iter->second) { return false; }
            ++q1iter; ++q2iter;
        } else if(q1iter->first<q2iter->first) {
            identical=false; ++q1iter;
        } else {
            identical=false; ++q2iter;
        }
    }
    if(q1iter!=q1sm.end() || q2iter!=q2sm.end()) { identical=false; }
    if(!identical) { ARIADNE_THROW(IndistinguishableLocationsError,"operator==(DiscreteLocation,DiscreteLocation)",
                               "Locations "<<q1<<" and "<<q2<<" are not identical, but no values differ."); }
    return true;
}

Bool operator!=(const DiscreteLocation& q1, const DiscreteLocation& q2) {
    return !(q1==q2);
}

Bool operator<(const DiscreteLocation& q1, const DiscreteLocation& q2) {
    const Map<Identifier,String>& q1sm=q1.values();
    const Map<Identifier,String>& q2sm=q2.values();
    Map<Identifier,String>::ConstIterator q1iter=q1sm.begin();
    Map<Identifier,String>::ConstIterator q2iter=q2sm.begin();

    while(q1iter!=q1sm.end() && q2iter!=q2sm.end()) {
        if(q1iter->first!=q2iter->first) {
            return (q1iter->first<q2iter->first);
        } else if(q1iter->second != q2iter->second) {
            return q1iter->second < q2iter->second;
        } else {
            ++q1iter; ++q2iter;
        }
    }
    return q2iter!=q2sm.end();
}

Bool are_distinguishable(const DiscreteLocation& location1, const DiscreteLocation& location2) {
    for(Map<Identifier,String>::ConstIterator iter1=location1._values.begin(); iter1!=location1._values.end(); ++iter1) {
        if(location2._values.has_key(iter1->first) && location2._values[iter1->first] != iter1->second) {
            return true;
        }
    }
    return false;
}

Bool are_same(const DiscreteLocation& location1, const DiscreteLocation& location2) {
    return location1._values == location2._values;
}

Bool is_restriction(const DiscreteLocation& partial_location, const DiscreteLocation& full_location) {
    for(Map<Identifier,String>::ConstIterator value_iter=partial_location._values.begin(); value_iter!=partial_location._values.end(); ++value_iter) {
        if(!full_location._values.has_key(value_iter->first) || full_location[StringVariable(value_iter->first)]!=value_iter->second) {
            return false;
        }
    }
    return true;
}

DiscreteLocation restrict(const DiscreteLocation& location, const Set<Identifier>& variables) {
    ARIADNE_ASSERT_MSG(subset(location._values.keys(),variables)," location "<<location<<" variables not a subset of "<<variables);
    return restrict_keys(location._values,variables);
}


} //namespace Ariadne
