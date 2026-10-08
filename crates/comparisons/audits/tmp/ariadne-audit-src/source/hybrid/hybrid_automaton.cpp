/***************************************************************************
 *            hybrid_automaton-composite.cpp
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

#include "function/functional.hpp"
#include "config.hpp"

#include <map>

#include "utility/macros.hpp"
#include "utility/container.hpp"
#include "helper/stlio.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/space.hpp"
#include "function/function.hpp"
#include "hybrid/hybrid_time.hpp"
#include "hybrid/hybrid_space.hpp"

#include "hybrid/hybrid_automaton.hpp"

namespace Ariadne {

EffectiveVectorMultivariateFunction make_auxiliary_function(Space<Real> const& space, List<RealAssignment> const& algebraic);
EffectiveVectorMultivariateFunction make_dynamic_function(Space<Real> const& space, List<RealAssignment> const& sorted_algebraic, List<DottedRealAssignment> const& differential);
EffectiveVectorMultivariateFunction make_reset_function(Space<Real> const& space, List<RealAssignment> const& sorted_algebraic, List<PrimedRealAssignment> const& primed);
EffectiveScalarMultivariateFunction make_constraint_function(Space<Real> const& space, List<RealAssignment> const& sorted_algebraic, ContinuousPredicate const& constraint, Sign sign);

DiscreteMode::DiscreteMode()
{
}

DiscreteMode::
DiscreteMode(DiscreteLocation location)
    : _location(location)
{
}

DiscreteMode::
DiscreteMode(DiscreteLocation location,
             List<RealAssignment> const& auxiliary,
             List<DottedRealAssignment> const& dynamic)
    : _location(location), _auxiliary(auxiliary), _dynamic(dynamic)
{
    _sorted_auxiliary=algebraic_sort(auxiliary);
}



template<class K, class T1, class T2>
Map<K,Pair<T1,T2> > merge(const Map<K,T1>& m1, const Map<K,T2>& m2) {
    ARIADNE_ASSERT(m1.keys()==m2.keys());
    Map<K,Pair<T1,T2> > r;
    Set<K> keys=m1.keys();
    for(typename Set<K>::ConstIterator iter=keys.begin(); iter!=keys.end(); ++iter) {
        r.insert(*iter,make_pair(m1[*iter],m2[*iter]));
    }
    return r;
}


template<class X> Set<Identifier> names(const Set< Variable<X> >& variables) {
    Set<Identifier> names;
    for(typename Set< Variable<X> >::ConstIterator variable_iter=variables.begin(); variable_iter!=variables.end(); ++variable_iter) {
        names.insert(variable_iter->name());
    }
    return names;
}

template<class X> Set<Identifier> names(const List< Variable<X> >& variables) {
    Set<Identifier> names;
    for(typename List< Variable<X> >::ConstIterator variable_iter=variables.begin(); variable_iter!=variables.end(); ++variable_iter) {
        names.insert(variable_iter->name());
    }
    return names;
}


Writer<DiscreteMode> DiscreteMode::_default_writer(new VerboseDiscreteModeWriter());

OutputStream&
DiscreteMode::_write(OutputStream& os) const {
    return os << _default_writer(*this);
}

OutputStream&
VerboseDiscreteModeWriter::_write(OutputStream& os, DiscreteMode const& m) const {
    os << "DiscreteMode( "
       << "location=" << m._location;
    if(m._auxiliary.size()>0) {
        os << ", algebraic_equations="<<m._auxiliary; }
    if(m._dynamic.size()>0) {
        os << ", differential_equations="<<m._dynamic; }
    if(m._invariants.size()>0) {
        os << ", invariants="<<m._invariants; }
    if(m._guards.size()>0) {
        os << ", guards="<<m._guards; }
    if(m._targets.size()>0) {
        os << ", targets="<<m._targets; }
    if(m._targets.size()>0) {
        os << ", resets="<<m._resets; }
    return os << " )";
}

OutputStream&
CompactDiscreteModeWriter::_write(OutputStream& os, DiscreteMode const& m) const {
    if (m._location.values().size() > 0)
        os << m._location << ": ";
    if(m._auxiliary.size()>0 || m._dynamic.size()>0)
        os << "eqt=";
    if(m._auxiliary.size()>0) {
        os << m._auxiliary; }
    if(m._auxiliary.size()>0 && m._dynamic.size()>0)
        os << ",";
    if(m._dynamic.size()>0) {
        os << m._dynamic; }
    if(m._invariants.size()>0) {
        os << ", inv="<<m._invariants; }
    if(m._guards.size()>0) {
        os << ", grd="<<m._guards; }
    if(m._targets.size()>0) {
        os << ", trg="<<m._targets; }
    if(m._targets.size()>0) {
        Map<DiscreteEvent, List<PrimedRealAssignment>> nonempty_resets;
        for (auto it = m._resets.begin(); it != m._resets.end(); ++it)
            if (it->second.size() != 0)
                nonempty_resets.insert(*it);
        if (not nonempty_resets.empty())
            os << ", rst="<<nonempty_resets;
    }
    return os;
}


DiscreteTransition::
DiscreteTransition(DiscreteLocation source,
                   DiscreteEvent event,
                   DiscreteLocation target,
                   const List<PrimedRealAssignment>& reset,
                   const ContinuousPredicate& guard,
                   EventKind kind)
    : _source(source), _event(event), _target(target),
      _guard(guard), _reset(reset)
{
}

OutputStream&
DiscreteTransition::_write(OutputStream& os) const
{
    const DiscreteTransition& transition=*this;
    return os << "DiscreteTransition( "
              << "event=" << transition._event << ", "
              << "source=" << transition._source << ", "
              << "target=" << transition._target << ", "
              << "reset=" << transition._reset << ", "
              << "guard=" << transition._guard << " )";
}


List<RealAssignment>
algebraic_sort(const List<RealAssignment>& auxiliary) {
    List<RealVariable> lhs_array=left_hand_sides(auxiliary);
    LinkedList<RealVariable> lhs_list(lhs_array.begin(),lhs_array.end());
    Map<Identifier, Set<UntypedVariable> > dependencies;
    Set<UntypedVariable> variables(lhs_array.begin(),lhs_array.end());

    for(List<RealAssignment>::ConstIterator asgn_iter=auxiliary.begin();
        asgn_iter!=auxiliary.end(); ++asgn_iter)
    {
        dependencies.insert(asgn_iter->lhs.base().name(), intersection(asgn_iter->rhs.arguments(),variables));
    }

    List<RealAssignment> sorted_auxiliary;
    sorted_auxiliary.reserve(auxiliary.size());
    if(!lhs_list.empty()) {
        Bool found=false;
        for(LinkedList<RealVariable>::Iterator iter=lhs_list.begin(); iter!=lhs_list.end(); ) {
            if(dependencies[iter->name()].empty()) {
                for(Map<Identifier, Set<UntypedVariable> >::Iterator dep_iter=dependencies.begin(); dep_iter!=dependencies.end(); ++dep_iter) {
                    dep_iter->second.remove(static_cast<UntypedVariable>(*iter));
                }
                for(List<RealAssignment>::ConstIterator asgn_iter=auxiliary.begin();
                    asgn_iter!=auxiliary.end(); ++asgn_iter)
                {
                    if(asgn_iter->lhs.name()==iter->name()) {
                        sorted_auxiliary.append(*asgn_iter);
                        break;
                    }
                }
                dependencies.erase(iter->name());
                found=true;
                lhs_list.erase(iter);
                iter=lhs_list.begin();
            } else {
                ++iter;
            }
        }
        if(!found) {
            ARIADNE_THROW(AlgebraicLoopError,"HybridAutomaton::sort(List<RealAssignment>)",
                          "Algebraic dependencies among variables "<<lhs_list<<" in auxiliary equations "<<auxiliary);
        }
    }
    ARIADNE_ASSERT(auxiliary.size()==sorted_auxiliary.size());
    return sorted_auxiliary;
}

EffectiveVectorMultivariateFunction make_auxiliary_function(
    Space<Real> const& state_space,
    List<RealAssignment> const& algebraic)
{
    List<RealAssignment> sorted_algebraic = algebraic_sort(algebraic);
    Space<Real> auxiliary_space(left_hand_sides(algebraic));
    Space<Real> sorted_auxiliary_space(left_hand_sides(sorted_algebraic));

    RealExpression default_expression;
    Vector<RealExpression> results(sorted_algebraic.size(),default_expression);
    for(SizeType j=0; j!=sorted_algebraic.size(); ++j) {
        SizeType i=auxiliary_space[sorted_auxiliary_space[j]];
        results[i]=substitute(sorted_algebraic[j].rhs,sorted_algebraic);
    }
    return make_function(state_space,results);
}

EffectiveVectorMultivariateFunction make_dynamic_function(
    Space<Real> const& space,
    List<RealAssignment> const& algebraic,
    List<DottedRealAssignment> const& differential)
{
    RealExpression default_expression;
    Vector<RealExpression> results(differential.size(),default_expression);
    for(SizeType i=0; i!=differential.size(); ++i) { results[space.index(differential[i].lhs.base())]=substitute(differential[i].rhs,algebraic); }

    return make_function(space,results);
}

EffectiveVectorMultivariateFunction make_reset_function(
    Space<Real> const& space,
    List<RealAssignment> const& algebraic,
    List<PrimedRealAssignment> const& primed)
{
    RealExpression default_expression;
    Vector<RealExpression> results(primed.size(),default_expression);
    for(SizeType i=0; i!=primed.size(); ++i) { results[i]=substitute(primed[i].rhs,algebraic); }

    return make_function(space,results);
}

EffectiveScalarMultivariateFunction make_constraint_function(
    Space<Real> const& space,
    List<RealAssignment> const& algebraic,
    ContinuousPredicate const& constraint,
    Sign sign)
{
    RealExpression constraint_expression=indicator(constraint,sign);
    return make_function(space,substitute(constraint_expression,algebraic));
}

HybridAutomaton::HybridAutomaton()
    : _name("automaton"),_modes()
{
}


HybridAutomaton::HybridAutomaton(Identifier name)
    : _name(name),_modes()
{
}

HybridAutomaton::HybridAutomaton(
		Identifier name,
		const List<StringVariable>& discrete_variables)
    : _name(name),_modes()
{
}





Void
HybridAutomaton::_new_mode(DiscreteLocation location,
                           const List<RealAssignment>& auxiliary,
                           const List<DottedRealAssignment>& dynamic)
{
    for(Map<DiscreteLocation,DiscreteMode>::ConstIterator mode_iter=this->_modes.begin(); mode_iter!=this->_modes.end(); ++mode_iter) {
        if(!are_distinguishable(location,mode_iter->first)) {
            ARIADNE_THROW(IndistinguishableModeError,"HybridAutomaton::new_mode",
                          "Location "<<location<<" is indistinguishable from location "<<mode_iter->first<<
                          " in hybrid automaton with locations "<<this->locations());
        }
    }

    Set<UntypedVariable> defined_variables;
    Set<UntypedVariable> argument_variables;

    // Compute the auxiliary variables ordered by the given equations
    for(SizeType i=0; i!=auxiliary.size(); ++i) {
        if(defined_variables.contains(auxiliary[i].lhs)) {
            ARIADNE_THROW(SystemSpecificationError,"HybridAutomaton::new_mode",
                          "Variable "<<auxiliary[i].lhs<<" is defined twice by the algebraic equations "<<auxiliary<<" for mode "<<location);
        }
        defined_variables.insert(auxiliary[i].lhs);
        argument_variables.adjoin(auxiliary[i].rhs.arguments());
    }

    // Compute the state variables ordered by the given differential equations
    for(SizeType i=0; i!=dynamic.size(); ++i) {
        if(defined_variables.contains(dynamic[i].lhs.base())) {
            ARIADNE_THROW(SystemSpecificationError,"HybridAutomaton::new_mode",
                          "Variable "<<dynamic[i].lhs.base()<<" defined by the differential equations "<<dynamic<<" for mode "<<location<<" is also defined in the algebraic equations "<<auxiliary);
        }
        defined_variables.insert(dynamic[i].lhs.base());
        argument_variables.adjoin(dynamic[i].rhs.arguments());
    }

    DiscreteMode new_mode=DiscreteMode(location,auxiliary,dynamic);

    this->_modes.insert(location,new_mode);
}


Void HybridAutomaton::_new_invariant(DiscreteLocation location, ContinuousPredicate invariant, DiscreteEvent event)
{
    if(!this->has_location(location)) {
        ARIADNE_THROW(NonExistentModeError,"HybridAutomaton::new_invariant",
                      "mode "<<location<<" is not a location of the automaton with locations "<<this->locations());
    }
    DiscreteMode& mode=this->_modes.value(location);
    if(mode._kinds.has_key(event)) {
        ARIADNE_THROW(MultipleGuardError,"HybridAutomaton::new_invariant",
                      "Constraint for event "<<event<<" is already defined in mode "<<mode);
    }
    mode._invariants.insert(event,invariant);
    mode._kinds.insert(event,EventKind::INVARIANT);
}

Void HybridAutomaton::_new_guard(DiscreteLocation location, DiscreteEvent event, ContinuousPredicate guard, EventKind kind)
{
    if(!this->has_location(location)) {
        ARIADNE_THROW(NonExistentModeError,"HybridAutomaton::new_guard",
                      "mode "<<location<<" is not a location of the automaton with locations "<<this->locations());
    }
    DiscreteMode& mode=this->_modes.value(location);
    if(mode._kinds.has_key(event)) {
        ARIADNE_THROW(MultipleGuardError,"HybridAutomaton::new_guard",
                      "Constraint for event "<<event<<" is already defined in mode "<<mode);
    }
    mode._guards.insert(event,guard);
    mode._kinds.insert(event,kind);
}


Void
HybridAutomaton::_new_action(DiscreteLocation location,
                             ContinuousPredicate invariant,
                             DiscreteEvent event,
                             ContinuousPredicate guard,
                             EventKind kind)
{
    if(!this->has_location(location)) {
        ARIADNE_THROW(NonExistentModeError,"HybridAutomaton::new_action",
                      "mode "<<location<<" is not a location of the automaton with locations "<<this->locations());
    }
    DiscreteMode& mode=this->_modes.value(location);
    if(mode._kinds.has_key(event)) {
        ARIADNE_THROW(MultipleGuardError,"HybridAutomaton::new_action",
                      "Constraint for event "<<event<<" is already defined in mode "<<mode);
    }
    mode._invariants.insert(event,invariant);
    mode._guards.insert(event,guard);
    mode._kinds.insert(event,kind);
}


Void
HybridAutomaton::_new_update(DiscreteLocation source,
                             DiscreteEvent event,
                             DiscreteLocation target,
                             List<PrimedRealAssignment> const& reset)
{
    if(!this->has_location(source)) {
        ARIADNE_THROW(NonExistentModeError,"HybridAutomaton::new_reset",
                      "Source mode "<<source<<" is not a location of the automaton with locations "<<this->locations());
    }

    if(!this->has_location(target)) {
        ARIADNE_THROW(NonExistentModeError,"HybridAutomaton::new_reset",
                      "Target mode "<<target<<" is not a location of the automaton with locations "<<this->locations());
    }

    DiscreteMode& source_mode=this->mode(source); // Non-constant since we may wish to update input variables
    if(source_mode._targets.has_key(event)) {
        ARIADNE_THROW(MultipleTransitionError,"HybridAutomaton::new_update",
                      "Update for event "<<event<<" is already defined in mode "<<source);
    }

    Set<RealVariable> target_auxiliary_variable=make_set(this->auxiliary_variables(target));
    if(!disjoint(target_auxiliary_variable,left_hand_sides(reset))) {
        ARIADNE_THROW(OverspecifiedResetError,"HybridAutomaton::new_update",
                      "reset "<<reset<<" for event "<<event<<" in source "<<source<<
                      " specifies variables "<<intersection(target_auxiliary_variable,left_hand_sides(reset))<<
                      " which are auxiliary variables in target "<<target);
    }

    if(!unique_elements(left_hand_sides(reset))) {
        ARIADNE_THROW(OverspecifiedResetError,"HybridAutomaton::new_update",
                      "reset "<<reset<<" for event "<<event<<" in source "<<source<<
                      "overspecifies variables "<<duplicate_elements(left_hand_sides(reset)));
    }

    List<RealVariable> target_state_space=this->state_variables(target);
    Set<RealVariable> target_state_variables(target_state_space);
    Set<RealVariable> reset_variables(left_hand_sides(reset));


    source_mode._targets.insert(event,target);
    source_mode._resets.insert(event,reset);
}


Void
HybridAutomaton::_new_transition(DiscreteLocation source,
                                 DiscreteEvent event,
                                 DiscreteLocation target,
                                 List<PrimedRealAssignment> const& reset,
                                 ContinuousPredicate guard,
                                 EventKind kind)
{
    if(kind==EventKind::URGENT || kind==EventKind::IMPACT) {
        this->_new_action(source,!guard,event,guard,kind);
    } else if(kind==EventKind::PERMISSIVE) {
        this->_new_guard(source,event,guard,kind);
    } else {
        ARIADNE_FAIL_MSG("Unhandled event kind "<<kind);
    }
    this->_new_update(source,event,target,reset);
}


Set<DiscreteLocation>
HybridAutomaton::locations() const
{
    return this->_modes.keys();
}




Bool
HybridAutomaton::has_location(DiscreteLocation location) const
{
    return this->_modes.has_key(location);
}

Bool
HybridAutomaton::has_mode(DiscreteLocation location) const
{
    for(Map<DiscreteLocation,DiscreteMode>::ConstIterator mode_iter=this->_modes.begin();
        mode_iter!=this->_modes.end(); ++mode_iter)
    {
        if(are_same(mode_iter->first,location)) {
            return true;
        }
    }
    return false;
}

Bool
HybridAutomaton::has_partial_mode(DiscreteLocation location) const
{
    for(Map<DiscreteLocation,DiscreteMode>::ConstIterator mode_iter=this->_modes.begin();
        mode_iter!=this->_modes.end(); ++mode_iter)
    {
        if(is_restriction(mode_iter->first,location)) {
            return true;
        }
    }
    return false;
}

Bool
HybridAutomaton::has_transition(DiscreteLocation source, DiscreteEvent event) const
{
   return this->has_partial_mode(source) && this->mode(source)._targets.has_key(event);
}

Bool
HybridAutomaton::has_invariant(DiscreteLocation source, DiscreteEvent event) const
{
   return this->has_partial_mode(source) && this->mode(source)._invariants.has_key(event);
}


Bool
HybridAutomaton::has_guard(DiscreteLocation source, DiscreteEvent event) const
{
   return this->has_partial_mode(source) && this->mode(source)._guards.has_key(event);
}



Map<DiscreteLocation,DiscreteMode> const&
HybridAutomaton::modes() const
{
    return this->_modes;
}

//Set<DiscreteMode>
//HybridAutomaton::modes() const
//{
//    return Set<DiscreteMode>(this->_modes.values());
//}

DiscreteMode&
HybridAutomaton::mode(DiscreteLocation location)
{
    const HybridAutomaton& self=*this;
    return const_cast<DiscreteMode&>(self.mode(location));

    if(!this->_modes.has_key(location)) {
        ARIADNE_THROW(SystemSpecificationError,"HybridAutomaton::mode(DiscreteLocation)",
                      location<<" is not a location of the automaton with locations "<<this->locations());
    }
    return this->_modes.find(location)->second;
}

const DiscreteMode&
HybridAutomaton::mode(DiscreteLocation location) const
{
    for(Map<DiscreteLocation,DiscreteMode>::ConstIterator mode_iter=this->_modes.begin();
        mode_iter!=this->_modes.end(); ++mode_iter)
    {
        const DiscreteLocation& partial_location=mode_iter->first;
        if(is_restriction(partial_location,location)) {
            return mode_iter->second;
        }
    }

    ARIADNE_THROW(SystemSpecificationError,"HybridAutomaton::mode(DiscreteLocation)",
                  location<<" does not define a mode of the automaton with locations "<<this->locations());
}

Writer<HybridAutomaton> HybridAutomaton::_default_writer(new VerboseHybridAutomatonWriter());

OutputStream&
HybridAutomaton::_write(OutputStream& os) const {
    return os << _default_writer(*this);
}

OutputStream&
VerboseHybridAutomatonWriter::_write(OutputStream& os, HybridAutomaton const& ha) const {
    os << "\nHybridAutomaton( \n  modes=\n";
    Set<DiscreteMode> modes(ha.modes().values());
    for(Set<DiscreteMode>::ConstIterator mode_iter=modes.begin();
        mode_iter!=modes.end(); ++mode_iter)
    {
        os << "    " <<*mode_iter<<",\n";
    }
    return os << ")\n";

}

OutputStream&
CompactHybridAutomatonWriter::_write(OutputStream& os, HybridAutomaton const& ha) const {
    os << ha.name() << ": ";
    Set<DiscreteMode> modes(ha.modes().values());
    Bool multiple_modes = (modes.size()>1);
    Writer<DiscreteMode> previous_mode_writer = DiscreteMode::default_writer();
    DiscreteMode::set_default_writer(CompactDiscreteModeWriter());
    for(Set<DiscreteMode>::ConstIterator mode_iter=modes.begin();
        mode_iter!=modes.end(); ++mode_iter)
    {
        if (multiple_modes) os << "\n - ";
        os << *mode_iter;
    }
    DiscreteMode::set_default_writer(previous_mode_writer);
    return os << "\n";
}



List<RealVariable>
HybridAutomaton::state_variables(DiscreteLocation location) const {
    List<RealVariable> result;
    const DiscreteMode& mode=this->mode(location);
    for(SizeType i=0; i!=mode._dynamic.size(); ++i) {
        result.append(mode._dynamic[i].lhs.base());
    }
    return result;
}

List<RealVariable>
HybridAutomaton::auxiliary_variables(DiscreteLocation location) const {
    List<RealVariable> result;
    const DiscreteMode& mode=this->mode(location);
    for(SizeType i=0; i!=mode._auxiliary.size(); ++i) {
        result.append(mode._auxiliary[i].lhs);
    }
    return result;
}

Set<RealVariable>
HybridAutomaton::input_variables(DiscreteLocation location) const {
    Set<UntypedVariable> used;
    Set<RealVariable> defined;
    const DiscreteMode& mode=this->mode(location);
    for(List<RealAssignment>::ConstIterator iter=mode._auxiliary.begin(); iter!=mode._auxiliary.end(); ++iter) {
        defined.insert(iter->lhs.base());
        used.adjoin(iter->rhs.arguments());
    }
    for(List<DottedRealAssignment>::ConstIterator iter=mode._dynamic.begin(); iter!=mode._dynamic.end(); ++iter) {
        defined.insert(iter->lhs.base());
        used.adjoin(iter->rhs.arguments());
    }
    for(Map<DiscreteEvent,ContinuousPredicate>::ConstIterator iter=mode._invariants.begin(); iter!=mode._invariants.end(); ++iter) {
        used.adjoin(iter->second.arguments());
    }
    for(Map<DiscreteEvent,ContinuousPredicate>::ConstIterator iter=mode._guards.begin(); iter!=mode._guards.end(); ++iter) {
        used.adjoin(iter->second.arguments());
    }
    for(Map<DiscreteEvent,List<PrimedRealAssignment> >::ConstIterator iter=mode._resets.begin(); iter!=mode._resets.end(); ++iter) {
        const List<PrimedRealAssignment>& reset=iter->second;
        for(List<PrimedRealAssignment>::ConstIterator rst_iter=reset.begin(); rst_iter!=reset.end(); ++rst_iter) {
            used.adjoin(rst_iter->rhs.arguments());
        }
    }
    Set<RealVariable> result;
    for(Set<UntypedVariable>::ConstIterator iter=used.begin(); iter!=used.end(); ++iter) {
        RealVariable var(iter->name());
        if(!defined.contains(var)) {
            result.insert(var);
        }
    }
    return result;
}


DiscreteLocation
HybridAutomaton::target(DiscreteLocation source, DiscreteEvent event) const {
    if(this->has_transition(source,event)) {
        return this->mode(source)._targets[event];
    } else {
        return source;
    }
}

List<RealAssignment>
HybridAutomaton::auxiliary_assignments(DiscreteLocation location) const {
    return this->mode(location)._auxiliary;
}

/*
List<RealAssignment>
HybridAutomaton::sorted_auxiliary_assignments(DiscreteLocation location) const {
    DiscreteMode const& mode=this->mode(location);
    assert(mode._sorted_auxiliary.size()==mode._auxiliary.size());
    return mode._sorted_auxiliary;
}
*/

List<DottedRealAssignment>
HybridAutomaton::dynamic_assignments(DiscreteLocation location) const {
    return this->mode(location)._dynamic;
}

List<PrimedRealAssignment>
HybridAutomaton::reset_assignments(DiscreteLocation source, DiscreteEvent event) const {
    if(this->has_transition(source,event)) {
        return this->mode(source)._resets[event];
    } else {
        List<PrimedRealAssignment> nonjump_assignments;
        List<RealVariable> state_variables=this->state_variables(source);
        for(SizeType i=0; i!=state_variables.size(); ++i) {
            nonjump_assignments.append(next(state_variables[i])=state_variables[i]);
        }
        return nonjump_assignments;
    }
}

ContinuousPredicate
HybridAutomaton::invariant_predicate(DiscreteLocation location, DiscreteEvent action) const {
    const DiscreteMode& mode=this->mode(location);
    if(mode._invariants.has_key(action)) {
        return mode._invariants[action];
    } else {
        return ContinuousPredicate(true);
    }
}

ContinuousPredicate
HybridAutomaton::guard_predicate(DiscreteLocation location, DiscreteEvent event) const {
    const DiscreteMode& mode=this->mode(location);
    if(mode._guards.has_key(event)) {
        return mode._guards[event];
    } else {
        return ContinuousPredicate(true);
    }
}


HybridSpace HybridAutomaton::state_space() const {
    MonolithicHybridSpace space;
    for(Map<DiscreteLocation,DiscreteMode>::ConstIterator iter=this->_modes.begin(); iter!=this->_modes.end(); ++iter) {
        const DiscreteLocation& loc=iter->first;
        space.new_location(loc,this->continuous_state_space(loc));
    }
    return space;
}

HybridSpace HybridAutomaton::state_auxiliary_space() const {
    MonolithicHybridSpace space;
    for(Map<DiscreteLocation,DiscreteMode>::ConstIterator iter=this->_modes.begin(); iter!=this->_modes.end(); ++iter) {
        const DiscreteLocation& loc=iter->first;
        space.new_location(loc,join(this->continuous_state_space(loc),this->continuous_auxiliary_space(loc)));
    }
    return space;
}

Set<DiscreteEvent> HybridAutomaton::events(DiscreteLocation location) const {
    const DiscreteMode& mode=this->mode(location);
    return join(join(mode._invariants.keys(),mode._guards.keys()),mode._targets.keys());
}

DimensionType HybridAutomaton::dimension(DiscreteLocation location) const {
    return this->mode(location)._dynamic.size();
}

RealSpace HybridAutomaton::continuous_state_space(DiscreteLocation location) const {
    return RealSpace(left_hand_sides(this->mode(location)._dynamic));
}

RealSpace HybridAutomaton::continuous_auxiliary_space(DiscreteLocation location) const {
    return RealSpace(left_hand_sides(this->mode(location)._sorted_auxiliary));
}

EventKind HybridAutomaton::event_kind(DiscreteLocation location, DiscreteEvent event) const {
    return this->mode(location)._kinds[event];
}

EffectiveVectorMultivariateFunction HybridAutomaton::auxiliary_function(DiscreteLocation location) const {
    DiscreteMode const& mode=this->mode(location);
    return Ariadne::make_auxiliary_function(this->continuous_state_space(location),mode._sorted_auxiliary);
}

EffectiveVectorMultivariateFunction HybridAutomaton::dynamic_function(DiscreteLocation location) const {
    DiscreteMode const& mode=this->mode(location);
    return Ariadne::make_dynamic_function(this->continuous_state_space(location),mode._sorted_auxiliary,mode._dynamic);
}

EffectiveVectorMultivariateFunction HybridAutomaton::reset_function(DiscreteLocation location, DiscreteEvent event) const {
    DiscreteMode const& mode=this->mode(location);
    return Ariadne::make_reset_function(this->continuous_state_space(location),mode._sorted_auxiliary,mode._resets[event]);
}

EffectiveScalarMultivariateFunction HybridAutomaton::invariant_function(DiscreteLocation location, DiscreteEvent event) const {
    DiscreteMode const& mode=this->mode(location);
    return Ariadne::make_constraint_function(this->continuous_state_space(location),mode._auxiliary,mode._invariants[event],Sign::NEGATIVE);
}

EffectiveScalarMultivariateFunction HybridAutomaton::guard_function(DiscreteLocation location, DiscreteEvent event) const {
    DiscreteMode const& mode=this->mode(location);
    return Ariadne::make_constraint_function(this->continuous_state_space(location),mode._sorted_auxiliary,mode._guards[event],Sign::POSITIVE);
}


Void HybridAutomaton::check_mode(DiscreteLocation location) const {
    const DiscreteMode& mode=this->mode(location);

    List<RealVariable> defined_real_variables(catenate(this->state_variables(location),this->auxiliary_variables(location)));
    Set<UntypedVariable> defined_variables(make_set(defined_real_variables));

    for(List<RealAssignment>::ConstIterator aux_iter=mode._auxiliary.begin(); aux_iter!=mode._auxiliary.end(); ++aux_iter) {
        if(!subset(aux_iter->rhs.arguments(),defined_variables)) {
            ARIADNE_THROW(UnderspecifiedDynamicError,"HybridAutomaton::check_mode(...)",
                          "Arguments "<<difference(aux_iter->rhs.arguments(),defined_variables)<<
                          " of "<<*aux_iter<<" are not defined in location "<<location<<" with variables "<<defined_variables);
        }
    }

    for(List<DottedRealAssignment>::ConstIterator dyn_iter=mode._dynamic.begin(); dyn_iter!=mode._dynamic.end(); ++dyn_iter) {
        if(!subset(dyn_iter->rhs.arguments(),defined_variables)) {
            ARIADNE_THROW(UnderspecifiedDynamicError,"HybridAutomaton::check_mode(...)",
                          "Arguments "<<difference(dyn_iter->rhs.arguments(),defined_variables)<<
                          " of "<<*dyn_iter<<" are not defined in location "<<location<<" with variables "<<defined_variables);
        }
    }

    for(Map<DiscreteEvent,ContinuousPredicate>::ConstIterator inv_iter=mode._invariants.begin(); inv_iter!=mode._invariants.end(); ++inv_iter) {
        if(!subset(inv_iter->second.arguments(),defined_variables)) {
            ARIADNE_THROW(UnderspecifiedConstraintError,"HybridAutomaton::check_mode(...)",
                          "Arguments "<<difference(inv_iter->second.arguments(),defined_variables)<<
                          " of "<<*inv_iter<<" are not defined in location "<<location<<" with variables "<<defined_variables);
        }
    }

    for(Map<DiscreteEvent,ContinuousPredicate>::ConstIterator grd_iter=mode._guards.begin(); grd_iter!=mode._guards.end(); ++grd_iter) {
        if(!subset(grd_iter->second.arguments(),defined_variables)) {
            ARIADNE_THROW(UnderspecifiedConstraintError,"HybridAutomaton::check_mode(...)",
                          "Arguments "<<difference(grd_iter->second.arguments(),defined_variables)<<
                          " of "<<*grd_iter<<" are not defined in location "<<location<<" with variables "<<defined_variables);
        }
    }

    for(Map<DiscreteEvent,List<PrimedRealAssignment> >::ConstIterator rst_iter=mode._resets.begin(); rst_iter!=mode._resets.end(); ++rst_iter) {
        const List<PrimedRealAssignment>& reset=rst_iter->second;
        for(List<PrimedRealAssignment>::ConstIterator pra_iter=reset.begin(); pra_iter!=reset.end(); ++pra_iter) {
            if(!subset(pra_iter->rhs.arguments(),defined_variables)) {
                ARIADNE_THROW(UnderspecifiedResetError,"HybridAutomaton::check_mode(...)",
                            "Arguments "<<difference(pra_iter->rhs.arguments(),defined_variables)<<
                            " of reset "<<*pra_iter<<" are not defined in location "<<location<<" with variables "<<defined_variables);
            }
        }
    }

    for(Map<DiscreteEvent,List<PrimedRealAssignment> >::ConstIterator rst_iter=mode._resets.begin(); rst_iter!=mode._resets.end(); ++rst_iter) {
        DiscreteLocation target = mode._targets[rst_iter->first];
        Set<RealVariable> reset_variables(left_hand_sides(rst_iter->second));
        Set<RealVariable> target_state_variables(this->state_variables(target));
        if(!subset(reset_variables,target_state_variables)) {
             ARIADNE_THROW(UnderspecifiedDynamicError,"HybridAutomaton::check_mode(...)",
                            "Variables "<<difference(reset_variables,target_state_variables)<<
                            " of reset"<<*rst_iter<<" are not defined in location "<<target<<" with state variables "<<target_state_variables);
        }
        if(!subset(target_state_variables,reset_variables)) {
             ARIADNE_THROW(UnderspecifiedResetError,"HybridAutomaton::check_mode(...)",
                            "Variables "<<difference(target_state_variables,reset_variables)<<
                            " in target"<<target<<" are not defined reset "<<*rst_iter);
        }
    }


}


Void HybridAutomaton::check_reachable_modes(const Set<DiscreteLocation>& initial_locations) const {
    Set<DiscreteLocation> reachable_locations=this->discrete_reachability(initial_locations);
    for(Set<DiscreteLocation>::ConstIterator iter=reachable_locations.begin(); iter!=reachable_locations.end(); ++iter) {
        this->check_mode(*iter);
    }
}

Set<DiscreteLocation> HybridAutomaton::discrete_reachability(const Set<DiscreteLocation>& initial_locations) const {
    const HybridAutomaton& automaton=*this;

    Set<DiscreteLocation> reached=initial_locations;
    Set<DiscreteLocation> working=initial_locations;
    Set<DiscreteLocation> found;

    Map<DiscreteLocation,Natural> steps;
    Natural step=0u;

    for(Set<DiscreteLocation>::ConstIterator initial_iter=initial_locations.begin(); initial_iter!=initial_locations.end(); ++initial_iter) {
        steps.insert(*initial_iter,step);
    }

    while(!working.empty()) {
        ++step;
        for(Set<DiscreteLocation>::ConstIterator source_iter=working.begin(); source_iter!=working.end(); ++source_iter) {
            DiscreteLocation location=*source_iter;
            Set<DiscreteEvent> events=automaton.events(location);
            for(Set<DiscreteEvent>::ConstIterator event_iter=events.begin(); event_iter!=events.end(); ++event_iter) {
                DiscreteEvent event=*event_iter;
                DiscreteLocation target=automaton.target(location,event);
                if(!reached.contains(target)) {
                    found.insert(target);
                    reached.insert(target);
                    steps.insert(target,step);
                }
           }

        }
        working.clear();
        working.swap(found);
    }

    return reached;
}


} // namespace Ariadne
