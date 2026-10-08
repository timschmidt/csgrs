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

#include "hybrid/hybrid_automaton-composite.hpp"

namespace Ariadne {

const SizeType MODES_CACHE_SIZE = 4;

class CompositeHybridStateSpace
    : public HybridSpaceInterface
{
  public:
    ~CompositeHybridStateSpace() { _system_ptr = 0; }
    CompositeHybridStateSpace(const CompositeHybridAutomaton& ha) : _system_ptr(&ha) { }
    virtual CompositeHybridStateSpace* clone() const { return new CompositeHybridStateSpace(*this); }
    virtual Bool has_location(const DiscreteLocation& q) const { return this->_system_ptr->has_mode(q); }
    virtual RealSpace operator[](const DiscreteLocation& q) const { return this->_system_ptr->continuous_state_space(q); }
    virtual OutputStream& _write(OutputStream& os) const { return os << "CompositeHybridSpace( " << this->_system_ptr->name() << " )"; }
    ValidatedKleenean operator==(const HybridSpaceInterface& other) const {
        const CompositeHybridStateSpace* chs_ptr = dynamic_cast<const CompositeHybridStateSpace* >(&other);
        if (!chs_ptr) return indeterminate;
        if (&*chs_ptr->_system_ptr == &*_system_ptr)
            return true;
        else return indeterminate;
    }
  private:
    const CompositeHybridAutomaton* _system_ptr;
};

class CompositeHybridSpace
    : public HybridSpaceInterface
{
  public:
    ~CompositeHybridSpace() { _system_ptr = 0; }
    CompositeHybridSpace(const CompositeHybridAutomaton& ha) : _system_ptr(&ha) { }
    virtual CompositeHybridSpace* clone() const { return new CompositeHybridSpace(*this); }
    virtual Bool has_location(const DiscreteLocation& q) const { return this->_system_ptr->has_mode(q); }
    virtual RealSpace operator[](const DiscreteLocation& q) const { return this->_system_ptr->continuous_state_auxiliary_space(q); }
    virtual OutputStream& _write(OutputStream& os) const { return os << "CompositeHybridSpace( " << this->_system_ptr->name() << " )"; }
    ValidatedKleenean operator==(const HybridSpaceInterface& other) const {
        const CompositeHybridSpace* chs_ptr = dynamic_cast<const CompositeHybridSpace* >(&other);
        if (!chs_ptr) return indeterminate;
        if (&*chs_ptr->_system_ptr == &*_system_ptr)
            return true;
        else return indeterminate;
    }
  private:
    const CompositeHybridAutomaton* _system_ptr;
};

namespace {

Identifier name_composition(const List<HybridAutomaton>& components)
{
    List<HybridAutomaton>::ConstIterator comp_it = components.begin();

    ARIADNE_ASSERT_MSG(comp_it != components.end(), "The components list is empty.");

    Identifier composed_name = comp_it->name();
    ++comp_it;
    for (; comp_it != components.end(); ++comp_it)
        composed_name += "&" + comp_it->name();

    return composed_name;
}

} // namespace

// Declare functions from hybrid_automaton.cpp
EffectiveVectorMultivariateFunction make_auxiliary_function(Space<Real> const& space, List<RealAssignment> const& sorted_algebraic);

CompositeHybridAutomaton::CompositeHybridAutomaton(const HybridAutomaton& automaton)
    : _name(automaton.name()), _components(1u,automaton), _modes_cache(MODES_CACHE_SIZE) { }

CompositeHybridAutomaton::CompositeHybridAutomaton(const List<HybridAutomaton>& components)
    : _name(name_composition(components)), _components(components), _modes_cache(MODES_CACHE_SIZE) { }

CompositeHybridAutomaton::CompositeHybridAutomaton(Identifier name, const List<HybridAutomaton>& components)
    : _name(name), _components(components), _modes_cache(MODES_CACHE_SIZE) { }

CompositeHybridAutomaton::CompositeHybridAutomaton()
    : CompositeHybridAutomaton("system",List<HybridAutomaton>()) { }

CompositeHybridAutomaton::CompositeHybridAutomaton(Identifier name)
    : CompositeHybridAutomaton(name,List<HybridAutomaton>()) { }

Nat
CompositeHybridAutomaton::number_of_components() const
{
    return this->_components.size();
}

const HybridAutomaton&
CompositeHybridAutomaton::component(Nat k) const {
    return this->_components[k];
}


typedef Map<DiscreteLocation,DiscreteMode>::ConstIterator ModesConstIterator;


Bool
CompositeHybridAutomaton::has_mode(DiscreteLocation location) const {
    DiscreteLocation automaton_location;
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        if(!this->_components[i].has_partial_mode(location)) {
            return false;
        }
        automaton_location.adjoin(_components[i].mode(location).location());
    }
    ARIADNE_ASSERT(is_restriction(automaton_location,location));
    return are_same(automaton_location,location);
}

Bool
CompositeHybridAutomaton::has_partial_mode(DiscreteLocation location) const {
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        if(!this->_components[i].has_partial_mode(location)) {
            return false;
        }
    }
    return true;
}

Bool
CompositeHybridAutomaton::has_invariant(DiscreteLocation location, DiscreteEvent event) const
{
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        if(this->_components[i].has_invariant(location,event)) {
            return true;
        }
    }
    return false;
}

Bool
CompositeHybridAutomaton::has_guard(DiscreteLocation location, DiscreteEvent event) const
{
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        if(this->_components[i].has_guard(location,event)) {
            return true;
        }
    }
    return false;
}

Bool
CompositeHybridAutomaton::has_transition(DiscreteLocation source, DiscreteEvent event) const {
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        if(this->_components[i].has_transition(source,event)) {
            return true;
        }
    }
    return false;
}


DiscreteMode const& CompositeHybridAutomaton::mode(DiscreteLocation location) const
{
    if(not this->_modes_cache.has_label(location)) this->_cache_mode(location);
    return this->_modes_cache.get(location);
}

Void CompositeHybridAutomaton::_cache_mode(DiscreteLocation location) const
{
    DiscreteMode cached_mode;

    for(List<HybridAutomaton>::ConstIterator component_iter=this->_components.begin();
        component_iter!=this->_components.end(); ++component_iter)
    {
        const DiscreteMode& component_mode=component_iter->mode(location);

        cached_mode._location.adjoin(component_mode._location);
        cached_mode._auxiliary.append(component_mode._auxiliary);
        cached_mode._dynamic.append(component_mode._dynamic);

        for(Map<DiscreteEvent,ContinuousPredicate>::ConstIterator invariant_iter=component_mode._invariants.begin();
            invariant_iter!=component_mode._invariants.end(); ++invariant_iter)
        {
            cached_mode._invariants.insert(*invariant_iter);
        }

        for(Map<DiscreteEvent,ContinuousPredicate>::ConstIterator guard_iter=component_mode._guards.begin();
            guard_iter!=component_mode._guards.end(); ++guard_iter)
        {
            cached_mode._guards.insert(*guard_iter);
        }

        for(Map<DiscreteEvent,EventKind>::ConstIterator kind_iter=component_mode._kinds.begin();
            kind_iter!=component_mode._kinds.end(); ++kind_iter)
        {
            cached_mode._kinds.insert(*kind_iter);
        }

        // FIXME: Need to introduce missing updates
        for(Map<DiscreteEvent,DiscreteLocation>::ConstIterator target_iter=component_mode._targets.begin();
            target_iter!=component_mode._targets.end(); ++target_iter)
        {
            DiscreteLocation& target=cached_mode._targets[target_iter->first];
            target=join(target,target_iter->second);
        }

        for(Map<DiscreteEvent,List<PrimedRealAssignment> >::ConstIterator reset_iter=component_mode._resets.begin();
            reset_iter!=component_mode._resets.end(); ++reset_iter)
        {
            List<PrimedRealAssignment>& reset=cached_mode._resets[reset_iter->first];
            reset.append(reset_iter->second);
        }
    }

    ARIADNE_ASSERT(join(cached_mode._invariants.keys(),cached_mode._guards.keys()) == cached_mode._kinds.keys());
    ARIADNE_ASSERT(cached_mode._targets.keys() == cached_mode._resets.keys());
    ARIADNE_ASSERT(subset(cached_mode._guards.keys(),cached_mode._resets.keys()));

    if (cached_mode._guards.keys() != cached_mode._resets.keys()) {
        Set<DiscreteEvent> transitions_missing_guards = difference(cached_mode._resets.keys(),cached_mode._guards.keys());
        for (DiscreteEvent event : transitions_missing_guards) {
            ARIADNE_ERROR("Event "<<event<<" in location " << location << " triggers a transition to mode "<<cached_mode._targets[event]<<" with reset "<<cached_mode._resets[event]
                            << " but no guard is defined.")
            for (HybridAutomaton const& component : this->_components) {
                for (DiscreteMode const& mode : component.modes().values()) {
                        if (mode._kinds.has_key(event)) {
                            ARIADNE_NOTIFY("  Component " << component.name() << " defines guard " << mode._guards[event] << " for " << event << " in location " << mode.location());
                    }
                }
            }
        }
        DiscreteEvent event = *transitions_missing_guards.begin();
        ARIADNE_THROW(MissingGuardError,"CompositeHybridAutomaton::mode(DiscreteLocation)","Automaton has location "<<location<<" with event "<<event<<" without a guard, but with transition to location "<<cached_mode._targets[event]<<" with resets "<<cached_mode._resets[event]);
    }

    ARIADNE_DEBUG_ASSERT(is_restriction(cached_mode._location,location));
    // TODO: Should we throw an exception here? I think it's worth allowing this to go through...
    if(false and not are_same(cached_mode._location,location)) {
        ARIADNE_THROW(OverspecifiedLocationException,"CompositeHybridAutomaton::mode(DiscreteLocation)","Automaton has mode "<<cached_mode._location<<" overspecified by location "<<location);
    }

    _modes_cache.put(location,cached_mode);
}


Set<DiscreteEvent>
CompositeHybridAutomaton::events(DiscreteLocation location) const
{
    DiscreteMode const& mode=this->mode(location);
    return join(join(mode._invariants.keys(),mode._guards.keys()),mode._targets.keys());
}

EventKind
CompositeHybridAutomaton::event_kind(DiscreteLocation location, DiscreteEvent event) const
{
    return this->mode(location)._kinds[event];
}



DiscreteLocation
CompositeHybridAutomaton::target(DiscreteLocation source, DiscreteEvent event) const {
    DiscreteLocation result;
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        DiscreteMode const& component_mode = this->_components[i].mode(source);
        if(component_mode._targets.has_key(event)) {
            result.adjoin(component_mode._targets[event]);
        } else {
            result.adjoin(component_mode._location);
        }
    }
    return result;
}


DimensionType
CompositeHybridAutomaton::dimension(DiscreteLocation location) const {
    return this->state_variables(location).size();
}

HybridSpace
CompositeHybridAutomaton::state_space() const {
    return new CompositeHybridStateSpace(*this);
}

HybridSpace
CompositeHybridAutomaton::state_auxiliary_space() const {
    return new CompositeHybridSpace(*this);
}

RealSpace
CompositeHybridAutomaton::continuous_state_space(DiscreteLocation location) const {
    return RealSpace(this->state_variables(location));
}

RealSpace
CompositeHybridAutomaton::continuous_auxiliary_space(DiscreteLocation location) const {
    return RealSpace(left_hand_sides(this->sorted_auxiliary_assignments(location)));
}

RealSpace
CompositeHybridAutomaton::continuous_state_auxiliary_space(DiscreteLocation location) const {
    return join(this->continuous_state_space(location),this->continuous_auxiliary_space(location));
}

List<RealVariable>
CompositeHybridAutomaton::variables(DiscreteLocation location) const {
    return catenate(this->state_variables(location),this->auxiliary_variables(location));
}

List<RealVariable>
CompositeHybridAutomaton::state_variables(DiscreteLocation location) const {
    List<RealVariable> result;
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        ARIADNE_ASSERT(this->_components[i].has_partial_mode(location));
        // Append state space in mode i; note that this automatically checks whether a variable is already present
        result.append(this->_components[i].state_variables(location));
    }
    return result;
}

List<RealVariable>
CompositeHybridAutomaton::auxiliary_variables(DiscreteLocation location) const {
    List<RealVariable> result;
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        // Append state space in mode i; note that this automatically checks whether a variable is already present
        result.append(this->_components[i].auxiliary_variables(location));
    }
    return result;
}

// Find all algebraic equations valid in the location
List<RealAssignment>
CompositeHybridAutomaton::auxiliary_assignments(DiscreteLocation location) const {
    List<RealAssignment> result;
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        result.append(this->_components[i].auxiliary_assignments(location));
    }
    return result;
}

// Find all algebraic equations valid in the location
List<RealAssignment>
CompositeHybridAutomaton::sorted_auxiliary_assignments(DiscreteLocation location) const {
    return algebraic_sort(this->auxiliary_assignments(location));
}

List<DottedRealAssignment>
CompositeHybridAutomaton::dynamic_assignments(DiscreteLocation location) const {
    List<DottedRealAssignment> result;
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        result.append(this->_components[i].dynamic_assignments(location));
    }
    // No need to sort result since dotted variables cannot appear in the right-hand side (currently)
    return result;
}


List<PrimedRealAssignment>
CompositeHybridAutomaton::reset_assignments(DiscreteLocation location, DiscreteEvent event) const {
    List<PrimedRealAssignment> result;
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        result.append(this->_components[i].reset_assignments(location,event));
    }
    return result;
}


ContinuousPredicate
CompositeHybridAutomaton::invariant_predicate(DiscreteLocation location, DiscreteEvent event) const {
    ContinuousPredicate result(true);
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        ContinuousPredicate guard=this->_components[i].invariant_predicate(location,event);
        if(is_constant(result,true) || is_constant(guard,false)) { result=guard; }
        else if(is_constant(guard,true) || is_constant(result,false)) { }
        else { result = result && guard; }
    }
    return result;
}


ContinuousPredicate
CompositeHybridAutomaton::guard_predicate(DiscreteLocation location, DiscreteEvent event) const {
    ContinuousPredicate result(true);
    for(SizeType i=0; i!=this->_components.size(); ++i) {
        ContinuousPredicate guard=this->_components[i].guard_predicate(location,event);
        if(is_constant(result,true) || is_constant(guard,false)) { result=guard; }
        else if(is_constant(guard,true) || is_constant(result,false)) { }
        else { result = result && guard; }
    }
    return result;
}





EffectiveVectorMultivariateFunction
CompositeHybridAutomaton::auxiliary_function(DiscreteLocation location) const {
    Space<Real> space=this->state_variables(location);
    List<RealAssignment> algebraic=this->sorted_auxiliary_assignments(location);
    return Ariadne::make_auxiliary_function(space,algebraic);
}

EffectiveVectorMultivariateFunction
CompositeHybridAutomaton::dynamic_function(DiscreteLocation location) const {
    RealExpression default_expression;
    Space<Real> space=this->state_variables(location);
    List<RealAssignment> algebraic=this->auxiliary_assignments(location);
    List<DottedRealAssignment> differential=this->dynamic_assignments(location);
    Vector<RealExpression> results(differential.size(),default_expression);
    for(SizeType i=0; i!=differential.size(); ++i) { results[space.index(differential[i].lhs.base())]=substitute(differential[i].rhs,algebraic); }
    return make_function(space,results);
}

EffectiveVectorMultivariateFunction
CompositeHybridAutomaton::reset_function(DiscreteLocation source, DiscreteEvent event) const {
    RealExpression default_expression;
    DiscreteLocation target=this->target(source,event);
    Space<Real> source_space=this->state_variables(source);
    Space<Real> target_space=this->state_variables(target);
    List<RealAssignment> algebraic=this->auxiliary_assignments(source);
    List<PrimedRealAssignment> update=this->reset_assignments(source,event);
    Vector<RealExpression> results(update.size(),default_expression);
    for(SizeType i=0; i!=update.size(); ++i) { results[target_space.index(update[i].lhs.base())]=substitute(update[i].rhs,algebraic); }
    return make_function(source_space,results);
}

EffectiveScalarMultivariateFunction
CompositeHybridAutomaton::invariant_function(DiscreteLocation location, DiscreteEvent event) const {
    Space<Real> space=this->state_variables(location);
    List<RealAssignment> algebraic=this->auxiliary_assignments(location);
    RealExpression invariant=indicator(invariant_predicate(location,event),Sign::NEGATIVE);
    return make_function(space,substitute(invariant,algebraic));
}

EffectiveScalarMultivariateFunction
CompositeHybridAutomaton::guard_function(DiscreteLocation location, DiscreteEvent event) const {
    Space<Real> space=this->state_variables(location);
    List<RealAssignment> algebraic=this->auxiliary_assignments(location);
    RealExpression guard=indicator(guard_predicate(location,event),Sign::POSITIVE);
    return make_function(space,substitute(guard,algebraic));
}

Writer<CompositeHybridAutomaton> CompositeHybridAutomaton::_default_writer(new VerboseCompositeHybridAutomatonWriter());

OutputStream&
CompositeHybridAutomaton::_write(OutputStream& os) const {
    return os << _default_writer(*this);
}

OutputStream&
VerboseCompositeHybridAutomatonWriter::_write(OutputStream& os, CompositeHybridAutomaton const& ha) const {
    os << "CompositeHybridAutomaton(\n";
    for(SizeType i = 0; i < ha.number_of_components(); ++i) {
        os << ha.component(i);
    }
    return os << "\n)\n";
}

OutputStream&
CompactCompositeHybridAutomatonWriter::_write(OutputStream& os, CompositeHybridAutomaton const& ha) const {
    os << ha.name() << " {\n";
    Writer<HybridAutomaton> previous_writer = HybridAutomaton::default_writer();
    HybridAutomaton::set_default_writer(CompactHybridAutomatonWriter());
    for(SizeType i = 0; i < ha.number_of_components(); ++i) {
        os << ha.component(i);
    }
    HybridAutomaton::set_default_writer(previous_writer);
    return os << "}\n";
}



template<class LHS, class RHS>
List<LHS> assigned(const List< Assignment<LHS,RHS> >& assignments) {
    List<LHS> result; result.reserve(assignments.size());
    for(SizeType i=0; i!=assignments.size(); ++i) {
        result.append(assignments[i].lhs);
    }
    return result;
}

template<class T>
Set<RealVariable> real_arguments(const Expression<T>& expression) {
    Set<RealVariable> result;
    Set<UntypedVariable> args=expression.arguments();
    for(Set<UntypedVariable>::ConstIterator iter=args.begin(); iter!=args.end(); ++iter) {
        if(iter->type()==variable_type<Real>()) {
            result.insert(RealVariable(iter->name()));
        }
    }
    return result;
}

template<class LHS, class RHS>
Set<RealVariable> real_arguments(const Assignment<LHS,RHS>& assignment) {
    return real_arguments(assignment.rhs);
}

template<class LHS, class RHS>
Set<RealVariable> real_arguments(const List< Assignment<LHS,RHS> >& assignments) {
    Set<RealVariable> result;
    for(SizeType i=0; i!=assignments.size(); ++i) {
        result.adjoin(real_arguments(assignments[i]));
    }
    return result;
}

template<class Tp, template<class> class Dec>
List< Variable<Tp> > base(const List< Dec<Tp> >& variables) {
    List< Variable<Tp> > result;
    for(SizeType i=0; i!=variables.size(); ++i) {
        result.append(variables[i].base());
    }
    return result;
}

template<class Var>
Bool unique(const List<Var>& variables) {
    Set<Var> found;
    for(SizeType i=0; i!=variables.size(); ++i) {
        if(found.contains(variables[i])) {
            return false;
        } else {
            found.insert(variables[i]);
        }
    }
    return true;
}

template<class Var>
Set<Var> duplicates(const List<Var>& variables) {
    Set<Var> result;
    Set<Var> found;
    for(SizeType i=0; i!=variables.size(); ++i) {
        if(found.contains(variables[i])) {
            result.insert(variables[i]);
        } else {
            found.insert(variables[i]);
        }
    }
    return result;
}

namespace {

// Order the assignments so that each variable depends only on previous ones, assuming the given input variables.
List<RealAssignment> order(const List<RealAssignment>& assignments, const Set<RealVariable>& inputs)
{
    List<RealAssignment> result;

    Map< RealVariable, Set<RealVariable> > unresolved_dependencies;
    Map< RealVariable, SizeType > assignment_table;

    for(SizeType i=0; i!=assignments.size(); ++i) {
        unresolved_dependencies.insert(assignments[i].lhs,difference(real_arguments(assignments[i].rhs),inputs));
        assignment_table.insert(assignments[i].lhs,i);
    }

    while(!unresolved_dependencies.empty()) {
        // Check for an which has been resolved and add to list of ok equations
        Bool found=false;
        for(Map< RealVariable, Set<RealVariable> >::Iterator iter=unresolved_dependencies.begin();
            iter!=unresolved_dependencies.end(); ++iter)
        {
            if(iter->second.empty()) {
                RealVariable variable=iter->first;
                result.append(assignments[assignment_table[variable]]);
                for(Map< RealVariable, Set<RealVariable> >::Iterator rem_iter=unresolved_dependencies.begin();
                        rem_iter!=unresolved_dependencies.end(); ++rem_iter) {
                    rem_iter->second.erase(variable);
                }
                unresolved_dependencies.erase(iter);
                found=true;
                break;
            }
        }

        if(!found) {
            ARIADNE_THROW(AlgebraicLoopError,"order(List<Assignment>,Set<Variable>)","Cannot order assignments "<<assignments
                            <<" with inputs "<<inputs<<" due to algebraic loops or undefined variables.");
        }
    }

    ARIADNE_ASSERT(result.size()==assignments.size());
    return result;
}

}


Void
CompositeHybridAutomaton::check_mode(DiscreteLocation location) const {
    List<RealAssignment> equations=this->auxiliary_assignments(location);
    List<DottedRealAssignment> dynamics=this->dynamic_assignments(location);

    List<RealVariable> state_variables=base(assigned(dynamics));
    List<RealVariable> auxiliary_variables=assigned(equations);
    List<RealVariable> location_variables=catenate(state_variables,auxiliary_variables);

    if(!unique(location_variables)) {
        ARIADNE_THROW(OverspecifiedDynamicError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                      "Variables "<<duplicates(location_variables)<<" in location "<<location<<" with defining equations "<<equations<<" and "<<dynamics<<" have more than one defining equation.");
    }

    Set<RealVariable> result_variables=make_set(location_variables);

    Set<RealVariable> undefined_variables=difference(real_arguments(equations),result_variables);
    if(!undefined_variables.empty()) {
        ARIADNE_THROW(UnderspecifiedDynamicError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                      "Variables "<<undefined_variables<<" are used in the definition of the algebraic equations "<<equations<<" in location "<<location<<" with state variables "<<state_variables<<", but are not defined.");
    }

    undefined_variables=difference(real_arguments(dynamics),result_variables);
    if(!undefined_variables.empty()) {
        ARIADNE_THROW(UnderspecifiedDynamicError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                      "Variables "<<undefined_variables<<" are used in the definition of the differential equations "<<dynamics<<" in location "<<location<<" with state variables "<<state_variables<<", but are not defined.");
    }

    List<RealAssignment> ordered_equations=Ariadne::order(equations,make_set(state_variables));

    Set<DiscreteEvent> events=this->events(location);

    for(Set<DiscreteEvent>::Iterator event_iter=events.begin(); event_iter!=events.end(); ++event_iter) {
        DiscreteEvent event=*event_iter;
        const ContinuousPredicate& invariant=this->invariant_predicate(location,event);
        if(!subset(real_arguments(invariant),result_variables)) {
            undefined_variables=difference(real_arguments(invariant),result_variables);
            ARIADNE_THROW(UnderspecifiedConstraintError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                          "Variables "<<undefined_variables<<" are used in the invariant "<<invariant<<" with label \""<<events<<"\" in location "<<location<<" with variables "<<location_variables<<", but are not defined.");
        }
    }

    for(Set<DiscreteEvent>::ConstIterator event_iter=events.begin(); event_iter!=events.end(); ++event_iter) {
        // Get transition information
        DiscreteEvent event=*event_iter;
        DiscreteLocation target=this->target(location,event);
        ContinuousPredicate guard=this->guard_predicate(location,event);
        List<PrimedRealAssignment> reset=this->reset_assignments(location,event);

        Set<RealVariable> target_state_variables=make_set(this->state_variables(target));
        List<RealVariable> reset_variables=base(assigned(reset));

        // Check kind of transitions of guard
        Set<Identifier> urgent_in;
        Set<Identifier> permissive_in;
        for(List<HybridAutomaton>::ConstIterator component_iter=this->_components.begin();
            component_iter!=this->_components.end(); ++component_iter)
        {
            HybridAutomaton const& component = *component_iter;
            if(component.has_guard(location,event)) {
                EventKind kind=component.event_kind(location,event);
                if (kind == EventKind::URGENT or kind == EventKind::IMPACT) {
                    urgent_in.insert(component.name());
                } else {
                    ContinuousPredicate local_guard=component.guard_predicate(location,event);
                    if(!is_constant(local_guard,true)) {
                        permissive_in.insert(component.name());
                    }
                }
            }
        }
        if (urgent_in.size()>1) {
            ARIADNE_THROW(MultipleUrgentDeclarationError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                          "Event "<<event<<" in location "<<location<<" is declared URGENT or IMPACT by multiple components "<<urgent_in<<".");
        }
        if (urgent_in.size()==1 && permissive_in.size()>0) {
            ARIADNE_THROW(UrgentDeclarationWithMultipleGuardsException,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                          "Event "<<event<<" in location "<<location<<" is declared URGENT or IMPACT by component "<<*urgent_in.begin()<<", but also has a nontrivial guard in components "<<permissive_in<<"; an urgent event may only have a guard declared in the restricting component.")
        }

        // Check arguments of guard
        if(!subset(real_arguments(guard),result_variables)) {
            undefined_variables=difference(real_arguments(guard),result_variables);
            ARIADNE_THROW(UnderspecifiedConstraintError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                          "Variables "<<undefined_variables<<" are used in the guard "<<guard<<" for event \""<<event<<"\" in location "<<location<<" with variables "<<location_variables<<", but are not defined.");
        }

        // Check arguments of reset
        if(!subset(real_arguments(reset),result_variables)) {
            undefined_variables=difference(real_arguments(reset),result_variables);
            ARIADNE_THROW(UnderspecifiedResetError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                          "Variables "<<undefined_variables<<" are used in the reset "<<reset<<" for event \""<<event<<"\" in location "<<location<<" with variables "<<location_variables<<", but are not defined.");
        }

        // Check duplication of reset
        if(!unique(reset_variables)) {
        ARIADNE_THROW(OverspecifiedResetError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                      "Variables "<<duplicates(reset_variables)<<" have more than one defining equation in resets "<<reset<<" for event \""<<event<<"\" in location "<<location<<".");
        }

        Set<RealVariable> updated_variables=make_set(reset_variables);

        if(!subset(updated_variables,target_state_variables)) {
            Set<RealVariable> extra_variables=difference(updated_variables,target_state_variables);
            ARIADNE_THROW(SystemSpecificationError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                          "Variables "<<extra_variables<<" are defined in the reset "<<reset<<" for event \""<<event<<"\" in location "<<location<<", but are not state variables "<<target_state_variables<<" in the target "<<target<<".");
        }

        if(!subset(target_state_variables,updated_variables)) {
            undefined_variables=difference(target_state_variables,updated_variables);
            ARIADNE_THROW(UnderspecifiedResetError,"CompositeHybridAutomaton::check_mode(DiscreteLocation)",
                          "Variables "<<undefined_variables<<" are state variables in location "<<target<<", but are not defined in the reset "<<reset<<" for the transition \""<<event<<"\" from location "<<location<<".");
        }

    } // Finished checking transitions

    return;
}


Void
CompositeHybridAutomaton::check_reachable_modes(const Set<DiscreteLocation>& initial_locations) const
{
    Set<DiscreteLocation> reachable_locations=this->discrete_reachability(initial_locations);
    for(Set<DiscreteLocation>::ConstIterator iter=reachable_locations.begin(); iter!=reachable_locations.end(); ++iter) {
        this->check_mode(*iter);
    }
}

Set<DiscreteLocation>
CompositeHybridAutomaton::discrete_reachability(const Set<DiscreteLocation>& initial_locations) const
{
    CONCLOG_SCOPE_CREATE;
    const CompositeHybridAutomaton& automaton=*this;

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
            CONCLOG_PRINTLN_AT(1,"new_mode");
            DiscreteLocation location=*source_iter;
            CONCLOG_PRINTLN_AT(1,"mode: "<<location<<":");
            CONCLOG_PRINTLN_AT(2,"auxiliary="<<automaton.auxiliary_assignments(location));
            CONCLOG_PRINTLN_AT(2,"function="<<automaton.auxiliary_function(location));
            CONCLOG_PRINTLN_AT(2,"dynamic="<<automaton.dynamic_assignments(location));
            CONCLOG_PRINTLN_AT(2,"function="<<automaton.dynamic_function(location));

            Set<DiscreteEvent> events=automaton.events(location);
            CONCLOG_PRINTLN_AT(1,"events: "<<events);
            for(Set<DiscreteEvent>::ConstIterator event_iter=events.begin(); event_iter!=events.end(); ++event_iter) {
                DiscreteEvent event=*event_iter;
                CONCLOG_PRINTLN_AT(2,"event:"<<event);
                DiscreteLocation target=automaton.target(location,event);
                CONCLOG_PRINTLN_AT(3,"transition: "<<event<<" -> "<<target);
                CONCLOG_PRINTLN_AT(3,"reset="<<automaton.reset_assignments(location,event));
                CONCLOG_PRINTLN_AT(3,"function="<<automaton.reset_function(location,event));
                CONCLOG_PRINTLN_AT(3,"guard="<<automaton.guard_predicate(location,event));
                CONCLOG_PRINTLN_AT(3,"function="<<automaton.guard_function(location,event));
                if(!reached.contains(target)) {
                    found.insert(target);
                    reached.insert(target);
                    steps.insert(target,step);
                }
           }

        }
        CONCLOG_PRINTLN_AT(1,"step "<<step<<" found: "<<found);
        working.clear();
        working.swap(found);
    }

    return reached;
}

CompositeHybridAutomaton parallel_composition(const List<HybridAutomaton>& components)
{
    return CompositeHybridAutomaton(name_composition(components),components);
}

inline HybridAutomaton flatten(const CompositeHybridAutomaton& composite_automaton, const List<DiscreteLocation>& locations)
{
    ARIADNE_NOT_IMPLEMENTED;
}


} // namespace Ariadne
