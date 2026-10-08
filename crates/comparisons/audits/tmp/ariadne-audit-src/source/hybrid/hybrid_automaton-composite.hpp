/***************************************************************************
 *            hybrid_automaton-composite.hpp
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

/*! \file hybrid_automaton-composite.hpp
 *  \brief Main composite hybrid system class.
 */

#ifndef ARIADNE_COMPOSITE_HYBRID_AUTOMATON_HPP
#define ARIADNE_COMPOSITE_HYBRID_AUTOMATON_HPP

#include <string>
#include <iostream>
#include <vector>
#include <set>
#include <map>

#include "utility/lru_cache.hpp"
#include "function/function.hpp"
#include "hybrid/discrete_location.hpp"
#include "hybrid/discrete_event.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/expression.hpp"
#include "conclog/logging.hpp"

#include "hybrid/hybrid_automaton.hpp"
#include "hybrid/hybrid_automaton_interface.hpp"

using namespace ConcLog;

namespace Ariadne {

class HybridTime;
class HybridSpace;

class DiscreteLocation;
class DiscreteMode;
class DiscreteTransition;
class HybridAutomaton;
class CompositeHybridAutomaton;


//! \ingroup SystemModule
//! \ingroup HybridAutomataSubModule
//! \brief A hybrid automaton, formed by running a finite number of HybridAutomaton classes in parallel.
//!
//! \sa \ref HybridAutomaton
class CompositeHybridAutomaton
    : public HybridAutomatonInterface
{
    static Writer<CompositeHybridAutomaton> _default_writer;
  public:
    static Void set_default_writer(Writer<CompositeHybridAutomaton> w) { _default_writer=w; }
    static Writer<CompositeHybridAutomaton> default_writer() { return _default_writer; }
  public:
    typedef HybridTime TimeType;
    typedef List<HybridAutomaton>::ConstIterator ConstIterator;
  public:
    //!@{
    //! \name Constructors.

    //! \brief Default constructor creates "system" named automaton.
    CompositeHybridAutomaton();
    //! \brief Constructor with name.
    CompositeHybridAutomaton(Identifier name);
    //! \brief Convert a single atomic hybrid automaton to a composite hybrid automaton for analysis.
    CompositeHybridAutomaton(const HybridAutomaton&);
    //! \brief Create the parallel composition of a list of atomic hybrid automata, with given name.
    CompositeHybridAutomaton(
    		Identifier name,
    		const List<HybridAutomaton>&);
    //! \brief Create the parallel composition of a list of atomic hybrid automata, with composed name.
    CompositeHybridAutomaton(const List<HybridAutomaton>&);

    //! \brief Construct dynamically-allocated copy. (Not currently implemented)
    CompositeHybridAutomaton* clone() const { return new CompositeHybridAutomaton(*this); }

    //! \brief Virtual destructor.
    virtual ~CompositeHybridAutomaton() = default;

    //!@}

    //!@{
    //! \name Methods for querying the component automata.

    //! \brief The name of the automaton
    const Identifier& name() const { return _name; }

    //! \brief The number of component automata.
    Nat number_of_components() const;
    //! \brief The \a i<sup>th</sup> component automaton.
    const HybridAutomaton& component(Nat i) const;

    //! \brief Iterator begin
    ConstIterator begin() const { return _components.begin(); }
    //! \brief Iterator end
    ConstIterator end() const { return _components.end(); }

    //!@}

    //!@{
    //! \name Methods for finding the modes, transitions and events of the composite automaton.

    //! The mode corresponding to the given location.
    DiscreteMode const& mode(DiscreteLocation) const;
    //! \brief Tests if the automaton has a mode corresponding to the given location.
    Bool has_mode(DiscreteLocation) const;
    //! \brief Test if the hybrid automaton has a discrete mode corresponding to a subset of variables of the given location.
    Bool has_partial_mode(DiscreteLocation location) const;
    //! \brief Tests if the automaton has an invariant corresponding to the given location and event.
    Bool has_invariant(DiscreteLocation, DiscreteEvent) const;
    //! \brief Tests if the automaton has an invariant or transition corresponding to the given location and event.
    Bool has_guard(DiscreteLocation, DiscreteEvent) const;
    //! \brief Tests if the automaton has a transition corresponding to the given location and event.
    Bool has_transition(DiscreteLocation, DiscreteEvent) const;

    //!@}

    //!@{
    //! \name Methods fo rextracting the continuous dynamics of the automaton.


    //! \brief The target location when \a event occurs in the \a source location.
    DiscreteLocation target_location(DiscreteLocation source, DiscreteEvent event) const;

    //! \brief The continuous variables which are defined in the location.
    List<RealVariable> variables(DiscreteLocation) const;
    //! \brief The continuous variables which are state variables in the location.
    //! The state variables are those defined by a differential equation i.e. the dotted variables.
    List<RealVariable> state_variables(DiscreteLocation) const;
    //! \brief The dependent continuous variables which are not state variables in the location.
    //! These variables are defined by algebraic equations.
    List<RealVariable> auxiliary_variables(DiscreteLocation) const;
    //! \brief The continuous variables which are not state or auxiliary variables in the location,
    //! but are required in one of the dynamic equations, constraints or resets.
    Set<RealVariable> input_variables(DiscreteLocation) const;

    //! \brief The algebraic equations valid in the location.
    List<RealAssignment> auxiliary_assignments(DiscreteLocation location) const;
    //! \brief The algebraic equations valid in the location, ordered so that the defining equation for a variable
    //! occurs before any equation using that variable.
    List<RealAssignment> sorted_auxiliary_assignments(DiscreteLocation location) const;
    //! \brief The differential equations valid in the location.
    List<DottedRealAssignment> dynamic_assignments(DiscreteLocation location) const;
    //! \brief The reset equations used when the \a event occurs in the \a source location.
    List<PrimedRealAssignment> reset_assignments(DiscreteLocation source, DiscreteEvent event) const;
    //! \brief The invariant (time-can-progress predicates) corresponding to the given \a event.
    ContinuousPredicate invariant_predicate(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The guard (activation predicate) corresponding to the given \a event.
    ContinuousPredicate guard_predicate( DiscreteLocation location, DiscreteEvent event) const;
    //!@}


    //!@{
    //! \name Functions for conformance to HybridAutomatonInterface

    //! \brief The continuous state space for each location.
    virtual HybridSpace state_space() const;
    //! \brief The continuous space for each location.
    virtual HybridSpace state_auxiliary_space() const;
    //! \brief The continuous state space in the given location.
    RealSpace continuous_state_space(DiscreteLocation) const;
    //! \brief The space of continuous auxiliary variables in the given location.
    RealSpace continuous_auxiliary_space(DiscreteLocation) const;
    //! \brief The space of continuous state and auxiliary variables in the given location.
    RealSpace continuous_state_auxiliary_space(DiscreteLocation) const;
    //! \brief The dimension of the continuous state space in the given location.
    DimensionType dimension(DiscreteLocation) const;

    //! \brief The events which are active in the given location.
    Set<DiscreteEvent> events(DiscreteLocation) const;
    //! \brief The target for \a event from location \a source. Returns \a source if \a event is not present.
    DiscreteLocation target(DiscreteLocation location, DiscreteEvent event) const;

    //! \brief The function outputting the auxiliary variables \f$y=h(x)\f$ in the location.
    EffectiveVectorMultivariateFunction auxiliary_function(DiscreteLocation location) const;
    //! \brief The function outputting the differential equations \f$dx/dt =f(x)\f$ in the location.
    EffectiveVectorMultivariateFunction dynamic_function(DiscreteLocation location) const;
    //! \brief The reset function \f$x'=r(x)\f$ for the given event.
    EffectiveVectorMultivariateFunction reset_function(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The invariant function \f$i(x)\leq 0\f$ corresponding to the given event.
    EffectiveScalarMultivariateFunction invariant_function(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The guard function \f$g(x)\geq 0\f$ corresponding to the given event.
    EffectiveScalarMultivariateFunction guard_function(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The type of the event (urgent, permissive, impact etc).
    EventKind event_kind(DiscreteLocation location, DiscreteEvent event) const;

    //!@}


    //!@{
    //! \name Methods for checking the validity of the automaton.

    //! \brief Checks whether the equations valid in the location are valid.
    //!
    //! Includes a check for algebraic dependencies, over-defined variables, under-defined variables, and
    //! variables which should be defined in a reset but are not.
    Void check_mode(DiscreteLocation) const;
    //! \brief Runs check_mode() in any mode reachable under the discrete dynamics from the given initial locations.
    Void check_reachable_modes(const Set<DiscreteLocation>&) const;
    //!@}

    //!@{
    //! \name Discrete reachability analysis.

    //! \brief Performs a discrete reachability analysis from the given initial locations.
    Set<DiscreteLocation> discrete_reachability(const Set<DiscreteLocation>&) const;
    //!@}

    //! \brief Write to an output stream.
    OutputStream& _write(OutputStream&) const;
  protected:
    Identifier _name;
  private:
    List<HybridAutomaton> _components;
    mutable LRUCache<DiscreteLocation,DiscreteMode> _modes_cache;
    Void _cache_mode(DiscreteLocation) const;
};

CompositeHybridAutomaton parallel_composition(const List<HybridAutomaton>& components);

inline OutputStream& operator<<(OutputStream& os, const CompositeHybridAutomaton& ha) {
    return ha._write(os); }

class CompactCompositeHybridAutomatonWriter : public WriterInterface<CompositeHybridAutomaton> {
    virtual OutputStream& _write(OutputStream& os, CompositeHybridAutomaton const& ha) const final override;
};

class VerboseCompositeHybridAutomatonWriter : public WriterInterface<CompositeHybridAutomaton> {
    virtual OutputStream& _write(OutputStream& os, CompositeHybridAutomaton const& ha) const final override;
};

} // namespace Ariadne

#endif // ARIADNE_HYBRID_AUTOMATON_HPP
