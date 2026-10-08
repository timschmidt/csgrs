/***************************************************************************
 *            hybrid/hybrid_automaton.hpp
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

/*! \file hybrid/hybrid_automaton.hpp
 *  \brief Main hybrid automaton class.
 */

#ifndef ARIADNE_HYBRID_AUTOMATON_HPP
#define ARIADNE_HYBRID_AUTOMATON_HPP

#include <string>
#include <iostream>
#include <vector>
#include <set>
#include <map>

#include "function/function.hpp"
#include "hybrid/discrete_location.hpp"
#include "hybrid/discrete_event.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/expression.hpp"
#include "conclog/logging.hpp"

#include "hybrid/hybrid_automaton_interface.hpp"

using namespace ConcLog;

namespace Ariadne {

class HybridTime;
class HybridSpace;

class DiscreteLocation;
class DiscreteMode;
class DiscreteTransition;
class HybridAutomaton;


//! \related HybridAutomaton
//! \brief A discrete transition of a hybrid automaton, representing an instantaneous
//! jump from one discrete mode to another, governed by an activation set and a reset map.
//!
//! A %DiscreteTransition can only be created using the new_transition() method in
//! the %HybridAutomaton class.
//!
//! An invariant is modelled by a discrete transition with negative event id and null reset pointer.
//!
//! \sa \ref HybridAutomaton, \ref DiscreteMode
class DiscreteTransition
{
    friend class DiscreteMode;
    friend class HybridAutomaton;
    friend class CompositeHybridAutomaton;
  private:
    //  The source of the discrete transition.
    DiscreteLocation _source;

    //  The discrete transition's identificator.
    DiscreteEvent _event;

    //  The target of the discrete transition.
    DiscreteLocation _target;

    //  The guard predicate of the discrete transition.
    ContinuousPredicate _guard;

    //  The reset assignments of the discrete transition.
    List<PrimedRealAssignment> _reset;

    //  The kind of the event.
    EventKind _kind;

  public:

    //! \brief The source mode of the discrete transition.
    DiscreteLocation source() const { return this->_source; }

    //! \brief The discrete event associated with the discrete transition.
    DiscreteEvent event() const { return this->_event; }

    //! \brief The target mode of the discrete transition.
    DiscreteLocation target() const { return this->_target; };

    //! \brief Write to an output stream.
    OutputStream& _write(OutputStream& os) const;
  private:
    DiscreteTransition(DiscreteLocation source,
                       DiscreteEvent event,
                       DiscreteLocation target,
                       const List<PrimedRealAssignment>& reset,
                       const ContinuousPredicate& guard,
                       EventKind kind);
};

inline OutputStream& operator<<(OutputStream& os, const DiscreteTransition& dt) {
    return dt._write(os); }

inline Bool operator<(const DiscreteTransition& transition1, const DiscreteTransition& transition2) {
    return transition1.event() < transition2.event()
        || (transition1.event() == transition2.event()
            && transition1.source() < transition2.source());
}

//! \related HybridAutomaton
//! \brief A discrete mode of a hybrid automaton, comprising continuous evolution given by a vector field
//! within and invariant constraint set.
//!
//! A %DiscreteMode can only be created using the new_mode() method in
//! the %HybridAutomaton class.
//!
//! \sa \ref HybridAutomaton, \ref DiscreteTransition
class DiscreteMode {
    friend class DiscreteTransition;
    friend class HybridAutomaton;
    friend class CompositeHybridAutomaton;
    friend class CompactDiscreteModeWriter;
    friend class VerboseDiscreteModeWriter;
  private:
    // The discrete mode's discrete state.
    DiscreteLocation _location;

    // The algebraic equations
    List<RealAssignment> _auxiliary;
    mutable List<RealAssignment> _sorted_auxiliary;

    // The algebraic equations
    List<DottedRealAssignment> _dynamic;

    // The discrete mode's invariants.
    Map<DiscreteEvent,ContinuousPredicate> _invariants;

    // The discrete mode's urgent guards.
    Map<DiscreteEvent, ContinuousPredicate> _guards;

    // The kind of each event
    Map<DiscreteEvent, EventKind> _kinds;

    // The target locations for the discrete transitions.
    Map<DiscreteEvent,DiscreteLocation> _targets;

    // The update assignments for the doscrete transitions.
    Map<DiscreteEvent, List<PrimedRealAssignment> > _resets;
  private:
    DiscreteMode(DiscreteLocation, List<RealAssignment>const&, List<DottedRealAssignment>const&);
    DiscreteMode(DiscreteLocation);
    DiscreteMode();
    static Writer<DiscreteMode> _default_writer;
  public:
    static Void set_default_writer(Writer<DiscreteMode> w) { _default_writer=w; }
    static Writer<DiscreteMode> default_writer() { return _default_writer; }
  public:
    //! \brief The mode's discrete state.
    DiscreteLocation location() const { return this->_location; }

    //! \brief Write to an output stream.
    OutputStream& _write(OutputStream& os) const;
};


inline OutputStream& operator<<(OutputStream& os, const DiscreteMode& dm) {
    return dm._write(os); }

class CompactDiscreteModeWriter : public WriterInterface<DiscreteMode> {
    virtual OutputStream& _write(OutputStream& os, DiscreteMode const& m) const final override;
};

class VerboseDiscreteModeWriter : public WriterInterface<DiscreteMode> {
    virtual OutputStream& _write(OutputStream& os, DiscreteMode const& m) const final override;
};

inline Bool operator<(const DiscreteMode& mode1, const DiscreteMode& mode2) {
    return mode1.location() < mode2.location(); }







//! \ingroup SystemModule
//! \ingroup HybridAutomataSubModule
//! \brief A hybrid automaton, comprising continuous-time behaviour
//! at each discrete mode, coupled by instantaneous discrete transitions.
//!  The state space is given by a hybrid set.
//!
//! A hybrid automaton is a dynamic system with evolution in both
//! continuous time and discrete time.
//! The state space is a product \f$X=\bigcup\{q\}\times X_q\f$
//! where \f$q\f$ is the <em>discrete state</em> and \f$X_q\f$
//! is the <em>continuous state space</em> of corresponding to
//! each discrete state.
//!
//! For each %DiscreteMode, the dynamics is given by a
//! %VectorField describing the continuous dynamics,
//! and a %Set giving an invariants which must be satisified at
//! all times.
//!
//! The discrete time behaviour is specified by %DiscreteTransition
//! objects.
//! Each discrete transition represents an jump from a \a source
//! mode to a \a target mode.
//! There can be at most one discrete transition in an automaton
//! with the same event and source.
//!
//! A discrete transision can either be \em urgent or \em permissive.
//! An urgent transition much occur as soon as it is activated.
//! A permissive transition may occur at any time it is activated,
//! but is only forced to occur if the continuous evolution is
//! blocked by an invariant.
//! An urgent transistion may not have an activation condition defined
//! elsewhere, even if this is permissive.
//!
//!
//! \sa \ref DiscreteMode, \ref DiscreteTransition, \ref CompositeHybridAutomaton
class HybridAutomaton
    : public HybridAutomatonInterface
{
    friend class CompositeHybridAutomaton;
    static Writer<HybridAutomaton> _default_writer;
public:
    static Void set_default_writer(Writer<HybridAutomaton> w) { _default_writer=w; }
    static Writer<HybridAutomaton> default_writer() { return _default_writer; }
  public:
    //! \brief The type used to represent time.
    typedef HybridTime TimeType;
    //! \brief The type used to represent real numbers.
    typedef double RealType ;
    //! \brief The type used to describe the state space.
    typedef HybridSpace StateSpaceType;

  protected:

    //! \brief The automaton name.
    Identifier _name;
    //! \brief The list of the hybrid automaton's discrete modes.
    Map< DiscreteLocation, DiscreteMode > _modes;

  protected:
    Void _new_mode(DiscreteLocation location,
                   List<RealAssignment> const& auxilary,
                   List<DottedRealAssignment> const& dynamic);

    Void _new_invariant(DiscreteLocation location,
                        ContinuousPredicate invariant,
                        DiscreteEvent event);

    Void _new_guard(DiscreteLocation location,
                     DiscreteEvent event,
                     ContinuousPredicate guard,
                     EventKind kind);

    Void _new_action(DiscreteLocation location,
                     ContinuousPredicate invariant,
                     DiscreteEvent event,
                     ContinuousPredicate guard,
                     EventKind kind);

    Void _new_update(DiscreteLocation location,
                     DiscreteEvent event,
                     DiscreteLocation target,
                     List<PrimedRealAssignment> const& reset);

    Void _new_transition(DiscreteLocation source,
                         DiscreteEvent event,
                         DiscreteLocation target,
                         List<PrimedRealAssignment> const& reset,
                         ContinuousPredicate guard,
                         EventKind kind);
  public:
    //!@{
    //! \name Constructors and destructors

    //! \brief Default constructor with "system" name.
    HybridAutomaton();

    //! \brief Construct an empty automaton with the given name
    HybridAutomaton(Identifier name);

    //! \brief Construct an empty automaton with the given name
    HybridAutomaton(
    		Identifier name,
    		const List<StringVariable>& discrete_variables);

    //! \brief Construct dynamically-allocated copy. (Not currently implemented)
    HybridAutomaton* clone() const { return new HybridAutomaton(*this); }

    //! \brief Destructor.
    virtual ~HybridAutomaton() = default;
    //!@}

    //!@{
    //! \name Methods for building the automaton.

    //! \brief Adds a discrete mode to the automaton.
    //! \details The \a location must be <em>distinguishable</em> from nodes already present in the automaton.
    //! The dependent variables of the \a auxiliary and \a dynamic assignments must all be distinct, following the <em>one-definition rule</em>.
    Void new_mode(DiscreteLocation location,
                  List<RealAssignment> const& auxiliary,
                  List<DottedRealAssignment> const& dynamic) {
        this->_new_mode(location,auxiliary,dynamic);
    }

    //! \brief Adds a discrete mode to the automaton.
    Void new_mode(DiscreteLocation location,
                  List<DottedRealAssignment> const& dynamic,
                  List<RealAssignment> const& auxiliary) {
        this->_new_mode(location,auxiliary,dynamic);
    }

    //! \brief Adds a discrete mode to the automaton.
    Void new_mode(DiscreteLocation location,
                  List<DottedRealAssignment> const& dynamic) {
        this->_new_mode(location,List<RealAssignment>(),dynamic);
    }

    //! \brief Adds a discrete mode to the automaton.
    Void new_mode(DiscreteLocation location,
                  List<RealAssignment> const& auxiliary) {
        this->_new_mode(location,auxiliary,List<DottedRealAssignment>());
    }

    //! \brief Adds a discrete mode to the automaton without any dynamics.
    Void new_mode(DiscreteLocation location) {
        this->_new_mode(location,List<RealAssignment>(),List<DottedRealAssignment>());
    }

    //! \brief Adds a discrete mode to the automaton.
    Void new_mode(List<RealAssignment> const& auxiliary, List<DottedRealAssignment> const& dynamic) {
        this->_new_mode(DiscreteLocation(),auxiliary,dynamic);
    }

    //! \brief Adds a discrete mode to the automaton.
    Void new_mode(List<DottedRealAssignment> const& dynamic, List<RealAssignment> const& auxiliary) {
        this->_new_mode(DiscreteLocation(),auxiliary,dynamic);
    }

    //! \brief Adds a discrete mode to the automaton.
    Void new_mode(List<DottedRealAssignment> const& dynamic) {
        this->_new_mode(DiscreteLocation(),List<RealAssignment>(),dynamic);
    }

    //! \brief Adds a discrete mode to the automaton.
    Void new_mode(List<RealAssignment> const& auxiliary) {
        this->_new_mode(DiscreteLocation(),auxiliary,List<DottedRealAssignment>());
    }

    //! \brief Adds a new internal/output event with a given enabling \a activation condition and triggering \a invariant.
    Void new_action(DiscreteLocation location,
                    ContinuousPredicate invariant,
                    DiscreteEvent event,
                    ContinuousPredicate activation,
                    EventKind kind=EventKind::PERMISSIVE) {
        ARIADNE_ASSERT(kind==EventKind::PERMISSIVE);
        this->_new_action(location,invariant,event,activation,kind);
    }

    //! \brief Adds a new urgent internal/output event with a given enabling \a guard.
    Void new_action(DiscreteLocation location,
                    DiscreteEvent event,
                    ContinuousPredicate guard,
                    EventKind kind=EventKind::URGENT) {
        ARIADNE_ASSERT(kind==EventKind::URGENT || kind==EventKind::IMPACT);
        this->_new_guard(location,event,guard,kind);
    }

    //! \brief Adds an invariant to the automaton.
    Void new_invariant(DiscreteLocation location,
                       ContinuousPredicate const& invariant,
                       DiscreteEvent event) {
        //this->_new_action(location,event,invariant,ContinuousPredicate(ValidatedKleenean(false)),INVARIANT);
        this->_new_invariant(location,invariant,event);
    }

    Void set_invariant(DiscreteLocation location,
                       ContinuousPredicate const& invariant,
                       DiscreteEvent event) {
        DiscreteMode& mode=this->_modes.value(location);
        mode._invariants[event] = invariant;
        mode._kinds[event] = EventKind::INVARIANT;
    }

    //! \brief Adds a guard to the automaton.
    Void new_guard(DiscreteLocation location,
                   DiscreteEvent event,
                   ContinuousPredicate const& guard,
                   EventKind kind) {
        //this->_new_action(location,event,ContinuousPredicate(true),guard,kind);
        this->_new_guard(location,event,guard,kind);
    }

    Void set_guard(DiscreteLocation location,
                   DiscreteEvent event,
                   ContinuousPredicate const& guard,
                   EventKind kind) {
        DiscreteMode& mode=this->_modes.value(location);
        mode._guards[event] = guard;
        mode._kinds[event] = kind;
    }


    //! \brief Adds an update/reset to the automaton.
    Void new_update(DiscreteLocation source,
                    DiscreteEvent event,
                    DiscreteLocation target,
                    List<PrimedRealAssignment> const& reset) {
        this->_new_update(source,event,target,reset);
    }

    //! \brief Adds a discrete update to the automaton to a mode with no continuous state variables.
    Void new_update(DiscreteLocation source,
                    DiscreteEvent event,
                    DiscreteLocation target) {
        this->_new_update(source,event,target,List<PrimedRealAssignment>());
    }

    //! \brief Adds a reset to an automaton with a single mode.
    Void new_update(DiscreteEvent event,
                    List<PrimedRealAssignment> const& reset) {
        this->_new_update(DiscreteLocation(),event,DiscreteLocation(),reset);
    }

    //! \brief Adds a reset to the automaton. (Same as new_update.)
    Void new_reset(DiscreteLocation source,
                   DiscreteEvent event,
                   DiscreteLocation target,
                   List<PrimedRealAssignment> const& reset) {
        this->_new_update(source,event,target,reset);
    }

    //! \brief Adds a reset to an automaton with no continuous state variables.
    Void new_reset(DiscreteLocation source,
                    DiscreteEvent event,
                    DiscreteLocation target) {
        this->_new_update(source,event,target,List<PrimedRealAssignment>());
    }

    //! \brief Adds a reset to an automaton with a single mode. (Same as new_update.)
    Void new_reset(DiscreteEvent event,
                    List<PrimedRealAssignment> const& reset) {
        this->_new_update(DiscreteLocation(),event,DiscreteLocation(),reset);
    }

    //! \brief Adds a discrete transition to the automaton using the discrete states to specify the source and target modes.
    //!
    //!    \param source is the transition's source location.
    //!    \param event is the transition's event.
    //!    \param target is the transition's target location.
    //!    \param reset is the transition's reset.
    //!    \param guard is the transition's activation region.
    //!    \param kind is a label indicating whether the transition is permissive, urgent, or an impact etc.
    Void new_transition(DiscreteLocation source,
                        DiscreteEvent event,
                        DiscreteLocation target,
                        List<PrimedRealAssignment> const& reset,
                        ContinuousPredicate guard,
                        EventKind kind) {
        this->_new_transition(source,event,target,reset,guard,kind);
    }

    //! \brief Adds a discrete transition to the automaton using the discrete states to specify the source and target modes.
    //! The reset is trivial. This form is for the case that there are no continuous state variables in the new location.
    Void new_transition(DiscreteLocation source,
                        DiscreteEvent event,
                        DiscreteLocation target,
                        ContinuousPredicate guard,
                        EventKind kind) {
        this->_new_transition(source,event,target,List<PrimedRealAssignment>(),guard,kind);
    }

    //! \brief Adds a discrete transition to the automaton using the discrete states to specify the source and target modes.
    //! The reset is trivial. This form is for the case that there are no continuous state variables in the new location.
    Void new_transition(DiscreteLocation source,
                        DiscreteEvent event,
                        ContinuousPredicate guard,
                        DiscreteLocation target,
                        EventKind kind) {
        this->_new_transition(source,event,target,List<PrimedRealAssignment>(),guard,kind);
    }

    //! \brief Adds an unguarded transition to the automaton.
    //! The guard is the constant "True" i.e. the event is an input event.
    Void new_transition(DiscreteLocation source,
                        DiscreteEvent event,
                        DiscreteLocation target,
                        List<PrimedRealAssignment> const& reset) {
        this->_new_update(source,event,target,reset);
    }

    //! \brief Adds an unguarded transition to the automaton.
    //! The guard is the constant "True" i.e. the event is an input event.
    Void new_transition(DiscreteLocation source,
                        DiscreteEvent event,
                        DiscreteLocation target) {
        this->_new_update(source,event,target,List<PrimedRealAssignment>());
    }

    //! \brief Adds a discrete transition for an automaton with a single mode.
    Void new_transition(DiscreteEvent event,
                        List<PrimedRealAssignment> const & reset,
                        ContinuousPredicate const& guard,
                        EventKind kind) {
        this->new_transition(DiscreteLocation(),event,DiscreteLocation(),reset,guard,kind);
    }

    //!@}

    //!@{
    //! \name Data access and queries.

    //! \brief The name of the automaton
    const Identifier& name() const { return _name; }

    //! \brief The modes of the automaton.
    Map<DiscreteLocation,DiscreteMode> const& modes() const;

    //! \brief The set of discrete locations.
    Set<DiscreteLocation> locations() const;

    //! \brief Test if the hybrid automaton has a discrete mode with the given exact \a location.
    Bool has_location(DiscreteLocation location) const;


    //! \brief The discrete mode with given discrete state.
    const DiscreteMode& mode(DiscreteLocation location) const;
    DiscreteMode& mode(DiscreteLocation location);

    //! \brief Checks validity of the mode for the given \a location.
    //! Only checks for underspecified dynamics; overspecification is determined at build-time.
    Void check_mode(DiscreteLocation location) const;
    //! \brief Runs check_mode() in any mode reachable under the discrete dynamics from the given initial locations.
    Void check_reachable_modes(const Set<DiscreteLocation>&) const;
    //! \brief Performs a discrete reachability analysis from the given initial locations.
    Set<DiscreteLocation> discrete_reachability(const Set<DiscreteLocation>&) const;
    //!@}

    //!@{
    //! \name Access for compositional hybrid automata.

    //! \brief The state (dotted) variables in the given location.
    List<RealVariable> state_variables(DiscreteLocation location) const;
    //! \brief The auxiliary (algebraic/output) variables in the given location.
    List<RealVariable> auxiliary_variables(DiscreteLocation location) const;
    //! \brief The input variables in the given location.
    Set<RealVariable> input_variables(DiscreteLocation location) const;
    //! \brief The target location when the \a event occurs in the \a source location.
    DiscreteLocation target_location(DiscreteLocation source, DiscreteEvent event) const;
    //! \brief The algebraic equations valid in the location.
    List<RealAssignment> auxiliary_assignments(DiscreteLocation location) const;
/*
    //! \brief The algebraic equations valid in the location, ordered so that the defining equation for a variable
    //! occurs before any equation using that variable.
    List<RealAssignment> sorted_auxiliary_assignments(DiscreteLocation location) const;
*/
    //! \brief The differential equations valid in the given location.
    List<DottedRealAssignment> dynamic_assignments(DiscreteLocation location) const;
    //! \brief The invariant predicates valid in the given location.
    ContinuousPredicate invariant_predicate(DiscreteLocation location, DiscreteEvent action) const;
    //! \brief The guard predicate for the given event in the given location.
    ContinuousPredicate guard_predicate(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The differential equations valid in the given location.
    List<PrimedRealAssignment> reset_assignments(DiscreteLocation source, DiscreteEvent event) const;
    //!@}

    //!@{
    //! \name Functions for conformance to HybridAutomatonInterface

    //! \brief The continuous state space for each location.
    virtual HybridSpace state_space() const;
    //! \brief The continuous space for each location.
    virtual HybridSpace state_auxiliary_space() const;
    //! \brief The continuous state space in the given location.
    virtual RealSpace continuous_state_space(DiscreteLocation) const;
    //! \brief The space of continuous auxiliary variables in the given location.
    virtual RealSpace continuous_auxiliary_space(DiscreteLocation) const;
    //! \brief The dimension of the continuous state space in the given location.
    virtual DimensionType dimension(DiscreteLocation) const;

    //! \brief Test if the hybrid automaton has a discrete mode corresponding to the given location.
    virtual Bool has_mode(DiscreteLocation location) const;
    //! \brief Test if the hybrid automaton has a discrete mode corresponding to a subset of variables of the given location.
    virtual Bool has_partial_mode(DiscreteLocation location) const;
    //! \brief Test if the hybrid automaton has an invariant (either explicit or from an urgent transition) with the given \a event label in \a location.
    virtual Bool has_invariant(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief Tests if the automaton has a nontrivial guard (activation) predicate fot the given \a event in \a location.
    virtual Bool has_guard(DiscreteLocation, DiscreteEvent) const;
    //! \brief Test if the hybrid automaton has a discrete transition starting from the given location with the given event.
    virtual Bool has_transition(DiscreteLocation source, DiscreteEvent event) const;
    //! \brief The events which are active in the given location.
    virtual Set<DiscreteEvent> events(DiscreteLocation) const;


    //! \brief The type of the event (urgent, permissive, impact etc).
    virtual EventKind event_kind(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The target for \a event from location \a source. Returns \a source if \a event is not present.
    virtual DiscreteLocation target(DiscreteLocation location, DiscreteEvent event) const;

    //! \brief The function outputting the auxiliary variables \f$y=h(x)\f$ in the location.
    virtual EffectiveVectorMultivariateFunction auxiliary_function(DiscreteLocation location) const;
    //! \brief The function outputting the differential equations \f$\dt{x}=f(x)\f$ in the location.
    virtual EffectiveVectorMultivariateFunction dynamic_function(DiscreteLocation location) const;
    //! \brief The invariant function \f$i(x)\leq 0\f$ corresponding to the given event.
    virtual EffectiveScalarMultivariateFunction invariant_function(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The guard function \f$g(x)\geq 0\f$ corresponding to the given event.
    virtual EffectiveScalarMultivariateFunction guard_function(DiscreteLocation location, DiscreteEvent event) const;
    //! \brief The reset function \f$x'=r(x)\f$ for the given event.
    virtual EffectiveVectorMultivariateFunction reset_function(DiscreteLocation location, DiscreteEvent event) const;

    //!@}

    //! \brief Write to an output stream.
    OutputStream& _write(OutputStream&) const;

};

inline OutputStream& operator<<(OutputStream& os, const HybridAutomaton& ha) {
    return ha._write(os);
}

class CompactHybridAutomatonWriter : public WriterInterface<HybridAutomaton> {
    virtual OutputStream& _write(OutputStream& os, HybridAutomaton const& ha) const final override;
};

class VerboseHybridAutomatonWriter : public WriterInterface<HybridAutomaton> {
    virtual OutputStream& _write(OutputStream& os, HybridAutomaton const& ha) const final override;
};


} // namespace Ariadne

#endif // ARIADNE_HYBRID_AUTOMATON_HPP
