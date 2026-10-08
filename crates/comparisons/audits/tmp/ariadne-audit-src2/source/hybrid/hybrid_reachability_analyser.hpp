/***************************************************************************
 *            hybrid/hybrid_reachability_analyser.hpp
 *
 *  Copyright  2006-20  Alberto Casagrande, Pieter Collins
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

/*! \file hybrid/hybrid_reachability_analyser.hpp
 *  \brief Methods for computing reachable sets of hybrid systems.
 */

#ifndef ARIADNE_HYBRID_REACHABILITY_ANALYSER_HPP
#define ARIADNE_HYBRID_REACHABILITY_ANALYSER_HPP


#include "solvers/configuration_interface.hpp"
#include "dynamics/reachability_analyser.hpp"
#include "hybrid/hybrid_set.decl.hpp"
#include "hybrid/hybrid_set_interface.hpp"
#include "hybrid/hybrid_evolver_interface.hpp"
#include "hybrid/hybrid_reachability_analyser_interface.hpp"

#include "hybrid/hybrid_orbit.hpp"
#include "hybrid/hybrid_grid.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "hybrid/hybrid_storage.hpp"

#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {

template<class ES> class Orbit;

class DiscreteLocation;
class HybridAutomatonInterface;

class HybridGrid;
class HybridGridCell;
class HybridGridTreePaving;
class HybridStorage;

template<class ES> class HybridListSet;

class HybridEvolverInterface;

class HybridAutomaton;

template<> struct SafetyCertificate<HybridSpace> {
    ValidatedSierpinskian is_safe;
    HybridStorage chain_reach_set;
    HybridStorage safe_set;
};

using HybridSafetyCertificate = SafetyCertificate<HybridSpace>;
using HybridReachabilityAnalyserConfiguration = ReachabilityAnalyserConfiguration<HybridAutomatonInterface>;

template<> class ReachabilityAnalyserConfiguration<HybridAutomatonInterface>;

//! \ingroup AnalysisModule
//! \ingroup HybridDynamicsSubModule
//! \brief A class for performing reachability analysis on a hybrid system.
class HybridReachabilityAnalyser
    : public ReachabilityAnalyser<HybridAutomatonInterface>
{
  public:
    typedef HybridReachabilityAnalyserConfiguration ConfigurationType;
  public:
    //!@{
    //! \name Constructors and destructors

    //! \brief Construct from an evolver.
    HybridReachabilityAnalyser(const HybridEvolverInterface& evolver);

    //! \brief Make a dynamically-allocated copy.
    virtual HybridReachabilityAnalyser* clone() const;
    //!@}

};


//! \brief Configuration for a HybridReachabilityAnalyser, essentially for controlling the
//!    accuracy of discretised evolution methods and reachability analysis.
template<> class ReachabilityAnalyserConfiguration<HybridAutomatonInterface> : public ConfigurationInterface {

  public:
    //! \brief The unsigned integer type.
    typedef Nat UnsignedIntType;
    //! \brief The real type.
    typedef ExactDouble RealType;

    //! \brief The real type.
    typedef HybridAutomatonInterface::TimeType TimeType;

    //! \brief Default constructor gives reasonable values.
    ReachabilityAnalyserConfiguration(ReachabilityAnalyser<HybridAutomatonInterface>& analyser);

    virtual ~ReachabilityAnalyserConfiguration() = default;

  private:

    //! \brief Reference back to the main class, for consistency checking.
    ReachabilityAnalyser<HybridAutomatonInterface>& _analyser;

    //! \brief The time after which infinite-time evolution routines
    //! may approximate computed sets on a grid.
    //! \details
    //! This value should be set to the time after which the transient
    //! behaviour has mostly died out. If there are no transients (i.e. the system evolves
    //! for a certain time and then stops), then this parameter should be set to a value
    //! slightly higher than the maximum running time.
    //! <br>
    //! Setting this value to the time at which transients die out can improve the
    //! speed and accuracy of the computations.
    //!  <br>
    //! This property is only used by chain_reach() routines.
    RealType _transient_time;

    //! \brief The number of discrete steps after which infinite-time evolution
    //! routines may approximate computed sets on the grid.
    //! \details
    //! Note that the transients are assumed to have died out after <em>either</em>
    //! transient_time or transient_steps has been reached.
    //! <br>
    //! For example, if a hybrid system makes at most three steps before settling down
    //! into its limiting behaviour, then this parameter should be set to three.
    //! <br>
    //! Setting this value to the number of steps at which transients die out can improve the
    //! speed and accuracy of the computations.
    //! <br>
    //! This property is only used by chain_reach() routines.
    //! \sa #transient_time
    UnsignedIntType _transient_steps;
    // (See the #transient_time parameter.)

    //! \brief The time after which an upper evolution or reachability analysis routine
    //! may approximate computed sets on a grid, in order to use previously cached
    //! integration results for the grid.
    //! \details
    //! Increasing this parameter improves the accuracy of the computations.
    //! Setting this parameter too low usually results in meaningless computations.
    //! As a rule of thumb, a typical system trajectory should move at least four
    //! times the grid size between locking to the grid. <br>
    //! For forced oscillators, this parameter should be set to the forcing time,
    //! or a multiple or fraction thereof.
    //! <br>
    //! This property is only used for continuous-time computation.
    RealType _lock_to_grid_time;

    //! \brief A set of events after which an upper evolution or reachability analysis
    //! routine should approximate computed sets on a grid, in order to use previously
    //! cached integration results for the grid.
    //! \details
    //! Choosing regularly-occurring events may greatly improve the speed and
    //! accuracy of the computation.
    //! <br>
    //! This property is only used for continuous-time computation.
    Set<DiscreteEvent> _lock_to_grid_events;

    //! \brief The time after which an evolver may approximate computed sets on a grid,
    //! in order to use previously cached results for the grid.
    //! \details
    //! Increasing this parameter may improve the accuracy of the computations.
    //! If there is recurrence in the system, then this parameter should be set to
    //! the average recurrence time, if known.
    //!  <br>
    //! This property is only used for discrete-time computation.
    UnsignedIntType _lock_to_grid_steps;

    //! \brief The fineness used for approximation on a grid for computations using upper semantics.
    //! \details
    //! Increasing this value increases the accuracy of the computation.
    //!  <br>
    //! This property is only used in upper_evolve(), upper_reach() and chain_reach() routines.
    UnsignedIntType _maximum_grid_fineness;

    //! \brief The maximum extent used for approximation on a grid for chain reachability computations.
    //! \details
    //! Increasing this value increases the bounding domain over which computation is performed.
    //!  <br>
    //! This property is only used in the chain_reach() routines.
    UnsignedIntType _maximum_grid_extent;

    //! \brief The explicit bounding domain for approximation on a grid for chain reachability computations.
    //! \details
    //! If defined, this property combines with _maximum_grid_extent to define the actual bounding domain.
    //!  <br>
    //! This property is only used in the chain_reach() routines.
    SharedPointer<HybridExactBoxes> _bounding_domain_ptr;

    //! \brief The grid used for approximation.
    //! \details Provided as a pointer in order to avoid double construction. It must always be defined.
    SharedPointer<HybridGrid> _grid_ptr;

    //! \brief The policy for overspill in outer chain reach computation.
    //! \details A OVERSPILL_IGNORE simply bypasses checks, OVERSPILL_WARNING issues a warning and OVERSPILL_ERROR
    //! issues a OuterChainOverspill exception.
    ChainOverspillPolicy _outer_overspill_policy;

  public:
    //FIXME: Be consistent in use of time and steps

    const TimeType transient_time() const { return HybridTime(Dyadic(_transient_time),_transient_steps); }
    const TimeType lock_to_grid_time() const { return HybridTime(Dyadic(_lock_to_grid_time),_lock_to_grid_steps); }

    //    const RealType& transient_time() const { return _transient_time; }
    Void set_transient_time(const Double value) { _transient_time = RealType(value); }

    const UnsignedIntType& transient_steps() const { return _transient_steps; }
    Void set_transient_steps(const UnsignedIntType value) { _transient_steps = value; }

//    const RealType& lock_to_grid_time() const { return _lock_to_grid_time; }
    Void set_lock_to_grid_time(const Double value) { _lock_to_grid_time = RealType(value); }

    const UnsignedIntType& lock_to_grid_steps() const { return _lock_to_grid_steps; }
    Void set_lock_to_grid_steps(const UnsignedIntType value) { _lock_to_grid_steps = value; }

    const Set<DiscreteEvent>& lock_to_grid_events() const { return _lock_to_grid_events; }
    Void set_lock_to_grid_events(const Set<DiscreteEvent> value) { _lock_to_grid_events = value; }

    const UnsignedIntType& maximum_grid_fineness() const { return _maximum_grid_fineness; }
    Void set_maximum_grid_fineness(const UnsignedIntType value) { _maximum_grid_fineness = value; }

    const UnsignedIntType& maximum_grid_extent() const { return _maximum_grid_extent; }
    Void set_maximum_grid_extent(const UnsignedIntType value) { _maximum_grid_extent = value; }

    const std::shared_ptr<HybridExactBoxes>& bounding_domain_ptr() const { return _bounding_domain_ptr; }
    //! \brief Check the consistency in respect to the system space, then set the bounding domain.
    Void set_bounding_domain_ptr(const std::shared_ptr<HybridExactBoxes> value);

    const HybridExactBoxes& bounding_domain() const { return *_bounding_domain_ptr; }
    Void set_bounding_domain(const HybridExactBoxes& bounding_domain);

    const HybridGrid& grid() const { return *_grid_ptr; }
    HybridGrid& grid() { return *_grid_ptr; }
    //! \brief Check the consistency in respect to the system space, then set the grid.
    Void set_grid(const std::shared_ptr<HybridGrid> value_ptr);

    Void set_scaling(const RealVariable& v, ApproximateDouble s) {
        dynamic_cast<SimpleHybridScalings&>(static_cast<HybridScalingsInterface&>(this->grid().scalings())) .set_scaling(v,s); }

    const ChainOverspillPolicy& outer_overspill_policy() const { return _outer_overspill_policy; }
    Void set_outer_overspill_policy(const ChainOverspillPolicy value) { _outer_overspill_policy = value; }

  public:

    virtual OutputStream& _write(OutputStream& os) const;
};


} // namespace Ariadne


#endif // ARIADNE_HYBRID_REACHABILITY_ANALYSER_HPP
