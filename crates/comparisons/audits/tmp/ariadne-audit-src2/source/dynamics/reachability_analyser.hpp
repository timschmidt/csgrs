/***************************************************************************
 *            dynamics/reachability_analyser.hpp
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

/*! \file dynamics/reachability_analyser.hpp
 *  \brief Methods for computing reachable sets of hybrid systems.
 */

#ifndef ARIADNE_REACHABILITY_ANALYSER_HPP
#define ARIADNE_REACHABILITY_ANALYSER_HPP

#include "solvers/configuration_interface.hpp"
#include "geometry/set.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/function_set.hpp"

#include "dynamics/enclosure.hpp"
#include "dynamics/storage.hpp"
#include "dynamics/orbit.hpp"
#include "dynamics/vector_field_evolver.hpp"
#include "dynamics/reachability_analyser_interface.hpp"

#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {

template<class ES> class Orbit;

class DiscreteLocation;
class AutomatonInterface;

class Grid;
class GridCell;
class GridTreePaving;
class Storage;

template<class ES> class ListSet;

template<class SYS> class ReachabilityAnalyserConfiguration;
template<class SYS> class ReachabilityAnalyser;

enum class ChainOverspillPolicy : std::uint8_t { IGNORE, WARNING, ERROR };

using ContinuousReachabilityAnalyser = ReachabilityAnalyser<VectorField>;

template<> struct SafetyCertificate<EuclideanSpace> {
    const ValidatedSierpinskian is_safe;
    const LabelledStorage chain_reach_set;
    const LabelledStorage safe_set;
};

//! \ingroup DynamicsModule
//! \brief A class for performing reachability analysis on a hybrid system.
template<class SYS> class ReachabilityAnalyser
    : public ReachabilityAnalyserInterface<SYS>
{
    typedef ReachabilityAnalyserInterface<SYS> Interface;
  public:
    typedef ReachabilityAnalyserConfiguration<SYS> ConfigurationType;
    using typename Interface::SystemType;
    using typename Interface::TimeType;
    using typename Interface::EvolverType;
    using typename Interface::BoundingDomainType;
    using typename Interface::BoundedSetInterfaceType;
    using typename Interface::OvertSetInterfaceType;
    using typename Interface::OpenSetInterfaceType;
    using typename Interface::CompactSetInterfaceType;
    using typename Interface::LocatedSetInterfaceType;
    using typename Interface::RegularSetInterfaceType;
    using typename Interface::RegularLocatedSetInterfaceType;
    using typename Interface::SetApproximationType;
    using typename Interface::SafetyCertificateType;

    typedef typename Interface::EnclosureType EnclosureType;
    typedef typename Interface::StorageType StorageType;
    using GridType = typename StorageType::GridType;
    using PavingType = typename StorageType::PavingType;
  private:
  protected:
    SharedPointer<SystemType> _system;
    SharedPointer<EvolverType> _evolver;
    SharedPointer<ConfigurationType> _configuration;
  public:
    //!@{
    //! \name Constructors and destructors

    //! \brief Virtual destructor
    virtual ~ReachabilityAnalyser() = default;

    //! \brief Construct from an evolver.
    ReachabilityAnalyser(const EvolverType& evolver);

    //! \brief Make a dynamically-allocated copy.
    virtual ReachabilityAnalyser* clone() const;
    //!@}

    //!@{
    //! \name Methods to set and get the configuration properties of the class.

    //! \brief A reference to the analyser's configuration parameters.
    ConfigurationType& configuration() { return *this->_configuration; }
    const ConfigurationType& configuration() const { return *this->_configuration; }

    //! \brief A reference to the analyser's evolver parameter.
    const EvolverType& evolver() const { return *this->_evolver; }

    //! \brief Get the system associated with the analyser.
    virtual const SystemType& system() const override { return *this->_system; }
    //!@}



    //!@{
    //! \name Evaluation of systems on abstract sets
    //! \brief Compute a lower-approximation to the set obtained by evolving the system starting in \a initial_set until \a time.
    virtual StorageType
    lower_evolve(const OvertSetInterfaceType& initial_set,
                 const TimeType& steps) const override;

    //! \brief Compute a lower-approximation to the reachable and evolved sets of the system starting in \a initial_set up to \a time.
    virtual Pair<StorageType,StorageType>
    lower_reach_evolve(const OvertSetInterfaceType& initial_set,
                       const TimeType& time) const override;

    //! \brief Compute a lower-approximation to the reachable set of the system starting in \a initial_set up to \a time.
    virtual StorageType
    lower_reach(const OvertSetInterfaceType& initial_set,
                const TimeType& time) const override;

    //! \brief Compute an infinite-time lower-approximation to the reachable set of the system starting in \a initial_set.
    virtual StorageType
    lower_reach(const OvertSetInterfaceType& initial_set) const override;

    //! \brief Compute an upper-approximation to the set obtained by evolving the system starting in \a initial_set up to \a time.
    virtual StorageType
    upper_evolve(const CompactSetInterfaceType& initial_set,
                 const TimeType& time) const override;

    //! \brief Compute upper-approximations to the reachable and evolved sets of the system starting in \a initial_set up to \a time.
    virtual Pair<StorageType,StorageType>
    upper_reach_evolve(const CompactSetInterfaceType& initial_set,
                       const TimeType& time) const override;

    //! \brief Compute an upper-approximation to the reachable set of the system starting in \a initial_set up to \a time.
    virtual StorageType
    upper_reach(const CompactSetInterfaceType& initial_set,
                const TimeType& time) const override;


    //! \brief Compute a (possibly-restricted) approximation to the outer chain-reachable set of the system starting in \a initial_set.
    virtual StorageType
    outer_chain_reach(const CompactSetInterfaceType& initial_set) const override;

    //! \brief Test if the system is safe.
    virtual SafetyCertificateType
    verify_safety(const CompactSetInterfaceType& initial_set, const OpenSetInterfaceType& safe_set) const override;

    //!@}

  protected:
    // Helper functions for operators on lists of sets.
    Pair<StorageType,StorageType> _reach_evolve_resume(const ListSet<EnclosureType>& initial_enclosures,
            const TimeType& time, const Nat accuracy, ListSet<EnclosureType>& evolve_enclosures,
            Semantics semantics, const EvolverType& evolver) const;
    StorageType _upper_reach(const StorageType& set, const TimeType& time,
            const Nat accuracy, const EvolverType& evolver) const;
    StorageType _upper_evolve(const StorageType& set, const TimeType& time,
            const Nat accuracy, const EvolverType& evolver) const;
    Void _adjoin_upper_reach_evolve(StorageType& reach_set, StorageType& final_set,
                                    const StorageType& set, const TimeType& time,
                                    const Nat accuracy, const EvolverType& evolver) const;
    //! \brief Perform restriction on \a set, using the overspill policy
    Void _checked_restriction(StorageType& set, const StorageType& bounding, ChainOverspillPolicy policy) const;
};

//! \brief Configuration for a ReachabilityAnalyser, essentially for controlling the
//!    accuracy of discretised evolution methods and reachability analysis.
template<class SYS> class ReachabilityAnalyserConfiguration : public ConfigurationInterface {
  public:
    //! \brief The unsigned integer type.
    typedef Nat UnsignedIntType;

    typedef ReachabilityAnalyser<SYS> ReachabilityAnalyserType;

    //! \brief The type representing time.
    typedef typename ReachabilityAnalyserType::TimeType TimeType;

    typedef typename ReachabilityAnalyserType::GridType GridType;
    typedef typename ReachabilityAnalyserType::BoundingDomainType BoundingDomainType;

    //! \brief Default constructor gives reasonable values.
    ReachabilityAnalyserConfiguration(ReachabilityAnalyser<SYS>& analyser);

    virtual ~ReachabilityAnalyserConfiguration() = default;

  private:

    //! \brief Reference back to the main class, for consistency checking.
    ReachabilityAnalyser<SYS>& _analyser;

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
    TimeType _transient_time;

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
    TimeType _lock_to_grid_time;

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
    SharedPointer<BoundingDomainType> _bounding_domain_ptr;

    //! \brief The grid used for approximation.
    //! \details Provided as a pointer in order to avoid double construction. It must always be defined.
    SharedPointer<GridType> _grid_ptr;

    //! \brief The policy for overspill in outer chain reach computation.
    //! \details A OVERSPILL_IGNORE simply bypasses checks, OVERSPILL_WARNING issues a warning and OVERSPILL_ERROR
    //! issues a OuterChainOverspill exception.
    ChainOverspillPolicy _outer_overspill_policy;

  public:

    const TimeType& transient_time() const { return _transient_time; }
    Void set_transient_time(const TimeType value) { _transient_time = TimeType(value); }

    const TimeType& lock_to_grid_time() const { return _lock_to_grid_time; }
    Void set_lock_to_grid_time(const TimeType value) { _lock_to_grid_time = TimeType(value); }

    const UnsignedIntType& maximum_grid_fineness() const { return _maximum_grid_fineness; }
    Void set_maximum_grid_fineness(const UnsignedIntType value) { _maximum_grid_fineness = value; }

    const UnsignedIntType& maximum_grid_extent() const { return _maximum_grid_extent; }
    Void set_maximum_grid_extent(const UnsignedIntType value) { _maximum_grid_extent = value; }

    const SharedPointer<BoundingDomainType>& bounding_domain_ptr() const { return _bounding_domain_ptr; }
    Void set_bounding_domain_ptr(SharedPointer<BoundingDomainType> bounding_domain_ptr) { _bounding_domain_ptr = bounding_domain_ptr; }

    const BoundingDomainType& bounding_domain() const { return *_bounding_domain_ptr; }
    Void set_bounding_domain(const BoundingDomainType& bounding_domain);

    const GridType& grid() const { return *_grid_ptr; }
    Void set_grid(const GridType& grid);

    const ChainOverspillPolicy& outer_overspill_policy() const { return _outer_overspill_policy; }
    Void set_outer_overspill_policy(const ChainOverspillPolicy value) { _outer_overspill_policy = value; }

  public:

    virtual OutputStream& _write(OutputStream& os) const;
};


} // namespace Ariadne


#endif // ARIADNE_REACHABILITY_ANALYSER_HPP
