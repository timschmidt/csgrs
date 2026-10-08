/***************************************************************************
 *            dynamics/reachability_analyser_interface.hpp
 *
 *  Copyright  2006-20  Pieter Collins
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

/*! \file dynamics/reachability_analyser_interface.hpp
 *  \brief Interface for performing reachability analysis.
 */

#ifndef ARIADNE_REACHABILITY_ANALYSER_INTERFACE_HPP
#define ARIADNE_REACHABILITY_ANALYSER_INTERFACE_HPP


#include "geometry/set_interface.hpp"

namespace Ariadne {

template<class SPC> struct SafetyCertificate;

//! \ingroup DynamicsModule
//! \brief Interface for computing (chain) reachable sets of a dynamic system.
//!
//! \sa \link Ariadne::EvolverInterface \c EvolverInterface<SYS,ES> \endlink
template<class SYS> class ReachabilityAnalyserInterface {
  public:
    //! \brief The type of the internal system.
    typedef SYS SystemType;
    //! \brief The type used to define the elapsed evolution time for the system type.
    typedef typename SystemType::EvolverType EvolverType;
    //! \brief The type used to define enclosures for local reach and evolve sets.
    typedef typename EvolverType::EnclosureType EnclosureType;
    //! \brief The type used to define global pavings of reach and evolve sets.
    typedef typename SystemType::StorageType StorageType;
    //! \brief The type used to define the elapsed evolution time for the system type.
    typedef typename SystemType::TimeType TimeType;
    //! \brief The type used to describe the state space of the system evolution.
    typedef typename SystemType::StateSpaceType StateSpaceType;
    //! \brief The type used to describe the interface for bounded sets in the state space.
    typedef typename StateSpaceType::BoundingDomainType BoundingDomainType;
    //! \brief The type used to describe the interface for overt sets in the state space, which are used as initial sets for lower reachability analysis.
    typedef typename StateSpaceType::OvertSetInterfaceType OvertSetInterfaceType;
    //! \brief The type used to describe the interface for open sets in the state space, which are used as safe sets for lower reachability analysis.
    typedef typename StateSpaceType::OpenSetInterfaceType OpenSetInterfaceType;
    //! \brief The type used to describe the interface for compact sets in the state space, which are used as initial sets for upper reachability analysis.
    typedef typename StateSpaceType::CompactSetInterfaceType CompactSetInterfaceType;
    //! \brief The type used to describe the interface for located sets in the state space, which are used as initial sets for verification.
    typedef typename StateSpaceType::LocatedSetInterfaceType LocatedSetInterfaceType;
    //! \brief The type used to describe the interface for regular sets in the state space, which are used as safe sets for verification.
    typedef typename StateSpaceType::RegularSetInterfaceType RegularSetInterfaceType;
    //! \brief The type used to describe the interface for bounded sets in the state space.
    typedef typename StateSpaceType::BoundedSetInterfaceType BoundedSetInterfaceType;
    //! \brief The type used to describe the interface for bounded regular sets in the state space.
    typedef typename StateSpaceType::RegularLocatedSetInterfaceType RegularLocatedSetInterfaceType;
    //! \brief The type used to describe the type used for upper approximations to sets.
    typedef typename StateSpaceType::SetApproximationType SetApproximationType;
    //! \brief The type used to represent a certificate of safety.
    typedef SafetyCertificate<StateSpaceType> SafetyCertificateType;
  public:
    //! \brief Virtual destructor.
    virtual ~ReachabilityAnalyserInterface() = default;

    //! \name Get the system associated with the analyser.
    virtual const SystemType& system() const = 0;

    //!@{
    //! \name Evaluation of maps on abstract sets

    //! \brief Compute an approximation to the set obtained by iterating \a steps times the system starting in \a initial_set.
    virtual StorageType
    lower_evolve(const OvertSetInterfaceType& initial_set,
                 const TimeType& steps) const = 0;

    //! \brief Compute a lower-approximation to the reachable and evolved sets of the system starting in \a initial_set up to \a time.
    virtual Pair<StorageType,StorageType>
    lower_reach_evolve(const OvertSetInterfaceType& initial_set,
                       const TimeType& time) const = 0;

    //! \brief Compute an approximation to the reachable set of the system starting in \a initial_set iterating at most \a steps times.
    virtual StorageType
    lower_reach(const OvertSetInterfaceType& initial_set,
                const TimeType& steps) const = 0;

    //! \brief Compute an infinite-time lower-approximation to the reachable set of the system starting in \a initial_set.
    //! of the system starting in \a initial_set.
    virtual StorageType
    lower_reach(const OvertSetInterfaceType& initial_set) const = 0;

    //! \brief Compute an approximation to the set obtained by iterating \a steps times the system starting in \a initial_set.
    virtual StorageType
    upper_evolve(const CompactSetInterfaceType& initial_set,
                 const TimeType& steps) const = 0;

    //! \brief Compute an upper-approximation to the reachable and evolved sets of the system starting in \a initial_set iterating at most \a time times.
    virtual Pair<StorageType,StorageType>
    upper_reach_evolve(const CompactSetInterfaceType& initial_set,
                       const TimeType& time) const = 0;

    //! \brief Compute an approximation to the reachable set
    //! of the system starting in \a initial_set iterating at most \a steps times.
    virtual StorageType
    upper_reach(const CompactSetInterfaceType& initial_set,
                const TimeType& steps) const = 0;

    //! \brief Compute an outer-approximation to the chain-reachable set of the system starting in \a initial_set.
    virtual StorageType
    outer_chain_reach(const CompactSetInterfaceType& initial_set) const = 0;

    //! \brief Test if the system is safe.
    virtual SafetyCertificateType
    verify_safety(const CompactSetInterfaceType& initial_set, const OpenSetInterfaceType& safe_set) const = 0;

    //!@}

};


//! \brief Factory for reachability analyser interface classes.
template<class SYS>
class ReachabilityAnalyserFactoryInterface
{
  public:

    //! \brief Create a copy of the factory.
    virtual ReachabilityAnalyserFactoryInterface* clone() const = 0;
    //! \brief Create a reachability analyser interface object around a \a system.
    virtual ReachabilityAnalyserInterface<SYS>* create(const SYS& system) const = 0;
};


} // namespace Ariadne


#endif // ARIADNE_ANALYSER_INTERFACE_HPP
