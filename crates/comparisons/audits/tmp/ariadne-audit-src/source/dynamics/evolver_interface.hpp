/***************************************************************************
 *            dynamics/evolver_interface.hpp
 *
 *  Copyright  2008-20  Pieter Collins
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

/*! \file dynamics/evolver_interface.hpp
 *  \brief Interface for computing a single time step of the evolution of a system.
 */

#ifndef ARIADNE_EVOLVER_INTERFACE_HPP
#define ARIADNE_EVOLVER_INTERFACE_HPP


namespace Ariadne {


template<class ES> class ListSet;
template<class ES> class Orbit;

//! \ingroup DynamicsModule
//! \brief The semantics used to determine the trajectories of the system.
//! \see EvolverInterface
enum class Semantics : std::uint8_t {
    LOWER, //!< Under-approximation with trajectories terminating at spacial discontinuities.
    UPPER  //!< Over-approximations with all possibilities included as spacial discontinuities.
};


//! \ingroup DynamicsModule
//! \brief Interface for evolving a dynamic system.
//! \sa HybridEvolverInterface
//! \sa ReachabilityAnalyserInterface
template<class SYS, class ES, class TRM>
class EvolverInterface
{
  public:
    //! \brief The type of system the evolver can compute the behaviour of.
    typedef SYS SystemType;
    //! \brief The type used to represent the evolution time of the system.
    typedef typename SystemType::TimeType TimeType;
    //! \brief The type used to represent conditions under which system evolution may be terminated.
    typedef TRM TerminationType;
    //! \brief The type of set used to enclose the flow tubes.
    typedef ES EnclosureType;
    //! \brief The type of a list of enclosure sets.
    typedef ListSet<EnclosureType> EnclosureListType;

    //! \brief Virtual destructor.
    virtual ~EvolverInterface() = default;

    //! \brief Cloning operator.
    virtual EvolverInterface<SystemType,EnclosureType,TerminationType>* clone() const = 0;

    //! \brief Gets the system associated with the evolver.
    virtual const SystemType& system() const = 0;

  public:
    //!@{
    //! \name Main evolution functions.

    //! \brief Compute an approximation to the evolved set under the given semantics.
    virtual Orbit<EnclosureType> orbit(const EnclosureType& initial_set, const TerminationType& time, Semantics semantics) const = 0;

    //!@}

};

//! \brief Factory for evolver interface classes.
template<class SYS, class ES, class TRM>
class EvolverFactoryInterface
{
  public:

    //! \brief Create a copy of the factory.
    virtual EvolverFactoryInterface* clone() const = 0;
    //! \brief Create an evolver interface object around a \a system.
    virtual EvolverInterface<SYS,ES,TRM>* create(const SYS& system) const = 0;
};


} // namespace Ariadne



#endif // ARIADNE_EVOLVER_INTERFACE_HPP
