/***************************************************************************
 *            dynamics/iterated_map_evolver.hpp
 *
 *  Copyright  2007-20  Alberto Casagrande, Pieter Collins
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

/*! \file dynamics/iterated_map_evolver.hpp
 *  \brief Evolver for iterated map systems.
 */

#ifndef ARIADNE_ITERATED_MAP_EVOLVER_HPP
#define ARIADNE_ITERATED_MAP_EVOLVER_HPP

#include <string>
#include <vector>
#include <list>
#include <iostream>

#include "utility/tuple.hpp"

#include "dynamics/iterated_map.hpp"
#include "function/function_interface.hpp"
#include "solvers/configuration_interface.hpp"
#include "dynamics/evolver_interface.hpp"

#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {

class Enclosure;
class IteratedMap;
template<class ES> class Orbit;

class IteratedMapEvolverConfiguration;

/*! \brief A class for computing the evolution of an iterated map.
 */
class IteratedMapEvolver
    : public EvolverInterface<IteratedMap,LabelledEnclosure,Integer>
{
  public:
    typedef EvolverInterface<IteratedMap,LabelledEnclosure,Integer> Interface;
    typedef IteratedMapEvolverConfiguration ConfigurationType;
    typedef IteratedMap SystemType;
    typedef Integer TimeType;
    typedef Integer TerminationType;
    typedef LabelledEnclosure EnclosureType;
    typedef Pair<TerminationType, EnclosureType> TimedEnclosureType;
    typedef Orbit<EnclosureType> OrbitType;
    typedef ListSet<EnclosureType> EnclosureListType;
    typedef ValidatedFunctionPatchFactory FunctionFactoryType;
  public:

    //! \brief Default constructor.
    IteratedMapEvolver(const SystemType& system);

    /*! \brief Make a dynamically-allocated copy. */
    IteratedMapEvolver* clone() const { return new IteratedMapEvolver(*this); }

    /* \brief Get the internal system. */
    virtual const SystemType& system() const { return *_system; }

    //! \brief Make an enclosure from a computed box set.
    EnclosureType enclosure(ExactBoxType const&) const;

    //!@{
    //! \name Configuration for the class.

    //! \brief A reference to the configuration.
    ConfigurationType& configuration() { return *this->_configuration; }
    const ConfigurationType& configuration() const { return *this->_configuration; }

    //! \brief The class which constructs functions for the enclosures.
    const FunctionFactoryType function_factory() const;

    //!@}

    //!@{
    //! \name Evolution using abstract sets.
    //! \brief Compute an approximation to the orbit set using upper semantics.
    Orbit<EnclosureType> orbit(const EnclosureType& initial_set, const TerminationType& termination, Semantics semantics) const;
    Orbit<EnclosureType> orbit(const RealExpressionBoundedConstraintSet& initial_set, const TerminationType& termination, Semantics semantics) const;

  protected:
    virtual Void _evolution(EnclosureListType& final, EnclosureListType& reachable, EnclosureListType& intermediate,
                            const EnclosureType& initial, const TerminationType& termination,
                            Semantics semantics) const;

    virtual Void _evolution_step(List< TimedEnclosureType >& working_sets,
                                 EnclosureListType& final, EnclosureListType& reachable, EnclosureListType& intermediate,
                                 const TimedEnclosureType& current_set, const TerminationType& termination,
                                 Semantics semantics) const;

  private:
    SharedPointer<SystemType> _system;
    SharedPointer<ConfigurationType> _configuration;
};


//! \brief Configuration for a IteratedMapEvolver, essentially to control accuracy of evolution.
class IteratedMapEvolverConfiguration : public ConfigurationInterface
{
  public:
    typedef double RealType;

    //! \brief Default constructor gives reasonable values.
    IteratedMapEvolverConfiguration();

    virtual ~IteratedMapEvolverConfiguration() = default;

  private:

    //! \brief The maximum allowable radius of a basic set.
    //! Decreasing this value increases the accuracy of the computation of an over-approximation.
    RealType _maximum_enclosure_radius;

    //! \brief Allow subdivisions in upper evolution.
    Bool _enable_subdivisions;

    //! \brief Allow premature termination of lower evolution.
    Bool _enable_premature_termination;

  public:

    const RealType& maximum_enclosure_radius() const { return _maximum_enclosure_radius; }
    Void set_maximum_enclosure_radius(const RealType value) { _maximum_enclosure_radius = value; }

    const Bool& enable_subdivisions() const { return _enable_subdivisions; }
    Void set_enable_subdivisions(const Bool value) { _enable_subdivisions = value; }

    const Bool& enable_premature_termination() const { return _enable_premature_termination; }
    Void set_enable_premature_termination(const Bool value) { _enable_premature_termination = value; }

  public:

    virtual OutputStream& _write(OutputStream& os) const;

};


} // namespace Ariadne

#endif // ARIADNE_ITERATED_MAP_EVOLVER_HPP
