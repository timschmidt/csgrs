/***************************************************************************
 *            dynamics/vector_field_evolver.hpp
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

/*! \file dynamics/vector_field_evolver.hpp
 *  \brief Evolver for vector_field systems.
 */

#ifndef ARIADNE_VECTOR_FIELD_EVOLVER_HPP
#define ARIADNE_VECTOR_FIELD_EVOLVER_HPP

#include <string>
#include <vector>
#include <list>
#include <iostream>


#include "utility/tuple.hpp"

#include "dynamics/vector_field.hpp"
#include "function/function_interface.hpp"
#include "solvers/configuration_interface.hpp"
#include "solvers/integrator_interface.hpp"
#include "dynamics/evolver_interface.hpp"

#include "betterthreads/workload.hpp"

#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {

class VectorField;
template<class ES> class Orbit;

using Mutex = std::mutex;
template<class T> using LockGuard = std::lock_guard<T>;
using BetterThreads::DynamicWorkload;

class VectorFieldEvolverConfiguration;

class RealExpressionBoundedConstraintSet;

//! \brief A class for computing the evolution of a vector_field system.
//!
//! The actual evolution steps are performed by the Integrator class.
class VectorFieldEvolver
    : public EvolverInterface<VectorField,LabelledEnclosure,typename VectorField::TimeType>
{
  public:
    typedef EvolverInterface<VectorField,LabelledEnclosure,typename VectorField::TimeType> Interface;
    typedef VectorFieldEvolverConfiguration ConfigurationType;
    typedef VectorField SystemType;
    typedef IntegratorInterface IntegratorType;
    typedef typename VectorField::TimeType TimeType;
    typedef Dyadic TimeStepType;
    typedef TimeType TerminationType;
    typedef LabelledEnclosure EnclosureType;
    typedef ListSet<EnclosureType> EnclosureListType;
    typedef Pair<TimeStepType,EnclosureType> TimedEnclosureType;
    typedef Orbit<EnclosureType> OrbitType;
    typedef ValidatedFunctionPatchFactory FunctionFactoryType;
  private:
    //! \brief Synchronised wrapping of orbit to allow concurrent adjoining
    struct SynchronisedOrbit : public OrbitType {
        SynchronisedOrbit(const EnclosureType& initial) : OrbitType(initial) { }
        Void adjoin_reach(const EnclosureType& set) override { LockGuard<Mutex> lock(_mux); OrbitType::adjoin_reach(set); }
        Void adjoin_intermediate(const EnclosureType& set) override { LockGuard<Mutex> lock(_mux); OrbitType::adjoin_intermediate(set); }
        Void adjoin_final(const EnclosureType& set) override { LockGuard<Mutex> lock(_mux); OrbitType::adjoin_final(set); }
        SizeType reach_size() { LockGuard<Mutex> lock(_mux); return OrbitType::reach().size(); }
      private:
        Mutex _mux;
    };
    typedef DynamicWorkload<TimedEnclosureType,TimeType const&,Semantics,SharedPointer<SynchronisedOrbit>> WorkloadType;
  public:

    //! \brief Construct from parameters and an integrator to compute the flow.
    VectorFieldEvolver(SystemType const&, IntegratorInterface const&);

    //! \brief Make a dynamically-allocated copy.
    VectorFieldEvolver* clone() const { return new VectorFieldEvolver(*this); }

    //! \brief Get the internal system.
    virtual const SystemType& system() const { return *_system; }

    //! \brief Make an enclosure from a computed box set.
    EnclosureType enclosure(ExactBoxType const&) const;

    //!@{
    //! \name Configuration for the class.
    //! \brief A reference to the configuration controlling the evolution.
    ConfigurationType& configuration() { return *this->_configuration; }
    const ConfigurationType& configuration() const { return *this->_configuration; }

    //! \brief The class which constructs functions for the enclosures.
    const FunctionFactoryType& function_factory() const;

    //!@}

    //!@{
    //! \name Evolution using abstract sets.
    //! \brief Compute an approximation to the orbit set using upper semantics.
    OrbitType orbit(EnclosureType const& initial_set, TimeType const& time, Semantics semantics) const;
    OrbitType orbit(RealVariablesBox const& initial_set, TimeType const& time, Semantics semantics) const;
    OrbitType orbit(RealExpressionBoundedConstraintSet const& initial_set, TimeType const& time, Semantics semantics) const;

    //!@}

  protected:

    Void _process_timed_enclosure(WorkloadType::Access& workload, TimedEnclosureType const& current_timed_set,
                                  TimeType const& maximum_time, Semantics semantics, SharedPointer<SynchronisedOrbit> result) const;

    Void _process_timed_enclosure_step(WorkloadType::Access& workload, TimedEnclosureType const& current_timed_set,
                                       TimeType const& maximum_time, Semantics semantics, SharedPointer<SynchronisedOrbit> result) const;

    Void _append_initial_set(WorkloadType& workload, TimeStepType const& initial_time, EnclosureType const& current_set) const;

  private:
    SharedPointer<SystemType> _system;
    SharedPointer<IntegratorType> _integrator;
    SharedPointer<ConfigurationType> _configuration;
};


//! \brief Configuration for a VectorFieldEvolver, essentially for controlling the accuracy of continuous evolution methods.
class VectorFieldEvolverConfiguration : public ConfigurationInterface
{
  public:
    typedef ExactDouble RealType;
    typedef ApproximateDouble ApproximateRealType;

    //! \brief Default constructor gives reasonable values.
    VectorFieldEvolverConfiguration();

    ~VectorFieldEvolverConfiguration() override = default;

  private:

    //! \brief The maximum allowable step size for integration.
    //! Decreasing this value increases the accuracy of the computation.
    RealType _maximum_step_size;

    //! \brief The maximum allowable radius of a basic set during integration.
    //! Decreasing this value increases the accuracy of the computation of an over-approximation.
    RealType _maximum_enclosure_radius;

    //! \brief The maximum allowable approximation error in the parameter-to-space mapping of an enclosure set.
    //! Decreasing this value increases the accuracy of the computation of an over-approximation.
    RealType _maximum_spacial_error;

    //! \brief Enable reconditioning of basic sets.
    Bool _enable_reconditioning;

    //! \brief Enable subdivisions of basic sets along evolution for upper semantics.
    //! \details Subdivisions are always allowed on the initial set.
    Bool _enable_subdivisions;

  public:

    const RealType& maximum_step_size() const { return _maximum_step_size; }
    Void set_maximum_step_size(const ApproximateRealType value) { _maximum_step_size = cast_exact(value); }

    const RealType& maximum_enclosure_radius() const { return _maximum_enclosure_radius; }
    Void set_maximum_enclosure_radius(const ApproximateRealType value) { _maximum_enclosure_radius = cast_exact(value); }

    const RealType& maximum_spacial_error() const { return _maximum_spacial_error; }
    Void set_maximum_spacial_error(const ApproximateRealType value) { _maximum_spacial_error = cast_exact(value); }

    const Bool& enable_reconditioning() const { return _enable_reconditioning; }
    Void set_enable_reconditioning(const Bool value) { _enable_reconditioning = value; }

    const Bool& enable_subdivisions() const { return _enable_subdivisions; }
    Void set_enable_subdivisions(const Bool value) { _enable_subdivisions = value; }

  public:

    OutputStream& _write(OutputStream& os) const override;
};

} // namespace Ariadne

#endif // ARIADNE_VECTOR_FIELD_EVOLVER_HPP
