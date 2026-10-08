/***************************************************************************
 *            dynamics/differential_inclusion_evolver.hpp
 *
 *  Copyright  2008-20  Luca Geretti, Pieter Collins, Sanja Zivanovic
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

/*! \file differential_inclusion_evolver.hpp
 *  \brief Evolver for differential inclusion dynamics.
 */

#ifndef ARIADNE_DIFFERENTIAL_INCLUSION_EVOLVER_HPP
#define ARIADNE_DIFFERENTIAL_INCLUSION_EVOLVER_HPP

#include "utility/typedefs.hpp"
#include "utility/attribute.hpp"
#include "numeric/numeric.hpp"
#include "algebra/sweeper.hpp"
#include "algebra/algebra.hpp"
#include "function/domain.hpp"
#include "function/function_patch.hpp"
#include "function/function_model.hpp"
#include "function/formula.hpp"
#include "function/symbolic_function.hpp"
#include "symbolic/expression_set.hpp"
#include "conclog/logging.hpp"
#include "solvers/integrator_interface.hpp"
#include "solvers/inclusion_integrator.hpp"
#include "solvers/configuration_interface.hpp"
#include "differential_inclusion.hpp"

using namespace ConcLog;

namespace Ariadne {

class Real;

struct StepSize : public Attribute<StepSizeType> { };
struct NumberOfStepsBetweenSimplifications : public Attribute<Nat> { };
struct NumberOfVariablesToKeep : public Attribute<Nat> { };

static const Generator<StepSize> step_size = Generator<StepSize>();
static const Generator<NumberOfStepsBetweenSimplifications> number_of_steps_between_simplifications = Generator<NumberOfStepsBetweenSimplifications>();
static const Generator<NumberOfVariablesToKeep> number_of_variables_to_keep = Generator<NumberOfVariablesToKeep>();

using ThresholdSweeperDP = ThresholdSweeper<FloatDP>;
using GradedSweeperDP = GradedSweeper<FloatDP>;
using GradedThresholdSweeperDP = GradedThresholdSweeper<FloatDP>;
using SweeperDP = Sweeper<FloatDP>;

using TimeStepType = Dyadic;

FloatDP volume(Vector<ApproximateIntervalType> const& box);

class InclusionEvolverState;
template<class T> class Orbit;

class ReconditionerInterface {
  public:
    virtual Bool must_reduce_parameters(InclusionEvolverState const& state) const = 0;
    virtual Bool must_incorporate_errors(InclusionEvolverState const& state) const = 0;
    virtual Void reduce_parameters(ValidatedVectorMultivariateFunctionPatch& phi) const = 0;
    virtual ValidatedVectorMultivariateFunctionPatch incorporate_errors(ValidatedVectorMultivariateFunctionPatch const& Phi) const = 0;
    virtual Void update_from(InclusionEvolverState const& state) = 0;
  public:
    virtual ReconditionerInterface* clone() const = 0;
    virtual ~ReconditionerInterface() = default;
};

class LohnerReconditioner : public ReconditionerInterface {
    Nat _number_of_variables;
    Nat _number_of_inputs;
    Nat _number_of_steps_between_simplifications;
    Nat _number_of_parameters_to_keep;
    ExactDouble _ratio_of_parameters_to_keep;
public:
    LohnerReconditioner(Nat number_of_variables, Nat number_of_inputs, Nat number_of_steps_between_simplifications_, ApproximateDouble ratio_of_parameters_to_keep)
        : _number_of_variables(number_of_variables),
          _number_of_inputs(number_of_inputs),
          _number_of_steps_between_simplifications(number_of_steps_between_simplifications_),
          _number_of_parameters_to_keep(USHRT_MAX),
          _ratio_of_parameters_to_keep(cast_exact(ratio_of_parameters_to_keep)) { }
    virtual LohnerReconditioner* clone() const override { return new LohnerReconditioner(*this); }
    virtual ValidatedVectorMultivariateFunctionPatch incorporate_errors(ValidatedVectorMultivariateFunctionPatch const& f) const override;
    virtual Void reduce_parameters(ValidatedVectorMultivariateFunctionPatch& f) const override;
    virtual Bool must_reduce_parameters(InclusionEvolverState const& state) const override;
    virtual Bool must_incorporate_errors(InclusionEvolverState const& state) const override;
    virtual Void update_from(InclusionEvolverState const& state) override;
};

class Reconditioner : public Handle<ReconditionerInterface> {
public:
    using Handle<ReconditionerInterface>::Handle;

    Bool must_reduce_parameters(InclusionEvolverState const& state) const { return _ptr->must_reduce_parameters(state); }
    Bool must_incorporate_errors(InclusionEvolverState const& state) const { return _ptr->must_incorporate_errors(state); }

    Void reduce_parameters(ValidatedVectorMultivariateFunctionPatch& phi) const { _ptr->reduce_parameters(phi); }
    ValidatedVectorMultivariateFunctionPatch incorporate_errors(ValidatedVectorMultivariateFunctionPatch const& Phi) const { return _ptr->incorporate_errors(Phi); }

    Void update_from(InclusionEvolverState const& state) {_ptr->update_from(state); }

    virtual ~Reconditioner() = default;
};

class DifferentialInclusionEvolverConfiguration;

class DifferentialInclusionEvolver {
  public:
    typedef DifferentialInclusionEvolverConfiguration ConfigurationType;
    typedef DifferentialInclusion SystemType;
    typedef SystemType::EnclosureType EnclosureType;
    typedef Orbit<EnclosureType> OrbitType;
  protected:
    SystemType _system;
    SharedPointer<IntegratorInterface> _integrator;
    Reconditioner _reconditioner;
    SharedPointer<ConfigurationType> _configuration;
  public:
    DifferentialInclusionEvolver(SystemType const& system, IntegratorInterface const& integrator, Reconditioner const& reconditioner);

    //!@{
    //! \name Configuration for the class.
    //! \brief A reference to the configuration controlling the evolution.
    ConfigurationType& configuration() { return *this->_configuration; }
    const ConfigurationType& configuration() const { return *this->_configuration; }

  public:
    OrbitType orbit(RealVariablesBox const& initial, Real const& T);
  private:
    Void _recondition_and_update(ValidatedVectorMultivariateFunctionPatch& function, InclusionEvolverState& state);
};

//! \brief Configuration for an DifferentialInclusionEvolver, essentially for controlling the accuracy of continuous evolution methods.
class DifferentialInclusionEvolverConfiguration : public ConfigurationInterface
{
  public:
    typedef ExactDouble RealType;
    typedef ApproximateDouble ApproximateRealType;

    //! \brief Default constructor gives reasonable values.
    DifferentialInclusionEvolverConfiguration();

    virtual ~DifferentialInclusionEvolverConfiguration() = default;

  private:

    //! \brief The maximum allowable step size for integration.
    //! Decreasing this value increases the accuracy of the computation.
    RealType _maximum_step_size;

    //! \brief The maximum allowable radius of a basic set during integration.
    //! Decreasing this value increases the accuracy of the computation of an over-approximation.
    RealType _maximum_enclosure_radius;

    //! \brief Enable reduction of the parameters (true by default).
    Bool _enable_parameter_reduction;

    //! \brief The approximations allowed in calculating the flow for each step.
    //! At least one approximation is required.
    List<InputApproximation> _approximations;

  public:

    const RealType& maximum_step_size() const { return _maximum_step_size; }
    Void set_maximum_step_size(const ApproximateRealType value) { _maximum_step_size = cast_exact(value); }

    const RealType& maximum_enclosure_radius() const { return _maximum_enclosure_radius; }
    Void set_maximum_enclosure_radius(const ApproximateRealType value) { _maximum_enclosure_radius = cast_exact(value); }

    const Bool& enable_parameter_reduction() const { return _enable_parameter_reduction; }
    Void set_enable_parameter_reduction(const Bool value) { _enable_parameter_reduction = value; }

    List<InputApproximation> const& approximations() const { return _approximations; }
    Void set_approximations(List<InputApproximation> const& value) { assert(value.size() > 0); _approximations = value; }

  public:

    virtual OutputStream& _write(OutputStream& os) const;
};

} // namespace Ariadne;

#endif // ARIADNE_DIFFERENTIAL_INCLUSION_EVOLVER_HPP
