/***************************************************************************
 *            solvers/integrator_interface.hpp
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

/*! \file solvers/integrator_interface.hpp
 *  \brief Interface for solver classes for differential equations.
 */

#ifndef ARIADNE_INTEGRATOR_INTERFACE_HPP
#define ARIADNE_INTEGRATOR_INTERFACE_HPP

#include <string>
#include <iosfwd>
#include <memory>

#include "utility/declarations.hpp"

namespace Ariadne {

struct FlowBoundsException : public std::runtime_error {
    FlowBoundsException(const StringType& what) : std::runtime_error(what) { }
};

class FlowStepModelType;
class FlowModelType;

template<class T> class Suggestion;

//! \ingroup DifferentialEquationSubModule
//! \brief A solution to the differential equation could not be computed over the requested time interval.
struct IncompleteFlowException : public std::runtime_error {
    IncompleteFlowException(const StringType& what, FlowStepModelType const& model);
    FlowStepModelType const& computed_model() const { return *this->_computed_model; }
  private:
    std::unique_ptr<FlowStepModelType> _computed_model;
};

//! \ingroup DifferentialEquationSubModule
//! \brief A solution to a differential equation could not be computed within the requested tolerances.
struct FlowTimeStepException : public std::runtime_error {
    FlowTimeStepException(const StringType& what) : std::runtime_error(what) { }
};

//! \ingroup DifferentialEquationSubModule
//! \brief The type to use for the size of a step of a continuous (or hybrid) system.
typedef Dyadic StepSizeType;

//! \ingroup DifferentialEquationSubModule
//! \brief Interface for integrating differential equations of the form \f$\dt{x}=f(x)\f$.
class IntegratorInterface
{
  public:
    //! \brief Virtual destructor.
    virtual ~IntegratorInterface() = default;

    //! \brief Make a dynamically-allocated copy.
    virtual IntegratorInterface* clone() const = 0;

    //! \brief Write to an output stream.
    virtual Void _write(OutputStream& os) const = 0;

    //! \brief Compute a validated version \f$\hat{\phi}\f$ of the flow \f$\phi(x,t)\f$ satisfying \f$\dt{\phi}(x,t)=f(\phi(x,t))\f$ for \f$x\in D\f$, using a time-step appropriate for the system.
    //! <br>
    //! Returns: A validated version \f$\hat{\phi}\f$ of the flow over a short step represented as a single function over a box.
    //! <br>
    //! Arguments: \f$f\f$ is the \a vector_field, \f$D\f$ is the \a state_domain, and \f$h_\mathrm{sug}\f$ is the \a suggested_time_step.
    virtual FlowStepModelType
    flow_step(const ValidatedVectorMultivariateFunction& vector_field,
              const ExactBoxType& state_domain) const = 0;

    //! \brief Compute a validated version \f$\hat{\phi}\f$ of the flow \f$\phi(x,t)\f$ satisfying \f$\dt{\phi}(x,t)=f(\phi(x,t))\f$ for \f$x\in D\f$ and \f$t\in[0,h]\f$ where \f$h\f$ may differ (typically be smaller) than the suggested value \f$h_\mathrm{sug}\f$.
    //! Returns: A validated version \f$\hat{\phi}\f$ of the flow over a short step represented as a single function over a box.
    //! <br>
    //! Arguments: \f$f\f$ is the \a vector_field, \f$D\f$ is the \a state_domain, and \f$h_\mathrm{sug}\f$ is the \a suggested_time_step.
    virtual FlowStepModelType
    flow_step(const ValidatedVectorMultivariateFunction& vector_field,
              const ExactBoxType& state_domain,
              const Suggestion<StepSizeType>& suggested_time_step) const = 0;

    //! \brief Compute a validated version \f$\hat{\phi}\f$ of the flow \f$\phi(x,t)\f$ satisfying \f$\dt{\phi}(x,t)=f(\phi(x,t))\f$ for \f$x\in D\f$ and \f$t\in[0,h]\f$.
    //! Throws: \c IncompleteFlowException if the flow cannot be computed over the requested time interval,
    //! and a \c FlowTimeStepException if the flow cannot be computed over the requested time interval.
     //! <br>
    //! Returns: A validated version \f$\hat{\phi}\f$ of the flow up to the requested time, represented as a single function over a box.
    //! <br>
    //! Arguments: \f$f\f$ is the \a vector_field, \f$D\f$ is the \a state_domain, and \f$h\f$ is the \a time_step.
    virtual FlowStepModelType
    flow_step(const ValidatedVectorMultivariateFunction& vector_field,
              const ExactBoxType& state_domain,
              const StepSizeType& time_step) const = 0;

    //! \brief Solve \f$\dt{\phi}(x,t)=f(\phi(x,t))\f$ for \f$x\in D\f$ and \f$t\in[0,h]\f$, assuming that the flow remains in \f$B\f$.
    //! If the flow does not remain in \f$B\f$, then \f$\hat{\phi}(x,t)\f$ may not be a bound for \f$\phi(x,t)\f$ if \f$\exists \tau\in[0,t],\ \phi(x,\tau)\not\in B\f$.
    //! If \f$(h,B)\f$ have been computed using the flow_bounds() method, then the flow is guarenteed to be correct for all \f$x\in D\f$ and \f$t\in[0,h]\f$.
    //! <br>
    //! Returns: A validated version \f$\hat{\phi}\f$ of the flow over a short step represented as a single function over a box.
    //! <br>
    //! Arguments: \f$f\f$ is the \a vector_field, \f$D\f$ is the \a state_domain, \f$h\f$ is the \a time_step, and \f$B\f$ is the \a state_bounding_box.
    //! <br>
    //! Throws: A FlowTimeStepException if the flow cannot be computed sufficiently accurately for the given time step.
    virtual FlowStepModelType
    flow_step(const ValidatedVectorMultivariateFunction& vector_field,
              const ExactBoxType& state_domain,
              const StepSizeType& time_step,
              const UpperBoxType& state_bounding_box) const = 0;

    //! \brief Compute the flow of \f$\dt{x}=f(x,t,a)\f$ starting in \f$D\f$ over time interval \f$T\f$ over parameter domain \f$A\f$,
    //! assuming the flow remains in \f$B\f$.
    //! <br>
    //! Arguments: \f$f\f$ is the \a differential_equation, \f$D\f$ is the \a state_domain, \f$T\f$ is the \a time_domain,
    //! \f$A\f$ is the \a parameter_domain and \f$B\f$ is the \a maximum_time_step.
    virtual FlowStepModelType
    flow_step(const ValidatedVectorMultivariateFunction& differential_equation,
              const ExactBoxType& state_domain,
              const Interval<StepSizeType>& time_domain,
              const ExactBoxType& parameter_domain,
              const UpperBoxType& bounding_box) const = 0;

    //! \brief Write to an output stream.
    friend inline OutputStream& operator<<(OutputStream& os, const IntegratorInterface& integrator) {
        integrator._write(os); return os;
}
};



} // namespace Ariadne

#endif /* ARIADNE_INTEGRATOR_INTERFACE_HPP */
