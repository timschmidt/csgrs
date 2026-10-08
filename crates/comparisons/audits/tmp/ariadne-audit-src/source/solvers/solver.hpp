/***************************************************************************
 *            solvers/solver.hpp
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

/*! \file solvers/solver.hpp
 *  \brief Solver classes for algebraic equations.
 */

#ifndef ARIADNE_SOLVER_HPP
#define ARIADNE_SOLVER_HPP

#include <exception>
#include <stdexcept>
#include <string>

#include "solvers/solver_interface.hpp"
#include "function/function_interface.hpp"
#include "function/function_patch_interface.hpp"

#include "conclog/logging.hpp"
#include "utility/attribute.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"
#include "numeric/numeric.hpp"

using namespace ConcLog;

namespace Ariadne {

template<class X> class Vector;


/*!
 *  \brief %Common functionality for solving (nonlinear) equations.
 */
class SolverBase
    : public SolverInterface
{
  public:
    /*! \brief Constructor. */
    SolverBase(ApproximateDouble max_error, Nat max_steps);

    /*! \brief The maximum permissible error of the solution. */
    ExactDouble maximum_error() const { return this->_max_error; }
    /*! \brief Set the maximum error. */
    Void set_maximum_error(ApproximateDouble max_error) { this->_max_error=cast_exact(max_error); };

    /*! \brief The maximum number of steps allowed before the method must quit. */
    Nat maximum_number_of_steps() const { return this->_max_steps; }
    /*! \brief Set the maximum number of steps. */
    Void set_maximum_number_of_steps(Nat max_steps) { this->_max_steps=max_steps; };

    /*! \brief The class which constructs functions for the implicit function solver. */
    const FunctionPatchFactoryInterface<ValidatedTag>& function_factory() const;
    /*! \brief Set the class which constructs functions for the implicit function solver. */
    Void set_function_factory(const FunctionPatchFactoryInterface<ValidatedTag>& factory);

    /*! \brief Solve \f$f(x)=0\f$, starting in the box \a bx. */
    virtual Vector<ValidatedNumericType> zero(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const;
    /*! \brief Solve \f$f(x)=0\f$, starting in the box \a bx. */
    virtual Vector<ValidatedNumericType> fixed_point(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const;

    /*! \brief Solve \f$f(x)=0\f$, starting in the box \a bx. */
    virtual Vector<ValidatedNumericType> solve(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const;
    virtual Vector<ValidatedNumericType> solve(const ValidatedVectorMultivariateFunction& f,const Vector<ValidatedNumericType>& ipt) const;
    /*! \brief Solve \f$f(a,x)=0\f$ for a in \a par, looking for a solution with x in \a ix. */
    virtual ValidatedVectorMultivariateFunctionPatch implicit(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& par, const ExactBoxType& ix) const;
    /*! \brief Solve \f$f(a,x)=0\f$ for a in \a par, looking for a solution with x in \a ix. */
    virtual ValidatedScalarMultivariateFunctionPatch implicit(const ValidatedScalarMultivariateFunction& f, const ExactBoxType& par, const ExactIntervalType& ix) const;
    //! \brief Solve \f$f(a,x)=0\f$ yielding a function \f$x=h(a)\f$ for a in \a A, looking for a solution with \f$h(A) \subset X\f$ and \f$h(a)\in x\f$.
    virtual ValidatedVectorMultivariateFunctionPatch continuation(const ValidatedVectorMultivariateFunction& f, const Vector<ApproximateNumericType>& a, const ExactBoxType& X, const ExactBoxType& A) const;


    /*! \brief Solve \f$f(x)=0\f$, starting in the interval point \a pt. */
    virtual Set< Vector<ValidatedNumericType> > solve_all(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const;
  protected:
    /*! \brief Perform one iterative step of the contractor. */
    virtual Vector<ValidatedNumericType> step(const ValidatedVectorMultivariateFunction& f,const Vector<ValidatedNumericType>& pt) const = 0;
    /*! \brief Perform one iterative step of the contractor. */
    virtual ValidatedVectorMultivariateFunctionPatch implicit_step(const ValidatedVectorMultivariateFunction& f,const ValidatedVectorMultivariateFunctionPatch& p,const ValidatedVectorMultivariateFunctionPatch& x) const = 0;
  private:
    ExactDouble _max_error;
    Nat _max_steps;
    std::shared_ptr< FunctionPatchFactoryInterface<ValidatedTag> > _function_factory_ptr;
};


/*! \ingroup AlgebraicEquationSubModule
 *  \brief ExactIntervalType Newton solver. Uses the contractor \f$[x']=x_0-Df^{-1}([x])f(x_0)\f$.
 */
class IntervalNewtonSolver
    : public SolverBase
{
  public:
    /*! \brief Constructor. */
    IntervalNewtonSolver(ApproximateDouble max_error, Nat max_steps) : SolverBase(max_error,max_steps) { }
    IntervalNewtonSolver(MaximumError max_error, MaximumNumericTypeOfSteps max_steps) : SolverBase(max_error,max_steps) { }
    /*! \brief Cloning operator. */
    virtual IntervalNewtonSolver* clone() const { return new IntervalNewtonSolver(*this); }
    /*! \brief Write to an output stream. */
    virtual Void _write(OutputStream& os) const;

    using SolverBase::implicit;
  public:
    virtual ValidatedVectorMultivariateFunctionPatch implicit_step(const ValidatedVectorMultivariateFunction& f, const ValidatedVectorMultivariateFunctionPatch& p, const ValidatedVectorMultivariateFunctionPatch& x) const;

    virtual Vector<ValidatedNumericType> step(const ValidatedVectorMultivariateFunction& f, const Vector<ValidatedNumericType>& pt) const;
};


/*! \ingroup AlgebraicEquationSubModule
 *  \brief Krawczyk solver. Uses the contractor \f$[x']=x_0-Mf(x_0)+(I-MDf([x])([x]-x_0)\f$
 *  where \f$M\f$ is typically \f$Df^{-1}(x_0)\f$.
 */
class KrawczykSolver
    : public SolverBase
{
  public:
    /*! \brief Constructor. */
    KrawczykSolver(ApproximateDouble max_error, Nat max_steps) : SolverBase(max_error,max_steps) { }
    KrawczykSolver(MaximumError max_error, MaximumNumericTypeOfSteps max_steps) : SolverBase(max_error,max_steps) { }
    /*! \brief Cloning operator. */
    virtual KrawczykSolver* clone() const { return new KrawczykSolver(*this); }
    /*! \brief Write to an output stream. */
    virtual Void _write(OutputStream& os) const;

    /*! \brief Solve \f$f(a,x)=0\f$ for a in \a par, looking for solutions with x in \a ix. */
    virtual ValidatedVectorMultivariateFunctionPatch implicit_step(const ValidatedVectorMultivariateFunction& f, const ValidatedVectorMultivariateFunctionPatch& p, const ValidatedVectorMultivariateFunctionPatch& x) const;
  public:
    /*! \brief A single step of the Krawczyk contractor. */
    virtual Vector<ValidatedNumericType>
    step(const ValidatedVectorMultivariateFunction& f,
          const Vector<ValidatedNumericType>& pt) const;
};


/*! \ingroup Solvers
 *  \brief Modified Krawczyk solver. Uses the contractor \f$[x']=x_0-J^{-1}(f(x_0)+(J-Df([x])([x]-x_0)\f$
 *  where \f$J\f$ is typically \f$Df(x_0)\f$.
 */
class FactoredKrawczykSolver
    : public KrawczykSolver
{
  public:
    /*! \brief Constructor. */
    FactoredKrawczykSolver(ApproximateDouble max_error, Nat max_steps) : KrawczykSolver(max_error,max_steps) { }
    FactoredKrawczykSolver(MaximumError max_error, MaximumNumericTypeOfSteps max_steps) : KrawczykSolver(max_error,max_steps) { }
    /*! \brief Cloning operator. */
    virtual FactoredKrawczykSolver* clone() const { return new FactoredKrawczykSolver(*this); }
    /*! \brief Write to an output stream. */
    virtual Void _write(OutputStream& os) const;
  public:
    /*! \brief A single step of the modified Krawczyk contractor. */
    virtual Vector<ValidatedNumericType>
    step(const ValidatedVectorMultivariateFunction& f,
          const Vector<ValidatedNumericType>& pt) const;
};

} // namespace Ariadne

#endif /* ARIADNE_SOLVER_HPP */
