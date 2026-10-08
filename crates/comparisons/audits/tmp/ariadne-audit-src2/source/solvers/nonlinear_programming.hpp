/***************************************************************************
 *            solvers/nonlinear_programming.hpp
 *
 *  Copyright  2009-20  Pieter Collins
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

/*! \file solvers/nonlinear_programming.hpp
 *  \brief Nonlinear programming.
 */

#ifndef ARIADNE_NONLINEAR_PROGRAMMING_HPP
#define ARIADNE_NONLINEAR_PROGRAMMING_HPP

#include "utility/declarations.hpp"

#include "conclog/logging.hpp"
#include "numeric/numeric.hpp"
#include "utility/tuple.hpp"

    using namespace ConcLog;

namespace Ariadne {

template<class X, class R> class Constraint;

class InfeasibleProblemException : public std::runtime_error {
  public: InfeasibleProblemException() : std::runtime_error("InfeasibleProblemException") { }
};
class IndeterminateFeasibilityException : public std::runtime_error {
  public: IndeterminateFeasibilityException() : std::runtime_error("IndeterminateFeasibilityException") { }
};
class DegenerateNonlinearFeasibilityProblemException : public std::runtime_error {
  public: DegenerateNonlinearFeasibilityProblemException() : std::runtime_error("DegenerateNonlinearFeasibilityProblemException") { }
};
class NearBoundaryOfFeasibleDomainException : public std::runtime_error {
  public: NearBoundaryOfFeasibleDomainException() : std::runtime_error("NearBoundaryOfFeasibleDomainException") { }
};

//! \ingroup OptimisationSubModule EvaluationModule
//! Interface for nonlinear programming solvers.
class OptimiserInterface {
    using FLT=FloatDP;
  public:
    typedef Bounds<FLT> ValidatedNumericType;
    typedef Vector<FLT> ExactVectorType;
    typedef Vector<Bounds<FLT>> ValidatedVectorType;
    typedef Vector<Approximation<FLT>> ApproximateVectorType;
  public:
    //! \brief Virtual destructor.
    virtual ~OptimiserInterface() = default;
    //! \brief Create a dynamically-allocated copy.
    virtual OptimiserInterface* clone() const = 0;

    //! \brief Solve the general nonlinear programming problem \f$\min f(x) \text{ such that } x\in D \text{ and } g(x)\in C\f$.
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const = 0;
    //! \brief Solve the standard nonlinear programming problem \f$\min f(x) \text{ such that } x\in D,\ g(x)\leq 0  \text{ and } h(x) = 0\f$.
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ValidatedVectorMultivariateFunction h) const = 0;

    //! \brief Tests is the general nonlinear feasibility problem \f$x\in D \text{ and } g(x)\in C\f$ is feasible.
    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const = 0;
    //! \brief Tests is the standard nonlinear feasibility problem \f$x\in D,\ g(x)\leq 0 \text{ and } h(x) = 0\f$ is feasible. Assumes \f$D\f$ is bounded with nonempty interior.
    //! \internal This is one of the simplest nonlinear programming problems, and is a good test case for new algorithms.
    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ValidatedVectorMultivariateFunction h) const = 0;

    //! \brief Tests if the point \a x is feasible, in that \f$x\in D\f$ and \f$g(x)\in N_\epsilon(C)\f$.
    virtual Bool almost_feasible_point(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                       FloatDPApproximationVector x, FloatDPApproximation eps) const = 0;
    //! \brief Tests if the point \a x is feasible.
    virtual Bool is_feasible_point(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                   FloatDPVector x) const = 0;
    //! \brief Tests if the point \a x is near feasible.
    virtual Bool validate_feasibility(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                      ExactVectorType x) const = 0;
    //! \brief Tests if the point \a x is near feasible, using approximate multipliers \a y to guide the search.
    virtual Bool validate_feasibility(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                      ExactVectorType x, ExactVectorType y) const = 0;
    //! \brief Tests if the feasibility problem is definitely unsolvable, using multipliers \a y and local centering point \a x.
    virtual Bool validate_infeasibility(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                      ExactVectorType x, ExactVectorType y) const = 0;
    //! \brief Tests if the box \a X definitely containss a feasible point.
    virtual ValidatedKleenean contains_feasible_point(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                            FloatDPBoundsVector X) const = 0;
    //! \brief Tests if the Lagrange multipliers \a y are a certificate of infeasiblity.
    virtual Bool is_infeasibility_certificate(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                              ExactVectorType y) const = 0;
};

//! \ingroup OptimisationSubModule
//! Common routines for nonlinear minimisation
class OptimiserBase
    : public OptimiserInterface
{
  protected:
    static const FloatDP zero;
    static const FloatDP one;
  public:
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const = 0;
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ValidatedVectorMultivariateFunction h) const;

    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const = 0;
    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ValidatedVectorMultivariateFunction h) const;

    virtual Bool almost_feasible_point(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                       ApproximateVectorType x, FloatDPApproximation error) const;
    virtual Bool is_feasible_point(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                   ExactVectorType x) const;
    virtual Bool validate_feasibility(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                      ExactVectorType x) const;
    virtual Bool validate_feasibility(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                      ExactVectorType x, ExactVectorType y) const;
    virtual Bool validate_infeasibility(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                        ExactVectorType x, ExactVectorType y) const;
    virtual ValidatedKleenean contains_feasible_point(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                            ValidatedVectorType X) const;
    virtual Bool is_infeasibility_certificate(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                              ExactVectorType lambda) const;
};

//! \ingroup OptimisationSubModule
//! \brief Solver for feasibility problems based on a penalty-function approach
//!
//! For the feasibility problem \f$x\in D,\ g(x)\in C,\ h(x)=0\f$ where \f$D\f$ is bounded and \f$D,C\f$ have nonempty interiors.
//! Introduce slack variable \f$w=g(x)\f$, and minimise \f[ \sum_{j} (g_j(x)-w_j)^2 + \sum_k h_k(x)^2 \f] with \f$x\in D\f$ and \f$w\in C\f$.
//! Since the minimiser is not unique, we need to add penalty terms \f$-\mu/2(\log(x_u-x)+log(x-x_l))\f$ and \f$-\nu/2(\log(w_u-w)+log(w-w_l))\f$.
//! It suffices to add a penalty in \f$x\f$.
class PenaltyFunctionOptimiser
    : public OptimiserBase
{
  public:
    using OptimiserBase::minimise;
    using OptimiserBase::feasible;
  public:
    virtual PenaltyFunctionOptimiser* clone() const;
    virtual ValidatedKleenean check_feasibility(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C, ExactVectorType x, ExactVectorType y) const;
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;
    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;
    virtual Void feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                                  FloatDPApproximationVector& x, FloatDPApproximationVector& w, FloatDPApproximation& mu) const;
    virtual Void feasibility_step(const ExactBoxType& D, const ValidatedVectorMultivariateFunction& g, const ExactBoxType& C,
                                  FloatDPBoundsVector& x, FloatDPBoundsVector& w) const;
    virtual Void feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                                  FloatDPApproximationVector& x, FloatDPApproximationVector& y, FloatDPApproximationVector& z) const;
};




//! \ingroup OptimisationSubModule
//! \brief Solver for linear programming problems using invalid interior point methods.
//! \details Introduces variables \f$w\f$ and attempts to find \f$x\in D\f$ and \f$w\in C\f$ such that \f$g(x)=w\f$.
//!   The dual variables \f$y\f$ are unconstrained Lagrange multipliers for \f$y\cdot(g(x)-w)=0\f$.
class NonlinearInfeasibleInteriorPointOptimiser
    : public OptimiserBase
{
  public:
    virtual NonlinearInfeasibleInteriorPointOptimiser* clone() const { return new NonlinearInfeasibleInteriorPointOptimiser(*this); }

    using OptimiserBase::minimise;
    using OptimiserBase::feasible;

    struct PrimalDualData;
    struct StepData;

    //! \brief Compute a \em local optimum of linear programming problem \f$\min f(x) \text{ such that } x\in D \text{ and } g(x)\in C\f$.
    //! \pre The domain \f$D\f$ is bounded and has nonempty interior, and the codomain \f$C\f$ is nonempty.
    //! \return A box \f$X\f$ which definitely contains a feasible point, and contains a local optimum.
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;
    //! \brief Tests is the nonlinear programming problem \f$x\in D \text{ and } g(x)\in C\f$ is feasible.
    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;

    //! \brief Test if the constraints \f$g(x)\in C\f$ are solvable for \f$x\in D\f$ using a nonlinear feasibility test,
    //! hotstarting the method with the overall primal and dual variables.
    Pair<ValidatedKleenean,FloatDPApproximationVector> feasible_hotstarted(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C, const PrimalDualData& wxy0) const;

    Void setup_feasibility(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                           StepData& stp) const;
    Void step(const ApproximateScalarMultivariateFunction& f, const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
              StepData& stp) const;
//    FloatDPApproximation compute_mu(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
//                     FloatDPApproximationVector& w, FloatDPApproximationVector& x, FloatDPApproximationVector& y) const;
};


//! \ingroup OptimisationSubModule
//! Solver for linear programming problems using interior point methods.
class NonlinearInteriorPointOptimiser
    : public OptimiserBase
{
  public:
    virtual NonlinearInteriorPointOptimiser* clone() const { return new NonlinearInteriorPointOptimiser(*this); }

    using OptimiserBase::minimise;
    using OptimiserBase::feasible;

    //! \brief Compute a \em local optimum of linear programming problem \f$\min f(x) \text{ such that } x\in D \text{ and } g(x)\in C.\f$.
    //! \pre The domain \f$D\f$ is bounded and has nonempty interior, and the codomain \f$C\f$ is nonempty.
    //! \return A box \f$X\f$ which definitely contains a feasible point, and contains a local optimum.
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;
    //! \brief Tests is the nonlinear programming problem \f$x\in D \text{ and } g(x)\in C\f$ is feasible.
    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;

    //! \brief Test if the constraints \f$g(x)\in C\f$ are solvable for \f$x\in D\f$ using a nonlinear feasibility test,
    //! hotstarting the method with the primal \f$x_0\f$ and dual variables \f$y_0\f$ overall constraint violation \f$t_0\f$.
    Pair<ValidatedKleenean,FloatDPApproximationVector> feasible_hotstarted(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                                  const FloatDPApproximationVector& x0, const FloatDPApproximationVector& y0, const FloatDPApproximation& t0) const;

    //! \brief Test if the constraints \f$g(x)\in C\f$ are solvable for \f$x\in D\f$ using a nonlinear feasibility test,
    //! hotstarting the method with the primal variables \f$x_0\f$ and dual variables \f$y_0\f$.
    Pair<ValidatedKleenean,FloatDPApproximationVector> feasible_hotstarted(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                                                  const FloatDPApproximationVector& x0, const FloatDPApproximationVector& y0) const;

    Void minimisation_step(const ApproximateScalarMultivariateFunction& f, const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C, const ApproximateVectorMultivariateFunction& h,
                           FloatDPApproximationVector& x, FloatDPApproximationVector& w, FloatDPApproximationVector& kappa, FloatDPApproximationVector& lambda, const FloatDPApproximation& mu) const;
    Void setup_feasibility(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                           FloatDPApproximationVector& x, FloatDPApproximationVector& lambda) const;
    Void setup_feasibility(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                           FloatDPApproximationVector& x, FloatDPApproximationVector& lambda, FloatDPApproximation& t) const;
    Void feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                          FloatDPApproximationVector& x, FloatDPApproximationVector& lambda) const;
    Void feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                          FloatDPApproximationVector& x, FloatDPApproximationVector& lambda, FloatDPApproximation& violation) const;
    Void initialise_lagrange_multipliers(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                                         const FloatDPApproximationVector& x, FloatDPApproximationVector& lambda) const;
    FloatDPApproximation compute_mu(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                     const FloatDPApproximationVector& x, const FloatDPApproximationVector& lambda) const;

  public: // Deprecated
    Void compute_tz(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                          FloatDPApproximationVector& x, FloatDPApproximation& t, FloatDPApproximationVector& z) const { }
    Void feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                          FloatDPApproximationVector& x, FloatDPApproximationVector& y, FloatDPApproximationVector& z, FloatDPApproximation& violation) const { };
    Void linearised_feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                                     FloatDPApproximation& slack, FloatDPApproximationVector& x, FloatDPApproximationVector& lambda) const { };
    Void linearised_feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                                     FloatDPApproximationVector& x, FloatDPApproximationVector& y, FloatDPApproximationVector& z, FloatDPApproximation& t) const { };
  private:
    FloatDPApproximation compute_mu(const ApproximateScalarMultivariateFunction& f, const ExactBoxType& D, const ValidatedVectorMultivariateFunction& g, const ExactBoxType& C,
                     const FloatDPApproximationVector& x, const FloatDPApproximationVector& y) const;
    Void compute_violation(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& g, const ExactBoxType& C,
                           FloatDPApproximationVector& x, FloatDPApproximation& t) const;
};




class IntervalOptimiser
    : public NonlinearInteriorPointOptimiser
{
    virtual IntervalOptimiser* clone() const { return new IntervalOptimiser(*this); }
    virtual ValidatedKleenean feasible_zero(ExactBoxType D, ValidatedVectorMultivariateFunction h) const;
    Void feasibility_step(const FloatDPVector& xl, const FloatDPVector& xu, const ValidatedVectorMultivariateFunction& h,
                          FloatDPBoundsVector& x, FloatDPBoundsVector& y, FloatDPBoundsVector& zl, FloatDPBoundsVector zu, FloatDPBounds& mu) const;
};


class ApproximateOptimiser
    : public NonlinearInteriorPointOptimiser
{
    virtual ApproximateOptimiser* clone() const { return new ApproximateOptimiser(*this); }
    virtual ValidatedKleenean feasible_zero(ExactBoxType D, ValidatedVectorMultivariateFunction h) const;
    Void feasibility_step(const ExactBoxType& D, const ApproximateVectorMultivariateFunction& h,
                          FloatDPApproximationVector& X, FloatDPApproximationVector& Lambda) const;
};


/*//! \ingroup OptimisationSubModule
//! Solver for linear programming problems using interior point methods.
//! WARNING: This class currently does not work; maybe there is a problem with the algorithms.
class KrawczykOptimiser
    : public OptimiserBase
{

  public:
    virtual KrawczykOptimiser* clone() const { return new KrawczykOptimiser(*this); }

    //! \brief Solve the linear programming problem \f$\max f(x) \text{ such that } x\in D \text{ and } g(x)\in C\f$.
    virtual Vector<ValidatedNumericType> minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;
    //! \brief Tests is the nonlinear programming problem \f$x\in D \text{ and } g(x)\in C\f$ is feasible.
    virtual ValidatedKleenean feasible(ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C) const;

  public:
    //! \brief Try to solve the nonlinear constraint problem by applying the Krawczyk contractor to the Kuhn-Tucker conditions,
    //! hotstarting the iteration with the primal and dual variables.
    ValidatedKleenean minimise(ValidatedScalarMultivariateFunction f, ExactBoxType D, ValidatedVectorMultivariateFunction g, ExactBoxType C,
                     const FloatDPBounds& t0, const FloatDPBoundsVector& x0, const FloatDPBoundsVector& y0, const FloatDPBoundsVector& z0) const;

    //! \brief A primal-dual feasibility step for the problem \f$g(y)\in C;\ y\in D\f$.
    Void minimisation_step(const ExactBoxType& D, const ValidatedVectorMultivariateFunction& g, const ExactBoxType& C,
                           FloatDPBoundsVector& x, FloatDPBoundsVector& y, FloatDPBoundsVector& z, FloatDPBounds& t) const;
    //! \brief A primal-dual feasibility step for the problem \f$g(y)\in C;\ y\in D\f$.
    Void feasibility_step(const ExactBoxType& D, const ValidatedVectorMultivariateFunction& g, const ExactBoxType& C,
                          FloatDPBoundsVector& x, FloatDPBoundsVector& y, FloatDPBoundsVector& z, FloatDPBounds& t) const;

    //! \brief A primal feasibility step for the problem \f$g(y)\in C;\ y\in D\f$. \deprecated
    Void feasibility_step(const ExactBoxType& D, const ValidatedVectorMultivariateFunction& g, const ExactBoxType& C,
                          FloatDPBoundsVector& y, FloatDPBounds& t) const;
    //! \brief A feasibility step for the problem \f$g(y)\leq 0\f$. \deprecated
    Void feasibility_step(const ValidatedVectorMultivariateFunction& g,
                          FloatDPBoundsVector& x, FloatDPBoundsVector& y, FloatDPBoundsVector& z, FloatDPBounds& t) const;
    //! \brief An optimization step for the problem \f$\max f(y) \text{ s.t. } g(y)\leq 0\f$. \deprecated
    Void minimisation_step(const ValidatedScalarMultivariateFunction& f, const ValidatedVectorMultivariateFunction& g,
                           FloatDPBoundsVector& x, FloatDPBoundsVector& y, FloatDPBoundsVector& z) const;
  protected:
    Void setup_feasibility(const ExactBoxType& D, const ValidatedVectorMultivariateFunction& g, const ExactBoxType& C,
                           FloatDPBoundsVector& x, FloatDPBoundsVector& y, FloatDPBoundsVector& z, FloatDPBounds& t) const;
    protected:
    Void compute_tz(const ExactBoxType& D, const ValidatedVectorMultivariateFunction& g, const ExactBoxType& C, const FloatDPBoundsVector& y, FloatDPBounds& t, FloatDPBoundsVector& z) const;
};

*/


} // namespace Ariadne

#endif
