/***************************************************************************
 *            solvers/constraint_solver.hpp
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

/*! \file solvers/constraint_solver.hpp
 *  \brief Class for solving systems of constraints over a box.
 */

#ifndef ARIADNE_CONSTRAINT_SOLVER_HPP
#define ARIADNE_CONSTRAINT_SOLVER_HPP

#include "conclog/logging.hpp"
#include "utility/container.hpp"

#include "utility/declarations.hpp"
#include "utility/tribool.hpp"
#include "numeric/numeric.hpp"
#include "function/constraint.hpp"

using namespace ConcLog;

namespace Ariadne {

class GridTreePaving;

template<class X, class R> class Constraint;

template<class X> class Procedure;
typedef Procedure<ValidatedNumber> ValidatedProcedure;

template<class P,class F> class TaylorModel;
template<class M> class VectorScaledFunctionPatch;
typedef VectorScaledFunctionPatch<TaylorModel<ValidatedTag,FloatDP>> ValidatedVectorMultivariateTaylorFunctionModelDP;

template<class X> struct FeasibilityState {
    X t;
    Vector<X> x;
    Vector<X> y;
    Vector<X> z;
};


//! \ingroup OptimisationSubModule
//! \brief A class for finding solutions of systems of constraints of the form \f$g(y) \leq c\f$.
class ConstraintSolverInterface {
    using FLT=FloatDP;
  public:
    typedef Point<FLT> ExactPointType;
    typedef FLT ExactNumericType;
    typedef Bounds<FLT> ValidatedNumericType;
    typedef Approximation<FLT> ApproximateNumericType;
  public:
    //! \brief Test if the image of the box \a domain under the function \a function intersects \a codomain.
    virtual Pair<ValidatedKleenean,ExactPointType> feasible(const ExactBoxType& domain, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain) const = 0;
    //! \brief Test if \a point is in \a domain and the image of \a point under the function \a function lies in \a codomain.
    virtual ValidatedKleenean check_feasibility(const ExactBoxType& domain, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain, const ExactPointType& point) const = 0;
    //! \brief Try to reduce the size of the domain by propagating interval constraints. Returns \c true if the reduced domain is empty.
    virtual Bool reduce(UpperBoxType& domain, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain) const = 0;

};



//! \ingroup OptimisationSubModule
//! \brief A class for finding solutions of systems of constraints of the form \f$g(y) \leq c\f$.
class ConstraintSolver
    : public ConstraintSolverInterface
{
  public:
    //! \brief Test if the image of the box \a domain under the function \a function intersects \a codomain.
    virtual Pair<ValidatedKleenean,ExactPointType> feasible(const ExactBoxType& domain, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain) const;
    //! \brief Test if \a point is in \a domain and the image of \a point under the function \a function lies in \a codomain.
    virtual ValidatedKleenean check_feasibility(const ExactBoxType& domain, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain, const ExactPointType& point) const;
    //! \brief Try to reduce the size of the domain by propagating interval constraints.
    virtual Bool reduce(UpperBoxType& domain, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain) const;


    //! \brief Test if the constraints are solvable using a nonlinear feasibility test. Returns an approximate feasible point if the result is true. (Deprecated)
    virtual Pair<ValidatedKleenean,ExactPointType> feasible(const ExactBoxType& domain, const List<ValidatedConstraint>& constraints) const;
    //! \brief Try to reduce the size of the domain by propagating interval constraints. (Deprecated)
    virtual Bool reduce(UpperBoxType& domain, const List<ValidatedConstraint>& constraints) const;

    //! \brief Try to enforce hull consistency by propagating several interval constraints at once.
    //! This method is sharp if each variable occurs at most once in the constraint.
    Bool hull_reduce(UpperBoxType& bx, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain) const;
    Bool hull_reduce(UpperBoxType& bx, const Vector<ValidatedProcedure>& procedure, const ExactBoxType& codomain) const;
    //! \brief Try to enforce hull consistency by propagating an interval constraint.
    //! This method is sharp if each variable occurs at most once in the constraint.
    Bool hull_reduce(UpperBoxType& bx, const ValidatedScalarMultivariateFunction& function, const ExactIntervalType& codomain) const;
    Bool hull_reduce(UpperBoxType& bx, const ValidatedProcedure& procedure, const ExactIntervalType& codomain) const;

    //! \brief Reduce the \a domain by testing intersection of \a multipliers inner product \a function(\a domain)
    //! with \a multipliers innner product \a codomain, centering at \a centre.
    //! Reduces \f$(\lambda\cdot f)(X) \cap (\lambda\cdot C)\f$, evaluating \f$g(x)=g(x^*)+Dg(X) (X-x^*)\f$.
    Bool lyapunov_reduce(UpperBoxType& domain, const ValidatedVectorMultivariateTaylorFunctionModelDP& function, const ExactBoxType& codomain,
                         Vector<FloatDP> centre, Vector<FloatDP> multpliers) const;
    Bool lyapunov_reduce(UpperBoxType& domain, const ValidatedVectorMultivariateTaylorFunctionModelDP& function, const ExactBoxType& codomain,
                         Vector<ApproximateNumericType> centre, Vector<ApproximateNumericType> multpliers) const;
    //! \brief Try to enforce hull consistency by reducing a constraint with respect to one variable.
    Bool box_reduce(UpperBoxType& bx, const ValidatedScalarMultivariateFunction& function, const ExactIntervalType&, SizeType j) const;
    //! \brief Try to enforce hull consistency by reducing an a monotone dimension.
    //! This method is sharp if each variable occurs at most once in the constraint.
    Bool monotone_reduce(UpperBoxType& bx, const ValidatedScalarMultivariateFunction& function, const ExactIntervalType&, SizeType j) const;

    //! Split the domain into two pieces to help try to solve the constraints.
    Pair<UpperBoxType,UpperBoxType> split(const UpperBoxType& domain, const ValidatedVectorMultivariateFunction& function, const ExactBoxType& codomain) const;

    Bool hull_reduce(UpperBoxType& bx, const ValidatedConstraint& constraint) const {
        return this->hull_reduce(bx,constraint.function(),constraint.bounds()); }
    Bool box_reduce(UpperBoxType& bx, const ValidatedConstraint& constraint, SizeType j) const {
        return this->box_reduce(bx,constraint.function(),constraint.bounds(),j); }
    Bool monotone_reduce(UpperBoxType& bx, const ValidatedConstraint& constraint, SizeType j) const {
        return this->monotone_reduce(bx,constraint.function(),constraint.bounds(),j); }

    virtual ~ConstraintSolver() = default;
};


} //namespace Ariadne

#endif
