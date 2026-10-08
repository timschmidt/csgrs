/***************************************************************************
 *            solvers/solver_interface.hpp
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

/*! \file solvers/solver_interface.hpp
 *  \brief Interface class for solving equations.
 */

#ifndef ARIADNE_SOLVER_INTERFACE_HPP
#define ARIADNE_SOLVER_INTERFACE_HPP

#include <exception>
#include <stdexcept>
#include <string>

#include "utility/declarations.hpp"
#include "conclog/logging.hpp"

using namespace ConcLog;

namespace Ariadne {


template<class X> class ContainerComparison<Vector<X>> {
  public:
    bool operator() (const Vector<X>& v1, const Vector<X>& v2) const {
        if(v1.size()!=v2.size()) { return v1.size()<v2.size(); }
        for(SizeType i=0; i!=v1.size(); ++i) {
            if(decide(v1[i]<v2[i])) { return true; }
            else if(decide(v1[i]>v2[i])) { return false; }
        }
        return false;
    }
};

template<class T, class CMP> class Set;
template<class T> class List;

class SolverInterface;

//! \relates SolverInterface \brief An exception occurring in the solution of a (parameterised) algebraic equation.
class SolverException : public std::runtime_error
{
  public:
    SolverException(const char* what) : std::runtime_error(what) { }
    SolverException(const StringType& what) : std::runtime_error(what) { }
};

//! \relates SolverInterface \brief It cannot be shown that there is no solution to a (parameterised) algebraic equation,
//! \brief but no solution can be found by the solver.
class UnknownSolutionException : public SolverException
{
  public:
    UnknownSolutionException(const char* what) : SolverException(what) { }
    UnknownSolutionException(const StringType& what) : SolverException(what) { }
};

//! \relates SolverInterface \brief There are no solutions to a (parameterised) algebraic equation in the domain.
class NoSolutionException : public SolverException
{
  public:
    NoSolutionException(const char* what) : SolverException(what) { }
    NoSolutionException(const StringType& what) : SolverException(what) { }
};

//! \relates SolverInterface \brief The solutions to a (parameterised) algebraic equation are degenerate due to the Jacobian being singular.
class SingularJacobianException : public SolverException
{
  public:
    SingularJacobianException(const char* what) : SolverException(what) { }
    SingularJacobianException(const StringType& what) : SolverException(what) { }
};


//! \ingroup AlgebraicEquationSubModule
//! \brief %Interface for solving (nonlinear) equations.
//!
//! \details Two main kinds of problem are considered.
//! The problem of finding roots of a function \f$ f \f$, which are solutions to the equation \f$f(x)=0\f$, is addressed by the solve() method,
//! The problem of finding a function \f$ h \f$ such that \f$ f(x,h(x))=0 \f$ is addressed by the implicit() method.
//!
//! Additionally two parameters are defined which are applicable to any implementation.
//! The maximum_error() provides an upper bound on the size of a box containing a solution point,
//! uniforma bounds for an implicit function.
//! The maximum_number_of_steps() provides a timeout mechanism.
class SolverInterface
{
    using FLT=FloatDP;
  public:
    typedef Bounds<FLT> ValidatedNumericType;
    typedef Approximation<FLT> ApproximateNumericType;
    typedef ValidatedVectorMultivariateFunctionPatch ValidatedVectorMultivariateFunctionModelType;
  public:
    //! \brief Virtual destructor.
    virtual ~SolverInterface() = default;
    //! \brief Make a dynamically-allocated copy.
    virtual SolverInterface* clone() const = 0;
    //! \brief Write to an output stream.
    virtual Void _write(OutputStream& os) const = 0;


    //! \brief The maximum permissible error of the solution.
    virtual ExactDouble maximum_error() const = 0;
    //! \brief Set the maximum error.
    virtual Void set_maximum_error(ApproximateDouble max_error) = 0;

    //! \brief The maximum number of steps allowed before the method must quit.
    virtual Nat maximum_number_of_steps() const = 0;
    //! \brief Set the maximum number of steps.
    virtual Void set_maximum_number_of_steps(Nat max_steps) = 0;

    //! \brief Solve \f$f(x)=0\f$, starting in the box \a bx.
    virtual Vector<ValidatedNumericType> zero(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const = 0;
    //! \brief Solve \f$f(x)=x\f$, starting in the box \a bx.
    virtual Vector<ValidatedNumericType> fixed_point(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const = 0;

    //! \brief Solve \f$f(x)=0\f$, starting in the box \a bx. Throws a SolverException if a unique solution is not found.
    //!  \param f A function \f$\R^n\to\R^n\f$ for which the root \f$p\f$ is to be found.
    //!  \param bx A coordinate-aligned box \f$B\f$ in \f$\R^n\f$ for which the root is to be found.
    //! \details
    //! A root \f$p\f$ of \f$f\f$ is <em>transverse</em> if the Jacobian derivative matrix \f$Df(p)\f$ is nonsingular.
    //! The solution method should converge to a transverse root \f$p\f$ if the box \f$B\f$ contains \f$p\f$ in its interior, and is sufficiently small.
    //!
    //! The choice of box in which to search is to be determined by the user.
    //! If the box is too big, then the solution typically fails.
    //! The semantics of the method requires that there be a uniqure root in the box,
    //! and many algorithms also require the Jacobian derivative matrix \f$Df(x)\f$ to be nonsingular over the entire box.
    //! The Jacobian derivative is nonsingular over a sufficiently small neighbourhood of a transverse root.
    //! However, choosing too small a box may also cause the algorithm to fail if the root is too close to the boundary.
    //!
    //! The solve_all() method is an algorithm which attempts to find all roots.
    virtual Vector<ValidatedNumericType> solve(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const = 0;
    virtual Vector<ValidatedNumericType> solve(const ValidatedVectorMultivariateFunction& f,const Vector<ValidatedNumericType>& ipt) const = 0;
    //! \brief Solve \f$f(a,x)=0\f$ for \f$a\f$ in \f$A\f$, looking for a solution with \f$x\f$ in \f$X\f$. The result is a function \f$h\f$ with domain \f$A\f$ such that \f$f(a,h(a))=0\f$ for all \f$a\in A\f$.
    //!  \param f A function \f$\R^{m+n}\to\R^n\f$.
    //!  \param A A subset of \f$\R^m\f$ giving the required domain of the solution function \f$h\f$.
    //!  \param X A subset of \f$\R^n\f$ giving a codomain of \f$h\f$ i.e. \f$h\f$ satisfies \f$h(a)\in X\f$ for all \f$a\in A\f$.
    //!    \warning If the domain \a A is too large, there may not be a continuous solution over it.
    //!      If the codomain \a X is too small, then there may not be a solution within it,
    //!      but if it is too large, there may not be a unique solution, and some solvers may have problems
    //!      with the first few steps of the solution.
    //! \details A <em>strong solution</em> to the problem exists if for every \f$a\in A\f$, there is a unique solution \f$x\in X\f$ to the equation \f$f(a,x)=0\f$, and this solution varies continuously in \f$a\f$. A <em>weak solution</em> is a function \f$h\f$ with domain \f$A\f$ such that \f$f(a,h(a))=0\f$ for all \f$a\in A\f$, if \f$f(a,x)=0\f$ for \f$x\in X\f$ then \f$h(a)=x\f$, and there exists \f$(a,x)\in A\times X\f$ such that \f$f(a,x)=0\f$.
    //! If \f$D_2f(a,x)\f$ is nonsingular for all \f$(a,x)\in A\times X\f$, then any solution is guaranteed to be unique.
    //! May throw a NoSolutionException, but \em only if there are no solutions to \f$f(a,x)=0\f$ in \f$A\times X\f$.
    //! If there is a continuous branch of solutions \f$x=h(a)\f$ such that \f$h(a)\in X\f$ for some parameter values,
    //! but \f$h(a)\not\in X\f$ for others, then \f$h\f$ is a valid result for the function.
    virtual ValidatedVectorMultivariateFunctionPatch implicit(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& A, const ExactBoxType& X) const = 0;
    //! \brief Solve \f$f(a,x)=0\f$ for a in \a A, looking for a solution with x in \a X.
    virtual ValidatedScalarMultivariateFunctionPatch implicit(const ValidatedScalarMultivariateFunction& f, const ExactBoxType& A, const ExactIntervalType& X) const = 0;

    //! \brief Solve \f$f(a,x)=0\f$ for x in \a X, and continue to a function \f$h\f$ solving \f$f(a,h(a))=0\f$ over \f$A\f$.
    virtual ValidatedVectorMultivariateFunctionPatch continuation(const ValidatedVectorMultivariateFunction& f, const Vector<ApproximateNumericType>& a, const ExactBoxType& X,  const ExactBoxType& A) const = 0;

    //! \brief Solve \f$f(x)=0\f$, starting in the box \a bx. Returns a set of bounds for which it can be <em>proved</em> that
    //! a solution exists in within the bounds. This means that some solutions may be omitted if they are not sufficiently robust.
    //!
    virtual Set< Vector<ValidatedNumericType> > solve_all(const ValidatedVectorMultivariateFunction& f,const ExactBoxType& bx) const = 0;
};

inline OutputStream& operator<<(OutputStream& os, const SolverInterface& solver) {
    solver._write(os); return os;
}

} // namespace Ariadne

#endif /* ARIADNE_SOLVER_INTERFACE_HPP */
