/***************************************************************************
 *            solvers/linear_programming.hpp
 *
 *  Copyright  2005-20  Alberto Casagrande, Pieter Collins
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

/*! \file solvers/linear_programming.hpp
 *  \brief Linear programming.
 */

#ifndef ARIADNE_LINEAR_PROGRAMMING_HPP
#define ARIADNE_LINEAR_PROGRAMMING_HPP

#include "conclog/logging.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "numeric/numeric.hpp"
#include "utility/tuple.hpp"

using namespace ConcLog;

namespace Ariadne {

template<class X> class Vector;
template<class X> class Matrix;
template<class X> class Affine;

template<class X> struct RigorousNumericsTraits { typedef X Type; };
template<> struct RigorousNumericsTraits<FloatDP> { typedef Bounds<FloatDP> Type; };
template<> struct RigorousNumericsTraits<FloatMP> { typedef Bounds<FloatMP> Type; };
template<class X> using RigorousNumericType = typename RigorousNumericsTraits<X>::Type;

template<class X> struct ApproximateNumericsTraits { typedef X Type; };
template<> struct ApproximateNumericsTraits<FloatDP> { typedef Approximation<FloatDP> Type; };
template<> struct ApproximateNumericsTraits<FloatMP> { typedef Approximation<FloatMP> Type; };
template<class X> using ApproximateNumericType = typename ApproximateNumericsTraits<X>::Type;


enum class LinearProgramStatus : std::uint8_t { INDETERMINATE_FEASIBILITY=0, PRIMAL_FEASIBLE=1, DUAL_FEASIBLE=2, PRIMAL_DUAL_FEASIBLE=3, DEGENERATE_FEASIBILITY=4};

class DegenerateFeasibilityProblemException : public std::runtime_error {
  public:
    DegenerateFeasibilityProblemException() : std::runtime_error("") { }
    DegenerateFeasibilityProblemException(const StringType& what) : std::runtime_error(what) { }
};

struct SingularLinearProgram : std::runtime_error {
    SingularLinearProgram(const StringType& what)
        : std::runtime_error(what) { };
};

struct UnboundedLinearProgram : std::runtime_error {
    UnboundedLinearProgram(const StringType& what)
        : std::runtime_error(what) { }
};

struct InfeasibleLinearProgram : std::runtime_error {
    InfeasibleLinearProgram(const StringType& what)
        : std::runtime_error(what) { }
};




//! \ingroup OptimisationSubModule
//! Solver for linear programming problems using interior point methods.
class InteriorPointSolver
{
    typedef FloatDP X;
    typedef RigorousNumericType<X> VX;
    typedef ApproximateNumericType<X> AX;
  public:
    //! \brief Find approximate optimal solution of \f$\min c^T x \text{ s.t. } Ax=b; x\geq0\f$.
    //! Returns the tuple (v,x,y) where v is the optimal value, x is the optimal point,
    //! and y the corresponding dual feasible point.
    Tuple< AX, Vector<AX>, Vector<AX> >
    minimise(const Vector<X>& c, const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b) const;

    //! \brief Find approximate optimal solution of \f$\min c^T x \text{ s.t. } Ax=b; x\geq0\f$.
    //! Returns the triple (v,x,y) where v is the optimal value, x is the optimal point, and y the corresponding dual feasible point.
    Tuple< AX, Vector<AX>, Vector<AX> >
    hotstarted_minimise(const Vector<X>& c, const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                        Vector<AX>& x, Vector<AX>& y, Vector<AX>& zl, Vector<AX>& zu) const;

    //! \brief Test feasibility of the problem \f$Ax=b; x_l\leq x\leq x_u\f$.
    //! Returns the pair (r,x) where r is the result, and x the (potential) feasible point.
    ValidatedKleenean
    feasible(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b) const;


    //! \brief Validate that \a x is primal feasible and \a y is dual feasible.
    //! Returns the interval of possible optimal values.
    ValidatedKleenean validate_feasibility(const Vector<X>& xl, const Vector<X>& xu,
                                           const Matrix<X>& A, const Vector<X>& b,
                                           const Vector<AX>& x, const Vector<AX>& y) const;
  public:
    //! \brief Perform a step of the optimization of \f$\min c^T x \text{ s.t. } Ax=b; x_l \leq x\leq x_u\f$.
    //! Returns true if a full Newton step (alpha=1) is taken. In this case, the problem is feasible (up to roundoff error).
    LinearProgramStatus
    _minimisation_step(const Vector<X>& c,
                       const Vector<X>& xl, const Vector<X>& xu,
                       const Matrix<X>& A, const Vector<X>& b,
                       Vector<AX>& x, Vector<AX>& y, Vector<AX>& zl, Vector<AX>& zu) const;

    //! \brief Perform a step of the feasibility problem \f$Ax=b,\ x_l \leq x \leq x_u\f$.
    LinearProgramStatus
    _feasibility_step(const Vector<X>& xl, const Vector<X>& xu,
                      const Matrix<X>& A, const Vector<X>& b,
                      Vector<AX>& x, Vector<AX>& y, Vector<AX>& zl, Vector<AX>& zu) const;


};




//! \ingroup OptimisationSubModule
//! \brief The type of variable; lower is_bounded, upper is_bounded, basic, or fixed (upper and lower bounded).
//! \see SimplexSolver
enum class Slackness : std::int8_t {
    LOWER=-1, //!< <p/>
    BASIS=0,  //!< <p/>
    UPPER=+1, //!< <p/>
    FIXED=+2  //!< <p/>
};

OutputStream& operator<<(OutputStream& os, Slackness t);

//! \ingroup OptimisationSubModule
//! Solver for linear programming problems using the simplex algorithm.
template<class X>
class SimplexSolver
{
    typedef RigorousNumericType<X> XX;
  public:

    //! \ingroup LinearProgrammingModule
    //! Solve the linear programming problem \f$\min cx \text{ s.t. } Ax=b;\ l\leq x\leq u\f$.
    //! Returns the optimal vector. Throws an error if the problem is infeasible.
    Vector<XX>
    minimise(const Vector<X>& c, const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b) const;

    //! \ingroup LinearProgrammingModule
    //! Solve the linear programming problem \f$\min cx \text{ s.t. } Ax=b;\ l\leq x\leq u\f$.
    //! Returns the optimal vector. Throws an error if the problem is infeasible.
    //! Uses starting variable types \a vt.
    Vector<XX>
    hotstarted_minimise(const Vector<X>& c, const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                        Array<Slackness>& vt) const;

    //! \ingroup LinearProgrammingModule
    //! Solve the linear programming problem \f$\min cx \text{ s.t. } Ax=b;\ l\leq x\leq u\f$.
    //! Returns the optimal vector. Throws an error if the problem is infeasible.
    //! Uses starting variable types vt, permutation p and inverse basic matrix B.
    Vector<XX>
    hotstarted_minimise(const Vector<X>& c, const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                        Array<Slackness>& vt, Array<SizeType>& p, Matrix<XX>& B) const;

    //! \ingroup LinearProgrammingModule
    //! Test if there exists a point \f$x\f$ with \f$0 \leq x\f$ and \f$Ax=b\f$.
    ValidatedKleenean
    primal_feasible(const Matrix<X>& A, const Vector<X>& b) const;

    //! \ingroup LinearProgrammingModule
    //! Test if there exists a point \f$y\f$ with \f$yA\leq c\f$.
    ValidatedKleenean
    dual_feasible(const Matrix<X>& A, const Vector<X>& c) const;

    //! \ingroup LinearProgrammingModule
    //! Test if there exists a point \f$x\f$ with \f$l \leq x \leq u\f$ and \f$Ax=b\f$.
    ValidatedKleenean
    feasible(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b) const;

    //! \ingroup LinearProgrammingModule
    //! Test if there exists a point \f$x\f$ with \f$l \leq x \leq u\f$ and \f$Ax=b\f$.
    //! The initial Array \a vt is used to define the initial basic point.
    ValidatedKleenean
    hotstarted_feasible(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                        Array<Slackness>& vt) const;

    //! \ingroup LinearProgrammingModule
    //! Test if there exists a point \f$x\f$ with \f$l \leq x \leq u\f$ and \f$Ax=b\f$.
    //! If the initial Array \a vt has size zero, then an initial basis is computed, otherwise the given variable types are assumed.
    //! The values of \a p and \a B corresponding to \a vt may be given.
    //! The values of \a x and \a y are output parameters, giving access to the final primal and dual variables.
    ValidatedKleenean
    hotstarted_feasible(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                        Array<Slackness>& vt, Array<SizeType>& p, Matrix<XX>& B, Vector<XX>& x, Vector<XX>& y) const;




    //! \ingroup LinearProgrammingModule
    //! Check whether the assignment of basis, lower and upper variables yields a certificate of feasibility or infeasibility.
    ValidatedKleenean
    verify_primal_feasibility(const Matrix<X>& A, const Vector<X>& b, const Array<Slackness>& vt) const;

    //! \ingroup LinearProgrammingModule
    //! Check whether the assignment of basis, lower and upper variables yields a certificate of feasibility or infeasibility.
    ValidatedKleenean
    verify_dual_feasibility(const Matrix<X>& A, const Vector<X>& c, const Array<Slackness>& vt) const;

    //! \ingroup LinearProgrammingModule
    //! Check whether the assignment of basis, lower and upper variables yields a certificate of feasibility or infeasibility.
    ValidatedKleenean
    verify_feasibility(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                       const Array<Slackness>& vt) const;

  public:

    //! \ingroup LinearProgrammingModule
    //! Compute a permutation \f$p\f$ such that \f$p_{0},\ldots,p_{m-1}\f$ are the basis variables of \f$A\f$, and the inverse matrix \f$A_B^{-1}\f$.
    Pair< Array<SizeType>, Matrix<XX> >
    compute_basis(const Matrix<X>& A) const;

    //! \ingroup LinearProgrammingModule
    //! Compute the point corresponding to the given Array of variable types.
    Vector<XX>
    compute_x(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b, const Array<Slackness>& vt) const;

    //! \ingroup LinearProgrammingModule
    //! Perform a single step of the standard linear programming problem, updating the variable type Array \a vt, the ordered variable Array \a p, the inverse basis matrix \a B and the variables \a x.
    Bool lpstep(const Vector<X>& c, const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                Array<Slackness>& vt, Array<SizeType>& p, Matrix<XX>& B, Vector<XX>& x) const;

    //! \ingroup LinearProgrammingModule
    //! Perform a step of the simplex algorithm, choosing basic variable \a s to exit the basis.
    //! The index s is with respect to the list of basic variables, and not the original variable list.
    //! The function to optimize is not needed for this procedure.
    //! The variable which left the basis is the new p[s]
    //! Returns \a r where x[p[r]] was the variable entering the basis.
    //!  If r=m, then no step performed
    SizeType lpstep(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                  Array<Slackness>& vt, Array<SizeType>& p, Matrix<XX>& B, Vector<XX>& x, SizeType s) const;

    //! \ingroup LinearProgrammingModule
    //! Perform a step of the simplex algorithm for a feasibility computation using validated (interval) arithmetic.
    ValidatedKleenean validated_feasible(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b) const;

    //! \ingroup LinearProgrammingModule
    //! Perform a step of the simplex algorithm for a feasibility computation using validated (interval) arithmetic.
    Bool validated_feasibility_step(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                                    Array<Slackness>& vt, Array<SizeType>& p) const;

  public:
    //! \brief Check that the feasibility problem data is consistent.
    //! \details For an m-by-n matrix \a A, checks the following:
    //!   - b has size m, B has size m-by-m, l,u,vt,p,x have size n
    //!   - vt[p[i]]==BASIC if, and only if, i<m
    //!   - B = inverse(A_B), where A_B is the mxm submatrix of A formed by columns for which vt[j]==BASIC
    //!   - x[j]=l[j] if j==LOWER and x[j]=u[j] if j==UPPER
    //!   - Ax=b
    Void consistency_check(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                           const Array<Slackness>& vt, const Array<SizeType>& p, const Matrix<XX>& B, const Vector<XX>& x) const;

    //! \brief Check that B is the inverse of the matrix with columns A[p[0],...,A[p[m-1]].
    Void consistency_check(const Matrix<X>& A, const Array<SizeType>& p, const Matrix<XX>& B) const;
    //! \brief Check that Ax=b
    Void consistency_check(const Matrix<X>& A, const Vector<X>& b, const Vector<XX>& x) const;
    //! \brief Check that the basic variable Array p is consistent with the variable type Array vt.
    //! Returns the number of basic variables.
    SizeType consistency_check(const Array<Slackness>& vt, const Array<SizeType>& p) const;
  private:
    ValidatedKleenean _feasible(const Vector<X>& xl, const Vector<X>& xu, const Matrix<X>& A, const Vector<X>& b,
                      Array<Slackness>& vt, Array<SizeType>& p, Matrix<XX>& B, Vector<XX>& x) const;


};

//!@{
//! \relates SimplexSolver \name Type synonyms
using RationalSimplexSolver = SimplexSolver<Rational>; //!< \relates SimplexSolver .
//!@}

} // namespace Ariadne

#endif
