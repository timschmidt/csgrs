/***************************************************************************
 *            geometry/affine_set.hpp
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

/*! \file geometry/affine_set.hpp
 *  \brief Affine sets described by equality, inequality and interval constraints over a is_bounded box.
 */

#ifndef ARIADNE_AFFINE_SET_HPP
#define ARIADNE_AFFINE_SET_HPP

#include <string>
#include <vector>
#include <list>
#include <iostream>

#include "utility/declarations.hpp"
#include "conclog/logging.hpp"
#include "utility/container.hpp"
#include "io/graphics_interface.hpp"
#include "function/affine.hpp"
#include "function/affine_model.hpp"
#include "function/constraint.hpp"

using namespace ConcLog;

namespace Ariadne {

template<class X> struct RigorousNumericsTraits;
template<class X, class XX=typename RigorousNumericsTraits<X>::Type> struct LinearProgram;

class Grid;
class PavingInterface;
class GridCell;
class DiscreteEvent;
class Figure;
class CanvasInterface;
struct Point2d;

template<class X, class B=X> using AffineConstraint = Constraint<Affine<X>,B>;
template<class F> using ValidatedAffineConstraint = Constraint<Affine<Bounds<F>>,Bounds<F>>;
template<class F> using ValidatedAffineModel = AffineModel<ValidatedTag,F>;
template<class F> using ValidatedAffineModelConstraint = Constraint<ValidatedAffineModel<F>,Bounds<F>>;

using EffectiveAffineConstraint = AffineConstraint<EffectiveNumber>;
using RealAffineConstraint = AffineConstraint<Real>;

using ValidatedAffineModelDP = ValidatedAffineModel<FloatDP>;
using ValidatedAffineConstraintDP = ValidatedAffineConstraint<FloatDP>;
using ValidatedAffineModelConstraintDP = ValidatedAffineModelConstraint<FloatDP>;


template<class X> AffineConstraint<X> operator<=(const SelfType<X>& l, const Affine<X>& a) {
    X u(+infty,l.precision()); return AffineConstraint<X>(l,a,u); }
template<class X> AffineConstraint<X> operator<=(const Affine<X>& a, const SelfType<X>& u) {
    X l(-infty,u.precision());return AffineConstraint<X>(l,a,u); }
template<class X> AffineConstraint<X> operator<=(const AffineConstraint<X>& ac, const SelfType<X>& u) {
    ARIADNE_ASSERT(decide(ac.upper_bound()==infty));
    return AffineConstraint<X>(ac.lower_bound(),ac.function(),u); }
template<class X> AffineConstraint<X> operator==(const Affine<X>& a, const SelfType<X>& c) {
    return AffineConstraint<X>(c,a,c); }

template<class F> ValidatedAffineModelConstraint<F> operator<=(const Bounds<F>& l, const ValidatedAffineModel<F>& am) {
    return ValidatedAffineModelConstraint<F>(l,am,+infty); }
template<class F> ValidatedAffineModelConstraint<F> operator<=(const ValidatedAffineModel<F>& am, const Bounds<F>& u) {
    return ValidatedAffineModelConstraint<F>(-infty,am,u); }
template<class F> ValidatedAffineModelConstraint<F> operator<=(const ValidatedAffineModelConstraint<F>& amc, const Bounds<F>& u) {
    ARIADNE_ASSERT(decide(amc.upper_bound()==infty));
    return ValidatedAffineModelConstraint<F>(amc.lower_bound(),amc.function(),u); }
template<class F> ValidatedAffineModelConstraint<F> operator==(const ValidatedAffineModel<F>& am, const Bounds<F>& c) {
    return ValidatedAffineModelConstraint<F>(c,am,c); }


//! \brief A constrained image set defined by affine functions.
//!  Defines a set of the form \f$S=\{ f(x) \mid x\in D \mid g(x)\leq 0 \wedge h(x)=0 \}\f$
//!  where \f$f\f$, \f$g\f$ and \f$h\f$ are vector-valued affine functions on \f$\R^n\f$ and \f$D\f$ is a box in \f$\R^n\f$.
//!
//! Includes the class of zonotopes \f$Z=\{ f(x) \mid x\in D\}\f$, polyhedra \f$\{ f(x) \mid x\in\R^n \mid g(x)\leq 0\}\f$ and polytopes \f$\{ f(x) \mid x\in[0,\infty)^n \mid \sum_{i=1}^{n} x_i -1 = 0\}\f$.
//! \sa ValidatedConstrainedImageSet
class ValidatedAffineConstrainedImageSet
    : public virtual ValidatedEuclideanCompactSetInterface
    , public virtual Drawable2dInterface
{
    ExactBoxType  _domain;
    Vector<ValidatedAffineModelDP> _space_models;
    List<ValidatedAffineModelConstraintDP> _constraint_models;
  public:
    //!\brief The set \f$\{ y \mid y\in D\}\f$.
    ValidatedAffineConstrainedImageSet(const ExactBoxType& D);
    //!\brief The set \f$\{ Gy+c \mid y\in D\}\f$.
    ValidatedAffineConstrainedImageSet(const ExactBoxType& D, const Matrix<FloatDP>& G, const Vector<FloatDP>& c);
    //!\brief The set \f$\{ x_i=f_i(s) \mid s\in D \}\f$.
    ValidatedAffineConstrainedImageSet(const ExactBoxType& D, const Vector<Affine<FloatDPBounds>>& f);
    //!\brief The set \f$\{ x_i=f_i(s) \mid s\in D \mid c(s) \}\f$.
    ValidatedAffineConstrainedImageSet(const ExactBoxType& D, const Vector<Affine<FloatDPBounds>>& f, const List<AffineConstraint<FloatDPBounds>>& c);
    //!\brief The set \f$\{ x_i=f_i(s) \mid s\in [-1,+1]^n \mid c(s) \}\f$.
    explicit ValidatedAffineConstrainedImageSet(const Vector<ValidatedAffineModelDP>& f, const List<ValidatedAffineModelConstraintDP>& c);
    explicit ValidatedAffineConstrainedImageSet(const Vector<ValidatedAffineModelDP>& f);

    ValidatedAffineConstrainedImageSet(const ExactBoxType& D, const Vector<ValidatedAffineModelDP>& f, const List<ValidatedAffineModelConstraintDP>& c);

    //!\brief The set \f$\{ x \mid x\in B\}\f$.
    ValidatedAffineConstrainedImageSet(const RealBox& B);
    //!\brief The set \f$\{ x \mid x\in B\}\f$.
    ValidatedAffineConstrainedImageSet(const Box<Interval<FloatDPBall>>& B);

    virtual ValidatedAffineConstrainedImageSet* clone() const override;
    Void new_parameter_constraint(const EffectiveAffineConstraint& c);
    Void new_parameter_constraint(const ValidatedAffineConstraintDP& c);
    Void new_constraint(const ValidatedAffineModelConstraintDP& c);

    virtual DimensionType dimension() const override;
    SizeType number_of_parameters() const;
    SizeType number_of_constraints() const;
    ExactBoxType domain() const;

    ValidatedKleenean is_bounded() const;
    virtual UpperBoxType bounding_box() const override;
    virtual ValidatedLowerKleenean separated(const ExactBoxType& bx) const override;
    virtual ValidatedLowerKleenean inside(const ExactBoxType& bx) const override;
    ValidatedLowerKleenean is_empty() const;

    //! \brief Compute the image of \f$S\f$ under the function \f$h\f$.
    friend ValidatedAffineConstrainedImageSet image(ValidatedAffineConstrainedImageSet set, ValidatedVectorMultivariateFunction const& h);

    Void adjoin_outer_approximation_to(PavingInterface& g, Nat fineness) const;
    GridTreePaving outer_approximation(const Grid& g, Nat fineness) const;
    Void robust_adjoin_outer_approximation_to(PavingInterface& paving, Nat fineness) const;

    List<Point2d> boundary(SizeType xc, SizeType yc) const;

    virtual Void draw(CanvasInterface&, const Projection2d& p) const override;
    virtual OutputStream& _write(OutputStream& os) const override;

  private:
    Void construct(const ExactBoxType& D, const Matrix<FloatDP>& G, const Vector<FloatDP>& c);
    Void construct_linear_program(LinearProgram<FloatDP,FloatDPBounds>& lp) const;
    static Void _robust_adjoin_outer_approximation_to(PavingInterface& paving, LinearProgram<FloatDP,FloatDPBounds>& lp, const Vector<FloatDPError>& errors, GridCell& cell, Nat fineness);
    static Void _adjoin_outer_approximation_to(PavingInterface& paving, LinearProgram<FloatDP,FloatDPBounds>& lp, const Vector<FloatDPError>& errors, GridCell& cell, Nat fineness);
};

inline OutputStream& operator<<(OutputStream& os, const ValidatedAffineConstrainedImageSet& as) {
    return as._write(os); }



} // namespace Ariadne

#endif // ARIADNE_AFFINE_SET_HPP
