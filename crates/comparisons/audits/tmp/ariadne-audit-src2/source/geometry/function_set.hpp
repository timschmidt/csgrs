/***************************************************************************
 *            geometry/function_set.hpp
 *
 *  Copyright  2008-20  Pieter Collins
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

/*! \file geometry/function_set.hpp
 *  \brief Images and preimages of boxes in Euclidean space.
 */

#ifndef ARIADNE_FUNCTION_SET_HPP
#define ARIADNE_FUNCTION_SET_HPP

#include <iosfwd>

#include <memory>

#include "utility/macros.hpp"
#include "utility/container.hpp"
#include "utility/container.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "symbolic/templates.hpp"
#include "geometry/set_interface.hpp"
#include "function/function.hpp"
#include "function/function_model.hpp"
#include "function/constraint.hpp"
#include "io/graphics_interface.hpp"

#include "geometry/box.hpp"

namespace Ariadne {


class ImageSet;
class ConstraintSet;
class BoundedConstraintSet;
class ConstrainedImageSet;

typedef ImageSet EffectiveImageSet;
typedef ConstraintSet EffectiveConstraintSet;
typedef BoundedConstraintSet EffectiveBoundedConstraintSet;
typedef ConstrainedImageSet EffectiveConstrainedImageSet;

class ValidatedConstrainedImageSet;
class ValidatedAffineConstrainedImageSet;


class Grid;
class PavingInterface;

class Drawer;


//! \ingroup GeometryModule ExactSetSubModule
//! \brief A set defined as the image of a box \f$D\f$ under a function \f$f\f$.
//! \see ConstrainedImageSet
class ImageSet { };

//! \ingroup GeometryModule ExactSetSubModule
//! \brief A set defined as the intersection of an exact box with preimage of an exact box (the \em codomain) under a continuous function.
//! The set is described as \f$S=D\cap g^{-1}(C) = \{ x\in D \mid g(x)\in C\}\f$ where \f$D\f$ is the domain, \f$C\f$ is the codomain and \f$g\f$ the function.
class ConstraintSet
    : public virtual EffectiveEuclideanRegularSetInterface
{
    SizeType _dimension;
    List< EffectiveConstraint > _constraints;
  public:
    //! \brief Construct the preimage of \a C under \a g.
    ConstraintSet(const EffectiveVectorMultivariateFunction& g, const RealBox& C);
    //! \brief Construct the restriction of \a D under the constraints \a c.
    ConstraintSet(const List<EffectiveConstraint>& c);
    //! \brief The codomain of the set.
    const RealBox codomain() const { return this->constraint_bounds(); }
    //! \brief The function used to define the constraints.
    const EffectiveVectorMultivariateFunction constraint_function() const;
    //! \brief The bounds of the constraints.
    const RealBox constraint_bounds() const;
    //! \brief The number of constraints.
    SizeType number_of_constraints() const { return this->_constraints.size(); };
    //! \brief The constraints.
    List<EffectiveConstraint> const& constraints() const { return this->_constraints; }
    //! \brief The \a i<sup>th</sup> constraint.
    EffectiveConstraint const& constraint(SizeType i) const { return this->_constraints[i]; }

    virtual ConstraintSet* clone() const override;
    virtual DimensionType dimension() const override;
    virtual LowerKleenean separated(const ExactBoxType&) const override;
    virtual LowerKleenean overlaps(const ExactBoxType&) const override;
    virtual LowerKleenean covers(const ExactBoxType&) const override;

    ValidatedLowerKleenean separated(const ExactBoxType&, Effort) const;
    ValidatedLowerKleenean overlaps(const ExactBoxType&, Effort) const;
    ValidatedLowerKleenean covers(const ExactBoxType&, Effort) const;

    friend ConstraintSet intersection(const ConstraintSet& cs1, const ConstraintSet& cs2);
    friend BoundedConstraintSet intersection(const ConstraintSet& cs, const RealBox& bx);
    friend BoundedConstraintSet intersection(const RealBox& bx, const ConstraintSet& cs);

    friend OutputStream& operator<<(OutputStream&, const ConstraintSet&);
  private:
    virtual OutputStream& _write(OutputStream&) const override;
};


//! \ingroup GeometryModule ExactSetSubModule
//! \brief A set defined as the intersection of an exact box with preimage of an exact box (the \em codomain) under a continuous function.
//! The set is described as \f$S=D\cap g^{-1}(C) = \{ x\in D \mid g(x)\in C\}\f$ where \f$D\f$ is the domain, \f$C\f$ is the codomain and \f$g\f$ the function.
class BoundedConstraintSet
    : public virtual EffectiveEuclideanRegularLocatedSetInterface
    , public virtual Drawable2dInterface
{
    RealBox _domain;
    List< EffectiveConstraint > _constraints;
  public:
    //! \brief Construct the preimage of \a C under \a g.
    BoundedConstraintSet(const RealBox& D, const EffectiveVectorMultivariateFunction& g, const RealBox& C);
    //! \brief Construct the restriction of \a D under the constraints \a c.
    BoundedConstraintSet(const RealBox& D, const List<EffectiveConstraint>& c);
    //! \brief Construct the box \a D.
    BoundedConstraintSet(const RealBox& bx);
    //! \brief The domain of the set.
    const RealBox& domain() const { return this->_domain; }
    //! \brief The codomain of the set.
    const RealBox codomain() const { return this->constraint_bounds(); }
    //! \brief The function used to define the constraints.
    const EffectiveVectorMultivariateFunction constraint_function() const;
    //! \brief The bounds for the constraints.
    const RealBox constraint_bounds() const;
    //! \brief The number of constraints.
    SizeType number_of_constraints() const { return this->_constraints.size(); };
    //! \brief The constraints.
    List<EffectiveConstraint> const& constraints() const { return this->_constraints; }
    //! \brief The \a i<sup>th</sup> constraint.
    EffectiveConstraint const& constraint(SizeType i) const { return this->_constraints[i]; }

    virtual BoundedConstraintSet* clone() const override;
    virtual DimensionType dimension() const override;
    virtual LowerKleenean separated(const ExactBoxType&) const override;
    virtual LowerKleenean overlaps(const ExactBoxType&) const override;
    virtual LowerKleenean covers(const ExactBoxType&) const override;
    virtual LowerKleenean inside(const ExactBoxType&) const override;
    virtual UpperBoxType bounding_box() const override;
    virtual Void draw(CanvasInterface&,const Projection2d&) const override;

    ValidatedLowerKleenean separated(const ExactBoxType&, Effort) const;
    ValidatedLowerKleenean overlaps(const ExactBoxType&, Effort) const;
    ValidatedLowerKleenean covers(const ExactBoxType&, Effort) const;
    ValidatedLowerKleenean inside(const ExactBoxType&, Effort) const;

    friend BoundedConstraintSet intersection(const BoundedConstraintSet& bcs1, const BoundedConstraintSet& bcs2);
    friend BoundedConstraintSet intersection(const BoundedConstraintSet& bcs1, const ConstraintSet& cs2);
    friend BoundedConstraintSet intersection(const ConstraintSet& cs1, const BoundedConstraintSet& bcs2);
    friend BoundedConstraintSet intersection(const BoundedConstraintSet& bcs1, const RealBox& bx2);
    friend BoundedConstraintSet intersection(const RealBox& bx1, const BoundedConstraintSet& bcs2);
    friend ConstrainedImageSet image(const BoundedConstraintSet& set, const EffectiveVectorMultivariateFunction& function);

    friend OutputStream& operator<<(OutputStream& os, const BoundedConstraintSet& set);
  private:
    virtual OutputStream& _write(OutputStream&) const override;
};


//! \ingroup GeometryModule ExactSetSubModule
//! \brief A set defined as the image of the intersection of a box \f$D\f$ and a constraint set \f$g^{-1}(C)\f$ under a function \f$f\f$.
//! In other words, \f$S=f(D\cap g^{-1}(C))\f$.
//! \see ValidatedConstrainedImageSet
class ConstrainedImageSet
    : public virtual EffectiveEuclideanLocatedSetInterface
    , public virtual Drawable2dInterface
{
    RealBox _domain;
    EffectiveVectorMultivariateFunction _function;
    List< EffectiveConstraint > _constraints;
  public:
    //! \brief Construct the set with zero-dimensional parameterisation in zero dimensions with no constraints.
    ConstrainedImageSet() : _domain(), _function() { }
    //! \brief Construct the box \a dom.
    ConstrainedImageSet(const RealBox& dom) : _domain(dom), _function(EffectiveVectorMultivariateFunction::identity(dom.size())) { }
    //! \brief Construct the image of \a dom under \a fn.
    ConstrainedImageSet(const RealBox& dom, const EffectiveVectorMultivariateFunction& fn) : _domain(dom), _function(fn) {
        ARIADNE_ASSERT_MSG(dom.size()==fn.argument_size(),"dom="<<dom<<", fn="<<fn); }
    //! \brief Construct the image of \a dom under \a fn, using constraints \a c.
    ConstrainedImageSet(const RealBox& dom, const EffectiveVectorMultivariateFunction& fn, const List<EffectiveConstraint>& c) : _domain(dom), _function(fn), _constraints(c) {
        ARIADNE_ASSERT_MSG(dom.size()==fn.argument_size(),"dom="<<dom<<", fn="<<fn); }
    //! \brief Convert from a bounded constraint set.
    ConstrainedImageSet(const BoundedConstraintSet& set);
    //! \brief The domain of the set.
    const RealBox& domain() const { return this->_domain; }
    //! \brief The function used to define the mapping from the parameter domain to the space.
    const EffectiveVectorMultivariateFunction& function() const { return this->_function; };
    //! \brief The bounds for the constraints.
    const EffectiveVectorMultivariateFunction constraint_function() const;
    //! \brief The bounds for the constraints.
    const RealBox constraint_bounds() const;
    //! \brief The function used to define the set.
    const List<EffectiveConstraint>& constraints() const { return this->_constraints; };
    //! \brief The number of parameters used to define the set, which equals the dimension of \f$D\f$.
    SizeType number_of_parameters() const { return this->_domain.size(); };
    //! \brief The number of constraints.
    SizeType number_of_constraints() const { return this->_constraints.size(); };
    //! \brief The \a i<sup>th</sup> constraint.
    EffectiveConstraint const& constraint(SizeType i) const { return this->_constraints[i]; }

    //! \brief Apply the function \f$h\f$ to obtain the set \f$h\circ f(D\cap g^{-1}(C))\f$.
    Void apply(const EffectiveVectorMultivariateFunction& h) {
        this->_function=compose(h,this->_function);
    }

    //! \brief Introduce a new constraint of the form \f$g(y)\in [c_l,c_u]\f$.
    Void new_parameter_constraint(const EffectiveConstraint& c) {
        ARIADNE_ASSERT_MSG(c.function().argument_size()==this->domain().size(),*this<<", "<<c);
        this->_constraints.append(c); }

    //! \brief Introduce a new constraint of the form \f$g(y)\in [c_l,c_u]\f$.
    Void new_space_constraint(const EffectiveConstraint& c) {
        ARIADNE_ASSERT_MSG(c.function().argument_size()==this->_function.result_size(),*this<<", "<<c);
        this->_constraints.append(EffectiveConstraint(c.lower_bound(),compose(c.function(),_function),c.upper_bound())); }

    virtual ConstrainedImageSet* clone() const override { return new ConstrainedImageSet(*this); }
    virtual DimensionType dimension() const override { return this->_function.result_size(); }

    //! \brief A coarse over-approximation to the set. Computed by taking the interval evaluation \f$h(D)\f$.
    virtual UpperBoxType bounding_box() const override;
    //! \brief Construct an affine over-approximation
    ValidatedAffineConstrainedImageSet affine_over_approximation() const;
    //! \brief Construct an affine approximation, with undefined accuracy.
    ValidatedAffineConstrainedImageSet affine_approximation() const;
    //! \brief Split into two pieces by subdividing along a coordinate direction.
    Pair<ConstrainedImageSet,ConstrainedImageSet> split() const;
    //! \brief Split into two pieces by subdividing along the \a j<sup>th</sup> coordinate direction.
    Pair<ConstrainedImageSet,ConstrainedImageSet> split(SizeType j) const;

    //! \brief Test if the set is contained in (the interior of) a box.
    virtual LowerKleenean inside(const ExactBoxType& bx) const override;
    //! \brief Test if the set is disjoint from a (closed) box.
    virtual LowerKleenean separated(const ExactBoxType&) const override;
    //! \brief Test if the set overlaps (intersects the interior of) a box.
    virtual LowerKleenean overlaps(const ExactBoxType&) const override;
    //! \brief Adjoin an outer approximation to a paving.
    Void adjoin_outer_approximation_to(PavingInterface& paving, Nat fineness) const;

    ValidatedLowerKleenean inside(const ExactBoxType&, Effort) const;
    ValidatedLowerKleenean separated(const ExactBoxType&, Effort) const;
    ValidatedLowerKleenean overlaps(const ExactBoxType&, Effort) const;

    //! \brief Test if the set satisfies the state constraint at all points.
    Kleenean satisfies(const EffectiveConstraint& c) const;
    ValidatedKleenean satisfies(const EffectiveConstraint& c, Effort) const;

    //! \brief Draw to a canvas.
    virtual Void draw(CanvasInterface&,const Projection2d&) const override;

    //! \brief Compute the image of \f$S\f$ under the function \f$h\f$.
    friend ConstrainedImageSet image(ConstrainedImageSet set, EffectiveVectorMultivariateFunction const& h);

    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream&, const ConstrainedImageSet&);
  private:
    virtual OutputStream& _write(OutputStream&) const override;
};



//! \ingroup GeometryModule ExactSetSubModule
//! \brief A set defined as the image of the intersection of a box \f$D\f$ and a constraint set \f$g^{-1}(C)\f$ under a function \f$f\f$.
//! In other words, \f$S=f(D\cap g^{-1}(C))\f$.
class ValidatedConstrainedImageSet
    : public virtual ValidatedEuclideanLocatedSetInterface
    , public virtual Drawable2dInterface
{
    ExactBoxType _domain;
    ExactBoxType _reduced_domain;
    ValidatedVectorMultivariateFunction _function;
    List< ValidatedConstraint > _constraints;
  public:
    //! \brief Construct the set with zero-dimensional parameterisation in zero dimensions with no constraints.
    ValidatedConstrainedImageSet() : _domain(), _function() { }
    //! \brief Construct the box \a dom.
    ValidatedConstrainedImageSet(const ExactBoxType& dom)
        : _domain(dom), _reduced_domain(dom)
        , _function(static_cast<ValidatedVectorMultivariateFunction const&>(EffectiveVectorMultivariateFunction::identity(dom.size()))) { }
    //! \brief Construct the image of \a dom under \a fn.
    ValidatedConstrainedImageSet(const ExactBoxType& dom, const ValidatedVectorMultivariateFunction& fn) : _domain(dom), _reduced_domain(dom), _function(fn) {
        ARIADNE_ASSERT_MSG(dom.size()==fn.argument_size(),"dom="<<dom<<", fn="<<fn); }
    ValidatedConstrainedImageSet(const ExactBoxType& dom, const ValidatedVectorMultivariateFunctionModelDP& fn) : _domain(dom), _reduced_domain(dom), _function(fn.raw_pointer()->_clone()) {
        ARIADNE_ASSERT_MSG(dom.size()==fn.argument_size(),"dom="<<dom<<", fn="<<fn); }
    //! \brief Construct the image of \a dom under \a fn.
    ValidatedConstrainedImageSet(const ExactBoxType& dom, const ValidatedVectorMultivariateFunction& fn, const List<ValidatedConstraint>& cnstr) : _domain(dom), _reduced_domain(dom), _function(fn), _constraints(cnstr) {
        ARIADNE_ASSERT_MSG(dom.size()==fn.argument_size(),"dom="<<dom<<", fn="<<fn); }

    //! \brief The domain of the set.
    const ExactBoxType& domain() const { return this->_domain; }
    const ExactBoxType& reduced_domain() const { return this->_reduced_domain; }
    //! \brief The function used to define the set.
    const ValidatedVectorMultivariateFunction& function() const { return this->_function; }
    //! \brief The constraints used to define the set.
    const List<ValidatedConstraint> constraints() const { return this->_constraints; }
    //! \brief The \a i<sup>th</sup> constraint.
    ValidatedConstraint const constraint(SizeType i) const { return this->_constraints[i]; };
    //! \brief The number of parameters used to define the set, which equals the dimension of \f$D\f$.
    SizeType number_of_parameters() const { return this->_domain.size(); };
    //! \brief The number of constraints.
    SizeType number_of_constraints() const { return this->_constraints.size(); };

    //! \brief Apply the function \f$h\f$ to obtain the set \f$h\circ f(D\cap g^{-1}(C))\f$.
    Void apply(const ValidatedVectorMultivariateFunction& h) {
        this->_function=compose(h,this->_function);
    }

    //! \brief Introduce a new constraint of the form \f$g(s)\in [c_l,c_u]\f$.
    Void new_parameter_constraint(const ValidatedConstraint& c) {
        ARIADNE_ASSERT_MSG(c.function().argument_size()==this->number_of_parameters(),*this<<", "<<c);
        this->_constraints.append(c);
    }

    //! \brief Introduce a new constraint of the form \f$g(x)\in [c_l,c_u]\f$.
    Void new_space_constraint(const ValidatedConstraint& c) {
        ARIADNE_ASSERT_MSG(c.function().argument_size()==this->dimension(),*this<<", "<<c);
        this->_constraints.append(ValidatedConstraint(c.lower_bound(),compose(c.function(),this->function()),c.upper_bound()));
    }

    virtual ValidatedConstrainedImageSet* clone() const override { return new ValidatedConstrainedImageSet(*this); }
    virtual DimensionType dimension() const override { return this->_function.result_size(); }

    ValidatedVectorMultivariateFunction constraint_function() const;
    ExactBoxType constraint_bounds() const;

    //! \brief Reduce the size of the domain by constraint propagation, if possible.
    Void reduce();
    //! \brief A coarse over-approximation to the set. Computed by taking the interval evaluation \f$h(D)\f$.
    virtual UpperBoxType bounding_box() const override;
    //! \brief Construct an affine over-approximation
    ValidatedAffineConstrainedImageSet affine_over_approximation() const;
    //! \brief Construct an affine approximation, with undefined accuracy.
    ValidatedAffineConstrainedImageSet affine_approximation() const;
    //! \brief Split into two pieces by subdividing along a coordinate direction.
    Pair<ValidatedConstrainedImageSet,ValidatedConstrainedImageSet> split() const;
    //! \brief Split into two pieces by subdividing along the \a j<sup>th</sup> coordinate direction.
    Pair<ValidatedConstrainedImageSet,ValidatedConstrainedImageSet> split(SizeType j) const;
    //! \brief Restrict the parameter domain to \a parameter_subdomain.
    ValidatedConstrainedImageSet restriction(ExactBoxType const& new_domain) const;

    //! \brief Test if the set is empty.
    ValidatedKleenean is_empty() const;
    //! \brief Test if the set is a strict subset of a box.
    virtual ValidatedLowerKleenean inside(const ExactBoxType& bx) const override;
    //! \brief Test if the set is disjoint from a box.
    virtual ValidatedLowerKleenean separated(const ExactBoxType&) const override;
    //! \brief Test if the set overlaps (intersects the interior of) a box.
    virtual ValidatedLowerKleenean overlaps(const ExactBoxType&) const override;
    //! \brief Adjoin an outer approximation to a paving.
    Void adjoin_outer_approximation_to(PavingInterface& paving, Nat fineness) const;
    //! \brief Compute an outer approximation on the \a grid to the given \a fineness.
    GridTreePaving outer_approximation(const Grid& grid, Nat fineness) const;

    //! \brief Test if the set satisfies the state constraint at all points.
    ValidatedKleenean satisfies(const ValidatedConstraint& c) const;

    //! \brief Draw to a canvas.
    virtual Void draw(CanvasInterface&, const Projection2d&) const override;
    Void box_draw(CanvasInterface&, const Projection2d&) const;
    Void affine_draw(CanvasInterface&, const Projection2d&, Nat splittings) const;
    Void grid_draw(CanvasInterface&, const Projection2d&, Nat fineness) const;

    friend ValidatedConstrainedImageSet image(ValidatedConstrainedImageSet set, ValidatedVectorMultivariateFunction const& h);
    friend ValidatedConstrainedImageSet join(const ValidatedConstrainedImageSet& set1, const ValidatedConstrainedImageSet& set2);

    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream&, const ValidatedConstrainedImageSet&);
  private:
    virtual OutputStream& _write(OutputStream&) const override;
};


} //namespace Ariadne


#endif
