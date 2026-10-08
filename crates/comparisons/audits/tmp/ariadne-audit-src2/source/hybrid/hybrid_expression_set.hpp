/***************************************************************************
 *            hybrid/hybrid_expression_set.hpp
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

/*! \file hybrid/hybrid_expression_set.hpp
 *  \brief Sets in hybrid spaces denoted by variables.
 */

#ifndef ARIADNE_HYBRID_EXPRESSION_SET_HPP
#define ARIADNE_HYBRID_EXPRESSION_SET_HPP

#include <map>
#include <memory>

#include "utility/macros.hpp"
#include "helper/stlio.hpp"
#include "utility/declarations.hpp"
#include "utility/container.hpp"
#include "geometry/function_set.hpp"
#include "geometry/list_set.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/curve.hpp"

#include "symbolic/expression_set.hpp"

#include "hybrid/hybrid_set.decl.hpp"
#include "hybrid/hybrid_set_interface.hpp"
#include "hybrid/hybrid_space.hpp"
#include "hybrid/hybrid_grid.hpp"
#include "geometry/point.hpp"
#include "geometry/box.hpp"

#include "hybrid/hybrid_graphics_interface.hpp"

namespace Ariadne {


//! \ingroup ExpressionSetSubModule
//! \ingroup HybridSetSubModule
//! \brief A hybrid set defined by a box in a single location.
//! \details Does not assume a canonical order of the real variables.
template<class IVL> class HybridVariablesBox
    : public Pair<DiscreteLocation,VariablesBox<IVL>>
{
    typedef Pair<DiscreteLocation,VariablesBox<IVL>> Base;
  public:
    HybridVariablesBox(const DiscreteLocation& loc, const VariablesBox<IVL>& bx)
        : Base(loc,bx) { }
    HybridVariablesBox(const DiscreteLocation& loc, const RealSpace& spc, const Box<IVL>& bx)
        : Base(loc,VariablesBox<IVL>(spc,bx)) { }

    //! \brief The location in which the box is defined.
    DiscreteLocation location() const { return this->Base::first; }
    //! \brief The location in which the box is defined.
    VariablesBox<IVL> const& continuous_set() const { return this->Base::second; }
    //! \brief The active variables in the location \a loc.
    Set<RealVariable> variables() const { return this->Base::second.variables(); }
    //! \brief The subset of \f$\mathbb{R}^n\f$ obtained by ordering the variables as defined by \a spc.
    Box<IVL> euclidean_set(const RealSpace& spc) const { return this->second.euclidean_set(spc); }
};

class HybridBoxSet
    : public virtual EffectiveHybridSetInterface
    , public virtual HybridDrawableInterface
    , public HybridVariablesBox<RealInterval>
{
  public:
    using HybridVariablesBox<RealInterval>::HybridVariablesBox;
    using HybridVariablesBox<RealInterval>::variables;
    using HybridVariablesBox<RealInterval>::euclidean_set;

    inline RealBox euclidean_set(DiscreteLocation, RealSpace) const;
    virtual Set<DiscreteLocation> locations() const override;
    virtual Set<RealVariable> variables(DiscreteLocation) const override;
    inline RealSpace space() const;
    virtual RealSpace space(DiscreteLocation) const;
    virtual LowerKleenean is_empty() const;

    virtual LowerKleenean overlaps(const HybridExactBox& bx) const override;
    virtual LowerKleenean separated(const HybridExactBox& bx) const override;
    virtual LowerKleenean covers(const HybridExactBox& bx) const override;
    virtual LowerKleenean inside(const HybridExactBox& bx) const;
//    virtual HybridUpperBox bounding_box() const;

    virtual LowerKleenean inside(const HybridExactBoxes& bx) const override;
    virtual HybridUpperBoxes bounding_box() const override;

    virtual OutputStream& _write(OutputStream& os) const override;
    virtual Void draw(CanvasInterface&, const Set<DiscreteLocation>&, const Variables2d&) const override;
  private:
    virtual HybridBoxSet* clone() const override;
    virtual EffectiveEuclideanSetInterface* _euclidean_set(DiscreteLocation, RealSpace) const override;
};

//! \ingroup ExpressionSetSubModule
//! \ingroup HybridSetSubModule
//! \brief A hybrid set defined by a box in each of a set of locations.
//! \details Does not assume a canonical order of the real variables.
template<class IVL> class HybridVariablesBoxes
    : public Map<DiscreteLocation,VariablesBox<IVL>>
{
    typedef Map<DiscreteLocation,VariablesBox<IVL>> Base;
  public:
    HybridVariablesBoxes() : Base() { }
    HybridVariablesBoxes(Set<DiscreteLocation> locs, VariablesBox<IVL> const& box) {
        for(auto loc : locs) { this->Base::insert(loc,box); } }
    HybridVariablesBoxes(List<HybridVariablesBox<IVL>> const& hboxes) {
        for(auto hbox : hboxes) { this->adjoin(hbox); } }

    HybridVariablesBoxes<IVL>& adjoin(HybridVariablesBox<IVL> const& hbox){
        this->Base::insert(hbox.location(),hbox.continuous_set()); return *this; }

    //! \brief The location in which the box is defined.
    Set<DiscreteLocation> locations() const { return this->Base::keys(); }
    //! \brief The location in which the box is defined.
    VariablesBox<IVL> const& continuous_set(DiscreteLocation loc) const { return (*this)[loc]; }
    //! \brief The active variables in the location \a loc.
    Set<RealVariable> variables(DiscreteLocation loc) const { return this->continuous_set(loc).variables(); }
    //! \brief The subset of \f$\mathbb{R}^n\f$ obtained by ordering the variables as defined by \a spc.
    Box<IVL> euclidean_set(DiscreteLocation loc, const RealSpace& spc) const { return this->continuous_set(loc).euclidean_set(spc); }
};

class HybridBoxesSet
    // TODO: Support
    : public HybridVariablesBoxes<RealInterval>
{
  public:
    using HybridVariablesBoxes<RealInterval>::HybridVariablesBoxes;
};


//! \ingroup ExpressionSetSubModule
//! \ingroup HybridSetSubModule
//! \brief A hybrid set defined by a constraint system in each location.
//! NOTE: If a location is not specified, the set is considered empty. This may be changed in future to the set being considered unconstrained (entire).
class HybridConstraintSet
    : public virtual EffectiveHybridRegularSetInterface
{
    Map<DiscreteLocation, RealExpressionConstraintSet> _sets;
  public:
    HybridConstraintSet();
    //! \brief Construct a set in a single \a location with a list of \a bounds on the variables and nonlinear \a constraints.
    HybridConstraintSet(const DiscreteLocation& location,
                        const List<ContinuousPredicate>& constraints);
    HybridConstraintSet(const DiscreteLocation& location,
                        const RealExpressionConstraintSet& constraint_set);
    HybridConstraintSet(const Set<DiscreteLocation>& locations,
                        const RealExpressionConstraintSet& constraint_set);

    //! \brief Adjoin the set defined by \a constraints in \a location.
    HybridConstraintSet& adjoin(const DiscreteLocation& location, const List<ContinuousPredicate>& set);
    HybridConstraintSet& adjoin(const DiscreteLocation& location, const RealExpressionConstraintSet& set);

    //! \brief Intersect with a set of boxes, one in each location, to form a bounded set.
    friend HybridBoundedConstraintSet intersection(const HybridBoxesSet& bounds, HybridConstraintSet const& constraints);

    virtual HybridConstraintSet* clone() const override;


    //! \brief The set of locations in which the set is nonempty.
    Set<DiscreteLocation> locations() const;
    Bool has_location(DiscreteLocation loc) const;
    //! \brief The active variables in the location \a loc.
    virtual Set<RealVariable> variables(DiscreteLocation loc) const override;
    //! \brief The subset of \f$\mathbb{R}^V\f$ obtained by restricting to location \a loc.
    RealExpressionConstraintSet const& continuous_set(DiscreteLocation loc) const;
    //! \brief The subset of \f$\mathbb{R}^n\f$ obtained by restricting to location \a loc and ordering the variables as defined by \a spc.
    ConstraintSet const euclidean_set(DiscreteLocation loc, RealSpace spc) const;

    virtual LowerKleenean overlaps(const HybridExactBox& bx) const override;
    virtual LowerKleenean separated(const HybridExactBox& bx) const override;
    virtual LowerKleenean covers(const HybridExactBox& bx) const override;

    virtual OutputStream& _write(OutputStream& os) const override;
  protected:
    virtual EffectiveEuclideanRegularSetInterface* _euclidean_set(DiscreteLocation loc, RealSpace spc) const override;
};

//! \ingroup ExpressionSetSubModule
//! \ingroup HybridSetSubModule
//! \brief A hybrid set defined by the intersection of a box and a constraint system in each location.
class HybridBoundedConstraintSet
    : public virtual EffectiveHybridSetInterface
    , public virtual HybridDrawableInterface
{
    Map<DiscreteLocation, RealExpressionBoundedConstraintSet> _sets;
  public:
    HybridBoundedConstraintSet();
    HybridBoundedConstraintSet(const DiscreteLocation& loc,
                               const RealVariablesBox& bx);
    HybridBoundedConstraintSet(const DiscreteLocation& loc,
                               const RealExpressionBoundedConstraintSet& set);
    HybridBoundedConstraintSet(const DiscreteLocation& loc,
                               const InitializerList<RealVariableInterval>& bnd);
    //! \brief Construct a set in a single \a location with a list of \a bounds on the variables and nonlinear \a constraints.
    HybridBoundedConstraintSet(const DiscreteLocation& location,
                               const InitializerList<RealVariableInterval>& bounds,
                               const InitializerList<ContinuousPredicate>& constraints);
    HybridBoundedConstraintSet(const Set<DiscreteLocation>& locations,
                               const RealExpressionBoundedConstraintSet& constraint_set);

    HybridBoundedConstraintSet& adjoin(const DiscreteLocation& loc, const RealExpressionBoundedConstraintSet& set);

    virtual HybridBoundedConstraintSet* clone() const override;

    //! \brief If the set only has a single location, returns this, otherwise fails. DEPRECATED
    DiscreteLocation location() const;

    //! \brief The set of discrete locations in which the set is nontrivial.
    virtual Set<DiscreteLocation> locations() const override;
    //! \brief The active variables in the location \a loc.
    virtual Set<RealVariable> variables(DiscreteLocation loc) const override;
    //! \brief The subset of \f$\mathbb{R}^V\f$ obtained by restricting to location \a loc.
    RealExpressionBoundedConstraintSet const& continuous_set(DiscreteLocation loc) const;
    //! \brief The subset of \f$\mathbb{R}^n\f$ obtained by restricting to location \a loc and ordering the variables as defined by \a spc.
    BoundedConstraintSet const euclidean_set(DiscreteLocation loc, RealSpace spc) const;

    virtual LowerKleenean overlaps(const HybridExactBox& bx) const override;
    virtual LowerKleenean inside(const HybridExactBoxes& bx) const override;

    virtual LowerKleenean separated(const HybridExactBox& bx) const override;
    virtual LowerKleenean covers(const HybridExactBox& bx) const override;
    virtual HybridUpperBoxes bounding_box() const override;

    virtual OutputStream& _write(OutputStream& os) const override;
    virtual Void draw(CanvasInterface&, const Set<DiscreteLocation>&, const Variables2d&) const override;
  protected:
    virtual BoundedConstraintSet* _euclidean_set(DiscreteLocation loc, RealSpace spc) const override;
};

} // namespace Ariadne

#endif // ARIADNE_HYBRID_EXPRESSION_SET_HPP
