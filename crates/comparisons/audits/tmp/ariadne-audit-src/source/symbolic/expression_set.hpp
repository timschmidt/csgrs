/***************************************************************************
 *            symbolic/expression_set.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

/*! \file symbolic/expression_set.hpp
 *  \brief Sets defined using expressions over real variables.
 */

#ifndef ARIADNE_EXPRESSION_SET_HPP
#define ARIADNE_EXPRESSION_SET_HPP

#include <iostream>

#include "utility/container.hpp"
#include "utility/declarations.hpp"
#include "symbolic/variable.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/space.hpp"
#include "numeric/floats.hpp"
#include "numeric/real.hpp"
#include "function/projection.hpp"
#include "geometry/box.hpp"

namespace Ariadne {

template<class UB> class Interval;
template<class IVL> class Box;
template<class BS> class ListSet;
class ConstraintSet;
class BoundedConstraintSet;

class ValidatedConstrainedImageSet;

template<class X> class Variable;
typedef Variable<Real> RealVariable;
template<class X> class Expression;
typedef Expression<Real> RealExpression;
template<class X> class Space;
typedef Space<Real> RealSpace;

template<class UB> class VariableInterval;
template<class IVL> class VariablesBox;
template<class S> class LabelledSet;

typedef VariableInterval<Real> RealVariableInterval;
typedef VariablesBox<RealInterval> RealVariablesBox;
typedef VariablesBox<ExactIntervalType> ExactVariablesBoxType;

class RealExpressionConstraintSet;
class RealExpressionBoundedConstraintSet;

Set<Identifier> variables(const Map<RealVariable,RealInterval>& bnds);
Set<Identifier> variables(const List<RealVariableInterval>& bnds);
Set<Identifier> arguments(const List<ContinuousPredicate>& c);

ExactIntervalType over_approximation(RealInterval ivl);
ExactIntervalType under_approximation(RealInterval ivl);
ExactIntervalType approximation(RealInterval ivl);
ExactVariablesBoxType over_approximation(const RealVariablesBox& ebx);
ExactVariablesBoxType approximation(const RealVariablesBox& ebx);
ExactVariablesBoxType under_approximation(const RealVariablesBox& ebx);

ValidatedConstrainedImageSet approximate_euclidean_set(const RealExpressionBoundedConstraintSet& set, const RealSpace& space);

template<class C, class P> decltype(auto) any(C const& c, P const& p) {
    typedef decltype(p(*c.begin())) R;
    R r=false; for(auto e:c) { r=r or p(e); if(definitely(r)) { return r; } } return r;
}
template<class C, class P> decltype(auto) all(C const& c, P const& p) {
    typedef decltype(p(*c.begin())) R;
    R r=true; for(auto e:c) { r=r and p(e); if(definitely(not r)) { return r; } } return r;
}

//! \ingroup ExpressionSetSubModule
//! \brief An interval range for a real variable.
template<class UB> class VariableInterval {
    typedef decltype(-declval<UB>()) LB;
  public:
    typedef LB LowerBoundType;
    typedef UB UpperBoundType;
    typedef Interval<UB> IntervalType;
  public:
    template<class U> requires Constructible<UB,U> VariableInterval(VariableInterval<U> const& eivl)
        : VariableInterval(eivl.variable(), Interval<UB>(eivl.interval())) { }
    VariableInterval(const RealVariable& v, const IntervalType& ivl) : _variable(v), _ivl(ivl) { }
    VariableInterval(const LowerBoundType& l, const Variable<Real>& v, const UpperBoundType& u) : _variable(v), _ivl(l,u) { }
    RealVariable const& variable() const { return this->_variable; }
    const IntervalType interval() const { return this->_ivl; }
    const LowerBoundType lower_bound() const { return this->_ivl.lower_bound(); }
    const UpperBoundType upper_bound() const { return this->_ivl.upper_bound(); }
    friend OutputStream& operator<<(OutputStream& os, const VariableInterval<UB>& eivl) {
        //return os << eivl._variable << ".in(" << eivl._ivl << "\n"; }
        return os << eivl._ivl.lower_bound() << "<=" << eivl._variable << "<=" << eivl._ivl.upper_bound(); }
  private:
    RealVariable _variable;
    IntervalType _ivl;
};

//! \ingroup ExpressionSetSubModule
//! \brief An interval range for a real variable.
template<class T> template<class XL, class XU> inline VariableInterval<XU> Variable<T>::in(const XL& l, const XU& u) {
    //static_assert(Same<XL,Real>,"Can only make box in Real variables.");
    ARIADNE_FAIL_MESSAGE("Can't create interval in non-real variable "<<*this);
    assert(false);
}

template<> template<class XL, class XU> inline VariableInterval<XU> Variable<Real>::in(const XL& l, const XU& u) {
    return VariableInterval<XU>(l,*this,u);
}

template<class UB> class VariableLowerInterval
    : public DeclareExpressionOperations<Kleenean>
{
    typedef NegationType<UB> LB;
    RealVariable _variable; LB _lower;
  public:
    template<class U> requires Constructible<UB,U> VariableLowerInterval(VariableLowerInterval<U> const& lv)
        : VariableLowerInterval<UB>(UB(lv.lower_bound()),lv.variable()) { }
    VariableLowerInterval(const LB& l, const RealVariable& v) : _variable(v),  _lower(l) { }
    operator VariableInterval<UB>() const { return VariableInterval<UB>(_lower,_variable,+infinity); }
    LB lower_bound() const { return _lower; }
    const RealVariable& variable() const { return _variable; }
    operator Expression<Kleenean>() const {
        static_assert(Same<UB,Real>); return ( Real(this->_lower) <= RealExpression(this->_variable) ); };
    friend OutputStream& operator<<(OutputStream& os, const VariableLowerInterval<UB>& elivl) {
        return os << elivl._lower << "<=" << elivl._variable; }
};

template<class UB> class VariableUpperInterval
    : public DeclareExpressionOperations<Kleenean>
{
    typedef NegationType<UB> LB;
    RealVariable _variable; UB _upper;
  public:
    VariableUpperInterval(const RealVariable& v, const UB& u) : _variable(v), _upper(u)  { }
    operator VariableInterval<UB>() const { return VariableInterval<UB>(-infinity,_variable,_upper); }
    const RealVariable& variable() const { return _variable; }
    UB upper_bound() const { return _upper; }
    operator Expression<Kleenean>() const {
        static_assert(Same<UB,Real>); return ( RealExpression(this->_variable) <= Real(this->_upper) ); }
    friend OutputStream& operator<<(OutputStream& os, const VariableUpperInterval<UB>& euivl) {
        return os << euivl._variable << "<=" << euivl._upper; }
};

typedef VariableLowerInterval<Real> RealVariableLowerInterval;
typedef VariableUpperInterval<Real> RealVariableUpperInterval;

template<class X> using UnaryPlusType = decltype(+declval<X>());
template<class X> using UnaryMinusType = decltype(-declval<X>());

template<class UB> inline VariableInterval<UB> operator<=(const VariableLowerInterval<UB>& lv, const UnaryPlusType<UB>& u) {
    return VariableInterval<UB>(lv.lower_bound(),lv.variable(),u); }
template<class UB> inline VariableInterval<UB> operator>=(const UnaryPlusType<UB>& u, const VariableLowerInterval<UB>& lv) {
    return VariableInterval<UB>(lv.lower_bound(),lv.variable(),u); }
template<class UB> inline VariableInterval<UB> operator<=(const UnaryMinusType<UB>& l, const VariableUpperInterval<UB>& vu) {
    return VariableInterval<UB>(l,vu.variable(),vu.upper_bound()); }
template<class UB> inline VariableInterval<UB> operator>=(const VariableUpperInterval<UB>& vu, const UnaryMinusType<UB>& l) {
    return VariableInterval<UB>(l,vu.variable(),vu.upper_bound()); }

inline VariableLowerInterval<Real> operator<=(const Real& l, const RealVariable& v) {
    return VariableLowerInterval<Real>(l,v); }
inline VariableLowerInterval<Real> operator>=(const RealVariable& v, const Real& l) {
    return VariableLowerInterval<Real>(l,v); }
inline VariableUpperInterval<Real> operator<=(const RealVariable& v, const Real& u) {
    return VariableUpperInterval<Real>(v,u); }
inline VariableUpperInterval<Real> operator>=(const Real& u, const RealVariable& v) {
    return VariableUpperInterval<Real>(v,u); }
inline VariableInterval<Real> operator==(const RealVariable& v, const Real& x) {
    return VariableInterval<Real>(x,v,x); }
inline VariableInterval<Real> operator==(const Real& x, const RealVariable& v) {
    return VariableInterval<Real>(x,v,x); }
inline VariableInterval<Real> operator<=(const VariableLowerInterval<Real>& lv, const Real& u) {
    return VariableInterval<Real>(lv.lower_bound(),lv.variable(),u); }

inline VariableLowerInterval<FloatDP> operator<=(const FloatDP& l, const RealVariable& v) { return VariableLowerInterval<FloatDP>(l,v); }
inline VariableLowerInterval<Dyadic> operator<=(const Dyadic& l, const RealVariable& v) { return VariableLowerInterval<Dyadic>(l,v); }
inline VariableLowerInterval<Dyadic> operator<=(const Int& l, const RealVariable& v) { return VariableLowerInterval<Dyadic>(l,v); }

inline VariableLowerInterval<Double> operator<=(const Double& l, const RealVariable& v) { return VariableLowerInterval<Double>(l,v); }


//! \ingroup ExpressionSetSubModule
//! \brief An point defining values for a collection of real variables.
template<class X> class VariablesPoint {
    Map<RealVariable,X> _vals;
  public:
    typedef X CoordinateType;
    typedef Point<X> PointType;
    typedef VariablesPoint<X> VariablesPointType;

    VariablesPoint() : _vals() { }
    VariablesPoint(const RealSpace& spc, const PointType& pt);
    VariablesPoint(const Map<RealVariable,CoordinateType>& vals) : _vals(vals) { }
    template<class XX> requires Convertible<XX,X> VariablesPoint(const LabelledSet<Point<XX>>& lpt);
    template<class XX> requires Constructible<X,XX> VariablesPoint(VariablesPoint<XX> const& ept) : _vals(ept.values()) { }
    Map<RealVariable,CoordinateType> values() const { return this->_vals; }
    Set<RealVariable> variables() const { return this->_vals.keys(); }
    const CoordinateType& operator[](const RealVariable& v) const { return this->_vals[v]; }
    PointType euclidean_set(const RealSpace& spc) const {
        return PointType(spc.dimension(),[this,&spc](SizeType i){
            auto entry = this->_vals.find(spc[i]); ARIADNE_ASSERT_MSG(entry != this->_vals.end(),"Variable " << spc[i] << " not found in VariablesPoint."); return entry->second; }); }
    friend OutputStream& operator<<(OutputStream& os, const VariablesPointType& ept) {
        return os << "VariablesPoint"<<class_name<X>()<<"( values=" << ept.values() << " )"; }
    explicit operator LabelledSet<Point<X>> () const;
};

template<class X> VariablesPoint<X>::VariablesPoint(const RealSpace& spc, const Point<X>& pt) {
    ARIADNE_PRECONDITION(spc.dimension()==pt.dimension());
    for(SizeType i=0; i!=spc.dimension(); ++i) {
        this->_vals.insert(spc.variable(i),pt[i]);
    }
}

//! \ingroup ExpressionSetSubModule
//! \brief An box defining ranges for a collection of real variables.
template<class IVL> class VariablesBox {
    typedef typename IVL::UpperBoundType UB;
    Map<RealVariable,IVL> _bnds;
  public:
    typedef IVL IntervalType;
    typedef VariableInterval<UB> VariableIntervalType;
    typedef Box<IVL> BoxType;
    typedef VariablesBox<IVL> VariablesBoxType;

    VariablesBox() : _bnds() { }
    template<class I> requires Convertible<I,IVL> VariablesBox(const LabelledSet<Box<I>>& lbx);
    VariablesBox(const RealSpace& spc, const Box<IVL>& bx);
    VariablesBox(const Map<RealVariable,IntervalType>& bnds) : _bnds(bnds) { }
    VariablesBox(const Set<VariableIntervalType>& bnds) : _bnds() {
        for(auto bnd : bnds) { this->_bnds.insert(bnd.variable(),bnd.interval()); } }
    VariablesBox(const List<VariableIntervalType>& bnds) :  _bnds() {
        for(auto bnd : bnds) { this->_bnds.insert(bnd.variable(),bnd.interval()); } }
    VariablesBox(const InitializerList<VariableIntervalType>& lst) : VariablesBox(List<VariableIntervalType>(lst)) { }
    template<class I> requires Constructible<IVL,I> VariablesBox(VariablesBox<I> const& ebx) : _bnds(ebx.bounds()) { }
    Map<RealVariable,IntervalType> const& bounds() const { return this->_bnds; }
    Set<RealVariable> variables() const { return this->_bnds.keys(); }
    const IntervalType& operator[](const RealVariable& v) const { return this->_bnds[v]; }
    BoxType euclidean_set(const RealSpace& spc) const {
        return BoxType(spc.dimension(),[this,&spc](SizeType i){auto entry = this->_bnds.find(spc[i]); ARIADNE_ASSERT_MSG(entry != this->_bnds.end(),"Variable " << spc[i] << " not found in VariablesBox."); return entry->second; }); }
    decltype(auto) is_empty() const { return any(_bnds,[](auto e){return e.second.is_empty();}); }
    friend OutputStream& operator<<(OutputStream& os, const VariablesBoxType& ebx) {
        return os << "VariablesBox"<<class_name<IVL>()<<"( bounds=" << ebx.bounds() << " )"; }
    explicit operator LabelledSet<Box<IVL>> () const;
};

template<class IVL> VariablesBox<IVL>::VariablesBox(const RealSpace& spc, const Box<IVL>& bx) {
    ARIADNE_PRECONDITION(spc.dimension()==bx.dimension());
    for(SizeType i=0; i!=spc.dimension(); ++i) {
        this->_bnds.insert(spc.variable(i),bx[i]);
    }
}

template<class T> template<class IVL> inline VariablesBox<IVL> Variables<T>::in(const List<IVL>& bx) const {
    static_assert(Same<T,Real>,"Can only make box in Real variables.");
    assert(false);
}

template<> template<class IVL> inline VariablesBox<IVL> Variables<Real>::in(const List<IVL>& bx) const {
    ARIADNE_PRECONDITION(this->size()==bx.size());
    Map<RealVariable,IVL> bnds;
    for(SizeType i=0; i!=this->size(); ++i) { bnds.insert((*this)[i],bx[i]); }
    return bnds;
}

//! \ingroup ExpressionSetSubModule
//! \brief A set defined as the preimage of a box under a continuous function.
//! The set is described as \f$S=g^{-1}(C)\f$ where \f$g\f$ the constraint function and \f$C\f$ the codomain.
class RealExpressionConstraintSet
{
    List<ContinuousPredicate> _constraints;
  public:
    RealExpressionConstraintSet(const List<ContinuousPredicate>& constraints);
    Set<RealVariable> variables() const { return Set<RealVariable>(arguments(this->_constraints)); }
    List<ContinuousPredicate> const& constraints() const { return this->_constraints; }
    ConstraintSet euclidean_set(const RealSpace& space) const;
    friend OutputStream& operator<<(OutputStream& os, const RealExpressionConstraintSet& eset);

    friend RealExpressionBoundedConstraintSet intersection(RealVariablesBox const&, RealExpressionConstraintSet const&);

};

//! \ingroup ExpressionSetSubModule
//! \brief A set defined as the intersection of a box and the preimage of a box under a continuous function.
//! The set is described as \f$S=D\cap g^{-1}(C)\f$ where \f$D\f$ is the domain, \f$g\f$ the constraint function and \f$C\f$ the codomain.
class RealExpressionBoundedConstraintSet
{
    Map<RealVariable,RealInterval> _bounds;
    List<ContinuousPredicate> _constraints;
  public:
    RealExpressionBoundedConstraintSet(const InitializerList<RealVariableInterval>& domain);
    RealExpressionBoundedConstraintSet(const InitializerList<RealVariableInterval>& domain, const InitializerList<ContinuousPredicate>& constraints);
    RealExpressionBoundedConstraintSet(const List<RealVariableInterval>& domain);
    RealExpressionBoundedConstraintSet(const List<RealVariableInterval>& domain, const List<ContinuousPredicate>& constraints);
    RealExpressionBoundedConstraintSet(const Map<RealVariable,RealInterval>& bounds);
    RealExpressionBoundedConstraintSet(const Map<RealVariable,RealInterval>& bounds, const List<ContinuousPredicate>& constraints);
    RealExpressionBoundedConstraintSet(const RealVariablesBox& box);
    RealExpressionBoundedConstraintSet(const RealVariablesBox& box, const RealExpressionConstraintSet& set);
    Set<RealVariable> variables() const { return this->_bounds.keys(); }
    Map<RealVariable,RealInterval> bounds() const { return this->_bounds; }
    List<ContinuousPredicate> const& constraints() const { return this->_constraints; }
    BoundedConstraintSet euclidean_set(const RealSpace& space) const;
    friend OutputStream& operator<<(OutputStream& os, const RealExpressionBoundedConstraintSet& eset);
};


//! \brief A set formed by labelling the variables of a Euclidean set of type \a S.
template<class S> class LabelledSet {
    RealSpace _spc;
    S _set;
  public:
    typedef S EuclideanSetType;

    LabelledSet(const RealSpace& spc, const EuclideanSetType& set) : _spc(spc), _set(set) { ARIADNE_ASSERT_MSG(spc.dimension()==set.dimension(),"spc="<<spc<<", set.dimension()="<<set.dimension()); }
    Set<RealVariable> variables() const { return _spc.variables(); }
    RealSpace const& space() const { return this->_spc; }
    EuclideanSetType const& continuous_set() const { return this->_set; }
    EuclideanSetType const& euclidean_set() const { return this->_set; }
    EuclideanSetType& euclidean_set() { return this->_set; }
    EuclideanSetType euclidean_set(const RealSpace& vars) const {
        Array<SizeType> prj(vars.dimension()); for(SizeType i=0; i!=vars.dimension(); ++i) { prj[i]=this->_spc.index(vars.variable(i)); } return project(this->_set,prj); }
    DimensionType dimension() const { return this->euclidean_set().dimension(); }
    template<class T> Void adjoin(LabelledSet<T> const& other) const {
        ARIADNE_ASSERT(this->space()==other.space()); this->euclidean_set().adjoin(other.euclidean_set()); }
    decltype(auto) bounding_box() const { return LabelledSet(this->space(),this->euclidean_set().bounding_box()); }
    decltype(auto) bounding_boxes() const { return LabelledSet(this->space(),this->euclidean_set().bounding_boxes()); }
    Void draw(CanvasInterface& cnvs, Variables2d const& vars) const;

    template<class E> friend decltype(auto) widen(const LabelledSet<S>& set, const E& e) {
        return LabelledSet(set.space(),widen(set.euclidean_set(),e)); }
    friend OutputStream& operator<<(OutputStream& os, const LabelledSet<S>& eset) {
        return os << "LabelledSet( space=" << eset.space() << ", set=" << eset.continuous_set() << " )"; }
};
template<class S> LabelledSet(const RealSpace&, const S&) -> LabelledSet<S>;

template<class IVL> class LabelledBox : public LabelledSet<Box<IVL>> {
  public:
    LabelledBox(const RealSpace& spc, const Box<IVL>& set) : LabelledSet<Box<IVL>>(spc,set) { }
    IVL operator[](const RealVariable& v) const { return VariablesBox<IVL>(*this)[v]; }
};

template<class S> EqualsType<S> operator==(LabelledSet<S> const& eset1, LabelledSet<S> const& eset2) {
    if(eset1.variables()!=eset2.variables()) { return false; }
    ARIADNE_ASSERT(eset1.space()==eset2.space());
    return eset1.euclidean_set()==eset2.euclidean_set();
}

template<class X> VariablesPoint<X>::operator LabelledSet<Point<X>>() const {
    RealSpace spc(List<RealVariable>(this->variables()));
    return LabelledSet<Point<X>>(spc,this->euclidean_set(spc));
}

template<class IVL> VariablesBox<IVL>::operator LabelledSet<Box<IVL>>() const {
    RealSpace spc(List<RealVariable>(this->variables()));
    return LabelledSet<Box<IVL>>(spc,this->euclidean_set(spc));
}

template<class IVL> template<class I> requires Convertible<I,IVL> VariablesBox<IVL>::VariablesBox(LabelledSet<Box<I>> const& lbx)
    : VariablesBox(lbx.space(),lbx.euclidean_set()) { }


template<class UB> using LabelledInterval = VariableInterval<UB>;
template<class BS> using LabelledListSet = LabelledSet<ListSet<BS>>;

RealBox make_box(RealSpace const&, RealVariablesBox const&);
RealBox make_set(RealSpace const&, RealVariablesBox const&);
ConstraintSet make_set(RealSpace const&, RealExpressionConstraintSet const&);
BoundedConstraintSet make_set(RealSpace const&, RealExpressionBoundedConstraintSet const&);
BoundedConstraintSet make_set(RealSpace const&, RealVariablesBox const&, RealExpressionConstraintSet const&);


} // namespace Ariadne



#endif
