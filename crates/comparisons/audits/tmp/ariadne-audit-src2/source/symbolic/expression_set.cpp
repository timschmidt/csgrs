/***************************************************************************
 *            symbolic/expression_set.cpp
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

#include "utility/standard.hpp"
#include "config.hpp"

#include "symbolic/expression_set.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/space.hpp"
#include "function/constraint.hpp"
#include "geometry/box.hpp"
#include "geometry/function_set.hpp"



namespace Ariadne {

template<class T> struct KeyValue { };

template<> struct KeyValue<RealVariableInterval> {
    typedef RealVariable KeyType; typedef RealInterval ValueType;
    KeyType const key(const RealVariableInterval& ivl) { return ivl.variable(); }
    ValueType value(const RealVariableInterval& ivl) { return ivl.interval(); }
};

template<class T> Bool unique_keys(const List<T>& lst) {
    typedef typename KeyValue<T>::KeyType K;
    Set<K> keys;
    for(typename List<T>::ConstIterator lst_iter=lst.begin(); lst_iter!=lst.end(); ++lst_iter) {
        const K k=KeyValue<T>().key(*lst_iter);
        if(keys.contains(k)) {
            return false;
        }
        keys.insert(k);
    }
    return true;
}

template<class T> Map<typename KeyValue<T>::KeyType,typename KeyValue<T>::ValueType>
make_key_value_map(const List<T>& lst) {
    typedef typename KeyValue<T>::KeyType K;
    typedef typename KeyValue<T>::ValueType V;
    Map<K,V> map;
    for(typename List<T>::ConstIterator lst_iter=lst.begin(); lst_iter!=lst.end(); ++lst_iter) {
        map.insert(KeyValue<T>().key(*lst_iter),KeyValue<T>().value(*lst_iter));
    }
    return map;
}

template<class K, class V> List<V> order(const Map<K,V>& map, const List<K>& key_ordering) {
    List<V> result;
    result.reserve(key_ordering.size());
    for(auto key : key_ordering) { result.append(map[key]); }
    return result;
}

Set<Identifier> variables(const Map<RealVariable,RealInterval>& bnds) {
    Set<Identifier> r;
    for(auto bnd : bnds) { r.insert(bnd.first.name()); }
    return r;
}

Set<Identifier> variables(const List<RealVariableInterval>& bnds) {
    Set<Identifier> r;
    for(auto bnd : bnds) { r.insert(bnd.variable().name()); }
    return r;
}

Set<Identifier> arguments(const List<ContinuousPredicate>& cs) {
    Set<Identifier> r;
    for(auto c : cs) { r.adjoin(arguments(c)); }
    return r;
}




ExactIntervalType over_approximation(RealInterval ivl) {
    DoublePrecision prec;
    return cast_exact_interval(UpperIntervalType(ivl,prec));
}

ExactIntervalType under_approximation(RealInterval ivl) {
    DoublePrecision prec;
    return cast_exact_interval(LowerIntervalType(ivl,prec));
}

ExactIntervalType approximation(RealInterval ivl) {
    DoublePrecision prec;
    return cast_exact_interval(ApproximateIntervalType(ivl,prec));
}


ExactVariablesBoxType over_approximation(const RealVariablesBox& ebx) {
    Map<RealVariable,ExactIntervalType> result;
    for(Map<RealVariable,RealInterval>::ConstIterator iter=ebx.bounds().begin();
        iter!=ebx.bounds().end(); ++iter)
    {
        result.insert(iter->first,over_approximation(iter->second));
    }
    return result;
}

ExactVariablesBoxType approximation(const RealVariablesBox& ebx) {
    Map<RealVariable,ExactIntervalType> result;
    for(Map<RealVariable,RealInterval>::ConstIterator iter=ebx.bounds().begin();
        iter!=ebx.bounds().end(); ++iter)
    {
        result[iter->first]=approximation(iter->second);
    }
    return result;
}

ExactVariablesBoxType under_approximation(const RealVariablesBox& ebx) {
    Map<RealVariable,ExactIntervalType> result;
    for(Map<RealVariable,RealInterval>::ConstIterator iter=ebx.bounds().begin();
        iter!=ebx.bounds().end(); ++iter)
    {
        result[iter->first]=under_approximation(iter->second);
    }
    return result;
}



RealExpressionConstraintSet::RealExpressionConstraintSet(const List<ContinuousPredicate>& constraints)
    : _constraints(constraints)
{
}

ConstraintSet RealExpressionConstraintSet::euclidean_set(const RealSpace& space) const {
    ARIADNE_ASSERT( subset(this->variables(), Set<RealVariable>(space.variables())) );
    const RealExpressionConstraintSet& set = *this;
    List<EffectiveConstraint> constraints;
    for(SizeType i=0; i!=set.constraints().size(); ++i) {
        RealExpression constraint_expression=indicator(set.constraints()[i],Sign::NEGATIVE);
        EffectiveScalarMultivariateFunction constraint_function( Ariadne::make_function(space,constraint_expression) );
        constraints.append( constraint_function <= Real(0) );
    }
    return ConstraintSet(constraints);
}

OutputStream& operator<<(OutputStream& os, const RealExpressionConstraintSet& eset) {
    os << "[";
    for(List<ContinuousPredicate>::ConstIterator iter=eset._constraints.begin(); iter!=eset._constraints.end(); ++iter) {
        os << (iter==eset._constraints.begin()?"":",") << *iter; }
    return os << "]";
}



RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const InitializerList<RealVariableInterval>& bounds)
    : RealExpressionBoundedConstraintSet(List<RealVariableInterval>(bounds))
{
}

RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const InitializerList<RealVariableInterval>& bounds, const InitializerList<ContinuousPredicate>& constraints)
    : RealExpressionBoundedConstraintSet(List<RealVariableInterval>(bounds),List<ContinuousPredicate>(constraints))
{
}

RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const List<RealVariableInterval>& bounds)
    : _bounds(make_key_value_map(bounds)), _constraints()
{
    ARIADNE_ASSERT(unique_keys(bounds));
}

RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const List<RealVariableInterval>& bounds, const List<ContinuousPredicate>& constraints)
    : RealExpressionBoundedConstraintSet(make_key_value_map(bounds),constraints)
{
    ARIADNE_ASSERT(unique_keys(bounds));
}

RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const Map<RealVariable,RealInterval>& bounds)
        : RealExpressionBoundedConstraintSet(bounds,List<ContinuousPredicate>())
{
}

RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const Map<RealVariable,RealInterval>& bounds, const List<ContinuousPredicate>& constraints)
    : _bounds(bounds), _constraints(constraints)
{
    ARIADNE_ASSERT( subset(arguments(constraints),Ariadne::variables(_bounds)) );
}

RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const RealVariablesBox& box)
    : RealExpressionBoundedConstraintSet(box.bounds())
{

}

RealExpressionBoundedConstraintSet::RealExpressionBoundedConstraintSet(const RealVariablesBox& box, const RealExpressionConstraintSet& set)
    : RealExpressionBoundedConstraintSet(box.bounds(), set.constraints())
{

}

BoundedConstraintSet RealExpressionBoundedConstraintSet::euclidean_set(const RealSpace& space) const {
    ARIADNE_ASSERT( this->variables() == make_set(space.variables()) );
    const RealExpressionBoundedConstraintSet& set = *this;
    RealBox domain=RealVariablesBox(set.bounds()).euclidean_set(space);
    List<EffectiveConstraint> constraints;
    for(SizeType i=0; i!=set.constraints().size(); ++i) {
        RealExpression constraint_expression=indicator(set.constraints()[i],Sign::NEGATIVE);
        EffectiveScalarMultivariateFunction constraint_function( Ariadne::make_function(space,constraint_expression) );
        constraints.append( constraint_function <= Real(0) );
    }
    return BoundedConstraintSet(domain,constraints);}

OutputStream& operator<<(OutputStream& os, const RealExpressionBoundedConstraintSet& eset) {
    os << "[";
    for(Map<RealVariable,RealInterval>::ConstIterator iter=eset._bounds.begin(); iter!=eset._bounds.end(); ++iter) {
        os << (iter==eset._bounds.begin()?"":",") << *iter; }
    os << ";";
    for(List<ContinuousPredicate>::ConstIterator iter=eset._constraints.begin(); iter!=eset._constraints.end(); ++iter) {
        os << (iter==eset._constraints.begin()?"":",") << *iter; }
    return os << "]";
    return os << eset._bounds << eset._constraints;
}

RealExpressionBoundedConstraintSet intersection(RealVariablesBox const& bx, RealExpressionConstraintSet const& cs) {
    return RealExpressionBoundedConstraintSet(bx.bounds(),cs.constraints());
}

ValidatedConstrainedImageSet approximate_euclidean_set(const RealExpressionBoundedConstraintSet& set, const RealSpace& space) {
    RealBox real_domain = RealVariablesBox(set.bounds()).euclidean_set(space);
    ExactBoxType domain=cast_exact_box(ApproximateBoxType(real_domain,dp));
    ValidatedVectorMultivariateFunction identity=ValidatedVectorMultivariateFunction::identity(domain.size());

    ValidatedConstrainedImageSet result(domain,identity);
    for(SizeType i=0; i!=set.constraints().size(); ++i) {
        RealExpression constraint_expression=indicator(set.constraints()[i],Sign::NEGATIVE);
        ValidatedScalarMultivariateFunction constraint_function( Ariadne::make_function(space,constraint_expression) );
        result.new_parameter_constraint(constraint_function <= ValidatedNumber(0) );
    }
    return result;
}


RealBox make_box(RealSpace const& spc, RealVariablesBox const& bx) {
    return bx.euclidean_set(spc);
}

RealBox make_set(RealSpace const& spc, RealVariablesBox const& bx) {
    return bx.euclidean_set(spc);
}

ConstraintSet make_set(RealSpace const& spc, RealExpressionConstraintSet const& set) {
    return set.euclidean_set(spc);
}

BoundedConstraintSet make_set(RealSpace const& spc, RealExpressionBoundedConstraintSet const& set) {
    return set.euclidean_set(spc);
}

BoundedConstraintSet make_set(RealSpace const& spc, RealVariablesBox const& bx, RealExpressionConstraintSet const& set) {
    return intersection(bx,set).euclidean_set(spc);
}



} // namespace Ariadne
