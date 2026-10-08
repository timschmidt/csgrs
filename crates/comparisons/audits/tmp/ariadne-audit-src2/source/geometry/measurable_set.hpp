/***************************************************************************
 *            measurable_set.hpp
 *
 *  Copyright  2020  Pieter Collins
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

/*! \file measurable_set.hpp
 *  \brief Lower-measurable sets.
 */

#ifndef ARIADNE_MEASURABLE_SET_HPP
#define ARIADNE_MEASURABLE_SET_HPP

#include "utility/handle.hpp"
#include "numeric/number.hpp"

#include "numeric/sequence.hpp"
#include "numeric/integer.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/twoexp.hpp"
#include "interval.hpp"

namespace Ariadne {

TwoExp exp2(Integer z) {
    Int n=z.get_si();
    ARIADNE_ASSERT(n==z);
    return exp2(n);
}

template<class S> class LowerSequence : public ConvergentSequence<S> {
  public:
    explicit LowerSequence(Sequence<S> const& seq) : ConvergentSequence<S>(seq) { }
    explicit LowerSequence(std::function<S(Natural)> fn) : ConvergentSequence<S>(fn) { }
    decltype(auto) measures() const {
        using X=typename S::MeasureType;
        return IncreasingSequence<X>(Sequence<X>([this](Natural n){return cast_positive(max(0u,(*this)[n].measure()-exp2(-n)));})); }
};

using Void = void;

template<class P, class T> class OpenSet;

template<class P, class T> class LowerMeasurableSet;
template<class T> using EffectiveLowerMeasurableSet = LowerMeasurableSet<EffectiveTag,T>;
template<class T> using ValidatedLowerMeasurableSet = LowerMeasurableSet<ValidatedTag,T>;

template<class S, class P, class T> struct IsLowerMeasurableSet {
    using LY=PositiveLowerNumber<P>; using BS=typename SetTraits<T>::BasicSetType;
    template<class SS,class=decltype(measure(intersection(declval<SS>(),declval<BS>())))>
        static std::true_type test(int);
    template<class SS>
        static std::false_type test(...);
    static const bool value = decltype(test<S>(1))::value;
};

template<class S, class P, class T> concept ALowerMeasurableSet = IsLowerMeasurableSet<S,P,T>::value;

template<class P, class T> class LowerMeasurableSetInterface {
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    virtual ~LowerMeasurableSetInterface() = default;
    virtual PositiveLowerNumber<P> _measure_in(BasicSetType const&) const = 0;
    virtual PositiveLowerNumber<P> _measure() const = 0;
//    virtual LowerMeasurableSetInterface<P,T>* _intersection(OpenSet<P,T> const&) const = 0;
    virtual OutputStream& _write(OutputStream& os) const = 0;
};

template<class P, class T> class LowerMeasurableSet : public Handle<const LowerMeasurableSetInterface<P,T>> {
  public:
    using typename Handle<const LowerMeasurableSetInterface<P,T>>::Interface;
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    using Handle<Interface>::Handle;
    template<ALowerMeasurableSet<P,T> S> LowerMeasurableSet(S const& s);
    PositiveLowerNumber<P> measure_in(BasicSetType const& bs) const { return this->reference()._measure_in(bs); }
    PositiveLowerNumber<P> measure() const { return this->reference()._measure(); }
    friend LowerMeasurableSet<P,T> intersection(LowerMeasurableSet<P,T>, OpenSet<P,T>);
    friend OutputStream& operator<<(OutputStream& os, LowerMeasurableSet<P,T> const& lms) { return lms.reference()._write(os); }
};


template<class S, class P, class T> class LowerMeasurableSetWrapper;

template<class S, class T> class LowerMeasurableSetWrapper<S,ValidatedTag,T>
    : public virtual LowerMeasurableSet<ValidatedTag,T>::Interface
{
    using P=ValidatedTag;
    S const& base() const { return this->_s; }
  private:
    S _s;
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    LowerMeasurableSetWrapper(S const& set) : _s(set) { }
    virtual PositiveLowerNumber<P> _measure_in(BasicSetType const& bs) const final override {
        return PositiveLowerNumber<P>(ValidatedLowerNumber(measure(intersection(this->base(),bs)))); }
    PositiveLowerNumber<P> _measure() const final override {
        return PositiveLowerNumber<P>(measure(this->base()));
    }
    OutputStream& _write(OutputStream& os) const final override {
        return os << this->base(); }
};

template<class S> decltype(auto) wrap_lower_measurable(S const& s) {
    using P=ValidatedTag; using T=Real;
    return LowerMeasurableSet<P,T>(std::make_shared<LowerMeasurableSetWrapper<S,P,T>>(s));
}

template<class P, class T> template<ALowerMeasurableSet<P,T> S>
LowerMeasurableSet<P,T>::LowerMeasurableSet(S const& s)
    : LowerMeasurableSet(std::make_shared<LowerMeasurableSetWrapper<S,P,T>>(s)) {
}






template<class S, class E> class LowerMeasurableSetModel {
    S _set; E _error;
  public:
    typedef typename S::BasicSetType BasicSetType;
    typedef decltype(cast_positive(measure(declval<S>())-declval<E>())) MeasureType;

    explicit LowerMeasurableSetModel(S s, E e) : _set(s), _error(min(max(e,nul(e)),measure(s))) { }
    template<class... PRES> requires Constructible<E,Nat,PRES...>
        explicit LowerMeasurableSetModel(S s, PRES... pres) : _set(s), _error(0u,pres...) { }
    friend MeasureType measure(LowerMeasurableSetModel<S,E> const& lms) {
        return lms._measure(); }
    friend LowerMeasurableSetModel<S,E> intersection(LowerMeasurableSetModel<S,E> const& lms, BasicSetType const& bs) {
        return LowerMeasurableSetModel(intersection(lms._set,bs),lms._error); }
    friend LowerMeasurableSetModel<S,E> intersection(LowerMeasurableSetModel<S,E> const& lms1, LowerMeasurableSetModel<S,E> const& lms2) {
        return LowerMeasurableSetModel(intersection(lms1._set,lms2._set),min(lms1._error,lms2._error)); }
    friend LowerMeasurableSetModel<S,E> join(LowerMeasurableSetModel<S,E> const& lms1, LowerMeasurableSetModel<S,E> const& lms2) {
        return LowerMeasurableSetModel(join(lms1._set,lms2._set),lms1._error+lms2._error); }
    friend OutputStream& operator<<(OutputStream& os, LowerMeasurableSetModel<S,E> const& lms) {
        return os << lms._set << "+/-" << lms._error; }
  private:
    MeasureType _measure() const {
        //return MeasureType(cast_positive(max(PositiveFloatDPBounds(FloatDPBounds(2,2,dp))-this->_error,0u))); }
        return cast_positive(max(measure(this->_set)-this->_error,0u)); }
};




} // namespace Ariadne

#include "union_of_intervals.hpp"

namespace Ariadne {
template class LowerMeasurableSetModel<UnionOfIntervals<Dyadic>,Dyadic>;
template class LowerMeasurableSetModel<UnionOfIntervals<FloatDP>,Error<FloatDP>>;
} // namespace Ariadne

#endif
