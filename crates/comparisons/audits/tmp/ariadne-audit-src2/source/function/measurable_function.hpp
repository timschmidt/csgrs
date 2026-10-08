/***************************************************************************
 *            measurable_function.hpp
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

/*! \file measurable_function.hpp
 *  \brief Built-in and user functions and expressions
 */

#ifndef ARIADNE_MEASURABLE_FUNCTION_HPP
#define ARIADNE_MEASURABLE_FUNCTION_HPP

#include "numeric/number.hpp"

#include "domain.hpp"
#include "function_model.hpp"

#include "geometry/set.hpp"
#include "geometry/measurable_set.hpp"
#include "geometry/set_wrapper.hpp"

namespace Ariadne {

template<class P, class SIG> class MeasurableFunction;
template<class SIG> using EffectiveMeasurableFunction = MeasurableFunction<EffectiveTag,SIG>;
template<class SIG> using ValidatedMeasurableFunction = MeasurableFunction<ValidatedTag,SIG>;
using EffectiveScalarUnivariateMeasurableFunction = MeasurableFunction<EffectiveTag,Real(Real)>;
using ValidatedScalarUnivariateMeasurableFunction = MeasurableFunction<ValidatedTag,Real(Real)>;

template<class P,class SIG> using ContinuousFunction = Function<P,SIG>;
template<class SIG> using EffectiveContinuousFunction = ContinuousFunction<EffectiveTag,SIG>;
template<class SIG> using ValidatedContinuousFunction = ContinuousFunction<ValidatedTag,SIG>;
using EffectiveScalarUnivariateContinuousFunction = ContinuousFunction<EffectiveTag,Real(Real)>;
using ValidatedScalarUnivariateContinuousFunction = ContinuousFunction<ValidatedTag,Real(Real)>;


template<class P, class SIG> class MeasurableFunctionInterface;

template<class P, class RES, class ARG> class MeasurableFunctionInterface<P,RES(ARG)> {
    using SIG=RES(ARG);
  public:
    virtual ~MeasurableFunctionInterface() = default;
    virtual LowerMeasurableSet<P,ARG> _preimage(OpenSet<P,RES>) const = 0;
};

template<class P, class SIG> class MeasurableFunction;

template<class F, class P, class SIG> struct IsMeasurableFunction;

template<class F, class P, class R, class A> struct IsMeasurableFunction<F,P,R(A)> {
    using LMS=LowerMeasurableSet<P,A>; using OPS=OpenSet<P,R>;
    template<class FF,class=decltype(declval<LMS>()=preimage(declval<OPS>(),declval<FF>()))>
        static std::true_type test(int);
    template<class FF>
        static std::false_type test(...);
    static const bool value = decltype(test<F>(1))::value;
};

template<class F, class P, class SIG> concept AMeasurableFunction = IsMeasurableFunction<F,P,SIG>::value;

template<class P, class RES, class ARG> class MeasurableFunction<P,RES(ARG)>
    : public Handle<const MeasurableFunctionInterface<P,RES(ARG)>>
{
    using SIG=RES(ARG);
    using typename Handle<const MeasurableFunctionInterface<P,SIG>>::Interface;
  public:
    using Handle<const Interface>::Handle;
    SizeType ref_count() const { return this->_ptr.ref_count(); }
    template<AMeasurableFunction<P,SIG> F> MeasurableFunction(F const&);
    LowerMeasurableSet<P,ARG> preimage(OpenSet<P,RES> const& u) const { return this->_ptr->_preimage(u); }
};

template<class F, class P, class SIG> class MeasurableFunctionWrapper;

template<class F, class P, class RES, class ARG> class MeasurableFunctionWrapper<F,P,RES(ARG)>
    : public MeasurableFunctionInterface<P,RES(ARG)>, public F
{
    using SIG=RES(ARG);
  public:
    virtual LowerMeasurableSet<P,ARG> _preimage(OpenSet<P,RES> const& ops) const final override {
        return preimage(ops,static_cast<F const&>(*this)); }
};


template<class SIG> class FastFanSequence;

template<class RES, class ARG> class FastFanSequence<RES(ARG)>
    : public Sequence<ValidatedContinuousFunction<RES(ARG)>>
{
    using P=EffectiveTag;
    using SIG=RES(ARG);
  public:
    using Sequence<ValidatedContinuousFunction<SIG>>::Sequence;
    LowerMeasurableSet<P,ARG> preimage(OpenSet<P,RES> const& ops) const;
    friend LowerMeasurableSet<P,ARG> preimage(OpenSet<P,RES> const& ops, FastFanSequence<RES(ARG)> const& ffs) {
        return ffs.preimage(ops); }
};

static_assert(AMeasurableFunction<FastFanSequence<Real(Real)>,EffectiveTag,Real(Real)>);


template<class SIG, class PR, class PRE=PR> class FanModel;

template<class PR, class PRE> class FanModel<Real(Real),PR,PRE> {
    using P=ValidatedTag; using RES=Real; using ARG=Real; using SIG=RES(ARG);
  public:
    typedef IntervalDomainType DomainType;
    typedef FloatError<PRE> ErrorType;

    DomainType _domain;
    ValidatedContinuousFunction<SIG> _function;
    FloatError<PRE> _measure_error;
  public:
    FanModel(DomainType dom, ValidatedContinuousFunction<SIG> fn, FloatError<PRE> err)
        : _domain(dom), _function(fn), _measure_error(err) { }
    DomainType const& domain() const { return this->_domain; }
    ValidatedContinuousFunction<SIG> const& function() const { return this->_function; }
    ErrorType const& measure_error() const { return this->_measure_error; }
    ValidatedLowerMeasurableSet<ARG> preimage(ValidatedOpenSet<RES> const&) const;
    friend LowerMeasurableSet<P,ARG> preimage(OpenSet<P,RES> const& ops, FanModel<Real(Real),PR,PRE> const& fm) {
        return fm.preimage(ops); }
    friend OutputStream& operator<<(OutputStream& os, FanModel<SIG,PR,PRE> const& fm) {
        return os << "FanModel(domain="<<fm._domain<<",function="<<fm._function<<",measure_error="<<fm._measure_error<<")"; }

};

static_assert(AMeasurableFunction<FanModel<Real(Real),DoublePrecision,DoublePrecision>,ValidatedTag,Real(Real)>);

// TODO: Move to Interval
template<class PR> Interval<UpperBound<PR>> image(Interval<UpperBound<PR>> const& ivl, ValidatedContinuousFunction<Real(Real)> const& f) {
    return f(cast_singleton(ivl));
}

ValidatedOpenSet<Real> preimage(ValidatedContinuousFunction<Real(Real)> const& f, ValidatedOpenSet<Real> const& ops, IntervalDomainType dom, Accuracy acc);

template<class PR, class PRE> auto
FanModel<Real(Real),PR,PRE>::preimage(ValidatedOpenSet<Real> const& rng) const -> ValidatedLowerMeasurableSet<Real> {
    // FIXME: Make accuracy dynamic
    Accuracy acc(exp2(-4));

    ValidatedOpenSet<Real> const prim=Ariadne::preimage(this->_function,rng,this->_domain,acc);
    auto ivls_ptr=dynamic_pointer_cast<const OpenSetWrapper<UnionOfIntervals<FloatDP>,ValidatedTag,Real>>(prim.managed_pointer());
    assert(ivls_ptr);
    UnionOfIntervals<FloatDP> const& ivls=*ivls_ptr;
    return LowerMeasurableSetModel(ivls,this->_measure_error);
}





template<class PR> UnionOfIntervals<Float<PR>> preimage_intervals(ValidatedContinuousFunction<Real(Real)> const& f, ValidatedRegularSet<Real> rgs, Interval<Float<PR>> ivl, Accuracy acc) {
    Interval<Float<PR>> rng=cast_exact(image(ivl,f));
    if (definitely(rgs.covers(rng))) {
        return {cast_exact_interval(ivl)};
    } else if (definitely(rgs.separated(rng))) {
        return {{},ivl.precision()};
    } else if (definitely(ivl.width()<acc.error())) {
        return {{},ivl.precision()};
    } else {
        Pair<Interval<Float<PR>>,Interval<Float<PR>>> subivls=split(ivl);
        return join(preimage_intervals(f,rgs,subivls.first,acc),preimage_intervals(f,rgs,subivls.second,acc));
    }
}

template<class PR> UnionOfIntervals<Float<PR>> preimage_intervals(ValidatedContinuousFunction<Real(Real)> const& f, ValidatedOpenSet<Real> ops, Interval<Float<PR>> ivl, Accuracy acc) {
    if (definitely(ops.covers(cast_exact(image(ivl,f))))) {
        return {ivl};
    } else if (definitely(ivl.width()<acc.error())) {
        return {{},ivl.precision()};
    } else {
        Pair<Interval<Float<PR>>,Interval<Float<PR>>> subivls=split(ivl);
        return join(preimage_intervals(f,ops,subivls.first,acc),preimage_intervals(f,ops,subivls.second,acc));
    }
}

ValidatedOpenSet<Real> preimage(ValidatedContinuousFunction<Real(Real)> const& f, ValidatedOpenSet<Real> const& ops, IntervalDomainType dom, Accuracy acc) {
    DoublePrecision pr;
    Interval<FloatDP> ivl=cast_exact(Interval<UpperBound<FloatDP>>(dom,pr));
    auto rgsp=dynamic_pointer_cast<ValidatedRegularSet<Real>::Interface>(ops.managed_pointer());
    if (false and rgsp) {
        auto rgs=ValidatedRegularSet<Real>(rgsp);
        return ValidatedOpenSet<Real>(wrap_open(preimage_intervals(f,rgs,ivl,acc)));
    } else {
        return ValidatedOpenSet<Real>(wrap_open(preimage_intervals(f,ops,ivl,acc)));
    }
}



} // namespace Ariadne

#endif
