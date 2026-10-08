/***************************************************************************
 *            numeric/number_wrapper.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file numeric/number_wrapper.hpp
 *  \brief
 */



#ifndef ARIADNE_NUMBER_WRAPPER_HPP
#define ARIADNE_NUMBER_WRAPPER_HPP

#include <cassert>

#include "utility/module.hpp"
#include "utility/dispatching.hpp"
#include "foundations/paradigm.hpp"

#include "number_interface.hpp"

#include "number.hpp"

#include "foundations/logical.hpp"
#include "builtin.hpp"
#include "floatdp.hpp"
#include "floatmp.hpp"
#include "float_ball.hpp"
#include "float_bounds.hpp"
#include "float_upper_bound.hpp"
#include "float_lower_bound.hpp"
#include "float_approximation.hpp"
#include "float_error.hpp"

#include "symbolic/templates.hpp"
#include "numeric/operators.hpp"

namespace Ariadne {

/************ Number *********************************************************/

// Declare fallbacks for Integer, Dyadic and Rational numbers
Real sqrt(Real const&);
Real exp(Real const&);
Real log(Real const&);
Real sin(Real const&);
Real cos(Real const&);
Real tan(Real const&);
Real asin(Real const&);
Real acos(Real const&);
Real atan(Real const&);

// Declare fallbacks for use by Lower/UpperBound
Approximation<FloatDP> div(Approximation<FloatDP> const& x1, Approximation<FloatDP> const& x2);
Approximation<FloatDP> sqr(Approximation<FloatDP> const& x);
Approximation<FloatDP> rec(Approximation<FloatDP> const& x);
Approximation<FloatDP> sin(Approximation<FloatDP> const& x);
Approximation<FloatDP> cos(Approximation<FloatDP> const& x);
Approximation<FloatDP> tan(Approximation<FloatDP> const& x);
Approximation<FloatDP> asin(Approximation<FloatDP> const& x);
Approximation<FloatDP> acos(Approximation<FloatDP> const& x);

Approximation<FloatMP> div(Approximation<FloatMP> const& x1, Approximation<FloatMP> const& x2);
Approximation<FloatMP> sqr(Approximation<FloatMP> const& x);
Approximation<FloatMP> rec(Approximation<FloatMP> const& x);
Approximation<FloatMP> sin(Approximation<FloatMP> const& x);
Approximation<FloatMP> cos(Approximation<FloatMP> const& x);
Approximation<FloatMP> tan(Approximation<FloatMP> const& x);
Approximation<FloatMP> asin(Approximation<FloatMP> const& x);
Approximation<FloatMP> acos(Approximation<FloatMP> const& x);

// Declare fallbacks for use by Dyadic/RationalBounds
template<class B> concept AlgebraicBounds = SameAs<B,DyadicBounds> || SameAs<B,DecimalBounds> || SameAs<B,RationalBounds>;
template<AlgebraicBounds B> B sqrt(B const&) { std::abort(); }
template<AlgebraicBounds B> B exp(B const&) { std::abort(); }
template<AlgebraicBounds B> B log(B const&) { std::abort(); }
template<AlgebraicBounds B> B sin(B const&) { std::abort(); }
template<AlgebraicBounds B> B cos(B const&) { std::abort(); }
template<AlgebraicBounds B> B tan(B const&) { std::abort(); }
template<AlgebraicBounds B> B asin(B const&) { std::abort(); }
template<AlgebraicBounds B> B acos(B const&) { std::abort(); }
template<AlgebraicBounds B> B atan(B const&) { std::abort(); }
template<AlgebraicBounds B> ValidatedKleenean operator>(B const& y1, Int n2) { return y1 > B(n2); }

class NumberInterface;
template<class X> class NumberMixin;
template<class X> class NumberWrapper;

template<class I> struct InterfaceTraits;
template<> struct InterfaceTraits<NumberInterface> {
    template<class X> using MixinType = NumberMixin<X>;
    template<class X> using WrapperType = NumberWrapper<X>;
};

template<class X> inline X const* extract(NumberInterface const* y) {
     return dynamic_cast<NumberWrapper<X>const*>(y);
}

inline OutputStream& operator<<(OutputStream& os, NumberInterface const& y) { return y._write(os); }


// FIXME: Should test for other potential infinities
inline Comparison cmp(NumberInterface const& y1, NumberInterface const& y2) {
    Comparison res;
    FloatDP const* x1=extract<FloatDP>(&y1);
    FloatDP const* x2=extract<FloatDP>(&y2);
    if(x1) {
        if(x2) { res= cmp(ExactDouble(x1->get_d()),ExactDouble(x2->get_d())); }
        else { res= cmp(ExactDouble(x1->get_d()),y2._get_q()); }
    } else {
        if(x2) { res= cmp(y1._get_q(),ExactDouble(x2->get_d())); }
        else { res= cmp(y1._get_q(),y2._get_q()); }
    }
    return res;
}


/*
template<class OP, class X1, class X2> NumberInterface* _concrete_apply(OP op, X1 const& x1, X2 const& x2) {
    if constexpr (CanApply<OP,X1,X2>::value) {
        return _make_number_wrapper(op(x1,x2));
    } else {
        String yc1=class_name<X1>(); String yc2=class_name<X2>();
        ARIADNE_THROW(DispatchException,op<<"(Number y1, Number y2) with y1="<<x1<<", y2="<<x2,"No dispatch for "<<op<<"("<<yc1<<", "<<yc2<<")");
    }
}
*/


template<class R> NumberInterface* _make_number_wrapper(R const& r) { return new NumberWrapper<R>(r); }
struct MakeNumberWrapper {
    template<class R> NumberInterface* operator() (R const& r) const { return new NumberWrapper<R>(r); }
};

template<class X> inline X make_unsigned(X x) { return x; }
template<class X> inline X make_unsigned(Positive<X> px) {
  #ifndef __clang__
    return px;
  #else
    return std::move(px);
  #endif
}
inline Real make_unsigned(PositiveReal px) {
  #ifndef __clang__
    return px;
  #else
    return std::move(px);
  #endif
}
inline Integer make_unsigned(Natural px) {
  #ifndef __clang__
    return px;
  #else
    return std::move(px);
  #endif
}

template<class R, class X> inline R _concrete_apply(UnaryElementaryOperator op, X const& x) {
    static_assert(Same<R,NumberInterface*>);
    return op.accept([&x](auto _op){return _make_number_wrapper(make_unsigned(_op(x)));});
}

template<class R, class X1, class X2> inline R _concrete_apply(BinaryElementaryOperator op, X1 const& x1, X2 const& x2) {
    static_assert(Same<R,NumberInterface*>);
    return op.accept([&x1,x2](auto _op){return _make_number_wrapper(_op(x1,x2));});
}

template<class R, class X1, class X2> inline R _concrete_apply_max_or_min(BinaryElementaryOperator op, X1 const& x1, X2 const& x2) {
    if (op.code()==BinaryElementaryOperator(Max()).code()) { return _make_number_wrapper(max(x1,x2)); }
    else if (op.code()==BinaryElementaryOperator(Min()).code()) { return _make_number_wrapper(min(x1,x2)); }
    String yc1=class_name<X1>(); String yc2=class_name<X2>();
    ARIADNE_THROW(DispatchException,op<<"(Number y1, Number y2) with y1="<<x1<<", y2="<<x2,"No dispatch for "<<op<<"("<<yc1<<", "<<yc2<<")");
}

// FIXME: Prefer symbolic dispatch
template<class R, ARawFloat F> inline R _concrete_apply(BinaryElementaryOperator op, F const& x1, F const& x2) {
    return _concrete_apply_max_or_min<R>(op,x1,x2);
}
template<class R, ARawFloat F> inline R _concrete_apply(BinaryElementaryOperator op, F const& x1, Integer const& z2) {
    return _concrete_apply_max_or_min<R>(op,x1,z2);
}
template<class R, ARawFloat F> inline R _concrete_apply(BinaryElementaryOperator op, Integer const& z1, F const& x2) {
    return _concrete_apply_max_or_min<R>(op,z1,x2);
}
template<class R, ARawFloat F> inline R _concrete_apply(BinaryElementaryOperator op, F const& x1, Dyadic const& w2) {
    return _concrete_apply_max_or_min<R>(op,x1,w2);
}
template<class R, ARawFloat F> inline R _concrete_apply(BinaryElementaryOperator op, Dyadic const& w1, F const& x2) {
    return _concrete_apply_max_or_min<R>(op,w1,x2);
}
template<class R, ARawFloat F> inline R _concrete_apply(BinaryElementaryOperator op, F const& x1, Rational const& q2) {
    String yc1=class_name<F>(); String yc2=class_name<Rational>();
    ARIADNE_THROW(DispatchException,op<<"(Number y1, Number y2) with y1="<<x1<<", y2="<<q2,"No dispatch for "<<op<<"("<<yc1<<", "<<yc2<<")");
}
template<class R, ARawFloat F> inline R _concrete_apply(BinaryElementaryOperator op, Rational const& q1, F const& x2) {
    String yc1=class_name<Rational>(); String yc2=class_name<F>();
    ARIADNE_THROW(DispatchException,op<<"(Number y1, Number y2) with y1="<<q1<<", y2="<<x2,"No dispatch for "<<op<<"("<<yc1<<", "<<yc2<<")");
}



template<class R, class OP, class X1, class X2> inline R _concrete_operator_apply(OP op, X1 const& x1, X2 const& x2) {
    auto res=op(x1,x2);
        if constexpr (Same<decltype(res),Bool>) {
        return new_logical_pointer_from_value(LogicalValue(res));
    } else if constexpr (Same<decltype(res.repr()),LogicalValue const&>) {
        return new_logical_pointer_from_value(res.repr());
    } else {
        static_assert(Same<decltype(res.repr()),LogicalHandle const&>);
        return res.repr().pointer()->_copy();
    }
}

template<class R, class X1, class X2> inline R _concrete_apply(BinaryComparisonOperator op, X1 const& x1, X2 const& x2) {
    static_assert(Same<R,LogicalInterface*>);
    return op.accept( [&x1,&x2](auto _op){return _concrete_operator_apply<R>(_op,x1,x2);} );
}



class AlgebraicNumberInterface;
class ValidatedAlgebraicNumberInterface;
template<class F> class ConcreteNumberInterface;
template<class F, class FE> class ConcreteBallInterface;

template<> struct Managed<AlgebraicNumberInterface> {
    typedef Aware<Integer,Dyadic,Rational,Real> Types;
    //FIXME: typedef Aware<ExactDouble,Integer,Dyadic,Rational,Real> Types;
};
template<> struct Managed<ValidatedAlgebraicNumberInterface> {
    typedef Aware<DyadicBounds,RationalBounds> Types;
    //FIXME: typedef Aware<ExactDouble,Integer,Dyadic,Rational,Real> Types;
};
template<class F> struct Managed<ConcreteNumberInterface<F>> {
    typedef Aware<F,Ball<F>,Bounds<F>,UpperBound<F>,LowerBound<F>,Approximation<F>> Types;
};
template<class F, class FE> struct Managed<ConcreteBallInterface<F,FE>> {
    typedef Aware<Ball<F,FE>> Types;
};

template<> struct DispatcherTraits<ExactDouble> { typedef AlgebraicNumberInterface Interface; };
template<> struct DispatcherTraits<Integer> { typedef AlgebraicNumberInterface Interface; };
template<> struct DispatcherTraits<Dyadic> { typedef AlgebraicNumberInterface Interface; };
template<> struct DispatcherTraits<Rational> { typedef AlgebraicNumberInterface Interface; };
template<> struct DispatcherTraits<Real> { typedef AlgebraicNumberInterface Interface; };

template<> struct DispatcherTraits<DyadicBounds> { typedef ValidatedAlgebraicNumberInterface Interface; };
template<> struct DispatcherTraits<RationalBounds> { typedef ValidatedAlgebraicNumberInterface Interface; };

template<> struct DispatcherTraits<FloatDP> { typedef ConcreteNumberInterface<FloatDP> Interface; };
template<> struct DispatcherTraits<FloatMP> { typedef ConcreteNumberInterface<FloatMP> Interface; };

template<class F> struct DispatcherTraits<Ball<F>> { typedef ConcreteNumberInterface<F> Interface; };
template<class F> struct DispatcherTraits<Bounds<F>> { typedef ConcreteNumberInterface<F> Interface; };
template<class F> struct DispatcherTraits<UpperBound<F>> { typedef ConcreteNumberInterface<F> Interface; };
template<class F> struct DispatcherTraits<LowerBound<F>> { typedef ConcreteNumberInterface<F> Interface; };
template<class F> struct DispatcherTraits<Approximation<F>> { typedef ConcreteNumberInterface<F> Interface; };

template<class F, class FE> struct DispatcherTraits<Ball<F,FE>> { typedef ConcreteBallInterface<F,FE> Interface; };


class AlgebraicNumberInterface
    : public virtual SelfOperableInterface<NumberInterface*,BinaryElementaryOperator,ManagedTypes<AlgebraicNumberInterface>>
{
};

class ValidatedAlgebraicNumberInterface
    : public virtual SelfOperableInterface<NumberInterface*,BinaryElementaryOperator,ManagedTypes<ValidatedAlgebraicNumberInterface>>
{
};

template<class F> class ConcreteNumberInterface
    : public virtual SelfOperableInterface<NumberInterface*,BinaryElementaryOperator,ManagedTypes<ConcreteNumberInterface<F>>>
    , public virtual OperableInterface<NumberInterface*,BinaryElementaryOperator,ManagedTypes<AlgebraicNumberInterface>>
{
};

template<class F, class FE> class ConcreteBallInterface
    : public virtual SelfOperableInterface<NumberInterface*,BinaryElementaryOperator,ManagedTypes<ConcreteBallInterface<F,FE>>>
    , public virtual OperableInterface<NumberInterface*,BinaryElementaryOperator,ManagedTypes<AlgebraicNumberInterface>>
//    , public virtual OperableInterface<NumberInterface*,BinaryElementaryOperator,ManagedTypes<ConcreteNumberInterface<F>>>
{
};



template<class X, class DI=DispatcherInterface<X>> class ElementaryBinaryNumberDispatcherMixin;

template<class Y> class ElementaryBinaryNumberDispatcherMixin<Y,AlgebraicNumberInterface>
    : public SelfOperableMixin<Y,NumberInterface,NumberInterface*,BinaryElementaryOperator,ManagedTypes<AlgebraicNumberInterface>>
{
};

template<class Y> class ElementaryBinaryNumberDispatcherMixin<Y,ValidatedAlgebraicNumberInterface>
    : public SelfOperableMixin<Y,NumberInterface,NumberInterface*,BinaryElementaryOperator,ManagedTypes<ValidatedAlgebraicNumberInterface>>
{
};

template<class Y, class F> class ElementaryBinaryNumberDispatcherMixin<Y,ConcreteNumberInterface<F>>
    : public SelfOperableMixin<Y,NumberInterface,NumberInterface*,BinaryElementaryOperator,ManagedTypes<ConcreteNumberInterface<F>>>
    , public OperableMixin<Y,NumberInterface,NumberInterface*,BinaryElementaryOperator,ManagedTypes<AlgebraicNumberInterface>>
{
};

template<class Y, class F, class FE> class ElementaryBinaryNumberDispatcherMixin<Y,ConcreteBallInterface<F,FE>>
    : public SelfOperableMixin<Y,NumberInterface,NumberInterface*,BinaryElementaryOperator,ManagedTypes<ConcreteBallInterface<F,FE>>>
    , public OperableMixin<Y,NumberInterface,NumberInterface*,BinaryElementaryOperator,ManagedTypes<AlgebraicNumberInterface>>
//    , public OperableMixin<Y,NumberInterface,NumberInterface*,BinaryElementaryOperator,ManagedTypes<ConcreteNumberInterface<F>>>
{
};


template<class X, class DI=DispatcherInterface<X>> class ComparisonBinaryNumberDispatcherMixin;

template<class Y> class ComparisonBinaryNumberDispatcherMixin<Y,AlgebraicNumberInterface>
    : public SelfOperableMixin<Y,NumberInterface,LogicalInterface*,BinaryComparisonOperator,ManagedTypes<AlgebraicNumberInterface>>
{
};

template<class Y> class ComparisonBinaryNumberDispatcherMixin<Y,ValidatedAlgebraicNumberInterface>
    : public SelfOperableMixin<Y,NumberInterface,LogicalInterface*,BinaryComparisonOperator,ManagedTypes<ValidatedAlgebraicNumberInterface>>
{
};

template<class Y, class F> class ComparisonBinaryNumberDispatcherMixin<Y,ConcreteNumberInterface<F>>
    : public SelfOperableMixin<Y,NumberInterface,LogicalInterface*,BinaryComparisonOperator,ManagedTypes<ConcreteNumberInterface<F>>>
    , public OperableMixin<Y,NumberInterface,LogicalInterface*,BinaryComparisonOperator,ManagedTypes<AlgebraicNumberInterface>>
{
};

template<class Y, class F, class FE> class ComparisonBinaryNumberDispatcherMixin<Y,ConcreteBallInterface<F,FE>>
    : public SelfOperableMixin<Y,NumberInterface,LogicalInterface*,BinaryComparisonOperator,ManagedTypes<ConcreteBallInterface<F,FE>>>
    , public OperableMixin<Y,NumberInterface,LogicalInterface*,BinaryComparisonOperator,ManagedTypes<AlgebraicNumberInterface>>
{
};






inline LogicalInterface* make_symbolic(BinaryComparisonOperator op, NumberInterface const* yp1, NumberInterface const* yp2) {
    String yc1=yp1->_class_name(); String yc2=yp2->_class_name();
    ARIADNE_THROW(DispatchException,op<<"(Number y1, Number y2) with y1="<<*yp1<<", y2="<<*yp2,"No dispatch for "<<op<<"("<<yc1<<", "<<yc2<<")");
}

template<class OP> inline NumberInterface* make_symbolic(OP op, NumberInterface const* yp1, NumberInterface const* yp2) {
//    Handle<NumberInterface> y1(const_cast<NumberInterface*>(yp1)->shared_from_this());
//    Handle<NumberInterface> y2(const_cast<NumberInterface*>(yp2)->shared_from_this());
    String yc1=yp1->_class_name(); String yc2=yp2->_class_name();
    ARIADNE_THROW(DispatchException,op<<"(Number y1, Number y2) with y1="<<*yp1<<", y2="<<*yp2,"No dispatch for "<<op<<"("<<yc1<<", "<<yc2<<")");
}












template<class X> struct ElementaryUnaryNumberOperationsMixin
    : public UnaryOperationMixin<X,NumberInterface,UnaryElementaryOperator,NumberInterface> { };
template<class X> struct ElementaryBinaryNumberOperationsMixin
    : public BinaryOperationMixin<X,NumberInterface,BinaryElementaryOperator,NumberInterface> { };
template<class X,class N=Int> struct ElementaryGradedNumberOperationsMixin
    : public GradedOperationMixin<X,NumberInterface,GradedElementaryOperator,NumberInterface,N> { };

template<class X> struct ComparisonBinaryNumberOperationsMixin
    : public BinaryOperationMixin<X,LogicalInterface,BinaryComparisonOperator,NumberInterface> { };

inline OutputStream& operator<<(OutputStream& os, ParadigmCode cd) {
    switch (cd) {
        case ParadigmCode::EXACT: return os << "EXACT";
        case ParadigmCode::VALIDATED: return os << "VALIDATED";
        case ParadigmCode::APPROXIMATE: return os << "APPROXIMATE";
        default: return os << "UNKNOWN";
    }
}

template<class X> class NumberGetterMixin : public virtual NumberInterface {
  public:
  //  operator X const& () const { return static_cast<NumberMixin<X>const&>(*this); }
    static X const& _cast(NumberGetterMixin<X> const& self) { return static_cast<NumberWrapper<X>const&>(static_cast<NumberMixin<X>const&>(self)); }
    static X& _cast(NumberGetterMixin<X>& self) { return static_cast<NumberWrapper<X>&>(static_cast<NumberMixin<X>&>(self)); }

    typedef Paradigm<X> P;
    friend class Number<P>;

    virtual NumberInterface* _copy() const override { return new NumberWrapper<X>(_cast(*this)); }
    virtual NumberInterface* _move() override { return new NumberWrapper<X>(std::move(_cast(*this))); }

    virtual LogicalInterface* _is_pos() const override {
        auto res=(_cast(*this) > 0);
        if constexpr (Same<decltype(res),bool>) {
            return new_logical_pointer_from_value(LogicalValue(res));
        } else if constexpr (Same<decltype(res.repr()),LogicalValue const&>) {
            return new_logical_pointer_from_value(res.repr());
        } else {
            static_assert(Same<decltype(res.repr()),LogicalHandle const&>);
            return res.repr().pointer()->_copy();
        }
    }

    virtual Rational _get_q() const override {
        return this->_get_as<Rational>(); }

    virtual FloatDPBall _get(MetricTag,DoublePrecision pr,DoublePrecision pre) const override {
        return this->_get_as<FloatDPBall>(pr); }
    virtual FloatDPBounds _get(OrderTag,DoublePrecision pr) const override {
        return this->_get_as<FloatDPBounds>(pr); }
    virtual FloatDPUpperBound _get(UpperTag,DoublePrecision pr) const override {
        return this->_get_as<FloatDPUpperBound>(pr); }
    virtual FloatDPLowerBound _get(LowerTag,DoublePrecision pr) const override {
        return this->_get_as<FloatDPLowerBound>(pr); }
    virtual FloatDPApproximation _get(ApproximateTag,DoublePrecision pr) const override {
        return this->_get_as<FloatDPApproximation>(pr); }
    virtual FloatMPDPBall _get(MetricTag,MultiplePrecision pr, DoublePrecision pre) const override {
        return this->_get_as<FloatMPDPBall>(pr,pre); }
    virtual FloatMPBall _get(MetricTag, MultiplePrecision pr, MultiplePrecision pre) const override {
        return this->_get_as<FloatMPBall>(pr,pre); }
    virtual FloatMPBounds _get(OrderTag, MultiplePrecision pr) const override {
        return this->_get_as<FloatMPBounds>(pr); }
    virtual FloatMPUpperBound _get(UpperTag, MultiplePrecision pr) const override {
        return this->_get_as<FloatMPUpperBound>(pr); }
    virtual FloatMPLowerBound _get(LowerTag, MultiplePrecision pr) const override {
        return this->_get_as<FloatMPLowerBound>(pr); }
    virtual FloatMPApproximation _get(ApproximateTag, MultiplePrecision pr) const override {
        return this->_get_as<FloatMPApproximation>(pr); }

    virtual ParadigmCode _paradigm() const override { return P::code(); }
    virtual String _class_name() const override { return class_name<X>(); }
    virtual OutputStream& _write(OutputStream& os) const override { return os << _cast(*this); }

  private:
    template<class R> inline R _get_as() const {
        if constexpr (Constructible<R,X>) { return static_cast<R>(_cast(*this)); }
        else { std::cerr<<"Warning: Cannot convert " << _cast(*this) << " of type " << this->_class_name() << " to " << class_name<R>() << "\n"; throw ParadigmError(); } }
    template<class R, class PR> inline R _get_as(PR pr) const {
        if constexpr (Constructible<R,X,PR>) { return R(_cast(*this),pr); }
        else { std::cerr<<"Warning: Cannot convert " << _cast(*this) << " of type " << this->_class_name() << " to " << class_name<R>() << " with precision " << pr << "\n"; throw ParadigmError(); } }
    template<class R, class PR, class PRE> inline R _get_as(PR pr, PRE pre) const {
        if constexpr (Constructible<R,X,PR,PRE>) { return R(_cast(*this),pr,pre); }
        else { std::cerr<<"Warning: Cannot convert " << _cast(*this) << " of type " << this->_class_name() << " to " << class_name<R>() << " with precision " << pr << " and error precision " << pre << "\n"; throw ParadigmError(); } }
};

template<class X> class NumberMixin
    : public ElementaryBinaryNumberOperationsMixin<X>
    , public ElementaryUnaryNumberOperationsMixin<X>
    , public ElementaryGradedNumberOperationsMixin<X>
    , public ElementaryBinaryNumberDispatcherMixin<X>
    , public ComparisonBinaryNumberOperationsMixin<X>
    , public ComparisonBinaryNumberDispatcherMixin<X>
    , public NumberGetterMixin<X>
{
  public:
    operator X const& () const { return static_cast<NumberWrapper<X>const&>(*this); }
    operator X& () { return static_cast<NumberWrapper<X>&>(*this); }
};


template<class X> class NumberWrapper
    : public X, public NumberMixin<X>
{
    inline static const X& _cast(const NumberWrapper<X>& x) { return static_cast<const X&>(x); }
    static_assert(not Same<X,Handle<NumberInterface>>,"X must be a concrete number, not a handle");
    static_assert(not Same<X,Number<Paradigm<X>>>,"X must be a concrete number, not a generic number");
  public:
    NumberWrapper(const X& a) : X(a) { }
    NumberWrapper(X&& a) : X(std::forward<X>(a)) { }
};


} // namespace Ariadne

#endif /* ARIADNE_NUMBER_WRAPPER_HPP */
