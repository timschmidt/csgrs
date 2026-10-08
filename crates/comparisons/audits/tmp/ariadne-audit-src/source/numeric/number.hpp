/***************************************************************************
 *            numeric/number.hpp
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

/*! \file numeric/number.hpp
 *  \brief Generic numbers
 */



#ifndef ARIADNE_NUMBER_HPP
#define ARIADNE_NUMBER_HPP

#include "utility/handle.hpp"
#include "foundations/paradigm.hpp"
#include "utility/prototype.hpp"

#include "foundations/logical.decl.hpp"
#include "number.decl.hpp"
#include "float.decl.hpp"

#include "number_interface.hpp"

#include "arithmetic.hpp"
#include "integer.hpp"
#include "dyadic.hpp"
#include "rational.hpp"
#include "real.hpp"

#include "number_interface.hpp"

namespace Ariadne {

/************ Number *********************************************************/

template<class X> struct IsNumber;

class NumberInterface;

template<class P> class Number;
template<class P> struct IsNumber<Number<P>> : True { };

struct DispatchException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

template<class PR> struct IsPrecisionTrait : False { };
template<> struct IsPrecisionTrait<DoublePrecision> : True { };
template<> struct IsPrecisionTrait<MultiplePrecision> : True { };

template<class PR> concept IsPrecision = IsPrecisionTrait<PR>::value;

template<class P> Positive<Number<P>> cast_positive(Number<P> y);


class DeclareNumberOperators {
    friend ApproximateNumber operator+(ApproximateNumber const&, ApproximateNumber const&);
    friend ApproximateNumber operator-(ApproximateNumber const&, ApproximateNumber const&);
    friend ApproximateNumber operator*(ApproximateNumber const&, ApproximateNumber const&);
    friend ApproximateNumber operator/(ApproximateNumber const&, ApproximateNumber const&);

    friend ValidatedLowerNumber operator+(ValidatedLowerNumber const& y1, ValidatedLowerNumber const& y2);
    friend ValidatedLowerNumber operator-(ValidatedLowerNumber const& y1, ValidatedUpperNumber const& y2);
    friend ValidatedUpperNumber operator+(ValidatedUpperNumber const& y1, ValidatedUpperNumber const& y2);
    friend ValidatedUpperNumber operator-(ValidatedUpperNumber const& y1, ValidatedLowerNumber const& y2);

    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator+(N const& y1, D const& d2) { return y1+Number<ApproximateTag>(d2); }
    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator-(N const& y1, D const& d2) { return y1-Number<ApproximateTag>(d2); }
    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator*(N const& y1, D const& d2) { return y1*Number<ApproximateTag>(d2); }
    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator/(N const& y1, D const& d2) { return y1/Number<ApproximateTag>(d2); }
    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator+(D const& d1, N const& y2) { return Number<ApproximateTag>(d1)+y2; }
    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator-(D const& d1, N const& y2) { return Number<ApproximateTag>(d1)-y2; }
    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator*(D const& d1, N const& y2) { return Number<ApproximateTag>(d1)*y2; }
    template<GenericNumber N, BuiltinFloatingPoint D> friend decltype(auto)
    operator/(D const& d1, N const& y2) { return Number<ApproximateTag>(d1)/y2; }

    template<ConcreteNumber R, class P> friend decltype(auto)
    operator+(R const& r1, Number<P> const& y2) { return Number<Paradigm<R>>(r1)+y2; }
    template<ConcreteNumber R, class P> friend decltype(auto)
    operator+(Number<P> const& y1, R const& r2) { return y1+Number<Paradigm<R>>(r2); }
};


template<class X, class P> concept ConvertibleBuiltinFloatingPointToNumber
    = Same<P,ApproximateTag> and BuiltinFloatingPoint<X>;
template<class X, class P> concept ConvertibleViaRealToNumber
    = WeakerThan<P,ParadigmTag<X>> and Convertible<X,Real>;
template<class X, class P> concept ConvertibleViaNumberToNumber
    = WeakerThan<P,ParadigmTag<X>> and (not Convertible<X,Real>) and Convertible<X,Number<ParadigmTag<X>>>;

//! \ingroup NumericModule
//! \brief Generic numbers with computational paradigm \a P,  which may be %EffectiveTag, %ValidatedTag, %UpperTag, %LowerTag or %ApproximateTag.
template<class P> class Number
    : public Handle<const NumberInterface>
    , public DeclareNumberOperators
{
    static_assert(IsParadigm<P>,"P must be a paradigm");
    static_assert(Same<P,ExactTag> or Same<P,EffectiveTag> or Same<P,ValidatedTag> or Same<P,ApproximateTag>);

    template<class PP> friend class Number;
    template<class PP> friend class LowerNumber;
    template<class PP> friend class UpperNumber;

    template<class PR> using ResultFloatType = FloatType<Weaker<P,ValidatedTag>,PR>;

    template<class X> static const bool IsGettableAs = ANumber<X> and WeakerThan<typename X::Paradigm,P> and (not Same<typename X::Paradigm,ExactTag>);
  public:
    typedef NumberInterface Interface;
    typedef P Paradigm;
    typedef Number<P> NumericType;
  public:
    using Handle<const Interface>::Handle;
    explicit Number(NumberInterface* p) : Handle<const NumberInterface>(p) { }
  private: public:
    Handle<const NumberInterface> handle() const { return *this; }
    explicit Number(Handle<const NumberInterface> h) : Handle<const NumberInterface>(h) { }
  public:
    Number() : Number(Integer(0)) { }

    // Construct from a Number of a stronger paradigm
    template<StrongerThan<P> SP> Number(const Number<SP>& y) : Number(y.handle()) { }

    //! Construct from a builtin integer
    template<BuiltinIntegral N> Number(const N& n) : Number(Integer(n)) { }
    // Construct from a builtin floating-point number
    template<ConvertibleBuiltinFloatingPointToNumber<P> X> Number(const X& x) : Number(Dyadic(x)) { }

    // Construct from a type which is convertible to Real.
    template<ConvertibleViaRealToNumber<P> X> Number(X const & x) : Number(x.operator Number<ParadigmTag<X>>()) { }

    // Construct from a type which is convertible to another Number type.
    // TODO: Decide conversion characteristics from concrete type to Number<P>
    template<ConvertibleViaNumberToNumber<P> X>
        explicit Number(X const & x) : Number(x.operator Number<ParadigmTag<X>>()) { }

    //! \brief Get the value of the number as a double-precision floating-point type
    ResultFloatType<DoublePrecision> get(DoublePrecision const& prec) const { return this->ref()._get(P(),prec); }
    //! \brief Get the value of the number as a multiple-precision floating-point type
    ResultFloatType<MultiplePrecision> get(MultiplePrecision const& prec) const { return this->ref()._get(P(),prec); }

    //! \brief Get the value of the number as a floating-point ball with the given precision and error precision.
    template<class PR, class PRE> requires IsPrecision<PR> and IsPrecision<PRE> and WeakerThan<ValidatedTag,P>
    FloatBall<PR,PRE> get(PR const& prec, PRE const& errprec) const { return this->ref()._get(P(),prec,errprec); }

    //! \brief Try to dynamic_cast the object to concrete type \a X.
    template<class X> X extract() const;

    friend Number<P> operator+(Number<P> const& y) { return pos(y); } //!< <p/>
    friend Number<P> operator-(Number<P> const& y) { return neg(y); } //!< <p/>
    friend Number<P> operator+(Number<P> const& y1, Number<P> const& y2) { return add(y1,y2); } //!< <p/>
    friend Number<P> operator-(Number<P> const& y1, Number<P> const& y2) { return sub(y1,y2); } //!< <p/>
    friend Number<P> operator*(Number<P> const& y1, Number<P> const& y2) { return mul(y1,y2); } //!< <p/>
    friend Number<P> operator/(Number<P> const& y1, Number<P> const& y2) { return div(y1,y2); } //!< <p/>
    friend Number<P>& operator+=(Number<P>& y1, Number<P> const& y2) { return y1=y1+y2; } //!< <p/>
    friend Number<P>& operator-=(Number<P>& y1, Number<P> const& y2) { return y1=y1-y2; } //!< <p/>
    friend Number<P>& operator*=(Number<P>& y1, Number<P> const& y2) { return y1=y1*y2; } //!< <p/>
    friend Number<P>& operator/=(Number<P>& y1, Number<P> const& y2) { return y1=y1/y2; } //!< <p/>

    friend Number<P> nul(Number<P> const& y) { return _apply<P>(Nul(),y); } //!< <p/>
    friend Number<P> pos(Number<P> const& y) { return _apply<P>(Pos(),y); } //!< <p/>
    friend Number<P> neg(Number<P> const& y) { return _apply<P>(Neg(),y); } //!< <p/>
    friend Number<P> sqr(Number<P> const& y) { return _apply<P>(Sqr(),y); } //!< <p/>
    friend Number<P> hlf(Number<P> const& y) { return _apply<P>(Hlf(),y); } //!< <p/>
    friend Number<P> rec(Number<P> const& y) { return _apply<P>(Rec(),y); } //!< <p/>
    friend Number<P> add(Number<P> const& y1, Number<P> const& y2) { return _apply<P>(Add(),y1,y2); } //!< <p/>
    friend Number<P> sub(Number<P> const& y1, Number<P> const& y2) { return _apply<P>(Sub(),y1,y2); } //!< <p/>
    friend Number<P> mul(Number<P> const& y1, Number<P> const& y2) { return _apply<P>(Mul(),y1,y2); } //!< <p/>
    friend Number<P> div(Number<P> const& y1, Number<P> const& y2) { return _apply<P>(Div(),y1,y2); } //!< <p/>
    friend Number<P> sqrt(Number<P> const& y) { return _apply<P>(Sqrt(),y); } //!< <p/>
    friend Number<P> exp(Number<P> const& y) { return _apply<P>(Exp(),y); } //!< <p/>
    friend Number<P> log(Number<P> const& y) { return _apply<P>(Log(),y); } //!< <p/>
    friend Number<P> sin(Number<P> const& y) { return _apply<P>(Sin(),y); } //!< <p/>
    friend Number<P> cos(Number<P> const& y) { return _apply<P>(Cos(),y); } //!< <p/>
    friend Number<P> tan(Number<P> const& y) { return _apply<P>(Tan(),y); } //!< <p/>
    friend Number<P> asin(Number<P> const& y) { return _apply<P>(Asin(),y); } //!< <p/>
    friend Number<P> acos(Number<P> const& y) { return _apply<P>(Acos(),y); } //!< <p/>
    friend Number<P> atan(Number<P> const& y) { return _apply<P>(Atan(),y); } //!< <p/>

    friend Number<P> pow(Number<P> const& y, Nat m) { return _apply<P>(Pow(),y,m); } //!< <p/>
    friend Number<P> pow(Number<P> const& y, Int n) { return _apply<P>(Pow(),y,n); } //!< <p/>

    friend Positive<Number<P>> abs(Number<P> const& y) { return Positive<Number<P>>(_apply<P>(Abs(),y)); } //!< <p/>
    friend Number<P> max(Number<P> const& y1, Number<P> const& y2) { return _apply<P>(Max(),y1,y2); } //!< <p/>
    friend Number<P> min(Number<P> const& y1, Number<P> const& y2) { return _apply<P>(Min(),y1,y2); } //!< <p/>


    friend LogicalType<EqualityInformation<P>> operator==(Number<P> const& y1, Number<P> const& y2) {
        return Detail::logical_type_from_pointer<EqualityInformation<P>>(y1.ref()._apply(BinaryComparisonOperator(Equal()),y2.ptr())); }
    friend LogicalType<LessThanInformation<P>> operator< (Number<P> const& y1, Number<P> const& y2) {
        return Detail::logical_type_from_pointer<LessThanInformation<P>>(y1.ref()._apply(BinaryComparisonOperator(Less()),y2.ptr())); }
    friend LogicalType<LessThanInformation<P>> operator> (Number<P> const& y1, Number<P> const& y2) { return (y2<y1); } //!< <p/>
    friend LogicalNegationType<LogicalType<EqualityInformation<P>>> operator!=(Number<P> const& y1, Number<P> const& y2) { return !(y1==y2); } //!< <p/>
    friend LogicalType<LessThanInformation<P>> operator<=(Number<P> const& y1, Number<P> const& y2) { return !(y1>y2); } //!< <p/>
    friend LogicalType<LessThanInformation<P>> operator>=(Number<P> const& y1, Number<P> const& y2) { return !(y1<y2); } //!< <p/>

    //! \brief The name of the dynamic type held by the %Number object.
    String class_name() const { return this->ref()._class_name(); }

    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, Number<P> const& y) { return y.ref()._write(os); }
  private:
    template<class RP, class OP> static Number<RP> _apply(OP op, Number<P> const& y) { return Number<RP>(y.ref()._apply(op)); }
    template<class RP, class OP> static Number<RP> _apply(OP op, Number<P> const& y, Int n) { return Number<RP>(y.ref()._apply(op,n)); }
    template<class RP, class OP, class P1, class P2> static Number<RP> _apply(OP op, Number<P1> const& y1, Number<P2> const& y2) {
        return Number<RP>(y1.ref()._apply(op,&y2.ref())); }
};


template<class P> class Positive<Number<P>> : public Number<P> {
    friend Number<P> const& unsign(Positive<Number<P>> const& y) { return y; }
  public:
    Positive() : Number<P>() { }
    explicit Positive(Number<P> const& y) : Number<P>(y) { }
    template<BuiltinUnsignedIntegral N> Positive(N n) : Number<P>(n) { }
    template<class N> requires Convertible<N,ExactNumber>
        Positive(const Positive<N>& n) : Number<P>(ExactNumber(static_cast<N const&>(n))) { }
    template<class N> requires Constructible<ExactNumber,N> and (not BuiltinIntegral<N>)
        explicit Positive(const N& n) : Number<P>(ExactNumber(n)) { }
    explicit operator Number<P> () const { return *this; }

    friend Positive<Number<P>> operator+(Positive<Number<P>> const& y1, Positive<Number<P>> const& y2) {
        return cast_positive(add(unsign(y1),unsign(y2))); }
    friend Positive<Number<P>> operator*(Positive<Number<P>> const& y1, Positive<Number<P>> const& y2) {
        return cast_positive(mul(unsign(y1),unsign(y2))); }
    friend Positive<Number<P>> add(Positive<Number<P>> const& y1, Positive<Number<P>> const& y2) {
        return cast_positive(add(unsign(y1),unsign(y2))); }
    friend Positive<Number<P>> mul(Positive<Number<P>> const& y1, Positive<Number<P>> const& y2) {
        return cast_positive(mul(unsign(y1),unsign(y2))); }
    friend Positive<Number<P>> max(Positive<Number<P>> const& y1, Positive<Number<P>> const& y2) {
        return cast_positive(max(unsign(y1),unsign(y2))); }
    friend Positive<Number<P>> abs(Positive<Number<P>> const& y) { return y; }
};


template<class X> decltype(auto) cast_generic(X const& x) { return x.generic(); }

template<class P> Positive<Number<P>> cast_positive(Number<P> y) {
    return Positive<Number<P>>(y); }
template<class P> Positive<ExactNumber> cast_exact(Positive<Number<P>> const& y) {
    return cast_positive(cast_exact(unsign(y))); }
template<class P> Positive<ExactNumber> cast_exact(Positive<UpperNumber<P>> const& y) {
    return cast_positive(cast_exact(unsign(y))); }
template<class P> Positive<ExactNumber> cast_exact(Positive<LowerNumber<P>> const& y) {
    return cast_positive(cast_exact(unsign(y))); }

} // namespace Ariadne

#endif
