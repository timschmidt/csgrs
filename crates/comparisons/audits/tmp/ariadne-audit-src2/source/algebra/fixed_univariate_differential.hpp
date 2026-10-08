/***************************************************************************
 *            algebra/fixed_univariate_differential.hpp
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

/*! \file algebra/fixed_univariate_differential.hpp
 *  \brief First- and second-order univariate differentials.
 */

#ifndef ARIADNE_FIXED_UNIVARIATE_DIFFERENTIAL_HPP
#define ARIADNE_FIXED_UNIVARIATE_DIFFERENTIAL_HPP

#include "utility/macros.hpp"

namespace Ariadne {

template<class X> class DifferentialFactory;

//! \ingroup DifferentiationSubModule
//! \brief A class representing the value and derivative of a scalar quantity
//! depending on a single argument.
template<class X>
class UnivariateFirstDifferential
    : public DispatchTranscendentalAlgebraOperations<UnivariateFirstDifferential<X>,X>
    , public DispatchLatticeAlgebraOperations<UnivariateFirstDifferential<X>,X>
    , public ProvideConcreteGenericArithmeticOperations<UnivariateFirstDifferential<X>>
{
  public:
    X _value;
    X _gradient;
  public:
    //! \brief The type of used to represent numbers in the coefficient.
    typedef typename X::NumericType NumericType;
    //! \brief The type of used to represent the coefficients.
    typedef X ValueType;

    template<class... PRS> requires Constructible<X,Nat,PRS...>
    explicit UnivariateFirstDifferential(PRS... prs) : _value(X(0u,prs...)), _gradient(_value) { }

    //! \brief Constructs a first differential with constant value \a c.
    explicit UnivariateFirstDifferential(const X& c) : _value(c), _gradient(nul(c)) { }

    //! \brief Constructs a first differential with value \a v and gradient \a g.
    explicit UnivariateFirstDifferential(const X& v, const X& g) : _value(v), _gradient(g) { }

    //! \brief Conversion constructor from a different numerical type.
    template<class XX> UnivariateFirstDifferential(const UnivariateFirstDifferential<XX>& x)
        : _value(x._value), _gradient(x._gradient) { }

    //! \brief Construct from  a power series.
    UnivariateFirstDifferential(const Series<X>& x)
        : _value(x[0]), _gradient(x[1]) { }

    template<class OP> UnivariateFirstDifferential(OP op, X const& c)
        : _value(op(c)), _gradient(next_series_coefficient(op,1u,c,&_value)) { }

    //! \brief Set the differential equal to a constant, without changing the degree or number of arguments.
    UnivariateFirstDifferential<X>& operator=(const X& c) { _value=c; _gradient=nul(c); return *this; }

    //! \brief A constant differential of degree \a deg in \a as arguments with value \a c.
    static UnivariateFirstDifferential<X> constant(const X& c) {
        UnivariateFirstDifferential<X> r(c); return r; }
    //! \brief A differential of degree \a deg in \a as arguments representing the quantity \f$v+x_j\f$.
    static UnivariateFirstDifferential<X> variable(const X& v) {
        UnivariateFirstDifferential<X> r(v); r._gradient=1; return r; }

    UnivariateFirstDifferential<X> create_zero() const {
        return UnivariateFirstDifferential(nul(this->_value)); }

    //! \brief Equality operator.
    EqualityType<X> operator==(const UnivariateFirstDifferential<X>& other) const {
        return this->_value==other._value && this->_gradient==other._gradient; }

    //! \brief Inequality operator.
    InequalityType<X> operator!=(const UnivariateFirstDifferential<X>& other) const {
        return !(*this==other); }

    //! \brief The number of independent variables.
    SizeType argument_size() const { return 1u; }
    //! \brief The maximum degree of the stored terms.
    DegreeType degree() const { return 1u; }
    //! \brief The value of the differential i.e. the coefficient of \f$1\f$.
    const X& value() const { return this->_value; }
    //! \brief The first derivative.
    const X& gradient() const { return this->_gradient; }

    //! \brief Set the coefficient of the constant term to \a c.
    Void set_value(const X& c) { this->_value=c; }

    //! \brief Set all coefficients to zero.
    Void clear() { this->_value=0; this->_gradient=0; }
  public:
    friend CharacteristicsType<X> characteristics(UnivariateFirstDifferential<X> const& dx) {
        return characteristics(dx._value); }
    friend DifferentialFactory<X> factory(UnivariateFirstDifferential<X> const& dx) {
        return DifferentialFactory<X>(dx.value().precision()); }
    friend OutputStream& operator<<(OutputStream& os, const UnivariateFirstDifferential<X>& x) {
        os << "D<R"<<x.argument_size()<<","<<x.degree()<<">{ ";
        os << x._value << "; " << x._gradient;
        return os << " }";
    }
  public:
    friend decltype(auto) operator>=(const UnivariateFirstDifferential<X>& x, const UnivariateFirstDifferential<X>& y) {
        return x._value>=y._value; }
    friend decltype(auto) operator<=(const UnivariateFirstDifferential<X>& x, const UnivariateFirstDifferential<X>& y) {
        return x._value<=y._value; }
    friend decltype(auto) operator>=(const UnivariateFirstDifferential<X>& x, const X& c) {
        return x._value>=c; }
    friend decltype(auto) operator<=(const UnivariateFirstDifferential<X>& x, const X& c) {
        return x._value<=c; }
    friend decltype(auto) operator> (const UnivariateFirstDifferential<X>& x, const X& c) {
        return x._value> c; }
    friend decltype(auto) operator< (const UnivariateFirstDifferential<X>& x, const X& c) {
        return x._value< c; }
    friend decltype(auto) operator>=(const X& c, const UnivariateFirstDifferential<X>& x) {
        return c>=x._value; }
    friend decltype(auto) operator<=(const X& c, const UnivariateFirstDifferential<X>& x) {
        return c<=x._value; }
    friend decltype(auto) operator< (const X& c, const UnivariateFirstDifferential<X>& x) {
        return c< x._value; }
    friend decltype(auto) operator> (const X& c, const UnivariateFirstDifferential<X>& x) {
        return c> x._value; }
};

template<class X> UnivariateFirstDifferential<X> create_zero(const UnivariateFirstDifferential<X>& c) {
    return UnivariateFirstDifferential<X>(create_zero(c.value())); }

template<class X> struct AlgebraOperations<UnivariateFirstDifferential<X>,X> {
    static UnivariateFirstDifferential<X> apply(Add, UnivariateFirstDifferential<X> x, const X& c) {
        x._value+=c; return x; }
    static UnivariateFirstDifferential<X> apply(Sub, UnivariateFirstDifferential<X> x, const X& c) {
        x._value-=c; return x; }
    static UnivariateFirstDifferential<X> apply(Mul, UnivariateFirstDifferential<X> x, const X& c) {
        x._value*=c; x._gradient*=c; return x; }
    static UnivariateFirstDifferential<X> apply(Div, UnivariateFirstDifferential<X> x, const X& c) {
        x._value/=c; x._gradient/=c; return x; }

    static UnivariateFirstDifferential<X> apply(Add, UnivariateFirstDifferential<X> x, const UnivariateFirstDifferential<X>& y) {
        x._value += y._value; x._gradient += y._gradient; return x; }
    static UnivariateFirstDifferential<X> apply(Sub, UnivariateFirstDifferential<X> x, const UnivariateFirstDifferential<X>& y) {
        x._value -= y._value; x._gradient -= y._gradient; return x; }
    static UnivariateFirstDifferential<X> apply(Mul, UnivariateFirstDifferential<X> x, const UnivariateFirstDifferential<X>& y) {
        x._gradient *= y._value; x._gradient += x._value * y._gradient; x._value *= y._value; return x; }
    static UnivariateFirstDifferential<X> apply(Div, UnivariateFirstDifferential<X> x, const UnivariateFirstDifferential<X>& y) {
        x._value /= y._value; x._gradient -= x._value * y._gradient; x._gradient /= y._value; return x; }

    static UnivariateFirstDifferential<X> apply(Pos, const UnivariateFirstDifferential<X>& x) {
        return UnivariateFirstDifferential<X>(+x._value,+x._gradient); }
    static UnivariateFirstDifferential<X> apply(Neg, const UnivariateFirstDifferential<X>& x) {
        return UnivariateFirstDifferential<X>(-x._value,-x._gradient); }

    static UnivariateFirstDifferential<X> apply(Min, const UnivariateFirstDifferential<X>& x1, const UnivariateFirstDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2); if(x1.value()==x2.value()) {
            ARIADNE_THROW(std::runtime_error,"min(UnivariateFirstDifferential<X> x1, UnivariateFirstDifferential<X> x2)","x1[0]==x2[0]"); }
        return x1.value()<x2.value() ? x1 : x2; }
    static UnivariateFirstDifferential<X> apply(Max, const UnivariateFirstDifferential<X>& x1,const UnivariateFirstDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2); if(x1.value()==x2.value()) {
            ARIADNE_THROW(std::runtime_error,"max(UnivariateFirstDifferential<X> x1, UnivariateFirstDifferential<X> x2)","x1[0]==x2[0]"); }
        return x1.value()>x2.value() ? x1 : x2; }
    static UnivariateFirstDifferential<X> apply(Abs, const UnivariateFirstDifferential<X>& x) {
        if(x.value()==0) {
            ARIADNE_THROW(std::runtime_error,"abs(UnivariateFirstDifferential<X> x)","x[0]==0"); }
        return x.value()>0 ? pos(x) : neg(x); }

    static UnivariateFirstDifferential<X> apply(Pow, const UnivariateFirstDifferential<X>& x, Int n) {
        return UnivariateFirstDifferential<X>( pow(x._value,n), (n*pow(x._value,n-1))*x._gradient ); }

    template<class OP> static UnivariateFirstDifferential<X> apply(OP op, const UnivariateFirstDifferential<X>& x) {
        return compose(UnivariateFirstDifferential<X>(op,x.value()),x); }

    static UnivariateFirstDifferential<X> apply(Sqrt, const UnivariateFirstDifferential<X>& x) {
        X sqrt_val = sqrt(x._value); return UnivariateFirstDifferential<X>( sqrt_val, rec(2*sqrt_val)*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Exp, const UnivariateFirstDifferential<X>& x) {
        X exp_val = exp(x._value); return UnivariateFirstDifferential<X>( exp_val, exp_val*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Log, const UnivariateFirstDifferential<X>& x) {
        X log_val = log(x._value); X rec_val = rec(x._value); return UnivariateFirstDifferential<X>( log_val, rec_val*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Sin, const UnivariateFirstDifferential<X>& x) {
        X sin_val = sin(x._value); X cos_val = cos(x._value); return UnivariateFirstDifferential<X>( sin_val, cos_val*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Cos, const UnivariateFirstDifferential<X>& x) {
        X cos_val = cos(x._value); X neg_sin_val = neg(sin(x._value)); return UnivariateFirstDifferential<X>( cos_val, neg_sin_val*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Tan, const UnivariateFirstDifferential<X>& x) {
        X tan_val = tan(x._value); X sqr_sec_val = sqr(rec(cos(x._value))); return UnivariateFirstDifferential<X>( tan_val, sqr_sec_val*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Asin, const UnivariateFirstDifferential<X>& x) {
        X asin_val = asin(x._value); X d_asin_val = rec(sqrt(1.0-sqr(x._value))); return UnivariateFirstDifferential<X>( asin_val, d_asin_val*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Acos, const UnivariateFirstDifferential<X>& x) {
        X acos_val = acos(x._value); X d_acos_val = neg(rec(sqrt(1.0-sqr(x._value)))); return UnivariateFirstDifferential<X>( acos_val, d_acos_val*x._gradient ); }

    static UnivariateFirstDifferential<X> apply(Atan, const UnivariateFirstDifferential<X>& x) {
        X atan_val = atan(x._value); X d_atan_val = rec(1+sqr(x._value)); return UnivariateFirstDifferential<X>( atan_val, d_atan_val*x._gradient ); }
};










//! \ingroup DifferentiationSubModule
//! \brief A class representing the value, first and second derivatives of a scalar quantity
//! depending on a single argument.
template<class X>
class UnivariateSecondDifferential
    : public DispatchTranscendentalAlgebraOperations<UnivariateSecondDifferential<X>,X>
    , public DispatchLatticeAlgebraOperations<UnivariateSecondDifferential<X>,X>
    , public ProvideConcreteGenericArithmeticOperations<UnivariateSecondDifferential<X>>
{
  public:
    X _value;
    X _gradient;
    X _half_hessian;
  public:
    //! \brief The type of used to represent numbers in the coefficient.
    typedef typename X::NumericType NumericType;
    //! \brief The type of used to represent the coefficients.
    typedef X ValueType;

    template<class... PRS> requires Constructible<X,Nat,PRS...>
    explicit UnivariateSecondDifferential(PRS... prs) : _value(X(0u,prs...)), _gradient(_value), _half_hessian(_value) { }

    //! \brief Constructs a constant differential with value \a c.
    explicit UnivariateSecondDifferential(const X& c) : _value(c), _gradient(nul(c)), _half_hessian(nul(c)) { }

    //! \brief Constructs a second differential with value \a v, derivative \a g and second derivative \a 0.
    explicit UnivariateSecondDifferential(const X& v, const X& g) : _value(v), _gradient(g), _half_hessian(nul(v)) { }

    //! \brief Constructs a second differential with value \a v, derivative \a g and second derivative \a h.
//    explicit UnivariateSecondDifferential(const X& v, const X& g, const X& h) : _value(v), _gradient(g), _half_hessian(h) { }

    //! \brief Construct from  a power series.
    UnivariateSecondDifferential(const Series<X>& x)
        : _value(x[0]), _gradient(x[1]), _half_hessian(x[2]) { }

    template<class OP> UnivariateSecondDifferential(OP op, X const& c)
        : _value(op(c)), _gradient(next_series_coefficient(op,1u,c,&_value)), _half_hessian(next_series_coefficient(op,2u,c,&_value)) { }

    //! \brief Conversion constructor from a different numerical type.
    template<class XX> UnivariateSecondDifferential(const UnivariateSecondDifferential<XX>& x)
        : _value(x._value), _gradient(x._gradient), _half_hessian(x._half_hessian) { }

    //! \brief Set the differential equal to a constant, without changing the degree or number of arguments.
    UnivariateSecondDifferential<X>& operator=(const X& c) {
        _value=c; _gradient=nul(c); _half_hessian=nul(c); return *this; }
    template<AssignableTo<X> W>
        UnivariateSecondDifferential<X>& operator=(const W& c) { X xc=nul(this->value()); xc=c; return (*this)=xc; }

    //! \brief A constant differential of degree \a deg in \a as arguments with value \a c.
    static UnivariateSecondDifferential<X> constant(const X& c) {
        UnivariateSecondDifferential<X> r(c); return r; }
    //! \brief A differential of degree \a deg in \a as arguments representing the quantity \f$v+x_j\f$.
    static UnivariateSecondDifferential<X> variable(const X& v) {
        UnivariateSecondDifferential<X> r(v); r._gradient=1; return r; }

    UnivariateSecondDifferential<X> create_zero() const {
        return UnivariateSecondDifferential(nul(this->_value)); }

        //! \brief Equality operator.
    EqualityType<X> operator==(const UnivariateSecondDifferential<X>& other) const {
        return this->_value==other._value && this->_gradient==other._gradient  && this->_half_hessian==other._half_hessian; }

    //! \brief Inequality operator.
    InequalityType<X> operator!=(const UnivariateSecondDifferential<X>& other) const {
        return !(*this==other); }

    //! \brief The number of independent variables.
    SizeType argument_size() const { return 1u; }
    //! \brief The maximum degree of the stored terms.
    DegreeType degree() const { return 2u; }
    //! \brief The value of the differential i.e. the coefficient of \f$1\f$.
    const X& value() const { return this->_value; }
    //! \brief The first derivative.
    const X& gradient() const { return this->_gradient; }
    //! \brief The second derivative.
    const X hessian() const { return this->_half_hessian*2; }
    //! \brief Half the second derivative; equal to the coefficient of the polynomial with the same derivatives.
    const X& half_hessian() const { return this->_half_hessian; }


    //! \brief Set the coefficient of the constant term to \a c.
    Void set_value(const X& c) { this->_value=c; }

    //! \brief Set all coefficients to zero.
    Void clear() { (*this)=nul(this->_value); }
  public:
    friend CharacteristicsType<X> characteristics(UnivariateSecondDifferential<X> const& dx) {
        return characteristics(dx._value); }

    friend DifferentialFactory<X> factory(UnivariateSecondDifferential<X> const& dx) {
        return DifferentialFactory<X>(dx.value().precision()); }

    friend UnivariateSecondDifferential<X> compose(UnivariateSecondDifferential<X> const& df, UnivariateSecondDifferential<X> dg) {
        dg._half_hessian *= df._gradient;
        dg._half_hessian += df._half_hessian * sqr(dg._gradient);
        dg._gradient *= df._gradient;
        dg._value = df._value;
        return dg;
    }
    friend OutputStream& operator<<(OutputStream& os, const UnivariateSecondDifferential<X>& x)  {
        os << "D<R"<<x.argument_size()<<","<<x.degree()<<">{";
        os << x._value << ";" << x._gradient << ";" << x._half_hessian;
        return os << " }";
    }
  public:
    typedef Number<Paradigm<X>> Y;
    friend UnivariateSecondDifferential<X> max(const UnivariateSecondDifferential<X>& x1, const Y& c2) { ARIADNE_NOT_IMPLEMENTED; }
    friend UnivariateSecondDifferential<X> min(const UnivariateSecondDifferential<X>& x1, const Y& c2) { ARIADNE_NOT_IMPLEMENTED; }
    friend UnivariateSecondDifferential<X> max(const Y& c1, const UnivariateSecondDifferential<X>& x2) { ARIADNE_NOT_IMPLEMENTED; }
    friend UnivariateSecondDifferential<X> min(const Y& c1, const UnivariateSecondDifferential<X>& x2) { ARIADNE_NOT_IMPLEMENTED; }

    friend decltype(auto) operator>=(const UnivariateSecondDifferential<X>& x, const UnivariateSecondDifferential<X>& y) {
        return x._value>=y._value; }
    friend decltype(auto) operator<=(const UnivariateSecondDifferential<X>& x, const UnivariateSecondDifferential<X>& y) {
        return x._value<=y._value; }
    friend decltype(auto) operator>=(const UnivariateSecondDifferential<X>& x, const X& c) { return x._value>=c; }
    friend decltype(auto) operator<=(const UnivariateSecondDifferential<X>& x, const X& c) { return x._value<=c; }
    friend decltype(auto) operator> (const UnivariateSecondDifferential<X>& x, const X& c) { return x._value> c; }
    friend decltype(auto) operator< (const UnivariateSecondDifferential<X>& x, const X& c) { return x._value< c; }
    friend decltype(auto) operator>=(const X& c, const UnivariateSecondDifferential<X>& x) { return c>=x._value; }
    friend decltype(auto) operator<=(const X& c, const UnivariateSecondDifferential<X>& x) { return c<=x._value; }
    friend decltype(auto) operator> (const X& c, const UnivariateSecondDifferential<X>& x) { return c> x._value; }
    friend decltype(auto) operator< (const X& c, const UnivariateSecondDifferential<X>& x) { return c< x._value; }
  public:
    explicit UnivariateSecondDifferential(const X& v, const X& g, const X& half_h) : _value(v), _gradient(g), _half_hessian(half_h) { }
};

template<class X> struct AlgebraOperations<UnivariateSecondDifferential<X>,X> {
    static UnivariateSecondDifferential<X> apply(Add, UnivariateSecondDifferential<X> x, const X& c) {
        x._value+=c; return x; }
    static UnivariateSecondDifferential<X> apply(Sub, UnivariateSecondDifferential<X> x, const X& c) {
        x._value-=c; return x; }
    static UnivariateSecondDifferential<X> apply(Mul, UnivariateSecondDifferential<X> x, const X& c) {
        x._value*=c; x._gradient*=c; x._half_hessian*=c; return x; }
    static UnivariateSecondDifferential<X> apply(Div, UnivariateSecondDifferential<X> x, const X& c) {
        x._value/=c; x._gradient/=c; x._half_hessian/=c; return x; }

    static UnivariateSecondDifferential<X> apply(Add, UnivariateSecondDifferential<X> x, const UnivariateSecondDifferential<X>& y) {
        x._value += y._value; x._gradient += y._gradient; x._half_hessian += y._half_hessian; return x; }

    static UnivariateSecondDifferential<X> apply(Sub, UnivariateSecondDifferential<X> x, const UnivariateSecondDifferential<X>& y) {
        x._value -= y._value; x._gradient -= y._gradient; x._half_hessian -= y._half_hessian; return x; }

    static UnivariateSecondDifferential<X> apply(Mul, UnivariateSecondDifferential<X> x, const UnivariateSecondDifferential<X>& y) {
        x._half_hessian *= y._value; x._half_hessian += x._gradient*y._gradient; x._half_hessian += x._value * y._half_hessian; x._gradient *= y._value; x._gradient += x._value * y._gradient; x._value *= y._value; return x; }
    static UnivariateSecondDifferential<X> apply(Div, const UnivariateSecondDifferential<X>& x, const UnivariateSecondDifferential<X>& y) {
        return UnivariateSecondDifferential<X>(x._value/y._value,(x._gradient-(x._value/y._value)*y._gradient)/y._value,
                                            (x._half_hessian/y._value)-(x._value/y._value)*(y._half_hessian/y._value)
                                                -(x._gradient/y._value)*(y._gradient/y._value)+(x._value/y._value)*sqr(y._gradient/y._value)); }

    static UnivariateSecondDifferential<X> apply(Pos, const UnivariateSecondDifferential<X>& x) {
        return UnivariateSecondDifferential<X>(+x._value,+x._gradient,+x._half_hessian); }
    static UnivariateSecondDifferential<X> apply(Neg, const UnivariateSecondDifferential<X>& x) {
        return UnivariateSecondDifferential<X>(-x._value,-x._gradient,-x._half_hessian); }

    static UnivariateSecondDifferential<X> apply(Min, const UnivariateSecondDifferential<X>& x1, const UnivariateSecondDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2); if(decide(x1.value()==x2.value())) {
            ARIADNE_THROW(std::runtime_error,"min(UnivariateSecondDifferential<X> x1, UnivariateSecondDifferential<X> x2)","x1[0]==x2[0]"); }
        return decide(x1.value()<x2.value()) ? x1 : x2; }
    static UnivariateSecondDifferential<X> apply(Max, const UnivariateSecondDifferential<X>& x1,const UnivariateSecondDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2); if(decide(x1.value()==x2.value())) {
            ARIADNE_THROW(std::runtime_error,"max(UnivariateSecondDifferential<X> x1, UnivariateSecondDifferential<X> x2)","x1[0]==x2[0]"); }
        return decide(x1.value()>x2.value()) ? x1 : x2; }
    static UnivariateSecondDifferential<X> apply(Abs, const UnivariateSecondDifferential<X>& x) {
        if(decide(x.value()==0)) {
            ARIADNE_THROW(std::runtime_error,"abs(UnivariateSecondDifferential<X> x)","x[0]==0"); }
        return decide(x.value()>0) ? pos(x) : neg(x); }

    static UnivariateSecondDifferential<X> apply(Rec, const UnivariateSecondDifferential<X>& x) {
        X rec_val = rec(x._value); X neg_sqr_rec_val = neg(sqr(rec_val)); X cub_rec_val = -rec_val*neg_sqr_rec_val;
        return UnivariateSecondDifferential<X>( rec_val, neg_sqr_rec_val*x._gradient, neg_sqr_rec_val*x._half_hessian + cub_rec_val * sqr(x._gradient) ); }

    template<class OP> static UnivariateSecondDifferential<X> apply(OP op, const UnivariateSecondDifferential<X>& x) {
        return compose(UnivariateSecondDifferential<X>(op,x.value()),x); }

    static UnivariateSecondDifferential<X> apply(Sqr, const UnivariateSecondDifferential<X>& x) {
        return UnivariateSecondDifferential<X>( sqr(x._value), (2*x._value)*x._gradient, (x._value)*x._half_hessian + sqr(x._gradient) ); }

    static UnivariateSecondDifferential<X> apply(Pow, const UnivariateSecondDifferential<X>& x, Int n) {
        return UnivariateSecondDifferential<X>( pow(x._value,n), (n*pow(x._value,n-1))*x._gradient, hlf(n*(n-1)*pow(x._value,n-2))*x._half_hessian ); }

    //ddf(y)/dxdx = d/dx ( f'(y) dy/dx) = f''(y) dy/dx dy/dx + f'(y) ddy/dxdx

    static UnivariateSecondDifferential<X> apply(Sqrt, const UnivariateSecondDifferential<X>& x) {
        X sqrt_val = sqrt(x._value); X rec_dbl_sqrt_val = rec(2*sqrt_val); X neg_rec_quad_pow_val = neg(rec(4*sqrt_val*x._value));
        return UnivariateSecondDifferential<X>( sqrt_val, rec_dbl_sqrt_val*x._gradient, rec_dbl_sqrt_val*x._half_hessian + hlf(neg_rec_quad_pow_val) * sqr(x._gradient) ); }

    static UnivariateSecondDifferential<X> apply(Exp, const UnivariateSecondDifferential<X>& x) {
        X exp_val = exp(x._value); return UnivariateSecondDifferential<X>( exp_val, exp_val*x._gradient, exp_val*x._half_hessian+hlf(exp_val)*sqr(x._gradient) ); }

    static UnivariateSecondDifferential<X> apply(Log, const UnivariateSecondDifferential<X>& x) {
        X log_val = log(x._value); X rec_val = rec(x._value); X neg_sqr_rec_val = neg(sqr(rec_val));
        return UnivariateSecondDifferential<X>( log_val, rec_val*x._gradient, rec_val*x._half_hessian+hlf(neg_sqr_rec_val)*sqr(x._gradient) ); }

    static UnivariateSecondDifferential<X> apply(Sin, const UnivariateSecondDifferential<X>& x) {
        X sin_val = sin(x._value); X cos_val = cos(x._value); X neg_sin_val = neg(sin_val);
        return UnivariateSecondDifferential<X>( sin_val, cos_val*x._gradient, cos_val*x._half_hessian+hlf(neg_sin_val)*sqr(x._gradient) ); }

    static UnivariateSecondDifferential<X> apply(Cos, const UnivariateSecondDifferential<X>& x) {
        X cos_val = cos(x._value); X neg_sin_val = neg(sin(x._value)); X neg_cos_val = neg(cos_val);
        return UnivariateSecondDifferential<X>( cos_val, neg_sin_val*x._gradient, neg_sin_val*x._half_hessian+hlf(neg_cos_val)*sqr(x._gradient) ); }

    static UnivariateSecondDifferential<X> apply(Tan, const UnivariateSecondDifferential<X>& x) {
        X tan_val = tan(x._value); X sqr_sec_val = sqr(rec(cos(x._value))); X dbl_tan_sqr_sec_val = 2*tan_val*sqr_sec_val;
        return UnivariateSecondDifferential<X>( tan_val, sqr_sec_val*x._gradient, sqr_sec_val*x._half_hessian+hlf(dbl_tan_sqr_sec_val)*sqr(x._gradient) ); }

    static UnivariateSecondDifferential<X> apply(Atan, const UnivariateSecondDifferential<X>& x) {
        X atan_val = atan(x._value); X atan_deriv = rec(1+sqr(x._value)); X atan_second_deriv = -2*x._value*sqr(atan_deriv);
        return UnivariateSecondDifferential<X>( atan_val, atan_deriv*x._gradient, atan_deriv*x._half_hessian+hlf(atan_second_deriv)*sqr(x._gradient) ); }
};





} //namespace Ariadne

#endif // ARIADNE_FIXED_UNIVARIATE_DIFFERENTIAL_HPP
