/***************************************************************************
 *            algebra/fixed_differential.hpp
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

/*! \file algebra/fixed_differential.hpp
 *  \brief First- and second-order multivariate differentials.
 */

#ifndef ARIADNE_FIXED_DIFFERENTIAL_HPP
#define ARIADNE_FIXED_DIFFERENTIAL_HPP

#include <map>

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/series.hpp"
#include "algebra/expansion.hpp"
#include "algebra/operations.hpp"

#include "algebra/fixed_univariate_differential.hpp"

namespace Ariadne {

template<class X> class Vector;
template<class X> class Matrix;
template<class X> class Series;

struct SeriesTag { };

template<class X> class SecondDifferential;

template<class V1, class V2>
struct SymmetricOuterProduct
    : public MatrixExpression< SymmetricOuterProduct<V1,V2> >
{
    typedef typename V1::value_type value_type;
    const V1& _v1; const V2& _v2;
    SymmetricOuterProduct(const V1& v1, const V2& v2) : _v1(v1), _v2(v2) { }
    value_type operator() (SizeType i, SizeType j) { return _v1[i]*_v2[j]+_v1[j]*_v2[i]; }
};
template<class V1, class V2> inline
SymmetricOuterProduct<V1,V2>
symmetric_outer_product(const V1& v1, const V2& v2) {
    return SymmetricOuterProduct<V1,V2>(v1,v2);
}

template<class V1, class V2>
struct OuterProduct
    : public MatrixExpression< OuterProduct<V1,V2> >
{
    typedef typename V1::value_type value_type;
    const V1& _v1; const V2& _v2;
    OuterProduct(const V1& v1, const V2& v2) : _v1(v1), _v2(v2) { }
    value_type operator() (SizeType i, SizeType j) { return _v1[i]*_v2[j]; }
};
template<class V1, class V2> inline
OuterProduct<V1,V2>
outer_product(const V1& v1, const V2& v2) {
    return OuterProduct<V1,V2>(v1,v2);
}

/*
template<class X>
Matrix<X> outer(const Vector<X>& v1, const Vector<X>& v2) {
    Matrix<X> r(v1.size(),v2.size());
    for(SizeType i1=0; i1!=v1.size(); ++i1) {
        for(SizeType i2=0; i2!=v2.size(); ++i2) {
            r[i1][i2]=v1[i1]*v2[i2];
        }
    }
    return r;
}

template<class X>
Matrix<X> outer(const Vector<X>& v) {
    return outer(v,v);
}
*/

template<class X>
Matrix<X> outer(const Covector<X>& u1, const Covector<X>& u2) {
    Matrix<X> r(u1.size(),u2.size());
    for(SizeType i1=0; i1!=u1.size(); ++i1) {
        for(SizeType i2=0; i2!=u2.size(); ++i2) {
            r[i1][i2]=u1[i1]*u2[i2];
        }
    }
    return r;
}

template<class X>
Matrix<X> outer(const Covector<X>& u) {
    return outer(u,u);
}

template<class X> class FirstDifferential;
template<class X> class SecondDifferential;

template<class X> class FixedDifferentialCharacteristics {
    SizeType _argument_size; X _zero_coefficient;
  public:
    template<class... PRS> requires Constructible<X,Nat,PRS...>
        FixedDifferentialCharacteristics(SizeType as, PRS... prs) : _argument_size(as), _zero_coefficient(0u,prs...) { }
    FixedDifferentialCharacteristics(SizeType as, X const& z) : _argument_size(as), _zero_coefficient(nul(z)) { }
    FixedDifferentialCharacteristics(const FirstDifferential<X>& d);
    FixedDifferentialCharacteristics(const SecondDifferential<X>& d);
    SizeType argument_size() const { return this->_argument_size; }
    X zero_coefficient() const { return this->_zero_coefficient; }
};

//! \ingroup DifferentiationSubModule
//! \brief A class representing the partial derivatives of a scalar quantity
//! depending on multiple arguments.
//!
//! Based on a power series Expansion, centred on the point at which the partial derivatives are
//! being evaluated.
//!
//! \invariant The expansion is sorted using graded_sort(). In particular, the
//! total degree of the terms is increasing, and the linear terms appear in coordinate order.
template<class X>
class FirstDifferential
    : public DispatchTranscendentalAlgebraOperations<FirstDifferential<X>,X>
    , public DispatchLatticeAlgebraOperations<FirstDifferential<X>,X>
    , public ProvideConcreteGenericArithmeticOperations<FirstDifferential<X>>
{
  public:
    X _value;
    Covector<X> _gradient;
  public:
    //! \brief The type of used to represent numbers in the coefficient.
    typedef typename X::NumericType NumericType;
    //! \brief The type of used to represent the coefficients.
    typedef X ValueType;

    //! \brief Constructs a differential with degree one in \a as variables, and zero element based on \a prs.
    template<class... PRS> requires Constructible<X,Nat,PRS...> explicit FirstDifferential(SizeType as, PRS... prs)
        : FirstDifferential(as,X(0u,prs...)) { }

    //! \brief Constructs a differential with degree one in \a as variables, initialising to the zero value z.
    explicit FirstDifferential(SizeType as, const X& z) : _value(z), _gradient(as,z) { }

    //! \brief Constructs a differential with degree zero in \a as variables. (Deprecated)
    explicit FirstDifferential(const X& v, const Covector<X>& g) : _value(v), _gradient(g) { }

    //! \brief Conversion constructor from a different numerical type.
    template<class XX> FirstDifferential(const FirstDifferential<XX>& x)
        : _value(x._value), _gradient(x._gradient) { }

    //! \brief Set the differential equal to a constant, without changing the degree or number of arguments.
    FirstDifferential<X>& operator=(const X& c) { _value=c; X z(c); z=0; for(SizeType i=0; i!=_gradient.size(); ++i) { _gradient[i]=z; } return *this; }
    template<AssignableTo<X> W>
        FirstDifferential<X>& operator=(const W& c) { X xc=nul(this->value()); xc=c; return (*this)=xc; }

    //! \brief A constant differential of degree \a deg in \a as arguments with value \a c.
    static FirstDifferential<X> constant(SizeType as, const X& c) {
        FirstDifferential<X> r(as); r._value=c; return r; }
    //! \brief A differential of degree \a deg in \a as arguments representing the quantity \f$v+x_j\f$.
    static FirstDifferential<X> variable(SizeType as, const X& v, SizeType j) {
        FirstDifferential<X> r(as); r._value=v; r._gradient[j]=1; return r; }
    //! \brief \brief A vector of differentials of degree \a deg in \a as arguments with values \f$c_i+x_i\f$.
    static Vector< FirstDifferential<X> > variables(const Vector<X>& x) {
        Vector< FirstDifferential<X> > result(x.size(),FirstDifferential<X>(x.size(),x.zero_element()));
        for(SizeType j=0; j!=x.size(); ++j) { result[j]._value=x[j]; result[j]._gradient[j]=1; }
        return result; }

    //! \brief Equality operator.
    friend EqualityType<X> operator==(const FirstDifferential<X>& dx1, const FirstDifferential<X>& dx2) {
        return dx1._value==dx2._value && dx1._gradient==dx2._gradient; }

    //! \brief Inequality operator.
    friend InequalityType<X> operator!=(const FirstDifferential<X>& dx1, const FirstDifferential<X>& dx2) {
        return !(dx1==dx2); }

    //! \brief Comparison operator.
    friend decltype(auto) operator>=(const FirstDifferential<X>& dx1, const FirstDifferential<X>& dx2) {
        return dx1._value>=dx2._value; }
    //! \brief Comparison operator.
    friend decltype(auto) operator<=(const FirstDifferential<X>& dx1, const FirstDifferential<X>& dx2) {
        return dx1._value<=dx2._value; }

    //! \brief Characteristics needed to construct a zero element.
    friend Pair<SizeType,CharacteristicsType<X>> characteristics(FirstDifferential<X>const& dx) {
        return std::make_pair(dx.argument_size(),characteristics(dx.zero_coefficient())); }
    //! \brief The number of independent variables.
    SizeType argument_size() const { return this->_gradient.size(); }
    //! \brief The maximum degree of the stored terms.
    DegreeType degree() const { return 1u; }
    //! \brief A zero with the precision argument of the numbers.
    X zero_coefficient() const { X z=this->_value; z=0; return z; }
    //! \brief The value of the differential i.e. the coefficient of \f$1\f$.
    const X& value() const { return this->_value; }
    //! \brief The vector of coefficients of \f$x_j\f$.
    const Covector<X>& gradient() const { return this->_gradient; }
    //! \brief The vector of coefficients of \f$x_j\f$.
    const X& gradient(SizeType j) const { return this->_gradient[j]; }


    //! \brief A reference to the coefficient of \f$x_j\f$.
    X& operator[](const SizeType& j) { return this->_gradient[j]; }
    //! \brief The coefficient of \f$x_j\f$.
    const X& operator[](const SizeType& j) const { return this->_gradient[j]; }
    //! \brief The coefficient of \f$x^a\f$ i.e. \f$\prod x_j^{a_j}\f$.
    const X& operator[](const MultiIndex& a) const {
        ARIADNE_ASSERT_MSG(a.number_of_variables()==this->argument_size()," d="<<*this<<", a="<<a);
        if(a.degree()==0) { return this->_value; }
        else if(a.degree()==1) { for(SizeType j=0; j!=this->argument_size(); ++j) { if(a[j]==1) { return this->_gradient[j]; } } }
        else { ARIADNE_ASSERT_MSG(a.degree()<=1u," d="<<*this<<", a="<<a); } }

    //! \brief Set the coefficient of the constant term to \a c.
    Void set_value(const X& c) { this->_value=c; }

    //! \brief Set all coefficients to zero.
    Void clear() { X z=this->_value; z=0; *this=z; }

    friend FirstDifferential<X> compose(UnivariateFirstDifferential<X> const& df, FirstDifferential<X> dg) {
        return _compose(df,dg); }

    friend OutputStream& operator<<(OutputStream& os, FirstDifferential<X> df) {
        return df._write(os); }

  private:
    static FirstDifferential<X> _compose(UnivariateFirstDifferential<X> const&, FirstDifferential<X>);
    OutputStream& _write(OutputStream&) const;
};

template<class X> FirstDifferential<X> FirstDifferential<X>::_compose(UnivariateFirstDifferential<X> const& x, FirstDifferential<X> y) {
    //d/dxi(f(g(x)) = f'(g(x)) dg(x)/dxi
    //d2/dxi2(f(g(x)) = f''(g(x)) * dg(x)/dxi * dg(x)/dxi + f'(g(x)) d2g(x)/dxi2
    //d2/dxidxi(f(g(x)) = f''(g(x)) * dg(x)/dxi * dg(x)/dxj + f'(g(x)) d2g(x)/dxidxj
    y._gradient *= x.gradient();
    y._value = x._value;
    return y;
}

template<class X> OutputStream& FirstDifferential<X>::_write(OutputStream& os) const {
    FirstDifferential<X>const& x=*this;
    os << "D<R"<<x.argument_size()<<","<<x.degree()<<">{ ";
    os << x._value;
    for(SizeType j=0; j!=x.argument_size(); ++j) {
        os << (j==0?"; ":",") << j << ":" << x[j];
    }
    return os << " }";
}


template<class X> struct AlgebraOperations<FirstDifferential<X>,X> {

    static FirstDifferential<X> apply(Pos, const FirstDifferential<X>& x) {
        return FirstDifferential<X>(+x._value,+x._gradient); }
    static FirstDifferential<X> apply(Neg, const FirstDifferential<X>& x) {
        return FirstDifferential<X>(-x._value,-x._gradient); }

    static FirstDifferential<X> apply(Add, FirstDifferential<X> x, const FirstDifferential<X>& y) {
        x._value+=y._value; x._gradient+=y._gradient; return x; }
    static FirstDifferential<X> apply(Sub, FirstDifferential<X> x, const FirstDifferential<X>& y) {
        x._value-=y._value; x._gradient-=y._gradient; return x; }
    static FirstDifferential<X> apply(Mul, const FirstDifferential<X>& x, const FirstDifferential<X>& y) {
        return FirstDifferential<X>(x._value*y._value,x._value*y._gradient+y._value*x._gradient); }
    static FirstDifferential<X> apply(Div, const FirstDifferential<X>& x, const FirstDifferential<X>& y) {
        return FirstDifferential<X>(x._value/y._value,((x._value/y._value)*y._gradient+x._gradient)/y._value); }

    static FirstDifferential<X> apply(Add, FirstDifferential<X> x, const X& c) {
        x._value+=c; return x; }
    static FirstDifferential<X> apply(Sub, FirstDifferential<X> x, const X& c) {
        x._value-=c; return x; }
    static FirstDifferential<X> apply(Mul, FirstDifferential<X> x, const X& c) {
        x._value*=c; x._gradient*=c; return x; }
    static FirstDifferential<X> apply(Div, FirstDifferential<X> x, const X& c) {
        x._value/=c; x._gradient/=c; return x; }


    static FirstDifferential<X> apply(Min, const FirstDifferential<X>& x1, const FirstDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2); if(x1.value()==x2.value()) {
            ARIADNE_THROW(std::runtime_error,"min(FirstDifferential<X> x1, FirstDifferential<X> x2)","x1[0]==x2[0]"); }
        return x1.value()<x2.value() ? x1 : x2; }
    static FirstDifferential<X> apply(Max, const FirstDifferential<X>& x1,const FirstDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2); if(x1.value()==x2.value()) {
            ARIADNE_THROW(std::runtime_error,"max(FirstDifferential<X> x1, FirstDifferential<X> x2)","x1[0]==x2[0]"); }
        return x1.value()>x2.value() ? x1 : x2; }
    static FirstDifferential<X> apply(Abs, const FirstDifferential<X>& x) {
        if(x.value()==0) {
            ARIADNE_THROW(std::runtime_error,"abs(FirstDifferential<X> x)","x[0]==0"); }
        return x.value()>0 ? pos(x) : neg(x); }

    template<class OP> static FirstDifferential<X> apply(OP op, const FirstDifferential<X>& x) {
        return compose(UnivariateFirstDifferential<X>(op,x._value),x); }


    static FirstDifferential<X> apply(Rec, const FirstDifferential<X>& x) {
        return FirstDifferential<X>( rec(x._value), x._gradient * (neg(sqr(rec(x._value)))) ); }

    static FirstDifferential<X> apply(Sqr, const FirstDifferential<X>& x) {
        return FirstDifferential<X>( sqr(x._value), (2*x._value)*x._gradient ); }

    static FirstDifferential<X> apply(Pow, const FirstDifferential<X>& x, Int n) {
        return FirstDifferential<X>( pow(x._value,n), (n*pow(x._value,n-1))*x._gradient ); }

    static FirstDifferential<X> apply(Sqrt, const FirstDifferential<X>& x) {
        X sqrt_val = sqrt(x._value); return FirstDifferential<X>( sqrt_val, rec(2*sqrt_val)*x._gradient ); }

    static FirstDifferential<X> apply(Exp, const FirstDifferential<X>& x) {
        X exp_val = exp(x._value); return FirstDifferential<X>( exp_val, exp_val*x._gradient ); }

    static FirstDifferential<X> apply(Log, const FirstDifferential<X>& x) {
        X log_val = log(x._value); X rec_val = rec(x._value); return FirstDifferential<X>( log_val, rec_val*x._gradient ); }

    static FirstDifferential<X> apply(Sin, const FirstDifferential<X>& x) {
        X sin_val = sin(x._value); X cos_val = cos(x._value); return FirstDifferential<X>( sin_val, cos_val*x._gradient ); }

    static FirstDifferential<X> apply(Cos, const FirstDifferential<X>& x) {
        X cos_val = cos(x._value); X neg_sin_val = neg(sin(x._value)); return FirstDifferential<X>( cos_val, neg_sin_val*x._gradient ); }

    static FirstDifferential<X> apply(Tan, const FirstDifferential<X>& x) {
        X tan_val = tan(x._value); X sqr_sec_val = sqr(rec(cos(x._value))); return FirstDifferential<X>( tan_val, sqr_sec_val*x._gradient ); }

    static FirstDifferential<X> apply(Asin, const FirstDifferential<X>& x) {
        X asin_val = asin(x._value); X d_asin_val = rec(sqrt(1.0-sqr(x._value))); return FirstDifferential<X>( asin_val, d_asin_val*x._gradient ); }

    static FirstDifferential<X> apply(Acos, const FirstDifferential<X>& x) {
        X acos_val = acos(x._value); X d_acos_val = neg(rec(sqrt(1.0-sqr(x._value)))); return FirstDifferential<X>( acos_val, d_acos_val*x._gradient ); }

    static FirstDifferential<X> apply(Atan, const FirstDifferential<X>& x) {
        X atan_val = atan(x._value); X d_atan_val = rec(1+sqr(x._value)); return FirstDifferential<X>( atan_val, d_atan_val*x._gradient ); }

};









/*! \brief A class representing the derivatives of a vector quantity depending on multiple arguments. */
template<class X>
class Vector< FirstDifferential<X> >
    : public VectorExpression< Vector<FirstDifferential<X> > >
{
    //BOOST_CONCEPT_ASSERT((DifferentialVectorConcept<DifferentialVector<X> >));
    FixedDifferentialCharacteristics<X> _chars;
    Array< FirstDifferential<X> > _ary;
  public:
    // The type of the class
    typedef Vector< FirstDifferential<X> > SelfType;
    // The type used for accessing elements
    typedef SizeType IndexType;
    // The value stored in the vector.
    typedef FirstDifferential<X> ValueType;
    // The type used for scalars.
    typedef X ScalarType;

    Vector(SizeType rs, SizeType as) : _chars(as), _ary(rs, FirstDifferential<X>(as)) { }
    Vector(SizeType rs, const FirstDifferential<X>& d) : _chars(d),  _ary(rs,d) {
        for(SizeType i=0; i!=rs; ++i) { (*this)[i]=d; } }
    Vector(SizeType rs, const FirstDifferential<X>* p) : _chars(p[0]), _ary(rs,FirstDifferential<X>(p[0].argument_size())) {
        for(SizeType i=0; i!=rs; ++i) { (*this)[i]=p[i]; } }
    Vector(const Array<FirstDifferential<X>>& ary) : _chars(ary[0]), _ary(ary) { }
    template<class E> Vector(const VectorExpression<E>& ve);
    template<class E> Vector< FirstDifferential<X> >& operator=(const VectorExpression<E>& ve);

    SizeType result_size() const { return this->size(); }
    SizeType argument_size() const { return this->_chars.argument_size(); }
    DegreeType degree() const { return 1u; }

    SizeType size() const { return this->_ary.size(); }
    const FirstDifferential<X>& get(SizeType i) const { return _ary[i]; }
    FirstDifferential<X>& at(SizeType i) { return _ary[i]; }
    Void set(SizeType i, const FirstDifferential<X>& d) const { _ary[i]=d; }
    const FirstDifferential<X>& operator[](SizeType i) const { return _ary[i]; }
    FirstDifferential<X>& operator[](SizeType i) { return _ary[i]; }
    const FirstDifferential<X> zero_element() const {
        return FirstDifferential(this->argument_size(),_chars.zero_coefficient()); }
    const Array<FirstDifferential<X>>& array() const { return _ary; }

    VectorRange<const Vector<FirstDifferential<X>>> operator[](Range rng) const { return project(*this,rng); }

    Vector<X> value() const {
        Vector<X> r(this->result_size());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=(*this)[i].value(); } return r; }
    Matrix<X> jacobian() const { Matrix<X> r(this->result_size(),this->argument_size(),this->_chars.zero_coefficient());
        for(SizeType i=0; i!=r.row_size(); ++i) { for(SizeType j=0; j!=r.column_size(); ++j) { r[i][j]=(*this)[i][j]; } } return r; }

    friend Vector<X> value(Vector<FirstDifferential<X>> const& x) { return x.value(); }
    friend Matrix<X> jacobian(Vector<FirstDifferential<X>> const& x) { return x.jacobian(); }

};





//! \ingroup DifferentiationSubModule
//! \brief A class representing the partial derivatives of a scalar quantity
//! depending on multiple arguments.
//!
//! Based on a power series Expansion, centred on the point at which the partial derivatives are
//! being evaluated.
//!
//! \invariant The expansion is sorted using graded_sort(). In particular, the
//! total degree of the terms is increasing, and the linear terms appear in coordinate order.
template<class X>
class SecondDifferential
    : public DispatchTranscendentalAlgebraOperations<SecondDifferential<X>,X>
    , public DispatchLatticeAlgebraOperations<SecondDifferential<X>,X>
    , public ProvideConcreteGenericArithmeticOperations<SecondDifferential<X>>
{
  public:
    X _value;
    Covector<X> _gradient;
    Matrix<X> _half_hessian;
  public:
    //! \brief The type of used to represent numbers in the coefficient.
    typedef typename X::NumericType NumericType;
    //! \brief The type of used to represent the coefficients.
    typedef X ValueType;

    //! \brief Constructs the zero second differential in \a as variables, with elements constructed from parameters \a prs.
    template<class... PRS> requires Constructible<X,Nat,PRS...> explicit SecondDifferential(SizeType as, PRS... prs) : SecondDifferential<X>(as,X(0u,prs...)) { }

    //! \brief Constructs a constant second differential in \a as variables with value \a c.
    explicit SecondDifferential(SizeType as, const X& c) : _value(c), _gradient(as,nul(c)), _half_hessian(as,as,nul(c)) { }

    //! \brief Constructs a second differential in \a as variables with value \a v and gradient \a g.
    explicit SecondDifferential(const X& v, const Covector<X>& g) : _value(v), _gradient(g), _half_hessian(g.size(),g.size(),nul(v)) { }

    //! \brief Constructs a differential with degree zero in \a as variables with value \a v, gradient \a g and hessian \a h.
    //explicit SecondDifferential(const X& v, const Covector<X>& g, const Matrix<X>& h) : _value(v), _gradient(g), _half_hessian(h/2) {
    //    ARIADNE_ASSERT_MSG(h.row_size()==g.size() && h.column_size()==g.size(), "SecondDifferential(v,g,h): v="<<v<<", g="<<g<<", h="<<h); }
//    explicit SecondDifferential(const X& v, const Covector<X>& g, const Matrix<X>& half_h) : _value(v), _gradient(g), _half_hessian(half_h) {
//        ARIADNE_ASSERT_MSG(half_h.row_size()==g.size() && half_h.column_size()==g.size(), "SecondDifferential(v,g,half_h): v="<<v<<", g="<<g<<", half_h="<<half_h); }

    //! \brief Conversion constructor from a different numerical type.
    template<class XX> SecondDifferential(const SecondDifferential<XX>& x)
        : _value(x._value), _gradient(x._gradient), _half_hessian(x._half_hessian) { }

    //! \brief Set the differential equal to a constant, without changing the degree or number of arguments.
    SecondDifferential<X>& operator=(const X& c) {
        X z=c; z=0; _value=c; for(SizeType j=0; j!=this->argument_size(); ++j) { _gradient[j]=z;
            for(SizeType k=0; k!=this->argument_size(); ++k) { _half_hessian[j][k]=z; } } return *this; }
    template<AssignableTo<X> W>
        SecondDifferential<X>& operator=(const W& c) { X xc=nul(this->value()); xc=c; return (*this)=xc; }

    //! \brief A constant differential of degree \a deg in \a as arguments with value \a c.
    static SecondDifferential<X> constant(SizeType as, const X& c) {
        SecondDifferential<X> r(as,c.precision()); r._value=c; return r; }
    //! \brief A differential of degree \a deg in \a as arguments representing the quantity \f$v+x_j\f$.
    static SecondDifferential<X> variable(SizeType as, const X& v, SizeType j) {
        SecondDifferential<X> r(as,v.precision()); r._value=v; r._gradient[j]=1; return r; }
    //! \brief \brief A vector of differentials of degree \a deg in \a as arguments with values \f$c_i+x_i\f$.
    static Vector< SecondDifferential<X> > variables(const Vector<X>& x) {
        Vector< SecondDifferential<X> > result(x.size(),SecondDifferential<X>(x.size(),x.zero_element()));
        for(SizeType j=0; j!=x.size(); ++j) { result[j]._value=x[j]; result[j]._gradient[j]=1; }
        return result; }

    //! \brief Equality operator.
    EqualityType<X> operator==(const SecondDifferential<X>& other) const {
        return this->_value==other._value && this->_gradient==other._gradient  && this->_half_hessian==other._half_hessian; }

    //! \brief Inequality operator.
    InequalityType<X> operator!=(const SecondDifferential<X>& other) const {
        return !(*this==other); }

    //! \brief Characteristics needed to construct a zero element.
    friend Pair<SizeType,CharacteristicsType<X>> characteristics(SecondDifferential<X>const& dx) {
        return std::make_pair(dx.argument_size(),characteristics(dx.zero_coefficient())); }
    //! \brief The number of independent variables.
    SizeType argument_size() const { return this->_gradient.size(); }
    //! \brief The maximum degree of the stored terms.
    DegreeType degree() const { return 2u; }
    //! \brief A zero with the precision argument of the numbers.
    X zero_coefficient() const { X z=this->_value; z=0; return z; }
        //! \brief The value of the differential i.e. the coefficient of \f$1\f$.
    const X& value() const { return this->_value; }
    //! \brief The vector of coefficients of \f$x_j\f$.
    const Covector<X>& gradient() const { return this->_gradient; }
    //! \brief The vector of coefficients of \f$x_j\f$.
    const X& gradient(SizeType j) const { return this->_gradient[j]; }
    //! \brief The Hessian matrix.
    Matrix<X> hessian() const { return this->_half_hessian*2; }


    //! \brief A reference to the coefficient of \f$x_j\f$.
    X& operator[](const SizeType& j) { return this->_gradient[j]; }
    //! \brief The coefficient of \f$x_j\f$.
    const X& operator[](const SizeType& j) const { return this->_gradient[j]; }
    //! \brief The coefficient of \f$x^a\f$ i.e. \f$\prod x_j^{a_j}\f$.
    const X& operator[](const MultiIndex& a) const {
        ARIADNE_ASSERT_MSG(a.number_of_variables()==this->argument_size()," d="<<*this<<", a="<<a);
        if(a.degree()==0) {
            return this->_value;
        } else if(a.degree()==1) {
            for(SizeType j=0; j!=this->argument_size(); ++j) { if(a[j]==1) { return this->_gradient[j]; } }
        } else if(a.degree()==2) {
            SizeType j1=0; while (a[j1]==0) { ++j1; }
            SizeType j2=j1; if (a[j2]==1) { ++j2; while (a[j2]==0) { ++j2; } }
            return this->_half_hessian[j1][j2];
        } else {
            ARIADNE_ASSERT_MSG(a.degree()<=2," d="<<*this<<", a="<<a);
        }
    }

    //! \brief Set the coefficient of the constant term to \a c.
    Void set_value(const X& c) { this->_value=c; }

    //! \brief Set all coefficients to zero.
    Void clear() { *this=0; }

    friend SecondDifferential<X> compose(UnivariateSecondDifferential<X> const& df, SecondDifferential<X> dg) {
        return _compose(df,dg); }
    friend OutputStream& operator<<(OutputStream& os, SecondDifferential<X> df) {
        return df._write(os); }

  private: public:
    explicit SecondDifferential(const X& v, const Covector<X>& g, const Matrix<X>& hh) : _value(v), _gradient(g), _half_hessian(hh) {
        ARIADNE_ASSERT(hh.row_size()==g.size() && hh.column_size()==g.size()); }
    explicit SecondDifferential(const X& v, const Covector<X>& g, const Matrix<X>& hh, SeriesTag) : _value(v), _gradient(g), _half_hessian(hh) {
        ARIADNE_ASSERT(hh.row_size()==g.size() && hh.column_size()==g.size()); }

    static SecondDifferential<X> _compose(UnivariateSecondDifferential<X> const&, SecondDifferential<X>);
    OutputStream& _write(OutputStream&) const;
};

template<class X> SecondDifferential<X> SecondDifferential<X>::_compose(UnivariateSecondDifferential<X> const& x, SecondDifferential<X> y) {
    //d/dxi(f(g(x)) = f'(g(x)) dg(x)/dxi
    //d2/dxi2(f(g(x)) = f''(g(x)) * dg(x)/dxi * dg(x)/dxi + f'(g(x)) d2g(x)/dxi2
    //d2/dxidxi(f(g(x)) = f''(g(x)) * dg(x)/dxi * dg(x)/dxj + f'(g(x)) d2g(x)/dxidxj
    y._half_hessian *= x.gradient();
    y._half_hessian += x.half_hessian()*outer(y._gradient);
    y._gradient *= x.gradient();
    y._value = x._value;
    return y;
}

template<class X> OutputStream& SecondDifferential<X>::_write(OutputStream& os) const {
    SecondDifferential<X> const& x = *this;
    os << "D<R"<<x.argument_size()<<","<<x.degree()<<">{";
    os << x._value;
    for(SizeType j=0; j!=x.argument_size(); ++j) {
        os << (j==0?"; ":",") << j << ":" << x[j];
    }
    for(SizeType j=0; j!=x.argument_size(); ++j) {
        for(SizeType k=0; k!=x.argument_size(); ++k) {
            os << (j==0&&k==0?"; ":",") << j << "," << k << ":" << x._half_hessian[j][k];
        }
    }
    return os << " }";
}


template<class X> struct AlgebraOperations<SecondDifferential<X>,X> {

    static SecondDifferential<X> apply(Pos, const SecondDifferential<X>& x) {
        return SecondDifferential<X>(+x._value,+x._gradient,+x._half_hessian); }
    static SecondDifferential<X> apply(Neg, const SecondDifferential<X>& x) {
        return SecondDifferential<X>(-x._value,-x._gradient,-x._half_hessian); }

    static SecondDifferential<X> apply(Add, SecondDifferential<X> x, const SecondDifferential<X>& y) {
        x._value += y._value; x._gradient += y._gradient; x._half_hessian += y._half_hessian; return x; }
    static SecondDifferential<X> apply(Sub, SecondDifferential<X> x, const SecondDifferential<X>& y) {
        x._value -= y._value; x._gradient -= y._gradient; x._half_hessian -= y._half_hessian; return x; }
    static SecondDifferential<X> apply(Mul, SecondDifferential<X> x, const SecondDifferential<X>& y) {
        x._half_hessian *= y._value;
        x._half_hessian += x._value * y._half_hessian;
        for(SizeType j=0; j!=x.argument_size(); ++j) { for(SizeType k=0; k!=x.argument_size(); ++k) {
            x._half_hessian[j][k]+=(x._gradient[j]*y._gradient[k]+x._gradient[k]*y._gradient[j])/2; } }
        x._gradient *= y._value;
        x._gradient += x._value * y._gradient;
        x._value *= y._value;
        return x;
    }
    static SecondDifferential<X> apply(Div, const SecondDifferential<X>& x, SecondDifferential<X> y) {
        return mul(x,rec(std::move(y))); }

    static SecondDifferential<X> apply(Add, SecondDifferential<X> x, const X& c) {
        x._value+=c; return x; }
    static SecondDifferential<X> apply(Sub, SecondDifferential<X> x, const X& c) {
        x._value-=c; return x; }
    static SecondDifferential<X> apply(Mul, SecondDifferential<X> x, const X& c) {
        x._value*=c; x._gradient*=c; x._half_hessian*=c; return x; }
    static SecondDifferential<X> apply(Div, SecondDifferential<X> x, const X& c) {
        x._value/=c; x._gradient/=c; x._half_hessian/=c; return x; }

    static SecondDifferential<X> apply(Min, const SecondDifferential<X>& x1, const SecondDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2);
        if(x1.value()==x2.value()) {
            ARIADNE_THROW(std::runtime_error,"min(SecondDifferential<X> x1, SecondDifferential<X> x2)","x1[0]==x2[0]");
        }
        return x1.value()<x2.value() ? x1 : x2;
    }
    static SecondDifferential<X> apply(Max, const SecondDifferential<X>& x1,const SecondDifferential<X>& x2) {
        ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2);
        if(x1.value()==x2.value()) {
            ARIADNE_THROW(std::runtime_error,"max(SecondDifferential<X> x1, SecondDifferential<X> x2)","x1[0]==x2[0]");
        }
        return x1.value()>x2.value() ? x1 : x2;
    }
    static SecondDifferential<X> apply(Abs, const SecondDifferential<X>& x) {
        if(x.value()==0) {
            ARIADNE_THROW(std::runtime_error,"abs(SecondDifferential<X> x)","x[0]==0");
        }
        return x.value()>0 ? pos(x) : neg(x);
    }

    template<class OP> static SecondDifferential<X> apply(OP op, const SecondDifferential<X>& x) {
        return compose(UnivariateSecondDifferential<X>(op,x.value(),x)); }

    static SecondDifferential<X> apply(Rec, const SecondDifferential<X>& x) {
        X rec_val = rec(x._value);
        X neg_sqr_rec_val = neg(sqr(rec_val));
        X cub_rec_val = -rec_val*neg_sqr_rec_val;
        return SecondDifferential<X>( rec_val, neg_sqr_rec_val*x._gradient, neg_sqr_rec_val*x._half_hessian + cub_rec_val * outer(x._gradient), SeriesTag() );
    }

    static SecondDifferential<X> apply(Sqr, const SecondDifferential<X>& x) {
        return SecondDifferential<X>( sqr(x._value), (2*x._value)*x._gradient, (2*x._value)*x._half_hessian + outer(x._gradient)/2 );
    }

    static SecondDifferential<X> apply(Pow, const SecondDifferential<X>& x, Int n) {
        return SecondDifferential<X>( pow(x._value,n), (n*pow(x._value,n-1))*x._gradient, n*(n-1)/2*pow(x._value,n-2)*x._half_hessian, SeriesTag() );
    }

    static SecondDifferential<X> apply(Sqrt, const SecondDifferential<X>& x) {
        X sqrt_val = sqrt(x._value);
        X rec_dbl_sqrt_val = rec(2*sqrt_val);
        X neg_rec_quad_pow_val = neg(rec(4*sqrt_val*x._value));
        return SecondDifferential<X>( sqrt_val, rec_dbl_sqrt_val*x._gradient, rec_dbl_sqrt_val*x._half_hessian + neg_rec_quad_pow_val * outer(x._gradient), SeriesTag() );
    }

    static SecondDifferential<X> apply(Exp, const SecondDifferential<X>& x) {
        X exp_val = exp(x._value);
        return SecondDifferential<X>( exp_val, exp_val*x._gradient, exp_val*x._half_hessian+exp_val*outer(x._gradient), SeriesTag() );
    }

    static SecondDifferential<X> apply(Log, const SecondDifferential<X>& x) {
        X log_val = log(x._value);
        X rec_val = rec(x._value);
        X neg_sqr_rec_val = neg(sqr(rec_val));
        return SecondDifferential<X>( log_val, rec_val*x._gradient, rec_val*x._half_hessian+neg_sqr_rec_val*outer(x._gradient), SeriesTag() );
    }

    static SecondDifferential<X> apply(Sin, const SecondDifferential<X>& x) {
        X sin_val = sin(x._value);
        X cos_val = cos(x._value);
        X neg_sin_val = neg(sin_val);
        return SecondDifferential<X>( sin_val, cos_val*x._gradient, cos_val*x._half_hessian+neg_sin_val*outer(x._gradient), SeriesTag() );
    }

    static SecondDifferential<X> apply(Cos, const SecondDifferential<X>& x) {
        X cos_val = cos(x._value);
        X neg_sin_val = neg(sin(x._value));
        X neg_cos_val = neg(cos_val);
        return SecondDifferential<X>( cos_val, neg_sin_val*x._gradient, neg_sin_val*x._half_hessian+neg_cos_val*outer(x._gradient), SeriesTag() );
    }

    static SecondDifferential<X> apply(Tan, const SecondDifferential<X>& x) {
        X tan_val = tan(x._value);
        X sqr_sec_val = sqr(rec(cos(x._value)));
        X dbl_tan_sqr_sec_val = 2*tan_val*sqr_sec_val;
        return SecondDifferential<X>( tan_val, sqr_sec_val*x._gradient, sqr_sec_val*x._half_hessian+dbl_tan_sqr_sec_val*outer(x._gradient), SeriesTag() );
    }

};









/*! \brief A class representing the derivatives of a vector quantity depending on multiple arguments. */
template<class X>
class Vector< SecondDifferential<X> >
    : public VectorExpression< Vector<SecondDifferential<X> > >
{
    //BOOST_CONCEPT_ASSERT((DifferentialVectorConcept<DifferentialVector<X> >));
    FixedDifferentialCharacteristics<X> _chars;
    Array< SecondDifferential<X> > _ary;
  public:
    // The type of the class
    typedef Vector< SecondDifferential<X> > SelfType;
    // The type used for accessing elements
    typedef SizeType IndexType;
    // The value stored in the vector.
    typedef SecondDifferential<X> ValueType;
    // The type used for scalars.
    typedef X ScalarType;

    template<class... PRS> requires Constructible<X,Nat,PRS...> Vector(SizeType rs, SizeType as, PRS... prs)
        : _chars(as,prs...), _ary(rs, SecondDifferential<X>(as,prs...)) { }
    Vector(SizeType rs, const SecondDifferential<X>& d) : _chars(d), _ary(rs,d) { }
    Vector(SizeType rs, const SecondDifferential<X>* p) : _chars(p[0]), _ary(rs,p) { }
    Vector(const Array<SecondDifferential<X>>& ary) : _chars(ary[0]), _ary(ary) { }
    template<class E> Vector(const VectorExpression<E>& ve);
    template<class E> Vector< SecondDifferential<X> >& operator=(const VectorExpression<E>& ve);


    SizeType result_size() const { return this->size(); }
    SizeType argument_size() const { return this->_chars.argument_size(); }
    DegreeType degree() const { return 1u; }
    SecondDifferential<X> zero_element() const {
        return SecondDifferential<X>(this->argument_size(),this->_chars.zero_coefficient()); }
    const Array<SecondDifferential<X>>& array() const { return _ary; }

    SizeType size() const { return this->_ary.size(); }
    const SecondDifferential<X>& get(SizeType i) const { return _ary[i]; }
    SecondDifferential<X>& at(SizeType i) { return _ary[i]; }
    Void set(SizeType i, const SecondDifferential<X>& d) const { _ary[i]=d; }
    const SecondDifferential<X>& operator[](SizeType i) const { return _ary[i]; }
    SecondDifferential<X>& operator[](SizeType i) { return _ary[i]; }

    VectorRange<const Vector<SecondDifferential<X>>> operator[](Range rng) const { return project(*this,rng); }

    Vector<X> value() const {
        Vector<X> r(this->result_size());
        for(SizeType i=0; i!=r.size(); ++i) { r[i]=(*this)[i].value(); } return r; }
    Matrix<X> jacobian() const { Matrix<X> r(this->result_size(),this->argument_size(),this->_chars.zero_coefficient());
        for(SizeType i=0; i!=r.row_size(); ++i) { for(SizeType j=0; j!=r.column_size(); ++j) { r[i][j]=(*this)[i].gradient(j); } } return r; }

    friend Vector<X> value(const Vector<SecondDifferential<X>>& x) { return x.value(); }
    friend Matrix<X> jacobian(const Vector<SecondDifferential<X>>& x) { return x.jacobian(); }
};


template<class X> inline FixedDifferentialCharacteristics<X>::FixedDifferentialCharacteristics(FirstDifferential<X> const& dx)
    : _argument_size(dx.argument_size()), _zero_coefficient(dx.zero_coefficient())
{ }

template<class X> inline FixedDifferentialCharacteristics<X>::FixedDifferentialCharacteristics(SecondDifferential<X> const& dx)
    : _argument_size(dx.argument_size()), _zero_coefficient(dx.zero_coefficient())
{ }


template<class X> class FixedDifferentialFactory {
    typedef PrecisionType<X> PR;
    PR _pr;
  public:
    FixedDifferentialFactory(PR const& pr) : _pr(pr) { }
    template<class Y> requires Constructible<X,Y,PR> X create(Y const& y) { return X(y,_pr); }
    template<class Y> FirstDifferential<X> create(FirstDifferential<Y> const&);
    template<class Y> SecondDifferential<X> create(SecondDifferential<Y> const&);
};
template<class X> inline FixedDifferentialFactory<X> differential_factory(X const& x) {
    return FixedDifferentialFactory<X>(x.precision()); }
template<class X> inline FixedDifferentialFactory<X> factory(FirstDifferential<X> const& dx) {
    return FixedDifferentialFactory<X>(dx.value().precision()); }
template<class X> inline FixedDifferentialFactory<X> factory(SecondDifferential<X> const& dx) {
    return FixedDifferentialFactory<X>(dx.value().precision()); }




} //namespace Ariadne

#endif // ARIADNE_FIXED_DIFFERENTIAL_HPP
