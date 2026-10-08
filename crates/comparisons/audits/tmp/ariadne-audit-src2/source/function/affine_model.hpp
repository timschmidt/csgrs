/***************************************************************************
 *            function/affine_model.hpp
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

/*! \file function/affine_model.hpp
 *  \brief Affine models defined on the unit box
 */

#ifndef ARIADNE_AFFINE_MODEL_HPP
#define ARIADNE_AFFINE_MODEL_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "utility/macros.hpp"
#include "utility/pointer.hpp"
#include "utility/declarations.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/operations.hpp"

namespace Ariadne {

template<class X> class Affine;

template<class P, class F> class AffineModel;

template<class P, class F> class TaylorModel;



//! An affine expression \f$f:\R^n\rightarrow\R\f$ given by \f$f(x) \approx \sum_{i=0}^{n-1} a_i x_i + b\f$.
template<class F>
class AffineModel<ApproximateTag,F>
    : DispatchAlgebraOperations<AffineModel<ApproximateTag,F>,FloatApproximation<typename F::PrecisionType>>
{
    typedef ApproximateTag P;
    typedef typename F::PrecisionType PR;
    friend struct AlgebraOperations<AffineModel<ApproximateTag,F>,FloatApproximation<PR>>;
  public:
    typedef P Paradigm;
    typedef PR PrecisionType;
    typedef Tuple<SizeType,PrecisionType> CharacteristicsType;
    typedef Approximation<F> NumericType;
    typedef Approximation<F> CoefficientType;
    typedef Interval<FloatApproximation<PR>> RangeType;

    explicit AffineModel(SizeType n, PrecisionType prec) : _c(0,prec), _g(n,CoefficientType(0,prec)) { }
    explicit AffineModel(SizeType n, const CoefficientType& c) : _c(c), _g(n,nul(c)) { }
    explicit AffineModel(const CoefficientType& c, const Covector<CoefficientType>& g) : _c(c), _g(g) { }
    explicit AffineModel(CoefficientType c, InitializerList<CoefficientType> g) : _c(c), _g(g) { }

    explicit AffineModel(const Affine<Approximation<F>>& affine);
    explicit AffineModel(const Affine<ApproximateNumber>& affine, PrecisionType precision);

    AffineModel(const BoxDomainType& domain, const ApproximateScalarMultivariateFunction& function, PrecisionType precision);
    AffineModel(const TaylorModel<ApproximateTag,F>&);

    AffineModel<ApproximateTag,F>& operator=(const CoefficientType& c) {
        this->_c=c; for(SizeType i=0; i!=this->_g.size(); ++i) { this->_g[i]=0.0; } return *this; }

    AffineModel<ApproximateTag,F> create_zero() const {
        return AffineModel<ApproximateTag,F>(this->argument_size(),this->precision()); }
    static AffineModel<ApproximateTag,F> constant(SizeType n, const CoefficientType& c) {
        AffineModel<ApproximateTag,F> res(n, c.precision()); res._c=c; return res; }
    static AffineModel<ApproximateTag,F> coordinate(SizeType n, SizeType j, PrecisionType pr) {
        AffineModel<ApproximateTag,F> res(n, pr); res._g[j]=1; return res; }
    static AffineModel<ApproximateTag,F> scaling(SizeType n, SizeType j, const IntervalDomainType& codom, PrecisionType precision);
    static Vector<AffineModel<ApproximateTag,F>> scalings(const BoxDomainType& codom, PrecisionType precision);

    SizeType argument_size() const { return this->_g.size(); }
    PrecisionType precision() const { return this->_c.precision(); }
    CharacteristicsType characteristics() const { return std::make_pair(this->argument_size(),this->_c.precision()); }
    const Covector<CoefficientType>& a() const { return this->_g; }
    const CoefficientType& b() const { return this->_c; }

    const CoefficientType& value() const { return this->_c; }
    const Covector<CoefficientType>& gradient() const { return this->_g; }
    const CoefficientType& gradient(SizeType i) const { return this->_g[i]; }
    CoefficientType& operator[](SizeType i) { return this->_g[i]; }
    const CoefficientType& operator[](SizeType i) const { return this->_g[i]; }

    RangeType range() const;

    Void set_value(const CoefficientType& c) { _c=c; }
    Void set_gradient(SizeType j, const CoefficientType& g) { _g[j]=g; }
    Void set_gradient(SizeType j, const ApproximateNumber& g) { _g[j]=g; }
    template<class X> X evaluate(const Vector<X>& v) const;
    friend OutputStream& operator<<(OutputStream& os, const AffineModel<ApproximateTag,F>& f) { return f._write(os); }
  private:
    OutputStream& _write(OutputStream&) const;
  private: public:
    static AffineModel<P,F> _compose(const ScalarMultivariateFunction<P>& f, Vector<AffineModel<P,F>> const& g);
    static Vector<AffineModel<P,F>> _compose(const VectorMultivariateFunction<P>& f, Vector<AffineModel<P,F>> const& g);
  private:
    CoefficientType _c;
    Covector<CoefficientType> _g;
};


//! An affine expression \f$f:[-1,+1]^n\rightarrow\R\f$ given by \f$f(x)=\sum_{i=0}^{n-1} a_i x_i + b \pm e\f$.
template<class F>
class AffineModel<ValidatedTag,F>
    : DispatchAlgebraOperations<AffineModel<ValidatedTag,F>,FloatBounds<typename F::PrecisionType>>
{
    typedef ValidatedTag P;
    typedef typename F::PrecisionType PR;
    friend struct AlgebraOperations<AffineModel<ValidatedTag,F>,FloatBounds<PR>>;
  public:
    typedef P Paradigm;
    typedef PR PrecisionType;
    typedef Tuple<SizeType,PrecisionType> CharacteristicsType;
    typedef Float<PR> CoefficientType;
    typedef FloatError<PR> ErrorType;
    typedef FloatBounds<PR> NumericType;
    typedef AffineModel<ValidatedTag,F> AffineModelType;
    typedef Interval<FloatUpperBound<PR>> RangeType;

    explicit AffineModel(SizeType n, PrecisionType prec) : _c(0,prec), _g(n,CoefficientType(0,prec)), _e(0u,prec) { }
    explicit AffineModel(SizeType n, const CoefficientType& c) : _c(c), _g(n,nul(c)), _e(nul(c)) { }
    explicit AffineModel(const CoefficientType& c, const Covector<CoefficientType>& g, const ErrorType& e) : _c(c), _g(g), _e(e) { }
    explicit AffineModel(CoefficientType c, InitializerList<CoefficientType> g) : _c(c), _g(g), _e(0u,c.precision()) { }

    explicit AffineModel(const Affine<FloatBounds<PR>>& affine);
    explicit AffineModel(const Affine<ValidatedNumber>& affine, PrecisionType precision);

    AffineModel(const TaylorModel<ValidatedTag,F>&);

    AffineModelType& operator=(const CoefficientType& c) {
        this->_c=c; for(SizeType i=0; i!=this->_g.size(); ++i) { this->_g[i]=0; } this->_e=0u; return *this; }

    CharacteristicsType characteristics() const { return std::make_pair(this->argument_size(),this->_c.precision()); }
    AffineModel<ValidatedTag,F> create_zero() const {
        return AffineModel<ValidatedTag,F>(this->argument_size(),this->precision()); }
    static AffineModel<ValidatedTag,F> constant(SizeType n, const CoefficientType& c) {
        AffineModel<ValidatedTag,F> res(n,c.precision()); res._c=c;  return res; }
    static AffineModel<ValidatedTag,F> coordinate(SizeType n, SizeType j, PrecisionType prec) {
        AffineModel<ValidatedTag,F> res(n,prec); res._g[j]=1;  return res; }

    static AffineModel<ValidatedTag,F> scaling(SizeType n, SizeType j, const IntervalDomainType& codom, PrecisionType precision);
    static Vector<AffineModel<ValidatedTag,F>> scalings(const BoxDomainType& codom, PrecisionType precision);

    SizeType argument_size() const { return this->_g.size(); }
    PrecisionType precision() const { return this->_c.precision(); }
    const Covector<CoefficientType>& a() const { return this->_g; }
    const CoefficientType& b() const { return this->_c; }
    const ErrorType& e() const { return this->_e; }

    const CoefficientType& value() const { return this->_c; }
    const Covector<CoefficientType>& gradient() const { return this->_g; }
    const CoefficientType& gradient(SizeType i) const { return this->_g[i]; }
    const ErrorType& error() const { return this->_e; }
    CoefficientType& value() { return this->_c; }
    CoefficientType& gradient(SizeType i) { return this->_g[i]; }
    ErrorType& error() { return this->_e; }
    CoefficientType& operator[](SizeType i) { return this->_g[i]; }
    const CoefficientType& operator[](SizeType i) const { return this->_g[i]; }

    RangeType range() const;

    Void set_value(const CoefficientType& c) { _c=c; }
    Void set_gradient(SizeType j, const CoefficientType& g) { _g[j]=g; }
    Void set_gradient(SizeType j, const Dyadic& g) { _g[j]=g; }
    Void set_error(const ErrorType& e) { _e=e; }
    Void set_error(Nat m) { _e=m; }

    template<class X> X evaluate(const Vector<X>& v) const;
    friend OutputStream& operator<<(OutputStream& os, const AffineModel<ValidatedTag,F>& f) { return f._write(os); }
  private:
    OutputStream& _write(OutputStream&) const;
  private: public:
    static AffineModel<P,F> _compose(const ScalarMultivariateFunction<P>& f, Vector<AffineModel<P,F>> const& g);
    static Vector<AffineModel<P,F>> _compose(const VectorMultivariateFunction<P>& f, Vector<AffineModel<P,F>> const& g);
  private:
    CoefficientType _c;
    Covector<CoefficientType> _g;
    ErrorType _e;
};

template<class F> template<class X> X AffineModel<ValidatedTag,F>::evaluate(const Vector<X>& v) const
{
    X r=v.zero_element()+static_cast<X>(this->_c);
    for(SizeType i=0; i!=this->_g.size(); ++i) {
        r+=X(this->_g[i])*v[i]; }
    r+=FloatBounds<PR>(-_e,+_e);
    return r;
}

template<class F> template<class X> X AffineModel<ApproximateTag,F>::evaluate(const Vector<X>& v) const
{
    X r=v.zero_element()+static_cast<X>(this->_c);
    for(SizeType i=0; i!=this->_g.size(); ++i) {
        r+=X(this->_g[i])*v[i]; }
    return r;
}

template<class P, class PR> inline AffineModel<P,RawFloatType<PR>> affine_model(const Affine<Number<P>>& affine, PR precision) {
    return AffineModel<P,RawFloatType<PR>>(affine,precision); }
template<class F> inline AffineModel<ApproximateTag,F> affine_model(const Affine<Approximation<F>>& affine) {
    return AffineModel<ApproximateTag,F>(affine); }
template<class F> inline AffineModel<ValidatedTag,F> affine_model(const Affine<Bounds<F>>& affine) {
    return AffineModel<ValidatedTag,F>(affine); }

template<class P, class F> inline AffineModel<P,F> affine_model(const TaylorModel<P,F>& taylor_model);
template<class P, class F> inline Vector<AffineModel<P,F>> affine_models(const Vector<TaylorModel<P,F>>& taylor_models);


// DEPRECATED
template<class P, class PR> AffineModel<P,RawFloatType<PR>> affine_model(const BoxDomainType& domain, const ScalarMultivariateFunction<P>& function, PR precision);
template<class P, class PR> inline Vector<AffineModel<P,RawFloatType<PR>>> affine_models(const BoxDomainType& domain, const VectorMultivariateFunction<P>& function, PR precision);

template<class PR> inline AffineModel<ValidatedTag,RawFloatType<PR>> affine_model(const BoxDomainType& domain, const ScalarMultivariateFunction<EffectiveTag>& function, PR precision) {
    return affine_model(domain,ScalarMultivariateFunction<ValidatedTag>(function),precision); }
template<class PR> inline Vector<AffineModel<ValidatedTag,RawFloatType<PR>>> affine_models(const BoxDomainType& domain, const VectorMultivariateFunction<EffectiveTag>& function, PR precision) {
    return affine_models(domain,VectorMultivariateFunction<ValidatedTag>(function),precision); }


template<class P, class F> inline AffineModel<P,F> affine_model(const TaylorModel<P,F>& taylor_model)
{
    return AffineModel<P,F>(taylor_model);
}

template<class P, class F> Vector<AffineModel<P,F>> affine_models(const Vector<TaylorModel<P,F>>& models)
{
    SizeType rs=models.size();
    SizeType as=models.zero_element().argument_size();
    auto zero=models.zero_element().value();

    Vector<AffineModel<ValidatedTag,F>> result(rs,AffineModel<ValidatedTag,F>(as,zero));
    for(SizeType i=0; i!=rs; ++i) { result[i]=affine_model(models[i]); }
    return result;
}

template<class P, class PR> Vector<AffineModel<P,RawFloatType<PR>>> affine_models(const BoxDomainType& domain, const VectorMultivariateFunction<P>& function, PR precision)
{
    typedef RawFloat<PR> F;
    SizeType rs=function.result_size();
    SizeType as=function.argument_size();

    Vector<AffineModel<P,F>> result(rs,AffineModel<P,F>(as,precision));
    for(SizeType i=0; i!=rs; ++i) { result[i]=affine_model(domain,function[i],precision); }
    return result;
}

template<class P, class F> Vector<typename AffineModel<P,F>::RangeType> ranges(const Vector<AffineModel<P,F>>& f) {
    return Vector<typename AffineModel<P,F>::RangeType>(f.size(), [&](SizeType i){return f[i].range();});
    //Vector<typename AffineModel<P,F>::RangeType> r(f.size(),f.zero_element().range()); for(SizeType i=0; i!=f.size(); ++i) { r[i]=f[i].range(); } return r;
}

template<class P, class F> Vector<AffineModel<P,F>> compose(const VectorMultivariateFunction<P>& f, Vector<AffineModel<P,F>> const& g) {
    return AffineModel<P,F>::_compose(f,g);
}

template<class P, class F> AffineModel<P,F> compose(const ScalarMultivariateFunction<P>& f, Vector<AffineModel<P,F>> const& g) {
    return AffineModel<P,F>::_compose(f,g);
}


} // namespace Ariadne

#endif /* ARIADNE_AFFINE_MODEL_HPP */
