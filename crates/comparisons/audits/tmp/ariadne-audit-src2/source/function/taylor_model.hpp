/***************************************************************************
 *            function/taylor_model.hpp
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

/*! \file function/taylor_model.hpp
 *  \brief ApproximateTag functions on a is_bounded domain with a sparse representation.
 */

#ifndef ARIADNE_TAYLOR_MODEL_HPP
#define ARIADNE_TAYLOR_MODEL_HPP

#include <map>

#include "utility/macros.hpp"
#include "utility/declarations.hpp"
#include "utility/array.hpp"
#include "utility/pointer.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/expansion.hpp"
#include "algebra/sweeper.hpp"
#include "algebra/operations.hpp"
#include "algebra/evaluate.hpp"
#include "function/domain.hpp"
#include "function/scaling.hpp"
#include "function/polynomial.hpp"

namespace Ariadne {

template<class PR> Bool operator==(Sweeper<PR> const&, Sweeper<PR> const&) { return true; }

template<class F> class ZeroError {
  public:
    template<class... PRS> requires Constructible<Error<F>,PRS...> ZeroError(PRS...) { }
    F raw() const;
    typename F::PrecisionType precision() const;
    operator Error<F> () const;
};

template<class F> class UnknownError {
  public:
//    template<class... PRS> requires Constructible<Error<F>,PRS...> UnknownError(PRS...) { }
    template<class... PRS> UnknownError(PRS...) { }
    F raw() const;
    typename F::PrecisionType precision() const;
    operator PositiveApproximation<F> () const;
};

template<class T1, class T2> struct Product;

template<class P, class F> class TaylorModel;

//! \relates TaylorModel
//! \name Template shorthands and type synonyms for Taylor models
//!@{
template<class F> using ValidatedTaylorModel = TaylorModel<ValidatedTag,F>; //!< <p/>
template<class F> using ValidatedBoundsTaylorModel = TaylorModel<ValidatedTag,Bounds<F>>; //!< <p/>
template<class F> using ValidatedIntervalTaylorModel = TaylorModel<ValidatedTag,UpperInterval<F>>; //!< <p/>
template<class F> using ApproximateTaylorModel = TaylorModel<ApproximateTag,F>; //!< <p/>
using ValidatedTaylorModelDP = TaylorModel<ValidatedTag,FloatDP>; //!< <p/>
using ValidatedTaylorModelMP = TaylorModel<ValidatedTag,FloatMP>; //!< <p/>
using ValidatedBoundsTaylorModelDP = TaylorModel<ValidatedTag,FloatDPBounds>; //!< <p/>
using ValidatedBoundsTaylorModelMP = TaylorModel<ValidatedTag,FloatMPBounds>; //!< <p/>
using ValidatedIntervalTaylorModelDP = TaylorModel<ValidatedTag,FloatDPUpperInterval>; //!< <p/>
using ValidatedIntervalTaylorModelMP = TaylorModel<ValidatedTag,FloatMPUpperInterval>; //!< <p/>
using ApproximateTaylorModelDP = TaylorModel<ApproximateTag,FloatDP>; //!< <p/>
using ApproximateTaylorModelMP = TaylorModel<ApproximateTag,FloatMP>; //!< <p/>
//!@}

template<class P, class F> struct IsScalar< TaylorModel<P,F> > { static const Bool value = true; };

class IntersectionException;

class IntersectionException : public std::runtime_error {
  public:
    IntersectionException(const StringType& what) : std::runtime_error(what) { }
};

template<class P, class F> struct ModelNumericTraits;
template<class F> struct ModelNumericTraits<ValidatedTag,Interval<UpperBound<F>>>
    : public FunctionModelTraits<ValidatedTag,PrecisionType<F>>
{
    typedef Interval<UpperBound<F>> CoefficientType;
    typedef Interval<UpperBound<F>> NumericType;
//    typedef Bounds<F> NumericType;
    typedef F RawFloatType;
};
template<class F> struct ModelNumericTraits<ValidatedTag,Bounds<F>>
    : public FunctionModelTraits<ValidatedTag,PrecisionType<F>>
{
    typedef Bounds<F> CoefficientType;
    typedef Bounds<F> NumericType;
    typedef F RawFloatType;
};
template<class F> struct ModelNumericTraits<ValidatedTag,F>
    : public FunctionModelTraits<ValidatedTag,PrecisionType<F>>
{
    typedef F CoefficientType;
};
template<class F> struct ModelNumericTraits<ApproximateTag,F>
    : public FunctionModelTraits<ApproximateTag,PrecisionType<F>>
{
    typedef Approximation<F> CoefficientType;
};

template<class P, class F> using ModelNumericType = typename ModelNumericTraits<P,F>::NumericType;


template<class P, class F> struct ModelNumericTypedef;
template<class F> struct ModelNumericTypedef<ValidatedTag,UpperInterval<F>> { typedef UpperInterval<F> Type; };
template<class F> struct ModelNumericTypedef<ValidatedTag,F> { typedef FloatBounds<typename F::PrecisionType> Type; };
template<class F> struct ModelNumericTypedef<ApproximateTag,F> { typedef FloatApproximation<typename F::PrecisionType> Type; };

template<class P, class F> struct AlgebraOperations<TaylorModel<P,F>>
    : NormedAlgebraOperations<TaylorModel<P,F>>
{
    typedef ModelNumericType<P,F> X;
    typedef TaylorModel<P,F> ModelType;
    typedef ModelNumericType<P,F> NumericType;
    using NormedAlgebraOperations<TaylorModel<P,F>>::apply;
    static TaylorModel<P,F> apply(Nul,TaylorModel<P,F> const& tm);
    static TaylorModel<P,F> apply(Pos,TaylorModel<P,F> tm);
    static TaylorModel<P,F> apply(Neg,TaylorModel<P,F> tm);
    static TaylorModel<P,F> apply(Add,TaylorModel<P,F> tm, X const& c);
    static TaylorModel<P,F> apply(Mul,TaylorModel<P,F> tm, X const& c);
    static TaylorModel<P,F> apply(Add,TaylorModel<P,F> const& tm1, TaylorModel<P,F> const& tm2);
    static TaylorModel<P,F> apply(Sub,TaylorModel<P,F> const& tm1, TaylorModel<P,F> const& tm2);
    static TaylorModel<P,F> apply(Mul,TaylorModel<P,F> const& tm1, TaylorModel<P,F> const& tm2);
    static TaylorModel<P,F> apply(Min,TaylorModel<P,F> const& tm1, TaylorModel<P,F> const& tm2);
    static TaylorModel<P,F> apply(Max,TaylorModel<P,F> const& tm1, TaylorModel<P,F> const& tm2);
    static TaylorModel<P,F> apply(Max,TaylorModel<P,F> const& tm, X const& c);
    static TaylorModel<P,F> apply(Min,TaylorModel<P,F> const& tm, X const& c);
    static TaylorModel<P,F> apply(Max,X const& c, TaylorModel<P,F> const& tm);
    static TaylorModel<P,F> apply(Min,X const& c, TaylorModel<P,F> const& tm);
    static TaylorModel<P,F> apply(Abs,TaylorModel<P,F> const& tm);
};

template<class P, class F> class TaylorModel;

//! \ingroup FunctionModelSubModule
//! \brief A class representing polynomial approximation to a function, scaled to the unit box, with a uniform error bound.
//!  \tparam P	The information paradigm tag, which can be either ValidatedTag, indicating that the approximation has a known (uniform) error bound, or ApproximateTag, indicating that no error bound is available. See the \ref InformationSubModule for more details.
//!  \tparam F  The (floating-point) number type used internally for the coefficients of the polynomial, such as \ref FloatDP or \ref FloatMP.
//!  \tparam FE  The (floating-point) number type used internally for the error bound.
//!
//! \see Expansion, ValidatedScalarMultivariateTaylorFunctionModelDP, ValidatedVectorMultivariateTaylorFunctionModelDP, TaylorConstrainedImageSet.
template<class P, class F>
class TaylorModel
    : public DispatchElementaryAlgebraOperations<TaylorModel<P,F>,typename ModelNumericTraits<P,F>::NumericType>
    , public DispatchConcreteGenericAlgebraNumberOperations<TaylorModel<P,F>,typename ModelNumericTraits<P,F>::NumericType,Number<P>>
{
    typedef typename F::PrecisionType PR;
    typedef typename F::PrecisionType PRE;
    typedef typename ModelNumericTraits<P,F>::NumericType X;
  public:
    typedef PR PrecisionType;
    typedef PRE ErrorPrecisionType;

    typedef typename ModelNumericTraits<P,F>::RawFloatType RawFloatType;
    typedef typename ModelNumericTraits<P,F>::CoefficientType CoefficientType;
    typedef typename ModelNumericTraits<P,F>::ValueType ValueType;
    typedef typename ModelNumericTraits<P,F>::ErrorType ErrorType;
    typedef typename ModelNumericTraits<P,F>::NormType NormType;
    typedef ReverseLexicographicIndexLess ComparisonType;
    typedef SortedExpansion<MultiIndex,CoefficientType,ComparisonType> ExpansionType;
    typedef Polynomial<MultiIndex,CoefficientType> PolynomialType;
    typedef Sweeper<RawFloatType> SweeperType;


    typedef typename ModelNumericTraits<P,F>::RangeType RangeType;
    typedef IntervalDomainType CodomainType;

    template<class X> using Argument = Vector<X>;
    template<class X> using Result = Scalar<X>;

    //! \brief The computational paradigm.
    typedef ValidatedTag Paradigm;
    //! \brief The properties needed to define the TaylorModel calculus.
    typedef Sweeper<RawFloatType> PropertiesType;
    //! \brief The characteristics of a zero TaylorModel.
    typedef Tuple<SizeType,Sweeper<RawFloatType>> CharacteristicsType;

    //! \brief The type used for algebraic operations.
    typedef typename ModelNumericTraits<P,F>::NumericType NumericType;
    typedef Number<P> GenericNumericType;

    typedef Interval<FloatUpperBound<PR>> IntervalNumericType;
    typedef FloatBounds<PR> ValidatedNumericType;
    typedef FloatApproximation<PR> ApproximateNumericType;

//    typedef decltype(std::declval<NumericType>() < std::declval<NumericType>()) ComparisonType;

    typedef ValidatedScalarMultivariateFunction FunctionType;
    typedef ValidatedScalarMultivariateFunction ScalarFunctionType;
    typedef ValidatedVectorMultivariateFunction VectorFunctionType;

    //! \brief The type used to index the coefficients.
    typedef MultiIndex IndexType;

    //! \brief An Iterator through the (index,coefficient) pairs of the expansion.
    typedef typename ExpansionType::Iterator Iterator;
    //! \brief A constant Iterator through the (index,coefficient) pairs of the expansion.
    typedef typename ExpansionType::ConstIterator ConstIterator;
  private:
    ExpansionType _expansion;
    ErrorType _error;
    mutable SweeperType _sweeper;
  public:
    //! \name Constructors and destructors.
    //!@{
    //
    //! \brief Construct a TaylorModel in \a as arguments with the given accuracy control.
    TaylorModel(SizeType as, SweeperType swp);
    //! \brief Construct from a map giving the expansion, a constant giving the error, and an accuracy parameter.
    TaylorModel(const Expansion<MultiIndex,ExactDouble>& f, const ExactDouble& e, SweeperType swp);
    TaylorModel(const Expansion<MultiIndex,CoefficientType>& f, const ErrorType& e, SweeperType swp);
    TaylorModel(const Expansion<MultiIndex,RawFloatType>& f, const RawFloatType& e, SweeperType swp);
    //! \brief Fast swap with another Taylor model.
    Void swap(TaylorModel<P,F>& tm);
    //! \brief The zero element of the algebra of Taylor models, with the same number of arguments and accuracy parameters.
    TaylorModel<P,F> create() const;
    //! \brief The zero element of the algebra of Taylor models, with the same number of arguments and accuracy parameters.
    TaylorModel<P,F> create_zero() const;
    //! \brief A constant element of the algebra of Taylor models, with the same number of arguments and accuracy parameters.
    TaylorModel<P,F> create_constant(NumericType c) const;
    TaylorModel<P,F> create_constant(GenericNumericType) const;
    //! \brief The \a j<sup>th</sup> coordinate element of the algebra of Taylor models, with the same number of arguments and accuracy parameters.
    TaylorModel<P,F> create_coordinate(SizeType j) const;
    //! \brief Set to zero.
    TaylorModel<P,F> create_ball(ErrorType e) const;
    //! \brief Set to zero.
    Void clear();
    //!@}

    //! \name Assignment to constant values.
    //!@{
    //
    //! \brief Set equal to an interval constant, keeping the same number of arguments.
    TaylorModel<P,F>& operator=(const NumericType& c);
    TaylorModel<P,F>& operator=(const GenericNumericType& c);
    //!@}

    //! \name Named constructors.
    //!@{
    //
    //! \brief Construct the zero quantity in \a as independent variables.
    static TaylorModel<P,F> zero(SizeType as, SweeperType swp) {
        TaylorModel<P,F> r(as,swp); return r; }
    //! \brief Construct a constant quantity in \a as independent variables.
    static TaylorModel<P,F> constant(SizeType as, const NumericType& c, SweeperType swp);
    //! \brief Construct a constant quantity in \a as independent variables.
    static TaylorModel<P,F> constant(SizeType as, const GenericNumericType& c, SweeperType swp);
    //! \brief Construct the quantity with expansion \f$x_j\f$ in \a as independent variables.
    static TaylorModel<P,F> coordinate(SizeType as, SizeType j, SweeperType swp) {
        TaylorModel<P,F> r(as,swp); r.set_gradient(j,1); return r; }
    //! \brief Construct a constant quantity in \a as independent variables with value zero and uniform error \a e
    static TaylorModel<P,F> error(SizeType as, ErrorType e, SweeperType swp) {
        TaylorModel<P,F> r(as,swp); r.set_error(e); return r; }
    //! \brief Construct a constant quantity in \a as independent variables with value zero and uniform error \a e
    static TaylorModel<P,F> ball(SizeType as, ErrorType e, SweeperType swp) {
        TaylorModel<P,F> r(as,swp); r.set_error(e); return r; }
    //! \brief Construct a constant quantity in \a as independent variables with value zero and uniform error \a e
    static TaylorModel<P,F> unit_ball(SizeType as, SweeperType swp) {
        TaylorModel<P,F> r(as,swp); r.set_error(1u); return r; }

    //! \brief Construct the quantity which scales the interval \a codom onto the unit interval.
    static TaylorModel<P,F> scaling(SizeType as, SizeType j, const IntervalDomainType& codom, SweeperType swp);

    //! \brief Return the vector of zero variables of size \a rs in \a as arguments.
    static Vector<TaylorModel<P,F>> zeros(SizeType rs, SizeType as, SweeperType swp);
    //! \brief Return the vector of constants with values \a c in \a as arguments.
    static Vector<TaylorModel<P,F>> constants(SizeType as, const Vector<NumericType>& c, SweeperType swp);
    //! \brief Return the vector of variables on the unit domain.
    static Vector<TaylorModel<P,F>> coordinates(SizeType as, SweeperType swp);

    //! \brief Return the vector scaling the box \a codom onto the unit box.
    static Vector<TaylorModel<P,F>> scalings(const BoxDomainType& codom, SweeperType swp);
    //!@}

    //! \name Comparison operators.
    //!@{
    //
    //! \brief Equality operator. Tests equality of representation, including error term.
    friend Bool same(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2) {
        return same(tm1._expansion, tm2._expansion) && same(tm1._error, tm2._error); }

    decltype(auto) operator<(const TaylorModel<P,F>& sd) const {
        return (sd-*this)>0; }
    //! \brief Comparison with another Taylor model.
    decltype(auto) operator>(const TaylorModel<P,F>& sd) const {
        return (*this-sd)>0; }

    //! \brief Comparison with a scalar.
    decltype(auto) operator<(Int c) const {
        return this->range().upper_bound()<c; }
    //! \brief Comparison with a scalar.
    decltype(auto) operator>(Int c) const {
        return this->range().lower_bound()>c; }
    //!@}

    //! \name Data access
    //!@{
    //
    //! \brief The expansion.
    const ExpansionType& expansion() const { return this->_expansion; }
    //! \brief A reference to the expansion.
    ExpansionType& expansion() { return this->_expansion; }
    //! \brief The error of the expansion over the domain.
    const ErrorType& error() const { return this->_error; }
    //! \brief A reference to the error of the expansion over the domain.
    ErrorType& error() { return this->_error; }

    //! \brief The constant term in the expansion.
    const ValueType value() const { return this->average(); }
//    //! \brief The coefficient of the gradient term \f$df/dx_j\f$.
//    const ValueType gradient_value(SizeType j) const ;//{ return (*this)[MultiIndex::unit(this->argument_size(),j)]; }
//    Covector<ValueType> gradient_value() const { Covector<ValueType> r(this->argument_size());
//        for(SizeType j=0; j!=this->argument_size(); ++j) { r[j]=this->gradient_value(j); } return r; }

    //! \brief The constant term in the expansion.
    ValueType average() const;
    //! \brief The radius of the smallest ball about a constant function containing the model.
    NormType radius() const;
    //! \brief An over-approximation to the supremum norm.
    NormType norm() const;
    //! \brief An over-approximation to the supremum norm.
    friend typename TaylorModel<P,F>::NormType norm(const TaylorModel<P,F>& tm) { return tm.norm(); }
    friend typename TaylorModel<P,F>::NormType mag(const TaylorModel<P,F>& tm) { return tm.norm(); }


    //! \brief A value \c e such that analytic functions are evaluated to a tolerance of \c e. Equal to the sweep threshold.
    RawFloatType tolerance() const;


    //! \brief Set the constant term in the expansion.
    Void set_value(const CoefficientType& c) {
        this->_expansion.set(MultiIndex::zero(this->argument_size()),c); }
    //! \brief Set the coefficient of the term \f$df/dx_j\f$.
    Void set_gradient(SizeType j, const CoefficientType& c) {
        this->_expansion.set(MultiIndex::unit(this->argument_size(),j),c); }
    Void set_gradient(SizeType j,const Dyadic& c) {
        this->set_gradient(j,CoefficientType(c,this->precision())); }
     //! \brief Set the error of the expansion.
    Void set_error(const ErrorType& ne) { ARIADNE_ASSERT(ne.raw()>=0.0_x); this->_error=ne; }
    Void set_error(Nat m) { this->_error=m; }

    //! \brief The coefficient of the term in $x^a$.
    const CoefficientType& operator[](const MultiIndex& a) const { return this->_expansion[a]; }
    //! \brief A read/write reference to the coefficient of the term in $x^a$.
    CoefficientType& operator[](const MultiIndex& a) { return this->_expansion.at(a); }

    //! \brief An Iterator to the first term in the expansion.
    Iterator begin() { return this->_expansion.begin(); }
    //! \brief A constant Iterator to the first term in the expansion.
    ConstIterator begin() const { return this->_expansion.begin(); }
    //! \brief An Iterator to the end of the expansion.
    Iterator end() { return this->_expansion.end(); }
    //! \brief A constant Iterator to the end of the expansion.
    ConstIterator end() const { return this->_expansion.end(); }

    //! \brief The number of variables in the argument of the quantity.
    SizeType argument_size() const { return this->_expansion.argument_size(); }
    //! \brief The maximum degree of terms in the expansion.
    DegreeType degree() const;
    //! \brief The number of nonzero terms in the expansion.
    SizeType number_of_terms() const { return this->_expansion.number_of_terms(); }
    SizeType number_of_nonzeros() const { return this->_expansion.number_of_nonzeros(); }
    //!@}

    //! \name Function evaluation.
    //!@{
    //
    //! \brief The domain of the quantity, always given by \f$[-1,1]^{\mathrm{as}}\f$.
    UnitBox domain() const;
    //! \brief The codomain of the quantity.
    IntervalDomainType codomain() const;
    //! \brief An over-approximation to the range of the quantity.
    RangeType range() const;

    //! \brief Evaluate the quantity over the interval of points \a x.
    friend ArithmeticType<CoefficientType,ApproximateNumericType> evaluate(const TaylorModel<P,F>& f, const Vector<ApproximateNumericType>& x) {
        return TaylorModel<P,F>::_evaluate(f,x); }
    friend ArithmeticType<CoefficientType,ValidatedNumericType> evaluate(const TaylorModel<P,F>& f, const Vector<ValidatedNumericType>& x) {
        return TaylorModel<P,F>::_evaluate(f,x); }
    template<class X=NumericType> requires (not Same<X,ValidatedNumericType>) friend ArithmeticType<CoefficientType,ValidatedNumericType> evaluate(const TaylorModel<P,F>& f, const Vector<X>& x) {
        return TaylorModel<P,F>::_evaluate(f,x); }
    //! \brief Evaluate the gradient over the interval of points \a x.
    friend Covector<NumericType> gradient(const TaylorModel<P,F>& f, const Vector<NumericType>& x) {
        return TaylorModel<P,F>::_gradient(f,x); }
    //! \brief Substite \a c for the \a k th variable.
    friend TaylorModel<P,F> partial_evaluate(const TaylorModel<P,F>& x, SizeType k, NumericType c) {
        return TaylorModel<P,F>::_partial_evaluate(x,k,c); }
    //! \relates TaylorModel<P,F Compose a vector of Taylor models with another.
    friend TaylorModel<P,F> compose(const Unscaling& uf, const TaylorModel<P,F>& tg) {
        return TaylorModel<P,F>::_compose(uf,tg); }
    //! \brief Evaluate the quantity over the interval of points \a x.
    friend TaylorModel<P,F> compose(const TaylorModel<P,F>& f, const Vector<TaylorModel<P,F>>& g) {
        return TaylorModel<P,F>::_compose(f,g); }
    //! \brief Scale the variable by post-composing with an affine map taking the interval ivl to the unit interval
    friend TaylorModel<P,F> compose(const TaylorModel<P,F>& tf, const VectorUnscaling& u);

    friend TaylorModel<P,F> evaluate(const TaylorModel<P,F>& f, const Vector<TaylorModel<P,F>>& g) { return compose(f,g); }
    template<class A> ArithmeticType<CoefficientType,A> operator() (Vector<A> const&) const;
    //!@}

    //! \name Inplace modifications.
    //!@{
    //
    //! \brief Scales the model by a function mapping \a dom into the unit interval.
    Void unscale(IntervalDomainType const& dom);
    //! \brief Compute the antiderivative (in place).
    Void antidifferentiate(SizeType k);
    //! \brief Compute the weak derivative (in place).
    Void differentiate(SizeType k);
    //! \brief Estimate the gradient at a point.
    friend Covector<X> gradient(const TaylorModel<P,F>& x, const Vector<X>& v);
//    Covector<X> gradient(Vector<X> const& v) const;

    friend TaylorModel<P,F> unscale(TaylorModel<P,F> tm, IntervalDomainType const& dom) {
        tm.unscale(dom); return tm; }
    friend TaylorModel<P,F> antiderivative(TaylorModel<P,F> tm, SizeType k) {
        tm.antidifferentiate(k); return tm; }
    friend TaylorModel<P,F> derivative(TaylorModel<P,F> tm, SizeType k) {
        tm.differentiate(k); return tm; }
    //!@}

    //! \name Operations on the domain.
    //!@{
    //
    //! \brief Split the Taylor model \a tm, subdividing along the independent variable \a k, taking the lower/middle/upper \a part.
    friend TaylorModel<P,F> split(const TaylorModel<P,F>& tm, SizeType k, SplitPart part) {
        return TaylorModel<P,F>::_split(tm,k,part); }
    //! \brief Split the Taylor model \a tm, subdividing along the independent variable \a k.
    friend Pair<TaylorModel<P,F>,TaylorModel<P,F>> split(const TaylorModel<P,F>& tm, SizeType k) {
        return make_pair(split(tm,k,SplitPart::LOWER),split(tm,k,SplitPart::UPPER)); }
    //! \relates TaylorModel<P,F> \brief Embed the model in a space of higher dimension
    friend TaylorModel<P,F> embed(SizeType as1, const TaylorModel<P,F>& tm2, SizeType as3) {
        return TaylorModel<P,F>::_embed(as1,tm2,as3); }
    //!@}

    //! \name Paradigm-based operations.
    //!@{
    //
    //! \brief Test if two models are compatible with each other, in the sense that they could both represent the same function.
    //! Equivalent to checking that the two sets of possible functions intersect.
    //! If the result is \a true, then the models are definitely consistent; may return \a false even if the models are inconsistent.
    friend Bool consistent(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2) {
        return TaylorModel<P,F>::_consistent(tm1,tm2); }
    //! \brief Test if one model is incompatible with another.
    //! Equivalent to checking that the two sets of possible functions are disjoint.
    //! If the result is \a true, then the models are definitely inconsistent; may return \a false even if the models are inconsistent.
    friend Bool inconsistent(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2) {
        return TaylorModel<P,F>::_inconsistent(tm1,tm2); }
    //! \brief Test if one model refines another, in the sense that it represents a subset of possible functions.
    //! If the result is \a true, then the first model definitely refines the second;
    //! may return \a false even if the first model actually does refine the second.
    friend Bool refines(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2) {
        return TaylorModel<P,F>::_refines(tm1,tm2); }
    //! \brief An over-approximation to the common refinement of two Taylor models.
    //! Since the intersection of represented sets of functions cannot be represented
    //! exactly in the class of TaylorModels, truncation errors as well as roundoff errors
    //! may be present. In the absence of roundoff errors, the result is a subset of both
    //! arguments, and is guaranteed to contain any function contained in both arguments.
    friend TaylorModel<P,F> refinement(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2) {
        return TaylorModel<P,F>::_refinement(tm1,tm2); }
    //!@}

    //! \name Simplification operations.
    //!@{
    //
    //! \brief Embed the model in a space of higher dimension, placing the error in the final variable.
    friend TaylorModel<P,F> embed_error(const TaylorModel<P,F>& tm) {
        return TaylorModel<P,F>::_embed_error(tm); }
    //! \relates TaylorModel<P,F> \brief Abstract away the given variables.
    //! For example, the model tm(x0,x1,x2,x3) becomes tm'(x0,x1)=tm(x0,[-1,+1],x1,[-1,+1]) on discarding x1 and x3.
    friend TaylorModel<P,F>  discard_variables(const TaylorModel<P,F>& tm, const Array<SizeType>& variables) {
        return TaylorModel<P,F>::_discard_variables(tm,variables); }

    //! \brief Remove all terms based on the \a swp conditions.
    TaylorModel<P,F>& sweep(const SweeperType& swp);
    TaylorModel<P,F>& simplify(const SweeperType& prp);

    //! \brief Remove all terms whose coefficient has magnitude
    //! lower than the cutoff threshold of the quantity.
    TaylorModel<P,F>& sweep();
    TaylorModel<P,F>& simplify();
    //! \brief Sorts keys.
    TaylorModel<P,F>& sort();
    //! \brief Remove terms with the same keys. Assumes sorted.
    TaylorModel<P,F>& unique();
    //! \brief Sort the terms in index order and combine terms with the same index.
    TaylorModel<P,F>& cleanup();
    //! \brief Set the error to zero.
    //! WARNING: This method does not preserve rigour of the model approximation.
    TaylorModel<P,F>& clobber();
    //!@}

    //! \name Accuracy parameters.
    //!@{
    //
    //! \brief Specify a policy to use to remove low-impact terms.
    Void set_sweeper(SweeperType swp) { this->_sweeper=swp; }
    Void set_properties(PropertiesType prp) { this->_sweeper=prp; }
    //! \brief A shared pointer to an object using for removing low-impact terms.
    SweeperType sweeper() const { return this->_sweeper; }
    //! \brief The numerical properties of the calculus.
    PropertiesType properties() const { return this->sweeper(); }
    //! \brief The characteristics of a zero value.
    CharacteristicsType characteristics() const { return std::make_pair(this->argument_size(),this->sweeper()); }
    //! \brief The properties of the zero element.
    //! \brief The precision of the coefficients.
    PrecisionType precision() const { return this->sweeper().precision(); }
    //!@}

    //! \name Order operators.
    //!@{
    //
    //! \brief The pointwise maximum.
    template<class FF> friend TaylorModel<P,FF> max(const TaylorModel<P,FF>& x, const TaylorModel<P,FF>& y);
    //! \brief The pointwise minimum.
    template<class FF> friend TaylorModel<P,FF> min(const TaylorModel<P,FF>& x, const TaylorModel<P,FF>& y);
    //! \brief The pointwise absolute value.
    //! \details If the range of \a x definitely does not include 0, returns +x or -x. Otherwise, uses a uniform polynomial approximation to abs.
    template<class FF> friend TaylorModel<P,FF> abs(const TaylorModel<P,FF>& x);
    //!@}

    //! \name Stream input/output operators.
    //!@{
    //
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, const TaylorModel<P,F>& x) { return x.str(os); }
    //!@}

  public:
    OutputStream& str(OutputStream&) const;
    OutputStream& repr(OutputStream&) const;
  public: // FIXME: Should be private
    Void _set_error(const RawFloat<PR>& ne) { ARIADNE_ASSERT(ne>=0); this->_error=ErrorType(ne); }
    Void _append(MultiIndex const& a, CoefficientType const& v) { this->_expansion.append(a,v); }
    static Bool _consistent(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2);
    static Bool _inconsistent(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2);
    static Bool _refines(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2);
    static TaylorModel<P,F> _refinement(const TaylorModel<P,F>& tm1, const TaylorModel<P,F>& tm2);
    static TaylorModel<P,F> _antiderivative(const TaylorModel<P,F>& tm, SizeType k);
    static TaylorModel<P,F> _weak_derivative(const TaylorModel<P,F>& tm, SizeType k);
    static TaylorModel<P,F> _embed_error(const TaylorModel<P,F>& tm);
    static TaylorModel<P,F>  _discard_variables(const TaylorModel<P,F>&, const Array<SizeType>& variables);
    static TaylorModel<P,F> _split(const TaylorModel<P,F>& tm, SizeType k, SplitPart part);
    static TaylorModel<P,F> _embed(SizeType as1, const TaylorModel<P,F>& tm2, SizeType as3);
    static TaylorModel<P,F> _compose(TaylorModel<P,F> const& tf, const Vector<TaylorModel<P,F>>& tg);
    static TaylorModel<P,F> _compose(Unscaling const& uf, TaylorModel<P,F> const& tg);
    static TaylorModel<P,F> _compose(TaylorModel<P,F> const& tf, VectorUnscaling const& u, const Vector<TaylorModel<P,F>>& tg);
    static TaylorModel<P,F> _partial_evaluate(const TaylorModel<P,F>& x, SizeType k, NumericType c);
    static Covector<X> _gradient(const TaylorModel<P,F>& x, Vector<X> const& v);
    static ArithmeticType<CoefficientType,IntervalNumericType> _evaluate(const TaylorModel<P,F>& x, Vector<IntervalNumericType> const& v);
    static ArithmeticType<CoefficientType,ValidatedNumericType> _evaluate(const TaylorModel<P,F>& x, Vector<ValidatedNumericType> const& v);
    static ArithmeticType<CoefficientType,ApproximateNumericType> _evaluate(const TaylorModel<P,F>& x, Vector<ApproximateNumericType> const& v);
};

// FIXME: Needed to dispatch gradient of ScaledFunctionPatch
template<class P, class F> template<class A> auto TaylorModel<P,F>::operator() (Vector<A> const& x) const -> ArithmeticType<CoefficientType,A> {
    return horner_evaluate(this->expansion(),x)+FloatBounds<typename F::PrecisionType>(-this->error(),+this->error());
}



template<class F> Bool inconsistent(const Vector<TaylorModel<ValidatedTag,F>>& tm1, const Vector<TaylorModel<ValidatedTag,F>>& tm2) {
    ARIADNE_ASSERT(tm1.size()==tm2.size());
    for(SizeType i=0; i!=tm1.size(); ++i) { if(inconsistent(tm1[i],tm2[i])) { return true; } }
    return false;
}

template<class F> Bool refines(const Vector<TaylorModel<ValidatedTag,F>>& tm1, const Vector<TaylorModel<ValidatedTag,F>>& tm2) {
    ARIADNE_ASSERT(tm1.size()==tm2.size());
    for(SizeType i=0; i!=tm1.size(); ++i) { if(not refines(tm1[i],tm2[i])) { return false; } }
    return true;
}

template<class F> Vector<TaylorModel<ValidatedTag,F>> refinement(const Vector<TaylorModel<ValidatedTag,F>>& tm1, const Vector<TaylorModel<ValidatedTag,F>>& tm2) {
    ARIADNE_ASSERT(tm1.size()==tm2.size());
    Vector<TaylorModel<ValidatedTag,F>> r(tm1.size());
    for(SizeType i=0; i!=tm1.size(); ++i) { r[i]=refinement(tm1[i],tm2[i]); }
    return r;
}





template<class P, class F, class X> decltype(auto) evaluate(const Vector<TaylorModel<P,F>>& tf, const Vector<X>& x)
{
    typedef decltype(evaluate(tf[0],x)) R;
    return Vector<R>(tf.size(),[&](SizeType i){return evaluate(tf[i],x);},characteristics(evaluate(tf.zero_element(),x)));
}

template<class F> Vector<TaylorModel<ValidatedTag,F>> partial_evaluate(const Vector<TaylorModel<ValidatedTag,F>>& tf, SizeType k, const FloatBounds<PrecisionType<F>>& c) {
    Vector<TaylorModel<ValidatedTag,F>> r(tf.size(),ValidatedTaylorModel<F>::zero(tf.zero_element().argument_size()-1u,tf.zero_element().sweeper()));
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=partial_evaluate(tf[i],k,c); }
    return r;
}

template<class F> Vector<TaylorModel<ValidatedTag,F>> compose(const Vector<TaylorModel<ValidatedTag,F>>& tf, const Vector<TaylorModel<ValidatedTag,F>>& tg) {
    Vector<TaylorModel<ValidatedTag,F>> r(tf.size(),tg.zero_element());
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=compose(tf[i],tg); }
    return r;
}

template<class P, class F> Vector<TaylorModel<P,F>> unscale(const Vector<TaylorModel<P,F>>& x, const Vector<IntervalDomainType>& dom) {
    Vector<TaylorModel<P,F>> r(x.size());
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=unscale(x[i],dom[i]); }
    return r;
}

template<class P, class F> Vector<TaylorModel<P,F>> compose(const VectorUnscaling& u, const Vector<TaylorModel<P,F>>& g) {
    ARIADNE_ASSERT_MSG(u.size()==g.size(),"u="<<u.domain()<<", g="<<g);
    return Vector<TaylorModel<P,F>>(u.size(),[&u,&g](SizeType i){return compose(u[i],g[i]);});
}

template<class P, class F> Vector<TaylorModel<P,F>> compose(const Vector<TaylorModel<P,F>>& x,
                                                            const VectorUnscaling& u,
                                                            const Vector<TaylorModel<P,F>>& y) {
    return compose(x,compose(u,y));
}

template<class P, class F> Vector<TaylorModel<P,F>> antiderivative(const Vector<TaylorModel<P,F>>& x, SizeType k) {
    Vector<TaylorModel<P,F>> r(x.size());
    for(SizeType i=0; i!=x.size(); ++i) { r[i]=antiderivative(x[i],k); }
    return r;
}

template<class P, class F> Vector<TaylorModel<P,F>> derivative(const Vector<TaylorModel<P,F>>& x, SizeType k) {
    Vector<TaylorModel<P,F>> r(x.size());
    for(SizeType i=0; i!=x.size(); ++i) { r[i]=derivative(x[i],k); }
    return r;
}

template<class P, class F> Vector<TaylorModel<P,F>> combine(const Vector<TaylorModel<P,F>>& x1, const Vector<TaylorModel<P,F>>& x2) {
    return join(embed(0u,x1,x2.zero_element().argument_size()),embed(x1.zero_element().argument_size(),x2,0u));
}

template<class F> Vector<TaylorModel<ValidatedTag,F>> embed(SizeType as1, const Vector<TaylorModel<ValidatedTag,F>>& x2, SizeType as3) {
    return Vector<TaylorModel<ValidatedTag,F>>(x2.size(),[as1,&x2,as3](SizeType i){return embed(as1,x2[i],as3);});
}

template<class P, class F> Vector<TaylorModel<P,F>> split(const Vector<TaylorModel<P,F>>& x, SizeType j, SplitPart h) {
    Vector<TaylorModel<P,F>> r(x.size());
    for(SizeType i=0; i!=x.size(); ++i) { r[i]=split(x[i],j,h); }
    return r;
}

template<class P, class F> Vector<typename TaylorModel<P,F>::RangeType> ranges(const Vector<TaylorModel<P,F>>& f) {
    Vector<typename TaylorModel<P,F>::RangeType> r(f.size()); for(SizeType i=0; i!=f.size(); ++i) { r[i]=f[i].range(); } return r;
}

template<class P, class F> Vector<typename TaylorModel<P,F>::ErrorType> errors(const Vector<TaylorModel<P,F>>& h) {
    Vector<typename TaylorModel<P,F>::ErrorType> e(h.size()); for(SizeType i=0; i!=h.size(); ++i) { e[i]=h[i].error(); } return e; }

template<class P, class F> Vector<typename TaylorModel<P,F>::NormType> norms(const Vector<TaylorModel<P,F>>& h) {
    Vector<typename TaylorModel<P,F>::NormType> r(h.size()); for(SizeType i=0; i!=h.size(); ++i) { r[i]=norm(h[i]); } return r; }

template<class P, class F> typename TaylorModel<P,F>::NormType norm(const Vector<TaylorModel<P,F>>& h) {
    typename TaylorModel<P,F>::NormType r=0u; for(SizeType i=0; i!=h.size(); ++i) { r=max(r,norm(h[i])); } return r;
}

template<class F> Matrix<Bounds<F>> jacobian(const Vector<TaylorModel<ValidatedTag,F>>& x, const Vector<Bounds<F>>& y);
template<class F> Matrix<Bounds<F>> jacobian(const Vector<TaylorModel<ValidatedTag,F>>& x, const Vector<Bounds<F>>& y, Array<SizeType>& p);
template<class F> Matrix<F> jacobian_value(const Vector<TaylorModel<ValidatedTag,F>>& x);
template<class F> Matrix<F> jacobian_value(const Vector<TaylorModel<ValidatedTag,F>>& x, const Array<SizeType>& p);
template<class F> Matrix<UpperInterval<F>> jacobian_range(const Vector<TaylorModel<ValidatedTag,F>>& x);
template<class F> Matrix<UpperInterval<F>> jacobian_range(const Vector<TaylorModel<ValidatedTag,F>>& x, const Array<SizeType>& p);

template<class F> TaylorModel<ValidatedTag,F> value_coefficients(TaylorModel<ValidatedTag,Bounds<F>> const& tm);
template<class F> TaylorModel<ValidatedTag,Bounds<F>> exact_coefficients(TaylorModel<ValidatedTag,Bounds<F>> const& tm);


} // namespace Ariadne

#endif // ARIADNE_TAYLOR_MODEL_HPP
