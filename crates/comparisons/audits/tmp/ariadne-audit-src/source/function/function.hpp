/***************************************************************************
 *            function/function.hpp
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

/*! \file function/function.hpp
 *  \brief Built-in and user functions and expressions
 */

#ifndef ARIADNE_FUNCTION_HPP
#define ARIADNE_FUNCTION_HPP

#include "config.hpp"

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "utility/declarations.hpp"
#include "utility/macros.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"
#include "utility/metaprogramming.hpp"

#include "function/function_interface.hpp"
#include "function/function_concepts.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/differential.hpp"
#include "function/domain.hpp"
#include "function/formula.hpp"

namespace Ariadne {

template<class P, class... ARGS> struct VectorFunctionElementReference;

template<class T> class Variable;
typedef Variable<Real> RealVariable;
template<class T> class Expression;
typedef Expression<Real> RealExpression;
template<class P, class SIG> class FunctionExpression;

//! \brief Named (static) constructors for constant and coordinate functions.
template<class P>
class FunctionConstructors {
    static_assert(Same<P,ApproximateTag> or Same<P,ValidatedTag> or Same<P,EffectiveTag>,"P must be an information level/paradigm.");
    typedef Number<P> Y;
  public:
    typedef Y NumericType;

    typedef RealDomain ScalarDomainType;
    typedef EuclideanDomain VectorDomainType;

    //!@{
    //! \name Named (static) constructors
    static ScalarMultivariateFunction<P> zero(SizeType as); //!< \brief %Constant scalar function zero in \a as arguments.
    static ScalarMultivariateFunction<P> constant(SizeType as, NumericType c); //!< \brief %Constant scalar function \a c in \a as arguments.
    static ScalarMultivariateFunction<P> coordinate(SizeType as, SizeType j); //!< \brief %Coordinate function taking \a j -th index in \a as arguments.
    static List<ScalarMultivariateFunction<P>> coordinates(SizeType ns); //!< \brief All coordinates over \a ns variables.
    static VectorMultivariateFunction<P> zeros(SizeType rs, SizeType as); //!< \brief The zero function over \a as variables returning a result of size \a rs.
    static VectorMultivariateFunction<P> constant(SizeType as, Vector<NumericType> c); //!< \brief %Constant vector function \a c in \a as arguments.
    static VectorMultivariateFunction<P> identity(SizeType ns); //!< \brief The identity over \a ns variables.

    static ScalarMultivariateFunction<P> zero(VectorDomainType dom); //!< \brief %Constant scalar function zero over domain \a dom.
    static ScalarMultivariateFunction<P> constant(VectorDomainType dom, NumericType c); //!< \brief %Constant function \a c over domain \a dom.
    static ScalarMultivariateFunction<P> coordinate(VectorDomainType dom, SizeType j); //!< \brief %Coordinate function taking \a j -th index over domain \a dom.
    static List<ScalarMultivariateFunction<P>> coordinates(VectorDomainType dom); //!< \brief %List of all coordinate functions over domain \a dom.
    static VectorMultivariateFunction<P> zeros(SizeType rs, VectorDomainType dom); //!< \brief Zero function over domain \a dom returning a result of size \a rs.
    static VectorMultivariateFunction<P> constant(VectorDomainType dom, Vector<NumericType> c); //!< \brief Constant function over domain \a dom  taking values \a c.
    static VectorMultivariateFunction<P> identity(VectorDomainType dom); //!< \brief Identity function over domain \a dom.

    static ScalarUnivariateFunction<P> zero(); //!< \brief %Constant scalar univariate function zero.
    static ScalarUnivariateFunction<P> constant(NumericType c); //!< \brief %Constant scalar univariate function with value \a c.
    static ScalarUnivariateFunction<P> coordinate(); //!< \brief %Scalar univariate coordinate function.
    static VectorUnivariateFunction<P> zeros(SizeType rs); //!< \brief %Constant vector univariate function zero returning a result of size \a rs.
    static VectorUnivariateFunction<P> constant(Vector<NumericType> c); //!< \brief Constant univariate function taking values \a c.
    static ScalarUnivariateFunction<P> identity(); //!< \brief %Scalar univariate identity function.

    static ScalarUnivariateFunction<P> zero(ScalarDomainType dom); //!< \brief %Constant scalar function zero over domain \a dom.
    static ScalarUnivariateFunction<P> constant(ScalarDomainType dom, NumericType c); //!< \brief %Constant scalar function \a c over domain \a dom.
    static ScalarUnivariateFunction<P> coordinate(ScalarDomainType dom, IndexZero j); //!< \brief %Coordinate function over domain \a dom.
    static ScalarUnivariateFunction<P> coordinate(ScalarDomainType dom); //!< \brief %Coordinate function over domain \a dom.
    static VectorUnivariateFunction<P> zeros(SizeType rs, ScalarDomainType dom); //!< \brief %Zero vector function over domain \a dom returning a result of size \a rs.
    static VectorUnivariateFunction<P> constant(ScalarDomainType dom, Vector<NumericType> c); //!< \brief Constant function over domain \a dom  taking values \a c.
    static ScalarUnivariateFunction<P> identity(ScalarDomainType dom); //!< \brief %Identity function over domain \a dom.

    //!@}

};

template<class P, class X> using EvaluateType = decltype(declval<ScalarMultivariateFunctionInterface<P>>()._call(declval<Vector<X>>()));

//! \brief Function operations which depend on the scalar/vector nature of the arguments and result.
template<class P, class SIG> class FunctionFacade {
};

//! \brief Function operations for scalar univariate functions.
template<class P> class FunctionFacade<P,RealScalar(RealScalar)> {
  public:
    //! \relates Function \brief The slope (derivative) of the function at the point \a x.
    template<class X> Scalar<EvaluateType<P,X>> slope(X const& x) const;
    //! \relates Function \brief The derivative of the function at the point \a x.
    FunctionExpression<P,RealScalar(RealScalar)> operator() (const RealVariable& x) const;
};

//! \brief Function operations for vector univariate functions.
template<class P> class FunctionFacade<P,RealVector(RealScalar)> {
  public:
    //! \relates Function \brief The tangent vector of derivatives of the function at the point \a x.
    template<class X> Vector<EvaluateType<P,X>> tangent(X const& x) const;
    FunctionExpression<P,RealVector(RealScalar)> operator() (const RealVariable& x) const;
};

//! \brief Function operations for scalar multivariate functions.
template<class P> class FunctionFacade<P,RealScalar(RealVector)> {
    typedef Number<P> Y;
  public:
    //! \relates Function \brief The gradient covector of derivatives of the function at the point \a x.
    template<class X> Covector<EvaluateType<P,X>> gradient(Vector<X> const& x) const;
    FunctionExpression<P,RealScalar(RealVector)> operator() (const Vector<RealVariable>& x) const;
    //FunctionExpression<P,RealScalar(RealVector)> operator() (const Vector<RealExpression>& x) const;
};

//! \brief Function operations for vector multivariate functions.
template<class P> class FunctionFacade<P,RealVector(RealVector)> {
    typedef Number<P> Y;
  public:
    //! \relates Function \brief The Jacobian matrix of derivatives of the function at the point \a x.
    template<class X> Matrix<EvaluateType<P,X>> jacobian(Vector<X> const& x) const;
    FunctionExpression<P,RealVector(RealVector)> operator() (const Vector<RealVariable>& x) const;
};

template<class P, class SIG> class DeclareFunctionOperations;
template<class P, class... ARGS> class DeclareFunctionOperations<P,RealScalar(ARGS...)>
    : DeclareElementaryAlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>> { };
template<class P, class... ARGS> class DeclareFunctionOperations<P,RealVector(ARGS...)>
    : DeclareVectorAlgebraOperators<VectorFunction<P,ARGS...>,ScalarFunction<P,ARGS...>,Vector<Number<P>>,Number<P>> { };

//! \brief A mixin which provides Scalar/Vector function methods and friends.
template<class P, class SIG> class DispatchFunctionOperations;
template<class P, class... ARGS> class DispatchFunctionOperations<P,RealScalar(ARGS...)>
    : DispatchElementaryAlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>>
{
};
template<class P, class... ARGS> class DispatchFunctionOperations<P,RealVector(ARGS...)>
    : DeclareVectorAlgebraOperators<VectorFunction<P,ARGS...>,ScalarFunction<P,ARGS...>,Vector<Number<P>>,Number<P>> { };

//! \ingroup FunctionModule
//! \brief A generic continuous function.
//! \tparam P The <em>\ref information_section "information paradigm"</em> tag, which can be either ExactTag, EffectiveTag, ValidatedTag, or ApproximateTag.
//! \tparam SIG The <em>\ref function_signature_section "signature"</em>, which has the standard C++ form \c RES(ARG), so signature \c Real(RealVector) indicates a function \f$f:\R^n\rightarrow\R\f$.
//! \details Class representing general continuous functions .
//! \paragraph Note: Currently, only \f$\R\f$ and \f$R^n\f$ are supported as argument and results types, denoted respectively by C++ classes \ref Real and \ref RealVector.
//! The %Ariadne %Function class provides an interface with support for evaluation over different numeric and algebraic types using \ref Function::operator().
//! Since any continuous function can be approximated to any given accuracy over a given domain by a polynomial,
//! \paragraph Note: A direct method to extract a polynomial approximation will be provided in a later version of %Ariadne.
//!
//! Functions are assumed to be sufficiently differentiable, and a list of all derivates up to a given degree can be computed using \ref Function::differential, and the derivative function with respect to a given argument by \ref Function::derivative.
//!
template<class P, class SIG>
class Function
    : public Handle<const FunctionInterface<P,SIG>>
    , public FunctionConstructors<P>
    , public SignatureTraits<SIG>
    , public FunctionFacade<P,SIG>
    , public DispatchFunctionOperations<P,SIG>
{
    static_assert(IsParadigm<P>,"P must be an information level/paradigm.");
    using Y=Number<P>;
    using D=typename SignatureTraits<SIG>::DomainType;
    using C=typename SignatureTraits<SIG>::CodomainType;
    using ARG=typename SignatureTraits<SIG>::ArgumentKind;
  public:
    typedef FunctionInterface<P,SIG> Interface;
    typedef P InformationTag; //!< The type of information (Exact, Effective, Validated or Approximate) provided by the function implementation.
    typedef P Paradigm;
    typedef D DomainType; //!< The type of the domain.
    typedef C CodomainType; //!< The type of the codomain.
    typedef Number<P> NumericType; //!< The numeric type required to construct a constant scalar function.
    typedef typename SignatureTraits<SIG>::ArgumentSpaceType ArgumentSpaceType; //!< The type used to descibe an ordered set of argument variables.
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType; //!< The type used to descibe the size of an element of the domain.
    typedef typename SignatureTraits<SIG>::ResultSizeType ResultSizeType; //!< The type used to descibe the size of an element of the codomain.
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType; //!< The type used to descibe an index into an element of the domain.
    typedef typename SignatureTraits<SIG>::ResultIndexType ResultIndexType; //!< The type used to descibe an index into an element of the codomain.

    //! \brief The type of an argument to the function whose scalar type is \a Y.
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;
    //! \brief The type for the result of calling the function whose scalar type is \a Y.
    template<class Y> using Result = typename SignatureTraits<SIG>::template Result<Y>;


    //! \name User constructors.
    //!@{

    //! \brief Construct the zero function with the given number of argument variables \a as and result variables \a rs.
    explicit Function(ResultSizeType rs, ArgumentSizeType as);

    //! \brief Construct the zero function with the given domain \a dom and default codomain (Zero-dimensional for a vector function, one-dimensional for a scalar function).
    explicit Function(DomainType dom);
    //! \brief Construct the zero function with the given domain \a dom and number of result variables \a rs.
    explicit Function(ResultSizeType rs, DomainType dom);

    //! \brief Construct a function with the given domain \a dom and number of result variables \a rs.
    explicit Function(DomainType dom, Result<Formula<Y>>const& e);
    //! \brief Construct a function each of whose components are \a sf with the given domain \a dom.
    explicit Function(ResultSizeType rs, ScalarFunction<P,ARG> sf);

    //! \brief Construct from argument variable(s) \a spc and expression(s) \a e in terms of these variables.
    explicit Function(ArgumentSpaceType const& spc, Result<RealExpression>const& e);

    //! \brief Create a vector function from an initializer list of scalar functions.
    Function(InitializerList<ScalarFunction<P,ARG>> const& lsf);
    //! \brief Create a vector function from a list of scalar functions.
    Function(List<ScalarFunction<P,ARG>> const& lsf);
    //! \brief Create a vector function from a vector of scalar functions.
    Function(Vector<ScalarFunction<P,ARG>> const& lsf);
    //!@}

    //! \name Prototype constructors.
    //!@{
    //
    //! \brief Construct a zero scalar function with the same domain.
    ScalarFunction<P,ARG> create_zero() const { return ScalarFunction<P,ARG>::zero(this->domain()); }
    //! \brief Construct a scalar constant function with value \a c and the same domain.
    ScalarFunction<P,ARG> create_constant(NumericType c) const { return ScalarFunction<P,ARG>::constant(this->domain(),c); }
    //! \brief Construct the scalar coordinate function for the index \a j.
    ScalarFunction<P,ARG> create_coordinate(ArgumentIndexType j) const { return ScalarFunction<P,ARG>::coordinate(this->domain(),j); }
    //! \brief Construct the vector constant function with values \a c.
    VectorFunction<P,ARG> create_constant(Vector<NumericType> c) const { return VectorFunction<P,ARG>::constant(this->domain(),c); }
    //!@}

    //!@{
    //! \name Handle-interface methods.
    using Handle<const Interface>::Handle;
    Function(); //!< \brief Create an invalid (null) function.
    //!@}

    //! \name Conversions and assignment.
    //!@{
    //
    //! \brief Copy constructor.
    Function(const Function<P,SIG>& f) = default;
    //! \brief Convert from a function class specifying more information.
    template<StrongerThan<P> PP>
    Function(const Function<PP,SIG>& f)
        : Handle<const Interface>(std::dynamic_pointer_cast< const Interface >(f.managed_pointer())) { }
    //! \brief Copy assignment
    Function<P,SIG>& operator=(const Function<P,SIG>& f) = default;
    //! \brief Assign from a function class specifying more information.
    template<StrongerThan<P> PP>
        Function<P,SIG>& operator=(Result<NumericType> const& c); // { return *this=this->create_constant(c); }
    //! \brief Set equal to the constant value \a c.
    Function<P,SIG>& operator=(const Result<NumericType>& c) {
        return (*this)=this->create_constant(c); }

    //! \brief Construct from a class satisfying the function concept.
    // FIXME: Avoid IsFunctionClass requirement, which is needed do prevent AFunction concept checking conformance before argument classes are defined.
    template<class F> requires (not IsFunctionClass<F,SIG>) and AFunction<F,P,SIG> Function(F const& f);
    //!@}

    //! \name Query domain and codomain.
    //!@{
    //
    //! \brief The domain of the function.
    DomainType domain() const {
        return DomainType(this->reference().argument_size()); }
    //! \brief The codomain of the function.
    CodomainType codomain() const {
        return CodomainType(this->reference().result_size()); }
    //! \brief The number of scalar variables in the argument.
    ArgumentSizeType argument_size() const {
        return this->reference().argument_size(); }
    //! \brief The number of scalar variables in the result.
    ResultSizeType result_size() const {
        return this->reference().result_size(); }
    //!@}

    //! \name Call/evaluate
    //!@{
    //
    //! \brief Call the function on an argument of concrete scalar type \a X.
    template<class X> auto operator() (const Argument<X>& x) const -> decltype(this->reference()._call(x)) {
        return this->reference()._call(x); }
    //! \brief Call the function on an argument of concrete scalar type \a X. \deprecated
    template<class X> auto evaluate(const Argument<X>& x) const -> decltype(this->reference()._call(x)) {
        return this->reference()._call(x); }
    //!@}

    //! \name Differential function operations.
    //!@{
    //
    //! \brief The differential (partial derivatives) of the function at the point \a x, computed to degree \a d.
    template<class X> decltype(auto) differential(const Argument<X>& x, DegreeType d) const {
        return this->_ptr->_call(Differential<EvaluateType<P,X>>::identity(d,x)); }
    //! \brief The derivative of the function with respect to the \a k -th variable.
    Function<P,SIG> derivative(ElementIndexType<D> k) const {
        return Function<P,SIG>(this->reference()._derivative(k)); }
    //!@}

    //! \name Evaluation and differentiation
    //!@{
    //
#ifdef DOXYGEN
    //! \brief Call the function on an argument of concrete scalar type \a X.
    friend template<class X> Result<X> evaluate(const Function<P,SIG>& f, const Argument<X>& x) const {
        return f(x); }
    //! \brief The differential (partial derivatives) of the function at the point \a x, computed to degree \a d.
    friend template<class X> Result<Differential<X>> differential(const Function<P,SIG>& f, const Argument<X>& x, DegreeType d) const {
        return f.differential(x,d); }
#endif
    //! \brief The derivative of the function \a f with respect to the \a k -th variable.
    friend Function<P,SIG> derivative(Function<P,SIG> const& f, ElementIndexType<D> k) {
        return f.derivative(k); }
    //!@}

    //! \name Vector (indexing) operations.
    //!@{
    //
    //! \brief Set the \a i -th component of a vector function to \a f.
    Void set(SizeType i, ScalarFunction<P,ARG> f);
    //! \brief Get the \a i -th component of a vector function.
    Function<P,Real(ARG)> get(SizeType i) const;
    //! \brief Get the \a i -th component of a vector function.
    const Function<P,Real(ARG)> operator[](SizeType i) const;
    //! \brief Get the components of a specified by \a rng function.
    const Function<P,RealVector(ARG)> operator[](Range rng) const;
    //! \brief A reference to the \a i -th component of a vector function.
    VectorFunctionElementReference<P,ARG> operator[](SizeType i);
    //!@}

    //! \name Input/output operations.
    //!@{
    //
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, Function<P,SIG> const& f) { f._ptr->_write(os); return os; }
    //!@}

    friend VectorFunction<P,ARG> operator*(ScalarFunction<P,ARG> const&, Vector<Y> const&);
};

template<class P, class... ARGS> decltype(auto) inline characteristics(Function<P,Real(ARGS...)> const& f) {
    return f.domain(); }

//! \relates Function
//! \name Function template deduction guides
//!@{
//
//! \brief Deduction guide for constructing scalar univariate functions from an expression.
explicit Function(RealVariable const& s, Scalar<RealExpression> const& e) -> Function<EffectiveTag,RealScalar(RealScalar)>;
//! \brief Deduction guide for constructing vector univariate functions from expressions.
explicit Function(RealVariable const& s, Vector<RealExpression> const& e) -> Function<EffectiveTag,RealVector(RealScalar)>;
//! \relates Function \brief Deduction guide for constructing scalar multivariate functions from an expression.
explicit Function(RealSpace const& s, Scalar<RealExpression> const& e) -> Function<EffectiveTag,RealScalar(RealVector)>;
//! \relates Function \brief Deduction guide for constructing vector multivariate functions from expressions.
explicit Function(RealSpace const& s, Vector<RealExpression> const& e) -> Function<EffectiveTag,RealVector(RealVector)>;
//!@}


template<class P, class SIG> OutputStream& operator<<(OutputStream& os, Representation<Function<P,SIG>> const& f) {
    f.reference().raw_pointer()->_repr(os); return os; }

template<class P, class... ARGS> struct AlgebraOperations<ScalarFunction<P,ARGS...>,Number<P>>
{
    using F=ScalarFunction<P,ARGS...>;
    using C=Number<P>;
    static ScalarFunction<P,ARGS...> apply(BinaryElementaryOperator op, ScalarFunction<P,ARGS...> const& f1, ScalarFunction<P,ARGS...> const& f2);
    static ScalarFunction<P,ARGS...> apply(UnaryElementaryOperator op, ScalarFunction<P,ARGS...> const& f);
    static ScalarFunction<P,ARGS...> apply(BinaryElementaryOperator op, ScalarFunction<P,ARGS...> const& f1, Number<P> const& c2);
    static ScalarFunction<P,ARGS...> apply(BinaryElementaryOperator op, Number<P> const& c1, ScalarFunction<P,ARGS...> const& f2);
    static ScalarFunction<P,ARGS...> apply(GradedElementaryOperator op, ScalarFunction<P,ARGS...> const& f, Int n);
};

template<class P, class SIG> inline OutputStream&
operator<<(OutputStream& os, const Function<P,SIG>& f) {
    return f._write(os); }

template<class P, class C, class X> inline decltype(auto)
evaluate(const UnivariateFunction<P,C>& f, const Scalar<X>& x) {
    return f(x); }

template<class P, class C, class X> inline decltype(auto)
evaluate(const MultivariateFunction<P,C>& f, const Vector<X>& x) {
    return f(x); }


template<class P, class C, class X> inline decltype(auto)
differential(const UnivariateFunction<P,C>& f, const X& x, DegreeType d) {
    return f.differential(x,d); }

template<class P, class C, class X> inline decltype(auto)
differential(const MultivariateFunction<P,C>& f, const Vector<X>& x, DegreeType d) {
    return f.differential(x,d); }

template<class P, class SIG> requires Same<typename SignatureTraits<SIG>::ArgumentKind,Real> inline
Function<P,SIG> derivative(Function<P,SIG> const& f) {
    return f.derivative(IndexZero()); }

template<class P, class X> Scalar<EvaluateType<P,X>>
slope(const ScalarUnivariateFunction<P>& f, const Scalar<X>& x) {
    return differential(f,x,1u).gradient()[0]; }

template<class P, class X> Vector<EvaluateType<P,X>>
tangent(const VectorUnivariateFunction<P>& f, const Scalar<X>& x) {
    return column(differential(f,x,1u).jacobian(),0u); }

template<class P, class X> Covector<EvaluateType<P,X>>
gradient(const ScalarMultivariateFunction<P>& f, const Vector<X>& x) {
    return differential(f,x,1u).gradient(); }

template<class P, class X> Matrix<EvaluateType<P,X>>
jacobian(const VectorMultivariateFunction<P>& f, const Vector<X>& x) {
    return differential(f,x,1u).jacobian(); }

template<class P, class X> Matrix<EvaluateType<P,X>>
hessian(const ScalarMultivariateFunction<P>& f, const Vector<X>& x) {
    return differential(f,x,2u).hessian(); }

template<class P> template<class X> EvaluateType<P,X>
FunctionFacade<P,RealScalar(RealScalar)>::slope(X const& x) const {
    return Ariadne::slope(static_cast<ScalarUnivariateFunction<P>const&>(*this),x);
}

template<class P> template<class X> Vector<EvaluateType<P,X>>
FunctionFacade<P,RealVector(RealScalar)>::tangent(X const& x) const {
    return Ariadne::tangent(static_cast<VectorUnivariateFunction<P>const&>(*this),x);
}

template<class P> template<class X> Covector<EvaluateType<P,X>>
FunctionFacade<P,RealScalar(RealVector)>::gradient(Vector<X> const& x) const {
    return Ariadne::gradient(static_cast<ScalarMultivariateFunction<P>const&>(*this),x);
}

template<class P> template<class X> Matrix<EvaluateType<P,X>>
FunctionFacade<P,RealVector(RealVector)>::jacobian(Vector<X> const& x) const {
    return Ariadne::jacobian(static_cast<VectorMultivariateFunction<P>const&>(*this),x);
}



EffectiveScalarUnivariateFunction compose(const EffectiveScalarUnivariateFunction& f, const EffectiveScalarUnivariateFunction& g);
EffectiveScalarUnivariateFunction compose(const EffectiveScalarMultivariateFunction& f, const EffectiveVectorUnivariateFunction& g);
EffectiveVectorUnivariateFunction compose(const EffectiveVectorUnivariateFunction& f, const EffectiveScalarUnivariateFunction& g);
EffectiveVectorUnivariateFunction compose(const EffectiveVectorMultivariateFunction& f, const EffectiveVectorUnivariateFunction& g);
EffectiveScalarMultivariateFunction compose(const EffectiveScalarUnivariateFunction& f, const EffectiveScalarMultivariateFunction& g);
EffectiveScalarMultivariateFunction compose(const EffectiveScalarMultivariateFunction& f, const EffectiveVectorMultivariateFunction& g);
EffectiveVectorMultivariateFunction compose(const EffectiveVectorUnivariateFunction& f, const EffectiveScalarMultivariateFunction& g);
EffectiveVectorMultivariateFunction compose(const EffectiveVectorMultivariateFunction& f, const EffectiveVectorMultivariateFunction& g);

ValidatedScalarUnivariateFunction compose(const ValidatedScalarUnivariateFunction& f, const ValidatedScalarUnivariateFunction& g);
ValidatedScalarUnivariateFunction compose(const ValidatedScalarMultivariateFunction& f, const ValidatedVectorUnivariateFunction& g);
ValidatedVectorUnivariateFunction compose(const ValidatedVectorUnivariateFunction& f, const ValidatedScalarUnivariateFunction& g);
ValidatedVectorUnivariateFunction compose(const ValidatedVectorMultivariateFunction& f, const ValidatedVectorUnivariateFunction& g);
ValidatedScalarMultivariateFunction compose(const ValidatedScalarUnivariateFunction& f, const ValidatedScalarMultivariateFunction& g);
ValidatedScalarMultivariateFunction compose(const ValidatedScalarMultivariateFunction& f, const ValidatedVectorMultivariateFunction& g);
ValidatedVectorMultivariateFunction compose(const ValidatedVectorUnivariateFunction& f, const ValidatedScalarMultivariateFunction& g);
ValidatedVectorMultivariateFunction compose(const ValidatedVectorMultivariateFunction& f, const ValidatedVectorMultivariateFunction& g);

ApproximateScalarUnivariateFunction compose(const ApproximateScalarUnivariateFunction& f, const ApproximateScalarUnivariateFunction& g);
ApproximateScalarUnivariateFunction compose(const ApproximateScalarMultivariateFunction& f, const ApproximateVectorUnivariateFunction& g);
ApproximateVectorUnivariateFunction compose(const ApproximateVectorUnivariateFunction& f, const ApproximateScalarUnivariateFunction& g);
ApproximateVectorUnivariateFunction compose(const ApproximateVectorMultivariateFunction& f, const ApproximateVectorUnivariateFunction& g);
ApproximateScalarMultivariateFunction compose(const ApproximateScalarUnivariateFunction& f, const ApproximateScalarMultivariateFunction& g);
ApproximateScalarMultivariateFunction compose(const ApproximateScalarMultivariateFunction& f, const ApproximateVectorMultivariateFunction& g);
ApproximateVectorMultivariateFunction compose(const ApproximateVectorUnivariateFunction& f, const ApproximateScalarMultivariateFunction& g);
ApproximateVectorMultivariateFunction compose(const ApproximateVectorMultivariateFunction& f, const ApproximateVectorMultivariateFunction& g);


ValidatedScalarMultivariateFunction& operator*=(ValidatedScalarMultivariateFunction& sf, const ExactNumber& c);
EffectiveVectorMultivariateFunction operator*(const EffectiveNumber& c, const EffectiveVectorMultivariateFunction& vf);

EffectiveScalarMultivariateFunction embed(SizeType as1, const EffectiveScalarMultivariateFunction& f2, SizeType as3);
EffectiveVectorMultivariateFunction embed(SizeType as1, const EffectiveVectorMultivariateFunction& f2, SizeType as3);

EffectiveVectorMultivariateFunction join(const EffectiveScalarMultivariateFunction& f1, const EffectiveScalarMultivariateFunction& f2);
EffectiveVectorMultivariateFunction join(const EffectiveScalarMultivariateFunction& f1, const EffectiveVectorMultivariateFunction& f2);
EffectiveVectorMultivariateFunction join(const EffectiveVectorMultivariateFunction& f1, const EffectiveScalarMultivariateFunction& f2);
EffectiveVectorMultivariateFunction join(const EffectiveVectorMultivariateFunction& f1, const EffectiveVectorMultivariateFunction& f2);

EffectiveVectorMultivariateFunction derivatives(const EffectiveScalarMultivariateFunction& f);

EffectiveScalarMultivariateFunction lie_derivative(const EffectiveScalarMultivariateFunction& g, const EffectiveVectorMultivariateFunction& f);
EffectiveVectorMultivariateFunction lie_derivative(const EffectiveVectorMultivariateFunction& g, const EffectiveVectorMultivariateFunction& f);

Formula<Real> make_formula(const EffectiveScalarMultivariateFunction& f);
Vector<Formula<Real>> make_formula(const EffectiveVectorMultivariateFunction& f);
//RealExpression evaluate(EffectiveScalarMultivariateFunction const& f, Vector<RealVariable> const& vars);



ValidatedVectorMultivariateFunction join(const ValidatedScalarMultivariateFunction& f1, const ValidatedScalarMultivariateFunction& f2);
ValidatedVectorMultivariateFunction join(const ValidatedScalarMultivariateFunction& f1, const ValidatedVectorMultivariateFunction& f2);
ValidatedVectorMultivariateFunction join(const ValidatedVectorMultivariateFunction& f1, const ValidatedScalarMultivariateFunction& f2);
ValidatedVectorMultivariateFunction join(const ValidatedVectorMultivariateFunction& f1, const ValidatedVectorMultivariateFunction& f2);


ApproximateVectorMultivariateFunction join(const ApproximateScalarMultivariateFunction& f1, const ApproximateScalarMultivariateFunction& f2);
ApproximateVectorMultivariateFunction join(const ApproximateScalarMultivariateFunction& f1, const ApproximateVectorMultivariateFunction& f2);
ApproximateVectorMultivariateFunction join(const ApproximateVectorMultivariateFunction& f1, const ApproximateScalarMultivariateFunction& f2);
ApproximateVectorMultivariateFunction join(const ApproximateVectorMultivariateFunction& f1, const ApproximateVectorMultivariateFunction& f2);

//! \brief A reference into an element of a vector function.
template<class P, class... ARGS>
struct VectorFunctionElementReference
    : DispatchFunctionOperations<P,Real(ARGS...)>
{
    using RES=RealScalar; using SIG=RES(ARGS...);
    typedef typename SignatureTraits<SIG>::DomainType DomainType;
    template<class Y> using Argument = typename SignatureTraits<SIG>::template Argument<Y>;
    typedef VectorFunctionElementReference<P,ARGS...> SelfType;
    typedef ScalarFunction<P,ARGS...> ElementType;
    typedef typename ElementType::NumericType NumericType;
    VectorFunction<P,ARGS...>& _vf; SizeType _i;
    VectorFunctionElementReference(VectorFunction<P,ARGS...>& vf, SizeType i) : _vf(vf), _i(i) { }
    template<class WP> operator ScalarFunction<WP,ARGS...> () const;
    Void operator=(const ScalarFunction<P,ARGS...>& sf);
    VectorFunctionElementReference<P,ARGS...>& operator=(const VectorFunctionElementReference<P,ARGS...>& sfr);
    DomainType domain() const;
    template<class XX> XX evaluate(const Argument<XX> & x) const;
    template<class XX> XX operator()(const Argument<XX>& x) const;
};

template<class P, class... ARGS> template<class WP> inline
VectorFunctionElementReference<P,ARGS...>::operator ScalarFunction<WP,ARGS...> () const {
    return this->_vf.get(this->_i);
}

template<class P, class... ARGS> inline VectorFunctionElementReference<P,ARGS...> make_element_reference(ScalarFunction<P,ARGS...>& sf, SizeType i) {
    throw std::runtime_error(""); }
template<class P, class... ARGS> inline VectorFunctionElementReference<P,ARGS...> make_element_reference(VectorFunction<P,ARGS...>& vf, SizeType i) {
    return VectorFunctionElementReference<P,ARGS...>(vf,i); }

template<class P, class SIG> inline auto Function<P,SIG>::operator[](SizeType i) -> VectorFunctionElementReference<P,ARG> {
    return make_element_reference(*this,i); }

template<class P, class SIG> inline auto Function<P,SIG>::operator[](SizeType i) const -> ScalarFunction<P,ARG> const {
    return this->get(i); }

template<class P, class... ARGS> inline OutputStream& operator<<(OutputStream& os, const VectorFunctionElementReference<P,ARGS...>& vfe) {
    return  os << static_cast< ScalarFunction<P,ARGS...> >(vfe); }

template<class P, class... ARGS> inline Void VectorFunctionElementReference<P,ARGS...>::operator=(const ScalarFunction<P,ARGS...>& sf) {
    _vf.set(_i,sf); }
template<class P, class... ARGS> inline VectorFunctionElementReference<P,ARGS...>& VectorFunctionElementReference<P,ARGS...>::operator=(const VectorFunctionElementReference<P,ARGS...>& sfr) {
    _vf.set(_i,static_cast<ScalarFunction<P,ARGS...>>(sfr)); return *this; }
template<class P, class... ARGS> inline auto VectorFunctionElementReference<P,ARGS...>::domain() const -> DomainType {
    return _vf.domain(); }
template<class P, class... ARGS> template<class XX> inline XX VectorFunctionElementReference<P,ARGS...>::evaluate(const Argument<XX> & x) const {
    return static_cast<ScalarFunction<P,ARGS...>>(*this).evaluate(x); }
template<class P, class... ARGS> template<class XX> inline XX VectorFunctionElementReference<P,ARGS...>::operator()(const Argument<XX> & x) const {
    return static_cast<ScalarFunction<P,ARGS...>>(*this).evaluate(x); }



UpperIntervalType evaluate_range(ScalarMultivariateFunction<ValidatedTag>const& f, const Vector<UpperIntervalType>& x);
Vector<UpperIntervalType> evaluate_range(VectorMultivariateFunction<ValidatedTag>const& f, const Vector<UpperIntervalType>& x);
Vector<Differential<UpperIntervalType>> derivative_range(VectorMultivariateFunction<ValidatedTag>const& f, const Vector<Differential<UpperIntervalType>>& x);
Covector<UpperIntervalType> gradient_range(ValidatedScalarMultivariateFunction const& f, const Vector<UpperIntervalType>& x);
Matrix<UpperIntervalType> jacobian_range(ValidatedVectorMultivariateFunction const& f, const Vector<UpperIntervalType>& x);



/*
inline Matrix<UpperIntervalType> jacobian(VectorMultivariateFunction<ValidatedTag>const& f, const Vector<UpperIntervalType>& x) {
    return static_cast<Matrix<UpperIntervalType>>(f.jacobian(reinterpret_cast<Vector<ValidatedNumericType>const&>(x))); }
inline Matrix<UpperIntervalType> jacobian(VectorMultivariateFunction<ValidatedTag>const& f, const Vector<ExactInterval>& x) {
    return static_cast<Matrix<UpperIntervalType>>(f.jacobian(reinterpret_cast<Vector<ValidatedNumericType>const&>(x))); }
inline Matrix<UpperIntervalType> jacobian_range(VectorMultivariateFunction<ValidatedTag>const& f, const Vector<UpperIntervalType>& x) {
    return static_cast<Matrix<UpperIntervalType>>(f.jacobian(reinterpret_cast<Vector<ValidatedNumericType>const&>(x))); }

// FIXME: Needed to override templated gradient and jacobian
inline Covector<UpperIntervalType> gradient(ScalarMultivariateFunction<EffectiveTag>const& f, const Vector<UpperIntervalType>& x) {
    return static_cast<Covector<UpperIntervalType>>(gradient(f,reinterpret_cast<Vector<ValidatedNumericType>const&>(x))); }
inline Covector<UpperIntervalType> gradient(ScalarMultivariateFunction<EffectiveTag>const& f, const Vector<ExactInterval>& x) {
    return gradient(f,static_cast<Vector<UpperIntervalType>>(x)); }
inline Matrix<UpperIntervalType> jacobian(VectorMultivariateFunction<EffectiveTag>const& f, const Vector<UpperIntervalType>& x) {
    return static_cast<Matrix<UpperIntervalType>>(f.jacobian(reinterpret_cast<Vector<ValidatedNumericType>const&>(x))); }
inline Matrix<UpperIntervalType> jacobian(VectorMultivariateFunction<EffectiveTag>const& f, const Vector<ExactInterval>& x) {
    return jacobian(f,static_cast<Vector<UpperIntervalType>>(x)); }
*/

//! \brief A class for constructing generic functions with a specific implementation.
//! \sa Function
template<class P> class FunctionFactory
    : public Handle<const FunctionFactoryInterface<P>>
{
  public:
    typedef FunctionFactoryInterface<P> Interface;
    using Handle<const Interface>::Handle;

    //! \brief Create a scalar function equal to \a f over domain \a dom.
    inline ScalarMultivariateFunction<P> create(const BoxDomainType& dom, const ScalarMultivariateFunctionInterface<P>& f) const;
    //! \brief Create a vector function equal to \a f over domain \a dom.
    inline VectorMultivariateFunction<P> create(const BoxDomainType& dom, const VectorMultivariateFunctionInterface<P>& f) const;
    //! \brief Create a scalar zero function over domain \a dom.
    inline ScalarMultivariateFunction<P> create_zero(const BoxDomainType& dom) const;
    //! \brief Create the identity function over domain \a dom.
    inline VectorMultivariateFunction<P> create_identity(const BoxDomainType& dom) const;
    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, const FunctionFactory<P>& factory);
};

typedef FunctionFactory<ValidatedTag> ValidatedFunctionFactory;

template<>
class FunctionFactory<ValidatedTag>
{
    using P=ValidatedTag;
  private:
    SharedPointer< const FunctionFactoryInterface<P> > _ptr;
  public:
    typedef FunctionFactoryInterface<P> Interface;
    FunctionFactory(const Interface& ref) : _ptr(ref.clone()) { }
    FunctionFactory(const Interface* ptr) : _ptr(ptr) { }
    FunctionFactory(SharedPointer< const Interface > ptr) : _ptr(ptr) { }

    inline ValidatedScalarMultivariateFunction create(const BoxDomainType& d, const ValidatedScalarMultivariateFunction::Interface& f) const;
    inline ValidatedVectorMultivariateFunction create(const BoxDomainType& d, const ValidatedVectorMultivariateFunction::Interface& f) const;
    inline ValidatedScalarMultivariateFunction create_zero(const BoxDomainType& d) const;
    inline ValidatedVectorMultivariateFunction create_identity(const BoxDomainType& d) const;
    friend OutputStream& operator<<(OutputStream& os, const FunctionFactory<ValidatedTag>& factory);
};

inline ValidatedScalarMultivariateFunction FunctionFactoryInterface<ValidatedTag>::create(const BoxDomainType& domain, const ValidatedScalarMultivariateFunction::Interface& function) const {
    return ValidatedScalarMultivariateFunction(SharedPointer<ValidatedScalarMultivariateFunction::Interface>(this->_create(domain,function))); }
inline ValidatedVectorMultivariateFunction FunctionFactoryInterface<ValidatedTag>::create(const BoxDomainType& domain, const ValidatedVectorMultivariateFunction::Interface& function) const {
    return ValidatedVectorMultivariateFunction(SharedPointer<ValidatedVectorMultivariateFunction::Interface>(this->_create(domain,function))); }
inline ValidatedScalarMultivariateFunction FunctionFactoryInterface<ValidatedTag>::create_zero(const BoxDomainType& domain) const {
    return this->create(domain,EffectiveScalarMultivariateFunction::zero(domain.dimension())); }
inline ValidatedVectorMultivariateFunction FunctionFactoryInterface<ValidatedTag>::create_identity(const BoxDomainType& domain) const {
    return this->create(domain,EffectiveVectorMultivariateFunction::identity(domain.dimension())); }

inline ValidatedScalarMultivariateFunction FunctionFactory<ValidatedTag>::create(const BoxDomainType& domain, const ValidatedScalarMultivariateFunction::Interface& function) const {
    return this->_ptr->create(domain,function); }
inline ValidatedVectorMultivariateFunction FunctionFactory<ValidatedTag>::create(const BoxDomainType& domain, const ValidatedVectorMultivariateFunction::Interface& function) const {
    return this->_ptr->create(domain,function); }
inline ValidatedScalarMultivariateFunction FunctionFactory<ValidatedTag>::create_zero(const BoxDomainType& domain) const {
    return this->_ptr->create_zero(domain); }
inline ValidatedVectorMultivariateFunction FunctionFactory<ValidatedTag>::create_identity(const BoxDomainType& domain) const {
    return this->_ptr->create_identity(domain); }

inline OutputStream& operator<<(OutputStream& os, const ValidatedFunctionFactory& factory) {
    factory._ptr->_write(os); return os; }


} // namespace Ariadne

#endif
