/***************************************************************************
 *            function/symbolic_function.hpp
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

/*! \file function/symbolic_function.hpp
 *  \brief Symbolic functions
 */

#ifndef ARIADNE_SYMBOLIC_FUNCTION_HPP
#define ARIADNE_SYMBOLIC_FUNCTION_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "function/function_interface.hpp"

#include "utility/macros.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"
#include "helper/stlio.hpp"
#include "utility/metaprogramming.hpp"

#include "numeric/numeric.hpp"
#include "numeric/operators.tpl.hpp"
#include "algebra/vector.hpp"

#include "function/function_mixin.hpp"
#include "function/function_wrapper.hpp"
#include "function/projection.hpp"
#include "function/formula.hpp"

namespace Ariadne {

//------------------------ Formula functions  -----------------------------------//

//! A function defined by a formula
template<class Y>
class ScalarUnivariateFormulaFunction
//     : public ScalarUnivariateFunctionMixin<ScalarUnivariateFormulaFunction<Y>,InformationTag<Y>>
{
    typedef InformationTag<Y> P;
    Formula<Y> _formula;
  public:
    ScalarUnivariateFormulaFunction(const Formula<Y>& f) : _formula(f) { }
    Formula<Y> formula() const { return _formula; }
    operator Formula<Y>() const { return _formula; }

    RealDomain domain() const { return RealDomain(); }
    SizeOne argument_size() const { return SizeOne(); }
    SizeOne result_size() const { return SizeOne(); }
    template<class X> X operator() (const X& x) const { return Ariadne::evaluate(_formula,Vector<X>({x})); }
    friend ScalarUnivariateFormulaFunction<Y> derivative(ScalarUnivariateFormulaFunction<Y> const& f, IndexZero j) {
        return ScalarUnivariateFormulaFunction<Y>(Ariadne::derivative(f._formula,0)); }
    friend OutputStream& operator<<(OutputStream& os, ScalarUnivariateFormulaFunction<Y> const& f) {
        return Formula<Y>::_write_univariate(os,f); }
    friend OutputStream& operator<<(OutputStream& os, Representation<ScalarUnivariateFormulaFunction<Y>> const& fr) {
        return os << "ScalarUnivariateFormulaFunction("<< fr.reference() <<")"; }
};

typedef ScalarUnivariateFormulaFunction<EffectiveNumber> EffectiveScalarUnivariateFormulaFunction;


//! A function defined by a formula
template<class Y>
class VectorUnivariateFormulaFunction
{
    typedef InformationTag<Y> P;
    Vector<Formula<Y>> _formulae;

 public:
    VectorUnivariateFormulaFunction(const List< Formula<Y> >& f) : _formulae(f) { }
    VectorUnivariateFormulaFunction(const Vector< Formula<Y> >& f) : _formulae(f) { }

    Vector<Formula<Y>> formulae() const { return _formulae; }
    ScalarUnivariateFormulaFunction<Y> operator[](SizeType i) const { return ScalarUnivariateFormulaFunction(this->_formulae[i]); }

    RealDomain domain() const { return RealDomain(); }
    SizeType result_size() const { return this->_formulae.size(); }
    SizeOne argument_size() const { return SizeOne(); }
    template<class X> Vector<X> operator() (const X& x) const { return Ariadne::evaluate(this->_formulae,Vector<X>({x})); }
    friend VectorUnivariateFormulaFunction<Y> derivative(VectorUnivariateFormulaFunction<Y> const& f, IndexZero j) {
        return VectorUnivariateFormulaFunction<Y>(Vector<Formula<Y>>(f._formulae.size(),[&](SizeType i){return derivative(f._formulae[i],j);})); }
    friend OutputStream& operator<<(OutputStream& os, VectorUnivariateFormulaFunction<Y> const& f) {
        return Formula<Y>::_write_univariate(os,f._formulae); }
    friend OutputStream& operator<<(OutputStream& os, Representation<VectorUnivariateFormulaFunction<Y>> const& fr) {
        return os << "VectorUnivariateFormulaFunction(" << fr.reference() << ")"; }
};

typedef VectorUnivariateFormulaFunction<EffectiveNumber> EffectiveVectorUnivariateFormulaFunction;

//! A function defined by a formula
template<class Y>
class ScalarMultivariateFormulaFunction
{
    typedef InformationTag<Y> P;
    SizeType _argument_size;
    Formula<Y> _formula;
  public:
    ScalarMultivariateFormulaFunction(SizeType as, const Formula<Y>& f) : _argument_size(as), _formula(f) { }
    Formula<Y> formula() const { return _formula; }
    operator Formula<Y>() const { return _formula; }

    EuclideanDomain domain() const { return EuclideanDomain(this->_argument_size); }
    SizeType argument_size() const { return _argument_size; }
    SizeOne result_size() const { return SizeOne(); }
    template<class X> X operator() (const Vector<X>& x) const { return Ariadne::evaluate(_formula,x); }
    friend ScalarMultivariateFormulaFunction<Y> derivative(ScalarMultivariateFormulaFunction<Y> const& f, SizeType j) {
        return ScalarMultivariateFormulaFunction<Y>(f._argument_size,Ariadne::derivative(f._formula,j)); }
    friend OutputStream& operator<<(OutputStream& os, ScalarMultivariateFormulaFunction<Y> const& f) {
        return os << f._formula; }
    friend OutputStream& operator<<(OutputStream& os, Representation<ScalarMultivariateFormulaFunction<Y>> const& fr) {
        ScalarMultivariateFormulaFunction<Y> const& f = fr.reference();
        return os << "ScalarMultivariateFormulaFunction(" << f._argument_size << "," << f._formula << ")"; }
};
typedef ScalarMultivariateFormulaFunction<EffectiveNumber> EffectiveScalarMultivariateFormulaFunction;

template<class Y> using ScalarFormulaFunction = ScalarMultivariateFormulaFunction<Y>;
typedef ScalarFormulaFunction<EffectiveNumber> EffectiveScalarFormulaFunction;

//! A vector function defined by formulae
template<class Y>
class VectorMultivariateFormulaFunction
{
    SizeType _argument_size;
    Vector< Formula<Y> > _formulae;
  public:
    VectorMultivariateFormulaFunction(SizeType as, const List< Formula<Y> >& f) : _argument_size(as), _formulae(f) { }
    VectorMultivariateFormulaFunction(SizeType as, const Vector< Formula<Y> >& f) : _argument_size(as), _formulae(f) { }
    Vector<Formula<Y>> formulae() const { return _formulae; }

    ScalarMultivariateFormulaFunction<Y> operator[](SizeType i) const { return ScalarMultivariateFormulaFunction(this->_argument_size,this->_formulae[i]); }
    EuclideanDomain domain() const { return EuclideanDomain(this->_argument_size); }
    SizeType result_size() const { return this->_formulae.size(); }
    SizeType argument_size() const { return this->_argument_size; }
    template<class X> Vector<X> operator() (const Vector<X>& x) const { return Ariadne::evaluate(this->_formulae,x); }
    friend VectorMultivariateFormulaFunction<Y> derivative(VectorMultivariateFormulaFunction<Y> const& f, SizeType j) {
        return VectorMultivariateFormulaFunction<Y>(f._argument_size, Vector<Formula<Y>>(f._formulae.size(),[&](SizeType i){return derivative(f._formulae[i],j);})); }
    friend OutputStream& operator<<(OutputStream& os, VectorMultivariateFormulaFunction<Y> const& f) {
        return os << f._formulae; }
    friend OutputStream& operator<<(OutputStream& os, Representation<VectorMultivariateFormulaFunction<Y>> const& fr) {
        VectorMultivariateFormulaFunction<Y> const& f = fr.reference();
        return os << "VectorMultivariateFormulaFunction("<<f._argument_size<<","<<f._formulae<<")"; }
};
typedef VectorMultivariateFormulaFunction<EffectiveNumber> EffectiveVectorMultivariateFormulaFunction;

template<class Y> using VectorFormulaFunction = VectorMultivariateFormulaFunction<Y>;
typedef VectorFormulaFunction<EffectiveNumber> EffectiveVectorFormulaFunction;

typedef Pair<Nat,EffectiveFormula> CoordinateFormulaPair;
typedef List<CoordinateFormulaPair> CoordinateFormulaPairs;

class NotFormulaFunctionException : public std::runtime_error {
  public:
    NotFormulaFunctionException(const String& str) : std::runtime_error(str) { }
};

//! \brief Returns \a true if the function \a f is syntactically constant in the indices \a is.
template<class Y> Bool is_constant_in(const ScalarMultivariateFormulaFunction<Y>& f, const Set<Nat>& is) { return is_constant_in(f._formula,is); }
//! \brief Returns \a true if the function \a f is syntactically affine in the indices \a is.
template<class Y> Bool is_affine_in(const ScalarMultivariateFormulaFunction<Y>& f, const Set<Nat>& is) { return is_affine_in(f._formula,is); }
//! \brief Returns \a true if the vector function \a f is syntactically affine in the indices \a is.
template<class Y> Bool is_affine_in(const VectorMultivariateFormulaFunction<Y>& f, const Set<Nat>& is) { return is_affine_in(f._formulae,is); }
//! \brief Returns \a true if the vector function \a f is syntactically additive (possibly with multipliers) in the indices \a is.
template<class Y> Bool is_additive_in(const VectorMultivariateFormulaFunction<Y>& f, const Set<Nat>& is) { return is_additive_in(f._formulae,is); }

template<class Y> Bool is_affine_in(const VectorMultivariateFunction<Y>& f, const Set<Nat>& is) {
    auto ff = dynamic_pointer_extract<const EffectiveVectorMultivariateFormulaFunction>(f.managed_pointer());
    if (ff == nullptr) ARIADNE_THROW(NotFormulaFunctionException,"is_affine_in(f,is)","Affinity checking currently available only for formula functions.");
    return is_affine_in(ff->formulae(),is);
}
template<class Y> Bool is_additive_in(const VectorMultivariateFunction<Y>& f, const Set<Nat>& is) {
    auto ff = dynamic_pointer_extract<const EffectiveVectorMultivariateFormulaFunction>(f.managed_pointer());
    if (ff == nullptr) ARIADNE_THROW(NotFormulaFunctionException,"is_additive_in(f,is)","Additivity checking currently available only for formula functions.");
    return is_additive_in(ff->formulae(),is);
}

inline EffectiveVectorMultivariateFunction noise_independent_component(EffectiveVectorMultivariateFunction const& function, SizeType num_inputs) {

    auto ff = dynamic_pointer_extract<const EffectiveVectorMultivariateFormulaFunction>(function.managed_pointer());
    if (ff == nullptr) ARIADNE_THROW(NotFormulaFunctionException,"noise_independent_component(f,num_inputs)","Noise independent component extraction currently available only for formula functions.");

    CoordinateFormulaPairs substitutions;
    for (auto i : range(ff->result_size(),ff->result_size()+num_inputs)) {
        substitutions.append({i,EffectiveFormula::zero()});
    }

    return EffectiveVectorMultivariateFormulaFunction(function.argument_size(),simplify(substitute(ff->formulae(),substitutions)));
}

inline Vector<EffectiveVectorMultivariateFunction> input_derivatives(EffectiveVectorMultivariateFunction const& function, SizeType num_inputs) {

    Vector<EffectiveVectorMultivariateFunction> result(num_inputs);

    auto ff = dynamic_pointer_extract<const EffectiveVectorMultivariateFormulaFunction>(function.managed_pointer());
    if (ff == nullptr) ARIADNE_THROW(NotFormulaFunctionException,"input_derivatives(f,num_inputs)","Input derivatives extraction currently available only for formula functions.");

    SizeType n = function.result_size();

    for (auto j : range(num_inputs)) {
        Vector<EffectiveFormula> derivative_formulae(n);
        for (auto i : range(n)) {
            derivative_formulae[i] = simplify(derivative(ff->formulae()[i],n+j));
        }
        result[j] = EffectiveVectorMultivariateFormulaFunction(function.argument_size(),derivative_formulae);
    }

    return result;
}


//------------------------ Arithmetic scalar functions  -----------------------------------//


//! A constant function f(x)=c
template<class Y, class... ARGS>
class ConstantFunction
{
    using P=InformationTag<Y>;
    using SIG=Real(ARGS...);
    using D=typename SignatureTraits<SIG>::DomainType;

    D _domain;
    Y _value;
  public:
    typedef D DomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;

    //ConstantFunction(SizeType as, const Y& c) : _argument_size(as), _value(c) { }
    ConstantFunction(DomainType dom, const Y& c) : _domain(dom), _value(c) { }
    operator Y() const { return _value; }

    DomainType domain() const { return _domain; }
    ArgumentSizeType argument_size() const { return _domain.dimension(); }
    SizeOne result_size() const { return SizeOne(); }
    Y const& value() const { return _value; }

    template<class X> inline X operator() (const ElementType<D,X>& x) const {
        return _make_constant(this->_value,x); }
    friend ConstantFunction<Y,ARGS...> derivative(ConstantFunction<Y,ARGS...> const& f, ElementIndexType<D> j) {
        return ConstantFunction<Y,ARGS...>(f._domain,Y(0)); }
    friend OutputStream& operator<<(OutputStream& os, ConstantFunction<Y,ARGS...> const& f) {
        return os << f._value; }
    friend OutputStream& operator<<(OutputStream& os, Representation<ConstantFunction<Y,ARGS...>> const& f) {
        return os << "ConstantFunction(" << f.reference().domain() << "," << f.reference().value() << ")"; }
  private:
    template<class X> inline static X _make_constant(Y y, X const& x) { return make_constant(y,x); }
    template<class X> inline static X _make_constant(Y y, Vector<X> const& vx) { return make_constant(y,vx.zero_element()); }


};

template<class P> using ConstantUnivariateFunction = ConstantFunction<P,RealScalar>;
template<class P> using ConstantMultivariateFunction = ConstantFunction<P,RealVector>;


//! A coordinate function \f$f:\R^n\rightarrow\R\f$ given by \f$f(x)=x_i\f$.
template<class P, class... ARGS>
class CoordinateFunction
{
    typedef Number<P> Y;
    using SIG=Real(ARGS...);
    using D=typename SignatureTraits<SIG>::DomainType;
  public:
    typedef D DomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;
  private:
    DomainType _domain;
    ArgumentIndexType _index;

  public:
    CoordinateFunction(DomainType dom, ArgumentIndexType i) : _domain(dom), _index(i) { }
    ArgumentIndexType index() const { return _index; }

    DomainType domain() const { return _domain; }
    ArgumentSizeType argument_size() const { return _domain.dimension(); }
    SizeOne result_size() const { return SizeOne(); }

    template<class X> inline X operator() (const Vector<X>& x) const { return x[this->_index]; }
    template<class X> inline X operator() (const Scalar<X>& x) const { return x; }

    friend ConstantFunction<Y,ARGS...> derivative(CoordinateFunction<P,ARGS...> const& f, ArgumentIndexType j) {
        if(j==f._index) { return ConstantFunction<Y,ARGS...>(f.domain(),Y(1)); }
        else { return ConstantFunction<Y,ARGS...>(f.domain(),Y(0)); } }
    friend OutputStream& operator<<(OutputStream& os, CoordinateFunction<P,ARGS...> const& f) {
        if constexpr (Same<Tuple<ARGS...>,Tuple<RealScalar>>) { return os << "x"; }
        else { return os << "x[" << f._index << "]"; } }
    friend OutputStream& operator<<(OutputStream& os, Representation<CoordinateFunction<P,ARGS...>> const& f) {
        return os << "CoordinateFunction(" << f.reference().argument_size() << "," << f.reference().index() << ")"; }
};


//! \brief The identity function \f$ x\mapsto x\f$ in \f$\R^n\f$.
template<class P, class... ARGS>
class UnaryFunction
{
    typedef Number<P> Y;
    using SIG=RealVector(ARGS...);
    using D=typename SignatureTraits<SIG>::DomainType;
  public:
    typedef D DomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;

    UnaryFunction(const UnaryElementaryOperator& op, const ScalarFunction<P,ARGS...>& arg)
        : _op(op), _arg(arg) { }
    DomainType domain() const { return this->_arg.domain(); }
    ArgumentSizeType argument_size() const { return this->_arg.argument_size(); }
    SizeOne result_size() const { return SizeOne(); }

    template<class X> inline X operator() (const ElementType<D,X>& x) const {
        return this->_op(this->_arg(x)); }
    friend ScalarFunction<P,ARGS...> derivative(UnaryFunction<P,ARGS...> const& f, ArgumentIndexType j) {
        return f._op.accept([&](auto op){
            if constexpr(Same<decltype(op),Abs>) { assert(false); return f._arg.derivative(j); }
            else { return op.derivative(f._arg,f._arg.derivative(j)); } } );
    }
    friend OutputStream& operator<<(OutputStream& os, UnaryFunction<P,ARGS...> const& f) {
        return os << f._op << '(' << f._arg << ')'; }
    friend OutputStream& operator<<(OutputStream& os, Representation<UnaryFunction<P,ARGS...>> const& f) {
        return os << "UnaryFunction(" << f.reference().argument_size() << "," << f.reference() << ")"; }
  private:
    UnaryElementaryOperator _op;
    ScalarFunction<P,ARGS...> _arg;
};


template<class P, class... ARGS> ScalarFunction<P,ARGS...> sqr(const ScalarFunction<P,ARGS...>& f) {
    return ScalarFunction<P,ARGS...>(new UnaryFunction<P,ARGS...>(OperatorCode::SQR,f)); }
template<class P, class... ARGS> ScalarFunction<P,ARGS...> sqrt(const ScalarFunction<P,ARGS...>& f) {
    return ScalarFunction<P,ARGS...>(new UnaryFunction<P,ARGS...>(OperatorCode::SQRT,f)); }
template<class P, class... ARGS> ScalarFunction<P,ARGS...> sin(const ScalarFunction<P,ARGS...>& f) {
    return ScalarFunction<P,ARGS...>(new UnaryFunction<P,ARGS...>(OperatorCode::SIN,f)); }
template<class P, class... ARGS> ScalarFunction<P,ARGS...> cos(const ScalarFunction<P,ARGS...>& f) {
    return ScalarFunction<P,ARGS...>(new UnaryFunction<P,ARGS...>(OperatorCode::COS,f)); }

template<class P, class... ARGS>
class BinaryFunction
{
    typedef Number<P> Y;
    using SIG=RealVector(ARGS...);
    using D=typename SignatureTraits<SIG>::DomainType;
  public:
    typedef D DomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;

    BinaryFunction(BinaryArithmeticOperator op, const ScalarFunction<P,ARGS...>& arg1, const ScalarFunction<P,ARGS...>& arg2)
        : BinaryFunction(BinaryElementaryOperator(op.code()),arg1,arg2) { }
    BinaryFunction(BinaryElementaryOperator op, const ScalarFunction<P,ARGS...>& arg1, const ScalarFunction<P,ARGS...>& arg2)
        : _op(op), _arg1(arg1), _arg2(arg2) { ARIADNE_ASSERT_MSG(arg1.argument_size()==arg2.argument_size(),"op='"<<op<<"', arg1="<<arg1<<", arg2="<<arg2); }
    DomainType domain() const {
        return intersection(this->_arg1.domain(),this->_arg2.domain()); }
    ArgumentSizeType argument_size() const {
        return this->_arg1.argument_size(); }
    SizeOne result_size() const {
        return SizeOne(); }

    template<class X> inline X operator() (const ElementType<D,X>& x) const {
        return this->_op(this->_arg1(x),this->_arg2(x)); }

    friend ScalarFunction<P,ARGS...> derivative(BinaryFunction<P,ARGS...> const& f, ElementIndexType<D> j) {
        return f._op.accept([&](auto op){
            if constexpr(Same<decltype(op),Max> || Same<decltype(op),Min>) { assert(false); return f._arg1.derivative(j); }
            else { return op.derivative(f._arg1,f._arg1.derivative(j),f._arg2,f._arg2.derivative(j)); }
        });
    }
    friend OutputStream& operator<<(OutputStream& os, BinaryFunction<P,ARGS...> const& f) {
        if(f._op.code()==OperatorCode::ADD || f._op.code()==OperatorCode::SUB) {
            return os << '(' << f._arg1 << symbol(f._op.code()) << f._arg2 << ')'; }
        else { return os << f._arg1 << symbol(f._op.code()) << f._arg2; } }
    friend OutputStream& operator<<(OutputStream& os, Representation<BinaryFunction<P,ARGS...>> const& f) {
        return os << "BinaryFunction(" << f.reference().argument_size() << "," << f.reference() << ")"; }

  private:
    BinaryElementaryOperator _op;
    ScalarFunction<P,ARGS...> _arg1;
    ScalarFunction<P,ARGS...> _arg2;
};


// \brief The power function \f$(x,n)\mapsto x^n\f$.
template<class P, class... ARGS>
class GradedFunction
{
    typedef Number<P> Y;
    using SIG=Real(ARGS...);
    using D=typename SignatureTraits<SIG>::DomainType;
  public:
    typedef D DomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    GradedFunction(GradedElementaryOperator op, const ScalarFunction<P,ARGS...>& arg1, const Int& arg2)
        : _op(op), _arg1(arg1), _arg2(arg2) {  }
    DomainType domain() const {
        return this->_arg1.domain(); }
    ArgumentSizeType argument_size() const {
        return this->_arg1.argument_size(); }
    SizeOne result_size() const {
        return SizeOne(); }

    template<class X> inline X operator() (const ElementType<D,X>& x) const {
        return this->_op(this->_arg1.evaluate(x),this->_arg2); }
    friend Function<P,SIG> derivative(GradedFunction<P,ARGS...> const& f, ElementIndexType<D> j) {
        return f._op.accept([&](auto op){ return op.derivative(f._arg1, f._arg1.derivative(j), f._arg2);}); }
    friend OutputStream& operator<<(OutputStream& os, GradedFunction<P,ARGS...> const& f) {
        return os << f._op.code() << "(" << f._arg1 << "," << f._arg2 << ")"; }
    friend OutputStream& operator<<(OutputStream& os, Representation<GradedFunction<P,ARGS...>> const& f) {
        return os << "GradedFunction(" << f.reference().argument_size() << "," << f.reference() << ")"; }
  private:
    GradedElementaryOperator _op;
    ScalarFunction<P,ARGS...> _arg1;
    Int _arg2;
};

template<class P> using UnaryMultivariateFunction = UnaryFunction<P,RealVector>;
template<class P> using BinaryMultivariateFunction = BinaryFunction<P,RealVector>;
template<class P> using GradedMultivariateFunction = GradedFunction<P,RealVector>;

//------------------------ Vector of Scalar functions  -----------------------------------//

template<class P, class... ARGS>
class NonResizableScalarFunction : public CheckedAssignable<ScalarFunction<P,ARGS...>> {
  public:
    NonResizableScalarFunction<P,ARGS...>& operator=(const ScalarFunction<P,ARGS...>& f) {
        ARIADNE_ASSERT_MSG(this->domain()==f.domain(), "this->domain()="<<this->domain()<<", f.domain()="<<f.domain()<<"\n\n*this="<<*this<<"\nf="<<f<<"\n");
        this->ScalarFunction<P,ARGS...>::operator=(f);
        return *this;
    }
};


template<class P, class... ARGS>
class VectorOfScalarFunction
{
    using SIG=RealVector(ARGS...);
    using D=typename SignatureTraits<SIG>::DomainType;
    friend class FunctionConstructors<P>;
  public:
    typedef D DomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;

    VectorOfScalarFunction(SizeType rs, SizeType as)
        : VectorOfScalarFunction(rs, ScalarFunction<P,ARGS...>(as)) { }
    VectorOfScalarFunction(SizeType rs, DomainType dom)
        : VectorOfScalarFunction(rs, ScalarFunction<P,ARGS...>(dom)) { }
    VectorOfScalarFunction(SizeType rs, const ScalarFunction<P,ARGS...>& f)
        : _dom(f.domain()), _vec(rs,f) { }
    VectorOfScalarFunction(const Vector<ScalarFunction<P,ARGS...>>& vsf)
        : _dom(vsf.zero_element().domain()), _vec(vsf) { }

    Void set(SizeType i, ScalarFunction<P,ARGS...> f) {
        if(this->argument_size()==0u) { this->_dom=f.domain(); }
        ARIADNE_ASSERT(f.argument_size()==this->argument_size());
        this->_vec[i]=f; }
    ScalarFunction<P,ARGS...> get(SizeType i) const {
        return this->_vec[i]; }

    SizeType result_size() const {
        return _vec.size(); }
    ArgumentSizeType argument_size() const {
        return _dom.dimension(); }
    DomainType domain() const {
        return _dom; }

    const ScalarFunction<P,ARGS...> operator[](SizeType i) const {
        return this->_vec[i]; }
    NonResizableScalarFunction<P,ARGS...>& operator[](SizeType i) {
        return static_cast<NonResizableScalarFunction<P,ARGS...>&>(this->_vec[i]); }

    template<class X> inline Vector<X> operator() (const ElementType<D,X>& x) const {
        Vector<X> r(this->_vec.size(),zero_element(x));
        for(SizeType i=0; i!=r.size(); ++i) {
            r[i]=_vec[i].evaluate(x); }
        return r; }

    friend VectorOfScalarFunction<P,ARGS...> derivative(VectorOfScalarFunction<P,ARGS...> const& f, ArgumentIndexType k) {
        auto chrs=get_characteristics(derivative(f._vec.zero_element(),k));
        auto r=VectorOfScalarFunction<P,ARGS...>(Vector<ScalarFunction<P,ARGS...>>(f._vec.size(),[&](SizeType i){return derivative(f._vec[i],k);},chrs)); return r; }

    friend OutputStream& operator<<(OutputStream& os, VectorOfScalarFunction<P,ARGS...> const& f) {
        os << "[";
        for(SizeType i=0; i!=f._vec.size(); ++i) {
            if(i!=0) { os << ","; }
            os << f[i]; }
        return os << "]"; }

    friend OutputStream& operator<<(OutputStream& os, Representation<VectorOfScalarFunction<P,ARGS...>> const& fr) {
        VectorOfScalarFunction<P,ARGS...> f = fr.reference();
        //os << "VoSF[R" << this->argument_size() << "->R" << this->result_size() << "]";
        os << "[";
        for(SizeType i=0; i!=f.result_size(); ++i) {
            if(i!=0) { os << ","; }
            os << representation(f[i]); }
        return os << "]"; }

  private:
    DomainType _dom;
    Vector<ScalarFunction<P,ARGS...>> _vec;

};

template<class P> using VectorOfScalarUnivariateFunction = VectorOfScalarFunction<P,RealScalar>;
template<class P> using VectorOfScalarMultivariateFunction = VectorOfScalarFunction<P,RealVector>;

template<class P, class... ARGS>
class FunctionElement
{
    using SIG=RealScalar(ARGS...);
    using D=typename SignatureTraits<SIG>::DomainType;
  public:
    typedef D DomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;

    FunctionElement(const VectorFunction<P,ARGS...>& vf, SizeType i)
        : _vf(vf), _i(i) { ARIADNE_ASSERT(i<vf.result_size()); }

    ArgumentSizeType argument_size() const { return _vf.argument_size(); }
    DomainType domain() const { return _vf.domain(); }

    template<class X> inline X operator() (const ElementType<D,X>& x) const {
        return this->_vf.evaluate(x)[_i]; }
    friend OutputStream& operator<<(OutputStream& os, FunctionElement<P,ARGS...> const& f) {
        return os<<f._vf<<"["<<f._i<<"]"; }

    friend FunctionElement<P,ARGS...> derivative(FunctionElement<P,ARGS...> const& f, ArgumentIndexType j) {
        ARIADNE_NOT_IMPLEMENTED; }
  private:
    VectorFunction<P,ARGS...> _vf;
    SizeType _i;
};

//------------------------ Results of functional operations  -----------------------------------//

template<class P, class D1, class D2, class D3, class C>
class EmbeddedFunction
{
    typedef CartesianProductType<D1,D2,D3> D;
    using ARG1=ElementKind<D1>; using ARG2=ElementKind<D2>; using ARG3=ElementKind<D3>;
    using ARG=ElementKind<D>; using RES=ElementKind<C>; using SIG=RES(ARG);
  public:

    typedef D DomainType;
    typedef C CodomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ResultSizeType ResultSizeType;

    EmbeddedFunction(ElementSizeType<D1> as1, const Function<P,RES(ARG2)>& f2, ElementSizeType<D3> as3)
        : _dom1((as1)), _f2(f2), _dom3(as3) { ARIADNE_NOT_IMPLEMENTED; }
    EmbeddedFunction(D1 dom1, const Function<P,RES(ARG2)>& f2, D3 dom3)
        : _dom1(dom1), _f2(f2), _dom3(dom3) { }
    DomainType domain() const { return product(_dom1,_f2.domain(),_dom3); }
    CodomainType const codomain() const { return _f2.codomain(); }
    ArgumentSizeType argument_size() const { return _dom1.dimension()+_f2.argument_size()+_dom3.dimension(); }
    ResultSizeType result_size() const { return _f2.result_size(); }
    template<class X> inline ElementType<C,X> operator() (const Vector<X>& x) const {
        Vector<X> px=project(x,Range(_dom1.dimension(),_dom1.dimension()+_f2.argument_size()));
        return _f2.evaluate(px); }
    template<class I> inline decltype(auto) operator[](I i) const { return EmbeddedFunction<P,D1,D2,D3,RealDomain>(_dom1,_f2[i],_dom3); }
    friend EmbeddedFunction<P,D1,D2,D3,C> derivative(EmbeddedFunction<P,D1,D2,D3,C> const& f, ArgumentSizeType j) {
        ARIADNE_NOT_IMPLEMENTED; }
    friend OutputStream& operator<<(OutputStream& os, EmbeddedFunction<P,D1,D2,D3,C> const& f) {
        return os << "EmbeddedFunction( dom1="<<f._dom1<<", f2="<<f._f2<<", dom3="<<f._dom3<<" )"; }
  private:
    D1 _dom1;
    Function<P,RES(ARG2)> _f2;
    D3 _dom3;
};


template<class P, class R, class T, class... AS>
class ComposedFunction
{
    using SIG=R(AS...);

    using D=typename DomainTraits<AS...>::EntireDomainType;
    using E=typename DomainTraits<T>::EntireDomainType;
    using C=typename DomainTraits<R>::EntireDomainType;

  public:
    typedef D DomainType;
    typedef C CodomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ResultSizeType ResultSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;
    typedef ElementIndexType<C> ResultIndexType;

    ComposedFunction(const Function<P,R(T)>& f, const Function<P,T(AS...)>& g)
        : _f(f), _g(g) { ARIADNE_ASSERT(f.argument_size()==g.result_size()); }
    DomainType domain() const { return _g.domain(); }
    CodomainType const codomain() const { return _f.codomain(); }
    ArgumentSizeType argument_size() const { return _g.argument_size(); }
    ResultSizeType result_size() const { return _f.result_size(); }
    ScalarFunction<P,AS...> operator[](ResultIndexType i) const { return compose(_f[i],_g); }

    template<class X> inline ElementType<C,X> operator() (const ElementType<D,X>& x) const {
        return _f.evaluate(_g.evaluate(x)); }
    Function<P,R(AS...)> derivative(ArgumentIndexType j) const {
        if constexpr (Same<P,ApproximateTag>) {
            ARIADNE_NOT_IMPLEMENTED;
        } else {
            if constexpr (Same<T,RealScalar>) {
                return compose(this->_f.derivative(IndexZero()),this->_g)*this->_g.derivative(j);
            } else {
                Function<P,R(AS...)> r=Function<P,R(AS...)>(this->result_size(),this->argument_size());
                for (SizeType k=0; k!=this->_g.result_size(); ++k) {
                    r=r+compose(this->_f.derivative(k),this->_g)*this->_g[k].derivative(j);
                }
                return r;
            }
        }
    }
    friend Function<P,R(AS...)> derivative(ComposedFunction<P,R,T,AS...> const& f, ArgumentIndexType j) {
        return f.derivative(j); }
    friend OutputStream& operator<<(OutputStream& os, ComposedFunction<P,R,T,AS...> const& f) {
        return os << "ComposedFunction( f="<<f._f<<", g="<<f._g<<" )"; }
  private:
  private:
    Function<P,R(T)> _f;
    Function<P,T(AS...)> _g;
};

template<class P, class D, class C1, class C2>
class JoinedFunction
{
    typedef CartesianProductType<C1,C2> C;

    using ARG=ElementKind<D>;
    using RES1=ElementKind<C2>;
    using RES2=ElementKind<C1>;
    using RES=ElementKind<C>;
    using SIG=RES(ARG);
    static_assert(Same<typename VectorFunctionMixin<JoinedFunction<P,D,C1,C2>,P,ARG>::CodomainType,CartesianProductType<C1,C2>>);

  public:
    typedef D DomainType;
    typedef C CodomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ResultSizeType ResultSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;

    JoinedFunction(Function<P,RES1(ARG)> f1, Function<P,RES2(ARG)> f2)
        : _f1(f1), _f2(f2) { ARIADNE_ASSERT(f1.argument_size()==f2.argument_size()); }
    ScalarFunction<P,ARG> operator[](SizeType i) const {
        return (i<_f1.result_size()) ? _f1[i] : _f2[i-_f1.result_size()]; }

    DomainType domain() const { return intersection(_f1.domain(),_f2.domain()); }
    CodomainType const codomain() const { return product(_f1.codomain(),_f2.codomain()); }
    SizeType result_size() const { return _f1.result_size()+_f2.result_size(); }
    ArgumentSizeType argument_size() const { return _f1.argument_size(); }
    template<class X> inline ElementType<C,X> operator() (const ElementType<D,X>& x) const {
        return join(_f1.evaluate(x),_f2.evaluate(x)); }
    friend JoinedFunction<P,D,C1,C2> derivative(JoinedFunction<P,D,C1,C2> const& f, ArgumentIndexType j) {
        ARIADNE_NOT_IMPLEMENTED; }
    friend OutputStream& operator<<(OutputStream& os, JoinedFunction<P,D,C1,C2> const& f) {
        return os << "JoinedFunction( f1="<<f._f1<<", f2="<<f._f2<<" )"; }
  private:
    Function<P,RES1(ARG)> _f1;
    Function<P,RES2(ARG)> _f2;
};


template<class P, class D1, class D2, class C1, class C2>
class CombinedFunction
{
    typedef CartesianProductType<D1,D2> D;
    typedef CartesianProductType<C1,C2> C;
    using RES=ElementKind<C>; using ARG=ElementKind<C>; using SIG=RES(ARG);
    template<class PP, class DD, class CC> using FunctionType = Function<PP,ElementKind<CC>(ElementKind<DD>)>;
    template<class PP, class DD, class CC> using FunctionInterfaceType = typename FunctionType<PP,DD,CC>::Interface;

  public:
    typedef D DomainType;
    typedef C CodomainType;
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ResultSizeType ResultSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;

    CombinedFunction(FunctionType<P,D1,C1> f1, FunctionType<P,D2,C2> f2)
        : _f1(f1), _f2(f2) { }
    DomainType domain() const { return product(_f1.domain(),_f2.domain()); }
    CodomainType codomain() const { return product(_f1.codomain(),_f2.codomain()); }
    ResultSizeType result_size() const { return _f1.result_size()+_f2.result_size(); }
    ArgumentSizeType argument_size() const { return _f1.argument_size()+_f2.argument_size(); }
    template<class X> inline ElementType<C,X> operator() (const ElementType<D,X>& x) const {
        return join(_f1.evaluate(project(x,range(0,_f1.argument_size()))),
                    _f2.evaluate(project(x,range(_f1.argument_size(),this->argument_size())))); }
    CombinedFunction<P,D1,D2,C1,C2> derivative(CombinedFunction<P,D1,D2,C1,C2> const& f, ArgumentIndexType j) {
        ARIADNE_NOT_IMPLEMENTED; }
    friend OutputStream& operator<<(OutputStream& os, CombinedFunction<P,D1,D2,C1,C2> const& f) {
        return os << "CombinedFunction( f1="<<f._f1<<", f2="<<f._f2<<" )"; }
  private:
    FunctionType<P,D1,C1> _f1;
    FunctionType<P,D2,C2> _f2;
};


template<class P, class D>
class ProjectedFunction
{
    using ARG=ElementKind<D>; using RES=RealVector; using SIG=RES(ARG);
  public:
    typedef typename SignatureTraits<SIG>::ArgumentSizeType ArgumentSizeType;
    typedef typename SignatureTraits<SIG>::ArgumentIndexType ArgumentIndexType;

    ProjectedFunction(VectorFunction<P,D> f, Projection prj)
        : _f(f), _prj(prj) { ARIADNE_PRECONDITION(f.result_size()==prj.argument_size()); }
    SizeType result_size() const { return _prj.result_size(); }
    ArgumentSizeType argument_size() const { return _f.argument_size(); }

    template<class X> inline Vector<X> operator() (const ElementType<D,X>& x) const {
        return _prj(_f(x)); }
    friend ProjectedFunction<P,D> derivative(ProjectedFunction<P,D> const& f, ArgumentIndexType j) {
        ARIADNE_NOT_IMPLEMENTED; }
    friend OutputStream& operator<<(OutputStream& os, ProjectedFunction<P,D>const& f) {
        return os << "ProjectedFunction( f="<<f._f<<", prj="<<f._prj<<" )"; }
  private:
    VectorFunction<P,D> _f;
    Projection _prj;
};


// A Lie deriviative \f$\nabla g\cdot f\f$.
template<class P>
class LieDerivativeFunction
{
  public:
    //! \brief Construct the identity function in dimension \a n.
    LieDerivativeFunction(const ScalarMultivariateFunction<P>& g, const VectorMultivariateFunction<P>& f)
        : _g(g), _dg(g.argument_size(),[&g](SizeType j){return g.derivative(j);}), _f(f)
    {
        ARIADNE_ASSERT(g.argument_size()==f.argument_size());
        ARIADNE_ASSERT(f.result_size()==f.argument_size());
    }

    SizeOne result_size() const { return _g.result_size(); }
    SizeType argument_size() const { return _g.argument_size(); }
    EuclideanDomain domain() const { return _g.domain(); }

    template<class X> inline X operator() (const Vector<X>& x) const {
        //const Vector<R> fx=_f.evaluate(x); r=0; for(SizeType i=0; i!=_dg.size(); ++i) { r+=fx[i]+_dg[i].evaluate(x); } }
        Vector<X> fx=_f.evaluate(x);
        X r=x.zero_element();
        for(SizeType i=0; i!=_dg.size(); ++i) {
            r+=fx[i]*_dg[i].evaluate(x);
        }
        return r;
    }
    friend LieDerivativeFunction<P> derivative(LieDerivativeFunction<P> const& f, SizeType j) {
        ARIADNE_NOT_IMPLEMENTED; }
    friend OutputStream& operator<<(OutputStream& os, LieDerivativeFunction<P> const& f) {
        return os << "LieDerivativeFunction( g="<<f._g<<", f="<<f._f<<" )"; }
  private:
    ScalarMultivariateFunction<P> _g;
    Vector< ScalarMultivariateFunction<P> > _dg;
    VectorMultivariateFunction<P> _f;
};


} // namespace Ariadne

#endif
