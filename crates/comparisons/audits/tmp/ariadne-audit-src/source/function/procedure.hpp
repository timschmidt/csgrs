/***************************************************************************
 *            function/procedure.hpp
 *
 *  Copyright  2010-20  Pieter Collins
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

/*! \file function/procedure.hpp
 *  \brief Procedure to compute a real function
 */

#ifndef ARIADNE_PROCEDURE_HPP
#define ARIADNE_PROCEDURE_HPP

#include <iostream>

#include "utility/container.hpp"
#include "algebra/vector.hpp"
#include "symbolic/templates.hpp"

#include "numeric/operators.hpp"

namespace Ariadne {

class MultiIndex;
template<class I, class X> class Expansion;
template<class Y> class Formula;
template<class X> class Graded;

template<class Y> class Procedure;
//!@{
//! \ingroup FunctionModule
//! \relates Procedure
//! \name Type synonyms
using ApproximateProcedure = Procedure<ApproximateNumber>; //!< <p/>
using ValidatedProcedure = Procedure<ValidatedNumber>; //!< <p/>
using EffectiveProcedure = Procedure<EffectiveNumber>; //!< <p/>
//!@}

Void simple_hull_reduce(UpperBoxType& dom, const ValidatedProcedure& f, IntervalDomainType codom);
Void simple_hull_reduce(UpperBoxType& dom, const Vector<ValidatedProcedure>& f, BoxDomainType codom);

struct ConstantProcedureInstruction : Symbolic<Cnst,SizeType> { using Symbolic<Cnst,SizeType>::Symbolic; };
struct IndexProcedureInstruction : Symbolic<Var,SizeType> { using Symbolic<Var,SizeType>::Symbolic; };
struct UnaryProcedureInstruction : Symbolic<UnaryElementaryOperator,SizeType> {
    using Symbolic<UnaryElementaryOperator,SizeType>::Symbolic; };
struct BinaryProcedureInstruction : Symbolic<BinaryElementaryOperator,SizeType,SizeType> {
    using Symbolic<BinaryElementaryOperator,SizeType,SizeType>::Symbolic; };
struct GradedProcedureInstruction : Symbolic<GradedElementaryOperator,SizeType,Int> {
    using Symbolic<GradedElementaryOperator,SizeType,Int>::Symbolic; };
struct ScalarProcedureInstruction : Symbolic<BinaryElementaryOperator,SizeType,SizeType> {
    using Symbolic<BinaryElementaryOperator,SizeType,SizeType>::Symbolic; };

typedef Variant<ConstantProcedureInstruction,IndexProcedureInstruction,
                UnaryProcedureInstruction,BinaryProcedureInstruction,GradedProcedureInstruction,ScalarProcedureInstruction> ProcedureInstructionVariant;

struct ProcedureInstruction : public ProcedureInstructionVariant {
    explicit ProcedureInstruction(Cnst o, SizeType c)
        : ProcedureInstructionVariant(ConstantProcedureInstruction(o,c)) { }
    explicit ProcedureInstruction(Var o, SizeType i)
        : ProcedureInstructionVariant(IndexProcedureInstruction(o,i)) { }
    explicit ProcedureInstruction(UnaryElementaryOperator o, SizeType a)
        : ProcedureInstructionVariant(UnaryProcedureInstruction(o,a)) { }
    explicit ProcedureInstruction(BinaryElementaryOperator o, SizeType a1, SizeType a2)
        : ProcedureInstructionVariant(BinaryProcedureInstruction(o,a1,a2)) { }
    explicit ProcedureInstruction(GradedElementaryOperator o, SizeType a, Int n)
        : ProcedureInstructionVariant(GradedProcedureInstruction(o,a,n)) { }
  public:
    ProcedureInstruction(ProcedureInstructionVariant var) : ProcedureInstructionVariant(var) { }
    ProcedureInstructionVariant const& base() const { return *this; }
    template<class VIS> decltype(auto) accept(VIS&& vis) const {
        return std::visit(std::forward<VIS>(vis),static_cast<ProcedureInstructionVariant const&>(*this)); }
    Operator op() const { return this->accept([](auto s){return Operator(s._op);}); }
    const SizeType& val() const {
        return std::get<ConstantProcedureInstruction>(this->base())._val; }
    const SizeType& ind() const {
        return std::get<IndexProcedureInstruction>(this->base())._ind; }
    const SizeType& arg() const {
        if (auto uptr=std::get_if<UnaryProcedureInstruction>(&this->base())) { return uptr->_arg; }
        else { return std::get<GradedProcedureInstruction>(this->base())._arg; } }
    const SizeType& arg1() const {  return std::get<BinaryProcedureInstruction>(this->base())._arg1; }
    const SizeType& arg2() const {  return std::get<BinaryProcedureInstruction>(this->base())._arg2; }
    const Int& num() const { return std::get<GradedProcedureInstruction>(this->base())._num; }
  public:
    friend OutputStream& operator<<(OutputStream& os, ProcedureInstruction const& pri);
};

//! \ingroup FunctionModule
//! \brief An algorithmic procedure for computing a function.
//!
//! A Procedure is more efficient to compute than a Formula, since common
//! subexpressions have already been removed. However, it is also more
//! difficult to manipulate, so it should usually only be used when no further
//! manipulations of the function are possible.
template<class Y>
class Procedure {
    typedef Paradigm<Y> P;
  public:
    explicit Procedure(SizeType as);
    explicit Procedure(ScalarMultivariateFunction<P> const& f);
    explicit Procedure(SizeType as, const Formula<Y>& f);
    template<ConvertibleTo<Y> X> explicit Procedure(const Expansion<MultiIndex,X>& e);
    friend OutputStream& operator<<(OutputStream& os, Procedure<Y> const& p) { return p._write(os); }
  public:
    SizeType argument_size() const { return this->_argument_size; }
    template<class X, class YY> friend X evaluate(const Procedure<YY>& p, const Vector<X>& x);
  private: public:
    SizeType _argument_size;
    List<Y> _constants;
    List<ProcedureInstruction> _instructions;
  public:
    Void new_constant(Y const& c) { _constants.append(c); }
    Void new_instruction(Cnst o, SizeType c) { _instructions.append(ProcedureInstruction(o,c)); }
    Void new_instruction(Var o, SizeType i) { _instructions.append(ProcedureInstruction(o,i)); }
    Void new_instruction(UnaryElementaryOperator o, SizeType a) { _instructions.append(ProcedureInstruction(o,a)); }
    Void new_instruction(BinaryElementaryOperator o, SizeType a1, SizeType a2) { _instructions.append(ProcedureInstruction(o,a1,a2)); }
    Void new_instruction_scalar(BinaryElementaryOperator o, SizeType c1, SizeType a2) {
        _instructions.append(ProcedureInstruction(ProcedureInstructionVariant(ScalarProcedureInstruction(o,c1,a2)))); }
    Void new_instruction(GradedElementaryOperator o, SizeType a, Int n) { _instructions.append(ProcedureInstruction(o,a,n)); }
  private:
    OutputStream& _write(OutputStream& os) const;
};


//! \brief An algorithmic procedure for computing a function.
//!
//! A Procedure is more efficient to compute than a Formula, since common
//! subexpressions have already been removed. However, it is also more
//! difficult to manipulate, so it should usually only be used when no further
//! manipulations of the function are possible.
template<class Y>
class Vector<Procedure<Y>> {
    typedef Paradigm<Y> P;
  public:
    explicit Vector(SizeType as, const Vector<Formula<Y>>& f);
    explicit Vector(VectorMultivariateFunction<P> const& f);
    explicit Vector(const Procedure<Y>& p);
    friend OutputStream& operator<<(OutputStream& os, Vector<Procedure<Y>> const& p) { return p._write(os); }
  public:
    SizeType result_size() const { return _results.size(); }
    SizeType temporaries_size() const { return _instructions.size(); }
    SizeType argument_size() const { return this->_argument_size; }
    Void new_instruction(UnaryElementaryOperator o, SizeType a) { _instructions.append(ProcedureInstruction(o,a)); }
    Void new_instruction(GradedElementaryOperator o, SizeType a, Int n) { _instructions.append(ProcedureInstruction(o,a,n)); }
    Void new_instruction(BinaryElementaryOperator o, SizeType a1, SizeType a2) { _instructions.append(ProcedureInstruction(o,a1,a2)); }
    Void set_return(SizeType i, SizeType a) { _results[i]=a; }
  public:
    SizeType _argument_size;
    List<Y> _constants;
    List<ProcedureInstruction> _instructions;
    Vector<SizeType> _results;
  private:
    OutputStream& _write(OutputStream& os) const;
};

template<class X, class Y> Void _execute(List<X>& v, const List<ProcedureInstruction>& p, const List<Y>& c, const Vector<X>& x);
template<class X, class Y> Void _backpropagate(Vector<X>& x, List<X>& v, const List<ProcedureInstruction>& p, const List<Y>& c);

// \related Procedure \brief Evaluate a function \a p defined by an algorithmic procedure.
template<class X, class Y> inline X evaluate(const Procedure<Y>& p, const Vector<X>& x) {
    List<X> t(p._instructions.size(),x.zero_element());
    _execute(t,p._instructions,p._constants,x);
    return std::move(t.back());
}

// \related Procedure \brief Evaluate a function \a p defined by an algorithmic procedure.
template<class X, class Y> inline Vector<X> evaluate(const Vector<Procedure<Y>>& p, const Vector<X>& x) {
    List<X> t(p._instructions.size(),x.zero_element());
    _execute(t,p._instructions,p._constants,x);
    Vector<X> r(p.result_size());
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=std::move(t[p._results[i]]); }
    return r;
}

// \related Procedure \brief Execute the procedure \a p storing the results in \a t.
template<class X, class Y> Void execute(List<X>& t, const Procedure<Y>& p, const Vector<X>& x) {
    _execute(t,p._instructions,p._constants,x);
}

template<class X, class Y> Void execute(List<X>& t, const Vector<Procedure<Y>>& p, const Vector<X>& x) {
    _execute(t,p._instructions,p._constants,x);
}

// \related Procedure \brief Backpropagate the results of a validated procedure to the inputs \a x.
template<class X, class Y> Void backpropagate(List<X>& t, const Procedure<Y>& p, Vector<X>& x) {
    _backpropagate(t,p._instructions,p._constants,x);
}

template<class X, class Y> Void backpropagate(List<X>& t, const Vector<Procedure<Y>>& p, Vector<X>& x) {
    _backpropagate(t,p._instructions,p._constants,x);
}

// \related Procedure \brief Compute the gradient of a Procedure by backwards automatic differentiation.
template<class X, class Y> Covector<X> gradient(Procedure<Y> const& f, Vector<X> const& x);
// \related Procedure \brief Compute the second derivative of a Procedure at \a x in direction \a s.
template<class X, class Y> X hessian(Procedure<Y> const& f, Vector<X> const& x, Vector<X> const& s);

// \related Convert a function into a procedure.
template<class P> Procedure<Number<P>> make_procedure(const ScalarMultivariateFunction<P>& f);



} // namespace Ariadne

#endif // ARIADNE_PROCEDURE_HPP
