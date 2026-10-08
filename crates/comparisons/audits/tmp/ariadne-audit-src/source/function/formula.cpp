/***************************************************************************
 *            function/formula.cpp
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

#include "formula.hpp"

#include "symbolic/templates.tpl.hpp"
#include "formula.tpl.hpp"

#include "numeric/operators.tpl.hpp"

namespace Ariadne {


template<class Y> inline Operator Formula<Y>::op() const {
    return this->_root->accept([](auto fn){return static_cast<Operator>(fn._op);}); }
template<class Y> inline OperatorCode Formula<Y>::code() const {
    return this->op().code(); }
template<class Y> inline OperatorKind Formula<Y>::kind() const {
    return this->op().kind(); }
template<class Y> inline const Y& Formula<Y>::val() const {
    return std::get<ConstantFormulaNode<Y>>(*this->_root)._val; }
template<class Y> inline const Index& Formula<Y>::ind() const {
    return std::get<IndexFormulaNode<Y>>(*this->_root)._ind; }
template<class Y> inline const Formula<Y>& Formula<Y>::arg() const {
    if (std::holds_alternative<GradedFormulaNode<Y>>(*this->_root)) { return std::get<GradedFormulaNode<Y>>(*this->_root)._arg; }
    if (std::holds_alternative<ScalarFormulaNode<Y>>(*this->_root)) { return std::get<ScalarFormulaNode<Y>>(*this->_root)._arg; }
    return std::get<UnaryFormulaNode<Y>>(*this->_root)._arg; }
template<class Y> inline const Int& Formula<Y>::num() const {
    return std::get<GradedFormulaNode<Y>>(*this->_root)._num; }
template<class Y> inline const Y& Formula<Y>::cnst() const {
    return std::get<ScalarFormulaNode<Y>>(*this->_root)._cnst; }
template<class Y> inline const Formula<Y>& Formula<Y>::arg1() const {
    return std::get<BinaryFormulaNode<Y>>(*this->_root)._arg1; }
template<class Y> inline const Formula<Y>& Formula<Y>::arg2() const {
    return std::get<BinaryFormulaNode<Y>>(*this->_root)._arg2; }


template<class Y> inline Formula<Y>::Formula() : Formula(Y()) { }
template<class Y> inline Formula<Y>::Formula(const Y& c) : _root(new FormulaNode<Y>(ConstantFormulaNode<Y>(Cnst(),c))) { }
template<class Y> inline Formula<Y>::Formula(const Index& i) : _root(new FormulaNode<Y>(IndexFormulaNode<Y>(Var(),i))) { }
template<class Y> inline Formula<Y>& Formula<Y>::operator=(const Y& c) { return *this=Formula<Y>::constant(c); }

template<class Y> inline Formula<Y> Formula<Y>::create_zero() const { return Formula<Y>::constant(static_cast<Y>(0)); }
template<class Y> inline Formula<Y> Formula<Y>::create_constant(const Y& c) const { return Formula<Y>::constant(c); }

template<class Y> inline Formula<Y> Formula<Y>::zero() {
    return Formula<Y>(new FormulaNode<Y>(ConstantFormulaNode<Y>(Cnst(),static_cast<Y>(0))),PointerTag()); }
template<class Y> inline Formula<Y> Formula<Y>::constant(const Y& c) {
    return Formula<Y>(new FormulaNode<Y>(ConstantFormulaNode<Y>(Cnst(),c)),PointerTag()); }
template<class Y> inline Formula<Y> Formula<Y>::coordinate(SizeType j) {
    return Formula<Y>(new FormulaNode<Y>(IndexFormulaNode<Y>(Var(),Index(j))),PointerTag()); }
template<class Y> inline Vector<Formula<Y>> Formula<Y>::coordinates(SizeType n) {
    return Vector<Formula<Y>>(n,[](SizeType i){return Formula<Y>::coordinate(i);}); }
template<class Y> inline Vector<Formula<Y>> Formula<Y>::identity(SizeType n) {
    return Formula<Y>::coordinates(n); }
template<class Y> inline Formula<Y> Formula<Y>::unary(const UnaryElementaryOperator& op, Formula<Y> const& a) {
    return Formula<Y>(new FormulaNode<Y>(UnaryFormulaNode<Y>(op,a)),PointerTag()); }
template<class Y> inline Formula<Y> Formula<Y>::binary(const BinaryElementaryOperator& op, Formula<Y> const& a1, Formula<Y> const& a2) {
    return Formula<Y>(new FormulaNode<Y>(BinaryFormulaNode<Y>(op,a1,a2)),PointerTag()); }
template<class Y> inline Formula<Y> Formula<Y>::graded(const GradedElementaryOperator& op, Formula<Y> const& a1, Int n2) {
    return Formula<Y>(new FormulaNode<Y>(GradedFormulaNode<Y>(op,a1,n2)),PointerTag()); }
template<class Y> inline Formula<Y> Formula<Y>::scalar(const BinaryElementaryOperator& op, Y const& c1, Formula<Y> const& a2) {
    return Formula<Y>(new FormulaNode<Y>(ScalarFormulaNode<Y>(op,c1,a2)),PointerTag()); }
//template<class Y> inline Formula<Y> Formula<Y>::scalar(const BinaryElementaryOperator& op, Formula<Y> const& a1, Y const& c2) {
//    return Formula<Y>(new FormulaNode<Y>(ScalarFormulaNode<Y>(op,a1,c2)),PointerTag()); }


template<class Y> class OperatorFormulaWriter {
  public:
    template<class OP, class... ARGS> Void _write_impl(OutputStream& os, Symbolic<OP,ARGS...> const& s) const {
        OperatorSymbolicWriter<OperatorFormulaWriter<Y>>(*this)._write(os,s); }
    Void _write(OutputStream& os, Formula<Y> const& f) const {
        f.node_ref().accept([this,&os](auto s){this->_write_impl(os,s);}); }
};

template<class Y> class OperationFormulaWriter {
  public:
    template<class OP, class... ARGS> Void _write_impl(OutputStream& os, Symbolic<OP,ARGS...> const& s) const {
        OperationSymbolicWriter<OperationFormulaWriter<Y>>(*this)._write(os,s); }
    Void _write(OutputStream& os, Formula<Y> const& f) const {
        f.node_ref().accept([this,&os](auto s){this->_write_impl(os,s);}); }
};

template<class Y> class OperatorUnivariateFormulaWriter {
  public:
    template<class OP, class... ARGS> Void _write_impl(OutputStream& os, Symbolic<OP,ARGS...> const& s) const {
        OperatorSymbolicWriter<OperatorUnivariateFormulaWriter<Y>>(*this)._write(os,s); }
    Void _write_impl(OutputStream& os, Symbolic<Var,Index> const& s) const {
        os << 'x'; }
    Void _write(OutputStream& os, Formula<Y> const& f) const {
        f.node_ref().accept([this,&os](auto s){this->_write_impl(os,s);}); }
};

template<class Y> class OperationUnivariateFormulaWriter {
  public:
    template<class OP, class... ARGS> Void _write_impl(OutputStream& os, Symbolic<OP,ARGS...> const& s) const {
        OperationSymbolicWriter<OperationUnivariateFormulaWriter<Y>>(*this)._write(os,s); }
    Void _write_impl(OutputStream& os, Symbolic<Var,Index> const& s) const {
        os << 'x'; }
    Void _write(OutputStream& os, Formula<Y> const& f) const {
        f.node_ref().accept([this,&os](auto s){this->_write_impl(os,s);}); }
};


template<class Y> Formula<Y> Formula<Y>::_derivative(SizeType j) const
{
    return this->_root->accept([j](auto fn){return static_cast<Formula<Y>>(derivative(fn,j));});
}

template<class Y> OutputStream& Formula<Y>::_write_multivariate(OutputStream& os, Formula<Y> const& f) {
    OperatorFormulaWriter<Y> w;
    w._write(os,f);
    return os;
}

template<class Y> OutputStream& Formula<Y>::_write_multivariate(OutputStream& os, Vector<Formula<Y>> const& f) {
    os << "["; for (SizeType i=0; i!=f.size(); ++i) { if (i!=0) { os << ","; } _write_multivariate(os,f[i]); } os << "]"; return os;
}

template<class Y> OutputStream& Formula<Y>::_write_univariate(OutputStream& os, Formula<Y> const& f) {
    OperatorUnivariateFormulaWriter<Y> w;
    w._write(os,f);
    return os;
}

template<class Y> OutputStream& Formula<Y>::_write_univariate(OutputStream& os, Vector<Formula<Y>> const& f) {
    os << "["; for (SizeType i=0; i!=f.size(); ++i) { if (i!=0) { os << ","; } _write_univariate(os,f[i]); } os << "]"; return os;
}



template class Formula<ApproximateNumber>;
template class Formula<ValidatedNumber>;
template class Formula<EffectiveNumber>;
template class Formula<ExactNumber>;

template class Formula<Real>;

template Formula<EffectiveNumber> simplify(Formula<EffectiveNumber> const&);
template Vector<Formula<EffectiveNumber>> simplify(Vector<Formula<EffectiveNumber>> const&);
template Formula<EffectiveNumber> substitute(const Formula<EffectiveNumber>& a, const Nat& i, const Formula<EffectiveNumber>& is);

template Bool identical(Formula<EffectiveNumber> const&, Formula<EffectiveNumber> const&);

template Bool identical(Vector<Formula<EffectiveNumber>> const&, Vector<Formula<EffectiveNumber>> const&);
template Bool is_affine_in(Vector<Formula<EffectiveNumber>> const&, Set<Nat> const&);
template Bool is_additive_in(Vector<Formula<EffectiveNumber>> const&, Set<Nat> const&);

} // namespace Ariadne
