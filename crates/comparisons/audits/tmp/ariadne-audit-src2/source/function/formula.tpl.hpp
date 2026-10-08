/***************************************************************************
 *            formula.tpl.hpp
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

#include "symbolic/templates.tpl.hpp"

namespace Ariadne {


template<class Y> using ConstantFormulaNode = Symbolic<Cnst,Y>;

template<class Y> class IndexFormulaNode : public Symbolic<Var,Index> { public: using Symbolic<Var,Index>::Symbolic; };

template<class Y> using UnaryFormulaNode = Symbolic<UnaryElementaryOperator,Formula<Y>>;
template<class Y> using BinaryFormulaNode = Symbolic<BinaryElementaryOperator,Formula<Y>,Formula<Y>>;
template<class Y> using GradedFormulaNode = Symbolic<GradedElementaryOperator,Formula<Y>,Int>;
template<class Y> using ScalarFormulaNode = Symbolic<BinaryElementaryOperator,Y,Formula<Y>>;

template<class O, class A, template<class>class E> struct Symbolic<O,A,E<A>> {
    O _op; A _cnst; E<A> _arg;
    Symbolic(O o, A c, E<A> a) : _op(o), _cnst(c), _arg(a) { }
    O op() const { return _op; } A cnst() const { return _cnst; } E<A> arg() const { return _arg; }
    template<class T> operator T() const { return _op(static_cast<T>(_cnst),static_cast<T>(_arg)); }
    template<class... AS> auto operator() (AS... vals) const -> decltype(_op(_cnst,_arg(vals...))) {
        return _op(_cnst(vals...),_arg(vals...)); }
    friend OutputStream& operator<<(OutputStream& os, Symbolic<O,A,E<A>> const& expr) {
        return os << expr._op.code() << "(" << expr._cnst << "," << expr._arg << ")"; }
};

template<class Y> using FormulaNodeVariantType = Variant<ConstantFormulaNode<Y>,IndexFormulaNode<Y>, UnaryFormulaNode<Y>,BinaryFormulaNode<Y>,GradedFormulaNode<Y>,ScalarFormulaNode<Y>>;

template<class Y>
class FormulaNode
    : public FormulaNodeVariantType<Y>
{
  public:
    template<class FN> FormulaNode(FN&& fn) : FormulaNodeVariantType<Y>(std::forward<FN>(fn)) { }
    template<class VIS> decltype(auto) accept(VIS&& vis) const {
        return std::visit(std::forward<VIS>(vis),static_cast<FormulaNodeVariantType<Y> const&>(*this)); }
    FormulaNodeVariantType<Y> const& base() const { return *this; }
};



template<class Y> inline Formula<Y> make_formula(const Y& c) {
    return Formula<Y>::constant(c); }
template<class Y> inline Formula<Y> make_formula(Cnst op, const Y& c) {
    return Formula<Y>::constant(c); }
template<class Y> inline Formula<Y> make_formula(Ind op, SizeType j) {
    return Formula<Y>::index(j); }
template<class Y> inline Formula<Y> make_formula(const UnaryElementaryOperator& op, const Formula<Y>& arg) {
    return Formula<Y>::unary(op,arg); }
template<class Y> inline Formula<Y> make_formula(const BinaryElementaryOperator& op, const Formula<Y>& arg1, const Formula<Y>& arg2) {
    return Formula<Y>::binary(op,arg1,arg2); }
template<class Y> inline Formula<Y> make_formula(const BinaryElementaryOperator& op, const Formula<Y>& arg1, const Y& arg2) {
    return Formula<Y>::binary(op,arg1,make_formula(arg2)); }
template<class Y> inline Formula<Y> make_formula(const GradedElementaryOperator& op, const Formula<Y>& arg, Int num) {
    return Formula<Y>::graded(op,arg,num); }
template<class Y, class OP> inline Formula<Y> make_formula(const OP& op, Y const& cnst, const Formula<Y>& arg) {
    OperatorCode op_code=static_cast<OperatorCode>((char)op.code()+((char)OperatorCode::SADD-(char)OperatorCode::ADD));
    return Formula<Y>::scalar(Operator(op_code,OperatorKind::SCALAR),cnst,arg); }

template<class Y> Formula<Y> make_formula(const Operator& op, const Formula<Y>& arg) {
    return make_formula(UnaryElementaryOperator(op.code()),arg); }
template<class Y> Formula<Y> make_formula(const Operator& op, const Formula<Y>& arg1, const Formula<Y>& arg2) {
    return make_formula(BinaryElementaryOperator(op.code()),arg1,arg2); }
template<class Y> Formula<Y> make_formula(const Operator& op, const Formula<Y>& arg1, Int n2) {
    return make_formula(GradedElementaryOperator(op.code()),arg1,n2); }
template<class Y> Formula<Y> make_formula(const Operator& op, const Y& c1, const Formula<Y>& arg2) {
    return make_formula(BinaryElementaryOperator(op.code()),c1,arg2); }


namespace {
template<class X, class Y> decltype(auto) evaluate(Symbolic<Cnst,Y> c, Vector<X> const& v) {
    return make_constant(c._val,v); }
template<class X, class I> decltype(auto) evaluate(Symbolic<Var,I> i, Vector<X> const& v) {
    return v[i._ind]; }
template<class X, class OP, class A> decltype(auto) evaluate(Symbolic<OP,A> s, Vector<X> const& v) {
    return s._op(evaluate(s._arg,v)); }
template<class X, class OP, class A1, class A2> decltype(auto) evaluate(Symbolic<OP,A1,A2> s, Vector<X> const& v) {
    return s._op(evaluate(s._arg1,v),evaluate(s._arg2,v)); }
template<class X, class OP, class A1, class A2, class A3> decltype(auto) evaluate(Symbolic<OP,A1,A2,A3> s, Vector<X> const& v) {
    return s._op(evaluate(s._arg1,v),evaluate(s._arg2,v),evaluate(s._arg3,v)); }
template<class X, class OP, class A> decltype(auto) evaluate(Symbolic<OP,A,Formula<A>> s, Vector<X> const& v) {
    return s._op(evaluate(Symbolic<Cnst,A>(s._cnst),v),evaluate(s._arg,v)); }
template<class X, class OP, class A> decltype(auto) evaluate(Symbolic<OP,A,Int> s, Vector<X> const& v) {
    return s._op(evaluate(s._arg,v),s._num); }

template<class X, class Y> decltype(auto) _cached_evaluate(Symbolic<Cnst,Y> const& c, Vector<X> const& v, Map<const Void*,X>& cache) {
    return make_constant(c._val,v); }
template<class X, class I> decltype(auto) _cached_evaluate(Symbolic<Var,I> const& i, Vector<X> const& v, Map<const Void*,X>& cache) {
    return v[i._ind]; }
template<class X, class OP, class A> decltype(auto) _cached_evaluate(Symbolic<OP,A> const& s, Vector<X> const& v, Map<const Void*,X>& cache) {
    return s._op(cached_evaluate(s._arg,v)); }
template<class X, class OP, class A> decltype(auto) _cached_evaluate(Symbolic<OP,A,Formula<A>> const& s, Vector<X> const& v, Map<const Void*,X>& cache) {
    return s._op(evaluate(ConstantFormulaNode<A>(s._cnst),v),cached_evaluate(s._arg,v)); }
template<class X, class OP, class A> decltype(auto) _cached_evaluate(Symbolic<OP,Formula<A>,A> const& s, Vector<X> const& v, Map<const Void*,X>& cache) {
    return s._op(s._cnst,cached_evaluate(s._arg,v)); }
template<class X, class OP, class A> decltype(auto) _cached_evaluate(Symbolic<OP,A,Int> const& s, Vector<X> const& v, Map<const Void*,X>& cache) {
    return s._op(cached_evaluate(s._arg,v),s._num); }
template<class X, class OP, class A1, class A2> decltype(auto) _cached_evaluate(Symbolic<OP,A1,A2> const& s, Vector<X> const& v, Map<const Void*,X>& cache) {
    return s._op(cached_evaluate(s._arg1,v),cached_evaluate(s._arg2,v)); }
template<class X, class OP, class A1, class A2, class A3> decltype(auto) _cached_evaluate(Symbolic<OP,A1,A2,A3> const& s, Vector<X> const& v, Map<const Void*,X>& cache) {
    return s._op(cached_evaluate(s._arg1,v),cached_evaluate(s._arg2,v),cached_evaluate(s._arg3,v)); }

template<class X, class OP, class... AS> decltype(auto) _cached_evaluate(Symbolic<OP,AS...> const& s, Vector<X> const& v, Map<const Void*,X>& cache) {
    return v[0]; }

} // namespace


template<class X, class Y> X direct_evaluate(const Formula<Y>& f, const Vector<X>& x) {
    return f.node_ref().accept([&x](auto s){return evaluate(s,x);});
}


//! \brief Convert the expression with index type \c I to one with variables indexed by \a J.
template<class X, class Y> const X& cached_evaluate(const Formula<Y>& f, const Vector<X>& x, Map<const Void*,X>& cache) {
    const FormulaNode<Y>* fptr=f.node_ptr();
    if(cache.has_key(fptr)) { return cache.get(fptr); }
    else { X r=f.node_ref().accept([&x,&cache](auto s){return _cached_evaluate(s,x,cache);}); return insert(cache,fptr,r); }
}

template<class X, class Y> inline X cached_evaluate(const Formula<Y>& f, const Vector<X>& v) {
    Map<const Void*,X> cache;
    return cached_evaluate(f,v,cache);
}

template<class X, class Y> Vector<X> cached_evaluate(const Vector<Formula<Y>>& f, const Vector<X>& v) {
    assert(v.size()!=0);
    Vector<X> r(f.size(),zero_element(v));
    Map<const Void*,X> cache;
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=cached_evaluate(f[i],v,cache);
    }
    return r;
}

template<class X, class Y> X evaluate(const Formula<Y>& f, const Vector<X>& x) {
    return cached_evaluate(f,x);
}
template<class X, class Y> Vector<X> evaluate(const Vector<Formula<Y>>& f, const Vector<X>& x) {
    return cached_evaluate(f,x);
}




namespace {
template<class Y, class J> decltype(auto) derivative(IndexFormulaNode<Y> const& s, J j) { return ConstantFormulaNode<Y>(Y(s._ind==j?1:0)); }
} // namespace



//! \ingroup FunctionModule
//! \brief Convert a power-series expansion into a formula using a version of Horner's rule.
//! See J. M. Pena and T. Sauer, "On the multivariate Horner scheme", SIAM J. Numer. Anal. 37(4) 1186-1197, 2000.
//!
template<class X> Formula<X> formula(const Expansion<MultiIndex,X>& e)
{
    Vector<Formula<X>> identity(e.argument_size());
    for(SizeType i=0; i!=identity.size(); ++i) { identity[i]=Formula<X>::coordinate(i); }
    return horner_evaluate(e,identity);
}

namespace {
template<class X, class Y> inline const Formula<Y>& _substitute_coordinate(const Nat& ie, const Nat& is, const Formula<Y>& a, const Formula<X>& s) {
    ARIADNE_ASSERT_MSG(ie!=is,"Cannot substitute formula "<<s<<" for coordinate "<<ie);
    return a; }
template<class Y> inline const Formula<Y>& _substitute_coordinate(const Nat& ie, const Nat& is, const Formula<Y>& e, const Formula<Y>& s) {
    return ie==is ? s : e; }
} // namespace

template<class Y> Formula<Y> substitute(const Formula<Y>& a, const Nat& i, const Formula<Y>& is) {
    return a.node_ref().accept([&i,&is](auto fn){return _substitute(fn,i,is);});
}


template<class Y> inline Bool is_constant(const Formula<Y>& f, const SelfType<Y>& c) {
    auto const* cfn = std::get_if<ConstantFormulaNode<Y>>(&f.node_ref()); return cfn && same(cfn->val(),c);
}

template<class Y> inline Formula<Y> simplify(const Formula<Y>& a) {
    return a.node_ref().accept([](auto fn){return _simplify_node<Formula<Y>>(fn);});
}



inline Bool same(EffectiveNumber const& v1, EffectiveNumber const& v2) {
    // FIXME: Use symbolic approach
    DoublePrecision pr;
    FloatDPBounds x1(v1,pr);
    FloatDPBounds x2(v2,pr);
    return x1.lower_raw()==x2.upper_raw() && x1.upper_raw() == x2.lower_raw();
}

template<class Y> Bool identical(const Formula<Y>& a1, const Formula<Y>& a2)
{
    return IdenticalSymbolic::identical_variant(a1.node_ref(),a2.node_ref());
}

template<class Y> Vector<Formula<Y>> simplify(const Vector<Formula<Y>>& f) {
    return Vector<Formula<Y>>(f.size(),[&f](SizeType i){return simplify(f[i]);});
}


template<class Y> Bool identical(const Vector<Formula<Y>>& f1, const Vector<Formula<Y>>& f2) {
    if (f1.size() != f2.size()) { return false; }
    for (auto i : range(f1.size())) {
        if (not identical(f1[i],f2[i])) { return false; }
    }
    return true;
}

template<class Y> Bool is_constant(Formula<Y> const& f) { return f.node_ref().accept([](auto fn){return _is_constant(fn);}); }

template<class Y> Bool is_constant_in(const Formula<Y>& f, const Set<Nat>& is) {
    return f.node_ref().accept([&is](auto fn){return is_constant_in(fn,is);});
}


template<class Y> Bool is_affine_in(const Formula<Y>& f, const Set<Nat>& is) {
    return f.node_ref().accept([&is](auto fn){return is_affine_in(fn,is);});
}

template<class Y> Bool is_affine_in(const Vector<Formula<Y>>& fs, const Set<Nat>& is) {
    for (auto idx : range(fs.size())) {
        if (not is_affine_in(fs[idx],is)) { return false; }
    }
    return true;
}

template<class Y> Bool is_additive_in(const Vector<Formula<Y>>& fs, const Set<Nat>& is) {
    // We treat the vector of formulas as additive in is if each variable in is appears at most once in all expressions,
    // with a constant multiplier
    // (FIXME: this simplifies the case of a diagonalisable matrix of constant multipliers)

    for (auto i : is) {
        Bool already_found = false;
        for (auto idx : range(fs.size())) {
            const Formula<Y>& f = fs[idx];
            if (not is_constant_in(f,{i})) {
                if (already_found) {
                    return false;
                } else {
                    already_found = true;
                    auto dfi = simplify(derivative(f, i));
                    if (not is_constant(dfi)) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}


} // namespace Ariadne
