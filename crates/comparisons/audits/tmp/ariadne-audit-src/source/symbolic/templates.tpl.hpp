/***************************************************************************
 *            symbolic/templates.tpl.hpp
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

#ifndef ARIADNE_SYMBOLIC_TEMPLATES_TPL_HPP
#define ARIADNE_SYMBOLIC_TEMPLATES_TPL_HPP

#include "templates.hpp"

namespace Ariadne {


template<class T> class Variable;

template<class... OPS> class OperatorVariant;
template<class... OPS> Bool are_inverses(OperatorVariant<OPS...> const& ops1, OperatorVariant<OPS...> const& ops2);

template<class W> class OperationSymbolicWriter {
    W _w;
  public:
    OperationSymbolicWriter(W const& w) : _w(w) { }

    template<class A, template<class>class E> inline Void _write(OutputStream& os, Vector<E<A>> const& ve) {
         os << ve; }

    template<class T> inline Void _write(OutputStream& os, Constant<T> const& c) {
        os << c; }
    template<class T> inline Void _write(OutputStream& os, Variable<T> const& v) {
        os << v; }

    template<class Y> void _write(OutputStream& os, Symbolic<Cnst,Y> const& c) {
        os << c._val; }
    template<class I> void _write(OutputStream& os, Symbolic<Var,I> const& v) {
        if constexpr (Same<I,IndexZero>) { os << "x"; }
        else if constexpr (Same<I,Index>) { os << "x[" << v._ind << "]"; }
        else { os << v._ind; } }
    template<class OP, class A, template<class>class E> void _write(OutputStream& os, Symbolic<OP,E<A>> const& s) {
        os << s._op << '(' << make_writable(this->_w,s._arg) << ')'; }
    template<class OP, class A1, class A2, template<class>class E> void _write(OutputStream& os, Symbolic<OP,E<A1>,E<A2>> const& s) {
        os << s._op << '(' << make_writable(this->_w,s._arg1) << ',' << make_writable(this->_w,s._arg2) << ')'; }
    template<class OP, class A, template<class>class E> void _write(OutputStream& os, Symbolic<OP,A,E<A>> const& s) {
        os << s._op << '(' << s._cnst << ',' << make_writable(this->_w,s._arg) << ')'; }
    template<class OP, class A, template<class>class E> void _write(OutputStream& os, Symbolic<OP,E<A>,A> const& s) {
        os << s._op << '(' << make_writable(this->_w,s._arg) << ',' << s._cnst << ')'; }
    template<class OP, class A, class N, template<class>class E> requires AGraded<OP> void _write(OutputStream& os, Symbolic<OP,E<A>,N> const& s) {
        os << s._op << '(' << make_writable(this->_w,s._arg) << ',' << s._num << ')'; }
    template<class OP, class A, class I, template<class>class VE> requires AGetter<OP> void _write(OutputStream& os, Symbolic<OP,VE<A>,I> const& s) {
        os << s._op << '(' << make_writable(this->_w,s._vec) << ',' << s._ind << ')'; }
};
template<class W> OperationSymbolicWriter(W const&) -> OperationSymbolicWriter<W>;


template<class W> class OperatorSymbolicWriter {
    W _w;
  public:
    OperatorSymbolicWriter(W const& w) : _w(w) { }

    template<class Y, template<class>class E> void _write_impl(OutputStream& os, Add, E<Y> e1, E<Y> e2) {
        os << make_writable(_w,e1) << '+' << make_writable(_w,e2); }
    template<class Y, template<class>class E> void _write_impl(OutputStream& os, Sub, E<Y> e1, E<Y> e2) {
        os << make_writable(_w,e1) << '-'; switch(e2.op().code()) { case Add::code(): case Sub::code(): os << '(' << make_writable(_w,e2) << ')'; break; default: os << make_writable(_w,e2); } }
    template<class Y, template<class>class E> void _write_impl(OutputStream& os, Mul, E<Y> e1, E<Y> e2) {
        switch(e1.op().code()) { case Add::code(): case Sub::code(): case Div::code(): os << '(' << make_writable(_w,e1) << ')'; break; default: os << make_writable(_w,e1); } os << '*';
        switch(e2.op().code()) { case Add::code(): case Sub::code(): os << '(' << make_writable(_w,e2) << ')'; break; default: os << make_writable(_w,e2); } }
    template<class Y, template<class>class E> void _write_impl(OutputStream& os, Div, E<Y> e1, E<Y> e2) {
        switch(e1.op()) { case Add::code(): case Sub::code(): os << '(' << make_writable(_w,e1) << ')'; break; default: os << make_writable(_w,e1); } os << '/';
        switch(e2.op()) { case Add::code(): case Sub::code(): case Mul::code(): case Div::code(): os << '(' << make_writable(_w,e2) << ')'; break; default: os << make_writable(_w,e2); } }
    template<class E1, class E2> void _write_impl(OutputStream& os, Max, E1 const& e1, E2 const& e2) { os << "max" << '(' << make_writable(_w,e1) << ',' << make_writable(_w,e2) << ')'; }
    template<class E1, class E2> void _write_impl(OutputStream& os, Min, E1 const& e1, E2 const& e2) { os << "min" << '(' << make_writable(_w,e1) << ',' << make_writable(_w,e2) << ')'; }
    template<class OP, class E1, class E2> void _write_impl(OutputStream& os, OP op, E1 const& e1, E2 const& e2) { os << op << '(' << make_writable(_w,e1) << ',' << make_writable(_w,e2) << ')'; }

    template<class E> void _write_impl(OutputStream& os, Pos op, E const& e) {
        os << '+' << make_writable(_w,e); }
    template<class E> void _write_impl(OutputStream& os, Neg op, E const& e) {
        os << '-'; switch(e.op()) { case Cnst::code(): case Var::code(): os << make_writable(_w,e); break; default: os << '(' << make_writable(_w,e) << ')'; } }
    template<class OP, class E> void _write_impl(OutputStream& os, OP op, E const& e) {
        os << op << '(' << make_writable(_w,e) << ')'; }

    template<class T> inline Void _write(OutputStream& os, Constant<T> const& c) {
        os << c; }
    template<class T> inline Void _write(OutputStream& os, Variable<T> const& v) {
        os << v; }


    template<class Y> void _write(OutputStream& os, Symbolic<Cnst,Y> const& c) {
        os << c._val; }
    template<class I> void _write(OutputStream& os, Symbolic<Var,I> const& v) {
        if constexpr (Same<I,IndexZero>) { os << "x"; }
        else if constexpr (Same<I,Index>) { os << "x[" << v._ind << "]"; }
        else { os << v._ind; } }

    template<class A1, class A2, class... OPS> void _write(OutputStream& os, Symbolic<OperatorVariant<OPS...>,A1,A2> const& s) {
        s._op.accept([this,&os,&s](auto op){this->_write_impl(os,op,s._arg1,s._arg2);}); }
    template<class A, class... OPS> void _write(OutputStream& os, Symbolic<OperatorVariant<OPS...>,A> const& s) {
        s._op.accept([this,&os,&s](auto op){this->_write_impl(os,op,s._arg);}); }

    template<class A1, class A2> void _write(OutputStream& os, Symbolic<BinaryLogicalOperator,A1,A2> const& s) {
        s._op.accept([this,&os,&s](auto op){this->_write_impl(os,op,s._arg1,s._arg2);}); }
    template<class A1, class A2> void _write(OutputStream& os, Symbolic<BinaryComparisonOperator,A1,A2> const& s) {
        s._op.accept([this,&os,&s](auto op){this->_write_impl(os,op,s._arg1,s._arg2);}); }
    template<class A1, class A2> void _write(OutputStream& os, Symbolic<BinaryElementaryOperator,A1,A2> const& s) {
        s._op.accept([this,&os,&s](auto op){this->_write_impl(os,op,s._arg1,s._arg2);}); }
    template<class A, template<class>class E> void _write(OutputStream& os, Symbolic<BinaryElementaryOperator,E<A>,A> const& s) {
        os << '('; _write(os,Symbolic<BinaryElementaryOperator,E<A>,E<A>>(s._op,s._arg,E<A>(s._cnst))); os << ')'; }
    template<class A, template<class>class E> void _write(OutputStream& os, Symbolic<BinaryElementaryOperator,A,E<A>> const& s) {
        os << '('; this->_write(os,Symbolic<BinaryElementaryOperator,E<A>,E<A>>(s._op,E<A>(s._cnst),s._arg)); os << ')'; }
    template<class A> void _write(OutputStream& os, Symbolic<UnaryElementaryOperator,A> const& s) {
        s._op.accept([this,&os,&s](auto op){this->_write_impl(os,op,s._arg);}); }
    template<class A, class N> void _write(OutputStream& os, Symbolic<GradedElementaryOperator,A,N> const& s) {
        os << s._op << '(' << s._arg << ',' << s._num << ')'; }
    template<class V, class I> void _write(OutputStream& os, Symbolic<OperatorVariant<Get>,V,I> const& s) {
        os << s._vec << '[' << s._ind << ']'; }

    template<class OP, class A1, class A2> void _write(OutputStream& os, Symbolic<OP,A1,A2> const& s) {
        this->_write_impl(os,s._op,s._arg1,s._arg2); }
    template<class OP, class A> void _write(OutputStream& os, Symbolic<OP,A> const& s) {
        this->_write_impl(os,s._op,s._arg); }
};
template<class W> OperatorSymbolicWriter(W const&) -> OperatorSymbolicWriter<W>;


namespace {

template<class R, class OP, class E, class V> R _evaluate_as_impl(const OP& op, const E& e, const V& v) {
    return op(evaluate(e,v)); }
template<class R, class OP, class E1, class E2, class V> R _evaluate_as_impl(const OP& op, const E1& e1, const E2& e2, const V& v) {
    return op(evaluate(e1,v),evaluate(e2,v)); }
template<class R, class OP, class E, class N, class V> R _graded_evaluate_as_impl(const OP& op, const E& e, N n, const V& v) {
    return op(evaluate(e,v),n); }

template<class R, class E, class V, class... OPS> R _evaluate_as_impl(const OperatorVariant<OPS...>& op, const E& e, const V& v) {
    return op.template call_as<R>(evaluate(e,v)); }
template<class R, class E, class V> R _evaluate_as_impl(const UnaryElementaryOperator& op, const E& e, const V& v) {
    return op.call_as<R>(evaluate(e,v)); }
template<class R, class E1, class E2, class V> R _evaluate_as_impl(const BinaryElementaryOperator& op, const E1& e1, const E2& e2, const V& v) {
    return op.call_as<R>(evaluate(e1,v),evaluate(e2,v)); }
template<class R, class E1, class E2, class V> R _evaluate_as_impl(const BinaryComparisonOperator& op, const E1& e1, const E2& e2, const V& v) {
    return op.call_as<R>(evaluate(e1,v),evaluate(e2,v)); }
template<class R, class E, class N, class V> R _graded_evaluate_as_impl(const GradedElementaryOperator& op, const E& e, const N& n, const V& v) {
    return op.call_as<R>(evaluate(e,v),n); }
template<class R, class OP, class VE, class I, class V> R _get_evaluate_as_impl(const OP& op, const VE& ve, const I& i, const V& v) {
    return evaluate(ve,v)[i]; }

template<class R, class A> R evaluate_as(const Constant<R>& c, const Map<Identifier,A>& x) { return c; }
template<class R, class A> R evaluate_as(const Variable<R>& v, const Map<Identifier,A>& x) { abort(); }
template<class R> R evaluate_as(const Variable<R>& v, const Map<Identifier,R>& x) { return x[v.name()]; }
template<class R, class OP, class E, class V> R evaluate_as(const Symbolic<OP,E>& e, const V& x) {
    return _evaluate_as_impl<R>(e._op,e._arg,x); }
template<class R, class OP, class E1, class E2, class V> R evaluate_as(const Symbolic<OP,E1,E2>& e, const V& x) {
    return _evaluate_as_impl<R>(e._op,e._arg1,e._arg2,x); }
template<class R, class OP, class E, class N, class V> requires AGraded<OP> R evaluate_as(const Symbolic<OP,E,N>& e, const V& x) {
    return _graded_evaluate_as_impl<R>(e._op,e._arg,e._num,x); }
template<class R, class OP, class E, class I, class V> requires AGetter<OP> R evaluate_as(const Symbolic<OP,E,I>& e, const V& x) {
    return _get_evaluate_as_impl<R>(e._op,e._vec,e._ind,x); }

template<class R, class E, class A> R evaluate_as(const Vector<E>& ev, const Map<Identifier,A>& x) {
    return R(ev.size(), [&](SizeType i){return evaluate(ev[i],x);}); }

} // namespace


namespace {

template<class OP, class F, class J> inline decltype(auto) _derivative_impl(OP op, F const& f, J j) {
    return op.derivative(f,derivative(f,j)); }
template<class F, class J> inline auto _derivative_impl(Abs op, F const& f, J j) -> F {
    ARIADNE_THROW(std::runtime_error,"derivative(abs(f))","Cannot take derivative of non-smooth function"); }

template<class F1, class F2, class J> inline decltype(auto) _derivative_impl(Add, F1 const& f1, F2 const& f2, J j) {
    return derivative(f1,j)+derivative(f2,j); }
template<class F1, class F2, class J> inline decltype(auto) _derivative_impl(Sub, F1 const& f1, F2 const& f2, J j) {
    return derivative(f1,j)-derivative(f2,j); }
template<class F1, class F2, class J> inline decltype(auto) _derivative_impl(Mul, F1 const& f1, F2 const& f2, J j) {
    return derivative(f1,j)*f2+f1*derivative(f2,j); }
template<class F1, class F2, class J> inline decltype(auto) _derivative_impl(Div, F1 const& f1, F2 const& f2, J j) {
    return (derivative(f1,j)-derivative(f2,j)*(f1/f2))/f2; }
template<class F1, class F2, class J> inline auto _derivative_impl(Max, F1 const& f1, F2 const& f2, J j) -> decltype(max(f1,f2)){
    ARIADNE_THROW(std::runtime_error,"derivative(max(f1,f2))","Cannot take derivative of non-smooth function."); }
template<class F1, class F2, class J> inline auto _derivative_impl(Min, F1 const& f1, F2 const& f2, J j) -> decltype(min(f1,f2)) {
    ARIADNE_THROW(std::runtime_error,"derivative(min(f1,f2))","Cannot take derivative of non-smooth function."); }

template<class VF, class I, class J> inline auto _derivative_impl(Get const&, VF const& vf, I const& i, J j) -> decltype(vf[i]) {
    return derivative(vf[i],j); }

template<class F, class N, class J> inline decltype(auto) _derivative_impl(Pow op, F const& f, N const& n, J j) {
    return op.derivative(f,derivative(f,j),n); }

template<class F, class J, class... OPS> decltype(auto) _derivative_impl(UnaryElementaryOperator ops, F const& f, J j) {
    return ops.accept([&f,j](auto op){return _derivative_impl(op,f,j);}); }
template<class F1, class F2, class J, class... OPS> decltype(auto) _derivative_impl(BinaryElementaryOperator ops, F2 const& f1, F2 const& f2, J j) {
    return ops.accept([&f1,&f2,j](auto op){return _derivative_impl(op,f1,f2,j);}); }

template<class F, class J, class... OPS> decltype(auto) _derivative_impl(OperatorVariant<OPS...> ops, F const& f, J j) {
    return ops.accept([&f,j](auto op){return _derivative_impl(op,f,j);}); }
template<class F1, class F2, class J, class... OPS> decltype(auto) _derivative_impl(OperatorVariant<OPS...> ops, F1 const& f1, F2 const& f2, J j) {
    return ops.accept([&f1,&f2,j](auto op){return _derivative_impl(op,f1,f2,j);}); }
template<class F, class N, class J> decltype(auto) _derivative_impl(OperatorVariant<Pow> ops, F const& f, N n, J j) {
    return ops.accept([&f,j,n](auto op){return _derivative_impl(op,f,n,j);}); }
template<class X, template<class>class A, class J, class... OPS> decltype(auto) _derivative_impl(OperatorVariant<OPS...> ops, A<X> const& f1, X const& c2, J j) {
    return _derivative_impl(ops,f1,A<X>(c2),j); }
template<class X, template<class>class A, class J, class... OPS> decltype(auto) _derivative_impl(OperatorVariant<OPS...> ops, X const& c1, A<X> const& f2, J j) {
    return _derivative_impl(ops,A<X>(c1),f2,j); }


template<class Y, class J> decltype(auto) derivative(Symbolic<Cnst,Y> const& s, J j) {
    return Symbolic<Cnst,Y>(Y(0)); }

template<class OP, class A, class J> decltype(auto) derivative(Symbolic<OP,A> const& s, J j) {
    return _derivative_impl(s._op,s._arg,j); }
template<class OP, class A1, class A2, class J> decltype(auto) derivative(Symbolic<OP,A1,A2> const& s, J j) {
    return _derivative_impl(s._op,s._arg1,s._arg2,j); }
template<class OP, class A1, class A2, template<class>class E, class J> decltype(auto) derivative(Symbolic<OP,E<A1>,E<A2>> const& s, J j) {
    return _derivative_impl(s._op,s._arg1,s._arg2,j); }
template<class OP, class A, template<class>class E, class J> decltype(auto) derivative(Symbolic<OP,A,E<A>> const& s, J j) {
    return _derivative_impl(s._op,E<A>(s._cnst),s._arg,j); }
template<class OP, class A, template<class>class E, class J> decltype(auto) derivative(Symbolic<OP,E<A>,A> const& s, J j) {
    return _derivative_impl(s._op,E<A>(s._arg),s._cnst,j); }
template<class OP, template<class> class E, class A, class N, class J> requires AGraded<OP> decltype(auto) derivative(Symbolic<OP,E<A>,N> const& s, J j) {
    return _derivative_impl(s._op,s._arg,s._num,j); }
template<class OP, class A, class N, class J> requires AGraded<OP> decltype(auto) derivative(Symbolic<OP,A,N> const& s, J j) {
    return _derivative_impl(s._op,s._arg,s._num,j); }
template<template<class> class E, class V, class I, class J> decltype(auto) derivative(Symbolic<OperatorVariant<Get>,E<V>,I> const& s, J j) {
    return _derivative_impl(s._op,s._vec,s._ind,j); }

} // namespace


namespace {

template<class E> inline E _simpl(Add op, E const& e1, E const& e2) {
    if (is_constant(e1,0)) { return e2; }
    else if (is_constant(e2,0)) { return e1; }
    else { return op(e1,e2); } }
template<class E> inline E _simpl(Sub op, E const& e1, E const& e2) {
    if (is_constant(e1,0)) { return simplify(neg(e2)); }
    else if (is_constant(e2,0)) { return e1; }
    else if (identical(e1,e2)) { return E::constant(0); }
    else { return op(e1,e2); } }
template<class E> inline E _simpl(Mul op, E const& e1, E const& e2) {
    if (is_constant(e1,0) or is_constant(e2,1)) { return e1; }
    else if (is_constant(e2,0) or is_constant(e1,1)) { return e2; }
    else { return op(e1,e2); } }
template<class E> inline E _simpl(Div op, E const& e1, E const& e2) {
    if (is_constant(e1,0) or is_constant(e2,1)) { return e1; }
    else if (is_constant(e1,1)) { return simplify(rec(e2)); }
    else if (identical(e1,e2)) { return E::constant(1); }
    else { return op(e1,e2); } }
template<class E> inline E _simpl(Max op, E const& e1, E const& e2) { return op(e1,e2); }
template<class E> inline E _simpl(Min op, E const& e1, E const& e2) { return op(e1,e2); }

template<class E> inline E _simpl(Pow op, E const& e, Int n) {
    switch (n) {
        case -1: return simplify(rec(e));
        case 0: return E::constant(1);
        case 1: return e;
        case 2: return simplify(sqr(e));
        default: return pow(e,n);
    }
}

template<class E> inline E _simpl(AndOp op, E const& e1, E const& e2) {
    if (is_constant(e1,false) or is_constant(e2,true)) { return e1; }
    else if (is_constant(e1,true) or is_constant(e2,false)) { return e2; }
    else { return op(e1,e2); } }
template<class E> inline E _simpl(OrOp op, E const& e1, E const& e2) {
    if (is_constant(e1,true) or is_constant(e2,false)) { return e1; }
    else if (is_constant(e1,false) or is_constant(e2,true)) { return e2; }
    else { return op(e1,e2); } }
template<class E> inline E _simpl(NotOp op, E const& e) {
    if (e.op().code() == op.code()) { return e.arg(); }
    else { return op(e); }
}

template<class E, class T> inline E _simplify_node(const Constant<T>& c) { return c; }
template<class E, class T> inline E _simplify_node(const Variable<T>& v) { return v; }

template<class E, class T> inline E _simplify_node(const Symbolic<Cnst,T>& c) { return static_cast<E>(c._val); }
template<class E, class I> inline E _simplify_node(const Symbolic<Var,I>& v) { return static_cast<E>(v._ind); }

template<class E, class OP, class A> inline E _simplify_node(const Symbolic<OP,A>& s) {
    auto op = s._op;
    auto sarg = simplify(s._arg);
    if (holds_alternative<Pos>(op)) {
        return sarg;
    } else if (sarg.op().code()==Cnst::code()) { // FIXME: This is a crude test for a constant
        return op(std::get<0>(sarg.node_ref().base()).val());
    } else {
        auto* up = std::get_if<Symbolic<OP,A>>(&sarg.node_ref());
        if (up && are_inverses(op,up->_op)) { return up->_arg; }
        else { return op(sarg); }
    }
}
template<class E, class OP, class A1, class A2> inline E _simplify_node(const Symbolic<OP,A1,A2>& s) {
    auto sarg1=simplify(s._arg1);
    auto sarg2=simplify(s._arg2);
    return s.op().accept([&](auto op){return _simpl(op,sarg1,sarg2);});
}
template<class E, class OP, class A, template<class>class F> inline E _simplify_node(const Symbolic<OP,A,F<A>>& s) {
    auto sarg=simplify(s._arg);
    return s.op().accept([&](auto op){return _simpl(op,F<A>(s._cnst),sarg);});
}
template<class E, class OP, class A, class N> requires AGraded<OP> inline E _simplify_node(const Symbolic<OP,A,N>& s) {
    auto sarg=simplify(s.arg());
    return s.op().accept([&](auto op){return _simpl(op,sarg,s.num());});
}
template<class E, class OP, class VA, class I> requires AGetter<OP> inline E _simplify_node(const Symbolic<OP,VA,I>& s) {
    auto sarg=simplify(s.vec());
    return s.op().accept([&](auto op){return _simpl(op,sarg,s.ind());});
}

template<class E, class SV> inline E simplify_variant(const SV& s) {
    return s.accept([](auto en){return _simplify_node<E>(en);});
}

} // namespace

namespace {

template<class X, class V, class SE> Expression<X> _substitute(const Constant<X>& c, const V& v, const SE& s) { return c; }
template<class X, class V, class SE> Expression<X> _substitute(const Variable<X>& var, const V& v, const SE& s) { return var; }


template<class V, class SE> SE _substitute(const V& var, const V& v, const SE& s) { if (var==v) { return s; } else { return SE(var); } }
template<class V, class SE, class X> SE _substitute(const Symbolic<Cnst,X>& c, const V& v, const SE& s) { return SE(c); }
template<class V, class SE, class I> SE _substitute(const Symbolic<Var,I>& var, const V& v, const SE& s) { if (var._ind==v) { return s; } else { return SE(var._ind); } }

template<class V, class SE, class OP, class E> decltype(auto) _substitute(const Symbolic<OP,E>& e, const V& v, const SE& s) {
    return e._op(substitute(e._arg,v,s)); }
template<class V, class SE, class OP, class E1, class E2> decltype(auto) _substitute(const Symbolic<OP,E1,E2>& e, const V& v, const SE& s) {
    return e._op(substitute(e._arg1,v,s),substitute(e._arg2,v,s)); }
template<class V, class SE, class OP, class A, template<class>class E> decltype(auto) _substitute(const Symbolic<OP,A,E<A>>& e, const V& v, const SE& s) {
    return e._op(e._cnst,substitute(e._arg,v,s)); }
template<class V, class SE, class OP, class E, class N> requires AGraded<OP> decltype(auto) _substitute(const Symbolic<OP,E,N>& e, const V& v, const SE& s) {
    return e._op(substitute(e._arg,v,s),e._num); }
template<class V, class SE, class OP, class VE, class I> requires AGetter<OP> decltype(auto) _substitute(const Symbolic<OP,VE,I>& e, const V& v, const SE& s) {
    return e._op(substitute(e._vec,v,s),e._ind); }

template<class X, class V, class SE> Vector<Expression<X>> _substitute(const Vector<Expression<X>>& ve, const V& v, const SE& s) {
    return Vector<Expression<X>>(ve.size(),[&ve,&v,&s](SizeType i){return substitute(ve[i],v,s);}); }

} // namespace

namespace {

template<class T> Bool same(Vector<T> v1, Vector<T> v2) {
    if (v1.size() != v2.size()) { return false; }
    for (SizeType i=0; i!=v1.size(); ++i) { if (not same(v1[i],v2[i])) { return false; } }
    return true; }

template<class T> Bool identical(const Constant<T>& c1, const Constant<T>& c2) {
    return c1.name() == c2.name() && same(c1.value(),c2.value()); }
template<class T> Bool identical(const Variable<T>& v1, const Variable<T>& v2) {
    return v1==v2; }
template<class OP, class A> Bool identical(const Symbolic<OP,A>& s1, const Symbolic<OP,A>& s2) {
    return s1._op.code()==s2._op.code() && identical(s1._arg,s2._arg); }
template<class OP, class A1, class A2> Bool identical(const Symbolic<OP,A1,A2>& s1, const Symbolic<OP,A1,A2>& s2) {
    return s1._op.code()==s2._op.code() && identical(s1._arg1,s2._arg1) && identical(s1._arg2,s2._arg2); }
template<class OP, class A, class N> requires AGraded<OP> Bool identical(const Symbolic<OP,A,N>& s1, const Symbolic<OP,A,N>& s2) {
    return s1._op.code()==s2._op.code() && identical(s1._arg,s2._arg) && s1._num==s2._num; }
template<class OP, class V, class I> requires AGetter<OP> Bool identical(const Symbolic<OP,V,I>& s1, const Symbolic<OP,V,I>& s2) {
    return s1._op.code()==s2._op.code() && identical(s1._vec,s2._vec) && s1._ind==s2._ind; }

template<class E> Bool _identical_dispatch(const E& e1, const E& e2) { return identical(e1,e2); }
template<class E1, class E2> Bool _identical_dispatch(const E1& e1, const E2& e2) { return false; }

//template<class E1, class E2> Bool identical(const E1& e1, const E2& e2) { return _identical_dispatch(e1,e2); }

} // namespace

struct IdenticalSymbolic {
    template<class T> static Bool _identical(const Constant<T>& c1, const Constant<T>& c2) {
        return c1.name() == c2.name() && same(c1.value(),c2.value()); }
    template<class T> static Bool _identical(const Variable<T>& v1, const Variable<T>& v2) {
        return v1==v2; }
    template<class T> static Bool _identical(const Symbolic<Cnst,T>& c1, const Symbolic<Cnst,T>& c2) {
        return same(c1._val,c2._val); }
    template<class I> static Bool _identical(const Symbolic<Var,I>& v1, const Symbolic<Var,I>& v2) {
        return v1._ind==v2._ind; }
    template<class OP, class A> static Bool _identical(const Symbolic<OP,A>& s1, const Symbolic<OP,A>& s2) {
        return s1._op.code()==s2._op.code() && identical(s1._arg,s2._arg); }
    template<class OP, class A1, class A2> static Bool _identical(const Symbolic<OP,A1,A2>& s1, const Symbolic<OP,A1,A2>& s2) {
        return s1._op.code()==s2._op.code() && identical(s1._arg1,s2._arg1) && identical(s1._arg2,s2._arg2); }
    template<class OP, class A, template<class>class E> static Bool _identical(const Symbolic<OP,A,E<A>>& s1, const Symbolic<OP,A,E<A>>& s2) {
        return s1._op.code()==s2._op.code() && same(s1._cnst,s2._cnst) && identical(s1._arg,s2._arg); }
    template<class OP, class A> static Bool _identical(const Symbolic<OP,A,Int>& s1, const Symbolic<OP,A,Int>& s2) {
        return s1._op.code()==s2._op.code() && identical(s1._arg,s2._arg) && s1._num==s2._num; }
    //template<class S1, class S2> static Bool identical(const S1& s1, const S2& s2) {
     //   return false; }

    template<class S, class SV> static Bool _identical_dispatch(const S& s1, const SV& sv2) {
        auto const* s2p = std::get_if<S>(&sv2); if(s2p) { return _identical(s1,*s2p); } else { return false; } }
    template<class SV> static Bool identical_variant(const SV& sv1, const SV& sv2) {
        return sv1.accept([&sv2](auto s1){return IdenticalSymbolic::_identical_dispatch(s1,sv2);}); }
};



class UntypedVariable;

namespace {
template<class T> Set<UntypedVariable> _arguments(Constant<T> const& var) { return {}; }
template<class T> Set<UntypedVariable> _arguments(Variable<T> const& var) { return {var}; }
template<class OP, class E> Set<UntypedVariable> _arguments(Symbolic<OP,E> const& s) {
    return s._arg.arguments(); }
template<class OP, class E1, class E2> Set<UntypedVariable> _arguments(Symbolic<OP,E1,E2> const& s) {
    return join(s._arg1.arguments(),s._arg2.arguments()); }
template<class OP, class E, class N> requires AGraded<OP> Set<UntypedVariable> _arguments(Symbolic<OP,E,N> const& s) {
    return s._arg.arguments(); }
template<class OP, class VE, class I> requires AGetter<OP> Set<UntypedVariable> _arguments(Symbolic<OP,VE,I> const& s) {
    return s._vec.arguments(); }
template<class E> Set<UntypedVariable> _arguments(Vector<E> const& ve) {
    Set<UntypedVariable> res; for (SizeType i=0; i!=ve.size(); ++i) { res.adjoin(ve[i].arguments()); } return res; }
}


namespace {

template<class T> Bool _is_constant(Constant<T> const& s) { return true; }
template<class T> Bool _is_constant(Variable<T> const& s) { return false; }
template<class T> Bool _is_constant(Symbolic<Cnst,T> const& s) { return true; }
template<class OP, class... AS> Bool _is_constant(Symbolic<OP,AS...> const& s) { return false; }


template<class OP, class E, class VARS> inline Bool _is_constant_in_impl(OP op, E const& e, VARS const& vars) {
    return is_constant_in(e,vars); }
template<class OP, class E1, class E2, class VARS> inline Bool _is_constant_in_impl(OP op, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_constant_in(e1,vars) && is_constant_in(e2,vars); }
template<class E, class N, class VARS> inline Bool _is_constant_in_impl(Pow op, E const& e, N n, VARS const& vars) {
    return n==0 || is_constant_in(e,vars); }

template<class E, class VARS, class... OPS> Bool _is_constant_in_impl(OperatorVariant<OPS...> ops, E const& e, VARS const& vars) {
    return ops.accept([&e,&vars](auto op){return _is_constant_in_impl(op,e, vars);}); }
template<class E1, class E2, class VARS, class... OPS> Bool _is_constant_in_impl(OperatorVariant<OPS...> ops, E1 const& e1, E2 const& e2, VARS const& vars) {
    return ops.accept([&e1,&e2,&vars](auto op){return _is_constant_in_impl(op,e1,e2, vars);}); }
template<class E, class N, class VARS, class... OPS> Bool _is_constant_in_graded_impl(OperatorVariant<OPS...> ops, E const& e, N n, VARS const& vars) {
    return ops.accept([&e,n,&vars](auto op){return _is_constant_in_impl(op,e,n, vars);}); }
template<class VE, class I, class VARS, class... OPS> inline Bool _is_constant_in_getter_impl(OperatorVariant<OPS...> ops, VE const& ve, I i, VARS const& vars) {
    return component_is_constant_in(ve,i,vars); }

template<class T, class VARS> constexpr inline Bool is_constant_in(Constant<T> const&, VARS const&) {
    return true; }
template<class T, class VARS> inline Bool is_constant_in(Variable<T> const& v, VARS const& vars) {
    if constexpr (Same<VARS,Set<Variable<T>>>) { return not vars.contains(v); } else { return false; } }
template<class T, class VARS> constexpr inline Bool is_constant_in(Symbolic<Cnst,T> const&, VARS const& vars) {
    return true; }
template<class I, class VARS> inline Bool is_constant_in(Symbolic<Var,I> const& v, VARS const& vars) {
    return not vars.contains(v._ind); }
template<class OP, class A, class VARS> inline Bool is_constant_in(Symbolic<OP,A> const& s, VARS const& vars) {
    return _is_constant_in_impl(s._op,s._arg,vars); }
template<class OP, class A1, class A2, class VARS> inline Bool is_constant_in(Symbolic<OP,A1,A2> const& s, VARS const& vars) {
    return _is_constant_in_impl(s._op,s._arg1,s._arg2,vars); }
template<class OP, class A, template<class>class E, class VARS> inline Bool is_constant_in(Symbolic<OP,A,E<A>> const& s, VARS const& vars) {
    return _is_constant_in_impl(s._op,s._arg,vars); }
template<class OP, class A, class N, class VARS> requires AGraded<OP> inline Bool is_constant_in(Symbolic<OP,A,N> const& s, VARS const& vars) {
    return _is_constant_in_graded_impl(s._op,s._arg,s._num,vars); }
template<class OP, class V, class I, class VARS> requires AGetter<OP> inline Bool is_constant_in(Symbolic<OP,V,I> const& s, VARS const& vars) {
    return _is_constant_in_getter_impl(s._op,s._vec,s._ind,vars); }

template<class E, class VARS> inline Bool _is_affine_in_impl(Atan, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Acos, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Asin, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Cos, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Sin, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Tan, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Abs, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Sqr, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Log, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Sqrt, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Exp, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Rec, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Hlf, E const& e, VARS const& vars) { return is_affine_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Pos, E const& e, VARS const& vars) { return is_affine_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Neg, E const& e, VARS const& vars) { return is_affine_in(e,vars); }
template<class E, class VARS> inline Bool _is_affine_in_impl(Nul, E const& e, VARS const& vars) { return true; }
template<class E1, class E2, class VARS> inline Bool _is_affine_in_impl(Add, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_affine_in(e1,vars) && is_affine_in(e2,vars); }
template<class E1, class E2, class VARS> inline Bool _is_affine_in_impl(Sub, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_affine_in(e1,vars) && is_affine_in(e2,vars); }
template<class E1, class E2, class VARS> inline Bool _is_affine_in_impl(Mul, E1 const& e1, E2 const& e2, VARS const& vars) {
    return (is_affine_in(e1,vars) && is_constant_in(e2,vars)) || (is_constant_in(e1,vars) && is_affine_in(e2,vars)); }
template<class E1, class E2, class VARS> inline Bool _is_affine_in_impl(Div, E1 const& e1, E2 const& e2, VARS const& vars) {
    return (is_affine_in(e1,vars) && is_constant_in(e2,vars)); }
template<class E1, class E2, class VARS> inline Bool _is_affine_in_impl(Max, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_constant_in(e1,vars) && is_constant_in(e2,vars); }
template<class E1, class E2, class VARS> inline Bool _is_affine_in_impl(Min, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_constant_in(e1,vars) && is_constant_in(e2,vars); }
template<class E, class N, class VARS> inline Bool _is_affine_in_impl(Pow, E const& e, N n, VARS const& vars) {
    return n == 0 || (n==1 && is_affine_in(e,vars)) || is_constant_in(e,vars); }
template<class VE, class I, class VARS> inline Bool _is_affine_in_impl(Get, VE const& ve, I i, VARS const& vars) {
    return is_affine_in(ve,vars); }

template<class E, class... OPS, class VARS> Bool _is_affine_in_impl(OperatorVariant<OPS...> ops, E const& e, VARS const& vars) {
    return ops.accept([&e,&vars](auto op){return _is_affine_in_impl(op,e, vars);}); }
template<class E1, class E2, class... OPS, class VARS> Bool _is_affine_in_impl(OperatorVariant<OPS...> ops, E1 const& e1, E2 const& e2, VARS const& vars) {
    return ops.accept([&e1,&e2,&vars](auto op){return _is_affine_in_impl(op,e1,e2, vars);}); }
template<class E, class N, class VARS> inline Bool _is_affine_in_impl(GradedElementaryOperator ops, E const& e, N n, VARS const& vars) {
    return ops.accept([&e,n,&vars](auto op){return _is_affine_in_impl(op,e,n,vars);}); }
template<class VE, class I, class VARS> inline Bool _is_affine_in_impl(OperatorVariant<Get> ops, VE const& ve, I i, VARS const& vars) {
    return ops.accept([&ve,i,&vars](auto op){return _is_affine_in_impl(op,ve,i,vars);}); }

template<class T, class VARS> constexpr Bool is_affine_in(Constant<T> const&, VARS const& vars) {
    return true; }
template<class T, class VARS> constexpr Bool is_affine_in(Variable<T> const&, VARS const& vars) {
    return true; }
template<class T, class VARS> constexpr Bool is_affine_in(Symbolic<Cnst,T> const&, VARS const& vars) {
    return true; }
template<class I, class VARS> constexpr Bool is_affine_in(Symbolic<Var,I> const& v, VARS const& vars) {
    return true; }
template<class OP, class A, class VARS> Bool is_affine_in(Symbolic<OP,A> const& s, VARS const& vars) {
    return _is_affine_in_impl(s._op,s._arg,vars); }
template<class OP, class A1, class A2, class VARS> Bool is_affine_in(Symbolic<OP,A1,A2> const& s, VARS const& vars) {
    return _is_affine_in_impl(s._op,s._arg1,s._arg2,vars); }
template<class OP, class A, template<class>class E, class VARS> Bool is_affine_in(Symbolic<OP,A,E<A>> const& s, VARS const& vars) {
    return _is_affine_in_impl(s._op,E<A>(s._cnst),s._arg,vars); }
template<class OP, class A, class N, class VARS> requires AGraded<OP> Bool is_affine_in(Symbolic<OP,A,N> const& s, VARS const& vars) {
    return _is_affine_in_impl(s._op,s._arg,s._num,vars); }
template<class OP, class VE, class I, class VARS> requires AGetter<OP> Bool is_affine_in(Symbolic<OP,VE,I> const& s, VARS const& vars) {
    return _is_affine_in_impl(s._op,s._vec,s._ind,vars); }


template<class E, class VARS> inline Bool _is_polynomial_in_impl(Atan, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Acos, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Asin, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Cos, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Sin, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Tan, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Abs, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Log, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Sqrt, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Exp, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Rec, E const& e, VARS const& vars) { return is_constant_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Sqr, E const& e, VARS const& vars) { return is_polynomial_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Hlf, E const& e, VARS const& vars) { return is_polynomial_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Pos, E const& e, VARS const& vars) { return is_polynomial_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Neg, E const& e, VARS const& vars) { return is_polynomial_in(e,vars); }
template<class E, class VARS> inline Bool _is_polynomial_in_impl(Nul, E const& e, VARS const& vars) { return true; }
template<class E1, class E2, class VARS> inline Bool _is_polynomial_in_impl(Add, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_polynomial_in(e1,vars) && is_polynomial_in(e2,vars); }
template<class E1, class E2, class VARS> inline Bool _is_polynomial_in_impl(Sub, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_polynomial_in(e1,vars) && is_polynomial_in(e2,vars); }
template<class E1, class E2, class VARS> inline Bool _is_polynomial_in_impl(Mul, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_polynomial_in(e1, vars) && is_polynomial_in(e2, vars); }
template<class E1, class E2, class VARS> inline Bool _is_polynomial_in_impl(Div, E1 const& e1, E2 const& e2, VARS const& vars) {
    return (is_polynomial_in(e1,vars) && is_constant_in(e2,vars)); }
template<class E1, class E2, class VARS> inline Bool _is_polynomial_in_impl(Max, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_constant_in(e1,vars) && is_constant_in(e2,vars); }
template<class E1, class E2, class VARS> inline Bool _is_polynomial_in_impl(Min, E1 const& e1, E2 const& e2, VARS const& vars) {
    return is_constant_in(e1,vars) && is_constant_in(e2,vars); }
template<class E, class N, class VARS> inline Bool _is_polynomial_in_impl(Pow, E const& e, N n, VARS const& vars) {
    return n==0 || (n>0 && is_polynomial_in(e, vars)); }

template<class E, class... OPS, class VARS> Bool _is_polynomial_in_impl(OperatorVariant<OPS...> ops, E const& e, VARS const& vars) {
    return ops.accept([&e,&vars](auto op){return _is_polynomial_in_impl(op,e, vars);}); }
template<class E1, class E2, class... OPS, class VARS> Bool _is_polynomial_in_impl(OperatorVariant<OPS...> ops, E1 const& e1, E2 const& e2, VARS const& vars) {
    return ops.accept([&e1,&e2,&vars](auto op){return _is_polynomial_in_impl(op,e1,e2, vars);}); }
template<class E, class N, class VARS, class... OPS> requires AGraded<OperatorVariant<OPS...>> Bool _is_polynomial_in_impl(OperatorVariant<OPS...> ops, E const& e, N n, VARS const& vars) {
    return ops.accept([&e,n,&vars](auto op){return _is_polynomial_in_impl(op,e,n, vars);}); }

template<class T, class VARS> constexpr Bool is_polynomial_in(Constant<T> const&, VARS const& vars) {
    return true; }
template<class T, class VARS> constexpr Bool is_polynomial_in(Variable<T> const&, VARS const& vars) {
    return true; }
template<class T, class VARS> constexpr Bool is_polynomial_in(Symbolic<Cnst,T> const&, VARS const& vars) {
    return true; }
template<class I, class VARS> constexpr Bool is_polynomial_in(Symbolic<Var,I> const& v, VARS const& vars) {
    return true; }
template<class OP, class A, class VARS> Bool is_polynomial_in(Symbolic<OP,A> const& s, VARS const& vars) {
    return _is_polynomial_in_impl(s._op,s._arg,vars); }
template<class OP, class A1, class A2, class VARS> Bool is_polynomial_in(Symbolic<OP,A1,A2> const& s, VARS const& vars) {
    return _is_polynomial_in_impl(s._op,s._arg1,s._arg2,vars); }
template<class OP, class A, template<class>class E, class VARS> Bool is_polynomial_in(Symbolic<OP,A,E<A>> const& s, VARS const& vars) {
    return _is_polynomial_in_impl(s._op,E<A>(s._cnst),s._arg,vars); }
template<class OP, class A, class N, class VARS> requires AGraded<OP> Bool is_polynomial_in(Symbolic<OP,A,N> const& s, VARS const& vars) {
    return _is_polynomial_in_impl(s._op,s._arg,s._num,vars); }
template<class OP, class V, class I, class VARS> requires AGetter<OP> Bool is_polynomial_in(Symbolic<OP,V,I> const& s, VARS const& vars) {
    ARIADNE_NOT_IMPLEMENTED; }

}

} // namespace Ariadne

#endif
