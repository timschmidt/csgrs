/***************************************************************************
 *            procedure.tcc
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

#include <iostream>

#include "utility/container.hpp"
#include "algebra/vector.hpp"

#include "numeric/operators.hpp"
#include "function/formula.hpp"
#include "algebra/expansion.hpp"
#include "geometry/box.hpp"

#include "algebra/evaluate.tpl.hpp"
#include "function/formula.tpl.hpp"

// FIXME: Added to prevent compilation error in Clang++-5.0. Should not be necessary.
#include "function/taylor_model.hpp"
#include "algebra/algebra.hpp"

namespace Ariadne {

extern template class Procedure<ApproximateNumber>;
extern template class Procedure<ValidatedNumber>;

template<class P> Procedure<Number<P>> make_procedure(ScalarMultivariateFunction<P> const& f) {
    typedef Number<P> Y;
    Formula<Y> e=f(Formula<Y>::identity(f.argument_size()));
    return Procedure<Y>(f.argument_size(),e);
}



namespace {

template<class Y> Void _convert_impl(List<ProcedureInstruction>& p, List<Y>& c, const FormulaNode<Y>& f, Map<const Void*,SizeType>& cache) {
    struct Visitor {
        List<ProcedureInstruction>& p; List<Y>& c; Map<const Void*,SizeType>& cache;
        Void operator()(ConstantFormulaNode<Y> const& f) {
            c.append(f._val); p.append(ProcedureInstruction(Cnst(),c.size()-1)); }
        Void operator()(IndexFormulaNode<Y> const& f) {
            p.append(ProcedureInstruction(Var(),f._ind)); }
        Void operator()(UnaryFormulaNode<Y> const& f) {
            p.append(ProcedureInstruction(f._op,_convert(p,c,f._arg,cache))); }
        Void operator()(BinaryFormulaNode<Y> const& f) {
            p.append(ProcedureInstruction(f._op,_convert(p,c,f._arg1,cache),_convert(p,c,f._arg2,cache))); }
        Void operator()(GradedFormulaNode<Y> const& f) {
            p.append(ProcedureInstruction(f._op,_convert(p,c,f._arg,cache),f._num)); }
        Void operator()(ScalarFormulaNode<Y> const& f) {
            c.append(f._cnst); p.append(ProcedureInstruction(f._op,c.size()-1u,_convert(p,c,f._arg,cache))); }
    };
    f.accept(Visitor{p,c,cache});
}

} // namespace

template<class Y> SizeType _convert(List<ProcedureInstruction>& p, List<Y>& c, const Formula<Y>& f, Map<const Void*,SizeType>& cache) {
    if(cache.has_key(f.node_ptr())) { return cache[f.node_ptr()]; }
    _convert_impl(p,c,f.node_ref(),cache);
    const SizeType num=p.size()-1;
    cache.insert(f.node_ptr(),num);
    return num;
}




OutputStream& operator<<(OutputStream& os, ConstantProcedureInstruction const& s) { return os << "c[" << s._val << "]"; }
OutputStream& operator<<(OutputStream& os, IndexProcedureInstruction const& s) { return os << "x[" << s._ind << "]"; }
OutputStream& operator<<(OutputStream& os, UnaryProcedureInstruction const& s) { return os << s._op << "(v[" << s._arg << "])"; }
OutputStream& operator<<(OutputStream& os, BinaryProcedureInstruction const& s) { return os << s._op << "(v[" << s._arg1 << "],v[" << s._arg2 << "])"; }
OutputStream& operator<<(OutputStream& os, GradedProcedureInstruction const& s) { return os << s._op << "(v[" << s._arg << "]," << s._num << ")"; }
OutputStream& operator<<(OutputStream& os, ScalarProcedureInstruction const& s) { return os << s._op << "(c[" << s._arg1 << "],v[" << s._arg2 << "])"; }


OutputStream& operator<<(OutputStream& os, ProcedureInstruction const& pri) { pri.accept([&os](auto s){os<<s;}); return os; }

template<class Y>
Void _write(OutputStream& os, const List<ProcedureInstruction>& p, const List<Y>& c)
{
    for(SizeType i=0; i!=p.size(); ++i) {
        os << "v[" << i << "]=" << p[i] << "; ";
    }

}



//template<class Y, class X> Formula<Y> to_formula(const Expansion<MultiIndex,X>& e) {
//    return horner_evaluate(e,Formula<Y>::identity(e.argument_size()));
//}


template<class Y>
Procedure<Y>::Procedure(SizeType as) : _argument_size(as)
{
}


template<class Y>
Procedure<Y>::Procedure(SizeType as, const Formula<Y>& f) : _argument_size(as)
{
    Map<const Void*, SizeType> ind;
    _convert(this->_instructions,this->_constants,f, ind);
}

template<class Y>
Procedure<Y>::Procedure(const ScalarMultivariateFunction<P>& f)
    : Procedure<Y>(f.argument_size(),f(Formula<Y>::identity(f.argument_size())))
{
}



template<class Y> template<ConvertibleTo<Y> X>
Procedure<Y>::Procedure(const Expansion<MultiIndex,X>& e)
    : Procedure(e.argument_size(),horner_evaluate(e,Formula<Y>::identity(e.argument_size())))
{
}



template<class Y> OutputStream& Procedure<Y>::_write(OutputStream& os) const {
    os<<"Procedure( ";
    Ariadne::_write(os,this->_instructions,this->_constants);
    os << "r=v[" << this->_instructions.size()-1u << "] )";
    return os;
}


template<class Y> Procedure<Y>& operator+=(Procedure<Y>& f, const Y& c) {
    f._constants.append(c);
    f.new_unary_instruction(OperatorCode::CNST,f._constants.size()-1);
    f.new_binary_instruction(OperatorCode::ADD,f._instructions.size()-1,f._instructions.size()-2);
    return f;
}



template<class Y>
Vector<Procedure<Y>>::Vector(SizeType as, const Vector<Formula<Y>>& f)
    : _argument_size(as),  _results(f.size(),0u)
{
    Map<const Void*, SizeType> ind;
    for(SizeType i=0; i!=f.size(); ++i) {
        _convert(this->_instructions,this->_constants,f[i], ind);
    }

    for(SizeType i=0; i!=f.size(); ++i) {
        this->_results[i]=ind[f[i].node_ptr()];
    }

}


template<class Y>
Vector<Procedure<Y>>::Vector(VectorMultivariateFunction<P> const& f)
    : Vector<Procedure<Y>>(f.argument_size(),f(Formula<Y>::identity(f.argument_size())))
{
}

template<class Y>
Vector<Procedure<Y>>::Vector(const Procedure<Y>& f)
    : _constants(f._constants)
    , _instructions(f._instructions)
    , _results(1u,f._instructions.size()-1u)
{
}


template<class Y>
OutputStream& Vector<Procedure<Y>>::_write(OutputStream& os) const {
    os<<"Procedure( ";
    Ariadne::_write(os,this->_instructions,this->_constants);
    os << "r=v" << this->_results << " )";
    return os;
}


template<class OP> struct AlgebraScalar { }
;




template<class X, class OP, class A> X evaluate(Symbolic<OP,A> const& s, X const& x);
template<class X, class OP, class A1, class A2> X evaluate(Symbolic<OP,A1,A2> const& s, X const& x1, X const& x2);
template<class X, class OP, class A1, class A2> X evaluate(Symbolic<OP,A1,A2> const& s, X const& x1, Int n2);

namespace {

template<class X, class OP> Void propagate(X& r, OP op, X const& x1, X const& x2) { r=op(x1,x2); }
template<class X, class OP> Void propagate(X& r, OP op, X const& x) { r=op(x); }
template<class X> Void propagate(X& r, BinaryElementaryOperator eop, X const& x1, X const& x2) { eop.accept([&](auto op){r=op(x1,x2);}); }
template<class X> Void propagate(X& r, UnaryElementaryOperator eop, X const& x) { eop.accept([&](auto op){r=op(x);}); }
template<class X, class N> Void propagate(X& r, GradedElementaryOperator eop, X const& x, N n) { eop.accept([&](auto op){r=op(x,n);}); }
template<class Y, class X> Void propagate(X& r, BinaryElementaryOperator eop, Y const& y1, X const& x2) { eop.accept([&](auto op){r=op(y1,x2);}); }

template<class X, class Y> Void _execute_impl(SizeType r, List<X>& v, const ProcedureInstruction& pri, const List<Y>& c, const Vector<X>& x) {
    struct Visitor {
        SizeType r; List<X>& v; const List<Y>& c; const Vector<X>& x;
        Void operator()(const ConstantProcedureInstruction& pri) { v[r]=c[pri._val]; }
        Void operator()(const IndexProcedureInstruction& pri) { v[r]=x[pri._ind]; }
        Void operator()(const UnaryProcedureInstruction& pri) { propagate(v[r],pri._op,v[pri._arg]); }
        Void operator()(const BinaryProcedureInstruction& pri) { propagate(v[r],pri._op,v[pri._arg1],v[pri._arg2]); }
        Void operator()(const GradedProcedureInstruction& pri) { propagate(v[r],pri._op,v[pri._arg],pri._num); }
        Void operator()(const ScalarProcedureInstruction& pri) { propagate(v[r],pri._op,c[pri._arg1],v[pri._arg2]); }
    };
    pri.accept(Visitor{r,v,c,x});
}

} // namespace

// \related Procedure \brief Evaluate a function \a f defined by an algorithmic procedure
template<class X, class Y> Void _execute(List<X>& v, const List<ProcedureInstruction>& p, const List<Y>& c, const Vector<X>& x) {
    ARIADNE_ASSERT(v.size()==p.size());
    for(SizeType i=0; i!=p.size(); ++i) {
        _execute_impl(i,v,p[i],c,x);
    }
}


template<class X> Void backpropagate(X const& r, Add, X& a1, X& a2) { restrict(a1,r-a2); restrict(a2,r-a1); }
template<class X> Void backpropagate(X const& r, Sub, X& a1, X& a2) { restrict(a1,a2+r); restrict(a2,a1-r); }
template<class X> Void backpropagate(X const& r, Mul, X& a1, X& a2) { restrict(a1,r/a2); restrict(a2,r/a1); }
template<class X> Void backpropagate(X const& r, Div, X& a1, X& a2) { restrict(a1,a2*r); restrict(a2,a1/r); }
template<class X> Void backpropagate(X const& r, Max, X& a1, X& a2) { restrict(a1,max(r,a2)); restrict(a2,max(r,a1)); }
template<class X> Void backpropagate(X const& r, Min, X& a1, X& a2) { restrict(a1,min(r,a2)); restrict(a2,min(r,a1)); }

template<class X, class Y> Void backpropagate(X const& r, Add, Y const& c1, X& a2) { restrict(a2,r-c1); }
template<class X, class Y> Void backpropagate(X const& r, Sub, Y const& c1, X& a2) { restrict(a2,c1-r); }
template<class X, class Y> Void backpropagate(X const& r, Mul, Y const& c1, X& a2) { restrict(a2,r/c1); }
template<class X, class Y> Void backpropagate(X const& r, Div, Y const& c1, X& a2) { restrict(a2,c1/r); }
template<class X, class Y> Void backpropagate(X const& r, Max, Y const& c1, X& a2) { restrict(a2,max(r,c1)); }
template<class X, class Y> Void backpropagate(X const& r, Min, Y const& c1, X& a2) { restrict(a2,min(r,c1)); }

template<class X, class Y> Void backpropagate(X const& r, Add, X& a1, Y const& c2) { restrict(a1,r-c2); }
template<class X, class Y> Void backpropagate(X const& r, Sub, X& a1, Y const& c2) { restrict(a1,c2+r); }
template<class X, class Y> Void backpropagate(X const& r, Mul, X& a1, Y const& c2) { restrict(a1,r/c2); }
template<class X, class Y> Void backpropagate(X const& r, Div, X& a1, Y const& c2) { restrict(a1,c2*r); }
template<class X, class Y> Void backpropagate(X const& r, Max, X& a1, Y const& c2) { restrict(a1,max(r,c2)); }
template<class X, class Y> Void backpropagate(X const& r, Min, X& a1, Y const& c2) { restrict(a1,min(r,c2)); }

template<class X> Void backpropagate(X const& r, Pos, X& a) { restrict(a,r); }
template<class X> Void backpropagate(X const& r, Neg, X& a) { restrict(a,neg(r)); }
template<class X> Void backpropagate(X const& r, Rec, X& a) { restrict(a,rec(r)); }
template<class X> Void backpropagate(X const& r, Sqr, X& a) { restrict(a,sqrt(r)); }
template<class X> Void backpropagate(X const& r, Pow, X& a, Int n) { restrict(a,exp(log(r)/n)); }
template<class X> Void backpropagate(X const& r, Sqrt, X& a) { restrict(a,sqr(r)); }
template<class X> Void backpropagate(X const& r, Exp, X& a) { restrict(a,log(r)); }
template<class X> Void backpropagate(X const& r, Log, X& a) { restrict(a,exp(r)); }
    // FIXME: restricting asin/acos of interval should not be dependent on any branch of asin/acos
template<class X> Void backpropagate(X const& r, Sin, X& a) { restrict(a,asin(r)); }
template<class X> Void backpropagate(X const& r, Cos, X& a) { restrict(a,acos(r)); }
template<class X> Void backpropagate(X const& r, Tan, X& a) { restrict(a,atan(r)); }
template<class X> Void backpropagate(X const& r, Asin, X& a) { restrict(a,sin(r)); }
template<class X> Void backpropagate(X const& r, Acos, X& a) { restrict(a,cos(r)); }
template<class X> Void backpropagate(X const& r, Atan, X& a) { restrict(a,tan(r)); }

template<class X> Void backpropagate(X const& r, Equal, X& a1, X& a2) {
    restrict(a1,r); restrict(a2,r); }
template<class X> Void backpropagate(X const& r, Leq, X& a1, X& a2) {
    restrict(a1,X(-inf,a2.upper())); restrict(a1,X(a2.lower(),+inf)); }

template<class X> Void backpropagate(X const& r, UnaryElementaryOperator eop, X& a) {
    return eop.accept([&r,&a](auto op){backpropagate(r,op,a);}); }
template<class X> Void backpropagate(X const& r, BinaryElementaryOperator eop, X& a1, X& a2) {
    return eop.accept([&r,&a1,&a2](auto op){backpropagate(r,op,a1,a2);}); }
template<class X> Void backpropagate(X const& r, GradedElementaryOperator eop, X& a1, Int n2) {
    return eop.accept([&r,&a1,n2](auto op){backpropagate(r,op,a1,n2);}); }
template<class X, class Y> Void backpropagate(X const& r, BinaryElementaryOperator eop, Y const& c1, X& a2) {
    return eop.accept([&r,&c1,&a2](auto op){backpropagate(r,op,c1,a2);}); }
template<class X, class Y> Void backpropagate(X const& r, BinaryElementaryOperator eop, X& a1, Y const& c2) {
    return eop.accept([&r,&a1,&c2](auto op){backpropagate(r,op,a1,c2);}); }

template<class X, class Y> Void _backpropagate(SizeType r, Vector<X>& x, List<X>& v, ProcedureInstruction pri, List<Y> const& c) {
    struct Visitor {
        SizeType r; Vector<X>& x; List<X>& v; List<Y> const& c;
        Void operator()(ConstantProcedureInstruction s) { }
        Void operator()(IndexProcedureInstruction s) { restrict(x[s._ind],v[r]); }
        Void operator()(UnaryProcedureInstruction s) { backpropagate(v[r],s._op,v[s._arg]); }
        Void operator()(BinaryProcedureInstruction s) { backpropagate(v[r],s._op,v[s._arg1],v[s._arg2]); }
        Void operator()(GradedProcedureInstruction s) { backpropagate(v[r],s._op,v[s._arg],s._num); }
        Void operator()(ScalarProcedureInstruction s) { backpropagate(v[r],s._op,c[s._arg1],v[s._arg2]); }
    };
    pri.accept(Visitor{r,x,v,c});
}




template<class X, class Y> Void _backpropagate(Vector<X>& x, List<X>& v, const List<ProcedureInstruction>& p, const List<Y>& c)
{
    ARIADNE_ASSERT(v.size()==p.size());
    SizeType r=p.size();
    while(r!=0u) {
        --r;
        _backpropagate(r,x,v,p[r],c);
        //p[r].accept([r,&x,&v,&c](auto s){_backpropagate(r,x,v,s,c);}
    }
    // POSTCONDITION: No nan's get propagated to x
}





template<class X, class D> void back_gradient(D const& dr, Add, X const& a1, D& da1, X const& a2, D& da2) { da1+=dr; da2+=dr; }
template<class X, class D> void back_gradient(D const& dr, Sub, X const& a1, D& da1, X const& a2, D& da2) { da1+=dr; da2-=dr; }
template<class X, class D> void back_gradient(D const& dr, Mul, X const& a1, D& da1, X const& a2, D& da2) { da1+=dr*a2; da2+=a1*dr; }
template<class X, class D> void back_gradient(D const& dr, Div, X const& a1, D& da1, X const& a2, D& da2) { da1+=dr/a2; da2-=dr/a2*(a1/a2); }
template<class X, class D> void back_gradient(D const& dr, Max, X const& a1, D& da1, X const& a2, D& da2) { assert(false); }
template<class X, class D> void back_gradient(D const& dr, Min, X const& a1, D& da1, X const& a2, D& da2) { assert(false); }

template<class Y, class X, class D> void back_gradient(D const& dr, Add op, Y const& c1, X const& a2, D& da2) { da2+=dr; }
template<class Y, class X, class D> void back_gradient(D const& dr, Sub op, Y const& c1, X const& a2, D& da2) { da2-=dr; }
template<class Y, class X, class D> void back_gradient(D const& dr, Mul op, Y const& c1, X const& a2, D& da2) { da2+=c1*dr; }
template<class Y, class X, class D> void back_gradient(D const& dr, Div op, Y const& c1, X const& a2, D& da2) { da2-=(dr/a2)*(c1/a2); }
template<class Y, class X, class D> void back_gradient(D const& dr, Max op, Y const& c1, X const& a2, D& da2) { assert(false); }
template<class Y, class X, class D> void back_gradient(D const& dr, Min op, Y const& c1, X const& a2, D& da2) { assert(false); }

template<class OP, class X, class D> void back_gradient(D const& dr, OP op, X const& a, D& da) { da+=op.derivative(a,dr); }
template<class X, class D> void back_gradient(D const& dr, Abs op, X const& a, D& da) { assert(false); }

template<class N, class X, class D> void back_gradient(D const& dr, Pow op, X const& a, D& da, N n) { da+=op.derivative(a,dr,n); }

template<class X, class D> void back_gradient(D const& dr, BinaryElementaryOperator eop, X const& a1, D& da1, X const& a2, D& da2) {
    eop.accept([&dr,&a1,&da1,&a2,&da2](auto op){return back_gradient(dr,op,a1,da1,a2,da2);}); }
template<class X, class D> void back_gradient(D const& dr, UnaryElementaryOperator eop, X const& a, D& da) {
    eop.accept([&dr,&a,&da](auto op){return back_gradient(dr,op,a,da);}); }
template<class Y, class X, class D> void back_gradient(D const& dr, BinaryElementaryOperator eop, Y const& c1, X const& a2, D& da2) {
    eop.accept([&dr,&c1,&a2,&da2](auto op){return back_gradient(dr,op,c1,a2,da2);}); }
template<class N, class X, class D> void back_gradient(D const& dr, GradedElementaryOperator eop, X const& a, D& da, N n) {
    eop.accept([&dr,&a,&da,n](auto op){return back_gradient(dr,op,a,da,n);}); }





namespace {
template<class X, class Y> void _back_gradient_impl(SizeType r, ProcedureInstruction const& pri, Vector<X> const& x, List<Y> const& c, List<X>& v, List<X>& dv, Covector<X>& dfx) {
    struct Visitor {
        SizeType r; Vector<X> const& x; List<Y> const& c; List<X>& v; List<X>& dv; Covector<X>& dfx;
        void operator()(ConstantProcedureInstruction const& s) { }
        void operator()(IndexProcedureInstruction const& s) { dfx[s._ind]+=dv[r]; }
        void operator()(UnaryProcedureInstruction const& s) { back_gradient(dv[r],s._op,v[s._arg],dv[s._arg]); }
        void operator()(BinaryProcedureInstruction const& s) { back_gradient(dv[r],s._op,v[s._arg1],dv[s._arg1],v[s._arg2],dv[s._arg2]); }
        void operator()(ScalarProcedureInstruction const& s) { back_gradient(dv[r],s._op,c[s._arg1],v[s._arg2],dv[s._arg2]); }
        void operator()(GradedProcedureInstruction const& s) { back_gradient(dv[r],s._op,v[s._arg],dv[s._arg],s._num); }
    };
    pri.accept(Visitor{r,x,c,v,dv,dfx});
}
} // namespace


template<class X, class Y> Covector<X> gradient(Procedure<Y> const& f, Vector<X> const& x) {
    X z=x.zero_element();
    auto& p=f._instructions;
    auto& c=f._constants;

    List<X> v(p.size(),z);
    List<X> dv(v.size(),z);
    Covector<X> dfx(x.size(),z);

    execute(v,f,x);

    SizeType r=p.size();
    dv[r-1]=1;
    while(r!=0u) {
        --r;
        _back_gradient_impl(r,p[r],x,c,v,dv,dfx);
    }
    return dfx;
}


} // namespace Ariadne

#include "algebra/fixed_univariate_differential.hpp"

namespace Ariadne {

template<class X, class Y> X hessian(Procedure<Y> const& f, Vector<X> const& x, Vector<X> const& s) {
    auto da=UnivariateSecondDifferential<X>::variable(s.zero_element());
    Vector<UnivariateSecondDifferential<X>> dxpas=x+da*s;
    UnivariateSecondDifferential<X> dfxpas=evaluate(f,dxpas);
    return dfxpas.hessian();
}


}


