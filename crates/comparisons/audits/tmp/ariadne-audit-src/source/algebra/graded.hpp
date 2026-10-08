/***************************************************************************
 *            algebra/graded.hpp
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

/*! \file algebra/graded.hpp
 *  \brief Graded algebras.
 */

#ifndef ARIADNE_GRADED_HPP
#define ARIADNE_GRADED_HPP

#include "utility/uniform_list.hpp"
#include "function/procedure.hpp"

namespace Ariadne {

template<class R, class A> inline R cast_sign(A&& a) { return static_cast<R>(std::forward<A>(a)); }

struct AntiDiff { };

template<class Op, class... AS> struct ClosureExpression;

template<class Op, class A> struct ClosureExpression<Op,A> {
    const A& arg;
    ClosureExpression(const A& a) : arg(a) { }
};

template<class Op, class A1, class A2> struct ClosureExpression<Op,A1,A2> {
    const A1& arg1; const A2& arg2;
    ClosureExpression(const A1& a1, const A2& a2) : arg1(a1), arg2(a2) { }
};

template<class Op, class A1, class A2, class A3> struct ClosureExpression<Op,A1,A2,A3> {
    const A1& arg1; const A2& arg2; const A2& arg3;
    ClosureExpression(const A1& a1, const A2& a2, const A3& a3) : arg1(a1), arg2(a2), arg3(a3) { }
};

template<class Op, class A> ClosureExpression<Op,A>
make_expression(Op op, const A& a) {
    return ClosureExpression<Op,A>(a);
}

template<class Op, class A1, class A2> ClosureExpression<Op,A1,A2>
make_expression(Op op, const A1& a1, const A2& a2) {
    return ClosureExpression<Op,A1,A2>(a1,a2);
}

template<class Op, class A1, class A2, class A3> ClosureExpression<Op,A1,A2,A3>
make_expression(Op op, const A1& a1, const A2& a2, const A3& a3) {
    return ClosureExpression<Op,A1,A2,A3>(a1,a2,a3);
}



inline Bool compatible(const FloatDP& x1, const FloatDP& x2) { return true; }
template<class X> inline Bool compatible(const Differential<X>& x1, const Differential<X>& x2) { return x1.argument_size()==x2.argument_size(); }

inline FloatDP create(const FloatDP& x) { return FloatDP(0,dp); }
//inline ExactIntervalType create(const ExactIntervalType& x) { return ExactIntervalType(0); }
template<class X> inline Differential<X> create(const Differential<X>& x) { return Differential<X>(x.argument_size(),x.degree(),x.zero_coefficient()); }

template<class X> inline X create(const X& x) { return nul(x); }

template<class A> class Graded : public UniformList<A>
{
  public:
    typedef typename A::NumericType NumericType;
    //typedef Tuple<DegreeType,Ariadne::CharacteristicsType<A>> CharacteristicsType;
    typedef Ariadne::CharacteristicsType<A> CharacteristicsType;
    typedef Graded<A> SelfType;

    Graded(const A& a) : UniformList<A>(1u,a) { }
    Graded(const List<A>& lst) : UniformList<A>(lst) { }
    Graded(const UniformList<A>& lst) : UniformList<A>(lst) { }
    Graded(const InitializerList<A>& lst) : Graded(List<A>(lst)) { }
    template<class... PRS> requires Constructible<UniformList<A>,PRS...> and Constructible<A,ExactDouble,PRS...> Graded(const InitializerList<ExactDouble>& lst, PRS... prs)
        : UniformList<A>(prs...) { for(auto x : lst) { this->UniformList<A>::append(A(x,prs...)); } }
    Graded(DegreeType deg, Ariadne::CharacteristicsType<A> prs) : UniformList<A>(deg+1u, prs) { }
    Graded(Ariadne::CharacteristicsType<A> prs) : UniformList<A>(prs) { }
    Graded(const Graded<A>& a) : UniformList<A>(static_cast<UniformList<A>const&>(a)) { }
    Graded<A>& operator=(const Graded<A>& a) { this->UniformList<A>::operator=(a); return *this; }
    Graded<A>& operator=(Graded<A>&& a) { this->UniformList<A>::operator=(a); return *this; }
    Graded<A>& operator=(const NumericType& x) {
        this->append(this->zero_element()); if (this->size()==1) { (*this)[0]=x; } return *this; }
    template<class Y> requires Assignable<Y,NumericType> Graded<A>& operator=(const Y& y) {
        this->append(this->zero_element()); if (this->size()==1) { (*this)[0]=y; } return *this; }
    template<class Op> Void operator=(const ClosureExpression<Op,SelfType>& expr);
    template<class Op> Void operator=(const ClosureExpression<Op,SelfType,SelfType>& expr);
    template<class Op, class N> Void operator=(const ClosureExpression<Op,SelfType,N>& expr);
    Void operator=(const ClosureExpression<AntiDiff,SelfType>& ad);
    static Graded<A> constant(A const& c, DegreeType deg) {
        assert(deg>=1); A z=nul(c); Graded<A> r(c); for(DegreeType d=1; d<=deg; ++d) { r.append(z); } return r; }
    static Graded<A> variable(A const& a0, DegreeType deg) {
        assert(deg>=1); A z=nul(a0); Graded<A> r(a0); r.append(z+1);
        for(DegreeType d=2; d<=deg; ++d) { r.append(z); } return r; }
    Graded<A> create_zero() const { return Graded<A>(UniformList<A>(this->degree()+1u, Ariadne::create_zero((*this)[0u]))); }
//    CharacteristicsType characteristics() const {
//        return std::make_tuple(this->degree(),get_characteristics((*this)[0])); }
    Ariadne::CharacteristicsType<A> characteristics() const { return this->UniformList<A>::element_characteristics(); }
    DegreeType degree() const { return this->size()-1u; }
    Void extend(const A& a) { this->UniformList<A>::append(a); }
    OutputStream& _write(OutputStream& os) const;
    friend OutputStream& operator<<(OutputStream& os, Graded<A> const& g) { return g._write(os); }
};

template<class A> OutputStream& Graded<A>::_write(OutputStream& os) const {
    Graded<A> const& g=*this;
    if(g.size()==0) { return os << "G[-]{}"; }
    os << "G[" << g.degree() << "]{";
    os << "(" << g[0] << ")";
    for(DegreeType i=1; i<=g.degree(); ++i) {
        os << " + (" << g[i] << ")*t";
        if(i>1) { os << "^"<<i; }
    }
    os << "}";
    return os;
}

template<> inline OutputStream& Graded<FloatDP>::_write(OutputStream& os) const {
    Graded<FloatDP> const& g=*this;
    if(g.size()==0) { return os << "G[-]{}"; }
    os << "G[" << g.degree() << "]{";
    Bool nonzero=false;
    if(g[0]!=0) { os << g[0]; nonzero=true; }
    for(DegreeType i=1; i<=g.degree(); ++i) {
        if(g[i]>0) {
            if(nonzero) { os << "+"; }
            if(g[i]==1) { os << "t"; } else { os << g[i]<<"*t"; }
            if(i>1) { os << "^"<<i; }
            nonzero=true;
        } else if(g[i]<0) {
            if(g[i]==-1) { os << "-t"; } else { os << g[i]<<"*t"; }
            if(i>1) { os << "^"<<i; }
            nonzero=true;
         }
    }
    if(!nonzero) { os << "0"; }
    os << "}";
    return os;
}

/*
template<> inline OutputStream& Graded<ExactIntervalType>::_write(OutputStream& os) const {
    Graded<ExactIntervalType> const& g=*this;
    if(g.size()==0) { return os << "0"; }
    os << g[0];
    for(DegreeType i=1; i<=g.degree(); ++i) {
        os << "+"<<g[i]<<"*t";
        if(i>1) { os << "^"<<i; }
    }
    return os;
}
*/

template<class X> Void compute(X& r, const Nul&, const X& a) { return nul(r,a); }
template<class X> Void compute(X& r, const Pos&, const X& a) { return pos(r,a); }
template<class X> Void compute(X& r, const Neg&, const X& a) { return neg(r,a); }
template<class X> Void compute(X& r, const Hlf&, const X& a) { return hlf(r,a); }
template<class X> Void compute(X& r, const Add&, const X& a1, const X& a2) { return add(r,a1,a2); }
template<class X> Void compute(X& r, const Sub&, const X& a1, const X& a2) { return sub(r,a1,a2); }
template<class X> Void compute(X& r, const Mul&, const X& a1, const X& a2) { return mul(r,a1,a2); }
template<class X> Void compute(X& r, const Div&, const X& a1, const X& a2) { return div(r,a1,a2); }
template<class X> Void compute(X& r, const Pow&, const X& a, const Int& n) { return pow(r,a,n); }
template<class X> Void compute(X& r, const Sqr&, const X& a) { return sqr(r,a); }
template<class X> Void compute(X& r, const Sqrt&, const X& a) { return sqrt(r,a); }
template<class X> Void compute(X& r, const Exp&, const X& a) { return exp(r,a); }
template<class X> Void compute(X& r, const Log&, const X& a) { return log(r,a); }
template<class X> Void compute(X& r, const Rec&, const X& a) { return rec(r,a); }
template<class X> Void compute(X& r, const Sin&, const X& a) { return sin(r,a); }
template<class X> Void compute(X& r, const Cos&, const X& a) { return cos(r,a); }
template<class X> Void compute(X& r, const Tan&, const X& a) { return tan(r,a); }
template<class X> Void compute(X& r, const Asin&, const X& a) { return asin(r,a); }
template<class X> Void compute(X& r, const Acos&, const X& a) { return acos(r,a); }
template<class X> Void compute(X& r, const Atan&, const X& a) { return atan(r,a); }

template<class A, class B> Graded<A>& operator+=(Graded<A>& a, const B& c) {
    ARIADNE_ASSERT(a.size()>0); if(a.degree()==0) { a[0]+=c; } return a; }
template<class A, class B> Graded<A>& operator*=(Graded<A>& a, const B& c) {
    a.back()*=c; return a; }

template<class A, class B> Graded<A> operator+(const Graded<A>& a, const B& c) { return add(a,c); }
template<class A, class B> Graded<A> operator*(const Graded<A>& a, const B& c) { return mul(a,c); }

template<class A> Graded<A> operator+(const GenericNumericType<A>& c, const Graded<A>& a) { return add(c,a); }
template<class A> Graded<A> operator-(const GenericNumericType<A>& c, const Graded<A>& a) { return sub(c,a); }
template<class A> Graded<A> operator*(const GenericNumericType<A>& c, const Graded<A>& a) { return mul(c,a); }
template<class A> Graded<A> operator/(const GenericNumericType<A>& c, const Graded<A>& a) { return div(c,a); }

template<class A> inline ClosureExpression<Pos,Graded<A>> operator+(const Graded<A>& a) { return pos(a); }
template<class A> inline ClosureExpression<Neg,Graded<A>> operator-(const Graded<A>& a) { return neg(a); }
template<class A> inline ClosureExpression<Add,Graded<A>,Graded<A>> operator+(const Graded<A>& a1, const Graded<A>& a2) { return add(a1,a2); }
template<class A> inline ClosureExpression<Sub,Graded<A>,Graded<A>> operator-(const Graded<A>& a1, const Graded<A>& a2) { return sub(a1,a2); }
template<class A> inline ClosureExpression<Mul,Graded<A>,Graded<A>> operator*(const Graded<A>& a1, const Graded<A>& a2) { return mul(a1,a2); }
template<class A> inline ClosureExpression<Div,Graded<A>,Graded<A>> operator/(const Graded<A>& a1, const Graded<A>& a2) { return div(a1,a2); }

template<class A, class B> Graded<A> add(const Graded<A>& a, const B& c) {
    Graded<A> r(a); r[0]+=c; return r;  }
template<class A, class B> Graded<A> mul(const Graded<A>& a, const B& c) {
    Graded<A> r(a); r*=c; return r;  }


template<class A> Graded<A> add(const GenericNumericType<A>& c, const Graded<A>& a) {
    Graded<A> r(a); r[0]+=c; return r;  }
template<class A> Graded<A> sub(const GenericNumericType<A>& c, const Graded<A>& a) {
    ARIADNE_NOT_IMPLEMENTED; }
template<class A> Graded<A> mul(const GenericNumericType<A>& c, const Graded<A>& a) {
    Graded<A> r(a); r[0]*=c; return r;  }
template<class A> Graded<A> div(const GenericNumericType<A>& c, const Graded<A>& a) {
    ARIADNE_NOT_IMPLEMENTED; }

template<class A> ClosureExpression<Add,Graded<A>,Graded<A>> add(const Graded<A>& a1, const Graded<A>& a2) {
    return make_expression(Add(),a1,a2); }
template<class A> ClosureExpression<Sub,Graded<A>,Graded<A>> sub(const Graded<A>& a1, const Graded<A>& a2) {
    return make_expression(Sub(),a1,a2); }
template<class A> ClosureExpression<Mul,Graded<A>,Graded<A>> mul(const Graded<A>& a1, const Graded<A>& a2) {
    return make_expression(Mul(),a1,a2); }
template<class A> ClosureExpression<Div,Graded<A>,Graded<A>> div(const Graded<A>& a1, const Graded<A>& a2) {
    return make_expression(Div(),a1,a2); }
template<class A> ClosureExpression<Nul,Graded<A>> nul(const Graded<A>& a) {
    return make_expression(Nul(),a); }
template<class A> ClosureExpression<Pos,Graded<A>> pos(const Graded<A>& a) {
    return make_expression(Pos(),a); }
template<class A> ClosureExpression<Neg,Graded<A>> neg(const Graded<A>& a) {
    return make_expression(Neg(),a); }
template<class A> ClosureExpression<Hlf,Graded<A>> hlf(const Graded<A>& a) {
    return make_expression(Hlf(),a); }
template<class A> ClosureExpression<Sqr,Graded<A>> sqr(const Graded<A>& a) {
    return make_expression(Sqr(),a); }
template<class A> ClosureExpression<Pow,Graded<A>,Int> pow(const Graded<A>& a, const Int& n) {
    return make_expression(Pow(),a,n); }
template<class A> ClosureExpression<Sqrt,Graded<A>> sqrt(const Graded<A>& a) {
    return make_expression(Sqrt(),a); }
template<class A> ClosureExpression<Exp,Graded<A>> exp(const Graded<A>& a) {
    return make_expression(Exp(),a); }
template<class A> ClosureExpression<Log,Graded<A>> log(const Graded<A>& a) {
    return make_expression(Log(),a); }
template<class A> ClosureExpression<Rec,Graded<A>> rec(const Graded<A>& a) {
    return make_expression(Rec(),a); }
template<class A> ClosureExpression<Sin,Graded<A>> sin(const Graded<A>& a) {
    return make_expression(Sin(),a); }
template<class A> ClosureExpression<Cos,Graded<A>> cos(const Graded<A>& a) {
    return make_expression(Cos(),a); }
template<class A> ClosureExpression<Tan,Graded<A>> tan(const Graded<A>& a) {
    return make_expression(Tan(),a); }
template<class A> ClosureExpression<Asin,Graded<A>> asin(const Graded<A>& a) {
    return make_expression(Asin(),a); }
template<class A> ClosureExpression<Acos,Graded<A>> acos(const Graded<A>& a) {
    return make_expression(Acos(),a); }
template<class A> ClosureExpression<Atan,Graded<A>> atan(const Graded<A>& a) {
    return make_expression(Atan(),a); }

template<class A> Void nul(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.degree()+1u == a.degree());
    r.append(nul(a.back()));
}

template<class A> Void pos(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.degree()+1u == a.degree());
    r.append(pos(a.back()));
}

template<class A> Void neg(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    r.append(-a.back());
}

template<class A> Void hlf(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.degree()+1u == a.degree());
    r.append(hlf(a.back()));
}

template<class A> Void add(Graded<A>& r, const Graded<A>& a1, const Graded<A>& a2) {
    ARIADNE_ASSERT(r.size()+1u == a1.size());
    ARIADNE_ASSERT(r.size()+1u == a2.size());
    r.append(a1.back()+a2.back());
}

template<class A> Void sub(Graded<A>& r, const Graded<A>& a1, const Graded<A>& a2) {
    ARIADNE_ASSERT(r.size()+1u == a1.size());
    ARIADNE_ASSERT(r.size()+1u == a2.size());
    r.append(a1.back()-a2.back());
}

template<class A> Void mul(Graded<A>& r, const Graded<A>& a1, const Graded<A>& a2) {
    ARIADNE_ASSERT_MSG(r.size()+1u == a1.size(),"r="<<r<<", a1="<<a1<<", a2="<<a2);
    ARIADNE_ASSERT_MSG(r.size()+1u == a2.size(),"r="<<r<<", a1="<<a1<<", a2="<<a2);
    ARIADNE_ASSERT(compatible(a1[0],a2[0]));
    r.append(create(a1[0]));
    DegreeType d = r.degree();
    for(DegreeType i=0; i<=d; ++i) {
        r[d] += a1[i]*a2[cast_sign<DegreeType>(d-i)];
    }
}

template<class A> Void div(Graded<A>& r, const Graded<A>& a1, const Graded<A>& a2) {
    ARIADNE_ASSERT(r.size()+1u == a1.size());
    ARIADNE_ASSERT(r.size()+1u == a2.size());
    ARIADNE_ASSERT(compatible(a1[0],a2[0]));
    r.append(create(a1[0]));
    DegreeType d = r.degree();
    r[d]+=a1[d];
    for(DegreeType i=0; i!=d; ++i) {
        r[d] -= a2[cast_sign<DegreeType>(d-i)]*r[i];
    }
    r[d]=r[d]/a2[0];
}

template<class A> Void sqr(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    r.append(create(a[0]));
    DegreeType d = r.degree();
    for(DegreeType i=0; i<=d; ++i) {
        r[d] += a[i]*a[cast_sign<DegreeType>(d-i)];
    }
}

template<class A> Void rec(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    r.append(create(a[0]));
    DegreeType d = r.degree();
    if(d==0) { r[d]=rec(a[0]); return; }
    for(DegreeType i=0; i!=d; ++i) {
        r[d] -= a[cast_sign<DegreeType>(d-i)]*r[i];
    }
    r[d]=r[d]*r[0];
}

// Use formula d*a[0]*r[d] = sum_{i=0}^{d-1} ((d-i)*n-i)*a[d-i]*r[i] for d>0
template<class A> Void pow(Graded<A>& r, const Graded<A>& a, Int n) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    r.append(create(a[0]));
    DegreeType d = r.degree();
    if(d==0u) { r[d]=pow(a[0],n); return; }
    for(DegreeType i=0; i<d; ++i) {
        r[d] += ((d-i)*n-i)*a[cast_sign<DegreeType>(d-i)]*r[i];
    }
    r[d]=r[d]/d/a[0];
}

template<class A> Void sqrt(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    r.append(create(a[0]));
    DegreeType d = r.degree();
    if(d==0) { r[d]=sqrt(a[0]); return; }
    r[d]=a[d];
    for(DegreeType i=1; i!=d; ++i) {
        r[d] -= r[cast_sign<DegreeType>(d-i)]*r[i];
    }
    r[d]=r[d]/(2*r[0]);
}

template<class A> Void exp(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    r.append(create(a[0]));
    DegreeType d = r.degree();
    if(d==0) { r[d]+=exp(a[0]); return; }
    for(DegreeType i=0; i!=d; ++i) {
        r[d] += (d-i)*a[cast_sign<DegreeType>(d-i)]*r[i];
    }
    r[d]/=d;
}

template<class A> Void log(Graded<A>& r, const Graded<A>& a) {
    // y=log x; r=1/x; s=r^2;
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    r.append(create(a[0]));
    DegreeType d = r.degree();
    if(d==0) { r[d]=log(a[0]); return; }
    if(d==1) { r[d]=a[d]/a[0];  return; }
    r[d]+=d*a[d];
    for(DegreeType i=1; i!=d; ++i) {
        r[d] -= a[cast_sign<DegreeType>(d-i)]*r[i]*i;
    }
    r[d]=r[d]/a[0];
    r[d]/=d;
}

/*
template<class A> Void sin(Graded<A>& r, const Graded<A>& a) {
    // Let f[0](t)=f(t), s[0](t)=sin(f(t)), c[0](t)=cos(f(t))
    // f[n+1](t)=df[n](t)/dt, s[n+1]=ds[n](t)/dt, c[n+1]=dc[n](t)/dt
    // Then s[1]=c[0]*f[1]; c[1]=-s[0]*f[1]
    const Graded<A>& f=a;
    Graded<A>& s=r;
    A z=create(f[0])*0;
    s.append(z);
    DegreeType d = s.degree();
    if(d==0) { s[d]=sin(f[0]); return; }
    if(d==1) { s[d]=cos(f[0])*f[1]; return; }
    Graded<A> c(cos(f[0]));
    c.append(-s[0]*f[1]);
    for(DegreeType i=2; i!=d; ++i) {
        c.append(z);
        for(DegreeType j=0; j!=i; ++j) {
            c.back() -= (i-j)*f[i-j]*s[j];
        }
        c.back()/=i;
    }
    for(DegreeType i=0; i!=d; ++i) {
        s[d] += (d-i)*f[cast_sign<DegreeType>(d-i)]*c[i];
    }
    s[d]/=d;
}
*/

template<class A> Void sincos(Graded<A>& s, Graded<A>& c, const Graded<A>& a) {
    // Let f[0](t)=f(t), s[0](t)=sin(f(t)), c[0](t)=cos(f(t))
    // f[n+1](t)=df[n](t)/dt, s[n+1]=ds[n](t)/dt, c[n+1]=dc[n](t)/dt
    // Then s[n]=+Sum_{m=1}^{n} (m*f[m]*c[n-m])/n
    // Then c[n]=-Sum_{m=1}^{n} (m*f[m]*s[n-m])/n
    DegreeType d = a.degree();
    A z=create(a[0]);
    ARIADNE_ASSERT(s.size()==0 && c.size()==0);
    for(DegreeType i=0; i<=d; ++i) { s.append(z); c.append(z); }
    s[0]=sin(a[0]);
    c[0]=cos(a[0]);
    for(DegreeType i=1; i<=d; ++i) {
        for(DegreeType j=1; j<=i; ++j) {
            s[i] += j*a[j]*c[cast_sign<DegreeType>(i-j)];
            c[i] += j*a[j]*s[cast_sign<DegreeType>(i-j)];
        }
        s[i]/=(+i);
        c[i]/=(-i);
    }
}

template<class A> Void sin(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    Graded<A> s(a.characteristics());
    Graded<A> c(a.characteristics());
    sincos(s,c,a);
    r=s;
}

template<class A> Void cos(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT(r.size()+1u <= a.size());
    Graded<A> s(a.characteristics());
    Graded<A> c(a.characteristics());
    sincos(s,c,a);
    r=c;
}

template<class A> Void tan(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_NOT_IMPLEMENTED;
}

template<class A> Void asin(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_NOT_IMPLEMENTED;
}

template<class A> Void acos(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_NOT_IMPLEMENTED;
}

template<class A> Void atan(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_NOT_IMPLEMENTED;
}



template<class A> Graded<A> max(const GenericNumericType<A>& c, const Graded<A>& a) {
    ARIADNE_THROW(std::runtime_error, "max(GenericNumericType<A>,Graded<A>)","Cannot apply non-analytic operation to Graded.");
}
template<class A> Graded<A> min(const GenericNumericType<A>& c, const Graded<A>& a) {
    ARIADNE_THROW(std::runtime_error, "min(GenericNumericType<A>,Graded<A>)","Cannot apply non-analytic operation to Graded.");
}

template<class A> Graded<A> max(Graded<A> const& ga1, Graded<A> const& ga2) {
    ARIADNE_THROW(std::runtime_error, "max(Graded<A>,Graded<A>)","Cannot apply non-analytic operation to Graded.")
}
template<class A> Graded<A> min(Graded<A> const& ga1, Graded<A> const& ga2) {
    ARIADNE_THROW(std::runtime_error, "min(Graded<A>,Graded<A>)","Cannot apply non-analytic operation to Graded.")
}
template<class A> Graded<A> abs(Graded<A> const& ga) {
    ARIADNE_THROW(std::runtime_error, "abs(Graded<A>)","Cannot apply non-analytic operation to Graded.")
}


template<class A> template<class Op> Void Graded<A>::operator=(const ClosureExpression<Op,Graded<A>,Graded<A>>& expr) {
    compute(*this,Op(),expr.arg1,expr.arg2);
}

template<class A> template<class Op, class N> Void Graded<A>::operator=(const ClosureExpression<Op,Graded<A>,N>& expr) {
    compute(*this,Op(),expr.arg1,expr.arg2);
}

template<class A> template<class Op> Void Graded<A>::operator=(const ClosureExpression<Op,Graded<A>>& expr) {
    compute(*this,Op(),expr.arg);
}

template<class A> Void Graded<A>::operator=(const ClosureExpression<AntiDiff,Graded<A>>& expr) {
    antidifferentiate(*this,expr.arg);
}

template<class A> Void antidifferentiate(Graded<A>& r, const Graded<A>& a) {
    ARIADNE_ASSERT_MSG(r.size() == a.size(),"r="<<r<<", a="<<a);
    r.append(a.back()/(r.degree()+1u));
}

template<class A> ClosureExpression<AntiDiff,Graded<A>> antidifferential(const Graded<A>& a) {
    return ClosureExpression<AntiDiff,Graded<A>>(a);
}


template<class X> Differential<X> make_differential_variable(SizeType as, DegreeType deg, X val, SizeType ind) {
    return Differential<X>::variable(as,deg,val,ind); }
template<class X> Graded<X> make_graded_constant(const X& val, DegreeType deg) {
    Graded<X> res(val); X z=nul(val); while(res.degree()<deg) { res.append(z); } return res; }
template<class X> Graded<X> make_graded_variable(const X& val, DegreeType deg) {
    ARIADNE_PRECONDITION(deg>=1u); Graded<X> res(val); X z=nul(val); res.append(z+1); while(res.degree()<deg) { res.apend(z); } return res; }

template<class X> Graded<X> make_graded(const X& val) {
    return Graded<X>(val); }
template<class X> Graded<X> create_graded(const X&) {
    return Graded<X>(); }


template<class X, class A> Void compute_procedure(const Vector<Procedure<X>>& p, Vector<Graded<A>>& r, List<Graded<A>>& t, const Vector<Graded<A>>& a) {
    execute(t,p,a);
    r=Vector<Graded<A>>(p._results.size(), [&t,&p](SizeType i){return t[p._results[i]];},a.element_characteristics());
}

} // namespace Ariadne

#endif /* ARIADNE_GRADED_HPP */
