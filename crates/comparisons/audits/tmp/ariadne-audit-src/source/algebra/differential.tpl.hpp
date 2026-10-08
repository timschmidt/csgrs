/***************************************************************************
 *            differential.tpl.hpp
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

#include <map>

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "numeric/float.decl.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/series.hpp"
#include "algebra/expansion.hpp"
#include "algebra/expansion.inl.hpp"

namespace Ariadne {

template<class X> class Vector;
template<class X> class Matrix;
template<class X> class Series;

template<class I, class X> class Expansion;
template<class X> class Differential;


inline DegreeType min(DegreeType d1, DegreeType d2) { return std::min(d1,d2); }
inline DegreeType max(DegreeType d1, DegreeType d2) { return std::max(d1,d2); }

//template<class X> Differential<X>::Differential() : _expansion(0), _degree(0) { }

template<class X> Differential<X>::Differential(SizeType as, DegreeType deg, X const& z) : _expansion(as,get_characteristics(z)), _degree(deg) { }

template<class X> Differential<X>::Differential(const Map<MultiIndex,X>& map, DegreeType deg)
    : _expansion((assert(!map.empty()),map.begin()->first.size()),get_characteristics(map.begin()->second))
{
    ARIADNE_NOT_IMPLEMENTED;
}

template<class X>
Differential<X>::Differential(SizeType as, DegreeType deg,
                              InitializerList< Pair<InitializerList<DegreeType>,X> > lst)
    : _expansion(Expansion<MultiIndex,X>(lst)), _degree(deg)
{
    this->cleanup();
}


template<class X> Differential<X>::Differential(const Expansion<MultiIndex,X>& e, DegreeType deg)
    : _expansion(e.argument_size(),e.coefficient_characteristics()),_degree(deg)
{
    for(typename Expansion<MultiIndex,X>::ConstIterator iter=e.begin(); iter!=e.end(); ++iter) {
        if(iter->index().degree()<=deg) { this->_expansion.append(iter->index(),iter->coefficient()); }
    }
    this->cleanup();
}


template<class X> Differential<X>& Differential<X>::operator=(const X& c) {
    this->_expansion.clear();
    this->_expansion.append(MultiIndex(this->argument_size()),c);
    return *this;
}


template<class X> Differential<X> Differential<X>::constant(SizeType as, DegreeType deg, const X& c) {
    Differential<X> r(as,deg,nul(c));
    r._expansion.append(MultiIndex(as),c);
    return r;
}

template<class X> Differential<X> Differential<X>::variable(SizeType as, DegreeType deg, const X& v, SizeType j) {
    Differential<X> r(as,deg,nul(v));
    MultiIndex a(as);
    r._expansion.append(a,v);
    a[j]=1;
    X one(v); one=1;
    r._expansion.append(a,one);
    return r;
}

template<class X> Differential<X> Differential<X>::affine(SizeType as, DegreeType deg, const X& x, const Covector<X>& g) {
    ARIADNE_ASSERT_MSG(as==g.size(), "g.size()="<<g.size()<<" must equal as="<<as);
    Differential<X> r(as,deg,nul(x));
    MultiIndex a(as);
    r._expansion.append(a,x);
    for (SizeType j=0; j!=as; ++j) {
        a[j]=1;
        r._expansion.append(a,g[j]);
        a[j]=0;
    }
    return r;
}

template<class X> Differential<X> Differential<X>::affine(DegreeType deg, const X& x, const Covector<X>& g) {
    return affine(g.size(),deg,x,g);
}


template<class X> Vector<Differential<X>> Differential<X>::constants(SizeType rs, SizeType as, DegreeType deg, const Vector<X>& c) {
    ARIADNE_ASSERT(c.size()==rs);
    return constants(as,deg,c);
}

template<class X> Vector<Differential<X>> Differential<X>::variables(SizeType rs, SizeType as, DegreeType deg, const Vector<X>& x) {
    ARIADNE_ASSERT(x.size()==rs);  ARIADNE_ASSERT(as==x.size());
    return variables(deg,x);
}

template<class X> Vector<Differential<X>> Differential<X>::constants(SizeType as, DegreeType deg, const Vector<X>& c) {
    Vector<Differential<X>> r(c.size(),Differential(as,deg,c.zero_element()));
    for(SizeType i=0; i!=c.size(); ++i) { r[i]=c[i]; }
    return r;
}

template<class X> Vector<Differential<X>> Differential<X>::variables(DegreeType deg, const Vector<X>& v) {
    X zero=v.zero_element();
    Vector<Differential<X>> r(v.size(),Differential<X>(v.size(),deg,zero));
    X& one=zero; one=1;
    MultiIndex a(v.size());
    for(SizeType i=0; i!=v.size(); ++i) {
        r[i]._expansion.append(a,v[i]);
        a[i]=1; r[i]._expansion.append(a,one); a[i]=0;
    }
    return r;
}

template<class X> Vector<Differential<X>> Differential<X>::affine(DegreeType deg, const Vector<X>& v, const Matrix<X>& G) {
    X zero=v.zero_element();
    Vector<Differential<X>> r(v.size(),Differential<X>(v.size(),deg,zero));
    MultiIndex a(r.argument_size());
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]._expansion.append(a,v[i]);
        for(SizeType j=0; j!=r.argument_size(); ++j) {
            const X& gij=G[i][j];
            if(decide(gij!=0)) { a[i]=1; r[i]._expansion.append(a,gij); a[i]=0; }
        }
    }
    return r;
}


template<class X> Differential<X> Differential<X>::identity(DegreeType deg, const X& x) {
    return variable(1u,deg,x,0u);
}
template<class X> Vector<Differential<X>> Differential<X>::identity(DegreeType deg, const Vector<X>& x) {
    return variables(deg,x);
}


template<class X> EqualityType<X> Differential<X>::operator==(const Differential<X>& other) const {
    Differential<X> const& self=*this;
    if(self.argument_size()!=other.argument_size()) { return false; }
    EqualityType<X> result=true;
    IndexComparisonType less;
    ConstIterator self_iter=self.begin(); ConstIterator other_iter=other.begin();
    while(self_iter!=self.end() && other_iter!=other.end()) {
        if(self_iter->index()==other_iter->index()) {
            result = result && (self_iter->coefficient()==other_iter->coefficient());
            ++self_iter; ++other_iter;
        } else if (less(self_iter->index(), other_iter->index())) {
            result = result && static_cast<EqualityType<X>>(self_iter->coefficient()==0);
            ++self_iter;
        } else { // (self_iter->index() > other_iter->index())
            result = result && static_cast<EqualityType<X>>(other_iter->coefficient()==0);
            ++other_iter;
        }
    }
    while(self_iter!=self.end()) {
        result = result && static_cast<EqualityType<X>>(self_iter->coefficient()==0);
        ++self_iter;
    }
    while(other_iter!=other.end()) {
        result = result && static_cast<EqualityType<X>>(other_iter->coefficient()==0);
        ++other_iter;
    }
    return result;
}

template<class X> InequalityType<X> Differential<X>::operator!=(const Differential<X>& other) const {
    return !(*this==other);
}


template<class X> SizeType Differential<X>::argument_size() const {
    return this->_expansion.argument_size();
}

template<class X> DegreeType Differential<X>::degree() const {
    return this->_degree;
}

template<class X> Ariadne::CharacteristicsType<X> Differential<X>::coefficient_characteristics() const {
    return this->_expansion.coefficient_characteristics();
}

template<class X> X Differential<X>::zero_coefficient() const {
    return this->_expansion.zero_coefficient();
}

template<class X> const Expansion<MultiIndex,X>& Differential<X>::expansion() const {
    return this->_expansion;
}

template<class X> Expansion<MultiIndex,X>& Differential<X>::expansion() {
    return this->_expansion;
}

template<class X> const X& Differential<X>::value() const {
    return this->operator[](MultiIndex(this->argument_size()));
}

template<class X> const X& Differential<X>::gradient(SizeType j) const {
    return this->operator[](MultiIndex::unit(this->argument_size(),j));
}

template<class X> Covector<X> Differential<X>::gradient() const {
    Covector<X> g(this->argument_size(),this->zero_coefficient());
    for(SizeType j=0; j!=g.size(); ++j) { g[j]=this->gradient(j); }
    return g;
}

template<class X> Matrix<X> Differential<X>::half_hessian() const {
    ARIADNE_PRECONDITION(this->degree()>=2);
    Matrix<X> H(this->argument_size(),this->argument_size(),nul(this->value()));
    ConstIterator iter=this->begin();
    while(iter!=this->end() && iter->index().degree()<=1) { ++iter; }
    SizeType i=0; SizeType j=1;
    while(iter!=this->end() && iter->index().degree()<=2) {
        UniformConstReference<MultiIndex> a=iter->index();
        UniformConstReference<X> c=iter->coefficient();
        while(a[i]==0) { ++i; j=i+1u; }
        if(a[i]==2) { H[i][i]=c; }
        else { while(a[j]==0) { ++j; } H[i][j]=hlf(c); H[j][i]=hlf(c); }
        ++iter;
    }
    return H;
}

template<class X> Matrix<X> Differential<X>::hessian() const {
    return this->half_hessian()*2;
}


template<class X> X& Differential<X>::operator[](const SizeType& j) {
    return this->operator[](MultiIndex::unit(this->argument_size(),j));
}

template<class X> const X& Differential<X>::operator[](const SizeType& j) const {
    return this->operator[](MultiIndex::unit(this->argument_size(),j));
}

template<class X> X& Differential<X>::operator[](const MultiIndex& a) {
    ARIADNE_ASSERT_MSG(a.number_of_variables()==this->argument_size()," d="<<*this<<", a="<<a);
    return this->_expansion.at(a);
}

template<class X> const X& Differential<X>::operator[](const MultiIndex& a) const {
    ARIADNE_ASSERT_MSG(a.number_of_variables()==this->argument_size()," d="<<*this<<", a="<<a);
    ConstIterator iter=this->_expansion.find(a);
    if(iter==this->_expansion.end()) { return this->_expansion.zero_coefficient(); }
    else { return iter->coefficient(); }
}


template<class X> Void Differential<X>::set_degree(DegreeType deg) {
    if(deg<this->_degree) {
        for(auto iter = this->_expansion.begin(); iter!=this->_expansion.end(); ++iter) {
            if (iter->index().degree() > deg) {
                this->_expansion.resize(static_cast<SizeType>(iter-this->_expansion.begin()));
                break;
            }
        }
    }
    this->_degree = deg;
}

template<class X> Void Differential<X>::set_value(const X& c) {
    this->operator[](MultiIndex(this->argument_size()))=c;
}

template<class X> Void Differential<X>::set_gradient(SizeType j, const X& d) {
    this->operator[](MultiIndex::unit(this->argument_size(),j))=d;
}


template<class X> typename Differential<X>::Iterator Differential<X>::begin() {
    return this->_expansion.begin();
}

template<class X> typename Differential<X>::Iterator Differential<X>::end() {
    return this->_expansion.end();
}

template<class X> typename Differential<X>::ConstIterator Differential<X>::begin() const {
    return this->_expansion.begin();
}

template<class X> typename Differential<X>::ConstIterator Differential<X>::end() const {
    return this->_expansion.end();
}


template<class X> Differential<X> Differential<X>::create() const {
    return Differential<X>(this->argument_size(),this->degree(),this->zero_coefficient());
}

template<class X> Differential<X> Differential<X>::create_zero() const {
    return Differential<X>(this->argument_size(),this->degree(),this->zero_coefficient());
}

template<class X> Differential<X> Differential<X>::create_constant(NumericType const& c) const {
    return Differential<X>::constant(this->argument_size(),this->degree(),c);
}

template<class X> Differential<X> Differential<X>::create_constant(Int c) const {
    X xc=this->value(); xc=c; return Differential<X>::constant(this->argument_size(),this->degree(),xc);
}


template<class X> Void Differential<X>::clear() {
    this->_expansion.clear();
}

template<class X> Void Differential<X>::cleanup() {
    this->_expansion.sort();
    this->_expansion.combine_terms();
    this->_expansion.remove_zeros();
}

template<class X> Void Differential<X>::check() const {
    for(auto iter=this->begin(); iter!=this->end(); ++iter) {
        ARIADNE_ASSERT_MSG(iter->index().degree()<=this->degree(), *this);
        auto next = iter; ++next;
        ARIADNE_ASSERT_MSG(graded_less(iter->index(),next->index()),"ErrorTag in ordering Differential "<<this->expansion());
    }
}


template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Pos op, Differential<X> x)
{
    for(auto iter=x.begin(); iter!=x.end(); ++iter) {
        UniformReference<X> xa=iter->coefficient();
        xa=+xa;
    }
    return x;
}

template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Neg op, Differential<X> x)
{
    for(auto iter=x.begin(); iter!=x.end(); ++iter) {
        UniformReference<X> xa=iter->coefficient();
        xa=-xa;
    }
    return x;
}

template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Add op, Differential<X> x, const X& c)
{
    MultiIndex a(x.argument_size());
    if(x.expansion().empty()) {
        x.expansion().append(a,c);
    } else if(x.begin()->index()!=a) {
        x.expansion().prepend(a,c);
    } else {
        x.begin()->coefficient()+=c;
    }
    return x;
}

template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Mul op, Differential<X> x, const X& c)
{
    if(decide(c==0)) {
        x.clear();
    } else {
        for(auto iter=x.begin(); iter!=x.end(); ++iter) {
            iter->coefficient()*=c;
        }
    }
    return x;
}



template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Add op, const Differential<X>& x, const Differential<X>& y)
{
    ARIADNE_ASSERT_MSG(x.argument_size()==y.argument_size(),"x="<<x<<" y="<<y);
    Differential<X> r(x.argument_size(),min(x.degree(),y.degree()),add(x.zero_coefficient(),y.zero_coefficient()));
    typename Differential<X>::ConstIterator xiter=x.begin();
    typename Differential<X>::ConstIterator yiter=y.begin();
    // No need to check if maximum degree has been reached below,
    // since if one Iterator is above the maximum degree, the other is at the end.
    while(xiter!=x.end() && yiter!=y.end()) {
        if(xiter->index()==yiter->index()) {
            r.expansion().append(xiter->index(),xiter->coefficient()+yiter->coefficient());
            ++xiter; ++yiter;
        } else if(graded_less(xiter->index(),yiter->index())) {
            r.expansion().append(xiter->index(),xiter->coefficient());
            ++xiter;
        } else {
            r.expansion().append(yiter->index(),yiter->coefficient());
            ++yiter;
        }
    }
    while(xiter!=x.end() && xiter->index().degree()<=r.degree()) {
        r.expansion().append(xiter->index(),xiter->coefficient());
        ++xiter;
    }
    while(yiter!=y.end() && yiter->index().degree()<=r.degree()) {
        r.expansion().append(yiter->index(),yiter->coefficient());
        ++yiter;
    }
    //std::cerr<<"x="<<x<<" y="<<y<<" x+y="<<r<<"\n";
    return r;
}

template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Sub op, const Differential<X>& x, const Differential<X>& y)
{
    ARIADNE_ASSERT_MSG(x.argument_size()==y.argument_size(),"x="<<x<<" y="<<y);
    Differential<X> r(x.argument_size(),min(x.degree(),y.degree()),sub(x.zero_coefficient(),y.zero_coefficient()));
    typename Differential<X>::ConstIterator xiter=x.begin();
    typename Differential<X>::ConstIterator yiter=y.begin();
    // No need to check if maximum degree has been reached below,
    // since if one Iterator is above the maximum degree, the other is at the end.
    while(xiter!=x.end() && yiter!=y.end()) {
        if(xiter->index()==yiter->index()) {
            r.expansion().append(xiter->index(),xiter->coefficient()-yiter->coefficient());
            ++xiter; ++yiter;
        } else if(graded_less(xiter->index(),yiter->index())) {
            r.expansion().append(xiter->index(),xiter->coefficient());
            ++xiter;
        } else {
            r.expansion().append(yiter->index(),-yiter->coefficient());
            ++yiter;
        }
    }
    while(xiter!=x.end() && xiter->index().degree()<=r.degree()) {
        r.expansion().append(xiter->index(),xiter->coefficient());
        ++xiter;
    }
    while(yiter!=y.end() && yiter->index().degree()<=r.degree()) {
        r.expansion().append(yiter->index(),-yiter->coefficient());
        ++yiter;
    }
    //std::cerr<<"x="<<x<<" y="<<y<<" x-y="<<r<<"\n";
    return r;
}

template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Mul op, const Differential<X>& x, const Differential<X>& y)
{
    typedef typename Differential<X>::ConstIterator ConstIterator;
    ARIADNE_ASSERT_MSG(x.argument_size()==y.argument_size(),"x="<<x<<" y="<<y);
    Differential<X> r(x.argument_size(),min(x.degree(),y.degree()),mul(x.zero_coefficient(),y.zero_coefficient()));
    MultiIndex a(x.argument_size());
    X c(x.zero_coefficient()*y.zero_coefficient());
    for(ConstIterator xiter=x.expansion().begin(); xiter!=x.expansion().end(); ++xiter) {
        if(xiter->index().degree()>r.degree()) { break; }
        for(ConstIterator yiter=y.expansion().begin(); yiter!=y.expansion().end(); ++yiter) {
            if(xiter->index().degree()+yiter->index().degree()>r.degree()) { break; }
            a=xiter->index()+yiter->index();
            c=xiter->coefficient()*yiter->coefficient();
            r.expansion().append(a,c);
        }
    }
    r.cleanup();
    //std::cerr<<"x="<<x<<" y="<<y<<" x*y="<<r<<"\n";
    return r;
}

template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Div op, const Differential<X>& x, const Differential<X>& y)
{
    return x * rec(y);
}

template<class X>
Differential<X> AlgebraOperations<Differential<X>>::apply(Pow op, const Differential<X>& x, Int n)
{
    return generic_pow(x,n);
}


template<class X> Differential<X> AlgebraOperations<Differential<X>>::apply(Min op, const Differential<X>& x1, const Differential<X>& x2) {
    // FIXME: Maybe need different code for validated and approximate paradigms
    ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2);
    if(decide(x1.value()==x2.value())) {
        ARIADNE_THROW(std::runtime_error,"min(Differential<X> x1, Differential<X> x2)","x1[0]==x2[0]");
    }
    return decide(x1.value()<x2.value()) ? x1 : x2;
}


template<class X> Differential<X> AlgebraOperations<Differential<X>>::apply(Max op, const Differential<X>& x1,const Differential<X>& x2) {
    ARIADNE_ASSERT_MSG(x1.argument_size()==x2.argument_size(),"x1="<<x1<<" x2="<<x2);
    if(decide(x1.value()==x2.value())) {
        ARIADNE_THROW(std::runtime_error,"max(Differential<X> x1, Differential<X> x2)","x1[0]==x2[0]");
    }
    return decide(x1.value()>x2.value()) ? x1 : x2;
}

template<class X> Differential<X> AlgebraOperations<Differential<X>>::apply(Abs op, const Differential<X>& x) {
    // FIXME: Maybe need different code for validated and approximate paradigms
    if(decide(x.value()==0)) {
        ARIADNE_THROW(std::runtime_error,"abs(Differential<X> x)","x[0]==0");
    }
    return decide(x.value()>0) ? pos(x) : neg(x);
}




template<class X> Differential<X> _evaluate(const Differential<X>& x, const Vector<Differential<X>>& a)
{
    ARIADNE_ASSERT_MSG(x.argument_size()==a.size(), "x="<<x<<" a="<<a);
    DegreeType d=x.degree();
    SizeType ms=a.size();
    ARIADNE_ASSERT(d>=1);

    Differential<X> zero = a.zero_element();
    X x_one=zero.zero_coefficient(); x_one=1;
    Differential<X> one=zero.create_constant(x_one);

    // Use inefficient brute-force approach with lots of storage...
    Array< Array< Differential<X> > > val(ms, Array< Differential<X> >(d+1u,zero));
    for(SizeType j=0; j!=ms; ++j) {
        val[j][0]=one;
        val[j][1]=a[j];
        for(SizeType k=2; k<=d; ++k) {
            val[j][k]=val[j][k-1u]*a[j];
        }
    }

    Differential<X> r(zero);
    for(auto iter=x.begin(); iter!=x.end(); ++iter)
    {
        UniformConstReference<MultiIndex> j=iter->index();
        UniformConstReference<X> c=iter->coefficient();
        Differential<X> t=one;
        for(SizeType k=0; k!=ms; ++k) {
            t=t*val[k][j[k]];
        }
        t*=c;
        r+=t;
        //std::cerr<<" j="<<j<<" c="<<c<<" r="<<r<<std::endl;
    }
    return r;
}


template<class X>
Differential<X> Differential<X>::_compose(const UnivariateDifferential<X>& x, const Differential<X>& y)
{
    SizeType as=y.argument_size();
    DegreeType d=min(x.degree(),y.degree());

    Differential<X> w=y;
    if(w.begin()->index().degree()==0) { w.begin()->coefficient()=0; }
    Differential<X> r(as,d,y.zero_coefficient());
    r[MultiIndex(as)]=x[d];
    for(SizeType n=1; n<=d; ++n) {
        r=r*w;
        r+=x[d-n];
    }
    return r;
}

template<class X>
Differential<X> Differential<X>::_compose(const Differential<X>& x, const Vector<Differential<X>>& y)
{
    Vector<Differential<X>>& ync=const_cast< Vector<Differential<X>>&>(y);
    X zero(y.zero_element().zero_coefficient());
    Vector<X> yv(y.size(),zero);
    DegreeType deg=min(x.degree(),y.degree());
    for(SizeType i=0; i!=ync.result_size(); ++i) { yv[i]=ync[i].value(); ync[i].set_value(zero); }
    Differential<X> r=_evaluate(x,ync);
    r.set_degree(deg);
    for(SizeType i=0; i!=ync.result_size(); ++i) { ync[i].set_value(yv[i]); }
    return r;
}


template<class X>
Vector<Differential<X>> Differential<X>::_compose(const Vector<Differential<X>>& x, const Vector<Differential<X>>& y)
{
    //std::cerr<<"compose(DV x, DV y)\n x="<<x<<"\n y="<<y<<std::endl;
    X zero(y.zero_element().zero_coefficient());
    Vector<Differential<X>>& ync=const_cast< Vector<Differential<X>>&>(y);
    Vector<X> yv(y.size(),zero);
    DegreeType deg=min(x.degree(),y.degree());
    for(SizeType i=0; i!=ync.result_size(); ++i) { yv[i]=ync[i].value(); ync[i].set_value(zero); }
    Vector<Differential<X>> r(x.size(),y.argument_size(),deg,y.zero_coefficient());
    for(SizeType i=0; i!=x.result_size(); ++i) { r[i]=_evaluate(x[i],y); r[i].set_degree(deg); }
    for(SizeType i=0; i!=ync.result_size(); ++i) { ync[i].set_value(yv[i]); }
    return r;
}


template<class X>
Differential<X> Differential<X>::_derivative(const Differential<X>& x, SizeType i)
{
    if(x.degree()==0) { return Differential<X>(x.argument_size(),0u,x.zero_coefficient()); }
    Differential<X> r(x.argument_size(), x.degree()-1u,x.zero_coefficient());
    MultiIndex a(x.argument_size());
    for(typename Differential<X>::ConstIterator iter=x.begin(); iter!=x.end(); ++iter) {
        a=iter->index();
        Nat n=a[i];
        if(n!=0) {
            const X& xc=x[a];
            --a[i];
            X& rc=r[a];
            rc=xc*n;
        }
    }
    return r;
}

template<class X>
Differential<X> Differential<X>::_antiderivative(const Differential<X>& x, SizeType i)
{
    Differential<X> r(x.argument_size(), x.degree()+1u,x.zero_coefficient());
    MultiIndex a(x.argument_size());
    MultiIndex ra=MultiIndex(x.argument_size());
    MultiIndex ai=MultiIndex::unit(x.argument_size(),i);
    for(typename Differential<X>::ConstIterator iter=x.begin(); iter!=x.end(); ++iter) {
        a=iter->index();
        const X& xc=x[a];
        ++a[i];
        Nat n=a[i];
        X& rc=r[a];
        rc=xc/n;
    }
    return r;
}


template<class X> OutputStream& write_expansion(OutputStream& os, Differential<X> const& dx) {
    Expansion<MultiIndex,X> e=dx.expansion();
    //e.graded_sort();
    os << "{";
    for(typename Expansion<MultiIndex,X>::ConstIterator iter=e.begin(); iter!=e.end(); ++iter) {
        if(iter!=e.begin()) { os << ","; } os << " ";
        for(SizeType i=0; i!=e.argument_size(); ++i) {
            if(i!=0) { os << ","; }
            os << SizeType(iter->index()[i]);
        }
        os << ":" << X(iter->coefficient());
    }
    return os << " }";
}

template<class X>
OutputStream& Differential<X>::_write(OutputStream& os) const
{
    Differential<X> const& dx=*this;
    os << "Differential(as="<<dx.argument_size()<<",deg="<<(uint)dx.degree()<<")";
    write_expansion(os,dx);
    return os;
}





template<class X> Vector<Differential<X>>::Vector(SizeType rs, SizeType as, DegreeType d, X const& z)
    : _ary(rs,Differential<X>(as,d,z)) {
}

template<class X> Vector<Differential<X>>::Vector(SizeType rs, const Differential<X>& sd)
    : _ary(rs,sd) {
}

template<class X> Vector<Differential<X>>::Vector(InitializerList<Differential<X>> const& lst)
    : _ary(lst)
{
}

template<class X> Vector<Differential<X>>::Vector(Array<Differential<X>> ary)
    : _ary(std::move(ary))
{
    ARIADNE_ASSERT(_ary.size()>0);
}



template<class X> Vector<X> Vector<Differential<X>>::value() const {
    Vector<Differential<X>> const& x=*this;
    Vector<X> r(x.result_size(),x.zero_coefficient());
    for(SizeType i=0; i!=r.size(); ++i) {
        r[i]=x[i].value();
    }
    return r;
}

template<class X> Matrix<X> Vector<Differential<X>>::jacobian() const {
    Vector<Differential<X>> const& x=*this;
    Matrix<X> r(x.size(),x.argument_size(),x.zero_coefficient());
    for(SizeType i=0; i!=x.size(); ++i) {
        for(SizeType j=0; j!=x[0].argument_size(); ++j) {
            r[i][j]=x[i].gradient(j);

        }
    }
    return r;
}

template<class X> Void Vector<Differential<X>>::set_value(const Vector<X>& c) {
    ARIADNE_ASSERT(this->result_size()==c.size());
    for(SizeType i=0; i!=c.size(); ++i) {
        (*this)[i].set_value(c[i]);
    }
}

template<class X> Void Vector<Differential<X>>::set_degree(DegreeType deg) {
    for(SizeType i=0; i!=this->size(); ++i) {
        (*this)[i].set_degree(deg);
    }
}



template<class X> Vector<Differential<X>> Vector<Differential<X>>::_compose(Vector<Differential<X>> const& x, Vector<Differential<X>> const& y) {
    Vector<Differential<X>> r(x.size(),y.zero_element());
    r.set_degree(min(x.degree(),y.degree()));
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=compose(x[i],y); }
    return r;
}

template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_derivative(const Vector<Differential<X>>& x, SizeType k) {
    X z=x.zero_element().zero_coefficient();
    Vector<Differential<X>> r(x.size(), Differential<X>(x.argument_size(),max(x.degree(),(DegreeType)1u)-1u,z));
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=derivative(x[i],k); }
    return r;
}

template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_antiderivative(const Vector<Differential<X>>& x, SizeType k) {
    X z=x.zero_element().zero_coefficient();
    Vector<Differential<X>> r(x.size(), Differential<X>(x.argument_size(),x.degree()+1u,z));
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=antiderivative(x[i],k); }
    return r;
}




template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_lie_derivative(const Vector<Differential<X> >& df, const Vector<Differential<X> >& dg)
{
    X z=dg.zero_element().zero_coefficient();
    Vector<Differential<X>> r(df.result_size(),df.argument_size(),df.degree()-1u,z);
    Differential<X> t(df.argument_size(), df.degree()-1u,z);
    MultiIndex a(df.argument_size()); X c=z;
    for(SizeType i=0; i!=df.result_size(); ++i) {
        Expansion<MultiIndex,X> const& dfi_expansion = df[i].expansion();
        Expansion<MultiIndex,X>& t_expansion = t.expansion();
        for(SizeType j=0; j!=df.argument_size(); ++j) {
            for(typename Expansion<MultiIndex,X>::ConstIterator iter=dfi_expansion.begin(); iter!=dfi_expansion.end(); ++iter) {
                if(iter->index()[j]!=0) {
                    a=iter->index();
                    c=iter->coefficient()*SizeType(a[j]);
                    a[j]-=1;
                    t_expansion.append(a,c);
                }
            }
            r[i]+=t*dg[j];
            t.clear();
        }
    }
    return r;
}



template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_solve(const Vector<Differential<X> >& df, const Vector<X>& y0)
{
    ARIADNE_ASSERT(df.result_size()<=df.argument_size());
    ARIADNE_ASSERT(df.result_size()==y0.size());

    const SizeType m=df.result_size();
    const SizeType n=df.argument_size();
    const SizeType deg=df.degree();
    const X z=df.zero_element().zero_coefficient();

    const SizeType l=n-m;

    Vector<Differential<X>> dx=Vector<Differential<X>>(l,l,deg,z);
    for(SizeType i=0; i!=l; ++i) { dx[i]=Differential<X>::variable(l,deg,z,i); }

    Matrix<X> Jf = df.jacobian();
    //Matrix<X> J1f = project(Jf,range(0,m),range(0,n-m));
    Matrix<X> J2f = project(Jf,range(0,m),range(n-m,n));
    Matrix<X> invJ2f = inverse(J2f);

    // Iterate Newton-like steps to find derivatives of solution
    // TODO: This iteration repeats computation of lower-degree terms, so could be made more efficient
    Vector<Differential<X>> dh=Differential<X>::constants(m,l,deg,y0);
    for(SizeType i=0; i<=df.degree()+1u; ++i) {
        dh = dh - invJ2f * compose(df,join(dx,dh));
    }
    return dh;
}

template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_flow(const Vector<Differential<X> >& df, Vector<X> const& x0)
{
    ARIADNE_ASSERT(df.result_size()==df.argument_size());
    ARIADNE_ASSERT(x0.size()==df.argument_size());
    const SizeType n=df.result_size();
    const SizeType deg=df.degree();
    const X z=df.zero_element().zero_coefficient();
    Vector<Differential<X>> dx0=Vector<Differential<X>>(n,[&](SizeType i){return Differential<X>::variable(n+1u,deg+1u,x0[i],i);});
    Vector<Differential<X>> dphi=Vector<Differential<X>>(n,n+1u,0u,z);
    // TODO: This iteration repeats computation of lower-degree terms, so could be made more efficient
    for(SizeType i=0; i<deg+1u; ++i) {
        dphi=dx0+antiderivative(compose(df,dphi),n);
    }
    return dphi;
}

template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_flow(const Vector<Differential<X> >& df, Vector<X> const& x0, X const& t0)
{
    ARIADNE_ASSERT(df.result_size()==x0.size());
    ARIADNE_ASSERT(df.argument_size()==x0.size()+1u);
    const SizeType n=x0.size(); // Number of state variables; also index of time variable
    const SizeType deg=df.degree();

    Vector<Differential<X>> dx0=Vector<Differential<X>>(n,[&](SizeType i){return Differential<X>::variable(n+1u,deg+1u,x0[i],i);});
    Vector<Differential<X>> dt0=Vector<Differential<X>>(1,Differential<X>::variable(n+1u,deg+1u,t0,n));

    return _flow(df,dx0, dt0);
}

template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_flow(const Vector<Differential<X> >& df, Vector<X> const& x0, Vector<X> const& a)
{
    ARIADNE_ASSERT(df.result_size()==x0.size());
    ARIADNE_ASSERT(df.argument_size()==x0.size()+a.size());
    const SizeType n=x0.size(); // Number of state variables; also index of time variable
    const SizeType m=a.size();
    const SizeType deg=df.degree();

    Vector<Differential<X>> dx0=Vector<Differential<X>>(n,[&](SizeType i){return Differential<X>::variable(n+1u+m,deg+1u,x0[i],i);});
    Vector<Differential<X>> da(m,n+1u+m,deg+1u,a.zero_element());
    for (SizeType i=0; i!=m; ++i) { da[i]=Differential<X>::variable(n+1u+m,deg+1u,a[i],n+i); }

    return _flow(df,dx0, da);
}

template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_flow(const Vector<Differential<X> >& df, Vector<X> const& x0, X const& t0, Vector<X> const& a)
{
    ARIADNE_ASSERT(df.result_size()==x0.size());
    ARIADNE_ASSERT(df.argument_size()==x0.size()+1u+a.size());
    const SizeType n=x0.size(); // Number of state variables; also index of time variable
    const SizeType m=a.size();
    const SizeType deg=df.degree();

    Vector<X> t0a=join(t0,a);
    Vector<Differential<X>> dx0=Vector<Differential<X>>(n,[&](SizeType i){return Differential<X>::variable(n+1u+m,deg+1u,x0[i],i);}, n+1u+m,deg+1u,x0.element_characteristics());
    Vector<Differential<X>> dt0a=Vector<Differential<X>>(m+1,[&](SizeType i){return Differential<X>::variable(n+1u+m,deg+1u,t0a[i],n+i);}, n+1u+m,deg+1u,a.element_characteristics());

    return _flow(df,dx0, dt0a);
}

template<class X>
Vector<Differential<X>>
Vector<Differential<X>>::_flow(const Vector<Differential<X>>& df, const Vector<Differential<X>>& dx0, const Vector<Differential<X>>& dt0a)
{
    ARIADNE_ASSERT(df.result_size()==dx0.result_size());
    ARIADNE_ASSERT(df.argument_size()==dx0.result_size()+dt0a.result_size());
    ARIADNE_ASSERT(dx0.argument_size()==dt0a.argument_size());
    ARIADNE_ASSERT(dx0.result_size()<dx0.argument_size()); // dx0 has strictly more arguments since time is an input

    const SizeType n=dx0.result_size(); const SizeType p=dx0.argument_size();
    const SizeType deg=min(min(df.degree(),dt0a.degree())+1u,dx0.degree());
    const X z=dx0.zero_element().zero_coefficient();

    Vector<Differential<X>> dphi=Vector<Differential<X>>(n,p,0u,z);
    for(SizeType i=0; i<deg; ++i) {
        dphi=dx0+antiderivative(compose(df,join(dphi,dt0a)),n);
    }
    return dphi;
}


template<class X>
OutputStream& Vector<Differential<X>>::_write(OutputStream& os) const
{
    Vector<Differential<X>> const& dx=*this;
    os << "DifferentialVector(rs="<<dx.result_size()<<",as="<<dx.argument_size()<<",deg="<<(uint)dx.degree()<<")";
    os << "[ "; for(SizeType i=0; i!=this->size(); ++i) { if(i!=0) { os << ", "; } write_expansion(os,dx[i]); } os << " ]";
    return os;
}


} //namespace Ariadne
