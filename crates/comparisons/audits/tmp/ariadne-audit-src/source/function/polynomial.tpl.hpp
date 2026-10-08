/***************************************************************************
 *            polynomial.tpl.hpp
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

#include "algebra/evaluate.tpl.hpp"
#include "algebra/expansion.inl.hpp"
#include "algebra/expansion.tpl.hpp"

namespace Ariadne {

template<class X> inline bool is_null(X const& x) { return decide(x==0); }
template<class X> inline bool is_unit(X const& x) { return decide(x==1); }
template<class X> inline bool is_positive(X const& x) { return decide(x>=0); }

inline MultiIndex zero_index(SizeType as) { return MultiIndex::zero(as); }
inline DegreeType zero_index(SizeOne) { return 0u; }

inline MultiIndex unit_index(SizeType as, DegreeType k) { return MultiIndex::unit(as,k); }
inline DegreeType unit_index(SizeOne, IndexZero) { return 1u; }


template<class I, class X> Polynomial<I,X>::Polynomial(ArgumentSizeType as, X const& z)
    : _expansion(as,get_characteristics(z))
{
}

template<class I, class X> Polynomial<I,X>::Polynomial(ArgumentSizeType as, CoefficientCharacteristicsType pr)
    : _expansion(as,pr)
{
}

template<class I, class X>
Polynomial<I,X>::Polynomial(InitializerList<Pair<IndexInitializerType,X>> lst)
    : _expansion(lst)
{
    this->cleanup();
}


template<class I, class X> auto Polynomial<I,X>::characteristics() const -> CharacteristicsType {
    return std::make_tuple(this->argument_size(),Ariadne::characteristics(this->zero_coefficient()));
}

template<class I, class X> Polynomial<I,X> Polynomial<I,X>::create_zero() const {
    return Polynomial<I,X>(this->argument_size(),this->zero_coefficient());
}


template<class I, class X> Polynomial<I,X> Polynomial<I,X>::_constant(ArgumentSizeType as, const X& c) {
    Polynomial<I,X> r(as,nul(c)); r[zero_index(as)]=c; return r;}

template<class I, class X> Polynomial<I,X> Polynomial<I,X>::_coordinate(ArgumentSizeType as, VariableIndexType j, const X& z) {
    ARIADNE_ASSERT(j<as); Polynomial<I,X> r(as,z); r[unit_index(as,j)]=1; return r;
}

template<class I, class X> auto Polynomial<I,X>::coordinates(ArgumentSizeType as, X const& z) -> Argument<Polynomial<I,X>> {
    Polynomial<I,X> zero(as,z);
    if constexpr (Same<I,MultiIndex>) {
        Vector<Polynomial<I,X>> r(as,zero); for(SizeType i=0; i!=as; ++i) { r[i]=coordinate(as,i,z); } return r;
    } else {
         return coordinate(as,IndexZero(),z);
    }
}

template<class I, class X> auto Polynomial<I,X>::variables(ArgumentSizeType as, X const& z) -> Argument<Polynomial<I,X>> {
    return coordinates(as,z);
}

template<class I, class X> Polynomial<I,X>& Polynomial<I,X>::operator=(const X& x) {
    this->_expansion.clear();
    if constexpr (Same<I,MultiIndex>) { this->_expansion.append(MultiIndex(this->argument_size()),x); }
    if constexpr (Same<I,DegreeType>) { this->_expansion.append(DegreeType(0u),x); }
    return *this;
}


template<class I, class X> auto Polynomial<I,X>::argument_size() const -> ArgumentSizeType { return this->_expansion.argument_size(); }

template<class I, class X> SizeType Polynomial<I,X>::number_of_terms() const { return this->_expansion.number_of_terms(); }

template<class I, class X> DegreeType Polynomial<I,X>::degree() const {
    DegreeType deg=0u; for(auto iter=this->_expansion.begin(); iter!=this->_expansion.end(); ++iter) {
        if constexpr (Same<I,MultiIndex>) { deg=std::max(deg,iter->index().degree()); }
        else if constexpr (Same<I,DegreeType>) { deg=std::max(deg,iter->index()); }
    }
    return deg;
}

template<class I, class X> const X& Polynomial<I,X>::value() const { return this->_expansion[zero_index(this->argument_size())]; }

template<class I, class X> X& Polynomial<I,X>::operator[](const IndexType& a) { return this->_expansion.at(a); }

template<class I, class X> const X& Polynomial<I,X>::operator[](const IndexType& a) const { return this->_expansion.get(a); }

template<class I, class X> const Expansion<I,X>& Polynomial<I,X>::expansion() const { return this->_expansion; }

template<class I, class X> Expansion<I,X>& Polynomial<I,X>::expansion() { return this->_expansion; }

template<class I, class X> X const& Polynomial<I,X>::zero_coefficient() const { return this->_expansion.zero_coefficient(); }



template<class I, class X> typename Polynomial<I,X>::Iterator Polynomial<I,X>::begin() { return this->_expansion.begin(); }

template<class I, class X> typename Polynomial<I,X>::Iterator Polynomial<I,X>::end() { return this->_expansion.end(); }

template<class I, class X> typename Polynomial<I,X>::Iterator Polynomial<I,X>::find(const I& a) { return this->_expansion.find(a); }

template<class I, class X> typename Polynomial<I,X>::ConstIterator Polynomial<I,X>::begin() const { return this->_expansion.begin(); }

template<class I, class X> typename Polynomial<I,X>::ConstIterator Polynomial<I,X>::end() const { return this->_expansion.end(); }

template<class I, class X> typename Polynomial<I,X>::ConstIterator Polynomial<I,X>::find(const I& a) const { return this->_expansion.find(a); }



template<class I, class X> Void Polynomial<I,X>::_append(const I& a, const X& c) { this->_expansion.append(a,c); }

template<class I, class X> Void Polynomial<I,X>::insert(const I& a, const X& c) { this->_expansion.insert(a,c); }

template<class I, class X> Void Polynomial<I,X>::reserve(SizeType n) { this->_expansion.reserve(n); }

template<class I, class X> Void Polynomial<I,X>::erase(Iterator iter) { this->_expansion.erase(iter); }

template<class I, class X> Void Polynomial<I,X>::clear() { this->_expansion.clear(); }



template<class I, class X>
Polynomial<I,X>&
Polynomial<I,X>::differentiate(VariableIndexType j) {
    for(typename Polynomial<I,X>::Iterator iter=this->begin(); iter!=this->end(); ++iter) {
        IndexReference a=iter->index();
        CoefficientReference c=iter->coefficient();
        c*=static_cast<Nat>(a[j]); if(a[j]!=0u) { --a[j]; }
    }
    return *this;
}

template<class I, class X>
Polynomial<I,X>&
Polynomial<I,X>::antidifferentiate(VariableIndexType j) {
    for(typename Polynomial<I,X>::Iterator iter=this->begin(); iter!=this->end(); ++iter) {
        IndexReference a=iter->index();
        CoefficientReference c=iter->coefficient();
        ++a[j];
        c/=static_cast<Nat>(a[j]);
    }
    return *this;
}

template<class I, class X>
Polynomial<I,X>& Polynomial<I,X>::truncate(DegreeType d) {
    Polynomial<I,X> r(this->argument_size(),this->zero_coefficient());
    for(typename Polynomial<I,X>::ConstIterator iter=this->begin(); iter!=this->end(); ++iter) {
        if(degree_of(iter->index())<=d && decide(iter->coefficient()!=0)) {
            r._append(iter->index(),iter->coefficient());
        }
    }
    this->_expansion.swap(r._expansion);
    return *this;
}

template<class FwdIter, class Op>
FwdIter unique_key(FwdIter first, FwdIter last, Op op) {
    FwdIter curr=first;
    FwdIter next=curr;
    while(next!=last) {
        if(curr!=next) { *curr=*next; }
        ++next;
        while(next!=last && curr->index()==next->index()) {
            if(curr->index()==next->index()) {
                curr->coefficient()=op(curr->coefficient(),next->coefficient());
                ++next;
            }
        }
        // Removes zero entries; the code below is preferred to the case "curr->coefficient()!=0" for ValidatedKleenean results
        if(definitely(curr->coefficient()==0)) { }
        else { ++curr; }
    }
    return curr;
}

template<class I, class X>
Void Polynomial<I,X>::cleanup()
{
    Polynomial<I,X>* self=const_cast<Polynomial<I,X>*>(this);
    self->_expansion.index_sort(IndexComparisonType());
    Iterator new_end=unique_key(self->_expansion.begin(), self->_expansion.end(), std::plus<X>());
    self->_expansion.resize(static_cast<SizeType>(new_end-self->_expansion.begin()));
}

template<class I, class X>
Void Polynomial<I,X>::check() const
{
    this->_expansion.check();
}




template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Nul, const Polynomial<I,X>& p) {
    return Polynomial<I,X>(p.argument_size(),p.zero_coefficient());
}

template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Pos, const Polynomial<I,X>& p) {
    Polynomial<I,X> r(p.argument_size(),p.zero_coefficient());
    for(auto iter=p.begin(); iter!=p.end(); ++iter) {
        r[iter->index()]=+iter->coefficient();
    }
    return r;
}

template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Neg, const Polynomial<I,X>& p) {
    Polynomial<I,X> r(p.argument_size(),p.zero_coefficient());
    for(auto iter=p.begin(); iter!=p.end(); ++iter) {
        r[iter->index()]=-iter->coefficient();
    }
    return r;
}


template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Add, Polynomial<I,X> p, const X& c) {
    p[IndexType(p.argument_size())]+=c;
    return p;
}

template<class I, class X> Polynomial<I,X>& AlgebraOperations<Polynomial<I,X>>::iapply(Add, Polynomial<I,X>& p, const X& c) {
    p[IndexType(p.argument_size())]+=c;
    return p;
}

template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Mul, Polynomial<I,X> p, const X& c) {
    if(is_null(c)) {
        p.expansion().clear();
    } else {
        for(auto iter=p.begin(); iter!=p.end(); ++iter) {
            iter->coefficient()*=c;
        }
    }
    return p;
}

template<class I, class X> Polynomial<I,X>& AlgebraOperations<Polynomial<I,X>>::iapply(Mul, Polynomial<I,X>& p, const X& c) {
    if(is_null(c)) {
        p.expansion().clear();
    } else {
        for(auto iter=p.begin(); iter!=p.end(); ++iter) {
            iter->coefficient()*=c;
        }
    }
    return p;
}

template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Add, const Polynomial<I,X>& p1, const Polynomial<I,X>& p2) {
    ARIADNE_ASSERT(p1.argument_size()==p2.argument_size());
    typename Polynomial<I,X>::IndexComparisonType less;
    Polynomial<I,X> r(p1.argument_size(),(p1.zero_coefficient()+p2.zero_coefficient()));
    auto iter1=p1.begin(); auto iter2=p2.begin();
    while (iter1!=p1.end() && iter2!=p2.end()) {
        if (iter1->index()==iter2->index()) {
            r._expansion.append(iter1->index(),iter1->coefficient()+iter2->coefficient());
           ++iter1; ++iter2;
         } else if (less(iter1->index(),iter2->index())) {
            r._expansion.append(iter1->index(),iter1->coefficient());
            ++iter1;
        } else { //  (greater(iter1->index(),iter2->index()))
            r._expansion.append(iter2->index(),iter2->coefficient());
            ++iter2;
        }
    }
    while (iter1!=p1.end()) {
        r._expansion.append(iter1->index(),iter1->coefficient());
        ++iter1;
    }
    while (iter2!=p2.end()) {
            r._expansion.append(iter2->index(),iter2->coefficient());
            ++iter2;
    }
    return r;
}

template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Sub, const Polynomial<I,X>& p1, const Polynomial<I,X>& p2) {
    ARIADNE_ASSERT(p1.argument_size()==p2.argument_size());
    typename Polynomial<I,X>::IndexComparisonType less;
    Polynomial<I,X> r(p1.argument_size(),(p1.zero_coefficient()-p2.zero_coefficient()));
    auto iter1=p1.begin(); auto iter2=p2.begin();
    while (iter1!=p1.end() && iter2!=p2.end()) {
        if (iter1->index()==iter2->index()) {
            r._expansion.append(iter1->index(),iter1->coefficient()-iter2->coefficient());
           ++iter1; ++iter2;
         } else if (less(iter1->index(),iter2->index())) {
            r._expansion.append(iter1->index(),iter1->coefficient());
            ++iter1;
        } else { //  (greater(iter1->index(),iter2->index()))
            r._expansion.append(iter2->index(),-iter2->coefficient());
            ++iter2;
        }
    }
    while (iter1!=p1.end()) {
        r._expansion.append(iter1->index(),iter1->coefficient());
        ++iter1;
    }
    while (iter2!=p2.end()) {
            r._expansion.append(iter2->index(),-iter2->coefficient());
            ++iter2;
    }
    return r;
}

template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Mul, const Polynomial<I,X>& p1, const Polynomial<I,X>& p2) {
    ARIADNE_ASSERT(p1.argument_size()==p2.argument_size());
    Polynomial<I,X> r(p1.argument_size(),(p1.zero_coefficient()*p2.zero_coefficient()));
    for(auto iter1=p1.begin(); iter1!=p1.end(); ++iter1) {
        for(auto iter2=p2.begin(); iter2!=p2.end(); ++iter2) {
            I a=iter1->index()+iter2->index();
            r[a]+=iter1->coefficient()*iter2->coefficient();
        }
    }
    return r;
}

template<class I, class X> Polynomial<I,X> AlgebraOperations<Polynomial<I,X>>::apply(Mul, Polynomial<I,X> p, const Monomial<I,X>& m) {
    if(is_null(m.coefficient())) { p.clear(); }
    for(auto iter=p.begin(); iter!=p.end(); ++iter) {
        iter->index()+=m.index();
        iter->coefficient()*=m.coefficient();
    }
    return p;
}

template<class I, class X> Polynomial<I,X>& AlgebraOperations<Polynomial<I,X>>::iapply(Mul, Polynomial<I,X>& p, const Monomial<I,X>& m) {
    if(is_null(m.coefficient())) { p.clear(); }
    for(auto iter=p.begin(); iter!=p.end(); ++iter) {
        iter->index()+=m.index();
        iter->coefficient()*=m.coefficient();
    }
    return p;
}


template<class I, class X> X Polynomial<I,X>::_evaluate(const Polynomial<I,X>& p, const Argument<X>& x) {
    return horner_evaluate(p._expansion,x);
}

template<class I, class X> Polynomial<UniIndex,X> Polynomial<I,X>::_compose(const Polynomial<I,X>& p, const Argument<Polynomial<UniIndex,X>>& q) {
    return evaluate(p,q);
}

template<class I, class X> Polynomial<MultiIndex,X> Polynomial<I,X>::_compose(const Polynomial<I,X>& p, const Argument<Polynomial<MultiIndex,X>>& q) {
    auto r = evaluate(p,q);
    return r;
}




template<class I, class X>
Polynomial<I,X>
Polynomial<I,X>::_partial_evaluate(const Polynomial<I,X>& x, SizeType k, const X& c)
{
    if constexpr (Same<I,MultiIndex>) {
        Polynomial<I,X> r(x.argument_size()-1u,x.zero_coefficient());
        MultiIndex ra(r.argument_size());
        if(is_null(c)) {
            for(typename Polynomial<I,X>::ConstIterator xiter=x.begin(); xiter!=x.end(); ++xiter) {
                IndexConstReference xa=xiter->index();
                MultiIndex::IndexType xak=xa[k];
                if(xak==0) {
                    CoefficientConstReference xv=xiter->coefficient();
                    for(SizeType i=0; i!=k; ++i) { ra[i]=xa[i]; }
                    for(SizeType i=k; i!=ra.size(); ++i) { ra[i]=xa[i+1u]; }
                    r.expansion().append(ra,xv);
                }
            }
        } else if(is_unit(c)) {
            Polynomial<I,X> s(x.argument_size()-1u,x.zero_coefficient());
            Array< Polynomial<I,X> > p(x.degree()+1u,Polynomial<I,X>(x.argument_size()-1u,x.zero_coefficient()));

            for(typename Polynomial<I,X>::ConstIterator xiter=x.begin(); xiter!=x.end(); ++xiter) {
                IndexConstReference xa=xiter->index();
                CoefficientConstReference xv=xiter->coefficient();
                MultiIndex::IndexType xak=xa[k];
                for(SizeType i=0; i!=k; ++i) { ra[i]=xa[i]; }
                for(SizeType i=k; i!=ra.size(); ++i) { ra[i]=xa[i+1u]; }
                assert(ra.degree()+xak==xa.degree());
                p[xak].expansion().append(ra,xv);
            }

            r=p[0];
            for(SizeType i=1; i!=p.size(); ++i) {
                r+=p[i];
            }
        } else {
            Polynomial<I,X> s(x.argument_size()-1u,x.zero_coefficient());
            Array< Polynomial<I,X> > p(x.degree()+1u,Polynomial<I,X>(x.argument_size()-1u,x.zero_coefficient()));

            Array<X> cpowers(x.degree()+1u,x.zero_coefficient());
            cpowers[0]=1; cpowers[1]=c;
            if(x.degree()>=2) { cpowers[2]=sqr(c); }
            for(DegreeType j=3; j<=x.degree(); ++j) {
                cpowers[j]=cpowers[j-2u]*cpowers[2];
            }

            for(typename Polynomial<I,X>::ConstIterator xiter=x.begin(); xiter!=x.end(); ++xiter) {
                IndexConstReference xa=xiter->index();
                CoefficientConstReference xv=xiter->coefficient();
                MultiIndex::IndexType xak=xa[k];
                for(SizeType i=0; i!=k; ++i) { ra[i]=xa[i]; }
                for(SizeType i=k; i!=ra.size(); ++i) { ra[i]=xa[i+1u]; }
                assert(ra.degree()+xak==xa.degree());
                p[xak].expansion().append(ra,xv);
            }
            for(SizeType i=1; i!=p.size(); ++i) {
                p[i]*=cpowers[i];
            }

            r=p[0];
            for(SizeType i=1; i!=p.size(); ++i) {
                r+=p[i];
            }
        }

        return r;
    } else {
        abort();
    }
}



/*
template<class I, class X>
OutputStream& operator<<(OutputStream& os, const Polynomial<I,X>& p) {
    if(p.begin()==p.end()) {
        return os << "{"<<MultiIndex::zero(p.argument_size())<<":0.0}"; }

    os << "{";
    for(typename Polynomial<I,X>::ConstIterator iter=p.begin(); iter!=p.end(); ++iter) {
        os << (iter==p.begin() ? "" : ",");
        for(SizeType i=0; i!=iter->index().size(); ++i) {
            os << (i==0?" ":",") << Int(iter->index()[i]); }
        os << ":" << iter->coefficient(); }
    return os << " }";
}
*/

String canonical_argument_names(SizeOne) {
    return "x";
}

Array<String> canonical_argument_names(SizeType n) {
    Array<String> argument_names(n);
    for(SizeType i=0; i!=n; ++i) {
        StringStream ss;
        ss << "x" << i;
        argument_names[i]=ss.str();
    }
    return argument_names;
}

template<class I, class X>
OutputStream& Polynomial<I,X>::_write(OutputStream& os) const {
    return this->_write(os,canonical_argument_names(this->argument_size()));
}

template<class I, class X>
OutputStream& Polynomial<I,X>::_write(OutputStream& os, typename IndexTraits<I>::NameType const& argument_names) const {
    return this->_expansion._write(os,argument_names);
}

template<class I, class X> Vector<Polynomial<I,X>> flow_polynomial_picard_iteration(Vector<Polynomial<I,X>> const& f, DegreeType d) {
    const SizeType rs = f.size();
    const SizeType as = f.zero_element().argument_size();
    auto pr = f.zero_element().zero_coefficient().precision();

    Vector<Polynomial<I,X>> r(rs,[as,pr](SizeType i){return Polynomial<I,X>::variable(as,i,pr);},std::make_tuple(as,pr));

    auto g = r;

    for (DegreeType j=1; j!=d+1; ++j) {
        r = truncate(g + antiderivative(compose(f,r),rs),j);
    }

    Vector<Polynomial<I,X>> ext(as, [as,rs,pr](SizeType i){return Polynomial<I,X>::variable(as+rs,i,pr);}, std::make_tuple(as+rs,pr));
    r = compose(r,ext);
    for (SizeType i=0; i!=rs; ++i) r[i] += Polynomial<I,X>::variable(as+rs,as+i,pr);
    return r;
}

template<class I, class X> Vector<Polynomial<I,X>> flow_polynomial_lie_derivative(Vector<Polynomial<I,X>> const& f, DegreeType d) {
    const SizeType rs = f.size();
    const SizeType as = f.zero_element().argument_size();
    auto pr = f.zero_element().zero_coefficient().precision();

    auto t = Polynomial<I,X>::variable(as,rs,pr);
    auto ti = t;
    Vector<Polynomial<I,X>> g(rs, [as,pr](SizeType i){return Polynomial<I,X>::variable(as,i,pr);}, std::make_tuple(as,pr) );
    Vector<Polynomial<I,X>> r = g;

    for (DegreeType i=1; i!=d+1; ++i) {
        g = truncate(lie_derivative(f,g),d-i);
        r += g*ti*rec(Factorial(i));
        ti *= t;
    }

    Vector<Polynomial<I,X>> ext(as, [rs,as,pr](SizeType i){return Polynomial<I,X>::variable(as+rs,i,pr);}, std::make_tuple(as+rs,pr));
    r = compose(r,ext);
    for (SizeType i=0; i!=rs; ++i) r[i] += Polynomial<I,X>::variable(as+rs,as+i,pr);
    return r;
}

template<class I, class X> Vector<Polynomial<I,X>> flow_polynomial(Vector<Polynomial<I,X>> const& f, DegreeType d) {
    return flow_polynomial_picard_iteration(f,d);
}





} // namespace Ariadne
