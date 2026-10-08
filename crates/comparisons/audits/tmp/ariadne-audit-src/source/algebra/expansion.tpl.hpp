/***************************************************************************
 *            algebra/expansion.tpl.hpp
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

/*! \file algebra/expansion.tpl.hpp
 *  \brief
 */

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <iostream>

#include "expansion.hpp"
#include "expansion.inl.hpp"
#include "foundations/logical.hpp"

namespace Ariadne {

inline SizeType word_size(SizeType as) { return (1u+as)/sizeof(int)+1; }

inline double nul(double d) { return 0.0; }
inline double abs(double d) { return std::fabs(d); }


template<class I, class X> Expansion<I,X>::~Expansion()
{
}

template<class I, class X> Expansion<I,X>::Expansion(ArgumentSizeType as, CharacteristicsType<X> const& prs, SizeType cap)
    : _indices(0u,I(as)), _coefficients(prs)
{
    _indices.reserve(cap); _coefficients.reserve(cap);
}

/*
template<class I, class X> Expansion<I,X>::Expansion(InitializerList<Pair<IndexInitializerType,X>> lst)
    : Expansion( Expansion(size_of(lst.begin()->first),nul(lst.begin()->second),std::max(DEFAULT_CAPACITY,lst.size()) ) )
{
    I a(this->argument_size());
    X x;
    for(auto iter=lst.begin();
        iter!=lst.end(); ++iter)
    {
        a=iter->first;
        x=iter->second;
        if(decide(x!=0)) { this->append(a,x); }
    }
}
*/

namespace {
template<class I, class X> CharacteristicsType<X> get_coefficient_characteristics(InitializerList<Pair<I,X>> const& lst) {
    ARIADNE_PRECONDITION(lst.size()!=0);
    return get_characteristics(lst.begin()->second);
}
} // namespace

template<class I, class X> Expansion<I,X>::Expansion(InitializerList<Pair<IndexInitializerType,X>> lst)
    : Expansion(ArgumentSizeType(),get_coefficient_characteristics(lst))
{
    ARIADNE_PRECONDITION(lst.size()!=0);

    _indices = UniformList<I>(0u,I(size_of(lst.begin()->first)));

    SizeType cap = std::max(DEFAULT_CAPACITY,lst.size());
    _indices.reserve(cap);
    _coefficients.reserve(cap);

    I a(this->argument_size());
    X x=this->zero_coefficient();
    for(auto iter=lst.begin();
        iter!=lst.end(); ++iter)
    {
        a=iter->first;
        x=iter->second;
        if(!decide(x==0)) { this->append(a,x); }
    }
}


//template<class I, class X> Expansion<I,X>::Expansion(InitializerList<Pair<InitializerList<DegreeType>,X>> lst) : Expansion(3) {
//    ARIADNE_PRECONDITION(lst.size()!=0);
//}

/*
// Call std::memcpy(dest,src,count) with same arguments as std::copy(src_begin,src_end,dest)
namespace std {
    template<> inline void copy(const unsigned char* b, const unsigned char* e, unsigned char* t) { memcpy(t,b,e-b); }
}
*/

template<class I, class X> Expansion<I,X>::Expansion(const Expansion<I,X>& e)
    : _indices(e._indices), _coefficients(e._coefficients)
{
    this->_indices.reserve(e.capacity());
    this->_coefficients.reserve(e.capacity());
}

template<class I, class X> Expansion<I,X>& Expansion<I,X>::operator=(const Expansion<I,X>& e)
{
    if(this!=&e) {
        // Perform memory reallocation if necessary
        this->_indices = e._indices;
        this->_coefficients = e._coefficients;
        this->_indices.reserve(e.capacity());
        this->_coefficients.reserve(e.capacity());
    }
    return *this;
}

template<class I, class X> Expansion<I,X>::Expansion(Expansion<I,X>&& e)
    : _indices(std::move(e._indices)), _coefficients(std::move(e._coefficients))
{
}

template<class I, class X> Expansion<I,X>& Expansion<I,X>::operator=(Expansion<I,X>&& e)
{
    if(this!=&e) {
        _indices=std::move(e._indices);
        _coefficients=std::move(e._coefficients);
    }
    return *this;
}

template<class I, class X> Void Expansion<I,X>::swap(Expansion<I,X>& other) {
    std::swap(this->_indices,other._indices);
    std::swap(this->_coefficients,other._coefficients);
}

template<class I, class X> SizeType Expansion<I,X>::number_of_terms() const {
    return this->size();
}

template<class I, class X> SizeType Expansion<I,X>::number_of_nonzeros() const {
    return this->size();
}

template<class I, class X> Bool Expansion<I,X>::empty() const {
    return this->size()==0;
}

template<class I, class X> SizeType Expansion<I,X>::size() const {
    return this->_indices.size();
}

template<class X> concept HasPrecision = requires(X const& x) { x.precision(); };

template<class I, class X> auto Expansion<I,X>::coefficient_characteristics() const -> CharacteristicsType<X> {
    if constexpr (HasPrecision<X>) { return this->zero_coefficient().precision(); }
    else if constexpr (HasCharacteristics<X>) { return Ariadne::characteristics(this->zero_coefficient()); }
    else if constexpr (DefaultConstructible<X>) { return std::tuple<>(); }
    else { return std::tuple<>(); }
}

template<class I, class X> auto Expansion<I,X>::characteristics() const -> Pair<ArgumentSizeType,CharacteristicsType<X>> {
    return std::make_pair(this->argument_size(),this->coefficient_characteristics());
}

template<class I, class X> auto Expansion<I,X>::argument_size() const -> ArgumentSizeType {
    return argument_size_of(this->_indices);
}

template<class I, class X> X const& Expansion<I,X>::zero_coefficient() const {
    return this->_coefficients.zero_element();
}

template<class I, class X> Void Expansion<I,X>::reserve(SizeType new_capacity) {
    if(this->capacity() < new_capacity) {
        this->_indices.reserve(new_capacity);
        this->_coefficients.reserve(new_capacity);
    }
}

template<class I, class X> Void Expansion<I,X>::resize(SizeType new_size) {
    if(new_size<this->size()) {
        this->_indices.resize(new_size);
        this->_coefficients.resize(new_size);
    } else {
        if(this->capacity() < new_size) {
            this->reserve(new_size);
        }
        I a(this->argument_size());
        X c=this->zero_coefficient();
        for (SizeType i=this->size(); i!=new_size; ++i) {
            this->append(a,c);
        }
    }
}

template<class I, class X> SizeType Expansion<I,X>::capacity() const {
    return std::min(this->_indices.capacity(),this->_coefficients.capacity());
}

template<class I, class X> Void Expansion<I,X>::clear() {
    this->_indices.clear(); this->_coefficients.clear();
}

template<class I, class X> Void Expansion<I,X>::remove_zeros() {
    this->resize(static_cast<SizeType>(std::remove_if(this->begin(),this->end(),CoefficientIsZero())-this->begin()));
}

template<class X, class Y> concept CanInplaceAdd = requires(X& x, Y const& y) { x+=y; };

template<class I, class X>Void combine_terms(Expansion<I,X>& e) {
    if constexpr (CanInplaceAdd<X,X>) {
        auto begin=e.begin();
        auto end=e.end();
        auto curr=begin;
        auto adv=begin;
        while (adv!=end) {
            curr->index()=adv->index();
            curr->coefficient()=adv->coefficient();
            ++adv;
            while (adv!=end && adv->index()==curr->index()) {
                curr->coefficient() += adv->coefficient();
                ++adv;
            }
            ++curr;
        }
        e.resize(static_cast<SizeType>(curr-begin));
    } else {
        ARIADNE_ASSERT_MSG(false, "Cannot combine terms of an expansion if the coefficients do not support inplace addition.");
    }
}

template<class I, class X> Void Expansion<I,X>::combine_terms() {
    Ariadne::combine_terms(*this);
}

template<class I, class X> Void Expansion<I,X>::check() const {
    ARIADNE_NOT_IMPLEMENTED;
}

template<class I, class X> ExpansionValueReference<I,X> Expansion<I,X>::operator[](const I& a) {
    return ExpansionValueReference<I,X>(*this,a);
}

template<class I, class X> const X& Expansion<I,X>::operator[](const I& a) const {
    return this->get(a);
}

template<class I, class X> X& Expansion<I,X>::at(const I& a) {
    auto iter=this->find(a);
    if(iter==this->end()) { this->append(a,this->zero_coefficient()); iter=this->end()-1; }
    return iter->coefficient();
}

template<class I, class X> Void Expansion<I,X>::set(const I& a, const X& c) {
    auto iter=this->find(a);
    if(iter==this->end()) { this->append(a,c); }
    else { iter->coefficient() = c; }
}

template<class I, class X> const X& Expansion<I,X>::get(const I& a) const {
    auto iter=this->find(a);
    if(iter==this->end()) { return this->zero_coefficient(); }
    else { return iter->coefficient(); }
}


template<class I, class X, class CMP> EqualityType<X,X> SortedExpansion<I,X,CMP>::operator==(const SortedExpansion<I,X,CMP>& other) const {
    SortedExpansion<I,X,CMP> const& e1=*this; SortedExpansion<I,X,CMP> const& e2=other; CMP less;
    if(e1.argument_size()!=e2.argument_size()) { return false; }
    typedef EqualityType<X,X> L;
    L r=true;
    auto iter1 = e1.begin(); auto iter2 = e2.begin();
    while (iter1!=e1.end() && iter2!=e2.end()) {
        if (iter1->index()==iter2->index()) {
            r = r && (iter1->coefficient()==iter2->coefficient());
            ++iter1; ++iter2;
        } else if (less(*iter1,*iter2)) {
            r = r && static_cast<L>(iter1->coefficient()==0);
            ++iter1;
        } else if (less(*iter2,*iter1)) {
            r = r && static_cast<L>(0==iter2->coefficient());
            ++iter2;
        } else {
             r = r && static_cast<L>(iter1->coefficient()==0) && static_cast<L>(0==iter2->coefficient());
        }
    }
    while (iter1!=e1.end()) {
        r = r && static_cast<L>(iter1->coefficient()==0);
        ++iter1;
    }
    while (iter2!=e2.end()) {
        r = r && static_cast<L>(0==iter2->coefficient());
        ++iter2;
    }
    return r;
}


template<class I, class X> Bool Expansion<I,X>::same_as(const Expansion<I,X>& other) const {
    auto iter1=this->begin();
    auto iter2=other.begin();
    auto end1=this->end();
    auto end2=other.end();

    if (this->size()!=other.size()) { return false; }
    if (this->argument_size()!=other.argument_size()) { return false; }

    while(true) {
        if (iter1!=end1 && iter2!=end2) {
            if (!same(*iter1,*iter2)) { return false; }
            ++iter1; ++iter2;
        } else if (iter1==end1 && iter2==end2) {
            return true;
        } else {
            return false;
        }
    }
}

template<class I, class X> auto Expansion<I,X>::insert(Iterator pos, const I& a, const X& c) -> Iterator {
    if(this->size()==this->capacity()) {
        SizeType where=static_cast<SizeType>(pos-this->begin());
        this->append(a,c);
        pos=this->begin()+static_cast<std::ptrdiff_t>(where);
    } else {
        this->append(a,c);
    }
    auto curr=this->end();
    auto prev=curr-1;
    while(prev!=pos) {
        --curr;
        --prev;
        *curr=*prev;
    }
    pos->index()=a;
    pos->coefficient()=c;
    return pos;
}

template<class I, class X> auto Expansion<I,X>::erase(Iterator pos) -> Iterator {
    this->_indices.erase(pos._ap);
    this->_coefficients.erase(pos._cp);
    return pos;
}

template<class I, class X> Void Expansion<I,X>::prepend(const I& a, const X& c) {
    this->append(a,c);
    auto curr=this->end();
    auto prev=curr-1;
    while(prev!=this->begin()) {
        --curr;
        --prev;
        *curr=*prev;
    }
    --curr;
    curr->index()=a;
    curr->coefficient()=c;
}

template<class I, class X> Void Expansion<I,X>::append(const I& a, const X& c) {
    if(this->number_of_terms()==this->capacity()) {
        this->reserve(2*this->capacity());
    }
    this->_indices.append(a);
    this->_coefficients.append(c);
}

template<class I, class X> Void Expansion<I,X>::append_sum(const I& a1, const I& a2, const X& c) {
    if(this->number_of_terms()==this->capacity()) {
        this->reserve(2*this->capacity());
    }
    this->_indices.append(a1+a2);
    this->_coefficients.append(c);
}


template<class I, class X> ExpansionReference<I,X> Expansion<I,X>::front() {
    return ExpansionReference<I,X>(_indices.front(),_coefficients.front());
}

template<class I, class X> ExpansionReference<I,X> Expansion<I,X>::back() {
    return ExpansionReference<I,X>(_indices.back(),_coefficients.back());
}

template<class I, class X> ExpansionConstReference<I,X> Expansion<I,X>::front() const {
    return ExpansionConstReference<I,X>(_indices.front(),_coefficients.front());
}

template<class I, class X> ExpansionConstReference<I,X> Expansion<I,X>::back() const {
    return ExpansionConstReference<I,X>(_indices.back(),_coefficients.back());
}


template<class I, class X> ExpansionIterator<I,X> Expansion<I,X>::begin() {
    return ExpansionIterator<I,X>(_indices.begin(),_coefficients.begin());
}

template<class I, class X> ExpansionIterator<I,X> Expansion<I,X>::end() {
    return ExpansionIterator<I,X>(_indices.end(),_coefficients.end());
}

template<class I, class X> ExpansionConstIterator<I,X> Expansion<I,X>::begin() const {
    return ExpansionConstIterator<I,X>(_indices.begin(),_coefficients.begin());
}

template<class I, class X> ExpansionConstIterator<I,X> Expansion<I,X>::end() const {
    return ExpansionConstIterator<I,X>(_indices.end(),_coefficients.end());
}

template<class I, class X> ExpansionIterator<I,X> Expansion<I,X>::find(const I& a) {
    ExpansionIterator<I,X> iter=this->begin();
    ExpansionIterator<I,X> end=this->end();
    while(iter!=end && iter->index()!=a) {
        ++iter;
    }
    return iter;
}

template<class I, class X> ExpansionConstIterator<I,X> Expansion<I,X>::find(const I& a) const {
    ExpansionConstIterator<I,X> iter=this->begin();
    ExpansionConstIterator<I,X> end=this->end();
    while(iter!=end && iter->index()!=a) {
        ++iter;
    }
    return iter;
}

template<class I, class X> Void Expansion<I,X>::index_sort(ReverseLexicographicLess) {
    std::sort(this->begin(),this->end(),IndexComparison<ReverseLexicographicLess>());
}

template<class I, class X> Void Expansion<I,X>::index_sort(GradedLess) {
    std::sort(this->begin(),this->end(),IndexComparison<GradedLess>());
}

template<class I, class X> Void Expansion<I,X>::sort(ReverseLexicographicIndexLess) {
    std::sort(this->begin(),this->end(),IndexComparison<ReverseLexicographicLess>());
}

template<class I, class X> Void Expansion<I,X>::sort(GradedIndexLess) {
    std::sort(this->begin(),this->end(),IndexComparison<GradedLess>());
}

template<class I, class X> Void Expansion<I,X>::reverse_lexicographic_sort() {
    std::sort(this->begin(),this->end(),IndexComparison<ReverseLexicographicLess>());
}

template<class I, class X> Void Expansion<I,X>::graded_sort() {
    std::sort(this->begin(),this->end(),IndexComparison<GradedLess>());
}

template<class I, class X> Bool Expansion<I,X>::is_sorted(ReverseLexicographicIndexLess) {
    return std::is_sorted(this->begin(),this->end(),ReverseLexicographicIndexLess());
}

template<class I, class X> Bool Expansion<I,X>::is_sorted(GradedIndexLess) {
    return std::is_sorted(this->begin(),this->end(),GradedIndexLess());
}

template<class I, class X> Expansion<MultiIndex,X> Expansion<I,X>::_embed(SizeType before_size, Expansion<I,X> const& x, SizeType after_size)
{
    ArgumentSizeType old_size=x.argument_size();
    SizeType new_size=before_size+old_size+after_size;
    Expansion<MultiIndex,X> r(new_size, x.coefficient_characteristics(), x.capacity());
    IndexType old_index(old_size);
    MultiIndex new_index(new_size);
    for(typename Expansion<I,X>::ConstIterator iter=x.begin(); iter!=x.end(); ++iter) {
        old_index=iter->index();
        static_assert(Same<I,MultiIndex> or Same<I,UniIndex>);
        if constexpr (Same<I,MultiIndex>) {
            for(SizeType j=0; j!=old_size; ++j) { new_index[j+before_size]=old_index[j]; }
        } else {
            new_index[before_size]=old_index;
        }
        r.append(new_index,iter->coefficient());
    }
    return r;
}

template<class X> inline OutputStream& operator<<(OutputStream& os, typename UniformList<X>::Iterator const& iter) {
    return os << iter.operator->(); }
template<class X> inline OutputStream& operator<<(OutputStream& os, typename UniformList<X>::ConstIterator const& iter) {
    return os << iter.operator->(); }

template<class I, class X> OutputStream& Expansion<I,X>::_write(OutputStream& os) const {
    os << "Expansion<I," << class_name<X>() <<">";
    os << "{"<<this->number_of_terms()<<"/"<<this->capacity()<<","<<this->argument_size()<<"\n";
    for(auto iter=this->begin(); iter!=this->end(); ++iter) {
        os << "  "<<iter->index()<<":"<<iter->coefficient()<<",\n";
    }
    os << "}\n";
    return os;
}


inline OutputStream& write(OutputStream& os, UniIndex const& a, String const& name, bool first_factor) {
    if (a!=0) { if (!first_factor) { os << "*"; } os << name; if (a!=1) { os << "^" << a; } } return os;
}

inline OutputStream& write(OutputStream& os, MultiIndex const& a, Array<String> const& names, bool first_factor) {
    for(SizeType j=0; j!=a.size(); ++j) {
        if(a[j]!=0) {
            if(first_factor) { first_factor=false; } else { os <<"*"; }
            os<<names[j]; if(a[j]!=1) { os<<"^"<<int(a[j]); }
        }
    }
    return os;
}

template<class I, class X>
OutputStream& Expansion<I,X>::_write(OutputStream& os, const typename IndexTraits<I>::NameType& variable_names) const
{
    const Expansion<I,X>& p=*this;
    ARIADNE_ASSERT(p.argument_size()==variable_names.size());
    if(p.size()==0) {
        os << "0";
    } else {
        bool first_term=true;
        for(auto iter=p.begin(); iter!=p.end(); ++iter) {
            I a=iter->index();
            X v=iter->coefficient();
            os << " ";
            if(decide(v>=0) && !first_term) { os<<"+"; }
            first_term=false;
            bool first_factor=true;
            if(decide(v<0)) { os<<"-"; }
            if(possibly(abs(v)!=1) || degree_of(a)==0) { os<<abs(v); first_factor=false; }
            write(os,a,variable_names,first_factor);
        }
    }
    return os;
}

template<class I, class X, class CMP> SortedExpansion<I,X,CMP>::SortedExpansion(Expansion<I,X> e)
    : Expansion<I,X>(std::move(e))
{
    this->sort();
}

/*
template<class I, class X, class CMP> auto SortedExpansion<I,X,CMP>::find(const I& a) -> Iterator {
    ExpansionValue<I,X> term(a,this->zero_coefficient());
    return  std::lower_bound(this->begin(),this->end(),a,CMP());
}

template<class I, class X, class CMP> auto SortedExpansion<I,X,CMP>::find(const I& a) const -> ConstIterator {
    ExpansionValue<I,X> term(a,this->zero_coefficient());
    return std::lower_bound(this->begin(),this->end(),a,CMP());
}
*/
template<class I, class X, class CMP> auto SortedExpansion<I,X,CMP>::get(const I& a) const -> CoefficientType const& {
    ExpansionValue<I,X> term(a,this->zero_coefficient());
    auto iter=std::lower_bound(this->begin(),this->end(),term,CMP());
    if (iter==this->end() || iter->index()!=a) {
        return this->zero_coefficient();
    } else {
        return iter->coefficient();
    }
}

template<class I, class X, class CMP> auto SortedExpansion<I,X,CMP>::at(const I& a) -> CoefficientType& {
    ExpansionValue<I,X> term(a,this->zero_coefficient());
    auto iter=std::lower_bound(this->begin(),this->end(),term,CMP());
    if (iter==this->end() || iter->index()!=a) {
        iter=this->Expansion<I,X>::insert(iter,a,this->zero_coefficient());
    }
    return iter->coefficient();
}

template<class I, class X, class CMP> Void SortedExpansion<I,X,CMP>::insert(const I& a, const X& c) {
    ExpansionValue<I,X> term(a,this->Expansion<I,X>::zero_coefficient());
    auto iter=std::lower_bound(this->begin(),this->end(),term,CMP());
    if (iter==this->end() || iter->index()!=a) {
        iter=this->Expansion<I,X>::insert(iter,a,c);
    } else {
        ARIADNE_THROW(std::runtime_error,"Expansion<I,X>::set(const I& a, const X& c):\n    e="<<*this,
                      " Index "<<a<<" already has a coefficient.");
    }
}

template<class I, class X, class CMP> void SortedExpansion<I,X,CMP>::set(const I& a, CoefficientType const& c) {
    this->at(a)=c;
}

template<class I, class X, class CMP> void SortedExpansion<I,X,CMP>::sort() {
    std::sort(this->begin(),this->end(),CMP());
}

inline OutputStream& operator<<(OutputStream& os, GradedIndexLess cmp) { return os << "GradedIndexLess"; }
inline OutputStream& operator<<(OutputStream& os, ReverseLexicographicIndexLess cmp) { return os << "ReverseLexicographicIndexLess"; }

template<class I, class X, class CMP> void SortedExpansion<I,X,CMP>::check() const {
    CMP cmp;
    for(auto iter=this->begin(), next=iter+1; next<this->end(); ++iter, ++next) {
        if(!cmp(*iter,*next)) {
            ARIADNE_THROW(std::runtime_error,"SortedExpansion<I,X,CMP>::check():\n    e="<<*this<<" cmp="<<cmp," Expansion is not sorted.")
        }
    }
}

} // namespace Ariadne
