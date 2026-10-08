/***************************************************************************
 *            multi_index.inl.hpp
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

#include "utility/iterator.hpp"

namespace Ariadne {


inline MultiIndexData::~MultiIndexData() { }
inline MultiIndexData::MultiIndexData(SizeType n, IndexType* p) : _n(n), _p(p) { }

inline SizeType MultiIndexData::size() const { return _n; }
inline SizeType MultiIndexData::number_of_variables() const { return _n; }
inline DegreeType MultiIndexData::degree() const { DegreeType d=0u; for(SizeType i=0; i!=_n; ++i) { d+=_p[i]; } return d; }
inline DegreeType const& MultiIndexData::operator[](SizeType i) const { assert(i<_n); return _p[i]; }
inline DegreeType& MultiIndexData::operator[](SizeType i) { assert(i<_n); return _p[i]; }

inline DegreeType MultiIndexData::get(SizeType i) const { assert(i<_n); return _p[i]; }
inline Void MultiIndexData::set(SizeType i, DegreeType n) { assert(i<_n); _p[i]=n; }

inline Void MultiIndexData::assign(const MultiIndexData& a) {
    assert(_n==a._n); for(SizeType i=0; i!=_n; ++i) { _p[i]=a._p[i]; } }

inline DegreeType* MultiIndexData::begin() { return _p; }
inline DegreeType* MultiIndexData::end() { return _p+_n; }
inline const DegreeType* MultiIndexData::begin() const { return _p; }
inline const DegreeType* MultiIndexData::end() const { return _p+_n; }

inline Bool operator==(const MultiIndexData& a1, const MultiIndexData& a2) {
    if(a1._n!=a2._n) { return false; } for(SizeType i=0; i!=a1._n; ++i) { if(a1._p[i]!=a2._p[i]) { return false; } } return true; }
inline Bool operator!=(const MultiIndexData& a1, const MultiIndexData& a2) {
    return !(a1==a2); }

inline Bool graded_less(const MultiIndexData& a1, const MultiIndexData& a2) {
    assert(a1.size()==a2.size()); DegreeType d1=a1.degree(); DegreeType d2=a2.degree(); if(d1!=d2) { return d1<d2; }
    for(SizeType i=0; i!=a1.size(); ++i) { if(a1[i]!=a2[i]) { return a1[i]>a2[i]; } } return false; }
inline Bool lexicographic_less(const MultiIndexData& a1, const MultiIndexData& a2) {
    assert(a1.size()==a2.size()); for(SizeType i=0; i!=a1.size(); ++i) { if(a1[i]!=a2[i]) { return a1[i]<a2[i]; } } return false; }
inline Bool reverse_lexicographic_less(const MultiIndexData& a1, const MultiIndexData& a2) {
    assert(a1.size()==a2.size()); SizeType i=a1.size(); while(i!=0) { --i; if(a1[i]!=a2[i]) { return a1[i]>a2[i]; } } return false; }

inline OutputStream& operator<<(OutputStream& os, const MultiIndexData& a) {
    os << "("; for(SizeType i=0; i!=a.size(); ++i) { if(i!=0) { os << ","; } os << a[i]; } return os << ";" << a.degree() << ")"; }

inline MultiIndex::~MultiIndex() { delete[] _p; _p=nullptr; }
inline MultiIndex::MultiIndex() : MultiIndex(0u) { }
inline MultiIndex::MultiIndex(SizeType nv) : MultiIndexData(nv,new DegreeType[std::max(nv,1lu)]) { for(SizeType i=0; i!=_n; ++i) { _p[i]=0u; } }
inline MultiIndex::MultiIndex(SizeType nv, const DegreeType* ary) : MultiIndex(nv) {
    for(SizeType i=0; i!=_n; ++i) { _p[i]=ary[i]; } }
inline MultiIndex::MultiIndex(InitializerList<DegreeType> lst) : MultiIndex(lst.size()) {
    DegreeType* p=_p; auto q=lst.begin(); while(q!=lst.end()) { *p=*q; ++p; ++q; } }

inline MultiIndex::MultiIndex(const MultiIndex& a)
    : MultiIndex(a.size(),a.begin()) { }
inline MultiIndex& MultiIndex::operator=(const MultiIndex& a) {
    if(this!=&a) { this->resize(a.size()); for(SizeType i=0; i!=_n; ++i) { _p[i]=a._p[i]; } } return *this; }

inline MultiIndex MultiIndex::zero(SizeType nv) { MultiIndex a(nv); a.clear(); return a; }
inline MultiIndex MultiIndex::unit(SizeType nv, SizeType j) { MultiIndex a=MultiIndex::zero(nv); a[j]=1u; return a; }

inline Void MultiIndex::resize(SizeType n) { if(_n!=n) { _n=n; delete[] _p; _p=new DegreeType[std::max(n,1lu)]; } }
inline Void MultiIndex::clear() { for(SizeType i=0; i!=_n; ++i) { _p[i]=0u; } }

inline MultiIndex& MultiIndex::operator+=(const MultiIndex& a) { assert(_n==a._n); for(SizeType i=0; i!=_n; ++i) { _p[i]+=a._p[i]; } return *this; }
inline MultiIndex& MultiIndex::operator-=(const MultiIndex& a) { assert(_n==a._n); for(SizeType i=0; i!=_n; ++i) { assert(_p[i]>=a._p[i]); _p[i]-=a._p[i]; } return *this; }
inline MultiIndex& MultiIndex::operator*=(const DegreeType& s) { for(SizeType i=0; i!=_n; ++i) { _p[i]*=s; } return *this; }

inline MultiIndex operator+(MultiIndex a1, const MultiIndex& a2) { a1+=a2; return a1; }
inline MultiIndex operator-(MultiIndex a1, const MultiIndex& a2) { a1-=a2; return a1; }
inline MultiIndex operator*(MultiIndex a, DegreeType s) { a*=s; return a; }
inline MultiIndex operator*(DegreeType s, MultiIndex a) { a*=s; return a; }

inline Void swap(MultiIndex& a1, MultiIndex& a2) { assert(a1._n==a2._n); for(SizeType i=0; i!=a1._n; ++i) { std::swap(a1._p[i],a2._p[i]); } }



inline Bool GradedLess::operator() (DegreeType const& a1, DegreeType const& a2) const { return a1<a2; }
inline Bool LexicographicLess::operator()(const DegreeType& a1, const DegreeType& a2) const { return a1<a2; }
inline Bool ReverseLexicographicLess::operator()(const DegreeType& a1, const DegreeType& a2) const { return a1>a2; }

inline Bool GradedLess::operator() (MultiIndex const& a1, MultiIndex const& a2) const { return graded_less(a1,a2); }
inline Bool LexicographicLess::operator()(const MultiIndex& a1, const MultiIndex& a2) const { return lexicographic_less(a1,a2); }
inline Bool ReverseLexicographicLess::operator()(const MultiIndex& a1, const MultiIndex& a2) const { return reverse_lexicographic_less(a1,a2); }

inline MultiIndexReference::MultiIndexReference(SizeType n, IndexType* p) : MultiIndexData(n,p) { }
inline MultiIndexReference::MultiIndexReference(MultiIndexData const& a) : MultiIndexData(a) { }
inline MultiIndexReference& MultiIndexReference::operator=(MultiIndexData const& a) { assert(_n==a._n); for(SizeType i=0; i!=_n; ++i) { _p[i]=a._p[i]; } return *this; }
inline MultiIndexReference& MultiIndexReference::operator=(MultiIndexReference const& a) { return this->operator=(static_cast<MultiIndexData const&>(a)); }
inline MultiIndexReference::operator MultiIndex& () { return static_cast<MultiIndex&>(static_cast<MultiIndexData&>(*this)); }
inline MultiIndexReference::operator MultiIndex const& () const { return static_cast<MultiIndex const&>(static_cast<MultiIndexData const&>(*this)); }

inline MultiIndexReference& MultiIndexReference::operator+=(MultiIndexData const& a) {
    assert(_n==a._n); for(SizeType i=0; i!=_n; ++i) { _p[i]+=a._p[i]; } return *this; }

inline Void swap(MultiIndexReference a1, MultiIndexReference a2) {
    assert(a1.size()==a2.size()); for(SizeType i=0; i!=a1.size(); ++i) { std::swap(a1._p[i],a2._p[i]); } }

inline MultiIndexConstReference::MultiIndexConstReference(SizeType n, IndexType const* p) : MultiIndexData(n,const_cast<IndexType*>(p)) { }
inline MultiIndexConstReference::MultiIndexConstReference(MultiIndexReference const& r) : MultiIndexData(r._n,const_cast<IndexType*>(r._p)) { }
inline MultiIndexConstReference::operator MultiIndex const& () const { return static_cast<MultiIndex const&>(static_cast<MultiIndexData const&>(*this)); }

inline OutputStream& operator<<(OutputStream& os, MultiIndexPointer const& p) { return os << p._r._p; }
inline OutputStream& operator<<(OutputStream& os, MultiIndexConstPointer const& p) { return os << p._r._p; }




inline MultiIndexListIterator::MultiIndexListIterator(SizeType n, DegreeType* p) : _r(n,p) { }
inline MultiIndexListIterator::MultiIndexListIterator(MultiIndexListIterator const& other) : _r(other._r) { }
inline MultiIndexListIterator& MultiIndexListIterator::operator=(MultiIndexListIterator const& other) { static_cast<MultiIndexData&>(_r)=other._r; return *this; }
inline Bool MultiIndexListIterator::operator==(MultiIndexListIterator const& other) const { return this->_r._p==other._r._p; }
inline Bool MultiIndexListIterator::operator!=(MultiIndexListIterator const& other) const { return this->_r._p!=other._r._p; }
inline MultiIndexListIterator& MultiIndexListIterator::operator++() { return (*this)+=1; }
inline MultiIndexListIterator& MultiIndexListIterator::operator--() { return (*this)+=(-1); }
inline MultiIndexListIterator& MultiIndexListIterator::operator+=(PointerDifferenceType k) { _r._p+=k*static_cast<PointerDifferenceType>(_r._n); return *this; }
inline MultiIndexListIterator MultiIndexListIterator::operator+(PointerDifferenceType k) const { return MultiIndexListIterator(_r._n,_r._p+static_cast<PointerDifferenceType>(_r._n)*k); }
inline MultiIndexReference MultiIndexListIterator::operator*() const { return _r; }
inline MultiIndexPointer MultiIndexListIterator::operator->() const { return MultiIndexPointer(const_cast<MultiIndexReference*>(&_r)); }

inline MultiIndexListConstIterator::MultiIndexListConstIterator(SizeType n, DegreeType const* p) : _r(n,p) { }
inline MultiIndexListConstIterator::MultiIndexListConstIterator(MultiIndexListIterator const& other) : _r(other._r._n,other._r._p) { }
inline MultiIndexListConstIterator::MultiIndexListConstIterator(MultiIndexListConstIterator const& other) : _r(other._r._n,other._r._p) { }
inline MultiIndexListConstIterator& MultiIndexListConstIterator::operator=(MultiIndexListConstIterator const& other) { static_cast<MultiIndexData&>(_r)=other._r; return *this; }
inline Bool MultiIndexListConstIterator::operator==(MultiIndexListConstIterator const& other) { return this->_r._p==other._r._p; }
inline Bool MultiIndexListConstIterator::operator!=(MultiIndexListConstIterator const& other) const { return this->_r._p!=other._r._p; }
inline MultiIndexListConstIterator& MultiIndexListConstIterator::operator++() { return (*this)+=1; }
inline MultiIndexListConstIterator& MultiIndexListConstIterator::operator--() { return (*this)+=(-1); }
inline MultiIndexListConstIterator& MultiIndexListConstIterator::operator+=(PointerDifferenceType k) { _r._p+=static_cast<PointerDifferenceType>(_r._n)*k; return *this; }
inline MultiIndexListConstIterator MultiIndexListConstIterator::operator+(PointerDifferenceType k) const { return MultiIndexListConstIterator(_r._n,_r._p+static_cast<PointerDifferenceType>(_r._n)*k); }
inline MultiIndexConstReference MultiIndexListConstIterator::operator*() const { return _r; }
inline MultiIndexConstPointer MultiIndexListConstIterator::operator->() const { return MultiIndexConstPointer(const_cast<MultiIndexConstReference*>(&_r)); }



inline MultiIndexList::MultiIndexList(MultiIndexList&& lst)
    : _capacity(lst._capacity), _size(lst._size), _argument_size(lst._argument_size), _indices(lst._indices)
{
    lst._indices=nullptr;
}

inline MultiIndexList& MultiIndexList::operator=(MultiIndexList&& lst) {
    if(this != &lst) {
        delete[] _indices;

        _capacity=lst._capacity;
        _size=lst._size;
        _argument_size=lst._argument_size;

        _indices=lst._indices;
        lst._indices = nullptr;
    }
    return *this;
}


inline SizeType MultiIndexList::size() const {
    return this->_size; }

inline SizeType MultiIndexList::capacity() const {
    return this->_capacity; }

inline SizeType MultiIndexList::argument_size() const {
    return this->_argument_size; }

inline Void MultiIndexList::append(MultiIndexData const& a) {
    assert(this->argument_size()==a.size());
    if (_size==_capacity) { this->reserve(std::max(2*_capacity,DEFAULT_CAPACITY)); }
    MultiIndexReference(this->_argument_size,this->_indices+this->_argument_size*this->_size)=static_cast<MultiIndex const&>(a);
    ++_size;
}

inline Void MultiIndexList::append_sum(MultiIndexData const& a1, MultiIndexData const& a2) {
    this->append(static_cast<MultiIndex const&>(a1)+static_cast<MultiIndex const&>(a2));
}

inline MultiIndexList::Reference MultiIndexList::operator[](SizeType i) {
    assert(i<_size); return Reference(_argument_size,_indices+i*_argument_size); }
inline MultiIndexList::ConstReference MultiIndexList::operator[](SizeType i) const {
    assert(i<_size); return ConstReference(_argument_size,_indices+i*_argument_size); }

inline MultiIndexList::Reference MultiIndexList::front() {
    return this->operator[](0u); }
inline MultiIndexList::ConstReference MultiIndexList::front() const {
    return this->operator[](0u); }
inline MultiIndexList::Reference MultiIndexList::back() {
    return this->operator[](_size-1u); }
inline MultiIndexList::ConstReference MultiIndexList::back() const {
    return this->operator[](_size-1u); }

inline MultiIndexList::Iterator MultiIndexList::begin() {
    return Iterator(_argument_size,_indices); }
inline MultiIndexList::Iterator MultiIndexList::end() {
    return Iterator(_argument_size,_indices+_size*_argument_size); }
inline MultiIndexList::ConstIterator MultiIndexList::begin() const {
    return ConstIterator(_argument_size,_indices); }
inline MultiIndexList::ConstIterator MultiIndexList::end() const {
    return ConstIterator(_argument_size,_indices+_size*_argument_size); }


} // namespace Ariadne
