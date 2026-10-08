
/***************************************************************************
 *            algebra/multi_index.hpp
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

/*! \brief \file algebra/multi_index.hpp
 *  \brief An index specifying the degree of differentiation.
 */

#ifndef ARIADNE_MULTI_INDEX_HPP
#define ARIADNE_MULTI_INDEX_HPP

#include <cassert>
#include <cstdarg>
#include <cstring>
#include <iostream>
#include <vector>

#include "utility/iterator.hpp"
#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "numeric/numeric.hpp"

namespace Ariadne {

uint32_t fac(uint8_t);
uint32_t bin(uint8_t,uint8_t);

typedef unsigned char uchar;
typedef double FloatDP;

class MultiIndexReference;
class MultiIndexValueReference;

class MultiIndexListIterator;

class MultiIndexBound;

template<class T> class Reference;
template<> class Reference<MultiIndexReference>;
template<> class Reference<const MultiIndexReference>;

//struct MultiIndexData { unsigned int _n; unsigned int* _p; };

//A reference to a MultiIndex
class MultiIndexReference {
    friend class MultiIndexListIterator;
    friend class MultiIndexListConstIterator;
  public:
    typedef unsigned int SizeType;
    typedef unsigned char ByteType;
    typedef unsigned int WordType;
    typedef char raw_data_type;

    typedef ByteType IndexType;
    typedef MultiIndexValueReference reference;
    typedef const IndexType& const_reference;
    //typedef unsigned int WordType;
    //typedef unsigned long long int WordType;
  protected:
  public:
    //! \brief Constructors.
    MultiIndexReference(SizeType n, IndexType* p);
  public:
    //! \brief Assign values.
    MultiIndexReference& operator=(const MultiIndexReference& a);
    //! \brief Resize to hold n variables.
    Void resize(SizeType n);
    //! \brief Assigns values from another index. Precondition: the size of \a a must equal the current size.
    Void assign(const MultiIndexReference& a);
    //! \brief Set all values to zero.
    Void clear();
    //! \brief The number of variables.
    SizeType size() const;
    //! \brief The degree of the multi-index, equal to the sum of the number of occurrences of the variables.
    IndexType degree() const;
     //! \brief The number of variables.
    SizeType number_of_variables() const;
    //! \brief The number of occurrences of the \a i th variable.
    IndexType get(SizeType i) const;
    //! \brief Set the number of occurrences of the \a i th variable to \a n.
    Void set(SizeType i, IndexType n);
    //! \brief The number of occurrences of the \a i th variable.
    const_reference operator[](SizeType i) const;
    //! \brief The number of occurrences of the \a i th variable.
    reference operator[](SizeType i);
    //! \brief Increment the value of the \a ith element
    Void increment(SizeType i);
    //! \brief Decrement the value of the \a ith element
    Void decrement(SizeType i);

    //! \brief Equality operator.
    friend Bool operator==(const MultiIndexReference& a1, const MultiIndexReference& a2); // inline
    //! \brief Inequality operator.
    friend Bool operator!=(const MultiIndexReference& a1, const MultiIndexReference& a2); // inline
    //! \brief Comparison operator.
    friend Bool operator<(const MultiIndexReference& a1, const MultiIndexReference& a2); // inline

    //! \brief Increment. No post-increment operator as we sometimes pass MultiIndexReference by reference.
    MultiIndexReference& operator++(); // inline
    // No post-increment operator as we sometimes pass MultiIndexReference by reference.Post increment.
    // MultiIndexReference operator++(Int);
    //! \brief Inplace sum.
    MultiIndexReference& operator+=(const MultiIndexReference& a); // inline
    //! \brief Inplace difference.
    MultiIndexReference& operator-=(const MultiIndexReference& a); // inline
    //! \brief Inplace scalar product.
    MultiIndexReference& operator*=(const IndexType& a); // inline
    //! \brief Sum.
    friend MultiIndexReference operator+(const MultiIndexReference& a1, const MultiIndexReference& a2); // inline
    //! \brief Difference.
    friend MultiIndexReference operator-(const MultiIndexReference& a1, const MultiIndexReference& a2); // inline
    //! \brief Scalar product.
    friend MultiIndexReference operator*(const MultiIndexReference& a, IndexType s); // inline
    //! \brief Scalar product.
    friend MultiIndexReference operator*(IndexType s, const MultiIndexReference& a); // inline

    //! \brief The position of the element in the Array of tensor values.
    unsigned int position() const;
    //! \brief The product of the factorials of the indices.
    unsigned int factorial() const;
    //! \brief The number of ordered index arrays with each element occurring the number of times specified by the multi index.
    unsigned int number() const;

    //! \brief Write to an output stream.
    friend OutputStream& operator<<(OutputStream&, const MultiIndexReference&);
  public:
    //IndexType& at(SizeType i) { return _p[i]; }
    const IndexType& at(SizeType i) const { return _p[i]; }
    IndexType& at(SizeType i) { return _p[i]; }
    IndexType* begin() { return _p; }
    const IndexType* begin() const { return _p; }
    IndexType* end() { return _p+(_n+1u); }
    const IndexType* end() const { return _p+(_n+1u); }
  public:
    //SizeType memory_size() { return sizeof(WordType)*(1+_n/sizeof(WordType)); }
    SizeType memory_size() { return _n+1; }
    Void allocate() { _p=new IndexType[memory_size()]; }
    Void deallocate() { delete[] _p; }
  private:
    SizeType _n;
    IndexType* _p;
};


class MultiIndex : public MultiIndexReference {
  public:
    //! \brief Destructor.
    ~MultiIndex();
    //! \brief Construct a multi index with no coefficients.
    explicit MultiIndex();
    //! \brief Construct a multi index of degree \a 0 with \a nv variables.
    explicit MultiIndex(SizeType nv);
    //! \brief Construct a multi index with \a nv variables from the Array \a ary.
    explicit MultiIndex(SizeType nv, const Int* ary);
    //! \brief Construct a multi index with \a nv variables from variable arguments.
    explicit MultiIndex(SizeType nv, Int a1, ...);

    //! \brief Copy constructor.
    MultiIndex(const MultiIndex& a);
    //! \brief Copy from a reference.
    MultiIndex(const MultiIndexReference& a);
    //! \brief Copy assignment operator.
    MultiIndex& operator=(const MultiIndexReference& a);

    //! \brief Construct the zero multi index with \a nv variables.
    static MultiIndex zero(SizeType nv);
    //! \brief Construct the unit multi index in variable \a j with \a nv variables.
    static MultiIndex unit(SizeType nv, SizeType j);
    //! \brief Construct the first multi index of degree \a d with \a nv variables.
    static MultiIndex first(SizeType nv, IndexType d);
};


class MultiIndexValueReference {
    typedef MultiIndexReference::SizeType SizeType;
    typedef MultiIndexReference::IndexType IndexType;
    typedef MultiIndexReference::WordType WordType;
  private:
    SizeType _n; IndexType* _p; SizeType _i;
  public:
    MultiIndexValueReference(SizeType n, IndexType* p, SizeType i) : _n(n), _p(p), _i(i) { }
    operator const IndexType& () { return _p[_i]; }
    MultiIndexValueReference& operator=(const IndexType& d) { _p[_n]+=(d-_p[_i]); _p[_i]=d; return *this; }
    MultiIndexValueReference& operator++() { ++_p[_n]; ++_p[_i]; return *this; }
    MultiIndexValueReference& operator--();
    MultiIndexValueReference& operator+=(Int k) { _p[_n]+=k; _p[_i]+=k; return *this; }
    MultiIndexValueReference& operator-=(Int k) { _p[_n]-=k; _p[_i]-=k; return *this; }
};

inline MultiIndexValueReference& MultiIndexValueReference::operator--() {
    if(_p[_i]==0) {
        ARIADNE_THROW(std::runtime_error,"--MultiIndexReference[i]"," decrementing zero value at "<<_i<<" in "<<MultiIndexReference(_n,_p)); }
    --_p[_n]; --_p[_i]; return *this;
}


inline MultiIndex::~MultiIndex()
{
    deallocate();
}

inline MultiIndex::MultiIndex()
    : MultiIndexReference(0,0)
{
    allocate();
    std::fill(begin(),end(),0);
}

inline MultiIndex::MultiIndex(SizeType n)
    : MultiIndexReference(n,0)
{
    allocate();
    std::fill(this->begin(),this->end(),0);
}

inline MultiIndex::MultiIndex(SizeType n, const Int* ary)
    : MultiIndexReference(n,0)
{
    allocate();
    IndexType* p=begin();
    p[n]=0; for(SizeType i=0; i!=n; ++i) { p[i]=ary[i]; p[n]+=ary[i]; }
}

inline MultiIndex::MultiIndex(SizeType n, Int a0, ...)
    : MultiIndexReference(n,0)
{
    ARIADNE_ASSERT(n>0);
    allocate();
    IndexType* p=begin();
    p[0]=a0; p[n]=a0;
    va_list args; va_start(args,a0);
    for(SizeType i=1; i!=n; ++i) {
        p[i]=va_arg(args,Int);
        p[n]+=p[i];
    }
    va_end(args);
}

inline MultiIndex::MultiIndex(const MultiIndex& a)
    : MultiIndexReference(a.size(),0)
{
    this->allocate();
    this->assign(a);
}

inline MultiIndex::MultiIndex(const MultiIndexReference& a)
    : MultiIndexReference(a.size(),0)
{
    this->allocate();
    this->assign(a);
}

inline MultiIndex& MultiIndex::operator=(const MultiIndexReference& a) {
    if(this!=&a) { this->resize(a.size()); this->assign(a); }
    return *this;
}

inline MultiIndex MultiIndex::zero(SizeType n)
{
    return MultiIndex(n);
}

inline MultiIndex MultiIndex::unit(SizeType n, SizeType i)
{
    MultiIndex result(n);
    ++result[i];
    return result;
}

inline MultiIndex MultiIndex::first(SizeType n, IndexType d)
{
    MultiIndex result(n);
    result[0]=d;
    return result;
}


inline MultiIndexReference::MultiIndexReference(SizeType n, IndexType* p)
    : _n(n), _p(p) { }

inline MultiIndexReference& MultiIndexReference::operator=(const MultiIndexReference& a) {
    assert(this->size()==a.size()); this->assign(a); return *this;
}

inline Void MultiIndexReference::assign(const MultiIndexReference& a) {
    memcpy(this->begin(),a.begin(),this->memory_size());
}

inline Void MultiIndexReference::clear()
{
    memset(this->begin(),this->size(),0);
    //std::fill(this->begin(),this->end(),0);
}

inline Void MultiIndexReference::resize(SizeType n) {
    if(this->_n!=n) { deallocate(); this->_n=n; allocate(); }
}

inline MultiIndexReference::SizeType MultiIndexReference::size() const {
    return this->_n;
}

inline MultiIndexReference::IndexType MultiIndexReference::degree() const {
    return this->_p[this->_n];
}

inline MultiIndexReference::SizeType MultiIndexReference::number_of_variables() const {
    return this->_n;
}

inline MultiIndexReference::IndexType MultiIndexReference::get(SizeType i) const {
    assert(i<this->size()); return this->_p[i];
}

inline Void MultiIndexReference::set(SizeType i, IndexType k) {
    assert(i<this->size());
    IndexType& ai=this->_p[i];
    IndexType& d=this->_p[this->_n];
    d+=k; d-=ai; ai=k;
}

inline MultiIndexReference::IndexType const& MultiIndexReference::operator[](SizeType i) const {
    assert(i<this->size()); return this->_p[i];
}

inline MultiIndexValueReference MultiIndexReference::operator[](SizeType i) {
    return MultiIndexValueReference(this->_n,this->_p,i);
}

inline Void MultiIndexReference::increment(SizeType i) {
    ++this->_p[i]; ++this->_p[this->_n];
}

inline Void MultiIndexReference::decrement(SizeType i) {
    ARIADNE_ASSERT(this->_p[i]>0u);
    --this->_p[i]; --this->_p[this->_n];
}


inline Bool operator==(const MultiIndexReference& a1, const MultiIndexReference& a2) {
    if(a1.size()!=a2.size()) { return false; }
    //return memcmp(a1.begin(),a2.begin(),a1.size());
    for(MultiIndexReference::SizeType i=0; i!=a1.size(); ++i) {
        if(a1.at(i)!=a2.at(i)) { return false; } }
    return true;
    //for(MultiIndexReference::SizeType j=0; j!=a1.word_size(); ++j) {
    //    if(a1.word_at(j)!=a2.word_at(j)) { return false; } }
    //return true;
}

inline Bool operator!=(const MultiIndexReference& a1, const MultiIndexReference& a2) {
    return !(a1==a2);
}

inline Bool operator<(const MultiIndexReference& a1, const MultiIndexReference& a2) {
    if(a1.size()!=a2.size()) {
        throw std::runtime_error("operator<(MultiIndexReference,MultiIndexReference): number of variables must match");
    }

    if(a1.degree()!=a2.degree()) {
        return a1.degree()<a2.degree();
    } else {
        for(MultiIndexReference::SizeType i=0; i!=a1.size(); ++i) {
            if(a1[i]!=a2[i]) {
                return a1[i]>a2[i];
            }
        }
        return false;
    }
}


inline
unsigned int MultiIndexReference::number() const
{
    unsigned int result=fac(this->degree());
    for(unsigned int k=0; k!=this->size(); ++k) {
        result/=fac((*this)[k]);
    }
    return result;
}

inline
unsigned int MultiIndexReference::factorial() const
{
    unsigned int result=1;
    for(unsigned int k=0; k!=this->size(); ++k) {
        result*=fac((*this)[k]);
    }
    return result;
}

inline
unsigned int MultiIndexReference::position() const
{
    unsigned int deg=this->degree()-1;
    unsigned int nvar=this->size();
    unsigned int result=bin(deg+nvar,nvar);
    for(unsigned int k=0; k!=this->size()-1; ++k) {
        --nvar;
        deg-=(*this)[k];
        result+=bin(deg+nvar,nvar);
    }
    return result;
}


inline
MultiIndexReference& MultiIndexReference::operator++()
{
    //std::cerr<<"MultiIndexReference::operator++() with *this="<<*this<<" "<<std::flush;
    assert(_n>0);

    SizeType const n=this->_n;
    IndexType* const p=this->_p;

    if(n==1) {
        ++p[0];
        ++p[n];
        return *this;
    }
    if(p[n-2]!=0) {
        --p[n-2];
        ++p[n-1];
        return *this;
    } else {
        IndexType li=p[n-1];
        p[n-1]=0;
        for(SizeType k=n-1; k!=0; --k) {
            if(p[k-1]!=0) {
                --p[k-1];
                p[k]=li+1;
                return *this;
            }
        }
        p[0]=li+1;
        ++p[n];
    }
    return *this;
}


inline
MultiIndexReference& MultiIndexReference::operator+=(const MultiIndexReference& a) {
    //for(SizeType j=0; j!=this->word_size(); ++j) {
    //    this->word_at(j)+=a.word_at(j);
    //}
    //return *this;
    for(SizeType i=0; i!=this->size()+1; ++i) {
        this->_p[i]+=a._p[i];
    }
    return *this;
}

inline
MultiIndexReference& MultiIndexReference::operator-=(const MultiIndexReference& a) {
    for(SizeType i=0; i!=this->size()+1; ++i) {
        ARIADNE_ASSERT(this->_p[i]>=a._p[i]);
        this->_p[i]-=a._p[i];
    }
    return *this;
}

inline
MultiIndexReference& MultiIndexReference::operator*=(const IndexType& s) {
    //for(SizeType j=0; j!=this->word_size(); ++j) {
    //    this->word_at(j)*=s;
    //}
    //return *this;
    for(SizeType i=0; i!=this->size()+1; ++i) {
        this->_p[i]*=s;
    }
    return *this;
}

inline
Void iadd(MultiIndexReference& r, const MultiIndexReference& a1, const MultiIndexReference& a2) {
    for(MultiIndexReference::SizeType j=0; j!=r.size(); ++j) {
        r.at(j)=a1.at(j)+a2.at(j);
    }
}

inline
MultiIndexReference operator+(const MultiIndexReference& a1, const MultiIndexReference& a2) {
    MultiIndexReference r(a1); r+=a2; return r;
}

inline
MultiIndexReference operator-(const MultiIndexReference& a1, const MultiIndexReference& a2) {
    MultiIndexReference r(a1); r-=a2; return r;
}

inline
MultiIndexReference operator*(const MultiIndexReference& a, MultiIndexReference::IndexType s) {
    MultiIndexReference r(a); r*=s; return r;
}

inline
MultiIndexReference operator*(MultiIndexReference::IndexType s, const MultiIndexReference& a) {
    MultiIndexReference r(a); r*=s; return r;
}


inline
unsigned int
number(const MultiIndexReference& i)
{
    unsigned int result=fac(i.degree());
    for(unsigned int k=0; k!=i.size(); ++k) {
        result/=fac(i[k]);
    }
    return result;
}

inline
unsigned int
fac(const MultiIndexReference& i)
{
    unsigned int result=1;
    for(unsigned int k=0; k!=i.size(); ++k) {
        result*=fac(i[k]);
    }
    return result;
}

inline
unsigned int
bin(const MultiIndexReference& n, const MultiIndexReference& k)
{
    assert(n.size()==k.size());
    unsigned int result=1;
    for(unsigned int i=0; i!=n.size(); ++i) {
        result*=bin(n[i],k[i]);
    }
    return result;
}

inline
OutputStream& operator<<(OutputStream& os, const MultiIndexReference& a) {
    //os<<"<"<<static_cast<const Void*>(a.begin())<<">";
    if(a.size()==0) { os << '('; }
    for(MultiIndex::SizeType i=0; i!=a.size(); ++i) { os << (i==0?'(':',') << Int(a[i]); }
    //os<<";"<<Int(a.degree());
    return os << ')';
}






//! \brief \brief A bound on a MultiIndexReference object, allowing different groups of
//!variables to have different maximum degrees.

class MultiIndexBound {
  public:
    typedef MultiIndexReference::SizeType SizeType;
    MultiIndexBound(SizeType as, SizeType d);
    MultiIndexBound(const MultiIndexReference& a);
    MultiIndexBound(SizeType as, SizeType ng, SizeType g1s, SizeType g1d, ...);
    SizeType size() const { return _groups.size(); }
    friend Bool operator<=(const MultiIndexReference& a, const MultiIndexBound& b);
  private:
    Array<SizeType> _groups;
    Array<SizeType> _max_degrees;
};

inline MultiIndexBound::MultiIndexBound(SizeType as, SizeType d)
    : _groups(as), _max_degrees(1u)
{
    for(SizeType i=0; i!=as; ++i) {
        _groups[i]=0u;
    }
    _max_degrees[0]=d;
}


inline MultiIndexBound::MultiIndexBound(const MultiIndexReference& a)
    : _groups(a.size()), _max_degrees(a.size())
{
    for(SizeType i=0; i!=a.size(); ++i) {
        _groups[i]=i;
        _max_degrees[i]=a[i];
    }
}


inline MultiIndexBound::MultiIndexBound(SizeType as, SizeType ng, SizeType g1s, SizeType g1d, ...)
    : _groups(as), _max_degrees(ng)
{
    va_list args; va_start(args,g1d);
    SizeType k=0;
    for( ; k!=g1s; ++k) {
        _groups[k]=0;
    }
    _max_degrees[0]=g1d;
    for(SizeType i=1; i!=ng; ++i) {
        SizeType nk=k+va_arg(args,Int);
        for( ; k!=nk; ++k) {
            _groups[k]=i;
        }
        _max_degrees[i]=va_arg(args,Int);
    }
    va_end(args);
    ARIADNE_ASSERT(k==as);
}

inline Bool operator<=(const MultiIndexReference& a, const MultiIndexBound& b) {
    typedef MultiIndexReference::SizeType SizeType;
    Array<SizeType> degrees(b._max_degrees.size());
    for(SizeType j=0; j!=a.size(); ++j) {
        degrees[b._groups[j]]+=a[j];
    }
    for(SizeType k=0; k!=degrees.size(); ++k) {
        if(degrees[k]>b._max_degrees[k]) {
            return false;
        }
    }
    return true;
}



class MultiIndexListIterator
    : public IteratorFacade<MultiIndexListIterator,MultiIndex,RandomAccessTraversalTag,MultiIndexReference>
{
    MultiIndexReference _ref;
  public:
    typedef MultiIndex::SizeType SizeType;
    typedef MultiIndex::IndexType IndexType;

    MultiIndexListIterator() : _ref(0u,0) { }
    MultiIndexListIterator(SizeType as, IndexType* p) : _ref(as,p) { }
    MultiIndexListIterator(const MultiIndexListIterator& iter) : _ref(iter._ref) { }
    MultiIndexListIterator& operator=(const MultiIndexListIterator& iter) { _ref._n=iter._ref._n; _ref._p=iter._ref._p; return *this; }

    Bool equal(const MultiIndexListIterator& other) const { return this->_ref._p==other._ref._p; }
    MultiIndexReference dereference() const { return _ref; }
    Void increment() { advance(1); }
    Void decrement() { advance(-1); }
    Void advance(Int m) { _ref._p+=m*(_ref._n+1); }

    friend OutputStream& operator<<(OutputStream& os, const MultiIndexListIterator& iter) {
        return os << "<" << &iter._ref[0] << ">" << iter._ref << "\n";
    }
};

class MultiIndexListConstIterator
    : public IteratorFacade<MultiIndexListConstIterator,MultiIndex,RandomAccessTraversalTag,const MultiIndexReference>
{
    MultiIndexReference _ref;
  public:
    typedef MultiIndex::SizeType SizeType;
    typedef MultiIndex::IndexType IndexType;

    MultiIndexListConstIterator() : _ref(0u,0) { }
    MultiIndexListConstIterator(SizeType as, const IndexType* p) : _ref(as,const_cast<IndexType*>(p)) { }
    MultiIndexListConstIterator(MultiIndexListIterator iter) : _ref(*iter) { }
    MultiIndexListConstIterator(const MultiIndexListConstIterator& iter) : _ref(iter._ref) { }
    MultiIndexListConstIterator& operator=(const MultiIndexListConstIterator& iter) { _ref._n=iter._ref._n; _ref._p=iter._ref._p; return *this; }
    Bool equal(const MultiIndexListConstIterator& other) const { return this->_ref._p==other._ref._p; }
    const MultiIndexReference dereference() const { return _ref; }
    Void increment() { advance(1); }
    Void decrement() { advance(-1); }
    Void advance(Int m) { _ref._p+=m*(_ref._n+1); }

    friend OutputStream& operator<<(OutputStream& os, const MultiIndexListConstIterator& iter) {
        return os << "<" << &iter._ref[0] << ">" << iter._ref << "\n";
    }
};


class MultiIndexList
{
  public:
    typedef MultiIndex::SizeType SizeType;
    typedef MultiIndex::IndexType IndexType;

    typedef MultiIndex value_type;
    typedef MultiIndexReference reference;
    typedef const MultiIndexReference const_reference;
    typedef MultiIndexListIterator Iterator;
    typedef MultiIndexListConstIterator ConstIterator;

    MultiIndexList() : _as(0u), _d() { }
    MultiIndexList(SizeType as) : _as(as), _d() { }
    Bool operator==(const MultiIndexList& other) const { return this==&other || (this->_as==other._as && this->_d==other._d); }
    Bool operator!=(const MultiIndexList& other) const { return !(*this==other); }
    SizeType element_size() const { return _as; }
    SizeType size() const { return _d.size()/(_as+1); }
    SizeType capacity() const { return _d.capacity()/(_as+1); }
    Void clear() { _d.clear(); }
    Void resize(SizeType n) { _d.resize(n*(_as+1)); }
    Void reserve(SizeType c) { _d.reserve(c*(_as+1)); };
    Void append(const MultiIndexReference& a) { this->resize(this->size()+1); (*this)[this->size()-1]=a; }

    reference operator[](SizeType i) { return MultiIndexReference(_as,_begin_pointer()+i*(_as+1)); }
    const_reference operator[](SizeType i) const { return MultiIndexReference(_as,const_cast<IndexType*>(_begin_pointer())+i*(_as+1)); }
    reference front() { return (*this)[0]; }
    const_reference front() const { return (*this)[0]; }
    reference back() { return (*this)[this->size()-1]; }
    const_reference back() const { return (*this)[this->size()-1]; }

    Iterator begin() { return Iterator(_as,_begin_pointer()); }
    Iterator end() { return Iterator(_as,_begin_pointer()+size()*(_as+1u)); }
    ConstIterator begin() const { return ConstIterator(_as,_begin_pointer()); }
    ConstIterator end() const { return ConstIterator(_as,_begin_pointer()+size()*(_as+1u)); }
  private:
  public:
    SizeType _data_size() const { return _d.size(); }
    IndexType* _begin_pointer() { return &_d[0]; }
    const IndexType* _begin_pointer() const { return &_d[0]; }
    const std::vector<IndexType>& _data() const { return _d; }
  private:
    SizeType _as; std::vector<IndexType> _d;
};

inline OutputStream& operator<<(OutputStream& os, const MultiIndexList& lst) {
    os << "MultiIndexList";
    os << "<" << lst.size() << "," << lst.capacity() <<  "," << lst.element_size() << "," << lst._data_size() << ">";
    for(MultiIndexList::SizeType i=0; i!=lst.size(); ++i) { os << (i==0?"[":",") << lst[i]; }
    return os << "]";
}

} // namespace Ariadne

#endif /* ARIADNE_MULTI_INDEX_HPP */
