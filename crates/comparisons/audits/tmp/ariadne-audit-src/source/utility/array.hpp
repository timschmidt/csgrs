/***************************************************************************
 *            utility/array.hpp
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

/*! \file utility/array.hpp
 *  \brief
 */



#ifndef ARIADNE_ARRAY_HPP
#define ARIADNE_ARRAY_HPP

#include <initializer_list>
#include <iterator>
#include <stdexcept>
#include <cassert>
#include "metaprogramming.hpp"

namespace Ariadne {

using SizeType=std::size_t;
template<class T> using InitializerList=std::initializer_list<T>;
class ExactDouble;

struct Uninitialised { };

template<class T>
class Array {
  private:
    static T* uninitialized_new(SizeType n) { return static_cast<T*>(::operator new(n*sizeof(T))); }
    static void uninitialized_delete(T* p) { ::operator delete(p); }
  public:
    typedef T ValueType;
    typedef SizeType IndexType;
    typedef ValueType* Iterator;
    typedef ValueType const* ConstIterator;

  public:
    // Standard typedefs
    typedef T value_type;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;
    typedef pointer iterator;
    typedef const_pointer const_iterator;
    typedef SizeType size_type;
    typedef std::ptrdiff_t difference_type;
  public:

    //! \brief Destructor
    ~Array() { this->_destroy_elements(); uninitialized_delete(_ptr); }

    //! \brief Default constructor. Constructs an empty Array.
    Array() : _size(0), _ptr(0) { }
    //! \brief Constructs an Array of size \a n with default-initialised elements.
    explicit Array(const SizeType n) : _size(n), _ptr(uninitialized_new(n)) { for(SizeType i=0; i!=n; ++i) { new (_ptr+i) T(); } }
    //! \brief Constructs an Array of size \a n with uninitialised elements. The elements should be initialised using placement new.
    explicit Array(const SizeType n, Uninitialised) : _size(n), _ptr(uninitialized_new(n)) { }
    //! \brief Constructs an Array of size \a n with elements initialised to \a x.
    Array(const SizeType n, const ValueType& x) : _size(n), _ptr(uninitialized_new(n)) { this->_uninitialized_fill(x); }

    //! \brief Converts an initializer list to an Array.
    Array(InitializerList<T> lst) : _size(lst.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(lst.begin()); }
    //! \brief Constructs an Array from an initializer list of doubles and a precision parameter.
    template<class PR> requires Constructible<T,double,PR>
    Array(InitializerList<double> lst, PR pr) : _size(lst.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(lst.begin(),pr); }
    template<class PR> requires Constructible<T,ExactDouble,PR>
    Array(InitializerList<ExactDouble> lst, PR pr) : _size(lst.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(lst.begin(),pr); }
    //! \brief Generate from a function (object) \a g of type \a G mapping an index to a value.
    template<class G> requires InvocableReturning<ValueType,G,SizeType>
    Array(SizeType n, G const& g) : _size(n), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_generate(g); }

    //! \brief Constructs an Array from the range \a first to \a last.
    template<class ForwardIterator>
    Array(ForwardIterator first, ForwardIterator last)
            : _size(static_cast<SizeType>(std::distance(first,last))), _ptr(uninitialized_new(_size)) {
        assert(std::distance(first,last) >= 0);
        this->_uninitialized_fill(first); }

    //! \brief Conversion constructor.
    template<class TT> requires Convertible<TT,T>
    Array(const Array<TT>& a) : _size(a.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(a.begin()); }

    //! \brief Explicit conversion constructor.
    //! \details Requirement ExplicitlyConvertible<TT,T> as a static_assert to avoid definition of \c TT being required.
    template<class TT>
    explicit Array(const Array<TT>& a) : _size(a.size()), _ptr(uninitialized_new(_size)) {
        static_assert(ExplicitlyConvertible<TT,T>); this->_uninitialized_fill(a.begin()); }

    //! \brief Explicit construction with characteristics (properties) parameter.
    template<class TT, class PR> requires Constructible<T,TT,PR>
    Array(const Array<TT>& a, PR pr) : _size(a.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(a.begin(),pr); }

    //! \brief Copy constructor.
    Array(const Array<T>& a) : _size(a.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(a.begin()); }
    //! \brief Move constructor.
    Array(Array<T>&& a) : _size(a._size), _ptr(a._ptr) {
        a._size=0u; a._ptr=nullptr; }
    //! \brief Copy assignment.
    Array<T>& operator=(const Array<T>& a) {
        if(this->size()==a.size()) { fill(a.begin()); }
        else { this->_destroy_elements(); uninitialized_delete(_ptr); _size=a.size(); _ptr=uninitialized_new(_size); this->_uninitialized_fill(a.begin()); }
        return *this; }
    //! \brief Move assignment.
    Array<T>& operator=(Array<T>&& a) {
        if(this!=&a) { this->_size=a._size; this->_ptr=a._ptr; a._size=0u; a._ptr=nullptr; } return *this; }

    //! \brief True if the Array's size is 0.
    bool empty() const { return _size==0u; }
    //! \brief The size of the Array.
    SizeType size() const { return _size; }
    //! \brief The maximum possible size of the Array.
    SizeType max_size() const { return (SizeType) (-1); }
    //! \brief Resizes the Array to hold \a n elements. If \a n is larger than the current size, the extra elements are default initialised.
    void resize(SizeType n) {
        static_assert(DefaultConstructible<T>);
        if(size()!=n) {
            pointer _new_ptr=uninitialized_new(n);
            for(SizeType i=0; i!=n; ++i) { if(i<_size) { new (_new_ptr+i) T(_ptr[i]); } else { new (_new_ptr+i) T(); } }
            this->_destroy_elements(); uninitialized_delete(_ptr); _size=n; _ptr=_new_ptr; } }
    //! \brief Resizes the Array to hold \a n elements. If \a n is larger than the current size, the extra elements are initialised with value \a t.
    void resize(SizeType n, const T& t) {
        if(size()!=n) {
            pointer _new_ptr=uninitialized_new(n);
            for(SizeType i=0; i!=n; ++i) { if(i<_size) { new (_new_ptr+i) T(_ptr[i]); } else { new (_new_ptr+i) T(t); } }
            this->_destroy_elements(); uninitialized_delete(_ptr); _size=n; _ptr=_new_ptr; } }
    //! \brief Reallocates the Array to hold \a n elements. The new elements are default-constructed.
    void reallocate(SizeType n) { if(size()!=n) { this->_destroy_elements(); uninitialized_delete(_ptr);
        _size=n; _ptr=uninitialized_new(_size); for(SizeType i=0; i!=_size; ++i) { new (_ptr+i) T(); } } }
    //! \brief Efficiently swap two arrays.
    void swap(Array<T>& a) { std::swap(_size,a._size); std::swap(_ptr,a._ptr); }

    //! \brief The \a n th element.
    ValueType& operator[](SizeType i) { return _ptr[i]; }
    //! \brief The \a n th element.
    const ValueType& operator[](SizeType i) const { return _ptr[i]; }
    //! \brief Checked access to the \a n th element.
    ValueType& at(SizeType i) { if(i<_size) { return _ptr[i]; } else { throw std::out_of_range("Array: index out-of-range"); } }
    //! \brief Checked access to the \a n th element.
    const ValueType& at(SizeType i) const { if(i<_size) { return _ptr[i]; } else { throw std::out_of_range("Array: index out-of-range"); } }

    //! \brief A reference to the first element of the Array.
    ValueType& front() { return _ptr[0]; }
    //! \brief A constant reference to the first element of the Array.
    const ValueType& front() const { return _ptr[0]; }
    //! \brief A reference to the last element of the Array.
    ValueType& back() { return _ptr[_size-1]; }
    //! \brief A constant reference  to the last element of the Array.
    const ValueType& back() const { return _ptr[_size-1]; }

    //! \brief An Iterator pointing to the beginning of the Array.
    Iterator begin() { return _ptr; }
    //! \brief A constant Iterator pointing to the beginning of the Array.
    ConstIterator begin() const { return _ptr; }
    //! \brief An Iterator pointing to the end of the Array.
    Iterator end() { return _ptr+_size; }
    //! \brief A constant Iterator pointing to the end of the Array.
    ConstIterator end() const { return _ptr+_size; }

    //! \brief Tests two arrays for equality
    bool operator==(const Array& other) const {
        if(size()!=other.size()) return false;
        T const* first=begin(); T const* last=end(); T const* curr=other.begin();
        while(first!=last) { if((*first)!=(*curr)) { return false; } ++first; ++curr; } return true; }
    //! \brief Tests two arrays for inequality
    bool operator!=(const Array& other) const { return !((*this)==other); }

    //! \brief Fills the Array with copies of \a x.
    void fill(const ValueType& x) {
        ValueType* curr=begin(); ValueType* end=this->end(); while(curr!=end) { *curr=x; ++curr; } }
    //! \brief Fills the Array from the sequence starting at \a first.
    template<class InputIterator> void fill(InputIterator first) {
        ValueType* curr=begin(); ValueType* end=this->end(); while(curr!=end) { *curr=*first; ++curr; ++first; } }
    //! \brief Assigns the sequence from \a first to \a last.
    template<class ForwardIterator> void assign(ForwardIterator first, ForwardIterator last) {
        resize(std::distance(first,last)); fill(first); }
  private:
    void _destroy_elements() { pointer curr=_ptr+_size; while(curr!=_ptr) { --curr; curr->~T(); } }
    void _uninitialized_fill(const ValueType& x) {
        pointer curr=_ptr; pointer end=_ptr+_size; while(curr!=end) { new (curr) T(x); ++curr; } }
    template<class InputIterator> void _uninitialized_fill(InputIterator first) {
        pointer curr=_ptr; pointer end=_ptr+_size;
        while(curr!=end) { new (curr) T(*first); ++curr; ++first; } }
    template<class InputIterator, class Parameters> void _uninitialized_fill(InputIterator first, Parameters parameters) {
        pointer curr=_ptr; pointer end=_ptr+_size;
        while(curr!=end) { new (curr) T(*first,parameters); ++curr; ++first; } }
    template<class G> void _uninitialized_generate(G g) {
        for(SizeType i=0u; i!=this->size(); ++i) { new (_ptr+i) T(g(i)); } }
  private:
    SizeType _size;
    pointer _ptr;
};

template<class T> class SharedArray {
  public:
    typedef T ValueType;
    typedef SizeType IndexType;
    typedef ValueType* Iterator;
    typedef ValueType const* ConstIterator;
    SizeType* _count; SizeType _size; T* _ptr;
  public:
    ~SharedArray() { --_count; if(*_count==0) { delete[] _ptr; } }
    SharedArray(SizeType n) : _count(new SizeType(1u)), _size(n), _ptr(new T[n]) { }
    SharedArray(SizeType n, const T& x) : SharedArray(n) {
        for(SizeType i=0; i!=n; ++i) { _ptr[i]=x; } }
    SharedArray(const InitializerList<T>& lst) : SharedArray(lst.size()) {
        auto from=lst.begin(); auto end=lst.end(); auto to=_ptr; while(from!=end) { *to = *from; ++from; ++to; } }
    SharedArray(const SharedArray<T>& ary) : _count(ary._count), _size(ary._size), _ptr(ary._ptr) { ++ *_count; }
    SharedArray<T>& operator=(const SharedArray<T>& ary) {
        if(this->_ptr!=ary._ptr) { --*_count; if(*_count==0) { delete _count; delete[] _ptr; } { _count=ary._count; _size=ary._size; _ptr=ary._ptr; } ++ *_count; } return *this; }
    SizeType size() const { return _size; }
    T& operator[](SizeType i) { return _ptr[i]; }
    const T& operator[](SizeType i) const { return _ptr[i]; }
    Iterator begin() { return _ptr; }
    ConstIterator begin() const { return _ptr; }
    Iterator end() { return _ptr+_size; }
    ConstIterator end() const { return _ptr+_size; }
};

inline Array<SizeType> complement(SizeType nmax, Array<SizeType> vars) {
    Array<SizeType> cmpl(nmax-vars.size());
    SizeType kr=0; SizeType kv=0;
    for(SizeType j=0; j!=nmax; ++j) {
        if(kv==vars.size() || j!=vars[kv]) {
            cmpl[kr]=j; ++kr;
        } else {
            ++kv;
        }
    }
    return cmpl;
}

} // namespace Ariadne

#endif
