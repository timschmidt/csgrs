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
#include <initializer_list>
#include <iostream>

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "utility/container.hpp"
#include "utility/uniform_list.hpp"
#include "numeric/numeric.hpp"

namespace Ariadne {

class UniIndex {
    DegreeType _a;
  public:
    UniIndex(DegreeType a=0u) : _a(a) { }
    UniIndex& operator++() { ++_a; return *this; }
    UniIndex& operator--() { --_a; return *this; }
    UniIndex operator+(UniIndex const& other) const { return UniIndex(this->_a+other._a); }
    short int operator-(UniIndex const& other) const { return (short int)this->_a-(short int)other._a; }
    UniIndex& operator+=(UniIndex const& other) { this->_a+=other._a; return *this; }
    template<BuiltinIntegral D> UniIndex operator+(D d) const { return UniIndex(this->_a+d); }
    DegreeType& operator[] (IndexZero) { return _a; }
    DegreeType const& operator[] (IndexZero) const { return _a; }
    SizeOne size() const { return SizeOne(); }
    DegreeType degree() const { return _a; }
    operator DegreeType() const { return _a; }
};

struct MultiIndexData {
    friend class MultiIndexList;
  public:
    typedef DegreeType IndexType;
    typedef DegreeType& Reference;
    typedef const DegreeType& ConstReference;
  protected:
    ~MultiIndexData();
    explicit MultiIndexData(SizeType n, IndexType* p);
  public:
    SizeType size() const;
    SizeType number_of_variables() const;
    DegreeType degree() const;
    IndexType const& operator[](SizeType i) const;
    IndexType& operator[](SizeType i);

    DegreeType get(SizeType i) const;
    Void set(SizeType i, DegreeType n);

    Void assign(const MultiIndexData& a);
  public:
    friend Bool operator==(const MultiIndexData& a1, const MultiIndexData& a2);
    friend Bool operator!=(const MultiIndexData& a1, const MultiIndexData& a2);
  public:
    friend Bool graded_less(const MultiIndexData& a1, const MultiIndexData& a2);
    friend Bool lexicographic_less(const MultiIndexData& a1, const MultiIndexData& a2);
    friend Bool reverse_lexicographic_less(const MultiIndexData& a1, const MultiIndexData& a2);
    friend OutputStream& operator<<(OutputStream&, const MultiIndexData&);
  public:
    IndexType* begin();
    IndexType* end();
    const IndexType* begin() const;
    const IndexType* end() const;
  protected: public:
    SizeType _n;
    IndexType* _p;
};

class MultiIndex
    : public MultiIndexData
{
  public:
    ~MultiIndex();
    explicit MultiIndex();
    explicit MultiIndex(SizeType nv);
    explicit MultiIndex(SizeType nv, const DegreeType* ary);
    MultiIndex(InitializerList<DegreeType> lst);

    MultiIndex(const MultiIndex& a);
    MultiIndex& operator=(const MultiIndex& a);

    static MultiIndex zero(SizeType nv);
    static MultiIndex unit(SizeType nv, SizeType j);

    Void resize(SizeType n);
    Void clear();

    MultiIndex& operator++();

    MultiIndex& operator+=(const MultiIndex& a);
    MultiIndex& operator-=(const MultiIndex& a);
    MultiIndex& operator*=(const DegreeType& a);
    friend MultiIndex operator+(MultiIndex a1, const MultiIndex& a2);
    friend MultiIndex operator-(MultiIndex a1, const MultiIndex& a2);
    friend MultiIndex operator*(MultiIndex a, IndexType s);
    friend MultiIndex operator*(DegreeType s, MultiIndex a);

    friend Void swap(MultiIndex& a1, MultiIndex& a2);

//    SizeType position() const;
//    SizeType factorial() const;
//    SizeType number() const;
};

struct LexicographicLess {
    Bool operator() (DegreeType const& a1, DegreeType const& a2) const;
    Bool operator() (MultiIndex const& a1, MultiIndex const& a2) const;
};

struct ReverseLexicographicLess {
    Bool operator() (DegreeType const& a1, DegreeType const& a2) const;
    Bool operator() (MultiIndex const& a1, MultiIndex const& a2) const;
};

struct GradedLess {
    Bool operator() (DegreeType const& a1, DegreeType const& a2) const;
    Bool operator() (MultiIndex const& a1, MultiIndex const& a2) const;
};


static const SizeType DEFAULT_CAPACITY = 4u;

struct MultiIndexReference : public MultiIndexData {
    MultiIndexReference(SizeType n, IndexType* p);
    MultiIndexReference(MultiIndexData const&);
    MultiIndexReference(MultiIndexReference const&) = default;
    MultiIndexReference& operator=(MultiIndexReference const&);
    MultiIndexReference& operator=(MultiIndexData const&);
    operator MultiIndex& ();
    operator MultiIndex const& () const;
    MultiIndexReference& operator+=(MultiIndexData const&);
    friend Void swap(MultiIndex&, MultiIndex&);
//    friend Void swap(MultiIndexReference, MultiIndexReference);
};


struct MultiIndexConstReference : public MultiIndexData {
    MultiIndexConstReference(SizeType n, DegreeType const* p);
    MultiIndexConstReference(MultiIndexConstReference const&) = default;
    MultiIndexConstReference& operator=(MultiIndexConstReference const&) = delete;
    MultiIndexConstReference(MultiIndexReference const&);
    operator MultiIndex const& () const;
    friend MultiIndex operator+(MultiIndex a1, const MultiIndex& a2);
};

struct MultiIndexPointer {
    MultiIndexReference _r;
  public:
    MultiIndexPointer(MultiIndexReference* _p) : _r(*_p) { }
    MultiIndexReference& operator*() { return _r; }
    MultiIndexReference* operator->() { return &_r; }
    friend OutputStream& operator<<(OutputStream& os, MultiIndexPointer const&);
};
struct MultiIndexConstPointer {
    MultiIndexConstReference _r;
  public:
    MultiIndexConstPointer(MultiIndexConstReference* _p) : _r(*_p) { }
    MultiIndexConstReference& operator*() const { return const_cast<MultiIndexConstReference&>(_r); }
    MultiIndexConstReference* operator->() const { return const_cast<MultiIndexConstReference*>(&_r); }
    friend OutputStream& operator<<(OutputStream& os, MultiIndexConstPointer const&);
};



class MultiIndexListIterator;
class MultiIndexListConstIterator;

class MultiIndexList {
    SizeType _capacity; SizeType _size; SizeType _argument_size;
    DegreeType* _indices;
  public:
    typedef MultiIndex ValueType;
    typedef MultiIndexReference Reference;
    typedef MultiIndexConstReference ConstReference;
    typedef MultiIndexPointer Pointer;
    typedef MultiIndexConstPointer ConstPointer;
    typedef MultiIndexListIterator Iterator;
    typedef MultiIndexListConstIterator ConstIterator;
  protected: public:
    explicit MultiIndexList(SizeType as);
  public:
    ~MultiIndexList();
//    MultiIndexList(SizeType as, SizeType cap);
    MultiIndexList(InitializerList<InitializerList<DegreeType>> const& lst);
    MultiIndexList(InitializerList<MultiIndex> const& lst);
    MultiIndexList(SizeType n, MultiIndex const& a);
    MultiIndexList(MultiIndexList const&);
    MultiIndexList(MultiIndexList&&);
    MultiIndexList& operator=(MultiIndexList const&);
    MultiIndexList& operator=(MultiIndexList&&);
    Void resize(SizeType n);
    Void reserve(SizeType n);
    SizeType size() const;
    SizeType capacity() const;
    SizeType argument_size() const;
    Void append(MultiIndexData const& a);
    Void append_sum(MultiIndexData const& a1, MultiIndexData const& a2);
    Reference operator[](SizeType i);
    ConstReference operator[](SizeType i) const;
    Reference front();
    ConstReference front() const;
    Reference back();
    ConstReference back() const;
    Iterator begin();
    Iterator end();
    ConstIterator begin() const;
    ConstIterator end() const;
    Iterator erase(Iterator pos);
    Void clear();
    friend Bool operator==(MultiIndexList const& lst1, MultiIndexList const& lst2);
    friend OutputStream& operator<<(OutputStream& os, MultiIndexList const& lst);
};

class MultiIndexListIterator {
    friend class MultiIndexListConstIterator;
    MultiIndexReference _r;
  public:
    explicit MultiIndexListIterator(SizeType n, DegreeType* p);
    MultiIndexListIterator(MultiIndexListIterator const&);
    MultiIndexListIterator& operator=(MultiIndexListIterator const&);
    Bool operator==(MultiIndexListIterator const& other) const;
    Bool operator!=(MultiIndexListIterator const& other) const;
    MultiIndexListIterator& operator++();
    MultiIndexListIterator& operator--();
    MultiIndexListIterator& operator+=(PointerDifferenceType k);
    MultiIndexListIterator operator+(PointerDifferenceType k) const;
    MultiIndexReference operator*() const;
    MultiIndexPointer operator->() const;
    friend OutputStream& operator<<(OutputStream& os, MultiIndexListIterator const&);
};

class MultiIndexListConstIterator {
    MultiIndexConstReference _r;
  public:
    explicit MultiIndexListConstIterator(SizeType n, DegreeType const* p);
    MultiIndexListConstIterator(MultiIndexListIterator const& iter);
    MultiIndexListConstIterator(MultiIndexListConstIterator const& iter);
    MultiIndexListConstIterator& operator=(MultiIndexListConstIterator const& iter);
    Bool operator==(MultiIndexListConstIterator const& other);
    Bool operator!=(MultiIndexListConstIterator const& other) const;
    MultiIndexListConstIterator& operator++();
    MultiIndexListConstIterator& operator--();
    MultiIndexListConstIterator& operator+=(PointerDifferenceType k);
    MultiIndexListConstIterator operator+(PointerDifferenceType k) const;
    MultiIndexConstReference operator*() const;
    MultiIndexConstPointer operator->() const;
    friend OutputStream& operator<<(OutputStream& os, MultiIndexListConstIterator const&);
};


class MultiIndexBound {
  public:
    MultiIndexBound(SizeType n, SizeType d);
    MultiIndexBound(const MultiIndex& a);
    SizeType size() const { return _groups.size(); }
    friend Bool operator<=(const MultiIndex& a, const MultiIndexBound& b);
  private:
    Array<SizeType> _groups;
    Array<SizeType> _max_degrees;
};


template<> class UniformList<MultiIndex>
    : public MultiIndexList
{
    using MultiIndexList::MultiIndexList;
};


} // namespace Ariadne

#include "multi_index.inl.hpp"

#endif /* ARIADNE_MULTI_INDEX_HPP */
