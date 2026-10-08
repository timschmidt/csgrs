/***************************************************************************
 *            algebra/expansion.hpp
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

/*! \file algebra/expansion.hpp
 *  \brief
 */



#ifndef ARIADNE_EXPANSION_HPP
#define ARIADNE_EXPANSION_HPP

#include "multi_index.hpp"

#include "utility/typedefs.hpp"
#include "utility/iterator.hpp"
#include "utility/macros.hpp"

namespace Ariadne {

/************ Expansion ******************************************************/

inline Void swap(UniIndex& a1, UniIndex& a2) { std::swap(a1,a2); }
//template<class T> inline Void swap(T& t1, T& t2) { std::swap(t1,t2); }

inline DegreeType degree_of(MultiIndex const& a) { return a.degree(); }
inline DegreeType degree_of(UniIndex const& a) { return a; }

inline SizeType size_of(MultiIndex const& a) { return a.size(); }
inline SizeOne size_of(UniIndex const& a) { return SizeOne(); }

inline SizeType argument_size_of(UniformList<MultiIndex> const& as) { return as.argument_size(); }
inline SizeOne argument_size_of(UniformList<UniIndex> const& a) { return SizeOne(); }


template<class T> using UniformReference = typename UniformList<T>::Reference;
template<class T> using UniformConstReference = typename UniformList<T>::ConstReference;
template<class T> using UniformPointer = typename UniformList<T>::Pointer;
template<class T> using UniformConstPointer = typename UniformList<T>::ConstPointer;

template<class I, class X> class ExpansionValue;
template<class I, class X> class ExpansionReference;
template<class I, class X> class ExpansionConstReference;
template<class I, class X> class ExpansionPointer;
template<class I, class X> class ExpansionConstPointer;
template<class I, class X> class ExpansionIterator;
template<class I, class X> class ExpansionConstIterator;
template<class I, class X> class ExpansionValueReference;

template<class CMP> struct IndexComparison;
struct GradedIndexLess;
struct LexicographicIndexLess;
struct ReverseLexicographicIndexLess;
struct CoefficientLess;
struct CoefficientIsZero;

template<class I> struct IndexTraits;

template<> struct IndexTraits<UniIndex> {
    typedef DegreeType InitializerType;
    typedef SizeOne SizeOfType;
    typedef IndexZero IndexIntoType;
    typedef String NameType;
    template<class Y> using Argument = Scalar<Y>;
};

template<> struct IndexTraits<MultiIndex> {
    typedef InitializerList<DegreeType> InitializerType;
    typedef SizeType SizeOfType;
    typedef SizeType IndexIntoType;
    typedef Array<String> NameType;
    template<class Y> using Argument = Vector<Y>;
};

template<class I, class X> using ArgumentOfType = typename IndexTraits<I>::template Argumentq<X>;

template<class I, class X> class Expansion;

//! \brief A sparse list of index-value pairs.
template<class I, class X> class Expansion {
    using S=typename IndexTraits<I>::SizeOfType;
    using V=typename IndexTraits<I>::IndexIntoType;
  public:
    using IndexInitializerType=typename IndexTraits<I>::InitializerType;
    template<class Y> using Argument = typename IndexTraits<I>::template Argument<Y>;

//    static const SizeType DEFAULT_CAPACITY=16;
  public:
    UniformList<I> _indices;
    UniformList<X> _coefficients;
  public:
    typedef S ArgumentSizeType;
    typedef V VariableIndexType;
    typedef I IndexType;
    typedef X CoefficientType;

    typedef ExpansionValue<I,X> ValueType;
    typedef ExpansionReference<I,X> Reference;
    typedef ExpansionConstReference<I,X> ConstReference;
    typedef ExpansionPointer<I,X> Pointer;
    typedef ExpansionConstPointer<I,X> ConstPointer;
    typedef ExpansionIterator<I,X> Iterator;
    typedef ExpansionConstIterator<I,X> ConstIterator;

    typedef typename UniformList<I>::Reference IndexReference;
    typedef typename UniformList<X>::Reference CoefficientReference;
    typedef typename UniformList<I>::ConstReference IndexConstReference;
    typedef typename UniformList<X>::ConstReference CoefficientConstReference;

    ~Expansion();
    template<class... PRS> requires Constructible<X,Nat,PRS...>
        explicit Expansion(ArgumentSizeType as, PRS... prs) : Expansion(as,get_characteristics(X(0u,prs...))) { } // DEPRECATED
    explicit Expansion(ArgumentSizeType as, CharacteristicsType<X> const& prs, SizeType cap=DEFAULT_CAPACITY);
    Expansion(InitializerList<Pair<IndexInitializerType,X>> lst);
    template<class... PRS> requires Constructible<X,Nat,PRS...>
        explicit Expansion(ArgumentSizeType as, PRS... prs, SizeType cap=DEFAULT_CAPACITY);
    template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
        Expansion(InitializerList<Pair<IndexInitializerType,ExactDouble>> lst, PRS... prs);
    template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
        explicit Expansion(Expansion<I,Y> const&, PRS... prs);
    Expansion(const Expansion<I,X>&);
    Expansion<I,X>& operator=(const Expansion<I,X>&);
    Expansion(Expansion<I,X>&&);
    Expansion<I,X>& operator=(Expansion<I,X>&&);
    Void swap(Expansion<I,X>&);

    Bool same_as(const Expansion<I,X>& e) const;

    Bool empty() const;

    Pair<ArgumentSizeType,CharacteristicsType<X>> characteristics() const;
    CharacteristicsType<X> coefficient_characteristics() const;
    ArgumentSizeType argument_size() const;
    SizeType number_of_terms() const;
    SizeType number_of_nonzeros() const; // DEPRECATED
    SizeType size() const;
    SizeType capacity() const;
    Void resize(SizeType sz);
    Void reserve(SizeType cap);

    CoefficientType const& zero_coefficient() const;
    const CoefficientType& operator[](const IndexType& a) const;
    ExpansionValueReference<I,X> operator[](const IndexType& a);

    Void set(const IndexType& a, const CoefficientType& c);
    CoefficientType& at(const IndexType& a);
    const CoefficientType& get(const IndexType& a) const;

    Reference front();
    Reference back();
    ConstReference front() const;
    ConstReference back() const;

    Iterator begin();
    Iterator end();
    ConstIterator begin() const;
    ConstIterator end() const;

    Void prepend(const IndexType& a, const CoefficientType& x);
    Void append(const IndexType& a, const CoefficientType& x);
    Void append_sum(const IndexType& a1, const IndexType& a2, const CoefficientType& x);

    template<class Y> requires Assignable<X,Y> Void append(const IndexType& a, const Y& y) {
        X x=this->zero_coefficient(); x=y; this->append(a,x); }

    Iterator find(const IndexType& a);
    ConstIterator find(const IndexType& a) const;

    Iterator insert(Iterator pos, const IndexType& a, const CoefficientType& x);
    Iterator erase(Iterator pos);

    Void clear();
    Void remove_zeros();
    Void combine_terms();
    Void check() const;

    //template<class CMP> Void sort(CMP cmp);
    Void index_sort(ReverseLexicographicLess cmp);
    Void index_sort(GradedLess cmp);
    Void sort(ReverseLexicographicIndexLess cmp);
    Void sort(GradedIndexLess cmp);

    Void reverse_lexicographic_sort();
    Void graded_sort();

    Bool is_sorted(ReverseLexicographicIndexLess cmp);
    Bool is_sorted(GradedIndexLess cmp);

    OutputStream& _write(OutputStream& os) const;
    OutputStream& _write(OutputStream& os, typename IndexTraits<I>::NameType const& vars) const;
  public:
    friend OutputStream& operator<<(OutputStream& os, Expansion<I,X> const& self) { return self._write(os); }
    friend Bool same(Expansion<I,X> const& e1, Expansion<I,X> const& e2) { return e1.same_as(e2); }
    friend Expansion<MultiIndex,CoefficientType> embed(SizeType as1, Expansion<IndexType,CoefficientType> const& e2, SizeType as3) { return _embed(as1,e2,as3); }
  private:
    static Expansion<MultiIndex,CoefficientType> _embed(SizeType as1, Expansion<IndexType,CoefficientType> const& e2, SizeType as3);
    template<class Y, class... PRS> Void _fill(Expansion<I,Y> const& other, PRS... prs);
};

template<class I, class X, class CMP> class SortedExpansion : public Expansion<I,X> {
public:
    typedef I IndexType;
    typedef X CoefficientType;
    typedef typename Expansion<I,X>::Iterator Iterator;
    typedef typename Expansion<I,X>::ConstIterator ConstIterator;
  public:
    using Expansion<I,X>::Expansion;
    SortedExpansion(Expansion<I,X> e);
    Void sort();
    Void insert(const IndexType& a, const CoefficientType& c);
    Void set(const IndexType& a, const CoefficientType& c);
    CoefficientType& at(const IndexType& a);
    CoefficientType const& get(const IndexType& a) const;
//    Iterator find(const IndexType& a);
//    ConstIterator find(const IndexType& a) const;
    Void check() const; // Check the expansion is sorted and has unique terms

    EqualityType<X> operator==(const SortedExpansion<I,X,CMP>& other) const;
    EqualityType<X> operator!=(const SortedExpansion<I,X,CMP>& other) const;

};


template<class I, class X> template<class... PRS> requires Constructible<X,Nat,PRS...>
Expansion<I,X>::Expansion(ArgumentSizeType as, PRS... prs, SizeType cap)
    : Expansion(as,X(0u,prs...),cap)
{
}

template<class I, class X> template<class... PRS> requires Constructible<X,ExactDouble,PRS...>
Expansion<I,X>::Expansion(InitializerList<Pair<IndexInitializerType,ExactDouble>> lst, PRS... prs)
    : Expansion(ArgumentSizeType(),make_characteristics<X>(prs...))
{
    ARIADNE_PRECONDITION(lst.size()!=0);

    _indices = UniformList<I>(0u,I(lst.begin()->first)*0);
    _coefficients = UniformList<X>(prs...);

    SizeType cap = std::max(DEFAULT_CAPACITY,lst.size());
    _indices.reserve(cap);
    _coefficients.reserve(cap);

    for(auto iter=lst.begin();
        iter!=lst.end(); ++iter)
    {
        I a=iter->first;
        X x(iter->second,prs...);
        if(decide(x!=0)) { this->append(a,x); }
    }
}


template<class I, class X> template<class Y, class... PRS> requires Constructible<X,Y,PRS...>
Expansion<I,X>::Expansion(Expansion<I,Y> const& other, PRS... prs)
    : Expansion(other.argument_size(),get_characteristics(X(other.zero_coefficient(),prs...)))
{
    this->_fill(other,prs...);
}


} // namespace Ariadne

#include "expansion.inl.hpp"

#endif
