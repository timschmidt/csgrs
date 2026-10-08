/***************************************************************************
 *            utility/uniform_array.hpp
 *
 *  Copyright  2013-25  Pieter Collins
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



#ifndef ARIADNE_UNIFORM_ARRAY_HPP
#define ARIADNE_UNIFORM_ARRAY_HPP

#include <initializer_list>
#include <iterator>
#include <stdexcept>
#include <cassert>
#include <utility>

#include "metaprogramming.hpp"
#include "macros.hpp"
#include "array.hpp"

namespace Ariadne {

using SizeType=std::size_t;
template<class T> using InitializerList=std::initializer_list<T>;
template<class... TS> using Tuple=std::tuple<TS...>;

template<class T1, class T2> concept EqualityComparible = requires(T1 const& t1, T2 const& t2) {
    { t1 == t2 } -> Same<Bool>;
};

struct Uninitialised;

template<class T, class PR=CharacteristicsType<T>> class UniformArray;
template<class T> requires (not HasCharacteristicsType<T>) and (not HasCharacteristics<T>) struct CharacteristicsTrait<T> { typedef std::tuple<> Type; };
template<class T> requires HasMemberCharacteristics<T> decltype(auto) characteristics(T const& t) { return t.characteristics(); }

template<class T> class CheckedAssignable : public T {
  public:
    using T::operator=;
    CheckedAssignable<T> const& operator=(T const& t) {
        assert(get_characteristics(*this)==get_characteristics(t));
        this->T::operator=(t);
        return *this;
    }
};

template<class T> struct ElementReferenceTrait { typedef CheckedAssignable<T>& Type; };
template<class T> requires BuiltinIntegral<T> struct ElementReferenceTrait<T> { typedef T& Type; };
template<class T> using ElementReference = typename ElementReferenceTrait<T>::Type;


// FIXME: Workaround since Apple Clang 14 fails when using std::make_from_tuple directly.
template<class T, class TUP> T make_from_a_tuple(TUP const& tup) { return std::make_from_tuple<T>(tup); }

template<class T, class TUP> concept MakeableFromATuple = requires (TUP const& tup) {
    { make_from_a_tuple<T>(tup) };
};

template<class T, class... PRS> requires Constructible<CharacteristicsType<T>,PRS...> CharacteristicsType<T> make_characteristics(PRS... prs) {
    return CharacteristicsType<T>(prs...); }
template<class T, class... PRS> requires MakeableFromATuple<CharacteristicsType<T>,Tuple<PRS...>> CharacteristicsType<T> make_characteristics(Tuple<PRS...> const& tup) {
    return make_from_a_tuple<CharacteristicsType<T>>(tup); }

template<class T, class... PRS> requires Constructible<T,PRS...> T make_from_characteristics(PRS... prs) {
    return T(prs...); }
template<class T, class... PRS> requires MakeableFromATuple<T,Tuple<PRS...>> T make_from_characteristics(Tuple<PRS...> const& tup) {
    return make_from_a_tuple<T>(tup); }

template<class T, class... PRS> requires Constructible<T,Int,PRS...> T make_zero_from_characteristics(PRS... prs) {
    return T(0,prs...); }
template<class T, class... PRS> requires MakeableFromATuple<T,Tuple<PRS...>> T make_zero_from_characteristics(Tuple<PRS...> const& tup) {
    return nul(make_from_a_tuple<T>(tup)); }

template<class T> requires HasMemberCharacteristics<T>
decltype(auto) get_characteristics(T const& t) {
    return t.characteristics(); }
template<class T> requires (not HasMemberCharacteristics<T>) and HasNonMemberCharacteristics<T>
decltype(auto) get_characteristics(T const& t) {
    return characteristics(t); }
template<class T> requires (not HasCharacteristics<T>) and DefaultConstructible<T>
decltype(auto) get_characteristics(T const& t) {
    return Tuple<>(); }

template<class T, class PR>
class UniformArray
    : private PR
{
  private:
    using _pr=PR;
    static T* uninitialized_new(SizeType n) { return static_cast<T*>(::operator new(n*sizeof(T))); }
    static void uninitialized_delete(T* p) { ::operator delete(p); }
  public:
    typedef T ValueType;
    typedef PR CharacteristicsType;
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
  private:
    static CharacteristicsType _default_characteristics() {
        ARIADNE_PRECONDITION(DefaultConstructible<PR>); if constexpr (DefaultConstructible<PR>) { return PR(); } else { std::abort(); } }
    static CharacteristicsType _get_initializer_list_characteristics(InitializerList<T> const& lst) {
        ARIADNE_PRECONDITION(lst.size()!=0); return get_characteristics(*lst.begin()); }
    static CharacteristicsType _get_array_characteristics(Array<T> const& ary) {
        ARIADNE_PRECONDITION(ary.size()!=0); return get_characteristics(*ary.begin()); }
    template<class G> requires InvocableReturning<ValueType,G,SizeType>
    static CharacteristicsType _get_generator_characteristics(SizeType n, G const& g) {
        if (n==0) { if constexpr (DefaultConstructible<ValueType>) { return get_characteristics(ValueType()); } else { ARIADNE_PRECONDITION(n!=0); } }
        return get_characteristics(g(0)); }
    template<class ForwardIterator>
    static CharacteristicsType _get_iterator_range_characteristics(ForwardIterator first, ForwardIterator last) {
        ARIADNE_PRECONDITION(first!=last); return get_characteristics(*first); }
    Void _set_characteristics(CharacteristicsType pr) {
        static_cast<PR&>(*this)=pr; }

    template <typename F, typename TUP, bool DONE, int N, int... IS> struct call_impl;
    template <typename F, typename TUP, int N, int... IS> struct call_impl<F, TUP, false, N, IS...> {
        static decltype(auto) call(F f, TUP && t) { return call_impl<F, TUP, N == 1 + sizeof...(IS), N, IS..., sizeof...(IS)>::call(f, std::forward<TUP>(t)); } };
    template <typename F, typename TUP, int N, int... IS> struct call_impl<F, TUP, true, N, IS...> {
        static decltype(auto) call(F f, TUP && t) { return f(std::get<IS>(std::forward<TUP>(t))...); } };
    template <typename F, typename TUP> static decltype(auto) call(F f, TUP && t) {
        constexpr SizeType N = std::tuple_size<std::decay_t<TUP>>::value; return call_impl<F, TUP, N==0, N>::call(f, std::forward<TUP>(t)); }

    template<class... CNFGS> requires Constructible<T,CNFGS...> static ValueType _make_default_from_args(CNFGS... cnfgs) {
        return T(cnfgs...); }
    template<class... CNFGS> requires Constructible<T,CNFGS...> static ValueType _make_default_from_tuple(Tuple<CNFGS...>const& cnfgs) {
        return call(&UniformArray<T,PR>::_make_default_from_args<CNFGS...>,cnfgs); }

    template<class CNFG> requires Constructible<T,CNFG> static ValueType _make_default(CNFG const& cnfg) { return T(cnfg); }
    template<class... CNFGS> requires Constructible<T,CNFGS...> static ValueType _make_default(Tuple<CNFGS...> const& cnfgs) { return _make_default_from_tuple(cnfgs); }
    template<class CNFG> requires Same<Tuple<>,CNFG> static ValueType _make_default(CNFG const& cnfg) { return T(); }
    ValueType _make_default() const { return _make_default(this->element_characteristics()); }
  public:
    //! \brief Destructor
    ~UniformArray() { this->_destroy_elements(); uninitialized_delete(_ptr); }

    //! \brief Default constructor. Constructs an empty UniformArray. Requires \a CharacteristicsType to be default constructible
    UniformArray() : _pr(_default_characteristics()), _size(0), _ptr(0) { }
    //! \brief Constructs a UniformArray of size \a n with default-initialised elements.
    explicit UniformArray(const SizeType n) : UniformArray(n,_default_characteristics()) { }
    //! \brief Constructs a UniformArray of size \a n with default-initialised elements.
    explicit UniformArray(const SizeType n, CharacteristicsType pr) : PR(pr), _size(n), _ptr(uninitialized_new(n)) { for(SizeType i=0; i!=n; ++i) { new (_ptr+i) T(_make_default()); } }
    //! \brief Constructs a UniformArray of size \a n with uninitialised elements. The elements should be initialised using placement new.
    explicit UniformArray(const SizeType n, CharacteristicsType pr, Uninitialised) : PR(pr), _size(n), _ptr(uninitialized_new(n)) { }
    //! \brief Constructs a UniformArray of size \a n with elements initialised to \a x.
    UniformArray(const SizeType n, const ValueType& x) : PR(get_characteristics(x)), _size(n), _ptr(uninitialized_new(n)) { this->_uninitialized_fill(x); }

    //! \brief Converts an initializer list to a UniformArray. Requires the size to be at least 1.
    UniformArray(InitializerList<T> lst)
            : PR(_get_initializer_list_characteristics(lst)), _size(lst.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(lst.begin()); }
    //! \brief Converts an ordinary array to a UniformArray. Requires the size to be at least 1.
    UniformArray(Array<T> const& ary)
            : PR(_get_array_characteristics(ary)), _size(ary.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(ary.begin()); }
    //! \brief Generate from a function (object) \a g of type \a G mapping an index to a value.
    //! Requires the size to be at least 1, or the elements to be default-constructible.
    template<class G> requires InvocableReturning<ValueType,G,SizeType>
    UniformArray(SizeType n, G const& g)
            : PR(_get_generator_characteristics(n,g)), _size(n), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_generate(g); }
    //! \brief Constructs a UniformArray from the range \a first to \a last. Requires the size to be at least 1.
    template<class ForwardIterator>
    UniformArray(ForwardIterator first, ForwardIterator last)
            : _pr(_get_iterator_range_characteristics(first,last)), _size(static_cast<SizeType>(std::distance(first,last))), _ptr(uninitialized_new(_size)) {
        assert(std::distance(first,last) >= 0); this->_uninitialized_fill(first); }

    //! \brief Generate from a function (object) \a g of type \a G mapping an index to a value and characteristic parameters.
    template<class G, class... PRS> requires InvocableReturning<ValueType,G,SizeType> && Constructible<ValueType,PRS...>
    UniformArray(SizeType n, G const& g, PRS... prs)
            : PR(get_characteristics(T(prs...))), _size(n), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_generate(g); }
    //! \brief Generate from a function (object) \a g of type \a G mapping an index to a value and a tuple of characteristic parameters.
    template<class G, class... PRS> requires InvocableReturning<ValueType,G,SizeType> && Constructible<ValueType,PRS...>
    UniformArray(SizeType n, G const& g, Tuple<PRS...> prs)
            : PR(get_characteristics(make_from_a_tuple<T>(prs))), _size(n), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_generate(g); }
    //! \brief Constructs a UniformArray from an initializer list and characteristic parameters.
    template<class X, class... PRS> requires Constructible<T,X,PRS...>
    UniformArray(Array<X> const& ary, PRS... prs)
            : PR(prs...), _size(ary.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(ary.begin(),prs...); }
    //! \brief Constructs a UniformArray from an initializer list of doubles and characteristics parameters.
    template<class X, class... PRS> requires Constructible<T,X,PRS...>
    UniformArray(InitializerList<X> lst, PRS... prs)
            : PR(prs...), _size(lst.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(lst.begin(),prs...); }

    //! \brief Conversion constructor.
    template<class TT> requires Convertible<TT,T>
    UniformArray(const UniformArray<TT>& a)
            : _pr(get_characteristics(static_cast<T>(make_from_characteristics<TT>(a.element_characteristics()))))
            , _size(a.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(a.begin()); }

    //! \brief Explicit conversion constructor.
    //! \details Requirement ExplicitlyConvertible<TT,T> as a static_assert to avoid definition of \c TT being required.
    template<class TT>
    explicit UniformArray(const UniformArray<TT>& a)
            : _pr(get_characteristics(static_cast<T>(make_from_characteristics<TT>(a.element_characteristics()))))
            , _size(a.size()), _ptr(uninitialized_new(_size)) {
        static_assert(ExplicitlyConvertible<TT,T>); this->_uninitialized_fill(a.begin()); }

    //! \brief Explicit construction with characteristics parameter.
    template<class X> requires Constructible<T,X,PR>
    explicit UniformArray(const UniformArray<X>& a, CharacteristicsType pr)
            : _pr(get_characteristics(T(make_from_characteristics<X>(a.element_characteristics()),pr)))
            , _size(a.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(a.begin(),pr); }
    //! \brief Explicit construction with characteristics parameter.
    template<class X, class... PRS> // requires Constructible<T,X,PRS...>
    explicit UniformArray(const UniformArray<X>& a, PRS... prs)
            : _pr(get_characteristics(T(prs...))), _size(a.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(a.begin(),prs...); }

    //! \brief Copy constructor.
    UniformArray(const UniformArray<T>& a) : _pr(a.element_characteristics()),_size(a.size()), _ptr(uninitialized_new(_size)) {
        this->_uninitialized_fill(a.begin()); }
    //! \brief Move constructor.
    UniformArray(UniformArray<T>&& a) : _pr(a.element_characteristics()), _size(a._size), _ptr(a._ptr) {
        a._size=0u; a._ptr=nullptr; }
    //! \brief Copy assignment.
    UniformArray<T>& operator=(const UniformArray<T>& a) {
        this->_set_characteristics(a.element_characteristics());
        if(this->size()==a.size()) { fill(a.begin()); }
        else { this->_destroy_elements(); uninitialized_delete(_ptr); _size=a.size(); _ptr=uninitialized_new(_size); this->_uninitialized_fill(a.begin()); }
        return *this; }
    //! \brief Move assignment.
    UniformArray<T>& operator=(UniformArray<T>&& a) {
        if(this!=&a) { this->_set_characteristics(a.element_characteristics()); this->_size=a._size; this->_ptr=a._ptr; a._size=0u; a._ptr=nullptr; } return *this; }

    //! \brief The common characteristics of the array's elements.
    CharacteristicsType element_characteristics() const { return static_cast<PR const&>(*this); }
    //! \brief The element constructed from the common characteristics.
    ValueType default_element() const { return this->_make_default(); }
    //! \brief True if the UniformArray's size is 0.
    bool empty() const { return _size==0u; }
    //! \brief The size of the UniformArray.
    SizeType size() const { return _size; }
    //! \brief The maximum possible size of the UniformArray.
    SizeType max_size() const { return (SizeType) (-1); }
    //! \brief Resizes the UniformArray to hold \a n elements. If \a n is larger than the current size, the extra elements are default initialised.
    void resize(SizeType n) {
        if(size()!=n) {
            pointer _new_ptr=uninitialized_new(n);
            for(SizeType i=0; i!=n; ++i) { if(i<_size) { new (_new_ptr+i) T(_ptr[i]); } else { new (_new_ptr+i) T(_make_default()); } }
            this->_destroy_elements(); uninitialized_delete(_ptr); _size=n; _ptr=_new_ptr; } }
    //! \brief Resizes the UniformArray to hold \a n elements. If \a n is larger than the current size, the extra elements are initialised with value \a t.
    void resize(SizeType n, const T& t) {
        if(size()!=n) {
            pointer _new_ptr=uninitialized_new(n);
            for(SizeType i=0; i!=n; ++i) { if(i<_size) { new (_new_ptr+i) T(_ptr[i]); } else { new (_new_ptr+i) T(t); } }
            this->_destroy_elements(); uninitialized_delete(_ptr); _size=n; _ptr=_new_ptr; } }
    //! \brief Reallocates the UniformArray to hold \a n elements. The new elements are zero-constructed.
    void reallocate(SizeType n) { if(size()!=n) { this->_destroy_elements(); uninitialized_delete(_ptr);
        _size=n; _ptr=uninitialized_new(_size); for(SizeType i=0; i!=_size; ++i) { new (_ptr+i) T(_make_default()); } } }
    //! \brief Efficiently swap two arrays.
    void swap(UniformArray<T>& a) { std::swap(_size,a._size); std::swap(_ptr,a._ptr); }


    //! \brief Get the \a n th element (unchecked).
    const ValueType& get(SizeType i) const { return _ptr[i]; }
    //! \brief Get the \a n th element (unchecked).
    Void set(SizeType i, const ValueType& x) const {
        if constexpr (EqualityComparible<PR,PR>) { ARIADNE_PRECONDITION(get_characteristics(x)==this->element_characteristics()); }
        _ptr[i]=x; }

    //! \brief The \a n th element.
    ElementReference<ValueType> operator[](SizeType i) { return static_cast<ElementReference<ValueType>>(_ptr[i]); }
    //! \brief The \a n th element.
    const ValueType& operator[](SizeType i) const { return _ptr[i]; }
    //! \brief Checked access to the \a n th element.
    ElementReference<ValueType> at(SizeType i) {
        if(i<_size) { return static_cast<ElementReference<ValueType>>(_ptr[i]); }
        else { throw std::out_of_range("UniformArray: index out-of-range"); } }
    //! \brief Checked access to the \a n th element.
    const ValueType& at(SizeType i) const { if(i<_size) { return _ptr[i]; } else { throw std::out_of_range("UniformArray: index out-of-range"); } }

    //! \brief A reference to the first element of the UniformArray.
    ValueType& front() { return _ptr[0]; }
    //! \brief A constant reference to the first element of the UniformArray.
    const ValueType& front() const { return _ptr[0]; }
    //! \brief A reference to the last element of the UniformArray.
    ValueType& back() { return _ptr[_size-1]; }
    //! \brief A constant reference  to the last element of the UniformArray.
    const ValueType& back() const { return _ptr[_size-1]; }

    //! \brief An Iterator pointing to the beginning of the UniformArray.
    Iterator begin() { return _ptr; }
    //! \brief A constant Iterator pointing to the beginning of the UniformArray.
    ConstIterator begin() const { return _ptr; }
    //! \brief An Iterator pointing to the end of the UniformArray.
    Iterator end() { return _ptr+_size; }
    //! \brief A constant Iterator pointing to the end of the UniformArray.
    ConstIterator end() const { return _ptr+_size; }

    //! \brief Tests two arrays for equality
    bool operator==(const UniformArray& other) const {
        if constexpr (EqualityComparible<T,T>) {
            if(size()!=other.size()) return false;
            T const* first=begin(); T const* last=end(); T const* curr=other.begin();
            while(first!=last) { if((*first)!=(*curr)) { return false; } ++first; ++curr; } return true; }
        else {
            ARIADNE_THROW(std::runtime_error,"UniformArray<T>::operator==(const UniformArray<T>&) const",
                          "Cannot compare arays "<<*this<<" and "<<other<<"; value type is not equality-comparible."); } }
    //! \brief Tests two arrays for inequality
    bool operator!=(const UniformArray& other) const { return !((*this)==other); }

    //! \brief Fills the UniformArray with copies of \a x.
    void fill(const ValueType& x) {
        ValueType* curr=begin(); ValueType* end=this->end(); while(curr!=end) { *curr=x; ++curr; } }
    //! \brief Fills the UniformArray from the sequence starting at \a first.
    template<class InputIterator> void fill(InputIterator fst) {
        ValueType* curr=begin(); ValueType* end=this->end(); while(curr!=end) { *curr=*fst; ++curr; ++fst; } }
    //! \brief Assigns the sequence from \a first to \a last.
    template<class ForwardIterator> void assign(ForwardIterator fst, ForwardIterator lst) {
        resize(std::distance(fst,lst)); fill(fst); }
  private:
    void _destroy_elements() { pointer curr=_ptr+_size; while(curr!=_ptr) { --curr; curr->~T(); } }
    void _uninitialized_fill(const ValueType& x) {
        pointer curr=_ptr; pointer end=_ptr+_size; while(curr!=end) { new (curr) T(x); ++curr; } }
    template<class InputIterator> void _uninitialized_fill(InputIterator fst) {
        pointer curr=_ptr; pointer end=_ptr+_size;
        while(curr!=end) { new (curr) T(*fst); ++curr; ++fst; } }
    template<class InputIterator, class Parameters> void _uninitialized_fill(InputIterator fst, Parameters parameters) {
        pointer curr=_ptr; pointer end=_ptr+_size;
        while(curr!=end) { new (curr) T(*fst,parameters); ++curr; ++fst; } }
    template<class G> void _uninitialized_generate(G g) {
        for(SizeType i=0u; i!=this->size(); ++i) {
            new (_ptr+i) T(g(i)); assert(get_characteristics(_ptr[i])==element_characteristics()); } }
  private:
    SizeType _size;
    pointer _ptr;
};

template<class T, class PR> OutputStream& operator<<(OutputStream& os, const UniformArray<T,PR>& a) {
    os << "<"<<a.element_characteristics()<<">";
    bool first=true;
    for(auto x : a) {
        os << (first ? "[" : ",") << x;
        first = false;
    }
    if(first) { os << "["; }
    return os << "]";
}


} // namespace Ariadne

#endif
