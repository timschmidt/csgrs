/***************************************************************************
 *            utility/path.hpp
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

/*! \file utility/path.hpp
 *  \brief
 */



#ifndef ARIADNE_PATH_HPP
#define ARIADNE_PATH_HPP

#include "macros.hpp"
#include "typedefs.hpp"

#include "container.hpp"

namespace Ariadne {

typedef Int32Type HeightType;
typedef HeightType DepthType;

//! \brief A dynamically-allocated word.
template<class T> class Path
{
    HeightType _height;
    List<T> _word;
  public:
    typedef typename List<T>::ConstIterator ConstIterator;

    //! \brief Makes an empty path based at height \a h.
    Path(HeightType h);

    //! \brief Construct from an initializer list.
    Path(HeightType h, const InitializerList<T>& wd);

    //! The height of the path (i.e. first nontrivial digit).
    HeightType height() const;
    //! The depth of the path (i.e. last known digit).
    DepthType depth() const;
    //! The number of elements in the word
    SizeType size() const;
    //! The word describing the path.
    List<T> const& word() const;

    //! Equality operator.
    bool operator==(const Path<T>& w) const;

    //! Comparison operator.
    bool operator<(const Path<T>& w) const;

    //! Append the symbol \a s.
    Path<T> operator+(const T& s) const;

    //! \brief Get the value at depth \a k.
    T operator[](DepthType k) const;

    //! \brief Sets the last bit to \a x.
    T get_back () const;

    //! \brief Sets the last bit to \a x.
    void set_back (T x);

    //! \brief appends the symbol \a s.
    void push_back(const T& s);

    //! \brief Erase and return the last element.
    T pop();

    //! \brief Erase and return the last element.
    T pop_back();

    //! \brief appends the symbol \a s.
    void push(const T& s);

    //! \brief appends the symbol \a s.
    void append(const T& s);
    //! \brief appends the word \a w.
    void append(const List<T>& w);
};

enum class LeftOrRight : bool { LEFT=false, RIGHT=true };
enum class LeftOrMiddleOrRight : char { LEFT=-1, MIDDLE=0, RIGHT=+1 };
OutputStream& operator<<(OutputStream& os, const LeftOrRight& s);
OutputStream& operator<<(OutputStream& os, const LeftOrMiddleOrRight& s);


class BinaryTreePath : public Path<bool> {
  public:
    using Path<bool>::Path;
    BinaryTreePath(const Path<bool>& pth) : Path<bool>(pth) { }
};
class TernaryDagPath : public Path<LeftOrMiddleOrRight> {
  public: using Path<LeftOrMiddleOrRight>::Path;
};


//! \related Path \brief write to an output stream.
template<class T> OutputStream& operator<<(OutputStream& os, const Path<T>& pth);


template<class T> Path<T>::Path(HeightType h)
    : _height(h), _word()
{
}

template<class T> Path<T>::Path(HeightType h, const InitializerList<T>& wd)
    : _height(h), _word(wd)
{
}

template<class T> HeightType Path<T>::height() const {
    return this->_height;
}

template<class T> DepthType Path<T>::depth() const {
    return static_cast<DepthType>(this->_word.size()) - this->height();
}

template<class T> SizeType Path<T>::size() const {
    return this->_word.size();
}

template<class T> List<T> const& Path<T>::word() const {
    return this->_word;
}

template<class T> bool Path<T>::operator==(const Path<T>& other) const {
    auto w1=this->word();
    auto w2=other.word();
    if(this->height()!=other.height()) { return false; }
    if(w1.size() != w2.size()) { return false; }
    for(SizeType i = 0; i != w1.size(); ++i ) {
        if( w1[i] != w2[i] ) {
            return false;
        }
    }
    return true;
}

template<class T> bool Path<T>::operator<(const Path<T>& other) const {
    auto w1=this->word();
    auto w2=other.word();
    if(this->height()!=other.height()) { return false; }
    for( SizeType i = 0; i != std::min( w1.size(),w2.size() ); ++i ) {
        if( w1[i] != w2[i] ) {
            return w1[i] < w2[i];
        }
    }
    return w1.size() < w2.size();
}

template<class T> Path<T> Path<T>::operator+(const T& s) const {
    Path<T> r=*this;
    r.append(s);
    return r;
}

template<class T> T Path<T>::operator[](DepthType k) const {
    return this->_word[k+this->height()];
}

template<class T> T Path<T>::get_back() const {
    return this->_word.back();
}

template<class T> void Path<T>::set_back(T t) {
    this->_word.back()=t;
}

template<class T> T Path<T>::pop() {
    T back=this->_word.back(); this->_word.pop_back(); return back;
}

template<class T> void Path<T>::push( const T& s ) {
    this->_word.push_back( s );
}

template<class T> T Path<T>::pop_back( ) {
    T back=this->_word.back(); this->_word.pop_back(); return back;
}

template<class T> void Path<T>::push_back( const T& s ) {
    this->_word.push_back( s );
}

template<class T> void Path<T>::append( const T& s ) {
    this->_word.push_back( s );
}

template<class T> void Path<T>::append( const List<T>& w ) {
    for( SizeType i = 0; i < w.size() ; i++ ){
        this->_word.push_back( w[i] );
    }
}

//! \brief Serializes data into a stream.
//! The data format is "01011" or "e" for an empty word.
template<class T> OutputStream& operator<<(OutputStream& os, const Path<T>& pth) {
    if( pth.word().empty() ) {
        return os << 'e';
    }
    os << std::noboolalpha;
    for(SizeType i=0; i!=pth.size(); ++i) {
        if(i==pth.height()) { os << "."; }
        os << pth.word()[i];
    }
    os << std::boolalpha;
    return os;
}

} // namespace Ariadne

#endif /* ARIADNE_PATH_HPP */
