/***************************************************************************
 *            writable.hpp
 *
 *  Copyright  2023  Pieter Collins
 *
 ****************************************************************************/

/*
 * This file is part of Helper, under the MIT license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*! \file writable.hpp
 *  \brief
 */

#ifndef HELPER_WRITABLE_HPP
#define HELPER_WRITABLE_HPP

#include "metaprogramming.hpp"

namespace Helper {

using std::ostream;

/************ WritableInterface **********************************************/

class WritableInterface {
  public:
    virtual ~WritableInterface() = default;
    friend ostream& operator<<(ostream& os, const WritableInterface& w);
  public:
    inline ostream& write(ostream& os) const { return this->_write(os); }
  protected:
  public:
    virtual ostream& _write(ostream&) const = 0;
};
inline ostream& operator<<(ostream& os, const WritableInterface& w) { w._write(os); return os; }

template<class T, class = decltype(declval<T>()._write(declval<ostream>()))> True has_write(int);
template<class T> False has_write(...);
template<class T, class = Fallback> struct IsWritable : decltype(has_write<T>(1)) { };

template<class T> requires IsWritable<T>::value ostream& operator<<(ostream& os, const T& t) {
    return t._write(os);
}

template<class T> class Writer;
template<class T> class WriterInterface;
template<class T, class W=WriterInterface<T>> class WritableTemporary;

template<class T> class WriterInterface {
  public:
    virtual ~WriterInterface() = default;
    inline WritableTemporary<T> operator() (T const& t) const;
  private:
    virtual ostream& _write(ostream& os, T const& t) const = 0;
    friend ostream& operator<<(ostream& os, WritableTemporary<T> const& t);
};

template<class T> class Handle;
template<class T> class Writer : public Handle<WriterInterface<T>> {
  public:
    using Handle<WriterInterface<T>>::Handle;
    Writer(Handle<WriterInterface<T> > wh) : Handle<WriterInterface<T> >(wh) { }
    inline WritableTemporary<T> operator() (T const& t) const;
};


template<class T, class W> class WritableTemporary {
  private:
    W const& _w; T const& _t;
    WritableTemporary(W const& w, T const& t) : _w(w), _t(t) { }
    friend W; friend class Writer<T>;
  public:
    friend ostream& operator<<(ostream& os, WritableTemporary<T,W> const& wt) { return wt._w._write(os,wt._t); }
};

template<class T> WritableTemporary<T> WriterInterface<T>::operator() (T const& t) const {
    return WritableTemporary<T>(*this,t); }
template<class T> WritableTemporary<T> Writer<T>::operator() (T const& t) const {
    return WritableTemporary<T>(*this->_ptr,t); }


template<class T> class RepresentationWriter;

} // namespace Helper

#endif
