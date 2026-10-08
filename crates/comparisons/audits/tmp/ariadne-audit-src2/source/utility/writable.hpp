/***************************************************************************
 *            utility/writable.hpp
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

/*! \file utility/writable.hpp
 *  \brief
 */



#ifndef ARIADNE_WRITABLE_HPP
#define ARIADNE_WRITABLE_HPP

#include "typedefs.hpp"
#include "metaprogramming.hpp"

namespace Ariadne {

template<class T> class Handle;

class WritableInterface {
  public:
    virtual ~WritableInterface() = default;
    friend OutputStream& operator<<(OutputStream& os, const WritableInterface& w);
  private: public:
    virtual OutputStream& _write(OutputStream&) const = 0;
};
inline OutputStream& operator<<(OutputStream& os, const WritableInterface& w) { return w._write(os); }

template<class T> concept IsWritable = requires (OutputStream& os, T const& t) {
    t._write(os);
};

template<class T> requires IsWritable<T> inline OutputStream& operator<<(OutputStream& os, const T& t) {
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
    virtual OutputStream& _write(OutputStream& os, T const& t) const = 0;
    friend OutputStream& operator<<(OutputStream& os, WritableTemporary<T> const& t);
};


template<class W, class T> concept AWriter = requires (W const& w, OutputStream& os, T const& t) {
    w._write(os,t);
};

template<class T, AWriter<T> W> class WriterMixin : public WriterInterface<T> {
    W _w;
  public:
    WriterMixin(W w) : _w(w) { }
    virtual OutputStream& _write(OutputStream& os, T const& t) const override { return this->_w._write(os,t); }
};

template<class T> class Writer : public Handle<const WriterInterface<T>> {
  public:
    using Handle<const WriterInterface<T>>::Handle;
    Writer(Handle<const WriterInterface<T> > wh) : Handle<WriterInterface<T> >(wh) { }
    template<AWriter<T> W> Writer(W w) : Writer(new WriterMixin<T,W>(w)) { }
    inline WritableTemporary<T> operator() (T const& t) const;
};


template<class T, class W> class WritableTemporary;
template<class T, class W> inline WritableTemporary<T,W> make_writable(W const& w, T const& t);

template<class T, class W> class WritableTemporary {
  private:
    W const& _w; T const& _t;
    WritableTemporary(W const& w, T const& t) : _w(w), _t(t) { }
    friend W;
    friend class Writer<T>;
    template<class TT, class WW> friend WritableTemporary<TT,WW> make_writable(WW const&, TT const&);
  public:
    friend OutputStream& operator<<(OutputStream& os, WritableTemporary<T,W> const& wt) { wt._w._write(os,wt._t); return os; }
};
template<class T, class W> WritableTemporary(W const&, T const&) -> WritableTemporary<T,W>;
template<class T, class W> inline WritableTemporary<T,W> make_writable(W const& w, T const& t) { return WritableTemporary<T,W>(w,t); }

template<class T> WritableTemporary<T> WriterInterface<T>::operator() (T const& t) const {
    return WritableTemporary<T>(*this,t); }
template<class T> WritableTemporary<T> Writer<T>::operator() (T const& t) const {
    return WritableTemporary<T>(*this->_ptr,t); }


template<class T> class RepresentationWriter;


template<class T> struct Representation { const T* pointer; Representation(const T& t) : pointer(&t) { } const T& reference() const { return *pointer; } };
template<class T> inline Representation<T> representation(const T& t) { return Representation<T>(t); }
template<class T> concept HasRepr = requires(OutputStream& os, T const& t) { t._repr(os); };
template<class T> inline OutputStream& operator<<(OutputStream& os, const Representation<T>& obj) {
    if constexpr (HasRepr<T>) {
        obj.reference()._repr(os); return os;
    } else {
        return os << obj.reference();
    }
}

} // namespace Ariadne

#endif
