/***************************************************************************
 *            algebra/tensor.hpp
 *
 *  Copyright  2018-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*! \file algebra/tensor.hpp
 *  \brief Arbitrary-rank tensors
 */

#ifndef ARIADNE_TENSOR_HPP
#define ARIADNE_TENSOR_HPP

#include "vector.hpp"
#include "matrix.hpp"
#include "diagonal_matrix.hpp"
#include "io/graphics_interface.hpp"
#include "io/figure.hpp"

namespace Ariadne {

//! \brief A row of a rank-2 tensor.
template<class T> class TensorRow {
    T _t; SizeType _i;
  public:
    TensorRow(T& t, SizeType i) : _t(t), _i(i) { };
    T const& operator[] (SizeType j) const { return _t[Array<SizeType>({_i,j})]; }
    T& operator[] (SizeType j) { return _t[Array<SizeType>({_i,j})]; }
};

//! \ingroup LinearAlgebraSubModule
//! \brief A rank-\a N tensor with elements of type \a X.
template<SizeType N, class X> class Tensor :
                      public LabelledDrawable2d3dInterface,
                      public Drawable2d3dInterface
 
 {
//   TODO: Also provide a version which is not templated on rank.
    Array<SizeType> _ns;
    Array<X> _a;

  public:
    typedef X ValueType;
    Tensor(Array<SizeType> const& ns, X const& z) : _ns(ns), _a(_total_size(),z) { }
    template<class G> requires InvocableReturning<X,G,Array<SizeType>> Tensor(Array<SizeType> const& ns, G const& g);
    constexpr SizeType rank() const { return _ns.size(); }
    Array<SizeType> sizes() const { return _ns; }
    SizeType size(SizeType i) const { return _ns[i]; }
    X const& operator[] (Array<SizeType> const& is) const { return _a[_index(is)]; }
    X& operator[] (Array<SizeType> const& is) { return _a[_index(is)]; }

    TensorRow<Tensor<N,X>const&> operator[] (SizeType const& i) const { return TensorRow<Tensor<N,X>const&>(*this,i); }
    TensorRow<Tensor<N,X>&> operator[] (SizeType const& i) { return TensorRow<Tensor<N,X>&>(*this,i); }
    friend OutputStream& operator<<(OutputStream& os, Tensor<N,X> const& t) { return t._write(os); }
    
    Tensor<N, X>* clone() const { return new Tensor(*this); /*new Tensor(_ns, _a[0]*0)*/;}
    Tensor<N, X>* clone2d3d() const { return new Tensor(*this); /*new Tensor(_ns, _a[0]*0)*/;}
    DimensionType dimension() const { return _ns.size(); }
    Void draw(CanvasInterface& canvas, const Projection2d& p) const;
    Void draw(CanvasInterface& c, const Variables2d& p) const;
    Void draw(CanvasInterface& canvas, const Projection3d& p) const;
    Void draw(CanvasInterface& c, const Variables3d& p) const;

  private: public:
    SizeType _index(Array<SizeType> is) const { SizeType k=0; SizeType r=is[k]; ++k; while(k!=is.size()) { r*=_ns[k]; r+=is[k]; ++k; } return r; }
    SizeType _total_size() const { SizeType r=1; for(SizeType i=0; i!=N; ++i) { r*=_ns[i]; } return r; }
    OutputStream& _write(OutputStream& os) const;
};

template<class X, class F> requires Invocable<F,X> auto transform(Tensor<2,X> const& t, F const& f) -> Tensor<2,ResultOf<F(X)>> {
    Tensor<2,ResultOf<F(X)>> r(t.sizes(), f(t[{0,0}]));
    for(SizeType i0=0; i0!=r.size(0); ++i0) { for(SizeType i1=0; i1!=r.size(1); ++i1) { r[{i0,i1}]=f(t[{i0,i1}]); } }
    return r;
}

template<SizeType N, class X> template<class G> requires InvocableReturning<X,G,Array<SizeType>>
Tensor<N,X>::Tensor(Array<SizeType> const& ns, G const& g)
    : _ns(ns), _a(_total_size(),g({0,0}))
{
    static_assert(N==2);
    for(SizeType i0=0; i0!=ns[0]; ++i0) {
        for(SizeType i1=0; i1!=ns[1]; ++i1) {
            _a[i0*ns[1]+i1]=g({i0,i1});
        }
    }
}

template<SizeType N, class X> OutputStream& write_tensor(OutputStream& os, Tensor<N,X> const& t);

template<SizeType N, class X> OutputStream& Tensor<N,X>::_write(OutputStream& os) const {
    return write_tensor(os,*this);
}

template<class X> OutputStream& write_tensor(OutputStream& os, Tensor<2,X> const& t) {
    os << "Tensor<" << t.sizes() << ">";
    os << '{';
    for(SizeType i0=0; i0!=t.size(0); ++i0) {
        if(i0!=0) { os << ','; }
        os << '{';
        for(SizeType i1=0; i1!=t.size(1); ++i1) {
            if(i1!=0) { os << ','; }
            os << t[i0][i1];
        }
        os << '}';
    }
    os << '}';
    return os;
}

template<class X> OutputStream& write_tensor(OutputStream& os, Tensor<3,X> const& t) {
    os << "Tensor<" << t.sizes() << ">";
    os << '{';
    for(SizeType i0=0; i0!=t.size(0); ++i0) {
        if(i0!=0) { os << ','; }
        os << '{';
        for(SizeType i1=0; i1!=t.size(1); ++i1) {
            if(i1!=0) { os << ','; }
            os << '{';
            for(SizeType i2=0; i2!=t.size(2); ++i2) {
                if(i2!=0) { os << ','; }
                os << t[{i0,i1,i2}];
            }
            os << '}';
        }
        os << '}';
    }
    os << '}';
    return os;
}

template<SizeType N, class X> OutputStream& write_tensor(OutputStream& os, Tensor<N,X> const& t) {
    SizeType M=t.rank();
    Array<SizeType> is(M,0);
    SizeType m=M;
    X const* p=t._a.begin();
    while (m!=0) {
        m=m-1;
        while(is[m]!=t._ns[m]) {
            os << is << " " << *p << ",\n";
            ++p;
            ++is[m];
        }
        for(SizeType k=m; k!=M; ++k) {
            is[k]=0;
        }
    }
    return os;
}


} // namespace Ariadne

#endif // ARIADNE_TENSOR_HPP
