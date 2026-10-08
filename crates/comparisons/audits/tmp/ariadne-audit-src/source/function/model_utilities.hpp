/***************************************************************************
 *            model_utilities.hpp
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


namespace Ariadne {

namespace {

template<class F> struct ValidatedApproximation {
    Bounds<F> _v; Approximation<F> _a;
    ValidatedApproximation(Bounds<F>const& x) : _v(x), _a(x) { }
    operator Bounds<F> const& () const { return _v; }
    LowerBound<F> lower() const { return _v.lower(); }
    Approximation<F> middle() const { return _a; }
    UpperBound<F> upper() const { return _v.upper(); }
    F const& lower_raw() const { return _v.lower_raw(); }
    F const& middle_raw() const { return _a.raw(); }
    F const& upper_raw() const { return _v.upper_raw(); }
    friend OutputStream& operator<<(OutputStream& os, ValidatedApproximation<F> const& x) {
        return os << "{"<<x._v.lower()<<":"<<x._a<<":"<<x._v.upper()<<"}"; }
};
template<class F> Approximation<F> const& make_validated_approximation(Approximation<F> const& x) { return x; }
template<class F> ValidatedApproximation<F> make_validated_approximation(Bounds<F> const& x) { return ValidatedApproximation<F>(x); }


template<ARawFloat F> Rounded<F> const& cast_rounded(F const& x) { return reinterpret_cast<Rounded<F>const&>(x); }
template<ARawFloat F> Rounded<F>& cast_rounded(Error<F>& x) { return reinterpret_cast<Rounded<F>&>(x); }

template<class F> inline Void acc_err(F const& ml, F const& u, F& e) {
    e=add(rounded,e,hlf(add(rounded,ml,u)));
}

template<class F, class PRE> Ball<F,RawFloatType<PRE>> add(F const& x1, F const& x2, PRE pre) {
    F mx1=-x1;
    F::set_rounding_to_nearest();
    F r(x1.raw() + x2.raw());
    F::set_rounding_upward();
    F u=x1.raw()+x2.raw();
    F ml=mx1.raw()-x2.raw();
    Error<RawFloatType<PRE>> e(pre);
    e += max(u-r,ml+r);
    return Ball(r,e);
}

template<class F, class PRE> Ball<F,RawFloatType<PRE>> mul(F const& x1, F const& x2, PRE pre) {
    F mx1=-x1;
    F::set_rounding_to_nearest();
    F r(x1.raw() * x2.raw());
    F::set_rounding_upward();
    F u=x1.raw()*x2.raw();
    F ml=mx1.raw()*x2.raw();
    Error<RawFloatType<PRE>> e(pre);
    e += max(u-r,ml+r);
    return Ball(r,e);
}



template<class F> F add_err(F const& x1, F const& x2, Error<F>& e) {
    F mx1=-x1;
    F::set_rounding_to_nearest();
    F r(add(rounded,x1,x2));
    F::set_rounding_upward();
    F u=add(rounded,x1,x2);
    F ml=sub(rounded,mx1,x2);
    acc_err(ml,u,e.raw());
    return r;
}

template<class F> F add_err(F const& x, ValidatedApproximation<F> const& c, Error<F>& e) {
    F const& xv=x;
    F const& cl=c.lower_raw();
    F const& cm=c.middle_raw();
    F const& cu=c.upper_raw();
    F& re=e.raw();
    F::set_rounding_to_nearest();
    F rv=add(rounded,xv,cm);
    F::set_rounding_upward();
    F u=add(rounded,xv,cu);
    F ml=add(rounded,(-xv),cl);
    acc_err(ml,u,re);
    return F(rv);
}

template<class F> F add_err(F const& x, Bounds<F> const& c, Error<F>& e) {
    return add_err(x,ValidatedApproximation<F>(c),e);
}

template<class F> Bounds<F> add_err(Bounds<F> const& x1, Bounds<F> const& x2, Error<F>& e) {
    return add(x1,x2);
}

template<class F> UpperInterval<F> add_err(UpperInterval<F> const& x1, UpperInterval<F> const& x2, Error<F>& e) {
    return add(x1,x2);
}

template<class F> UpperInterval<F> add_err(UpperInterval<F> const& x1, Nat n2, Error<F>& e) {
    return add(x1,n2);
}

template<class F> Approximation<F> add_err(Approximation<F> const& x1, Approximation<F> const& x2, UnknownError<F>& e) {
    return add(x1,x2);
}

template<class F> F sub_err(F const& x1, F const& x2, Error<F>& e) {
    F mx1=-x1;
    F::set_rounding_to_nearest();
    F r(sub(rounded,x1.raw(),x2.raw()));
    F::set_rounding_upward();
    F u=sub(rounded,x1.raw(),x2.raw());
    F ml=add(rounded,mx1.raw(),x2.raw());
    acc_err(ml,u,e.raw());
    return r;
}

template<class F> Bounds<F> sub_err(Bounds<F> const& x1, Bounds<F> const& x2, Error<F>& e) {
    return sub(x1,x2);
}

template<class F> UpperInterval<F> sub_err(UpperInterval<F> const& x1, UpperInterval<F> const& x2, Error<F>& e) {
    return sub(x1,x2);
}

template<class F> UpperInterval<F> sub_err(UpperInterval<F> const& x1, Nat n2, Error<F>& e) {
    return sub(x1,n2);
}

template<class F> Approximation<F> sub_err(Approximation<F> const& x1, Approximation<F> const& x2, UnknownError<F>& e) {
    return sub(x1,x2);
}

template<class F> F mul_no_err(F const& x1, F const& x2) {
    F::set_rounding_to_nearest();
    F r(x1.raw() * x2.raw());
    F::set_rounding_upward();
    return r;
}

template<class F> Approximation<F> mul_no_err(Approximation<F> const& x1, Approximation<F> const& x2) {
    return x1*x2;
}

template<class F> F mul_err(F const& x1, F const& x2, Error<F>& e) {
    F mx1=-x1;
    F::set_rounding_to_nearest();
    F r(mul(rounded,x1,x2));
    F::set_rounding_upward();
    F u=mul(rounded,x1,x2);
    F ml=mul(rounded,mx1,x2);
    acc_err(ml,u,e.raw());
    return r;
}

template<class F> F mul_err(F const& x, ValidatedApproximation<F> const& c, Error<F>& e) {
    F const& xv=x;
    F const& cu=c.upper_raw();
    F const& cm=c.middle_raw();
    F const& cl=c.lower_raw();
    F& re=e.raw();
    F::set_rounding_to_nearest();
    F rv=mul(rounded,xv,cm);
    F::set_rounding_upward();
    if(xv>=0) {
        F mcl=-cl;
        F u=mul(rounded,xv,cu);
        F ml=mul(rounded,xv,mcl);
        acc_err(ml,u,re);
    } else {
        F mcu=-cu;
        F u=mul(rounded,xv,cl);
        F ml=mul(rounded,xv,mcu);
        acc_err(ml,u,re);
    }
    return F(rv);
}

template<class F> F mul_err(F const& x, Bounds<F> const& c, Error<F>& e) {
    return mul_err(x,ValidatedApproximation<F>(c),e);
}

template<class F> F mul_err(F const& x1, Nat n2, Error<F>& e) {
    return mul_err(x1,F(n2,x1.precision()),e);
}

template<class F> Bounds<F> mul_err(Bounds<F> const& x1, Bounds<F> const& x2, Error<F>& e) {
    return mul(x1,x2);
}

template<class F> Bounds<F> mul_err(Bounds<F> const& x1, ValidatedApproximation<F> const& x2, Error<F>& e) {
    return mul(x1,static_cast<Bounds<F>>(x2));
}

template<class F> Bounds<F> mul_err(Bounds<F> const& x1, Nat const& n2, Error<F>& e) {
    return mul(x1,n2);
}

template<class F> UpperInterval<F> mul_err(UpperInterval<F> const& x1, UpperInterval<F> const& x2, Error<F>& e) {
    return mul(x1,x2);
}

template<class F> UpperInterval<F> mul_err(UpperInterval<F> const& x1, Nat n2, Error<F>& e) {
    return mul(x1,n2);
}

template<class F> Approximation<F> mul_err(Approximation<F> const& x1, Approximation<F> const& x2, UnknownError<F>& e) {
    return mul(x1,x2);
}

template<class F> Approximation<F> mul_err(Approximation<F> const& x1, Nat n2, UnknownError<F>& e) {
    return mul_err(x1,Approximation<F>(n2,x1.precision()),e);
}

template<class F> F div_err(F const& x1, F const& x2, Error<F>& e) {
    F mx1=-x1;
    F::set_rounding_to_nearest();
    F r(div(rounded,x1.raw(),x2.raw()));
    F::set_rounding_upward();
    F u=div(rounded,x1.raw(),x2.raw());
    F ml=div(rounded,mx1.raw(),x2.raw());
    acc_err(ml,u,e.raw());
    return r;
}

template<class F> F div_err(F const& x1, Nat n2, Error<F>& e) {
    return div_err(x1,F(n2,x1.precision()),e);
}

template<class F> Bounds<F> div_err(Bounds<F> const& x1, Bounds<F> const& x2, Error<F>& e) {
    return div(x1,x2);
}

template<class F> Bounds<F> div_err(Bounds<F> const& x1, Nat n2, Error<F>& e) {
    return div(x1,n2);
}

template<class F> UpperInterval<F> div_err(UpperInterval<F> const& x1, UpperInterval<F> const& x2, Error<F>& e) {
    return div(x1,x2);
}

template<class F> UpperInterval<F> div_err(UpperInterval<F> const& x1, Nat n2, Error<F>& e) {
    return div(x1,n2);
}

template<class F> Approximation<F> div_err(Approximation<F> const& x1, Approximation<F> const& x2, UnknownError<F>& e) {
    return div(x1,x2);
}

template<class F> Approximation<F> div_err(Approximation<F> const& x1, Nat n2, UnknownError<F>& e) {
    return div_err(x1,Approximation<F>(n2,x1.precision()),e);
}




template<class F> F fma_err(F const& x, F const& y, F z, Error<F>& e) {
    F const& xv=x.raw();
    F const& yv=y.raw();
    F const& zv=z.raw();
    F& re=e.raw();
    F::set_rounding_to_nearest();
    F rv=fma(rounded,xv,yv,zv);
    F::set_rounding_upward();
    F myv=-yv;
    F mzv=-zv;
    F u=fma(rounded,xv,yv,zv);
    F ml=fma(rounded,xv,myv,mzv);
    acc_err(ml,u,re);
    return F(rv);
}

template<class F> F fma_err(ValidatedApproximation<F> const& c, F const& x, F y, Error<F>& e) {
    F const& xv=x.raw();
    F const& cu=c.upper_raw();
    F const& cm=c.middle_raw();
    F const& cl=c.lower_raw();
    F const& yv=y.raw();
    F& re=e.raw();
    F::set_rounding_to_nearest();
    F rv=fma(rounded,xv,cm,yv);
    F::set_rounding_upward();
    F u,ml;
    if(xv>=0) {
        F mcl=-cl;
        F myv=-yv;
        u=fma(xv,cu,yv);
        ml=fma(xv,mcl,myv);
    } else {
        F mcu=-cu;
        F myv=-yv;
        u=fma(xv,cl,yv);
        ml=fma(xv,mcu,myv);
    }
    acc_err(ml,u,e);
    return F(rv);
}

template<class F> F fma_err(Bounds<F> const& c, F const& x, F y, Error<F>& e) {
    return fma_err(ValidatedApproximation<F>(c),x,y,e);
}

template<class F> Bounds<F> fma_err(Bounds<F> const& x, Bounds<F> const& y, Bounds<F> z, Error<F>& e) {
    return fma(x,y,z);
}

template<class F> UpperInterval<F> fma_err(UpperInterval<F> const& x, UpperInterval<F> const& y, UpperInterval<F> z, Error<F>& e) {
    return fma(x,y,z);
}

template<class F> Bounds<F> fma_err(ValidatedApproximation<F> const& x, Bounds<F> const& y, Bounds<F> z, Error<F>& e) {
    return fma(static_cast<Bounds<F>>(x),y,z);
}

template<class F> Approximation<F> fma_err(Approximation<F> const& x, Approximation<F> const& y, Approximation<F> z, UnknownError<F>& e) {
    return fma(x,y,z);
}

// Returns an approximation to a1*b2+a2*b2, adding error to e
template<class F> F lin_err(F const& a1, F const& b1, F const& a2, F const& b2, Error<F>& e) {
    F::set_rounding_to_nearest();
    F r=cast_exact(cast_rounded(a1)*cast_rounded(b2)+cast_rounded(b1)*cast_rounded(a2));
    F::set_rounding_upward();
    F mb1=-b1;
    F mb2=-b2;
    Rounded<F> u=cast_rounded(a1)*cast_rounded(b2)+cast_rounded(b1)*cast_rounded(a2);
    Rounded<F> ml=cast_rounded(a1)*cast_rounded(mb2)+cast_rounded(mb1)*cast_rounded(a2);
    cast_rounded(e) += hlf(ml+u);
    return r;
}

} // namespace

} //namespace Ariadne


