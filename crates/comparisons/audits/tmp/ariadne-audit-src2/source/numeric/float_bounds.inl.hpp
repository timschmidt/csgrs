/***************************************************************************
 *            float_bounds.inl.hpp
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


#ifndef ARIADNE_FLOAT_BOUNDS_INL_HPP
#define ARIADNE_FLOAT_BOUNDS_INL_HPP

#include "float_bounds.hpp"

namespace Ariadne {

template<class F> inline auto Operations<Bounds<F>>::_add(Bounds<F> const& x1, F const& x2) -> Bounds<F> {
    return Bounds<F>(add(down,x1._l,x2),add(up,x1._u,x2));
}

template<class F> inline auto Operations<Bounds<F>>::_add(F const& x1, Bounds<F> const& x2) -> Bounds<F> {
    return Bounds<F>(add(down,x1,x2._l),add(down,x1,x2._u));
}

template<class F> inline auto Operations<Bounds<F>>::_sub(Bounds<F> const& x1, F const& x2) -> Bounds<F> {
    return Bounds<F>(sub(down,x1._l,x2),sub(up,x1._u,x2));
}

template<class F> inline auto Operations<Bounds<F>>::_sub(F const& x1, Bounds<F> const& x2) -> Bounds<F> {
    return Bounds<F>(sub(down,x1,x2._u),sub(up,x1,x2._l));
}

template<class F> inline auto Operations<Bounds<F>>::_mul(Bounds<F> const& x1, F const& x2) -> Bounds<F>
{
    const F& x1l=x1.lower_raw(); const F& x1u=x1.upper_raw();
    const F& x2v=x2;
    PR pr(min(x1.precision(),x2.precision()));
    F rl(pr), ru(pr);
    if(x2v>=0.0_x) {
        rl=mul(down,x1l,x2v); ru=mul(up,x1u,x2v);
    } else {
        rl=mul(down,x1u,x2v); ru=mul(up,x1l,x2v);
    }
    return Bounds<F>(rl,ru);
}

template<class F> inline auto Operations<Bounds<F>>::_mul(F const& x1, Bounds<F> const& x2) -> Bounds<F>
{
    const F& x1v=x1;
    const F& x2l=x2.lower_raw(); const F& x2u=x2.upper_raw();
    PR pr(min(x1.precision(),x2.precision()));
    F rl(pr), ru(pr);
    if(x1v>=0.0_x) {
        rl=mul(down,x1v,x2l); ru=mul(up,x1v,x2u);
    } else {
        rl=mul(down,x1v,x2u); ru=mul(up,x1v,x2l);
    }
    return Bounds<F>(rl,ru);
}

template<class F> inline auto Operations<Bounds<F>>::_div(Bounds<F> const& x1, F const& x2) -> Bounds<F>
{
    const F& x1l=x1.lower_raw();
    const F& x1u=x1.upper_raw();
    const F& x2v=x2;
    PR pr(min(x1.precision(),x2.precision()));
    F rl(pr), ru(pr);
    if(x2v>0) {
        rl=div(down,x1l,x2v); ru=div(up,x1u,x2v);
    } else if(x2v<0) {
        rl=div(down,x1u,x2v); ru=div(up,x1l,x2v);
    } else {
        //ARIADNE_THROW(DivideByZeroException,"FloatBounds div(FloatBounds const& x1, Float x2)","x1="<<x1<<", x2="<<x2);
        rl=-F::inf(pr);
        ru=+F::inf(pr);
    }
    return Bounds<F>(rl,ru);
}

template<class F> inline auto Operations<Bounds<F>>::_div(F const& x1, Bounds<F> const& x2) -> Bounds<F>
{
    const F& x1v=x1;
    const F& i2l=x2.lower_raw();
    const F& i2u=x2.upper_raw();
    PR pr(min(x1.precision(),x2.precision()));
    F rl(pr), ru(pr);
    if(i2l<=0 && i2u>=0) {
        //ARIADNE_THROW(DivideByZeroException,"FloatBounds div(Float const& x1, FloatBounds x2)","x1="<<x1<<", x2="<<x2);
        rl=-F::inf(pr);
        ru=+F::inf(pr);
    } else if(x1v>=0) {
        rl=div(down,x1v,i2u); ru=div(up,x1v,i2l);
    } else {
        rl=div(down,x1v,i2l); ru=div(up,x1v,i2u);
    }
    return Bounds<F>(rl,ru);
}



} // namespace Ariadne

#endif
