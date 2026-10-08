/***************************************************************************
 *            numeric/extended.hpp
 *
 *  Copyright  2017-20  Pieter Collins
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

/*! \file numeric/extended.hpp
 *  \brief Extended real numbers.
 */

#ifndef ARIADNE_EXTENDED_HPP
#define ARIADNE_EXTENDED_HPP

#include "numeric/sign.hpp"
#include "numeric/integer.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/rational.hpp"

namespace Ariadne {

inline Comparison cmp(int n1, int n2) {
    return n1==n2 ? Comparison::EQUAL : n1<n2 ? Comparison::LESS : Comparison::GREATER;
}


template<class X> class FiniteOperations;
template<class X> class ExtensionOperations;

//! \ingroup NumericModule
//! \brief A number of type \a X, extended with infinite and not-a-number values.
template<class X> class ExtendedOperations {

    typedef FiniteOperations<X> Finite;
    typedef ExtensionOperations<X> Extension;
  public:

    static Bool is_nan(X const& x) { return Extension::is_nan(x); }
    static Bool is_inf(X const& x) { return Extension::is_inf(x); }
    static Bool is_finite(X const& x) { return Extension::is_finite(x); }
    static Bool is_zero(X const& x) { return Extension::is_zero(x); }

    static Void set_nan(X& r) { Extension::set_nan(r); }
    static Void set_inf(X& r, Sign sgn) { Extension::set_inf(r,sgn); }
    static Void set_zero(X& r) { Extension::set_zero(r); }

    static Sign sgn(X const& x) { return Extension::sgn(x); }

    static Sign abs(Sign s) { return (s==Sign::NEGATIVE) ? -s : s; }
    static Sign pow(Sign s, Nat m) { return (s==Sign::NEGATIVE && (m%2==0)) ? Sign::POSITIVE : s; }

    static Void neg(X& r, X const& x) {
        if (is_finite(x)) { Finite::neg(r,x); }
        else { set_inf(r,-sgn(x)); } }

    static Void add(X& r, X const& x1, X const& x2) {
        if (is_finite(x1) and is_finite(x2)) { Finite::add(r, x1,x2); }
        else if(is_finite(x1)) { set_inf(r,sgn(x2)); } else if(is_finite(x2)) { set_inf(r,sgn(x1)); }
        else if(sgn(x1)==sgn(x2)) { set_inf(r,sgn(x1)); } else { set_nan(r); } }

    static Void sub(X& r, X const& x1, X const& x2) {
        if (is_finite(x1) and is_finite(x2)) { Finite::sub(r, x1,x2); }
        else if(is_finite(x1)) { set_inf(r,-sgn(x2)); } else if(is_finite(x2)) { set_inf(r,sgn(x1)); }
        else if(sgn(x1)==(-sgn(x2))) { set_inf(r,sgn(x1)); } else { set_nan(r); } }
    static Void mul(X& r, X const& x1, X const& x2) {
        if (is_finite(x1) and is_finite(x2)) { Finite::mul(r, x1,x2); }
        else { set_inf(r,sgn(x1)*sgn(x2)); } }
    static Void div(X& r, X const& x1, X const& x2) {
        if (is_finite(x1) and is_finite(x2)) { Finite::div(r,x1,x2); } else { rec(r,x2); mul(r,x1,r); } }

    static Void sqr(X& r, X const& x) {
        if (is_finite(x)) { Finite::sqr(r,x); } else { set_inf(r,abs(sgn(x))); } }
    static Void pow(X& r, X const& x, Nat m) {
        if (is_finite(x)) { Finite::pow(r,x,m); } else if (is_nan(x)) { set_nan(r); }
        else { set_inf(r, (sgn(x)==Sign::POSITIVE || (m%2==0)) ? Sign::POSITIVE : Sign::NEGATIVE); } }
    static Void pow(X& r, X const& x, Int n) {
        if (is_finite(x)) { Finite::pow(r,x,n); } else { assert(false); } }

    static Void hlf(X& r, X const& x) {
        if (is_finite(x)) { Finite::hlf(r, x); } else { set_inf(r, sgn(x)); } }
    static Void rec(X& r, X const& x) {
        if (is_finite(x)) { Finite::rec(r,x); } else if(is_inf(x)) { set_zero(r); } else { set_nan(r); } }

    static Void abs(X& r, X const& x) {
        if (is_finite(x)) { Finite::abs(r, x); } else { set_inf(r,abs(sgn(x))); } }
    static Void max(X& r, X const& x1, X const& x2) {
        if (is_finite(x1) and is_finite(x2)) { Finite::max(r, x1,x2); }
        else if (is_nan(x1) or is_nan(x2)) { set_nan(r); }
        else if ((is_inf(x1) and sgn(x1)==Sign::POSITIVE) or (is_inf(x2) and sgn(x2)==Sign::POSITIVE)) { set_inf(r,Sign::POSITIVE); }
        else if (is_inf(x1)) { Finite::set(r,x2); } else { Finite::set(r,x1); } } // As x1==-inf or x2==-inf
    static Void min(X& r, X const& x1, X const& x2) {
        if (is_finite(x1) and is_finite(x2)) { Finite::min(r, x1,x2); }
        else if (is_nan(x1) or is_nan(x2)) { set_nan(r); }
        else if ((is_inf(x1) and sgn(x1)==Sign::NEGATIVE) or (is_inf(x2) and sgn(x2)==Sign::NEGATIVE)) { set_inf(r,Sign::NEGATIVE); }
        else if (is_inf(x1)) { Finite::set(r,x2); } else { Finite::set(r,x1); } } // As x1==+inf or x2==+inf

    static Boolean eq(X const& x1, X const& x2) {
        if(is_finite(x1) and is_finite(x2)) {
            return Finite::eq(x1,x2);
        } else if(is_finite(x1) or is_finite(x2)) {
            return false;
        } else if(is_nan(x1) or is_nan(x2)) {
            return false;
        } else {
            return sgn(x1)==sgn(x2);
        }
    }

    static Comparison cmp(X const& x1, X const& x2) {
        if(is_finite(x1) and is_finite(x2)) {
            return Finite::cmp(x1,x2);
        } else if(is_nan(x1) or is_nan(x2)) {
            return Comparison::INCOMPARABLE;
        } else if(is_finite(x1)) {
            return 0 < (int)sgn(x2) ? Comparison::LESS : Comparison::GREATER;
        } else if(is_finite(x2)) {
            return (int)sgn(x1) > 0 ? Comparison::GREATER : Comparison::LESS;
        } else {
            return cmp((int)sgn(x1),(int)sgn(x2));
        }
    }

    static Comparison cmp(X const& x1, Int const& n2) {
        if(is_finite(x1)) {
            return Finite::cmp(x1,n2);
        } else if(is_nan(x1)) {
            return Comparison::INCOMPARABLE;
        } else {
            return (int)sgn(x1) > 0 ? Comparison::GREATER : Comparison::LESS;
        }
    }

    static OutputStream& write(OutputStream& os, X const& x) {
        if(is_finite(x)) { return os << x; }
        else if(is_nan(x)) { return os << "nan"; }
        else { if(sgn(x)==Sign::NEGATIVE) { os << '-'; } return os << "inf"; }
    }
};

} // namespace Ariadne

#endif
