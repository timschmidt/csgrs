/***************************************************************************
 *            numeric/builtin.hpp
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

/*! \file numeric/builtin.hpp
 *  \brief Inclusion header for wrapper for builtin numbers.
 */

#ifndef ARIADNE_BUILTIN_HPP
#define ARIADNE_BUILTIN_HPP

#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>

#include "utility/metaprogramming.hpp"
#include "utility/macros.hpp"
#include "numeric/number.decl.hpp"

#include "foundations/logical.hpp"
#include "twoexp.hpp"

namespace Ariadne {

struct ExactTag;
enum class Comparison : ComparableEnumerationType;

template<class D> struct HasGetD {
    template<class DD, class=decltype(std::declval<DD>().get_d())> static std::true_type test(int);
    template<class DD> static std::false_type test(...);
    static const bool value = decltype(test<D>(1))::value;
};

//! \ingroup NumericModule
//! \brief A wrapper around a builtin double-precision floating-point number,
//! indicating that the stored value is an \em approximation to the value of a real quantity.
class ApproximateDouble {
    double _d;
  public:
    typedef ApproximateTag Paradigm;
    typedef ApproximateDouble NumericType;
    ApproximateDouble() : _d() { }
    ApproximateDouble(double d) : _d(d) { }
    ApproximateDouble(ExactDouble const& x);
    ApproximateDouble(Real const& r0);
    explicit operator double() const { return _d; }
    double get_d() const { return this->_d; }
    friend ApproximateDouble operator"" _a (long double lx) { double x=lx; return ApproximateDouble(x); }
    friend OutputStream& operator<<(OutputStream& os, ApproximateDouble x) { return os << x._d; }

    friend ApproximateDouble nul(ApproximateDouble x) { return ApproximateDouble(0.0); }
    friend ApproximateDouble pos(ApproximateDouble x) { return ApproximateDouble(+x._d); }
    friend ApproximateDouble neg(ApproximateDouble x) { return ApproximateDouble(-x._d); }
    friend ApproximateDouble add(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d+x2._d); }
    friend ApproximateDouble sub(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d-x2._d); }
    friend ApproximateDouble mul(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d*x2._d); }
    friend ApproximateDouble div(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d/x2._d); }
    friend ApproximateDouble abs(ApproximateDouble x) { return ApproximateDouble(x._d>=0?x._d:-x._d); }
    friend ApproximateDouble max(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d>=x2._d?x1._d:x2._d); }
    friend ApproximateDouble min(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d<=x2._d?x1._d:x2._d); }
    friend ApproximateDouble mag(ApproximateDouble x) { return abs(x); }
    friend ApproximateDouble mig(ApproximateDouble x) { return abs(x); }

    friend ApproximateDouble operator+(ApproximateDouble x) { return ApproximateDouble(+x._d); }
    friend ApproximateDouble operator-(ApproximateDouble x) { return ApproximateDouble(-x._d); }
    friend ApproximateDouble operator+(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d+x2._d); }
    friend ApproximateDouble operator-(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d-x2._d); }
    friend ApproximateDouble operator*(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d*x2._d); }
    friend ApproximateDouble operator/(ApproximateDouble x1, ApproximateDouble x2) { return ApproximateDouble(x1._d/x2._d); }
    friend ApproximateDouble operator*(ApproximateDouble x, TwoExp p) { return ApproximateDouble(x.get_d()*p.get_d()); }
    friend ApproximateDouble operator/(ApproximateDouble x, TwoExp p) { return ApproximateDouble(x.get_d()/p.get_d()); }
    friend ApproximateDouble& operator+=(ApproximateDouble& x1, ApproximateDouble x2) { x1._d+=x2._d; return x1; }
    friend ApproximateDouble& operator-=(ApproximateDouble& x1, ApproximateDouble x2) { x1._d-=x2._d; return x1; }
    friend ApproximateDouble& operator*=(ApproximateDouble& x1, ApproximateDouble x2) { x1._d*=x2._d; return x1; }
    friend ApproximateDouble& operator/=(ApproximateDouble& x1, ApproximateDouble x2) { x1._d/=x2._d; return x1; }

    friend Bool same(ApproximateDouble x1, ApproximateDouble x2) { return x1._d==x2._d; }
    friend ApproximateKleenean operator==(ApproximateDouble x1, ApproximateDouble x2) { return x1._d==x2._d; }
    friend ApproximateKleenean operator!=(ApproximateDouble x1, ApproximateDouble x2) { return x1._d!=x2._d; }
    friend ApproximateKleenean operator> (ApproximateDouble x1, ApproximateDouble x2) { return x1._d> x2._d; }
    friend ApproximateKleenean operator< (ApproximateDouble x1, ApproximateDouble x2) { return x1._d< x2._d; }
    friend ApproximateKleenean operator>=(ApproximateDouble x1, ApproximateDouble x2) { return x1._d>=x2._d; }
    friend ApproximateKleenean operator<=(ApproximateDouble x1, ApproximateDouble x2) { return x1._d<=x2._d; }
};

//! \ingroup NumericModule
//! \brief Indicate that a floating-point literal only represents an double-precision approximation to a number.
inline ApproximateDouble operator"" _a (long double lx);


//! \ingroup NumericModule
//! \brief A wrapper around a builtin double-precision floating-point number,
//! indicating that the stored value is the \em exact value of a real quantity.
class ExactDouble {
    double _d;
  public:
    typedef ExactTag Paradigm;
    double get_d() const { return this->_d; }
    ExactDouble() : _d() { }
    template<BuiltinIntegral N> ExactDouble(N n) : _d(n) { assert(_d==n); }
    template<BuiltinFloatingPoint X> explicit ExactDouble(X const& x) : _d(x) { assert(std::isnan(_d) || (_d==x)); }
    static ExactDouble infinity() { return ExactDouble(std::numeric_limits<double>::infinity()); }
    operator ExactNumber() const;
    friend ExactDouble nul(ExactDouble x) { return ExactDouble(0.0); }
    friend ExactDouble abs(ExactDouble x) { return ExactDouble(std::abs(x._d)); }
    static ExactDouble inf() { return ExactDouble(std::numeric_limits<double>::infinity()); }
    static ExactDouble nan() { return ExactDouble(std::numeric_limits<double>::quiet_NaN()); }
    friend Bool is_finite(ExactDouble x) { return std::isfinite(x._d); }
    friend ExactDouble operator+(ExactDouble x) { return ExactDouble(+x._d); }
    friend ExactDouble operator-(ExactDouble x) { return ExactDouble(-x._d); }
    friend ApproximateDouble operator+(ApproximateDouble x1, ApproximateDouble x2);
    friend ApproximateDouble operator-(ApproximateDouble x1, ApproximateDouble x2);
    friend ApproximateDouble operator*(ApproximateDouble x1, ApproximateDouble x2);
    friend ApproximateDouble operator/(ApproximateDouble x1, ApproximateDouble x2);
    friend ExactDouble operator*(ExactDouble x, TwoExp p) { return ExactDouble(x.get_d()*p.get_d()); }
    friend ExactDouble operator/(ExactDouble x, TwoExp p) { return ExactDouble(x.get_d()/p.get_d()); }
    friend Comparison cmp(ExactDouble const& x1, ExactDouble const& x2) {
        return Comparison( (x1._d==x2._d) ? 0 : (x1._d<x2._d) ? -1 : +1 ); }
    friend Boolean operator==(ExactDouble const& x1, ExactDouble const& x2) { return x1._d==x2._d; }
    friend Boolean operator!=(ExactDouble const& x1, ExactDouble const& x2) { return x1._d!=x2._d; }
    friend Boolean operator>=(ExactDouble const& x1, ExactDouble const& x2) { return x1._d>=x2._d; }
    friend Boolean operator<=(ExactDouble const& x1, ExactDouble const& x2) { return x1._d<=x2._d; }
    friend Boolean operator> (ExactDouble const& x1, ExactDouble const& x2) { return x1._d> x2._d; }
    friend Boolean operator< (ExactDouble const& x1, ExactDouble const& x2) { return x1._d< x2._d; }
    friend ExactDouble operator"" _x (long double lx) { double x=lx; ARIADNE_ASSERT_MSG(x==lx,"The value "<<lx<<" should be exactly representable as a double."); return ExactDouble(x); }
    friend ExactDouble operator"" _pr (long double lx) { double x=lx; return ExactDouble(x); }
    friend OutputStream& operator<<(OutputStream& os, ExactDouble x) { return os << std::setprecision(18) << x.get_d(); }
};

//! \ingroup ExtendedLiteralsSubModule
//! \brief Indicate that a floating-point literal is the exact value of a number in double-precision.
//! \details The input \a lx is converted first converted to a double-precision value \a x.
//! If \a lx and \a x differ, then the decimal literal input to \a lx almost certainly does not represent a double-precision number exactly, and an assertion fails. <p/>
//! For example, <c>0.625_x</c> yields the exact double-precision value \f$5/2^3\f$, but <c>0.6_x</c> fails, since \f$3/5\f$ is not exactly representable as a double-precision number.
ExactDouble operator"" _x(long double lx);

//! \ingroup ExtendedLiteralsSubModule
//! \brief Indicate that a floating-point literal is sufficiently precise that it can safely be taken to represent the closest double-precision floating-point number.
inline ExactDouble exact(long double lx) { return operator""_x(lx); }


//! \ingroup ExtendedLiteralsSubModule
//! \brief Indicate that a floating-point literal is sufficiently precise that it can safely be taken to represent the closest double-precision floating-point number.
//! \details For example, writing <c>0.14285714285714285_pr</c> claims that the exact decimal "0.14285714285714285" should be interpreted as the closest double-precision floating-point number, which has exact value \f$0.142857142857142849212692681248881854116916656494140625\f$.
inline ExactDouble operator"" _pr (long double lx);


#ifdef DOXYGRN
//! \ingroup ExtendedLiteralsSubModule
//! \brief Indicate that a floating-point literal is the exact value of a number in double-precision. (Python-only syntax)
ExactDouble x_(long double lx);

//! \ingroup ExtendedLiteralsSubModule
//! \brief Indicate that a floating-point literal is sufficiently precise that it can safely be taken to represent the closest double-precision floating-point number. (Python-only syntax)
ExactDouble pr_(long double lx);
#endif // DOXYGEN

inline ExactDouble cast_exact(double d) { return ExactDouble(d); }
inline ExactDouble cast_exact(ApproximateDouble ax) { return ExactDouble(ax.get_d()); }

static const ExactDouble inf = ExactDouble(std::numeric_limits<double>::infinity());



template<class X> class Positive;
template<> class Positive<ExactDouble> : public ExactDouble {
  public:
    explicit Positive(ExactDouble x) : ExactDouble(x) { }
};
inline Positive<ExactDouble> cast_positive(ExactDouble x) { return Positive<ExactDouble>(x); }

inline ApproximateDouble::ApproximateDouble(ExactDouble const& x) : _d(x.get_d()) { }

} // namespace Ariadne

#endif
