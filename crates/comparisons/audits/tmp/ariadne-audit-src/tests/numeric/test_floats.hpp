/***************************************************************************
 *            test_floats.hpp
 *
 *  Copyright  2006-20  Alberto Casagrande, Pieter Collins
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

#include <cassert>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>

#include "config.hpp"
#include "numeric/builtin.hpp"
#include "numeric/integer.hpp"
#include "numeric/dyadic.hpp"
#include "numeric/decimal.hpp"
#include "numeric/rational.hpp"
#include "numeric/float.decl.hpp"

#include "../test.hpp"

using namespace Ariadne;
using namespace std;

namespace Ariadne {

template<class F> Bool models(LowerBound<F> x, Rational q) { return x.raw() <= q; }
template<class F> Bool models(UpperBound<F> x, Rational q) { return x.raw() >= q; }
template<class F> Bool models(Bounds<F> x, Rational q) { return x.lower_raw() <= q and x.upper_raw() >= q; }
template<class F> Bool models(Ball<F> x, Rational q) { return x.error_raw() >= abs(Rational(x.value_raw())-q); }

template<class F, class FE> Bool models(Ball<F,FE> x, Rational y) {
    return abs(Dyadic(x.value())-y) <= Dyadic(x.error_raw()); }
template<class F, class FE> Bool models(Ball<F,FE> x, F v) {
    return abs(Dyadic(x.value())-Dyadic(v)) <= Dyadic(x.error_raw()); }


template<> String class_name<DoublePrecision>() { return "DoublePrecision"; }
template<> String class_name<MultiplePrecision>() { return "MultiplePrecision"; }

inline bool same(LogicalValue l1, LogicalValue l2) {
    return static_cast<uchar>(l1) == static_cast<uchar>(l2); }
inline bool same(ApproximateKleenean ak1, ApproximateKleenean ak2) {
    return same(reinterpret_cast<LogicalValue const&>(ak1),reinterpret_cast<LogicalValue const&>(ak2)); }

} // namespace Ariadne


template<class PR>
class TestFloats
{
    typedef double Double;
  protected:
    Nat m; Int n; Double ad; ExactDouble ed;
    Integer z; Dyadic w; Decimal d; Rational q; Real r;
  public:
    TestFloats();
  public:
    static Rational to_rational(FloatType<ApproximateTag,PR> x) { return Rational(x.raw()); }
    static Rational to_rational(FloatType<LowerTag,PR> x) { return Rational(x.raw()); }
    static Rational to_rational(FloatType<UpperTag,PR> x) { return Rational(x.raw()); }
    static Rational to_rational(FloatType<ExactTag,PR> x) { return Rational(x.raw()); }
};

template<class PR>
TestFloats<PR>::TestFloats()
    : m(1), n(1), ad(1.0), ed(1.0), z(1), w(1), d(1.0), q(1), r(1)
{
}
