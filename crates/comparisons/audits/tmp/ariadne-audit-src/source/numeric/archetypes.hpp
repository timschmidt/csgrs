/***************************************************************************
 *            numeric/archetypes.hpp
 *
 *  Copyright  2020  Pieter Collins
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

/*! \file numeric/archetypes.hpp
 *  \brief
 */

#include "../utility/typedefs.hpp"
#include "../foundations/paradigm.hpp"
#include "../numeric/concepts.hpp"

#ifndef ARIADNE_NUMERIC_ARCHETYPES_HPP
#define ARIADNE_NUMERIC_ARCHETYPES_HPP

namespace Ariadne {

class Real;
class TwoExp;

class DoublePrecision;
class MultiplePrecision;

struct DecimalPlaces;
struct DecimalPrecision;

template<class X> class Positive;

class ApproximateDouble;
class ExactDouble;
class Integer;
class Dyadic;
class Decimal;
class Rational;
class DoublePrecision;
struct RoundDownward;
struct RoundUpward;
struct RoundToNearest;
struct RoundApproximately;

template<class P> class Number;
template<class P> class LowerNumber;
template<class P> class UpperNumber;
using ValidatedLowerNumber = LowerNumber<ValidatedTag>;
using ValidatedUpperNumber = UpperNumber<ValidatedTag>;

template<class... PRS> class Float;
using FloatDP=Float<DoublePrecision>;
using FloatMP=Float<MultiplePrecision>;

class RoundedArchetype {
  public:
    class RoundingModeType { public:
        RoundingModeType(RoundDownward); RoundingModeType(RoundUpward); RoundingModeType(RoundToNearest); RoundingModeType(RoundApproximately);
    };
    class PrecisionType { public:
        friend bool operator==(PrecisionType, PrecisionType);
        friend PrecisionType max(PrecisionType, PrecisionType);
    };
  private:
    using X=RoundedArchetype;
    using RND=RoundingModeType;
    using PR=PrecisionType;
  public:
    PrecisionType precision() const;
    RoundedArchetype();
    explicit RoundedArchetype(int);
    explicit RoundedArchetype(int, PrecisionType);
    explicit RoundedArchetype(ApproximateDouble, RoundingModeType, PrecisionType);
    explicit RoundedArchetype(ExactDouble, PrecisionType);
    explicit RoundedArchetype(TwoExp, PrecisionType);
    explicit RoundedArchetype(ExactDouble, RoundingModeType, PrecisionType);
    explicit RoundedArchetype(Integer, RoundingModeType, PrecisionType);
    explicit RoundedArchetype(Dyadic, RoundingModeType, PrecisionType);
    explicit RoundedArchetype(Decimal const&, RoundingModeType, PrecisionType);
    explicit RoundedArchetype(Rational, RoundingModeType, PrecisionType);
    explicit RoundedArchetype(RoundedArchetype, RoundingModeType, PrecisionType);

    static RoundingModeType get_rounding_mode();
    static Void set_rounding_mode(RoundingModeType);

    static X inf(PR);
    static X pi(RND,PR);

    friend X round(X);

    friend X operator+(X);
    friend X operator-(X);

    friend auto nul(X) -> X;
    friend auto pos(X) -> X;
    friend auto neg(X) -> X;
    friend auto hlf(X) -> X;
    friend auto rec(RoundingModeType, X) -> X;
    friend auto add(RoundingModeType, X, X) -> X;
    friend auto sub(RoundingModeType, X, X) -> X;
    friend auto mul(RoundingModeType, X, X) -> X;
    friend auto div(RoundingModeType, X, X) -> X;
    friend auto fma(RoundingModeType, X, X, X) -> X;
    friend auto pow(RoundingModeType, X, Int) -> X;

    friend auto med(RoundToNearest, X, X) -> X;
/*
    friend auto sqrt(RND, X) -> X;
    friend auto exp(RND, X) -> X;
    friend auto log(RND, X) -> X;
    friend auto sin(RND, X) -> X;
    friend auto cos(RND, X) -> X;
    friend auto tan(RND, X) -> X;
*/
    friend auto atan(RND, X) -> X;
    friend auto abs(X) -> X;
    friend auto max(X,X) -> X;
    friend auto min(X,X) -> X;

    friend bool operator<=(X, X);
    friend bool operator> (X, X);
    friend bool operator>=(X, X);

    friend bool operator==(X, Int);
    friend bool operator< (X, Int);
    friend bool operator<=(X, Int);
    friend bool operator> (X, Int);
    friend bool operator>=(X, Int);

    friend OutputStream& operator<<(OutputStream&, X const&);
    double get_d() const;

};
OutputStream& write(OutputStream&, RoundedArchetype,DecimalPrecision,RoundedArchetype::RoundingModeType);
OutputStream& write(OutputStream&, RoundedArchetype,DecimalPlaces,RoundedArchetype::RoundingModeType);

auto get_lower(Real const& r, RoundedArchetype::PrecisionType pr) -> RoundedArchetype;
auto get_lower(ValidatedLowerNumber const& y, RoundedArchetype::PrecisionType pr) -> RoundedArchetype;
auto get_upper(Real const& r, RoundedArchetype::PrecisionType pr) -> RoundedArchetype;
auto get_upper(ValidatedUpperNumber const& y, RoundedArchetype::PrecisionType pr) -> RoundedArchetype;
auto get_approx(Real const& r, RoundedArchetype::PrecisionType pr) -> RoundedArchetype;
auto get_approx(ValidatedLowerNumber const& y, RoundedArchetype::PrecisionType pr) -> RoundedArchetype;

RoundedArchetype cast_raw_float(RoundedArchetype::PrecisionType);

template<class PR> struct RawFloatTypedef;
template<> struct RawFloatTypedef<RoundedArchetype::PrecisionType> { typedef RoundedArchetype Type; };

static_assert(IsRoundedField<RoundedArchetype>);
//template<class X> concept Rounded = RoundedMode<X,typename X::RoundingModeType>;

} // namespace Ariadne

#endif
