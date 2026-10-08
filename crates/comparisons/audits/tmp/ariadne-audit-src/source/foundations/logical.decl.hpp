/***************************************************************************
 *            foundations/logical.decl.hpp
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

/*! \file foundations/logical.decl.hpp
 *  \brief
 */



#ifndef ARIADNE_LOGICAL_DECL_HPP
#define ARIADNE_LOGICAL_DECL_HPP

#include "foundations/paradigm.hpp"

namespace Ariadne {

typedef void Void;
typedef bool Bool;

class Indeterminate;
extern const Indeterminate indeterminate;

class Boolean;
class Sierpinskian;
class Kleenean;

template<class L> class Negate;
template<class L> class Lower;
template<class L> class Upper;
template<class L> class Validated;
template<class L> class Approximate;

class NegatedSierpinskian;
class LowerKleenean;
class UpperKleenean;
class ValidatedSierpinskian;
class ValidatedNegatedSierpinskian;
class ValidatedKleenean;
class ValidatedLowerKleenean;
class ValidatedUpperKleenean;
class ApproximateKleenean;

using Fuzzy = ApproximateKleenean;

template<class P> struct LogicalTypedef;
template<class P> using LogicalType = typename LogicalTypedef<P>::Type;
template<> struct LogicalTypedef<ExactTag> { typedef Boolean Type; };
template<> struct LogicalTypedef<EffectiveTag> { typedef Kleenean Type; };
template<> struct LogicalTypedef<ValidatedTag> { typedef ValidatedKleenean Type; };
template<> struct LogicalTypedef<ApproximateTag> { typedef ApproximateKleenean Type; };

template<class P> struct LowerLogicalTypedef;
template<class P> using LowerLogicalType = typename LowerLogicalTypedef<P>::Type;
template<> struct LowerLogicalTypedef<EffectiveTag> { typedef LowerKleenean Type; };
template<> struct LowerLogicalTypedef<ValidatedTag> { typedef ValidatedLowerKleenean Type; };
template<> struct LowerLogicalTypedef<ApproximateTag> { typedef ApproximateKleenean Type; };

template<class P> struct UpperLogicalTypedef;
template<class P> using UpperLogicalType = typename UpperLogicalTypedef<P>::Type;
template<> struct UpperLogicalTypedef<EffectiveTag> { typedef UpperKleenean Type; };
template<> struct UpperLogicalTypedef<ValidatedTag> { typedef ValidatedUpperKleenean Type; };
template<> struct UpperLogicalTypedef<ApproximateTag> { typedef ApproximateKleenean Type; };

template<class P> using KleeneanType = LogicalType<P>;
template<class P> using LowerKleeneanType = LowerLogicalType<P>;
template<class P> using UpperKleeneanType = UpperLogicalType<P>;


using Decidable = Boolean;
using Quasidecidable = Kleenean;
using Verifyable = Sierpinskian;
using Falsifyable = NegatedSierpinskian;

Boolean operator!(Boolean const&);
NegatedSierpinskian operator!(Sierpinskian const&);
Kleenean operator!(Kleenean const&);


template<class P> struct ApartnessTraits;
template<class P> using ApartnessType = typename ApartnessTraits<P>::Type;
template<> struct ApartnessTraits<ExactTag> { typedef Boolean Type; };
template<> struct ApartnessTraits<EffectiveTag> { typedef Sierpinskian Type; };
template<> struct ApartnessTraits<ValidatedTag> { typedef ValidatedSierpinskian Type; };
template<> struct ApartnessTraits<ApproximateTag> { typedef ApproximateKleenean Type; };

template<class P> using EqualityLogicalType = decltype(not declval<ApartnessType<P>>());
template<class P> using InequalityLogicalType = ApartnessType<P>;


namespace Detail {

#if (defined __arm || defined __aarch64__)
typedef short ComparableEnumerationType;
#else
typedef char ComparableEnumerationType;
#endif

enum class LogicalValue : ComparableEnumerationType;
}
using Detail::LogicalValue;

}

#endif
