/***************************************************************************
 *            algebra/expansion.cpp
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

#include "numeric/float.decl.hpp"
#include "numeric/floats.hpp"
#include "numeric/rounded_float.hpp"
#include "geometry/interval.hpp"

#include "expansion.hpp"
#include "expansion.tpl.hpp"

#include <utility>
#include <tuple>

namespace Ariadne {
    template<> struct CharacteristicsTrait<double> { typedef std::tuple<> Type; };

    DP characteristics(FloatDP flt) { return flt.precision(); }
    MP characteristics(FloatMP flt) { return flt.precision(); }

    static_assert(Same<CharacteristicsType<FloatDP>,DP>);
    static_assert(Same<CharacteristicsType<RoundedFloatDP>,DP>);

    static_assert(HasCharacteristics<RoundedFloatDP>);
    static_assert(HasCharacteristics<RoundedFloatMP>);

    template class Expansion<UniIndex,FloatDP>;
    template class Expansion<UniIndex,RoundedFloatDP>;
    template class Expansion<UniIndex,FloatDPApproximation>;

    template class Expansion<MultiIndex,double>;
    template class Expansion<MultiIndex,ExactDouble>;
    template class Expansion<MultiIndex,Dyadic>;

    template class Expansion<MultiIndex,RoundedFloatDP>;
    template class Expansion<MultiIndex,FloatDP>;
    template class Expansion<MultiIndex,FloatDPBounds>;
    template class Expansion<MultiIndex,FloatDPApproximation>;

    template class SortedExpansion<MultiIndex,RoundedFloatDP,GradedIndexLess>;
    template class SortedExpansion<MultiIndex,FloatDP,GradedIndexLess>;
    template class SortedExpansion<MultiIndex,FloatDPApproximation,GradedIndexLess>;
    template class SortedExpansion<MultiIndex,FloatDPBounds,GradedIndexLess>;

    template class SortedExpansion<MultiIndex,RoundedFloatDP,ReverseLexicographicIndexLess>;
    template class SortedExpansion<MultiIndex,FloatDP,ReverseLexicographicIndexLess>;
    template class SortedExpansion<MultiIndex,FloatDPBounds,ReverseLexicographicIndexLess>;
    template class SortedExpansion<MultiIndex,FloatDPApproximation,ReverseLexicographicIndexLess>;

    template class Expansion<MultiIndex,FloatDPUpperInterval>;
    template class SortedExpansion<MultiIndex,FloatDPUpperInterval,GradedIndexLess>;
    template class SortedExpansion<MultiIndex,FloatDPUpperInterval,ReverseLexicographicIndexLess>;


    template class Expansion<MultiIndex,RoundedFloatMP>;
    template class Expansion<MultiIndex,FloatMP>;
    template class Expansion<MultiIndex,FloatMPBounds>;
    template class Expansion<MultiIndex,FloatMPApproximation>;

    template class SortedExpansion<MultiIndex,FloatMP,GradedIndexLess>;
    template class SortedExpansion<MultiIndex,FloatMPApproximation,GradedIndexLess>;
    template class SortedExpansion<MultiIndex,FloatMPBounds,GradedIndexLess>;

    template class SortedExpansion<MultiIndex,FloatMP,ReverseLexicographicIndexLess>;
    template class SortedExpansion<MultiIndex,FloatMPBounds,ReverseLexicographicIndexLess>;
    template class SortedExpansion<MultiIndex,FloatMPApproximation,ReverseLexicographicIndexLess>;

    template class Expansion<MultiIndex,FloatMPUpperInterval>;
    template class SortedExpansion<MultiIndex,FloatMPUpperInterval,GradedIndexLess>;
    template class SortedExpansion<MultiIndex,FloatMPUpperInterval,ReverseLexicographicIndexLess>;

    // FIXME: Put in accessible header file
    template Void Expansion<MultiIndex,FloatDPUpperInterval>::_fill(Expansion<MultiIndex,FloatDPBounds> const&);
    template Void Expansion<MultiIndex,FloatDPApproximation>::_fill(Expansion<MultiIndex,FloatDPBounds> const&);

    template Void Expansion<MultiIndex,FloatMPUpperInterval>::_fill(Expansion<MultiIndex,FloatMPBounds> const&);
    template Void Expansion<MultiIndex,FloatMPApproximation>::_fill(Expansion<MultiIndex,FloatMPBounds> const&);
}
