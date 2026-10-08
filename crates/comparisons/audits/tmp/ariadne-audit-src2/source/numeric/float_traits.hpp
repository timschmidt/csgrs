/***************************************************************************
 *            numeric/float_traits.hpp
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

/*! \file numeric/float_traits.hpp
 *  \brief Type traits for floating-point numbers.
 */

#ifndef ARIADNE_FLOAT_TRAITS_HPP
#define ARIADNE_FLOAT_TRAITS_HPP

#include "float.decl.hpp"

namespace Ariadne {

template<> struct GenericTrait<FloatDP> { typedef ExactNumber Type; };
template<> struct GenericTrait<FloatMP> { typedef ExactNumber Type; };
template<class F> struct GenericTrait<Approximation<F>> { typedef ApproximateNumber Type; };
template<class F> struct GenericTrait<LowerBound<F>> { typedef ValidatedLowerNumber Type; };
template<class F> struct GenericTrait<UpperBound<F>> { typedef ValidatedUpperNumber Type; };
template<class F> struct GenericTrait<Bounds<F>> { typedef ValidatedNumber Type; };
template<class F, class FE> struct GenericTrait<Ball<F,FE>> { typedef ValidatedNumber Type; };
template<class F> struct GenericTrait<Error<F>> { typedef PositiveValidatedUpperNumber Type; };

template<> struct CharacteristicsTrait<FloatDP> { typedef DP Type; };
template<> struct CharacteristicsTrait<FloatMP> { typedef MP Type; };
template<class F> struct CharacteristicsTrait<Approximation<F>> { typedef CharacteristicsType<F> Type; };
template<class F> struct CharacteristicsTrait<LowerBound<F>> { typedef CharacteristicsType<F> Type; };
template<class F> struct CharacteristicsTrait<UpperBound<F>> { typedef CharacteristicsType<F> Type; };
template<class F> struct CharacteristicsTrait<Bounds<F>> { typedef CharacteristicsType<F> Type; };
template<class F, class FE> struct CharacteristicsTrait<Ball<F,FE>> { typedef Pair<CharacteristicsType<F>,CharacteristicsType<FE>> Type; };
template<class F> struct CharacteristicsTrait<Error<F>> { typedef CharacteristicsType<F> Type; };
template<class F> struct CharacteristicsTrait<Rounded<F>> { typedef CharacteristicsType<F> Type; };

template<class F> struct ConcreteTraits<Rounded<F>> {
    typedef DP CharacteristicsType;
};

template<> struct ConcreteTraits<FloatDP> {
    typedef ExactNumber GenericType;
    typedef DP CharacteristicsType;
};

template<> struct ConcreteTraits<FloatMP> {
    typedef ExactNumber GenericType;
    typedef MP CharacteristicsType;
};

template<class F> struct ConcreteTraits<Approximation<F>> {
    typedef ApproximateNumber GenericType;
    typedef Ariadne::CharacteristicsType<F> CharacteristicsType;
};

template<class F> struct ConcreteTraits<LowerBound<F>> {
    typedef ValidatedLowerNumber GenericType;
    typedef Ariadne::CharacteristicsType<F> CharacteristicsType;
};

template<class F> struct ConcreteTraits<UpperBound<F>> {
    typedef ValidatedUpperNumber GenericType;
    typedef Ariadne::CharacteristicsType<F> CharacteristicsType;
};

template<class F> struct ConcreteTraits<Bounds<F>> {
    typedef ValidatedNumber GenericType;
    typedef Ariadne::CharacteristicsType<F> CharacteristicsType;
};

template<class F, class FE> struct ConcreteTraits<Ball<F,FE>> {
    typedef ValidatedNumber GenericType;
    typedef Pair<Ariadne::CharacteristicsType<F>,Ariadne::CharacteristicsType<FE>> CharacteristicsType;
};

template<class F> struct ConcreteTraits<Error<F>> {
    typedef PositiveValidatedUpperNumber GenericType;
    typedef Ariadne::CharacteristicsType<F> CharacteristicsType;
};

template<> struct NumericTraits<FloatDP> {
    using F=FloatDP;
    typedef F OppositeType;
    typedef Positive<F> PositiveType;
    typedef Boolean LessType;
    typedef Boolean EqualsType;
};

template<> struct NumericTraits<FloatMP> {
    using F=FloatMP;
    typedef F OppositeType;
    typedef Positive<F> PositiveType;
    typedef Boolean LessType;
    typedef Boolean EqualsType;
};

template<class F> struct NumericTraits<Approximation<F>> {
    typedef Positive<Approximation<F>> PositiveType;
    typedef Approximation<F> OppositeType;
    typedef Fuzzy LessType;
    typedef Fuzzy EqualsType;
};

template<class F> struct NumericTraits<LowerBound<F>> {
    typedef UpperBound<F> OppositeType;
    typedef Positive<LowerBound<F>> PositiveType;
    typedef ValidatedUpperKleenean LessType;
    typedef ValidatedNegatedSierpinskian EqualsType;
};

template<class F> struct NumericTraits<UpperBound<F>> {
    typedef LowerBound<F> OppositeType;
    typedef Positive<UpperBound<F>> PositiveType;
    typedef ValidatedLowerKleenean LessType;
    typedef ValidatedNegatedSierpinskian EqualsType;
};

template<class F> struct NumericTraits<Bounds<F>> {
    typedef Bounds<F> OppositeType;
    typedef Positive<Bounds<F>> PositiveType;
    typedef ValidatedKleenean LessType;
    typedef ValidatedKleenean EqualsType;
};

template<class F, class FE> struct NumericTraits<Ball<F,FE>> {
    typedef Ball<F,FE> OppositeType;
    typedef Positive<Ball<F,FE>> PositiveType;
    typedef ValidatedKleenean LessType;
    typedef ValidatedKleenean EqualsType;
};

template<class F> struct NumericTraits<Error<F>> {
    typedef Error<F> PositiveType;
    typedef Positive<LowerBound<F>> OppositeType;
    typedef ValidatedLowerKleenean LessType;
    typedef ValidatedNegatedSierpinskian EqualsType;
};

} // namespace Ariadne

#endif
