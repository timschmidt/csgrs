/***************************************************************************
 *            function/polynomial.cpp
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

#include "numeric/numeric.hpp"
#include "config.hpp"

#include "geometry/interval.hpp"
#include "function/polynomial.hpp"
#include "function/polynomial.tpl.hpp"

namespace Ariadne {

template class Polynomial<UniIndex,RoundedFloatDP>;
template class Polynomial<UniIndex,FloatDPApproximation>;
template class Polynomial<UniIndex,FloatDPBounds>;
template class Polynomial<UniIndex,FloatMPApproximation>;
template class Polynomial<UniIndex,FloatMPBounds>;

template struct AlgebraOperations<Polynomial<UniIndex,RoundedFloatDP>>;
template struct AlgebraOperations<Polynomial<UniIndex,FloatDPApproximation>>;
template struct AlgebraOperations<Polynomial<UniIndex,FloatDPBounds>>;
template struct AlgebraOperations<Polynomial<UniIndex,FloatMPApproximation>>;
template struct AlgebraOperations<Polynomial<UniIndex,FloatMPBounds>>;

template class Polynomial<MultiIndex,RoundedFloatDP>;
template class Polynomial<MultiIndex,FloatDPApproximation>;
template class Polynomial<MultiIndex,FloatDPBounds>;
template class Polynomial<MultiIndex,FloatDPUpperInterval>;

template struct AlgebraOperations<Polynomial<MultiIndex,RoundedFloatDP>>;
template struct AlgebraOperations<Polynomial<MultiIndex,FloatDPApproximation>>;
template struct AlgebraOperations<Polynomial<MultiIndex,FloatDPBounds>>;
template struct AlgebraOperations<Polynomial<MultiIndex,FloatDPUpperInterval>>;

template<> Void Polynomial<MultiIndex,FloatDP>::cleanup() { }

template Expansion<MultiIndex,FloatDP>& MultivariatePolynomial<FloatDP>::expansion();
template OutputStream& Polynomial<MultiIndex,FloatDP>::_write(OutputStream&) const;
template OutputStream& Polynomial<MultiIndex,FloatDP>::_write(OutputStream&, Array<String> const&) const;


template class Polynomial<MultiIndex,FloatMPApproximation>;
template class Polynomial<MultiIndex,FloatMPBounds>;
template class Polynomial<MultiIndex,FloatMPUpperInterval>;
template struct AlgebraOperations<Polynomial<MultiIndex,FloatMPApproximation>>;
template struct AlgebraOperations<Polynomial<MultiIndex,FloatMPBounds>>;
template struct AlgebraOperations<Polynomial<MultiIndex,FloatMPUpperInterval>>;

} //namespace Ariadne


