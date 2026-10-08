/***************************************************************************
 *            utility/declarations.hpp
 *
 *  Copyright  2011-20  Pieter Collins
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

/*! \file utility/declarations.hpp
 *  \brief Forward declarations of types and classes.
 */

#ifndef ARIADNE_DECLARATIONS_HPP
#define ARIADNE_DECLARATIONS_HPP

#include <iosfwd>

#include "utility/metaprogramming.hpp"
#include "utility/typedefs.hpp"

#include "foundations/paradigm.hpp"
#include "foundations/logical.decl.hpp"
#include "numeric/number.decl.hpp"
#include "numeric/float.decl.hpp"

#include "algebra/linear_algebra.decl.hpp"
#include "algebra/differential.decl.hpp"

#include "function/function.decl.hpp"

#include "geometry/interval.decl.hpp"
#include "geometry/box.decl.hpp"

namespace Ariadne {

template<class X> struct InformationTypedef;
template<> struct InformationTypedef<Real> { typedef EffectiveTag Type; };
template<class P> struct InformationTypedef<Number<P>> { typedef P Type; };
template<class X> using InformationTag = typename InformationTypedef<X>::Type;

template<class P, class F> class AffineModel;
template<class P, class F> class TaylorModel;
template<class X> class Formula;
template<class X> class Algebra;
template<class X> class ElementaryAlgebra;

template<class X> class Point;

} // namespace Ariadne

#endif
