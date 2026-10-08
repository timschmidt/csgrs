/***************************************************************************
 *            numeric/float_factory.hpp
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

/*! \file numeric/float_factory.hpp
 *  \brief Factories for creating floating-point numbers.
 */

#ifndef ARIADNE_FLOAT_FACTORY_HPP
#define ARIADNE_FLOAT_FACTORY_HPP

#include "float.decl.hpp"

namespace Ariadne {

template<class PR> class FloatFactory {
  protected:
    PR _pr; typedef RawFloatType<PR> F;
  public:
    typedef PR PrecisionType;
    typedef PR CharacteristicsType;
    FloatFactory(PR const& pr) : _pr(pr) { }
    PR precision() const { return this->_pr; }
    PR characteristics() const { return this->_pr; }
  public:
    FloatApproximation<PR> create(ApproximateNumber const& y);
    FloatLowerBound<PR> create(ValidatedLowerNumber const& y);
    FloatUpperBound<PR> create(ValidatedUpperNumber const& y);
    FloatBounds<PR> create(ValidatedNumber const& y);
    FloatBounds<PR> create(EffectiveNumber const& y);
    FloatBounds<PR> create(ExactNumber const& y);
    FloatBounds<PR> create(Real const& y);
    FloatBounds<PR> create(Rational const& y);
    FloatBounds<PR> create(Dyadic const& y);
    FloatBounds<PR> create(Integer const& y);
    Float<PR> create(ExactDouble const& y);
    Float<PR> create(Dyadic const& y, ExactTag);
    Float<PR> create(Integer const& y, ExactTag);
    template<BuiltinSignedIntegral N> Float<PR> create(N const& y);
    template<BuiltinUnsignedIntegral M> PositiveFloat<PR> create(M const& y);
    template<BuiltinFloatingPoint D> FloatApproximation<PR> create(D const& y);
    PositiveFloatApproximation<PR> create(PositiveApproximateNumber const& y);
    PositiveFloatLowerBound<PR> create(PositiveValidatedLowerNumber const& y);
    PositiveFloatUpperBound<PR> create(PositiveValidatedUpperNumber const& y);
    PositiveFloatBounds<PR> create(PositiveValidatedNumber const& y);
};

template<class PR, class PRE> class FloatBallFactory : public FloatFactory<PR> {
    PRE _pre;
  public:
    FloatBallFactory(PR const& pr, PRE const& pre) : FloatFactory<PR>(pr), _pre(pre) { }
    using FloatFactory<PR>::create;
    FloatBall<PR,PRE> create(Number<ValidatedTag> const& y);
    FloatBall<PR,PRE> create(Number<EffectiveTag> const& y);
    FloatBall<PR,PRE> create(Number<ExactTag> const& y);
    FloatBall<PR,PRE> create(Real const& y);
    FloatBall<PR,PRE> create(Rational const& y);
    FloatBall<PR,PRE> create(Dyadic const& y);
    FloatBall<PR,PRE> create(Integer const& y);
    PositiveFloatBall<PR,PRE> create(PositiveNumber<ValidatedTag> const& y);
};

template<class Y, class PR> using ConcreteType = decltype(declval<FloatFactory<PR>>().create(declval<Y>()));
template<class Y, class PR> inline decltype(auto) make_float(Y const& y, PR pr) { return float_factory(pr).create(y); }

template<class PR> inline FloatFactory<PR> float_factory(PR pr) { return FloatFactory<PR>(pr); }
template<class F> inline FloatFactory<PrecisionType<F>> factory(Approximation<F> const& flt);
template<class F> inline FloatFactory<PrecisionType<F>> factory(LowerBound<F> const& flt);
template<class F> inline FloatFactory<PrecisionType<F>> factory(UpperBound<F> const& flt);
template<class F> inline FloatFactory<PrecisionType<F>> factory(Bounds<F> const& flt);
template<class F, class FE> inline FloatBallFactory<PrecisionType<F>,PrecisionType<FE>> factory(Ball<F,FE> const& flt);
template<class F> requires Same<F,FloatDP> or Same<F,FloatMP> inline FloatFactory<PrecisionType<F>> factory(F const& flt);

template<ARawFloat F> inline FloatFactory<PrecisionType<F>> factory(F const& flt) {
    return FloatFactory<PrecisionType<F>>(flt.precision()); }
template<class PR> inline Float<PR> FloatFactory<PR>::create(Dyadic const& y, ExactTag) { return Float<PR>(y,_pr); }
template<class PR> inline Float<PR> FloatFactory<PR>::create(Integer const& y, ExactTag) { return Float<PR>(y,_pr); }
template<class PR> inline Float<PR> FloatFactory<PR>::create(ExactDouble const& y) { return Float<PR>(y,_pr); }

template<class PR> template<BuiltinSignedIntegral N> inline
Float<PR> FloatFactory<PR>::create(N const& y) { return Float<PR>(y,_pr); }
template<class PR> template<BuiltinUnsignedIntegral M> inline
Positive<Float<PR>> FloatFactory<PR>::create(M const& y) { return Positive<Float<PR>>(y,_pr); }

/*
template<class F> inline FloatFactory<PR> factory(Approximation<F> const& flt) { return FloatFactory<PR>(flt.precision()); }
template<class F> inline FloatFactory<PR> factory(LowerBound<F> const& flt) { return FloatFactory<PR>(flt.precision()); }
template<class F> inline FloatFactory<PR> factory(UpperBound<F> const& flt) { return FloatFactory<PR>(flt.precision()); }
template<class F> inline FloatFactory<PR> factory(Bounds<F> const& flt) { return FloatFactory<PR>(flt.precision()); }
template<class F, class FE> inline FloatFactory<PR> factory(FloatBall<PR,PRE> const& flt) { return FloatFactory<PR>(flt.precision()); }
template<class F> inline FloatFactory<PR> factory(F const& flt) { return FloatFactory<PR>(flt.precision()); }

template<class F> inline Approximation<F> FloatFactory<PR>::(Number<ApproximateTag> const& y) { return Approximation<F>(y,_pr); }
template<class F> inline LowerBound<F> FloatFactory<PR>::(Number<ValidatedLowerTag> const& y) { return LowerBound<F>(y,_pr); }
template<class F> inline UpperBound<F> FloatFactory<PR>::(Number<ValidatedUpperTag> const& y) { return UpperBound<F>(y,_pr); }

template<class F> inline Bounds<F> FloatFactory<PR>::(Number<ValidatedTag> const& y) { return Bounds<F>(y,_pr); }
template<class F> inline Bounds<F> FloatFactory<PR>::(Number<EffectiveTag> const& y) { return Bounds<F>(y,_pr); }
template<class F> inline Bounds<F> FloatFactory<PR>::(Number<ExactTag> const& y) { return Bounds<F>(y,_pr); }
template<class F> inline Bounds<F> FloatFactory<PR>::(Real const& y) { return Bounds<F>(y,_pr); }
template<class F> inline Bounds<F> FloatFactory<PR>::(Rational const& y) { return Bounds<F>(y,_pr); }
template<class F> inline Bounds<F> FloatFactory<PR>::(Dyadic const& y) { return Bounds<F>(y,_pr); }
template<class F> inline Bounds<F> FloatFactory<PR>::(Integer const& y) { return Bounds<F>(y,_pr); }

template<class F> inline F FloatFactory<PR>::(Dyadic const& y, ExactTag) { return F(y,_pr); }
template<class F> inline F FloatFactory<PR>::(Integer const& y, ExactTag) { return F(y,_pr); }

template<class F> inline template<IsBuiltinSignedIntegral N> F FloatFactory<PR>::(N const& y) { return F(y,_pr); }
template<class F> inline template<IsBuiltinUnsignedIntegral M> Positive<F> FloatFactory<PR>::(M const& y) { return Positive<F>(y,_pr); }
template<class F> inline template<IsBuiltinFloatingPoint D> Approximation<F> FloatFactory<PR>::(D const& y) { return Approximation<F>(F(y,_pr)); }
*/

} // namespace Ariadne

#endif
