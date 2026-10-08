/***************************************************************************
 *            solvers/runge_kutta_integrator.hpp
 *
 *  Copyright  2010-20  Pieter Collins
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

#ifndef ARIADNE_RUNGE_KUTTA_INTEGRATOR_HPP
#define ARIADNE_RUNGE_KUTTA_INTEGRATOR_HPP

#include <iostream>

#include "utility/container.hpp"
#include "utility/declarations.hpp"
#include "solvers/integrator.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "function/function.hpp"

namespace Ariadne {

template<class T1, class T2> using Pair = std::pair<T1,T2>;
template<class T> class List;

//! \brief An approximate differential equation integrator based on the classical 4th-order Runge-Kutta method.
class RungeKutta4Integrator
{
  public:
    RungeKutta4Integrator(ApproximateDouble step_size);

    FloatDPApproximationVector
    step(const ApproximateVectorMultivariateFunction& f, const FloatDPApproximationVector& x, const FloatDPApproximation& h) const;

    List< Pair<FloatDPApproximation,FloatDPApproximationVector> >
    evolve(const ApproximateVectorMultivariateFunction& f, const FloatDPApproximationVector& x0, const FloatDPApproximation& tmax) const;
  private:
    ApproximateDouble _step_size;
};

} // namespace Ariadne

#endif /* ARIADNE_RUNGE_KUTTA_INTEGRATOR_HPP */
