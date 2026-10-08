/***************************************************************************
 *            numeric/double.hpp
 *
 *  Copyright  2008-22  Pieter Collins
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

/*! \file numeric/double.hpp
 *  \brief Correctly rounded operations on double-precision numbers.
 */

#ifndef ARIADNE_DOUBLE_HPP
#define ARIADNE_DOUBLE_HPP

namespace Ariadne {

// Correctly rounded functions
double nul_rnd(double x);
double pos_rnd(double x);
double neg_rnd(double x);
double hlf_rnd(double x);
double sqr_rnd(double x);
double rec_rnd(double x);
double add_rnd(double x1, double x2);
double sub_rnd(double x1, double x2);
double mul_rnd(double x1, double x2);
double div_rnd(double x1, double x2);
double fma_rnd(double x1, double x2, double x3);
double pow_rnd(double x, Nat n);
double pow_rnd(double x, int n);
double sqrt_rnd(double x);
double exp_rnd(double x);
double log_rnd(double x);
double sin_rnd(double x);
double cos_rnd(double x);
double tan_rnd(double x);
double asin_rnd(double x);
double acos_rnd(double x);
double atan_rnd(double x);
double neg_rec_rnd(double x);
double atan_rnd_series(double x);
double pi_rnd();

double abs_rnd(double x);
double max_rnd(double x1, double x2);
double min_rnd(double x1, double x2);

double texp(double x);

double pi_opp();
double add_opp(double x, double y);
double sub_opp(double x, double y);
double mul_opp(double x, double y);
double div_opp(double x, double y);
double neg_rec_opp(double x);

} // namespace Ariadne

#endif
