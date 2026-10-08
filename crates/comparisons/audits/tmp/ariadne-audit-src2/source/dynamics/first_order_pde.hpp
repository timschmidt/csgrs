/***************************************************************************
 *            dynamics/first_order_pde.hpp
 *
 *  Copyright  2018-20  Pieter Collins, Svetlana Selivanova
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*! \file dynamics/first_order_pde.hpp
 *  \brief Rigorous solutions of first-order hyperbolic partial differential equations.
 */

#ifndef ARIADNE_FIRST_ORDER_PDE_HPP
#define ARIADNE_FIRST_ORDER_PDE_HPP

#include "numeric/float.decl.hpp"
#include "numeric/real.hpp"
#include "algebra/tensor.hpp"
#include "function/function.decl.hpp"

#include "algebra/tensor.tpl.hpp"

namespace Ariadne {

//! \brief Compute the value of the multiaffine interpolation of the data \a us at the point \a x.
template<class X> Vector<Bounds<X>> multiaffine_interpolate(Tensor<2,Vector<Bounds<X>>> const& us, Vector<X> const& x);

//! \brief The first-order partial differential equation
//! \f$ A u_{,t} + \sum_{j=1}^{n} B_j u_{,x_j} = f(u). \f$
struct FirstOrderPDE {
    Matrix<Real> A; //!< <p/>
    Array<Matrix<Real>> Bs; //!< <p/>
    Array<DiagonalMatrix<Real>> const& Ds; //!< <p/>
    Array<Matrix<Real>> const& Ts; //!< <p/>
    EffectiveVectorMultivariateFunction const& f; //!< <p/>
};

//! \brief The solution of a first-order partial differential equation.
//! \relates FirstOrderPDE
template<class PR> struct FirstOrderPDESolution {
    Float<PR> h; //!< <p/>
    Float<PR> tau; //!< <p/>
    Tensor<3,Vector<FloatBounds<PR>>> uts; //!< <p/>
    FloatUpperBound<PR> error; //!< <p/>
};

//! \brief Solve the first-order partial differential equation
//! \f$ A u_{,t} + \sum_{j=1}^{n} B_j u_{,x_j} = f(u); \  u(0,x) = \phi_0(x); \  u:\R^n \to \R^m . \f$
//! \relates FirstOrderPDE
template<class PR> FirstOrderPDESolution<PR>
first_order_pde(FirstOrderPDE const& pde, EffectiveVectorMultivariateFunction const& phi0, PR pr);

} // namespace Ariadne

#endif // ARIADNE_FIRST_ORDER_PDE_HPP
