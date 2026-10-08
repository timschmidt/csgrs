/***************************************************************************
 *            multifunction.cpp
 *
 *  Copyright 2008-21  Pieter Collins
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

#include "multifunction.hpp"

#include "function/taylor_model.hpp"
#include "algebra/algebra.hpp"
#include "function/taylor_function.hpp"
#include "function/function.hpp"
#include "function/function_model.hpp"

namespace Ariadne {

template<class P, class PR> MultifunctionModel<P,RealVector(RealVector),PR>::
    MultifunctionModel(BoxDomainType dom, VectorMultivariateFunction<P> f, BoxDomainType params, Sweeper<FLT> swp)
        : _f(VectorMultivariateTaylorFunctionModel<P,FLT>(product(dom,params),f,swp)), _as(dom.dimension()) { }

template class Multifunction<ValidatedTag,RealVector(RealVector)>;
template class MultifunctionPatch<ValidatedTag,RealVector(RealVector)>;
template class MultifunctionModel<ValidatedTag,RealVector(RealVector),DoublePrecision>;

} // namespace Ariadne

