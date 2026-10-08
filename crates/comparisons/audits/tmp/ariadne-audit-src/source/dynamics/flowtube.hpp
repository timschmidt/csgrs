/***************************************************************************
 *            dynamics/flowtube.hpp
 *
 *  Copyright  2007-20  Pieter Collins
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

/*! \file dynamics/flowtube.hpp
 *  \brief Flow tubes for dynamic systems
 */

#ifndef ARIADNE_FLOWTUBE_HPP
#define ARIADNE_FLOWTUBE_HPP


namespace Ariadne {

template<class ES> class Orbit;

using ValidatedImageSet = ValidatedConstrainedImageSet;

class FlowTube {
    ValidatedVectorMultivariateFunctionModelDP _phi;
    List<Identifier> _variable_names;
  public:
    SizeType number_of_state_variables() const { return _phi.result_size(); }
    template<class X, class T> requires Same<X,T> decltype(auto) operator()(Vector<X> const& x, Scalar<T> const& t) {
        return phi(join(x,t)); }
    decltype(auto) space_domain() const { SizeType n=this->number_of_state_variables(); return _phi.domain()[range(0,n)]; }
    decltype(auto) time_range() const { SizeType n=this->number_of_state_variables(); return _phi.domain()[n]; }
    decltype(auto) initial_time() const { return time_range().lower(); }
    decltype(auto) final_time() const { return time_range().upper(); }
    ValidatedVectorMultivariateFunctionModelDP reach_function() const;
    ValidatedVectorMultivariateFunctionModelDP final_function() const;
    ValidatedImageSet initial_set() const;
    ValidatedImageSet reach_set() const;
    ValidatedImageSet final_set() const;
};

template<class E, class R=E> struct OrbitStep {
    OrbitStep* _previous;
    E _initial; R _intermediate;
    List<OrbitStep> _next;
    List<E> _final;
};

template<> class Orbit<FlowTube> {
    using ES=FlowTube;
    OrbitStep<ES> _starting;
    List<OrbitStep<ES>*> _working;
  public:
    Orbit(const ES& set);
};

} // namespace Ariadne

#endif // ARIADNE_FLOWTUBE_HPP
