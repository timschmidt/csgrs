/***************************************************************************
 *            dynamics/vector_field.hpp
 *
 *  Copyright  2004-20  Alberto Casagrande, Pieter Collins
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

/*! \file dynamics/vector_field.hpp
 *  \brief Main continuous dynamics system class.
 */

#ifndef ARIADNE_VECTOR_FIELD_HPP
#define ARIADNE_VECTOR_FIELD_HPP

#include <memory>

#include "function/function.hpp"
#include "geometry/set_interface.hpp"
#include "geometry/grid.hpp"
#include "symbolic/expression.decl.hpp"

#include "symbolic/variable.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/space.hpp"


namespace Ariadne {

template<class S> class LabelledSet;

class Enclosure;
class LabelledEnclosure;
class Storage;
class LabelledStorage;

class VectorFieldEvolver;


//! \brief A vector field in Euclidean space.
class VectorField
{
  public:
    //! \brief The type used to represent time.
    typedef Real TimeType;
    //! \brief The type used to represent real numbers.
    typedef Real RealType;
    //! \brief The type used to describe the state space.
    typedef EuclideanSpace StateSpaceType;
    //! \brief The class used to compute the system evolution.
    typedef VectorFieldEvolver EvolverType;
    //! \brief The type used to store local over-approximations to reach and evolve sets.
    typedef LabelledEnclosure EnclosureType;
    //! \brief The type used to define global pavings of reach and evolve sets.
    typedef LabelledStorage StorageType;
  public:
    VectorField(const List<DottedRealAssignment>&);
    VectorField(const List<DottedRealAssignment>&, List<RealAssignment> const&);

    const EffectiveVectorMultivariateFunction& dynamic_function() const { return this->_dynamic_function; }
    const EffectiveVectorMultivariateFunction& auxiliary_function() const { return this->_auxiliary_function; }
    const EffectiveVectorMultivariateFunction& auxiliary_mapping() const { return this->_auxiliary_function; }

    RealSpace state_space() const;
    RealSpace auxiliary_space() const;
    RealSpace state_auxiliary_space() const;

    VectorField* clone() const { return new VectorField(*this); }
    ~VectorField() = default;
    DimensionType dimension() const { return this->_dynamic_function.result_size(); }
    const EffectiveVectorMultivariateFunction& function() const { return _dynamic_function; }

    friend OutputStream& operator<<(OutputStream& os, VectorField const& vector_field);
  private:
    List<DottedRealAssignment> _dynamics;
    List<RealAssignment> _auxiliary;
    EffectiveVectorMultivariateFunction _dynamic_function;
    EffectiveVectorMultivariateFunction _auxiliary_function;
};

} // namespace Ariadne

#endif // ARIADNE_VECTOR_FIELD_HPP
