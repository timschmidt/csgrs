/***************************************************************************
 *            symbolic/space.hpp
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

/*! \file symbolic/space.hpp
 *  \brief Spaces formed by variables.
 */

#ifndef ARIADNE_SPACE_HPP
#define ARIADNE_SPACE_HPP

#include <cstdarg>
#include <iosfwd>
#include <iostream>

#include "utility/macros.hpp"
#include "utility/pointer.hpp"
#include "utility/container.hpp"

#include "symbolic/variable.hpp"

namespace Ariadne {


template<class T> class Space;
template<class T> OutputStream& operator<<(OutputStream& os, const Space<T>& spc);

class Real;

//! \brief A space defined as a list of real variables.
//! \relates Space
typedef Space<Real> RealSpace;

//! \ingroup SymbolicModule
//! \brief A space defined as a list of named variables of type \a T.
//! Allows conversion between the indexed space \a T<sup>n</sup> and the space \a T<sup>V</sup> defined by named variables \a V.
//!
//! \details %Ariadne's main computational functionality is based around Euclidean space \f$\R^n\f$.
//! However, it is more convenient to define sets and functions symbolically in terms of named variables.
//! A Space<Real> provides an ordering of variables which allows conversion between the two kinds of representation.
//!
//! The main conversion allowed is between \ref Valuation of type \a T<sup>V</sup> for a set of \a n variables \a V,
//! and a \ref Vector or type \a T<sup>n</sup>.
//! It also allows conversion between sets defined in terms of named variables and those defined in terms of numbered indices,
//! such as \ref VariablesBox and \ref Box.
//! In general, the conversion operations are provided by the symbolic class.
//!
//! \b Example
//! \snippet tutorials/symbolic_usage.cpp Space_usage
//! \see Variable
template<class T> class Space
{
  public:
    typedef Variable<T> VariableType;
  public:
    //! \brief The trivial space \f$T^0\f$.
    Space();
    Space(const List<VariableType>& vl);
    Space(const List<Identifier>& vl);
    Space(const InitializerList<VariableType>& vl);

    Bool operator==(const Space<T>& other) const;
    Bool operator!=(const Space<T>& other) const;

    //! \brief The dimension of the space.
    SizeType size() const;
    //! \brief The dimension of the space.
    SizeType dimension() const;
    //! \brief The \a i<sup>th</sup> named variable.
    const VariableType operator[](SizeType i) const;
    //! \brief The \a i<sup>th</sup> named variable.
    const VariableType variable(SizeType i) const;

    //! \brief A list giving ordered variables.
    List<Identifier> variable_names() const;
    //! \brief A list giving ordered variables.
    List<VariableType> variables() const;
    //! \brief A map giving the index of a given variable.
    Map<Identifier,SizeType> indices_from_names() const;
    //! \brief A map giving the index of a given variable.
    Map<VariableType,SizeType> indices() const;

    //! \brief Tests if the variable \a v is in the space.
    Bool contains(const VariableType& v) const;
    //! \brief Tests if all the variables \a vs is in the space.
    Bool contains(const Set<VariableType>& vs) const;
    //! \brief The index of the named variable \a v.
    SizeType operator[](const VariableType& v) const;
    SizeType operator[](const Identifier& n) const;
    //! \brief The index of the named variable \a v.
    SizeType index(const VariableType& v) const;
    SizeType index(const Identifier& n) const;
    //! \brief Append the named variable \a v to the variables defining the space; ignores if the variable is already in the space.
    Space<T>& insert(const VariableType& v);
    //! \brief Adjoins the variables in \a spc.
    Space<T>& adjoin(const Space<T>& spc);
    //! \brief Append the named variable \a v to the variables defining the space.
    Space<T>& append(const VariableType& v);
  private:
    List<Identifier> _variables;
};

template<class T> OutputStream& operator<<(OutputStream& os, const Space<T>& spc) { return os << spc.variables(); }

template<class T> Space<T> join(const Space<T>& spc1, const Space<T>& spc2) {
    Space<T> r(spc1); r.adjoin(spc2); return r; }
template<class T> Space<T> join(const Space<T>& spc1, const Variable<T>& var2) {
    Space<T> r(spc1); r.append(var2); return r; }

// Compiled conversion operators to allow conversion between expression and function.
SizeType dimension(const Space<Real>& spc);
List<Identifier> variable_names(const Space<Real>& spc);
List<Identifier> variable_names(const List<Variable<Real>>& spc);
Space<Real> real_space(const List<Identifier>& vars);

template<class T> Variable<T> variable(const Identifier& s);
template<class T> Space<T> variables(const List<Identifier>& s);


} // namespace Ariadne

#endif /* ARIADNE_SPACE_HPP */
