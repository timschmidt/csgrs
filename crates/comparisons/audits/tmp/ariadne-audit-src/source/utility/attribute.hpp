/***************************************************************************
 *            utility/attribute.hpp
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

/*! \file utility/attribute.hpp
 *  \brief Generic attributes for named parameters.
 */

#ifndef ARIADNE_ATTRIBUTE_HPP
#define ARIADNE_ATTRIBUTE_HPP

#include "typedefs.hpp"
#include "numeric/builtin.hpp"

namespace Ariadne {

template<class T> class Generator {
    typedef typename T::Type V;
  public:
    inline T operator=(const typename T::Type& v) const;
};

template<class V> class Attribute {
    template<class T> friend class Generator;
  private:
    V _v;
  protected: public:
    Attribute(const V& v) : _v(v) { }
  public:
    typedef V Type;
    operator V() const { return this->_v; }
    V value() const { return this->_v; }
};

template<class T> inline T Generator<T>::operator=(const typename T::Type& v) const {
    Attribute<typename T::Type> attr(v); return static_cast<const T&>(attr); }

struct MaximumError : Attribute<ApproximateDouble> {
    MaximumError(ApproximateDouble x) : Attribute<ApproximateDouble>(x) { }
    MaximumError(double x) : Attribute<ApproximateDouble>(x) { }
};
struct SweepThreshold : Attribute<ApproximateDouble> { using Attribute<ApproximateDouble>::Attribute; };
struct MaximumNumericTypeOfSteps : Attribute<Nat> { using Attribute<Nat>::Attribute; };

static const Generator<MaximumError> maximum_error = Generator<MaximumError>();
static const Generator<SweepThreshold> sweep_threshold = Generator<SweepThreshold>();
static const Generator<MaximumNumericTypeOfSteps> maximum_number_of_steps = Generator<MaximumNumericTypeOfSteps>();

struct Capacity : Attribute<SizeType> { };
static const Generator<Capacity> capacity = Generator<Capacity>();
struct Size : Attribute<SizeType> { };
static const Generator<Size> size = Generator<Size>();
struct ResultSize : Attribute<SizeType> { };
static const Generator<ResultSize> result_size = Generator<ResultSize>();
struct ArgumentSize : Attribute<SizeType> { };
static const Generator<ArgumentSize> argument_size = Generator<ArgumentSize>();
struct Degree : Attribute<DegreeType> { };
static const Generator<Degree> degree = Generator<Degree>();


} // namespace Ariadne

#endif /* ARIADNE_ATTRIBUTE_HPP */
