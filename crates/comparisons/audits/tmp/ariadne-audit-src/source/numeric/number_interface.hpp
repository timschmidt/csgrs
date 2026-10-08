/***************************************************************************
 *            numeric/number_interface.hpp
 *
 *  Copyright  2013-20  Pieter Collins
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

/*! \file numeric/number_interface.hpp
 *  \brief
 */



#ifndef ARIADNE_NUMBER_INTERFACE
#define ARIADNE_NUMBER_INTERFACE

#include "utility/clonable.hpp"
#include "utility/writable.hpp"

#include "number.decl.hpp"
#include "float.decl.hpp"
#include "operators.hpp"

namespace Ariadne {

/************ Number *********************************************************/

class NumberInterface;
class UnaryOperatorInterface;
template<class... YS> struct Aware;
template<class X> class NumberWrapper;

namespace Detail { class LogicalInterface; }
using Detail::LogicalInterface;

class NumberInterface
    : public virtual WritableInterface
    , public virtual ClonableInterface
// Remove 'enable_shared_from_this' due to "relocation against undefined hidden symbol when making a shared object" error in g++
//    , public std::enable_shared_from_this<NumberInterface>
{
    template<class X> friend class NumberWrapper;
    friend class Handle<NumberInterface>;
  public:
    virtual ~NumberInterface() = default;
  public:
    virtual NumberInterface* _copy() const = 0;
    virtual NumberInterface* _move() = 0;

    virtual NumberInterface* _apply(UnaryElementaryOperator op) const = 0;
    virtual NumberInterface* _apply(BinaryElementaryOperator op, NumberInterface const* y) const = 0;
    virtual NumberInterface* _rapply(BinaryElementaryOperator op, NumberInterface const* y) const = 0;
    virtual NumberInterface* _apply(GradedElementaryOperator op, Int n) const = 0;

    virtual LogicalInterface* _apply(BinaryComparisonOperator op, NumberInterface const* y) const = 0;
    virtual LogicalInterface* _rapply(BinaryComparisonOperator op, NumberInterface const* y) const = 0;

    virtual Rational _get_q() const = 0;

    virtual FloatDPBall _get(MetricTag, DoublePrecision, DoublePrecision) const = 0;
    virtual FloatDPBounds _get(OrderTag, DoublePrecision) const = 0;
    virtual FloatDPUpperBound _get(UpperTag, DoublePrecision) const = 0;
    virtual FloatDPLowerBound _get(LowerTag, DoublePrecision) const = 0;
    virtual FloatDPApproximation _get(ApproximateTag, DoublePrecision) const = 0;

    virtual FloatMPDPBall _get(MetricTag, MultiplePrecision, DoublePrecision) const = 0;

    virtual FloatMPBall _get(MetricTag, MultiplePrecision, MultiplePrecision) const = 0;
    virtual FloatMPBounds _get(OrderTag, MultiplePrecision) const = 0;
    virtual FloatMPUpperBound _get(UpperTag, MultiplePrecision) const = 0;
    virtual FloatMPLowerBound _get(LowerTag, MultiplePrecision) const = 0;
    virtual FloatMPApproximation _get(ApproximateTag, MultiplePrecision) const = 0;

    // FIXME: Only used in templates; should not really be needed
    FloatDPBall _get(MetricTag p, DoublePrecision pr) const;
    FloatMPBall _get(MetricTag p, MultiplePrecision pr) const;

    FloatDPBounds _get(ValidatedTag, DoublePrecision pr) const;
    FloatMPBounds _get(ValidatedTag, MultiplePrecision pr) const;

    virtual LogicalInterface* _is_pos() const = 0;

    virtual ParadigmCode _paradigm() const = 0;
    virtual String _class_name() const = 0;

};


} // namespace Ariadne

#endif /* ARIADNE_NUMBER_INTERFACE */
