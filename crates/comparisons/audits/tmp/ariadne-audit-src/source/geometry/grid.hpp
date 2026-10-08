/***************************************************************************
 *            geometry/grid.hpp
 *
 *  Copyright  2008-20  Ivan S. Zapreev, Pieter Collins
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

/*! \file geometry/grid.hpp
 *  \brief Coordinate-aligned grids.
 */

#ifndef ARIADNE_GRID_HPP
#define ARIADNE_GRID_HPP

#include "utility/array.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"

#include "geometry/point.hpp"
#include "geometry/box.hpp"

namespace Ariadne {

class Grid;
class GridCell;

//! \brief An infinite, uniform grid of rectangles in Euclidean space.
//!
//! \internal Maybe a Grid should be a type of Paving or Cover.
//! Then rather than having GridXXX classes, we can have classes such that cells of
//! some type are mapped into concrete sets by a Paving or Cover.
//! This should be more general, and will unify the concepts of Paving and Cover,
//! as well as different types of covers.
class Grid {
    using FLT=FloatDP;
    using Double = double;
  public:
    typedef double DyadicType;
    typedef Int IntegerType;
    typedef FLT ExactNumericType;
    typedef UpperBound<FLT> UpperNumericType;
    typedef LowerBound<FLT> LowerNumericType;
    typedef Point<FLT> ExactPointType;
  private:
    // Structure containing actual data values
    struct Data;
  public:
    //! Destructor.
    ~Grid();

    //! Default constructor constructs a grid from a null pointer. Needed for some iterators.
    explicit Grid();

    //! Construct from a dimension.
    explicit Grid(DimensionType d);

    //! Construct from a dimension and a spacing in each direction.
    explicit Grid(DimensionType d, RawFloatDP l);
    explicit Grid(DimensionType d, ApproximateDouble l);

    //! Construct from a vector of lengths.
    explicit Grid(const Vector<RawFloatDP>& lengths);
    explicit Grid(const Vector<ApproximateDouble>& lengths);

    //! Construct from a centre point and a vector of lengths.
    explicit Grid(const Vector<RawFloatDP>& origin, const Vector<RawFloatDP>& lengths);
    explicit Grid(const Vector<ApproximateDouble>& origin, const Vector<ApproximateDouble>& lengths);

    //! Copy constructor. Copies a reference to the grid data.
    Grid(const Grid& g);

    //! Copy assignment. Copies a reference to the grid data.
    Grid& operator=(const Grid& g);

    //! The underlying dimension of the grid.
    DimensionType dimension() const;

    //! Tests equality of two grids. Tests equality of references first.
    Bool operator==(const Grid& g) const;

    //! Tests inequality of two grids.
    Bool operator!=(const Grid& g) const;

    //! The origin of the grid.
    const Vector<RawFloatDP>& origin() const;

    //! The strides between successive integer points.
    const Vector<RawFloatDP>& lengths() const;

    //! Construct a grid for the product space,
    //! comprising subdivisions of \a g1 for the first,
    //! then those of \a g2.
    friend Grid join(Grid const& g1, Grid const& g2);

    //! Write to an output stream.
    friend OutputStream& operator<<(OutputStream& os, const Grid& g);

    FloatDP coordinate(DimensionType d, DyadicType x) const;
    FloatDP subdivision_coordinate(DimensionType d, DyadicType x) const;
    FloatDP subdivision_coordinate(DimensionType d, IntegerType n) const;

    Int subdivision_index(DimensionType d, const FloatDP& x) const;
    Int subdivision_lower_index(DimensionType d, const LowerBound<FloatDP>& x) const;
    Int subdivision_upper_index(DimensionType d, const UpperBound<FloatDP>& x) const;

    Array<DyadicType> index(const Point<FloatDP>& pt) const;
    Array<DyadicType> lower_index(const ExactBoxType& bx) const;
    Array<DyadicType> upper_index(const ExactBoxType& bx) const;

    Point<FloatDP> point(const Array<IntegerType>& a) const;
    Point<FloatDP> point(const Array<DyadicType>& a) const;
    ExactBoxType box(const Array<DyadicType>& l, const Array<DyadicType>& u) const;
    ExactBoxType box(const GridCell& cell) const;
  private:
    // Create new data
    Void _create(const Vector<RawFloatDP>& o, const Vector<RawFloatDP>& l);
  private:
    // Pointer to data. We can test grids for equality using reference semantics since data is a constant.
    std::shared_ptr<Data> _data;
};


} // namespace Ariadne

#endif /* ARIADNE_GRID_HPP */

