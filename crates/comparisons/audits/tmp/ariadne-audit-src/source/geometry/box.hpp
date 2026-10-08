/***************************************************************************
 *            geometry/box.hpp
 *
 *  Copyright  2008-20  Alberto Casagrande, Pieter Collins
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

/*! \file geometry/box.hpp
 *  \brief Coordinate-aligned boxes in Euclidean space.
 */

#ifndef ARIADNE_BOX_HPP
#define ARIADNE_BOX_HPP

#include "utility/container.hpp"

#include "foundations/logical.hpp"
#include "numeric/floatdp.hpp"
#include "geometry/interval.hpp"
#include "geometry/point.hpp"
#include "geometry/set_interface.hpp"

#include "box.decl.hpp"

namespace Ariadne {

struct DoubleInput {
    double _d;
    DoubleInput(double d) : _d(d) { }
    operator double () const { return _d; }
};



//! \ingroup BasicSetSubModule GeometryModule
//! \brief A box in Euclidean space.
template<class I>
class Box
{
  private:
    Vector<I> _vec;
  private:
    typedef typename I::UpperBoundType U;
    typedef typename I::LowerBoundType L;
    typedef typename I::MidpointType M;
    typedef typename I::CentreType C;
    typedef typename I::RadiusType R;
    typedef typename I::WidthType W;
    typedef decltype(max(declval<L>(),declval<U>())) V;
  public:
    //! \brief The type used for drawing ListSet of Box. FIXME: Should not be present
    typedef Drawable2dInterface DrawableInterfaceType;

    //! \brief The type returned by the dimension() method.
    typedef Ariadne::DimensionType DimensionType;
    //! The type used for a component interval.
    typedef I IntervalType;
    //! The type used for the lower bound of a component interval.
    typedef L LowerBoundType;
    //! The type used for the upper bound of a component interval.
    typedef U UpperBoundType;
    //! The type used for the centre of a component interval.
    typedef C CentreValueType;
    //! The type used for the midpoint of a component interval.
    //! Must be convertible to both the lower and upper bound types.
    typedef M MidpointValueType;
    //! A type which can be used for the both the lower and upper bound of a component interval.
    typedef V VertexBoundType;
    //! A type which can be used for the radius or length of a side.
    typedef R RadiusType;
    //! A type which can be used for the measure (area, volume) of the box.
    typedef R MeasureType;
  public:
    //! A type which can be used for the vertices of the box.
    typedef Point<V> VertexType;
    //! A type which can be used for the midpoint of the box.

    typedef Point<M> MidpointType;
    //! A type which can be used for the exact centre of the box.
    typedef Point<C> CentreType;
  public:
    Box();
    explicit Box(SizeType n);
    Box(SizeType n, IntervalType ivl);
    Box(const InitializerList<IntervalType>& lst);

    template<class II> explicit Box(const Array<II>& bx);

    // Non-templated constructor to allow conversion from VectorExpression
    Box(const Vector<I>& bx) : _vec(bx) { }

    //! Convert from a box of a different type
    template<class II> requires Convertible<II,I>
        Box(const Box<II>& bx) : _vec(cast_vector(bx)) { }

    //! Construct from a box of a different type
    template<class II> requires ExplicitlyConvertible<II,I>
        explicit Box(const Box<II>& bx) : _vec(cast_vector(bx)) { }

    //! Convert from a box of a different type
    template<class II> requires Convertible<II,I>
        Box(const Vector<II>& vec) : _vec(vec) { }

    //! Construct from a box of a different type
    template<class II> requires ExplicitlyConvertible<II,I>
        explicit Box(const Vector<II>& vec) : _vec(vec) { }

    template<class II, class PR> requires Constructible<I,II,PR>
        Box(const Box<II>& bx, PR pr) : _vec(cast_vector(bx),pr) { }

    //! \brief Construct from an interval of a different type using a default precision.
    template<class II> requires ConstructibleGivenDefaultPrecision<I,II> and (not Constructible<I,II>)
        explicit Box(Box<II> const& x) : Box(x,PrecisionType<II>()) { }

    /*! \brief Generate from a function (object) \a g of type \a G mapping an index to a value. */
    template<class G> requires InvocableReturning<I,G,SizeType>
    Box(SizeType n, G const& g) : _vec(n,g) { }

    friend Vector<I> const& cast_vector(Box<I> const& bx) { return bx._vec; }
    friend Vector<I>& cast_vector(Box<I>& bx) { return bx._vec; }

    //! The unit box \f$[-1,1]^n\f$ in \a n dimensions.
    static Box<IntervalType> unit_box(SizeType n);
    //! The upper quadrant box \f$[0,\infty]^n\f$ in \a n dimensions.
    static Box<IntervalType> upper_orthant(SizeType n);

    //! An array of the data values.
    UniformArray<I> const& array() const;
    //! The dimension of the set. DEPRECATED
    SizeType size() const;
    //! The dimension of the set.
    DimensionType dimension() const;
    //! Indexing the bounds in each dimension yields an exact interval.
    const IntervalType& operator[](SizeType i) const;
    IntervalType& operator[](SizeType i);

    const Box<IntervalType> operator[](Range is) const;

    //! The lower corner of the box.
    VertexType lower_bounds() const;
    //! The upper corner of the box.
    VertexType  upper_bounds() const;
    //! The vertices of the box.
    List<VertexType> vertices() const;
    //! The bounding box.
    Box<IntervalType> bounding_box() const;

    //! \brief Test if the box is empty. Returns the same type as IntervalType::is_empty().
    auto is_empty() const -> decltype(declval<IntervalType>().is_empty());
    //! \brief Test if the box is bounded.Returns the same type as IntervalType::is_bounded().
    auto is_bounded() const -> decltype(declval<IntervalType>().is_bounded());

    //! Splits the box along coordinate \a k and takes the lower , middle, or upper part as given by \a lmu.
    Box<IntervalType> split(SizeType k, SplitPart lmu) const;
    //! Splits the box along the longest direction \a k and takes the lower , middle, or upper part as given by \a lmu.
    Box<IntervalType> split(SplitPart lmu) const;
    //! Splits the box along widest coordinate \a k and takes the lower and upper parts.
    Pair< Box<IntervalType>, Box<IntervalType> > split(SizeType k) const;
    //! Splits the box along widest coordinate \a k and takes the lower and upper parts.
    Pair< Box<IntervalType>, Box<IntervalType> > split() const;

    //! \brief Test if the box contains the point \a pt.
    template<class X> auto contains(const Point<X>& pt) const -> decltype(Ariadne::contains(declval<I>(),declval<X>()));

    template<class II> auto intersects(const Box<II>& bx) const -> decltype(Ariadne::intersect(declval<I>(),declval<II>()));
    template<class II> auto disjoint(const Box<II>& bx) const -> decltype(Ariadne::disjoint(declval<I>(),declval<II>()));
    template<class II> auto subset(const Box<II>& bx) const -> decltype(Ariadne::subset(declval<I>(),declval<II>()));
    template<class II> auto superset(const Box<II>& bx) const -> decltype(Ariadne::superset(declval<I>(),declval<II>()));
    template<class II> auto covers(const Box<II>& bx) const -> decltype(Ariadne::covers(declval<I>(),declval<II>()));
    template<class II> auto overlaps(const Box<II>& bx) const -> decltype(Ariadne::overlap(declval<I>(),declval<II>()));
    template<class II> auto inside(const Box<II>& bx) const -> decltype(Ariadne::inside(declval<I>(),declval<II>()));
    template<class II> auto separated(const Box<II>& bx) const -> decltype(Ariadne::separated(declval<I>(),declval<II>()));


    //! The exact centre of the box.
    CentreType centre() const;
    //! The (mid)point of the box.
    MidpointType midpoint() const;
    //! Half the length of the longest side.
    RadiusType radius() const;
    //! The radii of the sides.
    Vector<RadiusType> radii() const;
    //! The widths of the sides.
    Vector<RadiusType> widths() const;
    //! The sum of the lengths of the sides.
    RadiusType semiperimeter() const;
    //! The measure (area or volume) of the box.
    MeasureType measure() const;
    MeasureType volume() const;

    Void draw(CanvasInterface& c, const Projection2d& p) const;
  public:
    static Box<I> _product(const Box<I>& bx1, const Box<I>& bx2, const Box<I>& bx3);
    static Box<I> _product(const Box<I>& bx1, const Box<I>& bx2);
    static Box<I> _product(const Box<I>& bx1, const I& ivl2);
    static Box<I> _product(const I& ivl1, const Box<I>& bx2);

    static Box<I> _project(const Box<I>& bx, const Array<SizeType>& rng);
    static Box<I> _project(const Box<I>& bx, const Range& rng);
};

template<class I> inline OutputStream& operator<<(OutputStream& os, const Box<I>& bx) {
    return os << cast_vector(bx);
}

template<class I> Box(InitializerList<I>) -> Box<I>;

template<class I> template<class II> inline Box<I>::Box(const Array<II>& ary) : _vec(ary) { }
template<class I> inline Box<I>::Box(InitializerList<I> const& lst) : _vec(lst) { }

template<class I> decltype(declval<Box<I>>().is_empty()) is_empty(const Box<I>& bx) { return bx.is_empty(); }
template<class I> decltype(declval<Box<I>>().is_bounded()) is_bounded(const Box<I>& bx) { return bx.is_bounded(); }

template<class I> decltype(declval<Box<I>>().centre()) centre(const Box<I>& bx) { return bx.centre(); }
template<class I> decltype(declval<Box<I>>().midpoint()) midpoint(const Box<I>& bx) { return bx.midpoint(); }
template<class I> decltype(declval<Box<I>>().radius()) radius(const Box<I>& bx) { return bx.radius(); }
template<class I> decltype(declval<Box<I>>().radii()) radii(const Box<I>& bx) { return bx.radii(); }
template<class I> decltype(declval<Box<I>>().widths()) widths(const Box<I>& bx) { return bx.widths(); }
template<class I> decltype(declval<Box<I>>().measure()) measure(const Box<I>& bx) { return bx.measure(); }
template<class I> decltype(declval<Box<I>>().volume()) volume(const Box<I>& bx) { return bx.volume(); }

template<class I> inline Box<I> split(const Box<I>& bx, SizeType k, SplitPart lmu) { return bx.split(k,lmu); }
template<class I> inline Box<I> split(const Box<I>& bx, SplitPart lmu) { return bx.split(lmu); }
template<class I> inline Pair<Box<I>,Box<I>> split(const Box<I>& bx) { return bx.split(); }
template<class I> inline Pair<Box<I>,Box<I>> split(const Box<I>& bx, SizeType k) { return bx.split(k); }

//! \relates Box \brief The Cartesian product of two boxes, defined as \f$ B_1 \times B_2 = \{ (x_1,x_2) \mid x_1 \in B_1 \wedge x_2 \in B_2 \} \f$.
template<class I> inline Box<I> product(const Box<I>& bx1, const Box<I>& bx2) { return Box<I>::_product(bx1,bx2); }
//! \relates Box \brief The Cartesian product of an interval and a box.
template<class I> inline Box<I> product(const Interval<I>& ivl1, const Box<I>& bx2) { return Box<I>::_product(ivl1,bx2); }
//! \relates Box \brief The Cartesian product of a box and an interval.
template<class I> inline Box<I> product(const Box<I>& bx1, const I& ivl2) { return Box<I>::_product(bx1,ivl2); }
//! \relates Box \brief The Cartesian product of three boxes.
template<class I> inline Box<I> product(const Box<I>& bx1, const Box<I>& bx2, const Box<I>& bx3) { return Box<I>::_product(bx1,bx2,bx3); }
template<class I> inline Box<I> product(const Box<I>& bx1, const I& ivl2, const Box<I>& bx3) { return Box<I>::_product(bx1,Box<I>({ivl2}),bx3); }

template<class I> inline Box<I> join(const Box<I>& bx1, const I& ivl2, const Box<I>& bx3) { return Box<I>::_product(bx1,Box<I>({ivl2}),bx3); }
template<class I> inline Box<I> join(const Box<I>& bx1, const Box<I>& bx2, const Box<I>& bx3) { return Box<I>::_product(bx1,bx2,bx3); }

template<class S1, class S2, class S3> inline decltype(auto) product(S1 const& s1, S2 const& s2, S3 const& s3) { return product(product(s1,s2),s3); }

template<class I> inline Box<I> remove(const Box<I>& bx, SizeType k) {
    Box<I> rbx(bx.dimension()-1); for(SizeType i=0; i!=k; ++i) { rbx[i]=bx[i]; } for(SizeType i=k; i!=rbx.dimension(); ++i) { rbx[i]=bx[i+1]; } return rbx; }

//! \relates Interval \brief Compute an over-approximation to the image \f$f(I) = \{ f(x) \mid x\in I \}\f$.
UpperIntervalType apply(ValidatedScalarMultivariateFunction const& f, UpperIntervalType const& ivl);
//! \relates Box \brief Compute an over-approximation to the image \f$f(I) = \{ f(x) \mid x\in I \}\f$.
UpperBoxType apply(ValidatedVectorMultivariateFunction const& f, UpperIntervalType const& ivl);
//! \relates Box \brief Compute an over-approximation to the image \f$f(B) = \{ f(x) \mid x\in B \}\f$.
UpperIntervalType apply(ValidatedScalarMultivariateFunction const& f, UpperBoxType const& bx);
//! \relates Box \brief Compute an over-approximation to the image \f$f(B) = \{ f(x) \mid x\in B \}\f$.
UpperBoxType apply(ValidatedVectorMultivariateFunction const& f, UpperBoxType const& bx);

//! \relates Interval \brief Compute an over-approximation to the image \f$f(I) = \{ f(x) \mid x\in I \}\f$.
UpperIntervalType image(UpperIntervalType const& ivl, ValidatedScalarUnivariateFunction const& f);
//! \relates Box \brief Compute an over-approximation to the image \f$f(I) = \{ f(x) \mid x\in I \}\f$.
UpperBoxType image(UpperIntervalType const& ivl, ValidatedVectorUnivariateFunction const& f);
//! \relates Box \brief Compute an over-approximation to the image \f$f(B) = \{ f(x) \mid x\in B \}\f$.
UpperIntervalType image(UpperBoxType const& bx, ValidatedScalarMultivariateFunction const& f);
//! \relates Box \brief Compute an over-approximation to the image \f$f(B) = \{ f(x) \mid x\in B \}\f$.
UpperBoxType image(UpperBoxType const& bx, ValidatedVectorMultivariateFunction const& f);

//! \relates Box \brief Project onto the variables \a rng.
template<class I> inline Box<I> project(const Box<I> & bx, Array<SizeType> const& rng) { return Box<I>::_project(bx,rng); }

template<class I> inline Box<I> project(const Box<I> & bx, Range const& rng) { return Box<I>::_project(bx,rng); }

template<class I> auto lower_bounds(const Box<I>& bx) -> typename Box<I>::VertexType {
    return bx.lower_bounds();
}

template<class I> auto upper_bounds(const Box<I>& bx) -> typename Box<I>::VertexType {
    return bx.upper_bounds();
}


template<class I1, class I2> EqualsType<I1,I2> operator==(const Box<I1>& bx1, const Box<I2>& bx2) {
    if (bx1.dimension()!=bx2.dimension()) { return EqualsType<I1,I2>(false); }
    EqualsType<I1,I2> r(true);
    for(SizeType i=0; i!=bx1.dimension(); ++i) {
        r = r && (bx1[i]==bx2[i]);
    }
    return r;
}

template<class I1, class I2> decltype(declval<I1>()!=declval<I2>()) operator!=(const Box<I1>& bx1, const Box<I2>& bx2) {
    return !(bx1==bx2);
}

template<class I> Bool same(const Box<I>& bx1, const Box<I>& bx2) {
    if(bx1.dimension()!=bx2.dimension()) { return false; }
    for(SizeType i=0; i!=bx1.dimension(); ++i) {
        if(not same(bx1[i],bx2[i])) { return false; }
    }
    return true;
}

//! \relates Box \brief The smallest box containing the two boxes.
template<class I1, class I2> Box<decltype(hull(declval<I1>(),declval<I2>()))> hull(const Box<I1>& bx1, const Box<I2>& bx2) {
    ARIADNE_ASSERT(bx1.dimension()==bx2.dimension());
    Box<decltype(hull(declval<I1>(),declval<I2>()))> r(bx1.dimension());
    for(SizeType i=0; i!=bx1.dimension(); ++i) {
        r[i]=hull(bx1[i],bx2[i]);
    }
    return r;
}

//! \relates Box \brief The smallest box containing the box and a point.
template<class I, class X> Box<I> hull(const Box<I>& bx1, const Point<X>& pt2) {
    ARIADNE_ASSERT(bx1.dimension()==pt2.dimension());
    Box<I> r(bx1.dimension());
    for(SizeType i=0; i!=bx1.dimension(); ++i) {
        r[i]=hull(bx1[i],pt2[i]);
    }
    return r;
}

//! \relates Box \brief The smallest box containing the box and a point.
template<class I, class X> Box<I> hull(const Point<X>& pt1, const Box<I>& bx2) {
    ARIADNE_ASSERT(pt1.dimension()==bx2.dimension());
    Box<I> r(bx2.dimension());
    for(SizeType i=0; i!=bx2.dimension(); ++i) {
        r[i]=hull(pt1[i],bx2[i]);
    }
    return r;
}

//! \relates Box \brief The intersection of the two boxes.
template<class I1, class I2> Box<decltype(intersection(declval<I1>(),declval<I2>()))> intersection(const Box<I1>& bx1, const Box<I2>& bx2) {
    Box<decltype(intersection(declval<I1>(),declval<I2>()))> res(bx1.dimension());
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res[i]=intersection(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if a value is an element of a box.
template<class I, class X> decltype(element(declval<X>(),declval<I>())) element(const Vector<X>& x1, const Box<I>& bx2) {
    decltype(element(declval<X>(),declval<I>())) res=true;
    for(SizeType i=0; i!=bx2.dimension(); ++i) { res = res && element(x1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if a box contains a value.
template<class I, class X> decltype(contains(declval<I>(),declval<X>())) contains(const Box<I>& bx1, const Vector<X>& x2) {
    decltype(contains(declval<I>(),declval<X>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && contains(bx1[i],x2[i]); }
    return res;
}

//! \relates Box \brief Tests if a box contains a point.
template<class I, class X> decltype(contains(declval<I>(),declval<X>())) contains(const Box<I>& bx1, const Point<X>& pt2) {
    decltype(contains(declval<I>(),declval<X>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && contains(bx1[i],pt2[i]); }
    return res;
}

//! \relates Box \brief Tests if two boxes are disjoint.
template<class I1, class I2> decltype(disjoint(declval<I1>(),declval<I2>())) disjoint(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(disjoint(declval<I1>(),declval<I2>())) res=false;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res || disjoint(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if two boxes intersect.
template<class I1, class I2> decltype(intersect(declval<I1>(),declval<I2>())) intersect(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(intersect(declval<I1>(),declval<I2>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && intersect(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if two boxes are equal.
template<class I1, class I2> decltype(equal(declval<I1>(),declval<I2>())) equal(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(equal(declval<I1>(),declval<I2>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && equal(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if a box is a subset of another.
template<class I1, class I2> decltype(subset(declval<I1>(),declval<I2>())) subset(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(subset(declval<I1>(),declval<I2>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && subset(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if a box is a superset of another.
template<class I1, class I2> decltype(superset(declval<I1>(),declval<I2>())) superset(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(superset(declval<I1>(),declval<I2>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && superset(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if the interior of a box is a superset of the closure of another.
template<class I1, class I2> decltype(covers(declval<I1>(),declval<I2>())) covers(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(covers(declval<I1>(),declval<I2>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && covers(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if the interiors of two boxes intersect.
template<class I1, class I2> decltype(overlap(declval<I1>(),declval<I2>())) overlap(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(overlap(declval<I1>(),declval<I2>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && overlap(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if the closures of two boxes are disjoint.
template<class I1, class I2> decltype(separated(declval<I1>(),declval<I2>())) separated(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(separated(declval<I1>(),declval<I2>())) res=false;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res || separated(bx1[i],bx2[i]); }
    return res;
}

//! \relates Box \brief Tests if the closure of a box is a subset of the interior of another.
template<class I1, class I2> decltype(inside(declval<I1>(),declval<I2>())) inside(const Box<I1>& bx1, const Box<I2>& bx2) {
    decltype(inside(declval<I1>(),declval<I2>())) res=true;
    for(SizeType i=0; i!=bx1.dimension(); ++i) { res = res && inside(bx1[i],bx2[i]); }
    return res;
}


//! \relates Box \brief Tests if an over-approximation to a box is a tighter over-approximation than another.
template<class I> inline decltype(refines(declval<I>(),declval<I>())) refines(const Box<typename Box<I>::IntervalType>& bx1, const Box<I>& bx2) {
    ARIADNE_ASSERT(bx1.dimension()==bx2.dimension());
    for(SizeType i=0; i!=bx1.dimension(); ++i) {
        if(!refines(bx1[i],bx2[i])) { return false; }
    }
    return true;
}


template<class I> template<class X> inline auto Box<I>::contains(const Point<X>& pt) const -> decltype(Ariadne::contains(declval<I>(),declval<X>())) {
    return Ariadne::contains(*this,pt); }

template<class I1> template<class I2> inline auto Box<I1>::intersects(const Box<I2>& bx) const -> decltype(Ariadne::intersect(declval<I1>(),declval<I2>())) {
    return Ariadne::intersect(*this,bx); }
template<class I1> template<class I2> inline auto Box<I1>::disjoint(const Box<I2>& bx) const -> decltype(Ariadne::disjoint(declval<I1>(),declval<I2>())) {
    return Ariadne::disjoint(*this,bx); }
template<class I1> template<class I2> inline auto Box<I1>::subset(const Box<I2>& bx) const -> decltype(Ariadne::subset(declval<I1>(),declval<I2>())) {
    return Ariadne::subset(*this,bx); }
template<class I1> template<class I2> inline auto Box<I1>::superset(const Box<I2>& bx) const -> decltype(Ariadne::superset(declval<I1>(),declval<I2>())) {
    return Ariadne::superset(*this,bx); }
template<class I1> template<class I2> inline auto Box<I1>::covers(const Box<I2>& bx) const -> decltype(Ariadne::covers(declval<I1>(),declval<I2>())) {
    return Ariadne::covers(*this,bx); }
template<class I1> template<class I2> inline auto Box<I1>::overlaps(const Box<I2>& bx) const -> decltype(Ariadne::overlap(declval<I1>(),declval<I2>())) {
    return Ariadne::overlap(*this,bx); }
template<class I1> template<class I2> inline auto Box<I1>::inside(const Box<I2>& bx) const -> decltype(Ariadne::inside(declval<I1>(),declval<I2>())) {
    return Ariadne::inside(*this,bx); }
template<class I1> template<class I2> inline auto Box<I1>::separated(const Box<I2>& bx) const -> decltype(Ariadne::separated(declval<I1>(),declval<I2>())) {
    return Ariadne::separated(*this,bx); }



//! \relates Box \brief Draws a two-dimensional projection by \a p of box \a bx.
Void draw(CanvasInterface& c, Projection2d const& p, ApproximateBoxType const& bx);
ExactBoxType make_box(const String& str);


// Inline functions so as not to have to instantiate box class

template<class I> Box<I>::Box()
    : _vec() { }

template<class I> inline Box<I>::Box(SizeType n)
    : Box(n,IntervalType::empty_interval()) { }

template<class I> inline Box<I>::Box(SizeType n, IntervalType ivl)
    : _vec(n,ivl) { }

template<class I> inline Box<I> Box<I>::unit_box(SizeType n) {
    return Box<IntervalType>(n,IntervalType(-1,1));
}

template<class I> inline UniformArray<I> const& Box<I>::array() const {
    return this->_vec.array();
}
template<class I> inline DimensionType Box<I>::dimension() const {
    return this->_vec.size();
}
template<class I> inline SizeType Box<I>::size() const {
    return this->_vec.size();
}
template<class I> inline const I& Box<I>::operator[](SizeType i) const {
    return this->_vec.operator[](i);
}

template<class I> inline I& Box<I>::operator[](SizeType i) {
    return static_cast<I&>(this->_vec.operator[](i));
}

template<class I> inline const Box<I> Box<I>::operator[](Range rng) const {
    return Box<I>(project(*this,rng));
}

template<class I> inline auto cast_singleton(Box<I> const& bx) -> Vector<decltype(cast_singleton(declval<I>()))> {
    typedef decltype(cast_singleton(declval<I>())) X;
    return Vector<X>(bx.dimension(),[&](SizeType i){return cast_singleton(bx[i]);},bx.array().element_characteristics());
}

template<class I, class PR> inline auto cast_singleton(Box<I> const& bx, PR pr) -> Vector<decltype(cast_singleton(declval<I>(),declval<PR>()))> {
    typedef decltype(cast_singleton(declval<I>(),declval<PR>())) X;
    return Vector<X>(bx.dimension(),[&](SizeType i){return cast_singleton(bx[i],pr);},pr);
}

template<class I> inline Void Box<I>::draw(CanvasInterface& c, const Projection2d& p) const {
    Ariadne::draw(c,p,ApproximateBoxType(*this)); }

template<> inline Void Box<Interval<Real>>::draw(CanvasInterface& c, const Projection2d& p) const {
    Ariadne::draw(c,p,ApproximateBoxType(*this,dp)); }

inline FloatDPExactBox cast_exact_box(FloatDPApproximateBox const& abx) {
    return FloatDPExactBox(reinterpret_cast<FloatDPExactBox const&>(abx));
}

inline Box<FloatDPExactInterval> cast_exact_box(Vector<FloatDPBounds> const& bv) {
    return Box<FloatDPExactInterval>(reinterpret_cast<Vector<FloatDPExactInterval>const&>(bv));
}

template<class I, class X> inline Box<decltype(declval<I>()+declval<X>())> operator+(Box<I> const& bx1, Vector<X> const& v2) {
    return Box(cast_vector(bx1)+v2); }

template<class I, class X> inline Vector<decltype(declval<I>()-declval<X>())> operator-(Box<I> const& bx1, Vector<X> const& v2) {
    return cast_vector(bx1)-v2; }


inline FloatDPApproximateBox widen(const FloatDPApproximateBox& bx, FloatDPApproximation e) {
    FloatDPApproximation eps(e);
    FloatDPApproximateBox r(bx.dimension());
    for(DimensionType i=0; i!=bx.dimension(); ++i) {
        r[i]=ApproximateIntervalType(bx[i].lower_bound()-eps,bx[i].upper_bound()+eps);
    }
    return r;
}

inline FloatDPUpperBox widen(const FloatDPUpperBox& bx, FloatDPUpperBound eps) {
    FloatDPUpperBox r(bx.dimension());
    for(DimensionType i=0; i!=bx.dimension(); ++i) {
        r[i]=widen(bx[i],eps);
    }
    return r;
}
// TODO: Add widen for other generic values
inline FloatDPUpperBox widen(const FloatDPUpperBox& bx, ValidatedUpperNumber const& eps) {
    return widen(bx,FloatDPUpperBound(eps,dp));
}

inline FloatDPUpperBox widen(const FloatDPExactBox& bx, FloatDP eps) {
    return widen(reinterpret_cast<const FloatDPUpperBox&>(bx),FloatDPUpperBound(eps));
}

inline FloatDPUpperBox widen(const FloatDPUpperBox& bx) {
    FloatDPUpperBox r(bx.dimension());
    for(DimensionType i=0; i!=bx.dimension(); ++i) {
        r[i]=widen(bx[i]);
    }
    return r;
}

inline FloatDPExactBox widen_domain(const FloatDPUpperBox& bx) {
    FloatDPExactBox r(bx.dimension());
    for(DimensionType i=0; i!=bx.dimension(); ++i) {
        r[i]=widen_domain(bx[i]);
    }
    return r;
}

inline FloatDPLowerBox narrow(const FloatDPLowerBox& bx, FloatDPUpperBound eps) {
    FloatDPLowerBox r(bx.dimension());
    for(DimensionType i=0; i!=bx.dimension(); ++i) {
        r[i]=narrow(bx[i],eps);
    }
    return r;
}

inline FloatDPLowerBox narrow(const FloatDPLowerBox& bx) {
    FloatDPLowerBox r(bx.dimension());
    for(DimensionType i=0; i!=bx.dimension(); ++i) {
        r[i]=narrow(bx[i]);
    }
    return r;
}


template<class IVL> class BoxSet;

template<> class BoxSet<ExactIntervalType>
    : public virtual EffectiveEuclideanRegularLocatedSetInterface
    ,  public virtual Drawable2dInterface
    ,  public Box<ExactIntervalType>
{

  public:
    typedef typename EuclideanSetTraits::DimensionType DimensionType;
    using Box<ExactIntervalType>::Box;
    BoxSet(Box<ExactIntervalType>const& bx) : Box<ExactIntervalType>(bx) { }

    virtual ExactBoxSetType* clone() const override { return new ExactBoxSetType(*this); }
    virtual DimensionType dimension() const override final { return this->ExactBoxType::dimension(); }
    virtual LowerKleenean separated(const ExactBoxType& other) const override final { return this->ExactBoxType::separated(other); }
    virtual LowerKleenean overlaps(const ExactBoxType& other) const override final { return this->ExactBoxType::overlaps(other); }
    virtual LowerKleenean covers(const ExactBoxType& other) const override final { return this->ExactBoxType::covers(other); }
    virtual LowerKleenean inside(const ExactBoxType& other) const override final { return this->ExactBoxType::inside(other); }
    virtual UpperBoxType bounding_box() const override final { return this->ExactBoxType::bounding_box(); }
    virtual Void draw(CanvasInterface& c, const Projection2d& p) const override final { return Ariadne::draw(c,p,*this); }
    virtual OutputStream& _write(OutputStream& os) const override final { return os << static_cast<const ExactBoxType&>(*this); }
};

template<> class BoxSet<ApproximateIntervalType>
    : public virtual Drawable2dInterface
    , public Box<ApproximateIntervalType>
{

  public:
    using Box<ApproximateIntervalType>::Box;
    BoxSet(Box<ApproximateIntervalType>const& bx) : Box<ApproximateIntervalType>(bx) { }

    virtual ApproximateBoxSetType* clone() const override { return new ApproximateBoxSetType(*this); }
    virtual DimensionType dimension() const override final { return this->ApproximateBoxType::dimension(); }
    virtual Void draw(CanvasInterface& c, const Projection2d& p) const override final { return Ariadne::draw(c,p,*this); }
    virtual OutputStream& _write(OutputStream& os) const final { return os << static_cast<const ApproximateBoxType&>(*this); }
};


template<> class BoxSet<RealInterval>
    : public virtual RegularLocatedSetInterface<EffectiveTag,RealVector>
    , public virtual Drawable2dInterface
    , public Box<RealInterval>
{
    using BoxType = Box<RealInterval>;
  public:
    typedef typename EuclideanSetTraits::DimensionType DimensionType;

    using Box<RealInterval>::Box;
    BoxSet(Box<RealInterval>const& bx) : Box<RealInterval>(bx) { }

    virtual RealBoxSet* clone() const override { return new RealBoxSet(*this); }
    virtual DimensionType dimension() const override final { return this->Box<RealInterval>::dimension(); }
    virtual Void draw(CanvasInterface& c, const Projection2d& p) const override final { return this->Box<RealInterval>::draw(c,p); }
    virtual OutputStream& _write(OutputStream& os) const override final { return os << static_cast<const Box<RealInterval>&>(*this); }

    virtual LowerKleenean separated(const ExactBoxType& other) const override final { return this->BoxType::separated(DyadicBox(other)); }
    virtual LowerKleenean overlaps(const ExactBoxType& other) const override final { return this->BoxType::overlaps(DyadicBox(other)); }
    virtual LowerKleenean covers(const ExactBoxType& other) const override final { return this->BoxType::covers(DyadicBox(other)); }
    virtual LowerKleenean inside(const ExactBoxType& other) const override final { return this->BoxType::inside(DyadicBox(other)); }
    virtual UpperBoxType bounding_box() const override final { return UpperBoxType(this->BoxType::bounding_box()); }
};


} // namespace Ariadne


#endif /* ARIADNE_BOX_HPP */
