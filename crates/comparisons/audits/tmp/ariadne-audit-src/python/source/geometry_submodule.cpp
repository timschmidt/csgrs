/***************************************************************************
 *            geometry_submodule.cpp
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

#include "pybind11.hpp"
#include "pybind11.hpp"
#include "utilities.hpp"
#include "numeric_submodule.hpp"

#include "config.hpp"

#include "geometry/geometry.hpp"
#include "io/geometry2d.hpp"
#include "geometry/point.hpp"
#include "geometry/curve.hpp"
#include "geometry/interval.hpp"
#include "geometry/box.hpp"
#include "geometry/grid_paving.hpp"
#include "geometry/function_set.hpp"
#include "geometry/affine_set.hpp"

namespace Ariadne {


template<class F> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Bounds<F>>& x);

template<> struct PythonTemplateName<Point> { static std::string get() { return "Point"; } };
template<> struct PythonTemplateName<Interval> { static std::string get() { return "Interval"; } };
template<> struct PythonTemplateName<Box> { static std::string get() { return "Box"; } };

template<class X> struct PythonClassName<Point<X>> {
    std::string get() const { return python_template_class_name<X>("Point"); } };
template<class X> struct PythonClassName<Interval<X>> {
    std::string get() const { return python_template_class_name<X>("Interval"); } };
template<class X> struct PythonClassName<Box<X>> {
    std::string get() const { return python_template_class_name<X>("Box"); } };

template<> struct PythonClassName<Interval<FloatDP>> {
    std::string get() const { return "FloatDPExactInterval"; } };
template<> struct PythonClassName<Interval<FloatDPUpperBound>> {
    std::string get() const { return "FloatDPUpperInterval"; } };
template<> struct PythonClassName<Interval<FloatDPLowerBound>> {
    std::string get() const { return "FloatDPLowerInterval"; } };
template<> struct PythonClassName<Interval<FloatDPApproximation>> {
    std::string get() const { return "FloatDPApproximateInterval"; } };

template<class UB> struct PythonClassName<Box<Interval<UB>>> {
    std::string get() const { return python_class_name<UB>()+"Box"; } };
template<> struct PythonClassName<Box<Interval<FloatDP>>> {
    std::string get() const { return "FloatDPExactBox"; } };
template<> struct PythonClassName<Box<Interval<FloatDPUpperBound>>> {
    std::string get() const { return "FloatDPUpperBox"; } };
template<> struct PythonClassName<Box<Interval<FloatDPLowerBound>>> {
    std::string get() const { return "FloatDPLowerBox"; } };
template<> struct PythonClassName<Box<Interval<FloatDPApproximation>>> {
    std::string get() const { return "FloatDPApproximateBox"; } };


template<class UB> OutputStream& operator<<(OutputStream& os, const PythonLiteral<Interval<UB>>& repr) {
    Interval<UB> const& ivl=repr.reference();
    // Use tuple literal rather than dict, as not all types support hashing
    return os << "(" << python_literal(ivl.lower_bound()) << "," << python_literal(ivl.upper_bound()) << ")";
}

template<class UB> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Interval<UB>>& repr) {
    Interval<UB> const& ivl=repr.reference();
    return os << python_class_name<Interval<UB>>() << "(" << python_literal(ivl.lower_bound()) << "," << python_literal(ivl.upper_bound()) << ")";
}

template<class UB> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Box<UB>>& repr) {
    Box<UB> const& bx=repr.reference();
    os << python_class_name<Box<UB>>() << "([";
    for(SizeType i=0; i!=bx.dimension(); ++i) {
        if(i!=0) { os << ','; }
        os << python_literal(bx[i]);
    }
    os << "])";
    return os;
}

class DrawableWrapper
  : public pybind11::wrapper< Drawable2dInterface >
{
  public:
    virtual Drawable2dInterface* clone() const { return this->get_override("clone")(); }
    virtual Void draw(CanvasInterface& c, const Projection2d& p) const { this->get_override("draw")(c,p); }
    virtual DimensionType dimension() const { return this->get_override("dimension")(); }
    virtual OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};

template<class P, class T> class OpenSetWrapper;
template<class P, class T> class ClosedSetWrapper;
template<class P, class T> class OvertSetWrapper;
template<class P, class T> class BoundedSetWrapper;
template<class P, class T> class CompactSetWrapper;
template<class P, class T> class RegularSetWrapper;
template<class P, class T> class LocatedSetWrapper;

template<class T> class OpenSetWrapper<EffectiveTag,T>
  : public pybind11::wrapper<OpenSetInterface<EffectiveTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    OpenSetInterface<EffectiveTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    LowerKleenean covers(const BasicSetType& r) const { return this->get_override("covers")(r); }
    LowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};

template<class T> class ClosedSetWrapper<EffectiveTag,T>
  : public pybind11::wrapper<ClosedSetInterface<EffectiveTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    ClosedSetInterface<EffectiveTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    LowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};


template<class T> class OvertSetWrapper<EffectiveTag,T>
  : public pybind11::wrapper<OvertSetInterface<EffectiveTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    OvertSetInterface<EffectiveTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    LowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};


template<class T> class BoundedSetWrapper<EffectiveTag,T>
  : public pybind11::wrapper<BoundedSetInterface<EffectiveTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    typedef typename SetTraits<T>::BoundingSetType BoundingSetType;
    BoundedSetInterface<EffectiveTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    LowerKleenean inside(const BasicSetType& r) const { return this->get_override("inside")(); }
    BoundingSetType bounding_box() const { return this->get_override("bounding_box")(); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(); }
};

template<class T> class CompactSetWrapper<EffectiveTag,T>
  : public pybind11::wrapper<CompactSetInterface<EffectiveTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    typedef typename SetTraits<T>::BoundingSetType BoundingSetType;
    CompactSetInterface<EffectiveTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    LowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(); }
    LowerKleenean inside(const BasicSetType& r) const { return this->get_override("inside")(); }
    LowerKleenean is_bounded() const { return this->get_override("is_bounded")(); }
    BoundingSetType bounding_box() const { return this->get_override("bounding_box")(); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(); }
};

template<class T> class RegularSetWrapper<EffectiveTag,T>
  : public pybind11::wrapper<RegularSetInterface<EffectiveTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    RegularSetInterface<EffectiveTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    LowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    LowerKleenean covers(const BasicSetType& r) const { return this->get_override("covers")(r); }
    LowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};

template<class T> class LocatedSetWrapper<EffectiveTag,T>
  : public pybind11::wrapper<LocatedSetInterface<EffectiveTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    typedef typename SetTraits<T>::BoundingSetType BoundingSetType;
    LocatedSetInterface<EffectiveTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    LowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    LowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(r); }
    LowerKleenean inside(const BasicSetType& r) const { return this->get_override("inside")(r); }
    LowerKleenean is_bounded() const { return this->get_override("is_bounded")(); }
    BoundingSetType bounding_box() const { return this->get_override("bounding_box")(); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};




template<class T> class OpenSetWrapper<ValidatedTag,T>
  : public pybind11::wrapper<OpenSetInterface<ValidatedTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    OpenSetInterface<ValidatedTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    ValidatedLowerKleenean covers(const BasicSetType& r) const { return this->get_override("covers")(r); }
    ValidatedLowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};

template<class T> class ClosedSetWrapper<ValidatedTag,T>
  : public pybind11::wrapper<ClosedSetInterface<ValidatedTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    ClosedSetInterface<ValidatedTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    ValidatedLowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};


template<class T> class OvertSetWrapper<ValidatedTag,T>
  : public pybind11::wrapper<OvertSetInterface<ValidatedTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    OvertSetInterface<ValidatedTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    ValidatedLowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};


template<class T> class BoundedSetWrapper<ValidatedTag,T>
  : public pybind11::wrapper<BoundedSetInterface<ValidatedTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    typedef typename SetTraits<T>::BoundingSetType BoundingSetType;
    BoundedSetInterface<ValidatedTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    ValidatedLowerKleenean inside(const BasicSetType& r) const { return this->get_override("inside")(); }
    BoundingSetType bounding_box() const { return this->get_override("bounding_box")(); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(); }
};


template<class T> class CompactSetWrapper<ValidatedTag,T>
  : public pybind11::wrapper<CompactSetInterface<ValidatedTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    typedef typename SetTraits<T>::BoundingSetType BoundingSetType;
    CompactSetInterface<ValidatedTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    ValidatedLowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(); }
    ValidatedLowerKleenean inside(const BasicSetType& r) const { return this->get_override("inside")(); }
    ValidatedLowerKleenean is_bounded() const { return this->get_override("is_bounded")(); }
    BoundingSetType bounding_box() const { return this->get_override("bounding_box")(); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(); }
};

template<class T> class RegularSetWrapper<ValidatedTag,T>
  : public pybind11::wrapper<RegularSetInterface<ValidatedTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    RegularSetInterface<ValidatedTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    ValidatedLowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    ValidatedLowerKleenean covers(const BasicSetType& r) const { return this->get_override("covers")(r); }
    ValidatedLowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(r); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};

template<class T> class LocatedSetWrapper<ValidatedTag,T>
  : public pybind11::wrapper<LocatedSetInterface<ValidatedTag,T>>
{
  public:
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    typedef typename SetTraits<T>::BoundingSetType BoundingSetType;
    LocatedSetInterface<ValidatedTag,T>* clone() const { return this->get_override("clone")(); }
    SizeType dimension() const { return this->get_override("dimension")(); }
    ValidatedLowerKleenean overlaps(const BasicSetType& r) const { return this->get_override("overlaps")(r); }
    ValidatedLowerKleenean separated(const BasicSetType& r) const { return this->get_override("separated")(r); }
    ValidatedLowerKleenean inside(const BasicSetType& r) const { return this->get_override("inside")(r); }
    ValidatedLowerKleenean is_bounded() const { return this->get_override("is_bounded")(); }
    BoundingSetType bounding_box() const { return this->get_override("bounding_box")(); }
    OutputStream& _write(OutputStream& os) const { return this->get_override("_write")(os); }
};


} // namespace Ariadne


using namespace Ariadne;

template<class X> X from_python_object_or_literal_allowing_default_precision(pybind11::handle h) {
    if constexpr (Same<X,Decimal>) {
        try {
            return X(pybind11::cast<double>(h));
        } catch(pybind11::cast_error&) { }
    }
    if constexpr (Constructible<X,String>) {
        try {
            return X(pybind11::cast<String>(h));
        } catch (pybind11::cast_error&) { }
    }
    if constexpr (ConstructibleGivenDefaultPrecision<X,String>) {
        try {
            PrecisionType<X> pr;
            return X(pybind11::cast<String>(h),pr);
        } catch (pybind11::cast_error&) { }
    }
    if constexpr (ConstructibleGivenDefaultPrecision<X,Dyadic>) {
        PrecisionType<X> pr;
        try {
            return X(pybind11::cast<Dyadic>(h),pr);
        } catch (pybind11::cast_error&) { }
        try {
            return X(Dyadic(pybind11::cast<String>(h)),pr);
        } catch (pybind11::cast_error&) { }
    }
    return pybind11::cast<X>(h);
}

template<class IVL> IVL interval_from_pair(pybind11::handle lh, pybind11::handle uh) {
    typedef typename IVL::LowerBoundType LB;
    typedef typename IVL::UpperBoundType UB;
    LB lb = from_python_object_or_literal_allowing_default_precision<LB>(lh);
    UB ub = from_python_object_or_literal_allowing_default_precision<UB>(uh);
    return IVL(lb,ub);
}

template<class IVL> IVL interval_from_dict(pybind11::dict dct) {
    assert(dct.size()==1);
    pybind11::detail::dict_iterator::reference item = *dct.begin();
    pybind11::handle lh = item.first;
    pybind11::handle uh = item.second;
    return interval_from_pair<IVL>(lh,uh);
}

template<class IVL> IVL interval_from_tuple(pybind11::tuple tup) {
    assert(tup.size()==2);
    pybind11::handle lh = tup[0];
    pybind11::handle uh = tup[1];
    return interval_from_pair<IVL>(lh,uh);
}

template<class BX> BX box_from_list(pybind11::list lst) {
    typedef typename BX::IntervalType IVL;
    Array<IVL> ary( lst.size(), [&lst](SizeType i){return pybind11::cast<IVL>(lst[i]);} );
    return BX(ary);
}


Void export_drawable_interface(pybind11::module& module) {
    pybind11::class_<Drawable2dInterface,DrawableWrapper> drawable_class(module, "Drawable");
    drawable_class.def("clone", &Drawable2dInterface::clone);
    drawable_class.def("draw", &Drawable2dInterface::draw);
    drawable_class.def("dimension", &Drawable2dInterface::dimension);
}


Void export_set_interface(pybind11::module& module) {
    using P=EffectiveTag; using T=RealVector;
    typedef typename SetTraits<T>::BasicSetType BasicSetType;
    typedef typename SetInterfaceBase<T>::BoundingSetType BoundingSetType;

    pybind11::class_<OvertSetInterface<P,T>, OvertSetWrapper<P,T>> overt_set_interface_class(module,"OvertSet");
    overt_set_interface_class.def("overlaps",(LowerKleenean(OvertSetInterface<P,T>::*)(const BasicSetType& bx)const) &OvertSetInterface<P,T>::overlaps);

    pybind11::class_<OpenSetInterface<P,T>, OpenSetWrapper<P,T>> open_set_interface_class(module,"OpenSet",overt_set_interface_class);
    open_set_interface_class.def("covers",(LowerKleenean(OpenSetInterface<P,T>::*)(const BasicSetType& bx)const) &OpenSetInterface<P,T>::covers);

    pybind11::class_<ClosedSetInterface<P,T>, ClosedSetWrapper<P,T>> closed_set_interface_class(module,"ClosedSet");
    closed_set_interface_class.def("separated",(LowerKleenean(ClosedSetInterface<P,T>::*)(const BasicSetType& bx)const) &ClosedSetInterface<P,T>::separated);

    pybind11::class_<BoundedSetInterface<P,T>, BoundedSetWrapper<P,T>> bounded_set_interface_class(module,"BoundedSet");
    bounded_set_interface_class.def("inside",(LowerKleenean(BoundedSetInterface<P,T>::*)(const BasicSetType& bx)const) &BoundedSetInterface<P,T>::inside);
    bounded_set_interface_class.def("bounding_box", (BoundingSetType(BoundedSetInterface<P,T>::*)()const)&BoundedSetInterface<P,T>::bounding_box);

    pybind11::class_<CompactSetInterface<P,T>, CompactSetWrapper<P,T>, ClosedSetInterface<P,T>, BoundedSetInterface<P,T>> compact_set_interface_class(module,"CompactSet", pybind11::multiple_inheritance());

    pybind11::class_<RegularSetInterface<P,T>, RegularSetWrapper<P,T>, OpenSetInterface<P,T>,ClosedSetInterface<P,T>> regular_set_interface_class(module,"RegularSet", pybind11::multiple_inheritance());
    pybind11::class_<LocatedSetInterface<P,T>, LocatedSetWrapper<P,T>, OvertSetInterface<P,T>,CompactSetInterface<P,T>> located_set_interface_class(module,"LocatedSet", pybind11::multiple_inheritance());


    pybind11::class_<OvertSetInterface<ValidatedTag,T>, OvertSetWrapper<ValidatedTag,T>> validated_overt_set_interface_class(module,"ValidatedOvertSet");
    validated_overt_set_interface_class.def("overlaps",(ValidatedLowerKleenean(OvertSetInterface<ValidatedTag,T>::*)(const BasicSetType& bx)const) &OvertSetInterface<ValidatedTag,T>::overlaps);

    pybind11::class_<OpenSetInterface<ValidatedTag,T>, OpenSetWrapper<ValidatedTag,T>> validated_open_set_interface_class(module,"ValidatedOpenSet",validated_overt_set_interface_class);
    validated_open_set_interface_class.def("covers",(ValidatedLowerKleenean(OpenSetInterface<ValidatedTag,T>::*)(const BasicSetType& bx)const) &OpenSetInterface<ValidatedTag,T>::covers);

    pybind11::class_<ClosedSetInterface<ValidatedTag,T>, ClosedSetWrapper<ValidatedTag,T>> validated_closed_set_interface_class(module,"ValidatedClosedSet");
    validated_closed_set_interface_class.def("separated",(ValidatedLowerKleenean(ClosedSetInterface<ValidatedTag,T>::*)(const BasicSetType& bx)const) &ClosedSetInterface<ValidatedTag,T>::separated);

    pybind11::class_<BoundedSetInterface<ValidatedTag,T>, BoundedSetWrapper<ValidatedTag,T>> validated_bounded_set_interface_class(module,"ValidatedBoundedSet");
    validated_bounded_set_interface_class.def("inside",(ValidatedLowerKleenean(BoundedSetInterface<ValidatedTag,T>::*)(const BasicSetType& bx)const) &BoundedSetInterface<ValidatedTag,T>::inside);
    validated_bounded_set_interface_class.def("bounding_box", (BoundingSetType(BoundedSetInterface<ValidatedTag,T>::*)()const)&BoundedSetInterface<ValidatedTag,T>::bounding_box);

    pybind11::class_<CompactSetInterface<ValidatedTag,T>, CompactSetWrapper<ValidatedTag,T>, ClosedSetInterface<ValidatedTag,T>, BoundedSetInterface<ValidatedTag,T>> validated_compact_set_interface_class(module,"ValidatedCompactSet", pybind11::multiple_inheritance());

    pybind11::class_<RegularSetInterface<ValidatedTag,T>, RegularSetWrapper<ValidatedTag,T>, OpenSetInterface<ValidatedTag,T>, ClosedSetInterface<ValidatedTag,T>> validated_regular_set_interface_class(module,"ValidatedRegularSet", pybind11::multiple_inheritance());
    pybind11::class_<LocatedSetInterface<ValidatedTag,T>, LocatedSetWrapper<ValidatedTag,T>, OvertSetInterface<ValidatedTag,T>,CompactSetInterface<ValidatedTag,T>> validated_located_set_interface_class(module,"ValidatedLocatedSet", pybind11::multiple_inheritance());
}


template<class PT> PT point_from_python(pybind11::list pylst) {
    typedef typename PT::ValueType X;
    std::vector<X> lst=pybind11::cast<std::vector<X>>(pylst);
    Array<X> ary(lst.begin(),lst.end());
    return PT(Vector<X>(ary));
}

template<class PT> Void export_point(pybind11::module& module, std::string name=python_class_name<PT>())
{
    typedef typename PT::ValueType X;
    pybind11::class_<PT, Drawable2dInterface> point_class(module,name.c_str());
    point_class.def(pybind11::init(&point_from_python<PT>));
    point_class.def(pybind11::init<PT>());
    if constexpr (DefaultConstructible<X>) {
        point_class.def(pybind11::init<Nat>());
    }
    if constexpr (HasPrecisionType<X>) {
        typedef typename X::PrecisionType PR;
        point_class.def(pybind11::init<Nat,PR>());
    }
    point_class.def("__getitem__", &__getitem__<PT,Int,X>);
    point_class.def("__str__", &__cstr__<PT>);
}

Void export_points(pybind11::module& module) {
    export_point<RealPoint>(module);
    export_point<FloatDPPoint>(module);
    export_point<FloatDPBoundsPoint>(module);
    export_point<FloatDPApproximationPoint>(module);

    template_<Point> point_template(module);
    point_template.instantiate<Real>();
    point_template.instantiate<FloatDP>();
    point_template.instantiate<FloatDPBounds>();
    point_template.instantiate<FloatDPApproximation>();
}

template<class IVL> Void export_interval_arithmetic(pybind11::module& module, pybind11::class_<IVL>& interval_class) {
}

template<> Void export_interval_arithmetic(pybind11::module& module, pybind11::class_<UpperIntervalType>& interval_class) {
    define_arithmetic(module,interval_class);
    define_transcendental(module,interval_class);

    define_mixed_arithmetic(module,interval_class,Tag<ValidatedNumber>());
}

template<template<class>class T,class F> Void export_conversions(pybind11::class_<T<Approximation<F>>>& cls) {
    cls.def(pybind11::init<T<F>>());
    cls.def(pybind11::init<T<Bounds<F>>>());
    cls.def(pybind11::init<T<UpperBound<F>>>());
    cls.def(pybind11::init<T<LowerBound<F>>>());
}
template<template<class>class T,class F> Void export_conversions(pybind11::class_<T<LowerBound<F>>>& cls) {
    cls.def(pybind11::init<T<F>>());
    cls.def(pybind11::init<T<Bounds<F>>>());
}
template<template<class>class T,class F> Void export_conversions(pybind11::class_<T<UpperBound<F>>>& cls) {
    cls.def(pybind11::init<T<F>>());
    cls.def(pybind11::init<T<Bounds<F>>>());
}
template<template<class>class T,class F> Void export_conversions(pybind11::class_<T<Bounds<F>>>& cls) {
    cls.def(pybind11::init<T<F>>());
}
template<template<class>class T,class F> Void export_conversions(pybind11::class_<T<F>>& cls) {
}



template<class IVL> Void export_interval(pybind11::module& module, std::string name=python_class_name<IVL>()) {
    typedef IVL IntervalType;
    typedef typename IntervalType::LowerBoundType LowerBoundType;
    typedef typename IntervalType::UpperBoundType UpperBoundType;
    typedef typename IntervalType::MidpointType MidpointType;

    typedef decltype(contains(declval<IntervalType>(),declval<MidpointType>())) ContainsType;
    typedef decltype(disjoint(declval<IntervalType>(),declval<IntervalType>())) DisjointType;
    typedef decltype(subset(declval<Interval<UpperBoundType>>(),declval<Interval<LowerBoundType>>())) SubsetType;

    pybind11::class_< IntervalType > interval_class(module,name.c_str());
    interval_class.def(pybind11::init<IntervalType>());
    interval_class.def(pybind11::init<MidpointType>());
    interval_class.def(pybind11::init<LowerBoundType,UpperBoundType>());
    interval_class.def(pybind11::init([](pybind11::dict pydct){return interval_from_dict<IntervalType>(pydct);}));
    interval_class.def(pybind11::init([](pybind11::tuple pytup){return interval_from_tuple<IntervalType>(pytup);}));
    pybind11::implicitly_convertible<pybind11::dict, IntervalType>();
    pybind11::implicitly_convertible<pybind11::tuple, IntervalType>();

    if constexpr (Constructible<UpperBoundType,String>) {
        interval_class.def(pybind11::init([](String lb, String ub){return Interval(LowerBoundType(lb),UpperBoundType(ub));}));
    } else if constexpr (ConstructibleGivenDefaultPrecision<UpperBoundType,String>) {
        interval_class.def(pybind11::init([](String lb, String ub){PrecisionType<UpperBoundType> pr;return Interval(LowerBoundType(lb,pr),UpperBoundType(ub,pr));}));
    } else if constexpr (ConstructibleGivenDefaultPrecision<UpperBoundType,Decimal>) {
        interval_class.def(pybind11::init([](String lb, String ub){PrecisionType<UpperBoundType> pr;return Interval(LowerBoundType(Decimal(lb),pr),UpperBoundType(Decimal(ub),pr));}));
    } else if constexpr (ConstructibleGivenDefaultPrecision<UpperBoundType,Dyadic>) {
        interval_class.def(pybind11::init([](String lb, String ub){PrecisionType<UpperBoundType> pr;return Interval(LowerBoundType(Dyadic(lb),pr),UpperBoundType(Dyadic(ub),pr));}));
    }

    if constexpr (Constructible<IntervalType,IntervalDomainType> and not Same<IntervalType,IntervalDomainType>) {
        interval_class.def(pybind11::init<IntervalDomainType>());
        if constexpr(Convertible<IntervalDomainType,IntervalType>) {
             pybind11::implicitly_convertible<IntervalDomainType,IntervalType>();
         }
    }
    if constexpr (Constructible<IntervalType,DyadicInterval> and not Same<IntervalType,DyadicInterval>) {
        interval_class.def(pybind11::init<DyadicInterval>());
        interval_class.def(pybind11::init([](Dyadic l, Dyadic u){return IntervalType(DyadicInterval(l,u));}));
        if constexpr(Convertible<DyadicInterval,IntervalType>) {
            pybind11::implicitly_convertible<DyadicInterval,IntervalType>();
        }
    }
    if constexpr (Constructible<IntervalType,RationalInterval> and not Same<IntervalType,RationalInterval>) {
        interval_class.def(pybind11::init<RationalInterval>());
        interval_class.def(pybind11::init([](Rational l, Rational u){return IntervalType(RationalInterval(l,u));}));
        if constexpr(Convertible<RationalInterval,IntervalType>) {
            pybind11::implicitly_convertible<RationalInterval,IntervalType>();
        }
    }
    if constexpr (Constructible<IntervalType,RealInterval> and not Same<IntervalType,RealInterval>) {
        interval_class.def(pybind11::init<RealInterval>());
        interval_class.def(pybind11::init([](Real l, Real u){return IntervalType(RealInterval(l,u));}));
        if constexpr(Convertible<RealInterval,IntervalType>) {
            pybind11::implicitly_convertible<DyadicInterval,IntervalType>();
        }
    }
    if constexpr (HasPrecisionType<UpperBoundType>) {
        typedef PrecisionType<UpperBoundType> PrecisionType;
        if constexpr (Constructible<UpperBoundType,String,PrecisionType>) {
            interval_class.def(pybind11::init([](String ls, String us, PrecisionType pr){return IntervalType(LowerBoundType(ls,pr),UpperBoundType(us,pr));}));
        } else if constexpr (Constructible<UpperBoundType,Decimal,PrecisionType>) {
            interval_class.def(pybind11::init([](String ls, String us, PrecisionType pr){return IntervalType(LowerBoundType(Decimal(ls),pr),UpperBoundType(Decimal(us),pr));}));
        }

        if constexpr (Constructible<IntervalType,FloatBounds<PrecisionType>>) {
            interval_class.def(pybind11::init<FloatBounds<PrecisionType>>());
            if constexpr(Convertible<FloatBounds<PrecisionType>,IntervalType>) {
                pybind11::implicitly_convertible<FloatBounds<PrecisionType>,IntervalType>();
            }
        }
        if constexpr (Constructible<IntervalType,RealInterval,PrecisionType>) {
            interval_class.def(pybind11::init<RealInterval,PrecisionType>());
            interval_class.def(pybind11::init([](Real l, Real u, PrecisionType pr){return IntervalType(RealInterval(l,u),pr);}));
        } else if constexpr (Constructible<IntervalType,DyadicInterval,PrecisionType>) {
            interval_class.def(pybind11::init<DyadicInterval,PrecisionType>());
            interval_class.def(pybind11::init([](Dyadic l, Dyadic u, PrecisionType pr){return IntervalType(DyadicInterval(l,u),pr);}));
        }
        export_conversions(interval_class);
    }

    export_interval_arithmetic(module,interval_class);

    if constexpr (HasEquality<IVL,IVL>) {
        interval_class.def("__eq__",  &__eq__<IVL,IVL , Return<EqualityType<IVL,IVL>> >);
        interval_class.def("__ne__",  &__ne__<IVL,IVL , Return<InequalityType<IVL,IVL>> >);
    }

    interval_class.def("lower_bound", &IntervalType::lower_bound);
    interval_class.def("upper_bound", &IntervalType::upper_bound);
    interval_class.def("centre", &IntervalType::centre  );
    interval_class.def("midpoint", &IntervalType::midpoint);
    interval_class.def("radius", &IntervalType::radius);
    interval_class.def("width", &IntervalType::width);
    interval_class.def("empty", &IntervalType::is_empty);
    interval_class.def("__str__",&__cstr__<IntervalType>);
    interval_class.def("__repr__",&__repr__<IntervalType>);

    interval_class.def("contains", (ContainsType(*)(IntervalType const&,MidpointType const&)) &contains);
    module.def("contains", (ContainsType(*)(IntervalType const&,MidpointType const&)) &contains);
    if constexpr (HasPrecisionType<UpperBoundType>) {
        typedef PrecisionType<UpperBoundType> PrecisionType;
        if constexpr (Same<UpperBoundType,FloatUpperBound<PrecisionType>> and false) {
            interval_class.def("contains", (ContainsType(*)(IntervalType const&, FloatBounds<PrecisionType> const&)) &contains);
            module.def("contains", (ContainsType(*)(IntervalType const&, FloatBounds<PrecisionType> const&)) &contains);
        }
    }

    module.def("centre", &IntervalType::centre);
    module.def("midpoint", &IntervalType::midpoint);
    module.def("radius", &IntervalType::radius);
    module.def("width", &IntervalType::width);

    module.def("disjoint", (DisjointType(*)(IntervalType const&,IntervalType const&)) &disjoint);
    module.def("subset", (SubsetType(*)(Interval<UpperBoundType> const&,Interval<LowerBoundType> const&)) &subset);

    module.def("intersection", (IntervalType(*)(IntervalType const&,IntervalType const&)) &intersection);
    module.def("hull", (IntervalType(*)(IntervalType const&, IntervalType const&)) &hull);
    module.def("split", (Pair<IntervalType,IntervalType>(*)(IntervalType const&)) &split);

    if constexpr (Same<IVL,IntervalDomainType>) {
        module.attr("IntervalDomainType")=interval_class;
    } else if constexpr (Same<IVL,IntervalValidatedRangeType>) {
        module.attr("IntervalValidatedRangeType")=interval_class;
    } else if constexpr (Same<IVL,IntervalApproximateRangeType>) {
        module.attr("IntervalApproximateRangeType")=interval_class;
    }

}

Void export_intervals(pybind11::module& module) {
//    export_interval<ExactIntervalType>(module,"ExactIntervalType");
//    export_interval<UpperIntervalType>(module,"UpperIntervalType");
//    export_interval<ApproximateIntervalType>(module,"ApproximateIntervalType");
    export_interval<DyadicInterval>(module);
    export_interval<DecimalInterval>(module);
    export_interval<RationalInterval>(module);
    export_interval<RealInterval>(module);
    pybind11::implicitly_convertible<DyadicInterval,DecimalInterval>();
    pybind11::implicitly_convertible<DyadicInterval,RationalInterval>();
    pybind11::implicitly_convertible<DyadicInterval,RealInterval>();
    pybind11::implicitly_convertible<DecimalInterval,RationalInterval>();
    pybind11::implicitly_convertible<DecimalInterval,RealInterval>();
    pybind11::implicitly_convertible<RationalInterval,RealInterval>();

    export_interval<FloatDPExactInterval>(module);
    export_interval<FloatDPLowerInterval>(module);
    export_interval<FloatDPUpperInterval>(module);
    export_interval<FloatDPApproximateInterval>(module);

    pybind11::implicitly_convertible<FloatDPExactInterval,FloatDPUpperInterval>();
    pybind11::implicitly_convertible<FloatDPExactInterval,FloatDPLowerInterval>();
//    pybind11::implicitly_convertible<FloatDPExactInterval,FloatDPApproximateInterval>();
//    pybind11::implicitly_convertible<FloatDPUpperInterval,FloatDPApproximateInterval>();
//    pybind11::implicitly_convertible<FloatDPLowerInterval,FloatDPApproximateInterval>();


    //    export_interval<FloatMPUpperInterval>(module,"FloatMPUpperInterval");
    module.def("cast_singleton", (FloatDPBounds(*)(Interval<FloatDPUpperBound> const&)) &cast_singleton);
    module.def("cast_singleton", (FloatMPBounds(*)(Interval<FloatMPUpperBound> const&)) &cast_singleton);

    module.def("image", (UpperIntervalType(*)(UpperIntervalType const&, ValidatedScalarUnivariateFunction const&)) &_image_);

    template_<Interval> interval_template(module);
    interval_template.instantiate<Dyadic>();
    interval_template.instantiate<Decimal>();
    interval_template.instantiate<Rational>();
    interval_template.instantiate<Real>();
    interval_template.instantiate<FloatDP>();
    interval_template.instantiate<FloatDPUpperBound>();
    interval_template.instantiate<FloatDPLowerBound>();
    interval_template.instantiate<FloatDPApproximation>();
}
template<class UB> using BoxWithUpperBound=Box<Interval<UB>>;

template<class BX> Void export_box(pybind11::module& module, std::string name=python_class_name<BX>())
{
    using IVL=typename BX::IntervalType;

    using BoxType = BX;
    using IntervalType = IVL;

    typedef typename BoxType::MidpointType MidpointType;

    typedef decltype(contains(declval<BoxType>(),declval<MidpointType>())) ContainsType;
    typedef decltype(disjoint(declval<BX>(),declval<BX>())) DisjointType;
    typedef decltype(subset(declval<BX>(),declval<BX>())) SubsetType;
    typedef decltype(separated(declval<BX>(),declval<BX>())) SeparatedType;
    typedef decltype(overlap(declval<BX>(),declval<BX>())) OverlapType;
    typedef decltype(covers(declval<BX>(),declval<BX>())) CoversType;
    typedef decltype(inside(declval<BX>(),declval<BX>())) InsideType;

    //NOTE: Boxes do not inherit SetInterface<T>s or Drawable2dInterface in C++ API
    //pybind11::class_<BasicSetType,pybind11::bases<CompactSetInterface<T>,OpenSetInterface<T>,Drawable2dInterface>>
    pybind11::class_<BoxType> box_class(module,name.c_str());
    box_class.def(pybind11::init<BoxType>());
    box_class.def(pybind11::init<DimensionType>());

    define_conversion<BoxDomainType>(box_class);
    define_conversion<DyadicBox>(box_class);
    define_conversion<DecimalBox>(box_class);
    define_conversion<RationalBox>(box_class);
    define_conversion<RealBox>(box_class);

    if constexpr (HasPrecisionType<IntervalType>) {
        typedef PrecisionType<IntervalType> PrecisionType;
        if constexpr (Constructible<BoxType,RealBox,PrecisionType>) {
            box_class.def(pybind11::init<RealBox,PrecisionType>());
        } else if constexpr (Constructible<BoxType,DyadicBox,PrecisionType>) {
            box_class.def(pybind11::init<DyadicBox,PrecisionType>());
        }

        typedef typename IntervalType::UpperBoundType FloatType;
        export_conversions<BoxWithUpperBound,FloatType>(box_class);
    }

    box_class.def(pybind11::init<Array<IntervalType>>());
    box_class.def(pybind11::init(&box_from_list<BoxType>));
    pybind11::implicitly_convertible<pybind11::list,BoxType>();

    if constexpr (HasEquality<BX,BX>) {
        box_class.def("__eq__",  __eq__<BX,BX , Return<EqualityType<BX,BX>> >);
        box_class.def("__ne__",  __ne__<BX,BX , Return<InequalityType<BX,BX>> >);
    }

    box_class.def("dimension", (DimensionType(BX::*)()const) &BX::dimension);
    box_class.def("__getitem__", &__getitem__<BX,Int>);
    box_class.def("centre", (typename BX::CentreType(BX::*)()const) &BX::centre);
    box_class.def("midpoint", (typename BX::MidpointType(BX::*)()const) &BX::midpoint);
    box_class.def("radius", (typename BX::RadiusType(BX::*)()const) &BX::radius);
    box_class.def("radii", (Vector<typename BX::RadiusType>(BX::*)()const) &BX::radii);
    box_class.def("widths", (Vector<typename BX::RadiusType>(BX::*)()const) &BX::widths);
    box_class.def("measure", (typename BX::MeasureType(BX::*)()const) &BX::measure);
    box_class.def("volume", (typename BX::MeasureType(BX::*)()const) &BX::volume);
    box_class.def("separated", (SeparatedType(BX::*)(const BX&)const) &BX::separated);
    box_class.def("overlaps", (OverlapType(BX::*)(const BX&)const) &BX::overlaps);
    box_class.def("covers", (CoversType(BX::*)(const BX&)const) &BX::covers);
    box_class.def("inside", (InsideType(BX::*)(const BX&)const) &BX::inside);
    box_class.def("is_empty", (SeparatedType(BX::*)()const) &BX::is_empty);
    box_class.def("split", (Pair<BX,BX>(BX::*)()const) &BX::split);
    box_class.def("split", (Pair<BX,BX>(BX::*)(SizeType)const) &BX::split);
    box_class.def("__str__",&__cstr__<BX>);
    box_class.def("__repr__",&__repr__<BX>);

    module.def("centre", (typename BX::CentreType(*)(BX const&)) &centre);
    module.def("midpoint", (typename BX::MidpointType(*)(BX const&)) &midpoint);
    module.def("radius", (typename BX::RadiusType(*)(BX const&)) &radius);
    module.def("radii", (Vector<typename BX::RadiusType>(*)(BX const&)) &radii);
    module.def("widths", (Vector<typename BX::RadiusType>(*)(BX const&)) &widths);
    module.def("measure", (typename BX::MeasureType(*)(BX const&)) &measure);
    module.def("volume", (typename BX::MeasureType(*)(BX const&)) &volume);

    box_class.def("contains", (ContainsType(BX::*)(MidpointType const&)const) &BX::contains);
    module.def("contains", (ContainsType(*)(BX const&,MidpointType const&)) &contains);
    if constexpr (HasPrecisionType<typename IntervalType::UpperBoundType>) {
        typedef PrecisionType<typename IntervalType::UpperBoundType> PrecisionType;
        if constexpr (Same<typename IntervalType::UpperBoundType,FloatUpperBound<PrecisionType>> and false) {
            box_class.def("contains", (ContainsType(*)(BoxType const&, Vector<FloatBounds<PrecisionType>> const&)) &contains);
            module.def("contains", (ContainsType(*)(BoxType const&, Vector<FloatBounds<PrecisionType>> const&)) &contains);
        }
    }

    module.def("disjoint", (DisjointType(*)(BX const&,BX const&)) &disjoint);
    if constexpr (Same<typename IVL::UpperBoundType, typename IVL::LowerBoundType>) {
        module.def("subset", (SubsetType(*)(const BX&,const BX&)) &subset);
    } else {
        typedef Box<Interval<typename IVL::LowerBoundType>> LBX;
        using SubsetOfLowerType = decltype(subset(declval<BX>(),declval<LBX>()));
        module.def("subset", (SubsetOfLowerType(*)(const BX&,const LBX&)) &subset);
    }

    module.def("product", (BX(*)(const BX&,const IVL&)) &product);
    module.def("product", (BX(*)(const BX&,const BX&)) &product);
    module.def("hull", (BX(*)(const BX&,const BX&)) &hull);
    module.def("intersection", (BX(*)(const BX&,const BX&)) &intersection);
    module.def("split", (Pair<BX,BX>(*)(BX const&)) &split);

    if constexpr (Same<BX,BoxDomainType>) {
        module.attr("BoxDomainType")=box_class;
    } else if constexpr (Same<BX,BoxValidatedRangeType>) {
        module.attr("BoxValidatedRangeType")=box_class;
    } else if constexpr (Same<BX,BoxApproximateRangeType>) {
        module.attr("BoxApproximateRangeType")=box_class;
    }

    module.def("cast_exact",(IntervalDomainType(*)(IntervalApproximateRangeType const&)) &cast_exact_interval);
    module.def("cast_exact",(BoxDomainType(*)(BoxApproximateRangeType const&)) &cast_exact_box);

}


template<class BX> Void export_simple_box(pybind11::module& module, std::string name=python_class_name<BX>())
{
    using BoxType = BX;
    pybind11::class_<BoxType> box_class(module,name.c_str());
    box_class.def(pybind11::init(&box_from_list<BoxType>));
    if constexpr (Constructible<BX,DyadicBox>) {
        box_class.def(pybind11::init<DyadicBox>());
    }
    if constexpr (Constructible<BX,DecimalBox>) {
        box_class.def(pybind11::init<DecimalBox>());
    }
    if constexpr (Constructible<BX,RationalBox>) {
        box_class.def(pybind11::init<RationalBox>());
    }
    if constexpr (Constructible<BoxType,BoxDomainType>) {
        box_class.def(pybind11::init<BoxDomainType>());
        pybind11::implicitly_convertible<BoxDomainType,BoxType>();
    }
    box_class.def("dimension", (DimensionType(BX::*)()const) &BX::dimension);
    box_class.def("__getitem__", &__getitem__<BX,Int>);
    box_class.def("__str__",&__cstr__<BoxType>);
    box_class.def("__repr__",&__repr__<BoxType>);
}

Void export_boxes(pybind11::module& module) {
    export_box<RealBox>(module);
    export_simple_box<RationalBox>(module);
    export_simple_box<DecimalBox>(module);
    export_simple_box<DyadicBox>(module);
//    export_box<ExactBoxType>(module,"ExactBoxType");
//    export_box<UpperBoxType>(module,"UpperBoxType");
//    export_box<ApproximateBoxType>(module,"ApproximateBoxType");
    pybind11::implicitly_convertible<DyadicBox,DecimalBox>();
    pybind11::implicitly_convertible<DyadicBox,RationalBox>();
    pybind11::implicitly_convertible<DyadicBox,RealBox>();
    pybind11::implicitly_convertible<DecimalBox,RationalBox>();
    pybind11::implicitly_convertible<DecimalBox,RealBox>();
    pybind11::implicitly_convertible<RationalBox,RealBox>();

    export_box<FloatDPExactBox>(module);
    export_box<FloatDPUpperBox>(module);
    export_box<FloatDPLowerBox>(module);
    export_box<FloatDPApproximateBox>(module);
//    export_box<FloatMPUpperBox>(module);

    pybind11::implicitly_convertible<FloatDPExactBox,FloatDPUpperBox>();
    pybind11::implicitly_convertible<FloatDPExactBox,FloatDPLowerBox>();
    pybind11::implicitly_convertible<FloatDPExactBox,FloatDPApproximateBox>();
    pybind11::implicitly_convertible<FloatDPUpperBox,FloatDPApproximateBox>();
    pybind11::implicitly_convertible<FloatDPLowerBox,FloatDPApproximateBox>();

    module.def("widen", (FloatDPUpperBox(*)(FloatDPExactBox const&, FloatDP eps)) &widen);
    module.def("image", (FloatDPUpperBox(*)(FloatDPUpperBox const&, ValidatedVectorMultivariateFunction const&)) &_image_);
    module.def("image", (FloatDPUpperInterval(*)(FloatDPUpperBox const&, ValidatedScalarMultivariateFunction const&)) &_image_);
    module.def("image", (FloatDPUpperBox(*)(FloatDPUpperInterval const&, ValidatedVectorUnivariateFunction const&)) &_image_);

    module.def("cast_singleton", (Vector<FloatDPBounds>(*)(Box<Interval<FloatDPUpperBound>> const&)) &cast_singleton);
    module.def("cast_singleton", (Vector<FloatMPBounds>(*)(Box<Interval<FloatMPUpperBound>> const&)) &cast_singleton);

    template_<Box> box_template(module);
    // TODO: Change templates so that
    box_template.as_instantiate<DyadicBox,Dyadic>();
    box_template.as_instantiate<DecimalBox,Decimal>();
    box_template.as_instantiate<RationalBox,Rational>();
    box_template.as_instantiate<RealBox,Real>();
    box_template.as_instantiate<FloatDPExactBox,FloatDP>();
    box_template.as_instantiate<FloatDPUpperBox,FloatDPUpperBound>();
    box_template.as_instantiate<FloatDPLowerBox,FloatDPLowerBound>();
    box_template.as_instantiate<FloatDPApproximateBox,FloatDPApproximation>();

}

/*

Void export_zonotope(pybind11::module& module)
{
    pybind11::class_<Zonotope,pybind11::bases<CompactSetInterface<T>,OpenSetInterface<T>,Drawable2dInterface>> zonotope_class(module,"Zonotope");
    zonotope_class.def(pybind11::init<Zonotope>());
    zonotope_class.def(pybind11::init<Vector<FloatDP>,Matrix<FloatDP>,Vector<FloatDPError>>());
    zonotope_class.def(pybind11::init<Vector<FloatDP>,Matrix<FloatDP>>());
    zonotope_class.def(pybind11::init<BasicSetType>());
    zonotope_class.def("centre",&Zonotope::centre);
    zonotope_class.def("generators",&Zonotope::generators);
    zonotope_class.def("error",&Zonotope::error);
    zonotope_class.def("contains",&Zonotope::contains);
    zonotope_class.def("split", (ListSet<Zonotope>(*)(const Zonotope&)) &split);
    zonotope_class.def("__str__",&__cstr__<Zonotope>);

    module.def("contains", (ValidatedKleenean(*)(const Zonotope&,const ExactPoint&)) &contains);
    module.def("separated", (ValidatedKleenean(*)(const Zonotope&,const BasicSetType&)) &separated);
    module.def("overlaps", (ValidatedKleenean(*)(const Zonotope&,const BasicSetType&)) &overlaps);
    module.def("separated", (ValidatedKleenean(*)(const Zonotope&,const Zonotope&)) &separated);

    module.def("polytope", (Polytope(*)(const Zonotope&)) &polytope);
    module.def("orthogonal_approximation", (Zonotope(*)(const Zonotope&)) &orthogonal_approximation);
    module.def("orthogonal_over_approximation", (Zonotope(*)(const Zonotope&)) &orthogonal_over_approximation);
    module.def("error_free_over_approximation", (Zonotope(*)(const Zonotope&)) &error_free_over_approximation);

//    module.def("image", (Zonotope(*)(const Zonotope&, const ValidatedVectorMultivariateFunction&)) &image);
}

Void export_polytope(pybind11::module& module)
{
    pybind11::class_<Polytope,pybind11::bases<LocatedSetInterface<T>,Drawable2dInterface>> polytope_class(module,"Polytope");
    polytope_class.def(pybind11::init<Polytope>());
    polytope_class.def(pybind11::init<Int>());
    polytope_class.def("new_vertex",&Polytope::new_vertex);
    polytope_class.def("__iter__",boost::python::range(&Polytope::vertices_begin,&Polytope::vertices_end));
    polytope_class.def(self_ns::str(self));
}

*/

Void export_curve(pybind11::module& module)
{
    pybind11::class_<InterpolatedCurve, Drawable2dInterface> interpolated_curve_class(module,"InterpolatedCurve");
    interpolated_curve_class.def(pybind11::init<InterpolatedCurve>());
    interpolated_curve_class.def(pybind11::init<FloatDP,FloatDPPoint>());
    interpolated_curve_class.def("insert", (Void(InterpolatedCurve::*)(const FloatDP&, const Point<FloatDPApproximation>&)) &InterpolatedCurve::insert);
    interpolated_curve_class.def("__iter__", [](InterpolatedCurve const& c){return pybind11::make_iterator(c.begin(),c.end());});
    interpolated_curve_class.def("__str__", &__cstr__<InterpolatedCurve>);


}



Void export_affine_set(pybind11::module& module)
{
    pybind11::class_<ValidatedAffineConstrainedImageSet,pybind11::bases<Drawable2dInterface,ValidatedEuclideanCompactSetInterface>>
        affine_set_class(module,"ValidatedAffineConstrainedImageSet", pybind11::multiple_inheritance());
    affine_set_class.def(pybind11::init<ValidatedAffineConstrainedImageSet>());
    affine_set_class.def(pybind11::init<RealBox>());
    affine_set_class.def(pybind11::init<ExactBoxType>());
    affine_set_class.def(pybind11::init<Vector<ExactIntervalType>, Matrix<FloatDP>, Vector<FloatDP> >());
    affine_set_class.def("new_parameter_constraint", (Void(ValidatedAffineConstrainedImageSet::*)(const Constraint<Affine<FloatDPBounds>,FloatDPBounds>&)) &ValidatedAffineConstrainedImageSet::new_parameter_constraint);
    affine_set_class.def("new_constraint", (Void(ValidatedAffineConstrainedImageSet::*)(const Constraint<AffineModel<ValidatedTag,FloatDP>,FloatDPBounds>&)) &ValidatedAffineConstrainedImageSet::new_constraint);
    affine_set_class.def("dimension", &ValidatedAffineConstrainedImageSet::dimension);
    affine_set_class.def("is_bounded", &ValidatedAffineConstrainedImageSet::is_bounded);
    affine_set_class.def("is_empty", &ValidatedAffineConstrainedImageSet::is_empty);
    affine_set_class.def("bounding_box", &ValidatedAffineConstrainedImageSet::bounding_box);
    affine_set_class.def("separated", &ValidatedAffineConstrainedImageSet::separated);
    affine_set_class.def("adjoin_outer_approximation_to", &ValidatedAffineConstrainedImageSet::adjoin_outer_approximation_to);
    affine_set_class.def("outer_approximation", &ValidatedAffineConstrainedImageSet::outer_approximation);
    affine_set_class.def("boundary", &ValidatedAffineConstrainedImageSet::boundary);
    affine_set_class.def("__str__",&__cstr__<ValidatedAffineConstrainedImageSet>);

    module.def("image", (ValidatedAffineConstrainedImageSet(*)(ValidatedAffineConstrainedImageSet const&,ValidatedVectorMultivariateFunction const&)) &_image_);
}

Void export_constraint_set(pybind11::module& module)
{
//    from_python< List<EffectiveConstraint> >();

    pybind11::class_<ConstraintSet,pybind11::bases<EffectiveEuclideanRegularSetInterface,EffectiveEuclideanOpenSetInterface> >
        constraint_set_class(module,"ConstraintSet", pybind11::multiple_inheritance());
    constraint_set_class.def(pybind11::init<ConstraintSet>());
    constraint_set_class.def(pybind11::init< List<EffectiveConstraint> >());
    constraint_set_class.def("dimension", &ConstraintSet::dimension);
    constraint_set_class.def("__str__", &__cstr__<ConstraintSet>);

//    pybind11::class_<BoundedConstraintSet,pybind11::bases<DrawableWrapper> >
    pybind11::class_<BoundedConstraintSet,pybind11::bases<EffectiveEuclideanRegularSetInterface,EffectiveEuclideanLocatedSetInterface,Drawable2dInterface> >
        bounded_constraint_set_class(module,"BoundedConstraintSet", pybind11::multiple_inheritance());
    bounded_constraint_set_class.def(pybind11::init<BoundedConstraintSet>());
    bounded_constraint_set_class.def(pybind11::init< RealBox, List<EffectiveConstraint> >());
    bounded_constraint_set_class.def("dimension", &BoundedConstraintSet::dimension);
    bounded_constraint_set_class.def("__str__", &__cstr__<BoundedConstraintSet>);

    module.def("intersection", (ConstraintSet(*)(ConstraintSet const&,ConstraintSet const&)) &_intersection_);
    module.def("intersection", (BoundedConstraintSet(*)(ConstraintSet const&, RealBox const&)) &_intersection_);
    module.def("intersection", (BoundedConstraintSet(*)(RealBox const&, ConstraintSet const&)) &_intersection_);

    module.def("intersection", (BoundedConstraintSet(*)(BoundedConstraintSet const&, BoundedConstraintSet const&)) &_intersection_);
    module.def("intersection", (BoundedConstraintSet(*)(BoundedConstraintSet const&, RealBox const&)) &_intersection_);
    module.def("intersection", (BoundedConstraintSet(*)(RealBox const&, BoundedConstraintSet const&)) &_intersection_);
    module.def("intersection", (BoundedConstraintSet(*)(ConstraintSet const&, BoundedConstraintSet const&)) &_intersection_);
    module.def("intersection", (BoundedConstraintSet(*)(BoundedConstraintSet const&, ConstraintSet const&)) &_intersection_);

    module.def("image", (ConstrainedImageSet(*)(BoundedConstraintSet const&, EffectiveVectorMultivariateFunction const&)) &_image_);

}


Void export_constrained_image_set(pybind11::module& module)
{
//    from_python< List<ValidatedConstraint> >();

    pybind11::class_<ConstrainedImageSet,pybind11::bases<EffectiveEuclideanLocatedSetInterface,Drawable2dInterface> >
        constrained_image_set_class(module,"ConstrainedImageSet");
    constrained_image_set_class.def(pybind11::init<ConstrainedImageSet>());
    constrained_image_set_class.def(pybind11::init<BoundedConstraintSet>());
    constrained_image_set_class.def(pybind11::init<RealBox,EffectiveVectorMultivariateFunction>());
    constrained_image_set_class.def(pybind11::init<RealBox,EffectiveVectorMultivariateFunction,List<EffectiveConstraint> >());
    constrained_image_set_class.def("dimension", &ConstrainedImageSet::dimension);
    constrained_image_set_class.def("split", (Pair<ConstrainedImageSet,ConstrainedImageSet>(ConstrainedImageSet::*)(SizeType)const) &ConstrainedImageSet::split);
    constrained_image_set_class.def("split", (Pair<ConstrainedImageSet,ConstrainedImageSet>(ConstrainedImageSet::*)()const) &ConstrainedImageSet::split);
//    	constrained_image_set_class.def("affine_over_approximation", &ValidatedConstrainedImageSet::affine_over_approximation);
    constrained_image_set_class.def("__str__",&__cstr__<ConstrainedImageSet>);

//    pybind11::class_<ValidatedConstrainedImageSet,pybind11::bases<CompactSetInterface<T>,Drawable2dInterface> >
    pybind11::class_<ValidatedConstrainedImageSet,pybind11::bases<ValidatedEuclideanLocatedSetInterface,Drawable2dInterface> >
        validated_constrained_image_set_class(module,"ValidatedConstrainedImageSet", pybind11::multiple_inheritance());
    validated_constrained_image_set_class.def(pybind11::init<ValidatedConstrainedImageSet>());
    validated_constrained_image_set_class.def(pybind11::init<ExactBoxType>());
    validated_constrained_image_set_class.def(pybind11::init<ExactBoxType,EffectiveVectorMultivariateFunction>());
    validated_constrained_image_set_class.def(pybind11::init<ExactBoxType,ValidatedVectorMultivariateFunction>());
    validated_constrained_image_set_class.def(pybind11::init<ExactBoxType,ValidatedVectorMultivariateFunction,List<ValidatedConstraint> >());
    validated_constrained_image_set_class.def(pybind11::init<ExactBoxType,ValidatedVectorMultivariateFunctionModelDP>());
    validated_constrained_image_set_class.def("domain", &ValidatedConstrainedImageSet::domain);
    validated_constrained_image_set_class.def("function", &ValidatedConstrainedImageSet::function);
    validated_constrained_image_set_class.def("constraint", &ValidatedConstrainedImageSet::constraint);
    validated_constrained_image_set_class.def("number_of_parameters", &ValidatedConstrainedImageSet::number_of_parameters);
    validated_constrained_image_set_class.def("number_of_constraints", &ValidatedConstrainedImageSet::number_of_constraints);
    validated_constrained_image_set_class.def("apply", &ValidatedConstrainedImageSet::apply);
    validated_constrained_image_set_class.def("new_space_constraint", (Void(ValidatedConstrainedImageSet::*)(const ValidatedConstraint&))&ValidatedConstrainedImageSet::new_space_constraint);
    validated_constrained_image_set_class.def("new_parameter_constraint", (Void(ValidatedConstrainedImageSet::*)(const ValidatedConstraint&))&ValidatedConstrainedImageSet::new_parameter_constraint);
    validated_constrained_image_set_class.def("outer_approximation", &ValidatedConstrainedImageSet::outer_approximation);
    validated_constrained_image_set_class.def("affine_approximation", &ValidatedConstrainedImageSet::affine_approximation);
    validated_constrained_image_set_class.def("affine_over_approximation", &ValidatedConstrainedImageSet::affine_over_approximation);
    validated_constrained_image_set_class.def("adjoin_outer_approximation_to", &ValidatedConstrainedImageSet::adjoin_outer_approximation_to);
    validated_constrained_image_set_class.def("bounding_box", &ValidatedConstrainedImageSet::bounding_box);
    validated_constrained_image_set_class.def("inside", &ValidatedConstrainedImageSet::inside);
    validated_constrained_image_set_class.def("separated", &ValidatedConstrainedImageSet::separated);
    validated_constrained_image_set_class.def("overlaps", &ValidatedConstrainedImageSet::overlaps);
    validated_constrained_image_set_class.def("split", (Pair<ValidatedConstrainedImageSet,ValidatedConstrainedImageSet>(ValidatedConstrainedImageSet::*)()const) &ValidatedConstrainedImageSet::split);
    validated_constrained_image_set_class.def("split", (Pair<ValidatedConstrainedImageSet,ValidatedConstrainedImageSet>(ValidatedConstrainedImageSet::*)(SizeType)const) &ValidatedConstrainedImageSet::split);
    validated_constrained_image_set_class.def("__str__", &__cstr__<ValidatedConstrainedImageSet>);
    validated_constrained_image_set_class.def("__repr__", &__cstr__<ValidatedConstrainedImageSet>);

    //module.def("product", (ValidatedConstrainedImageSet(*)(const ValidatedConstrainedImageSet&,const BasicSetType&)) &product);
}

Void geometry_submodule(pybind11::module& module) {
    export_drawable_interface(module);
    export_set_interface(module);

    export_points(module);
    export_intervals(module);
    export_boxes(module);
//    export_zonotope(module);
//    export_polytope(module);
    export_curve(module);

    export_affine_set(module);

    export_constraint_set(module);
    export_constrained_image_set(module);

}

