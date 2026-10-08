/***************************************************************************
 *            differentiation_submodule.cpp
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
#include <pybind11/stl.h>

#include "utilities.hpp"
#include "numeric_submodule.hpp"

#include "utility/typedefs.hpp"
#include "utility/array.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/covector.hpp"
#include "algebra/differential.hpp"
#include "algebra/univariate_differential.hpp"
#include "algebra/fixed_differential.hpp"
#include "algebra/fixed_univariate_differential.hpp"
#include "algebra/expansion.inl.hpp"

using namespace Ariadne;

namespace Ariadne {

template<class X> struct PythonClassName<Vector<X>> { static std::string get() { return python_template_class_name<X>("Vector"); } };
template<class X> struct PythonClassName<Differential<X>> { static std::string get() { return python_template_class_name<X>("Differential"); } };
template<class X> struct PythonClassName<UnivariateDifferential<X>> { static std::string get() { return python_template_class_name<X>("UnivariateDifferential"); } };

template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Expansion<MultiIndex,X>>& repr);

template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation< Differential<X> >& repr) {
    const Differential<X>& diff=repr.reference();
    os << python_class_name<Differential<X>>().c_str() << "(" << python_representation(diff.expansion()) << "," << diff.degree() << ")";
    return os;
}

template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation< UnivariateDifferential<X> >& repr) {
    const UnivariateDifferential<X>& diff=repr.reference();
    os << python_class_name<UnivariateDifferential<X>>().c_str() << "(" << diff.array() << "," << diff.degree() << ")";
    return os;
}

} // namespace Ariadne


template<class D> using ValueType = decltype(declval<D>().value());
template<class D> using GradientType = decltype(declval<D>().gradient());
template<class D> using HessianType = decltype(declval<D>().hessian());

template<class DIFF>
pybind11::class_<DIFF> export_differential(pybind11::module& module, const String& name=python_class_name<DIFF>())
{
    typedef typename DIFF::ValueType X;
    typedef DIFF D;

    pybind11::class_<D> differential_class(module,name.c_str());
    differential_class.def(pybind11::init<D>());

    if constexpr (HasGenericType<X>) {
        typedef typename X::PrecisionType PR;
        differential_class.def( pybind11::init<SizeType,DegreeType,PR>());
    } else {
        differential_class.def( pybind11::init<SizeType,DegreeType>());
    }

    differential_class.def_static("constant",(D(*)(SizeType as, DegreeType deg, const X& c)) &D::constant);
    differential_class.def_static("variable",(D(*)(SizeType as, DegreeType deg, const X& v, SizeType j)) &D::variable);
    differential_class.def_static("constants",(Vector<D>(*)(SizeType as, DegreeType deg, const Vector<X>& c)) &D::constants);
    differential_class.def_static("variables",(Vector<D>(*)(DegreeType deg, const Vector<X>& v)) &D::variables);
    if constexpr (HasGenericType<X>) {
        typedef typename X::GenericType Y; typedef typename X::PrecisionType PR;
        differential_class.def_static("constant",(D(*)(SizeType, DegreeType, const Y&, const PR&)) &D::constant);
        differential_class.def_static("variable",(D(*)(SizeType as, DegreeType deg, const Y& y, SizeType j, const PR& pr)) &D::variable);
        differential_class.def_static("constants",(Vector<D>(*)(SizeType, DegreeType, const Vector<Y>&, const PR&)) &D::constants);
        differential_class.def_static("variables",(Vector<D>(*)(DegreeType, const Vector<Y>&, const PR&)) &D::variables);
    }

    differential_class.def( pybind11::init<Expansion<MultiIndex,X>,DegreeType>());
    differential_class.def("__getitem__", &__getitem__<D,MultiIndex,X>);
    differential_class.def("__setitem__",&__setitem__<D,MultiIndex,X>);

    define_elementary_algebra<D,X>(module, differential_class);
    define_inplace_algebra<D,X>(module,differential_class);
    differential_class.def("__str__", &__cstr__<D>);
    differential_class.def("__repr__", &__repr__<D>);

    differential_class.def("value", (ValueType<D>(D::*)()const)&D::value);
    differential_class.def("gradient", (GradientType<D>(D::*)()const)&D::gradient);
    differential_class.def("hessian", (HessianType<D>(D::*)()const)&D::hessian);
    differential_class.def("expansion", (Expansion<MultiIndex,X>const&(D::*)()const)&D::expansion);

    if constexpr (HasGenericType<X>) {
        typedef typename X::GenericType Y;
        define_mixed_arithmetic<D,Y>(module, differential_class);
    }

    module.def("derivative", (D(*)(const D&, SizeType))&D::_derivative);
    module.def("antiderivative", (D(*)(const D&, SizeType))&D::_antiderivative);

    return differential_class;
}

template<class DIFF>
pybind11::class_<Vector<DIFF>>
export_differential_vector(pybind11::module& module, const String& name=python_class_name<Vector<DIFF>>())
{
    typedef typename DIFF::ValueType X;
    typedef Vector<X> V;
    typedef DIFF D;
    typedef Vector<D> DV;

    pybind11::class_<DV> differential_vector_class(module,name.c_str());
    differential_vector_class.def(pybind11::init<DV>());
    differential_vector_class.def(pybind11::init<Vector<D>>());
    if constexpr (HasPrecisionType<X>) {
        typedef typename X::PrecisionType PR;
        differential_vector_class.def(pybind11::init<Nat,Nat,Nat,PR>());
    } else {
        differential_vector_class.def(pybind11::init<Nat,Nat,Nat>());
    }
    differential_vector_class.def("__getitem__", &__getitem2__<DV,Nat,MultiIndex,X>);
    differential_vector_class.def("__getitem__", &__getitem__<DV,Nat,D>);
    differential_vector_class.def("__setitem__",&__setitem__<DV,Nat,X>);
    differential_vector_class.def("__setitem__",&__setitem__<DV,Nat,D>);

    define_vector_algebra_arithmetic(module, differential_vector_class);

    differential_vector_class.def("size", &DV::size);
    differential_vector_class.def("argument_size", &DV::argument_size);
    differential_vector_class.def("degree", &DV::degree);
    differential_vector_class.def("value", &DV::value);
    differential_vector_class.def("jacobian", &DV::jacobian);
    differential_vector_class.def("__str__",&__cstr__<DV>);
    //differential_vector_class.def("__repr__",&__repr__<DV>);

    module.def("compose",(D(*)(const D&,const DV&))&D::_compose);
    module.def("compose",(DV(*)(const DV&,const DV&))&DV::_compose);

    module.def("solve",(DV(*)(const DV&,const V&))&DV::_solve);
    module.def("flow",(DV(*)(const DV&,const V&))&DV::_flow);

    //NOTE: Not in C++ interface
    //module.def("lie_derivative", (DV(*)(const DV&,const DV&))&lie_derivative);

    return differential_vector_class;
}

template<class DIFF>
pybind11::class_<DIFF> export_univariate_differential(pybind11::module& module, const String& name=python_class_name<DIFF>())
{
    typedef typename DIFF::ValueType X;
    typedef DIFF D;

    pybind11::class_<D> univariate_differential_class(module,name.c_str());
    univariate_differential_class.def(pybind11::init<D>());

    if constexpr (HasGenericType<X>) {
        typedef typename X::PrecisionType PR;
        univariate_differential_class.def( pybind11::init<DegreeType,PR>());
    } else {
        univariate_differential_class.def( pybind11::init<DegreeType>());
    }

    univariate_differential_class.def_static("constant",(D(*)(DegreeType, const X&)) &D::constant);
    univariate_differential_class.def_static("variable",(D(*)(DegreeType, const X&)) &D::variable);
    if constexpr (HasGenericType<X>) {
        typedef typename X::GenericType Y; typedef typename X::PrecisionType PR;
        univariate_differential_class.def_static("constant",(D(*)(DegreeType, const Y&, const PR&)) &D::constant);
        univariate_differential_class.def_static("variable",(D(*)(DegreeType, const Y&, const PR&)) &D::variable);
    }

    univariate_differential_class.def( pybind11::init<Array<X>>());
    univariate_differential_class.def("__getitem__", &__getitem__<D,SizeType,X>);
    univariate_differential_class.def("__setitem__",&__setitem__<D,SizeType,X>);

    define_elementary_algebra<D,X>(module, univariate_differential_class);
//    define_inplace_algebra<D,X>(module,univariate_differential_class);
    univariate_differential_class.def("__str__", &__cstr__<D>);
    univariate_differential_class.def("__repr__", &__repr__<D>);

    univariate_differential_class.def("value", (ValueType<D>(D::*)()const)&D::value);
    univariate_differential_class.def("gradient", (GradientType<D>(D::*)()const)&D::gradient);
    univariate_differential_class.def("hessian", (HessianType<D>(D::*)()const)&D::hessian);

    univariate_differential_class.def("array", (Array<X>const&(D::*)()const)&D::array);

    if constexpr (HasGenericType<X>) {
        typedef typename X::GenericType Y;
        define_mixed_arithmetic<D,Y>(module, univariate_differential_class);
    }

    module.def("derivative", &_derivative_<D>);
    module.def("antiderivative", &_antiderivative_<D>);

    return univariate_differential_class;
}


Void differentiation_submodule(pybind11::module& module)
{
    export_differential< Differential<FloatDPApproximation> >(module);
    export_differential< Differential<FloatDPBounds> >(module);
    export_differential_vector< Differential<FloatDPApproximation> >(module);
    export_differential_vector< Differential<FloatDPBounds> >(module);

    export_differential< Differential<FloatMPApproximation> >(module);
    export_differential< Differential<FloatMPBounds> >(module);
    export_differential_vector< Differential<FloatMPApproximation> >(module);
    export_differential_vector< Differential<FloatMPBounds> >(module);

    export_univariate_differential< UnivariateDifferential<FloatDPApproximation> >(module);
    export_univariate_differential< UnivariateDifferential<FloatDPBounds> >(module);
    export_univariate_differential< UnivariateDifferential<FloatMPApproximation> >(module);
    export_univariate_differential< UnivariateDifferential<FloatMPBounds> >(module);

    template_<Differential> differential_template(module,"Differential");
    differential_template.instantiate<FloatDPApproximation>();
    differential_template.instantiate<FloatMPApproximation>();
    differential_template.instantiate<FloatDPBounds>();
    differential_template.instantiate<FloatMPBounds>();

    template_<Vector> vector_template(module,"Vector");
    vector_template.instantiate<FloatDPApproximationDifferential>();
    vector_template.instantiate<FloatMPApproximationDifferential>();
    vector_template.instantiate<FloatDPBoundsDifferential>();
    vector_template.instantiate<FloatMPBoundsDifferential>();
}

