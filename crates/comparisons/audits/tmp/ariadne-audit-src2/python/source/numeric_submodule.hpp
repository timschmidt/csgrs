/***************************************************************************
 *            numeric_submodule.hpp
 *
 *  Copyright  2021  Pieter Collins
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

#ifndef ARIADNE_PYTHON_NUMERIC_SUBMODULE_HPP
#define ARIADNE_PYTHON_NUMERIC_SUBMODULE_HPP

#include "pybind11.hpp"
#include <pybind11/operators.h>

#include "utilities.hpp"

namespace Ariadne {

template<class T> std::string numeric_class_tag();
template<> inline std::string numeric_class_tag<DoublePrecision>() { return "DP"; }
template<> inline std::string numeric_class_tag<MultiplePrecision>() { return "MP"; }


template<> struct PythonTemplateName<Positive> { static std::string get() { return "Positive"; } };

template<> struct PythonTemplateName<Float> { };
template<> struct PythonTemplateName<Approximation> { static std::string get() { return "Approximation"; } };
template<> struct PythonTemplateName<LowerBound> { static std::string get() { return "LowerBound"; } };
template<> struct PythonTemplateName<UpperBound> { static std::string get() { return "UpperBound"; } };
template<> struct PythonTemplateName<Bounds> { static std::string get() { return "Bounds"; } };
template<> struct PythonTemplateName<Ball> { static std::string get() { return "Ball"; } };
template<> struct PythonTemplateName<Error> { static std::string get() { return "Error"; } };
template<> struct PythonTemplateName<Rounded> { static std::string get() { return "Rounded"; } };

template<> struct PythonTemplateName<Number> { static std::string get() { return "Number"; } };

template<class T> struct PythonClassName { static std::string get() { return class_name<T>(); } };

template<> struct PythonClassName<FloatDP> { static std::string get() { return "FloatDP"; } };
template<> struct PythonClassName<FloatMP> { static std::string get() { return "FloatMP"; } };

template<template<class>class T, class X> struct PythonClassName<T<X>> {
    static std::string get() { return python_class_name<X>()+python_template_name<T>(); }
};

template<class F,class FE> struct PythonClassName<Ball<F,FE>> { static std::string get() { typedef typename FE::PrecisionType PRE; return python_class_name<F>()+numeric_class_tag<PRE>()+"Ball"; } };
template<class F> struct PythonClassName<Ball<F>> { static std::string get() { return python_class_name<F>()+"Ball"; } };

OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Dyadic>& repr);
OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Decimal>& repr);
OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Rational>& repr);
OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Real>& repr);
OutputStream& operator<<(OutputStream& os, const PythonRepresentation<FloatDP>& repr);
OutputStream& operator<<(OutputStream& os, const PythonRepresentation<FloatMP>& repr);

template<class P> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Number<P>>& repr);

template<class F> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Approximation<F>>& repr);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<LowerBound<F>>& repr);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<UpperBound<F>>& repr);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Bounds<F>>& repr);
template<class F, class FE> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Ball<F,FE>>& repr);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<F>& repr);
template<class FE> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Error<FE>>& repr);

OutputStream& operator<<(OutputStream& os, const PythonLiteral<Dyadic>& lit);
OutputStream& operator<<(OutputStream& os, const PythonLiteral<Decimal>& lit);
OutputStream& operator<<(OutputStream& os, const PythonLiteral<Rational>& lit);
OutputStream& operator<<(OutputStream& os, const PythonLiteral<Real>& lit);

OutputStream& operator<<(OutputStream& os, const PythonLiteral<FloatDP>& lit);
OutputStream& operator<<(OutputStream& os, const PythonLiteral<FloatMP>& lit);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonLiteral<Approximation<F>>& lit);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonLiteral<LowerBound<F>>& lit);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonLiteral<UpperBound<F>>& lit);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonLiteral<Bounds<F>>& lit);
template<class F, class FE> OutputStream& operator<<(OutputStream& os, const PythonLiteral<Ball<F,FE>>& lit);
template<class F> OutputStream& operator<<(OutputStream& os, const PythonLiteral<F>& lit);
template<class FE> OutputStream& operator<<(OutputStream& os, const PythonLiteral<Error<FE>>& lit);


template<class X> X from_python_object_or_literal(pybind11::handle h) {
    if constexpr (Constructible<X,std::string>) {
        try { return X(pybind11::cast<std::string>(h)); }
        catch(...) { }
    }
    return pybind11::cast<X>(h);
}
template<> ValidatedNumber from_python_object_or_literal<ValidatedNumber>(pybind11::handle h);
template<> ValidatedUpperNumber from_python_object_or_literal<ValidatedUpperNumber>(pybind11::handle h);
template<> ValidatedLowerNumber from_python_object_or_literal<ValidatedLowerNumber>(pybind11::handle h);
template<> ApproximateNumber from_python_object_or_literal<ApproximateNumber>(pybind11::handle h);

} // namespace Ariadne

#endif /* ARIADNE_PYTHON_NUMERIC_SUBMODULE_HPP */
