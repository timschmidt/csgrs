/***************************************************************************
 *            function_submodule.cpp
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

#include "utilities.hpp"
#include "numeric_submodule.hpp"

#include <iostream>
#include <iomanip>

#include "utility/array.hpp"
#include "utility/container.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/expansion.hpp"
#include "algebra/expansion.inl.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/differential.hpp"
#include "function/formula.hpp"
#include "function/polynomial.hpp"
#include "function/affine.hpp"
#include "function/taylor_model.hpp"
#include "function/taylor_function.hpp"
#include "function/constraint.hpp"
#include "function/procedure.hpp"
#include "function/function.hpp"
#include "function/function_mixin.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/space.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/function_expression.hpp"
#include "solvers/constraint_solver.hpp"

#include "function/function_mixin.tpl.hpp"

namespace Ariadne {

template<> struct PythonTemplateName<MultivariatePolynomial> { static std::string get() { return "MultivariatePolynomial"; } };
template<> struct PythonTemplateName<Procedure> { static std::string get() { return "Procedure"; } };
template<> struct PythonTemplateName<Function> { static std::string get() { return "Function"; } };

template<class X> struct PythonClassName<MultivariatePolynomial<X>> { static std::string get() { return python_template_class_name<X>("MultivariatePolynomial"); } };

template struct PythonClassName<MultivariatePolynomial<FloatDPApproximation>>;
template struct PythonClassName<MultivariatePolynomial<FloatMPApproximation>>;
template struct PythonClassName<MultivariatePolynomial<FloatDPBounds>>;
template struct PythonClassName<MultivariatePolynomial<FloatMPBounds>>;

MultiIndex multi_index_from_python(pybind11::tuple pytup) {
    MultiIndex res(static_cast<SizeType>(len(pytup)));
    for(SizeType i=0; i!=res.size(); ++i) { res.set(i,pybind11::cast<Nat>(pytup[i])); }
    return res;
}

template<class P, class ARG> pybind11::object function_from_python(typename DomainTraits<ARG>::EntireDomainType dom, pybind11::object pyf) {
    Function<P,ARG(ARG)> id = Function<P,ARG(ARG)>::identity(dom);
    pybind11::object pyobjf = pyf(id);
    try {
        VectorFunction<P,ARG> vf=pybind11::cast<VectorFunction<P,ARG>>(pyobjf);
        return pybind11::cast(vf);
    } catch (...) { }
    try {
        pybind11::list pylstf = pybind11::cast<pybind11::list>(pyobjf);
        SizeType rs = len(pylstf);
        //VectorFunction<P,ARG>(Vector<ScalarFunction<P,ARG>>(rs, [&id,&lstf](SizeType i){return pybind11::cast<ScalarFunction<P,ARG>>(lstf[i](id));}));
        List<ScalarFunction<P,ARG>> lsf;
        for (SizeType i=0; i!=rs; ++i) {
            lsf.append(pybind11::cast<ScalarFunction<P,ARG>>(pylstf[i]));
        }
        return pybind11::cast(VectorFunction<P,ARG>(lsf));
    } catch (...) { }
    return pybind11::cast(pybind11::cast<ScalarFunction<P,ARG>>(pyobjf));
}

pybind11::object univariate_function_from_python(pybind11::object pyf) {
    return function_from_python<EffectiveTag,RealScalar>(RealDomain(),pyf);
}
pybind11::object multivariate_function_from_python(SizeType as, pybind11::object pyf) {
    return function_from_python<EffectiveTag,RealVector>(EuclideanDomain(as),pyf);
}



template<class X, class D>
inline Matrix<X> get_jacobian(const Vector<D>& d) {
    const Nat rs=d.size(); const Nat as=d[0].argument_size();
    Matrix<X> J(rs,as);
    for(Nat i=0; i!=rs; ++i) {
        for(Nat j=0; j!=as; ++j) {
            J[i][j]=d[i][j];
        }
    }
    return J;
}

OutputStream& operator<<(OutputStream& os, const PythonRepresentation<MultiIndex>& arepr) {
    MultiIndex const& a=arepr.reference(); os << "("; for(SizeType i=0; i!=a.size(); ++i) { if(i!=0) { os << ','; } os << a[i]; } os << ")"; return os;
}

template<class P, class SIG> OutputStream& operator<<(OutputStream& os, const PythonRepresentation< Function<P,SIG> >& frepr) {
    Function<P,SIG> const& f = frepr.reference();
    os << "Function(";
    if constexpr (Same<decltype(f.argument_size()),SizeOne>) { }
    else { os << frepr.reference().argument_size() << ","; }
    return os << " lambda x: " << frepr.reference() << " )";
}

}// namespace Ariadne

using namespace Ariadne;

typedef ExactIntervalType I;
typedef Vector<FloatDPApproximation> FV;
typedef Vector<ExactIntervalType> IV;
typedef Matrix<FloatDPApproximation> FMx;
typedef Matrix<ExactIntervalType> IMx;
typedef Vector< Differential<FloatDPApproximation> > FSDV;
typedef Vector< Differential<ExactIntervalType> > ISDV;
typedef Vector<ValidatedTaylorModelDP> TMV;
typedef ValidatedVectorMultivariateTaylorFunctionModelDP TFM;
typedef ValidatedTaylorModelDP TM;


template<class AT, class F> void define_call(pybind11::class_<F>& f) {
    typedef typename F::template Argument<AT> A; typedef ResultOf<F(A)> R; f.def("__call__", (R(F::*)(A const&)const) &F::operator()); }

template<class AT, class P> void define_named_derivatives(pybind11::module& module, pybind11::class_<ScalarUnivariateFunction<P>>& function_class) {
    typedef ScalarUnivariateFunction<P> F; typedef typename F::template Argument<AT> A; typedef decltype(slope(declval<F>(),declval<A>())) R;
    module.def("slope", (R(*)(F const&,A const&)) &slope);
    function_class.def("slope", (R(F::*)(A const&)const) &F::slope);
}
template<class AT, class P> void define_named_derivatives(pybind11::module& module, pybind11::class_<ScalarMultivariateFunction<P>>& function_class) {
    typedef ScalarMultivariateFunction<P> F; typedef typename F::template Argument<AT> A;
    typedef decltype(gradient(declval<F>(),declval<A>())) R;
    module.def("gradient", (R(*)(F const&,A const&)) &gradient);
    function_class.def("gradient", (R(F::*)(A const&)const) &F::gradient);
    typedef decltype(hessian(declval<F>(),declval<A>())) H;
    module.def("hessian", (H(*)(F const&,A const&)) &hessian);
    //function_class.def("hessian", (H(F::*)(A const&)const) &F::hessian);
}
template<class AT, class P> void define_named_derivatives(pybind11::module& module, pybind11::class_<VectorUnivariateFunction<P>>& function_class) {
    typedef VectorUnivariateFunction<P> F; typedef typename F::template Argument<AT> A; typedef decltype(tangent(declval<F>(),declval<A>())) R;
    module.def("tangent", (R(*)(F const&,A const&)) &tangent);
    function_class.def("tangent", (R(F::*)(A const&)const) &F::tangent);
}
template<class AT, class P> void define_named_derivatives(pybind11::module& module, pybind11::class_<VectorMultivariateFunction<P>>& function_class) {
    typedef VectorMultivariateFunction<P> F; typedef typename F::template Argument<AT> A; typedef decltype(jacobian(declval<F>(),declval<A>())) R;
    module.def("jacobian", (R(*)(F const&,A const&)) &jacobian);
    function_class.def("jacobian", (R(F::*)(A const&)const) &F::jacobian);
}


template<class F, class T> using ArgumentType = typename F::template Argument<T>;

template<class F> Void export_function_evaluation(pybind11::module module, pybind11::class_<F>& function_class, ApproximateTag) {
    define_call<FloatDPApproximation>(function_class);
    define_call<FloatMPApproximation>(function_class);
    define_call<Differential<FloatDPApproximation>>(function_class);
    define_call<Differential<FloatMPApproximation>>(function_class);

    module.def("evaluate", _evaluate_<F,ArgumentType<F,FloatDPApproximation>>);
    module.def("evaluate", _evaluate_<F,ArgumentType<F,FloatMPApproximation>>);
}

template<class F> Void export_function_evaluation(pybind11::module module, pybind11::class_<F>& function_class, ValidatedTag) {
    define_call<FloatDPBounds>(function_class);
    define_call<FloatMPBounds>(function_class);
    define_call<Differential<FloatDPBounds>>(function_class);
    define_call<Differential<FloatMPBounds>>(function_class);

    module.def("evaluate", _evaluate_<F,ArgumentType<F,FloatDPBounds>>);
    module.def("evaluate", _evaluate_<F,ArgumentType<F,FloatMPBounds>>);

    export_function_evaluation(module,function_class,ApproximateTag());
}

template<class F> Void export_function_evaluation(pybind11::module module, pybind11::class_<F>& function_class, EffectiveTag) {
    export_function_evaluation(module,function_class,ValidatedTag());
}

template<class F> Void export_function_evaluation(pybind11::module module, pybind11::class_<F>& function_class)
{
    export_function_evaluation(module, function_class, Paradigm<F>());
}


template<class F> Void export_function_derivatives(pybind11::module module, pybind11::class_<F>& function_class, ApproximateTag) {
    function_class.def("differential", _differential_<F,ArgumentType<F,FloatDPApproximation>,DegreeType>);
    function_class.def("differential", _differential_<F,ArgumentType<F,FloatMPApproximation>,DegreeType>);

    module.def("differential", _differential_<F,ArgumentType<F,FloatDPApproximation>,DegreeType>);
    module.def("differential", _differential_<F,ArgumentType<F,FloatMPApproximation>,DegreeType>);

    define_named_derivatives<FloatDPApproximation>(module,function_class);
    define_named_derivatives<FloatMPApproximation>(module,function_class);
}

template<class F> Void export_function_derivatives(pybind11::module module, pybind11::class_<F>& function_class, ValidatedTag) {
    function_class.def("differential", _differential_<F,ArgumentType<F,FloatDPBounds>,DegreeType>);
    function_class.def("differential", _differential_<F,ArgumentType<F,FloatMPBounds>,DegreeType>);

    module.def("differential", _differential_<F,ArgumentType<F,FloatDPBounds>,DegreeType>);
    module.def("differential", _differential_<F,ArgumentType<F,FloatMPBounds>,DegreeType>);

    define_named_derivatives<FloatDPBounds>(module,function_class);
    define_named_derivatives<FloatMPBounds>(module,function_class);

    export_function_derivatives(module,function_class,ApproximateTag());
}

template<class F> Void export_function_derivatives(pybind11::module module, pybind11::class_<F>& function_class, EffectiveTag) {
    export_function_derivatives(module,function_class,ValidatedTag());
}

template<class F> Void export_function_derivatives(pybind11::module module, pybind11::class_<F>& function_class)
{
    export_function_derivatives(module, function_class, Paradigm<F>());
}


Void export_multi_index(pybind11::module& module)
{
    pybind11::class_< MultiIndex > multi_index_class(module,"MultiIndex");
    multi_index_class.def(pybind11::init<Nat>());
    multi_index_class.def(pybind11::init(&multi_index_from_python));
    multi_index_class.def(pybind11::init<MultiIndex>());
    multi_index_class.def("__getitem__",&MultiIndex::get);
    multi_index_class.def("__setitem__",&MultiIndex::set);
    multi_index_class.def("degree",&MultiIndex::degree);
    multi_index_class.def("__str__", &__cstr__<MultiIndex>);
    multi_index_class.def("__repr__", &__repr__<MultiIndex>);

    pybind11::implicitly_convertible<pybind11::tuple,MultiIndex>();
}


template<class X>
pybind11::class_< MultivariatePolynomial<X> > export_polynomial(pybind11::module& module)
{
    pybind11::class_< MultivariateMonomial<X> > monomial_class(module,python_template_class_name<X>("MultivariateMonomial").c_str());
    monomial_class.def(pybind11::init<MultiIndex,X>());
    monomial_class.def("index", (MultiIndex const&(MultivariateMonomial<X>::*)()const) &MultivariateMonomial<X>::index);
    monomial_class.def("coefficient", (X const&(MultivariateMonomial<X>::*)()const) &MultivariateMonomial<X>::coefficient);
    monomial_class.def("__str__", &__cstr__<MultivariateMonomial<X>>);


    pybind11::class_< MultivariatePolynomial<X> > polynomial_class(module,python_template_class_name<X>("MultivariatePolynomial").c_str());
    polynomial_class.def(pybind11::init< MultivariatePolynomial<X> >());
    polynomial_class.def_static("constant", (MultivariatePolynomial<X>(*)(SizeType,X const&)) &MultivariatePolynomial<X>::constant);

    if constexpr (HasPrecisionType<X>) {
        typedef typename X::PrecisionType PR;
        polynomial_class.def(pybind11::init<Nat,PR>());
        polynomial_class.def_static("variable", (MultivariatePolynomial<X>(*)(SizeType,SizeType,PR)) &MultivariatePolynomial<X>::variable);
        polynomial_class.def_static("coordinate", (MultivariatePolynomial<X>(*)(SizeType,SizeType,PR)) &MultivariatePolynomial<X>::variable);
        polynomial_class.def_static("variables", [](Nat as,PR pr){return MultivariatePolynomial<X>::variables(as,pr).array();});
    } else {
        polynomial_class.def(pybind11::init<Nat>());
        polynomial_class.def_static("variable", (MultivariatePolynomial<X>(*)(SizeType,SizeType)) &MultivariatePolynomial<X>::variable);
        polynomial_class.def_static("coordinate", (MultivariatePolynomial<X>(*)(SizeType,SizeType)) &MultivariatePolynomial<X>::variable);
        polynomial_class.def_static("variables", [](Nat as){return MultivariatePolynomial<X>::variables(as).array();});
    }

    polynomial_class.def("argument_size", &MultivariatePolynomial<X>::argument_size);
    polynomial_class.def("insert", &MultivariatePolynomial<X>::insert);

    define_algebra(module,polynomial_class);
    polynomial_class.def("__str__",&__cstr__<MultivariatePolynomial<X>>);

    export_vector<MultivariatePolynomial<X>>(module, (python_template_class_name<X>("MultivariatePolynomialVector")).c_str());

    return polynomial_class;
}

Void export_polynomials(pybind11::module& module)
{
    export_polynomial<FloatDPBounds>(module);
    export_polynomial<FloatDPApproximation>(module);
    export_polynomial<FloatMPBounds>(module);
    export_polynomial<FloatMPApproximation>(module);

    template_<MultivariatePolynomial> multivariate_polynomial_template(module);
    multivariate_polynomial_template.instantiate<FloatDPBounds>();
    multivariate_polynomial_template.instantiate<FloatDPApproximation>();
    multivariate_polynomial_template.instantiate<FloatMPBounds>();
    multivariate_polynomial_template.instantiate<FloatMPApproximation>();
}

Void export_domains(pybind11::module& module)
{
    pybind11::class_<RealDomain> real_domain_class(module,"RealDomain");
    real_domain_class.def(pybind11::init<>());
    //    real_domain_class.def(pybind11::init<SizeOne>());
    real_domain_class.def("__repr__", &__cstr__<RealDomain>);

    pybind11::class_<EuclideanDomain> euclidean_domain_class(module,"EuclideanDomain");
    euclidean_domain_class.def(pybind11::init<SizeType>());
    euclidean_domain_class.def("__repr__", &__cstr__<EuclideanDomain>);
}

template<class SIG> String signature_name() {
    using RES = typename SignatureTraits<SIG>::ResultKind;
    using ARG = typename SignatureTraits<SIG>::ArgumentKind;
    return String(Same<RES,RealScalar> ? "Scalar" : "Vector") + (Same<ARG,RealScalar> ? "Univariate" : "Multivariate");
}

template<class P, class SIG> Void export_function(pybind11::module& module) {
    using F=Function<P,SIG>;
    using RES = typename SignatureTraits<SIG>::ResultKind;
    using ARG = typename SignatureTraits<SIG>::ArgumentKind;
    using DomainType = typename SignatureTraits<SIG>::DomainType;
    using CodomainType = typename SignatureTraits<SIG>::CodomainType;
    using ResultSizeType = ElementSizeType<CodomainType>;
    using ArgumentSizeType = ElementSizeType<DomainType>;

    pybind11::class_<F> function_class(module,(class_name<P>()+signature_name<SIG>()+"Function").c_str());

    function_class.def(pybind11::init<F>());
    if constexpr (Same<P,ValidatedTag>) {
        function_class.def(pybind11::init<Function<EffectiveTag,SIG>>());
        pybind11::implicitly_convertible<Function<EffectiveTag,SIG>,Function<ValidatedTag,SIG>>();
    }

    function_class.def(pybind11::init<ResultSizeType,ArgumentSizeType>());
    function_class.def(pybind11::init<ResultSizeType,DomainType>());
    if constexpr (Same<RES,RealScalar>) {
        function_class.def(pybind11::init<ArgumentSizeType>());
        function_class.def(pybind11::init<DomainType>());
    }

    function_class.def(pybind11::init<typename SignatureTraits<SIG>::ArgumentSpaceType, typename SignatureTraits<SIG>::template Result<RealExpression>>());

    if constexpr (Same<RES,RealVector>) {
        // NOTE: This must go *after* the conversion constructor
        function_class.def(pybind11::init([](std::vector<Function<P,RealScalar(ARG)>> const& lst){return F(lst);}));
    }

    function_class.def("result_size", &F::result_size);
    function_class.def("argument_size", &F::argument_size);

    if constexpr (Same<RES,RealVector>) {
        function_class.def("__getitem__", &F::get);
        function_class.def("__setitem__", &F::set);
    }

    if constexpr (Same<ARG,RealScalar>) {
        function_class.def("derivative", (F(F::*)(IndexZero)const) &F::derivative);
        function_class.def("derivative", (F(*)(const F&)) &derivative);
        module.def("derivative", (F(F::*)(IndexZero)const) &F::derivative);
        module.def("derivative", (F(*)(const F&)) &derivative);
    } else if constexpr (Same<ARG,RealVector>) {
        function_class.def("derivative", (F(F::*)(SizeType)const) &F::derivative);
        module.def("derivative", (F(F::*)(SizeType)const) &F::derivative);
    }


    if constexpr (Same<RES,RealScalar>) { define_elementary_algebra<F,Number<P>>(module,function_class); }

    // TODO: Put these in C++ API
    // define_vector_algebra_arithmetic<VectorMultivariateFunction<P>,ScalarMultivariateFunction<P>,Number<P>>(module,vector_function_class);
    // FIXME: Define vector function operations for Validated and Approximate
    if constexpr (Same<RES,RealVector>) {
        if constexpr (Same<ARG,RealVector> and Same<P,EffectiveTag>) {
            define_vector_algebra_arithmetic<VectorFunction<P,ARG>,ScalarFunction<P,ARG>>(module,function_class);
        }
    }

    export_function_evaluation(module,function_class);
    export_function_derivatives(module,function_class);

    if constexpr (Same<SIG,RealScalar(RealScalar)>) {
        if constexpr (not Same<P,ApproximateTag>) {
            module.def("derivative", [](const F& f, const FloatDPBounds& x){return static_cast<FloatDPBounds>(differential(f,x,1u).gradient()[0]);} );
            module.def("derivative", [](const F& f, const FloatMPBounds& x){return static_cast<FloatMPBounds>(differential(f,x,1u).gradient()[0]);} );
            module.def("second_derivative", [](const F& f, const FloatDPBounds& x){return static_cast<FloatDPBounds>(differential(f,x,2u).hessian()[0][0]);} );
            module.def("second_derivative", [](const F& f, const FloatMPBounds& x){return static_cast<FloatMPBounds>(differential(f,x,2u).hessian()[0][0]);} );
        }
        module.def("derivative", [](const F& f, const FloatDPApproximation& x){return static_cast<FloatDPApproximation>(differential(f,x,1u).gradient()[0]);} );
        module.def("derivative", [](const F& f, const FloatMPApproximation& x){return static_cast<FloatMPApproximation>(differential(f,x,1u).gradient()[0]);} );
        module.def("second_derivative", [](const F& f, const FloatDPApproximation& x){return static_cast<FloatDPApproximation>(differential(f,x,2u).hessian()[0][0]);} );
        module.def("second_derivative", [](const F& f, const FloatMPApproximation& x){return static_cast<FloatMPApproximation>(differential(f,x,2u).hessian()[0][0]);} );
    }

    if constexpr (Same<SIG,RealScalar(RealScalar)>) {
        function_class.def_static("constant", (F(*)(Number<P>)) &F::constant);
        function_class.def_static("constant", (F(*)(RealDomain,Number<P>)) &F::constant);
        function_class.def_static("coordinate", (F(*)()) &F::coordinate);
        function_class.def_static("coordinate", (F(*)(RealDomain)) &F::coordinate);
        function_class.def_static("identity", (F(*)()) &F::identity);
        function_class.def_static("identity", (F(*)(RealDomain)) &F::identity);
    }
    if constexpr (Same<SIG,RealVector(RealScalar)>) {
        function_class.def_static("zeros", (F(*)(SizeType)) &F::zeros);
        function_class.def_static("zeros", (F(*)(SizeType,RealDomain)) &F::zeros);
    }
    if constexpr (Same<SIG,RealScalar(RealVector)>) {
        function_class.def_static("constant", (F(*)(SizeType,Number<P>)) &F::constant);
        function_class.def_static("constant", (F(*)(EuclideanDomain,Number<P>)) &F::constant);
        function_class.def_static("coordinate", (F(*)(SizeType,SizeType)) &F::coordinate);
        function_class.def_static("coordinate", (F(*)(EuclideanDomain,SizeType)) &F::coordinate);
    }
    if constexpr (Same<SIG,RealVector(RealVector)>) {
        function_class.def_static("zeros", (F(*)(SizeType,SizeType)) &F::zeros);
        function_class.def_static("zeros", (F(*)(SizeType,EuclideanDomain)) &F::zeros);
        function_class.def_static("identity", (F(*)(SizeType)) &F::identity);
        function_class.def_static("identity", (F(*)(EuclideanDomain)) &F::identity);
    }

    function_class.def("__str__", &__cstr__<F>);
    function_class.def("__repr__", &__repr__<F>);
    function_class.def("__crepr__", &__crepr__<F>);

    if constexpr (Same<RES,RealVector>) if constexpr (Same<ARG,RealVector>) {
        module.def("join", (VectorFunction<P,ARG>(*)(const ScalarFunction<P,ARG>&, const ScalarFunction<P,ARG>&)) &join);
        module.def("join", (VectorFunction<P,ARG>(*)(const VectorFunction<P,ARG>&, const ScalarFunction<P,ARG>&)) &join);
        module.def("join", (VectorFunction<P,ARG>(*)(const ScalarFunction<P,ARG>&, const VectorFunction<P,ARG>&)) &join);
        module.def("join", (VectorFunction<P,ARG>(*)(const VectorFunction<P,ARG>&, const VectorFunction<P,ARG>&)) &join);
    }

    module.def("compose", (Function<P,RES(ARG)>(*)(const Function<P,RES(RealScalar)>&,const Function<P,RealScalar(ARG)>&)) &compose);
    module.def("compose", (Function<P,RES(ARG)>(*)(const Function<P,RES(RealVector)>&,const Function<P,RealVector(ARG)>&)) &compose);

    if constexpr (Same<P,EffectiveTag> and Same<SIG,RealScalar(RealVector)>) {
        function_class.def("__eq__", &__eq__<EffectiveScalarMultivariateFunction,EffectiveNumber,Return<EffectiveConstraint>>);
        function_class.def("__le__", &__le__<EffectiveScalarMultivariateFunction,EffectiveNumber,Return<EffectiveConstraint>>);
        function_class.def("__ge__", &__ge__<EffectiveScalarMultivariateFunction,EffectiveNumber,Return<EffectiveConstraint>>);
    }

}


template<class SIG> Void export_functions(pybind11::module& module)
{
    export_function<EffectiveTag,SIG>(module);
    export_function<ValidatedTag,SIG>(module);
    export_function<ApproximateTag,SIG>(module);

    pybind11::implicitly_convertible<Function<EffectiveTag,SIG>,Function<ValidatedTag,SIG>>();
    pybind11::implicitly_convertible<Function<EffectiveTag,SIG>,Function<ApproximateTag,SIG>>();
    pybind11::implicitly_convertible<Function<ValidatedTag,SIG>,Function<ApproximateTag,SIG>>();

    if constexpr (Same<SIG,RealScalar(RealVector)>) {
        module.def("derivatives", (VectorMultivariateFunction<EffectiveTag>(*)(const ScalarMultivariateFunction<EffectiveTag>&)) &derivatives);
        module.def("lie_derivative", (ScalarMultivariateFunction<EffectiveTag>(*)(const ScalarMultivariateFunction<EffectiveTag>&,const VectorMultivariateFunction<EffectiveTag>&)) &lie_derivative);
    }
}


template<class Y, class X> Void define_procedure_evaluation(pybind11::module& module) {
    module.def("evaluate", (X(*)(Procedure<Y> const&, Vector<X> const&)) &evaluate);
    module.def("gradient", (Covector<X>(*)(Procedure<Y> const&, Vector<X> const&)) &gradient);
    module.def("hessian", (X(*)(Procedure<Y> const&, Vector<X> const&, Vector<X> const&)) &hessian);
}

template<class Y> pybind11::class_<Procedure<Y>> export_procedure(pybind11::module& module) {
    typedef Paradigm<Y> P;
    pybind11::class_<Procedure<Y>> procedure_class(module,(class_name<P>()+"Procedure").c_str());
    procedure_class.def("__str__", &__cstr__<Procedure<Y>>);
    module.def("make_procedure", (Procedure<Y>(*)(ScalarMultivariateFunction<P> const&)) &make_procedure);
    if constexpr (Same<Y,ApproximateNumber>) {
        define_procedure_evaluation<Y,FloatDPApproximation>(module);
        //define_procedure_evaluation<Y,FloatMPApproximation>(module);
    }
    if constexpr (Same<Y,ValidatedNumber>) {
        define_procedure_evaluation<Y,FloatDPBounds>(module);
        //define_procedure_evaluation<Y,FloatMPBounds>(module);
    }
    return procedure_class;
}

Void export_procedures(pybind11::module& module) {
    export_procedure<ApproximateNumber>(module);
    export_procedure<ValidatedNumber>(module);
}



template<class P, class SIG> Void export_function_patch(pybind11::module& module) {
    using FP=FunctionPatch<P,SIG>;
    using RES = typename SignatureTraits<SIG>::ResultKind;
    using ARG = typename SignatureTraits<SIG>::ArgumentKind;
    using DomainType = typename FunctionPatch<P,SIG>::DomainType;
    using ArgumentIndexType = ElementIndexType<DomainType>;

    pybind11::class_<FP> function_patch_class(module,(class_name<P>()+signature_name<SIG>()+"FunctionPatch").c_str());
    function_patch_class.def(pybind11::init<FunctionPatch<P,SIG>>());

    function_patch_class.def("result_size", &FP::result_size);
    function_patch_class.def("argument_size", &FP::argument_size);
    function_patch_class.def("domain", &FP::domain);
    function_patch_class.def("codomain", &FP::codomain);
    function_patch_class.def("range", &FP::range);

    function_patch_class.def("__str__", &__cstr__<FP>);

    if constexpr (Same<RES,RealScalar>) {
        define_elementary_algebra<FP,Number<P>>(module,function_patch_class);
    }
    if constexpr (Same<RES,RealVector>) {
        define_vector_algebra_arithmetic<FunctionPatch<P,RealVector(ARG)>,FunctionPatch<P,RealScalar(ARG)>,Number<P>>(module,function_patch_class);
    }

    // TODO: Export all function operators
    // export_function_evaluation(module,function_patch_class);
    // export_function_derivatives(module,function_patch_class);


    if constexpr (IsStronger<P,ApproximateTag>::value) {
        define_call<FloatDPApproximation>(function_patch_class);
        define_call<FloatMPApproximation>(function_patch_class);
    }
    if constexpr (IsStronger<P,ValidatedTag>::value) {
        define_call<FloatDPBounds>(function_patch_class);
        define_call<FloatMPBounds>(function_patch_class);
    }

    if  constexpr (Same<RES,RealVector>) {
        module.def("antiderivative", [](FunctionPatch<P,SIG>const& fp, ArgumentIndexType j){return _antiderivative_(fp,j);});
        module.def("antiderivative", [](FunctionPatch<P,SIG>const& fp, ArgumentIndexType j, Number<P> const& c){return _antiderivative_(fp,j,c);});
    }

    module.def("compose", [](const Function<P,RealScalar(RES)>& f, const FunctionPatch<P,SIG>& gp){return compose(f,gp);});
    module.def("compose", [](const Function<P,RealVector(RES)>& f, const FunctionPatch<P,SIG>& gp){return compose(f,gp);});
    module.def("compose", [](const FunctionPatch<P,RealScalar(RES)>& fp, const FunctionPatch<P,SIG>& gp){return compose(fp,gp);});
    module.def("compose", [](const FunctionPatch<P,RealVector(RES)>& fp, const FunctionPatch<P,SIG>& gp){return compose(fp,gp);});
    module.def("unchecked_compose", [](const FunctionPatch<P,RealScalar(RES)>& fp, const FunctionPatch<P,SIG>& gp){return unchecked_compose(fp,gp);});
    module.def("unchecked_compose", [](const FunctionPatch<P,RealVector(RES)>& fp, const FunctionPatch<P,SIG>& gp){return unchecked_compose(fp,gp);});

    //module.def("restriction", (FunctionPatch<P,SIG>(*)(Function<P,SIG> const&, DomainType const&)) &restriction);
}

template<class SIG> Void export_function_patches(pybind11::module& module) {
    export_function_patch<ValidatedTag,SIG>(module);
    export_function_patch<ApproximateTag,SIG>(module);
}

Void export_function_patches(pybind11::module& module) {
    // TODO: Correct export of univariate function patches.
    export_function_patches<RealScalar(RealScalar)>(module);
    // export_function_patches<RealVector(RealScalar)>(module);
    export_function_patches<RealScalar(RealVector)>(module);
    export_function_patches<RealVector(RealVector)>(module);
}


Void function_submodule(pybind11::module& module) {

    export_multi_index(module);

    export_polynomials(module);

    export_domains(module);

    export_functions<RealScalar(RealScalar)>(module);
    export_functions<RealVector(RealScalar)>(module);
    export_functions<RealScalar(RealVector)>(module);
    export_functions<RealVector(RealVector)>(module);

    export_function_patches(module);
    export_procedures(module);

    template_<Function> function_template(module,python_template_name<Function>().c_str());
    function_template.def_new([](RealVariable var, Scalar<RealExpression> e){return EffectiveScalarUnivariateFunction(var,e);});
    function_template.def_new([](RealVariable var, Vector<RealExpression> e){return EffectiveVectorUnivariateFunction(var,e);});
    function_template.def_new([](RealSpace spc, Scalar<RealExpression> e){return EffectiveScalarMultivariateFunction(spc,e);});
    function_template.def_new([](RealSpace spc, Vector<RealExpression> e){return EffectiveVectorMultivariateFunction(spc,e);});

    function_template.def_new([](pybind11::object pyf){return univariate_function_from_python(pyf);});
    function_template.def_new([](SizeType as, pybind11::object pyf){return multivariate_function_from_python(as,pyf);});

}
