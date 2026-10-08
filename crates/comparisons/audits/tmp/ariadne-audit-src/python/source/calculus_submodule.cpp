/***************************************************************************
 *            calculus_submodule.cpp
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

#include <type_traits>

#include "pybind11.hpp"
#include "utilities.hpp"
#include "numeric_submodule.hpp"

#include "algebra/expansion.tpl.hpp"
#include "algebra/algebra.hpp"
#include "function/function_interface.hpp"
#include "function/polynomial.hpp"
#include "function/function.hpp"
#include "function/procedure.hpp"

#include "function/function_model.hpp"
#include "function/taylor_model.hpp"
#include "function/taylor_function.hpp"

using namespace Ariadne;

namespace Ariadne {

template<class PR> std::string numeric_class_tag();

template<template<class...>class T, class PR> std::string python_template_tag_name() { return python_template_name<T>()+numeric_class_tag<PR>(); }
template<template<class...>class T, class FLT> std::string python_template_precision_tag_name() { return python_template_name<T>()+numeric_class_tag<typename FLT::PrecisionType>(); }

template<> struct PythonTemplateName<Sweeper> { static std::string get() { return "Sweeper"; } };
template<> struct PythonTemplateName<ThresholdSweeper> { static std::string get() { return "ThresholdSweeper"; } };
template<> struct PythonTemplateName<GradedSweeper> { static std::string get() { return "GradedSweeper"; } };

template<class FLT> struct PythonClassName<Sweeper<FLT>> {
    static std::string get() { return python_template_precision_tag_name<Sweeper,FLT>(); } };
template<class FLT> struct PythonClassName<ThresholdSweeper<FLT>> {
    static std::string get() { return python_template_precision_tag_name<ThresholdSweeper,FLT>(); } };
template<class FLT> struct PythonClassName<GradedSweeper<FLT>> {
    static std::string get() { return python_template_precision_tag_name<GradedSweeper,FLT>(); } };

template<> struct PythonTemplateName<ApproximateTaylorModel> { static std::string get() { return "ApproximateTaylorModel"; } };
template<> struct PythonTemplateName<ValidatedTaylorModel> { static std::string get() { return "ValidatedTaylorModel"; } };
template<> struct PythonTemplateName<ValidatedScalarMultivariateFunctionModel> { static std::string get() { return "ValidatedScalarMultivariateFunctionModel"; } };
template<> struct PythonTemplateName<ValidatedVectorMultivariateFunctionModel> { static std::string get() { return "ValidatedVectorMultivariateFunctionModel"; } };
template<> struct PythonTemplateName<ValidatedScalarMultivariateTaylorFunctionModel> { static std::string get() { return "ValidatedScalarMultivariateTaylorFunctionModel"; } };
template<> struct PythonTemplateName<ValidatedVectorMultivariateTaylorFunctionModel> { static std::string get() { return "ValidatedVectorMultivariateTaylorFunctionModel"; } };

template<class PR> struct PythonClassName<ValidatedScalarMultivariateFunctionModel<PR>> { static std::string get() { return python_template_tag_name<ValidatedScalarMultivariateFunctionModel,PR>(); } };
template<class PR> struct PythonClassName<ValidatedVectorMultivariateFunctionModel<PR>> { static std::string get() { return python_template_tag_name<ValidatedVectorMultivariateFunctionModel,PR>(); } };


template<class FLT> struct PythonClassName<ApproximateTaylorModel<FLT>> {
    static std::string get() { return python_template_precision_tag_name<ApproximateTaylorModel,FLT>(); } };
template<class FLT> struct PythonClassName<ValidatedTaylorModel<FLT>> {
    static std::string get() { return python_template_precision_tag_name<ValidatedTaylorModel,FLT>(); } };

template<class FLT> struct PythonClassName<ValidatedScalarMultivariateTaylorFunctionModel<FLT>> {
    static std::string get() { return python_template_precision_tag_name<ValidatedScalarMultivariateTaylorFunctionModel,FLT>(); } };
template<class FLT> struct PythonClassName<ValidatedVectorMultivariateTaylorFunctionModel<FLT>> {
    static std::string get() { return python_template_precision_tag_name<ValidatedVectorMultivariateTaylorFunctionModel,FLT>(); } };


//ValidatedNumericType evaluate(const ValidatedScalarMultivariateFunctionModelDP& f, const Vector<ValidatedNumericType>& x) { return f(x); }
//Vector<ValidatedNumericType> evaluate(const ValidatedVectorMultivariateFunctionModelDP& f, const Vector<ValidatedNumericType>& x) { return f(x); }

//ValidatedScalarMultivariateFunctionModelDP partial_evaluate(const ValidatedScalarMultivariateFunctionModelDP&, SizeType, const ValidatedNumericType&);
//ValidatedVectorMultivariateFunctionModelDP partial_evaluate(const ValidatedVectorMultivariateFunctionModelDP&, SizeType, const ValidatedNumericType&);

//ValidatedScalarMultivariateFunctionModelDP antiderivative(const ValidatedScalarMultivariateFunctionModelDP&, SizeType, ValidatedNumericType);
//ValidatedVectorMultivariateFunctionModelDP antiderivative(const ValidatedVectorMultivariateFunctionModelDP&, SizeType, ValidatedNumericType);

//ValidatedScalarMultivariateFunctionModelDP compose(const ValidatedScalarMultivariateFunctionModelDP&, const ValidatedVectorMultivariateFunctionModelDP&);
//ValidatedScalarMultivariateFunctionModelDP compose(const ValidatedScalarMultivariateFunction&, const ValidatedVectorMultivariateFunctionModelDP&);
//ValidatedVectorMultivariateFunctionModelDP compose(const ValidatedVectorMultivariateFunctionModelDP&, const ValidatedVectorMultivariateFunctionModelDP&);
//ValidatedVectorMultivariateFunctionModelDP compose(const ValidatedVectorMultivariateFunction&, const ValidatedVectorMultivariateFunctionModelDP&);

//ValidatedVectorMultivariateFunctionModelDP join(const ValidatedScalarMultivariateFunctionModelDP&, const ValidatedScalarMultivariateFunctionModelDP&);
//ValidatedVectorMultivariateFunctionModelDP join(const ValidatedScalarMultivariateFunctionModelDP&, const ValidatedVectorMultivariateFunctionModelDP&);
//ValidatedVectorMultivariateFunctionModelDP join(const ValidatedVectorMultivariateFunctionModelDP&, const ValidatedScalarMultivariateFunctionModelDP&);
//ValidatedVectorMultivariateFunctionModelDP join(const ValidatedVectorMultivariateFunctionModelDP&, const ValidatedVectorMultivariateFunctionModelDP&);

template<class P, class SIG, class PR, class PRE> OutputStream& operator<<(OutputStream& os, const Representation< FunctionModel<P,SIG,PR,PRE> >& frepr) {
    static_cast<const FunctionModelInterface<P,SIG,PR,PRE>&>(frepr.reference())._repr(os); return os;
}

ValidatedVectorMultivariateTaylorFunctionModelDP __getslice__(const ValidatedVectorMultivariateTaylorFunctionModelDP& tf, Int start, Int stop) {
    Int rs = tf.result_size();
    if(start<0) { start+=rs; }
    if(stop<0) { stop+=rs; }
    ARIADNE_ASSERT_MSG(0<=start&&start<=stop&&stop<=rs,
            "result_size="<<rs<<", start="<<start<<", stop="<<stop);
    return ValidatedVectorMultivariateTaylorFunctionModelDP(tf.domain(),Vector<ValidatedTaylorModelDP>(project(tf.models(),range(static_cast<SizeType>(start),static_cast<SizeType>(stop)))));
}



template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation< X >& repr) {
    return os << repr.reference();
}

template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation< Expansion<MultiIndex,X> >& repr) {
    const Expansion<MultiIndex,X>& exp=repr.reference();
    for(typename Expansion<MultiIndex,X>::ConstIterator iter=exp.begin(); iter!=exp.end(); ++iter) {
        os << (iter==exp.begin()?'{':',') << "(";
        for(SizeType j=0; j!=iter->index().size(); ++j) {
            if(j!=0) { os << ','; } os << Int(iter->index()[j]);
        }
        os << "):" << python_representation(iter->coefficient());
    }
    os << "}";
    return os;
}

template<class X, class CMP> OutputStream& operator<<(OutputStream& os, const PythonRepresentation< SortedExpansion<MultiIndex,X,CMP> >& repr) {
    return os << python_representation(static_cast<const Expansion<MultiIndex,X>&>(repr.reference()));
}

template OutputStream& operator<<(OutputStream&, const PythonRepresentation< Expansion<MultiIndex,FloatDPApproximation> >&);
template OutputStream& operator<<(OutputStream&, const PythonRepresentation< Expansion<MultiIndex,FloatDPBounds> >&);
template OutputStream& operator<<(OutputStream&, const PythonRepresentation< Expansion<MultiIndex,FloatDP> >&);

template OutputStream& operator<<(OutputStream&, const PythonRepresentation< Expansion<MultiIndex,FloatMPApproximation> >&);
template OutputStream& operator<<(OutputStream&, const PythonRepresentation< Expansion<MultiIndex,FloatMPBounds> >&);
template OutputStream& operator<<(OutputStream&, const PythonRepresentation< Expansion<MultiIndex,FloatMP> >&);


template<class X> OutputStream& operator<<(OutputStream& os, const PythonRepresentation< Vector<X> >& repr) {
    const Vector<X>& vec=repr.reference();
    os << "[";
    for(SizeType i=0; i!=vec.size(); ++i) {
        if(i!=0) { os << ","; }
        os << python_representation(vec[i]);
    }
    os << "]";
    return os;
}

OutputStream& operator<<(OutputStream& os, const PythonRepresentation< ExactBoxType >& bx) {
    return os << PythonRepresentation< Vector<ExactIntervalType> >(cast_vector(bx.reference())); }

OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Sweeper<FloatDP>>& repr) {
    const Sweeper<FloatDP>& swp=repr.reference();
    auto swp_ptr = &static_cast<const SweeperInterface<FloatDP>&>(swp);
    auto thresh_swp_ptr = dynamic_cast<const ThresholdSweeper<FloatDP>*>(swp_ptr);
    if(thresh_swp_ptr) {
        os << "ThresholdSweeperDP(" << thresh_swp_ptr->sweep_threshold() << ")";
    } else {
        os << swp;
    }
    return os;
}

OutputStream& operator<<(OutputStream& os, const PythonRepresentation<ValidatedScalarMultivariateTaylorFunctionModelDP>& repr) {
    const ValidatedScalarMultivariateTaylorFunctionModelDP& stf=repr.reference();
    os << std::setprecision(17);
    os << "ValidatedScalarMultivariateTaylorFunctionModelDP"
       << "(" << python_representation(stf.domain())
       << "," << python_representation(stf.expansion())
       << "," << python_representation(stf.error())
       << "," << python_representation(stf.properties())
       << ")";
    return os;
}

OutputStream& operator<<(OutputStream& os, const PythonRepresentation<ValidatedVectorMultivariateTaylorFunctionModelDP>& repr) {
    const ValidatedVectorMultivariateTaylorFunctionModelDP& vtf=repr.reference();
    os << std::setprecision(17);
    os << "ValidatedVectorMultivariateTaylorFunctionModelDP"
       << "(" << python_representation(vtf.domain())
       << "," << python_representation(vtf.expansions())
       << "," << python_representation(vtf.errors())
       << "," << python_representation(vtf.properties())
       << ")";
    return os;
}

template<class P, class F> List<MultiIndex> keys(const TaylorModel<P,F>& tm) {
    List<MultiIndex> r(tm.argument_size());
    for(auto iter=tm.expansion().begin(); iter!=tm.expansion().end(); ++iter) {
        r.append(iter->index());
    }
    return r;
}

ValidatedScalarMultivariateFunction unrestrict(const ValidatedScalarMultivariateFunctionModelDP& fm) {
    return ValidatedScalarMultivariateFunction(fm.raw_pointer()->_clone());
}

ValidatedVectorMultivariateFunction unrestrict(const ValidatedVectorMultivariateFunctionModelDP& fm) {
    return ValidatedVectorMultivariateFunction(fm.raw_pointer()->_clone());
}

} // namespace Ariadne

static constexpr auto self = pybind11::detail::self;

Sweeper<FloatDP> make_threshold_sweeper(DoublePrecision pr, double x) {
    return Sweeper<FloatDP>(std::make_shared<ThresholdSweeper<FloatDP>>(pr,x)); }
Sweeper<FloatDP> make_graded_sweeper(DoublePrecision pr, SizeType n) {
    return Sweeper<FloatDP>(std::make_shared<GradedSweeper<FloatDP>>(pr,n)); }


template<class FLT> Void export_sweepers(pybind11::module& module)
{
    using PR=PrecisionType<FLT>;
    pybind11::class_<Sweeper<FLT>> sweeper_class(module,python_class_name<Sweeper<FLT>>().c_str());
    sweeper_class.def(pybind11::init<Sweeper<FLT>>());
    sweeper_class.def("__str__", &__cstr__<Sweeper<FLT>>);

    pybind11::class_<ThresholdSweeper<FLT>> threshold_sweeper_class(module,python_class_name<ThresholdSweeper<FLT>>().c_str());
    threshold_sweeper_class.def(pybind11::init<PR,double>());
    threshold_sweeper_class.def("__str__", &__cstr__<ThresholdSweeper<FLT>>);
    sweeper_class.def(pybind11::init<ThresholdSweeper<FLT>>());
    pybind11::implicitly_convertible<ThresholdSweeper<FLT>,Sweeper<FLT>>();

    pybind11::class_<GradedSweeper<FLT>> graded_sweeper_class(module,python_class_name<GradedSweeper<FLT>>().c_str());
    graded_sweeper_class.def(pybind11::init<PR,int>());
    graded_sweeper_class.def("__str__", &__cstr__<GradedSweeper<FLT>>);
    sweeper_class.def(pybind11::init<GradedSweeper<FLT>>());
    pybind11::implicitly_convertible<GradedSweeper<FLT>,Sweeper<FLT>>();

}


Expansion<MultiIndex,FloatDP>const& get_expansion(ValidatedTaylorModelDP const& tm) { return tm.expansion(); }
Expansion<MultiIndex,FloatDPApproximation>const& get_expansion(ApproximateTaylorModelDP const& tm) { return tm.expansion(); }


template<class FLT> Void export_validated_taylor_model(pybind11::module& module)
{
    typedef ValidatedTaylorModel<FLT> ModelType;
    typedef NumericType<ModelType> NumericType;
    typedef GenericType<NumericType> GenericNumericType;

    Tag<GenericNumericType> generic_number_tag;

    pybind11::class_<ValidatedTaylorModel<FLT>> taylor_model_class(module,python_class_name<ValidatedTaylorModel<FLT>>().c_str());
    taylor_model_class.def(pybind11::init<ValidatedTaylorModel<FLT>>());
    taylor_model_class.def(pybind11::init< SizeType,Sweeper<FLT> >());
    taylor_model_class.def("keys", (List<MultiIndex>(*)(const ValidatedTaylorModel<FLT>&))&keys);
    taylor_model_class.def("value", (const FloatDP(ValidatedTaylorModel<FLT>::*)()const) &ValidatedTaylorModel<FLT>::value);
    taylor_model_class.def("error", (const FloatDPError&(ValidatedTaylorModel<FLT>::*)()const) &ValidatedTaylorModel<FLT>::error);
    taylor_model_class.def("expansion", (const Expansion<MultiIndex,FLT>&(*)(ValidatedTaylorModel<FLT> const&)) &get_expansion);
    taylor_model_class.def("set_error", (Void(ValidatedTaylorModel<FLT>::*)(const FloatDPError&)) &ValidatedTaylorModel<FLT>::set_error);
    taylor_model_class.def("argument_size", &ValidatedTaylorModel<FLT>::argument_size);
    taylor_model_class.def("domain", &ValidatedTaylorModel<FLT>::domain);
    taylor_model_class.def("range", &ValidatedTaylorModel<FLT>::range);
    taylor_model_class.def("norm", &ValidatedTaylorModel<FLT>::norm);
    taylor_model_class.def("set_sweeper", &ValidatedTaylorModel<FLT>::set_sweeper);
    taylor_model_class.def("sweeper", &ValidatedTaylorModel<FLT>::sweeper);
    taylor_model_class.def("sweep", (ValidatedTaylorModel<FLT>&(ValidatedTaylorModel<FLT>::*)()) &ValidatedTaylorModel<FLT>::sweep, pybind11::return_value_policy::reference);
    taylor_model_class.def("__getitem__", &__getitem__<ValidatedTaylorModel<FLT>,MultiIndex,FLT>);
    taylor_model_class.def("__setitem__",&__setitem__<ValidatedTaylorModel<FLT>,MultiIndex,FLT>);

    define_elementary_algebra(module,taylor_model_class);
    define_inplace_algebra(module,taylor_model_class);
    define_mixed_arithmetic(module, taylor_model_class, generic_number_tag);

    define_lattice(module,taylor_model_class);


    taylor_model_class.def("__str__", &__cstr__<ValidatedTaylorModel<FLT>>);

    taylor_model_class.def_static("constant",(ValidatedTaylorModel<FLT>(*)(SizeType, const FloatDPBounds&,Sweeper<FLT>))&ValidatedTaylorModel<FLT>::constant);
    taylor_model_class.def_static("coordinate",(ValidatedTaylorModel<FLT>(*)(SizeType, SizeType,Sweeper<FLT>))&ValidatedTaylorModel<FLT>::coordinate);

    taylor_model_class.def("range", (UpperIntervalType(ValidatedTaylorModel<FLT>::*)()const) &ValidatedTaylorModel<FLT>::range);

    module.def("evaluate",&_evaluate_<ValidatedTaylorModel<FLT>,Vector<FloatDPBounds>>);
    //module.def("split",&_split_<ValidatedTaylorModel<FLT>,SizeType,SplitPart>);

    //to_python< Vector<ValidatedTaylorModel<FLT>> >();
    //from_python< Vector<ValidatedTaylorModel<FLT>> >();
}

template<class FLT> Void export_approximate_taylor_model(pybind11::module& module)
{
    typedef ApproximateTaylorModel<FLT> ModelType;

    pybind11::class_<ApproximateTaylorModel<FLT>> taylor_model_class(module,python_class_name<ApproximateTaylorModel<FLT>>().c_str());
    taylor_model_class.def(pybind11::init<ModelType>());
    taylor_model_class.def(pybind11::init< SizeType,Sweeper<FLT> >());
    taylor_model_class.def("keys", (List<MultiIndex>(*)(const ModelType&))&keys);
    taylor_model_class.def("value", (const Approximation<FLT>(ModelType::*)()const) &ModelType::value);
    taylor_model_class.def("expansion", (const Expansion<MultiIndex,Approximation<FLT>>&(*)(ModelType const&)) &get_expansion);
    taylor_model_class.def("argument_size", &ModelType::argument_size);
    taylor_model_class.def("domain", &ModelType::domain);
    taylor_model_class.def("range", &ModelType::range);
    taylor_model_class.def("norm", &ModelType::norm);
    taylor_model_class.def("set_sweeper", &ModelType::set_sweeper);
    taylor_model_class.def("sweeper", &ModelType::sweeper);
    taylor_model_class.def("sweep", (ModelType&(ModelType::*)()) &ModelType::sweep, pybind11::return_value_policy::reference);
    taylor_model_class.def("__getitem__", &__getitem__<ModelType,MultiIndex,Approximation<FLT>>);
    taylor_model_class.def("__setitem__",&__setitem__<ModelType,MultiIndex,Approximation<FLT>>);

    define_elementary_algebra(module,taylor_model_class);
    define_inplace_algebra(module,taylor_model_class);
    define_lattice(module,taylor_model_class);

    taylor_model_class.def("__str__",&__cstr__<ModelType>);

    taylor_model_class.def_static("constant",(ModelType(*)(SizeType, const Approximation<FLT>&,Sweeper<FLT>))&ModelType::constant);
    taylor_model_class.def_static("coordinate",(ModelType(*)(SizeType, SizeType,Sweeper<FLT>))&ModelType::coordinate);

    //FIXME: Not in C++ API
    //module.def("evaluate",&_evaluate_<ModelType,Vector<ApproximateNumericType>>);
    //module.def("split",&_split_<ModelType,SizeType,SplitPart>);

    //from_python< Vector<ModelType> >();
    //to_python< Vector<ModelType> >();
}



template<class PR> Void export_scalar_function_model(pybind11::module& module)
{
    using FLT = RawFloatType<PR>;

    typedef typename ValidatedScalarMultivariateFunctionModel<PR>::NumericType NumericType;
    typename NumericType::PrecisionType pr;

    pybind11::class_<ValidatedScalarMultivariateFunctionModel<PR>> scalar_function_model_class(module,python_class_name<ValidatedScalarMultivariateFunctionModel<PR>>().c_str());
    scalar_function_model_class.def(pybind11::init<ValidatedScalarMultivariateFunctionModel<PR>>());
    scalar_function_model_class.def(pybind11::init<ValidatedScalarMultivariateTaylorFunctionModel<FLT>>());
    scalar_function_model_class.def("argument_size", &ValidatedScalarMultivariateFunctionModel<PR>::argument_size);
    scalar_function_model_class.def("domain", &ValidatedScalarMultivariateFunctionModel<PR>::domain);
    scalar_function_model_class.def("codomain", &ValidatedScalarMultivariateFunctionModel<PR>::codomain);
    scalar_function_model_class.def("range", &ValidatedScalarMultivariateFunctionModel<PR>::range);
    scalar_function_model_class.def("clobber", &ValidatedScalarMultivariateFunctionModel<PR>::clobber);
    scalar_function_model_class.def("error", &ValidatedScalarMultivariateFunctionModel<PR>::error);
    scalar_function_model_class.def("__call__", (FloatBounds<PR>(ValidatedScalarMultivariateFunctionModel<PR>::*)(const Vector<FloatBounds<PR>>&)const) &ValidatedScalarMultivariateFunctionModel<PR>::operator());
    scalar_function_model_class.def(self+self);
    scalar_function_model_class.def(self-self);
    scalar_function_model_class.def(self*self);
    scalar_function_model_class.def(self/self);
    scalar_function_model_class.def(self+NumericType(pr));
    scalar_function_model_class.def(self-NumericType(pr));
    scalar_function_model_class.def(self*NumericType(pr));
    scalar_function_model_class.def(self/NumericType(pr));
    scalar_function_model_class.def(NumericType(pr)+self);
    scalar_function_model_class.def(NumericType(pr)-self);
    scalar_function_model_class.def(NumericType(pr)*self);
    scalar_function_model_class.def(NumericType(pr)/self);
    scalar_function_model_class.def("__str__", &__cstr__<ValidatedScalarMultivariateFunctionModel<PR>>);
    scalar_function_model_class.def("__repr__", &__crepr__<ValidatedScalarMultivariateFunctionModel<PR>>);

    module.def("norm", &_norm_<ValidatedScalarMultivariateFunctionModel<PR>>);

    module.def("evaluate", &_evaluate_<ValidatedScalarMultivariateFunctionModel<PR>,Vector<NumericType>>);
    module.def("partial_evaluate", &_partial_evaluate_<ValidatedScalarMultivariateFunctionModel<PR>,SizeType,NumericType>);

    module.def("compose", _compose_<ValidatedScalarMultivariateFunctionModel<PR>,ValidatedVectorMultivariateFunctionModel<PR>>);
    module.def("compose", _compose_<ValidatedScalarMultivariateFunction,ValidatedVectorMultivariateFunctionModel<PR>>);

    module.def("unrestrict", (ValidatedScalarMultivariateFunction(*)(const ValidatedScalarMultivariateFunctionModel<PR>&)) &unrestrict);

    module.def("antiderivative",  &_antiderivative_<ValidatedScalarMultivariateFunctionModel<PR>,SizeType,NumericType>);
}

template<class PR> Void export_vector_function_model(pybind11::module& module)
{
    using FLT = RawFloatType<PR>;

    typedef typename ValidatedScalarMultivariateFunctionModel<PR>::NumericType NumericType;
    //using VectorMultivariateFunctionModelType = ValidatedVectorMultivariateFunctionModel<PR>;
    //using ScalarMultivariateFunctionModelType = ValidatedScalarMultivariateFunctionModel<PR>;

    pybind11::class_<ValidatedVectorMultivariateFunctionModel<PR>> vector_function_model_class(module,python_class_name<ValidatedVectorMultivariateFunctionModel<PR>>().c_str());
    vector_function_model_class.def(pybind11::init<ValidatedVectorMultivariateFunctionModel<PR>>());
    vector_function_model_class.def(pybind11::init<ValidatedVectorMultivariateTaylorFunctionModel<FLT>>());
    vector_function_model_class.def("result_size", &ValidatedVectorMultivariateFunctionModel<PR>::result_size);
    vector_function_model_class.def("argument_size", &ValidatedVectorMultivariateFunctionModel<PR>::argument_size);
    vector_function_model_class.def("domain", &ValidatedVectorMultivariateFunctionModel<PR>::domain);
    vector_function_model_class.def("codomain", &ValidatedVectorMultivariateFunctionModel<PR>::codomain);
    vector_function_model_class.def("range", &ValidatedVectorMultivariateFunctionModel<PR>::range);
    //vector_function_model_class.def("__getslice__", (ValidatedVectorMultivariateTaylorFunctionModel<PR>(*)(const ValidatedVectorMultivariateTaylorFunctionModel<PR>&,Int,Int))&__getslice__);
    vector_function_model_class.def("__getitem__", &__getitem__<ValidatedVectorMultivariateFunctionModel<PR>,SizeType,ValidatedScalarMultivariateFunctionModel<PR>>);
    vector_function_model_class.def("__setitem__",&__setitem__<ValidatedVectorMultivariateFunctionModel<PR>,SizeType,ValidatedScalarMultivariateFunctionModel<PR>>);
    //vector_function_model_class.def("__setitem__",&__setitem__<ValidatedVectorMultivariateFunctionModel<PR>,SizeType,ValidatedScalarMultivariateFunction>);
    vector_function_model_class.def("__call__", (Vector<FloatBounds<PR>>(ValidatedVectorMultivariateFunctionModel<PR>::*)(const Vector<FloatBounds<PR>>&)const) &ValidatedVectorMultivariateFunctionModel<PR>::operator());

    // NOTE: Not all operations are exported in C++ API.
    //define_vector_algebra_arithmetic<VectorMultivariateFunctionModelType,ScalarMultivariateFunctionModelType,NumericType>(module,vector_function_model_class);
    //define_vector_arithmetic<VectorMultivariateFunctionModelType,ScalarMultivariateFunctionModelType>(module,vector_function_model_class);

    vector_function_model_class.def("__str__", &__cstr__<ValidatedVectorMultivariateFunctionModel<PR>>);
    vector_function_model_class.def("__repr__", &__crepr__<ValidatedVectorMultivariateFunctionModel<PR>>);
    //export_vector_function_model.def(pybind11::module& module, "__repr__",&__repr__<ValidatedVectorMultivariateFunctionModel<PR>>);


//    module.def("evaluate", (Vector<ValidatedNumericType>(*)(const ValidatedVectorMultivariateFunctionModel<PR>&,const Vector<ValidatedNumericType>&)) &evaluate);
    module.def("partial_evaluate", &_partial_evaluate_<ValidatedVectorMultivariateFunctionModel<PR>,SizeType,NumericType>);

    module.def("compose", &_compose_<ValidatedVectorMultivariateFunctionModel<PR>,ValidatedVectorMultivariateFunctionModel<PR>>);
    module.def("compose", &_compose_<ValidatedVectorMultivariateFunction,ValidatedVectorMultivariateFunctionModel<PR>>);

    module.def("unrestrict", (ValidatedVectorMultivariateFunction(*)(const ValidatedVectorMultivariateFunctionModel<PR>&)) &unrestrict);

    module.def("norm", &_norm_<ValidatedVectorMultivariateFunctionModel<PR>>);

    module.def("join", &_join_<ValidatedScalarMultivariateFunctionModel<PR>,ValidatedScalarMultivariateFunctionModel<PR>>);
    module.def("join", &_join_<ValidatedScalarMultivariateFunctionModel<PR>,ValidatedVectorMultivariateFunctionModel<PR>>);
    module.def("join", &_join_<ValidatedVectorMultivariateFunctionModel<PR>,ValidatedScalarMultivariateFunctionModel<PR>>);
    module.def("join", &_join_<ValidatedVectorMultivariateFunctionModel<PR>,ValidatedVectorMultivariateFunctionModel<PR>>);

    module.def("combine", &_combine_<ValidatedVectorMultivariateFunctionModel<PR>,ValidatedVectorMultivariateFunctionModel<PR>>);
        module.def("combine", &_combine_<ValidatedVectorMultivariateFunctionModel<PR>,ValidatedScalarMultivariateFunctionModel<PR>>);

    module.def("antiderivative", &_antiderivative_<ValidatedVectorMultivariateFunctionModel<PR>,SizeType,NumericType>);
    module.def("antiderivative", &_antiderivative_<ValidatedVectorMultivariateFunctionModel<PR>,SizeType,ValidatedNumber>);

//    to_python< List<ValidatedVectorMultivariateFunctionModel<PR>> >();
}


template<class FLT> Void export_scalar_taylor_function(pybind11::module& module)
{
    using FunctionModelType = ValidatedScalarMultivariateTaylorFunctionModel<FLT>;
//    using FunctionType = ScalarMultivariateFunction<Paradigm<FunctionModelType>>;
    using GenericFunctionType = ScalarMultivariateFunction<ValidatedTag>;
    using NumericType = NumericType<FunctionModelType>;
    using GenericNumericType = GenericType<NumericType>;

    typedef ValidatedScalarUnivariateFunction SFN;
    typedef ValidatedVectorUnivariateFunction VFN;
    typedef ValidatedScalarMultivariateTaylorFunctionModel<FLT> F;
    typedef ValidatedVectorMultivariateTaylorFunctionModel<FLT> VF;
    typedef typename F::DomainType D;
    typedef typename F::NumericType X;
    typedef Vector<X> VX;
    typedef SizeType I;

    Tag<GenericNumericType> generic_number_tag;
    Tag<GenericFunctionType> generic_function_tag;

    pybind11::class_<ValidatedScalarMultivariateTaylorFunctionModel<FLT>> scalar_taylor_function_class(module,python_class_name<ValidatedScalarMultivariateTaylorFunctionModel<FLT>>().c_str());
    scalar_taylor_function_class.def(pybind11::init<ValidatedScalarMultivariateTaylorFunctionModel<FLT>>());
    scalar_taylor_function_class.def(pybind11::init<ExactBoxType,ValidatedTaylorModel<FLT>>());
    scalar_taylor_function_class.def(pybind11::init< ExactBoxType,Sweeper<FLT> >());
    scalar_taylor_function_class.def(pybind11::init< ExactBoxType, const EffectiveScalarMultivariateFunction&,Sweeper<FLT> >());
    scalar_taylor_function_class.def(pybind11::init< ExactBoxType, Expansion<MultiIndex,FLT>, Error<FLT>, Sweeper<FLT> >());
    scalar_taylor_function_class.def("error", (const Error<FLT>(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()const) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::error);
    scalar_taylor_function_class.def("set_error", (Void(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)(const Error<FLT>&)) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::set_error);
    scalar_taylor_function_class.def("argument_size", &F::argument_size);
    scalar_taylor_function_class.def("domain", &F::domain);
    scalar_taylor_function_class.def("codomain", &F::codomain);
    scalar_taylor_function_class.def("range", &F::range);
    scalar_taylor_function_class.def("model", (const ValidatedTaylorModel<FLT>&(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()const)&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::model);
    scalar_taylor_function_class.def("polynomial", (MultivariatePolynomial<NumericType>(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()const)&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::polynomial);
    scalar_taylor_function_class.def("number_of_nonzeros", (SizeType(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()const)&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::number_of_nonzeros);
    scalar_taylor_function_class.def("__getitem__", &__getitem__<ValidatedScalarMultivariateTaylorFunctionModel<FLT>,MultiIndex,FLT>);
    scalar_taylor_function_class.def("__setitem__",&__setitem__<ValidatedScalarMultivariateTaylorFunctionModel<FLT>,MultiIndex,FLT>);

    define_elementary_algebra(module,scalar_taylor_function_class);
    define_inplace_algebra(module,scalar_taylor_function_class);
    define_mixed_arithmetic(module,scalar_taylor_function_class,generic_number_tag);
    define_mixed_arithmetic(module,scalar_taylor_function_class,generic_function_tag);
    define_transcendental(module,scalar_taylor_function_class);
    define_lattice(module,scalar_taylor_function_class);

    scalar_taylor_function_class.def("__str__", &__cstr__<F>);
    scalar_taylor_function_class.def("__repr__", &__crepr__<F>);

    scalar_taylor_function_class.def("value", (const FLT(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()const) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::value);
    scalar_taylor_function_class.def("clobber", (Void(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::clobber);
    scalar_taylor_function_class.def("set_properties",&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::set_properties);
    scalar_taylor_function_class.def("properties",&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::properties);
    scalar_taylor_function_class.def("__call__", (Approximation<FLT>(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)(const Vector<Approximation<FLT>>&)const) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::operator());
    scalar_taylor_function_class.def("__call__", (Bounds<FLT>(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)(const Vector<Bounds<FLT>>&)const) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::operator());
//      scalar_taylor_function_class.def("gradient", (Covector<Bounds<FLT>>(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)(const Vector<Bounds<FLT>>&)const) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::gradient);
    scalar_taylor_function_class.def("function", (ValidatedScalarMultivariateFunction(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()const) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::function);
    scalar_taylor_function_class.def("polynomial", (MultivariatePolynomial<Bounds<FLT>>(ValidatedScalarMultivariateTaylorFunctionModel<FLT>::*)()const) &ValidatedScalarMultivariateTaylorFunctionModel<FLT>::polynomial);
    scalar_taylor_function_class.def("restriction", &_restriction_<F,D>);
    module.def("restrict", &_restriction_<F,D>);
//    scalar_taylor_function_class.def("extension",&_extension_<F,D>);

    scalar_taylor_function_class.def_static("zero",(ValidatedScalarMultivariateTaylorFunctionModel<FLT>(*)(const ExactBoxType&,Sweeper<FLT>))&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::zero);
    scalar_taylor_function_class.def_static("constant",(ValidatedScalarMultivariateTaylorFunctionModel<FLT>(*)(const ExactBoxType&,const NumericType&,Sweeper<FLT>))&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::constant);
    scalar_taylor_function_class.def_static("constant",[](const ExactBoxType& bx, const ValidatedNumber& c,Sweeper<FLT> swp){return ValidatedScalarMultivariateTaylorFunctionModel<FLT>::constant(bx,NumericType(c,swp.precision()),swp);});
    scalar_taylor_function_class.def_static("coordinate",(ValidatedScalarMultivariateTaylorFunctionModel<FLT>(*)(const ExactBoxType&,SizeType,Sweeper<FLT>))&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::coordinate);
    scalar_taylor_function_class.def_static("unit_ball",(ValidatedScalarMultivariateTaylorFunctionModel<FLT>(*)(const ExactBoxType&,Sweeper<FLT>))&ValidatedScalarMultivariateTaylorFunctionModel<FLT>::unit_ball);


    module.def("restriction", &_restriction_<F,D>);
    module.def("join",(VF(*)(F,F)) &_join_<F,F>);
//    module.def("extension",&_extension_<F,D>);
    module.def("embed", &_embed_<D,F,D>);
//    module.def("split", &_split_<F,I>);
    module.def("evaluate", &_evaluate_<F,VX>);
    module.def("partial_evaluate", &_partial_evaluate_<F,I,X>);
    module.def("midpoint", &_midpoint_<F>);
    module.def("derivative", &_derivative_<F,I>);
    module.def("antiderivative", &_antiderivative_<F,I>);
    module.def("antiderivative", &_antiderivative_<F,I,X>);
    module.def("norm", &_norm_<F>);

    module.def("inconsistent", &_inconsistent_<F,F>);
    module.def("refines", &_refines_<F,F>);
    module.def("refinement", &_refinement_<F,F>);

    module.def("compose", &_compose_<SFN,F>);
    module.def("compose", &_compose_<VFN,F>);

//    to_python< Vector<ValidatedScalarMultivariateTaylorFunctionModel<FLT>> >();
}


template<class FLT> Void export_vector_taylor_function(pybind11::module& module)
{
    using PR = typename FLT::PrecisionType;
    using ScalarMultivariateFunctionModelType = ValidatedScalarMultivariateFunctionModel<PR>;
    using NumericType = NumericType<ScalarMultivariateFunctionModelType>;

    typedef SizeType I;
    typedef ValidatedScalarMultivariateFunction SFN;
    typedef ValidatedVectorMultivariateFunction VFN;
    typedef ValidatedScalarMultivariateTaylorFunctionModel<FLT> SF;
    typedef ValidatedVectorMultivariateTaylorFunctionModel<FLT> VF;
    typedef typename VF::DomainType D;
    typedef typename D::IntervalType Di;
    typedef typename VF::NumericType X;
    typedef Vector<X> VX;

    Tag<ValidatedScalarMultivariateFunctionModel<PR>> scalar_taylor_function_tag;
    Tag<Vector<NumericType>> number_vector_tag;

    pybind11::class_<ValidatedVectorMultivariateTaylorFunctionModel<FLT>> vector_taylor_function_class(module,python_class_name<ValidatedVectorMultivariateTaylorFunctionModel<FLT>>().c_str());
    vector_taylor_function_class.def( pybind11::init([](List<ValidatedScalarMultivariateTaylorFunctionModel<FLT>> lst){return ValidatedVectorMultivariateTaylorFunctionModel<FLT>(Vector<ValidatedScalarMultivariateTaylorFunctionModel<FLT>>(lst));}));
    vector_taylor_function_class.def( pybind11::init< SizeType, ExactBoxType, Sweeper<FLT> >());
    vector_taylor_function_class.def( pybind11::init< ExactBoxType,const EffectiveVectorMultivariateFunction&,Sweeper<FLT> >());
    vector_taylor_function_class.def(pybind11::init< ExactBoxType, Vector< Expansion<MultiIndex,FLT> >, Vector<Error<FLT>>, Sweeper<FLT> >());
    vector_taylor_function_class.def( pybind11::init< Vector<ValidatedScalarMultivariateTaylorFunctionModel<FLT>> >());
    vector_taylor_function_class.def("_len_", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::result_size);
    vector_taylor_function_class.def("result_size", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::result_size);
    vector_taylor_function_class.def("argument_size", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::argument_size);
    vector_taylor_function_class.def("domain", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::domain);
    vector_taylor_function_class.def("codomain", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::codomain);
    // FIXME: Omitted since const and non-const versions
    // vector_taylor_function_class.def("models", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::models);
    vector_taylor_function_class.def("centre", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::centre);
    vector_taylor_function_class.def("range", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::range);
    vector_taylor_function_class.def("errors", &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::errors);
    vector_taylor_function_class.def("clobber", (Void(ValidatedVectorMultivariateTaylorFunctionModel<FLT>::*)()) &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::clobber);
    vector_taylor_function_class.def("set_properties",&ValidatedVectorMultivariateTaylorFunctionModel<FLT>::set_properties);
    vector_taylor_function_class.def("properties",&ValidatedVectorMultivariateTaylorFunctionModel<FLT>::properties);
    //FIXME: Omitted since project(...) fails
    //vector_taylor_function_class.def("__getslice__", &__getslice__<ValidatedVectorMultivariateTaylorFunctionModel<FLT>,SizeType,SizeType,ValidatedVectorMultivariateTaylorFunctionModel<FLT>>);
    vector_taylor_function_class.def("__getitem__", &__getitem__<ValidatedVectorMultivariateTaylorFunctionModel<FLT>,SizeType,ValidatedScalarMultivariateTaylorFunctionModel<FLT>>);
    vector_taylor_function_class.def("__setitem__",&__setitem__<ValidatedVectorMultivariateTaylorFunctionModel<FLT>,SizeType,ValidatedScalarMultivariateTaylorFunctionModel<FLT>>);

    define_vector_arithmetic(module,vector_taylor_function_class, scalar_taylor_function_tag);
    //FIXME: Not completely defined in C++ API
    //define_inplace_vector_arithmetic(module,vector_taylor_function_class, scalar_taylor_function_tag);
    define_inplace_mixed_vector_arithmetic(module,vector_taylor_function_class, number_vector_tag);

    vector_taylor_function_class.def("__str__", &__cstr__<ValidatedVectorMultivariateTaylorFunctionModel<FLT>>);
    vector_taylor_function_class.def("__repr__", &__crepr__<ValidatedVectorMultivariateTaylorFunctionModel<FLT>>);
    vector_taylor_function_class.def("clobber", (Void(ValidatedVectorMultivariateTaylorFunctionModel<FLT>::*)()) &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::clobber);
    vector_taylor_function_class.def("__call__", (Vector<Approximation<FLT>>(ValidatedVectorMultivariateTaylorFunctionModel<FLT>::*)(const Vector<Approximation<FLT>>&)const) &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::operator());
    vector_taylor_function_class.def("__call__", (Vector<Bounds<FLT>>(ValidatedVectorMultivariateTaylorFunctionModel<FLT>::*)(const Vector<Bounds<FLT>>&)const) &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::operator());
     vector_taylor_function_class.def("jacobian", (Matrix<Bounds<FLT>>(ValidatedVectorMultivariateTaylorFunctionModel<FLT>::*)(const Vector<Bounds<FLT>>&)const) &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::jacobian);
    vector_taylor_function_class.def("polynomials", (Vector< MultivariatePolynomial<Bounds<FLT>> >(ValidatedVectorMultivariateTaylorFunctionModel<FLT>::*)()const) &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::polynomials);
    vector_taylor_function_class.def("function", (ValidatedVectorMultivariateFunction(ValidatedVectorMultivariateTaylorFunctionModel<FLT>::*)()const) &ValidatedVectorMultivariateTaylorFunctionModel<FLT>::function);

    vector_taylor_function_class.def_static("constant",(ValidatedVectorMultivariateTaylorFunctionModel<FLT>(*)(const ExactBoxType&, const Vector<NumericType>&,Sweeper<FLT>))&ValidatedVectorMultivariateTaylorFunctionModel<FLT>::constant);
    vector_taylor_function_class.def_static("constant",[](const ExactBoxType& bx, const Vector<ValidatedNumber>& c,Sweeper<FLT> swp){return ValidatedVectorMultivariateTaylorFunctionModel<FLT>::constant(bx,Vector<NumericType>(c,swp.precision()),swp);});
    vector_taylor_function_class.def_static("identity",(ValidatedVectorMultivariateTaylorFunctionModel<FLT>(*)(const ExactBoxType&,Sweeper<FLT>))&ValidatedVectorMultivariateTaylorFunctionModel<FLT>::identity);

    module.def("norm", &_norm_<VF>);

    module.def("inconsistent", &_inconsistent_<VF,VF>);
    module.def("refinement", &_refinement_<VF,VF>);
    module.def("refines", &_refines_<VF,VF>);

    module.def("join", &_join_<SF,SF>);
    module.def("join", &_join_<SF,VF>);
    module.def("join", &_join_<VF,SF>);
    module.def("join", &_join_<VF,VF>);
    module.def("combine", &_combine_<SF,SF>);
    module.def("combine", &_combine_<SF,VF>);
    module.def("combine", &_combine_<VF,SF>);
    module.def("combine", &_combine_<VF,VF>);
    module.def("embed", &_embed_<VF,D>);
        module.def("embed", &_embed_<D,VF,D>);

    module.def("restriction", &_restriction_<VF,I,Di>);
    module.def("restrict", &_restriction_<VF,I,Di>);
//    module.def("split", &_split_<VF,I>);

    module.def("evaluate", &_evaluate_<VF,VX>);
    module.def("partial_evaluate", &_partial_evaluate_<VF,I,X>);
    module.def("compose", &_compose_<VF,VF>);
    module.def("compose", &_compose_<SF,VF>);
    module.def("compose", &_compose_<SFN,VF>);
    module.def("compose", &_compose_<VFN,VF>);
    module.def("unchecked_compose", &_compose_<SF,VF>);
    module.def("unchecked_compose", &_compose_<VF,VF>);
    module.def("derivative", &_derivative_<VF,I>);
    module.def("antiderivative", &_antiderivative_<VF,I>);
    module.def("antiderivative", &_antiderivative_<VF,I,X>);

    module.def("compose", &_compose_<EffectiveScalarMultivariateFunction,VF>);
    module.def("compose", &_compose_<EffectiveVectorMultivariateFunction,VF>);

}


Void calculus_submodule(pybind11::module& module)
{
    export_sweepers<FloatDP>(module);

    export_approximate_taylor_model<FloatDP>(module);
    export_validated_taylor_model<FloatDP>(module);

    export_scalar_function_model<DP>(module);
    export_vector_function_model<DP>(module);
    export_scalar_taylor_function<FloatDP>(module);
    export_vector_taylor_function<FloatDP>(module);

    template_<ThresholdSweeper> threshold_sweeper_template(module);
    threshold_sweeper_template.instantiate<FloatDP>();
    threshold_sweeper_template.def_new([](DP pr,ApproximateDouble eps){return ThresholdSweeper<FloatDP>(pr,eps);});
    template_<GradedSweeper> graded_sweeper_template(module);
    graded_sweeper_template.instantiate<FloatDP>();
    graded_sweeper_template.def_new([](DP pr,DegreeType deg){return GradedSweeper<FloatDP>(pr,deg);});

    template_<ValidatedScalarMultivariateFunctionModel> scalar_function_model_template(module);
    scalar_function_model_template.instantiate<DP>();
    template_<ValidatedVectorMultivariateFunctionModel> vector_function_model_template(module);
    vector_function_model_template.instantiate<DP>();

    template_<ValidatedScalarMultivariateTaylorFunctionModel> scalar_taylor_function_model_template(module);
    scalar_taylor_function_model_template.instantiate<FloatDP>();
    scalar_taylor_function_model_template.def_new([](BoxDomainType dom,ValidatedScalarMultivariateFunction f,Sweeper<FloatDP> swp){return ValidatedScalarMultivariateTaylorFunctionModel<FloatDP>(dom,f,swp);});
    template_<ValidatedVectorMultivariateTaylorFunctionModel> vector_taylor_function_model_template(module);
    vector_taylor_function_model_template.instantiate<FloatDP>();
    vector_taylor_function_model_template.def_new([](BoxDomainType dom,ValidatedVectorMultivariateFunction f,Sweeper<FloatDP> swp){return ValidatedVectorMultivariateTaylorFunctionModel<FloatDP>(dom,f,swp);});



}


