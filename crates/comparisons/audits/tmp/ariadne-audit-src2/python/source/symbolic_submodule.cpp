/***************************************************************************
 *            symbolic_submodule.cpp
 *
 *  Copyright  2009-20  Pieter Collins
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
#include <functional>

#include "utility/tribool.hpp"
#include "numeric/numeric.hpp"
#include "function/function.hpp"
#include "geometry/function_set.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/space.hpp"
#include "symbolic/expression_set.hpp"

using namespace Ariadne;


namespace Ariadne {

template<> struct PythonTemplateName<Constant> { static std::string get() { return "Constant"; } };
template<> struct PythonTemplateName<Variable> { static std::string get() { return "Variable"; } };
template<> struct PythonTemplateName<Expression> { static std::string get() { return "Expression"; } };
template<> struct PythonTemplateName<Valuation> { static std::string get() { return "Valuation"; } };

template<> struct PythonClassName<Expression<Boolean>> { static std::string get() { return "DiscretePredicate"; } };
template<> struct PythonClassName<Expression<Kleenean>> { static std::string get() { return "ContinuousPredicate"; } };

template<> struct PythonClassName<String> { static std::string get() { return "String"; } };
template<> struct PythonClassName<Valuation<Real>> { static std::string get() { return python_class_name<Real>()+python_template_name<Valuation>(); } };

template<class T> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Constant<T>>& repr) {
    Constant<T> const& cnst=repr.reference();
    return os << python_class_name<Constant<T>>() << "(\"" << cnst.name() << "," << cnst.value() << "\")";
}
template<class T> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Variable<T>>& repr) {
    Variable<T> const& var=repr.reference();
    return os << python_class_name<Variable<T>>() << "(\"" << var.name() << "\")";
}
template<class T> OutputStream& operator<<(OutputStream& os, const PythonRepresentation<Expression<T>>& repr) {
    Expression<T> const& expr=repr.reference();
    return os << python_class_name<Expression<T>>() << "(" << expr << ")";
}

template<class T, class Y> Expression<T> substitute(const Expression<T>& e, const Map<Variable<Y>,Expression<Y>>& a) {
    List<Assignment<Variable<Y>,Expression<Y>>> lst_a;
    for(auto key_val : a) { lst_a.append(let(key_val.first)=key_val.second); }
    return substitute(e,lst_a);
}

template<class T, class Y> Expression<T> substitute(const Expression<T>& e, const Map<Variable<Y>,Y>& a) {
    List<Assignment<Variable<Y>,Expression<Y>>> lst_a;
    for(auto key_val : a) { lst_a.append(let(key_val.first)=Expression<Y>(key_val.second)); }
    return substitute(e,lst_a);
}

template<class T> Nat __hash__(const T&);
template<> Nat __hash__<StringVariable>(const StringVariable& v) {
    return std::hash<const char*>()(v.name().c_str()); }



LetIntegerVariable let(const IntegerVariable&);
LetRealVariable let(const RealVariable&);

} // namespace Ariadne



Void export_constants(pybind11::module& module)
{
    pybind11::class_<RealConstant,pybind11::bases<Real>> real_constant_class(module,python_class_name<RealConstant>().c_str());
    real_constant_class.def(pybind11::init<Identifier,Real>());
    real_constant_class.def("name", [](RealConstant const& c){return c.name();});
    real_constant_class.def("value", &RealConstant::value);
    real_constant_class.def("__str__", &__cstr__<RealConstant>);
    real_constant_class.def("__repr__", &__repr__<RealConstant>);

    pybind11::class_<String> string_class(module,"String");
    string_class.def(pybind11::init<const char*>());
    pybind11::implicitly_convertible<const char*,String>();

    pybind11::class_<StringConstant,pybind11::bases<String>> string_constant_class(module,python_class_name<StringConstant>().c_str());
    string_constant_class.def(pybind11::init<String>());
//    string_constant_class.def(pybind11::init<Identifier,String>());
    string_constant_class.def("name", &StringConstant::name);
    string_constant_class.def("value", &StringConstant::value);
    string_constant_class.def("__str__", &__cstr__<StringConstant>);
    string_constant_class.def("__repr__", &__repr__<StringConstant>);

    template_<Constant> constant_template(module);
    constant_template.instantiate<String>();
    constant_template.instantiate<Real>();
}

pybind11::class_<RealVariable> export_variables(pybind11::module& module)
{
    pybind11::class_<Identifier> identifier_class(module,"Identifier");
    identifier_class.def(pybind11::init<StringType>());
    identifier_class.def("__str__", &__cstr__<Identifier>);
    identifier_class.def("__repr__", &__cstr__<Identifier>);
    pybind11::implicitly_convertible<StringType,Identifier>();

    pybind11::class_<StringVariable> string_variable_class(module,python_class_name<StringVariable>().c_str());
    string_variable_class.def(pybind11::init<StringType>());
    string_variable_class.def("name", &StringVariable::name);
    string_variable_class.def("__str__", &__cstr__<StringVariable>);
    string_variable_class.def("__repr__", &__repr__<StringVariable>);
    string_variable_class.def("__hash__", &__hash__<StringVariable>);
    string_variable_class.def("__eq__", &__eq__<StringVariable,StringType>);
    string_variable_class.def("__ne__", &__ne__<StringVariable,StringType>);

    pybind11::class_<PrimedStringVariable> string_next_variable_class(module,"PrimedStringVariable");
    string_next_variable_class.def("__lshift__", (PrimedStringAssignment(PrimedStringVariable::*)(const StringExpression&)const) &PrimedStringVariable::operator=);
    string_next_variable_class.def("__str__", &__cstr__<PrimedStringVariable>);
    module.def("next", (PrimedStringVariable(*)(const StringVariable&)) &next);


    pybind11::class_<IntegerVariable> integer_variable_class(module,python_class_name<IntegerVariable>().c_str());
    integer_variable_class.def(pybind11::init<StringType>());
    integer_variable_class.def("__str__", &__cstr__<IntegerVariable>);
    integer_variable_class.def("__repr__", &__repr__<IntegerVariable>);

    integer_variable_class.def("__pos__", &__pos__<IntegerVariable>);
    integer_variable_class.def("__neg__", &__neg__<IntegerVariable>);
    integer_variable_class.def("__add__", &__add__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__sub__", &__sub__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__mul__", &__mul__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__radd__", &__radd__<IntegerVariable,Integer>);
    integer_variable_class.def("__rsub__", &__rsub__<IntegerVariable,Integer>);
    integer_variable_class.def("__rmul__", &__rmul__<IntegerVariable,Integer>);
    integer_variable_class.def("__eq__", &__eq__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__ne__", &__ne__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__le__", &__le__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__ge__", &__ge__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__lt__", &__lt__<IntegerVariable,IntegerExpression>);
    integer_variable_class.def("__gt__", &__gt__<IntegerVariable,IntegerExpression>);

    pybind11::class_<LetIntegerVariable> let_integer_variable_class(module,"LetIntegerVariable");
    let_integer_variable_class.def("__lshift__", (IntegerAssignment(LetIntegerVariable::*)(const IntegerExpression&)const) &LetIntegerVariable::operator=);
    module.def("let", (LetIntegerVariable(*)(const IntegerVariable&)) &let);


    pybind11::class_<PrimedIntegerVariable> integer_next_variable_class(module,"PrimedIntegerVariable");
    integer_next_variable_class.def("__lshift__", (PrimedIntegerAssignment(PrimedIntegerVariable::*)(const IntegerExpression&)const) &PrimedIntegerVariable::operator=);
    integer_next_variable_class.def("__str__", &__cstr__<PrimedIntegerVariable>);
    module.def("next", (PrimedIntegerVariable(*)(const IntegerVariable&)) &next);

    pybind11::class_<RealVariable> real_variable_class(module,python_class_name<RealVariable>().c_str());
    real_variable_class.def(pybind11::init<StringType>());
    real_variable_class.def("__str__", &__cstr__<RealVariable>);
    real_variable_class.def("__repr__", &__repr__<RealVariable>);

    real_variable_class.def("__pos__", &__pos__<RealVariable>);
    real_variable_class.def("__neg__", &__neg__<RealVariable>);
    //NOTE The following four are required, and do not dispatch to __xxx___(RealExpression,RealVariable)
    real_variable_class.def("__add__", &__add__<RealVariable,RealExpression>);
    real_variable_class.def("__sub__", &__sub__<RealVariable,RealExpression>);
    real_variable_class.def("__mul__", &__mul__<RealVariable,RealExpression>);
    real_variable_class.def(__py_div__, &__div__<RealVariable,RealExpression>);
    //NOTE The following four are required, and do not dispatch to __rxxx___(RealExpression,RealVariable)
    real_variable_class.def("__radd__", &__radd__<RealVariable,RealExpression>);
    real_variable_class.def("__rsub__", &__rsub__<RealVariable,RealExpression>);
    real_variable_class.def("__rmul__", &__rmul__<RealVariable,RealExpression>);
    real_variable_class.def(__py_rdiv__, &__rdiv__<RealVariable,RealExpression>);
    //NOTE The following are not required, as they dispatch to __rxxx___(RealVariable,RealExpression)
    //real_variable_class.def("__add__", &__add__<RealVariable,RealVariable>);
    //real_variable_class.def("__sub__", &__sub__<RealVariable,RealVariable>);
    //real_variable_class.def("__mul__", &__mul__<RealVariable,RealVariable>);
    //real_variable_class.def(__py_div__, &__div__<RealVariable,RealVariable>);
    //real_variable_class.def("__add__", &__add__<RealVariable,Real>);
    //real_variable_class.def("__sub__", &__sub__<RealVariable,Real>);
    //real_variable_class.def("__mul__", &__mul__<RealVariable,Real>);
    //real_variable_class.def(__py_div__, &__div__<RealVariable,Real>);
    real_variable_class.def("__radd__", &__radd__<RealVariable,Real>);
    real_variable_class.def("__rsub__", &__rsub__<RealVariable,Real>);
    real_variable_class.def("__rmul__", &__rmul__<RealVariable,Real>);
    real_variable_class.def(__py_rdiv__, &__rdiv__<RealVariable,Real>);
    real_variable_class.def("__pow__", &__pow__<RealVariable,Int>);
    //real_variable_class.def("__le__", &__le__<RealVariable,RealExpression>);
    //real_variable_class.def("__ge__", &__ge__<RealVariable,RealExpression>);
    real_variable_class.def("__lt__", &__lt__<RealVariable,RealExpression>);
    real_variable_class.def("__gt__", &__gt__<RealVariable,RealExpression>);

    //real_variable_class.def("__ge__", &__ge__<RealVariable,Real>);

    pybind11::class_<RealVariables> real_variables_class(module,"RealVariables");
    real_variables_class.def(pybind11::init<StringType,SizeType>());
    real_variables_class.def("__getitem__", &__getitem__<RealVariables,SizeType,RealVariable>);
    real_variables_class.def("__str__", &__cstr__<RealVariables>);

    pybind11::class_<LetRealVariable> real_let_variable_class(module,"LetRealVariable");
    real_let_variable_class.def("__lshift__", (RealAssignment(LetRealVariable::*)(const RealExpression&)const) &LetRealVariable::operator=);
    module.def("let", (LetRealVariable(*)(const RealVariable&)) &let);

    pybind11::class_<DottedRealVariable> real_dotted_variable_class(module,"DottedRealVariable");
    real_dotted_variable_class.def("__lshift__", (DottedRealAssignment(DottedRealVariable::*)(const RealExpression&)const) &DottedRealVariable::operator=);
    real_dotted_variable_class.def("__str__", &__cstr__<DottedRealVariable>);
    module.def("dot", (DottedRealVariable(*)(const RealVariable&)) &dot);

    pybind11::class_<PrimedRealVariable> real_next_variable_class(module,"PrimedRealVariable");
    real_next_variable_class.def("__lshift__", (PrimedRealAssignment(PrimedRealVariable::*)(const RealExpression&)const) &PrimedRealVariable::operator=);
    real_next_variable_class.def("__str__", &__cstr__<PrimedRealVariable>);
    module.def("next", (PrimedRealVariable(*)(const RealVariable&)) &next);

    pybind11::class_<TimeVariable,pybind11::bases<RealVariable>> time_variable_class(module,"TimeVariable");
    time_variable_class.def(pybind11::init<>());

    pybind11::class_<BooleanVariable> boolean_variable_class(module,"BooleanVariable");
    boolean_variable_class.def(pybind11::init<StringType>());
    boolean_variable_class.def("__str__",&__cstr__<BooleanVariable>);

    pybind11::class_<KleeneanVariable> kleenean_variable_class(module,"KleeneanVariable");
    kleenean_variable_class.def(pybind11::init<StringType>());
    kleenean_variable_class.def("__str__",&__cstr__<KleeneanVariable>);


    pybind11::class_<RealAssignment> real_assignment_class(module,"RealAssignment");
    real_assignment_class.def("__str__",&__cstr__<RealAssignment>);
    pybind11::class_<DottedRealAssignment> dotted_real_assignment_class(module,"DottedRealAssignment");
    dotted_real_assignment_class.def("__str__",&__cstr__<DottedRealAssignment>);
    pybind11::class_<PrimedRealAssignment> primed_real_assignment_class(module,"PrimedRealAssignment");
    primed_real_assignment_class.def("__str__",&__cstr__<PrimedRealAssignment>);
    pybind11::class_<PrimedStringAssignment> primed_string_assignment_class(module,"PrimedStringAssignment");
    primed_string_assignment_class.def("__str__",&__cstr__<PrimedStringAssignment>);
    pybind11::class_<IntegerAssignment> integer_assignment_class(module,"IntegerAssignment");
    integer_assignment_class.def("__str__",&__cstr__<IntegerAssignment>);
    pybind11::class_<PrimedIntegerAssignment> primed_integer_assignment_class(module,"PrimedIntegerAssignment");
    primed_integer_assignment_class.def("__str__",&__cstr__<PrimedIntegerAssignment>);

    template_<Variable> variable_template(module);
    variable_template.instantiate<Integer>();
    variable_template.instantiate<String>();
    variable_template.instantiate<Real>();
    variable_template.instantiate<Boolean>();
    variable_template.instantiate<Kleenean>();

    return real_variable_class;
}

Void export_valuations(pybind11::module& module)
{
    pybind11::class_<RealValuation> real_valuation_class(module,"RealValuation");
    real_valuation_class.def(pybind11::init<RealValuation>());
    real_valuation_class.def(pybind11::init<std::map<RealVariable,Real,ContainerComparison<RealVariable>>>());
    real_valuation_class.def(pybind11::init<Array<Real>,Space<Real>>());
    //Valuation(const List<Assignment<Variable<T>,X> >& la);
    real_valuation_class.def("insert", &RealValuation::insert);
    real_valuation_class.def("set", &RealValuation::set);
    real_valuation_class.def("get", &RealValuation::get);
    real_valuation_class.def("__getitem__", &RealValuation::get);
    real_valuation_class.def("__setitem__", &RealValuation::set);
    real_valuation_class.def("__repr__", &__cstr__<RealValuation>);

    template_<Valuation> valuation_template(module);
    valuation_template.instantiate<Real>();
}

Void export_expressions(pybind11::module& module)
{
    pybind11::class_<RealSpace> real_space_class(module,"RealSpace");
    real_space_class.def(pybind11::init<RealVariables>());
    real_space_class.def(pybind11::init<std::vector<RealVariable>>());
    real_space_class.def(pybind11::init<RealSpace>());
    real_space_class.def("dimension", &RealSpace::dimension);
    real_space_class.def("variable", &RealSpace::variable);
    real_space_class.def("index", (SizeType(RealSpace::*)(const Identifier&)const) &RealSpace::index);
    real_space_class.def("index", (SizeType(RealSpace::*)(const RealVariable&)const) &RealSpace::index);
    real_space_class.def("__str__", &__cstr__<RealSpace>);

    pybind11::implicitly_convertible<RealVariables,RealSpace>();
    pybind11::implicitly_convertible<std::vector<RealVariable>,RealSpace>();

    pybind11::class_<StringExpression> string_expression_class(module,"StringExpression");
    string_expression_class.def(pybind11::init<StringExpression>());
    string_expression_class.def(pybind11::init<String>());
    string_expression_class.def(pybind11::init<StringVariable>());
    string_expression_class.def("__str__", &__cstr__<StringExpression>);
    string_expression_class.def("__repr__", &__repr__<StringExpression>);
    //NOTE Not in C++ API
    //string_expression_class.def("__eq__", &__eq__<DiscretePredicate,StringExpression,StringExpression>);
    //string_expression_class.def("__ne__", &__ne__<DiscretePredicate,StringExpression,StringExpression>);


    pybind11::class_<IntegerExpression> integer_expression_class(module,"IntegerExpression");
    integer_expression_class.def(pybind11::init<IntegerExpression>());
    integer_expression_class.def(pybind11::init<Integer>());
    integer_expression_class.def(pybind11::init<IntegerVariable>());
    integer_expression_class.def("__str__", &__cstr__<IntegerExpression>);
    integer_expression_class.def("__repr__", &__repr__<IntegerExpression>);
    define_algebra<IntegerExpression,Integer>(module,integer_expression_class);
    integer_expression_class.def("__radd__", &__radd__<IntegerExpression,IntegerExpression>);
    integer_expression_class.def("__rsub__", &__rsub__<IntegerExpression,IntegerExpression>);
    integer_expression_class.def("__rmul__", &__rmul__<IntegerExpression,IntegerExpression>);

    integer_expression_class.def("__eq__", &__eq__<IntegerExpression,IntegerExpression>);
    integer_expression_class.def("__ne__", &__ne__<IntegerExpression,IntegerExpression>);
    integer_expression_class.def("__le__", &__le__<IntegerExpression,IntegerExpression>);
    integer_expression_class.def("__ge__", &__ge__<IntegerExpression,IntegerExpression>);
    integer_expression_class.def("__lt__", &__lt__<IntegerExpression,IntegerExpression>);
    integer_expression_class.def("__gt__", &__gt__<IntegerExpression,IntegerExpression>);


    pybind11::class_<RealExpression> real_expression_class(module,"RealExpression");
    real_expression_class.def(pybind11::init<RealExpression>());
    real_expression_class.def(pybind11::init<Real>());
    real_expression_class.def(pybind11::init<RealVariable>());
    module.def("simplify", (RealExpression(*)(const RealExpression&)) &simplify);

    define_elementary_algebra<RealExpression,Real>(module,real_expression_class);

    real_expression_class.def("__radd__", &__radd__<RealExpression,RealExpression>);
    real_expression_class.def("__rsub__", &__rsub__<RealExpression,RealExpression>);
    real_expression_class.def("__rmul__", &__rmul__<RealExpression,RealExpression>);
    real_expression_class.def(__py_rdiv__, &__rdiv__<RealExpression,RealExpression>);
    real_expression_class.def("__pow__", &__pow__<RealExpression,Int>);
    real_expression_class.def("__le__", &__le__<RealExpression,RealExpression>);
    real_expression_class.def("__ge__", &__ge__<RealExpression,RealExpression>);
    real_expression_class.def("__lt__", &__lt__<RealExpression,RealExpression>);
    real_expression_class.def("__gt__", &__gt__<RealExpression,RealExpression>);
    //real_expression_class.def("__cmp__", &__cmp__<RealExpression,RealExpression , Return<ContinuousPredicate> >);
    real_expression_class.def("__str__", &__cstr__<RealExpression>);
    real_expression_class.def("__repr__", &__repr__<RealExpression>);

    real_expression_class.def("__le__", &__le__<RealExpression,Real>);
    real_expression_class.def("__ge__", &__ge__<RealExpression,Real>);
    real_expression_class.def("__le__", &__le__<Real,RealExpression>);
    real_expression_class.def("__ge__", &__ge__<Real,RealExpression>);


    module.def("max", &_max_<RealExpression,RealExpression>);
    module.def("min", &_min_<RealExpression,RealExpression>);
    module.def("abs", &_abs_<RealExpression>);

    module.def("evaluate", (Real(*)(RealExpression const&, RealValuation const&)) &evaluate);

    module.def("substitute",(RealExpression(*)(RealExpression const&, const List<Assignment<RealVariable,RealExpression>>&)) &substitute);
    module.def("substitute",(RealExpression(*)(RealExpression const&, const Map<RealVariable,RealExpression>&)) &substitute);
    module.def("substitute",(RealExpression(*)(RealExpression const&, const Map<RealVariable,Real>&)) &substitute);

    module.def("derivative",(RealExpression(*)(RealExpression const&, const RealVariable)) &derivative);

    auto real_expression_vector_class=export_vector<RealExpression>(module, "RealExpressionVector");
    real_expression_vector_class.def(pybind11::init([](pybind11::list const& lst){return Vector<RealExpression>(pybind11::cast<List<RealExpression>>(lst));}));
    pybind11::implicitly_convertible<pybind11::list,Vector<RealExpression>>();
    // Would ideally like to convert from a List<Expression>, but pybind11 does not allow conversion from e.g. a [Variable,Expression]
    // real_expression_vector_class.def(pybind11::init<List<RealExpression>>());
    // pybind11::implicitly_convertible<List<RealExpression>,Vector<RealExpression>>();


    pybind11::class_<DiscretePredicate> discrete_predicate_class(module,"DiscretePredicate");
    discrete_predicate_class.def(pybind11::init<DiscretePredicate>());
    discrete_predicate_class.def(pybind11::init<Bool>());
    discrete_predicate_class.def(pybind11::init<BooleanVariable>());
    discrete_predicate_class.def("__and__", &__and__<DiscretePredicate,DiscretePredicate>);
    discrete_predicate_class.def("__or__", &__or__<DiscretePredicate,DiscretePredicate>);
    discrete_predicate_class.def("__invert__", &__not__<DiscretePredicate>);
    discrete_predicate_class.def("__str__",&__cstr__<DiscretePredicate>);

    pybind11::class_<ContinuousPredicate> continuous_predicate_class(module,"ContinuousPredicate");
    continuous_predicate_class.def(pybind11::init<ContinuousPredicate>());
    continuous_predicate_class.def(pybind11::init<Kleenean>());
    continuous_predicate_class.def(pybind11::init<KleeneanVariable>());
    continuous_predicate_class.def(pybind11::init<RealVariableLowerInterval>());
    continuous_predicate_class.def(pybind11::init<RealVariableUpperInterval>());
    continuous_predicate_class.def("__and__", &__and__<ContinuousPredicate,ContinuousPredicate>);
    continuous_predicate_class.def("__or__", &__or__<ContinuousPredicate,ContinuousPredicate>);
    continuous_predicate_class.def("__invert__", &__not__<ContinuousPredicate>);
    continuous_predicate_class.def("__str__",&__cstr__<ContinuousPredicate>);

    module.def("make_function", (EffectiveScalarUnivariateFunction(*)(RealVariable const&, RealExpression const&)) &make_function);
    module.def("make_function", (EffectiveVectorUnivariateFunction(*)(RealVariable const&, Vector<RealExpression> const&)) &make_function);
    module.def("make_function", (EffectiveScalarMultivariateFunction(*)(RealSpace const&, RealExpression const&)) &make_function);
    module.def("make_function", (EffectiveVectorMultivariateFunction(*)(RealSpace const&, Vector<RealExpression> const&)) &make_function);

    pybind11::implicitly_convertible<StringVariable,StringExpression>();
    pybind11::implicitly_convertible<IntegerVariable,IntegerExpression>();
    pybind11::implicitly_convertible<RealVariable,RealExpression>();
    pybind11::implicitly_convertible<BooleanVariable,DiscretePredicate>();
    pybind11::implicitly_convertible<KleeneanVariable,ContinuousPredicate>();

    pybind11::implicitly_convertible<RealVariableLowerInterval,ContinuousPredicate>();
    pybind11::implicitly_convertible<RealVariableUpperInterval,ContinuousPredicate>();

    module.def("sgn", &_sgn_<RealExpression>);


    pybind11::implicitly_convertible<std::string,StringExpression>();
    pybind11::implicitly_convertible<String,StringExpression>();
    pybind11::implicitly_convertible<StringVariable,StringExpression>();

    pybind11::implicitly_convertible<int,IntegerExpression>();
    pybind11::implicitly_convertible<Integer,IntegerExpression>();
    pybind11::implicitly_convertible<IntegerVariable,IntegerExpression>();

    pybind11::implicitly_convertible<Int,RealExpression>();
    pybind11::implicitly_convertible<Integer,RealExpression>();
    pybind11::implicitly_convertible<Decimal,RealExpression>();
    pybind11::implicitly_convertible<Dyadic,RealExpression>();
    pybind11::implicitly_convertible<Rational,RealExpression>();
    pybind11::implicitly_convertible<Real,RealExpression>();
    pybind11::implicitly_convertible<RealVariable,RealExpression>();

    template_<Expression> expression_template(module);
    expression_template.instantiate<Integer>();
    expression_template.instantiate<String>();
    expression_template.instantiate<Real>();
    expression_template.instantiate<Boolean>();
    expression_template.instantiate<Kleenean>();
}


Void export_sets(pybind11::module& module, pybind11::class_<RealVariable>& real_variable_class)
{
    pybind11::class_<RealVariableLowerInterval> real_variable_lower_interval_class(module,"RealVariableLowerInterval");
    real_variable_lower_interval_class.def(pybind11::init<Real,RealVariable>());
    real_variable_lower_interval_class.def("__str__",&__cstr__<RealVariableLowerInterval>);
    real_variable_lower_interval_class.def("__bool__", [](RealVariableLowerInterval const&){ARIADNE_THROW(std::runtime_error,"RealVariableLowerInterval.__bool__(self)","Cannot convert RealVariableLowerInterval to bool");});

    pybind11::class_<RealVariableUpperInterval> real_variable_upper_interval_class(module,"RealVariableUpperInterval");
    real_variable_upper_interval_class.def(pybind11::init<RealVariable,Real>());
    real_variable_upper_interval_class.def("__str__",&__cstr__<RealVariableUpperInterval>);
    real_variable_upper_interval_class.def("__bool__", [](RealVariableUpperInterval const&){ARIADNE_THROW(std::runtime_error,"RealVariableUpperInterval.__bool__(self)","Cannot convert RealVariableUpperInterval to bool");});

    real_variable_class.def("__le__", &__le__<Real,RealVariable , Return<RealVariableLowerInterval> >);
    real_variable_class.def("__ge__", &__ge__<RealVariable,Real , Return<RealVariableLowerInterval> >);
    real_variable_class.def("__le__", &__le__<RealVariable,Real , Return<RealVariableUpperInterval> >);
    real_variable_class.def("__or__", [](RealVariable const& x, RealInterval ivl){return RealVariableInterval(x,ivl);});
    real_variable_class.def("__lshift__",&__eq__<RealVariable,Real,Return<RealVariableInterval>>);

    real_variable_lower_interval_class.def("__le__", &__le__<RealVariableLowerInterval,Real , Return<RealVariableInterval> >);
//    real_variable_upper_interval_class.def("__le__", &__le__<Real,RealVariableUpperInterval , Return<RealVariableInterval> >);
    real_variable_upper_interval_class.def("__ge__", &__ge__<RealVariableUpperInterval,Real , Return<RealVariableInterval> >);


    // NOTE: This syntax allows creating a variable-interval using 'l<=x & x<=u'
    // It does not allow allows 'l<=x<=u' to define a variable-interval
    // since the Python short-circuit converts to  '(l<=x) and (x<=u)'
    real_variable_lower_interval_class.def("__and__",
        [] (RealVariableLowerInterval livl,RealVariableUpperInterval uivl) {
            assert(livl.variable()==uivl.variable());
            return RealVariableInterval(livl.lower_bound(),livl.variable(),uivl.upper_bound());});


    pybind11::class_<RealVariableInterval> real_variable_interval_class(module,"RealVariableInterval");
    real_variable_interval_class.def(pybind11::init<RealVariable,RealInterval>());
    real_variable_interval_class.def(pybind11::init<Real,RealVariable,Real>());
    real_variable_interval_class.def("variable", &RealVariableInterval::variable);
    real_variable_interval_class.def("interval", &RealVariableInterval::interval);
    real_variable_interval_class.def("lower", &RealVariableInterval::lower_bound);
    real_variable_interval_class.def("upper", &RealVariableInterval::upper_bound);
    real_variable_interval_class.def("__str__",&__cstr__<RealVariableInterval>);

    pybind11::class_<RealVariablesBox> real_variables_box_class(module,"RealVariablesBox");
    real_variables_box_class.def(pybind11::init([](pybind11::dict dct){return RealVariablesBox(pybind11::cast<Map<RealVariable,RealInterval>>(dct));}));
    real_variables_box_class.def(pybind11::init<Map<RealVariable,RealInterval>>());
    real_variables_box_class.def(pybind11::init<List<RealVariableInterval>>());
    real_variables_box_class.def("__getitem__", &RealVariablesBox::operator[]);
    real_variables_box_class.def("__str__",&__cstr__<RealVariablesBox>);
    pybind11::implicitly_convertible<pybind11::dict,RealVariablesBox>();

    typedef VariablesBox<FloatDPUpperInterval> FloatDPVariablesBox;
    pybind11::class_<FloatDPVariablesBox> floatdp_variables_box_class(module,"FloatDPVariablesBox");
    floatdp_variables_box_class.def("__getitem__", &FloatDPVariablesBox::operator[]);
    floatdp_variables_box_class.def("__str__",&__cstr__<FloatDPVariablesBox>);

    pybind11::class_<LabelledBox<FloatDPUpperInterval>> labelled_floatdp_upper_box_class(module,"LabelledFloatDPUpperBox");
    labelled_floatdp_upper_box_class.def("__getitem__", &LabelledBox<FloatDPUpperInterval>::operator[]);
    labelled_floatdp_upper_box_class.def("__str__",&__cstr__<LabelledBox<FloatDPUpperInterval>>);

    pybind11::class_<RealExpressionConstraintSet> real_expression_constraint_set_class(module,"RealExpressionConstraintSet");
    real_expression_constraint_set_class.def(pybind11::init<List<ContinuousPredicate>>());
    real_expression_constraint_set_class.def("__str__",&__cstr__<RealExpressionConstraintSet>);

    pybind11::class_<RealExpressionBoundedConstraintSet> real_expression_bounded_constraint_set_class(module,"RealExpressionBoundedConstraintSet");
    real_expression_bounded_constraint_set_class.def(pybind11::init<List<RealVariableInterval>,List<ContinuousPredicate>>());
    real_expression_bounded_constraint_set_class.def(pybind11::init<RealVariablesBox,RealExpressionConstraintSet>());
    real_expression_bounded_constraint_set_class.def("euclidean_set", (BoundedConstraintSet(RealExpressionBoundedConstraintSet::*)(RealSpace const&)) &RealExpressionBoundedConstraintSet::euclidean_set);
    real_expression_bounded_constraint_set_class.def("__str__",&__cstr__<RealExpressionBoundedConstraintSet>);

    module.def("intersection", [](RealVariablesBox const& bx, RealExpressionConstraintSet const& cs){return intersection(bx,cs);});

    module.def("make_box", (RealBox(*)(RealSpace const&, RealVariablesBox const&)) &make_box);
    module.def("make_set", (RealBox(*)(RealSpace const&, RealVariablesBox const&)) &make_box);
    module.def("make_set", (ConstraintSet(*)(RealSpace const&, RealExpressionConstraintSet const&)) &make_set);
    module.def("make_set", (BoundedConstraintSet(*)(RealSpace const&, RealExpressionBoundedConstraintSet const&)) &make_set);
    module.def("make_set", (BoundedConstraintSet(*)(RealSpace const&, RealVariablesBox const&, RealExpressionConstraintSet const&)) &make_set);

    pybind11::implicitly_convertible<Pair<Real,Real>,RealInterval>();
    pybind11::implicitly_convertible<std::map<RealVariable,RealInterval>,RealVariablesBox>();
//    pybind11::implicitly_convertible<List<RealVariableInterval>,RealVariablesBox>();
    pybind11::implicitly_convertible<List<ContinuousPredicate>,RealExpressionConstraintSet>();
}


Void symbolic_submodule(pybind11::module& module) {
    export_constants(module);
    auto real_variable_class=export_variables(module);
    export_valuations(module);
    export_expressions(module);
    export_sets(module,real_variable_class);
}

