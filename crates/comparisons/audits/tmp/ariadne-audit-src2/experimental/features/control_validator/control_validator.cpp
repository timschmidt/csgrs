/***************************************************************************
 *            control_validator.cpp
 *
 *  Copyright  2015 Pieter Collins
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

#include "function/functional.hpp"

#include <type_traits>

#include "utility/declarations.hpp"
#include "symbolic/variable.hpp"

namespace Ariadne {
template<class X> class Vector;
template<> struct Vector<RealVariable> : List<RealVariable> {
    using List<RealVariable>::List;
    template<class... ARGS> Vector<RealVariable>(ARGS... args);
};
} // namespace Ariadne

#include "symbolic/expression.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/expression_set.hpp"
#include "symbolic/function_expression.hpp"


#include "function/function.hpp"
#include "function/scaled_function_patch.hpp"
#include "function/taylor_function.hpp"


#define ARIADNE_PRINT(expression) std::cout << #expression << "=" << (expression) << "\n"

namespace Ariadne {


template<class T> Set<T> join(Set<T> s1, Set<T> const& s2, Set<T> const& s3) {
    s1.adjoin(s2); s1.adjoin(s3); return std::move(s1);
}

template<class T> Set<T> join(Set<T> s1, Set<T> const& s2, Set<T> const& s3, Set<T> const& s4) {
    s1.adjoin(s2); s1.adjoin(s3); s1.adjoin(s4); return std::move(s1);
}

template<class T> List<T> join(List<T> l1, List<T> const& l2) {
    l1.append(l2); return std::move(l1);
}

template<class T> List<T> join(List<T> l1, List<T> const& l2, List<T> const& l3) {
    l1.append(l2); l1.append(l3); return std::move(l1);
}

template<class T> Bool subset(Set<T> const& s, List<T> const& l) {
    return subset(s,Set<T>(l)); }
template<class T> Bool subset(List<T> const& l, Set<T> const& s) {
    for(auto t:l) { if(!s.contains(t)) { return false; } } return true; }

using FloatType = FloatDP;
using ExactFloatInterval = FloatDPExactInterval;
using ExactFloatBox = FloatDPExactBox;
using ExactFloatVariablesBox = ExactFloatDPVariablesBox;

template<class T> using remove_cv_t = typename std::remove_cv<T>::type;
template<class T> using remove_reference_t = typename std::remove_reference<T>::type;
template<class T> using remove_const_reference_t = typename std::remove_const<typename std::remove_reference<T>::type>::type;
template<class F, class... AS> using invoke_result_t = typename std::invoke_result<F,AS...>::type;
template<class It> using dereference_t = decltype(*declval<It>);

template<class InputIt, class UnaryPredicate> auto
quasi_all_of(InputIt first, InputIt last, UnaryPredicate p) -> invoke_result_t<UnaryPredicate,dereference_t<InputIt>> {
    decltype(p(*first)) r=true; while(first!=last) { r=r && p(*first); if(not possibly(r)) { return r; } ++first; }  return r; }
template<class InputIt, class UnaryPredicate> auto quasi_any_of(InputIt first, InputIt last, UnaryPredicate p) -> decltype(p(*first)) {
    decltype(p(*first)) r=false; while(first!=last) { r=r || p(*first); if(definitely(r)) { return r; } ++first; }  return r; }


typedef Assignment<RealVariable,ValidatedScalarFunctionExpression> ValidatedScalarAssignment;
typedef Assignment<Vector<RealVariable>,ValidatedVectorFunctionExpression> ValidatedVectorAssignment;

template<class T> using RealValuation = Map<RealVariable,T>;

template<class P> VectorFunction<P> make_function(RealSpace const& spc, VectorFunctionExpression<P> const& expr) {
    typedef remove_const_reference_t<decltype(expr.domain()[spc[0]])> IntervalType;
    Box<IntervalType> dom(spc.size(),IntervalType(-infty,+infty));
    for(SizeType i=0; i!=spc.size(); ++i) {
        dom[i]=intersection(dom[i],expr.domain()[spc[i]]);
    }
    std::cerr << "dom="<<dom<<"\n";
}

RealVectorFunction make_function(List<RealVariable> const& vars, List<DottedRealAssignment> const& dyn) {
    List<RealScalarFunction> lsf;
    for(SizeType i=0; i!=dyn.size(); ++i) {
        lsf.append(make_function(dyn[i].rhs,vars));
    }
    return lsf;
}



template<class P> VectorFunction<P> compose(Vector<RealVariable> const& s, RealValuation<ScalarFunction<P>> const& v) {
    List<ValidatedScalarFunction> r;
    for(SizeType i=0; i!=s.size(); ++i) {
        r.append(v[s[i]]);
    }
    return ValidatedVectorFunction(std::move(r));
}

template<class P> ScalarFunction<P> compose(ScalarFunctionExpression<P> const& e, RealValuation<ScalarFunction<P>> const& v) {
    return compose(e.function(), compose(e.variables(),v));
}

template<class P> VectorFunction<P> compose(VectorFunctionExpression<P> const& e, RealValuation<ScalarFunction<P>> const& v) {
    return compose(e.function(), compose(e.variables(),v));
}


Set<RealVariable> arguments(RealExpression const& expr) {
    Set<UntypedVariable> args=expr.arguments();
    Set<RealVariable> real_args;
    for(auto arg:args) { real_args.insert(RealVariable(arg.name())); }
    return std::move(real_args);
}

template<class P> Set<RealVariable> arguments(ScalarFunctionExpression<P> const& expr) {
    return expr.arguments();
}

template<class LHS, class RHS> Set<RealVariable> arguments(Assignment<LHS,RHS> const& assignment) {
    return arguments(assignment.rhs); }

template<class LHS, class RHS> Set<RealVariable> arguments(List<Assignment<LHS,RHS>> const& assignments) {
    Set<RealVariable> res; inplace_accumulate(assignments.begin(),assignments.end(),res,[](Set<RealVariable>& s, Assignment<LHS,RHS>const& a) { s.adjoin(arguments(a.rhs)); }); return res; }

template<class LHS, class RHS> List<RealVariable> results(List<Assignment<LHS,RHS>> const& assignments) {
    List<RealVariable> res; for(SizeType i=0; i!=assignments.size(); ++i) { res[i]=assignments[i].lhs; } return res; }


template<class InputIterator, class Output, class BinaryOperation>
void inplace_accumulate(InputIterator first, InputIterator last, Output& init, BinaryOperation op) {
    while(first!=last) { op(init,*first); ++first; } }

template<class OutputContainer, class InputIterator, class UnaryOperation>
OutputContainer transform_create(InputIterator first, InputIterator last, UnaryOperation unary_op) {
    OutputContainer c; inplace_accumulate(first,last,c,[=](OutputContainer& c, decltype(*first) t){c.append(unary_op(t));}); return std::move(c); }
//    OutputContainer result; while(first!=last) { result.push_back(unary_op(*first)); ++first; } return std::move(result); }

template<class T> auto left_hand_side(T const& l) -> decltype(declval<T>().left_hand_side()) {
    l.left_hand_side(); }

template<template<typename> class L, class T> auto left_hand_sides(L<T> const& l) -> L<decltype(declval<T>().left_hand_side())> {
    typedef decltype(declval<T>().left_hand_side()) R; return transform_create<L<R>>(l.begin(),l.end(),[](T t){return t.left_hand_side();}); }

template<template<typename> class L, class T> auto variables(L<T> const& l) -> L<remove_const_reference_t<decltype(declval<T>().variable())>> {
    typedef remove_const_reference_t<decltype(declval<T>().variable())> R; return transform_create<L<R>>(l.begin(),l.end(),[](T t){return t.variable();}); }


class ControlSystem {
  private:
  public:
    List<DottedRealAssignment> _dynamics;
    List<RealVariableInterval> _parameter_ranges;
  public:
    template<class... RULES> ControlSystem(RULES... rules) { _apply(rules...); }
    List<DottedRealAssignment> dynamics() const { return this->_dynamics; }
    List<RealVariableInterval> parameter_ranges() const { return this->_parameter_ranges; }
    List<RealVariable> state_variables() const { return left_hand_sides(_dynamics); }
    List<RealVariable> parameters() const { return variables(this->_parameter_ranges); }
    RealInterval parameter_range(RealVariable v) const {
        for(SizeType i=0; i!=_parameter_ranges.size(); ++i) { if(this->_parameter_ranges[i].variable()==v) { return this->_parameter_ranges[i].interval(); } } }
  private:
    void _apply() { }
    void _apply(DottedRealAssignment rule) { _dynamics.append(rule); }
    void _apply(RealVariableInterval rule) { _parameter_ranges.append(rule); }
    template<class RULE, class... RULES> void _apply(RULE rule, RULES... rules) { _apply(rule), _apply(rules...); }
  private:
    friend OutputStream& operator<<(OutputStream& os, ControlSystem const& cs) {
        os << "ControlSystem( dynamics="<< cs._dynamics << ", parameter_ranges=" << cs._parameter_ranges << " )"; }
};

ValidatedVectorFunctionModelDP join(ValidatedVectorFunctionModelDP vf1, ValidatedVectorFunctionModelDP vf2, ValidatedVectorFunctionModelDP vf3) {
    return join(join(vf1,vf2),vf3);
}

ValidatedVectorFunctionModelDP join(ValidatedVectorFunctionModelDP vf1, ValidatedVectorFunctionModelDP vf2, ValidatedScalarFunctionModelDP sf3) {
    return join(join(vf1,vf2),sf3);
}


ValidatedVectorFunctionModelDP antiderivative(ValidatedVectorFunction const& vf, SizeType k, ValidatedNumericType a) {
    auto vfp = std::dynamic_pointer_cast<ValidatedVectorFunctionModelDPInterface const>(vf.managed_pointer());
    if(vfp) { return antiderivative(ValidatedVectorFunctionModelDP(vfp->_clone()),k,a); }
    std::cerr<<"\n\nvf="<<vf<<"\n\n\n"; assert(false);
}

ValidatedVectorFunction operator+(ValidatedVectorFunction const& vf1, ValidatedVectorFunction const& vf2) {
    auto vfp1 = std::dynamic_pointer_cast<ValidatedVectorFunctionModelDPInterface const>(vf1.managed_pointer());
    auto vfp2 = std::dynamic_pointer_cast<ValidatedVectorFunctionModelDPInterface const>(vf1.managed_pointer());
    if(vfp1 && vfp2) { return ValidatedVectorFunctionModelDP(vfp1->_clone())+ValidatedVectorFunctionModelDP(vfp2->_clone()); }
    std::cerr<<"vf1="<<vf1<<"\n"; std::cerr<<"vf2="<<vf2<<"\n"; assert(false);
}

// Solve dot(x) = f(a,x,u) with x(0)=x0 and u=mu(a,u0,t) on a given domain
// The result is a function phi(a,x0,u0,t)
ValidatedVectorFunctionModelDP
flow_step(const ValidatedVectorFunction& control_system,
          const ExactBoxType& parameter_domain,
          const ExactBoxType& state_domain,
          const FloatType& step_size,
          const ValidatedVectorFunctionModelDP& inputs);


// Solve dot(x) = f(a,x,u) with x(0)=x0 and u=mu(a,u0,t) on a given domain
// The result is a function phi(a,x0,u0,t)
//ValidatedVectorFunctionExpression
void
control_flow_step(const ControlSystem& control_system,
          const ExactVariablesBoxType& parameter_domain,
          const ExactVariablesBoxType& state_domain,
          const FloatType& step_size,
          const List<ValidatedScalarAssignment>& inputs)
{
    ValidatedScalarAssignment input = inputs[0];

    ARIADNE_PRINT(control_system);
    ARIADNE_PRINT(parameter_domain);
    ARIADNE_PRINT(state_domain);
    ARIADNE_PRINT(step_size);
    ARIADNE_PRINT(inputs);
    std::cout<<std::endl;

    auto dynamic = control_system.dynamics();
    auto parameter_ranges = control_system.parameter_ranges();

    Set<RealVariable> dynamic_arguments=arguments(control_system.dynamics());
    List<RealVariable> parameters=control_system.parameters();
    List<RealVariable> state_variables=control_system.state_variables();
    List<RealVariable> input_variables={left_hand_side(input)};
    Set<RealVariable> input_arguments=arguments(input);

    ARIADNE_PRINT(dynamic_arguments);
    ARIADNE_PRINT(parameters);
    ARIADNE_PRINT(state_variables);
    ARIADNE_PRINT(input_variables);
    ARIADNE_PRINT(input_arguments);
    std::cout<<std::endl;

    ValidatedScalarFunctionExpression input_expression = input.rhs;
    RealSpace input_space=input_expression.variables();
    ARIADNE_PRINT(input_space);
    ExactVariablesBoxType input_domain = input_expression.domain();
    ARIADNE_PRINT(input_domain.bounds());

    RealSpace flow_space = input_space;
    ARIADNE_PRINT(flow_space);

    ValidatedScalarFunction input_function = input_expression.function();
    ARIADNE_PRINT(input_function);
    ExactBoxType input_function_domain = input_function.domain();
    ARIADNE_PRINT(input_function_domain);

    ARIADNE_PRINT(typeid(input_function.reference()).name());
    ARIADNE_PRINT(typeid(input_function.raw_pointer()).name());
    ValidatedScalarFunctionModelDP input_function_model(dynamic_cast<ValidatedScalarFunctionModelDPInterface const&>(input_function.reference()));
    ARIADNE_PRINT(input_function_model);

    ValidatedScalarFunctionModelDP zero=input_function_model.create_zero();
    ValidatedVectorFunctionModelDP identity=input_function_model.create_identity();
    ARIADNE_PRINT(identity);
    Map<RealVariable,ValidatedScalarFunctionModelDP> coordinates;
    for(SizeType i=0; i!=flow_space.dimension(); ++i) { coordinates[flow_space[i]]=identity[i]; }
    ARIADNE_PRINT(coordinates);
    std::cout<<std::endl;

    List<ValidatedScalarFunctionModelDP> parameter_functions;
    for(SizeType i=0; i!=parameters.size(); ++i) {
        parameter_functions.append(coordinates[parameters[i]]); }
    List<ValidatedScalarFunctionModelDP> initial_state_functions;
    for(SizeType i=0; i!=state_variables.size(); ++i) {
        initial_state_functions.append(coordinates[state_variables[i]]); }

    ARIADNE_PRINT(parameter_functions);
    ARIADNE_PRINT(initial_state_functions);
    ARIADNE_PRINT(input_function);

    //ValidatedVectorFunction dynamic_function(dynamic.size(),parameters.size()+state_variables.size()+input_variables.size());
    auto dynamic_space = join(state_variables,parameters,input_variables);
    ARIADNE_PRINT(dynamic_space);
    auto dynamic_function = make_function(dynamic_space,dynamic);
    ARIADNE_PRINT(dynamic_function);

    ARIADNE_ASSERT(subset(dynamic_arguments,dynamic_space));
    ARIADNE_ASSERT(subset(join(parameters,state_variables),input_arguments));

    ValidatedVectorFunctionModelDP parameter_function ( parameter_functions );
    ValidatedVectorFunctionModelDP initial_state_function = initial_state_functions;

    ValidatedVectorFunctionModelDP state_function = initial_state_function;
    ValidatedVectorFunctionModelDP old_state_function = initial_state_function;

    TimeVariable t;
    SizeType time_index = input_space.index(t);

    for(Nat i=0; i!=12; ++i) {
        state_function.clobber();
        ValidatedVectorFunctionModelDP dynamic_flow_function =
            compose(dynamic_function,ValidatedVectorFunctionModelDP(join(state_function,parameter_function,input_function)));
        //ARIADNE_PRINT(dynamic_flow_function);

        //ValidatedVectorFunctionModelDP antiderivative_dynamic_flow_function = antiderivative(dynamic_flow_function,time_index);
        ValidatedVectorFunctionModelDP antiderivative_dynamic_flow_function = antiderivative(dynamic_flow_function,time_index,FloatDP(0));
        //ARIADNE_PRINT(antiderivative_dynamic_flow_function);

        old_state_function = state_function;
        state_function = initial_state_function + antiderivative_dynamic_flow_function;
        ARIADNE_PRINT(norm(state_function-old_state_function));

        std::cout << std::endl;
    }

    ARIADNE_PRINT(state_function);

}

static_assert(IsBaseOf<List<RealVariable>,RealVariables>::value,"");

template<class T, class... ARGS> List<T> make_list(List<T> l1, SelfType<T> t2, ARGS... args);
template<class T, class... ARGS> List<T> make_list(List<T> l1, List<SelfType<T>> const& l2, ARGS... args);

template<class T> List<T> make_list(List<T> lst) { return std::move(lst); }
template<class T, class... ARGS> inline List<T> make_list(List<T> l1, SelfType<T> t2, ARGS... as3) {
    l1.append(t2); return make_list(std::move(l1),as3...); }
template<class T, class... ARGS> inline List<T> make_list(List<T> l1, List<SelfType<T>> const& l2, ARGS... as3) {
    l1.append(l2); return make_list(std::move(l1),as3...); }

template<class... ARGS> List<RealVariable> make_list(RealVariable v, ARGS... args) { return make_list(List<RealVariable>(1u,v),args...); }

template<class T> template<class... ARGS> Space<T>::Space(VariableType var, ARGS... args) : Space(make_list(var, args...)) { }
template<class T> template<class... ARGS> Space<T>::Space(List<VariableType> vars, VariableType var, ARGS... args) : Space(make_list(vars, var, args...)) { }
template<class T> template<class... ARGS> Space<T>::Space(List<VariableType> vars1, List<VariableType> vars2, ARGS... args) : Space(make_list(vars1, vars2,args...)) { }

//template<class... ARGS> Vector<RealVariable>::Vector(ARGS... args) : Vector(make_list(args...)) { }
template<class... ARGS> Vector<RealVariable>::Vector(ARGS... args) { static_cast<List<RealVariable>&>(*this)=make_list(args...); }


} // namespace Ariadne

using namespace Ariadne;


int main() {

    TimeVariable t;
    RealVariable a("a");
    RealVariable b("b");
    RealVariable x("x");
    RealVariable y("y");
    RealVariable u("u");

    ExactFloatBox domain = {{0.5,1.5},{1,2},{-0.5,0.5},{0,1},{0,0.125}};
    ThresholdSweeper sweeper(1e-12);
    //ValidatedVectorFunction idv=ValidatedVectorFunction::identity(domain);
    ValidatedVectorTaylorFunctionModelDP idv=ValidatedVectorTaylorFunctionModelDP::identity(domain,sweeper);
    Array<ValidatedScalarTaylorFunctionModelDP> id={idv[0],idv[1],idv[2],idv[3],idv[4]};
    ControlSystem system={ dot(x)=-a*x+u, a.in(1,4) };
    ExactVariablesBoxType parameter_domain={a.in(2,3)};
    ExactVariablesBoxType state_domain={x.in(0,1)};
    FloatType step_size=0.5_exact;
    ARIADNE_PRINT(step_size);
    ValidatedScalarFunctionModelDP ufm=id[3]*exp(-id[1]*id[4]);
    ARIADNE_PRINT(ufm.range());
    ValidatedScalarFunction uf=ufm;
    ARIADNE_PRINT(uf);
    ValidatedScalarFunctionExpression ufe=evaluate(uf,{a,b,x,y,t});
    List<ValidatedScalarAssignment> inputs={{u,ufe}};
    std::cout<<std::endl;
    control_flow_step(system,parameter_domain,state_domain,step_size,inputs);


}
