/***************************************************************************
 *            dynamics/differential_inclusion.cpp
 *
 *  Copyright  2008-20  Luca Geretti
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

#include "differential_inclusion.hpp"
#include "symbolic/expression_set.hpp"
#include "function/symbolic_function.hpp"

namespace Ariadne {

List<Nat> input_indices(SizeType num_variables, SizeType num_inputs) {
    List<Nat> result;
    for (SizeType i : range(num_variables,num_variables+num_inputs)) { result.append(i); }
    return result;
}

BoxDomainType input_bounds_to_domain(RealVariablesBox const& inputs) {
    List<IntervalDomainType> result;

    auto vars = inputs.variables();
    for (auto v : vars) {
        result.push_back(IntervalDomainType(cast_exact(inputs[v].lower_bound().get(dp)),cast_exact(inputs[v].upper_bound().get(dp))));
    }
    return Vector<IntervalDomainType>(result);
}

Pair<CoordinateFormulaPair,ExactIntervalType> centered_coordinate_transformation(Nat const& i, ExactIntervalType const& bounds) {
    if (bounds.lower_bound()==-bounds.upper_bound()) { return Pair<CoordinateFormulaPair,ExactIntervalType>({i,EffectiveFormula::coordinate(i)},bounds); }
    else { return Pair<CoordinateFormulaPair,ExactIntervalType>({i,EffectiveFormula::coordinate(i)+EffectiveFormula::constant(EffectiveNumber(bounds.midpoint()))},ExactIntervalType(cast_exact(bounds.lower_bound()-bounds.midpoint()),cast_exact(bounds.upper_bound()-bounds.midpoint()))); }
}


Pair<CoordinateFormulaPairs,BoxDomainType> centered_coordinates_transformation(Nat const& dim, BoxDomainType const& inputs) {
    CoordinateFormulaPairs assignments;
    BoxDomainType new_bounds(inputs.size());
    for (auto i : range(inputs.size())) {
        auto tr = centered_coordinate_transformation(i+dim,inputs[i]);
        assignments.push_back(tr.first);
        new_bounds[i] = tr.second;
    }
    return Pair<CoordinateFormulaPairs,BoxDomainType>(assignments,new_bounds);
}

Void incorporate_additive_inputs_coefficients(Vector<EffectiveFormula>& transformed_formulae, BoxDomainType& transformed_inputs) {

    SizeType n = transformed_formulae.size();
    SizeType m = transformed_inputs.size();
    for (SizeType j : range(m)) {
        for (SizeType i : range(n)) {
            EffectiveFormula der = simplify(derivative(transformed_formulae[i],n+j));
            if (not identical(der,EffectiveFormula::zero()) and not identical(der,EffectiveFormula::constant(1))) {
                transformed_inputs[j] = cast_exact(transformed_inputs[j] * der.val());
                transformed_formulae[i] = simplify(substitute(transformed_formulae[i],n+j,EffectiveFormula::zero())+EffectiveFormula::coordinate(n+j));
            }
        }
    }
}

Void DifferentialInclusion::_transform_and_assign(EffectiveVectorMultivariateFunction const& function, BoxDomainType const& inputs) {

    const EffectiveVectorFormulaFunction& ff = dynamic_handle_extract<const EffectiveVectorFormulaFunction>(function);

    auto transformation = centered_coordinates_transformation(function.result_size(),inputs);
    CoordinateFormulaPairs centering_substitution = transformation.first;
    BoxDomainType transformed_inputs = transformation.second;

    Vector<EffectiveFormula> transformed_formulae = substitute(ff.formulae(),centering_substitution);

    List<SizeType> input_indices = Ariadne::input_indices(function.result_size(),inputs.size());
    if (is_additive_in(transformed_formulae,input_indices)) {
        incorporate_additive_inputs_coefficients(transformed_formulae,transformed_inputs);
    }

    _function = EffectiveVectorFormulaFunction(function.argument_size(),transformed_formulae);
    _inputs = transformed_inputs;
}

DifferentialInclusion::DifferentialInclusion(DottedRealAssignments const& dynamics, RealVariablesBox const& inputs)
{
    List<RealVariable> dyn_var_list = left_hand_sides(dynamics);
    List<RealVariable> inp_var_list;
    for (auto var : inputs.variables()) { inp_var_list.append(var); }

    Set<UntypedVariable> dyn_arg_set;
    for (auto expr : right_hand_sides(dynamics)) {
        dyn_arg_set.adjoin(expr.arguments());
    }

    Set<RealVariable> inp_var_set(inp_var_list);
    for (auto iv : inp_var_set) {
        if(not dyn_arg_set.contains(iv))
            ARIADNE_THROW(UnusedInputException,"DifferentialInclusion(dynamics,inputs) with dynamics="<<dynamics<<" and inputs="<<inputs,"The input '" << iv << "' is not in the arguments of the dynamics");
    }

    Set<UntypedVariable> dyn_var_set(dyn_var_list);
    Set<UntypedVariable> inp_var_set_untyped(inp_var_set);
    for (auto av : dyn_arg_set) {
        if (not dyn_var_set.contains(av) and not inp_var_set_untyped.contains(av))
            ARIADNE_THROW(MissingInputException,"DifferentialInclusion(dynamics,inputs) with dynamics="<<dynamics<<" and inputs="<<inputs,"The argument '" << av << "' of the dynamics is not among the inputs");
    }

    RealSpace var_spc(dyn_var_list);
    RealSpace inp_spc(inp_var_list);
    RealSpace spc = var_spc.adjoin(inp_spc);

    _variable_names = variable_names(left_hand_sides(dynamics));

    _transform_and_assign(make_function(spc,Vector<RealExpression>(right_hand_sides(dynamics))),input_bounds_to_domain(inputs));
}

RealSpace DifferentialInclusion::state_space() const
{
    return real_space(this->_variable_names);
}

Bool DifferentialInclusion::is_input_affine() const {
    return is_affine_in(_function,input_indices(dimension(),number_of_inputs()));
}

Bool DifferentialInclusion::is_input_additive() const {
    return is_additive_in(_function,input_indices(dimension(),number_of_inputs()));
}

EffectiveVectorMultivariateFunction DifferentialInclusion::noise_independent_component() const {
    return Ariadne::noise_independent_component(_function,number_of_inputs()); }
//! \brief Return the dynamics components given by the derivatives for each input.
Vector<EffectiveVectorMultivariateFunction> DifferentialInclusion::input_derivatives() const {
    return Ariadne::input_derivatives(_function,number_of_inputs()); }

}
