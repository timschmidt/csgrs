/***************************************************************************
 *            optimization_submodule.cpp
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

#include "config.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "geometry/box.hpp"

#include "function/function.hpp"
#include "function/constraint.hpp"
#include "function/procedure.hpp"
#include "solvers/linear_programming.hpp"
#include "solvers/nonlinear_programming.hpp"
#include "solvers/constraint_solver.hpp"

using namespace Ariadne;




Void export_slackness(pybind11::module& module)
{
    pybind11::enum_<Slackness> variable_enum(module,"Slackness");
    variable_enum.value("BASIS", Slackness::BASIS);
    variable_enum.value("LOWER", Slackness::LOWER);
    variable_enum.value("UPPER", Slackness::UPPER);
}

Void export_constraint(pybind11::module& module)
{
    pybind11::class_<EffectiveConstraint> effective_nonlinear_constraint_class(module,"EffectiveConstraint");
    effective_nonlinear_constraint_class.def(pybind11::init<EffectiveConstraint>());
    effective_nonlinear_constraint_class.def(pybind11::init<EffectiveNumber,EffectiveScalarMultivariateFunction,EffectiveNumber>());
    effective_nonlinear_constraint_class.def(pybind11::init([](EffectiveScalarMultivariateFunction const& f, EffectiveNumber const& u){return EffectiveConstraint(EffectiveNumber(-infty),f,u);}));
    effective_nonlinear_constraint_class.def(pybind11::init([](EffectiveNumber const& l, EffectiveScalarMultivariateFunction const& f){return EffectiveConstraint(l,f,EffectiveNumber(infty));}));
    effective_nonlinear_constraint_class.def(pybind11::init<EffectiveScalarMultivariateFunction,EffectiveNumber>());

    effective_nonlinear_constraint_class.def("__str__",&__cstr__<EffectiveConstraint>);

    pybind11::class_<ValidatedConstraint> validated_nonlinear_constraint_class(module,"ValidatedConstraint");
    validated_nonlinear_constraint_class.def(pybind11::init<ValidatedNumber,ValidatedScalarMultivariateFunction,ValidatedNumber
    >());
    validated_nonlinear_constraint_class.def(pybind11::init<ValidatedConstraint>());
    validated_nonlinear_constraint_class.def(pybind11::init<EffectiveConstraint>());
    validated_nonlinear_constraint_class.def("lower_bound", &ValidatedConstraint::lower_bound);
    validated_nonlinear_constraint_class.def("upper_bound", &ValidatedConstraint::upper_bound);
    validated_nonlinear_constraint_class.def("function", (const ValidatedScalarMultivariateFunction&(ValidatedConstraint::*)()const) &ValidatedConstraint::function);
    validated_nonlinear_constraint_class.def("__str__",&__cstr__<ValidatedConstraint>);
}

Void export_optimiser_interface(pybind11::module& module)
{
    typedef OptimiserInterface::ValidatedNumericType ValidatedNumericType;
    typedef OptimiserInterface::ExactVectorType ExactVectorType;

    pybind11::class_<OptimiserInterface> optimiser_interface_class(module,"OptimiserInterface");
    optimiser_interface_class.def("minimise", (Vector<ValidatedNumericType>(OptimiserInterface::*)(ValidatedScalarMultivariateFunction, ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType)const) &OptimiserInterface::minimise);
    optimiser_interface_class.def("minimise", (Vector<ValidatedNumericType>(OptimiserInterface::*)(ValidatedScalarMultivariateFunction, ExactBoxType, ValidatedVectorMultivariateFunction, ValidatedVectorMultivariateFunction)const) &OptimiserInterface::minimise);
    optimiser_interface_class.def("feasible", (ValidatedKleenean(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType)const) &OptimiserInterface::feasible);
    optimiser_interface_class.def("feasible", (ValidatedKleenean(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ValidatedVectorMultivariateFunction)const) &OptimiserInterface::feasible);
    optimiser_interface_class.def("almost_feasible_point", (Bool(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType,FloatDPApproximationVector, FloatDPApproximation)const) &OptimiserInterface::almost_feasible_point);
    optimiser_interface_class.def("is_feasible_point", (Bool(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType,FloatDPVector)const) &OptimiserInterface::is_feasible_point);
    optimiser_interface_class.def("validate_feasibility", (Bool(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType,ExactVectorType)const) &OptimiserInterface::validate_feasibility);
    optimiser_interface_class.def("validate_feasibility", (Bool(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType,ExactVectorType,ExactVectorType)const) &OptimiserInterface::validate_feasibility);
    optimiser_interface_class.def("validate_infeasibility", (Bool(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType,ExactVectorType,ExactVectorType)const) &OptimiserInterface::validate_feasibility);
    optimiser_interface_class.def("validate_infeasibility", (ValidatedKleenean(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType,FloatDPBoundsVector)const) &OptimiserInterface::contains_feasible_point);
    optimiser_interface_class.def("is_infeasibility_certificate", (ValidatedKleenean(OptimiserInterface::*)(ExactBoxType, ValidatedVectorMultivariateFunction, ExactBoxType,FloatDPBoundsVector)const) &OptimiserInterface::contains_feasible_point);
    //NOTE: Not in C++ API
    //optimiser_interface_class.def("__str__", &__cstr__<OptimiserInterface>);
}

Void export_interior_point_solvers(pybind11::module& module)
{
    pybind11::class_<NonlinearInfeasibleInteriorPointOptimiser,OptimiserInterface> nonlinear_infeasible_interior_point_solver_class(module,"NonlinearInfeasibleInteriorPointOptimiser");
    nonlinear_infeasible_interior_point_solver_class.def(pybind11::init<>());

    pybind11::class_<NonlinearInteriorPointOptimiser,OptimiserInterface> nonlinear_interior_point_solver_class(module,"NonlinearInteriorPointOptimiser");
    nonlinear_interior_point_solver_class.def(pybind11::init<>());
}

Void export_interior_point_solver(pybind11::module& module)
{
    pybind11::class_<InteriorPointSolver> interior_point_solver_class(module,"InteriorPointSolver");
    interior_point_solver_class.def(pybind11::init<>());
    interior_point_solver_class.def("minimise", &InteriorPointSolver::minimise);
    interior_point_solver_class.def("feasible", (ValidatedKleenean(InteriorPointSolver::*)(const Vector<FloatDP>&,const Vector<FloatDP>&, const Matrix<FloatDP>&,const Vector<FloatDP>&)const) &InteriorPointSolver::feasible);
    interior_point_solver_class.def("validate_feasibility", &InteriorPointSolver::validate_feasibility);
}


Void export_constraint_solver(pybind11::module& module)
{
    pybind11::class_<ConstraintSolver> constraint_solver_class(module,"ConstraintSolver");
    constraint_solver_class.def(pybind11::init<>());
    constraint_solver_class.def("hull_reduce", (Bool(ConstraintSolver::*)(UpperBoxType&,const ValidatedScalarMultivariateFunction&,const ExactIntervalType&)const) &ConstraintSolver::hull_reduce);
    constraint_solver_class.def("box_reduce", (Bool(ConstraintSolver::*)(UpperBoxType&,const ValidatedScalarMultivariateFunction&,const ExactIntervalType&,SizeType)const) &ConstraintSolver::box_reduce);
    constraint_solver_class.def("monotone_reduce", (Bool(ConstraintSolver::*)(UpperBoxType&,const ValidatedScalarMultivariateFunction&,const ExactIntervalType&,SizeType)const) &ConstraintSolver::monotone_reduce);
    constraint_solver_class.def("reduce", (Bool(ConstraintSolver::*)(UpperBoxType&,const List<ValidatedConstraint>&)const) &ConstraintSolver::reduce);
    constraint_solver_class.def("reduce", (Bool(ConstraintSolver::*)(UpperBoxType&,const ValidatedVectorMultivariateFunction&,const ExactBoxType&)const) &ConstraintSolver::reduce);
}


template<class X>
Void export_simplex_solver(pybind11::module& module)
{
    typedef Array<SizeType> SizeArray;

    typedef RigorousNumericType<X> XX;

    pybind11::class_< SimplexSolver<X> > simplex_solver_class(module,(class_name<X>()+"SimplexSolver").c_str());
    simplex_solver_class.def(pybind11::init<>());
    simplex_solver_class.def("lpstep",(Bool(SimplexSolver<X>::*)(const Vector<X>&,const Vector<X>&,const Vector<X>&,const Matrix<X>&,const Vector<X>&,Array<Slackness>& ,SizeArray&,Matrix<XX>&,Vector<XX>&)const) &SimplexSolver<X>::lpstep);


    simplex_solver_class.def("feasible",(ValidatedKleenean(SimplexSolver<X>::*)(const Vector<X>&,const Vector<X>&,const Matrix<X>&,const Vector<X>&)const) &SimplexSolver<X>::feasible);

    simplex_solver_class.def("verify_feasibility",(ValidatedKleenean(SimplexSolver<X>::*)(const Vector<X>&,const Vector<X>&,const Matrix<X>&,const Vector<X>&,const Array<Slackness>&)const) &SimplexSolver<X>::verify_feasibility);

    simplex_solver_class.def("compute_basis",(Pair< SizeArray, Matrix<XX> >(SimplexSolver<X>::*)(const Matrix<X>&)const) &SimplexSolver<X>::compute_basis);

}


Void optimization_submodule(pybind11::module& module) {
    export_slackness(module);
    export_constraint(module);
    export_optimiser_interface(module);
    export_interior_point_solvers(module);
    export_simplex_solver<FloatDPApproximation>(module);
    export_simplex_solver<FloatDP>(module);
    export_simplex_solver<Rational>(module);
    export_interior_point_solver(module);
    export_constraint_solver(module);
}
