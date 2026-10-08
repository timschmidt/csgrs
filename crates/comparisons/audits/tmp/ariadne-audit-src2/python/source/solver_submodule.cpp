/***************************************************************************
 *            solver_submodule.cpp
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

#include "algebra/algebra.hpp"
#include "function/function.hpp"
#include "function/formula.hpp"
#include "solvers/solver_interface.hpp"
#include "solvers/solver.hpp"
#include "function/taylor_function.hpp"

#include "solvers/integrator_interface.hpp"
#include "solvers/integrator.hpp"
#include "solvers/runge_kutta_integrator.hpp"

using namespace Ariadne;

namespace Ariadne {

template<class T> Nat __hash__(const T&);
template<> Nat __hash__<FloatDPBoundsVector>(const FloatDPBoundsVector& v) {
    return 0;
}

class SolverWrapper
  : public pybind11::wrapper< SolverInterface >
{
    typedef SolverInterface::ValidatedNumericType ValidatedNumericType;
    typedef SolverInterface::ApproximateNumericType ApproximateNumericType;
  public:
    SolverInterface* clone() const { return this->get_override("clone")(); }
    Void set_maximum_error(ApproximateDouble me) { this->get_override("set_maximum_error")(me); }
    ExactDouble maximum_error() const { return this->get_override("maximum_error")(); }
    Void set_maximum_number_of_steps(Nat ns) { this->get_override("set_maximum_number_of_steps")(ns); }
    Nat maximum_number_of_steps() const { return this->get_override("maximum_number_of_steps")(); }
    Vector<ValidatedNumericType> zero(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& bx) const {
        return this->get_override("zero")(f,bx); }
    Vector<ValidatedNumericType> fixed_point(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& bx) const {
        return this->get_override("fixed_point")(f,bx); }
    Vector<ValidatedNumericType> solve(const ValidatedVectorMultivariateFunction& f, const Vector<ValidatedNumericType>& pt) const {
        return this->get_override("solve")(f,pt); }
    Vector<ValidatedNumericType> solve(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& bx) const {
        return this->get_override("solve")(f,bx); }
    ValidatedVectorMultivariateFunctionPatch implicit(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& pd, const ExactBoxType& bx) const {
        return this->get_override("implicit")(f,pd,bx); }
    ValidatedScalarMultivariateFunctionPatch implicit(const ValidatedScalarMultivariateFunction& f, const ExactBoxType& pd, const ExactIntervalType& ivl) const {
        return this->get_override("implicit")(f,pd,ivl); }
    ValidatedVectorMultivariateFunctionPatch continuation(const ValidatedVectorMultivariateFunction& f, const Vector< ApproximateNumericType>& a, const ExactBoxType& X,  const ExactBoxType& A) const {
        return this->get_override("continuation")(f,a,X,A); }
    List< Vector<ValidatedNumericType> > list_solve_all(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& bx) const {
        Set< Vector<ValidatedNumericType> > set = this->get_override("solve_all")(f,bx);
        return make_list(set);
    }
    Void _write(OutputStream& os) const { this->get_override("_write")(os); }
};


class IntegratorWrapper
  : public pybind11::wrapper<IntegratorInterface>
{
  public:
    IntegratorInterface* clone() const {
        return this->get_override("clone")(); }
    FlowStepModelType flow_step(const ValidatedVectorMultivariateFunction& vf, const ExactBoxType& D) const {
        return this->get_override("flow_step")(vf,D); }
    FlowStepModelType flow_step(const ValidatedVectorMultivariateFunction& vf, const ExactBoxType& D, const StepSizeType& h) const {
        return this->get_override("flow_step")(vf,D,h); }
    FlowStepModelType flow_step(const ValidatedVectorMultivariateFunction& vf, const ExactBoxType& D, const StepSizeType& h, const UpperBoxType& B) const {
        return this->get_override("flow_step")(vf,D,h,B); }
    Void _write(OutputStream& os) const {
        this->get_override("_write")(os); }
};

} // namespace Ariadne


Void export_solvers(pybind11::module& module)
{
    typedef SolverInterface::ValidatedNumericType ValidatedNumericType;
    pybind11::class_<SolverInterface,SolverWrapper> solver_interface_class(module,"SolverInterface");
    solver_interface_class.def("solve", (Vector<ValidatedNumericType>(SolverInterface::*)(const ValidatedVectorMultivariateFunction&,const ExactBoxType&)const) &SolverInterface::solve);
    solver_interface_class.def("implicit",(ValidatedVectorMultivariateFunctionPatch(SolverInterface::*)(const ValidatedVectorMultivariateFunction&,const ExactBoxType&,const ExactBoxType&)const) &SolverInterface::implicit);
    solver_interface_class.def("implicit",(ValidatedScalarMultivariateFunctionPatch(SolverInterface::*)(const ValidatedScalarMultivariateFunction&,const ExactBoxType&,const ExactIntervalType&)const) &SolverInterface::implicit);
    solver_interface_class.def("solve_all",[](SolverInterface const& s, const ValidatedVectorMultivariateFunction& f, const ExactBoxType& bx){ return static_cast< List<Vector<ValidatedNumericType>> >( s.solve_all(f,bx) );});
    solver_interface_class.def("__str__",&__cstr__<SolverInterface>);

    pybind11::class_<IntervalNewtonSolver, SolverInterface> interval_newton_solver_class(module,"IntervalNewtonSolver");
    interval_newton_solver_class.def(pybind11::init<ApproximateDouble,Nat>());

    pybind11::class_<KrawczykSolver, SolverInterface> krawczyk_solver_class(module,"KrawczykSolver");
    krawczyk_solver_class.def(pybind11::init<ApproximateDouble,Nat>());

    pybind11::class_<FactoredKrawczykSolver, SolverInterface> factored_krawczyk_solver_class(module,"FactoredKrawczykSolver");
    factored_krawczyk_solver_class.def(pybind11::init<ApproximateDouble,Nat>());

}



Void export_integrators(pybind11::module& module)
{
    pybind11::class_<FlowStepModelType,pybind11::bases<ValidatedVectorMultivariateFunctionPatch>> flow_step_model_class(module,"FlowStepModelType");
    flow_step_model_class.def("__str__", &__cstr__<FlowStepModelType>);

    pybind11::class_<FlowModelType> flow_model_class(module,"FlowModelType");
    flow_model_class.def("size",&FlowModelType::size);
    // Use a lambda here to prevent errors when using Clang/GCC
    flow_model_class.def("__len__",[](FlowModelType const& fm){return fm.size();});
    // NOTE: The export below gives 'ValueError: vector::reserve' at runtime when compiled using Clang
    // flow_model_class.def("__len__",&FlowModelType::size);
    // NOTE: The export below produces 'internal compiler error: in fold_convert_loc' with GCC
    // flow_model_class.def("__len__",(SizeType(FlowModelType::*)()const)&FlowModelType::size);
    flow_model_class.def("__getitem__",&__getitem__<FlowModelType,SizeType>);
    flow_model_class.def("__str__", &__cstr__<FlowModelType>);

    pybind11::class_<IntegratorInterface,IntegratorWrapper> integrator_interface_class(module,"IntegratorInterface");
    integrator_interface_class.def("flow_step",(FlowStepModelType(IntegratorInterface::*)(const ValidatedVectorMultivariateFunction&, const ExactBoxType&)const)&IntegratorInterface::flow_step);
    integrator_interface_class.def("flow_step",(FlowStepModelType(IntegratorInterface::*)(const ValidatedVectorMultivariateFunction&, const ExactBoxType&, const StepSizeType&)const)&IntegratorInterface::flow_step);
    integrator_interface_class.def("flow_step",(FlowStepModelType(IntegratorInterface::*)(const ValidatedVectorMultivariateFunction&,const ExactBoxType&,const StepSizeType&,const UpperBoxType&)const)&IntegratorInterface::flow_step);
    integrator_interface_class.def("__str__", &__cstr__<IntegratorInterface>);

    pybind11::class_<TaylorPicardIntegrator,IntegratorInterface> taylor_picard_integrator_class(module,"TaylorPicardIntegrator");
    taylor_picard_integrator_class.def(pybind11::init<ApproximateDouble>());
    taylor_picard_integrator_class.def("minimum_temporal_order",&TaylorPicardIntegrator::minimum_temporal_order);
    taylor_picard_integrator_class.def("maximum_temporal_order",&TaylorPicardIntegrator::maximum_temporal_order);
    taylor_picard_integrator_class.def("step_maximum_error",&TaylorPicardIntegrator::step_maximum_error);
    taylor_picard_integrator_class.def("set_minimum_temporal_order",&TaylorPicardIntegrator::set_minimum_temporal_order);
    taylor_picard_integrator_class.def("set_maximum_temporal_order",&TaylorPicardIntegrator::set_maximum_temporal_order);
    taylor_picard_integrator_class.def("set_step_maximum_error",&TaylorPicardIntegrator::set_step_maximum_error);

    pybind11::class_<GradedTaylorPicardIntegrator,IntegratorInterface> unbounded_taylor_picard_integrator_class(module, "GradedTaylorPicardIntegrator");
    unbounded_taylor_picard_integrator_class.def(pybind11::init<ApproximateDouble,Order>());
    unbounded_taylor_picard_integrator_class.def("order",&GradedTaylorPicardIntegrator::order);
    unbounded_taylor_picard_integrator_class.def("step_maximum_error",&GradedTaylorPicardIntegrator::step_maximum_error);
    unbounded_taylor_picard_integrator_class.def("error_refinement_minimum_improvement_percentage",&GradedTaylorPicardIntegrator::error_refinement_minimum_improvement_percentage);
    unbounded_taylor_picard_integrator_class.def("set_order",&GradedTaylorPicardIntegrator::set_order);
    unbounded_taylor_picard_integrator_class.def("set_step_maximum_error",&GradedTaylorPicardIntegrator::set_step_maximum_error);
    unbounded_taylor_picard_integrator_class.def("set_error_refinement_minimum_improvement_percentage",&GradedTaylorPicardIntegrator::set_error_refinement_minimum_improvement_percentage);

    pybind11::class_<TaylorSeriesIntegrator,IntegratorInterface> taylor_series_integrator_class(module,"TaylorSeriesIntegrator");
    taylor_series_integrator_class.def(pybind11::init<ApproximateDouble,Nat>());
    taylor_series_integrator_class.def(pybind11::init<ApproximateDouble,Nat>());
    taylor_series_integrator_class.def("order",&TaylorSeriesIntegrator::order);
    taylor_series_integrator_class.def("set_order",&TaylorSeriesIntegrator::set_order);

    pybind11::class_<GradedTaylorSeriesIntegrator,IntegratorInterface> graded_taylor_series_integrator_class(module,"GradedTaylorSeriesIntegrator");
    graded_taylor_series_integrator_class.def(pybind11::init<ApproximateDouble>());
    graded_taylor_series_integrator_class.def("maximum_spacial_order",&GradedTaylorSeriesIntegrator::maximum_spacial_order);
    graded_taylor_series_integrator_class.def("maximum_temporal_order",&GradedTaylorSeriesIntegrator::maximum_temporal_order);
    graded_taylor_series_integrator_class.def("step_maximum_error",&GradedTaylorSeriesIntegrator::step_maximum_error);
    graded_taylor_series_integrator_class.def("set_maximum_spacial_order",&GradedTaylorSeriesIntegrator::set_maximum_spacial_order);
    graded_taylor_series_integrator_class.def("set_maximum_temporal_order",&GradedTaylorSeriesIntegrator::set_maximum_temporal_order);
    graded_taylor_series_integrator_class.def("set_step_maximum_error",&GradedTaylorSeriesIntegrator::set_step_maximum_error);

    pybind11::class_<RungeKutta4Integrator> runge_kutta_4_integrator_class(module,"RungeKutta4Integrator");
    runge_kutta_4_integrator_class.def(pybind11::init<ApproximateDouble>());
    runge_kutta_4_integrator_class.def("step", &RungeKutta4Integrator::step);
    runge_kutta_4_integrator_class.def("evolve", &RungeKutta4Integrator::evolve);
}


Void solver_submodule(pybind11::module& module)
{
    export_solvers(module);
    export_integrators(module);
}


