/***************************************************************************
 *            dynamics/vector_field_evolver.cpp
 *
 *  Copyright  2008-20  Alberto Casagrande, Pieter Collins
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
#include "config.hpp"

#include "utility/macros.hpp"
#include "utility/array.hpp"
#include "utility/tuple.hpp"
#include "helper/stlio.hpp"
#include "utility/container.hpp"
#include "algebra/vector.hpp"
#include "function/function.hpp"
#include "function/constraint.hpp"
#include "dynamics/enclosure.hpp"
#include "dynamics/orbit.hpp"

#include "solvers/integrator.hpp"

#include "conclog/logging.hpp"

#include "dynamics/vector_field.hpp"
#include "dynamics/vector_field_evolver.hpp"

#include "symbolic/space.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/expression_set.hpp"

using namespace ConcLog;

namespace Ariadne {

// TODO: Move to Numeric module
inline PositiveValidatedUpperNumber abs(PositiveValidatedUpperNumber y) { return y; }

namespace {



template<class ES> List<ES> subdivide(const ES& enclosure) {
    List<ES> result;
    Pair<ES,ES> split=enclosure.split();
    result.append(split.first);
    result.append(split.second);
    return result;
}

} // namespace

VectorFieldEvolver::VectorFieldEvolver(const SystemType& system, const IntegratorInterface& i)
    : _system(system.clone())
    , _integrator(i.clone())
    , _configuration(new ConfigurationType())
{
}

auto VectorFieldEvolver::enclosure(const ExactBoxType& box) const -> EnclosureType {
    return EnclosureType(box,this->system().state_space(),EnclosureConfiguration(this->function_factory()));
}

auto VectorFieldEvolver::function_factory() const -> FunctionFactoryType const& {
    return std::dynamic_pointer_cast<const IntegratorBase>(this->_integrator)->function_factory();
}

auto VectorFieldEvolver::orbit(RealVariablesBox const& initial_set, TimeType const& time, Semantics semantics) const -> Orbit<EnclosureType> {
    auto enclosure = EnclosureType(initial_set,this->system().state_space(),EnclosureConfiguration(this->function_factory()));
    enclosure.set_auxiliary(this->system().auxiliary_space(),this->system().auxiliary_mapping());
    return orbit(enclosure,time,semantics);
}

auto VectorFieldEvolver::orbit(RealExpressionBoundedConstraintSet const& initial_set, TimeType const& time, Semantics semantics) const -> Orbit<EnclosureType> {
    auto enclosure = EnclosureType(initial_set.euclidean_set(this->system().state_space()),this->system().state_space(),EnclosureConfiguration(this->function_factory()));
    enclosure.set_auxiliary(this->system().auxiliary_space(),this->system().auxiliary_mapping());
    return orbit(enclosure,time,semantics);
}

auto VectorFieldEvolver::orbit(EnclosureType const& initial_set, TimeType const& time, Semantics semantics) const -> Orbit<EnclosureType>
{
    CONCLOG_SCOPE_CREATE
    ARIADNE_PRECONDITION(this->system().state_auxiliary_space() == initial_set.state_auxiliary_space())

    auto result = std::make_shared<SynchronisedOrbit>(initial_set);
    WorkloadType workload([time](TimedEnclosureType const& timed_enclosure, SharedPointer<ProgressIndicator> indicator){
                                indicator->update_current(timed_enclosure.first.get_d());
                                indicator->update_final(time.get_d());
                              },
                          std::bind_front(&VectorFieldEvolver::_process_timed_enclosure,this),time,semantics,result);
    _append_initial_set(workload,TimeStepType(0u),initial_set);
    workload.process();

    return std::move(*result);
}

Void VectorFieldEvolver::
_append_initial_set(WorkloadType& workload, TimeStepType const& initial_time, EnclosureType const& current_set) const
{
    if (possibly(current_set.euclidean_set().bounding_box().radius() > this->_configuration->maximum_enclosure_radius())) {
        CONCLOG_PRINTLN_AT(1,"set is too large, splitting")
        Pair<EnclosureType,EnclosureType> split_sets = current_set.split();
        if(!definitely(split_sets.first.is_empty())) { _append_initial_set(workload,initial_time,split_sets.first); }
        if(!definitely(split_sets.second.is_empty())) { _append_initial_set(workload,initial_time,split_sets.second); }
    } else {
        workload.append({initial_time,current_set});
    }
}

Void
VectorFieldEvolver::
_process_timed_enclosure(WorkloadType::Access& workload,
                         TimedEnclosureType const& current_timed_set,
                         TimeType const& maximum_time,
                         Semantics semantics,
                         SharedPointer<SynchronisedOrbit> result) const {
    CONCLOG_SCOPE_CREATE
    TimeStepType current_time=current_timed_set.first;
    EnclosureType current_set=current_timed_set.second;
    FloatDPUpperBound current_set_radius=current_set.euclidean_set().bounding_box().radius();

    CONCLOG_PRINTLN("#r="<<std::setw(5)<<std::left<<result->reach_size()
                             <<" t="<<std::setw(7)<<std::fixed<<current_time.get_d()
                             <<" p="<<std::setw(4)<<std::left<<current_set.number_of_parameters()
                             <<" r="<<std::setw(7)<<current_set.radius()
                             <<" c="<<current_set.centre())

    if (definitely(current_time>=maximum_time)) {
        result->adjoin_final(current_set);
    } else if (semantics == Semantics::UPPER and this->_configuration->enable_subdivisions() and decide(current_set_radius>this->_configuration->maximum_enclosure_radius())) {
        // Subdivide
        List< EnclosureType > subdivisions=subdivide(current_set);
        for(SizeType i=0; i!=subdivisions.size(); ++i) {
            EnclosureType const& subdivided_set_model=subdivisions[i];
            workload.append({current_time,subdivided_set_model});
        }
    } else if (semantics == Semantics::LOWER and decide(current_set_radius>this->_configuration->maximum_enclosure_radius())) {
        CONCLOG_PRINTLN("Terminating lower evolution at time " << current_time << " and set " << current_set << " due to maximum radius being exceeded.")
    } else {
        this->_process_timed_enclosure_step(workload,current_timed_set,maximum_time,semantics,result);
    }
}

Void
VectorFieldEvolver::
_process_timed_enclosure_step(WorkloadType::Access& workload,
                              TimedEnclosureType const& working_timed_set_model,
                              TimeType const& maximum_time,
                              Semantics semantics,
                              SharedPointer<SynchronisedOrbit> result) const
{
    CONCLOG_SCOPE_CREATE
    typedef EffectiveVectorMultivariateFunction FunctionType;

    EnclosureType current_set;
    TimeStepType current_time;
    CONCLOG_PRINTLN_AT(1,"working_timed_set_model = "<<working_timed_set_model)
    make_lpair(current_time, current_set)=working_timed_set_model;

    CONCLOG_PRINTLN("current_time = "<<current_time)
    CONCLOG_PRINTLN("current_set = " << current_set)

    CONCLOG_PRINTLN("box = " << current_set.bounding_box())
    CONCLOG_PRINTLN("radius = " << current_set.euclidean_set().bounding_box().radius())

    // Test to see if set requires reconditioning
    if (this->_configuration->enable_reconditioning() && possibly(norm(current_set.state_function().errors()) > this->_configuration->maximum_spacial_error())) {
        CONCLOG_PRINTLN("reconditioning from errors " << current_set.state_function().errors())
        current_set.recondition();
        workload.append({current_time,current_set});
        return;
    }

    /////////////// Main Evolution ////////////////////////////////
    const FunctionType& dynamic=_system->function();

    // Set evolution parameters
    const StepSizeType maximum_step_size=this->_configuration->maximum_step_size();

    // Get bounding boxes for time and space bounding_box
    auto current_set_bounds=cast_exact_box(current_set.euclidean_set().bounding_box());
    CONCLOG_PRINTLN("current_set_bounds = "<<current_set_bounds)

    // Compute flow model
    IntegratorInterface const* integrator=this->_integrator.operator->();
    FlowStepModelType flow_model=integrator->flow_step(dynamic,current_set_bounds,suggest(maximum_step_size));
    StepSizeType step_size = static_cast<StepSizeType>(flow_model.domain()[flow_model.argument_size()-1u].upper_bound());
    CONCLOG_PRINTLN("step_size = "<<step_size)
    CONCLOG_PRINTLN_AT(1,"flow_model = "<<flow_model)
    FlowStepModelType flow_step_model=partial_evaluate(flow_model,flow_model.domain().size()-1u,step_size);
    CONCLOG_PRINTLN_AT(1,"flow_step_model = "<<flow_step_model)

    // Compute the integration time model
    TimeStepType next_time=current_time+TimeStepType(step_size);
    CONCLOG_PRINTLN_AT(1,"next_time = "<<next_time)
    // Compute the flow tube (reachable set) model and the final set
    EnclosureType reach_set=current_set;
    reach_set.apply_full_reach_step(flow_model);
    CONCLOG_PRINTLN_AT(1,"reach_set = " << reach_set)
    EnclosureType next_set=current_set;
    next_set.apply_fixed_evolve_step(flow_model, step_size);
    CONCLOG_PRINTLN_AT(1,"next_set = " << next_set)

    result->adjoin_reach(reach_set);
    result->adjoin_intermediate(next_set);

    workload.append({next_time,next_set});
}


VectorFieldEvolverConfiguration::VectorFieldEvolverConfiguration()
{
    set_maximum_step_size(1);
    set_maximum_enclosure_radius(100.0);
    set_maximum_spacial_error(1e-2);
    set_enable_reconditioning(true);
    set_enable_subdivisions(false);
}


OutputStream&
VectorFieldEvolverConfiguration::_write(OutputStream& os) const
{
    os << "VectorFieldEvolverConfiguration("
       << "\n maximum_step_size=" << maximum_step_size()
       << ",\n maximum_enclosure_radius=" << maximum_enclosure_radius()
       << ",\n maximum_spacial_error=" << maximum_spacial_error()
       << ",\n enable_reconditioning=" << enable_reconditioning()
       << ",\n enable_subdivisions=" << enable_subdivisions()
       << "\n)";
    return os;
}


}  // namespace Ariadne

