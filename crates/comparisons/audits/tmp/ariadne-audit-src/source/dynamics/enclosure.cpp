/***************************************************************************
 *            dynamics/enclosure.cpp
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

#include "function/functional.hpp"
#include "config.hpp"

#include <iomanip>

#include "function/constraint.hpp"
#include "function/formula.hpp"
#include "function/procedure.hpp"
#include "dynamics/enclosure.hpp"
#include "dynamics/storage.hpp"

#include "utility/macros.hpp"
#include "utility/exceptions.hpp"
#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/matrix.hpp"
#include "algebra/multi_index.hpp"
#include "algebra/differential.hpp"
#include "algebra/algebra.hpp"
#include "function/polynomial.hpp"
#include "function/function.hpp"

#include "function/function_model.hpp"
#include "function/taylor_function.hpp"

#include "geometry/box.hpp"
#include "geometry/grid.hpp"

#include "geometry/function_set.hpp"
#include "geometry/affine_set.hpp"

#include "geometry/paving_interface.hpp"
#include "geometry/paver.hpp"
#include "geometry/grid_paving.hpp"

#include "solvers/constraint_solver.hpp"
#include "solvers/nonlinear_programming.hpp"

#include "io/graphics_interface.hpp"
#include "io/drawer.hpp"
#include "io/progress_indicator.hpp"

#include "hybrid/discrete_event.hpp"

#include "io/figure.hpp"
#include "io/graphics_manager.hpp"
#include "conclog/logging.hpp"

#include "function/functional.hpp"

#include "numeric/operators.hpp"
#include "symbolic/space.hpp"
#include "symbolic/expression_set.hpp"

#include "algebra/expansion.inl.hpp"

#include "betterthreads/workload.hpp"

using namespace ConcLog;

namespace Ariadne {

using BetterThreads::StaticWorkload;

template<class T> inline StringType str(const T& t) { StringStream ss; ss<<t; return ss.str(); }

using ValidatedConstraintModelDP = Constraint<ValidatedScalarMultivariateFunctionPatch,FloatDPBounds>;

inline ValidatedConstraintModelDP operator>=(ValidatedScalarMultivariateFunctionPatch const& f, FloatDPBounds const& l) {
    return ValidatedConstraintModel(l,f,FloatDP::inf(dp)); }
inline ValidatedConstraintModelDP operator<=(ValidatedScalarMultivariateFunctionPatch const& f, FloatDPBounds const& u) {
    return ValidatedConstraintModel(-FloatDP::inf(dp),f,u); }
inline ValidatedConstraintModelDP operator==(ValidatedScalarMultivariateFunctionPatch const& f, FloatDPBounds const& c) {
    return ValidatedConstraintModel(c,f,c); }

Pair<Interval<FloatDP>,FloatDPError> make_domain(Interval<Real> const& ivl);

namespace {

ValidatedVectorMultivariateFunctionPatch make_identity(const RealBox& bx, const EnclosureConfiguration& configuration) {
    ExactBoxType dom(bx.dimension());
    Vector<FloatDPError> errs(bx.dimension(),dp);

    for(SizeType i=0; i!=bx.dimension(); ++i) {
        make_lpair(dom[i],errs[i])=make_domain(bx[i]);
    }

    ValidatedVectorMultivariateFunctionPatch res=configuration.function_factory().create_identity(dom);
    for(SizeType i=0; i!=bx.dimension(); ++i) {
        res[i]=res[i]+FloatDPBounds(-errs[i],+errs[i]);
    }

    return res;
}

} // namespace


OutputStream& operator<<(OutputStream& os, const EnclosureVariableKind& vk) {
    switch (vk) {
        case EnclosureVariableKind::INITIAL: return os << "x";
        case EnclosureVariableKind::TEMPORAL: return os << "t";
        case EnclosureVariableKind::PARAMETER: return os << "a";
        case EnclosureVariableKind::INPUT: return os << "u";
        case EnclosureVariableKind::NOISE: return os << "v";
        case EnclosureVariableKind::ERROR: return os << "e";
        case EnclosureVariableKind::UNKNOWN: default: return os << "s";
    }
}

List<Identifier> canonical_variable_names(const List<EnclosureVariableKind>& vks) {
    std::map<EnclosureVariableKind,Nat> counts;
    List<Identifier> result;
    for(SizeType i=0; i!=vks.size(); ++i) {
        result.append( str(vks[i]) + str(counts[vks[i]]++) );
    }
    return result;
}

inline Pair<ValidatedScalarMultivariateFunctionPatch,ValidatedScalarMultivariateFunctionPatch> split(const ValidatedScalarMultivariateFunctionPatch& f, SizeType k) {
    Pair<ExactBoxType,ExactBoxType> domains=split(f.domain(),k);
    return make_pair(restriction(f,domains.first),restriction(f,domains.second));
}

inline Pair<ValidatedVectorMultivariateFunctionPatch,ValidatedVectorMultivariateFunctionPatch> split(const ValidatedVectorMultivariateFunctionPatch& f, SizeType k) {
    Pair<ExactBoxType,ExactBoxType> domains=split(f.domain(),k);
    return make_pair(restriction(f,domains.first),restriction(f,domains.second));
}

EnclosureConfiguration::EnclosureConfiguration(ValidatedFunctionPatchFactory function_factory, SizeType reconditioning_num_blocks)
    : _function_factory(function_factory), _paver(new AffinePaver()), _reconditioning_num_blocks(reconditioning_num_blocks) { }


OutputStream& operator<<(OutputStream& os, EnclosureConfiguration const& ec) {
    return os << "EnclosureConfiguration( function_factory=" << ec._function_factory
              << ", paver=" << ec._paver
              <<", reconditioning_num_blocks=" << ec._reconditioning_num_blocks <<")";
}

Void Enclosure::_check() const {
    ARIADNE_ASSERT_MSG(this->_state_function.argument_size()==this->domain().size(),*this);
    ARIADNE_ASSERT_MSG(this->_time_function.argument_size()==this->domain().size(),*this<<"\n\n"<<this->_domain<<"\n"<<this->_time_function<<"\n");
    ARIADNE_ASSERT_MSG(this->_dwell_time_function.argument_size()==this->domain().size(),*this<<"\n\n"<<this->_domain<<"\n"<<this->_dwell_time_function<<"\n");
    for(List<ValidatedConstraintModel>::ConstIterator iter=this->_constraints.begin(); iter!=this->_constraints.end(); ++iter) {
        ARIADNE_ASSERT_MSG(iter->function().argument_size()==this->domain().size(),*this);
    }
    ARIADNE_ASSERT_MSG(this->_variable_kinds.size()==this->domain().size(),*this<<"\n\n"<<this->_domain<<"\n"<<this->_variable_kinds<<"\n");
    ARIADNE_ASSERT_MSG(this->_state_function.result_size()==this->_auxiliary_mapping.argument_size(),*this<<"\n\n"<<this->_state_function<<"\n"<<this->_auxiliary_mapping<<"\n");
}

EnclosureConfiguration const&
Enclosure::configuration() const {
    return this->_configuration;
}

/*

// TODO: Make more efficient
inline Void assign_all_but_last(MultiIndex& r, const MultiIndex& a) {
    for(SizeType i=0; i!=r.size(); ++i) { r[i]=a[i]; }
}

// FIXME: What if solving for constraint leaves domain?
Void Enclosure::_solve_zero_constraints() {
    this->_check();
    for(List<ValidatedScalarMultivariateFunctionPatch>::Iterator iter=this->_zero_constraints.begin(); iter!=this->_zero_constraints.end(); ) {
        const ExactBoxType& domain=this->domain();
        const ValidatedTaylorModelDP& model=iter->model();
        const SizeType k=model.argument_size()-1u;
        ValidatedTaylorModelDP zeroth_order(k,this->sweeper());
        ValidatedTaylorModelDP first_order(k,this->sweeper());
        Bool is_zeroth_order=true;
        Bool is_first_order=true;
        MultiIndex r(k);
        // Try linear approach in last coefficient
        for(ValidatedTaylorModelDP::ConstIterator tmiter=model.begin(); tmiter!=model.end(); ++tmiter) {
            if(tmiter->index()[k]==0) {
                assign_all_but_last(r,tmiter->index());
                zeroth_order.expansion().append(r,tmiter->coefficient());
            } else if(tmiter->index()[k]==1) {
                is_zeroth_order=false;
                assign_all_but_last(r,tmiter->index());
                first_order.expansion().append(r,tmiter->coefficient());
            } else {
                is_first_order=false; break;
            }
        }
        if(is_first_order && !is_zeroth_order) {
            const ExactBoxType new_domain=project(domain,range(0,k));
            ValidatedTaylorModelDP substitution_model=-zeroth_order/first_order;
            this->_state_function=this->configuration().function_factory().create(new_domain,Ariadne::substitute(this->_state_function.models(),k,substitution_model));
            for(List<ValidatedScalarMultivariateFunctionPatch>::Iterator constraint_iter=this->_negative_constraints.begin();
                    constraint_iter!=this->_negative_constraints.end(); ++constraint_iter) {
                ValidatedScalarMultivariateFunctionPatch& constraint=*constraint_iter;
                constraint=this->configuration().function_factory().create(new_domain,Ariadne::substitute(constraint.model(),k,substitution_model));
            }
            for(List<ValidatedScalarMultivariateFunctionPatch>::Iterator constraint_iter=this->_zero_constraints.begin();
                    constraint_iter!=this->_zero_constraints.end(); ++constraint_iter) {
                ValidatedScalarMultivariateFunctionPatch& constraint=*constraint_iter;
                constraint=this->configuration().function_factory().create(new_domain,Ariadne::substitute(constraint.model(),k,substitution_model));
            }
            // Since we are using an std::vector, assign Iterator to next element
            iter=this->_zero_constraints.erase(iter);
            this->_check();
        } else {
            ARIADNE_WARN("No method for solving constraint "<<*iter<<" currently implemented.");
            ++iter;
        }
    }
}
*/

Enclosure::Enclosure()
    : _domain(), _auxiliary_mapping(), _state_function()
    , _time_function(), _dwell_time_function()
    , _reduced_domain(), _is_fully_reduced(true)
    , _variable_kinds()
    , _configuration(ValidatedFunctionPatchFactory(nullptr),Paver(nullptr))
{
}

Enclosure::Enclosure(EnclosureConfiguration const& config)
    : _domain(), _auxiliary_mapping(), _state_function()
    , _time_function(), _dwell_time_function()
    , _reduced_domain(), _is_fully_reduced(true)
    , _variable_kinds()
    , _configuration(config)
{
}

Enclosure* Enclosure::clone() const
{
    return new Enclosure(*this);
}

Enclosure::Enclosure(const RealBox& box, const EnclosureConfiguration& configuration)
    : Enclosure(BoundedConstraintSet(box),configuration)
{
}

Enclosure::Enclosure(const BoundedConstraintSet& set, const EnclosureConfiguration& configuration)
    : _configuration(configuration)
{
    this->_state_function=make_identity(set.domain(),configuration);
    this->_domain=this->_state_function.domain();
    this->_time_function=this->configuration().function_factory().create_zero(this->domain());
    this->_dwell_time_function=this->configuration().function_factory().create_zero(this->domain());
    this->_auxiliary_mapping=EffectiveVectorMultivariateFunction(0u,EuclideanDomain(this->_state_function.result_size()));
    this->_variable_kinds=List<EnclosureVariableKind>(this->_domain.size(),EnclosureVariableKind::INITIAL);
    for(SizeType i=0; i!=set.number_of_constraints(); ++i) {
        this->new_state_constraint(set.constraint(i));
    }
    this->_reduced_domain=this->_domain;
    this->_is_fully_reduced=true;
    this->_check();
}

Enclosure::Enclosure(const ExactBoxType& box, const EnclosureConfiguration& configuration)
    : _configuration(configuration)
{
    // Ensure domain elements have nonempty radius
    this->_domain=box;

    this->_variable_kinds=List<EnclosureVariableKind>(this->_domain.dimension(),EnclosureVariableKind::INITIAL);

    this->_state_function=this->configuration().function_factory().create_identity(this->_domain);
    this->_time_function=this->configuration().function_factory().create_zero(this->_domain);
    this->_dwell_time_function=this->configuration().function_factory().create_zero(this->domain());
    this->_auxiliary_mapping=EffectiveVectorMultivariateFunction(0u,EuclideanDomain(this->_state_function.result_size()));
    this->_reduced_domain=this->_domain;
    this->_is_fully_reduced=true;
    this->_check();
}


Enclosure::Enclosure(const ExactBoxType& domain, const ValidatedVectorMultivariateFunction& function, const EnclosureConfiguration& configuration)
    : Enclosure(domain,function,List<ValidatedConstraint>(),configuration)
{
}

Enclosure::Enclosure(const ExactBoxType& domain, const ValidatedVectorMultivariateFunction& function, const List<ValidatedConstraint>& constraints, const EnclosureConfiguration& configuration)
    : Enclosure(domain,function,configuration.function_factory().create_zero(domain),constraints,configuration)
{
}

Enclosure::Enclosure(const ExactBoxType& domain, const ValidatedVectorMultivariateFunction& state_function, const ValidatedScalarMultivariateFunction& time_function, const List<ValidatedConstraint>& constraints, const EnclosureConfiguration& configuration)
    : _configuration(configuration)
{
    ARIADNE_ASSERT_MSG(domain.size()==state_function.argument_size(),"domain="<<domain<<", state_function="<<state_function);
    ARIADNE_ASSERT_MSG(domain.size()==time_function.argument_size(),"domain="<<domain<<", time_function="<<time_function);
    ARIADNE_ASSERT_MSG(state_function.domain()==time_function.domain(),"state_function.domain()="<<state_function.domain()<<", time_function.domain()="<<time_function.domain());
    this->_domain=domain;
    this->_variable_kinds=List<EnclosureVariableKind>(this->_domain.size(),EnclosureVariableKind::INITIAL);

    this->_state_function=this->configuration().function_factory().create(this->_domain,state_function);
    this->_time_function=this->configuration().function_factory().create(this->_domain,time_function);
    this->_dwell_time_function=this->configuration().function_factory().create_zero(this->domain());
    this->_auxiliary_mapping=EffectiveVectorMultivariateFunction(0u,EuclideanDomain(this->_state_function.result_size()));

    for(SizeType i=0; i!=constraints.size(); ++i) {
        ARIADNE_ASSERT_MSG(domain.size()==constraints[i].function().argument_size(),"domain="<<domain<<", constraint="<<constraints[i]);
        this->new_parameter_constraint(constraints[i]);
    }

    this->_reduced_domain=domain;
    this->_check();
    this->reduce();
    this->_check();
}




// Returns true if the entire set is positive; false if entire set is negative
ValidatedKleenean Enclosure::satisfies(ValidatedScalarMultivariateFunction constraint) const
{
    UpperIntervalType constraint_range=apply(constraint,this->codomain());
    if(definitely(constraint_range.upper_bound()<0)) { return false; }
    if(definitely(constraint_range.lower_bound()>0)) { return true; }
    return ValidatedKleenean(indeterminate);
}


/*
Void Enclosure::substitute(SizeType j, ValidatedScalarMultivariateFunctionPatch v)
{
    ARIADNE_ASSERT_MSG(v.argument_size()+1u==this->number_of_parameters(),
                       "number_of_parameters="<<this->number_of_parameters()<<", variable="<<v);
                       this->_state_function = Ariadne::substitute(this->_state_function,j,v);
                       for(List<ValidatedScalarMultivariateFunctionPatch>::Iterator iter=this->_negative_constraints.begin(); iter!=this->_negative_constraints.end(); ++iter) {
                           *iter = Ariadne::substitute(*iter,j,v);
                       }
                       for(List<ValidatedScalarMultivariateFunctionPatch>::Iterator iter=this->_zero_constraints.begin(); iter!=this->_zero_constraints.end(); ++iter) {
                           *iter = Ariadne::substitute(*iter,j,v);
                       }

                       this->_check();
}

Void Enclosure::substitute(SizeType j, FloatDP c)
{
    this->_state_function = Ariadne::partial_evaluate(this->_state_function,j,c);
    for(List<ValidatedScalarMultivariateFunctionPatch>::Iterator iter=this->_negative_constraints.begin(); iter!=this->_negative_constraints.end(); ++iter) {
        *iter = Ariadne::partial_evaluate(*iter,j,c);
    }
    for(List<ValidatedScalarMultivariateFunctionPatch>::Iterator iter=this->_zero_constraints.begin(); iter!=this->_zero_constraints.end(); ++iter) {
        *iter = Ariadne::partial_evaluate(*iter,j,c);
    }
    this->_check();
}
*/

const List<EnclosureVariableKind>& Enclosure::variable_kinds() const {
    return this->_variable_kinds;
}

EffectiveVectorMultivariateFunction const& Enclosure::auxiliary_mapping() const {
    return this->_auxiliary_mapping;
}

Void Enclosure::set_auxiliary_mapping(const EffectiveVectorMultivariateFunction& aux) {
    ARIADNE_PRECONDITION(this->_state_function.result_size()==aux.argument_size());
    this->_auxiliary_mapping=aux;
}

Void Enclosure::_unchecked_new_variable(ExactIntervalType ivl, EnclosureVariableKind vk)
{
    ValidatedScalarMultivariateFunctionPatch variable_function = this->configuration().function_factory().create_coordinate({ivl},0);
    this->_domain=product(this->_domain,ivl);
    this->_reduced_domain=product(this->_reduced_domain,ivl);
    this->_state_function=combine(this->_state_function,variable_function);
    this->_time_function=embed(this->_time_function,ivl);
    this->_dwell_time_function=embed(this->_dwell_time_function,ivl);

    for(SizeType i=0; i!=this->_constraints.size(); ++i) {
        ValidatedConstraintModel& constraint=this->_constraints[i];
        constraint.set_function(embed(constraint.function(),ivl));
    }
    this->_variable_kinds.append(vk);
}

Void Enclosure::clear_time()
{
    this->_time_function=0;
    this->_dwell_time_function=0;
    this->_check();
}

Void Enclosure::apply_map(ValidatedVectorMultivariateFunction map)
{
    ARIADNE_ASSERT_MSG(map.argument_size()==this->state_dimension(),"state_dimension="<<this->state_dimension()<<", map="<<map);
    this->_state_function=compose(map,this->_state_function);
    this->_dwell_time_function=this->configuration().function_factory().create_zero(this->domain());
    this->_check();
}

Void Enclosure::apply_map(ValidatedVectorMultivariateFunction map, EffectiveVectorMultivariateFunction aux_map)
{
    ARIADNE_ASSERT_MSG(map.argument_size()==this->state_dimension(),"state_dimension="<<this->state_dimension()<<", map="<<map);
    ARIADNE_ASSERT_MSG(aux_map.argument_size()==map.result_size(),"map="<<map<<", aux_map="<<aux_map);
    this->_state_function=compose(map,this->_state_function);
    this->_dwell_time_function=this->configuration().function_factory().create_zero(this->domain());
    this->_auxiliary_mapping=aux_map;
    this->_check();
}


Void Enclosure::apply_discrete_time_map_step(ValidatedVectorMultivariateFunction map)
{
    this->apply_map(map);
    this->_time_function=this->_time_function+StepSizeType(1,0u);
}


Void Enclosure::apply_fixed_evolve_step(ValidatedVectorMultivariateFunction flow, StepSizeType time)
{
    ARIADNE_ASSERT_MSG(flow.argument_size()==this->state_dimension()+1u,"state_dimension="<<this->state_dimension()<<", flow="<<flow);
    try {
        ValidatedVectorMultivariateFunctionPatch flow_model=dynamic_handle_cast<const ValidatedVectorMultivariateFunctionPatch>(flow);
        ValidatedVectorMultivariateFunctionPatch flow_step_model=partial_evaluate(flow_model,flow_model.argument_size()-1u,time);
        this->_state_function=compose(flow_step_model,this->_state_function);
    } catch (std::bad_cast const&) {
        ValidatedScalarMultivariateFunctionPatch evolve_time_function=this->configuration().function_factory().create_constant(this->domain(),time);
        this->_state_function=compose(flow,join(this->_state_function,evolve_time_function));
    }
    this->_time_function=this->_time_function + time;
    this->_dwell_time_function=this->_dwell_time_function + time;
    this->_check();
}

Void Enclosure::apply_space_evolve_step(ValidatedVectorMultivariateFunction flow, ValidatedScalarMultivariateFunction time)
{
    ARIADNE_ASSERT_MSG(flow.argument_size()==this->state_dimension()+1u,"state_dimension="<<this->state_dimension()<<", flow="<<flow);
    ARIADNE_ASSERT_MSG(time.argument_size()==this->state_dimension(),"state_dimension="<<this->state_dimension()<<", time="<<time);
    ValidatedScalarMultivariateFunctionPatch evolve_time_function=compose(time,this->_state_function);
    this->_state_function=compose(flow,join(this->_state_function,evolve_time_function));
    this->_time_function=this->_time_function + evolve_time_function;
    this->_dwell_time_function=this->_dwell_time_function + evolve_time_function;
    this->_check();
}

Void Enclosure::apply_spacetime_evolve_step(ValidatedVectorMultivariateFunction flow, ValidatedScalarMultivariateFunction time)
{
    ARIADNE_ASSERT_MSG(flow.argument_size()==this->state_dimension()+1u,"state_dimension="<<this->state_dimension()<<", flow="<<flow);
    ARIADNE_ASSERT_MSG(time.argument_size()==this->state_dimension()+1u,"state_dimension="<<this->state_dimension()<<", time="<<time);
    ValidatedScalarMultivariateFunctionPatch evolve_time_function=compose(time,join(this->_state_function,this->_time_function));
    this->_state_function=compose(flow,join(this->_state_function,evolve_time_function));
    this->_time_function=this->_time_function + evolve_time_function;
    this->_dwell_time_function=this->_dwell_time_function + evolve_time_function;
    this->_check();
}

Void Enclosure::apply_parameter_evolve_step(ValidatedVectorMultivariateFunction flow, ValidatedScalarMultivariateFunction time)
{
    ARIADNE_ASSERT_MSG(flow.argument_size()==this->state_dimension()+1u,"state_dimension="<<this->state_dimension()<<", flow="<<flow);
    ARIADNE_ASSERT_MSG(time.argument_size()==this->number_of_parameters(),"number_of_parameters="<<this->number_of_parameters()<<", time="<<time<<", enclosure="<<*this);
    ARIADNE_ASSERT_MSG(time.domain().dimension()==this->parameter_domain().dimension(),"parameter_domain="<<this->parameter_domain()<<", time="<<time);
    this->_state_function=compose(flow,join(this->_state_function,this->configuration().function_factory().create(this->_state_function.domain(),time)));
    this->_time_function=this->_time_function + time;
    this->_dwell_time_function=this->_dwell_time_function + time;
    this->_check();
}

Void Enclosure::apply_finishing_parameter_evolve_step(ValidatedVectorMultivariateFunction flow, ValidatedScalarMultivariateFunction finishing_time)
{
    ARIADNE_ASSERT_MSG(flow.argument_size()==this->state_dimension()+1u,"state_dimension="<<this->state_dimension()<<", flow="<<flow);
    ARIADNE_ASSERT_MSG(finishing_time.argument_size()==this->number_of_parameters(),"number_of_parameters="<<this->number_of_parameters()<<", finishing_time="<<finishing_time);
    ARIADNE_ASSERT_MSG(finishing_time.domain().dimension()==this->parameter_domain().dimension(),"parameter_domain="<<this->parameter_domain()<<", finishing_time="<<finishing_time);
    ValidatedScalarMultivariateFunctionPatch omega=this->configuration().function_factory().create(this->domain(),finishing_time);
    this->_state_function=compose(flow,join(this->_state_function,omega-this->_time_function));
    this->_dwell_time_function=this->_dwell_time_function + (omega-this->_time_function);
    this->_time_function=omega;
    this->_check();
}

Void Enclosure::apply_full_reach_step(ValidatedVectorMultivariateFunctionPatch phi)
{
    // xi'(s,t) = phi(xi(s),t) for t in [0,h] with constraint t<=eps(s) where range(eps) in [0,h]
    // tau'(s) = tau(s)+t
    ARIADNE_ASSERT(phi.result_size()==this->state_dimension());
    ARIADNE_ASSERT(phi.argument_size()==this->state_dimension()+1);
    FloatDP h=phi.domain()[phi.result_size()].upper_bound();
    ValidatedScalarMultivariateFunctionPatch elps=this->configuration().function_factory().create_constant(this->domain(),h);
    this->apply_parameter_reach_step(phi,elps);
    this->_check();
}

Void Enclosure::apply_spacetime_reach_step(ValidatedVectorMultivariateFunctionPatch phi, ValidatedScalarMultivariateFunction elps)
{
    ARIADNE_ASSERT(phi.result_size()==this->state_dimension());
    ARIADNE_ASSERT(phi.argument_size()==this->state_dimension()+1);
    ARIADNE_ASSERT(elps.argument_size()==this->state_dimension()+1);
    this->apply_parameter_reach_step(phi,compose(elps,join(this->state_function(),this->time_function())));
    this->_check();
}

Void Enclosure::apply_parameter_reach_step(ValidatedVectorMultivariateFunctionPatch phi, ValidatedScalarMultivariateFunction elps)
{
    // xi'(s,t) = phi(xi(s),t) for t in [0,h] with constraint t<=eps(s) where range(eps) in [0,h]
    // tau'(s) = tau(s)+t
    ARIADNE_ASSERT(phi.result_size()==this->state_dimension());
    ARIADNE_ASSERT(phi.argument_size()==this->state_dimension()+1);
    ARIADNE_ASSERT(elps.argument_size()==this->number_of_parameters());
    FloatDP h=phi.domain()[phi.result_size()].upper_bound();
    ExactBoxType parameter_domain=this->parameter_domain();
    ExactIntervalType time_domain=ExactIntervalType(FloatDP(0,dp),h);
    ValidatedScalarMultivariateFunctionPatch time_function=this->configuration().function_factory().create_coordinate({time_domain},0u);
    this->_unchecked_new_variable(time_domain,EnclosureVariableKind::TEMPORAL);
    ARIADNE_ASSERT(phi.argument_size()==this->state_dimension());
    this->apply_map(phi);
    ExactBoxType new_domain=this->parameter_domain();
    ValidatedScalarMultivariateFunctionPatch time_step_function=this->configuration().function_factory().create_coordinate(new_domain,new_domain.size()-1u);
    this->_time_function=this->_time_function+time_step_function;
    this->_dwell_time_function=this->_dwell_time_function+time_step_function;
    if(phi.domain()[phi.result_size()].lower_bound()<time_domain.upper_bound()) {
        auto restricted_elps=this->configuration().function_factory().create(parameter_domain,elps);
        this->new_negative_parameter_constraint(time_step_function-embed(restricted_elps,time_domain));
    }
    this->_check();
}

Void Enclosure::new_state_time_bound(ValidatedScalarMultivariateFunction gamma) {
    ARIADNE_ASSERT(gamma.argument_size()==this->state_dimension());
    this->_is_fully_reduced=false;
    this->_constraints.append(compose(gamma,this->state_function())<=this->time_function());
    this->_check();
}

Void Enclosure::new_state_constraint(ValidatedConstraint constraint) {
    ARIADNE_ASSERT(constraint.function().argument_size()==this->state_dimension());
    this->_is_fully_reduced=false;
    ValidatedNumber lower_bound=constraint.lower_bound();
    ValidatedScalarMultivariateFunctionPatch composed_function_model=compose(constraint.function(),this->_state_function);
    ValidatedNumber upper_bound=constraint.upper_bound();
    this->_constraints.append(ValidatedConstraintModel(lower_bound,composed_function_model,upper_bound));
    this->_check();
}

Void Enclosure::new_state_time_constraint(ValidatedConstraint constraint) {
    ARIADNE_ASSERT(constraint.function().argument_size()==this->state_dimension()+1u);
    this->_is_fully_reduced=false;
    ValidatedNumber lower_bound=constraint.lower_bound();
    ValidatedScalarMultivariateFunctionPatch composed_function_model=compose(constraint.function(),join(this->_state_function,this->_time_function));
    ValidatedNumber upper_bound=constraint.upper_bound();
    this->_constraints.append(ValidatedConstraintModel(lower_bound,composed_function_model,upper_bound));
    this->_check();
}

Void Enclosure::new_parameter_constraint(ValidatedConstraint constraint) {
    ARIADNE_ASSERT(constraint.function().argument_size()==this->number_of_parameters());
    this->_is_fully_reduced=false;
    ValidatedNumber lower_bound=constraint.lower_bound();
    ValidatedScalarMultivariateFunctionPatch function_model=this->configuration().function_factory().create(this->domain(),constraint.function());
    ValidatedNumber upper_bound=constraint.upper_bound();
    this->_constraints.append(ValidatedConstraintModel(lower_bound,function_model,upper_bound));
    this->_check();
}

Void Enclosure::new_positive_state_constraint(ValidatedScalarMultivariateFunction constraint_function) {
    ARIADNE_ASSERT_MSG(constraint_function.argument_size()==this->state_dimension(),"state_dimension="<<this->state_dimension()<<", constraint_function="<<constraint_function);
    this->_is_fully_reduced=false;
    ExactNumber zero=0;
    this->_constraints.append(compose(constraint_function,this->state_function())>=zero);
    this->_check();
}

Void Enclosure::new_negative_state_constraint(ValidatedScalarMultivariateFunction constraint_function) {
    ARIADNE_ASSERT_MSG(constraint_function.argument_size()==this->state_dimension(),"state_dimension="<<this->state_dimension()<<", constraint_function="<<constraint_function);
    this->_is_fully_reduced=false;
    ExactNumber zero=0;
    this->_constraints.append(compose(constraint_function,this->state_function())<=zero);
    this->_check();
}

Void Enclosure::new_zero_state_constraint(ValidatedScalarMultivariateFunction constraint_function) {
    ARIADNE_ASSERT_MSG(constraint_function.argument_size()==this->state_dimension(),"state_dimension="<<this->state_dimension()<<", constraint_function="<<constraint_function);
    this->_is_fully_reduced=false;
    ExactNumber zero=0;
    this->_constraints.append(compose(constraint_function,this->state_function())==zero);
    this->_check();
}

Void Enclosure::new_negative_parameter_constraint(ValidatedScalarMultivariateFunction constraint_function) {
    ARIADNE_ASSERT_MSG(constraint_function.argument_size()==this->domain().size(),"domain="<<this->domain()<<", constraint_function="<<constraint_function);
    this->_is_fully_reduced=false;
    ExactNumber zero=0;
    this->_constraints.append(this->configuration().function_factory().create(this->domain(),constraint_function)<=zero);
    this->_check();
}

Void Enclosure::new_zero_parameter_constraint(ValidatedScalarMultivariateFunction constraint_function) {
    ARIADNE_ASSERT_MSG(constraint_function.argument_size()==this->domain().size(),"domain="<<this->domain()<<", constraint_function="<<constraint_function);
    this->_is_fully_reduced=false;
    ExactNumber zero=0;
    this->_constraints.append(this->configuration().function_factory().create(this->domain(),constraint_function)==zero);
    this->_check();
}




ExactBoxType Enclosure::domain() const {
    return this->_domain;
}

ExactBoxType Enclosure::parameter_domain() const {
    return this->_domain;
}

ExactBoxType Enclosure::reduced_domain() const {
    return this->_reduced_domain;
}

ExactBoxType Enclosure::codomain() const {
    return cast_exact_box(widen(this->_state_function.range()));
}

ValidatedVectorMultivariateFunctionPatch const Enclosure::state_time_auxiliary_function() const {
    return join(join(this->state_function(),this->time_function()),this->auxiliary_function());
}

ValidatedVectorMultivariateFunctionPatch const Enclosure::state_auxiliary_function() const {
    return join(this->state_function(),this->auxiliary_function());
}

ValidatedVectorMultivariateFunctionPatch const& Enclosure::state_function() const {
    return this->_state_function;
}

ValidatedScalarMultivariateFunctionPatch const& Enclosure::time_function() const {
    return this->_time_function;
}

ValidatedScalarMultivariateFunctionPatch const& Enclosure::dwell_time_function() const {
    return this->_dwell_time_function;
}

ValidatedVectorMultivariateFunctionPatch const Enclosure::auxiliary_function() const {
    static_assert(Same<decltype(this->_auxiliary_mapping),EffectiveVectorMultivariateFunction>);
    static_assert(Same<decltype(this->_state_function),ValidatedVectorMultivariateFunctionPatch>);
    static_assert(not Convertible<ValidatedVectorMultivariateFunction,EffectiveVectorMultivariateFunction>);
    static_assert(not Convertible<ValidatedVectorMultivariateFunctionPatch,EffectiveVectorMultivariateFunction>);
    static_assert(not Convertible<EffectiveVectorMultivariateFunction,ValidatedVectorMultivariateFunctionPatch>);
    return compose(this->_auxiliary_mapping,this->_state_function);
}

SizeType Enclosure::number_of_constraints() const {
    return this->_constraints.size();
}

List<ValidatedConstraintModel> const& Enclosure::constraint_models() const {
    return this->_constraints;
}

List<ValidatedConstraint> const Enclosure::constraints() const {
    List<ValidatedConstraint> result;
    for(SizeType i=0; i!=this->_constraints.size(); ++i) {
        result.append(ValidatedConstraint(this->_constraints[i].lower_bound(),this->_constraints[i].function(),this->_constraints[i].upper_bound()));
    }
    return result;
}

ValidatedConstraintModel const& Enclosure::constraint(SizeType i) const {
    return this->_constraints[i];
}

ValidatedVectorMultivariateFunctionPatch const Enclosure::constraint_function() const {
    ValidatedVectorMultivariateFunctionPatch g=this->configuration().function_factory().create_zeros(this->number_of_constraints(),this->domain());
    for(SizeType i=0; i!=this->number_of_constraints(); ++i) {
        g.set(i,this->constraint(i).function());
    }
    return g;
}

ExactBoxType const Enclosure::constraint_bounds() const {
    ExactBoxType c(this->number_of_constraints());
    for(SizeType i=0; i!=this->number_of_constraints(); ++i) {
        c[i]=this->constraint(i).bounds();
    }
    return c;
}

DimensionType Enclosure::dimension() const {
    return this->_state_function.result_size() + this->_auxiliary_mapping.result_size();
}

DimensionType Enclosure::state_dimension() const {
    return this->_state_function.result_size();
}

SizeType Enclosure::number_of_parameters() const {
    return this->_state_function.argument_size();
}

UpperBoxType Enclosure::bounding_box() const {
    return this->_state_function.codomain().bounding_box();
}

FloatDPError Enclosure::radius() const {
    return cast_positive(this->bounding_box().radius());
}

FloatDPPoint Enclosure::centre() const {
    return cast_exact(this->bounding_box().centre());
}


ValidatedKleenean
Enclosure::satisfies(ValidatedConstraint c) const
{
    Enclosure copy=*this;
    copy.new_state_constraint(c);
    if(definitely(copy.is_empty())) { return false; }
    else { return ValidatedKleenean(indeterminate); }
}

ValidatedLowerKleenean Enclosure::is_bounded() const
{
    return this->domain().is_bounded() || ValidatedKleenean(indeterminate);
}

ValidatedLowerKleenean Enclosure::is_empty() const
{
    if(definitely(this->_reduced_domain.is_empty())) { return true; }
    if(this->_constraints.empty()) { return this->domain().is_empty(); }
    if(!this->_is_fully_reduced) { this->reduce(); this->reduce(); this->reduce(); }

    for(SizeType i=0; i!=this->_constraints.size(); ++i) {
        UpperIntervalType constraint_range = Ariadne::apply(this->_constraints[i].function(),this->_reduced_domain);
        if( definitely(disjoint(constraint_range,this->_constraints[i].bounds())) ) {
            if(this->_reduced_domain.size()>0) { this->_reduced_domain[0] = ExactIntervalType(1,-1); }
            return true;
        }
    }
    if(this->_reduced_domain.is_empty()) { return true; }
    return ValidatedKleenean(indeterminate);
}

ValidatedLowerKleenean Enclosure::inside(const ExactBoxType& bx) const
{
    return Ariadne::subset(Ariadne::apply(this->_state_function,this->_reduced_domain),bx);
}

ValidatedLowerKleenean Enclosure::subset(const ExactBoxType& bx) const
{
    this->reduce();

    return ValidatedLowerKleenean(Ariadne::subset(Ariadne::apply(this->_state_function,this->_reduced_domain),bx)) || ValidatedKleenean(indeterminate);
}

ValidatedLowerKleenean Enclosure::separated(const ExactBoxType& bx) const
{
    ARIADNE_ASSERT_MSG(this->state_dimension()==bx.dimension(),"Enclosure::subset(ExactBoxType): self="<<*this<<", box="<<bx);
    List<ValidatedConstraint> constraints = this->constraints();
    ConstraintSolver contractor=ConstraintSolver();
    contractor.reduce(reinterpret_cast<UpperBoxType&>(this->_reduced_domain),constraints);

    if(_reduced_domain.is_empty()) { return true; }

    const ExactBoxType test_domain=this->_reduced_domain;
    for(SizeType i=0; i!=bx.dimension(); ++i) {
        // FIXME: Conversion should be automatic
        ValidatedScalarMultivariateFunction fi(static_cast<ValidatedScalarMultivariateFunction::Interface const&>(this->_state_function[i]));
        constraints.append(fi >= bx[i].lower_bound());
        constraints.append(fi <= bx[i].upper_bound());
    }
    return !contractor.feasible(test_domain,constraints).first;
}

Void Enclosure::reduce() const
{
    List<ValidatedConstraint> constraints=this->constraints();
    ConstraintSolver contractor=ConstraintSolver();
    contractor.reduce(reinterpret_cast<UpperBoxType&>(this->_reduced_domain),constraints);

    for(SizeType i=0; i!=this->number_of_parameters(); ++i) {
        FloatDP l=this->_reduced_domain[i].lower_bound();
        FloatDP u=this->_reduced_domain[i].upper_bound();
        if(is_nan(l) || is_nan(u)) {
            ARIADNE_WARN("Reducing domain "<<_domain<<" yields "<<this->_reduced_domain);
            _reduced_domain[i]=_domain[i];
        }
    }

/*
    // Remove redundant constraints
    SizeType j=0;
    List<ValidatedScalarMultivariateFunctionPatch>& mutable_constraints=const_cast<List<ValidatedScalarMultivariateFunctionPatch>&>(this->_negative_constraints);
    for(SizeType i=0; i!=mutable_constraints.size(); ++i) {
        if(mutable_constraints[i](this->_reduced_domain).upper_bound()<0.0) { redundant_constraints.append(i); }
        else { if(i>j) { mutable_constraints[j]=mutable_constraints[j]; } ++j; }
    }
    mutable_constraints.resize(j);
*/


}



Matrix<FloatDPError> nonlinearities_zeroth_order(const ValidatedVectorMultivariateFunction& f, const ExactBoxType& dom);
Pair<SizeType,FloatDPError> nonlinearity_index_and_error(const ValidatedVectorMultivariateFunction& function, const ExactBoxType& domain);
Pair<SizeType,FloatDPError> lipschitz_index_and_error(const ValidatedVectorMultivariateFunction& function, const ExactBoxType& domain);

Pair<Enclosure,Enclosure>
Enclosure::split_zeroth_order() const
{
    return this->split(this->splitting_index_zeroth_order());
}


List<ExactBoxType>
Enclosure::splitting_subdomains_zeroth_order() const
{
    List<ExactBoxType> result;
    SizeType k=this->splitting_index_zeroth_order();
    if(k==this->number_of_parameters()) {
        result.append(this->_reduced_domain);
    } else {
        Pair<ExactBoxType,ExactBoxType> subdomains = this->_reduced_domain.split(this->splitting_index_zeroth_order());
        result.append(subdomains.first);
        result.append(subdomains.second);
    }
    return result;
}


SizeType
Enclosure::splitting_index_zeroth_order() const
{
    Matrix<UpperIntervalType> jacobian=Ariadne::jacobian_range(this->state_function(),cast_vector(this->reduced_domain()));

    // Compute the column of the matrix which has the norm
    // i.e. the highest sum of $mag(a_ij)$ where mag([l,u])=max(|l|,|u|)
    SizeType jmax=this->number_of_parameters();
    FloatDPError max_column_norm(0u,dp);
    for(SizeType j=0; j!=this->number_of_parameters(); ++j) {
        FloatDPError column_norm(0u,dp);
        for(SizeType i=0; i!=this->state_dimension(); ++i) {
            column_norm+=mag(jacobian[i][j]);
        }
        column_norm *= this->reduced_domain()[j].radius();
        if(column_norm.raw()>max_column_norm.raw()) {
            max_column_norm=column_norm;
            jmax=j;
        }
    }

    return jmax;
}


Pair<Enclosure,Enclosure>
Enclosure::split_first_order() const
{
    Matrix<FloatDPError> nonlinearities=Ariadne::nonlinearities_zeroth_order(this->_state_function,this->_reduced_domain);

    // Compute the row of the nonlinearities Array which has the highest norm
    // i.e. the highest sum of $mag(a_ij)$ where mag([l,u])=max(|l|,|u|)
    SizeType jmax_in_row_imax=nonlinearities.column_size();
    FloatDPError max_row_sum(0u,dp);
    for(SizeType i=0; i!=nonlinearities.row_size(); ++i) {
        SizeType jmax=nonlinearities.column_size();
        FloatDPError row_sum(0u,dp);
        FloatDPError max_mag_j_in_i(0u,dp);
        for(SizeType j=0; j!=nonlinearities.column_size(); ++j) {
            row_sum+=mag(nonlinearities[i][j]);
            if(mag(nonlinearities[i][j]).raw()>max_mag_j_in_i.raw()) {
                jmax=j;
                max_mag_j_in_i=mag(nonlinearities[i][j]);
            }
        }
        if(row_sum.raw()>max_row_sum.raw()) {
            max_row_sum=row_sum;
            jmax_in_row_imax=jmax;
        }
    }

    if(jmax_in_row_imax==nonlinearities.column_size()) { ARIADNE_THROW(std::runtime_error, "split_first_order", "No need to split"); }

    return this->split(jmax_in_row_imax);
}


Pair<Enclosure,Enclosure>
Enclosure::split() const
{
    return this->split_zeroth_order();
}

Pair<Enclosure,Enclosure>
Enclosure::split(SizeType d) const
{
    ARIADNE_PRECONDITION(d<this->number_of_parameters());
    ExactBoxType subdomain1,subdomain2;
    make_lpair(subdomain1,subdomain2)=Ariadne::split(this->_state_function.domain(),d);

    ValidatedVectorMultivariateFunctionPatch function1,function2;
    make_lpair(function1,function2)=Ariadne::split(this->_state_function,d);

    Pair<Enclosure,Enclosure>
    result=make_pair(Enclosure(function1.domain(),function1,this->configuration()),
                     Enclosure(function2.domain(),function2,this->configuration()));
    Enclosure& result1=result.first;
    Enclosure& result2=result.second;

    ValidatedScalarMultivariateFunctionPatch constraint_function1,constraint_function2;
    for(List<ValidatedConstraintModel>::ConstIterator iter=this->_constraints.begin();
        iter!=this->_constraints.end(); ++iter)
    {
        const ValidatedConstraintModel& constraint=*iter;
        make_lpair(constraint_function1,constraint_function2)=Ariadne::split(constraint.function(),d);
        result1._constraints.append(ValidatedConstraintModel(constraint.lower_bound(),constraint_function1,constraint.upper_bound()));
        result2._constraints.append(ValidatedConstraintModel(constraint.lower_bound(),constraint_function2,constraint.upper_bound()));
    }

    ValidatedScalarMultivariateFunctionPatch time_function1,time_function2;
    make_lpair(time_function1,time_function2)=Ariadne::split(this->_time_function,d);
    result1._time_function=time_function1;
    result2._time_function=time_function2;
    ValidatedScalarMultivariateFunctionPatch dwell_time_function1,dwell_time_function2;
    make_lpair(dwell_time_function1,dwell_time_function2)=Ariadne::split(this->_dwell_time_function,d);
    result1._dwell_time_function=dwell_time_function1;
    result2._dwell_time_function=dwell_time_function2;

    result1._auxiliary_mapping=this->_auxiliary_mapping;
    result2._auxiliary_mapping=this->_auxiliary_mapping;

    result1._check();
    result2._check();
    return result;
}






ValidatedConstrainedImageSet Enclosure::state_set() const
{
    return ValidatedConstrainedImageSet(this->domain(),this->state_function(),this->constraints());
}

ValidatedConstrainedImageSet Enclosure::state_auxiliary_set() const
{
    return ValidatedConstrainedImageSet(this->domain(),this->state_auxiliary_function(),this->constraints());
}

ValidatedConstrainedImageSet Enclosure::state_time_auxiliary_set() const
{
    return ValidatedConstrainedImageSet(this->domain(),this->state_time_auxiliary_function(),this->constraints());
}


Void Enclosure::adjoin_outer_approximation_to(Storage& storage, Nat fineness) const
{
    if (this->auxiliary_mapping().result_size()!=0 &&
            this->auxiliary_mapping().managed_pointer()==storage.auxiliary_mapping().managed_pointer())
    {
        ARIADNE_WARN("enclosure="<<*this<<", "
                    "enclosure.auxiliary_mapping()="<<this->auxiliary_mapping()<<", "
                    "storage.auxiliary_mapping()="<<storage.auxiliary_mapping());
    }
    this->configuration().paver().adjoin_outer_approximation(storage.state_set(),this->state_set(),fineness);
}


Storage Enclosure::outer_approximation(const Grid& grid, Nat fineness) const
{
    Storage paving(grid,this->auxiliary_mapping());
    this->adjoin_outer_approximation_to(paving,fineness);
    return paving;
}



TaylorModel<ValidatedTag,FloatDP> recondition(const TaylorModel<ValidatedTag,FloatDP>& tm, Array<SizeType>& discarded_variables, SizeType number_of_error_variables, SizeType index_of_error);
TaylorModel<ValidatedTag,FloatDP> recondition(const TaylorModel<ValidatedTag,FloatDP>& tm, Array<SizeType>& discarded_variables, SizeType number_of_error_variables);


Void
Enclosure::recondition()
{
    this->uniform_error_recondition();
    this->kuhn_recondition();
}


Void Enclosure::
uniform_error_recondition()
{
    const ExactDouble MAXIMUM_ERROR = cast_exact(std::numeric_limits<double>::epsilon() * 1024);
    SizeType old_number_of_parameters = this->number_of_parameters();

    List<SizeType> large_error_indices;

    for(SizeType i=0; i!=this->_state_function.result_size(); ++i) {
        ValidatedErrorNumber error=this->_state_function.get(i).error();
        if (possibly(error > MAXIMUM_ERROR)) {
            large_error_indices.append(i);
        }
    }

    if (large_error_indices.size() > 0) {

        ExactBoxType error_domains(large_error_indices.size());
        for(SizeType i=0; i!=large_error_indices.size(); ++i) {
            FloatDP error=cast_exact(this->_state_function.get(large_error_indices[i]).error().get(double_precision));
            error_domains[i]=ExactIntervalType(-error,+error);
        }
        error_domains=ExactBoxType(large_error_indices.size(),ExactIntervalType(-1,+1));
        SizeType k=this->number_of_parameters();

        this->_domain=product(this->_domain,error_domains);
        this->_reduced_domain=product(this->_reduced_domain,error_domains);
        this->_state_function=embed(this->_state_function,error_domains);
        for(SizeType i=0; i!=this->_constraints.size(); ++i) {
            this->_constraints[i].function()=embed(this->_constraints[i].function(),error_domains);
        }

        for(SizeType i=0; i!=large_error_indices.size(); ++i) {
            FloatDP error=cast_exact(this->_state_function.get(large_error_indices[i]).error().get(double_precision));
            if (possibly(error > MAXIMUM_ERROR)) {
                this->_state_function[large_error_indices[i]].clobber();
                this->_state_function[large_error_indices[i]] = this->_state_function.get(large_error_indices[i]) + this->configuration().function_factory().create_coordinate(this->_domain,k)*FloatDPBounds(+error);
                ++k;
            }
        }

        ExactBoxType new_variables = project(this->parameter_domain(),range(old_number_of_parameters,this->number_of_parameters()));
        this->_time_function = embed(this->_time_function,new_variables);
        this->_dwell_time_function = embed(this->_dwell_time_function,new_variables);
    }
    SizeType number_of_extra_parameters=this->number_of_parameters()-old_number_of_parameters;
    this->_variable_kinds.concatenate(List<EnclosureVariableKind>(number_of_extra_parameters,EnclosureVariableKind::ERROR));

    this->_check();
}

TaylorModel<ValidatedTag,FloatDP> recondition(const TaylorModel<ValidatedTag,FloatDP>& tm, Array<SizeType>& discarded_variables, SizeType number_of_error_variables, SizeType index_of_error)
{
    for(SizeType i=0; i!=discarded_variables.size()-1; ++i) {
        ARIADNE_PRECONDITION(discarded_variables[i]<discarded_variables[i+1]);
    }
    ARIADNE_PRECONDITION(discarded_variables[discarded_variables.size()-1]<tm.argument_size());
    ARIADNE_PRECONDITION(index_of_error<=number_of_error_variables);

    const SizeType number_of_variables = tm.argument_size();
    const SizeType number_of_discarded_variables = discarded_variables.size();
    const SizeType number_of_kept_variables = number_of_variables - number_of_discarded_variables;

    // Make an Array of the variables to be kept
    Array<SizeType> kept_variables=complement(number_of_variables,discarded_variables);

    // Construct result and reserve memory
    TaylorModel<ValidatedTag,FloatDP> r(number_of_kept_variables+number_of_error_variables,tm.sweeper());
    r.expansion().reserve(tm.number_of_nonzeros()+1u);
    MultiIndex ra(number_of_kept_variables+number_of_error_variables);

    // Set the uniform error of the original model
    // If index_of_error == number_of_error_variables, then the error is kept as a uniform error bound
    FloatDPError* error_ptr;
    if(number_of_error_variables==index_of_error) {
        error_ptr = &r.error();
    } else {
        ra[number_of_kept_variables+index_of_error]=1;
        r.expansion().append(ra,FloatDP(dp));
        ra[number_of_kept_variables+index_of_error]=0;
        error_ptr = reinterpret_cast<FloatDPError*>(&r.begin()->coefficient());
    }
    FloatDPError& error=*error_ptr;
    error += tm.error();

    for(TaylorModel<ValidatedTag,FloatDP>::ConstIterator iter=tm.begin(); iter!=tm.end(); ++iter) {
        UniformConstReference<MultiIndex> xa=iter->index();
        UniformConstReference<FloatDP> xv=iter->coefficient();
        Bool keep=true;
        for(SizeType k=0; k!=number_of_discarded_variables; ++k) {
            if(xa[discarded_variables[k]]!=0) {
                error += mag(xv);
                keep=false;
                break;
            }
        }
        if(keep) {
            for(SizeType k=0; k!=number_of_kept_variables; ++k) {
                ra[k]=xa[kept_variables[k]];
            }
            r.expansion().append(ra,xv);
        }
    }
    return r;
}

TaylorModel<ValidatedTag,FloatDP> recondition(const TaylorModel<ValidatedTag,FloatDP>& tm, Array<SizeType>& discarded_variables, SizeType number_of_error_variables) {
    return recondition(tm,discarded_variables,number_of_error_variables,number_of_error_variables);
}


Void
Enclosure::kuhn_recondition()
{
    if(!dynamic_cast<const ValidatedVectorMultivariateTaylorFunctionModelDP*>(&this->state_function().reference())) {
        ARIADNE_WARN("Cannot Kuhn reduce an Enclosure which is not given by TaylorFunctions.");
    }

    const SizeType number_of_kept_parameters = (this->configuration().reconditioning_num_blocks()-1)*this->state_dimension();
    const SizeType number_of_discarded_parameters=this->number_of_parameters()-number_of_kept_parameters;
    const SizeType number_of_error_parameters = this->state_dimension();

    if(this->number_of_parameters()<=number_of_kept_parameters) {
        this->uniform_error_recondition();
        return;
    }

    const ValidatedVectorMultivariateTaylorFunctionModelDP& function=dynamic_cast<const ValidatedVectorMultivariateTaylorFunctionModelDP&>(this->state_function().reference());
    const Vector<ValidatedTaylorModelDP>& models = function.models();
    Matrix<FloatDPApproximation> dependencies(this->state_dimension(),this->number_of_parameters(),dp);
    for(SizeType i=0; i!=dependencies.row_size(); ++i) {
        for(ValidatedTaylorModelDP::ConstIterator iter=models[i].begin(); iter!=models[i].end(); ++iter) {
            for(SizeType j=0; j!=dependencies.column_size(); ++j) {
                if(iter->index()[j]!=0) {
                    dependencies[i][j]=dependencies[i][j]+abs(iter->coefficient());
                }
            }
        }
    }
    Array< Pair<FloatDP,SizeType> > column_max_dependencies(this->number_of_parameters(),make_pair(FloatDP(dp),0u));
    for(SizeType j=0; j!=dependencies.column_size(); ++j) {
        column_max_dependencies[j] = make_pair(FloatDP(0.0_x,dp),SizeType(j));
        for(SizeType i=0; i!=dependencies.row_size(); ++i) {
            column_max_dependencies[j].first=max(column_max_dependencies[j].first,cast_exact(dependencies[i][j]));
        }
    }
    std::sort(column_max_dependencies.begin(),column_max_dependencies.end(),std::greater< Pair<FloatDP,SizeType> >());

    Array<SizeType> kept_parameters(number_of_kept_parameters);
    Array<SizeType> discarded_parameters(number_of_discarded_parameters);
    for(SizeType j=0; j!=number_of_kept_parameters; ++j) { kept_parameters[j]=column_max_dependencies[j].second; }
    for(SizeType j=0; j!=number_of_discarded_parameters; ++j) { discarded_parameters[j]=column_max_dependencies[number_of_kept_parameters+j].second; }
    std::sort(kept_parameters.begin(),kept_parameters.end());
    std::sort(discarded_parameters.begin(),discarded_parameters.end());

    Vector<ValidatedTaylorModelDP> new_models(models.size(),ValidatedTaylorModelDP(number_of_kept_parameters+number_of_error_parameters,function.properties()));
    for(SizeType i=0; i!=this->state_dimension(); ++i) {
        new_models[i] = Ariadne::recondition(models[i],discarded_parameters,number_of_error_parameters,i);
    }

    ExactBoxType new_domain(number_of_kept_parameters+number_of_error_parameters);
    ExactBoxType new_reduced_domain(number_of_kept_parameters+number_of_error_parameters);
    for(SizeType j=0; j!=number_of_kept_parameters; ++j) {
        new_domain[j]=this->parameter_domain()[kept_parameters[j]];
        new_reduced_domain[j]=this->reduced_domain()[kept_parameters[j]];
    }
    for(SizeType j=number_of_kept_parameters; j!=number_of_kept_parameters+number_of_error_parameters; ++j) {
        new_domain[j]=ExactIntervalType(-1,+1);
        new_reduced_domain[j]=ExactIntervalType(-1,+1);
    }
    this->_domain = new_domain;
    this->_reduced_domain = new_reduced_domain;

    Enclosure new_set(new_domain,ValidatedVectorMultivariateTaylorFunctionModelDP(new_domain,new_models),this->configuration());
    for(SizeType i=0; i!=this->_constraints.size(); ++i) {
        const ValidatedConstraintModel& constraint=this->_constraints[i];
        ValidatedScalarMultivariateTaylorFunctionModelDP const& constraint_function=dynamic_cast<const ValidatedScalarMultivariateTaylorFunctionModelDP&>(constraint.function().reference());
        ValidatedScalarMultivariateFunctionPatch new_constraint_function=ValidatedScalarMultivariateTaylorFunctionModelDP(new_domain,Ariadne::recondition(constraint_function.model(),discarded_parameters,number_of_error_parameters));
        new_set._constraints.append(ValidatedConstraintModel(constraint.lower_bound(),new_constraint_function,constraint.upper_bound()));
    }
    ValidatedScalarMultivariateTaylorFunctionModelDP const& time=dynamic_cast<const ValidatedScalarMultivariateTaylorFunctionModelDP&>(this->_time_function.reference());
    new_set._time_function=ValidatedScalarMultivariateTaylorFunctionModelDP(new_domain,Ariadne::recondition(time.model(),discarded_parameters,number_of_error_parameters));
    ValidatedScalarMultivariateTaylorFunctionModelDP const& dwell_time=dynamic_cast<const ValidatedScalarMultivariateTaylorFunctionModelDP&>(this->_dwell_time_function.reference());
    new_set._dwell_time_function=ValidatedScalarMultivariateTaylorFunctionModelDP(new_domain,Ariadne::recondition(dwell_time.model(),discarded_parameters,number_of_error_parameters));

    new_set._auxiliary_mapping=this->_auxiliary_mapping;

    (*this)=new_set;

    this->_check();
}


Void Enclosure::restrict(const ExactBoxType& subdomain)
{
    ARIADNE_ASSERT_MSG(subdomain.size()==this->number_of_parameters(),"set="<<*this<<", subdomain="<<subdomain);
    ARIADNE_ASSERT_MSG(Ariadne::subset(subdomain,this->domain()),"set.domain()="<<this->domain()<<", subdomain="<<subdomain);
    Enclosure& result(*this);
    result._domain=subdomain;
    result._reduced_domain=Ariadne::intersection(static_cast<const ExactBoxType&>(result._reduced_domain),subdomain);
    result._state_function=restriction(result._state_function,subdomain);
    result._time_function=restriction(result._time_function,subdomain);
    result._dwell_time_function=restriction(result._dwell_time_function,subdomain);
    ValidatedScalarMultivariateFunctionPatch new_constraint;
    for(List<ValidatedConstraintModel>::Iterator iter=result._constraints.begin();
        iter!=result._constraints.end(); ++iter)
    {
        ValidatedScalarMultivariateFunctionPatch& constraint_function=iter->function();
        constraint_function=restriction(constraint_function,subdomain);
    }
    this->reduce();
}

Enclosure restriction(Enclosure const& encl, const ExactBoxType& subdomain)
{
    Enclosure result(encl);
    result.restrict(subdomain);
    return result;
}


ValidatedScalarMultivariateFunctionPatch const Enclosure::get_function(SizeType i) const {
    if(i<this->state_dimension()) { return this->_state_function[i]; }
    else if (i==this->state_dimension()) { return this->_time_function; }
    else { return compose(this->_auxiliary_mapping[i-this->state_dimension()-1u],this->_state_function); }
}

inline ValidatedVectorMultivariateFunctionPatch join(const ValidatedVectorMultivariateFunctionPatch& f1, const ValidatedScalarMultivariateFunctionPatch& f2, const ValidatedVectorMultivariateFunctionPatch& f3) {
    return join(join(f1,f2),f3);
}

Void Enclosure::draw(CanvasInterface& canvas, const Projection2d& projection) const {
    GraphicsManager::instance().drawer().draw(canvas,projection,this->state_time_auxiliary_set());
}

template<class K, class V> Map<K,V> filter(const Map<K,V>& m, const Set<K>& s) {
    Map<K,V> r;
    for(typename Set<K>::ConstIterator iter=s.begin(); iter!=s.end(); ++iter) {
        r.insert(*m.find(*iter));
    }
    return r;
}


inline const ValidatedScalarMultivariateFunctionPatch& repr(const ValidatedScalarMultivariateFunctionPatch& f) { return f; }
inline const ValidatedVectorMultivariateFunctionPatch& repr(const ValidatedVectorMultivariateFunctionPatch& f) { return f; }
inline const List<ValidatedScalarMultivariateFunctionPatch>& repr(const List<ValidatedScalarMultivariateFunctionPatch>& f) { return f; }

OutputStream& Enclosure::_write(OutputStream& os) const {
    const Bool LONG_FORMAT=true;

    // TODO: improve by using writers
    if(LONG_FORMAT) {
        os << "Enclosure"
           << "(\n  domain=" << this->domain()
           << ",\n  range=" << this->bounding_box()
           << ",\n  reduced_domain=" << this->reduced_domain()
           << ",\n  is_empty=" << this->reduced_domain().is_empty()
           << ",\n  state_function=" << this->state_function()
           << ",\n  time_function=" << this->time_function()
           << ",\n  constraints=" << this->constraints()
           << ",\n  auxiliary_mapping = [" << this->auxiliary_mapping().argument_size() << "]" << this->auxiliary_mapping()
           << ",\n  parameters = " << canonical_variable_names(this->variable_kinds())
           << "\n)\n";
    } else {
        os << "Enclosure"
           << "( domain=" << this->domain()
           << ", range=" << this->bounding_box()
           << ", state_function=" << repr(this->state_function())
           << ", time_function=" << repr(this->time_function())
           << ", constraints=" << this->constraints()
           << ", auxiliary_mapping = " << this->auxiliary_mapping()
           << ")";

    } return os;
}


Enclosure product(const Enclosure& set, const ExactIntervalType& ivl) {
    typedef List<ValidatedConstraintModel>::ConstIterator ConstIterator;

    ValidatedVectorMultivariateFunctionPatch new_function=combine(set.state_function(),set.configuration().function_factory().create_coordinate({ivl},0));

    Enclosure result(new_function.domain(),new_function,set.configuration());
    for(ConstIterator iter=set._constraints.begin(); iter!=set._constraints.end(); ++iter) {
        result._constraints.append(ValidatedConstraintModel(iter->lower_bound(),embed(iter->function(),ivl),iter->upper_bound()));
    }
    result._time_function=embed(set._time_function,ivl);
    result._dwell_time_function=embed(set._dwell_time_function,ivl);

    result._check();

    return result;
}

Enclosure product(const Enclosure& set, const ExactBoxType& bx) {
    typedef List<ValidatedConstraintModel>::ConstIterator ConstIterator;

    ValidatedVectorMultivariateFunctionPatch new_function=combine(set.state_function(),set.configuration().function_factory().create_identity(bx));

    Enclosure result(new_function.domain(),new_function,set.configuration());
    for(ConstIterator iter=set._constraints.begin(); iter!=set._constraints.end(); ++iter) {
        result._constraints.append(ValidatedConstraintModel(iter->lower_bound(),embed(iter->function(),bx),iter->upper_bound()));
    }
    result._time_function=embed(set._time_function,bx);
    result._dwell_time_function=embed(set._dwell_time_function,bx);

    result._check();

    return result;
}

Enclosure product(const Enclosure& set1, const Enclosure& set2) {
    ARIADNE_ASSERT(same(set1.time_function().range(),set2.time_function().range()));
    ARIADNE_ASSERT(same(set1.dwell_time_function().range(),set2.dwell_time_function().range()));

    typedef List<ValidatedConstraintModel>::ConstIterator ConstIterator;

    ValidatedVectorMultivariateFunctionPatch new_state_function=combine(set1.state_function(),set2.state_function());

    Enclosure result(new_state_function.domain(),new_state_function,set1.configuration());
    for(ConstIterator iter=set1._constraints.begin(); iter!=set1._constraints.end(); ++iter) {
        result._constraints.append(ValidatedConstraintModel(iter->lower_bound(),embed(iter->function(),set2.domain()),iter->upper_bound()));
    }
    for(ConstIterator iter=set2._constraints.begin(); iter!=set2._constraints.end(); ++iter) {
        result._constraints.append(ValidatedConstraintModel(iter->lower_bound(),embed(set1.domain(),iter->function()),iter->upper_bound()));
    }
    result._time_function=embed(set1.time_function(),set2.time_function().domain());
    result._dwell_time_function=embed(set1.dwell_time_function(),set2.dwell_time_function().domain());

    result._check();

    return result;
}

Enclosure apply(const ValidatedVectorMultivariateFunction& function, const Enclosure& set) {
    Enclosure result(set);
    result.apply_map(function);
    return result;
}

Enclosure unchecked_apply(const ValidatedVectorMultivariateFunctionPatch& function, const Enclosure& set) {
    Enclosure result(set);
    const ValidatedVectorMultivariateFunctionPatch& state_function=result.state_function();
    const_cast<ValidatedVectorMultivariateFunctionPatch&>(state_function)=unchecked_compose(function,set.state_function());
    return result;
}



LabelledEnclosure::LabelledEnclosure(LabelledExactBoxType const& bx, EnclosureConfiguration const& config)
    : Enclosure(bx.euclidean_set(),config), _state_variables(bx.space().variable_names())
{
}

LabelledEnclosure::LabelledEnclosure(ExactBoxType const& bx, RealSpace const& state_space, EnclosureConfiguration const& config)
    : Enclosure(bx,config), _state_variables(state_space.variable_names())
{
}

LabelledEnclosure::LabelledEnclosure(RealBox const& bx, RealSpace const& state_space, EnclosureConfiguration const& config)
    : Enclosure(bx,config), _state_variables(state_space.variable_names())
{
}

LabelledEnclosure::LabelledEnclosure(RealVariablesBox const& bx, RealSpace const& state_space, EnclosureConfiguration const& config)
    : Enclosure(bx.euclidean_set(state_space),config), _state_variables(state_space.variable_names())
{
}

LabelledEnclosure::LabelledEnclosure(BoundedConstraintSet const& set, RealSpace const& state_space, EnclosureConfiguration const& config)
    : Enclosure(set,config), _state_variables(state_space.variable_names())
{
}

LabelledEnclosure::LabelledEnclosure(Enclosure const& set, RealSpace const& state_space)
    : Enclosure(set), _state_variables(state_space.variable_names())
{
}

LabelledEnclosure::LabelledEnclosure(Enclosure const& set, RealSpace const& state_space, RealSpace const& auxiliary_space)
    : Enclosure(set), _state_variables(state_space.variable_names()), _auxiliary_variables(auxiliary_space.variable_names())
{
}

LabelledEnclosure* LabelledEnclosure::clone() const {
    return new LabelledEnclosure(*this);
}


Void LabelledEnclosure::set_state_space(RealSpace const& space) {
    ARIADNE_PRECONDITION(this->state_function().result_size()==space.dimension());
    this->_state_variables=space.variable_names();
}

const RealSpace LabelledEnclosure::state_space() const {
    return RealSpace(this->_state_variables);
}

const RealVariable LabelledEnclosure::time_variable() const
{
    return TimeVariable();
}

const RealSpace LabelledEnclosure::auxiliary_space() const {
    return RealSpace(this->_auxiliary_variables);
}

const RealSpace LabelledEnclosure::state_auxiliary_space() const {
    return join(this->state_space(),this->auxiliary_space());
}

const RealSpace LabelledEnclosure::state_time_auxiliary_space() const
{
    auto state_time_space=(this->state_space().contains(this->time_variable()) ? this->state_space() : join(this->state_space(),this->time_variable()));
    return join(state_time_space,this->auxiliary_space());
}


const RealSpace LabelledEnclosure::space() const {
    return this->state_time_auxiliary_space();
}


Void LabelledEnclosure::set_auxiliary(const RealSpace& spc, const EffectiveVectorMultivariateFunction& aux) {
    ARIADNE_PRECONDITION(this->state_function().result_size()==aux.argument_size());
    ARIADNE_PRECONDITION(spc.size()==aux.result_size());
    this->Enclosure::set_auxiliary_mapping(aux);
    this->_auxiliary_variables=spc.variable_names();
}


Pair<LabelledEnclosure,LabelledEnclosure> LabelledEnclosure::split() const {
    Pair<Enclosure,Enclosure> split_enclosures = this->Enclosure::split();
    return make_pair(LabelledEnclosure(split_enclosures.first,this->state_space(),this->auxiliary_space()),
                     LabelledEnclosure(split_enclosures.second,this->state_space(),this->auxiliary_space()));
}

LabelledEnclosure product(const LabelledEnclosure& set1, const ExactIntervalType& ivl2) {
    std::stringstream strstr;
    strstr << "x" << to_str(set1.dimension());
    return product(set1,LabelledExactIntervalType(RealVariable(strstr.str())  ,ivl2));
}

LabelledEnclosure product(const LabelledEnclosure& set1, const LabelledExactIntervalType& ivl2) {
    return LabelledEnclosure(product(set1.euclidean_set(),ivl2.interval()),
                             join(set1.state_space(),ivl2.variable()),
                             set1.auxiliary_space());
}

LabelledEnclosure product(const LabelledEnclosure& set1, const LabelledExactBoxType& bx2) {
    return LabelledEnclosure(product(set1.euclidean_set(),bx2.euclidean_set()),
                             join(set1.state_space(),bx2.space()),
                             set1.auxiliary_space());
}

LabelledEnclosure product(const LabelledEnclosure& set1, const LabelledEnclosure& set2) {
    return LabelledEnclosure(product(static_cast<Enclosure const&>(set1),static_cast<Enclosure const&>(set2)),
                             join(set1.state_space(),set2.state_space()),join(set1.auxiliary_space(),set2.auxiliary_space()));
}

Void LabelledEnclosure::apply_full_reach_step(ValidatedVectorMultivariateFunctionPatch phi) {
    this->Enclosure::apply_full_reach_step(phi);
}

Void LabelledEnclosure::apply_map(ValidatedVectorMultivariateFunction const& f) {
    this->Enclosure::apply_map(f);
}

Void LabelledEnclosure::apply_map(ValidatedVectorMultivariateFunction const& f, RealSpace const& spc) {
    ARIADNE_PRECONDITION(spc.size()==f.result_size());
    this->Enclosure::apply_map(f);
    this->_state_variables=spc.variable_names();
}

Void LabelledEnclosure::apply_map(ValidatedVectorMultivariateFunction const& f, RealSpace const& spc,
                                  EffectiveVectorMultivariateFunction const& aux_f, RealSpace const& aux_spc) {
    ARIADNE_PRECONDITION(spc.size()==aux_f.argument_size());
    ARIADNE_PRECONDITION(aux_spc.size()==aux_f.result_size());
    this->Enclosure::apply_map(f,aux_f);
    this->_state_variables=spc.variable_names();
    this->_auxiliary_variables=aux_spc.variable_names();
}

Void LabelledEnclosure::draw(CanvasInterface& canvas, const Variables2d& axes) const
{
    Projection2d proj=projection(this->state_time_auxiliary_space(),axes);
    this->euclidean_set().draw(canvas,proj);
}


OutputStream& LabelledEnclosure::_write(OutputStream& os) const {
    const Bool LONG_FORMAT=true;

    // TODO: improve by using writers
    if(LONG_FORMAT) {
        os << "LabelledEnclosure"
           << "(\n  domain=" << this->domain()
           << ",\n  range=" << this->bounding_box()
           << ",\n  reduced_domain=" << this->reduced_domain()
           << ",\n  is_empty=" << this->reduced_domain().is_empty()
           << ",\n  state_function=" << this->state_function()
           << ",\n  time_function=" << this->time_function()
           << ",\n  constraints=" << this->constraints()
           << ",\n  auxiliary_mapping = [" << this->auxiliary_mapping().argument_size() << "]" << this->auxiliary_mapping()
           << ",\n  parameters = " << canonical_variable_names(this->variable_kinds())
           << ",\n  space =" << this->space()
           << ",\n  auxiliary_space =" << this->auxiliary_space()
           << "\n)\n";
    } else {
        os << "LabelledEnclosure"
           << "( domain=" << this->domain()
           << ", range=" << this->bounding_box()
           << ", state_function=" << repr(this->state_function())
           << ", time_function=" << repr(this->time_function())
           << ", constraints=" << this->constraints()
           << ", auxiliary_mapping = " << this->auxiliary_mapping()
           << ", space =" << this->space()
           << ")";

    } return os;
}




LabelledStorage::LabelledStorage(Grid const& grid,
                                 RealSpace const& state_space,
                                 EffectiveVectorMultivariateFunction const& auxiliary_mapping,
                                 RealSpace const& auxiliary_space)
    : LabelledStorage(GridTreePaving(grid), state_space, auxiliary_mapping, auxiliary_space) { }

LabelledStorage::LabelledStorage(GridTreePaving const& paving,
                                 RealSpace const& state_space,
                                 EffectiveVectorMultivariateFunction const& auxiliary_mapping,
                                 RealSpace const& auxiliary_space)
    : Storage(paving,auxiliary_mapping)
    , _state_variables(state_space.variable_names())
    , _auxiliary_variables(auxiliary_space.variable_names())
{ }


LabelledGridTreePaving inner_approximation(EffectiveEuclideanSetInterface const& set, LabelledGrid const& grid, Nat fineness) {
    LabelledGridTreePaving paving(grid); paving.euclidean_set().adjoin_inner_approximation(set,fineness); return paving;
}

const RealSpace LabelledStorage::state_space() const {
    return RealSpace(this->_state_variables);
}

const RealSpace LabelledStorage::auxiliary_space() const {
    return RealSpace(this->_auxiliary_variables);
}

const RealSpace LabelledStorage::state_auxiliary_space() const {
    return join(this->state_space(),this->auxiliary_space());
}

const LabelledGrid LabelledStorage::grid() const {
    return LabelledGrid(this->euclidean_set().grid(),this->state_space());
}

const Mapping LabelledStorage::auxiliary_function() const {
    return this->Storage::auxiliary_mapping();
}

const LabelledMapping LabelledStorage::auxiliary_mapping() const {
    return LabelledMapping(this->state_space(), this->auxiliary_function(), this->auxiliary_space());
}

const LabelledMapping LabelledStorage::auxiliary_data() const {
    return this->auxiliary_mapping();
}

Void LabelledStorage::draw(CanvasInterface& canvas, const Variables2d& axes) const
{
    Projection2d proj=projection(this->state_auxiliary_space(),axes);
    this->euclidean_set().draw(canvas,proj);
}

LabelledUpperBoxType LabelledEnclosure::bounding_box() const {
    return LabelledBox(this->state_space(), this->euclidean_set().bounding_box());
}

LabelledUpperBoxType
ListSet<LabelledEnclosure>::bounding_box() const
{
    auto iter=this->begin();
    auto box = iter->state_auxiliary_set().bounding_box();
    RealSpace space = join(iter->state_space(),iter->auxiliary_space());
    ++iter;
    for(; iter!=this->end(); ++iter) {
        box = hull(box,iter->state_auxiliary_set().bounding_box());
    }
    return LabelledUpperBoxType(space,box);
}

const ListSet<LabelledSet<UpperBoxType>> ListSet<LabelledEnclosure>::bounding_boxes() const {
    ListSet<LabelledSet<UpperBoxType>> boxes;
    for(SizeType i=0; i!=this->size(); ++i) {
        boxes.adjoin(LabelledSet((*this)[i].state_space(),(*this)[i].euclidean_set().bounding_box()));
    }
    return boxes;
}

ListSet<LabelledEnclosure>* ListSet<LabelledEnclosure>::clone() const {
    return new ListSet<LabelledEnclosure>(*this);
}

Void ListSet<LabelledEnclosure>::draw(CanvasInterface& cnvs, const Variables2d& prj) const {
    StaticWorkload<LabelledEnclosure,CanvasInterface*,Variables2d const&> workload(
            [](LabelledEnclosure const& e, CanvasInterface* c, Variables2d const& p){ e.draw(*c, p); },&cnvs,prj);
    workload.append(this->_data).process();
}

} // namespace Ariadne


