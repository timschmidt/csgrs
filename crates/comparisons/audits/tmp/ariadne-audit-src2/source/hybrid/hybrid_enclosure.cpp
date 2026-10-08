/***************************************************************************
 *            hybrid/hybrid_enclosure.cpp
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

#include "function/functional.hpp"
#include "config.hpp"

#include "numeric/numeric.hpp"
#include "algebra/vector.hpp"
#include "algebra/algebra.hpp"
#include "numeric/operators.hpp"
#include "function/function.hpp"
#include "function/procedure.hpp"
#include "function/constraint.hpp"
#include "function/polynomial.hpp"
#include "function/taylor_function.hpp"
#include "geometry/box.hpp"
#include "geometry/grid_paving.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "hybrid/hybrid_storage.hpp"
#include "hybrid/hybrid_time.hpp"
#include "hybrid/discrete_event.hpp"
#include "hybrid/discrete_location.hpp"

#include "solvers/linear_programming.hpp"
#include "solvers/nonlinear_programming.hpp"
#include "solvers/constraint_solver.hpp"
#include "geometry/affine_set.hpp"

#include "io/figure.hpp"
#include "hybrid/hybrid_enclosure.hpp"
#include "hybrid/hybrid_expression_set.hpp"

#include "betterthreads/workload.hpp"

namespace Ariadne {

using BetterThreads::StaticWorkload;

OutputStream& operator<<(OutputStream& os, ValidatedConstraint const& c);
OutputStream& operator<<(OutputStream& os, List<ValidatedConstraint> const& c);

template<class T> List<T> catenate(List<T> lst1, T const& val2, List<T> const& lst3) {
    lst1.append(val2); return catenate(lst1,lst3); }

//-------------- HybridEnclosure -----------------------------------------//

HybridEnclosure::~HybridEnclosure() {
}

HybridEnclosure::HybridEnclosure()
    : _location(), _events(), _set()
{
}

HybridEnclosure::HybridEnclosure(EnclosureConfiguration const& configuration)
    : _location(), _events(), _set(configuration)
{
}

HybridEnclosure::HybridEnclosure(const HybridBoxSet& hbox,
                                 const RealSpace& state_space,
                                 const EnclosureConfiguration& configuration)
    : HybridEnclosure(hbox.location(),state_space,hbox.euclidean_set(state_space),configuration)
{
}

HybridEnclosure::HybridEnclosure(const HybridBoundedConstraintSet& hybrid_set,
                                 const RealSpace& state_space,
                                 const EnclosureConfiguration& configuration)
    : _location(hybrid_set.location()), _events(), _set()
{
    BoundedConstraintSet euclidean_set=hybrid_set.euclidean_set(this->_location,state_space);
    this->_set=LabelledEnclosure(euclidean_set,state_space,configuration);
}


HybridEnclosure::HybridEnclosure(const DiscreteLocation& location, const RealSpace& state_space,
                                 const RealBox& box, const EnclosureConfiguration& configuration)
    : _location(location), _events(), _set(box,state_space,configuration)
{
}

HybridEnclosure::HybridEnclosure(const HybridRealBox& hbox, const EnclosureConfiguration& configuration)
    : HybridEnclosure(hbox.location(),hbox.space(),hbox.euclidean_set(),configuration)
{
}

HybridEnclosure::HybridEnclosure(const HybridExactBoxType& hbox, const EnclosureConfiguration& configuration)
    : HybridEnclosure(hbox.location(),LabelledEnclosure(hbox.euclidean_set(),hbox.space(),configuration))
{
}


HybridEnclosure::HybridEnclosure(const DiscreteLocation& location, const RealSpace& spc, const Enclosure& set)
    : _location(location), _events(), _set(set,spc)
{
}

HybridEnclosure::HybridEnclosure(const DiscreteLocation& location, const LabelledEnclosure& set)
    : _location(location), _events(), _set(set)
{
}


HybridEnclosure* HybridEnclosure::clone() const {
    return new HybridEnclosure(*this);
}

const RealSpace
HybridEnclosure::state_auxiliary_space() const
{
    return join(this->state_space(),this->auxiliary_space());
}

const RealSpace
HybridEnclosure::state_time_auxiliary_space() const
{
    auto state_time_space=(this->state_space().contains(this->time_variable()) ? this->state_space() : join(this->state_space(),this->time_variable()));
    return join(state_time_space,this->auxiliary_space());
}

const RealSpace
HybridEnclosure::state_space() const
{
    return this->_set.state_space();
}

const RealVariable
HybridEnclosure::time_variable() const
{
    return TimeVariable();
}

const RealSpace
HybridEnclosure::auxiliary_space() const
{
    return this->_set.auxiliary_space();
}

List<DiscreteEvent> const&
HybridEnclosure::previous_events() const
{
    return this->_events;
}

EnclosureConfiguration const&
HybridEnclosure::configuration() const
{
    return this->_set.configuration();
}

ValidatedScalarMultivariateFunctionPatch const
HybridEnclosure::function(RealVariable var) const
{
    if(this->state_space().contains(var)) {
        return this->state_function()[this->state_space().index(var)];
    } else if(this->auxiliary_space().contains(var)) {
        return this->auxiliary_function()[this->auxiliary_space().index(var)];
    } else if(TimeVariable()==var) {
        return this->time_function();
    } else {
        ARIADNE_THROW(std::runtime_error,"HybridEnclosure::function(RealVariable var) const",
                      "Variable "<<var<<" is not defined by HybridEnclosure with variables "<<this->state_time_auxiliary_space());
    }
}

ValidatedVectorMultivariateFunctionPatch const&
HybridEnclosure::state_function() const
{
    return this->_set.state_function();
}

ValidatedScalarMultivariateFunctionPatch const&
HybridEnclosure::time_function() const
{
    return this->_set.time_function();
}

ValidatedScalarMultivariateFunctionPatch const&
HybridEnclosure::dwell_time_function() const
{
    return this->_set.dwell_time_function();
}

ValidatedVectorMultivariateFunctionPatch const
HybridEnclosure::auxiliary_function() const
{
    return this->_set.auxiliary_function();
}

ValidatedVectorMultivariateFunctionPatch const
HybridEnclosure::state_auxiliary_function() const
{
    return this->_set.state_auxiliary_function();
}

ValidatedVectorMultivariateFunctionPatch const
HybridEnclosure::state_time_auxiliary_function() const
{
    return this->_set.state_time_auxiliary_function();
}


Void HybridEnclosure::set_time_function(const ValidatedScalarMultivariateFunctionPatch& time_function)
{
    ARIADNE_NOT_IMPLEMENTED;
    ARIADNE_ASSERT_MSG(Ariadne::subset(this->parameter_domain(),time_function.domain()),
                       "Domain of "<<time_function<<" does not contain parameter domain "<<this->parameter_domain());
    ARIADNE_ASSERT_MSG(this->parameter_domain()==time_function.domain(),
                       "Domain of "<<time_function<<" does not equal parameter domain "<<this->parameter_domain());
    //this->_set._time_function=time_function;
}

UpperBoxType
HybridEnclosure::state_bounding_box() const
{
    return cast_exact_box(this->_set.state_set().bounding_box());
}

UpperIntervalType
HybridEnclosure::range_of(EffectiveScalarMultivariateFunction const& g) const
{
    return apply(compose(g,this->state_function()),this->_set.reduced_domain());
}

UpperIntervalType
HybridEnclosure::time_range() const
{
    return apply(this->time_function(),this->_set.reduced_domain());
}

UpperIntervalType
HybridEnclosure::dwell_time_range() const
{
    return apply(this->dwell_time_function(),this->_set.reduced_domain());
}

SizeType
HybridEnclosure::number_of_constraints() const
{
    return this->_set.number_of_constraints();
}

SizeType
HybridEnclosure::number_of_parameters() const
{
    return this->_set.number_of_parameters();
}

const ExactBoxType
HybridEnclosure::parameter_domain() const
{
    return this->_set.domain();
}

HybridUpperBoxType
HybridEnclosure::bounding_box() const
{
    return HybridUpperBoxType(this->location(),this->state_space(),cast_exact_box(this->state_bounding_box()));
}


Void HybridEnclosure::set_auxiliary(List<RealVariable> vars, EffectiveVectorMultivariateFunction aux)
{
    ARIADNE_ASSERT(this->state_space().size()==aux.argument_size());
    ARIADNE_ASSERT(vars.size()==aux.result_size());
    this->_set.set_auxiliary(vars,aux);
}

Void HybridEnclosure::new_state_time_bound(DiscreteEvent e, ValidatedScalarMultivariateFunction gamma) {
    this->_set.new_state_time_bound(gamma);
}

Void HybridEnclosure::new_invariant(DiscreteEvent event, ValidatedScalarMultivariateFunction constraint_function) {
    this->_set.new_negative_state_constraint(constraint_function);
}

Void HybridEnclosure::new_activation(DiscreteEvent event, ValidatedScalarMultivariateFunction constraint_function) {
    this->_set.new_positive_state_constraint(constraint_function);
}

Void HybridEnclosure::new_guard(DiscreteEvent event, ValidatedScalarMultivariateFunction constraint_function) {
    this->_set.new_zero_state_constraint(constraint_function);
}

Void HybridEnclosure::new_parameter_constraint(DiscreteEvent event, ValidatedConstraint constraint) {
    this->_set.new_parameter_constraint(constraint);
}

Void HybridEnclosure::new_state_constraint(DiscreteEvent event, ValidatedConstraint constraint) {
    this->_set.new_state_constraint(constraint);
}

Void HybridEnclosure::new_state_time_constraint(DiscreteEvent event, ValidatedConstraint constraint) {
    this->_set.new_state_time_constraint(constraint);
}

Void HybridEnclosure::clear_time()
{
    this->_set.clear_time();
}

Void HybridEnclosure::clear_events()
{
    this->_events.clear();
}

Void HybridEnclosure::apply_reset(DiscreteEvent event, DiscreteLocation target, RealSpace state_space, const ValidatedVectorMultivariateFunction& map, RealSpace auxiliary_space, const EffectiveVectorMultivariateFunction& auxiliary_mapping)
{
    ARIADNE_ASSERT_MSG(map.argument_size()==this->state_dimension(),"*this="<<*this<<", event="<<event<<", state_space="<<state_space<<", target="<<target<<", map="<<map);
    this->_events.append(event);
    this->_location=target;
    this->_set.apply_map(map,state_space,auxiliary_mapping,auxiliary_space);
}

Void HybridEnclosure::apply_fixed_evolve_step(const ValidatedVectorMultivariateFunctionPatch& phi, const StepSizeType& elps)
{
    this->_set.apply_fixed_evolve_step(phi,elps);
}

Void HybridEnclosure::apply_space_evolve_step(const ValidatedVectorMultivariateFunctionPatch& phi, const ValidatedScalarMultivariateFunctionPatch& elps)
{
    this->_set.apply_space_evolve_step(phi,elps);
}

Void HybridEnclosure::apply_spacetime_evolve_step(const ValidatedVectorMultivariateFunctionPatch& phi, const ValidatedScalarMultivariateFunctionPatch& elps)
{
    this->_set.apply_spacetime_evolve_step(phi,elps);
}

Void HybridEnclosure::apply_spacetime_reach_step(const ValidatedVectorMultivariateFunctionPatch& phi, const ValidatedScalarMultivariateFunctionPatch& elps)
{
    this->_set.apply_spacetime_reach_step(phi,elps);
}


Void HybridEnclosure::apply_parameter_evolve_step(const ValidatedVectorMultivariateFunctionPatch& phi, const ValidatedScalarMultivariateFunctionPatch& elps)
{
    this->_set.apply_parameter_evolve_step(phi,elps);
}

Void HybridEnclosure::apply_finishing_parameter_evolve_step(const ValidatedVectorMultivariateFunctionPatch& phi, const ValidatedScalarMultivariateFunctionPatch& omega)
{
    this->_set.apply_finishing_parameter_evolve_step(phi,omega);
}


Void HybridEnclosure::apply_parameter_reach_step(const ValidatedVectorMultivariateFunctionPatch& phi, const ValidatedScalarMultivariateFunctionPatch& elps)
{
    this->_set.apply_parameter_reach_step(phi,elps);
}

Void HybridEnclosure::apply_full_reach_step(const ValidatedVectorMultivariateFunctionPatch& phi)
{
    this->_set.apply_full_reach_step(phi);
}



Void HybridEnclosure::bound_time(Real tmax) {
    if(possibly(this->time_range().upper_bound()>tmax)) {
       this->_set.new_negative_parameter_constraint(this->time_function()-tmax);
    }
}

Void HybridEnclosure::bound_time(ValidatedScalarMultivariateFunction tmax) {
    this->_set.new_negative_parameter_constraint(this->time_function()-this->_set.configuration().function_factory().create(this->_set.domain(),tmax));
}

Void HybridEnclosure::set_time(Real time)
{
    this->_set.new_zero_parameter_constraint(this->time_function()-time);
}

Void HybridEnclosure::set_time(ValidatedScalarMultivariateFunction time)
{
    this->_set.new_zero_parameter_constraint(this->time_function()-this->configuration().function_factory().create(this->_set.domain(),time));
}

const DiscreteLocation& HybridEnclosure::location() const {
    return this->_location;
}

HybridBasicSet<Enclosure> HybridEnclosure::state_set() const {
    Enclosure enclosure(this->parameter_domain(),this->state_function(),this->time_function(),this->constraints(),this->configuration());
    return HybridBasicSet<Enclosure>(this->location(),this->state_space(),enclosure);
}

HybridBasicSet<Enclosure> HybridEnclosure::state_time_set() const {
    auto state_time_space=(this->state_space().contains(TimeVariable()) ? this->state_space() : join(this->state_space(),TimeVariable()));
    auto state_time_function=join(this->state_function(),this->time_function());
    Enclosure enclosure(this->parameter_domain(),state_time_function,this->time_function(),this->constraints(),this->configuration());
    HybridBasicSet<Enclosure> hset(this->location(),state_time_space,enclosure);
    return hset;
}

HybridBasicSet<Enclosure> HybridEnclosure::state_auxiliary_set() const {
    auto state_auxiliary_space=join(this->state_space(),this->auxiliary_space());
    auto state_auxiliary_function=join(this->state_function(),this->auxiliary_function());
//    ValidatedConstrainedImageSet set(this->parameter_domain(),join(this->state_function(),this->auxiliary_function()),this->constraints());
//    ValidatedConstrainedImageSet set(this->parameter_domain(),join(this->state_function(),this->auxiliary_function()),this->constraints());
    Enclosure enclosure(this->parameter_domain(),state_auxiliary_function,this->time_function(),this->constraints(),this->configuration());
    HybridBasicSet<Enclosure> hset(this->location(),state_auxiliary_space,enclosure);
    return hset;
}

HybridBasicSet<Enclosure> project(HybridEnclosure const& encl, RealSpace const& spc) {
    ValidatedVectorMultivariateFunctionPatch spc_funct=encl.configuration().function_factory().create_zeros(spc.dimension(),encl.parameter_domain());
    for(SizeType i=0; i!=spc.dimension(); ++i) { spc_funct[i] = encl.function(spc[i]); }
    Enclosure spc_set(encl.parameter_domain(),spc_funct,encl.time_function(),encl.constraints(),encl.configuration());
    return HybridBasicSet<Enclosure>(encl.location(),spc,spc_set);
}


const Enclosure&
HybridEnclosure::continuous_set() const {
    return this->_set;
}


DimensionType HybridEnclosure::dimension() const {
    return this->_set.dimension();
}

DimensionType HybridEnclosure::state_dimension() const {
    return this->_set.state_dimension();
}

ValidatedLowerKleenean HybridEnclosure::is_empty() const {
    return this->_set.is_empty();
}

ValidatedLowerKleenean HybridEnclosure::inside(const HybridExactBox& hbx) const {
    if(this->_location==hbx.location()) { return this->continuous_set().inside(hbx.euclidean_set(this->state_space())); }
    else { return this->continuous_set().is_empty(); }
}

ValidatedLowerKleenean HybridEnclosure::separated(const HybridExactBox& hbx) const {
    if(this->_location==hbx.location()) { return this->continuous_set().separated(hbx.euclidean_set(this->state_space())); }
    else { return true; }
}

ValidatedKleenean HybridEnclosure::satisfies(EffectiveConstraint c) const
{
    return this->continuous_set().satisfies(c);
}


List<ValidatedConstraint> HybridEnclosure::constraints() const {
    return this->continuous_set().constraints();
}

Void
HybridEnclosure::restrict(const ExactBoxType& subdomain)
{
    this->_set.restrict(subdomain);
}

Void
HybridEnclosure::reduce()
{
    this->_set.reduce();
}

Void
HybridEnclosure::recondition()
{
    this->uniform_error_recondition();
    this->kuhn_recondition();
}

Void
HybridEnclosure::uniform_error_recondition()
{
    this->_set.uniform_error_recondition();
    this->_check();
}


Void
HybridEnclosure::kuhn_recondition()
{
    this->_set.kuhn_recondition();
    this->_check();
}


List<HybridEnclosure>
HybridEnclosure::split() const
{
    List<ExactBoxType> subdomains = this->continuous_set().splitting_subdomains_zeroth_order();
    List<HybridEnclosure> result(subdomains.size(),*this);
    for(SizeType i=0; i!=result.size(); ++i) {
        result[i].restrict(subdomains[i]);
    }
    return result;
}


inline Void check_subset(const ExactBoxType& dom1, const ExactBoxType& dom2, const char* msg)
{
    if(dom1.size()!=dom2.size()) {
        ARIADNE_FAIL_MSG(msg<<" size("<<dom1<<")!=size("<<dom2<<")");
    } else if(!subset(dom1,dom2)) {
        ARIADNE_FAIL_MSG(msg<<" !subset("<<dom1<<","<<dom2<<")");
    }
}

Void
HybridEnclosure::_check() const
{
    //this->_set.check();
    const ExactBoxType& reduced_domain = this->_set.reduced_domain();
    check_subset(reduced_domain,this->_set.domain(),"domain");
    check_subset(reduced_domain,this->state_function().domain(),"function domain");
    for(SizeType i=0; i!=this->_set.constraints().size(); ++i) {
        check_subset(reduced_domain,this->_set.constraint(i).function().domain(),"constraint");
    }
    check_subset(reduced_domain,this->time_function().domain(),"time");
}

Void HybridEnclosure::draw(CanvasInterface& canvas, const Set<DiscreteLocation>& locations, const Variables2d& axes) const
{
    Projection2d proj=Ariadne::projection(this->state_time_auxiliary_space(),axes);
    this->continuous_set().draw(canvas,proj);
}


OutputStream& operator<<(OutputStream& os, ValidatedConstraint const& c) {
    auto tcf = std::dynamic_pointer_cast<const ValidatedScalarMultivariateTaylorFunctionModelDP>(c.function().managed_pointer());
    if(tcf != nullptr) {
        auto pcf = MultivariatePolynomial<FloatDPApproximation>(tcf->polynomial());
        auto ecf = tcf->error();
        return os << c.lower_bound() << "<=" << pcf << "+/-" << ecf << "<=" << c.upper_bound();
    }
    return os << c.lower_bound() << "<=" << c.function() << "<=" << c.upper_bound();
}

OutputStream& operator<<(OutputStream& os, List<ValidatedConstraint> const& c) {
    os << "    \n[ "; for(SizeType i=0; i!=c.size(); ++i) { if (i!=0) { os << ",\n      "; } os << c[i]; } os << "]"; return os;
}

OutputStream& HybridEnclosure::_write(OutputStream& os) const
{
    return os << "HybridEnclosure"
              << "( variables = " << canonical_variable_names(this->_set.variable_kinds())
              << ",\n   events=" << this->_events
              << ",\n   location=" << this->_location
              << ",\n   state_space=" << this->state_space()
              << ",\n   range=" << apply(this->state_function(),this->_set.domain())
              << ",\n   domain=" << this->_set.domain()
              << ",\n   reduced_domain=" << this->_set.reduced_domain()
              << ",\n   is_(reduced_domain_)empty=" << this->_set.reduced_domain().is_empty()
              << ",\n   state=" << this->_set.state_function()
              << ",\n   constraints=" << this->_set.constraints()
              << ",\n   time="<< this->_set.time_function()
              << ")";
}

OutputStream& HybridEnclosure::print(OutputStream& os) const
{
    return os << "HybridEnclosure"
              << "( events=" << this->_events
              << ", location=" << this->_location
              << ", range=" << apply(this->state_function(),this->_set.domain())
              << ", domain=" << this->_set.domain()
              << ", reduced_domain=" << this->_set.reduced_domain()
              << ", #constraints=" << this->_set.constraints().size()
              << ", time_range="<<this->time_range() << ")";
}

OutputStream& HybridEnclosure::repr(OutputStream& os) const
{
    return os << "HybridEnclosure"
              << "( " << this->_location
              << ", " << this->_set.domain()
              << ", " << this->_set.reduced_domain()
              << ", " << this->_set.state_function()
              << ", " << this->_set.constraints()
              << ", " << this->time_function() << ")";
}

HybridExactBox under_approximation(const HybridRealBox& hbx);

ValidatedLowerKleenean inside(const HybridEnclosure& he, const HybridRealBox& hbx) {
    return he.inside(under_approximation(hbx));
}

Void HybridEnclosure::adjoin_outer_approximation_to(HybridStorage& hst, Nat fineness) const {
    DiscreteLocation location=this->location();
    const Enclosure& set = this->continuous_set();
    HybridGridTreePaving& hgtp=hst.state_set();
    GridTreePaving& paving = hgtp[location];
    RealSpace paving_space=hgtp.space(location);
    RealSpace state_space=this->state_space();
    if(state_space==paving_space) {
        set.state_set().adjoin_outer_approximation_to(paving,fineness);
    } else {
        ARIADNE_FAIL_MSG("HybridEnclosure's state variables "<<state_space<<" do not match variables "<<paving_space<<" of HybridStorage in location "<<location);
    }
}

/*
HybridStorage outer_approximation(const ListSet<HybridEnclosure>& hls, const HybridGrid& g, Nat fineness) {
    HybridStorage result(g);
    for(ListSet<HybridEnclosure>::ConstIterator iter=hls.begin(); iter!=hls.end(); ++iter) {
        result.state_set()[iter->location()].adjoin_outer_approximation(iter->continuous_set().state_set(),fineness);
    }
    return result;
}
*/

ListSet<HybridEnclosure::ContinuousStateSetType>
ListSet<HybridEnclosure>::operator[](const DiscreteLocation& loc) const
{
    ListSet<HybridEnclosure::ContinuousStateSetType> result;
    for(ConstIterator iter=this->begin(); iter!=this->end(); ++iter) {
        if(iter->location()==loc) {
            result.adjoin(iter->continuous_set());
        }
    }
    return result;
}

Void
ListSet<HybridEnclosure>::draw(CanvasInterface& cnvs, const Set<DiscreteLocation>& locs, const Variables2d& vars) const {
    StaticWorkload<HybridEnclosure, CanvasInterface*, Set<DiscreteLocation> const&, Variables2d const&> workload(
            [](HybridEnclosure const& e, CanvasInterface* c, Set<DiscreteLocation> const& l, Variables2d const& v)
            { e.draw(*c, l, v); }, &cnvs, locs, vars);
    workload.append(_list).process();
}

VariablesUpperBoxType
ListSet<HybridEnclosure>::bounding_box() const
{
    CONCLOG_SCOPE_CREATE;
    auto iter=this->begin();
    auto box = iter->state_auxiliary_set().euclidean_set().bounding_box();
    RealSpace space = join(iter->state_space(),iter->auxiliary_space());
    ++iter;
    for(; iter!=this->end(); ++iter) {
        box = hull(box,iter->state_auxiliary_set().euclidean_set().bounding_box());
    }
    return VariablesUpperBoxType(space,box);
}


} // namespace Ariadne
