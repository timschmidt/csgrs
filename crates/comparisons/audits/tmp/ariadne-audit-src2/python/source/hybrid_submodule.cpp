/***************************************************************************
 *
 *            hybrid_submodule.cpp
 *
 *  Copyright  2009-21  Pieter Collins
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

#include <iostream>
#include <iomanip>
#include <functional>

#include "utility/tribool.hpp"
#include "numeric/numeric.hpp"
#include "function/function.hpp"
#include "dynamics/orbit.hpp"
#include "symbolic/expression.hpp"
#include "symbolic/assignment.hpp"
#include "symbolic/space.hpp"
#include "io/figure.hpp"

#include "hybrid/discrete_event.hpp"
#include "hybrid/discrete_location.hpp"
#include "hybrid/hybrid_automata.hpp"
#include "hybrid/hybrid_time.hpp"
#include "hybrid/hybrid_orbit.hpp"
#include "hybrid/hybrid_set.hpp"
#include "hybrid/hybrid_expression_set.hpp"
#include "hybrid/hybrid_paving.hpp"
#include "hybrid/hybrid_storage.hpp"
#include "hybrid/hybrid_enclosure.hpp"
#include "hybrid/hybrid_simulator.hpp"
#include "hybrid/hybrid_evolver.hpp"
#include "hybrid/hybrid_reachability_analyser.hpp"
#include "hybrid/hybrid_graphics.hpp"

using namespace Ariadne;


namespace Ariadne {

template<class T> Nat __hash__(const T&);
template<> Nat __hash__<DiscreteEvent>(const DiscreteEvent& e) {
    return std::hash<const char*>()(e.name().c_str()); }
template<> Nat __hash__<DiscreteLocation>(const DiscreteLocation& q) {
    return std::hash<const char*>()(to_string(q).c_str()); }

template class HybridPoint<Real>;

} // namespace Ariadne

Void export_discrete_location(pybind11::module& module) {
    pybind11::class_<DiscreteLocation> discrete_location_class(module,"DiscreteLocation");
    discrete_location_class.def(pybind11::init<>());
    discrete_location_class.def(pybind11::init<std::map<StringVariable,String>>());
    discrete_location_class.def(pybind11::init<DiscreteLocation>());
    discrete_location_class.def("__eq__", &__eq__<DiscreteLocation,DiscreteLocation , Return<Bool> >);
    discrete_location_class.def("__ne__", &__ne__<DiscreteLocation,DiscreteLocation , Return<Bool> >);
    discrete_location_class.def("__hash__", &__hash__<DiscreteLocation>);
    discrete_location_class.def("__repr__",&__cstr__<DiscreteLocation>);
    pybind11::implicitly_convertible<std::map<StringVariable,String>,DiscreteLocation>();
}

Void export_discrete_event(pybind11::module& module) {
    pybind11::class_<DiscreteEvent> discrete_event_class(module,"DiscreteEvent");
    discrete_event_class.def(pybind11::init<StringType>());
    discrete_event_class.def(pybind11::init<DiscreteEvent>());
    discrete_event_class.def("__eq__", &__eq__<DiscreteEvent,DiscreteEvent , Return<Bool> >);
    discrete_event_class.def("__ne__", &__ne__<DiscreteEvent,DiscreteEvent , Return<Bool> >);
    discrete_event_class.def("__hash__", &__hash__<DiscreteEvent>);
    discrete_event_class.def("__repr__", &__cstr__<DiscreteEvent>);
    pybind11::implicitly_convertible<Int,DiscreteEvent>();
    pybind11::implicitly_convertible<StringType,DiscreteEvent>();
}

Void export_event_kind(pybind11::module& module) {
    pybind11::enum_<EventKind> event_kind_enum(module,"EventKind");
    event_kind_enum.value("INVARIANT", EventKind::INVARIANT);
    event_kind_enum.value("PROGRESS", EventKind::PROGRESS);
    event_kind_enum.value("PERMISSIVE", EventKind::PERMISSIVE);
    event_kind_enum.value("URGENT", EventKind::URGENT);
    event_kind_enum.value("IMPACT", EventKind::IMPACT);
    event_kind_enum.export_values();
}

Void export_hybrid_sets(pybind11::module& module) {
    pybind11::class_<HybridSetInterfaceBase>
        hybrid_set_interface_base_class(module,"HybridSetInterfaceBase");
    hybrid_set_interface_base_class.def("variables", &HybridSetInterfaceBase::variables);
    hybrid_set_interface_base_class.def("__repr__", &__cstr__<HybridSetInterfaceBase>);

    pybind11::class_<EffectiveHybridBoundedSetInterface, pybind11::bases<HybridSetInterfaceBase>>
        hybrid_bounded_set_interface_class(module,"EffectiveHybridBoundedSetInterface");
    hybrid_bounded_set_interface_class.def("locations", &EffectiveHybridBoundedSetInterface::locations);
    hybrid_bounded_set_interface_class.def("inside", &EffectiveHybridBoundedSetInterface::inside);
    hybrid_bounded_set_interface_class.def("bounding_box", &EffectiveHybridBoundedSetInterface::bounding_box);

    pybind11::class_<EffectiveHybridOvertSetInterface, pybind11::bases<HybridSetInterfaceBase>>
        hybrid_overt_set_interface(module,"EffectiveHybridOvertSetInterface");
    hybrid_overt_set_interface.def("euclidean_set", &EffectiveHybridOvertSetInterface::euclidean_set);
    hybrid_overt_set_interface.def("overlaps", &EffectiveHybridOvertSetInterface::overlaps);

    pybind11::class_<EffectiveHybridOpenSetInterface, pybind11::bases<EffectiveHybridOvertSetInterface>>
        hybrid_open_set_interface(module,"EffectiveHybridOpenSetInterface");
    hybrid_open_set_interface.def("euclidean_set", &EffectiveHybridOpenSetInterface::euclidean_set);
    hybrid_open_set_interface.def("covers", &EffectiveHybridOpenSetInterface::covers);

    pybind11::class_<EffectiveHybridClosedSetInterface, pybind11::bases<HybridSetInterfaceBase>>
        hybrid_closed_set_interface(module,"EffectiveHybridClosedSetInterface");
    hybrid_closed_set_interface.def("euclidean_set", &EffectiveHybridClosedSetInterface::euclidean_set);
    hybrid_closed_set_interface.def("separated", &EffectiveHybridClosedSetInterface::separated);

    pybind11::class_<EffectiveHybridCompactSetInterface,
            pybind11::bases<EffectiveHybridBoundedSetInterface,EffectiveHybridClosedSetInterface>>
        hybrid_compact_set_interface(module,"EffectiveHybridCompactSetInterface");
    hybrid_compact_set_interface.def("euclidean_set", &EffectiveHybridCompactSetInterface::euclidean_set);

    pybind11::class_<EffectiveHybridRegularSetInterface,
            pybind11::bases<EffectiveHybridClosedSetInterface,EffectiveHybridOpenSetInterface>>
        hybrid_regular_set_interface(module,"EffectiveHybridRegularSetInterface");
    hybrid_regular_set_interface.def("euclidean_set", &EffectiveHybridRegularSetInterface::euclidean_set);

    pybind11::class_<EffectiveHybridLocatedSetInterface,
            pybind11::bases<EffectiveHybridCompactSetInterface,EffectiveHybridOvertSetInterface>>
        hybrid_located_set_interface(module,"EffectiveRegularLocatedSetInterface");
    hybrid_located_set_interface.def("euclidean_set", &EffectiveHybridLocatedSetInterface::euclidean_set);

    pybind11::class_<EffectiveHybridRegularLocatedSetInterface,
            pybind11::bases<EffectiveHybridRegularSetInterface,EffectiveHybridLocatedSetInterface>>
        hybrid_regular_located_set_interface(module,"EffectiveHybridRegularLocatedSetInterface",pybind11::multiple_inheritance());
    hybrid_regular_located_set_interface.def("euclidean_set", &EffectiveHybridRegularLocatedSetInterface::euclidean_set);

}

template<class IVL> Void export_hybrid_variables_box(pybind11::module& module, const char* name) {
    pybind11::class_<HybridVariablesBox<IVL>> hybrid_variables_box_class(module,name);
    hybrid_variables_box_class.def(pybind11::init<DiscreteLocation,VariablesBox<IVL>>());
    hybrid_variables_box_class.def(pybind11::init<DiscreteLocation,RealSpace,Box<IVL>>());
    hybrid_variables_box_class.def("location", &HybridVariablesBox<IVL>::location);
    hybrid_variables_box_class.def("variables", &HybridVariablesBox<IVL>::variables);
    hybrid_variables_box_class.def("continuous_set", &HybridVariablesBox<IVL>::continuous_set);
    hybrid_variables_box_class.def("euclidean_set", &HybridVariablesBox<IVL>::euclidean_set);
}

template<class IVL> Void export_hybrid_variables_boxes(pybind11::module& module, const char* name) {
    pybind11::class_<HybridVariablesBoxes<IVL>> hybrid_variables_boxes_class(module,name);
    hybrid_variables_boxes_class.def(pybind11::init<Set<DiscreteLocation>,VariablesBox<IVL>>());
    hybrid_variables_boxes_class.def(pybind11::init<List<HybridVariablesBox<IVL>>>());
    hybrid_variables_boxes_class.def("adjoin", &HybridVariablesBoxes<IVL>::adjoin);
    hybrid_variables_boxes_class.def("locations", &HybridVariablesBoxes<IVL>::locations);
    hybrid_variables_boxes_class.def("variables", &HybridVariablesBoxes<IVL>::variables);
    hybrid_variables_boxes_class.def("continuous_set", &HybridVariablesBoxes<IVL>::continuous_set);
    hybrid_variables_boxes_class.def("euclidean_set", &HybridVariablesBoxes<IVL>::euclidean_set);
}

Void export_hybrid_constraint_set(pybind11::module& module) {
    pybind11::class_<HybridConstraintSet,pybind11::bases<EffectiveHybridRegularSetInterface>>
        hybrid_constraint_set_class(module,"HybridConstraintSet",pybind11::multiple_inheritance());
    hybrid_constraint_set_class.def(pybind11::init<DiscreteLocation,List<ContinuousPredicate>>());
    hybrid_constraint_set_class.def(pybind11::init<DiscreteLocation,RealExpressionConstraintSet>());
    hybrid_constraint_set_class.def(pybind11::init<Set<DiscreteLocation>,RealExpressionConstraintSet>());
    hybrid_constraint_set_class.def("adjoin",pybind11::overload_cast<DiscreteLocation const&,List<ContinuousPredicate> const&>(&HybridConstraintSet::adjoin));
    hybrid_constraint_set_class.def("adjoin",pybind11::overload_cast<DiscreteLocation const&,RealExpressionConstraintSet const&>(&HybridConstraintSet::adjoin));

    auto const& reference_internal = pybind11::return_value_policy::reference_internal;

    hybrid_constraint_set_class.def("locations",&HybridConstraintSet::locations);
    hybrid_constraint_set_class.def("has_location",&HybridConstraintSet::has_location);
    hybrid_constraint_set_class.def("variables",&HybridConstraintSet::variables);
    hybrid_constraint_set_class.def("continuous_set",&HybridConstraintSet::continuous_set,reference_internal);
    hybrid_constraint_set_class.def("euclidean_set",&HybridConstraintSet::euclidean_set);

    hybrid_constraint_set_class.def("overlaps",&HybridConstraintSet::overlaps);
    hybrid_constraint_set_class.def("separated",&HybridConstraintSet::separated);
    hybrid_constraint_set_class.def("covers",&HybridConstraintSet::covers);

    hybrid_constraint_set_class.def("__repr__",&__cstr__<HybridConstraintSet>);

    module.def("intersection", &_intersection_<HybridBoxesSet const&, HybridConstraintSet const&>);
}

Void export_hybrid_bounded_constraint_set(pybind11::module& module) {
    pybind11::class_<HybridBoundedConstraintSet,pybind11::bases<EffectiveHybridRegularLocatedSetInterface>>
        hybrid_bounded_constraint_set_class(module,"HybridBoundedConstraintSet",pybind11::multiple_inheritance());
    hybrid_bounded_constraint_set_class.def(pybind11::init<DiscreteLocation,RealVariablesBox>());
    hybrid_bounded_constraint_set_class.def(pybind11::init<Set<DiscreteLocation>,RealVariablesBox>());
    hybrid_bounded_constraint_set_class.def(pybind11::init<DiscreteLocation,RealExpressionBoundedConstraintSet>());
    hybrid_bounded_constraint_set_class.def(pybind11::init<Set<DiscreteLocation>,RealExpressionBoundedConstraintSet>());
//    hybrid_bounded_constraint_set_class.def(pybind11::init<DiscreteLocation,List<RealVariableInterval>>());
//    hybrid_bounded_constraint_set_class.def(pybind11::init<DiscreteLocation,List<RealVariableInterval>,List<ContinuousPredicate>>());
    hybrid_bounded_constraint_set_class.def("adjoin", &HybridBoundedConstraintSet::adjoin);
    hybrid_bounded_constraint_set_class.def("locations", &HybridBoundedConstraintSet::locations);
    hybrid_bounded_constraint_set_class.def("variables", &HybridBoundedConstraintSet::variables);
    hybrid_bounded_constraint_set_class.def("continuous_set", &HybridBoundedConstraintSet::continuous_set);
    hybrid_bounded_constraint_set_class.def("continuous_set", &HybridBoundedConstraintSet::euclidean_set);
    hybrid_bounded_constraint_set_class.def("overlaps", &HybridBoundedConstraintSet::overlaps);
    hybrid_bounded_constraint_set_class.def("inside", &HybridBoundedConstraintSet::inside);
    hybrid_bounded_constraint_set_class.def("separated", &HybridBoundedConstraintSet::separated);
    hybrid_bounded_constraint_set_class.def("covers", &HybridBoundedConstraintSet::covers);
    hybrid_bounded_constraint_set_class.def("bounding_box", &HybridBoundedConstraintSet::bounding_box);
    hybrid_bounded_constraint_set_class.def("draw", &HybridBoundedConstraintSet::draw);
    hybrid_bounded_constraint_set_class.def("__repr__", &__cstr__<HybridBoundedConstraintSet>);
}

template<class HBS> Void export_hybrid_basic_set(pybind11::module& module,pybind11::class_<HBS>& hybrid_basic_set_class) {
    //typedef typename HBS::ContinuousSetType EBS;
    hybrid_basic_set_class.def("location", &HBS::location);
    hybrid_basic_set_class.def("variables", &HBS::variables);
    hybrid_basic_set_class.def("continuous_set", &HBS::continuous_set);
    hybrid_basic_set_class.def("euclidean_set", pybind11::overload_cast<>(&HBS::euclidean_set,pybind11::const_));
    hybrid_basic_set_class.def("dimension", &HBS::dimension);
//    hybrid_basic_set_class.def("inside", &HBS::template inside<UpperIntervalType>);
//    hybrid_basic_set_class.def("inside", &HBS::template inside<UpperIntervalType>);
//    hybrid_basic_set_class.def("separated", &HBS::template separated<UpperIntervalType>);
//    hybrid_basic_set_class.def("overlaps", &HBS::template overlaps<UpperIntervalType>);
//    hybrid_basic_set_class.def("covers", &HBS::template covers<UpperIntervalType>);
//    hybrid_basic_set_class.def("is_empty", &HBS::template is_empty<UpperIntervalType>);
    hybrid_basic_set_class.def("bounding_box", &HBS::bounding_box);
    hybrid_basic_set_class.def("bounding_boxes", &HBS::bounding_boxes);

//    hybrid_basic_set_class.def("adjoin_outer_approximation_to", &HBS::adjoin_outer_approximation_to);
//    hybrid_basic_set_class.def("draw", &HBS::draw);
    hybrid_basic_set_class.def("__repr__", __cstr__<HBS>);
}

template<class HDS> Void export_hybrid_denotable_set(pybind11::module& module,pybind11::class_<HDS>& hybrid_denotable_set_class) {
    typedef typename HDS::ContinuousSetType EDS;

    hybrid_denotable_set_class.def("insert", pybind11::overload_cast<DiscreteLocation const&, LabelledSet<EDS> const&>(&HDS::insert));
    hybrid_denotable_set_class.def("insert", pybind11::overload_cast<DiscreteLocation const&, RealSpace const&, EDS const&>(&HDS::insert));
    hybrid_denotable_set_class.def("locations", &HDS::locations);
    hybrid_denotable_set_class.def("locations", &HDS::locations);
    hybrid_denotable_set_class.def("space", &HDS::space);
    hybrid_denotable_set_class.def("continuous_set", &HDS::continuous_set);
    hybrid_denotable_set_class.def("euclidean_set", pybind11::overload_cast<DiscreteLocation const&>(&HDS::euclidean_set,pybind11::const_));
    hybrid_denotable_set_class.def("has_location", &HDS::has_location);
    hybrid_denotable_set_class.def("__repr__", __cstr__<HDS>);
}

template<class X> Void export_hybrid_point(pybind11::module& module, const char* name) {
    pybind11::class_<HybridPoint<X>> hybrid_point_class(module,name);
    hybrid_point_class.def(pybind11::init<DiscreteLocation,Map<RealVariable,X>>());
    hybrid_point_class.def(pybind11::init<DiscreteLocation,List<Assignment<RealVariable,X>>>());
    export_hybrid_basic_set(module,hybrid_point_class);
}

template<class IVL> Void export_hybrid_box(pybind11::module& module, const char* name) {
    pybind11::class_<HybridBox<IVL>> hybrid_box_class(module,name);
    hybrid_box_class.def(pybind11::init<DiscreteLocation,Map<RealVariable,IVL>>());
    export_hybrid_basic_set(module,hybrid_box_class);
}

template<class IVL> Void export_hybrid_boxes(pybind11::module& module, const char* name) {
    pybind11::class_<HybridBoxes<IVL>> hybrid_boxes_class(module,name);
    export_hybrid_denotable_set(module,hybrid_boxes_class);
}


Void export_hybrid_storage(pybind11::module& module) {
    pybind11::class_<HybridStorage> hybrid_storage_class(module,"HybridStorage");
    hybrid_storage_class.def("__repr__", &__cstr__<HybridStorage>);
}

Void export_hybrid_enclosure(pybind11::module& module) {
    auto const& reference_internal = pybind11::return_value_policy::reference_internal;

    pybind11::class_<HybridEnclosure> hybrid_enclosure_class(module,"HybridEnclosure");
    hybrid_enclosure_class.def("previous_events", &HybridEnclosure::previous_events,reference_internal);
    hybrid_enclosure_class.def("location", &HybridEnclosure::location,reference_internal);
    hybrid_enclosure_class.def("continuous_set", &HybridEnclosure::continuous_set,reference_internal);
    hybrid_enclosure_class.def("state_space", &HybridEnclosure::state_space);
    hybrid_enclosure_class.def("auxiliary_space", &HybridEnclosure::auxiliary_space);
    hybrid_enclosure_class.def("state_auxiliary_space", &HybridEnclosure::state_auxiliary_space);
    hybrid_enclosure_class.def("state_time_auxiliary_space", &HybridEnclosure::state_time_auxiliary_space);
    hybrid_enclosure_class.def("time_range", &HybridEnclosure::time_range);
    hybrid_enclosure_class.def("state_bounding_box", &HybridEnclosure::state_bounding_box);
    hybrid_enclosure_class.def("__repr__", &__cstr__<HybridEnclosure>);
}

Void export_list_set_hybrid_enclosure(pybind11::module& module) {
    auto const& reference_internal = pybind11::return_value_policy::reference_internal;
    pybind11::class_<ListSet<HybridEnclosure>> list_set_hybrid_enclosure_class(module,"HybridEnclosureListSet");
    list_set_hybrid_enclosure_class.def("__iter__", [](ListSet<HybridEnclosure> const& l){return pybind11::make_iterator(l.begin(),l.end());});
    list_set_hybrid_enclosure_class.def("__len__",&ListSet<HybridEnclosure>::size);
    list_set_hybrid_enclosure_class.def("__getitem__",&ListSet<HybridEnclosure>::get,reference_internal);
    list_set_hybrid_enclosure_class.def("bounding_box",&ListSet<HybridEnclosure>::bounding_box);
}

template<class V, class E> decltype(auto) to_list(Map<V,E>const& map) {
    using A=decltype(declval<V>()=declval<E>());
    List<A> lst; for (auto ve : map) { lst.append(ve.first=ve.second); } return lst;
}

/*
template<class V, class E> List<Assignment<V,E>> to_list(Map<V,E>const& map) {
    List<Assignment<V,E>> lst; for (auto ve : map) { lst.append(ve.first=ve.second); } return lst;
}
template<class E> List<Assignment<RealVariable,E>> to_list(Map<LetRealVariable,E>const& map) {
    List<Assignment<RealVariable,E>> lst; for (auto ve : map) { lst.append(RealAssignment(ve.first.base(),ve.second)); } return lst;
}
*/

Void export_hybrid_automaton(pybind11::module& module)
{
    using RealAssignmentMap = Map<LetRealVariable,RealExpression>;
    using DottedRealAssignmentMap = Map<DottedRealVariable,RealExpression>;
    using PrimedRealAssignmentMap = Map<PrimedRealVariable,RealExpression>;

    // Don't use return_value_policy<copy_const_reference> since reference lifetime should not exceed automaton lifetime

    using pybind11::overload_cast;

    pybind11::class_<HybridAutomatonInterface> hybrid_automaton_interface_class(module,"HybridAutomatonInterface");

    pybind11::class_<HybridAutomaton,pybind11::bases<HybridAutomatonInterface>> hybrid_automaton_class(module,"HybridAutomaton");
    hybrid_automaton_class.def(pybind11::init<>());
    hybrid_automaton_class.def(pybind11::init<Identifier>());
    hybrid_automaton_class.def("name", &HybridAutomaton::name);
    hybrid_automaton_class.def("locations", &HybridAutomaton::locations);
    hybrid_automaton_class.def("events", &HybridAutomaton::events);
    hybrid_automaton_class.def("event_kind", &HybridAutomaton::event_kind);
    hybrid_automaton_class.def("target", &HybridAutomaton::target);
    hybrid_automaton_class.def("dynamic_function", &HybridAutomaton::dynamic_function);
    hybrid_automaton_class.def("guard_function", &HybridAutomaton::guard_function);
    hybrid_automaton_class.def("reset_function", &HybridAutomaton::reset_function);
    hybrid_automaton_class.def("guard_predicate", &HybridAutomaton::guard_predicate);
    hybrid_automaton_class.def("invariant_predicate", &HybridAutomaton::invariant_predicate);
    hybrid_automaton_class.def("dynamic_assignments", &HybridAutomaton::dynamic_assignments);
    hybrid_automaton_class.def("auxiliary_assignments", &HybridAutomaton::auxiliary_assignments);
    hybrid_automaton_class.def("reset_assignments", &HybridAutomaton::reset_assignments);
/*
    auto const& reference_internal = pybind11::return_value_policy::reference_internal;
    hybrid_automaton_class.def("new_mode",overload_cast<List<DottedRealAssignment>const&>(&HybridAutomaton::new_mode),reference_internal);
    hybrid_automaton_class.def("new_mode",overload_cast<List<RealAssignment>const&>( &HybridAutomaton::new_mode),reference_internal);
    hybrid_automaton_class.def("new_mode",overload_cast<List<RealAssignment>const&,List<DottedRealAssignment>const&>(&HybridAutomaton::new_mode),reference_internal);
    hybrid_automaton_class.def("new_mode",overload_cast<List<DottedRealAssignment>const&,List<RealAssignment>const&>(&HybridAutomaton::new_mode),reference_internal);
    hybrid_automaton_class.def("new_mode",overload_cast<DiscreteLocation>(&HybridAutomaton::new_mode),reference_internal);
    hybrid_automaton_class.def("new_mode",overload_cast<DiscreteLocation,List<DottedRealAssignment>const&>(&HybridAutomaton::new_mode),reference_internal);
    hybrid_automaton_class.def("new_mode",overload_cast<DiscreteLocation,List<RealAssignment>const&>( &HybridAutomaton::new_mode),reference_internal);
    hybrid_automaton_class.def("new_mode",overload_cast<DiscreteLocation,List<RealAssignment>const&,List<DottedRealAssignment>const&>(&HybridAutomaton::new_mode),reference_internal);
*/
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,DottedRealAssignmentMap dyn){ha.new_mode(to_list(dyn));});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,RealAssignmentMap alg){ha.new_mode(to_list(alg));});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,RealAssignmentMap alg, DottedRealAssignmentMap dyn){ha.new_mode(to_list(alg),to_list(dyn));});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha, DottedRealAssignmentMap dyn,RealAssignmentMap alg){ha.new_mode(to_list(dyn),to_list(alg));});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,DiscreteLocation loc){ha.new_mode(loc);});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,DiscreteLocation loc,DottedRealAssignmentMap dyn){ha.new_mode(loc,to_list(dyn));});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,DiscreteLocation loc,RealAssignmentMap alg){ha.new_mode(loc,to_list(alg));});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,DiscreteLocation loc,RealAssignmentMap alg, DottedRealAssignmentMap dyn){ha.new_mode(loc,to_list(alg),to_list(dyn));});
    hybrid_automaton_class.def("new_mode",[](HybridAutomaton& ha,DiscreteLocation loc,DottedRealAssignmentMap dyn, RealAssignmentMap alg){ha.new_mode(loc,to_list(dyn),to_list(alg));});

    hybrid_automaton_class.def("new_invariant",overload_cast<DiscreteLocation,ContinuousPredicate const&,DiscreteEvent>(&HybridAutomaton::new_invariant));
/*
    hybrid_automaton_class.def("new_transition",overload_cast<DiscreteLocation,DiscreteEvent,DiscreteLocation,List<PrimedRealAssignment>const&,ContinuousPredicate const&,EventKind>(&HybridAutomaton::new_transition));
    hybrid_automaton_class.def("new_transition",overload_cast<DiscreteLocation,DiscreteEvent,DiscreteLocation,ContinuousPredicate const&,EventKind>(&HybridAutomaton::new_transition));
    hybrid_automaton_class.def("new_transition",overload_cast<DiscreteLocation,DiscreteEvent,DiscreteLocation,List<PrimedRealAssignment>const&>(&HybridAutomaton::new_transition));
*/
    hybrid_automaton_class.def("new_transition",[](HybridAutomaton& ha, DiscreteLocation s,DiscreteEvent e,DiscreteLocation t,ContinuousPredicate grd,EventKind knd){return ha.new_transition(s,e,t,grd,knd);});
    hybrid_automaton_class.def("new_transition",[](HybridAutomaton& ha, DiscreteLocation s,DiscreteEvent e,DiscreteLocation t,PrimedRealAssignmentMap res){return ha.new_transition(s,e,t,to_list(res));});
    hybrid_automaton_class.def("new_transition",[](HybridAutomaton& ha, DiscreteLocation s,DiscreteEvent e,DiscreteLocation t,PrimedRealAssignmentMap res,ContinuousPredicate grd,EventKind knd){return ha.new_transition(s,e,t,to_list(res),grd,knd);});
    hybrid_automaton_class.def("new_transition",[](HybridAutomaton& ha, DiscreteLocation s,DiscreteEvent e,DiscreteLocation t,ContinuousPredicate grd,PrimedRealAssignmentMap res,EventKind knd){return ha.new_transition(s,e,t,to_list(res),grd,knd);});    hybrid_automaton_class.def("__str__ ", &__cstr__<HybridAutomaton>);
    hybrid_automaton_class.def("__repr__", &__crepr__<HybridAutomaton>);

    hybrid_automaton_class.def("discrete_reachability",&HybridAutomaton::discrete_reachability);
    hybrid_automaton_class.def("check_reachable_modes",&HybridAutomaton::check_reachable_modes);

    pybind11::class_<CompositeHybridAutomaton,pybind11::bases<HybridAutomatonInterface>>
        composite_hybrid_automaton_class(module,"CompositeHybridAutomaton");
    composite_hybrid_automaton_class.def(pybind11::init<const List<HybridAutomaton>&>());
    composite_hybrid_automaton_class.def(pybind11::init<Identifier,const List<HybridAutomaton>&>());
    composite_hybrid_automaton_class.def("discrete_reachability",&CompositeHybridAutomaton::discrete_reachability);
    composite_hybrid_automaton_class.def("check_reachable_modes",&CompositeHybridAutomaton::check_reachable_modes);
    composite_hybrid_automaton_class.def("__str__", &__cstr__<CompositeHybridAutomaton>);
    composite_hybrid_automaton_class.def("__repr__", &__crepr__<CompositeHybridAutomaton>);
    composite_hybrid_automaton_class.def("__iter__", [](CompositeHybridAutomaton const& c){return pybind11::make_iterator(c.begin(),c.end());});
}


Void export_hybrid_time(pybind11::module& module) {
    pybind11::class_<HybridTime> hybrid_time_class(module,"HybridTime");
    hybrid_time_class.def(pybind11::init<ExactDouble,Int>());
    hybrid_time_class.def(pybind11::init<Real,Integer>());
    hybrid_time_class.def("continuous_time",&HybridTime::continuous_time);
    hybrid_time_class.def("discrete_time",&HybridTime::discrete_time);
    hybrid_time_class.def("__str__", &__cstr__<HybridTime>);
    hybrid_time_class.def("__repr__", &__crepr__<HybridTime>);
}

Void export_hybrid_termination_criterion(pybind11::module& module) {
    typedef HybridTime::DiscreteTimeType DiscreteTimeType;
    typedef HybridTime::ContinuousTimeType ContinuousTimeType;

    pybind11::class_<HybridTerminationCriterion> hybrid_termination_criterion_class(module,"HybridTerminationCriterion");
    hybrid_termination_criterion_class.def(pybind11::init<ContinuousTimeType,DiscreteTimeType,Set<DiscreteEvent>>());
    hybrid_termination_criterion_class.def(pybind11::init<ContinuousTimeType,DiscreteTimeType>());
    hybrid_termination_criterion_class.def(pybind11::init<HybridTime>());
    hybrid_termination_criterion_class.def("maximum_time", &HybridTerminationCriterion::maximum_time);
    hybrid_termination_criterion_class.def("maximum_steps", &HybridTerminationCriterion::maximum_steps);
    hybrid_termination_criterion_class.def("terminating_events", &HybridTerminationCriterion::terminating_events);
    hybrid_termination_criterion_class.def("__str__", &__cstr__<HybridTerminationCriterion>);
    hybrid_termination_criterion_class.def("__repr__", &__crepr__<HybridTerminationCriterion>);
}

template<class SIM> Void export_simulator(pybind11::module& module, const char* name);

template<> Void export_simulator<HybridSimulator>(pybind11::module& module, const char* name)
{
    typedef HybridSimulator::TerminationType TerminationType;
    typedef HybridSimulator::HybridApproximatePointType HybridApproximatePointType;
    typedef HybridSimulator::OrbitType OrbitType;
    //typedef HybridSimulator::OrbitListType OrbitListType;

    auto const& reference_internal = pybind11::return_value_policy::reference_internal;

    pybind11::class_<HybridSimulator::OrbitType,pybind11::bases<HybridDrawableInterface>> hybrid_simulator_orbit_class(module,"HybridApproximatePointOrbit");
    hybrid_simulator_orbit_class.def("__repr__",&__cstr__<OrbitType>);
    pybind11::class_<HybridSimulator::OrbitListType,pybind11::bases<HybridDrawableInterface>> hybrid_simulator_list_orbit_class(module,"HybridApproximatePointListOrbit");
    //hybrid_simulator_list_orbit_class.def("__repr__",&__cstr__<OrbitListType>);

    pybind11::class_<HybridSimulator> hybrid_simulator_class(module,name);
    hybrid_simulator_class.def(pybind11::init<HybridSimulator::SystemType const&>());
    hybrid_simulator_class.def("configuration",pybind11::overload_cast<>(&HybridSimulator::configuration),reference_internal);
    hybrid_simulator_class.def("orbit", (OrbitType(HybridSimulator::*)(const HybridApproximatePointType&, const TerminationType&)const) &HybridSimulator::orbit);
    hybrid_simulator_class.def("orbit", pybind11::overload_cast<HybridApproximatePointType const&,TerminationType const&>(&HybridSimulator::orbit,pybind11::const_));
    hybrid_simulator_class.def("orbit", pybind11::overload_cast<HybridRealPoint const&,TerminationType const&>(&HybridSimulator::orbit,pybind11::const_));
    hybrid_simulator_class.def("orbit", pybind11::overload_cast<HybridBoundedConstraintSet const&,TerminationType const&>(&HybridSimulator::orbit,pybind11::const_));

    typedef typename HybridSimulator::ConfigurationType Configuration;
    pybind11::class_<Configuration> hybrid_simulator_configuration_class(module,"HybridSimulatorConfiguration");
    hybrid_simulator_configuration_class.def("set_step_size", &Configuration::set_step_size);
    hybrid_simulator_configuration_class.def("__repr__",&__cstr__<Configuration>);
}

Void export_hybrid_drawable_interface(pybind11::module& module)
{
    pybind11::class_<HybridDrawableInterface> hybrid_drawable_interface_class(module,"HybridDrawableInterface");
}

template<class ORB>
Void export_orbit(pybind11::module& module, const char* name)
{
    pybind11::class_<ORB,pybind11::bases<HybridDrawableInterface>> orbit_class(module,name);

    orbit_class.def("reach", &ORB::reach);
    orbit_class.def("evolve", &ORB::final);
    orbit_class.def("final", &ORB::final);
    orbit_class.def("__repr__", &__cstr__<ORB>);
}

template<class EV>
Void export_evolver_interface(pybind11::module& module, const char* name)
{
    pybind11::class_<EV> evolver_interface_class(module,name);
}

template<class EV, class... PARAMS> Void export_evolver(pybind11::module& module, const char* name);

template<>
Void export_evolver<GeneralHybridEvolver>(pybind11::module& module, const char* name)
{
    typedef GeneralHybridEvolver EV;
    typedef EV Evolver;
    typedef typename EV::Interface Interface;
    typedef typename EV::EnclosureType EnclosureType;
    typedef typename EV::TerminationType TerminationType;
    typedef typename EV::OrbitType OrbitType;

    auto const& reference_internal = pybind11::return_value_policy::reference_internal;

    pybind11::class_<Evolver,pybind11::bases<Interface>> evolver_class(module,name);
    evolver_class.def(pybind11::init<GeneralHybridEvolver::SystemType const&>());
    evolver_class.def("orbit",(OrbitType(Evolver::*)(const EnclosureType&,const TerminationType&,Semantics)const) &Evolver::orbit);
    evolver_class.def("orbit",(OrbitType(Evolver::*)(const HybridBoundedConstraintSet&,const TerminationType&,Semantics)const) &Evolver::orbit);
    evolver_class.def("set_integrator",&Evolver::set_integrator);
    evolver_class.def("configuration",pybind11::overload_cast<>(&Evolver::configuration),reference_internal);
    evolver_class.def("__str__",&__cstr__<GeneralHybridEvolver>);

    typedef typename EV::ConfigurationType Configuration;
    pybind11::class_<Configuration> evolver_configuration_class(module,"GeneralHybridEvolverConfiguration");
    evolver_configuration_class.def("set_maximum_enclosure_radius", &Configuration::set_maximum_enclosure_radius);
    evolver_configuration_class.def("set_maximum_step_size", &Configuration::set_maximum_step_size);
    evolver_configuration_class.def("set_maximum_spacial_error", &Configuration::set_maximum_spacial_error);
    evolver_configuration_class.def("set_enable_subdivisions", &Configuration::set_enable_subdivisions);
    evolver_configuration_class.def("set_enable_reconditioning", &Configuration::set_enable_reconditioning);
    evolver_configuration_class.def("__str__",&__cstr__<Configuration>);
}

template<class RA> Void export_safety_certificate(pybind11::module& module, const char* name) {
    typedef typename RA::StorageType StorageType;
    typedef typename RA::StateSpaceType StateSpaceType;
    typedef SafetyCertificate<StateSpaceType> SafetyCertificateType;

    pybind11::class_<SafetyCertificateType> safety_certificate_class(module,name);
    safety_certificate_class.def(pybind11::init<ValidatedSierpinskian,StorageType,StorageType >());
    safety_certificate_class.def_readonly("is_safe", &SafetyCertificateType::is_safe);
    safety_certificate_class.def_readonly("chain_reach_set", &SafetyCertificateType::chain_reach_set);
    safety_certificate_class.def_readonly("safe_set", &SafetyCertificateType::safe_set);
}

template<class RA, class... PARAMS> Void export_reachability_analyser(pybind11::module& module, const char* name);

template<>
Void export_reachability_analyser<HybridReachabilityAnalyser>(pybind11::module& module, const char* name)
{
    using RA=HybridReachabilityAnalyser;
    typedef typename RA::ConfigurationType Configuration;
    typedef typename RA::StorageType StorageType;
    typedef typename RA::OvertSetInterfaceType OvertSetType;
    typedef typename RA::OpenSetInterfaceType OpenSetType;
    typedef typename RA::CompactSetInterfaceType CompactSetType;
    typedef typename RA::SafetyCertificateType SafetyCertificateType;
    typedef typename RA::TimeType TimeType;

    auto const& reference_internal = pybind11::return_value_policy::reference_internal;

    pybind11::class_<RA> reachability_analyser_class(module,name);
    reachability_analyser_class.def(pybind11::init<HybridEvolverInterface const&>());
    reachability_analyser_class.def("configuration",(Configuration&(RA::*)())&RA::configuration,reference_internal);
    reachability_analyser_class.def("evolver",&RA::evolver,reference_internal);
    reachability_analyser_class.def("lower_reach",(StorageType(RA::*)(OvertSetType const&,TimeType const&)const) &RA::lower_reach);
    reachability_analyser_class.def("upper_reach",(StorageType(RA::*)(CompactSetType const&,TimeType const&)const) &RA::upper_reach);
    reachability_analyser_class.def("outer_chain_reach",&RA::outer_chain_reach);
    reachability_analyser_class.def("verify_safety",(SafetyCertificateType(RA::*)(CompactSetType const&, OpenSetType const&)const) &RA::verify_safety);

    pybind11::class_<Configuration> reachability_analyser_configuration_class(module,"HybridReachabilityAnalyserConfiguration");
    reachability_analyser_configuration_class.def("set_maximum_grid_fineness", &Configuration::set_maximum_grid_fineness);
    reachability_analyser_configuration_class.def("set_lock_to_grid_time", &Configuration::set_lock_to_grid_time);
    reachability_analyser_configuration_class.def("set_lock_to_grid_steps", &Configuration::set_lock_to_grid_steps);
    reachability_analyser_configuration_class.def("set_scaling", &Configuration::set_scaling);
    reachability_analyser_configuration_class.def("__repr__",&__cstr__<Configuration>);
}

Void export_hybrid_figure(pybind11::module& module) {
    static constexpr auto reference_internal = pybind11::return_value_policy::reference_internal ;

    pybind11::class_<HybridFigure> hybrid_figure_class(module,"HybridFigure");
    hybrid_figure_class.def(pybind11::init<>());
    hybrid_figure_class.def("set_axes", &HybridFigure::set_axes, reference_internal);
    hybrid_figure_class.def("set_line_style", &HybridFigure::set_line_style, reference_internal);
    hybrid_figure_class.def("set_line_width", &HybridFigure::set_line_width, reference_internal);
    hybrid_figure_class.def("set_line_colour", (Void(HybridFigure::*)(Colour))&HybridFigure::set_line_colour, reference_internal);
    hybrid_figure_class.def("set_fill_style", &HybridFigure::set_fill_style, reference_internal);
    hybrid_figure_class.def("set_fill_opacity", &HybridFigure::set_fill_opacity, reference_internal);
    hybrid_figure_class.def("set_fill_colour", (Void(HybridFigure::*)(Colour))&HybridFigure::set_fill_colour, reference_internal);
    hybrid_figure_class.def("draw",(Void(HybridFigure::*)(const HybridDrawableInterface&))&HybridFigure::draw, reference_internal);
    hybrid_figure_class.def("clear", &HybridFigure::clear, reference_internal);
    hybrid_figure_class.def("write",(Void(HybridFigure::*)(const Char*)const)&HybridFigure::write);
    hybrid_figure_class.def("write",(Void(HybridFigure::*)(const Char*,Nat,Nat)const)&HybridFigure::write);
}

Void export_hybrid_plots(pybind11::module& module) {
    module.def("plot", (Void(*)(const char*,Axes2d const&,HybridSimulator::OrbitListType const&))&plot);
    module.def("plot", (Void(*)(const char*,Axes2d const&,Colour const&,HybridSimulator::OrbitListType const&))&plot);
    module.def("plot", (Void(*)(const char*,Axes2d const&,HybridSimulator::OrbitType const&))&plot);
    module.def("plot", (Void(*)(const char*,Axes2d const&,Colour const&,HybridSimulator::OrbitType const&))&plot);

    module.def("plot", (Void(*)(const char*,Axes2d const&,GeneralHybridEvolver::OrbitType const&))&plot);
    module.def("plot", (Void(*)(const char*,Axes2d const&,Colour const&,GeneralHybridEvolver::OrbitType const&))&plot);

    module.def("plot", (Void(*)(const char*,Axes2d const&,HybridStorage const&))&plot);
    module.def("plot", (Void(*)(const char*,Axes2d const&,Colour const&,HybridStorage const&))&plot);
}

Void hybrid_submodule(pybind11::module& module) {
    export_discrete_location(module);
    export_discrete_event(module);
    export_event_kind(module);

    export_hybrid_sets(module);
    export_hybrid_point<Real>(module, "HybridRealPoint");
    //export_hybrid_point<FloatDPApproximation>(module, "HybridApproximatePointType");
    export_hybrid_box<FloatDPUpperInterval>(module, "HybridFloatDPUpperBox");
    export_hybrid_boxes<FloatDPUpperInterval>(module, "HybridFloatDPUpperBoxes");

    export_hybrid_variables_box<RealInterval>(module, "HybridVariablesBox");
    export_hybrid_variables_boxes<RealInterval>(module, "HybridVariablesBoxes");
    export_hybrid_constraint_set(module);
    export_hybrid_bounded_constraint_set(module);
    export_hybrid_storage(module);
    export_hybrid_enclosure(module);

    export_list_set_hybrid_enclosure(module);

    export_hybrid_automaton(module);

    export_hybrid_time(module);
    export_hybrid_termination_criterion(module);

    export_hybrid_drawable_interface(module);

    export_simulator<HybridSimulator>(module,"HybridSimulator");

    export_orbit<Orbit<HybridEnclosure>>(module, "HybridEnclosureOrbit");
    export_evolver_interface<HybridEvolverInterface>(module,"HybridEvolverInterface");
    export_evolver<GeneralHybridEvolver>(module,"GeneralHybridEvolver");

    export_orbit<Orbit<HybridStorage>>(module,"HybridStorageOrbit");

    export_safety_certificate<HybridReachabilityAnalyser>(module,"HybridSafetyCertificate");
    export_reachability_analyser<HybridReachabilityAnalyser>(module,"HybridReachabilityAnalyser");

    export_hybrid_figure(module);
    export_hybrid_plots(module);
}

