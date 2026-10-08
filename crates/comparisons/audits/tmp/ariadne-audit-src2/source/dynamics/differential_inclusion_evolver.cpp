/***************************************************************************
 *            dynamics/differential_inclusion_evolver.cpp
 *
 *  Copyright  2008-20  Luca Geretti, Pieter Collins, Sanja Zivanovic
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

#include <optional>

#include "function/function_patch.hpp"
#include "function/taylor_function.hpp"
#include "function/constraint.hpp"
#include "solvers/integrator.hpp"
#include "solvers/bounder.hpp"
#include "algebra/expansion.inl.hpp"
#include "io/progress_indicator.hpp"

#include "differential_inclusion_evolver.hpp"
#include "enclosure.hpp"
#include "orbit.hpp"

namespace Ariadne {

inline ValidatedVectorMultivariateTaylorFunctionModelDP compose(ValidatedVectorMultivariateFunctionPatch const& g, ValidatedVectorMultivariateTaylorFunctionModelDP const& f) {
    return compose(cast_unrestricted(g),f);
}

template<class M> inline VectorScaledFunctionPatch<M> compose(ValidatedVectorMultivariateFunctionPatch const& g, VectorScaledFunctionPatch<M> const& f) {
    return compose(cast_unrestricted(g),f);
}

inline FloatDPUpperInterval operator*(PositiveValidatedUpperNumber x, FloatDPUpperInterval ivl) {
    return cast_exact(x.get(double_precision)*ivl); }

BoxDomainType initial_ranges_to_box(RealVariablesBox const& var_ranges) {
    auto vars = var_ranges.variables();
    List<IntervalDomainType> result;
    for (auto v : vars) {
        result.push_back(cast_exact(widen(IntervalDomainType(cast_exact(var_ranges[v].lower_bound().get(dp)),cast_exact(var_ranges[v].upper_bound().get(dp))))));
    }
    return Vector<IntervalDomainType>(result);
}

struct ScheduledApproximator
{
    Nat step;
    InclusionIntegrator approximator;

    ScheduledApproximator(Nat s, InclusionIntegrator a) : step(s), approximator(a) {}
};

inline OutputStream& operator<<(OutputStream& os, ScheduledApproximator const& sa) {
    return os << "(" << sa.step << ":" << sa.approximator << ")"; }

struct ScheduledApproximatorComparator {
    inline bool operator() (ScheduledApproximator const& sa1, ScheduledApproximator const& sa2) {
        return (sa1.step > sa2.step); }
};

class InclusionEvolverState {
  private:
    Nat _step;
    List<ScheduledApproximator> _schedule;
    Map<InclusionIntegrator,Nat> _approximator_check_delay;
    Map<InclusionIntegrator,Nat> _approximator_global_optima_count;
    Map<InclusionIntegrator,Nat> _approximator_local_optima_count;
  public:
    InclusionEvolverState(DifferentialInclusion const& ivf, List<InputApproximation> const& approximations, IntegratorInterface const& integrator)
        : _step(0u)
    {
        InclusionIntegratorFactory factory(integrator);
        bool has_reach_rigorous_approximations = false;
        for (auto appro : approximations) {
            if (appro.is_reach_rigorous()) has_reach_rigorous_approximations = true;
            InclusionIntegrator approximator = factory.create(ivf.function(), ivf.inputs(), appro);
            _schedule.push_back(ScheduledApproximator(0u,approximator));
            _approximator_global_optima_count[approximator] = 0;
            _approximator_local_optima_count[approximator] = 0;
            _approximator_check_delay[approximator] = 0;
        }
        if (not has_reach_rigorous_approximations) {
            InclusionIntegrator approximator = factory.create(ivf.function(), ivf.inputs(), ConstantApproximation());
            _schedule.push_back(ScheduledApproximator(0u,approximator));
            _approximator_global_optima_count[approximator] = 0;
            _approximator_local_optima_count[approximator] = 0;
            _approximator_check_delay[approximator] = 0;
        }
    }

    Nat step() const { return _step; }
    List<ScheduledApproximator> const& schedule() const { return _schedule; }
    Map<InclusionIntegrator,Nat> const& check_delay() const { return _approximator_check_delay; }
    Map<InclusionIntegrator,Nat> const& global_optima_count() const { return _approximator_global_optima_count; }
    Map<InclusionIntegrator,Nat> const& local_optima_count() const { return _approximator_local_optima_count; }

    Void reset_local_optima_count() {
        for (auto entry : _approximator_local_optima_count)
            _approximator_local_optima_count[entry.first] = 0;
    }

    Void update_with_best(InclusionIntegrator const& best) {
        Nat max_reach_rigorous_index = 0;
        for (auto appro : approximators_to_use()) {
            if (appro.is_reach_rigorous()) max_reach_rigorous_index = std::max(max_reach_rigorous_index,appro.index());
        }
        for (auto appro : approximators_to_use()) {
            if (best == appro) _reset_check_delay(appro);
            else {
                if (not best.is_reach_rigorous() and appro.index() == max_reach_rigorous_index)
                    _reset_check_delay(appro);
                else _increase_check_delay(appro);
            }

            Nat offset = 1u<<_approximator_check_delay[appro];
            _schedule.push_back(ScheduledApproximator(_step+offset,appro));
        }
        std::sort(_schedule.begin(),_schedule.end(),ScheduledApproximatorComparator());

        _increase_optima_count(best);
    }

    List<InclusionIntegrator> approximators_to_use() const {
        List<InclusionIntegrator> result;
        for (auto i = _schedule.size(); i > 0; --i) {
            auto entry = _schedule[i-1];
            if (entry.step == _step) {
                result.push_back(entry.approximator);
            } else if (entry.step > _step) {
                break;
            }
        }
        return result;
    }

    Void next_step() {
        while (!_schedule.empty()) {
            auto entry = _schedule.back();
            if (entry.step == _step) {
                _schedule.pop_back();
            } else if (entry.step > _step) {
                break;
            }
        }
        _step++;
    }

  private:
    Void _reset_check_delay(InclusionIntegrator const& approximator) { _approximator_check_delay[approximator] = 0; }
    Void _increase_check_delay(InclusionIntegrator const& approximator) { _approximator_check_delay[approximator]++; }
    Void _increase_optima_count(InclusionIntegrator const& approximator) {
        _approximator_global_optima_count[approximator]++;
        _approximator_local_optima_count[approximator]++;
    }
};

inline Map<InclusionIntegrator,ApproximateDouble> convert_to_percentages(Map<InclusionIntegrator,Nat> const& approximation_global_frequencies) {

    Nat total_steps(0);
    for (auto entry: approximation_global_frequencies) {
        total_steps += entry.second;
    }

    Map<InclusionIntegrator,ApproximateDouble> result;
    for (auto entry: approximation_global_frequencies) {
        result[entry.first] = 1.0/total_steps*entry.second;
    }

    return result;
}

DifferentialInclusionEvolver::DifferentialInclusionEvolver(SystemType const& system, IntegratorInterface const& integrator, Reconditioner const& reconditioner)
    : _system(system)
    , _integrator(integrator.clone())
    , _reconditioner(reconditioner)
    , _configuration(new ConfigurationType())
{
    ARIADNE_PRECONDITION(system.inputs().size() > 0);
    CONCLOG_SCOPE_CREATE;
}

Void DifferentialInclusionEvolver::_recondition_and_update(ValidatedVectorMultivariateFunctionPatch& function, InclusionEvolverState& state) {
    if (_reconditioner.must_reduce_parameters(state)) {
        _reconditioner.update_from(state);
        _reconditioner.reduce_parameters(function);
        state.reset_local_optima_count();
    }

    if (_reconditioner.must_incorporate_errors(state)) {
        function = _reconditioner.incorporate_errors(function);
    }
}

template<class T> using Optional=std::optional<T>;

auto DifferentialInclusionEvolver::orbit(RealVariablesBox const& initial, Real const& tmax) -> OrbitType {
    CONCLOG_SCOPE_CREATE;
    CONCLOG_PRINTLN_AT(1,"System: "<<_system);
    CONCLOG_PRINTLN_AT(1,"Initial: "<<initial);

    auto initial_box = initial_ranges_to_box(initial);

    StepSizeType hsug=_configuration->maximum_step_size();

    EulerBounder bounder;

    auto const& function_factory = static_cast<IntegratorBase const&>(*this->_integrator).function_factory();

    EnclosureConfiguration config(function_factory);

    EnclosureType evolve(initial,_system.state_space(),config);
    CONCLOG_PRINTLN_VAR_AT(1,evolve)
    ValidatedVectorMultivariateFunctionPatch evolve_function = function_factory.create_identity(initial_box);

    TimeStepType t;

    InclusionEvolverState state(_system,_configuration->approximations(),*_integrator);

    OrbitType result(evolve);

    ProgressIndicator indicator(tmax.get_d());

    while (possibly(t<lower_bound(tmax))) {
        CONCLOG_SCOPE_PRINTHOLD("[" << indicator.symbol() << "] " << indicator.percentage() << "% ");

        CONCLOG_PRINTLN_AT(2,"n. of parameters="<<evolve_function.argument_size());

        auto approximators_to_use = state.approximators_to_use();

        CONCLOG_PRINTLN_AT(1,"#s="<<std::setw(4)<<result.reach().size()
                                 <<" t="<<std::setw(7)<<t.get_d()
                                 <<" p="<<std::setw(4)<<evolve_function.argument_size()
                                 <<" r="<<std::setw(7)<<evolve_function.range().radius()
                                 <<" c="<<evolve_function.range().centre()
                                 <<" a="<<approximators_to_use)

        auto domx = cast_exact_box(evolve_function.range());

        UpperBoxType B;
        StepSizeType h;

        CONCLOG_RUN_AT(1,std::tie(h,B)=bounder.compute(_system.function(),domx,_system.inputs(),suggest(hsug)));
        CONCLOG_PRINTLN_AT(2,"flow bounds = "<<B<<" (using h = " << h << ")");

        TimeStepType new_t = lower_bound(t+h);

        Optional<ValidatedVectorMultivariateFunctionPatch> best_reach_function;
        Optional<ValidatedVectorMultivariateFunctionPatch> best_evolve_function;
        InclusionIntegrator best = approximators_to_use.at(0);
        FloatDPApproximation best_volume(inf,dp);
        Optional<ValidatedVectorMultivariateFunctionPatch> best_reach_rigorous_function;
        FloatDPApproximation best_reach_rigorous_volume(inf,dp);

        for (auto const& approximator : approximators_to_use) {
            CONCLOG_PRINTLN_AT(3,"checking "<<approximator<<" approximator");

            CONCLOG_RUN_AT(2,auto current_reach=approximator.reach(domx,evolve_function,B,t,h));
            auto current_evolve=approximator.evolve(current_reach.at(current_reach.size()-1u),new_t);

            FloatDPApproximation current_volume = volume(current_evolve.range());
            if (decide(current_volume < best_volume)) {
                best = approximator;
                CONCLOG_PRINTLN_AT(3,"best approximator: " << best);
                if (best.is_reach_rigorous()) {
                    best_reach_function = current_reach.at(0);
                }
                best_evolve_function = current_evolve;
                best_volume = current_volume;
            }

            if (approximator.is_reach_rigorous() and decide(current_volume < best_reach_rigorous_volume)) {
                best_reach_rigorous_function = current_reach.at(0);
                best_reach_rigorous_volume = current_volume;
            }
        }

        if (not best.is_reach_rigorous())
            best_reach_function = best_reach_rigorous_function;

        if (approximators_to_use.size() > 1)
            CONCLOG_PRINTLN_AT(2,"chosen approximator: " << best << (best.is_reach_rigorous() ? "" : " (reach taken from other approximator)"));

        state.update_with_best(best);

        evolve_function = best_evolve_function.value();
        ValidatedVectorMultivariateFunctionPatch reach_function = best_reach_function.value();

        CONCLOG_PRINTLN_AT(2,"evolve bounds="<<evolve_function.range());

        CONCLOG_RUN_AT(2, this->_recondition_and_update(evolve_function, state));

        auto time_ivl_function = function_factory.create_coordinate(reach_function.domain(),reach_function.result_size());
        EnclosureType reach_enclosure(Enclosure(reach_function.domain(),reach_function,time_ivl_function,List<ValidatedConstraint>(),config),_system.state_space());
        result.adjoin_reach(reach_enclosure);

        auto time_function = function_factory.create_constant(evolve_function.domain(),new_t);
        evolve = EnclosureType(Enclosure(evolve_function.domain(), evolve_function, time_function, List<ValidatedConstraint>(), config), _system.state_space());
        result.adjoin_intermediate(evolve);

        state.next_step();
        t=new_t;
        indicator.update_current(t.get_d());
        CONCLOG_PRINTLN_AT(2,"updated schedule: " << state.schedule());
    }

    CONCLOG_PRINTLN_AT(1,"approximation % ="<<convert_to_percentages(state.global_optima_count()));
    result.adjoin_final(evolve);

    return result;
}

struct IndexedFloatDPError
{
    SizeType index;
    FloatDPError value;

    IndexedFloatDPError() : index(0), value(FloatDPError(dp)) {}
};

inline OutputStream& operator<<(OutputStream& os, IndexedFloatDPError const& ifl) {
    return os << "(" << ifl.index << ":" << std::scientific << ifl.value.raw() << std::fixed << ")"; }

struct IndexedFloatDPErrorComparator
{
    inline bool operator() (const IndexedFloatDPError& ifl1, const IndexedFloatDPError& ifl2)
    {
        return (ifl1.value.raw() < ifl2.value.raw());
    }
};

Bool LohnerReconditioner::must_reduce_parameters(InclusionEvolverState const& state) const {
    if (_number_of_steps_between_simplifications == 0) return false;
    return (state.step()%_number_of_steps_between_simplifications == _number_of_steps_between_simplifications-1);
}

Bool LohnerReconditioner::must_incorporate_errors(InclusionEvolverState const& state) const {
    return true;
}

Void LohnerReconditioner::update_from(InclusionEvolverState const& state) {
    auto freq = _number_of_steps_between_simplifications;
    auto n = _number_of_variables;
    auto m = _number_of_inputs;

    FloatDPApproximation npk(0,dp);
    FloatDPApproximation rho(_ratio_of_parameters_to_keep,dp);
    for (auto entry: state.local_optima_count()) {
        SizeType ppi = entry.first.num_params_per_input();
        FloatDPApproximation partial = n + rho*(n+2*m) + (freq-1)*m*(2 - ppi);
        npk += partial*entry.second/freq;
    }

    _number_of_parameters_to_keep = static_cast<Nat>(round(npk).get_d());
}

ValidatedVectorMultivariateFunctionPatch LohnerReconditioner::incorporate_errors(ValidatedVectorMultivariateFunctionPatch const& f) const {
    CONCLOG_SCOPE_CREATE;
    ValidatedVectorMultivariateTaylorFunctionModelDP const& tf = dynamic_cast<ValidatedVectorMultivariateTaylorFunctionModelDP const&>(f.reference());

    BoxDomainType domain=f.domain();
    auto ferrors=f.errors(); BoxDomainType errors(f.result_size(),[&](SizeType i){return cast_exact_interval(ferrors[i]*FloatDPUpperInterval(-1,+1));}); // TODO: Avoid cast;
    //    BoxDomainType errors=cast_exact(cast_exact(f.errors())*FloatDPUpperInterval(-1,+1));

    CONCLOG_PRINTLN("Uniform errors:"<<errors);

    ValidatedVectorMultivariateFunctionPatch error_function(ValidatedVectorMultivariateTaylorFunctionModelDP::identity(errors,tf.properties()));
    ValidatedVectorMultivariateFunctionPatch result = embed(f,errors)+embed(domain,error_function);
    for(SizeType i=0; i!=result.result_size(); ++i) { result[i].clobber(); }
    return result;
}

Void LohnerReconditioner::reduce_parameters(ValidatedVectorMultivariateFunctionPatch& f) const {
    CONCLOG_SCOPE_CREATE;
    CONCLOG_PRINTLN("f="<<f);
    auto m=f.argument_size();
    auto n=f.result_size();

    CONCLOG_PRINTLN("num.parameters="<<m<<", to keep="<< this->_number_of_parameters_to_keep );

    ValidatedVectorMultivariateTaylorFunctionModelDP& tf = dynamic_cast<ValidatedVectorMultivariateTaylorFunctionModelDP&>(f.reference());

    auto sweeper = tf.properties();

    // Compute effect of error terms, but not of original variables;
    Matrix<FloatDPError> C(m,n,tf.properties().precision());
    for (auto i : range(n)) {
        auto p=tf[i].model().expansion();

        for (auto ac : p) {
            UniformConstReference<MultiIndex> a=ac.index();
            UniformReference<FloatDP> c=ac.coefficient();
            for (auto j : range(m)) {
                if (a[j]!=0) {
                    C[j][i] += mag(c);
                }
            }
        }
    }

    CONCLOG_PRINTLN_AT(1,"C"<<C);
    Array<IndexedFloatDPError> Ce(m);
    for (auto j : range(m)) {
        Ce[j].index = j;
        for (auto i : range(n)) {
            Ce[j].value += C[j][i];
        }
    }
    CONCLOG_PRINTLN_AT(1,"Ce:"<<Ce);
    auto SCe=Ce;
    std::sort(SCe.begin(),SCe.end(),IndexedFloatDPErrorComparator());
    CONCLOG_PRINTLN_AT(1,"SortedCe:"<<SCe);
    List<SizeType> keep_indices;
    List<SizeType> remove_indices;

    if (m <= this->_number_of_parameters_to_keep) {
        CONCLOG_PRINTLN("Insufficient number of variables, not simplifying");
        return;
    }

    Nat number_of_variables_to_remove = m - this->_number_of_parameters_to_keep;
    CONCLOG_PRINTLN_AT(1, "Number of parameters to remove:" << _number_of_parameters_to_keep);

    for (auto j : range(number_of_variables_to_remove)) {
        remove_indices.append(SCe[j].index);
    }

    for (auto j : range(number_of_variables_to_remove,m)) {
        keep_indices.append(SCe[j].index);
    }

    CONCLOG_PRINTLN_AT(1,"number of kept parameters: " << keep_indices.size() << "/" << m);

    CONCLOG_PRINTLN_AT(2,"keep_indices:"<<keep_indices);
    CONCLOG_PRINTLN_AT(2,"remove_indices:"<<remove_indices);

    for (auto i : range(n)) {
        FloatDPError error = tf[i].error();
        for(SizeType k=0; k!=remove_indices.size(); ++k) {
            error += mag(C[remove_indices[k]][i]);
        }
        tf[i].set_error(error);
    }

    auto old_domain=f.domain();
    auto new_domain=BoxDomainType(Vector<IntervalDomainType>(keep_indices.size(),[&old_domain,&keep_indices](SizeType j){return old_domain[keep_indices[j]];}));
    auto projection=ValidatedVectorMultivariateTaylorFunctionModelDP(m,new_domain,sweeper);
    for (auto i : range(new_domain.size())) { projection[keep_indices[i]]=ValidatedScalarMultivariateTaylorFunctionModelDP::coordinate(new_domain,i,sweeper); }
    for (auto i : range(remove_indices.size())) {
        auto j=remove_indices[i]; auto cj=old_domain[j].midpoint();
        projection[j]=ValidatedScalarMultivariateTaylorFunctionModelDP::constant(new_domain,cj,sweeper); }
    f=compose(f,projection);
}

DifferentialInclusionEvolverConfiguration::DifferentialInclusionEvolverConfiguration()
{
    this->set_maximum_step_size(1.0_x);
    this->set_maximum_enclosure_radius(100.0_x);
    set_enable_parameter_reduction(true);
    set_approximations({ZeroApproximation(), ConstantApproximation(), AffineApproximation(), SinusoidalApproximation(), PiecewiseApproximation()});
}

OutputStream&
DifferentialInclusionEvolverConfiguration::_write(OutputStream& os) const
{
    os << "DifferentialInclusionEvolverConfiguration("
       << "\n  maximum_step_size=" << maximum_step_size()
       << ",\n  maximum_enclosure_radius=" << maximum_enclosure_radius()
       << ",\n  enable_parameter_reduction=" << enable_parameter_reduction()
       << ",\n  approximations=" << approximations()
       << "\n)\n";
    return os;
}

} // namespace Ariadne;
