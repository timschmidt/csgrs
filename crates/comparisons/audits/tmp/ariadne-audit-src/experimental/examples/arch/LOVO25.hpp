/***************************************************************************
 *            LOVO25.hpp
 *
 *  Copyright  2025  Luca Geretti
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include "arch.hpp"

using namespace Ariadne;

void LOVO25()
{
    ArchBenchmark benchmark("LOVO25");

    CONCLOG_PRINTLN("Lotka-Volterra benchmark " << benchmark.name() << ":");

    RealVariable x("x");
    RealVariable y("y");

    RealConstant alpha("alpha",3);
    RealConstant beta("beta",3);
    RealConstant gamma("gamma",1);
    RealConstant delta("delta",1);
    RealConstant radius("radius",0.161_dec);

    StringVariable lotkavolterra("lotkavolterra");
    StringConstant outside("outside");
    StringConstant inside("inside");
    HybridAutomaton automaton(lotkavolterra.name());

    DiscreteEvent enter("enter");
    DiscreteEvent exit("exit");

    Real cx = gamma/delta;
    Real cy = alpha/beta;

    automaton.new_mode(lotkavolterra|outside,{dot(x)=alpha*x-beta*x*y, dot(y)= delta*x*y-gamma*y});
    automaton.new_mode(lotkavolterra|inside,{dot(x)=alpha*x-beta*x*y, dot(y)= delta*x*y-gamma*y});
    automaton.new_transition(lotkavolterra|outside,enter,lotkavolterra|inside,{next(x)=x,next(y)=y},sqr(x-cx)+sqr(y-cy)<=sqr(radius),EventKind::IMPACT);
    automaton.new_transition(lotkavolterra|inside,exit,lotkavolterra|outside,{next(x)=x,next(y)=y},sqr(x-cx)+sqr(y-cy)>=sqr(radius),EventKind::IMPACT);

    StepMaximumError max_err=1e-7;
    GradedTaylorSeriesIntegrator integrator(max_err);

    GeneralHybridEvolver evolver(automaton);
    evolver.set_integrator(integrator);
    evolver.configuration().set_maximum_enclosure_radius(1.0);
    evolver.configuration().set_maximum_step_size(0.2);
    evolver.configuration().set_maximum_spacial_error(1e-4);

    RealPoint ic({1.3_dec,1.0_dec});
    Real e(0.012_dec);
    HybridSet initial_set(lotkavolterra|outside,{ic[0]-e<=x<=ic[0]+e,y==ic[1]});
    HybridTime evolution_time(3.64_dec,5);

    Stopwatch<Milliseconds> sw;
    EnclosureConfiguration config(evolver.function_factory());
    config.set_reconditioning_num_blocks(5);
    auto initial_enclosure = HybridEnclosure(initial_set,RealSpace({x,y}),config);

    CONCLOG_PRINTLN("Computing evolution... ")
    auto orbit = evolver.orbit(initial_enclosure,evolution_time,Semantics::UPPER);

    CONCLOG_PRINTLN("Checking properties... ")

    ListSet<HybridEnclosure> actual_final;

    Bool has_2_number_of_transitions = false;
    Bool has_4_number_of_transitions = false;
    Bool has_different_number_of_transitions = false;
    for (HybridEnclosure encl : orbit.final()) {
        if (encl.time_range().lower_bound().get_d() > 3.6) {
            if (encl.previous_events().size() == 2) has_2_number_of_transitions = true;
            else if (encl.previous_events().size() == 4) has_4_number_of_transitions = true;
            else has_different_number_of_transitions = true;
            actual_final.append(encl);
        }
    }

    CONCLOG_RUN_AT(2,auto final_bounds = actual_final.bounding_box())

    sw.click();

    if (has_2_number_of_transitions and has_4_number_of_transitions and not has_different_number_of_transitions) {
        CONCLOG_PRINTLN("All final sets have either 2 or 4 transitions.")
    } else { CONCLOG_PRINTLN("Final set with a different number of transitions have been found!") }

    double final_set_area = (final_bounds[x].width()*final_bounds[y].width()).get_d();

    CONCLOG_PRINTLN("Done in " << sw.elapsed_seconds() << " seconds.")
    CONCLOG_PRINTLN("# of final sets: " << actual_final.size())
    CONCLOG_PRINTLN("Final set area: " << final_set_area)

    auto instance = benchmark.create_instance();
    if (final_set_area < 1e-2 and has_2_number_of_transitions and has_4_number_of_transitions and not has_different_number_of_transitions) {
        instance.set_verified(1)
                .set_execution_time(sw.elapsed_seconds())
                .add_loss(final_set_area);
    }
    instance.write();

    HybridAutomaton circle;
    DiscreteLocation rotate;
    RealVariable t("t");
    circle.new_mode(rotate,{let(x)=cx+radius*cos(t),let(y)=cy-radius*sin(t)},{dot(t)=1});
    HybridSimulator simulator(circle);
    simulator.configuration().set_step_size(0.1);
    HybridRealPoint circle_initial(rotate,{t=0});
    HybridTime circle_time(2*pi,1);
    CONCLOG_RUN_MUTED(auto circle_orbit = simulator.orbit(circle_initial,circle_time))

    CONCLOG_PRINTLN("Drawing figure... ")
    plot(benchmark.name().c_str(),Axes2d(0.6<=x<=1.4,0.6<=y<=1.4), orange, orbit, black, circle_orbit);
    CONCLOG_PRINTLN("File " << benchmark.name() << ".png written.")
}