/***************************************************************************
 *            test_hybrid_automaton.cpp
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

#include <iostream>
#include <fstream>

#include "config.hpp"
#include "../test.hpp"

#include "symbolic/expression.hpp"
#include "symbolic/valuation.hpp"
#include "hybrid/hybrid_automata.hpp"
#include "io/command_line_interface.hpp"

using namespace Ariadne;

class TestHybridAutomaton {
  public:
    Void test();

    Void test_distinct_modes();
    Void test_overspecified_location();
    Void test_overspecified_dynamic();
    Void test_algebraic_loops();
    Void test_nonexistent_mode();
    Void test_multiple_guard();
    Void test_multiple_transition();
    Void test_overspecified_reset();

    Void test_underspecified_mode();
    Void test_underspecified_reset();

    Void test_build_hybrid_system();
    Void test_static_analysis();
  private:
    CompositeHybridAutomaton _system;
    HybridAutomaton _get_tank();
    HybridAutomaton _get_valve();
    HybridAutomaton _get_controller();
};

Void
TestHybridAutomaton::test()
{
    ARIADNE_TEST_CALL(test_distinct_modes());
    ARIADNE_TEST_CALL(test_overspecified_location());
    ARIADNE_TEST_CALL(test_overspecified_dynamic());
    ARIADNE_TEST_CALL(test_algebraic_loops());
    ARIADNE_TEST_CALL(test_nonexistent_mode());
    ARIADNE_TEST_CALL(test_multiple_guard());
    ARIADNE_TEST_CALL(test_multiple_transition());
    ARIADNE_TEST_CALL(test_overspecified_reset());
    ARIADNE_TEST_CALL(test_underspecified_mode());
    ARIADNE_TEST_CALL(test_underspecified_reset());
    ARIADNE_TEST_CALL(test_build_hybrid_system());
    ARIADNE_TEST_CALL(test_static_analysis());
}

Void
TestHybridAutomaton::test_distinct_modes()
{
    HybridAutomaton system;
    StringVariable q1("q1");
    StringVariable q2("q2");
    ARIADNE_TEST_EXECUTE(system.new_mode(q1|"s1"));
    ARIADNE_TEST_EXECUTE(system.new_mode(q1|"s2"));
    ARIADNE_TEST_THROWS(system.new_mode(q2|"s3"),IndistinguishableModeError);
}

Void
TestHybridAutomaton::test_overspecified_location()
{
    StringVariable q1("q1");
    StringVariable q2("q2");
    StringVariable q3("q3");

    HybridAutomaton system1;
    ARIADNE_TEST_EXECUTE(system1.new_mode(q1|"s1"));
    DiscreteLocation loc1({q1|"s1"});
    ARIADNE_TEST_ASSERT(system1.has_mode(loc1));
    ARIADNE_TEST_ASSERT(system1.has_partial_mode(loc1));
    ARIADNE_TEST_PRINT(system1.mode(loc1));
    DiscreteLocation loc1e({q1|"s1",q3|"s3"});
    ARIADNE_TEST_ASSERT(not system1.has_mode(loc1e));
    ARIADNE_TEST_ASSERT(system1.has_partial_mode(loc1e));
    ARIADNE_TEST_PRINT(system1.mode(loc1e));

    HybridAutomaton system2;
    ARIADNE_TEST_EXECUTE(system2.new_mode(q2|"s2"));
    CompositeHybridAutomaton system({system1,system2});
    DiscreteLocation loc({q1|"s1",q2|"s2"});
    ARIADNE_TEST_ASSERT(system.has_mode(loc));
    ARIADNE_TEST_ASSERT(system.has_partial_mode(loc));
    ARIADNE_TEST_PRINT(system.mode(loc));
    DiscreteLocation loce({q1|"s1",q2|"s2",q3|"s3"});
    ARIADNE_TEST_ASSERT(not system.has_mode(loce));
    ARIADNE_TEST_ASSERT(system.has_partial_mode(loce));
    ARIADNE_TEST_PRINT(system.mode(loce));
}

Void
TestHybridAutomaton::test_overspecified_dynamic()
{
    HybridAutomaton system;
    RealVariable x("x");
    RealVariable y("y");
    ARIADNE_TEST_EXECUTE( system.new_mode( {let(x)=y},{dot(y)=x} ) );
    ARIADNE_TEST_THROWS( system.new_mode( {let(x)=1,let(x)=x+2} ),SystemSpecificationError);
    ARIADNE_TEST_THROWS( system.new_mode( {dot(x)=1,dot(x)=0} ),SystemSpecificationError);
    ARIADNE_TEST_THROWS( system.new_mode( {let(x)=1},{dot(x)=0} ),SystemSpecificationError);
}

Void
TestHybridAutomaton::test_algebraic_loops()
{
    HybridAutomaton system;
    RealVariable x("x");
    RealVariable y("y");
    RealVariable z("z");
    ARIADNE_TEST_THROWS( HybridAutomaton().new_mode( {let(x)=y+1,let(y)=x} ),AlgebraicLoopError);
    ARIADNE_TEST_THROWS( HybridAutomaton().new_mode( {let(x)=2*x+1} ),AlgebraicLoopError);
    ARIADNE_TEST_EXECUTE( HybridAutomaton().new_mode( {let(x)=y,let(y)=z} ) );
    ARIADNE_TEST_EXECUTE( HybridAutomaton().new_mode( {let(x)=y,let(y)=z},{dot(z)=x+y+z} ) );
}

Void
TestHybridAutomaton::test_nonexistent_mode()
{
    HybridAutomaton system;
    StringVariable q("q");
    StringVariable r("r");
    DiscreteEvent e("e");
    RealVariable x("x");
    ARIADNE_TEST_EXECUTE( system.new_mode( {q|"1"},{dot(x)=0} ) );
    ARIADNE_TEST_EXECUTE( system.new_mode( {q|"2",r|"1"} ) );
    ARIADNE_TEST_EXECUTE( system.new_mode( {q|"2",r|"2"} ) );
    ARIADNE_TEST_PRINT( system );
    ARIADNE_TEST_THROWS( system.new_action( {q|"2"}, e, x>=0 ), NonExistentModeError );
    ARIADNE_TEST_THROWS( system.new_transition( {q|"1"}, e, {q|"3"} ), NonExistentModeError );
    ARIADNE_TEST_THROWS( system.new_transition( {q|"3"}, e, {q|"1"} ), NonExistentModeError );
    ARIADNE_TEST_THROWS( system.new_transition( {q|"1"}, e, {q|"1",r|"2"} ), NonExistentModeError );
    ARIADNE_TEST_EXECUTE( system.new_transition( {q|"1"}, e, {q|"2",r|"1"} ) );
}

Void
TestHybridAutomaton::test_multiple_guard()
{
    HybridAutomaton system;
    DiscreteLocation q1(1);
    DiscreteLocation q2(2);
    DiscreteEvent e1("e1");
    DiscreteEvent e2("e2");
    RealVariable x("x");
    RealVariable y("y");
    Dyadic c(0.125);
    ARIADNE_TEST_EXECUTE( system.new_mode( q1,(dot(x)=1,dot(y)=1) ) );
    ARIADNE_TEST_EXECUTE( system.new_mode( q2,(dot(x)=2) ) );
    ARIADNE_TEST_EXECUTE( system.new_action( q1, x<=c, e1, x>=0 ) );
    ARIADNE_TEST_EXECUTE( system.new_action( q2, x<=c, e1, x>=0 ) );
    ARIADNE_TEST_EXECUTE( system.new_action( q1, y<=c, e2, y>=0 ) );
    ARIADNE_TEST_THROWS( system.new_action( q1, x+y<=c, e1, x+y>=0 ), MultipleGuardError );
}

Void
TestHybridAutomaton::test_multiple_transition()
{
    HybridAutomaton system;
    DiscreteLocation q1(1);
    DiscreteLocation q2(2);
    DiscreteEvent e1("e1");
    DiscreteEvent e2("e2");
    RealVariable x("x");
    ARIADNE_TEST_EXECUTE( system.new_mode( q1,(dot(x)=1) ) );
    ARIADNE_TEST_EXECUTE( system.new_mode( q2,(dot(x)=2) ) );
    ARIADNE_TEST_EXECUTE( system.new_transition( q1, e1, q1, (prime(x)=x) ) );
    ARIADNE_TEST_EXECUTE( system.new_transition( q2, e1, q1, (prime(x)=x) ) );
    ARIADNE_TEST_EXECUTE( system.new_transition( q1, e2, q1, (prime(x)=x) ) );
    ARIADNE_TEST_THROWS( system.new_transition( q1, e1, q2, (prime(x)=x) ), MultipleTransitionError );
}

Void
TestHybridAutomaton::test_overspecified_reset()
{
    HybridAutomaton system;
    DiscreteLocation q1(1);
    DiscreteEvent e1("e1");
    DiscreteEvent e2("e2");
    DiscreteEvent e3("e3");
    DiscreteEvent e4("e4");
    DiscreteEvent e5("e5");
    RealVariable x("x");
    RealVariable y("y");
    RealVariable z("z");
    ARIADNE_TEST_EXECUTE( system.new_mode( q1,{let(x)=1}, {dot(y)=1} ) );
    ARIADNE_TEST_EXECUTE( system.new_transition( q1, e1, q1 ) ); // OK, underspecified
    ARIADNE_TEST_EXECUTE( system.new_transition( q1, e2, q1, {prime(y)=y+1} ) );
    ARIADNE_TEST_EXECUTE( system.new_transition( q1, e3, q1, {prime(y)=y+1,prime(z)=x} ) ); // OK, z in another component
    ARIADNE_TEST_THROWS( system.new_transition( q1, e4, q1, {prime(x)=x+1} ), OverspecifiedResetError );
    ARIADNE_TEST_THROWS( system.new_transition( q1, e5, q1, {prime(y)=2*x+4,prime(y)=2*(x+2)} ), OverspecifiedResetError );
}

Void
TestHybridAutomaton::test_underspecified_mode()
{
    HybridAutomaton system;
    DiscreteLocation qs(0);
    DiscreteLocation q1(1);
    DiscreteLocation q2(2);
    DiscreteLocation q3(3);
    DiscreteLocation q4(4);
    DiscreteLocation q5(5);
    DiscreteEvent e("e");
    RealVariable x("x");
    RealVariable y("y");
    RealVariable z("z");

    // The following mode should be valid
    ARIADNE_TEST_EXECUTE( system.new_mode( qs,{let(x)=y+1},{dot(y)=x+y} ) );
    ARIADNE_TEST_EXECUTE( system.new_action( qs, x+y<=1, e, x+y>=0 ) );
    ARIADNE_TEST_EXECUTE( system.new_update( qs, e, qs, {prime(y)=x+y} ) );
    ARIADNE_TEST_EXECUTE( system.check_mode( qs ) );
    // Test invalidity of the following modes
    ARIADNE_TEST_EXECUTE( system.new_mode( q1,{let(x)=y} ) );
    ARIADNE_TEST_THROWS( system.check_mode( q1 ), UnderspecifiedDynamicError );
    ARIADNE_TEST_EXECUTE( system.new_mode( q2,{dot(x)=x+y} ) );
    ARIADNE_TEST_THROWS( system.check_mode( q2 ), UnderspecifiedDynamicError );
    ARIADNE_TEST_EXECUTE( system.new_mode( q3,{dot(x)=x} ) );
    ARIADNE_TEST_EXECUTE( system.new_action( q3, y<=1, e, x>=0 ) );
    ARIADNE_TEST_THROWS( system.check_mode( q3 ), UnderspecifiedConstraintError );
    ARIADNE_TEST_EXECUTE( system.new_mode( q4,{dot(x)=x} ) );
    ARIADNE_TEST_EXECUTE( system.new_action( q4, x<=1, e, y>=0 ) );
    ARIADNE_TEST_THROWS( system.check_mode( q4 ), UnderspecifiedConstraintError );
    ARIADNE_TEST_EXECUTE( system.new_mode( q5,{dot(x)=x} ) );
    ARIADNE_TEST_EXECUTE( system.new_update( q5, e, q5, {prime(x)=y} ) );
    ARIADNE_TEST_THROWS( system.check_mode( q5 ), UnderspecifiedResetError );
}

Void
TestHybridAutomaton::test_underspecified_reset()
{
    HybridAutomaton system;
    StringVariable q("q");
    DiscreteLocation q1(q|"1");
    DiscreteLocation q2(q|"2");
    DiscreteLocation q3(q|"3");
    DiscreteLocation qt(q|"t");
    DiscreteEvent e("e");
    RealVariable x("x");
    RealVariable y("y");
    RealVariable z("z");

    ARIADNE_TEST_EXECUTE( system.new_mode( qt,{let(x)=1},{dot(y)=1} ) );
    ARIADNE_TEST_EXECUTE( system.new_mode( q1,{dot(x)=x} ) );
    ARIADNE_TEST_EXECUTE( system.new_mode( q2,{dot(x)=x} ) );
    ARIADNE_TEST_EXECUTE( system.new_mode( q3,{dot(x)=x} ) );
    ARIADNE_TEST_EXECUTE( system.new_update( q1, e, qt, {next(y)=1} ) );
    ARIADNE_TEST_EXECUTE( system.check_mode( q1 ) );
    ARIADNE_TEST_EXECUTE( system.new_update( q2, e, qt ) );
    ARIADNE_TEST_THROWS( system.check_mode( q2 ), UnderspecifiedResetError );
    ARIADNE_TEST_EXECUTE( system.new_update( q3, e, qt, {next(y)=1,prime(z)=1} ) );
    ARIADNE_TEST_THROWS( system.check_mode( q3 ), UnderspecifiedDynamicError );
}


HybridAutomaton TestHybridAutomaton::_get_tank() {
    RealConstant alpha("alpha",0.02_decimal);
    RealConstant beta("beta",0.3_decimal);

    RealVariable aperture("aperture");
    RealVariable height("height");

    HybridAutomaton automaton("tank");

    DiscreteLocation flow;

    automaton.new_mode(flow,{dot(height)=beta*aperture-alpha*height});

    ARIADNE_TEST_EQUALS(automaton.input_variables(flow),Set<RealVariable>({aperture}));

    return automaton;
}

HybridAutomaton TestHybridAutomaton::_get_valve() {
    RealConstant T("T",4);

    RealVariable aperture("aperture");

    DiscreteEvent stop_opening("stop_opening");
    DiscreteEvent stop_closing("stop_closing");
    DiscreteEvent can_open("can_open");
    DiscreteEvent can_close("can_close");

    StringVariable valve("valve");

    HybridAutomaton automaton(valve.name());

    DiscreteLocation opening(valve|"opening");
    DiscreteLocation closed(valve|"closed");
    DiscreteLocation opened(valve|"opened");
    DiscreteLocation closing(valve|"closing");

    automaton.new_mode(opened,{let(aperture)=1});
    automaton.new_mode(closed,{let(aperture)=0});
    automaton.new_mode(opening,{dot(aperture)=+1/T});
    automaton.new_mode(closing,{dot(aperture)=-1/T});

    automaton.new_transition(closed,can_open,opening,{next(aperture)=aperture});
    automaton.new_transition(opening,stop_opening,opened,aperture>=1,EventKind::URGENT);
    automaton.new_transition(opened,can_close,closing,{next(aperture)=aperture});
    automaton.new_transition(closing,stop_closing,closed,aperture<=0,EventKind::URGENT);
    automaton.new_transition(opening,can_close,closing,{next(aperture)=aperture});
    automaton.new_transition(closing,can_open,opening,{next(aperture)=aperture});

    return automaton;
}

HybridAutomaton TestHybridAutomaton::_get_controller() {
    RealConstant hmin("hmin",5.75_decimal);
    RealConstant hmax("hmax",7.75_decimal);
    RealConstant delta("delta",0.02_decimal);

    RealVariable height("height");

    DiscreteEvent can_open("can_open");
    DiscreteEvent can_close("can_close");
    DiscreteEvent must_open("must_open");
    DiscreteEvent must_close("must_close");

    StringVariable controller("controller");

    HybridAutomaton automaton(controller.name());

    DiscreteLocation rising(controller|"rising");
    DiscreteLocation falling(controller|"falling");

    automaton.new_mode(rising);
    automaton.new_mode(falling);

    automaton.new_invariant(falling,height>=hmin-delta,must_open);
    automaton.new_invariant(rising,height<=hmax+delta,must_close);

    automaton.new_transition(falling,can_open,rising,height<=hmin+delta,EventKind::PERMISSIVE);
    automaton.new_transition(rising,can_close,falling,height>=hmax-delta,EventKind::PERMISSIVE);

    return automaton;
}

Void
TestHybridAutomaton::test_build_hybrid_system()
{
    CompositeHybridAutomaton watertank_system("watertank",{_get_tank(),_get_valve(),_get_controller()});
    std::cout << "watertank_system:\n" << watertank_system << "\n";
    ARIADNE_TEST_EQUALS(watertank_system.number_of_components(),3);
    _system=watertank_system;
}

Void
TestHybridAutomaton::test_static_analysis()
{
    StringVariable valve("valve"), controller("controller");
    DiscreteLocation valve_opening_controller_rising={valve|"opening",controller|"rising"};

    ARIADNE_TEST_PRINT (_system.mode(valve_opening_controller_rising));

    HybridAutomaton const& tank_automaton = _system.component(0);
    HybridAutomaton const& valve_automaton = _system.component(1);
    HybridAutomaton const& controller_automaton = _system.component(2);
    ARIADNE_TEST_PRINT(tank_automaton);
    ARIADNE_TEST_PRINT(valve_automaton);
    ARIADNE_TEST_PRINT(controller_automaton);

    ARIADNE_TEST_EXECUTE(_system.check_mode(valve_opening_controller_rising));
    ARIADNE_TEST_EXECUTE(_system.discrete_reachability({valve_opening_controller_rising}));
    ARIADNE_TEST_EXECUTE(_system.check_reachable_modes({valve_opening_controller_rising}));
}


Int main(Int argc, const char* argv[]) {
    if (not CommandLineInterface::instance().acquire(argc,argv)) return -1;
    ARIADNE_TEST_CALL(TestHybridAutomaton().test());
    return ARIADNE_TEST_FAILURES;
}

