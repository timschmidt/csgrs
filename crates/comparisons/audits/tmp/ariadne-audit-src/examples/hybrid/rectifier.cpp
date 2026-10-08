/***************************************************************************
 *            rectifier.cpp
 *
 *  Copyright  2009-20  Luca Geretti
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
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

#include "ariadne_main.hpp"

void ariadne_main()
{
    Real amplitude(4.0_dec);
    Real frequency(50.0_dec);
    Real Ron (10.0_dec);
    Real Cl = 0.0001_dec;
    Real Rl (1000.0_dec);

    /// Introduces the dynamics parameters
    Vector<Real> parameters(5);
    parameters[0] = amplitude; /// Amplitude of the input voltage, Vi
    parameters[1] = frequency; /// Sinusoid frequency, f
    parameters[2] = Ron; /// Diode resistance when on, Ron
    parameters[3] = Cl; /// Load capacitance, Cl
    parameters[4] = Rl; /// Load resistance, Rl

    RealConstant pi_c("pi",pi);

    /// Introduces the global parameters
    Real TIME_LIMIT = 1/frequency;
    //float TIME_LIMIT = 0.0042;
    Int TRAN_LIMIT = 1;
    double MAX_ENCL_RADIUS = 1.0;
    double MAX_STEP_SIZE = 1e-2/frequency.get_d();
    //float LOCK_TOGRID_TIME = 2.0/frequency;
    //double LOCK_TOGRID_TIME = 0.25/frequency;
    //Int MAX_GRID_DEPTH = 7;
    Bool ENABLE_SUBDIV=false;

    /// Build the Hybrid System

    /// Create a HybridAutomaton object
    HybridAutomaton rectifier_automaton("rectifier_automaton");

    // Create the coordinates
    RealVariable t("t"); // Time
    RealVariable vi("Vin"); // Input voltage
    RealVariable vo("Vout"); // Output voltage

    Real one=1;

    /// Create the discrete states
    StringVariable rectifier("rectifier");
    DiscreteLocation offoff(rectifier|"offoff");
    DiscreteLocation onoff(rectifier|"onoff");
    DiscreteLocation offon(rectifier|"offon");
    DiscreteLocation onon(rectifier|"onon");

    /// Create the discrete events
    DiscreteEvent resettime("reset_time");
    DiscreteEvent jump1("jump1"), jump2("jump2"), jump3("jump3");

    /// Create the resets

    /// Reset the time (t^=0,vi^=vi,vo^=vo)
    PrimedRealAssignments resettime_r( next({t,vi,vo}) = {Real(0),vi,vo} );
    /// Do nothing (t^=t,vi^=vi,vo^=vo)
    PrimedRealAssignments noop_r( next({t,vi,vo}) = {t,vi,vo} );

    /// Create the guards
    Real f=parameters[1];
    /// Guard for the reset of time (t>=1/f)
    ContinuousPredicate resettime_g( t>=1/f );
    /// Guard for the jump from onoff to offoff (vi-vo<=0)
    ContinuousPredicate onoff_offoff_g( vi<=vo );
    /// Guard for the jump from offon to offoff (-vi-vo<=0)
    ContinuousPredicate offon_offoff_g( vi+vo>=0 );
    /// Guard for the jump from offoff to onoff (vi-vo>=0)
    ContinuousPredicate offoff_onoff_g( vi>=vo );
    /// Guard for the jump from onon to onoff (-vi-vo<=0)
    ContinuousPredicate onon_onoff_g( vi+vo>=0 );
    /// Guard for the jump from offoff to offon (-vi-vo>=0)
    ContinuousPredicate offoff_offon_g( vi+vo<=0 );
    /// Guard for the jump from onon to offon (vi-vo<=0)
    ContinuousPredicate onon_offon_g( vi<=vo );
    /// Guard for the jump from offon to onon (vi-vo>=0)
    ContinuousPredicate offon_onon_g( vi>=vo );
    /// Guard for the jump from onoff to onon (-vi-vo>=0)
    ContinuousPredicate onoff_onon_g( vi+vo<=0 );

    /// Build the automaton

    /// Create the dynamics

    /// Dynamics for the case of both diodes being off
    /// t'=1, vi'= A*cos(2*pi*f*t), vo'=-vo/(Rl*Cl)
    DottedRealAssignments offoff_d( dot({t,vi,vo}) = {one,amplitude*2*pi*frequency*cos(2*pi*frequency*t),-vo/(Rl*Cl)} );
    /// Dynamics for the case of the first diode being on, the second being off
    /// t'=1, vi'= A*cos(2*pi*f*t), vo'=-vo/(Rl*Cl)+(vi-vo)/(Ron*Cl)
    RealExpressions onoff_d({one,amplitude*2*pi*frequency*cos(2*pi*frequency*t),-vo/(Rl*Cl)+(vi-vo)/(Ron*Cl)});
    /// Dynamics for the case of the first diode being off, the second being on
    /// t'=1, vi'= A*cos(2*pi*f*t), vo'=-vo/(Rl*Cl)-(vi+vo)/(Ron*Cl)
    RealExpressions offon_d({one,amplitude*2*pi*frequency*cos(2*pi*frequency*t),-vo/(Rl*Cl)+(vo-vi)/(Ron*Cl)});
    /// Dynamics for the case of both diodes being on
    /// t'=1, vi'= A*cos(2*pi*f*t), vo'=-vo/(Rl*Cl)-2*vo/(Ron*Cl)
    RealExpressions onon_d({one,amplitude*2*pi*frequency*cos(2*pi*frequency*t),-vo/(Rl*Cl)-2*vo/(Ron*Cl)});

    List<RealVariable> space( {t,vi,vo} );
    /// Locations
    rectifier_automaton.new_mode(offoff,offoff_d);
    rectifier_automaton.new_mode(onoff,dot(space)=onoff_d);
    rectifier_automaton.new_mode(offon,dot(space)=offon_d);
    rectifier_automaton.new_mode(onon,dot(space)=onon_d);
    /// OffOff events
    rectifier_automaton.new_transition(offoff,resettime,offoff,resettime_r,resettime_g,EventKind::URGENT);
    rectifier_automaton.new_transition(offoff,jump1,onoff,noop_r,offoff_onoff_g,EventKind::URGENT);
    rectifier_automaton.new_transition(offoff,jump2,offon,noop_r,offoff_offon_g,EventKind::URGENT);
    /// OnOff events
    rectifier_automaton.new_transition(onoff,resettime,onoff,resettime_r,resettime_g,EventKind::URGENT);
    rectifier_automaton.new_transition(onoff,jump1,offoff,noop_r,onoff_offoff_g,EventKind::URGENT);
    rectifier_automaton.new_transition(onoff,jump3,onon,noop_r,onoff_onon_g,EventKind::URGENT);
    /// OffOn events
    rectifier_automaton.new_transition(offon,resettime,offon,resettime_r,resettime_g,EventKind::URGENT);
    rectifier_automaton.new_transition(offon,jump1,offoff,noop_r,offon_offoff_g,EventKind::URGENT);
    rectifier_automaton.new_transition(offon,jump3,onon,noop_r,offon_onon_g,EventKind::URGENT);
    /// OnOn events
    rectifier_automaton.new_transition(onon,resettime,onon,resettime_r,resettime_g,EventKind::URGENT);
    rectifier_automaton.new_transition(onon,jump2,onoff,noop_r,onon_onoff_g,EventKind::URGENT);
    rectifier_automaton.new_transition(onon,jump3,offon,noop_r,onon_offon_g,EventKind::URGENT);


    /// Finished building the automaton

    CONCLOG_PRINTLN_VAR(rectifier);

    /// Compute the system evolution

    /// Create a GeneralHybridEvolver object
    GeneralHybridEvolver evolver(rectifier_automaton);

    /// Set the evolution parameters
    evolver.configuration().set_maximum_enclosure_radius(MAX_ENCL_RADIUS);
    evolver.configuration().set_maximum_step_size(MAX_STEP_SIZE);
    evolver.configuration().set_enable_subdivisions(ENABLE_SUBDIV);
    CONCLOG_PRINTLN_VAR(evolver.configuration());

    // Declare the type to be used for the system evolution
    typedef GeneralHybridEvolver::OrbitType OrbitType;

    CONCLOG_PRINTLN("Computing evolution...");

    RealVariablesBox initial_box({t==0, vi==0, vo==Real(0.8_dec)*parameters[0]});
    HybridSet initial_set(offoff,initial_box);

    CONCLOG_PRINTLN(initial_set);

    HybridTime evolution_time(TIME_LIMIT,TRAN_LIMIT);

    CONCLOG_PRINTLN("Computing orbit... ");
    OrbitType orbit = evolver.orbit(initial_set,evolution_time,Semantics::UPPER);
    CONCLOG_PRINTLN("done.");

    CONCLOG_PRINTLN_VAR(orbit.final().size());

    Axes2d graphic_axes(0.0<=t<=1.0/parameters[1].get_d(),-parameters[0]<=vi<=parameters[0]);
    Axes2d graphic_axes2(-parameters[0]<=t<=parameters[0],2<=vi<=parameters[0]);

    CONCLOG_PRINTLN("Plotting results...");

    plot("rectifier_orbit_t_vin", graphic_axes, Colour(0.0,0.5,1.0), orbit);
    plot("rectifier_orbit_t_vout", graphic_axes, Colour(0.0,0.5,1.0), orbit);
    plot("rectifier_orbit_vin_vout", graphic_axes2, Colour(0.0,0.5,1.0), orbit);
}
