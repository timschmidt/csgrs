/***************************************************************************
 *            power_converters.hpp
 *
 *  Copyright  2017-20  Luca Geretti, Pieter Collins
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

#include "ariadne.hpp"

namespace Ariadne {

template<class IS, class SYS, class SS> struct Problem {
    IS initial_set; SYS system; SS safe_set; };
template<class IS, class SYS, class SS> Problem<IS,SYS,SS> make_problem(IS initial_set, SYS system, SS safe_set) {
    return Problem<IS,SYS,SS>{initial_set,system,safe_set}; }

TimeVariable t;
RealVariable V("V"), I("I");
RealVariable tau("tau");

RealInterval Vs_range(11.95_dec, 12.05_dec);
RealInterval R_range(0.95_dec, 1.05_dec);
RealInterval C_range(23.75_dec, 26.25_dec);
RealInterval L_range(47.5_dec, 52.5_dec);
RealInterval T_range(24.5_dec, 25.5_dec);

RealConstant Vs("Vs",12.0_dec);    // Input Voltage Vs [11.95, 12.05] V
RealConstant Vref("Vref",5.0_dec); // Desired Output Voltage Vref 5V
RealConstant Vo("Vo",5.0_dec);     // Actual Output Voltage Vo 5V ±
RealConstant R("R",1.0_dec);       // Load Resistance R [0.95, 1.05]Ω
RealConstant C("C",25.0_dec);      // Capacitor C [23.75, 26.25] uF
RealConstant L("L",50.0_dec);      // Inductor L [47.5, 52.5] uH
RealConstant T("T",25.0_dec);      // Switching Period T [24.5, 25.5] us
RealConstant D("D",0.6_dec);     // Switch-closed duty cycle D 0.6
RealConstant Verr("Verr",0.1_dec); // Desired Output Voltage Vref 5V

StringVariable swtch("switch");
StringConstant open("open");
StringConstant closed("closed");

DiscreteEvent close_switch("close_switch");
DiscreteEvent open_switch("open_switch");

decltype(auto) make_buck_problem() {

    HybridAutomaton buck("buck");

    HybridBoundedConstraintSet initial_set(swtch|open,{V==0,I==0,tau==0});
//    HybridBoundedConstraintSet initial_set(swtch|open,{0.11_dec<=V<=0.12_dec,1.11_dec<=I<=1.12_dec,0.01<=tau<=0.001});

    HybridConstraintSet safe_set;
    safe_set.adjoin(swtch|open,{V<=Vref+Verr});
    safe_set.adjoin(swtch|closed,{V<=Vref+Verr});

    buck.new_mode(swtch|open, {dot(I)=-V/L, dot(V)=I/C-V/(R*C), dot(tau)=1});
    buck.new_mode(swtch|closed, {dot(I)=(1-V)/L, dot(V)=I/C-V/(R*C), dot(tau)=1});

    buck.new_transition(swtch|closed,open_switch,swtch|open, next({I,V,tau})={I,V,Real(0)}, tau>=(1-D)*T, urgent);
    buck.new_transition(swtch|open,close_switch,swtch|closed, {next(I)=I,next(V)=V,next(tau)=0}, tau>=D*T, urgent);

    return make_problem(initial_set,buck,safe_set);
}

} // namespace Ariadne
