/***************************************************************************
 *            noisy-utilities.hpp
 *
 *  Copyright  2008-18 Luca Geretti
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
#include "utility/stopwatch.hpp"

namespace Ariadne {

typedef Tuple<String,DottedRealAssignments,RealVariablesBox,RealVariablesBox,Real,double> SystemType;
typedef Real ContinuousTimeType;

void run_single(String name, DifferentialInclusion const& ivf, RealVariablesBox const& initial, ContinuousTimeType evolution_time, double step, List<InputApproximation> approximations, Reconditioner const& reconditioner);

void run_noisy_system(String name, DottedRealAssignments const& dynamics, RealVariablesBox const& inputs, RealVariablesBox const& initial, ContinuousTimeType evolution_time, double step);
void run_noisy_system(SystemType system);

inline ApproximateDouble score(ListSet<LabelledEnclosure> const& bbx) {
    return 1.0/std::pow(volume(bbx.bounding_box().euclidean_set()).get_d(),1.0/bbx.bounding_box().dimension());
}

void run_single(String name, DifferentialInclusion const& ivf, RealVariablesBox const& initial, Real evolution_time, ApproximateDouble step, List<InputApproximation> approximations, IntegratorInterface const& integrator, Reconditioner const& reconditioner) {
    CONCLOG_SCOPE_CREATE;
    auto evolver = DifferentialInclusionEvolver(ivf, integrator, reconditioner);
    evolver.configuration().set_approximations(approximations);
    evolver.configuration().set_maximum_step_size(step);

    Stopwatch<Milliseconds> sw;
    CONCLOG_PRINTLN("Evolving...");
    auto orbit = evolver.orbit(initial,evolution_time);
    CONCLOG_PRINTLN("Done.")
    sw.click();

    CONCLOG_PRINTLN("Score: " << score(orbit.final()) << ", time: " << sw.elapsed_seconds() << " s");

    CONCLOG_PRINTLN("Plotting...");
    auto n = ivf.dimension();
    DifferentialInclusion::EnclosureType::BoundingBoxType::EuclideanSetType graphics_box(n);
    for (auto set: orbit.reach()) {
        graphics_box = hull(graphics_box,set.euclidean_set().bounding_box());
    }
    for (SizeType xi = 0; xi < n; ++xi) {
        auto x = ivf.state_space()[xi];
        for (SizeType yi = xi+1; yi < n; ++yi) {
            auto y = ivf.state_space()[yi];
            LabelledFigure fig(Axes2d(graphics_box[xi].lower_bound().get_d(),x,graphics_box[xi].upper_bound().get_d(),
                                      graphics_box[yi].lower_bound().get_d(),y,graphics_box[yi].upper_bound().get_d()));
            fig.draw(orbit);
            char num_char[64] = "";
            if (n > 2) snprintf(num_char,64,"[%s,%s]",x.name().c_str(),y.name().c_str());
            fig.write((name+num_char).c_str());
        }
    }
    CONCLOG_PRINTLN("Done.")
}

void run_noisy_system(String name, const DottedRealAssignments& dynamics, const RealVariablesBox& inputs,
              const RealVariablesBox& initial, Real evolution_time, double step) {

    DifferentialInclusion ivf(dynamics, inputs);

    SizeType period_of_parameter_reduction=12;
    ExactDouble ratio_of_parameters_to_keep=6.0_x;
    double sw_threshold = 1e-8;
    ThresholdSweeperDP sweeper(DoublePrecision(),sw_threshold);

    List<InputApproximation> approximations;
    approximations.append(ZeroApproximation());
    approximations.append(ConstantApproximation());
    approximations.append(AffineApproximation());
    approximations.append(SinusoidalApproximation());
    approximations.append(PiecewiseApproximation());

    TaylorPicardIntegrator integrator(
            step_maximum_error=1e-3,
            sweeper,
            lipschitz_tolerance=0.5_x,
            minimum_temporal_order=4,
            maximum_temporal_order=12);

    LohnerReconditioner reconditioner(initial.variables().size(),inputs.variables().size(),period_of_parameter_reduction,ratio_of_parameters_to_keep);

    run_single(name,ivf,initial,evolution_time,step,approximations,integrator,reconditioner);
}

void run_noisy_system(SystemType system) {
    run_noisy_system(std::get<0>(system),std::get<1>(system),std::get<2>(system),std::get<3>(system),std::get<4>(system),std::get<5>(system));
}

}
