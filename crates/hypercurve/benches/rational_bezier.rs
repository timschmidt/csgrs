#[path = "../tests/support/mod.rs"]
mod support;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Axis2, Classification, Curve2, CurveCertainty, CurveIntersectionResult2, Point2,
    PredicatePolicy, RationalBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).expect("benchmark denominator is nonzero")
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

/// Exact contact and overlap evidence through the public curve interface.
fn contacts(
    first: &RationalBezier2,
    second: &RationalBezier2,
    policy: &PredicatePolicy,
) -> CurveIntersectionResult2 {
    let evidence = crate::support::under(policy, || {
        Curve2::from(first.clone()).intersect_curve(&Curve2::from(second.clone()))
    })
    .expect("benchmark contacts are exact");
    assert_eq!(evidence.certainty, CurveCertainty::Certified);
    let evidence = evidence.value;
    assert!(evidence.is_complete(), "benchmark contacts are complete");
    evidence
}

fn evidence_count(evidence: &CurveIntersectionResult2) -> usize {
    evidence.contacts().len() + evidence.overlaps().len()
}

fn large_rational_control_count() -> usize {
    std::env::var("HYPERCURVE_BENCH_RATIONAL_CONTROLS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(64)
        .clamp(2, i32::MAX as usize)
}

fn large_rational_iterations() -> u32 {
    std::env::var("HYPERCURVE_BENCH_RATIONAL_ITERATIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(10)
        .max(1)
}

fn large_rational_inputs(control_count: usize) -> (Vec<Point2>, Vec<Real>) {
    let controls = (0..control_count)
        .map(|index| {
            let x = i32::try_from(index).unwrap();
            let y = i32::try_from(index.wrapping_mul(19) % 37).unwrap() - 18;
            p(x, y)
        })
        .collect();
    let weights = (0..control_count)
        .map(|index| r(i32::try_from(index % 5 + 1).unwrap()))
        .collect();
    (controls, weights)
}

fn bench_large_rational_bezier() {
    let policy = PredicatePolicy::STRICT;
    let control_count = large_rational_control_count();
    let iterations = large_rational_iterations();
    let (controls, weights) = large_rational_inputs(control_count);
    let parameter = q(1, 2);
    let operation = std::env::var("HYPERCURVE_BENCH_RATIONAL_OPERATION").ok();
    let run_evaluation = operation
        .as_deref()
        .is_none_or(|value| value == "evaluation");
    let run_split = operation.as_deref().is_none_or(|value| value == "split");

    if run_evaluation {
        let started = Instant::now();
        let mut cold_checksum = 0_usize;
        for _ in 0..iterations {
            let curve = RationalBezier2::try_new(controls.clone(), weights.clone()).unwrap();
            let point =
                crate::support::under_value(&policy, || curve.point_at(&parameter)).unwrap();
            cold_checksum ^= black_box(point.x().to_f64_lossy().unwrap().to_bits() as usize);
        }
        let elapsed = started.elapsed();
        println!(
            "rational_bezier_large_cold_evaluation_{control_count}_controls: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cold_checksum}",
            elapsed / iterations
        );

        let curve = RationalBezier2::try_new(controls.clone(), weights.clone()).unwrap();
        crate::support::under_value(&policy, || curve.point_at(&parameter)).unwrap();
        let started = Instant::now();
        let mut cached_checksum = 0_usize;
        for _ in 0..iterations {
            let point =
                crate::support::under_value(&policy, || curve.point_at(&parameter)).unwrap();
            cached_checksum ^= black_box(point.y().to_f64_lossy().unwrap().to_bits() as usize);
        }
        let elapsed = started.elapsed();
        println!(
            "rational_bezier_large_cached_evaluation_{control_count}_controls: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cached_checksum}",
            elapsed / iterations
        );
    }

    if run_split {
        let curve = RationalBezier2::try_new(controls, weights).unwrap();
        let started = Instant::now();
        let mut split_checksum = 0_usize;
        for _ in 0..iterations {
            let (left, right) = decided(
                crate::support::under_classified_result(&policy, || {
                    curve.split_at_exact(&parameter)
                })
                .unwrap(),
            );
            split_checksum ^=
                black_box(left.homogeneous_controls().len() + right.homogeneous_controls().len());
        }
        let elapsed = started.elapsed();
        println!(
            "rational_bezier_large_exact_split_{control_count}_controls: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={split_checksum}",
            elapsed / iterations
        );
    }
}

fn main() {
    if std::env::var_os("HYPERCURVE_BENCH_RATIONAL_ONLY").is_some() {
        bench_large_rational_bezier();
        return;
    }

    let policy = PredicatePolicy::STRICT;
    let curve = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .expect("benchmark curve is valid");
    let general = hypercurve::Curve2::from(curve.clone());
    let start = hypercurve::CurvePoint2::from(curve.start().clone());
    crate::support::under(&policy, || general.point_locations(&start))
        .expect("benchmark point incidence is exact");

    let stationary_monotone_curve = || {
        RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(0, 0), p(1, 0)], vec![r(1); 4])
            .expect("benchmark stationary curve is valid")
    };
    let cold_monotonicity_inputs = (0..250)
        .map(|_| stationary_monotone_curve())
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut cold_monotonicity_count = 0_usize;
    for curve in &cold_monotonicity_inputs {
        cold_monotonicity_count = cold_monotonicity_count.wrapping_add(black_box(usize::from(
            crate::support::under_value(black_box(&policy), || {
                black_box(curve).axis_is_monotone(Axis2::X)
            })
            .expect("benchmark monotonicity is exact"),
        )));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_exact_mixed_axis_monotonicity: {} curves in {elapsed:?} ({:?}/curve), checksum={cold_monotonicity_count}",
        cold_monotonicity_inputs.len(),
        elapsed / u32::try_from(cold_monotonicity_inputs.len()).unwrap()
    );

    let high_degree_monotonicity_inputs = (0..250)
        .map(|_| {
            RationalBezier2::try_new(
                (0..=12)
                    .map(|index| p(index, (index * index) % 7))
                    .collect(),
                (0..=12).map(|index| r(1 + index % 3)).collect(),
            )
            .expect("benchmark high-degree curve is valid")
        })
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut high_degree_monotonicity_count = 0_usize;
    for curve in &high_degree_monotonicity_inputs {
        high_degree_monotonicity_count =
            high_degree_monotonicity_count.wrapping_add(black_box(usize::from(
                crate::support::under_value(black_box(&policy), || {
                    black_box(curve).axis_is_monotone(Axis2::X)
                })
                .expect("benchmark high-degree monotonicity is exact"),
            )));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_exact_degree_12_axis_monotonicity: {} curves in {elapsed:?} ({:?}/curve), checksum={high_degree_monotonicity_count}",
        high_degree_monotonicity_inputs.len(),
        elapsed / u32::try_from(high_degree_monotonicity_inputs.len()).unwrap()
    );

    let stationary_monotone = stationary_monotone_curve();
    assert!(
        crate::support::under_value(&policy, || stationary_monotone.axis_is_monotone(Axis2::X))
            .expect("benchmark monotonicity is exact")
    );
    let monotonicity_iterations = 100_000_u32;
    let started = Instant::now();
    let mut monotonicity_count = 0_usize;
    for _ in 0..monotonicity_iterations {
        monotonicity_count = monotonicity_count.wrapping_add(black_box(usize::from(
            crate::support::under_value(black_box(&policy), || {
                black_box(&stationary_monotone).axis_is_monotone(Axis2::X)
            })
            .expect("benchmark monotonicity is exact"),
        )));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_retained_mixed_axis_monotonicity: {monotonicity_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={monotonicity_count}",
        elapsed / monotonicity_iterations
    );

    let reversing_inputs = (0..2_000)
        .map(|_| {
            RationalBezier2::try_new(vec![p(0, 0), p(1, 0), p(1, 0), p(0, 0)], vec![r(1); 4])
                .expect("benchmark reversing curve is valid")
        })
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut reversing_count = 0_usize;
    for curve in &reversing_inputs {
        reversing_count += usize::from(
            crate::support::under_value(&policy, || curve.axis_is_monotone(Axis2::X))
                .expect("benchmark reversing monotonicity is exact"),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_endpoint_sign_reversal_monotonicity: {} curves in {elapsed:?} ({:?}/curve), checksum={reversing_count}",
        reversing_inputs.len(),
        elapsed / u32::try_from(reversing_inputs.len()).unwrap()
    );

    let iterations = 5_000_u32;
    let started = Instant::now();
    let mut incidence_count = 0_usize;
    for _ in 0..iterations {
        let incidence =
            crate::support::under(&policy, || general.point_locations(black_box(&start)))
                .expect("benchmark point incidence is exact")
                .value;
        incidence_count = incidence_count.wrapping_add(black_box(match incidence {
            hypercurve::CurvePointLocations2::EntireCurve => 1,
            hypercurve::CurvePointLocations2::Locations(locations) => locations.len(),
        }));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_cached_point_incidence: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={incidence_count}",
        elapsed / iterations
    );

    let related_first = decided(
        crate::support::under_classified_result(&policy, || {
            curve.subcurve_between_exact(&Real::zero(), &q(3, 4))
        })
        .expect("benchmark source subdivision is exact"),
    );
    let related_second = decided(
        crate::support::under_classified_result(&policy, || {
            curve.subcurve_between_exact(&q(1, 4), &Real::one())
        })
        .expect("benchmark source subdivision is exact"),
    );
    let lineage_iterations = 5_000_u32;
    let started = Instant::now();
    let mut lineage_count = 0_usize;
    for _ in 0..lineage_iterations {
        let contacts = contacts(&related_first, &related_second, &policy);
        lineage_count = lineage_count.wrapping_add(black_box(contacts.overlaps().len()));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_retained_lineage_partial_overlap: {lineage_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={lineage_count}",
        elapsed / lineage_iterations
    );

    let nonlinear_line = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(q(1, 4), r(0)), p(1, 0)],
        vec![r(1), r(1), r(1)],
    )
    .expect("benchmark line image is valid");
    let partial_line =
        RationalBezier2::try_new(vec![Point2::new(q(1, 2), r(0)), p(1, 0)], vec![r(1), r(1)])
            .expect("benchmark partial line image is valid");
    let algebraic_overlap_iterations = 500_u32;
    let started = Instant::now();
    let mut algebraic_overlap_count = 0_usize;
    for _ in 0..algebraic_overlap_iterations {
        let contacts = contacts(&nonlinear_line, &partial_line, &policy);
        algebraic_overlap_count = algebraic_overlap_count.wrapping_add(black_box(
            contacts
                .overlaps()
                .iter()
                .filter(|overlap| overlap.first_range().start().scalar().is_none())
                .count(),
        ));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_line_image_overlap: {algebraic_overlap_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={algebraic_overlap_count}",
        elapsed / algebraic_overlap_iterations
    );

    let partial_parabola = RationalBezier2::try_new(
        vec![
            Point2::new(q(1, 2), q(1, 4)),
            Point2::new(q(3, 4), q(1, 2)),
            p(1, 1),
        ],
        vec![r(1); 3],
    )
    .expect("benchmark partial parabola is valid");
    let nonlinear_parabola = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(1, 8), r(0)),
            Point2::new(q(1, 3), q(1, 24)),
            Point2::new(q(5, 8), q(1, 4)),
            p(1, 1),
        ],
        vec![r(1); 5],
    )
    .expect("benchmark nonlinear parabola is valid");
    let graph_overlap_iterations = 250_u32;
    let started = Instant::now();
    let mut graph_overlap_count = 0_usize;
    for _ in 0..graph_overlap_iterations {
        let contacts = contacts(&partial_parabola, &nonlinear_parabola, &policy);
        graph_overlap_count = graph_overlap_count.wrapping_add(black_box(
            contacts
                .overlaps()
                .iter()
                .filter(|overlap| overlap.second_range().start().scalar().is_none())
                .count(),
        ));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_polynomial_graph_overlap: {graph_overlap_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={graph_overlap_count}",
        elapsed / graph_overlap_iterations
    );

    let parabola = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1)],
        vec![r(1), r(1), r(1)],
    )
    .expect("benchmark parabola is valid");
    let horizontal = RationalBezier2::try_new(
        vec![Point2::new(r(0), q(1, 2)), Point2::new(r(1), q(1, 2))],
        vec![r(1), r(1)],
    )
    .expect("benchmark line is valid");
    let crossing = contacts(&parabola, &horizontal, &policy);
    let selected_parameter = crossing.contacts()[0].first().local_parameter().clone();
    assert!(
        selected_parameter.scalar().is_none(),
        "benchmark expected an algebraic parameter"
    );

    let contact_iterations = 250_u32;
    let started = Instant::now();
    let mut contact_count = 0_usize;
    for _ in 0..contact_iterations {
        let contacts = contacts(&parabola, &horizontal, &policy);
        contact_count = contact_count.wrapping_add(black_box(evidence_count(&contacts)));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_contact_replay: {contact_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={contact_count}",
        elapsed / contact_iterations
    );

    let pi_conic = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(q(1, 2), r(0)), p(1, 1)],
        vec![Real::one(), Real::pi(), Real::one()],
    )
    .expect("benchmark pi-weight conic is valid");
    let elevated_horizontal = RationalBezier2::try_new(
        vec![
            Point2::new(r(0), q(1, 2)),
            Point2::new(q(1, 3), q(1, 2)),
            Point2::new(q(2, 3), q(1, 2)),
            Point2::new(r(1), q(1, 2)),
        ],
        vec![Real::one(); 4],
    )
    .expect("benchmark degree-elevated horizontal line is valid");
    let pi_conic_contact_iterations = 100_u32;
    let started = Instant::now();
    let mut pi_conic_contact_count = 0_usize;
    for _ in 0..pi_conic_contact_iterations {
        let contacts = contacts(&pi_conic, &elevated_horizontal, &policy);
        pi_conic_contact_count =
            pi_conic_contact_count.wrapping_add(black_box(evidence_count(&contacts)));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_pi_conic_cubic_contacts: {pi_conic_contact_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={pi_conic_contact_count}",
        elapsed / pi_conic_contact_iterations
    );

    let general_parabola = hypercurve::Curve2::from(parabola.clone());
    let derivative_iterations = 250_u32;
    let started = Instant::now();
    let mut derivative_count = 0_usize;
    for _ in 0..derivative_iterations {
        let derivatives = crate::support::under(&policy, || {
            general_parabola.derivatives_at(black_box(&selected_parameter), black_box(3))
        })
        .expect("algebraic derivatives remain exact")
        .into_value();
        derivative_count = derivative_count.wrapping_add(black_box(derivatives.len()));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_derivatives_1_through_3: {derivative_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={derivative_count}",
        elapsed / derivative_iterations
    );

    let exact_parameter = q(1, 2);
    let exact_derivative_iterations = 20_000_u32;
    let started = Instant::now();
    let mut exact_derivative_count = 0_usize;
    for _ in 0..exact_derivative_iterations {
        let derivatives = crate::support::under_value(&policy, || {
            curve.derivatives_at(black_box(&exact_parameter), black_box(3))
        })
        .expect("exact benchmark derivatives are certified");
        exact_derivative_count = exact_derivative_count.wrapping_add(black_box(derivatives.len()));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_exact_derivatives_1_through_3: {exact_derivative_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={exact_derivative_count}",
        elapsed / exact_derivative_iterations
    );

    let disjoint_cubic =
        RationalBezier2::try_new(vec![p(10, 0), p(11, 1), p(11, 2), p(10, 3)], vec![r(1); 4])
            .expect("benchmark disjoint cubic is valid");
    let conic = RationalBezier2::try_new(vec![p(1, 0), p(1, 1), p(0, 1)], vec![r(1), r(1), r(2)])
        .expect("benchmark conic is valid");
    let disjoint_contacts_iterations = 2_000_u32;
    let started = Instant::now();
    let mut disjoint_contacts_count = 0_usize;
    for _ in 0..disjoint_contacts_iterations {
        let contacts = contacts(
            black_box(&conic),
            black_box(&disjoint_cubic),
            black_box(&policy),
        );
        disjoint_contacts_count =
            disjoint_contacts_count.wrapping_add(black_box(usize::from(contacts.is_disjoint())));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_disjoint_conic_cubic_cold_contacts: {disjoint_contacts_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={disjoint_contacts_count}",
        elapsed / disjoint_contacts_iterations
    );

    let immediate_iterations = 250_u32;
    let started = Instant::now();
    let mut immediate_count = 0_usize;
    for _ in 0..immediate_iterations {
        let contacts = contacts(
            black_box(&parabola),
            black_box(&horizontal),
            black_box(&policy),
        );
        immediate_count = immediate_count.wrapping_add(black_box(evidence_count(&contacts)));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_immediate_contacts: {immediate_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={immediate_count}",
        elapsed / immediate_iterations
    );

    let parabola_curve = hypercurve::Curve2::from(parabola.clone());
    let horizontal_curve = hypercurve::Curve2::from(horizontal.clone());
    let started = Instant::now();
    let mut topology_count = 0_usize;
    for _ in 0..immediate_iterations {
        let topology = crate::support::under(black_box(&policy), || {
            black_box(&parabola_curve).intersection_topology(black_box(&horizontal_curve))
        })
        .unwrap()
        .into_value();
        topology_count = topology_count
            .wrapping_add(black_box(topology.first().len() + topology.second().len()));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_immediate_topology: {immediate_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={topology_count}",
        elapsed / immediate_iterations
    );

    let elevation_inputs = (0..1_000)
        .map(|_| {
            RationalBezier2::try_new(
                vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
                vec![r(1), r(2), r(3), r(4)],
            )
            .unwrap()
        })
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut elevation_count = 0_usize;
    for source in &elevation_inputs {
        elevation_count =
            elevation_count.wrapping_add(black_box(source.elevated_to_degree(8).unwrap().degree()));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_exact_degree_elevation: {} curves in {elapsed:?} ({:?}/curve), checksum={elevation_count}",
        elevation_inputs.len(),
        elapsed / u32::try_from(elevation_inputs.len()).unwrap()
    );

    let retained_elevation_iterations = 100_000_u32;
    let started = Instant::now();
    let mut retained_elevation_count = 0_usize;
    for _ in 0..retained_elevation_iterations {
        retained_elevation_count = retained_elevation_count
            .wrapping_add(black_box(curve.elevated_to_degree(8).unwrap().degree()));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_retained_degree_elevation: {retained_elevation_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_elevation_count}",
        elapsed / retained_elevation_iterations
    );

    bench_large_rational_bezier();
}
