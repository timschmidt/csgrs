use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Axis2, BezierParameter2, Classification, CurvePolicy, Point2, RationalBezier2,
    RationalBezierIntersectionCandidates2, RationalBezierIntersectionContacts2,
    RationalBezierPointIncidence2, Real,
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

fn main() {
    let policy = CurvePolicy::certified();
    let curve = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .expect("benchmark curve is valid");
    curve
        .point_incidence(curve.start(), &policy)
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
            black_box(curve)
                .axis_is_monotone(Axis2::X, black_box(&policy))
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
                black_box(curve)
                    .axis_is_monotone(Axis2::X, black_box(&policy))
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
        stationary_monotone
            .axis_is_monotone(Axis2::X, &policy)
            .expect("benchmark monotonicity is exact")
    );
    let monotonicity_iterations = 100_000_u32;
    let started = Instant::now();
    let mut monotonicity_count = 0_usize;
    for _ in 0..monotonicity_iterations {
        monotonicity_count = monotonicity_count.wrapping_add(black_box(usize::from(
            black_box(&stationary_monotone)
                .axis_is_monotone(Axis2::X, black_box(&policy))
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
            curve
                .axis_is_monotone(Axis2::X, &policy)
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
        let incidence = curve
            .point_incidence(black_box(curve.start()), &policy)
            .expect("benchmark point incidence is exact");
        incidence_count = incidence_count.wrapping_add(black_box(match incidence {
            RationalBezierPointIncidence2::EntireCurve => 1,
            RationalBezierPointIncidence2::Parameters(parameters) => parameters.len(),
        }));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_cached_point_incidence: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={incidence_count}",
        elapsed / iterations
    );

    let related_first = decided(
        curve
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .expect("benchmark source subdivision is exact"),
    );
    let related_second = decided(
        curve
            .subcurve_between_exact(&q(1, 4), &Real::one(), &policy)
            .expect("benchmark source subdivision is exact"),
    );
    let lineage_iterations = 5_000_u32;
    let started = Instant::now();
    let mut lineage_count = 0_usize;
    for _ in 0..lineage_iterations {
        let contacts = related_first
            .intersection_contacts(&related_second, &policy)
            .expect("benchmark lineage contacts are exact");
        lineage_count = lineage_count.wrapping_add(black_box(match contacts {
            RationalBezierIntersectionContacts2::Overlap(_) => 1,
            _ => 0,
        }));
    }
    let elapsed = started.elapsed();
    assert!(!related_first.is_homogeneous_power_basis_cached());
    assert!(!related_second.is_homogeneous_power_basis_cached());
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
        let contacts = nonlinear_line
            .intersection_contacts(&partial_line, &policy)
            .expect("benchmark line-image contacts are exact");
        algebraic_overlap_count = algebraic_overlap_count.wrapping_add(black_box(match contacts {
            RationalBezierIntersectionContacts2::Overlap(overlap)
                if matches!(
                    overlap.first_range().start(),
                    BezierParameter2::Algebraic(_)
                ) =>
            {
                1
            }
            _ => 0,
        }));
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
        let contacts = partial_parabola
            .intersection_contacts(&nonlinear_parabola, &policy)
            .expect("benchmark graph contacts are exact");
        graph_overlap_count = graph_overlap_count.wrapping_add(black_box(match contacts {
            RationalBezierIntersectionContacts2::Overlap(overlap)
                if matches!(
                    overlap.second_range().start(),
                    BezierParameter2::Algebraic(_)
                ) =>
            {
                1
            }
            _ => 0,
        }));
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
    let algebraic_parameter = match parabola
        .intersection_candidates(&horizontal, &policy)
        .expect("benchmark resultant candidates are exact")
    {
        RationalBezierIntersectionCandidates2::Candidates {
            first_parameters, ..
        } => match &first_parameters[0] {
            BezierParameter2::Algebraic(parameter) => parameter.clone(),
            BezierParameter2::Exact(_) => panic!("benchmark expected an algebraic parameter"),
        },
        other => panic!("benchmark expected candidates, got {other:?}"),
    };

    let resultant_iterations = 500_u32;
    let started = Instant::now();
    let mut resultant_count = 0_usize;
    for _ in 0..resultant_iterations {
        let candidates = parabola
            .intersection_candidates(&horizontal, &policy)
            .expect("benchmark resultant candidates are exact");
        resultant_count = resultant_count.wrapping_add(black_box(match candidates {
            RationalBezierIntersectionCandidates2::NoIntersection => 0,
            RationalBezierIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            } => first_parameters.len() + second_parameters.len(),
            RationalBezierIntersectionCandidates2::DegenerateResultant => 1,
        }));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_cached_basis_resultant: {resultant_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={resultant_count}",
        elapsed / resultant_iterations
    );

    let contact_iterations = 250_u32;
    let started = Instant::now();
    let mut contact_count = 0_usize;
    for _ in 0..contact_iterations {
        let contacts = parabola
            .intersection_contacts(&horizontal, &policy)
            .expect("benchmark contacts are exact");
        contact_count = contact_count.wrapping_add(black_box(match contacts {
            RationalBezierIntersectionContacts2::NoIntersection => 0,
            RationalBezierIntersectionContacts2::Contacts(contacts) => contacts.len(),
            RationalBezierIntersectionContacts2::Overlap(_) => 1,
            RationalBezierIntersectionContacts2::Incomplete { contacts, .. } => contacts.len(),
            RationalBezierIntersectionContacts2::DegenerateResultant => 1,
        }));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_contact_replay: {contact_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={contact_count}",
        elapsed / contact_iterations
    );

    let derivative_iterations = 250_u32;
    let started = Instant::now();
    let mut derivative_count = 0_usize;
    for _ in 0..derivative_iterations {
        let derivatives = parabola
            .derivatives_at_algebraic_parameter(
                black_box(&algebraic_parameter),
                black_box(3),
                &policy,
            )
            .expect("algebraic derivatives remain represented");
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
        let derivatives = curve
            .derivatives_at(black_box(&exact_parameter), black_box(3), &policy)
            .expect("exact benchmark derivatives are certified");
        exact_derivative_count = exact_derivative_count.wrapping_add(black_box(derivatives.len()));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_exact_derivatives_1_through_3: {exact_derivative_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={exact_derivative_count}",
        elapsed / exact_derivative_iterations
    );

    let prepared = parabola
        .try_prepare_intersection(&horizontal, &policy)
        .unwrap();
    prepared.try_contact_view().unwrap();
    let prepared_iterations = 20_000_u32;
    let started = Instant::now();
    let mut prepared_count = 0_usize;
    for _ in 0..prepared_iterations {
        let contacts = prepared.try_contact_view().unwrap();
        prepared_count = prepared_count.wrapping_add(black_box(match contacts {
            RationalBezierIntersectionContacts2::NoIntersection => 0,
            RationalBezierIntersectionContacts2::Contacts(contacts) => contacts.len(),
            RationalBezierIntersectionContacts2::Overlap(_) => 1,
            RationalBezierIntersectionContacts2::Incomplete { contacts, .. } => contacts.len(),
            RationalBezierIntersectionContacts2::DegenerateResultant => 1,
        }));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_prepared_contact_view: {prepared_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={prepared_count}",
        elapsed / prepared_iterations
    );

    let topology = prepared.try_topology_view().unwrap();
    topology
        .arrangement_graph_view()
        .expect("prepared topology assembles an arrangement");
    let started = Instant::now();
    let mut topology_count = 0_usize;
    for _ in 0..prepared_iterations {
        let topology = prepared.try_topology_view().unwrap();
        topology_count = topology_count.wrapping_add(black_box(
            topology.first().fragments().len()
                + topology.second().fragments().len()
                + topology.arrangement_graph_view().unwrap().len(),
        ));
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_prepared_topology_view: {prepared_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={topology_count}",
        elapsed / prepared_iterations
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
}
