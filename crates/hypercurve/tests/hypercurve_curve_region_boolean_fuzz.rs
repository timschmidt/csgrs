#![cfg(feature = "predicates")]

#[path = "../benches/common/pathological.rs"]
mod pathological_fixture;

use std::collections::HashSet;

use hypercurve::{
    BooleanOp, CircularArc2, CubicBezier2, Curve2, CurveBoundaryInteriorSide2, CurvePath2,
    CurvePolicy, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, Real,
};
use proptest::prelude::*;
use proptest::test_runner::{FileFailurePersistence, TestCaseError};

const BOOLEAN_OPERATIONS: [BooleanOp; 4] = [
    BooleanOp::Union,
    BooleanOp::Intersection,
    BooleanOp::Difference,
    BooleanOp::Xor,
];

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum RetiredFailure {
    AlgebraicPolylineContacts,
    MixedFamilyRealSign,
    RealCoefficientConicParameterImage,
    UniformWeightGeneralRationalArea,
    CandidateImageIntervalPruning,
    RationalImageDegreeBound,
    FiniteLineImageContactReplay,
    SharedEndpointXorTraversal,
    ConicChartAbsencePropagation,
    PolynomialGraphProjectionReplay,
    RationalResultantDegreeDropSampling,
    DegreeElevatedLineImageOverlap,
    ConicEndpointRootIsolation,
    TransformedDegreeElevatedLineImage,
    CircularConicSharedComponent,
    ReversedProjectiveConicOverlap,
    CircularLineEndpointReplay,
    DistinctCircularConicContacts,
    SignedCompoundCircularSubtraction,
    ThermalSpokeCircularSubtraction,
}

impl RetiredFailure {
    const ALL: [Self; 20] = [
        Self::AlgebraicPolylineContacts,
        Self::MixedFamilyRealSign,
        Self::RealCoefficientConicParameterImage,
        Self::UniformWeightGeneralRationalArea,
        Self::CandidateImageIntervalPruning,
        Self::RationalImageDegreeBound,
        Self::FiniteLineImageContactReplay,
        Self::SharedEndpointXorTraversal,
        Self::ConicChartAbsencePropagation,
        Self::PolynomialGraphProjectionReplay,
        Self::RationalResultantDegreeDropSampling,
        Self::DegreeElevatedLineImageOverlap,
        Self::ConicEndpointRootIsolation,
        Self::TransformedDegreeElevatedLineImage,
        Self::CircularConicSharedComponent,
        Self::ReversedProjectiveConicOverlap,
        Self::CircularLineEndpointReplay,
        Self::DistinctCircularConicContacts,
        Self::SignedCompoundCircularSubtraction,
        Self::ThermalSpokeCircularSubtraction,
    ];
}

#[derive(Debug)]
struct RetiredFailureCase {
    failure: RetiredFailure,
    first: CurveRegion2,
    second: CurveRegion2,
}

#[derive(Clone, Debug)]
struct GeneratedRegion {
    origin_x: i16,
    origin_y: i16,
    width: i16,
    height: i16,
    lower_family: u8,
    upper_family: u8,
    curvature: i16,
    weight_numerator: i16,
    weight_denominator: i16,
}

#[derive(Clone, Debug)]
struct GeneratedBooleanCase {
    first: GeneratedRegion,
    second: GeneratedRegion,
}

fn integer(value: i16) -> Real {
    Real::from(value)
}

fn fraction(numerator: i16, denominator: i16) -> Real {
    (integer(numerator) / integer(denominator)).expect("generated denominator is positive")
}

fn point(x: i16, y: i16) -> Point2 {
    Point2::new(integer(x), integer(y))
}

fn exact_f64_point(x: f64, y: f64) -> Point2 {
    Point2::new(
        Real::try_from(x).expect("retired binary-rational x is finite"),
        Real::try_from(y).expect("retired binary-rational y is finite"),
    )
}

fn affine_control(
    start: &Point2,
    end: &Point2,
    numerator: i16,
    denominator: i16,
    outward: i16,
) -> Point2 {
    let parameter = fraction(numerator, denominator);
    Point2::new(
        start.x() + &((end.x() - start.x()) * &parameter),
        start.y() + &((end.y() - start.y()) * parameter) + integer(outward),
    )
}

fn clamped_cubic_knots() -> Vec<Real> {
    vec![
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::one(),
        Real::one(),
        Real::one(),
    ]
}

fn generated_boundary_curve(
    family: u8,
    start: Point2,
    end: Point2,
    outward: i16,
    weight: &Real,
) -> Curve2 {
    let first_control = affine_control(&start, &end, 1, 3, outward);
    let second_control = affine_control(&start, &end, 2, 3, outward);
    let middle = affine_control(&start, &end, 1, 2, outward);
    match family % 8 {
        0 => Curve2::from(LineSeg2::try_new(start, end).expect("generated edge is nonzero")),
        1 => {
            let chord = (end.x() - start.x()).abs();
            let bulge = (integer(outward.abs()) / chord)
                .expect("generated horizontal edge has a nonzero chord");
            Curve2::from(
                CircularArc2::from_bulge(start, end, bulge)
                    .expect("generated positive bulge forms an exact arc"),
            )
        }
        2 => Curve2::from(QuadraticBezier2::new(start, middle, end)),
        3 => Curve2::from(CubicBezier2::new(start, first_control, second_control, end)),
        4 => Curve2::from(
            RationalQuadraticBezier2::try_new(
                start,
                middle,
                end,
                Real::one(),
                weight.clone(),
                Real::one(),
            )
            .expect("generated rational quadratic has positive weights"),
        ),
        5 => Curve2::from(
            RationalBezier2::try_new(
                vec![start, first_control, second_control, end],
                vec![Real::one(), weight.clone(), weight.clone(), Real::one()],
            )
            .expect("generated rational Bezier has positive weights"),
        ),
        6 => Curve2::try_polynomial_bspline(
            3,
            vec![start, first_control, second_control, end],
            clamped_cubic_knots(),
        )
        .expect("generated clamped polynomial spline is valid"),
        _ => Curve2::try_nurbs(
            3,
            vec![start, first_control, second_control, end],
            vec![Real::one(), weight.clone(), weight.clone(), Real::one()],
            clamped_cubic_knots(),
        )
        .expect("generated clamped NURBS is valid"),
    }
}

fn generated_path(specification: &GeneratedRegion) -> CurvePath2 {
    let min_x = specification.origin_x;
    let min_y = specification.origin_y;
    let max_x = min_x + specification.width;
    let max_y = min_y + specification.height;
    let lower_left = point(min_x, min_y);
    let lower_right = point(max_x, min_y);
    let upper_right = point(max_x, max_y);
    let upper_left = point(min_x, max_y);
    let weight = fraction(
        specification.weight_numerator,
        specification.weight_denominator,
    );
    CurvePath2::try_new(vec![
        generated_boundary_curve(
            specification.lower_family,
            lower_left.clone(),
            lower_right.clone(),
            -specification.curvature,
            &weight,
        ),
        Curve2::from(
            LineSeg2::try_new(lower_right, upper_right.clone())
                .expect("generated right edge is nonzero"),
        ),
        generated_boundary_curve(
            specification.upper_family,
            upper_right,
            upper_left.clone(),
            specification.curvature,
            &weight,
        ),
        Curve2::from(
            LineSeg2::try_new(upper_left, lower_left).expect("generated left edge is nonzero"),
        ),
    ])
    .expect("generated exact boundary is connected")
}

fn generated_region(specification: &GeneratedRegion) -> CurveRegion2 {
    let path = generated_path(specification);
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Left],
    )
    .expect("outward graph curves form a simple exact region")
}

fn generated_region_strategy() -> impl Strategy<Value = GeneratedRegion> {
    (
        -24_i16..=24,
        -24_i16..=24,
        8_i16..=28,
        6_i16..=24,
        0_u8..8,
        0_u8..8,
        1_i16..=5,
        1_i16..=5,
        1_i16..=5,
    )
        .prop_map(
            |(
                origin_x,
                origin_y,
                width,
                height,
                lower_family,
                upper_family,
                curvature,
                weight_numerator,
                weight_denominator,
            )| GeneratedRegion {
                origin_x,
                origin_y,
                width,
                height,
                lower_family,
                upper_family,
                curvature,
                weight_numerator,
                weight_denominator,
            },
        )
}

fn generated_boolean_case_strategy() -> impl Strategy<Value = GeneratedBooleanCase> {
    (generated_region_strategy(), generated_region_strategy())
        .prop_map(|(first, second)| GeneratedBooleanCase { first, second })
}

fn exact_boolean_fuzz_config() -> ProptestConfig {
    let cases = std::env::var("HYPERCURVE_EXACT_BOOLEAN_FUZZ_CASES")
        .ok()
        .map(|value| {
            value.parse::<u32>().unwrap_or_else(|error| {
                panic!("invalid HYPERCURVE_EXACT_BOOLEAN_FUZZ_CASES={value:?}: {error}")
            })
        })
        .unwrap_or(32);
    ProptestConfig {
        cases,
        max_shrink_iters: 1024,
        failure_persistence: Some(Box::new(FileFailurePersistence::WithSource(
            "proptest-regressions",
        ))),
        ..ProptestConfig::default()
    }
}

fn exact_boolean_results(
    label: &str,
    first: &CurveRegion2,
    second: &CurveRegion2,
    compare_individual_calls: bool,
) -> Result<(), String> {
    let policy = CurvePolicy::certified();
    let evidence = first
        .intersect_region(second, &policy)
        .map_err(|error| format!("{label}: exact intersection failed: {error}"))?;
    if !evidence.blockers().is_empty() {
        return Err(format!(
            "{label}: exact intersection retained blockers: {:#?}",
            evidence.blockers()
        ));
    }
    let batch = first.boolean_regions(second, &policy).map_err(|error| {
        let immediate = BOOLEAN_OPERATIONS.map(|operation| {
            (
                operation,
                first.boolean_region(second, operation, &policy).err(),
            )
        });
        let contacts = evidence
            .contacts()
            .iter()
            .map(|contact| {
                (
                    contact.first().carrier_index(),
                    contact.first().family(),
                    contact.second().carrier_index(),
                    contact.second().family(),
                    contact.contact().first().exact_curve_parameter(),
                    contact.contact().second().exact_curve_parameter(),
                    contact.contact().is_certified_transverse(),
                )
            })
            .collect::<Vec<_>>();
        format!(
            "{label}: exact Boolean batch failed: {error}; immediate failures: {immediate:?}; contacts: {contacts:?}; overlap count: {}",
            evidence.overlaps().len(),
        )
    })?;
    if compare_individual_calls {
        for operation in BOOLEAN_OPERATIONS {
            let immediate = first
                .boolean_region(second, operation, &policy)
                .map_err(|error| {
                    format!("{label}: immediate {operation:?} failed after batch success: {error}")
                })?;
            if &immediate != batch.region(operation) {
                return Err(format!(
                    "{label}: immediate and batch {operation:?} results differ"
                ));
            }
        }
    }
    Ok(())
}

fn retired_algebraic_polyline_case() -> RetiredFailureCase {
    let p0 = exact_f64_point(-18.6, -6.2);
    let p1 = exact_f64_point(-14.20650382593274, -19.07368476688862);
    let p2 = exact_f64_point(-6.4565038147568705, -21.17386977761984);
    let p3 = exact_f64_point(4.03, -4.03);
    let p4 = exact_f64_point(14.600491094738247, -20.78282692939043);
    let p5 = exact_f64_point(20.150000000000002, 6.820000000000001);
    let p6 = exact_f64_point(7.4399999999999995, 10.85);
    let p7 = exact_f64_point(-16.43, 7.13);
    let first_path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p0.clone(), p1.clone()).unwrap()),
        Curve2::from(
            CircularArc2::from_bulge(
                p1,
                p2.clone(),
                Real::try_from(0.46).expect("retired bulge is finite"),
            )
            .unwrap(),
        ),
        Curve2::from(QuadraticBezier2::new(
            p2,
            exact_f64_point(-0.31000000000000005, 7.13),
            p3.clone(),
        )),
        Curve2::from(CubicBezier2::new(
            p3,
            exact_f64_point(7.184295191466809, -46.13835323035717),
            exact_f64_point(10.85, 5.89),
            p4.clone(),
        )),
        Curve2::from(
            RationalQuadraticBezier2::try_new(
                p4,
                exact_f64_point(18.91, 6.2),
                p5.clone(),
                Real::one(),
                Real::try_from(0.36).expect("retired weight is finite"),
                Real::one(),
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(p5, p6.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(p6, p7.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(p7, p0).unwrap()),
    ])
    .unwrap();

    let q0 = exact_f64_point(-24.8, -18.6);
    let q1 = exact_f64_point(24.8, -18.6);
    let q2 = exact_f64_point(24.8, 8.06);
    let q3 = exact_f64_point(-24.8, 8.06);
    let second_path = CurvePath2::try_new(vec![
        Curve2::from(CubicBezier2::new(
            q0.clone(),
            exact_f64_point(-12.4, -20.77),
            exact_f64_point(12.4, -20.77),
            q1.clone(),
        )),
        Curve2::from(LineSeg2::try_new(q1, q2.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(q2, q3.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(q3, q0).unwrap()),
    ])
    .unwrap();

    RetiredFailureCase {
        failure: RetiredFailure::AlgebraicPolylineContacts,
        first: CurveRegion2::try_from_boundary_paths(&[first_path]).unwrap(),
        second: CurveRegion2::try_from_boundary_paths(&[second_path]).unwrap(),
    }
}

fn retired_pathological_case(index: usize, failure: RetiredFailure) -> RetiredFailureCase {
    let cell = pathological_fixture::build_native_cell(index);
    RetiredFailureCase {
        failure,
        first: cell.source,
        second: cell.rotated,
    }
}

fn retired_uniform_weight_area_case() -> RetiredFailureCase {
    let line_region = GeneratedRegion {
        origin_x: 0,
        origin_y: 0,
        width: 8,
        height: 6,
        lower_family: 0,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    let rational_region = GeneratedRegion {
        upper_family: 5,
        ..line_region.clone()
    };
    RetiredFailureCase {
        failure: RetiredFailure::UniformWeightGeneralRationalArea,
        first: CurveRegion2::try_from_boundary_paths(&[generated_path(&line_region)])
            .expect("retired line region is valid"),
        second: CurveRegion2::try_from_boundary_paths(&[generated_path(&rational_region)])
            .expect("retired uniform-weight rational region is valid"),
    }
}

fn retired_candidate_interval_pruning_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 0,
        origin_y: 20,
        width: 8,
        height: 6,
        lower_family: 5,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 2,
    };
    let second = GeneratedRegion {
        origin_x: 0,
        origin_y: 21,
        width: 8,
        height: 6,
        lower_family: 3,
        upper_family: 0,
        curvature: 3,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    RetiredFailureCase {
        failure: RetiredFailure::CandidateImageIntervalPruning,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

#[test]
fn retired_candidate_interval_pruning_completes() {
    let case = retired_candidate_interval_pruning_case();
    exact_boolean_results(
        "retired candidate interval pruning",
        &case.first,
        &case.second,
        true,
    )
    .unwrap();
}

fn retired_rational_image_degree_bound_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 0,
        origin_y: 19,
        width: 8,
        height: 6,
        lower_family: 5,
        upper_family: 0,
        curvature: 4,
        weight_numerator: 2,
        weight_denominator: 3,
    };
    let second = GeneratedRegion {
        origin_x: -5,
        origin_y: 19,
        width: 8,
        height: 6,
        lower_family: 5,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 2,
        weight_denominator: 1,
    };
    RetiredFailureCase {
        failure: RetiredFailure::RationalImageDegreeBound,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

fn retired_finite_line_image_contact_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 1,
        origin_y: -11,
        width: 19,
        height: 21,
        lower_family: 0,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    let second = GeneratedRegion {
        origin_x: 11,
        origin_y: 7,
        width: 18,
        height: 6,
        lower_family: 4,
        upper_family: 0,
        curvature: 3,
        weight_numerator: 2,
        weight_denominator: 4,
    };
    RetiredFailureCase {
        failure: RetiredFailure::FiniteLineImageContactReplay,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

fn shared_endpoint_xor_case() -> (CurveRegion2, CurveRegion2) {
    let first = GeneratedRegion {
        origin_x: 5,
        origin_y: 17,
        width: 10,
        height: 7,
        lower_family: 0,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    let second = GeneratedRegion {
        origin_x: -11,
        origin_y: 24,
        width: 26,
        height: 6,
        lower_family: 1,
        upper_family: 2,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    (generated_region(&first), generated_region(&second))
}

#[test]
fn shared_endpoint_xor_completes() {
    let (first, second) = shared_endpoint_xor_case();
    exact_boolean_results("shared endpoint XOR", &first, &second, true).unwrap();
}

fn retired_shared_endpoint_xor_case() -> RetiredFailureCase {
    let (first, second) = shared_endpoint_xor_case();
    RetiredFailureCase {
        failure: RetiredFailure::SharedEndpointXorTraversal,
        first,
        second,
    }
}

fn retired_conic_chart_absence_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 5,
        origin_y: 6,
        width: 18,
        height: 6,
        lower_family: 4,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 4,
        weight_denominator: 2,
    };
    let second = GeneratedRegion {
        origin_x: 5,
        origin_y: -14,
        width: 23,
        height: 18,
        lower_family: 0,
        upper_family: 3,
        curvature: 4,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    RetiredFailureCase {
        failure: RetiredFailure::ConicChartAbsencePropagation,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

fn retired_polynomial_graph_projection_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 5,
        origin_y: 6,
        width: 18,
        height: 6,
        lower_family: 5,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 4,
        weight_denominator: 2,
    };
    let second = GeneratedRegion {
        origin_x: 5,
        origin_y: -14,
        width: 23,
        height: 18,
        lower_family: 0,
        upper_family: 3,
        curvature: 4,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    RetiredFailureCase {
        failure: RetiredFailure::PolynomialGraphProjectionReplay,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

fn retired_resultant_degree_drop_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 5,
        origin_y: 6,
        width: 18,
        height: 6,
        lower_family: 5,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 4,
        weight_denominator: 2,
    };
    let second = GeneratedRegion {
        origin_x: 5,
        origin_y: -14,
        width: 23,
        height: 18,
        lower_family: 0,
        upper_family: 5,
        curvature: 4,
        weight_numerator: 4,
        weight_denominator: 1,
    };
    RetiredFailureCase {
        failure: RetiredFailure::RationalResultantDegreeDropSampling,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

fn retired_degree_elevated_line_overlap_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 20,
        origin_y: 0,
        width: 8,
        height: 6,
        lower_family: 0,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    let second = GeneratedRegion {
        origin_x: 19,
        origin_y: 0,
        width: 9,
        height: 6,
        lower_family: 0,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    RetiredFailureCase {
        failure: RetiredFailure::DegreeElevatedLineImageOverlap,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

fn retired_conic_endpoint_root_isolation_case() -> RetiredFailureCase {
    let first = GeneratedRegion {
        origin_x: 24,
        origin_y: -3,
        width: 13,
        height: 6,
        lower_family: 4,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 3,
        weight_denominator: 1,
    };
    let second = GeneratedRegion {
        origin_x: 19,
        origin_y: -9,
        width: 18,
        height: 6,
        lower_family: 0,
        upper_family: 1,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    };
    RetiredFailureCase {
        failure: RetiredFailure::ConicEndpointRootIsolation,
        first: generated_region(&first),
        second: generated_region(&second),
    }
}

fn exact_circle_region(start_quarter: usize, reversed: bool) -> CurveRegion2 {
    let points = [point(5, 0), point(0, 5), point(-5, 0), point(0, -5)];
    let center = point(0, 0);
    let mut curves = (0..4)
        .map(|offset| {
            let index = (start_quarter + offset) % 4;
            Curve2::from(
                CircularArc2::try_from_center(
                    points[index].clone(),
                    points[(index + 1) % 4].clone(),
                    center.clone(),
                    false,
                )
                .unwrap(),
            )
        })
        .collect::<Vec<_>>();
    if reversed {
        curves = CurvePath2::try_new(curves)
            .unwrap()
            .reversed()
            .unwrap()
            .curves()
            .to_vec();
    }
    let path = CurvePath2::try_new(curves).unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[if reversed {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .unwrap()
}

fn retired_signed_compound_circular_subtraction_case() -> RetiredFailureCase {
    let top_left = point(-11, 1);
    let top_right = point(0, 1);
    let bottom_right = point(0, -1);
    let bottom_left = point(-11, -1);
    let capsule = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(top_left.clone(), top_right.clone()).unwrap()),
        Curve2::from(
            CircularArc2::try_from_center(top_right, bottom_right.clone(), point(0, 0), true)
                .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(bottom_right, bottom_left.clone()).unwrap()),
        Curve2::from(
            CircularArc2::try_from_center(bottom_left, top_left, point(-11, 0), true).unwrap(),
        ),
    ])
    .unwrap();
    let via_points = [point(2, 0), point(0, -2), point(-2, 0), point(0, 2)];
    let via = CurvePath2::try_new(
        (0..4)
            .map(|index| {
                Curve2::from(
                    CircularArc2::try_from_center(
                        via_points[index].clone(),
                        via_points[(index + 1) % 4].clone(),
                        point(0, 0),
                        true,
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let first = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
        &[capsule, via],
        &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Material],
        &[FillRule::NonZero, FillRule::NonZero],
    )
    .unwrap();
    RetiredFailureCase {
        failure: RetiredFailure::SignedCompoundCircularSubtraction,
        first,
        second: exact_circle_region(0, true)
            .transform_affine(
                &fraction(1, 5),
                &Real::zero(),
                &Real::zero(),
                &fraction(1, 5),
                &Real::zero(),
                &Real::zero(),
                &CurvePolicy::certified(),
            )
            .unwrap(),
    }
}

fn retired_thermal_spoke_circular_subtraction_case() -> RetiredFailureCase {
    let horizontal = generated_region(&GeneratedRegion {
        origin_x: -10,
        origin_y: -1,
        width: 20,
        height: 2,
        lower_family: 0,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    });
    let vertical = generated_region(&GeneratedRegion {
        origin_x: -1,
        origin_y: -10,
        width: 2,
        height: 20,
        lower_family: 0,
        upper_family: 0,
        curvature: 1,
        weight_numerator: 1,
        weight_denominator: 1,
    });
    let first = horizontal
        .boolean_region(&vertical, BooleanOp::Union, &CurvePolicy::certified())
        .unwrap();
    RetiredFailureCase {
        failure: RetiredFailure::ThermalSpokeCircularSubtraction,
        first,
        second: exact_circle_region(0, true)
            .transform_affine(
                &fraction(1, 5),
                &Real::zero(),
                &Real::zero(),
                &fraction(1, 5),
                &Real::zero(),
                &Real::zero(),
                &CurvePolicy::certified(),
            )
            .unwrap(),
    }
}

fn retired_transformed_degree_elevated_line_case() -> RetiredFailureCase {
    let corners = [point(0, 0), point(8, 0), point(8, 6), point(0, 6)];
    let curves = (0..4)
        .map(|index| {
            Curve2::from(QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(corners[index].clone(), corners[(index + 1) % 4].clone())
                    .unwrap(),
            ))
        })
        .collect();
    let source =
        CurveRegion2::try_from_boundary_paths(&[CurvePath2::try_new(curves).unwrap()]).unwrap();
    let policy = CurvePolicy::certified();
    let transformed = source
        .transform_affine(
            &Real::zero(),
            &-Real::one(),
            &Real::one(),
            &Real::zero(),
            &integer(7),
            &integer(1),
            &policy,
        )
        .unwrap();
    RetiredFailureCase {
        failure: RetiredFailure::TransformedDegreeElevatedLineImage,
        first: transformed,
        second: generated_region(&GeneratedRegion {
            origin_x: 0,
            origin_y: 3,
            width: 10,
            height: 6,
            lower_family: 1,
            upper_family: 0,
            curvature: 1,
            weight_numerator: 1,
            weight_denominator: 1,
        }),
    }
}

fn retired_circular_shared_component_case() -> RetiredFailureCase {
    RetiredFailureCase {
        failure: RetiredFailure::CircularConicSharedComponent,
        first: exact_circle_region(0, false),
        second: exact_circle_region(1, false),
    }
}

fn retired_reversed_projective_conic_case() -> RetiredFailureCase {
    RetiredFailureCase {
        failure: RetiredFailure::ReversedProjectiveConicOverlap,
        first: exact_circle_region(0, false),
        second: exact_circle_region(0, true),
    }
}

fn retired_circular_line_endpoint_case() -> RetiredFailureCase {
    RetiredFailureCase {
        failure: RetiredFailure::CircularLineEndpointReplay,
        first: exact_circle_region(0, false),
        second: generated_region(&GeneratedRegion {
            origin_x: -5,
            origin_y: 0,
            width: 5,
            height: 6,
            lower_family: 0,
            upper_family: 0,
            curvature: 1,
            weight_numerator: 1,
            weight_denominator: 1,
        }),
    }
}

fn retired_distinct_circular_conic_contacts_case() -> RetiredFailureCase {
    let policy = CurvePolicy::certified();
    let translated = exact_circle_region(1, false)
        .transform_affine(
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &integer(4),
            &Real::zero(),
            &policy,
        )
        .unwrap();
    RetiredFailureCase {
        failure: RetiredFailure::DistinctCircularConicContacts,
        first: exact_circle_region(0, false),
        second: translated,
    }
}

#[test]
fn conic_endpoint_root_isolation_completes() {
    let case = retired_conic_endpoint_root_isolation_case();
    exact_boolean_results(
        "conic endpoint root isolation",
        &case.first,
        &case.second,
        true,
    )
    .unwrap();
}

fn retired_failure_corpus() -> Vec<RetiredFailureCase> {
    vec![
        retired_algebraic_polyline_case(),
        retired_pathological_case(0, RetiredFailure::MixedFamilyRealSign),
        retired_pathological_case(2, RetiredFailure::RealCoefficientConicParameterImage),
        retired_uniform_weight_area_case(),
        retired_candidate_interval_pruning_case(),
        retired_rational_image_degree_bound_case(),
        retired_finite_line_image_contact_case(),
        retired_shared_endpoint_xor_case(),
        retired_conic_chart_absence_case(),
        retired_polynomial_graph_projection_case(),
        retired_resultant_degree_drop_case(),
        retired_degree_elevated_line_overlap_case(),
        retired_conic_endpoint_root_isolation_case(),
        retired_transformed_degree_elevated_line_case(),
        retired_circular_shared_component_case(),
        retired_reversed_projective_conic_case(),
        retired_circular_line_endpoint_case(),
        retired_distinct_circular_conic_contacts_case(),
        retired_signed_compound_circular_subtraction_case(),
        retired_thermal_spoke_circular_subtraction_case(),
    ]
}

#[test]
fn retired_exact_curve_region_boolean_failures_remain_in_the_corpus() {
    let corpus = retired_failure_corpus();
    let covered = corpus
        .iter()
        .map(|case| case.failure)
        .collect::<HashSet<_>>();
    assert_eq!(
        covered,
        RetiredFailure::ALL.into_iter().collect(),
        "every retired exact-Boolean failure must retain at least one corpus reproducer"
    );
    for case in corpus {
        exact_boolean_results(
            &format!("retired failure {:?}", case.failure),
            &case.first,
            &case.second,
            false,
        )
        .unwrap_or_else(|failure| panic!("{failure}"));
    }
}

#[test]
fn algebraic_polyline_contacts_preserve_exact_contact_distinction() {
    let case = retired_algebraic_polyline_case();
    exact_boolean_results(
        "algebraic polyline contact distinction",
        &case.first,
        &case.second,
        false,
    )
    .unwrap();
}

#[test]
fn explicit_loop_topology_supports_reversed_nonuniform_rational_regions() {
    let specification = GeneratedRegion {
        origin_x: -4,
        origin_y: -3,
        width: 12,
        height: 8,
        lower_family: 7,
        upper_family: 5,
        curvature: 3,
        weight_numerator: 2,
        weight_denominator: 3,
    };
    let forward_path = generated_path(&specification);
    assert!(
        CurveRegion2::try_from_boundary_paths_with_loop_topology(
            std::slice::from_ref(&forward_path),
            &[CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            &[],
        )
        .is_err(),
        "interior-side evidence count must match the authored loops"
    );
    let forward = CurveRegion2::try_from_boundary_paths_with_loop_topology(
        std::slice::from_ref(&forward_path),
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Left],
    )
    .unwrap();
    let reversed_path = forward_path.reversed().unwrap();
    let reversed = CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[reversed_path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Right],
    )
    .unwrap();
    exact_boolean_results(
        "oppositely oriented nonuniform rational regions",
        &forward,
        &reversed,
        true,
    )
    .unwrap();
}

proptest! {
    #![proptest_config(exact_boolean_fuzz_config())]

    #[test]
    fn generated_exact_curve_region_booleans_complete_and_match_immediate_results(
        case in generated_boolean_case_strategy(),
    ) {
        let first = generated_region(&case.first);
        let second = generated_region(&case.second);
        exact_boolean_results("generated exact curve-region pair", &first, &second, true)
            .map_err(TestCaseError::fail)?;
    }
}
