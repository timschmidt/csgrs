//! Stateful composed-workload qualification.
//!
//! Exact regions from every authored curve family pass through generated
//! operation sequences whose outputs become later operands, so boundary
//! topology and coefficient dependencies grow along the sequence. Each step is
//! checked against oracles that do not trust the result's own construction:
//! Boolean membership against the operands at exact sample points, offset
//! monotonicity, translation invariance, locality of corner edits, and
//! reconstruction of every result from its exported boundary paths.
//! Required exact operations must complete; a blocked operation is a failure,
//! not a skipped case.

use hypercurve::{
    BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2, CurveContext, CurveCornerMode2,
    CurveFillet2, CurvePath2, CurvePoint2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2,
    OffsetCornerStyle2, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real,
    RegionPointLocation,
};
use proptest::prelude::*;
use proptest::test_runner::{FileFailurePersistence, TestCaseError};

const STRICT: CurveContext = CurveContext::STRICT;

fn integer(value: i16) -> Real {
    Real::from(value)
}

fn fraction(numerator: i16, denominator: i16) -> Real {
    (integer(numerator) / integer(denominator)).expect("generated denominator is positive")
}

fn point(x: i16, y: i16) -> Point2 {
    Point2::new(integer(x), integer(y))
}

fn affine_control(start: &Point2, end: &Point2, numerator: i16, outward: i16) -> Point2 {
    let parameter = fraction(numerator, 6);
    Point2::new(
        start.x() + &((end.x() - start.x()) * &parameter),
        start.y() + &((end.y() - start.y()) * parameter) + integer(outward),
    )
}

fn clamped_cubic_knots() -> Vec<Real> {
    [0, 0, 0, 0, 1, 1, 1, 1]
        .into_iter()
        .map(Real::from)
        .collect()
}

/// One horizontal boundary edge of the given family bulging by `outward`.
fn family_edge(family: u8, start: Point2, end: Point2, outward: i16, weight: &Real) -> Curve2 {
    let first = affine_control(&start, &end, 2, outward);
    let second = affine_control(&start, &end, 4, outward);
    let middle = affine_control(&start, &end, 3, outward);
    match family % 8 {
        0 => Curve2::from(LineSeg2::try_new(start, end).unwrap()),
        1 => {
            let chord = (end.x() - start.x()).abs();
            let bulge = (integer(outward.abs()) / chord).unwrap();
            let bulge = if outward < 0 {
                Real::zero() - bulge
            } else {
                bulge
            };
            // A bulge measures to the left of travel; both edges bulge outward.
            let bulge = if end.x() > start.x() {
                Real::zero() - bulge
            } else {
                bulge
            };
            Curve2::from(CircularArc2::from_bulge(start, end, bulge).unwrap())
        }
        2 => Curve2::from(QuadraticBezier2::new(start, middle, end)),
        3 => Curve2::from(CubicBezier2::new(start, first, second, end)),
        4 => Curve2::from(
            RationalQuadraticBezier2::try_new(
                start,
                middle,
                end,
                Real::one(),
                weight.clone(),
                Real::one(),
            )
            .unwrap(),
        ),
        5 => Curve2::from(
            RationalBezier2::try_new(
                vec![start, first, second, end],
                vec![Real::one(), weight.clone(), weight.clone(), Real::one()],
            )
            .unwrap(),
        ),
        6 => Curve2::try_polynomial_bspline(
            3,
            vec![start, first, second, end],
            clamped_cubic_knots(),
            &STRICT,
        )
        .unwrap()
        .into_value(),
        _ => Curve2::try_nurbs(
            3,
            vec![start, first, second, end],
            vec![Real::one(), weight.clone(), weight.clone(), Real::one()],
            clamped_cubic_knots(),
            &STRICT,
        )
        .unwrap()
        .into_value(),
    }
}

#[derive(Clone, Debug)]
struct Seed {
    x: i16,
    y: i16,
    width: i16,
    height: i16,
    lower: u8,
    upper: u8,
    curvature: i16,
    weight: i16,
}

fn seed_region(seed: &Seed) -> CurveRegion2 {
    let (min_x, min_y) = (seed.x, seed.y);
    let (max_x, max_y) = (min_x + seed.width, min_y + seed.height);
    let weight = fraction(seed.weight, 3);
    let path = CurvePath2::try_new(vec![
        family_edge(
            seed.lower,
            point(min_x, min_y),
            point(max_x, min_y),
            -seed.curvature,
            &weight,
        ),
        Curve2::from(LineSeg2::try_new(point(max_x, min_y), point(max_x, max_y)).unwrap()),
        family_edge(
            seed.upper,
            point(max_x, max_y),
            point(min_x, max_y),
            seed.curvature,
            &weight,
        ),
        Curve2::from(LineSeg2::try_new(point(min_x, max_y), point(min_x, min_y)).unwrap()),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &STRICT,
    )
    .unwrap()
    .into_value()
}

/// A square frame with a hole containing an island: three nesting levels.
fn nested_region(x: i16, y: i16, size: i16) -> CurveRegion2 {
    let square = |x: i16, y: i16, size: i16| {
        seed_region(&Seed {
            x,
            y,
            width: size,
            height: size,
            lower: 0,
            upper: 0,
            curvature: 1,
            weight: 3,
        })
    };
    let frame = square(x, y, size)
        .boolean_region(
            &square(x + 2, y + 2, size - 4),
            BooleanOp::Difference,
            &STRICT,
        )
        .unwrap()
        .into_value();
    frame
        .boolean_region(&square(x + 4, y + 4, size - 8), BooleanOp::Union, &STRICT)
        .unwrap()
        .into_value()
}

#[derive(Clone, Debug)]
enum Step {
    Boolean(usize, usize, u8),
    Offset(usize, i16, u8),
    Fillet(usize, u8),
    Chamfer(usize, u8),
    Translate(usize, i16, i16),
    /// Quarter turns, scale index into {1, 3/2, 2}, and an optional mirror.
    Similarity(usize, u8, u8, bool),
}

fn step_strategy() -> impl Strategy<Value = Step> {
    prop_oneof![
        4 => (0_usize..16, 0_usize..16, 0_u8..4).prop_map(|(a, b, op)| Step::Boolean(a, b, op)),
        2 => (0_usize..16, -2_i16..=2, 0_u8..3).prop_map(|(a, d, style)| Step::Offset(a, d, style)),
        1 => (0_usize..16, 0_u8..8).prop_map(|(a, vertex)| Step::Fillet(a, vertex)),
        1 => (0_usize..16, 0_u8..8).prop_map(|(a, vertex)| Step::Chamfer(a, vertex)),
        1 => (0_usize..16, -9_i16..=9, -9_i16..=9).prop_map(|(a, x, y)| Step::Translate(a, x, y)),
        1 => (0_usize..16, 0_u8..4, 0_u8..3, any::<bool>())
            .prop_map(|(a, turns, scale, mirror)| Step::Similarity(a, turns, scale, mirror)),
    ]
}

fn seed_strategy() -> impl Strategy<Value = Seed> {
    (
        -12_i16..=12,
        -12_i16..=12,
        8_i16..=20,
        6_i16..=16,
        0_u8..8,
        0_u8..8,
        1_i16..=3,
        1_i16..=6,
    )
        .prop_map(
            |(x, y, width, height, lower, upper, curvature, weight)| Seed {
                x,
                y,
                width,
                height,
                lower,
                upper,
                curvature,
                weight,
            },
        )
}

fn boolean_op(op: u8) -> BooleanOp {
    [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ][usize::from(op % 4)]
}

/// Exact sample points off the integer lattice, so generated boundaries
/// (which pass through integer and simple rational points) are rarely hit.
fn samples() -> Vec<CurvePoint2> {
    let mut points = Vec::new();
    for i in 0..9_i16 {
        for j in 0..9_i16 {
            let x = (integer(-20 + i * 6) + fraction(1, 7)).clone();
            let y = (integer(-20 + j * 6) + fraction(2, 11)).clone();
            points.push(CurvePoint2::from(Point2::new(x, y)));
        }
    }
    points
}

fn inside(region: &CurveRegion2, point: &CurvePoint2) -> Result<Option<bool>, TestCaseError> {
    if region.is_empty() {
        return Ok(Some(false));
    }
    let location = region
        .classify_point(point, &STRICT)
        .map_err(|error| TestCaseError::fail(format!("classification failed: {error}")))?
        .into_value();
    Ok(match location {
        Classification::Decided(RegionPointLocation::Inside) => Some(true),
        Classification::Decided(RegionPointLocation::Outside) => Some(false),
        Classification::Decided(RegionPointLocation::Boundary) => None,
        Classification::Uncertain(reason) => {
            return Err(TestCaseError::fail(format!(
                "rational sample classification was uncertain: {reason:?}"
            )));
        }
    })
}

/// The exported boundary must rebuild the same filled set.
fn assert_round_trip(label: &str, region: &CurveRegion2) -> Result<(), TestCaseError> {
    if region.is_empty() {
        return Ok(());
    }
    let Classification::Decided(paths) = region
        .boundary_paths(&STRICT)
        .map_err(|error| TestCaseError::fail(format!("{label}: boundary export: {error}")))?
        .into_value()
    else {
        return Err(TestCaseError::fail(format!(
            "{label}: boundary export uncertain"
        )));
    };
    let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero, &STRICT)
        .map_err(|error| TestCaseError::fail(format!("{label}: reconstruction: {error}")))?
        .into_value();
    let difference = rebuilt
        .boolean_region(region, BooleanOp::Xor, &STRICT)
        .map_err(|error| TestCaseError::fail(format!("{label}: round-trip xor: {error}")))?
        .into_value();
    prop_assert!(
        difference.is_empty(),
        "{label}: exported boundary rebuilt a different set"
    );
    Ok(())
}

fn required<T>(label: &str, result: hypercurve::ExactCurveResult<T>) -> Result<T, TestCaseError> {
    result
        .map_err(|error| TestCaseError::fail(format!("{label}: required exact operation: {error}")))
}

fn first_loop_vertex_count(region: &CurveRegion2) -> usize {
    region
        .boundary_loops()
        .first()
        .map_or(0, |boundary| boundary.len())
}

fn run_sequence(seeds: &[Seed], steps: &[Step], nested: bool) -> Result<(), TestCaseError> {
    let mut pool: Vec<CurveRegion2> = seeds.iter().map(seed_region).collect();
    if nested {
        pool.push(nested_region(-6, -6, 14));
    }
    let points = samples();
    if std::env::var_os("HYPERCURVE_COMPOSED_WORKLOAD_TRACE").is_some() {
        eprintln!("CASE seeds={seeds:?} steps={steps:?} nested={nested}");
    }
    for (index, step) in steps.iter().enumerate() {
        let pick = |slot: usize| pool[slot % pool.len()].clone();
        let label = format!("step {index} {step:?}");
        let started = std::time::Instant::now();
        let result = match *step {
            Step::Boolean(a, b, op) => {
                let (first, second) = (pick(a), pick(b));
                let op = boolean_op(op);
                let result =
                    required(&label, first.boolean_region(&second, op, &STRICT))?.into_value();
                for point in &points {
                    let (Some(left), Some(right), Some(actual)) = (
                        inside(&first, point)?,
                        inside(&second, point)?,
                        inside(&result, point)?,
                    ) else {
                        continue;
                    };
                    let expected = match op {
                        BooleanOp::Union => left || right,
                        BooleanOp::Intersection => left && right,
                        BooleanOp::Difference => left && !right,
                        BooleanOp::Xor => left != right,
                    };
                    prop_assert_eq!(actual, expected, "{}: membership at {:?}", label, point);
                }
                result
            }
            Step::Offset(a, distance, style) => {
                let source = pick(a);
                if distance == 0 || source.is_empty() {
                    continue;
                }
                let distance = fraction(distance, 4);
                let style = match style {
                    0 => OffsetCornerStyle2::Round,
                    1 => OffsetCornerStyle2::Bevel,
                    _ => OffsetCornerStyle2::Miter { limit: integer(4) },
                };
                let outward = distance > Real::zero();
                let result =
                    required(&label, source.offset(distance, &style, &STRICT))?.into_value();
                // Dilation contains the source and erosion is contained in it.
                for point in &points {
                    let (Some(before), Some(after)) =
                        (inside(&source, point)?, inside(&result, point)?)
                    else {
                        continue;
                    };
                    if outward {
                        prop_assert!(!before || after, "{}: dilation lost {:?}", label, point);
                    } else {
                        prop_assert!(!after || before, "{}: erosion gained {:?}", label, point);
                    }
                }
                result
            }
            Step::Fillet(a, vertex) | Step::Chamfer(a, vertex) => {
                let source = pick(a);
                let count = first_loop_vertex_count(&source);
                if count == 0 {
                    continue;
                }
                let vertex = usize::from(vertex) % count;
                let solutions = if matches!(step, Step::Fillet(..)) {
                    source.fillet_loop_vertex(
                        0,
                        vertex,
                        &CurveFillet2::new(fraction(1, 4)),
                        CurveCornerMode2::TrimOnly,
                        &STRICT,
                    )
                } else {
                    source.chamfer_loop_vertex_by_setbacks(
                        0,
                        vertex,
                        fraction(1, 4),
                        fraction(1, 4),
                        CurveCornerMode2::TrimOnly,
                        &STRICT,
                    )
                };
                // A corner may have no admissible solution (for example a
                // smooth join or a setback beyond a short edge); that is a
                // decided geometric outcome, not a skipped capability.
                let solutions = required(&label, solutions)?.into_value();
                let Some(result) = solutions.into_solutions().into_iter().next() else {
                    continue;
                };
                // A corner edit changes membership only near the edited
                // corner, so most samples keep their membership.
                let mut changed = 0;
                let mut compared = 0;
                for point in &points {
                    let (Some(before), Some(after)) =
                        (inside(&source, point)?, inside(&result, point)?)
                    else {
                        continue;
                    };
                    compared += 1;
                    changed += usize::from(before != after);
                }
                prop_assert!(
                    changed <= 2,
                    "{}: {} of {} samples changed",
                    label,
                    changed,
                    compared
                );
                result
            }
            Step::Translate(a, x, y) => {
                let source = pick(a);
                let (dx, dy) = (fraction(x, 2), fraction(y, 2));
                let result = required(
                    &label,
                    source.transform_affine(
                        &Real::one(),
                        &Real::zero(),
                        &Real::zero(),
                        &Real::one(),
                        &dx,
                        &dy,
                        &STRICT,
                    ),
                )?
                .into_value();
                for point in points.iter().take(27) {
                    let (x, y) = point
                        .coordinates()
                        .map(|p| (p.x().clone(), p.y().clone()))
                        .unwrap();
                    let moved = CurvePoint2::from(Point2::new(&x + &dx, &y + &dy));
                    let (Some(before), Some(after)) =
                        (inside(&source, point)?, inside(&result, &moved)?)
                    else {
                        continue;
                    };
                    prop_assert_eq!(before, after, "{}: translation moved membership", label);
                }
                result
            }
            Step::Similarity(a, turns, scale, mirror) => {
                let source = pick(a);
                let scale = match scale {
                    0 => integer(1),
                    1 => fraction(3, 2),
                    _ => integer(2),
                };
                let (cos, sin) = match turns % 4 {
                    0 => (1, 0),
                    1 => (0, 1),
                    2 => (-1, 0),
                    _ => (0, -1),
                };
                // [m00 m01; m10 m11] = scale * R(turns) * diag(mirror ? -1 : 1, 1).
                let flip = if mirror { -1 } else { 1 };
                let m00 = &scale * integer(cos * flip);
                let m01 = &scale * integer(-sin);
                let m10 = &scale * integer(sin * flip);
                let m11 = &scale * integer(cos);
                let result = required(
                    &label,
                    source.transform_affine(
                        &m00,
                        &m01,
                        &m10,
                        &m11,
                        &Real::zero(),
                        &Real::zero(),
                        &STRICT,
                    ),
                )?
                .into_value();
                for point in points.iter().take(27) {
                    let (x, y) = point
                        .coordinates()
                        .map(|p| (p.x().clone(), p.y().clone()))
                        .unwrap();
                    let mapped = CurvePoint2::from(Point2::new(
                        &m00 * &x + &m01 * &y,
                        &m10 * &x + &m11 * &y,
                    ));
                    let (Some(before), Some(after)) =
                        (inside(&source, point)?, inside(&result, &mapped)?)
                    else {
                        continue;
                    };
                    prop_assert_eq!(before, after, "{}: similarity moved membership", label);
                }
                result
            }
        };
        let operated = started.elapsed();
        assert_round_trip(&label, &result)?;
        if std::env::var_os("HYPERCURVE_COMPOSED_WORKLOAD_TRACE").is_some() {
            eprintln!(
                "TRACE {label}: operation+oracle {operated:?}, total {:?}, loops {}",
                started.elapsed(),
                result.boundary_loops().len()
            );
        }
        pool.push(result);
    }
    Ok(())
}

/// An inward round offset of a chamfered weighted-conic corner.
///
/// The chamfer's cut vertex lies on a retained rational circular arc with
/// coordinates in a quadratic field. The offset's band connectors pass
/// through that vertex; they are split at the retained vertex itself, so the
/// Boolean sees the same point the region boundary uses rather than a
/// recomputed connector crossing whose equality it could not certify.
#[test]
fn chamfered_conic_inward_round_offset_completes() {
    let seeds = [Seed {
        x: 0,
        y: 0,
        width: 10,
        height: 6,
        lower: 1,
        upper: 0,
        curvature: 3,
        weight: 1,
    }];
    let steps = [Step::Chamfer(0, 5), Step::Offset(1, -1, 0)];
    run_sequence(&seeds, &steps, false).unwrap();
}

/// The regular suite replays a fixed seed so it is deterministic; setting
/// HYPERCURVE_COMPOSED_WORKLOAD_CASES explores that many random cases.
fn config() -> ProptestConfig {
    let requested = std::env::var("HYPERCURVE_COMPOSED_WORKLOAD_CASES")
        .ok()
        .map(|value| value.parse::<u32>().expect("case count"));
    ProptestConfig {
        cases: requested.unwrap_or(6),
        rng_seed: if requested.is_some() {
            proptest::test_runner::RngSeed::Random
        } else {
            proptest::test_runner::RngSeed::Fixed(0x68_79_70_65_72)
        },
        max_shrink_iters: 256,
        // Exploration runs report their minimal case; closed gaps become named
        // regression tests rather than replayed seeds in the regular suite.
        failure_persistence: if requested.is_some() {
            None
        } else {
            Some(Box::new(FileFailurePersistence::WithSource(
                "proptest-regressions",
            )))
        },
        ..ProptestConfig::default()
    }
}

proptest! {
    #![proptest_config(config())]

    /// The randomized qualification gate; size it with
    /// HYPERCURVE_COMPOSED_WORKLOAD_CASES. Every gap it found is also a
    /// regular regression test below.
    #[test]
    fn composed_operation_sequences_satisfy_independent_oracles(
        seeds in prop::collection::vec(seed_strategy(), 2..=3),
        steps in prop::collection::vec(step_strategy(), 3..=5),
        nested in any::<bool>(),
    ) {
        run_sequence(&seeds, &steps, nested)?;
    }
}

/// Generated reproducer: the two sides of one inward band are parallels of
/// the same source at opposite distances, whose squared incidence system
/// contains the source's own diagonal. The component filter must not spend
/// an exact subresultant chain to learn that.
#[test]
fn inward_bevel_band_of_a_quadratic_nurbs_seed_regularizes() {
    let region = seed_region(&Seed {
        x: -9,
        y: 4,
        width: 16,
        height: 15,
        lower: 2,
        upper: 7,
        curvature: 3,
        weight: 5,
    });
    let eroded = region
        .offset(fraction(-1, 4), &OffsetCornerStyle2::Bevel, &STRICT)
        .unwrap()
        .into_value();
    assert!(!eroded.is_empty());
    assert!(
        eroded
            .boolean_region(&region, BooleanOp::Difference, &STRICT)
            .unwrap()
            .into_value()
            .is_empty()
    );
}

/// Generated reproducer: rebuilding an offset from its exported boundary and
/// comparing it with the original pairs carriers on the same analytic
/// parallels over nested ranges, which is the identity overlap.
#[test]
fn round_offset_of_a_nurbs_rational_seed_round_trips() {
    let region = seed_region(&Seed {
        x: -1,
        y: 3,
        width: 8,
        height: 12,
        lower: 7,
        upper: 5,
        curvature: 2,
        weight: 2,
    });
    let dilated = region
        .offset(fraction(1, 2), &OffsetCornerStyle2::Round, &STRICT)
        .unwrap()
        .into_value();
    assert_round_trip("round offset", &dilated).unwrap();
}

/// Generated reproducer: cutting an arc at irrational points materializes
/// conic pieces with surd control points. Rebuilding the union from its
/// exported boundary pairs each piece with an identical copy, which must be
/// decided from the shared definition rather than an implicit equation.
#[test]
fn arc_union_round_trips_after_irrational_cuts() {
    let seeds = [
        Seed {
            x: 4,
            y: 0,
            width: 10,
            height: 11,
            lower: 0,
            upper: 0,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: 0,
            y: 0,
            width: 8,
            height: 11,
            lower: 0,
            upper: 0,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: 0,
            y: -7,
            width: 13,
            height: 6,
            lower: 0,
            upper: 1,
            curvature: 3,
            weight: 1,
        },
    ];
    let steps = [
        Step::Boolean(10, 11, 1),
        Step::Boolean(14, 6, 1),
        Step::Boolean(2, 5, 0),
    ];
    run_sequence(&seeds, &steps, true).unwrap();
}

/// Generated reproducer: the intersection variant leaves a conic piece and a
/// line meeting at an irrational vertex. The loop keeps one exact
/// representation for that vertex, so the rebuilt boundary can re-prove the
/// contact without a zero test on nested surds.
#[test]
fn arc_intersection_round_trips_after_irrational_cuts() {
    let seeds = [
        Seed {
            x: 4,
            y: 0,
            width: 10,
            height: 11,
            lower: 0,
            upper: 0,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: 0,
            y: 0,
            width: 8,
            height: 11,
            lower: 0,
            upper: 0,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: 0,
            y: -7,
            width: 13,
            height: 6,
            lower: 0,
            upper: 1,
            curvature: 3,
            weight: 1,
        },
    ];
    let steps = [
        Step::Boolean(10, 11, 1),
        Step::Boolean(14, 6, 1),
        Step::Boolean(2, 5, 1),
    ];
    run_sequence(&seeds, &steps, true).unwrap();
}

/// Generated reproducer: an arc cut at irrational points beside a conic cut
/// at selected algebraic points. Materializing the arc piece gave it
/// nested-surd controls, so a straight chord on x = 10 could not be located
/// against it. Irrational cuts now keep the source chart.
#[test]
fn conic_cut_at_algebraic_points_round_trips() {
    let seeds = [
        Seed {
            x: 4,
            y: 0,
            width: 10,
            height: 11,
            lower: 4,
            upper: 0,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: 0,
            y: -1,
            width: 10,
            height: 12,
            lower: 0,
            upper: 0,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: 0,
            y: -7,
            width: 11,
            height: 7,
            lower: 0,
            upper: 1,
            curvature: 1,
            weight: 1,
        },
    ];
    let steps = [
        Step::Boolean(10, 12, 1),
        Step::Boolean(14, 1, 1),
        Step::Boolean(2, 5, 1),
    ];
    run_sequence(&seeds, &steps, true).unwrap();
}

/// Generated reproducer: inward round offset of a chamfered NURBS /
/// B-spline seed. A monotone chord/parallel contact search bisected toward
/// a retained algebraic endpoint that its side certificate placed strictly
/// off the chord, and never terminated. The search is now bounded and
/// reports uncertainty, and the pair completes through its general route.
#[test]
fn chamfered_spline_inward_round_offset_terminates() {
    let seeds = [
        Seed {
            x: 5,
            y: 1,
            width: 18,
            height: 6,
            lower: 7,
            upper: 6,
            curvature: 2,
            weight: 5,
        },
        Seed {
            x: 1,
            y: 5,
            width: 9,
            height: 7,
            lower: 2,
            upper: 5,
            curvature: 1,
            weight: 6,
        },
    ];
    let steps = [
        Step::Offset(14, 1, 0),
        Step::Chamfer(9, 4),
        Step::Offset(9, -2, 1),
        Step::Offset(8, -2, 0),
    ];
    run_sequence(&seeds, &steps, false).unwrap();
}

/// Generated reproducer: rebuilding an inward bevel offset of a chamfered
/// rational-cubic / B-spline seed from its exported boundary. The chord-pair
/// rejection filter refined selected-fiber endpoint bounds to 512 steps for
/// chords that really meet, each level a larger local Sturm refinement (no
/// result in 15 minutes); the filter now stops at 64 steps.
#[test]
fn chamfered_rational_cubic_bevel_offset_round_trips() {
    let seeds = [
        Seed {
            x: -2,
            y: 1,
            width: 12,
            height: 14,
            lower: 5,
            upper: 6,
            curvature: 2,
            weight: 4,
        },
        Seed {
            x: -12,
            y: -9,
            width: 13,
            height: 13,
            lower: 2,
            upper: 4,
            curvature: 1,
            weight: 3,
        },
    ];
    let steps = [Step::Chamfer(8, 2), Step::Offset(12, -1, 0)];
    run_sequence(&seeds, &steps, false).unwrap();
}

/// Generated reproducer: a rectangle-and-arc seed unioned with a translate
/// of itself. Rebuilding the union pairs a retained arc piece with a chord
/// from the other operand that ends exactly at the piece's endpoint; the
/// kernel re-derived that vertex as a selected root and could not order it
/// against the chord's own endpoint. A certified equal endpoint is owned.
#[test]
fn translated_arc_union_round_trips() {
    let seeds = [
        Seed {
            x: 0,
            y: 0,
            width: 13,
            height: 14,
            lower: 0,
            upper: 0,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: 0,
            y: 0,
            width: 16,
            height: 6,
            lower: 0,
            upper: 1,
            curvature: 1,
            weight: 1,
        },
    ];
    let steps = [
        Step::Translate(1, 0, 1),
        Step::Boolean(9, 0, 0),
        Step::Boolean(9, 3, 0),
        Step::Boolean(0, 0, 0),
        Step::Boolean(10, 8, 0),
    ];
    run_sequence(&seeds, &steps, false).unwrap();
}

/// Open computational-closure gap found by the generator: rebuilding a round
/// dilation of a thrice-filleted union from its exported boundary. Unary
/// regularization pairs a round-join arc with the adjacent analytic parallel
/// it touches tangentially. Common-root counting is now certified modularly
/// (Hypersolve c88abc6), but isolating the tangential double root in the
/// selected fiber falls back from Bernstein subdivision to a local Sturm
/// sequence whose subresultant coefficients grow without bound in practice.
/// Modular/CRT subresultants over the extension would be the remedy.
#[test]
#[ignore = "open: fiber gcd growth in adjacent join-arc/parallel regularization"]
fn filleted_union_round_dilation_round_trips() {
    let seeds = [
        Seed {
            x: 3,
            y: -7,
            width: 10,
            height: 14,
            lower: 2,
            upper: 4,
            curvature: 1,
            weight: 4,
        },
        Seed {
            x: 0,
            y: -7,
            width: 19,
            height: 7,
            lower: 0,
            upper: 6,
            curvature: 1,
            weight: 1,
        },
        Seed {
            x: -6,
            y: -2,
            width: 17,
            height: 14,
            lower: 6,
            upper: 2,
            curvature: 3,
            weight: 6,
        },
    ];
    let steps = [
        Step::Boolean(12, 12, 0),
        Step::Fillet(13, 4),
        Step::Fillet(9, 6),
        Step::Fillet(13, 7),
        Step::Offset(5, 1, 0),
    ];
    run_sequence(&seeds, &steps, false).unwrap();
}

/// Generated reproducer for a repeated inward round offset of a quadratic
/// seed. The second offset's band arrangement paired a chord with a parallel
/// from the other operand whose endpoint lies exactly on the chord; the
/// monotone kernel accepted only strictly opposite endpoint sides and
/// reported Boundary. That endpoint is now the certified unique contact.
/// It now completes in about 20 s (formerly about 290 s) after the local
/// singleton promotion, gcd vanishing test and adaptive enclosure filter.
#[test]
fn repeated_inward_round_offset_of_a_quadratic_seed_completes() {
    let seed = Seed {
        x: 0,
        y: 0,
        width: 8,
        height: 6,
        lower: 2,
        upper: 2,
        curvature: 1,
        weight: 1,
    };
    let region = seed_region(&seed);
    let once = region
        .offset(fraction(-1, 4), &OffsetCornerStyle2::Round, &STRICT)
        .unwrap()
        .into_value();
    let twice = once
        .offset(fraction(-1, 4), &OffsetCornerStyle2::Round, &STRICT)
        .unwrap()
        .into_value();
    assert!(!twice.is_empty());
    assert!(
        twice
            .boolean_region(&once, BooleanOp::Difference, &STRICT)
            .unwrap()
            .into_value()
            .is_empty()
    );
}

/// Round erosion composes: two quarter erosions of a quadratic seed equal one
/// half erosion. Clipping their shared-parallel overlap compares a monotone
/// chord/parallel endpoint of the repeated offset with the equal algebraic
/// endpoint of the single offset; the squared incidence, with its conjugate
/// sheet excluded on the bracket, decides that equality exactly.
#[test]
fn repeated_round_erosion_composes_with_a_single_erosion() {
    let region = seed_region(&Seed {
        x: 0,
        y: 0,
        width: 8,
        height: 6,
        lower: 2,
        upper: 2,
        curvature: 1,
        weight: 1,
    });
    let quarter = |region: &CurveRegion2| {
        region
            .offset(fraction(-1, 4), &OffsetCornerStyle2::Round, &STRICT)
            .unwrap()
            .into_value()
    };
    let twice = quarter(&quarter(&region));
    let half = region
        .offset(fraction(-1, 2), &OffsetCornerStyle2::Round, &STRICT)
        .unwrap()
        .into_value();
    assert!(
        twice
            .boolean_region(&half, BooleanOp::Xor, &STRICT)
            .unwrap()
            .into_value()
            .is_empty()
    );
}

/// Inward miter offset of a chamfered, thrice-filleted nested pool, found by
/// the generator. Band regularization asks whether a round-join arc meets an
/// adjacent chord only at their shared endpoint. Two exact reductions keep it
/// off coordinate materialization: the chord/parallel circle incidence is
/// replayed in its recursive field, and a derived join point
/// `C + a (P - C) + T`, whose source `P` is the circle center `C` displaced by
/// an exact vector, compares on an axis through the center's own order.
/// No result after 40 minutes became about 33 s.
#[test]
fn chamfered_filleted_pool_inward_miter_offset_completes() {
    let seeds = [
        Seed {
            x: 2,
            y: -4,
            width: 9,
            height: 14,
            lower: 2,
            upper: 4,
            curvature: 2,
            weight: 2,
        },
        Seed {
            x: -8,
            y: -11,
            width: 9,
            height: 14,
            lower: 3,
            upper: 2,
            curvature: 1,
            weight: 6,
        },
    ];
    let steps = [
        Step::Chamfer(13, 3),
        Step::Fillet(13, 5),
        Step::Fillet(5, 1),
        Step::Fillet(0, 4),
        Step::Offset(4, -2, 2),
    ];
    run_sequence(&seeds, &steps, true).unwrap();
}

/// A union whose contacts are coordinate-represented algebraic points must
/// round-trip after an identity map and after a pure translation. Translating
/// such a contact previously dropped its represented coordinates, leaving
/// later equality against another carrier's contact unsupported.
#[test]
fn translated_union_with_algebraic_contacts_round_trips() {
    let seeds = [
        Seed {
            x: 1,
            y: 0,
            width: 8,
            height: 9,
            lower: 0,
            upper: 1,
            curvature: 2,
            weight: 1,
        },
        Seed {
            x: -10,
            y: -7,
            width: 14,
            height: 6,
            lower: 0,
            upper: 5,
            curvature: 3,
            weight: 1,
        },
    ];
    let union = seed_region(&seeds[0])
        .boolean_region(&seed_region(&seeds[1]), BooleanOp::Union, &STRICT)
        .unwrap()
        .into_value();
    assert_round_trip("union", &union).unwrap();
    for (label, tx) in [("identity", Real::zero()), ("shifted", fraction(1, 2))] {
        let moved = union
            .transform_affine(
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &Real::one(),
                &tx,
                &Real::zero(),
                &STRICT,
            )
            .unwrap()
            .into_value();
        assert_round_trip(label, &moved).unwrap();
    }
}

/// A round dilation of a chamfered spline joins the chamfer chord's offset
/// with an arc framed by the chord normal. Reconstructing the result asks
/// that arc to meet a nonadjacent parallel, which needs the chord's exact
/// unit normal. With one exact endpoint and one rational-map endpoint, the
/// direction and speed are represented in the single source field instead of
/// through a resultant of two independent coordinate roots.
#[test]
fn chamfered_spline_round_dilation_round_trips() {
    let seeds = [Seed {
        x: 0,
        y: 0,
        width: 8,
        height: 6,
        lower: 2,
        upper: 0,
        curvature: 1,
        weight: 1,
    }];
    let steps = [Step::Chamfer(0, 1), Step::Offset(1, 1, 0)];
    run_sequence(&seeds, &steps, false).unwrap();
}

/// Open computational-closure gap found by the generator: filleting a
/// fillet-adjacent corner of a weighted cubic spline seed. The second fillet
/// meets the first fillet's selected parallel contact; locating its center
/// and contact signs dense recursive-field polynomials at an algebraic
/// parameter (selected-fiber Sturm subresultants and
/// `sign_at_selected_root`) whose coefficients grow without completing in
/// 38 minutes. The selected-field equality and sign replay without dense
/// elimination (review step 3) is the remedy.
#[test]
#[ignore = "open: dense recursive-field signs in fillet of a fillet-adjacent corner"]
fn refilleted_weighted_spline_corner_completes() {
    let seeds = [Seed {
        x: 6,
        y: -3,
        width: 18,
        height: 9,
        lower: 7,
        upper: 7,
        curvature: 3,
        weight: 2,
    }];
    let steps = [Step::Fillet(0, 5), Step::Fillet(1, 6)];
    run_sequence(&seeds, &steps, false).unwrap();
}

/// Open computational-closure gap found by the generator: a second inward
/// miter offset of a weight-4 quadratic seed. Regularizing the band orders a
/// chord/parallel contact's recursive projective parameter against an equal
/// carrier range endpoint held in another representation. The bounded
/// equality probe replays a field Sturm sequence whose coefficients grow (0.4 s,
/// 2 s, then 14 s per sign), and the complete fallback then projects the
/// recursive relation through a Bareiss resultant. The selected-field equality
/// of review step 3, without scalar elimination, is the remedy.
#[test]
#[ignore = "open: scalar promotion of a recursive projective parameter in repeated miter offset"]
fn repeated_inward_miter_offset_of_a_weighted_quadratic_completes() {
    let seeds = [Seed {
        x: -12,
        y: 8,
        width: 17,
        height: 6,
        lower: 6,
        upper: 4,
        curvature: 1,
        weight: 4,
    }];
    let steps = [Step::Offset(0, -1, 2), Step::Offset(1, -1, 2)];
    run_sequence(&seeds, &steps, false).unwrap();
}
