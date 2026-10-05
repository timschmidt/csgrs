//! Stateful open-path composition qualification.
//!
//! Open paths from every authored curve family pass through generated
//! sequences of vertex fillets, vertex chamfers, reversals, translations,
//! region trims and strokes. Each result is checked against oracles that do
//! not trust its construction: preserved path endpoints and connectivity,
//! exact translation of endpoints, trimmed pieces inside the trimming region,
//! every path vertex strictly inside its round stroke, and reconstruction of
//! each stroke from its exported boundary. Required exact operations must
//! complete; a blocked operation is a failure, while a corner request with no
//! admissible exact solution is a legitimate outcome.

mod support;
use hypercurve::{
    BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2, CurveContext, CurveCornerMode2,
    CurveFillet2, CurvePath2, CurvePoint2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2,
    OffsetCap, OffsetCornerStyle2, Point2, QuadraticBezier2, RationalBezier2,
    RationalQuadraticBezier2, Real, RegionPointLocation,
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

/// One path edge of the given family bowing by `outward`.
fn family_edge(family: u8, start: Point2, end: Point2, outward: i16, weight: &Real) -> Curve2 {
    let first = affine_control(&start, &end, 2, outward);
    let second = affine_control(&start, &end, 4, outward);
    let middle = affine_control(&start, &end, 3, outward);
    match family % 8 {
        0 => Curve2::from(LineSeg2::try_new(start, end).unwrap()),
        1 => Curve2::from(CircularArc2::from_bulge(start, end, fraction(outward, 4)).unwrap()),
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
        )
        .unwrap(),
        _ => Curve2::try_nurbs(
            3,
            vec![start, first, second, end],
            vec![Real::one(), weight.clone(), weight.clone(), Real::one()],
            clamped_cubic_knots(),
        )
        .unwrap(),
    }
}

#[derive(Clone, Debug)]
struct Edge {
    family: u8,
    dx: i16,
    dy: i16,
    outward: i16,
}

#[derive(Clone, Debug)]
struct Seed {
    x: i16,
    y: i16,
    edges: Vec<Edge>,
    weight: i16,
}

/// A zigzag open path: x strictly increases, so distinct edges cannot
/// overlap, while every family and bow direction is exercised.
fn seed_path(seed: &Seed) -> CurvePath2 {
    let weight = fraction(seed.weight, 3);
    let (mut x, mut y) = (seed.x, seed.y);
    let curves = seed
        .edges
        .iter()
        .map(|edge| {
            let start = point(x, y);
            x += edge.dx;
            y += edge.dy;
            family_edge(edge.family, start, point(x, y), edge.outward, &weight)
        })
        .collect();
    CurvePath2::try_new(curves).unwrap()
}

/// A closed axis-aligned trimming region.
fn trim_region(x: i16, y: i16, width: i16, height: i16) -> CurveRegion2 {
    let (max_x, max_y) = (x + width, y + height);
    let corners = [
        point(x, y),
        point(max_x, y),
        point(max_x, max_y),
        point(x, max_y),
    ];
    let curves = (0..4)
        .map(|index| {
            Curve2::from(
                LineSeg2::try_new(corners[index].clone(), corners[(index + 1) % 4].clone())
                    .unwrap(),
            )
        })
        .collect();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[CurvePath2::try_new(curves).unwrap()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap()
}

#[derive(Clone, Debug)]
enum Step {
    Fillet(usize, i16),
    Chamfer(usize, i16),
    Reverse,
    Translate(i16, i16),
    Trim(i16, i16, i16, i16),
    Stroke(i16, u8),
}

fn required<T>(label: &str, result: hypercurve::ExactCurveResult<T>) -> Result<T, TestCaseError> {
    result
        .map_err(|error| TestCaseError::fail(format!("{label}: required exact operation: {error}")))
}

fn same_point(
    label: &str,
    actual: &CurvePoint2,
    expected: &CurvePoint2,
) -> Result<(), TestCaseError> {
    let equal = required(label, Ok(actual.coincides_with(expected, &STRICT)))?;
    prop_assert_eq!(
        equal.value,
        Classification::Decided(true),
        "{}: expected coincident points",
        label
    );
    Ok(())
}

fn region_location(
    label: &str,
    region: &CurveRegion2,
    point: &CurvePoint2,
) -> Result<RegionPointLocation, TestCaseError> {
    let location = crate::support::under(&STRICT, || region.classify_point(point))
        .map_err(|error| TestCaseError::fail(format!("{label}: classification failed: {error}")))?
        .into_value();
    Ok(location)
}

fn assert_round_trip(label: &str, region: &CurveRegion2) -> Result<(), TestCaseError> {
    if region.is_empty() {
        return Ok(());
    }
    let paths = required(
        label,
        crate::support::under(&STRICT, || region.boundary_paths()),
    )?
    .into_value();
    let rebuilt = required(
        label,
        crate::support::under(&STRICT, || {
            CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero)
        }),
    )?
    .into_value();
    let difference = required(
        label,
        crate::support::under(&STRICT, || rebuilt.boolean_region(region, BooleanOp::Xor)),
    )?;
    prop_assert!(
        difference.value.is_empty(),
        "{label}: exported stroke boundary rebuilt a different set"
    );
    Ok(())
}

fn path_vertices(path: &CurvePath2) -> Vec<CurvePoint2> {
    let mut vertices = vec![path.start()];
    vertices.extend(path.curves().iter().map(Curve2::end));
    vertices
}

fn run_sequence(seed: &Seed, steps: &[Step]) -> Result<(), TestCaseError> {
    let mut path = seed_path(seed);
    if std::env::var_os("HYPERCURVE_OPEN_PATH_TRACE").is_some() {
        eprintln!("CASE seed={seed:?} steps={steps:?}");
    }
    for (index, step) in steps.iter().enumerate() {
        let label = format!("step {index} {step:?}");
        let started = std::time::Instant::now();
        let (start, end) = (path.start(), path.end());
        match *step {
            Step::Fillet(vertex, radius) => {
                let vertex = 1 + vertex % path.curves().len().saturating_sub(1).max(1);
                if vertex >= path.curves().len() {
                    continue;
                }
                let solutions = required(
                    &label,
                    crate::support::under(&STRICT, || {
                        path.fillet_vertex(
                            vertex,
                            &CurveFillet2::new(fraction(radius, 4)),
                            CurveCornerMode2::TrimOnly,
                        )
                    }),
                )?
                .into_value();
                if let Some(filleted) = solutions.into_solutions().into_iter().next() {
                    same_point(&label, &filleted.start(), &start)?;
                    same_point(&label, &filleted.end(), &end)?;
                    path = filleted;
                }
            }
            Step::Chamfer(vertex, setback) => {
                let vertex = 1 + vertex % path.curves().len().saturating_sub(1).max(1);
                if vertex >= path.curves().len() {
                    continue;
                }
                let setback = fraction(setback, 4);
                let solutions = required(
                    &label,
                    crate::support::under(&STRICT, || {
                        path.chamfer_vertex_by_setbacks(
                            vertex,
                            setback.clone(),
                            setback,
                            CurveCornerMode2::TrimOnly,
                        )
                    }),
                )?
                .into_value();
                if let Some(chamfered) = solutions.into_solutions().into_iter().next() {
                    same_point(&label, &chamfered.start(), &start)?;
                    same_point(&label, &chamfered.end(), &end)?;
                    path = chamfered;
                }
            }
            Step::Reverse => {
                let reversed =
                    required(&label, crate::support::under(&STRICT, || path.reversed()))?
                        .into_value();
                same_point(&label, &reversed.start(), &end)?;
                same_point(&label, &reversed.end(), &start)?;
                let restored = required(
                    &label,
                    crate::support::under(&STRICT, || reversed.reversed()),
                )?
                .into_value();
                same_point(&label, &restored.start(), &start)?;
                path = reversed;
            }
            Step::Translate(x, y) => {
                let (dx, dy) = (fraction(x, 2), fraction(y, 2));
                let translation = hypercurve::Similarity2::try_from_real_affine(
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    dx.clone(),
                    dy.clone(),
                )
                .unwrap();
                let translated = required(
                    &label,
                    crate::support::under(&STRICT, || path.transform_similarity(&translation)),
                )?
                .into_value();
                for (before, after) in [(&start, translated.start()), (&end, translated.end())] {
                    let Some(before) = before.coordinates() else {
                        continue;
                    };
                    let expected = Point2::new(before.x() + &dx, before.y() + &dy);
                    same_point(&label, &after, &expected.into())?;
                }
                path = translated;
            }
            Step::Trim(x, y, width, height) => {
                let region = trim_region(x, y, width, height);
                let trims = required(
                    &label,
                    crate::support::under(&STRICT, || path.trim_inside_region(&region)),
                )?
                .into_value();
                for trim in &trims {
                    for fragment in trim.fragments() {
                        let curve = fragment.trim_fragment().curve();
                        for endpoint in [curve.start(), curve.end()] {
                            let location = region_location(&label, &region, &endpoint)?;
                            prop_assert!(
                                location != RegionPointLocation::Outside,
                                "{}: trimmed piece left its region",
                                label
                            );
                        }
                    }
                }
            }
            Step::Stroke(half_width, corner) => {
                let corner = match corner % 3 {
                    0 => OffsetCornerStyle2::Round,
                    1 => OffsetCornerStyle2::Bevel,
                    _ => OffsetCornerStyle2::Miter { limit: integer(4) },
                };
                let stroke = required(
                    &label,
                    crate::support::under(&STRICT, || {
                        CurveRegion2::stroke_path(
                            &path,
                            fraction(half_width, 4),
                            &corner,
                            OffsetCap::Round,
                        )
                    }),
                )?
                .into_value();
                for vertex in path_vertices(&path) {
                    let location = region_location(&label, &stroke, &vertex)?;
                    prop_assert_eq!(
                        location,
                        RegionPointLocation::Inside,
                        "{}: a path vertex left its round-capped stroke",
                        label
                    );
                }
                assert_round_trip(&label, &stroke)?;
            }
        }
        if std::env::var_os("HYPERCURVE_OPEN_PATH_TRACE").is_some() {
            eprintln!("TRACE {label}: {:?}", started.elapsed());
        }
    }
    Ok(())
}

fn edge() -> impl Strategy<Value = Edge> {
    (0_u8..8, 3_i16..9, -6_i16..=6, -2_i16..=2)
        .prop_filter("an edge bows by a nonzero amount", |(_, _, _, outward)| {
            *outward != 0
        })
        .prop_map(|(family, dx, dy, outward)| Edge {
            family,
            dx,
            dy,
            outward,
        })
}

fn seed() -> impl Strategy<Value = Seed> {
    (
        -8_i16..=8,
        -8_i16..=8,
        prop::collection::vec(edge(), 2..=4),
        1_i16..=6,
    )
        .prop_map(|(x, y, edges, weight)| Seed {
            x,
            y,
            edges,
            weight,
        })
}

fn step() -> impl Strategy<Value = Step> {
    prop_oneof![
        (0_usize..4, 1_i16..=4).prop_map(|(vertex, radius)| Step::Fillet(vertex, radius)),
        (0_usize..4, 1_i16..=4).prop_map(|(vertex, setback)| Step::Chamfer(vertex, setback)),
        Just(Step::Reverse),
        (-6_i16..=6, -6_i16..=6).prop_map(|(x, y)| Step::Translate(x, y)),
        (-10_i16..=6, -10_i16..=6, 4_i16..=16, 4_i16..=16)
            .prop_map(|(x, y, width, height)| Step::Trim(x, y, width, height)),
        (1_i16..=3, 0_u8..3).prop_map(|(half_width, corner)| Step::Stroke(half_width, corner)),
    ]
}

/// The regular suite replays a fixed seed so it is deterministic; setting
/// HYPERCURVE_OPEN_PATH_CASES explores that many random cases.
fn config() -> ProptestConfig {
    let requested = std::env::var("HYPERCURVE_OPEN_PATH_CASES")
        .ok()
        .map(|value| value.parse::<u32>().expect("case count"));
    ProptestConfig {
        cases: requested.unwrap_or(6),
        rng_seed: if requested.is_some() {
            proptest::test_runner::RngSeed::Random
        } else {
            proptest::test_runner::RngSeed::Fixed(0x6f_70_65_6e)
        },
        max_shrink_iters: 256,
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

    /// Generated open-path sequences satisfy independent exact oracles.
    #[test]
    fn open_path_sequences_satisfy_independent_oracles(
        seed in seed(),
        steps in prop::collection::vec(step(), 1..=4),
    ) {
        run_sequence(&seed, &steps)?;
    }
}

/// Filleting the same vertex twice. The first fillet makes the vertex a
/// tangent junction between the trimmed rational cubic and the fillet arc;
/// the second request locates candidate contacts on the radius-offset
/// support through point-incidence signs that do not complete.
#[test]
#[ignore = "open: fillet of a tangent junction left by an earlier fillet"]
fn refilleting_a_filleted_vertex_completes() {
    let seed = Seed {
        x: -7,
        y: 5,
        edges: vec![
            Edge {
                family: 5,
                dx: 5,
                dy: -3,
                outward: -2,
            },
            Edge {
                family: 0,
                dx: 6,
                dy: 2,
                outward: -2,
            },
            Edge {
                family: 6,
                dx: 6,
                dy: -6,
                outward: -1,
            },
        ],
        weight: 1,
    };
    run_sequence(&seed, &[Step::Fillet(0, 4), Step::Fillet(0, 4)]).unwrap();
}

/// Stroking a line followed by a sharply curved rational quadratic. Near the
/// curve's start its curvature radius is smaller than the half-width, so the
/// concave-side offset runs against its source there. Inner and outer joins
/// follow the source turn, read from the offset ends about the shared vertex,
/// so the outer side keeps its join and the vertex stays inside the stroke.
#[test]
fn stroke_keeps_the_outer_join_at_a_high_curvature_vertex() {
    let seed = Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 0,
                dx: 3,
                dy: 0,
                outward: 1,
            },
            Edge {
                family: 4,
                dx: 3,
                dy: 0,
                outward: -1,
            },
        ],
        weight: 1,
    };
    for corner in 0..3 {
        run_sequence(&seed, &[Step::Stroke(2, corner)]).unwrap();
    }
}

/// Stroking a filleted line/arc path whose half-width equals the fillet
/// radius. The fillet arc's concave offset collapses to its center, which the
/// adjacent line's offset end reaches exactly. The trimmed line runs along an
/// axis to a radical contact, so its offset normal must be the exact axis
/// unit vector; dividing the radical length by the square root of its own
/// square left an expression equal to one that no exact zero test could use.
#[test]
fn stroke_of_a_fillet_at_its_radius_completes() {
    let seed = Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 0,
                dx: 3,
                dy: 0,
                outward: -1,
            },
            Edge {
                family: 1,
                dx: 3,
                dy: -1,
                outward: -1,
            },
        ],
        weight: 1,
    };
    run_sequence(&seed, &[Step::Fillet(0, 1), Step::Stroke(1, 0)]).unwrap();
}

/// A miter stroke of a four-edge line/arc/rational-cubic/NURBS path. The
/// stroke completes, and rebuilding its exported boundary pairs a rational
/// Bezier with a quadratic piece whose relation needs a sign over nested
/// radical coordinates; the exact iterated square-root tower sign decides it.
/// The round trip takes about two minutes.
#[test]
#[ignore = "slow: the mixed-family miter stroke round trip takes about two minutes"]
fn mixed_family_miter_stroke_round_trips() {
    let seed = Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 0,
                dx: 6,
                dy: 1,
                outward: 2,
            },
            Edge {
                family: 1,
                dx: 8,
                dy: 1,
                outward: -1,
            },
            Edge {
                family: 5,
                dx: 4,
                dy: 3,
                outward: 1,
            },
            Edge {
                family: 7,
                dx: 4,
                dy: 1,
                outward: 1,
            },
        ],
        weight: 5,
    };
    run_sequence(&seed, &[Step::Stroke(2, 2)]).unwrap();
}

/// Stroking a filleted line/parabola path. The fillet arc against the
/// parabola is a generated algebraic circle carrier with no native Bezier
/// image; the stroke offsets the path's retained fragments, as a region
/// offset does, instead of requiring native Bezier spans.
#[test]
fn stroke_of_a_generated_fillet_arc_completes() {
    let seed = Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 0,
                dx: 3,
                dy: 0,
                outward: -1,
            },
            Edge {
                family: 2,
                dx: 3,
                dy: 0,
                outward: 1,
            },
        ],
        weight: 1,
    };
    run_sequence(&seed, &[Step::Fillet(0, 1), Step::Stroke(1, 0)]).unwrap();
}

/// A round stroke of a chamfered line/arc path. Rebuilding the exported
/// boundary pairs an offset line, cut at irrational parameters, with the
/// join arc it meets. The split shares the junction's representative point
/// between both pieces and keeps the line's source support, so the line/arc
/// kernel proves the incidence structurally and decides the tangency along
/// the exact source direction.
#[test]
fn chamfered_line_arc_stroke_round_trips() {
    let seed = Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 0,
                dx: 3,
                dy: 0,
                outward: 1,
            },
            Edge {
                family: 1,
                dx: 5,
                dy: 0,
                outward: -1,
            },
        ],
        weight: 1,
    };
    run_sequence(&seed, &[Step::Chamfer(0, 1), Step::Stroke(1, 0)]).unwrap();
}

/// A bevel stroke of a filleted hyperbola/hyperbola path (two weight-2
/// rational quadratics). The stroke alone completes in about 24 s. After the
/// fillet trims both hyperbolas at algebraic tangency parameters, the
/// stroke's tangent junctions and its join chord/parallel pairs are
/// certified, but regularizing its bands then isolates a selected
/// cusp-semicircle/parallel fiber through a local Sturm sequence that did
/// not finish within fifteen minutes.
#[test]
#[ignore = "open: selected cusp-semicircle/parallel fiber isolation is too slow for fillet-trimmed hyperbolas"]
fn bevel_stroke_of_a_filleted_hyperbola_pair_completes() {
    let seed = Seed {
        x: 3,
        y: 8,
        edges: vec![
            Edge {
                family: 4,
                dx: 6,
                dy: -1,
                outward: -1,
            },
            Edge {
                family: 4,
                dx: 6,
                dy: 0,
                outward: -2,
            },
        ],
        weight: 2,
    };
    run_sequence(&seed, &[Step::Fillet(2, 2), Step::Stroke(3, 1)]).unwrap();
}

/// A round stroke of an arc followed by a line. The exported offset line
/// meets the round join arc tangentially; both pieces share the junction's
/// representative point and the line keeps its exact source direction, so
/// the rebuild decides that tangency without a nested-radical zero test.
#[test]
fn arc_line_round_stroke_round_trips() {
    let seed = Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 1,
                dx: 3,
                dy: 1,
                outward: 1,
            },
            Edge {
                family: 0,
                dx: 4,
                dy: -1,
                outward: 1,
            },
        ],
        weight: 1,
    };
    run_sequence(&seed, &[Step::Stroke(1, 0)]).unwrap();
}

/// A miter stroke of a cubic/rational-cubic/rational-cubic/arc path. The
/// parallel/parallel intersection projections form rational resultants of
/// degree up to 214; their square-free parts isolate with Bernstein
/// subdivision instead of Sturm sequences, which never finished. The stroke
/// and its boundary round trip complete in about three minutes, mostly
/// forming those resultants and certifying which of their roots are
/// rational.
#[test]
#[ignore = "slow: high-degree parallel/parallel resultants take about three minutes"]
fn mixed_cubic_arc_miter_stroke_completes() {
    let seed = Seed {
        x: -2,
        y: 1,
        edges: vec![
            Edge {
                family: 3,
                dx: 5,
                dy: -4,
                outward: -1,
            },
            Edge {
                family: 5,
                dx: 3,
                dy: 3,
                outward: -1,
            },
            Edge {
                family: 5,
                dx: 7,
                dy: -1,
                outward: 2,
            },
            Edge {
                family: 1,
                dx: 6,
                dy: 5,
                outward: 1,
            },
        ],
        weight: 5,
    };
    run_sequence(&seed, &[Step::Stroke(3, 2)]).unwrap();
}

/// A bevel stroke of a chamfered NURBS/B-spline/rational-quadratic/cubic
/// path. Rebuilding its exported boundary pairs an algebraic chord with a
/// parallel carrier; `parallel_tangent_cross_sign_on_region_range` refines a
/// recursive projective parameter whose defining sign is evaluated in a
/// recursive quadratic tower through algebraic tensor images and Sturm
/// refinement, which did not finish within fourteen minutes.
#[test]
#[ignore = "open: chord/parallel tangent sign over a recursive quadratic tower is too slow"]
fn chamfered_spline_bevel_stroke_round_trips() {
    let seed = Seed {
        x: 1,
        y: -5,
        edges: vec![
            Edge {
                family: 7,
                dx: 5,
                dy: -3,
                outward: -2,
            },
            Edge {
                family: 6,
                dx: 8,
                dy: -1,
                outward: -1,
            },
            Edge {
                family: 4,
                dx: 3,
                dy: 6,
                outward: 2,
            },
            Edge {
                family: 3,
                dx: 8,
                dy: 4,
                outward: 2,
            },
        ],
        weight: 2,
    };
    run_sequence(&seed, &[Step::Chamfer(2, 2), Step::Stroke(2, 1)]).unwrap();
}

/// A miter stroke of a filleted rational-cubic/arc path (weight 6). The
/// offset spans meet the fillet's tangency at a selected-fiber parameter.
/// Promoting it to a Bezier parameter (endpoint tangents, span fragments,
/// recursive support lines) forms a high-degree bivariate resultant, and its
/// square-free reduction runs a Euclidean GCD over radical coefficients;
/// keeping the selected-fiber form instead moves the cost into repeated
/// local-field refinements. The stroke did not finish within ten minutes.
#[test]
#[ignore = "open: selected-fiber fillet tangency in a high-weight rational stroke is too slow"]
fn miter_stroke_of_a_filleted_heavy_rational_cubic_completes() {
    let seed = Seed {
        x: -1,
        y: 1,
        edges: vec![
            Edge {
                family: 5,
                dx: 8,
                dy: 2,
                outward: 1,
            },
            Edge {
                family: 1,
                dx: 3,
                dy: 5,
                outward: 1,
            },
        ],
        weight: 6,
    };
    run_sequence(
        &seed,
        &[Step::Reverse, Step::Fillet(3, 2), Step::Stroke(2, 2)],
    )
    .unwrap();
}

fn chamfered_cubic_line_seed() -> Seed {
    Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 3,
                dx: 4,
                dy: -5,
                outward: -1,
            },
            Edge {
                family: 0,
                dx: 5,
                dy: -4,
                outward: -2,
            },
            Edge {
                family: 5,
                dx: 6,
                dy: 2,
                outward: 2,
            },
            Edge {
                family: 0,
                dx: 4,
                dy: -6,
                outward: 2,
            },
        ],
        weight: 2,
    }
}

/// A chamfer cut on a cubic is an algebraic vertex; the bevel stroke's
/// boundary beside it contains straight parallel pieces cut at algebraic
/// parameters. Locating that vertex classifies those pieces through their
/// retained rational parallel component instead of declining for want of
/// explicit endpoints.
#[test]
fn bevel_stroke_contains_an_algebraic_chamfer_vertex() {
    let seed = chamfered_cubic_line_seed();
    let path = seed_path(&seed);
    let chamfered = path
        .chamfer_vertex_by_setbacks(
            1,
            fraction(1, 4),
            fraction(1, 4),
            CurveCornerMode2::TrimOnly,
        )
        .unwrap()
        .into_solutions()
        .into_iter()
        .next()
        .expect("the cubic/line vertex admits a setback chamfer");
    let stroke = CurveRegion2::stroke_path(
        &chamfered,
        fraction(2, 4),
        &OffsetCornerStyle2::Bevel,
        OffsetCap::Round,
    )
    .unwrap();
    for vertex in path_vertices(&chamfered) {
        assert_eq!(
            region_location("chamfered stroke", &stroke, &vertex).unwrap(),
            RegionPointLocation::Inside
        );
    }
}

/// The same chamfered bevel stroke through the harness, including its
/// boundary round trip. Rebuilding that boundary runs the recursive
/// chord/parallel incidence system over a recursive quadratic tower and does
/// not finish within minutes.
#[test]
#[ignore = "open: chord/parallel incidence over a recursive quadratic tower is too slow"]
fn chamfered_cubic_line_bevel_stroke_round_trips() {
    run_sequence(
        &chamfered_cubic_line_seed(),
        &[Step::Chamfer(3, 1), Step::Stroke(2, 1)],
    )
    .unwrap();
}

/// A round stroke of a filleted rational-cubic/NURBS/NURBS path (weight 5).
/// The fillet itself takes about five minutes. The stroke's tangent
/// junctions and join chord/parallel endpoints are certified, but the stroke
/// then did not finish within twenty-five minutes.
#[test]
#[ignore = "open: a filleted heavy NURBS round stroke is too slow"]
fn round_stroke_of_a_filleted_heavy_nurbs_path_completes() {
    let seed = Seed {
        x: 2,
        y: 4,
        edges: vec![
            Edge {
                family: 5,
                dx: 7,
                dy: 6,
                outward: 2,
            },
            Edge {
                family: 7,
                dx: 4,
                dy: 5,
                outward: 2,
            },
            Edge {
                family: 7,
                dx: 3,
                dy: -1,
                outward: -1,
            },
        ],
        weight: 5,
    };
    run_sequence(&seed, &[Step::Fillet(3, 2), Step::Stroke(3, 0)]).unwrap();
}

/// A bevel stroke of a chamfered quadratic/quadratic/rational-quadratic path
/// (weight 1, so all low-degree). Rebuilding its exported boundary runs the
/// recursive chord/parallel system, whose local candidates are filtered by
/// `parameter_is_in_ordered_range`. A candidate equal to a range endpoint is
/// compared by refining both recursive projective parameters; every step
/// evaluates near-zero defining signs in a recursive quadratic tower, and
/// the comparison did not finish within ten minutes.
#[test]
#[ignore = "open: equal recursive projective parameters are compared by refinement"]
fn chamfered_quadratic_bevel_stroke_round_trips() {
    let seed = Seed {
        x: 2,
        y: 5,
        edges: vec![
            Edge {
                family: 2,
                dx: 7,
                dy: 0,
                outward: 2,
            },
            Edge {
                family: 2,
                dx: 4,
                dy: -3,
                outward: -1,
            },
            Edge {
                family: 4,
                dx: 7,
                dy: 6,
                outward: -1,
            },
        ],
        weight: 1,
    };
    run_sequence(
        &seed,
        &[
            Step::Translate(3, -5),
            Step::Translate(-2, -1),
            Step::Chamfer(2, 3),
            Step::Stroke(3, 1),
        ],
    )
    .unwrap();
}

/// A miter stroke of a chamfered rational-quadratic/NURBS path (weight 4).
/// Regularizing the stroke's bands pairs a join chord with a parallel; the
/// recursive chord/parallel kernel projects its selected dense system onto
/// the last axis through quotient-ring fiber resultants (Bareiss
/// determinants of integer polynomial matrices), which did not finish
/// within fourteen minutes.
#[test]
#[ignore = "open: recursive chord/parallel dense projection is too slow"]
fn chamfered_rational_nurbs_miter_stroke_completes() {
    let seed = Seed {
        x: 8,
        y: -6,
        edges: vec![
            Edge {
                family: 4,
                dx: 4,
                dy: 4,
                outward: 2,
            },
            Edge {
                family: 7,
                dx: 6,
                dy: 6,
                outward: -1,
            },
        ],
        weight: 4,
    };
    run_sequence(&seed, &[Step::Chamfer(2, 3), Step::Stroke(2, 2)]).unwrap();
}

/// A bevel stroke of a trimmed, chamfered arc/cubic/rational-cubic path
/// (weight 5). A bevel triangle edge overlaps half of a segment band's end
/// cap in reverse; ordering a cap point along that edge must relate two
/// collinear chords with different monotone parameter axes, which once
/// misordered the point and left a dangling cap edge. Band regularization
/// now completes, and rebuilding the stroke region then spends its time in
/// recursive quadratic tower refinement of chord/parallel tangent signs.
#[test]
#[ignore = "open: recursive tower refinement while rebuilding the stroke region"]
fn trimmed_chamfered_arc_cubic_bevel_stroke_completes() {
    let seed = Seed {
        x: 0,
        y: 0,
        edges: vec![
            Edge {
                family: 1,
                dx: 8,
                dy: 3,
                outward: -2,
            },
            Edge {
                family: 3,
                dx: 5,
                dy: 6,
                outward: 1,
            },
            Edge {
                family: 5,
                dx: 3,
                dy: 3,
                outward: 1,
            },
        ],
        weight: 5,
    };
    run_sequence(
        &seed,
        &[
            Step::Trim(4, -4, 13, 8),
            Step::Chamfer(1, 2),
            Step::Stroke(1, 1),
        ],
    )
    .unwrap();
}

/// A round stroke, a fillet of the stroked boundary path and a bevel stroke
/// of a cubic/arc path (weight 5). The second stroke proves analytic
/// parallel points equal through recursive quadratic field signs and
/// algebraic tensor images; it did not finish within fourteen minutes.
#[test]
#[ignore = "open: point equality over recursive quadratic fields in a repeated stroke is too slow"]
fn stroke_fillet_stroke_of_a_cubic_arc_path_completes() {
    let seed = Seed {
        x: -6,
        y: -4,
        edges: vec![
            Edge {
                family: 3,
                dx: 8,
                dy: -5,
                outward: -1,
            },
            Edge {
                family: 1,
                dx: 8,
                dy: -5,
                outward: -1,
            },
        ],
        weight: 5,
    };
    run_sequence(
        &seed,
        &[Step::Stroke(2, 0), Step::Fillet(1, 2), Step::Stroke(2, 1)],
    )
    .unwrap();
}

/// A round stroke of a filleted NURBS/NURBS path (weight 6). Regularizing
/// the stroke bands isolates the round join's selected-fiber intersections
/// with the neighbouring parallel; the fillet's tangent junction leaves a
/// double root that Bernstein subdivision cannot separate, and the local
/// Sturm fallback did not finish within fourteen minutes.
#[test]
#[ignore = "open: tangent-junction double root in selected cusp-semicircle/parallel isolation"]
fn round_stroke_of_a_filleted_nurbs_pair_completes() {
    let seed = Seed {
        x: -3,
        y: 4,
        edges: vec![
            Edge {
                family: 7,
                dx: 5,
                dy: 3,
                outward: -2,
            },
            Edge {
                family: 7,
                dx: 8,
                dy: -2,
                outward: 1,
            },
        ],
        weight: 6,
    };
    run_sequence(&seed, &[Step::Fillet(1, 2), Step::Stroke(1, 0)]).unwrap();
}
