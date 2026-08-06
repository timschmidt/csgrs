#[path = "common/pathological.rs"]
mod pathological_fixture;

use std::env;
use std::hint::black_box;
use std::time::{Duration, Instant};

use cavalier_contours::polyline::{
    BooleanOp as CavalierBooleanOp, PlineSource, PlineSourceMut, Polyline,
};
use curvo::prelude::{
    CurveOffsetOption as CurvoCurveOffsetOption, Interpolation as _, Intersects as _,
    NurbsCurve2D as CurvoNurbsCurve2D, Offset as CurvoOffset, Split as _,
};
use geo::{BooleanOps as _, Coord, LineString, Polygon};
#[cfg(feature = "predicates")]
use hypercurve::{
    BezierAlgebraicChord2, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierSplitFragment2, CurveBoundaryInteriorSide2,
    CurveRegionBoundaryLoop2, RationalBezierIntersectionPointEvidence2,
};
use hypercurve::{
    BezierFlatteningOptions, BezierParallelVerificationOptions, BooleanOp, BulgeVertex2,
    Classification, Contour2, CubicBezier2, Curve2, CurveContext, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, CurveString2, FillRule, LineArcRegion2, LineSeg2, NurbsCurve2,
    OffsetCornerStyle2, Point2, RationalBezier2, RationalBezierIntersectionContacts2, Real,
    Segment2, Similarity2,
};
use i_overlay::core::fill_rule::FillRule as OverlayFillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;
use nalgebra::{Point2 as NalgebraPoint2, Point3};

use pathological_fixture::{CrossSuiteDataset, MemoryTier, selected_tiers};

const DEFAULT_SAMPLES: usize = 7;
const DEFAULT_SAMPLE_MILLIS: u64 = 75;
const MAX_CALIBRATED_ITERATIONS: u64 = 1 << 20;

#[derive(Clone, Copy)]
enum CommonBooleanOp {
    Union,
    Intersection,
    Difference,
    Xor,
}

impl CommonBooleanOp {
    const fn hypercurve(self) -> BooleanOp {
        match self {
            Self::Union => BooleanOp::Union,
            Self::Intersection => BooleanOp::Intersection,
            Self::Difference => BooleanOp::Difference,
            Self::Xor => BooleanOp::Xor,
        }
    }
}

struct Runner {
    samples: usize,
    sample_target: Duration,
    fixed_iterations: Option<u64>,
    group_filter: Option<String>,
    implementation_filter: Option<String>,
}

impl Runner {
    fn from_environment() -> Self {
        let samples = parse_env("HYPERCURVE_COMPARE_SAMPLES").unwrap_or(DEFAULT_SAMPLES);
        let sample_millis =
            parse_env("HYPERCURVE_COMPARE_SAMPLE_MS").unwrap_or(DEFAULT_SAMPLE_MILLIS);
        let fixed_iterations = parse_env("HYPERCURVE_COMPARE_ITERS");
        let group_filter = env::var("HYPERCURVE_COMPARE_GROUP").ok();
        let implementation_filter = env::var("HYPERCURVE_COMPARE_IMPL").ok();
        assert!(samples > 0, "HYPERCURVE_COMPARE_SAMPLES must be nonzero");
        assert!(
            fixed_iterations != Some(0),
            "HYPERCURVE_COMPARE_ITERS must be nonzero",
        );
        Self {
            samples,
            sample_target: Duration::from_millis(sample_millis),
            fixed_iterations,
            group_filter,
            implementation_filter,
        }
    }

    fn group_enabled(&self, group: &str) -> bool {
        self.group_filter
            .as_ref()
            .is_none_or(|filter| group.contains(filter))
    }

    fn measure<F>(&self, group: &str, implementation: &str, mut operation: F)
    where
        F: FnMut() -> usize,
    {
        if self
            .implementation_filter
            .as_ref()
            .is_some_and(|filter| implementation != filter)
        {
            return;
        }
        if !self.group_enabled(group) {
            return;
        }
        black_box(operation());
        let iterations = self
            .fixed_iterations
            .unwrap_or_else(|| self.calibrate(&mut operation));
        let mut nanoseconds_per_iteration = Vec::with_capacity(self.samples);
        let mut checksum = 0_usize;

        for _ in 0..self.samples {
            let started = Instant::now();
            for _ in 0..iterations {
                checksum ^= black_box(operation());
            }
            let elapsed = started.elapsed();
            nanoseconds_per_iteration.push(elapsed.as_secs_f64() * 1.0e9 / iterations as f64);
        }

        nanoseconds_per_iteration.sort_by(f64::total_cmp);
        let median = nanoseconds_per_iteration[nanoseconds_per_iteration.len() / 2];
        let minimum = nanoseconds_per_iteration[0];
        let maximum = *nanoseconds_per_iteration.last().unwrap();
        println!(
            "{group:<42} {implementation:<20} median={median:>12.1} ns/iter  min={minimum:>12.1}  max={maximum:>12.1}  iterations/sample={iterations:<8} checksum={checksum}",
        );
    }

    fn calibrate<F>(&self, operation: &mut F) -> u64
    where
        F: FnMut() -> usize,
    {
        let mut iterations = 1_u64;
        loop {
            let started = Instant::now();
            let mut checksum = 0_usize;
            for _ in 0..iterations {
                checksum ^= black_box(operation());
            }
            black_box(checksum);
            let elapsed = started.elapsed();
            if elapsed >= self.sample_target || iterations >= MAX_CALIBRATED_ITERATIONS {
                return iterations;
            }

            let elapsed_seconds = elapsed.as_secs_f64().max(1.0e-9);
            let multiplier = (self.sample_target.as_secs_f64() / elapsed_seconds)
                .ceil()
                .clamp(2.0, 16.0) as u64;
            iterations = iterations
                .saturating_mul(multiplier)
                .min(MAX_CALIBRATED_ITERATIONS);
        }
    }
}

fn parse_env<T>(name: &str) -> Option<T>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    env::var(name).ok().map(|value| {
        value
            .parse()
            .unwrap_or_else(|error| panic!("invalid {name}={value:?}: {error}"))
    })
}

fn real(value: f64) -> Real {
    Real::try_from(value).expect("finite benchmark coordinate")
}

fn hypercurve_contour(points: &[[f64; 2]]) -> Contour2 {
    let vertices = points
        .iter()
        .map(|point| {
            BulgeVertex2::new(
                Point2::new(real(point[0]), real(point[1])),
                Real::from(0_i8),
            )
        })
        .collect::<Vec<_>>();
    Contour2::from_bulge_vertices(&vertices).expect("valid hypercurve benchmark contour")
}

fn hypercurve_line_arc_contour(points: &[[f64; 2]], bulges: &[f64]) -> Contour2 {
    let vertices = points
        .iter()
        .zip(bulges)
        .map(|(point, bulge)| {
            BulgeVertex2::new(Point2::new(real(point[0]), real(point[1])), real(*bulge))
        })
        .collect::<Vec<_>>();
    Contour2::from_bulge_vertices(&vertices).expect("valid line/arc benchmark contour")
}

fn hypercurve_region(points: &[[f64; 2]]) -> LineArcRegion2 {
    LineArcRegion2::from_material_contours(vec![hypercurve_contour(points)])
}

fn cavalier_polyline(points: &[[f64; 2]], bulges: Option<&[f64]>) -> Polyline<f64> {
    let mut polyline = Polyline::new_closed();
    for (index, point) in points.iter().enumerate() {
        polyline.add(
            point[0],
            point[1],
            bulges.map_or(0.0, |values| values[index]),
        );
    }
    polyline
}

fn geo_polygon(points: &[[f64; 2]]) -> Polygon<f64> {
    let mut coordinates = points
        .iter()
        .map(|point| Coord {
            x: point[0],
            y: point[1],
        })
        .collect::<Vec<_>>();
    coordinates.push(coordinates[0]);
    Polygon::new(LineString::new(coordinates), Vec::new())
}

fn hypercurve_boolean_result_size(
    first: &LineArcRegion2,
    second: &LineArcRegion2,
    operation: CommonBooleanOp,
    policy: &CurveContext,
) -> usize {
    let operation = operation.hypercurve();
    let result = first
        .boolean_region(second, operation, FillRule::EvenOdd, policy)
        .expect("hypercurve boolean benchmark completes");
    let Classification::Decided(result) = result else {
        panic!("hypercurve boolean benchmark became uncertain");
    };
    result
        .material_contours()
        .iter()
        .chain(result.hole_contours())
        .map(Contour2::len)
        .sum()
}

fn hypercurve_boundary_contour_result_size(
    first: &LineArcRegion2,
    second: &LineArcRegion2,
    operation: CommonBooleanOp,
    policy: &CurveContext,
) -> usize {
    let operation = operation.hypercurve();
    let result = first
        .boolean_boundary_contours(second, operation, FillRule::EvenOdd, policy)
        .expect("hypercurve boundary-contour benchmark completes");
    let Classification::Decided(result) = result else {
        panic!("hypercurve boundary-contour benchmark became uncertain");
    };
    result.iter().map(Contour2::len).sum()
}

fn hypercurve_boundary_loop_result_size(
    first: &LineArcRegion2,
    second: &LineArcRegion2,
    operation: CommonBooleanOp,
    policy: &CurveContext,
) -> usize {
    let operation = operation.hypercurve();
    let result = first
        .boolean_boundary_loops(second, operation, policy)
        .expect("hypercurve boundary-loop benchmark completes");
    let Classification::Decided(result) = result else {
        panic!("hypercurve boundary-loop benchmark became uncertain");
    };
    result.loops().iter().map(|boundary| boundary.len()).sum()
}

fn cavalier_boolean_result_size(
    first: &Polyline<f64>,
    second: &Polyline<f64>,
    operation: CommonBooleanOp,
) -> usize {
    let operation = match operation {
        CommonBooleanOp::Union => CavalierBooleanOp::Or,
        CommonBooleanOp::Intersection => CavalierBooleanOp::And,
        CommonBooleanOp::Difference => CavalierBooleanOp::Not,
        CommonBooleanOp::Xor => CavalierBooleanOp::Xor,
    };
    let result = first.boolean(second, operation);
    result
        .pos_plines
        .iter()
        .chain(result.neg_plines.iter())
        .map(|result| result.pline.vertex_count())
        .sum()
}

fn ioverlay_boolean_result_size(
    first: &Vec<[f64; 2]>,
    second: &Vec<[f64; 2]>,
    operation: CommonBooleanOp,
) -> usize {
    let operation = match operation {
        CommonBooleanOp::Union => OverlayRule::Union,
        CommonBooleanOp::Intersection => OverlayRule::Intersect,
        CommonBooleanOp::Difference => OverlayRule::Difference,
        CommonBooleanOp::Xor => OverlayRule::Xor,
    };
    first
        .overlay(second, operation, OverlayFillRule::EvenOdd)
        .iter()
        .flatten()
        .map(Vec::len)
        .sum()
}

fn geo_boolean_result_size(
    first: &Polygon<f64>,
    second: &Polygon<f64>,
    operation: CommonBooleanOp,
) -> usize {
    let result = match operation {
        CommonBooleanOp::Union => first.union(second),
        CommonBooleanOp::Intersection => first.intersection(second),
        CommonBooleanOp::Difference => first.difference(second),
        CommonBooleanOp::Xor => first.xor(second),
    };
    result
        .iter()
        .map(|polygon| {
            polygon.exterior().0.len()
                + polygon
                    .interiors()
                    .iter()
                    .map(|ring| ring.0.len())
                    .sum::<usize>()
        })
        .sum()
}

fn benchmark_boolean_case(
    runner: &Runner,
    name: &str,
    first_points: Vec<[f64; 2]>,
    second_points: Vec<[f64; 2]>,
    operation: CommonBooleanOp,
) {
    if !runner.group_enabled(name) {
        return;
    }
    let hypercurve_first = hypercurve_region(&first_points);
    let hypercurve_second = hypercurve_region(&second_points);
    let cavalier_first = cavalier_polyline(&first_points, None);
    let cavalier_second = cavalier_polyline(&second_points, None);
    let geo_first = geo_polygon(&first_points);
    let geo_second = geo_polygon(&second_points);
    let policy = CurveContext::STRICT;

    let hypercurve_result_size =
        hypercurve_boolean_result_size(&hypercurve_first, &hypercurve_second, operation, &policy);
    assert_ne!(
        hypercurve_result_size, 0,
        "hypercurve {name} fixture must produce a boundary",
    );
    for (path, result_size) in [
        (
            "boundary contours",
            hypercurve_boundary_contour_result_size(
                &hypercurve_first,
                &hypercurve_second,
                operation,
                &policy,
            ),
        ),
        (
            "boundary loops",
            hypercurve_boundary_loop_result_size(
                &hypercurve_first,
                &hypercurve_second,
                operation,
                &policy,
            ),
        ),
    ] {
        assert_eq!(
            result_size, hypercurve_result_size,
            "hypercurve {name} {path} must match the ordinary region boundary size",
        );
    }
    assert_ne!(
        cavalier_boolean_result_size(&cavalier_first, &cavalier_second, operation),
        0,
        "cavalier_contours {name} fixture must produce a boundary",
    );
    assert_ne!(
        ioverlay_boolean_result_size(&first_points, &second_points, operation),
        0,
        "i_overlay {name} fixture must produce a boundary",
    );
    assert_ne!(
        geo_boolean_result_size(&geo_first, &geo_second, operation),
        0,
        "geo {name} fixture must produce a boundary",
    );

    runner.measure(name, "hypercurve", || {
        hypercurve_boolean_result_size(
            black_box(&hypercurve_first),
            black_box(&hypercurve_second),
            operation,
            &policy,
        )
    });
    runner.measure(name, "hypercurve_contours", || {
        hypercurve_boundary_contour_result_size(
            black_box(&hypercurve_first),
            black_box(&hypercurve_second),
            operation,
            &policy,
        )
    });
    runner.measure(name, "hypercurve_loops", || {
        hypercurve_boundary_loop_result_size(
            black_box(&hypercurve_first),
            black_box(&hypercurve_second),
            operation,
            &policy,
        )
    });
    runner.measure(name, "cavalier_contours", || {
        cavalier_boolean_result_size(
            black_box(&cavalier_first),
            black_box(&cavalier_second),
            operation,
        )
    });
    runner.measure(name, "i_overlay", || {
        ioverlay_boolean_result_size(
            black_box(&first_points),
            black_box(&second_points),
            operation,
        )
    });
    runner.measure(name, "geo", || {
        geo_boolean_result_size(black_box(&geo_first), black_box(&geo_second), operation)
    });
}

fn star_polygon(
    vertex_count: usize,
    center_x: f64,
    center_y: f64,
    outer_radius: f64,
    inner_radius: f64,
    rotation: f64,
) -> Vec<[f64; 2]> {
    assert_eq!(vertex_count % 2, 0);
    (0..vertex_count)
        .map(|index| {
            let angle = rotation + std::f64::consts::TAU * index as f64 / vertex_count as f64;
            let radius = if index % 2 == 0 {
                outer_radius
            } else {
                inner_radius
            };
            [
                center_x + radius * angle.cos(),
                center_y + radius * angle.sin(),
            ]
        })
        .collect()
}

fn benchmark_polygon_booleans(runner: &Runner) {
    benchmark_boolean_case(
        runner,
        "polygon_boolean/rectangles_union",
        vec![[0.0, 0.0], [400.0, 0.0], [400.0, 300.0], [0.0, 300.0]],
        vec![
            [200.0, -100.0],
            [600.0, -100.0],
            [600.0, 200.0],
            [200.0, 200.0],
        ],
        CommonBooleanOp::Union,
    );
    benchmark_boolean_case(
        runner,
        "polygon_boolean/star64_intersection",
        star_polygon(64, 0.0, 0.0, 100.0, 72.0, 0.0),
        star_polygon(64, 18.0, 7.0, 96.0, 68.0, std::f64::consts::PI / 64.0),
        CommonBooleanOp::Intersection,
    );
    benchmark_boolean_case(
        runner,
        "polygon_boolean/star256_intersection",
        star_polygon(256, 0.0, 0.0, 100.0, 72.0, 0.0),
        star_polygon(256, 18.0, 7.0, 96.0, 68.0, std::f64::consts::PI / 256.0),
        CommonBooleanOp::Intersection,
    );
    benchmark_boolean_case(
        runner,
        "polygon_boolean/star1024_intersection",
        star_polygon(1024, 0.0, 0.0, 100.0, 72.0, 0.0),
        star_polygon(1024, 18.0, 7.0, 96.0, 68.0, std::f64::consts::PI / 1024.0),
        CommonBooleanOp::Intersection,
    );
}

fn benchmark_line_arc_boolean(runner: &Runner) {
    let name = "line_arc_boolean/capsules_all_four";
    if !runner.group_enabled(name) {
        return;
    }
    let first_points = vec![[-3.0, -2.0], [3.0, -2.0], [3.0, 2.0], [-3.0, 2.0]];
    let second_points = vec![[-1.0, -2.0], [5.0, -2.0], [5.0, 2.0], [-1.0, 2.0]];
    let bulges = vec![0.0, 1.0, 0.0, 1.0];
    let policy = CurveContext::STRICT;
    let first = CurveRegion2::try_from_native_material_contours(
        vec![hypercurve_line_arc_contour(&first_points, &bulges)],
        &policy,
    )
    .expect("valid first exact capsule")
    .into_value();
    let second = CurveRegion2::try_from_native_material_contours(
        vec![hypercurve_line_arc_contour(&second_points, &bulges)],
        &policy,
    )
    .expect("valid second exact capsule")
    .into_value();
    let cavalier_first = cavalier_polyline(&first_points, Some(&bulges));
    let cavalier_second = cavalier_polyline(&second_points, Some(&bulges));
    let operations = [
        CommonBooleanOp::Union,
        CommonBooleanOp::Intersection,
        CommonBooleanOp::Difference,
        CommonBooleanOp::Xor,
    ];

    runner.measure(name, "hypercurve_exact_batch", || {
        let results = first
            .boolean_regions(black_box(&second), &policy)
            .expect("exact capsule batch completes")
            .into_value();
        [
            results.union(),
            results.intersection(),
            results.difference(),
            results.xor(),
        ]
        .into_iter()
        .flat_map(|region| region.boundary_loops())
        .map(|boundary| boundary.len())
        .sum()
    });
    runner.measure(name, "cavalier_four_calls", || {
        operations
            .into_iter()
            .map(|operation| {
                cavalier_boolean_result_size(
                    black_box(&cavalier_first),
                    black_box(&cavalier_second),
                    operation,
                )
            })
            .sum()
    });
}

fn benchmark_contour_offset(runner: &Runner) {
    if !runner.group_enabled("line_arc_offset/capsule_inward") {
        return;
    }
    let points = vec![[-50.0, -20.0], [50.0, -20.0], [50.0, 20.0], [-50.0, 20.0]];
    let bulges = vec![0.0, 1.0, 0.0, 1.0];
    let hypercurve_vertices = points
        .iter()
        .zip(&bulges)
        .map(|(point, bulge)| {
            BulgeVertex2::new(Point2::new(real(point[0]), real(point[1])), real(*bulge))
        })
        .collect::<Vec<_>>();
    let hypercurve_contour = Contour2::from_bulge_vertices(&hypercurve_vertices)
        .expect("valid hypercurve capsule contour");
    let cavalier_contour = cavalier_polyline(&points, Some(&bulges));
    let policy = CurveContext::STRICT;
    let distance = real(5.0);

    let hypercurve_offset = hypercurve_contour
        .offset_left_checked(distance.clone(), &policy)
        .expect("hypercurve capsule offset completes");
    assert!(matches!(hypercurve_offset, Classification::Decided(_)));
    assert!(!cavalier_contour.parallel_offset(5.0).is_empty());

    let name = "line_arc_offset/capsule_inward";
    runner.measure(name, "hypercurve", || {
        let result = hypercurve_contour
            .offset_left_checked(distance.clone(), &policy)
            .expect("hypercurve capsule offset completes");
        let Classification::Decided(result) = result else {
            panic!("hypercurve capsule offset became uncertain");
        };
        result.len()
    });
    runner.measure(name, "cavalier_contours", || {
        cavalier_contour
            .parallel_offset(black_box(5.0))
            .iter()
            .map(PlineSource::vertex_count)
            .sum()
    });
}

#[cfg(feature = "predicates")]
fn benchmark_algebraic_round_offset(runner: &Runner) {
    let offset_name = "algebraic_round_offset/rectangle";
    let boolean_name = "algebraic_round_boolean/translated_rectangle";
    let reentry_name = "algebraic_round_boolean/correlated_endpoint_reentry";
    let shared_chord_name = "algebraic_round_boolean/shared_chord_circle_order";
    if !runner.group_enabled(offset_name)
        && !runner.group_enabled(boolean_name)
        && !runner.group_enabled(reentry_name)
        && !runner.group_enabled(shared_chord_name)
    {
        return;
    }
    let policy = CurveContext::STRICT;
    let half = (Real::one() / Real::from(2_u8)).expect("exact benchmark half");
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(
        vec![-half, Real::zero(), Real::one()],
        &policy,
    )
    .expect("valid benchmark parameter polynomial")
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => panic!("benchmark polynomial: {reason:?}"),
    };
    let interval = match BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy)
        .expect("valid benchmark parameter interval")
    {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("benchmark interval: {reason:?}"),
    };
    let parameter = match BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
        .expect("valid benchmark parameter isolator")
    {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => panic!("benchmark parameter: {reason:?}"),
    };
    let horizontal = |height: Real| {
        RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), height.clone()),
                Point2::new(Real::one(), height),
            ],
            vec![Real::one(); 2],
        )
        .expect("valid benchmark line image")
    };
    let bottom_right = RationalBezierIntersectionPointEvidence2::Algebraic(
        horizontal(Real::zero())
            .point_at_algebraic_parameter(&parameter, &policy)
            .expect("selected benchmark endpoint"),
    );
    let top_right = RationalBezierIntersectionPointEvidence2::Algebraic(
        horizontal(Real::one())
            .point_at_algebraic_parameter(&parameter, &policy)
            .expect("selected benchmark endpoint"),
    );
    let bottom_left =
        RationalBezierIntersectionPointEvidence2::Exact(Point2::new(Real::zero(), Real::zero()));
    let top_left =
        RationalBezierIntersectionPointEvidence2::Exact(Point2::new(Real::zero(), Real::one()));
    let chord = |start, end| {
        let chord =
            BezierAlgebraicChord2::try_new(start, end, &policy).expect("valid benchmark chord");
        BezierSplitFragment2::AlgebraicChord(match chord {
            Classification::Decided(chord) => chord,
            Classification::Uncertain(reason) => panic!("benchmark chord: {reason:?}"),
        })
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            chord(bottom_left.clone(), bottom_right.clone()),
            chord(bottom_right, top_right.clone()),
            chord(top_right, top_left.clone()),
            chord(top_left, bottom_left),
        ],
        &policy,
    )
    .expect("closed benchmark boundary");
    let hypercurve = CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .expect("valid benchmark region");
    let cavalier = cavalier_polyline(
        &[
            [0.0, 0.0],
            [std::f64::consts::FRAC_1_SQRT_2, 0.0],
            [std::f64::consts::FRAC_1_SQRT_2, 1.0],
            [0.0, 1.0],
        ],
        None,
    );
    let distance = (Real::one() / Real::from(10_u8)).expect("exact benchmark distance");
    let round = OffsetCornerStyle2::Round;
    let miter = OffsetCornerStyle2::Miter {
        limit: Real::from(2_u8),
    };
    assert_eq!(
        hypercurve
            .offset(distance.clone(), &round, &policy)
            .expect("exact algebraic round offset completes")
            .into_value()
            .boundary_loops()
            .len(),
        1,
    );
    assert!(!cavalier.parallel_offset(-0.1).is_empty());

    if runner.group_enabled(offset_name) {
        runner.measure(offset_name, "hypercurve_exact_round", || {
            hypercurve
                .offset(distance.clone(), &round, &policy)
                .expect("exact algebraic round offset completes")
                .into_value()
                .boundary_loops()
                .iter()
                .map(|boundary| boundary.fragments().len())
                .sum()
        });
        runner.measure(offset_name, "hypercurve_exact_miter", || {
            hypercurve
                .offset(distance.clone(), &miter, &policy)
                .expect("exact algebraic miter offset completes")
                .into_value()
                .boundary_loops()
                .iter()
                .map(|boundary| boundary.fragments().len())
                .sum()
        });
        runner.measure(offset_name, "cavalier_f64", || {
            cavalier
                .parallel_offset(black_box(-0.1))
                .iter()
                .map(PlineSource::vertex_count)
                .sum()
        });
    }

    if runner.group_enabled(shared_chord_name) {
        let rounded = hypercurve
            .offset(
                (Real::one() / Real::from(20_u8)).expect("exact shared-chord radius"),
                &round,
                &policy,
            )
            .expect("shared-chord round offset completes")
            .into_value();
        let tall = hypercurve
            .transform_affine(
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &Real::from(3_u8),
                &Real::zero(),
                &Real::from(-1_i8),
                &policy,
            )
            .expect("shared-chord cutter transform completes")
            .into_value();
        let cutter = tall
            .offset(
                (Real::one() / Real::from(40_u8)).expect("exact shared-chord distance"),
                &miter,
                &policy,
            )
            .expect("shared-chord cutter offset completes")
            .into_value();
        let evidence_complete = rounded
            .intersect_region(&cutter, &policy)
            .is_ok_and(|result| result.value.is_complete());
        let all_four_complete = rounded.boolean_regions(&cutter, &policy).is_ok();

        let mut cavalier_rounded = cavalier.parallel_offset(-0.05);
        assert_eq!(cavalier_rounded.len(), 1);
        let cavalier_rounded = cavalier_rounded.pop().unwrap();
        let cavalier_tall = cavalier_polyline(
            &[
                [0.0, -1.0],
                [std::f64::consts::FRAC_1_SQRT_2, -1.0],
                [std::f64::consts::FRAC_1_SQRT_2, 2.0],
                [0.0, 2.0],
            ],
            None,
        );
        let mut cavalier_cutters = cavalier_tall.parallel_offset(-0.025);
        assert_eq!(cavalier_cutters.len(), 1);
        let cavalier_cutter = cavalier_cutters.pop().unwrap();
        let operations = [
            CommonBooleanOp::Union,
            CommonBooleanOp::Intersection,
            CommonBooleanOp::Difference,
            CommonBooleanOp::Xor,
        ];

        runner.measure(
            shared_chord_name,
            if evidence_complete {
                "hypercurve_exact_evidence"
            } else {
                "hypercurve_rejected_evidence"
            },
            || match rounded.intersect_region(&cutter, &policy) {
                Ok(result) => {
                    result.value.contacts().len()
                        + result.value.overlaps().len()
                        + result.value.blockers().len()
                        + usize::from(result.value.is_complete())
                }
                Err(_) => 0,
            },
        );
        runner.measure(
            shared_chord_name,
            if all_four_complete {
                "hypercurve_exact_all_four"
            } else {
                "hypercurve_rejected_all_four"
            },
            || match rounded.boolean_regions(&cutter, &policy) {
                Ok(result) => [
                    result.value.union(),
                    result.value.intersection(),
                    result.value.difference(),
                    result.value.xor(),
                ]
                .into_iter()
                .flat_map(CurveRegion2::boundary_loops)
                .map(|boundary| boundary.fragments().len())
                .sum(),
                Err(_) => 0,
            },
        );
        runner.measure(shared_chord_name, "cavalier_f64_four_calls", || {
            operations
                .into_iter()
                .map(|operation| {
                    cavalier_boolean_result_size(
                        black_box(&cavalier_rounded),
                        black_box(&cavalier_cutter),
                        operation,
                    )
                })
                .sum()
        });
    }

    if runner.group_enabled(boolean_name) || runner.group_enabled(reentry_name) {
        let exact_first = hypercurve
            .offset(distance, &round, &policy)
            .expect("exact algebraic round offset completes")
            .into_value();
        let translation = Similarity2::try_from_real_affine(
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            (Real::one() / Real::from(20_u8)).expect("exact benchmark translation"),
            (Real::one() / Real::from(40_u8)).expect("exact benchmark translation"),
        )
        .expect("valid benchmark translation");
        let exact_second = exact_first
            .transform_similarity(&translation, &policy)
            .expect("selected round region translation completes")
            .into_value();

        let mut cavalier_offset = cavalier.parallel_offset(-0.1);
        assert_eq!(cavalier_offset.len(), 1);
        let cavalier_first = cavalier_offset.pop().unwrap();
        let mut cavalier_second = cavalier_first.clone();
        cavalier_second.translate_mut(0.05, 0.025);
        let operations = [
            CommonBooleanOp::Union,
            CommonBooleanOp::Intersection,
            CommonBooleanOp::Difference,
            CommonBooleanOp::Xor,
        ];

        if runner.group_enabled(boolean_name) {
            let intersection_complete = exact_first
                .intersect_region(&exact_second, &policy)
                .expect("selected round intersection evaluates")
                .into_value()
                .is_complete();
            runner.measure(
                boolean_name,
                if intersection_complete {
                    "hypercurve_exact_intersection"
                } else {
                    "hypercurve_rejected_intersection"
                },
                || {
                    let result = exact_first
                        .intersect_region(&exact_second, &policy)
                        .expect("selected round intersection evaluates")
                        .into_value();
                    result.contacts().len()
                        + result.overlaps().len()
                        + result.blockers().len()
                        + usize::from(result.is_complete())
                },
            );
            if intersection_complete {
                runner.measure(boolean_name, "hypercurve_exact_all_four", || {
                    let result = exact_first
                        .boolean_regions(&exact_second, &policy)
                        .expect("selected round Boolean batch completes")
                        .into_value();
                    [
                        result.union(),
                        result.intersection(),
                        result.difference(),
                        result.xor(),
                    ]
                    .into_iter()
                    .flat_map(CurveRegion2::boundary_loops)
                    .map(|boundary| boundary.fragments().len())
                    .sum()
                });
            }
            runner.measure(boolean_name, "cavalier_f64_four_calls", || {
                operations
                    .into_iter()
                    .map(|operation| {
                        cavalier_boolean_result_size(
                            black_box(&cavalier_first),
                            black_box(&cavalier_second),
                            operation,
                        )
                    })
                    .sum()
            });
        }

        if runner.group_enabled(reentry_name) {
            let reentry_distance =
                (Real::one() / Real::from(20_u8)).expect("exact re-entry distance");
            let reentry_first = hypercurve
                .offset(reentry_distance, &round, &policy)
                .expect("first re-entry round offset completes")
                .into_value();
            let reentry_second = reentry_first
                .transform_similarity(&translation, &policy)
                .expect("second re-entry round region translation completes")
                .into_value();
            let exact_intersection = reentry_first
                .boolean_regions(&reentry_second, &policy)
                .expect("first selected round Boolean batch completes")
                .into_value()
                .intersection()
                .clone();
            let exact_third = reentry_second
                .transform_similarity(&translation, &policy)
                .expect("third selected round region translation completes")
                .into_value();
            let reentry_complete = exact_intersection
                .boolean_regions(&exact_third, &policy)
                .is_ok();
            let evidence_complete = exact_intersection
                .intersect_region(&exact_third, &policy)
                .is_ok_and(|result| result.value.is_complete());
            let single_intersection_complete = exact_intersection
                .boolean_region(&exact_third, BooleanOp::Intersection, &policy)
                .is_ok();
            let mut cavalier_reentry_offset = cavalier.parallel_offset(-0.05);
            assert_eq!(cavalier_reentry_offset.len(), 1);
            let cavalier_reentry_first = cavalier_reentry_offset.pop().unwrap();
            let mut cavalier_reentry_second = cavalier_reentry_first.clone();
            cavalier_reentry_second.translate_mut(0.05, 0.025);
            let cavalier_intersection_result =
                cavalier_reentry_first.boolean(&cavalier_reentry_second, CavalierBooleanOp::And);
            assert_eq!(cavalier_intersection_result.pos_plines.len(), 1);
            assert!(cavalier_intersection_result.neg_plines.is_empty());
            let cavalier_intersection = cavalier_intersection_result.pos_plines[0].pline.clone();
            let mut cavalier_third = cavalier_reentry_second.clone();
            cavalier_third.translate_mut(0.05, 0.025);

            runner.measure(
                reentry_name,
                if evidence_complete {
                    "hypercurve_exact_evidence"
                } else {
                    "hypercurve_rejected_evidence"
                },
                || match exact_intersection.intersect_region(&exact_third, &policy) {
                    Ok(result) => {
                        result.value.contacts().len()
                            + result.value.overlaps().len()
                            + result.value.blockers().len()
                            + usize::from(result.value.is_complete())
                    }
                    Err(_) => 0,
                },
            );
            runner.measure(
                reentry_name,
                if single_intersection_complete {
                    "hypercurve_exact_intersection"
                } else {
                    "hypercurve_rejected_intersection"
                },
                || {
                    exact_intersection
                        .boolean_region(&exact_third, BooleanOp::Intersection, &policy)
                        .map_or(0, |result| {
                            result
                                .value
                                .boundary_loops()
                                .iter()
                                .map(|boundary| boundary.fragments().len())
                                .sum()
                        })
                },
            );
            runner.measure(
                reentry_name,
                if reentry_complete {
                    "hypercurve_exact_all_four"
                } else {
                    "hypercurve_rejected_all_four"
                },
                || match exact_intersection.boolean_regions(&exact_third, &policy) {
                    Ok(result) => [
                        result.value.union(),
                        result.value.intersection(),
                        result.value.difference(),
                        result.value.xor(),
                    ]
                    .into_iter()
                    .flat_map(CurveRegion2::boundary_loops)
                    .map(|boundary| boundary.fragments().len())
                    .sum(),
                    Err(_) => 0,
                },
            );
            runner.measure(reentry_name, "cavalier_f64_four_calls", || {
                operations
                    .into_iter()
                    .map(|operation| {
                        cavalier_boolean_result_size(
                            black_box(&cavalier_intersection),
                            black_box(&cavalier_third),
                            operation,
                        )
                    })
                    .sum()
            });
        }
    }
}

fn benchmark_orthogonal_neck_split(runner: &Runner) {
    let name = "orthogonal_offset/neck_split";
    if !runner.group_enabled(name) {
        return;
    }
    let points = vec![
        [0.0, 0.0],
        [4.0, 0.0],
        [4.0, 1.0],
        [8.0, 1.0],
        [8.0, 0.0],
        [12.0, 0.0],
        [12.0, 4.0],
        [8.0, 4.0],
        [8.0, 3.0],
        [4.0, 3.0],
        [4.0, 4.0],
        [0.0, 4.0],
    ];
    let policy = CurveContext::STRICT;
    let hypercurve =
        CurveRegion2::try_from_native_material_contours(vec![hypercurve_contour(&points)], &policy)
            .expect("hypercurve dumbbell fixture must remain exact")
            .into_value();
    let style = OffsetCornerStyle2::Miter {
        limit: Real::from(2_u8),
    };
    let cavalier = cavalier_polyline(&points, None);
    assert_eq!(
        hypercurve
            .offset(real(-1.5), &style, &policy)
            .expect("hypercurve dumbbell erosion must split")
            .into_value()
            .boundary_loops()
            .len(),
        2,
    );
    assert_eq!(cavalier.parallel_offset(1.5).len(), 2);

    runner.measure(name, "hypercurve_curve_region", || {
        hypercurve
            .offset(real(-1.5), black_box(&style), &policy)
            .expect("hypercurve dumbbell erosion must split")
            .into_value()
            .boundary_loops()
            .iter()
            .map(|boundary| boundary.fragments().len())
            .sum()
    });
    runner.measure(name, "cavalier_contours", || {
        cavalier
            .parallel_offset(black_box(1.5))
            .iter()
            .map(PlineSource::vertex_count)
            .sum()
    });
}

fn benchmark_bezier_offset(runner: &Runner) {
    if !runner.group_enabled("bezier_offset/open_cubic") {
        return;
    }
    let policy = CurveContext::STRICT;
    let controls = [[0.0, 0.0], [1.0, 2.0], [2.0, 1.0], [4.0, 0.0]];
    let source = CubicBezier2::new(
        Point2::new(real(0.0), real(0.0)),
        Point2::new(real(1.0), real(2.0)),
        Point2::new(real(2.0), real(1.0)),
        Point2::new(real(4.0), real(0.0)),
    );
    let source_curve = Curve2::from(source.clone());
    let verification = BezierParallelVerificationOptions::try_new(real(0.05), 14, &policy)
        .expect("valid parallel verification options");
    let flattening = BezierFlatteningOptions::try_new(real(0.05), 14, &policy)
        .expect("valid source flattening options");
    let distance = real(0.1);
    let curvo_curve = CurvoNurbsCurve2D::<f64>::try_new(
        3,
        controls
            .iter()
            .map(|point| Point3::new(point[0], point[1], 1.0))
            .collect(),
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    )
    .expect("valid curvo cubic fixture");
    let curvo_options = CurvoCurveOffsetOption::default()
        .with_distance(-0.1)
        .with_normal_tolerance(0.05)
        .with_knot_tolerance(0.05);

    assert!(matches!(
        source.approximate_parallel_blend2d_certified(distance.clone(), &verification, &policy),
        Ok(Classification::Decided(_))
    ));
    assert!(curvo_curve.offset(curvo_options.clone()).is_ok());

    let name = "bezier_offset/open_cubic";
    runner.measure(name, "hypercurve_certified", || {
        let Classification::Decided(path) = source
            .approximate_parallel_blend2d_certified(distance.clone(), &verification, &policy)
            .expect("hypercurve certified cubic offset completes")
        else {
            panic!("hypercurve certified cubic offset became uncertain");
        };
        path.spans().len()
    });
    runner.measure(name, "hypercurve_chord_fallback", || {
        let Classification::Decided(segmented) = source_curve
            .segment_certified(&flattening, &policy)
            .expect("hypercurve cubic source segmentation completes")
        else {
            panic!("hypercurve cubic source segmentation became uncertain");
        };
        let segments = segmented
            .points()
            .windows(2)
            .map(|edge| {
                LineSeg2::try_new(edge[0].clone(), edge[1].clone())
                    .map(Segment2::Line)
                    .expect("certified source chord is nondegenerate")
            })
            .collect();
        let curve = CurveString2::try_new(segments).expect("certified chords stay connected");
        let Classification::Decided(offset) = curve
            .offset_left_checked(distance.clone(), &policy)
            .expect("legacy chord offset completes")
        else {
            panic!("legacy chord offset became uncertain");
        };
        offset.len()
    });
    runner.measure(name, "curvo_heuristic", || {
        curvo_curve
            .offset(curvo_options.clone())
            .expect("curvo cubic offset completes")
            .iter()
            .map(|compound| compound.spans().len())
            .sum()
    });
}

fn exact_rational_bezier_contact_count(result: RationalBezierIntersectionContacts2) -> usize {
    match result {
        RationalBezierIntersectionContacts2::NoIntersection => 0,
        RationalBezierIntersectionContacts2::Contacts(contacts) => contacts.len(),
        RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, .. } => contacts.len(),
        RationalBezierIntersectionContacts2::Overlap(_)
        | RationalBezierIntersectionContacts2::Incomplete { .. }
        | RationalBezierIntersectionContacts2::DegenerateResultant => {
            panic!("isolated-loop benchmark did not produce a complete contact set")
        }
    }
}

fn benchmark_rational_bezier_self_contact_case(
    runner: &Runner,
    name: &str,
    controls: &[[f64; 2]; 4],
    weights: &[f64; 4],
) {
    if !runner.group_enabled(name) {
        return;
    }

    let policy = CurveContext::STRICT;
    let hypercurve_curve = RationalBezier2::try_new(
        controls
            .iter()
            .map(|point| Point2::new(real(point[0]), real(point[1])))
            .collect(),
        weights.iter().copied().map(real).collect(),
    )
    .expect("valid exact self-contact fixture");
    let curvo_curve = CurvoNurbsCurve2D::<f64>::try_new(
        3,
        controls
            .iter()
            .zip(weights)
            .map(|(point, weight)| Point3::new(point[0] * weight, point[1] * weight, *weight))
            .collect(),
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    )
    .expect("valid finite self-contact fixture");
    let raw_region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[CurvePath2::try_new(vec![
            Curve2::from(hypercurve_curve.clone()),
            Curve2::from(
                LineSeg2::try_new(
                    Point2::new(real(controls[3][0]), real(controls[3][1])),
                    Point2::new(real(controls[0][0]), real(controls[0][1])),
                )
                .expect("self-contact fixture closes with a nondegenerate line"),
            ),
        ])
        .expect("self-contact benchmark path is connected")],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .expect("self-contact benchmark region is valid")
    .into_value();

    // Curvo has no whole-carrier self-contact operation. Give both pairwise
    // solvers the same disjoint outer parameter ranges, which retain the loop
    // crossing while excluding the artificial shared endpoint introduced by
    // a single midpoint split. Fixture construction stays outside timing.
    let lower_cut = real(0.49);
    let upper_cut = real(0.51);
    let Classification::Decided((hypercurve_left, _)) = hypercurve_curve
        .split_at_exact(&lower_cut, &policy)
        .expect("exact lower benchmark split completes")
    else {
        panic!("exact lower benchmark split became uncertain");
    };
    let Classification::Decided((_, hypercurve_right)) = hypercurve_curve
        .split_at_exact(&upper_cut, &policy)
        .expect("exact upper benchmark split completes")
    else {
        panic!("exact upper benchmark split became uncertain");
    };
    let (curvo_left, _) = curvo_curve
        .try_split(0.49)
        .expect("finite lower benchmark split completes");
    let (_, curvo_right) = curvo_curve
        .try_split(0.51)
        .expect("finite upper benchmark split completes");

    assert_eq!(
        exact_rational_bezier_contact_count(
            hypercurve_curve
                .self_intersection_contacts(&policy)
                .expect("exact whole-carrier self contact completes"),
        ),
        1,
    );
    assert_eq!(
        exact_rational_bezier_contact_count(
            hypercurve_left
                .intersection_contacts(&hypercurve_right, &policy)
                .expect("exact disjoint-pair contact completes"),
        ),
        1,
    );
    assert_eq!(
        curvo_left
            .find_intersection(&curvo_right, None)
            .expect("finite disjoint-pair contact completes")
            .len(),
        1,
    );
    assert_eq!(
        raw_region
            .regularized_region(&policy)
            .expect("exact region regularization completes")
            .into_value()
            .boundary_loops()
            .len(),
        2,
    );

    runner.measure(name, "hypercurve_exact_self", || {
        exact_rational_bezier_contact_count(
            hypercurve_curve
                .self_intersection_contacts(black_box(&policy))
                .expect("exact whole-carrier self contact replays"),
        )
    });
    runner.measure(name, "hypercurve_exact_pair", || {
        exact_rational_bezier_contact_count(
            hypercurve_left
                .intersection_contacts(black_box(&hypercurve_right), black_box(&policy))
                .expect("exact disjoint-pair contact replays"),
        )
    });
    runner.measure(name, "hypercurve_exact_region", || {
        raw_region
            .regularized_region(black_box(&policy))
            .expect("exact region regularization replays")
            .into_value()
            .boundary_loops()
            .iter()
            .map(|boundary| boundary.len())
            .sum()
    });
    runner.measure(name, "curvo_f64_pair", || {
        curvo_left
            .find_intersection(black_box(&curvo_right), None)
            .expect("finite disjoint-pair contact replays")
            .len()
    });
}

fn benchmark_rational_bezier_self_contacts(runner: &Runner) {
    let controls = [[9.0, 0.0], [-7.0, 3.0], [-7.0, -10.0], [9.0, 9.0]];
    benchmark_rational_bezier_self_contact_case(
        runner,
        "rational_bezier_self_contact/polynomial_cubic",
        &controls,
        &[1.0; 4],
    );
    benchmark_rational_bezier_self_contact_case(
        runner,
        "rational_bezier_self_contact/projective_cubic",
        &controls,
        &[1.0, 2.0, 4.0, 8.0],
    );
}

fn benchmark_nurbs_evaluation(runner: &Runner) {
    if !runner.group_enabled("nurbs_evaluation/rational_cubic_three_parameters") {
        return;
    }
    let control_points = [[0.0, 0.0], [1.0, 3.0], [3.0, 3.0], [5.0, 3.0], [6.0, 0.0]];
    let weights = [1.0, 2.0, 4.0, 8.0, 16.0];
    let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0];
    let parameters = [0.25, 0.75, 1.5];

    let hypercurve_curve = NurbsCurve2::try_new(
        3,
        control_points
            .iter()
            .map(|point| Point2::new(real(point[0]), real(point[1])))
            .collect(),
        weights.iter().copied().map(real).collect(),
        knots.iter().copied().map(real).collect(),
        &CurveContext::STRICT,
    )
    .expect("valid hypercurve NURBS fixture")
    .into_value();
    let curvo_curve = CurvoNurbsCurve2D::<f64>::try_new(
        3,
        control_points
            .iter()
            .zip(weights)
            .map(|(point, weight)| Point3::new(point[0] * weight, point[1] * weight, weight))
            .collect(),
        knots.to_vec(),
    )
    .expect("valid curvo NURBS fixture");
    let hypercurve_parameters = parameters.map(real);

    for (hypercurve_parameter, curvo_parameter) in
        hypercurve_parameters.iter().zip(parameters.iter().copied())
    {
        let hypercurve_point = hypercurve_curve
            .point_at(hypercurve_parameter, &CurveContext::STRICT)
            .expect("hypercurve NURBS fixture evaluates")
            .into_value();
        let curvo_point = curvo_curve.point_at(curvo_parameter);
        let x = hypercurve_point.x().to_f64_lossy().unwrap();
        let y = hypercurve_point.y().to_f64_lossy().unwrap();
        assert!((x - curvo_point.x).abs() < 1.0e-12);
        assert!((y - curvo_point.y).abs() < 1.0e-12);
    }

    let name = "nurbs_evaluation/rational_cubic_three_parameters";
    let mut hypercurve_index = 0_usize;
    runner.measure(name, "hypercurve", || {
        let index = hypercurve_index % hypercurve_parameters.len();
        hypercurve_index += 1;
        black_box(
            hypercurve_curve
                .point_at(
                    black_box(&hypercurve_parameters[index]),
                    &CurveContext::STRICT,
                )
                .expect("hypercurve NURBS fixture evaluates")
                .into_value(),
        );
        index
    });
    let mut curvo_index = 0_usize;
    runner.measure(name, "curvo", || {
        let index = curvo_index % parameters.len();
        curvo_index += 1;
        black_box(curvo_curve.point_at(black_box(parameters[index])));
        index
    });
}

fn benchmark_nurbs_interpolation(runner: &Runner) {
    let name = "nurbs_interpolation/chord_length_collinear_degree_2";
    if !runner.group_enabled(name) {
        return;
    }

    // The collinear integer fixture gives both implementations the same exact
    // chord parameters. The contracts still differ: Hypercurve certifies the
    // exact solve and residuals; Curvo solves numerically in f64.
    let coordinates = [[0.0, 0.0], [1.0, 0.0], [5.0, 0.0], [14.0, 0.0]];
    let hypercurve_points = coordinates
        .iter()
        .map(|point| Point2::new(real(point[0]), real(point[1])))
        .collect::<Vec<_>>();
    let curvo_points = coordinates
        .iter()
        .map(|point| NalgebraPoint2::new(point[0], point[1]))
        .collect::<Vec<_>>();

    let exact =
        NurbsCurve2::interpolate_chord_length(2, hypercurve_points.clone(), &CurveContext::STRICT)
            .expect("exact chord-length interpolation is certified")
            .into_value();
    let numeric = CurvoNurbsCurve2D::<f64>::interpolate(&curvo_points, 2)
        .expect("finite chord-length interpolation completes");
    assert_eq!(exact.control_points().len(), numeric.control_points().len());

    runner.measure(name, "hypercurve_exact_strict_certified", || {
        black_box(
            NurbsCurve2::interpolate_chord_length(
                2,
                hypercurve_points.clone(),
                &CurveContext::STRICT,
            )
            .expect("exact chord-length interpolation remains certified")
            .into_value(),
        )
        .control_points()
        .len()
    });
    runner.measure(name, "curvo_f64_numeric", || {
        black_box(
            CurvoNurbsCurve2D::<f64>::interpolate(&curvo_points, 2)
                .expect("finite chord-length interpolation remains solvable"),
        )
        .control_points()
        .len()
    });
}

fn benchmark_nurbs_editing(runner: &Runner) {
    let refinement_name = "nurbs_editing/retained_batch_refinement";
    let elevation_name = "nurbs_editing/retained_degree_elevation";
    if !runner.group_enabled(refinement_name) && !runner.group_enabled(elevation_name) {
        return;
    }

    let control_points = [[0.0, 0.0], [1.0, 3.0], [3.0, 3.0], [5.0, 3.0], [6.0, 0.0]];
    let weights = [1.0, 2.0, 4.0, 8.0, 16.0];
    let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0];
    let hypercurve_curve = NurbsCurve2::try_new(
        3,
        control_points
            .iter()
            .map(|point| Point2::new(real(point[0]), real(point[1])))
            .collect(),
        weights.iter().copied().map(real).collect(),
        knots.iter().copied().map(real).collect(),
        &CurveContext::STRICT,
    )
    .expect("valid exact NURBS edit fixture")
    .into_value();
    let curvo_curve = CurvoNurbsCurve2D::<f64>::try_new(
        3,
        control_points
            .iter()
            .zip(weights)
            .map(|(point, weight)| Point3::new(point[0] * weight, point[1] * weight, weight))
            .collect(),
        knots.to_vec(),
    )
    .expect("valid finite NURBS edit fixture");

    if runner.group_enabled(refinement_name) {
        let refined = hypercurve_curve
            .insert_knots(vec![real(0.5), real(1.5)], &CurveContext::STRICT)
            .expect("exact retained refinement fixture is valid")
            .into_value();
        assert_eq!(refined.control_points().len(), 7);
        runner.measure(refinement_name, "hypercurve_exact_retained", || {
            black_box(
                hypercurve_curve
                    .insert_knots(vec![real(0.5), real(1.5)], &CurveContext::STRICT)
                    .expect("exact retained refinement replays")
                    .into_value(),
            )
            .control_points()
            .len()
        });
        runner.measure(refinement_name, "curvo_f64_recomputed", || {
            let mut curve = curvo_curve.clone();
            curve
                .try_refine_knot(vec![0.5, 1.5])
                .expect("finite refinement fixture remains valid");
            black_box(curve).control_points().len()
        });
    }

    if runner.group_enabled(elevation_name) {
        let elevated = hypercurve_curve
            .degree_elevation(6, &CurveContext::STRICT)
            .expect("exact retained elevation fixture is valid")
            .into_value();
        assert_eq!(elevated.target_degree(), 6);
        runner.measure(elevation_name, "hypercurve_exact_retained", || {
            black_box(
                hypercurve_curve
                    .degree_elevation(6, &CurveContext::STRICT)
                    .expect("exact retained elevation replays")
                    .into_value(),
            )
            .spans()
            .len()
        });
        runner.measure(elevation_name, "curvo_f64_recomputed", || {
            black_box(
                curvo_curve
                    .try_elevate_degree(6)
                    .expect("finite degree elevation fixture remains valid"),
            )
            .degree()
        });
    }
}

fn benchmark_pathological_cross_suite(runner: &Runner) {
    if env::var_os("HYPERCURVE_COMPARE_PATHOLOGICAL_TIERS").is_none() {
        println!(
            "pathological cross-suite tiers disabled; set HYPERCURVE_COMPARE_PATHOLOGICAL_TIERS=100mb,500mb,1gb or all"
        );
        return;
    }

    for tier in selected_tiers(
        "HYPERCURVE_COMPARE_PATHOLOGICAL_TIERS",
        &[MemoryTier::Mib100],
    ) {
        let dataset = CrossSuiteDataset::build(tier);
        println!(
            "pathological cross-suite fixture {}: cells={}, allocated neutral coordinates={:.1} MiB",
            tier.name(),
            dataset.cells.len(),
            dataset.allocated_coordinate_bytes as f64 / (1024.0 * 1024.0),
        );
        assert_eq!(dataset.tier, tier);

        for operation in [
            CommonBooleanOp::Union,
            CommonBooleanOp::Intersection,
            CommonBooleanOp::Difference,
            CommonBooleanOp::Xor,
        ] {
            let operation_name = match operation {
                CommonBooleanOp::Union => "union",
                CommonBooleanOp::Intersection => "intersection",
                CommonBooleanOp::Difference => "difference",
                CommonBooleanOp::Xor => "xor",
            };
            let group = format!("pathological_{}/{operation_name}", tier.name());

            let hypercurve_cells = dataset
                .cells
                .iter()
                .map(|cell| {
                    (
                        hypercurve_region(&cell.source),
                        hypercurve_region(&cell.rotated),
                    )
                })
                .collect::<Vec<_>>();
            let policy = CurveContext::STRICT;
            runner.measure(&group, "hypercurve_flattened", || {
                hypercurve_cells
                    .iter()
                    .map(|(source, rotated)| {
                        hypercurve_boolean_result_size(source, rotated, operation, &policy)
                    })
                    .sum()
            });
            drop(hypercurve_cells);

            let cavalier_cells = dataset
                .cells
                .iter()
                .map(|cell| {
                    (
                        cavalier_polyline(&cell.source, None),
                        cavalier_polyline(&cell.rotated, None),
                    )
                })
                .collect::<Vec<_>>();
            runner.measure(&group, "cavalier_contours", || {
                cavalier_cells
                    .iter()
                    .map(|(source, rotated)| {
                        cavalier_boolean_result_size(source, rotated, operation)
                    })
                    .sum()
            });
            drop(cavalier_cells);

            runner.measure(&group, "i_overlay", || {
                dataset
                    .cells
                    .iter()
                    .map(|cell| {
                        ioverlay_boolean_result_size(&cell.source, &cell.rotated, operation)
                    })
                    .sum()
            });

            let geo_cells = dataset
                .cells
                .iter()
                .map(|cell| (geo_polygon(&cell.source), geo_polygon(&cell.rotated)))
                .collect::<Vec<_>>();
            runner.measure(&group, "geo", || {
                geo_cells
                    .iter()
                    .map(|(source, rotated)| geo_boolean_result_size(source, rotated, operation))
                    .sum()
            });
        }
    }
}

fn main() {
    let runner = Runner::from_environment();
    println!(
        "hypercurve comparative benchmarks: samples={}, target/sample={:?}, fixed iterations={:?}, group filter={:?}, implementation filter={:?}",
        runner.samples,
        runner.sample_target,
        runner.fixed_iterations,
        runner.group_filter,
        runner.implementation_filter,
    );
    println!(
        "timed operations exclude fixture construction; results use each crate's native numeric and topology model"
    );
    benchmark_polygon_booleans(&runner);
    benchmark_line_arc_boolean(&runner);
    benchmark_contour_offset(&runner);
    #[cfg(feature = "predicates")]
    benchmark_algebraic_round_offset(&runner);
    benchmark_orthogonal_neck_split(&runner);
    benchmark_bezier_offset(&runner);
    benchmark_rational_bezier_self_contacts(&runner);
    benchmark_nurbs_evaluation(&runner);
    benchmark_nurbs_interpolation(&runner);
    benchmark_nurbs_editing(&runner);
    benchmark_pathological_cross_suite(&runner);
}
