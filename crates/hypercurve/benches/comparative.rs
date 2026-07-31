#[path = "common/pathological.rs"]
mod pathological_fixture;

use std::env;
use std::hint::black_box;
use std::time::{Duration, Instant};

use cavalier_contours::polyline::{
    BooleanOp as CavalierBooleanOp, PlineSource, PlineSourceMut, Polyline,
};
use curvo::prelude::{
    CurveOffsetOption as CurvoCurveOffsetOption, NurbsCurve2D as CurvoNurbsCurve2D,
    Offset as CurvoOffset,
};
use geo::{BooleanOps as _, Coord, LineString, Polygon};
use hypercurve::{
    BezierFlatteningOptions, BezierParallelVerificationOptions, BooleanOp, BulgeVertex2,
    Classification, Contour2, CubicBezier2, Curve2, CurveContext, CurveString2, FillRule,
    LineArcRegion2, LineSeg2, NurbsCurve2, Point2, Real, Segment2,
};
use i_overlay::core::fill_rule::FillRule as OverlayFillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;
use nalgebra::Point3;

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
            .point_at(hypercurve_parameter)
            .expect("hypercurve NURBS fixture evaluates");
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
                .point_at(black_box(&hypercurve_parameters[index]))
                .expect("hypercurve NURBS fixture evaluates"),
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
    benchmark_contour_offset(&runner);
    benchmark_bezier_offset(&runner);
    benchmark_nurbs_evaluation(&runner);
    benchmark_pathological_cross_suite(&runner);
}
