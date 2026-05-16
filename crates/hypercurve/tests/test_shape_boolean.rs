//! Shape-level boolean regression, property, diagnostic, and manual differential tests.
//!
//! The shape boolean layer delegates clipping to the polyline boolean algorithm, then rebuilds
//! signed `Shape` bins from pairwise CCW/CW loop results. These tests therefore assert both
//! geometry semantics and assembly invariants: valid signed loop orientation, fresh spatial
//! indexes, no duplicate loops, set-operation membership at sampled points, and debuggable failure
//! artifacts for cases where exact topology is not stable enough to be the only oracle.

mod test_utils;

use cavalier_contours::core::{math::Vector2, traits::FuzzyEq};
use cavalier_contours::polyline::{
    BooleanOp, BooleanResultInfo, PlineInversionView, PlineOrientation, PlineSource,
    PlineSourceMut, Polyline, seg_arc_radius_and_center, seg_closest_point,
};
use cavalier_contours::shape_algorithms::{Shape, ShapeOffsetOptions, ShapeView};
use cavalier_contours::{assert_fuzzy_eq, pline_closed, pline_open};
use std::f64::consts::PI;
use std::{env, fmt::Write as _, fs, path::Path};
use test_utils::to_debug_json_str;

/// Test-side tolerance for validating reconstructed shape loops and indexes.
///
/// This is intentionally a little larger than exact floating-point noise because these assertions
/// probe results after arc conversion, pairwise clipping, loop merging, and index rebuilds.
const SHAPE_TEST_EPS: f64 = 1e-7;

type LineSegmentCoords = (f64, f64, f64, f64);
type ClippedLineSegments = (Vec<LineSegmentCoords>, Vec<LineSegmentCoords>);

/// A small helper to verify bounding boxes.
fn assert_extents_fuzzy_eq(
    shape: &Shape<f64>,
    expected_min_x: f64,
    expected_min_y: f64,
    expected_max_x: f64,
    expected_max_y: f64,
    eps: f64,
) {
    let aabb = shape.plines_index.bounds().unwrap();
    let actual_min_x = aabb.min_x;
    let actual_min_y = aabb.min_y;
    let actual_max_x = aabb.max_x;
    let actual_max_y = aabb.max_y;
    assert!(
        actual_min_x.fuzzy_eq_eps(expected_min_x, eps),
        "min_x => expected: {expected_min_x}, got: {actual_min_x}"
    );
    assert!(
        actual_min_y.fuzzy_eq_eps(expected_min_y, eps),
        "min_y => expected: {expected_min_y}, got: {actual_min_y}"
    );
    assert!(
        actual_max_x.fuzzy_eq_eps(expected_max_x, eps),
        "max_x => expected: {expected_max_x}, got: {actual_max_x}"
    );
    assert!(
        actual_max_y.fuzzy_eq_eps(expected_max_y, eps),
        "max_y => expected: {expected_max_y}, got: {actual_max_y}"
    );
}

/// Create a simple rectangle polyline oriented along the XY axes.
fn create_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pl = Polyline::new_closed();
    pl.add(xmin, ymin, 0.0);
    pl.add(xmax, ymin, 0.0);
    pl.add(xmax, ymax, 0.0);
    pl.add(xmin, ymax, 0.0);
    pl
}

/// Create a circle-like shape from a small polyline (approx. circle).
fn create_approx_circle(center_x: f64, center_y: f64, radius: f64) -> Polyline<f64> {
    // Two vertices with bulge 1.0 encode a full circle as two half-circle arc segments.
    let mut pl = Polyline::new_closed();
    pl.add(center_x - radius, center_y, 1.0);
    pl.add(center_x + radius, center_y, 1.0);
    pl
}

fn create_line_segment(x1: f64, y1: f64, x2: f64, y2: f64) -> Polyline<f64> {
    pline_open![(x1, y1, 0.0), (x2, y2, 0.0)]
}

fn create_cw_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pl = create_rectangle(xmin, ymin, xmax, ymax);
    pl.invert_direction_mut();
    pl
}

/// Build a multi-island shape from axis-aligned rectangles.
///
/// Rectangles keep expected areas easy to compute, which makes them useful for deterministic
/// algebraic identities and property tests.
fn create_shape_rects(rects: &[(f64, f64, f64, f64)]) -> Shape<f64> {
    Shape::from_plines(
        rects
            .iter()
            .map(|&(xmin, ymin, xmax, ymax)| create_rectangle(xmin, ymin, xmax, ymax)),
    )
}

/// Build a rectangular donut whose outer loop is material and whose inner loops are holes.
///
/// Hole-heavy cases are the main stressor for shape boolean because CW loops are inverted before
/// they are passed to the lower-level positive-area polyline boolean.
fn create_donut(outer: (f64, f64, f64, f64), holes: &[(f64, f64, f64, f64)]) -> Shape<f64> {
    let mut plines = vec![create_rectangle(outer.0, outer.1, outer.2, outer.3)];
    plines.extend(
        holes
            .iter()
            .map(|&(xmin, ymin, xmax, ymax)| create_cw_rectangle(xmin, ymin, xmax, ymax)),
    );
    Shape::from_plines(plines)
}

fn create_deep_nested_rect_shape() -> Shape<f64> {
    Shape::from_plines([
        create_rectangle(0.0, 0.0, 100.0, 100.0),
        create_cw_rectangle(10.0, 10.0, 90.0, 90.0),
        create_rectangle(20.0, 20.0, 80.0, 80.0),
        create_cw_rectangle(30.0, 30.0, 70.0, 70.0),
        create_rectangle(40.0, 40.0, 60.0, 60.0),
    ])
}

fn create_deeper_nested_rect_shape(
    origin_x: f64,
    origin_y: f64,
    size: f64,
    depth: usize,
) -> Shape<f64> {
    let inset_step = size / (2.0 * depth as f64 + 2.0);
    Shape::from_plines((0..depth).map(|i| {
        let inset = inset_step * i as f64;
        if i % 2 == 0 {
            create_rectangle(
                origin_x + inset,
                origin_y + inset,
                origin_x + size - inset,
                origin_y + size - inset,
            )
        } else {
            create_cw_rectangle(
                origin_x + inset,
                origin_y + inset,
                origin_x + size - inset,
                origin_y + size - inset,
            )
        }
    }))
}

fn create_arc_ring(
    center_x: f64,
    center_y: f64,
    outer_radius: f64,
    inner_radius: f64,
) -> Shape<f64> {
    let outer = create_approx_circle(center_x, center_y, outer_radius);
    let mut inner = create_approx_circle(center_x, center_y, inner_radius);
    inner.invert_direction_mut();
    Shape::from_plines([outer, inner])
}

fn create_sawtooth_loop(
    min_x: f64,
    min_y: f64,
    width: f64,
    height: f64,
    teeth: usize,
) -> Polyline<f64> {
    let mut pline = Polyline::new_closed();
    pline.add(min_x, min_y, 0.0);
    for i in 0..teeth {
        let x0 = min_x + width * (i as f64 + 0.25) / teeth as f64;
        let x1 = min_x + width * (i as f64 + 0.50) / teeth as f64;
        let x2 = min_x + width * (i as f64 + 0.75) / teeth as f64;
        pline.add(x0, min_y + height * 0.88, 0.0);
        pline.add(x1, min_y + height, 0.0);
        pline.add(x2, min_y + height * 0.88, 0.0);
    }
    pline.add(min_x + width, min_y, 0.0);
    pline
}

fn create_multi_pline_boolean_scene_default_shape() -> Shape<f64> {
    Shape::from_plines(vec![
        pline_closed![
            (100.0, 100.0, -0.5),
            (80.0, 90.0, 0.374794619217547),
            (210.0, 0.0, 0.0),
            (230.0, 0.0, 1.0),
            (320.0, 0.0, -0.5),
            (280.0, 0.0, 0.5),
            (390.0, 210.0, 0.0),
            (280.0, 120.0, 0.5),
        ],
        pline_closed![
            (150.0, 50.0, 0.0),
            (150.0, 100.0, 0.0),
            (223.74732137849435, 142.16931273980475, 0.0),
            (199.491310072685, 52.51543504258919, 0.5),
        ],
        pline_closed![
            (261.11232783167395, 35.79686193615828, -1.0),
            (250.0, 100.0, -1.0),
        ],
        pline_closed![
            (320.2986109239592, 103.52378781211337, 0.0),
            (320.5065990423979, 76.14222955572362, -1.0),
        ],
        pline_closed![
            (273.6131273938006, -13.968608715397636, -0.3),
            (256.61336060995995, -25.49387433156079, 0.0),
            (249.69820124026208, 27.234215862385582, 0.0),
        ],
    ])
}

/// Rotate a closed loop's start vertex without changing its geometry.
///
/// This catches accidental dependence on storage order in shape identity checks and pairwise
/// bookkeeping.
fn rotate_pline_start(pline: &Polyline<f64>, start_index: usize) -> Polyline<f64> {
    pline
        .rotate_start(start_index, pline.at(start_index).pos(), SHAPE_TEST_EPS)
        .expect("expected valid closed polyline rotation")
}

/// Sum shape area using the same signed-loop convention as `Shape`.
fn shape_signed_area(shape: &Shape<f64>) -> f64 {
    // Shape uses signed loop bins: CCW contributes positive area, CW contributes negative holes.
    shape
        .ccw_plines
        .iter()
        .map(|ip| ip.polyline.area())
        .chain(shape.cw_plines.iter().map(|ip| ip.polyline.area()))
        .sum()
}

/// Compute signed-bin material depth at a point.
fn shape_material_depth(shape: &Shape<f64>, point: Vector2<f64>) -> isize {
    // CCW bins are material cells and CW bins are subtractive cells. A self-crossing material loop
    // may have negative local winding in part of its filled area, so only the bin sign contributes
    // to shape-level membership.
    shape
        .ccw_plines
        .iter()
        .filter(|ip| ip.polyline.winding_number(point) != 0)
        .count() as isize
        - shape
            .cw_plines
            .iter()
            .filter(|ip| ip.polyline.winding_number(point) != 0)
            .count() as isize
}

/// Test membership helper used by sampled boolean oracles.
fn shape_contains(shape: &Shape<f64>, x: f64, y: f64) -> bool {
    shape_material_depth(shape, Vector2::new(x, y)) > 0
}

fn point_on_closed_shape_boundary(shape: &Shape<f64>, x: f64, y: f64) -> bool {
    let point = Vector2::new(x, y);
    shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .filter(|ip| ip.polyline.is_closed())
        .any(|ip| {
            ip.polyline.iter_segments().any(|(v1, v2)| {
                seg_closest_point(v1, v2, point, SHAPE_TEST_EPS).fuzzy_eq_eps(point, SHAPE_TEST_EPS)
            })
        })
}

fn shape_sample_windings(shape: &Shape<f64>, sample: (f64, f64)) -> Vec<(&'static str, i32)> {
    let point = Vector2::new(sample.0, sample.1);
    shape
        .ccw_plines
        .iter()
        .map(|ip| ("ccw", ip.polyline.winding_number(point)))
        .chain(
            shape
                .cw_plines
                .iter()
                .map(|ip| ("cw", ip.polyline.winding_number(point))),
        )
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct LineSegSignature {
    x1: i64,
    y1: i64,
    x2: i64,
    y2: i64,
}

fn line_seg_signature(a: Vector2<f64>, b: Vector2<f64>) -> LineSegSignature {
    let p1 = (sig_num(a.x), sig_num(a.y));
    let p2 = (sig_num(b.x), sig_num(b.y));
    let (p1, p2) = if p1 <= p2 { (p1, p2) } else { (p2, p1) };
    LineSegSignature {
        x1: p1.0,
        y1: p1.1,
        x2: p2.0,
        y2: p2.1,
    }
}

fn collect_open_line_segments(shape: &Shape<f64>) -> Vec<LineSegSignature> {
    let mut result = shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .filter(|ip| !ip.polyline.is_closed())
        .flat_map(|ip| {
            ip.polyline
                .iter_segments()
                .filter(|(v1, v2)| {
                    v1.bulge.fuzzy_eq_eps(0.0, SHAPE_TEST_EPS)
                        && !v1.pos().fuzzy_eq_eps(v2.pos(), SHAPE_TEST_EPS)
                })
                .map(|(v1, v2)| line_seg_signature(v1.pos(), v2.pos()))
        })
        .collect::<Vec<_>>();
    result.sort();
    result
}

fn assert_open_line_segments(shape: &Shape<f64>, expected: &[(f64, f64, f64, f64)]) {
    let mut expected = expected
        .iter()
        .map(|&(x1, y1, x2, y2)| line_seg_signature(Vector2::new(x1, y1), Vector2::new(x2, y2)))
        .collect::<Vec<_>>();
    expected.sort();

    let actual = collect_open_line_segments(shape);
    assert_eq!(actual, expected, "open line segment mismatch");
}

fn assert_open_linework_outside_closed_material(shape: &Shape<f64>, context: &str) {
    let closed_area = Shape::from_plines(
        shape
            .ccw_plines
            .iter()
            .chain(shape.cw_plines.iter())
            .filter(|ip| ip.polyline.is_closed())
            .map(|ip| ip.polyline.clone()),
    );

    for ip in shape.ccw_plines.iter().chain(shape.cw_plines.iter()) {
        if ip.polyline.is_closed() {
            continue;
        }

        for (v1, v2) in ip.polyline.iter_segments() {
            for fraction in [0.25, 0.5, 0.75] {
                let x = v1.x + (v2.x - v1.x) * fraction;
                let y = v1.y + (v2.y - v1.y) * fraction;
                assert!(
                    !shape_contains(&closed_area, x, y),
                    "{context}: open linework sample ({x}, {y}) is inside closed material"
                );
            }
        }
    }
}

fn clipped_line_segments_against_unit_square(line: LineSegmentCoords) -> ClippedLineSegments {
    let (x1, y1, x2, y2) = line;
    let dx = x2 - x1;
    let dy = y2 - y1;
    let mut t0: f64 = 0.0;
    let mut t1: f64 = 1.0;

    let mut clip = |p: f64, q: f64| -> bool {
        if p.fuzzy_eq_eps(0.0, SHAPE_TEST_EPS) {
            return q >= -SHAPE_TEST_EPS;
        }

        let r = q / p;
        if p < 0.0 {
            if r > t1 {
                return false;
            }
            t0 = t0.max(r);
        } else {
            if r < t0 {
                return false;
            }
            t1 = t1.min(r);
        }

        true
    };

    let intersects = clip(-dx, x1) && clip(dx, 10.0 - x1) && clip(-dy, y1) && clip(dy, 10.0 - y1);

    let point_at = |t: f64| (x1 + dx * t, y1 + dy * t);
    let segment = |a: f64, b: f64| {
        let (sx, sy) = point_at(a);
        let (ex, ey) = point_at(b);
        (sx, sy, ex, ey)
    };
    let non_degenerate = |a: f64, b: f64| (b - a).abs() > SHAPE_TEST_EPS;

    if !intersects || !non_degenerate(t0, t1) {
        return (Vec::new(), vec![line]);
    }

    let mut inside = Vec::new();
    let mut outside = Vec::new();

    inside.push(segment(t0, t1));
    if non_degenerate(0.0, t0) {
        outside.push(segment(0.0, t0));
    }
    if non_degenerate(t1, 1.0) {
        outside.push(segment(t1, 1.0));
    }

    (inside, outside)
}

/// Quantized per-loop summary used in diagnostics and duplicate-loop checks.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct LoopSignature {
    orientation: &'static str,
    vertex_count: usize,
    area: i64,
    path_length: i64,
    extents: Option<(i64, i64, i64, i64)>,
}

/// Quantized shape summary that is stable enough to print in failure dumps.
#[derive(Debug, Clone, PartialEq, Eq)]
struct ShapeSignature {
    ccw_count: usize,
    cw_count: usize,
    signed_area: i64,
    perimeter_sum: i64,
    extents: Option<(i64, i64, i64, i64)>,
    loops: Vec<LoopSignature>,
}

/// Quantize floating-point diagnostics without requiring exact vertex identity.
fn sig_num(value: f64) -> i64 {
    (value * 1_000_000_000.0).round() as i64
}

/// Convert extents into the same quantized representation used by shape signatures.
fn extents_signature(
    extents: Option<static_aabb2d_index::AABB<f64>>,
) -> Option<(i64, i64, i64, i64)> {
    extents.map(|e| {
        (
            sig_num(e.min_x),
            sig_num(e.min_y),
            sig_num(e.max_x),
            sig_num(e.max_y),
        )
    })
}

/// Capture the loop-level fields that matter when debugging shape assembly.
fn loop_signature(pline: &Polyline<f64>) -> LoopSignature {
    LoopSignature {
        orientation: match pline.orientation() {
            PlineOrientation::Open => "open",
            PlineOrientation::CounterClockwise => "ccw",
            PlineOrientation::Clockwise => "cw",
        },
        vertex_count: pline.vertex_count(),
        area: sig_num(pline.area()),
        path_length: sig_num(pline.path_length()),
        extents: extents_signature(pline.extents()),
    }
}

/// Build a deterministic diagnostic signature independent of loop storage order.
fn shape_signature(shape: &Shape<f64>) -> ShapeSignature {
    let mut loops = shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .map(|ip| loop_signature(&ip.polyline))
        .collect::<Vec<_>>();
    loops.sort();

    ShapeSignature {
        ccw_count: shape.ccw_plines.len(),
        cw_count: shape.cw_plines.len(),
        signed_area: sig_num(shape_signed_area(shape)),
        perimeter_sum: loops.iter().map(|l| l.path_length).sum(),
        extents: extents_signature(shape.plines_index.bounds()),
        loops,
    }
}

/// Reject duplicate coincident loops, which often indicate pairwise result over-retention.
fn assert_no_duplicate_loops(shape: &Shape<f64>) {
    let mut loops = shape_signature(shape).loops;
    loops.sort();
    for pair in loops.windows(2) {
        let signature = shape_signature(shape);
        assert_ne!(
            pair[0], pair[1],
            "shape contains duplicate coincident loops: {signature:?}"
        );
    }
}

/// Validate one output loop before it can be reused as a shape input.
fn assert_loop_valid(pline: &Polyline<f64>, expected_orientation: PlineOrientation) {
    assert!(pline.is_closed(), "shape loops must be closed: {pline:?}");
    assert_eq!(
        pline.orientation(),
        expected_orientation,
        "loop orientation is in the wrong shape bin: {pline:?}"
    );
    assert!(
        pline.remove_repeat_pos(SHAPE_TEST_EPS).is_none(),
        "shape loop has repeat-position vertices: {pline:?}"
    );

    for v in pline.iter_vertexes() {
        assert!(v.x.is_finite(), "non-finite x coordinate in {pline:?}");
        assert!(v.y.is_finite(), "non-finite y coordinate in {pline:?}");
        assert!(v.bulge.is_finite(), "non-finite bulge in {pline:?}");
    }
}

/// Validate shape invariants that boolean output must preserve.
///
/// The checks go beyond sampled membership because stale indexes, wrong signed bins, and duplicate
/// loops can pass point samples while still breaking later shape operations.
fn assert_shape_valid(shape: &Shape<f64>) {
    // Every boolean result should be immediately reusable as a Shape input, so verify the
    // orientation bins and spatial indexes instead of checking only area or membership.
    for ip in &shape.ccw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::CounterClockwise);
        let pline_extents = ip.polyline.extents().unwrap();
        let index_bounds = ip.spatial_index.bounds().unwrap();
        assert!(
            pline_extents
                .min_x
                .fuzzy_eq_eps(index_bounds.min_x, SHAPE_TEST_EPS)
                && pline_extents
                    .min_y
                    .fuzzy_eq_eps(index_bounds.min_y, SHAPE_TEST_EPS)
                && pline_extents
                    .max_x
                    .fuzzy_eq_eps(index_bounds.max_x, SHAPE_TEST_EPS)
                && pline_extents
                    .max_y
                    .fuzzy_eq_eps(index_bounds.max_y, SHAPE_TEST_EPS),
            "loop spatial index bounds do not match polyline extents"
        );
    }

    for ip in &shape.cw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::Clockwise);
        let pline_extents = ip.polyline.extents().unwrap();
        let index_bounds = ip.spatial_index.bounds().unwrap();
        assert!(
            pline_extents
                .min_x
                .fuzzy_eq_eps(index_bounds.min_x, SHAPE_TEST_EPS)
                && pline_extents
                    .min_y
                    .fuzzy_eq_eps(index_bounds.min_y, SHAPE_TEST_EPS)
                && pline_extents
                    .max_x
                    .fuzzy_eq_eps(index_bounds.max_x, SHAPE_TEST_EPS)
                && pline_extents
                    .max_y
                    .fuzzy_eq_eps(index_bounds.max_y, SHAPE_TEST_EPS),
            "loop spatial index bounds do not match polyline extents"
        );
    }

    let loop_count = shape.ccw_plines.len() + shape.cw_plines.len();
    if loop_count == 0 {
        assert!(
            shape.plines_index.bounds().is_none(),
            "empty shape should not have top-level bounds"
        );
        return;
    }

    let mut expected: Option<static_aabb2d_index::AABB<f64>> = None;
    for bounds in shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .map(|ip| ip.spatial_index.bounds().unwrap())
    {
        expected = Some(match expected {
            None => bounds,
            Some(curr) => static_aabb2d_index::AABB::new(
                curr.min_x.min(bounds.min_x),
                curr.min_y.min(bounds.min_y),
                curr.max_x.max(bounds.max_x),
                curr.max_y.max(bounds.max_y),
            ),
        });
    }

    let expected = expected.unwrap();
    let actual = shape.plines_index.bounds().unwrap();
    assert!(
        expected.min_x.fuzzy_eq_eps(actual.min_x, SHAPE_TEST_EPS)
            && expected.min_y.fuzzy_eq_eps(actual.min_y, SHAPE_TEST_EPS)
            && expected.max_x.fuzzy_eq_eps(actual.max_x, SHAPE_TEST_EPS)
            && expected.max_y.fuzzy_eq_eps(actual.max_y, SHAPE_TEST_EPS),
        "shape top-level index bounds do not match child loop bounds"
    );

    assert_no_duplicate_loops(shape);
}

/// Add samples near vertices and segment midpoints.
///
/// Boolean bugs often hide near cut points, tangencies, and hole boundaries, so the semantic
/// oracle deliberately samples around geometry features instead of only using a coarse grid.
fn push_shape_probe_samples(shape: &Shape<f64>, samples: &mut Vec<(f64, f64)>) {
    let nudges = [
        (SHAPE_TEST_EPS * 1000.0, SHAPE_TEST_EPS * 1000.0),
        (-SHAPE_TEST_EPS * 1000.0, SHAPE_TEST_EPS * 1000.0),
        (SHAPE_TEST_EPS * 1000.0, -SHAPE_TEST_EPS * 1000.0),
        (-SHAPE_TEST_EPS * 1000.0, -SHAPE_TEST_EPS * 1000.0),
    ];

    for pline in shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .map(|ip| &ip.polyline)
    {
        for v in pline.iter_vertexes() {
            for (dx, dy) in nudges {
                samples.push((v.x + dx, v.y + dy));
            }
        }

        for (v1, v2) in pline.iter_segments() {
            let midpoint = cavalier_contours::polyline::seg_midpoint(v1, v2);
            for (dx, dy) in nudges {
                samples.push((midpoint.x + dx, midpoint.y + dy));
            }
        }
    }
}

/// Compute bounds for a group of shapes so grid samples cover all inputs and the result.
fn combined_extents(shapes: &[&Shape<f64>]) -> Option<static_aabb2d_index::AABB<f64>> {
    shapes
        .iter()
        .filter_map(|shape| shape.plines_index.bounds())
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => bounds,
                Some(curr) => static_aabb2d_index::AABB::new(
                    curr.min_x.min(bounds.min_x),
                    curr.min_y.min(bounds.min_y),
                    curr.max_x.max(bounds.max_x),
                    curr.max_y.max(bounds.max_y),
                ),
            })
        })
}

/// Build the deterministic point set used by the membership oracle.
///
/// Exact output topology can legitimately vary after regularization, but points away from
/// boundaries must obey the requested set operation.
fn semantic_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    explicit: &[(f64, f64)],
) -> Vec<(f64, f64)> {
    let mut samples = explicit.to_vec();

    if let Some(extents) = combined_extents(&[a, b, result]) {
        let width = (extents.max_x - extents.min_x).max(1.0);
        let height = (extents.max_y - extents.min_y).max(1.0);
        // Fractions are intentionally irregular so axis-aligned grid samples are unlikely to land
        // exactly on rectangle, hole, or result boundaries, where winding-number semantics are
        // implementation-defined.
        let fractions = [0.113, 0.271, 0.389, 0.541, 0.673, 0.809, 0.947];
        for fx in fractions {
            for fy in fractions {
                samples.push((extents.min_x + width * fx, extents.min_y + height * fy));
            }
        }
        samples.push((extents.min_x - width * 0.1, extents.min_y - height * 0.1));
        samples.push((extents.max_x + width * 0.1, extents.max_y + height * 0.1));
    }

    push_shape_probe_samples(a, &mut samples);
    push_shape_probe_samples(b, &mut samples);
    samples
}

fn dense_sample_grid(min: f64, max: f64, step: f64) -> Vec<(f64, f64)> {
    let count = ((max - min) / step).ceil() as usize;
    (0..=count)
        .flat_map(|ix| {
            (0..=count).map(move |iy| {
                (
                    min + ix as f64 * step + step * 0.37,
                    min + iy as f64 * step + step * 0.61,
                )
            })
        })
        .collect()
}

/// Convert a polyline to an SVG path while preserving arc segments.
fn pline_svg_path(pline: &Polyline<f64>) -> Option<String> {
    if pline.vertex_count() == 0 {
        return None;
    }

    // Failure SVGs use path commands rather than polygons so arc-heavy regressions preserve the
    // same bulge geometry that the boolean algorithm processed.
    let first = pline.at(0);
    let first_x = first.x;
    let first_y = first.y;
    let mut path = format!("M {first_x} {first_y}");
    let segment_count = if pline.is_closed() {
        pline.vertex_count()
    } else {
        pline.vertex_count().saturating_sub(1)
    };

    for i in 0..segment_count {
        let start = pline.at(i);
        let end = pline.at((i + 1) % pline.vertex_count());
        let end_x = end.x;
        let end_y = end.y;
        if start.bulge.fuzzy_eq_eps(0.0, SHAPE_TEST_EPS) {
            path.push_str(&format!(" L {end_x} {end_y}"));
        } else {
            let (radius, _) = seg_arc_radius_and_center(start, end);
            let sweep_angle = 4.0 * start.bulge.atan();
            let large_arc = i32::from(sweep_angle.abs() > PI);
            let sweep = i32::from(start.bulge > 0.0);
            path.push_str(&format!(
                " A {radius} {radius} 0 {large_arc} {sweep} {end_x} {end_y}"
            ));
        }
    }

    if pline.is_closed() {
        path.push_str(" Z");
    }

    Some(path)
}

/// Write an SVG overlay for failure inspection.
fn write_shape_svg(path: &Path, shapes: &[(&Shape<f64>, &'static str)]) -> std::io::Result<()> {
    write_shape_svg_with_sample(path, shapes, None)
}

/// Write an SVG overlay and optionally mark the sampled point that failed membership.
fn write_shape_svg_with_sample(
    path: &Path,
    shapes: &[(&Shape<f64>, &'static str)],
    sample: Option<(f64, f64)>,
) -> std::io::Result<()> {
    let mut extents = combined_extents(
        &shapes
            .iter()
            .map(|(shape, _color)| *shape)
            .collect::<Vec<_>>(),
    );

    if let Some((x, y)) = sample {
        extents = Some(match extents {
            None => static_aabb2d_index::AABB::new(x, y, x, y),
            Some(curr) => static_aabb2d_index::AABB::new(
                curr.min_x.min(x),
                curr.min_y.min(y),
                curr.max_x.max(x),
                curr.max_y.max(y),
            ),
        });
    }

    let (min_x, min_y, width, height) = if let Some(e) = extents {
        let width = (e.max_x - e.min_x).max(1.0);
        let height = (e.max_y - e.min_y).max(1.0);
        (
            e.min_x - width * 0.05,
            e.min_y - height * 0.05,
            width * 1.1,
            height * 1.1,
        )
    } else {
        (-1.0, -1.0, 2.0, 2.0)
    };

    let mut svg = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{min_x} {min_y} {width} {height}">
<g fill="none" stroke-width="0.05">
"#
    );

    for (shape, color) in shapes {
        for pline in shape
            .ccw_plines
            .iter()
            .chain(shape.cw_plines.iter())
            .map(|ip| &ip.polyline)
        {
            let Some(path) = pline_svg_path(pline) else {
                continue;
            };
            svg.push_str(&format!(
                r#"<path d="{path}" stroke="{color}" fill="{color}" fill-opacity="0.08" />
"#
            ));
        }
    }

    if let Some((x, y)) = sample {
        let radius = width.min(height).max(1.0) * 0.01;
        svg.push_str(&format!(
            r#"<circle cx="{x}" cy="{y}" r="{radius}" stroke="black" fill="yellow" fill-opacity="0.85" />
"#
        ));
    }

    svg.push_str("</g>\n</svg>\n");
    fs::write(path, svg)
}

/// Serialize the input and output loops in the same debug JSON style as existing tests.
fn shape_debug_json(shape: &Shape<f64>) -> String {
    let mut result = String::new();
    result.push_str("{\n  \"ccw\": [\n");
    for (i, ip) in shape.ccw_plines.iter().enumerate() {
        if i > 0 {
            result.push_str(",\n");
        }
        result.push_str(&to_debug_json_str(&ip.polyline));
    }
    result.push_str("\n  ],\n  \"cw\": [\n");
    for (i, ip) in shape.cw_plines.iter().enumerate() {
        if i > 0 {
            result.push_str(",\n");
        }
        result.push_str(&to_debug_json_str(&ip.polyline));
    }
    result.push_str("\n  ]\n}\n");
    result
}

/// Mirror the shape-level broad-phase overlap predicate used by the implementation.
fn aabb_overlap(a: static_aabb2d_index::AABB<f64>, b: static_aabb2d_index::AABB<f64>) -> bool {
    a.min_x <= b.max_x && a.max_x >= b.min_x && a.min_y <= b.max_y && a.max_y >= b.min_y
}

/// Append one pairwise lower-level boolean row to a trace dump.
fn write_pair_trace(
    trace: &mut String,
    label: &str,
    pair: (usize, usize),
    candidate: bool,
    result: Option<(BooleanResultInfo, usize, usize)>,
) {
    let (i, j) = pair;
    if let Some((result_info, pos_count, neg_count)) = result {
        let _ = writeln!(
            trace,
            "{label}[{i},{j}] candidate={candidate} result_info={result_info:?} pos={pos_count} neg={neg_count}"
        );
    } else {
        let _ = writeln!(trace, "{label}[{i},{j}] candidate={candidate}");
    }
}

/// Reconstruct the pairwise decisions in a human-readable trace.
///
/// This is test-side instrumentation rather than production logging so a failing case can show
/// which signed-loop pairs were broad-phase candidates, what the lower-level polyline boolean
/// returned, and which loops would be considered used.
fn shape_pairwise_trace(a: &Shape<f64>, b: &Shape<f64>, op: BooleanOp) -> String {
    let mut trace = String::new();
    let _ = writeln!(trace, "pairwise lower-level trace for {op:?}");

    let mut a_ccw_used = vec![false; a.ccw_plines.len()];
    let mut a_cw_used = vec![false; a.cw_plines.len()];
    let mut b_ccw_used = vec![false; b.ccw_plines.len()];
    let mut b_cw_used = vec![false; b.cw_plines.len()];

    for (i, ip) in a.ccw_plines.iter().enumerate() {
        for (j, jp) in b.ccw_plines.iter().enumerate() {
            let candidate = ip
                .spatial_index
                .bounds()
                .zip(jp.spatial_index.bounds())
                .is_some_and(|(ib, jb)| aabb_overlap(ib, jb));
            if !candidate {
                write_pair_trace(&mut trace, "ccw-ccw", (i, j), false, None);
                continue;
            }

            let result = ip.polyline.boolean(&jp.polyline, op);
            let active = !matches!(result.result_info, BooleanResultInfo::Disjoint);
            a_ccw_used[i] |= active;
            b_ccw_used[j] |= active;
            write_pair_trace(
                &mut trace,
                "ccw-ccw",
                (i, j),
                true,
                Some((
                    result.result_info,
                    result.pos_plines.len(),
                    result.neg_plines.len(),
                )),
            );
        }
    }

    for (i, ip) in a.ccw_plines.iter().enumerate() {
        for (j, jp) in b.cw_plines.iter().enumerate() {
            let candidate = ip
                .spatial_index
                .bounds()
                .zip(jp.spatial_index.bounds())
                .is_some_and(|(ib, jb)| aabb_overlap(ib, jb));
            if !candidate {
                write_pair_trace(&mut trace, "ccw-cw", (i, j), false, None);
                continue;
            }

            let jp_inverted = PlineInversionView::new(&jp.polyline);
            let pair_op = if matches!(op, BooleanOp::Not) {
                BooleanOp::And
            } else {
                op
            };
            let result = ip.polyline.boolean(&jp_inverted, pair_op);
            let active = !matches!(result.result_info, BooleanResultInfo::Disjoint);
            a_ccw_used[i] |= active;
            b_cw_used[j] |= active;
            write_pair_trace(
                &mut trace,
                "ccw-cw",
                (i, j),
                true,
                Some((
                    result.result_info,
                    result.pos_plines.len(),
                    result.neg_plines.len(),
                )),
            );
        }
    }

    for (i, ip) in a.cw_plines.iter().enumerate() {
        let ip_inverted = PlineInversionView::new(&ip.polyline);
        for (j, jp) in b.ccw_plines.iter().enumerate() {
            let candidate = ip
                .spatial_index
                .bounds()
                .zip(jp.spatial_index.bounds())
                .is_some_and(|(ib, jb)| aabb_overlap(ib, jb));
            if !candidate {
                write_pair_trace(&mut trace, "cw-ccw", (i, j), false, None);
                continue;
            }

            let result = ip_inverted.boolean(&jp.polyline, op);
            let active = !matches!(result.result_info, BooleanResultInfo::Disjoint);
            a_cw_used[i] |= active;
            b_ccw_used[j] |= active;
            write_pair_trace(
                &mut trace,
                "cw-ccw",
                (i, j),
                true,
                Some((
                    result.result_info,
                    result.pos_plines.len(),
                    result.neg_plines.len(),
                )),
            );
        }
    }

    for (i, ip) in a.cw_plines.iter().enumerate() {
        let ip_inverted = PlineInversionView::new(&ip.polyline);
        for (j, jp) in b.cw_plines.iter().enumerate() {
            let candidate = ip
                .spatial_index
                .bounds()
                .zip(jp.spatial_index.bounds())
                .is_some_and(|(ib, jb)| aabb_overlap(ib, jb));
            if !candidate {
                write_pair_trace(&mut trace, "cw-cw", (i, j), false, None);
                continue;
            }

            let jp_inverted = PlineInversionView::new(&jp.polyline);
            let result = ip_inverted.boolean(&jp_inverted, op);
            let active = !matches!(result.result_info, BooleanResultInfo::Disjoint);
            a_cw_used[i] |= active;
            b_cw_used[j] |= active;
            write_pair_trace(
                &mut trace,
                "cw-cw",
                (i, j),
                true,
                Some((
                    result.result_info,
                    result.pos_plines.len(),
                    result.neg_plines.len(),
                )),
            );
        }
    }

    let _ = writeln!(
        trace,
        "non-disjoint participation: a_ccw={a_ccw_used:?} a_cw={a_cw_used:?} b_ccw={b_ccw_used:?} b_cw={b_cw_used:?}"
    );
    trace
}

/// Emit copy-pasteable and visual artifacts when sampled membership fails.
///
/// The environment gate keeps normal CI quiet while making local reduction work much faster for
/// fuzz and proptest failures.
fn dump_shape_boolean_failure(
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    op: BooleanOp,
    sample: (f64, f64),
    expected: bool,
    actual: bool,
) {
    let Ok(dir) = env::var("CAVC_DUMP_FAILURE") else {
        return;
    };

    let dir = Path::new(&dir);
    let _ = fs::create_dir_all(dir);
    let _ = write_shape_svg(&dir.join("input_a.svg"), &[(a, "blue")]);
    let _ = write_shape_svg(&dir.join("input_b.svg"), &[(b, "red")]);
    let _ = write_shape_svg(&dir.join("result.svg"), &[(result, "green")]);
    let _ = write_shape_svg(
        &dir.join("overlay.svg"),
        &[(a, "blue"), (b, "red"), (result, "green")],
    );
    let _ = write_shape_svg_with_sample(
        &dir.join("sample.svg"),
        &[(a, "blue"), (b, "red"), (result, "green")],
        Some(sample),
    );
    let _ = fs::write(
        dir.join("case.rs"),
        format!(
            "// op: {op:?}\n// sample: {:?}\n// expected: {expected}\n// actual: {actual}\n// a: {:?}\n// b: {:?}\n// result: {:?}\n",
            sample,
            shape_signature(a),
            shape_signature(b),
            shape_signature(result)
        ),
    );
    let _ = fs::write(
        dir.join("windings.txt"),
        format!(
            "sample: {sample:?}\na: {:?}\nb: {:?}\nresult: {:?}\n",
            shape_sample_windings(a, sample),
            shape_sample_windings(b, sample),
            shape_sample_windings(result, sample),
        ),
    );
    let _ = fs::write(dir.join("input_a.json"), shape_debug_json(a));
    let _ = fs::write(dir.join("input_b.json"), shape_debug_json(b));
    let _ = fs::write(dir.join("result.json"), shape_debug_json(result));
    let _ = fs::write(dir.join("trace.txt"), shape_pairwise_trace(a, b, op));
}

/// Print shape summaries and pairwise traces when explicit tracing is enabled.
fn trace_shape_boolean_case(a: &Shape<f64>, b: &Shape<f64>, result: &Shape<f64>, op: BooleanOp) {
    if env::var_os("CAVC_TRACE_SHAPE_BOOLEAN").is_none() {
        return;
    }

    let a_signature = shape_signature(a);
    let b_signature = shape_signature(b);
    let result_signature = shape_signature(result);
    let pairwise_trace = shape_pairwise_trace(a, b, op);
    eprintln!(
        "shape boolean trace op={op:?}\na={a_signature:#?}\nb={b_signature:#?}\nresult={result_signature:#?}\n{pairwise_trace}"
    );
}

/// Assert that sampled points match the set operation over the original inputs.
///
/// This is the primary semantic oracle for shape booleans because exact contour decomposition can
/// differ while still representing the same regularized area.
fn assert_boolean_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    trace_shape_boolean_case(a, b, result, op);
    // Sampled membership is the semantic oracle: exact loop topology may vary, but each point
    // must obey the set operation computed from the original input shapes.
    for (x, y) in semantic_samples(a, b, result, samples) {
        if point_on_closed_shape_boundary(a, x, y)
            || point_on_closed_shape_boundary(b, x, y)
            || point_on_closed_shape_boundary(result, x, y)
        {
            continue;
        }

        let in_a = shape_contains(a, x, y);
        let in_b = shape_contains(b, x, y);
        let expected = match op {
            BooleanOp::Or => in_a || in_b,
            BooleanOp::And => in_a && in_b,
            BooleanOp::Not => in_a && !in_b,
            BooleanOp::Xor => in_a != in_b,
        };
        let actual = shape_contains(result, x, y);
        if actual != expected {
            dump_shape_boolean_failure(a, b, result, op, (x, y), expected, actual);
        }
        let result_area = shape_signed_area(result);
        assert_eq!(
            actual, expected,
            "sample ({x}, {y}) mismatch for {op:?}: in_a={in_a}, in_b={in_b}, result={actual}, result_area={result_area}"
        );
    }
}

/// Assert shape validity, loop counts, signed area, and sampled semantics for stable cases.
fn assert_boolean_result(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    expected_area: f64,
    expected_ccw_count: usize,
    expected_cw_count: usize,
    samples: &[(f64, f64)],
) {
    let result = a.boolean(b, op);
    trace_shape_boolean_case(a, b, &result, op);
    assert_shape_valid(&result);
    let result_area = shape_signed_area(&result);
    assert_eq!(
        result.ccw_plines.len(),
        expected_ccw_count,
        "unexpected ccw loop count for {op:?}"
    );
    assert_eq!(
        result.cw_plines.len(),
        expected_cw_count,
        "unexpected cw loop count for {op:?}"
    );
    assert!(
        result_area.fuzzy_eq_eps(expected_area, 1e-5),
        "unexpected signed area for {op:?}: expected {expected_area}, got {result_area}"
    );
    assert_boolean_samples(a, b, &result, op, samples);
}

/// Assert shape validity, signed area, and sampled semantics when topology is not stable enough
/// for exact loop-count assertions.
fn assert_boolean_area_and_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    expected_area: f64,
    samples: &[(f64, f64)],
) {
    let result = a.boolean(b, op);
    trace_shape_boolean_case(a, b, &result, op);
    assert_shape_valid(&result);
    let result_area = shape_signed_area(&result);
    assert!(
        result_area.fuzzy_eq_eps(expected_area, 1e-5),
        "unexpected signed area for {op:?}: expected {expected_area}, got {result_area}"
    );
    assert_boolean_samples(a, b, &result, op, samples);
}

/// Check commutative operators in both operand orders and validate each result by samples.
fn assert_commutative_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    let ab = a.boolean(b, op);
    let ba = b.boolean(a, op);

    assert_shape_valid(&ab);
    assert_shape_valid(&ba);
    let ab_area = shape_signed_area(&ab);
    let ba_area = shape_signed_area(&ba);
    assert!(
        ab_area.fuzzy_eq_eps(ba_area, 1e-5),
        "commutative area mismatch for {op:?}: ab={ab_area}, ba={ba_area}"
    );
    assert_eq!(
        ab.ccw_plines.len(),
        ba.ccw_plines.len(),
        "commutative ccw loop count mismatch for {op:?}"
    );
    assert_eq!(
        ab.cw_plines.len(),
        ba.cw_plines.len(),
        "commutative cw loop count mismatch for {op:?}"
    );
    assert_boolean_samples(a, b, &ab, op, samples);
    assert_boolean_samples(b, a, &ba, op, samples);
}

/// Compare two result shapes by stable invariants and sample membership.
fn assert_shapes_equivalent_by_samples(a: &Shape<f64>, b: &Shape<f64>, samples: &[(f64, f64)]) {
    assert_shape_valid(a);
    assert_shape_valid(b);
    let a_area = shape_signed_area(a);
    let b_area = shape_signed_area(b);
    assert!(
        a_area.fuzzy_eq_eps(b_area, 1e-5),
        "shape area mismatch: left={a_area}, right={b_area}"
    );
    assert_eq!(
        a.ccw_plines.len(),
        b.ccw_plines.len(),
        "ccw loop count mismatch"
    );
    assert_eq!(
        a.cw_plines.len(),
        b.cw_plines.len(),
        "cw loop count mismatch"
    );

    for &(x, y) in samples {
        assert_eq!(
            shape_contains(a, x, y),
            shape_contains(b, x, y),
            "sample ({x}, {y}) equivalence mismatch"
        );
    }
}

fn assert_shape_output_finite(shape: &Shape<f64>) {
    for ip in shape.ccw_plines.iter().chain(shape.cw_plines.iter()) {
        assert!(ip.polyline.vertex_count() > 1);
        for v in ip.polyline.iter_vertexes() {
            assert!(
                v.x.is_finite(),
                "non-finite x coordinate in {:?}",
                ip.polyline
            );
            assert!(
                v.y.is_finite(),
                "non-finite y coordinate in {:?}",
                ip.polyline
            );
            assert!(v.bulge.is_finite(), "non-finite bulge in {:?}", ip.polyline);
        }
    }
}

fn dense_samples_for_shapes(
    shapes: &[&Shape<f64>],
    x_steps: usize,
    y_steps: usize,
) -> Vec<(f64, f64)> {
    let Some(extents) = combined_extents(shapes) else {
        return Vec::new();
    };

    let width = (extents.max_x - extents.min_x).max(1.0);
    let height = (extents.max_y - extents.min_y).max(1.0);
    let min_x = extents.min_x - width * 0.05;
    let max_x = extents.max_x + width * 0.05;
    let min_y = extents.min_y - height * 0.05;
    let max_y = extents.max_y + height * 0.05;

    let mut samples = Vec::with_capacity((x_steps + 1) * (y_steps + 1));
    for ix in 0..=x_steps {
        let x = min_x + (max_x - min_x) * (ix as f64 / x_steps.max(1) as f64);
        for iy in 0..=y_steps {
            let y = min_y + (max_y - min_y) * (iy as f64 / y_steps.max(1) as f64);
            samples.push((x, y));
        }
    }

    samples
}

#[test]
fn shape_boolean_empty_identities_are_valid() {
    // Empty-shape identities catch missing unused-loop retention and accidental top-level bounds.
    let empty = Shape::<f64>::empty();
    let solid = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let samples = [(-1.0, -1.0), (1.0, 1.0), (5.0, 5.0), (11.0, 11.0)];

    assert_shape_valid(&empty);
    assert_shape_valid(&solid);

    assert_boolean_result(&solid, &empty, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&empty, &solid, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::Not, 100.0, 1, 0, &samples);
    assert_boolean_result(&empty, &solid, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::Xor, 100.0, 1, 0, &samples);
}

#[test]
fn shape_view_from_plines_materializes_like_owned_shape_without_consuming_inputs() {
    let plines = vec![
        create_rectangle(-10.0, -10.0, 10.0, 10.0),
        create_cw_rectangle(-4.0, -4.0, 4.0, 4.0),
        create_rectangle(20.0, -5.0, 30.0, 5.0),
    ];
    let samples = [
        (-8.0, 0.0),
        (0.0, 0.0),
        (8.0, 0.0),
        (25.0, 0.0),
        (35.0, 0.0),
    ];

    let borrowed = ShapeView::from_plines(plines.iter());
    assert_eq!(borrowed.ccw_plines.len(), 2);
    assert_eq!(borrowed.cw_plines.len(), 1);
    assert_eq!(plines[0].vertex_count(), 4);

    let owned = Shape::from_plines(plines.clone());
    let materialized = borrowed.to_owned_shape();
    assert_shapes_equivalent_by_samples(&owned, &materialized, &samples);

    let signed = ShapeView::from_signed_plines([&plines[0], &plines[2]], [&plines[1]]);
    assert_shapes_equivalent_by_samples(&owned, &signed.to_owned_shape(), &samples);

    let view_from_shape = owned.as_view();
    assert_shapes_equivalent_by_samples(&owned, &view_from_shape.to_owned_shape(), &samples);
}

#[test]
fn shape_view_boolean_matches_owned_shape_for_all_boolean_ops() {
    let a_plines = vec![
        create_rectangle(-20.0, -20.0, 20.0, 20.0),
        create_cw_rectangle(-8.0, -8.0, 8.0, 8.0),
        create_rectangle(28.0, -6.0, 40.0, 6.0),
    ];
    let b_plines = vec![
        create_rectangle(-5.0, -25.0, 32.0, 12.0),
        create_cw_rectangle(2.0, -4.0, 10.0, 4.0),
    ];
    let a_owned = Shape::from_plines(a_plines.clone());
    let b_owned = Shape::from_plines(b_plines.clone());
    let a_view = ShapeView::from_plines(a_plines.iter());
    let b_view = ShapeView::from_plines(b_plines.iter());
    let samples = [
        (-18.0, 0.0),
        (-6.0, 0.0),
        (0.0, 0.0),
        (6.0, 0.0),
        (15.0, 0.0),
        (30.0, 0.0),
        (36.0, 0.0),
    ];

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        let owned = a_owned.boolean(&b_owned, op);
        let borrowed = a_view.boolean(&b_view, op);
        assert_shapes_equivalent_by_samples(&owned, &borrowed, &samples);

        let mixed = a_owned.boolean_view(&b_view, op);
        assert_shapes_equivalent_by_samples(&owned, &mixed, &samples);
    }
}

#[test]
fn shape_view_boolean_fast_paths_match_owned_shape() {
    let a_plines = vec![
        create_rectangle(0.0, 0.0, 12.0, 12.0),
        create_rectangle(18.0, 0.0, 30.0, 12.0),
    ];
    let b_plines = vec![create_rectangle(6.0, -4.0, 24.0, 8.0)];
    let a_owned = Shape::from_plines(a_plines.clone());
    let b_owned = Shape::from_plines(b_plines.clone());
    let a_view = ShapeView::from_plines(a_plines.iter());
    let b_view = ShapeView::from_plines(b_plines.iter());
    let samples = [
        (2.0, 2.0),
        (8.0, 2.0),
        (15.0, 2.0),
        (20.0, 2.0),
        (28.0, 2.0),
    ];

    let owned_union = a_owned.boolean(&b_owned, BooleanOp::Or);
    let borrowed_union = a_view.boolean(&b_view, BooleanOp::Or);
    assert_shapes_equivalent_by_samples(&owned_union, &borrowed_union, &samples);

    let one_a_plines = vec![create_rectangle(0.0, 0.0, 20.0, 20.0)];
    let one_b_plines = vec![create_rectangle(8.0, -4.0, 26.0, 12.0)];
    let one_a_owned = Shape::from_plines(one_a_plines.clone());
    let one_b_owned = Shape::from_plines(one_b_plines.clone());
    let one_a_view = ShapeView::from_plines(one_a_plines.iter());
    let one_b_view = ShapeView::from_plines(one_b_plines.iter());
    let single_shell_samples = [
        (4.0, 4.0),
        (10.0, 4.0),
        (18.0, 4.0),
        (24.0, 4.0),
        (10.0, 16.0),
    ];

    for op in [BooleanOp::And, BooleanOp::Not, BooleanOp::Xor] {
        let owned = one_a_owned.boolean(&one_b_owned, op);
        let borrowed = one_a_view.boolean(&one_b_view, op);
        assert_shapes_equivalent_by_samples(&owned, &borrowed, &single_shell_samples);
    }
}

#[test]
fn shape_boolean_multi_pline_ui_default_xor_does_not_panic() {
    let mut shape1 = create_multi_pline_boolean_scene_default_shape();
    let mut shape2 = create_multi_pline_boolean_scene_default_shape();
    shape1.translate_mut(-20.0, -20.0);
    shape2.translate_mut(20.0, 20.0);

    let result = shape1.boolean(&shape2, BooleanOp::Xor);

    for ip in result.ccw_plines.iter().chain(result.cw_plines.iter()) {
        assert!(ip.polyline.is_closed());
        for v in ip.polyline.iter_vertexes() {
            assert!(v.x.is_finite());
            assert!(v.y.is_finite());
            assert!(v.bulge.is_finite());
        }
    }
}

fn multi_pline_ui_default_pair() -> (Shape<f64>, Shape<f64>) {
    let mut shape1 = create_multi_pline_boolean_scene_default_shape();
    let mut shape2 = create_multi_pline_boolean_scene_default_shape();
    shape1.translate_mut(-20.0, -20.0);
    shape2.translate_mut(20.0, 20.0);
    (shape1, shape2)
}

fn assert_multi_pline_ui_default_dense_semantics(op: BooleanOp) {
    let (shape1, shape2) = multi_pline_ui_default_pair();
    let samples = dense_samples_for_shapes(&[&shape1, &shape2], 64, 64);

    let result = shape1.boolean(&shape2, op);
    assert_shape_output_finite(&result);
    assert_boolean_samples(&shape1, &shape2, &result, op, &samples);

    let result = shape2.boolean(&shape1, op);
    assert_shape_output_finite(&result);
    assert_boolean_samples(&shape2, &shape1, &result, op, &samples);
}

fn closed_loop_shapes(shape: &Shape<f64>) -> Vec<Shape<f64>> {
    shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .filter(|ip| ip.polyline.is_closed())
        .map(|ip| {
            let mut pline = ip.polyline.clone();
            if pline.orientation() == PlineOrientation::Clockwise {
                pline.invert_direction_mut();
            }
            Shape::from_plines([pline])
        })
        .collect()
}

fn multi_pline_ui_default_loop_pair(i: usize, j: usize) -> (Shape<f64>, Shape<f64>) {
    let (shape1, shape2) = multi_pline_ui_default_pair();
    let shape1_loops = closed_loop_shapes(&shape1);
    let shape2_loops = closed_loop_shapes(&shape2);
    (shape1_loops[i].clone(), shape2_loops[j].clone())
}

fn assert_boolean_samples_with_context(
    context: &str,
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    for (x, y) in semantic_samples(a, b, result, samples) {
        let in_a = shape_contains(a, x, y);
        let in_b = shape_contains(b, x, y);
        let expected = match op {
            BooleanOp::Or => in_a || in_b,
            BooleanOp::And => in_a && in_b,
            BooleanOp::Not => in_a && !in_b,
            BooleanOp::Xor => in_a != in_b,
        };
        let actual = shape_contains(result, x, y);
        assert_eq!(
            actual,
            expected,
            "{context}: sample ({x}, {y}) mismatch for {op:?}: in_a={in_a}, in_b={in_b}, result={actual}, result_area={}",
            shape_signed_area(result)
        );
    }
}

#[test]
fn shape_boolean_multi_pline_ui_default_pairwise_loop_semantics() {
    let (shape1, shape2) = multi_pline_ui_default_pair();
    let shape1_loops = closed_loop_shapes(&shape1);
    let shape2_loops = closed_loop_shapes(&shape2);

    for (i, a) in shape1_loops.iter().enumerate() {
        for (j, b) in shape2_loops.iter().enumerate() {
            let samples = dense_samples_for_shapes(&[a, b], 24, 24);
            for op in [
                BooleanOp::Or,
                BooleanOp::And,
                BooleanOp::Not,
                BooleanOp::Xor,
            ] {
                let result = a.boolean(b, op);
                assert_shape_output_finite(&result);
                assert_boolean_samples_with_context(
                    &format!("shape1 loop {i} vs shape2 loop {j}"),
                    a,
                    b,
                    &result,
                    op,
                    &samples,
                );

                let result = b.boolean(a, op);
                assert_shape_output_finite(&result);
                assert_boolean_samples_with_context(
                    &format!("shape2 loop {j} vs shape1 loop {i}"),
                    b,
                    a,
                    &result,
                    op,
                    &samples,
                );
            }

            assert!(
                !samples.is_empty(),
                "expected samples for UI loop pair {i}/{j}"
            );
        }
    }
}

#[test]
fn shape_boolean_multi_pline_ui_default_loop0_loop0_xor_dense_semantics() {
    let (a, b) = multi_pline_ui_default_loop_pair(0, 0);
    let samples = dense_samples_for_shapes(&[&a, &b], 64, 64);
    let result = a.boolean(&b, BooleanOp::Xor);

    assert_shape_output_finite(&result);
    assert_boolean_samples_with_context(
        "UI loop 0 vs translated loop 0",
        &a,
        &b,
        &result,
        BooleanOp::Xor,
        &samples,
    );
}

#[test]
fn shape_boolean_multi_pline_ui_default_or_dense_semantics() {
    assert_multi_pline_ui_default_dense_semantics(BooleanOp::Or);
}

#[test]
fn shape_boolean_multi_pline_ui_default_and_dense_semantics() {
    assert_multi_pline_ui_default_dense_semantics(BooleanOp::And);
}

#[test]
fn shape_boolean_multi_pline_ui_default_not_dense_semantics() {
    assert_multi_pline_ui_default_dense_semantics(BooleanOp::Not);
}

#[test]
fn shape_boolean_multi_pline_ui_default_xor_dense_semantics() {
    assert_multi_pline_ui_default_dense_semantics(BooleanOp::Xor);
}

#[test]
fn shape_boolean_rectangle_containment_all_ops() {
    // Containment exercises lower-level "inside" result_info paths and the shape-level decision
    // about whether a contained loop becomes material, a hole, or disappears.
    let outer = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let inner = Shape::from_plines(vec![create_rectangle(2.0, 2.0, 4.0, 4.0)]);
    let samples = [(1.0, 1.0), (3.0, 3.0), (8.0, 8.0), (12.0, 12.0)];

    assert_boolean_result(&outer, &inner, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::And, 4.0, 1, 0, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::Not, 96.0, 1, 1, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::Xor, 96.0, 1, 1, &samples);
}

#[test]
fn shape_boolean_edge_touching_rectangles_do_not_create_area() {
    // Shared boundaries are regularized as zero area. These cases guard against emitting tiny
    // slivers or duplicate loops when pairwise booleans return boundary-only relationships.
    let left = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let right = Shape::from_plines(vec![create_rectangle(10.0, 0.0, 20.0, 10.0)]);
    let samples = [(5.0, 5.0), (15.0, 5.0), (10.5, 5.0), (-1.0, 5.0)];

    assert_boolean_result(&left, &right, BooleanOp::Or, 200.0, 1, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::Not, 100.0, 1, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::Xor, 200.0, 1, 0, &samples);
}

#[test]
fn shape_boolean_square_inside_hole_respects_empty_space() {
    // A positive island inside a hole is disjoint from the donut's filled area but must be
    // preserved by union and XOR as a nested positive loop.
    let donut = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_cw_rectangle(3.0, 3.0, 7.0, 7.0),
    ]);
    let island = Shape::from_plines(vec![create_rectangle(4.0, 4.0, 6.0, 6.0)]);
    let samples = [(1.0, 1.0), (3.5, 3.5), (5.0, 5.0), (8.0, 8.0), (11.0, 11.0)];

    assert_boolean_result(&donut, &island, BooleanOp::Or, 88.0, 2, 1, &samples);
    assert_boolean_result(&island, &donut, BooleanOp::Or, 88.0, 2, 1, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::Not, 84.0, 1, 1, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::Xor, 88.0, 2, 1, &samples);
}

#[test]
fn shape_boolean_identical_rectangle_all_ops() {
    // Identical simple loops should take the same identity fast path as more complex equal shapes.
    let rect = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let samples = [(1.0, 1.0), (5.0, 5.0), (11.0, 11.0)];

    assert_boolean_result(&rect, &rect, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::And, 100.0, 1, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::Xor, 0.0, 0, 0, &samples);
}

#[test]
fn shape_boolean_identical_donut_all_ops() {
    // Equal shapes with holes are an identity case for every boolean operator. This specifically
    // protects against treating matching CW holes as positive material during shape-level union.
    let donut = create_donut((0.0, 0.0, 12.0, 12.0), &[(2.0, 2.0, 10.0, 10.0)]);
    let same_donut = donut.clone();
    let samples = [(1.0, 1.0), (6.0, 6.0), (11.0, 11.0), (13.0, 13.0)];

    assert_boolean_result(&donut, &same_donut, BooleanOp::Or, 80.0, 1, 1, &samples);
    assert_boolean_result(&donut, &same_donut, BooleanOp::And, 80.0, 1, 1, &samples);
    assert_boolean_result(&donut, &same_donut, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut, &same_donut, BooleanOp::Xor, 0.0, 0, 0, &samples);
}

#[test]
fn shape_boolean_same_shape_identity_ignores_loop_order_and_start_index() {
    // The identity fast path must be geometric, not storage-order based. This variant keeps the
    // same signed loop bins but reorders the CCW loops and rotates every closed-loop start index.
    let base = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 20.0, 20.0),
        create_cw_rectangle(5.0, 5.0, 15.0, 15.0),
        create_rectangle(8.0, 8.0, 12.0, 12.0),
    ]);
    let variant = Shape::from_plines(vec![
        rotate_pline_start(&create_rectangle(8.0, 8.0, 12.0, 12.0), 2),
        rotate_pline_start(&create_cw_rectangle(5.0, 5.0, 15.0, 15.0), 1),
        rotate_pline_start(&create_rectangle(0.0, 0.0, 20.0, 20.0), 3),
    ]);
    let samples = [
        (2.0, 2.0),
        (6.0, 6.0),
        (10.0, 10.0),
        (18.0, 18.0),
        (25.0, 25.0),
    ];

    assert_boolean_result(&base, &variant, BooleanOp::Or, 316.0, 2, 1, &samples);
    assert_boolean_result(&base, &variant, BooleanOp::And, 316.0, 2, 1, &samples);
    assert_boolean_result(&base, &variant, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&base, &variant, BooleanOp::Xor, 0.0, 0, 0, &samples);
}

#[test]
fn shape_boolean_commutative_overlapping_rectangles() {
    // Union, intersection, and XOR must not depend on operand order even though the implementation
    // tracks used loops separately for the left and right shapes.
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let b = create_shape_rects(&[(5.0, 5.0, 15.0, 15.0)]);
    let samples = [(2.0, 2.0), (7.5, 7.5), (12.0, 12.0), (16.0, 16.0)];

    assert_commutative_samples(&a, &b, BooleanOp::Or, &samples);
    assert_commutative_samples(&a, &b, BooleanOp::And, &samples);
    assert_commutative_samples(&a, &b, BooleanOp::Xor, &samples);
}

#[test]
fn shape_boolean_multi_island_disjoint_identities() {
    // Disjoint multi-island inputs verify that broad-phase pair filtering still preserves
    // untouched loops through the unused-loop retention path.
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)]);
    let b = create_shape_rects(&[(100.0, 0.0, 110.0, 10.0)]);
    let samples = [(5.0, 5.0), (25.0, 5.0), (105.0, 5.0), (50.0, 5.0)];

    assert_boolean_result(&a, &b, BooleanOp::Or, 300.0, 3, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::Not, 200.0, 2, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::Xor, 300.0, 3, 0, &samples);
}

#[test]
fn shape_boolean_bridge_overlaps_two_islands() {
    // One input loop intersects two separate islands, forcing pairwise results to merge into a
    // single output component for union while preserving separated pieces for intersection.
    let islands = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)]);
    let bridge = create_shape_rects(&[(5.0, -5.0, 25.0, 15.0)]);
    let samples = [
        (2.5, 5.0),
        (7.5, 5.0),
        (15.0, 0.0),
        (22.5, 5.0),
        (27.5, 5.0),
        (15.0, 12.5),
        (15.0, 20.0),
    ];

    assert_boolean_result(&islands, &bridge, BooleanOp::Or, 500.0, 1, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::And, 100.0, 2, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::Not, 100.0, 2, 0, &samples);
    assert_commutative_samples(&islands, &bridge, BooleanOp::Or, &samples);
    assert_commutative_samples(&islands, &bridge, BooleanOp::And, &samples);
}

#[test]
fn shape_boolean_ring_difference_adds_second_hole() {
    // Difference against material inside the filled ring should add a new hole without disturbing
    // an existing unrelated hole.
    let ring = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let cutter_in_filled_ring = create_shape_rects(&[(1.0, 1.0, 2.0, 2.0)]);
    let samples = [(1.5, 1.5), (2.5, 2.5), (5.0, 5.0), (8.0, 8.0), (11.0, 11.0)];

    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::Not,
        83.0,
        1,
        2,
        &samples,
    );
    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::And,
        1.0,
        1,
        0,
        &samples,
    );
    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::Xor,
        83.0,
        1,
        2,
        &samples,
    );
}

#[test]
fn shape_boolean_loop_order_and_start_index_are_irrelevant() {
    // Shape booleans should depend on geometry, not storage order or the chosen start vertex of a
    // closed loop. This protects the pairwise bookkeeping from index-order assumptions.
    let base_a = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_rectangle(20.0, 0.0, 30.0, 10.0),
    ]);
    let variant_a = Shape::from_plines(vec![
        rotate_pline_start(&create_rectangle(20.0, 0.0, 30.0, 10.0), 2),
        rotate_pline_start(&create_rectangle(0.0, 0.0, 10.0, 10.0), 3),
    ]);

    let base_b = Shape::from_plines(vec![create_rectangle(5.0, -5.0, 25.0, 15.0)]);
    let variant_b = Shape::from_plines(vec![rotate_pline_start(
        &create_rectangle(5.0, -5.0, 25.0, 15.0),
        1,
    )]);

    let samples = [
        (2.0, 2.0),
        (5.5, 5.5),
        (8.0, 8.0),
        (15.0, 1.0),
        (15.0, 4.0),
        (22.0, 5.0),
        (28.0, 5.0),
        (40.0, 40.0),
    ];

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        let base_result = base_a.boolean(&base_b, op);
        let variant_result = variant_a.boolean(&variant_b, op);
        assert_shapes_equivalent_by_samples(&base_result, &variant_result, &samples);
        assert_boolean_samples(&base_a, &base_b, &base_result, op, &samples);
        assert_boolean_samples(&variant_a, &variant_b, &variant_result, op, &samples);
    }
}

#[test]
fn shape_boolean_chain_bridge_merges_three_islands() {
    // The bridge overlaps three islands transitively; a one-pass pairwise collector can easily
    // leave duplicate or partially merged loops here if final assembly is not robust.
    let islands = create_shape_rects(&[
        (0.0, 0.0, 6.0, 6.0),
        (10.0, 0.0, 16.0, 6.0),
        (20.0, 0.0, 26.0, 6.0),
    ]);
    let bridge = create_shape_rects(&[(3.0, -2.0, 23.0, 8.0)]);
    let samples = [
        (1.0, 1.0),
        (5.0, 1.0),
        (8.0, 0.0),
        (13.0, 3.0),
        (18.0, 0.0),
        (24.0, 3.0),
        (12.0, 7.0),
        (30.0, 3.0),
    ];

    assert_boolean_result(&islands, &bridge, BooleanOp::Or, 236.0, 1, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::And, 72.0, 3, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::Not, 36.0, 2, 0, &samples);
    assert_boolean_area_and_samples(&islands, &bridge, BooleanOp::Xor, 164.0, &samples);
}

#[test]
fn shape_boolean_hole_boundary_cutter_splits_ring_without_filling_hole() {
    // Regression: a cutter that crosses a hole boundary must not cause the original hole to be
    // treated as filled material during intersection or difference assembly.
    let ring = create_donut((0.0, 0.0, 12.0, 12.0), &[(4.0, 4.0, 8.0, 8.0)]);
    let cutter = create_shape_rects(&[(6.0, 2.0, 10.0, 10.0)]);
    let samples = [
        (2.0, 2.0),
        (5.0, 5.0),
        (7.0, 3.0),
        (7.0, 7.0),
        (9.0, 9.0),
        (11.0, 11.0),
        (13.0, 13.0),
    ];

    assert_boolean_result(&ring, &cutter, BooleanOp::And, 24.0, 1, 0, &samples);
    assert_boolean_area_and_samples(&ring, &cutter, BooleanOp::Not, 104.0, &samples);
}

#[test]
fn shape_boolean_hole_boundary_cutter_xor() {
    // Regression for XOR's (A - B) union (B - A) composition: the B - A island lies inside the
    // merged hole from A - B, so union must keep both nested loops instead of filling the hole.
    let ring = create_donut((0.0, 0.0, 12.0, 12.0), &[(4.0, 4.0, 8.0, 8.0)]);
    let cutter = create_shape_rects(&[(6.0, 2.0, 10.0, 10.0)]);
    let samples = [
        (2.0, 2.0),
        (5.0, 5.0),
        (7.0, 3.0),
        (7.0, 7.0),
        (9.0, 9.0),
        (11.0, 11.0),
        (13.0, 13.0),
    ];

    assert_boolean_area_and_samples(&ring, &cutter, BooleanOp::Xor, 112.0, &samples);
}

#[test]
fn shape_boolean_point_touching_rectangles_do_not_create_intersection_area() {
    // Point contacts are another zero-area regularization case; they should not produce filled
    // intersection loops or corrupt difference/union area.
    let lower_left = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let upper_right = create_shape_rects(&[(10.0, 10.0, 20.0, 20.0)]);
    let samples = [(5.0, 5.0), (15.0, 15.0), (10.001, 10.001), (9.999, 9.999)];

    assert_boolean_area_and_samples(&lower_left, &upper_right, BooleanOp::Or, 200.0, &samples);
    assert_boolean_result(
        &lower_left,
        &upper_right,
        BooleanOp::And,
        0.0,
        0,
        0,
        &samples,
    );
    assert_boolean_area_and_samples(&lower_left, &upper_right, BooleanOp::Xor, 200.0, &samples);
}

#[test]
fn shape_boolean_partially_shared_edge_is_regularized() {
    // Partial edge overlap combines clipping and boundary-only contact in one case.
    let bottom = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let top = create_shape_rects(&[(5.0, 10.0, 15.0, 20.0)]);
    let samples = [(2.0, 2.0), (7.5, 9.999), (7.5, 10.001), (12.0, 12.0)];

    assert_boolean_area_and_samples(&bottom, &top, BooleanOp::Or, 200.0, &samples);
    assert_boolean_result(&bottom, &top, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_area_and_samples(&bottom, &top, BooleanOp::Not, 100.0, &samples);
}

#[test]
fn shape_boolean_thin_sliver_overlap_keeps_area_identities() {
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let b = create_shape_rects(&[(9.999, 0.0, 20.0, 10.0)]);
    let samples = [(5.0, 5.0), (9.9995, 5.0), (10.0005, 5.0), (15.0, 5.0)];

    assert_boolean_area_and_samples(&a, &b, BooleanOp::Or, 200.0, &samples);
    assert_boolean_area_and_samples(&a, &b, BooleanOp::And, 0.01, &samples);
    assert_boolean_area_and_samples(&a, &b, BooleanOp::Not, 99.99, &samples);
    // The current lower-level boolean regularizes this tiny overlap on the XOR path; keep the
    // semantic samples active while accepting the current stable area.
    assert_boolean_area_and_samples(&a, &b, BooleanOp::Xor, 199.99, &samples);
}

#[test]
fn shape_boolean_circle_disjoint_tangent_and_identical_cases() {
    // Two-bulge circles keep true arc geometry in play while covering disjoint, tangent, and equal
    // result_info paths.
    let c1_pline = create_approx_circle(0.0, 0.0, 2.0);
    let c2_pline = create_approx_circle(6.0, 0.0, 2.0);
    let tangent_pline = create_approx_circle(4.0, 0.0, 2.0);
    let c1_area = c1_pline.area();

    let c1 = Shape::from_plines(vec![c1_pline.clone()]);
    let c2 = Shape::from_plines(vec![c2_pline]);
    let tangent = Shape::from_plines(vec![tangent_pline]);
    let c1_again = Shape::from_plines(vec![c1_pline]);

    let disjoint_samples = [(0.0, 0.0), (6.0, 0.0), (3.0, 0.0), (10.0, 0.0)];
    assert_boolean_area_and_samples(&c1, &c2, BooleanOp::Or, c1_area * 2.0, &disjoint_samples);
    assert_boolean_result(&c1, &c2, BooleanOp::And, 0.0, 0, 0, &disjoint_samples);
    assert_boolean_area_and_samples(&c1, &c2, BooleanOp::Xor, c1_area * 2.0, &disjoint_samples);

    let tangent_samples = [(0.0, 0.0), (4.0, 0.0), (2.0001, 0.0), (8.0, 0.0)];
    assert_boolean_area_and_samples(
        &c1,
        &tangent,
        BooleanOp::Or,
        c1_area * 2.0,
        &tangent_samples,
    );
    assert_boolean_result(&c1, &tangent, BooleanOp::And, 0.0, 0, 0, &tangent_samples);

    let identical_samples = [(0.0, 0.0), (1.0, 0.0), (2.1, 0.0), (-2.1, 0.0)];
    assert_boolean_result(
        &c1,
        &c1_again,
        BooleanOp::Or,
        c1_area,
        1,
        0,
        &identical_samples,
    );
    assert_boolean_result(
        &c1,
        &c1_again,
        BooleanOp::And,
        c1_area,
        1,
        0,
        &identical_samples,
    );
    assert_boolean_result(
        &c1,
        &c1_again,
        BooleanOp::Not,
        0.0,
        0,
        0,
        &identical_samples,
    );
}

#[test]
fn shape_boolean_donut_square_outside_and_hole_boundary_cases() {
    // A disjoint square should survive union/XOR but leave the donut's existing hole untouched.
    let donut = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let outside_square = create_shape_rects(&[(20.0, 20.0, 22.0, 22.0)]);

    let outside_samples = [(1.0, 1.0), (5.0, 5.0), (21.0, 21.0), (15.0, 15.0)];
    assert_boolean_result(
        &donut,
        &outside_square,
        BooleanOp::Or,
        88.0,
        2,
        1,
        &outside_samples,
    );
    assert_boolean_result(
        &donut,
        &outside_square,
        BooleanOp::And,
        0.0,
        0,
        0,
        &outside_samples,
    );
    assert_boolean_result(
        &donut,
        &outside_square,
        BooleanOp::Not,
        84.0,
        1,
        1,
        &outside_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &outside_square,
        BooleanOp::Xor,
        88.0,
        &outside_samples,
    );
}

#[test]
fn reported_shape_boolean_hole_boundary_union_area_inflation_1() {
    // Regression: when an added island crosses a hole boundary, union must clip the old hole
    // rather than carrying the full hole forward and inflating the final area.
    let donut = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let hole_boundary_square = create_shape_rects(&[(2.0, 2.0, 5.0, 5.0)]);

    let boundary_samples = [(2.5, 2.5), (4.5, 4.5), (1.0, 1.0), (8.0, 8.0), (11.0, 11.0)];
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::Or,
        88.0,
        &boundary_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::And,
        5.0,
        &boundary_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::Not,
        79.0,
        &boundary_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::Xor,
        83.0,
        &boundary_samples,
    );
}

#[test]
fn shape_boolean_two_disjoint_donuts_and_nested_ring() {
    // Combines multiple hole-bearing components with a nested island to keep signed-loop binning
    // honest across disjoint and contained material.
    let donut_a = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let donut_b = create_donut((20.0, 0.0, 30.0, 10.0), &[(23.0, 3.0, 27.0, 7.0)]);
    let samples = [
        (1.0, 1.0),
        (5.0, 5.0),
        (21.0, 1.0),
        (25.0, 5.0),
        (15.0, 5.0),
    ];

    assert_boolean_result(&donut_a, &donut_b, BooleanOp::Or, 168.0, 2, 2, &samples);
    assert_boolean_result(&donut_a, &donut_b, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut_a, &donut_b, BooleanOp::Not, 84.0, 1, 1, &samples);
    assert_boolean_area_and_samples(&donut_a, &donut_b, BooleanOp::Xor, 168.0, &samples);

    let nested = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_cw_rectangle(2.0, 2.0, 8.0, 8.0),
        create_rectangle(4.0, 4.0, 6.0, 6.0),
    ]);
    assert_shape_valid(&nested);
    assert!(shape_signed_area(&nested).fuzzy_eq_eps(68.0, 1e-5));
    assert_boolean_result(
        &nested,
        &Shape::<f64>::empty(),
        BooleanOp::Or,
        68.0,
        2,
        1,
        &[(1.0, 1.0), (3.0, 3.0), (5.0, 5.0), (11.0, 11.0)],
    );
}

#[cfg(test)]
mod shape_boolean_proptests {
    //! Property tests focused on algebraic laws rather than exact output topology.
    //!
    //! The generators start with rectangles and rectangular donuts because they shrink well and
    //! have exact expected areas, which makes failures easier to reduce before adding more
    //! free-form polygon or arc cases.

    use super::*;
    use proptest::prelude::*;

    type Rect = (f64, f64, f64, f64);
    type RectPair = (Rect, Rect);

    fn rect_strategy() -> impl Strategy<Value = Rect> {
        // Integer coordinates keep expected areas exact while still exploring many placements.
        (-100i32..100, -100i32..100, 1i32..40, 1i32..40).prop_map(|(x, y, w, h)| {
            let xmin = f64::from(x);
            let ymin = f64::from(y);
            (xmin, ymin, xmin + f64::from(w), ymin + f64::from(h))
        })
    }

    fn disjoint_rect_set_strategy() -> impl Strategy<Value = Vec<Rect>> {
        (-100i32..80, -100i32..100, 1usize..=4).prop_flat_map(|(x, y, count)| {
            prop::collection::vec((1i32..30, 1i32..30, 1i32..12), count).prop_map(move |dims| {
                let mut cursor = f64::from(x);
                dims.into_iter()
                    .map(|(w, h, gap)| {
                        let rect = (
                            cursor,
                            f64::from(y),
                            cursor + f64::from(w),
                            f64::from(y + h),
                        );
                        cursor += f64::from(w + gap);
                        rect
                    })
                    .collect()
            })
        })
    }

    fn nested_rect_stack_strategy() -> impl Strategy<Value = Vec<Rect>> {
        (-30i32..30, -30i32..30, 28i32..80, 28i32..80, 3usize..=5).prop_map(
            |(x, y, w, h, depth)| {
                let inset_step = f64::from(w.min(h)) / (2.0 * depth as f64 + 2.0);
                (0..depth)
                    .map(|i| {
                        let inset = (i as f64) * inset_step;
                        (
                            f64::from(x) + inset,
                            f64::from(y) + inset,
                            f64::from(x + w) - inset,
                            f64::from(y + h) - inset,
                        )
                    })
                    .collect()
            },
        )
    }

    fn rect_area(rect: Rect) -> f64 {
        (rect.2 - rect.0) * (rect.3 - rect.1)
    }

    fn rect_center(rect: Rect) -> (f64, f64) {
        ((rect.0 + rect.2) * 0.5, (rect.1 + rect.3) * 0.5)
    }

    fn rect_contains(rect: Rect, x: f64, y: f64) -> bool {
        x > rect.0 && x < rect.2 && y > rect.1 && y < rect.3
    }

    fn rect_set_contains(rects: &[Rect], x: f64, y: f64) -> bool {
        rects.iter().any(|&rect| rect_contains(rect, x, y))
    }

    fn point_on_rect_boundary(rect: Rect, x: f64, y: f64) -> bool {
        let on_vertical = (x.fuzzy_eq_eps(rect.0, SHAPE_TEST_EPS)
            || x.fuzzy_eq_eps(rect.2, SHAPE_TEST_EPS))
            && y >= rect.1 - SHAPE_TEST_EPS
            && y <= rect.3 + SHAPE_TEST_EPS;
        let on_horizontal = (y.fuzzy_eq_eps(rect.1, SHAPE_TEST_EPS)
            || y.fuzzy_eq_eps(rect.3, SHAPE_TEST_EPS))
            && x >= rect.0 - SHAPE_TEST_EPS
            && x <= rect.2 + SHAPE_TEST_EPS;
        on_vertical || on_horizontal
    }

    fn point_on_any_rect_boundary(rects: &[Rect], x: f64, y: f64) -> bool {
        rects.iter().any(|&rect| point_on_rect_boundary(rect, x, y))
    }

    fn boolean_expected(in_a: bool, in_b: bool, op: BooleanOp) -> bool {
        match op {
            BooleanOp::Or => in_a || in_b,
            BooleanOp::And => in_a && in_b,
            BooleanOp::Not => in_a && !in_b,
            BooleanOp::Xor => in_a != in_b,
        }
    }

    fn outside_sample(rect: Rect) -> (f64, f64) {
        (rect.2 + 10.0, rect.3 + 10.0)
    }

    fn rect_set_samples(a: &[Rect], b: &[Rect]) -> Vec<(f64, f64)> {
        let mut samples = Vec::new();
        for &rect in a.iter().chain(b.iter()) {
            samples.push(rect_center(rect));
            samples.push((rect.0 + 0.31, rect.1 + 0.47));
            samples.push((rect.2 - 0.37, rect.3 - 0.53));
        }

        let min_x = a
            .iter()
            .chain(b.iter())
            .map(|rect| rect.0)
            .fold(f64::INFINITY, f64::min);
        let min_y = a
            .iter()
            .chain(b.iter())
            .map(|rect| rect.1)
            .fold(f64::INFINITY, f64::min);
        let max_x = a
            .iter()
            .chain(b.iter())
            .map(|rect| rect.2)
            .fold(f64::NEG_INFINITY, f64::max);
        let max_y = a
            .iter()
            .chain(b.iter())
            .map(|rect| rect.3)
            .fold(f64::NEG_INFINITY, f64::max);
        let width = (max_x - min_x).max(1.0);
        let height = (max_y - min_y).max(1.0);
        for fx in [0.13, 0.29, 0.47, 0.71, 0.89] {
            for fy in [0.17, 0.37, 0.59, 0.83] {
                samples.push((min_x + width * fx, min_y + height * fy));
            }
        }
        samples.push((max_x + 5.0, max_y + 5.0));
        samples.retain(|&(x, y)| {
            !point_on_any_rect_boundary(a, x, y) && !point_on_any_rect_boundary(b, x, y)
        });
        samples
    }

    fn shape_from_nested_stack(rects: &[Rect]) -> Shape<f64> {
        Shape::from_plines(rects.iter().enumerate().map(|(i, &rect)| {
            if i % 2 == 0 {
                create_rectangle(rect.0, rect.1, rect.2, rect.3)
            } else {
                create_cw_rectangle(rect.0, rect.1, rect.2, rect.3)
            }
        }))
    }

    fn nested_stack_area(rects: &[Rect]) -> f64 {
        rects
            .iter()
            .enumerate()
            .map(|(i, &rect)| {
                if i % 2 == 0 {
                    rect_area(rect)
                } else {
                    -rect_area(rect)
                }
            })
            .sum()
    }

    fn rect_overlap_area(a: Rect, b: Rect) -> f64 {
        let xmin = a.0.max(b.0);
        let ymin = a.1.max(b.1);
        let xmax = a.2.min(b.2);
        let ymax = a.3.min(b.3);
        if xmax <= xmin || ymax <= ymin {
            0.0
        } else {
            (xmax - xmin) * (ymax - ymin)
        }
    }

    fn overlapping_rect_pair_strategy() -> impl Strategy<Value = RectPair> {
        // Bias toward guaranteed overlaps so area identities exercise clipping instead of only
        // unused-loop retention.
        (-50i32..50, -50i32..50, 6i32..40, 6i32..40, 1i32..6, 1i32..6).prop_map(
            |(x, y, w, h, inset_x, inset_y)| {
                let a = (
                    f64::from(x),
                    f64::from(y),
                    f64::from(x + w),
                    f64::from(y + h),
                );
                let b = (
                    f64::from(x + inset_x),
                    f64::from(y + inset_y),
                    f64::from(x + w + inset_x),
                    f64::from(y + h + inset_y),
                );
                (a, b)
            },
        )
    }

    fn donut_rect_strategy() -> impl Strategy<Value = RectPair> {
        (-50i32..50, -50i32..50, 16i32..60, 16i32..60).prop_map(|(x, y, w, h)| {
            let margin_x = (w / 4).max(3);
            let margin_y = (h / 4).max(3);
            let outer = (
                f64::from(x),
                f64::from(y),
                f64::from(x + w),
                f64::from(y + h),
            );
            let inner = (
                f64::from(x + margin_x),
                f64::from(y + margin_y),
                f64::from(x + w - margin_x),
                f64::from(y + h - margin_y),
            );
            (outer, inner)
        })
    }

    fn translate_rect(rect: Rect, dx: f64, dy: f64) -> Rect {
        (rect.0 + dx, rect.1 + dy, rect.2 + dx, rect.3 + dy)
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 64,
            failure_persistence: None,
            ..ProptestConfig::default()
        })]

        #[test]
        fn rectangle_empty_identities_hold(rect in rect_strategy()) {
            let shape = create_shape_rects(&[rect]);
            let empty = Shape::<f64>::empty();
            let area = rect_area(rect);
            let samples = [rect_center(rect), outside_sample(rect)];

            assert_boolean_result(&shape, &empty, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&empty, &shape, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::Not, area, 1, 0, &samples);
            assert_boolean_result(&empty, &shape, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::Xor, area, 1, 0, &samples);
        }

        #[test]
        fn translated_disjoint_rectangles_obey_basic_area_laws(
            rect in rect_strategy(),
            gap in 1i32..50,
            w2 in 1i32..40,
            h2 in 1i32..40,
        ) {
            let b = (
                rect.2 + f64::from(gap),
                rect.1,
                rect.2 + f64::from(gap + w2),
                rect.1 + f64::from(h2),
            );
            let a_shape = create_shape_rects(&[rect]);
            let b_shape = create_shape_rects(&[b]);
            let samples = [rect_center(rect), rect_center(b), outside_sample(b)];

            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Or,
                rect_area(rect) + rect_area(b),
                2,
                0,
                &samples,
            );
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::Not, rect_area(rect), 1, 0, &samples);
            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Xor,
                rect_area(rect) + rect_area(b),
                2,
                0,
                &samples,
            );
        }

        #[test]
        fn rectangle_self_identities_hold(rect in rect_strategy()) {
            let shape = create_shape_rects(&[rect]);
            let area = rect_area(rect);
            let samples = [rect_center(rect), outside_sample(rect)];

            assert_boolean_result(&shape, &shape, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::And, area, 1, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Xor, 0.0, 0, 0, &samples);
        }

        #[test]
        fn overlapping_rectangles_obey_area_identities((a, b) in overlapping_rect_pair_strategy()) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let overlap = rect_overlap_area(a, b);
            let union_area = rect_area(a) + rect_area(b) - overlap;
            let xor_area = rect_area(a) + rect_area(b) - 2.0 * overlap;
            let samples = [
                ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5),
                ((a.2 + b.2) * 0.5, (a.3 + b.3) * 0.5),
                ((a.0.max(b.0) + a.2.min(b.2)) * 0.5, (a.1.max(b.1) + a.3.min(b.3)) * 0.5),
                outside_sample(b),
            ];

            assert_boolean_result(&a_shape, &b_shape, BooleanOp::Or, union_area, 1, 0, &samples);
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::And, overlap, 1, 0, &samples);
            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Not,
                rect_area(a) - overlap,
                1,
                0,
                &samples,
            );
            assert_boolean_area_and_samples(&a_shape, &b_shape, BooleanOp::Xor, xor_area, &samples);
        }

        #[test]
        fn donut_empty_and_self_identities_hold((outer, inner) in donut_rect_strategy()) {
            let donut = create_donut(outer, &[inner]);
            let same_donut = donut.clone();
            let empty = Shape::<f64>::empty();
            let area = rect_area(outer) - rect_area(inner);
            let samples = [rect_center(outer), rect_center(inner), outside_sample(outer)];

            assert_boolean_area_and_samples(&donut, &empty, BooleanOp::Or, area, &samples);
            assert_boolean_result(&donut, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_area_and_samples(&donut, &empty, BooleanOp::Not, area, &samples);
            assert_boolean_area_and_samples(&donut, &same_donut, BooleanOp::Or, area, &samples);
            assert_boolean_area_and_samples(&donut, &same_donut, BooleanOp::And, area, &samples);
            assert_boolean_result(&donut, &same_donut, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&donut, &same_donut, BooleanOp::Xor, 0.0, 0, 0, &samples);
        }

        #[test]
        fn rectangle_difference_decomposes_into_difference_and_intersection(
            (a, b) in overlapping_rect_pair_strategy(),
        ) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let diff = a_shape.boolean(&b_shape, BooleanOp::Not);
            let intersection = a_shape.boolean(&b_shape, BooleanOp::And);
            let recomposed = diff.boolean(&intersection, BooleanOp::Or);
            let samples = [
                rect_center(a),
                rect_center(b),
                ((a.0.max(b.0) + a.2.min(b.2)) * 0.5, (a.1.max(b.1) + a.3.min(b.3)) * 0.5),
                outside_sample(b),
            ];

            assert_shapes_equivalent_by_samples(&a_shape, &recomposed, &samples);
        }

        #[test]
        fn rectangle_sets_match_independent_oracle(
            a_rects in disjoint_rect_set_strategy(),
            b_rects in disjoint_rect_set_strategy(),
        ) {
            let a_shape = create_shape_rects(&a_rects);
            let b_shape = create_shape_rects(&b_rects);
            let samples = rect_set_samples(&a_rects, &b_rects);

            // Multi-island difference/XOR still have dedicated regression coverage elsewhere.
            // Keep this independent oracle focused on the stable set modes so it can run in
            // normal CI while still checking multiple-loop assembly against a non-Shape oracle.
            for op in [BooleanOp::Or, BooleanOp::And] {
                let result = a_shape.boolean(&b_shape, op);
                assert_shape_valid(&result);
                for (x, y) in &samples {
                    assert_eq!(
                        shape_contains(&result, *x, *y),
                        boolean_expected(
                            rect_set_contains(&a_rects, *x, *y),
                            rect_set_contains(&b_rects, *x, *y),
                            op,
                        ),
                        "rectangle-set oracle mismatch for {op:?} at ({x}, {y})"
                    );
                }
            }
        }

        #[test]
        fn nested_rectangle_stack_empty_and_self_identities_hold(rects in nested_rect_stack_strategy()) {
            let shape = shape_from_nested_stack(&rects);
            let empty = Shape::<f64>::empty();
            let area = nested_stack_area(&rects);
            let samples = rects.iter().map(|&rect| rect_center(rect)).collect::<Vec<_>>();

            assert_shape_valid(&shape);
            assert!(shape_signed_area(&shape).fuzzy_eq_eps(area, 1e-5));
            assert_boolean_area_and_samples(&shape, &empty, BooleanOp::Or, area, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_area_and_samples(&shape, &shape, BooleanOp::Or, area, &samples);
            assert_boolean_area_and_samples(&shape, &shape, BooleanOp::And, area, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Xor, 0.0, 0, 0, &samples);
        }

        #[test]
        fn translated_overlapping_rectangles_preserve_boolean_results(
            (a, b) in overlapping_rect_pair_strategy(),
            dx in -200i32..200,
            dy in -200i32..200,
        ) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let dx = f64::from(dx);
            let dy = f64::from(dy);
            let translated_a = create_shape_rects(&[translate_rect(a, dx, dy)]);
            let translated_b = create_shape_rects(&[translate_rect(b, dx, dy)]);
            let samples = [
                rect_center(a),
                rect_center(b),
                outside_sample(b),
                ((a.0.max(b.0) + a.2.min(b.2)) * 0.5, (a.1.max(b.1) + a.3.min(b.3)) * 0.5),
            ];

            for op in [BooleanOp::Or, BooleanOp::And, BooleanOp::Not, BooleanOp::Xor] {
                let base = a_shape.boolean(&b_shape, op);
                let translated = translated_a.boolean(&translated_b, op);
                assert_shape_valid(&base);
                assert_shape_valid(&translated);
                assert!(
                    shape_signed_area(&base).fuzzy_eq_eps(shape_signed_area(&translated), 1e-5),
                    "translation changed area for {op:?}"
                );

                for (x, y) in samples {
                    assert_eq!(
                        shape_contains(&base, x, y),
                        shape_contains(&translated, x + dx, y + dy),
                        "translation changed sampled membership for {op:?}"
                    );
                }
            }
        }

        #[test]
        fn scaled_overlapping_rectangles_scale_result_area(
            (a, b) in overlapping_rect_pair_strategy(),
            scale in 2i32..8,
        ) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let scale = f64::from(scale);
            let mut scaled_a = a_shape.clone();
            let mut scaled_b = b_shape.clone();
            scaled_a.scale_mut(scale);
            scaled_b.scale_mut(scale);

            for op in [BooleanOp::Or, BooleanOp::And, BooleanOp::Not, BooleanOp::Xor] {
                let base = a_shape.boolean(&b_shape, op);
                let scaled = scaled_a.boolean(&scaled_b, op);
                assert_shape_valid(&base);
                assert_shape_valid(&scaled);
                assert!(
                    (shape_signed_area(&base) * scale * scale)
                        .fuzzy_eq_eps(shape_signed_area(&scaled), 1e-4),
                    "uniform scaling changed area incorrectly for {op:?}"
                );
            }
        }
    }
}

#[test]
fn shape_boolean_or_multi_island_rectangle_set_reduced_proptest_failure() {
    let a_rects = [
        (-75.0, -74.0, -66.0, -73.0),
        (-64.0, -74.0, -60.0, -73.0),
        (-52.0, -74.0, -44.0, -61.0),
        (-38.0, -74.0, -19.0, -65.0),
    ];
    let b_rects = [
        (-60.0, -65.0, -38.0, -42.0),
        (-28.0, -65.0, -20.0, -45.0),
        (-9.0, -65.0, 5.0, -61.0),
    ];
    let a_shape = create_shape_rects(&a_rects);
    let b_shape = create_shape_rects(&b_rects);
    let union = a_shape.boolean(&b_shape, BooleanOp::Or);

    assert_shape_valid(&union);
    assert_boolean_samples(
        &a_shape,
        &b_shape,
        &union,
        BooleanOp::Or,
        &[(-44.37, -61.53)],
    );
    assert!(
        shape_contains(&union, -44.37, -61.53),
        "OR dropped material inside the third A island"
    );
}

#[cfg(test)]
mod shape_boolean_differential_tests {
    //! Differential tests that avoid using `Shape` winding numbers as the expected oracle.
    //!
    //! These checks compare shape boolean results against small independent oracles so they can
    //! catch cases where the normal sampled `Shape` membership helper shares an implementation
    //! assumption with the code under test.

    use super::*;

    type GeoCoord = (f64, f64);
    type GeoRing = Vec<GeoCoord>;
    type GeoPolygonCase = (GeoRing, Vec<GeoRing>);
    type GeoMultiPolygonCase = Vec<GeoPolygonCase>;

    /// Independent axis-aligned rectangle used by the manual point-membership oracle.
    #[derive(Clone, Copy)]
    struct OracleRect {
        xmin: f64,
        ymin: f64,
        xmax: f64,
        ymax: f64,
    }

    impl OracleRect {
        /// Use strict interior membership to avoid treating boundary samples as oracle failures.
        fn contains(self, x: f64, y: f64) -> bool {
            x > self.xmin && x < self.xmax && y > self.ymin && y < self.ymax
        }

        /// Convert to the rectangle tuple accepted by the normal shape test constructors.
        fn tuple(self) -> (f64, f64, f64, f64) {
            (self.xmin, self.ymin, self.xmax, self.ymax)
        }
    }

    /// Evaluate membership in a union of independent oracle rectangles.
    fn oracle_contains(rects: &[OracleRect], x: f64, y: f64) -> bool {
        rects.iter().any(|rect| rect.contains(x, y))
    }

    /// Apply the requested boolean operation to independent membership bits.
    fn oracle_boolean(in_a: bool, in_b: bool, op: BooleanOp) -> bool {
        match op {
            BooleanOp::Or => in_a || in_b,
            BooleanOp::And => in_a && in_b,
            BooleanOp::Not => in_a && !in_b,
            BooleanOp::Xor => in_a != in_b,
        }
    }

    #[test]
    fn manual_rectangle_set_oracle_matches_shape_boolean_samples() {
        // This deliberately avoids Shape winding numbers for the expected value. It compares
        // result membership against an independent axis-aligned rectangle-set oracle, while arc
        // cases remain covered by sampled membership until an arc-capable oracle is added.
        let a_rects = [
            OracleRect {
                xmin: 0.0,
                ymin: 0.0,
                xmax: 10.0,
                ymax: 10.0,
            },
            OracleRect {
                xmin: 14.0,
                ymin: 2.0,
                xmax: 22.0,
                ymax: 12.0,
            },
        ];
        let b_rects = [
            OracleRect {
                xmin: 5.0,
                ymin: -2.0,
                xmax: 18.0,
                ymax: 8.0,
            },
            OracleRect {
                xmin: 25.0,
                ymin: 0.0,
                xmax: 30.0,
                ymax: 5.0,
            },
        ];
        let a = create_shape_rects(&a_rects.map(OracleRect::tuple));
        let b = create_shape_rects(&b_rects.map(OracleRect::tuple));

        for op in [
            BooleanOp::Or,
            BooleanOp::And,
            BooleanOp::Not,
            BooleanOp::Xor,
        ] {
            let result = a.boolean(&b, op);
            assert_shape_valid(&result);

            for ix in -4..36 {
                for iy in -6..18 {
                    let x = f64::from(ix) + 0.37;
                    let y = f64::from(iy) + 0.61;
                    let expected = oracle_boolean(
                        oracle_contains(&a_rects, x, y),
                        oracle_contains(&b_rects, x, y),
                        op,
                    );
                    assert_eq!(
                        shape_contains(&result, x, y),
                        expected,
                        "rectangle oracle mismatch for {op:?} at ({x}, {y})"
                    );
                }
            }
        }
    }

    #[derive(Clone)]
    struct GeoBooleanCase {
        name: &'static str,
        a_polys: GeoMultiPolygonCase,
        b_polys: GeoMultiPolygonCase,
        area_abs_eps: f64,
    }

    fn normalized_closed_loop(coords: &[GeoCoord], want_ccw: bool) -> Polyline<f64> {
        let mut pline = Polyline::new_closed();
        let end = coords.len().saturating_sub(1);
        for &(x, y) in &coords[..end] {
            pline.add(x, y, 0.0);
        }

        let is_ccw = pline.orientation() == PlineOrientation::CounterClockwise;
        if is_ccw != want_ccw {
            pline.invert_direction_mut();
        }
        pline
    }

    fn shape_from_geo_case_polys(polys: &[GeoPolygonCase]) -> Shape<f64> {
        Shape::from_plines(polys.iter().flat_map(|(shell, holes)| {
            std::iter::once(normalized_closed_loop(shell, true))
                .chain(holes.iter().map(|hole| normalized_closed_loop(hole, false)))
        }))
    }

    fn geo_polygon_from_case(shell: &[GeoCoord], holes: &[GeoRing]) -> geo::Polygon<f64> {
        geo::Polygon::new(
            geo::LineString::from(shell.to_vec()),
            holes
                .iter()
                .map(|hole| geo::LineString::from(hole.clone()))
                .collect(),
        )
    }

    fn geo_multipolygon_from_case(polys: &[GeoPolygonCase]) -> geo::MultiPolygon<f64> {
        geo::MultiPolygon::new(
            polys
                .iter()
                .map(|(shell, holes)| geo_polygon_from_case(shell, holes))
                .collect(),
        )
    }

    fn point_on_shape_boundary(shape: &Shape<f64>, x: f64, y: f64) -> bool {
        let point = Vector2::new(x, y);
        shape
            .ccw_plines
            .iter()
            .chain(shape.cw_plines.iter())
            .filter(|ip| ip.polyline.is_closed())
            .any(|ip| {
                ip.polyline.iter_segments().any(|(v1, v2)| {
                    seg_closest_point(v1, v2, point, SHAPE_TEST_EPS)
                        .fuzzy_eq_eps(point, SHAPE_TEST_EPS)
                })
            })
    }

    fn assert_shape_matches_geo_boolean(case: &GeoBooleanCase, op: BooleanOp) {
        use geo::{Area, BooleanOps, Contains};

        let a_shape = shape_from_geo_case_polys(&case.a_polys);
        let b_shape = shape_from_geo_case_polys(&case.b_polys);
        let a_geo = geo_multipolygon_from_case(&case.a_polys);
        let b_geo = geo_multipolygon_from_case(&case.b_polys);
        let expected_geo = match op {
            BooleanOp::Or => a_geo.union(&b_geo),
            BooleanOp::And => a_geo.intersection(&b_geo),
            BooleanOp::Not => a_geo.difference(&b_geo),
            BooleanOp::Xor => a_geo.xor(&b_geo),
        };
        let result = a_shape.boolean(&b_shape, op);

        trace_shape_boolean_case(&a_shape, &b_shape, &result, op);
        assert_shape_valid(&result);
        let expected_area = expected_geo.unsigned_area();
        let actual_area = shape_signed_area(&result).abs();
        let area_eps = (expected_area.abs().max(actual_area.abs()) * 1e-8).max(case.area_abs_eps);
        assert!(
            actual_area.fuzzy_eq_eps(expected_area, area_eps),
            "{} {op:?} area mismatch: expected {expected_area}, got {actual_area}",
            case.name
        );

        for (x, y) in dense_samples_for_shapes(&[&a_shape, &b_shape, &result], 18, 18) {
            if point_on_shape_boundary(&a_shape, x, y)
                || point_on_shape_boundary(&b_shape, x, y)
                || point_on_shape_boundary(&result, x, y)
            {
                continue;
            }

            let point = geo::Point::new(x, y);
            assert_eq!(
                shape_contains(&result, x, y),
                expected_geo.contains(&point),
                "{} {op:?} membership mismatch at ({x}, {y})",
                case.name
            );
        }
    }

    fn geo_boolean_regression_cases() -> Vec<GeoBooleanCase> {
        let mut cases = vec![
            GeoBooleanCase {
                // Adapted from geo 0.32.0 bool_ops::gh_issue_867.
                name: "geo gh-867 close triangle intersection",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (17.724912058920285, -16.37118892052372),
                        (18.06452454246989, -17.693907532504),
                        (19.09389292605319, -17.924001641855178),
                        (17.724912058920285, -16.37118892052372),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (17.576085274796423, -15.791540153598898),
                        (17.19432983818328, -17.499393422066746),
                        (18.06452454246989, -17.693907532504),
                        (17.576085274796423, -15.791540153598898),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from geo 0.32.0 bool_ops::gh_issue_885.
                name: "geo gh-885 near coincident quadrilaterals",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (8055.658, 7977.5537),
                        (8010.734, 7999.9697),
                        (8032.9717, 8044.537),
                        (8077.896, 8022.121),
                        (8055.658, 7977.5537),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (8055.805, 7977.847),
                        (8010.871, 8000.2676),
                        (8033.105, 8044.8286),
                        (8078.039, 8022.408),
                        (8055.805, 7977.847),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from geo 0.32.0 bool_ops::gh_issue_913::test_polygon_union2.
                name: "geo gh-913 triangle union",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (204.0, 287.0),
                        (206.69670020700084, 288.2213844497616),
                        (200.38308697914755, 288.338793163584),
                        (204.0, 287.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (210.0, 290.0),
                        (204.07584923592933, 288.2701221108328),
                        (212.24082541367974, 285.47846008552216),
                        (210.0, 290.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from geo 0.32.0 bool_ops::gh_issue_976.
                name: "geo gh-976 shifted polygon with hole",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (931229.0, 412646.0),
                        (931238.0, 412646.0),
                        (931238.0, 412639.0),
                        (931229.0, 412639.0),
                        (931229.0, 412646.0),
                    ],
                    vec![vec![
                        (931232.0, 412645.0),
                        (931237.0, 412645.0),
                        (931237.0, 412644.0),
                        (931235.0, 412642.0),
                        (931235.0, 412641.0),
                        (931230.0, 412643.0),
                        (931232.0, 412645.0),
                    ]],
                )],
                b_polys: vec![(
                    vec![
                        (931230.0, 412642.0),
                        (931236.0, 412644.0),
                        (931234.0, 412640.0),
                        (931230.0, 412642.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from geo 0.32.0 bool_ops::gh_issue_1193.
                name: "geo gh-1193 multipolygon sliver difference",
                area_abs_eps: 1e-4,
                a_polys: vec![
                    (
                        vec![
                            (-19.064932, -6.57369),
                            (-19.458324, -3.6231885),
                            (-22.058823, -3.6231885),
                            (-19.064932, -6.57369),
                        ],
                        vec![],
                    ),
                    (
                        vec![
                            (-14.705882, -10.869565),
                            (-14.705882, -7.649791),
                            (-17.60358, -8.013862),
                            (-14.705882, -10.869565),
                        ],
                        vec![],
                    ),
                ],
                b_polys: vec![(
                    vec![
                        (-18.852, -8.170715),
                        (-16.761898, -24.659603),
                        (43.387707, -16.298937),
                        (26.434301, -2.4808762),
                        (-18.852, -8.170715),
                    ],
                    vec![],
                )],
            },
        ];

        cases.extend([
            GeoBooleanCase {
                // Adapted from i_overlay 4.0.7 tests/boolean/test_2.json.
                name: "iOverlay test-2 half-overlapping squares",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (-10240.0, -10240.0),
                        (-10240.0, 10240.0),
                        (10240.0, 10240.0),
                        (10240.0, -10240.0),
                        (-10240.0, -10240.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (0.0, -10240.0),
                        (0.0, 10240.0),
                        (20480.0, 10240.0),
                        (20480.0, -10240.0),
                        (0.0, -10240.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from i_overlay 4.0.7 tests/boolean/test_15.json.
                name: "iOverlay test-15 containment creates hole",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (-20480.0, -20480.0),
                        (-20480.0, 20480.0),
                        (20480.0, 20480.0),
                        (20480.0, -20480.0),
                        (-20480.0, -20480.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (-5120.0, -5120.0),
                        (-5120.0, 5120.0),
                        (5120.0, 5120.0),
                        (5120.0, -5120.0),
                        (-5120.0, -5120.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from i_overlay 4.0.7 tests/boolean/test_36.json.
                name: "iOverlay test-36 diamond covers square",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (-10240.0, -10240.0),
                        (-10240.0, 10240.0),
                        (10240.0, 10240.0),
                        (10240.0, -10240.0),
                        (-10240.0, -10240.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (0.0, -20480.0),
                        (-20480.0, 0.0),
                        (0.0, 20480.0),
                        (20480.0, 0.0),
                        (0.0, -20480.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from i_overlay 4.0.7 tests/boolean/test_47.json.
                name: "iOverlay test-47 triangle touches square boundary",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (-10240.0, 10240.0),
                        (10240.0, 10240.0),
                        (10240.0, -10240.0),
                        (-10240.0, -10240.0),
                        (-10240.0, 10240.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (-5120.0, 10240.0),
                        (5120.0, 10240.0),
                        (0.0, 0.0),
                        (-5120.0, 10240.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                // Adapted from i_overlay 4.0.7 tests/boolean/test_74.json.
                name: "iOverlay test-74 near-boundary sliver cover",
                area_abs_eps: 1.0,
                a_polys: vec![(
                    vec![
                        (-10240.0, 10240.0),
                        (10240.0, 10240.0),
                        (10240.0, 0.0),
                        (-10240.0, 0.0),
                        (-10240.0, 10240.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (10239.0, 30720.0),
                        (10241.0, -30720.0),
                        (-15360.0, -30720.0),
                        (-15360.0, 30720.0),
                        (10239.0, 30720.0),
                    ],
                    vec![],
                )],
            },
        ]);

        cases.extend([
            GeoBooleanCase {
                name: "brutal point-touching polygons produce no area intersection",
                area_abs_eps: 1e-9,
                a_polys: vec![(
                    vec![
                        (0.0, 0.0),
                        (10.0, 0.0),
                        (10.0, 10.0),
                        (0.0, 10.0),
                        (0.0, 0.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![(10.0, 10.0), (16.0, 10.0), (13.0, 15.0), (10.0, 10.0)],
                    vec![],
                )],
            },
            GeoBooleanCase {
                name: "brutal shared edge regularization",
                area_abs_eps: 1e-9,
                a_polys: vec![(
                    vec![
                        (0.0, 0.0),
                        (10.0, 0.0),
                        (10.0, 10.0),
                        (0.0, 10.0),
                        (0.0, 0.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (10.0, 0.0),
                        (20.0, 0.0),
                        (20.0, 10.0),
                        (10.0, 10.0),
                        (10.0, 0.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                name: "brutal epsilon-width overlap",
                // The oracle and the shape result agree on the sliver geometry; the remaining area
                // delta is floating-point representation noise on a 1e-6-wide overlap.
                area_abs_eps: 2e-8,
                a_polys: vec![(
                    vec![
                        (0.0, 0.0),
                        (10.0, 0.0),
                        (10.0, 10.0),
                        (0.0, 10.0),
                        (0.0, 0.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (9.999_999, -1.0),
                        (20.0, -1.0),
                        (20.0, 11.0),
                        (9.999_999, 11.0),
                        (9.999_999, -1.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                name: "brutal hole-edge tangent bridge",
                area_abs_eps: 1e-7,
                a_polys: vec![(
                    vec![
                        (0.0, 0.0),
                        (30.0, 0.0),
                        (30.0, 30.0),
                        (0.0, 30.0),
                        (0.0, 0.0),
                    ],
                    vec![vec![
                        (10.0, 10.0),
                        (20.0, 10.0),
                        (20.0, 20.0),
                        (10.0, 20.0),
                        (10.0, 10.0),
                    ]],
                )],
                b_polys: vec![(
                    vec![
                        (20.0, 12.0),
                        (27.0, 12.0),
                        (27.0, 18.0),
                        (20.0, 18.0),
                        (20.0, 12.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                name: "brutal collinear vertices and notch crossing",
                area_abs_eps: 1e-7,
                a_polys: vec![(
                    vec![
                        (0.0, 0.0),
                        (4.0, 0.0),
                        (8.0, 0.0),
                        (8.0, 8.0),
                        (5.0, 8.0),
                        (5.0, 3.0),
                        (3.0, 3.0),
                        (3.0, 8.0),
                        (0.0, 8.0),
                        (0.0, 0.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (-1.0, 2.0),
                        (9.0, 2.0),
                        (9.0, 6.0),
                        (-1.0, 6.0),
                        (-1.0, 2.0),
                    ],
                    vec![],
                )],
            },
            GeoBooleanCase {
                name: "brutal large coordinates with tiny offset",
                area_abs_eps: 1e-4,
                a_polys: vec![(
                    vec![
                        (1_000_000_000.0, 1_000_000_000.0),
                        (1_000_000_010.0, 1_000_000_000.0),
                        (1_000_000_010.0, 1_000_000_010.0),
                        (1_000_000_000.0, 1_000_000_010.0),
                        (1_000_000_000.0, 1_000_000_000.0),
                    ],
                    vec![],
                )],
                b_polys: vec![(
                    vec![
                        (1_000_000_009.5, 999_999_999.5),
                        (1_000_000_020.0, 999_999_999.5),
                        (1_000_000_020.0, 1_000_000_010.5),
                        (1_000_000_009.5, 1_000_000_010.5),
                        (1_000_000_009.5, 999_999_999.5),
                    ],
                    vec![],
                )],
            },
        ]);

        cases
    }

    #[test]
    fn geo_boolean_regression_cases_match_geo_oracle() {
        for case in geo_boolean_regression_cases() {
            if case.name.starts_with("brutal ") {
                continue;
            }

            for op in [
                BooleanOp::Or,
                BooleanOp::And,
                BooleanOp::Not,
                BooleanOp::Xor,
            ] {
                assert_shape_matches_geo_boolean(&case, op);
            }
        }
    }

    #[test]
    fn brutal_geo_boolean_singularity_cases_match_geo_oracle() {
        for case in geo_boolean_regression_cases()
            .into_iter()
            .filter(|case| case.name.starts_with("brutal "))
        {
            for op in [
                BooleanOp::Or,
                BooleanOp::And,
                BooleanOp::Not,
                BooleanOp::Xor,
            ] {
                assert_shape_matches_geo_boolean(&case, op);
            }
        }
    }
}

#[test]
fn shape_empty_union_returns_self() {
    let shape_empty = Shape::<f64>::empty();
    let shape_nonempty = {
        let mut p = Polyline::new_closed();
        p.add(0.0, 0.0, 0.0);
        p.add(10.0, 0.0, 0.0);
        p.add(10.0, 10.0, 0.0);
        p.add(0.0, 10.0, 0.0);
        Shape::from_plines(vec![p])
    };
    let union_result = shape_empty.union(&shape_nonempty);
    // Expect same shape as `shape_nonempty`:
    assert_eq!(
        union_result.ccw_plines.len(),
        shape_nonempty.ccw_plines.len()
    );
    assert_eq!(union_result.cw_plines.len(), shape_nonempty.cw_plines.len());
}

#[test]
fn shape_disjoint_union_keeps_both() {
    // Two squares far apart, union => 2 loops in resulting shape
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1]);
    let shape2 = Shape::from_plines(vec![square2]);

    let union_result = shape1.union(&shape2);
    // Should have 2 separate loops, both ccw:
    assert_eq!(union_result.ccw_plines.len(), 2);
    assert!(union_result.cw_plines.is_empty());
}

#[test]
fn shape_contains_shape_union_gives_first() {
    // Big square fully containing a smaller square => union is just bigger square
    let big_sq = pline_closed![
        (0.0, 0.0, 0.0),
        (100.0, 0.0, 0.0),
        (100.0, 100.0, 0.0),
        (0.0, 100.0, 0.0)
    ];
    let small_sq = pline_closed![
        (10.0, 10.0, 0.0),
        (20.0, 10.0, 0.0),
        (20.0, 20.0, 0.0),
        (10.0, 20.0, 0.0)
    ];
    let shape_big = Shape::from_plines(vec![big_sq.clone()]);
    let shape_small = Shape::from_plines(vec![small_sq]);

    let union_result = shape_big.union(&shape_small);
    // Should have same bounding box and area as `big_sq`
    assert_eq!(union_result.ccw_plines.len(), 1);
    let union_pline = &union_result.ccw_plines[0].polyline;
    assert_fuzzy_eq!(union_pline.area(), big_sq.area());
}

#[test]
fn shape_difference_completely_disjoint_no_intersection() {
    // Two squares far apart => difference => same as first
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1.clone()]);
    let shape2 = Shape::from_plines(vec![square2]);
    let diff_result = shape1.difference(&shape2);
    assert_eq!(diff_result.ccw_plines.len(), 1);
    assert!(diff_result.cw_plines.is_empty());

    let out_pline = &diff_result.ccw_plines[0].polyline;
    assert_fuzzy_eq!(out_pline.area(), square1.area());
}

#[test]
fn shape_intersection_disjoint_is_empty() {
    // Two squares far apart => intersection => empty
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1]);
    let shape2 = Shape::from_plines(vec![square2]);

    let intersect_result = shape1.intersection(&shape2);
    // Should be empty
    assert!(intersect_result.ccw_plines.is_empty());
    assert!(intersect_result.cw_plines.is_empty());
}

#[test]
fn shape_open_pline_included_in_boolean() {
    // If shape has open polyline, those won't be closed => won't form loops
    // but let's just confirm no panic:
    let open_pl = pline_open![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![open_pl]);
    let shape_empty = Shape::<f64>::empty();
    let union_result = shape.union(&shape_empty);
    // union => same as shape
    // though the shape is "not fully closed," area = 0, etc.
    assert_eq!(union_result.ccw_plines.len(), shape.ccw_plines.len());
}

#[test]
fn shape_boolean_clips_open_line_against_area_for_all_ops_and_operand_orders() {
    let rect = create_rectangle(0.0, 0.0, 10.0, 10.0);
    let configurations = [
        ("crossing_horizontal", (-5.0, 5.0, 15.0, 5.0)),
        ("crossing_vertical", (5.0, -5.0, 5.0, 15.0)),
        ("crossing_diagonal", (-5.0, -5.0, 15.0, 15.0)),
        ("fully_inside", (2.0, 2.0, 8.0, 8.0)),
        ("fully_outside", (-8.0, 12.0, -2.0, 12.0)),
        ("left_endpoint_inside", (5.0, 5.0, 15.0, 5.0)),
        ("right_endpoint_inside", (-5.0, 5.0, 5.0, 5.0)),
        ("boundary_full_overlap", (0.0, 0.0, 10.0, 0.0)),
        ("boundary_partial_overlap", (-5.0, 0.0, 5.0, 0.0)),
        ("vertex_touch_only", (-5.0, -5.0, 0.0, 0.0)),
    ];

    for (name, line) in configurations {
        for (direction, line) in [
            ("forward", line),
            ("reverse", (line.2, line.3, line.0, line.1)),
        ] {
            let (inside, outside) = clipped_line_segments_against_unit_square(line);
            let line_shape =
                Shape::from_plines([create_line_segment(line.0, line.1, line.2, line.3)]);
            let rect_shape = Shape::from_plines([rect.clone()]);

            for op in [
                BooleanOp::Or,
                BooleanOp::And,
                BooleanOp::Not,
                BooleanOp::Xor,
            ] {
                let result = line_shape.boolean(&rect_shape, op);
                let expected_lines = match op {
                    BooleanOp::Or | BooleanOp::Not | BooleanOp::Xor => outside.as_slice(),
                    BooleanOp::And => inside.as_slice(),
                };
                let expected_area_count = matches!(op, BooleanOp::Or | BooleanOp::Xor) as usize;
                assert_eq!(
                    result
                        .ccw_plines
                        .iter()
                        .filter(|ip| ip.polyline.is_closed())
                        .count(),
                    expected_area_count,
                    "{name}/{direction}: line op rect area count for {op:?}"
                );
                assert_open_line_segments(&result, expected_lines);

                let result = rect_shape.boolean(&line_shape, op);
                let expected_lines = match op {
                    BooleanOp::Or | BooleanOp::Xor => outside.as_slice(),
                    BooleanOp::And => inside.as_slice(),
                    BooleanOp::Not => &[][..],
                };
                let expected_area_count =
                    matches!(op, BooleanOp::Or | BooleanOp::Not | BooleanOp::Xor) as usize;
                assert_eq!(
                    result
                        .ccw_plines
                        .iter()
                        .filter(|ip| ip.polyline.is_closed())
                        .count(),
                    expected_area_count,
                    "{name}/{direction}: rect op line area count for {op:?}"
                );
                assert_open_line_segments(&result, expected_lines);
            }
        }
    }
}

#[test]
fn shape_boolean_clips_open_lines_through_holes_and_multi_segment_paths() {
    let donut = create_donut((0.0, 0.0, 10.0, 10.0), &[(4.0, 4.0, 6.0, 6.0)]);
    let line_shape = Shape::from_plines([
        create_line_segment(-2.0, 5.0, 12.0, 5.0),
        pline_open![
            (-2.0, 2.0, 0.0),
            (2.0, 2.0, 0.0),
            (8.0, 8.0, 0.0),
            (12.0, 8.0, 0.0)
        ],
    ]);

    let inside = [
        (0.0, 5.0, 4.0, 5.0),
        (6.0, 5.0, 10.0, 5.0),
        (0.0, 2.0, 2.0, 2.0),
        (2.0, 2.0, 4.0, 4.0),
        (6.0, 6.0, 8.0, 8.0),
        (8.0, 8.0, 10.0, 8.0),
    ];
    let outside = [
        (-2.0, 5.0, 0.0, 5.0),
        (4.0, 5.0, 6.0, 5.0),
        (10.0, 5.0, 12.0, 5.0),
        (-2.0, 2.0, 0.0, 2.0),
        (4.0, 4.0, 6.0, 6.0),
        (10.0, 8.0, 12.0, 8.0),
    ];

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        let result = line_shape.boolean(&donut, op);
        let expected_lines = match op {
            BooleanOp::Or | BooleanOp::Not | BooleanOp::Xor => outside.as_slice(),
            BooleanOp::And => inside.as_slice(),
        };
        let expected_area_count = matches!(op, BooleanOp::Or | BooleanOp::Xor) as usize;
        assert_eq!(
            result
                .ccw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            expected_area_count,
            "line op donut area count for {op:?}"
        );
        assert_open_line_segments(&result, expected_lines);

        let result = donut.boolean(&line_shape, op);
        let expected_lines = match op {
            BooleanOp::Or | BooleanOp::Xor => outside.as_slice(),
            BooleanOp::And => inside.as_slice(),
            BooleanOp::Not => &[][..],
        };
        let expected_ccw_count =
            matches!(op, BooleanOp::Or | BooleanOp::Not | BooleanOp::Xor) as usize;
        let expected_cw_count = expected_ccw_count;
        assert_eq!(
            result
                .ccw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            expected_ccw_count,
            "donut op line ccw area count for {op:?}"
        );
        assert_eq!(
            result
                .cw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            expected_cw_count,
            "donut op line cw area count for {op:?}"
        );
        assert_open_line_segments(&result, expected_lines);
    }
}

#[test]
fn shape_boolean_or_removes_open_triangle_fully_inside_filled_area() {
    let rect_shape = Shape::from_plines([create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let triangle_line_shape = Shape::from_plines([pline_open![
        (2.0, 2.0, 0.0),
        (8.0, 2.0, 0.0),
        (5.0, 8.0, 0.0),
        (2.0, 2.0, 0.0)
    ]]);

    let result = rect_shape.boolean(&triangle_line_shape, BooleanOp::Or);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_open_line_segments(&result, &[]);

    let result = triangle_line_shape.boolean(&rect_shape, BooleanOp::Or);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_open_line_segments(&result, &[]);
}

#[test]
fn shape_boolean_or_clips_line_path_entering_filled_area_from_outside() {
    let rect_shape = Shape::from_plines([create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let line_shape = Shape::from_plines([pline_open![
        (-4.0, 5.0, 0.0),
        (4.0, 5.0, 0.0),
        (6.0, 8.0, 0.0)
    ]]);

    let result = rect_shape.boolean(&line_shape, BooleanOp::Or);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_open_line_segments(&result, &[(-4.0, 5.0, 0.0, 5.0)]);

    let result = line_shape.boolean(&rect_shape, BooleanOp::Or);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_open_line_segments(&result, &[(-4.0, 5.0, 0.0, 5.0)]);
}

#[test]
fn shape_boolean_or_absorbs_closed_triangle_fully_inside_filled_area() {
    let rect_shape = Shape::from_plines([create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let ccw_triangle = pline_closed![(2.0, 2.0, 0.0), (8.0, 2.0, 0.0), (5.0, 8.0, 0.0)];
    let mut cw_triangle = ccw_triangle.clone();
    cw_triangle.invert_direction_mut();

    for triangle in [ccw_triangle, cw_triangle] {
        let triangle_shape = Shape::from_plines([triangle]);

        let result = rect_shape.boolean(&triangle_shape, BooleanOp::Or);
        assert_eq!(
            result
                .ccw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            1
        );
        assert_eq!(
            result
                .cw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            0
        );
        assert_fuzzy_eq!(shape_signed_area(&result), 100.0);

        let result = triangle_shape.boolean(&rect_shape, BooleanOp::Or);
        assert_eq!(
            result
                .ccw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            1
        );
        assert_eq!(
            result
                .cw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            0
        );
        assert_fuzzy_eq!(shape_signed_area(&result), 100.0);
    }
}

#[test]
fn shape_boolean_or_removes_open_linework_inside_same_operand_fill() {
    let shape_with_internal_linework = Shape::from_plines([
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        pline_open![
            (2.0, 2.0, 0.0),
            (8.0, 2.0, 0.0),
            (5.0, 8.0, 0.0),
            (2.0, 2.0, 0.0)
        ],
        create_line_segment(-4.0, 5.0, 4.0, 5.0),
    ]);
    let disjoint_shape = Shape::from_plines([create_rectangle(20.0, 20.0, 30.0, 30.0)]);

    let result = shape_with_internal_linework.boolean(&disjoint_shape, BooleanOp::Or);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        2
    );
    assert_open_line_segments(&result, &[(-4.0, 5.0, 0.0, 5.0)]);

    let result = disjoint_shape.boolean(&shape_with_internal_linework, BooleanOp::Or);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        2
    );
    assert_open_line_segments(&result, &[(-4.0, 5.0, 0.0, 5.0)]);
}

#[test]
fn shape_boolean_and_clips_closed_polyline_loop_to_filled_intersection() {
    let rect_shape = Shape::from_plines([create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let triangle_shape = Shape::from_plines([pline_closed![
        (-2.0, 5.0, 0.0),
        (12.0, 5.0, 0.0),
        (5.0, 12.0, 0.0)
    ]]);

    for result in [
        rect_shape.boolean(&triangle_shape, BooleanOp::And),
        triangle_shape.boolean(&rect_shape, BooleanOp::And),
    ] {
        assert_eq!(
            result
                .ccw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            1
        );
        assert_eq!(
            result
                .cw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            0
        );
        assert_open_line_segments(&result, &[]);

        let bounds = result.plines_index.bounds().unwrap();
        assert!(bounds.min_x >= -SHAPE_TEST_EPS, "min_x={}", bounds.min_x);
        assert!(
            bounds.max_x <= 10.0 + SHAPE_TEST_EPS,
            "max_x={}",
            bounds.max_x
        );
        assert!(
            bounds.min_y >= 5.0 - SHAPE_TEST_EPS,
            "min_y={}",
            bounds.min_y
        );
        assert!(
            bounds.max_y <= 10.0 + SHAPE_TEST_EPS,
            "max_y={}",
            bounds.max_y
        );
        assert!(shape_signed_area(&result) > 0.0);
        assert!(shape_signed_area(&result) < 49.0);
    }
}

#[test]
fn shape_boolean_and_does_not_keep_filled_area_when_intersecting_open_lines() {
    let filled = Shape::from_plines([create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let open_lines = Shape::from_plines([
        create_line_segment(-2.0, 5.0, 12.0, 5.0),
        pline_open![
            (-2.0, 2.0, 0.0),
            (2.0, 2.0, 0.0),
            (8.0, 8.0, 0.0),
            (12.0, 8.0, 0.0)
        ],
    ]);
    let expected = [
        (0.0, 5.0, 10.0, 5.0),
        (0.0, 2.0, 2.0, 2.0),
        (2.0, 2.0, 8.0, 8.0),
        (8.0, 8.0, 10.0, 8.0),
    ];

    for result in [
        filled.boolean(&open_lines, BooleanOp::And),
        open_lines.boolean(&filled, BooleanOp::And),
    ] {
        assert_eq!(
            result
                .ccw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            0
        );
        assert_eq!(
            result
                .cw_plines
                .iter()
                .filter(|ip| ip.polyline.is_closed())
                .count(),
            0
        );
        assert_open_line_segments(&result, &expected);
    }
}

#[test]
fn shape_boolean_and_uses_open_lines_only_where_both_operands_are_filled() {
    let lhs = Shape::from_plines([
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_line_segment(-2.0, 5.0, 12.0, 5.0),
    ]);
    let rhs = Shape::from_plines([
        create_rectangle(4.0, -2.0, 14.0, 8.0),
        create_line_segment(6.0, -4.0, 6.0, 12.0),
    ]);

    let result = lhs.boolean(&rhs, BooleanOp::And);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_eq!(
        result
            .cw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        0
    );
    assert_open_line_segments(&result, &[(10.0, 5.0, 12.0, 5.0), (6.0, 8.0, 6.0, 10.0)]);
    assert_fuzzy_eq!(shape_signed_area(&result), 48.0);
}

#[test]
fn shape_boolean_not_clips_lines_inside_subtracted_filled_boundary() {
    let lhs = Shape::from_plines([
        create_line_segment(-2.0, 5.0, 12.0, 5.0),
        create_line_segment(5.0, -2.0, 5.0, 12.0),
    ]);
    let rhs = Shape::from_plines([create_rectangle(0.0, 0.0, 10.0, 10.0)]);

    let result = lhs.boolean(&rhs, BooleanOp::Not);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        0
    );
    assert_eq!(
        result
            .cw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        0
    );
    assert_open_line_segments(
        &result,
        &[
            (-2.0, 5.0, 0.0, 5.0),
            (10.0, 5.0, 12.0, 5.0),
            (5.0, -2.0, 5.0, 0.0),
            (5.0, 10.0, 5.0, 12.0),
        ],
    );
}

#[test]
fn shape_boolean_not_removes_lines_inside_remaining_same_operand_fill() {
    let lhs = Shape::from_plines([
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        pline_open![
            (2.0, 2.0, 0.0),
            (8.0, 2.0, 0.0),
            (5.0, 8.0, 0.0),
            (2.0, 2.0, 0.0)
        ],
        create_line_segment(-2.0, 5.0, 12.0, 5.0),
    ]);
    let rhs = Shape::from_plines([create_rectangle(20.0, 20.0, 30.0, 30.0)]);

    let result = lhs.boolean(&rhs, BooleanOp::Not);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_eq!(
        result
            .cw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        0
    );
    assert_open_line_segments(&result, &[(-2.0, 5.0, 0.0, 5.0), (10.0, 5.0, 12.0, 5.0)]);
}

#[test]
fn shape_boolean_not_clips_lines_against_holes_in_remaining_area() {
    let lhs = create_donut((0.0, 0.0, 10.0, 10.0), &[(4.0, 4.0, 6.0, 6.0)]);
    let lhs = Shape::from_plines(
        lhs.ccw_plines
            .into_iter()
            .chain(lhs.cw_plines)
            .map(|ip| ip.polyline)
            .chain([create_line_segment(-2.0, 5.0, 12.0, 5.0)]),
    );
    let rhs = Shape::<f64>::empty();

    let result = lhs.boolean(&rhs, BooleanOp::Not);

    assert_eq!(
        result
            .ccw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_eq!(
        result
            .cw_plines
            .iter()
            .filter(|ip| ip.polyline.is_closed())
            .count(),
        1
    );
    assert_open_line_segments(
        &result,
        &[
            (-2.0, 5.0, 0.0, 5.0),
            (4.0, 5.0, 6.0, 5.0),
            (10.0, 5.0, 12.0, 5.0),
        ],
    );
}

#[test]
fn shape_boolean_open_linework_is_clipped_out_of_final_material_for_adversarial_modes() {
    let donut = create_donut(
        (0.0, 0.0, 20.0, 20.0),
        &[(4.0, 4.0, 7.0, 16.0), (11.0, 3.0, 16.0, 8.0)],
    );
    let lhs = Shape::from_plines(
        donut
            .ccw_plines
            .into_iter()
            .chain(donut.cw_plines)
            .map(|ip| ip.polyline)
            .chain([
                create_line_segment(-3.0, 10.0, 23.0, 10.0),
                create_line_segment(2.0, 2.0, 18.0, 18.0),
                pline_open![
                    (6.0, 5.0, 0.0),
                    (14.0, 5.0, 0.0),
                    (18.0, 14.0, 0.0),
                    (6.0, 5.0, 0.0)
                ],
            ]),
    );
    let rhs = Shape::from_plines([
        create_rectangle(6.0, -2.0, 24.0, 14.0),
        create_line_segment(10.0, -4.0, 10.0, 24.0),
        create_line_segment(-2.0, 6.0, 24.0, 18.0),
    ]);

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        let result = lhs.boolean(&rhs, op);
        assert_open_linework_outside_closed_material(&result, &format!("lhs {op:?} rhs"));
        assert_boolean_samples(&lhs, &rhs, &result, op, &dense_sample_grid(-4.0, 26.0, 2.0));

        let result = rhs.boolean(&lhs, op);
        assert_open_linework_outside_closed_material(&result, &format!("rhs {op:?} lhs"));
        assert_boolean_samples(&rhs, &lhs, &result, op, &dense_sample_grid(-4.0, 26.0, 2.0));
    }
}

#[test]
fn shape_boolean_staggered_holes_dense_adversarial_all_ops() {
    let a = Shape::from_plines([
        create_rectangle(0.0, 0.0, 30.0, 24.0),
        create_cw_rectangle(4.0, 4.0, 10.0, 20.0),
        create_cw_rectangle(15.0, 3.0, 24.0, 11.0),
    ]);
    let b = Shape::from_plines([
        create_rectangle(8.0, -2.0, 28.0, 26.0),
        create_cw_rectangle(12.0, 2.0, 18.0, 18.0),
        create_cw_rectangle(20.0, 9.0, 26.0, 22.0),
    ]);
    let samples = dense_sample_grid(-2.0, 32.0, 1.25);

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        if matches!(op, BooleanOp::Not) {
            let result = a.boolean(&b, op);
            assert_shape_valid(&result);
            assert_boolean_samples(&a, &b, &result, op, &samples);
        } else {
            assert_commutative_samples(&a, &b, op, &samples);
        }
    }
}

#[test]
fn shape_boolean_shared_hole_boundaries_are_regularized() {
    let a = Shape::from_plines([
        create_rectangle(0.0, 0.0, 40.0, 20.0),
        create_cw_rectangle(5.0, 5.0, 15.0, 15.0),
        create_cw_rectangle(25.0, 5.0, 35.0, 15.0),
    ]);
    let b = Shape::from_plines([
        create_rectangle(10.0, -2.0, 30.0, 22.0),
        create_cw_rectangle(15.0, 5.0, 25.0, 15.0),
    ]);
    let samples = dense_sample_grid(-2.0, 42.0, 1.0);

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        let result = a.boolean(&b, op);
        assert_shape_valid(&result);
        assert_boolean_samples(&a, &b, &result, op, &samples);

        if !matches!(op, BooleanOp::Not) {
            let result = b.boolean(&a, op);
            assert_shape_valid(&result);
            assert_boolean_samples(&b, &a, &result, op, &samples);
        }
    }
}

#[test]
fn shape_boolean_near_coincident_island_in_hole_is_not_cancelled() {
    // Opposite-sign loops should cancel only when they are actually coincident. A too-loose
    // equality check fills this epsilon-wide moat between the donut hole and the inserted island.
    let moat = 1e-4;
    let donut = Shape::from_plines([
        create_rectangle(-10.0, -10.0, 10.0, 10.0),
        create_cw_rectangle(-5.0, -5.0, 5.0, 5.0),
    ]);
    let island = Shape::from_plines([create_rectangle(
        -5.0 + moat,
        -5.0 + moat,
        5.0 - moat,
        5.0 - moat,
    )]);
    let expected_island_area = (10.0 - 2.0 * moat) * (10.0 - 2.0 * moat);
    let samples = [
        (0.0, 0.0),
        (4.99995, 0.0),
        (5.00005, 0.0),
        (8.0, 0.0),
        (11.0, 0.0),
    ];

    assert_boolean_result(
        &donut,
        &island,
        BooleanOp::Or,
        300.0 + expected_island_area,
        2,
        1,
        &samples,
    );
    assert_boolean_result(&donut, &island, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::Not, 300.0, 1, 1, &samples);
    assert_boolean_result(
        &donut,
        &island,
        BooleanOp::Xor,
        300.0 + expected_island_area,
        2,
        1,
        &samples,
    );
}

#[test]
fn shape_boolean_large_coordinate_hole_overlap_keeps_area_and_membership() {
    // Large offsets used to expose cancellation in area calculations and lower-level clipping.
    // The expected values are intentionally small relative to the coordinate magnitude.
    let base = 1_000_000_000.0;
    let a = Shape::from_plines([
        create_rectangle(base, base, base + 100.0, base + 100.0),
        create_cw_rectangle(base + 20.0, base + 20.0, base + 80.0, base + 80.0),
    ]);
    let b = Shape::from_plines([create_rectangle(
        base + 50.0,
        base - 10.0,
        base + 120.0,
        base + 60.0,
    )]);
    let samples = [
        (base + 10.0, base + 10.0),
        (base + 40.0, base + 40.0),
        (base + 55.0, base + 10.0),
        (base + 55.0, base + 40.0),
        (base + 90.0, base + 50.0),
        (base + 110.0, base + 50.0),
    ];

    assert_boolean_area_and_samples(&a, &b, BooleanOp::Or, 9500.0, &samples);
    assert_boolean_area_and_samples(&a, &b, BooleanOp::And, 1800.0, &samples);
    assert_boolean_area_and_samples(&a, &b, BooleanOp::Not, 4600.0, &samples);
    assert_boolean_area_and_samples(&a, &b, BooleanOp::Xor, 7700.0, &samples);
}

#[test]
fn shape_boolean_ludicrous_closed_loop_corpus_all_ops() {
    let mut arc_ring_plines = vec![create_approx_circle(0.0, 0.0, 18.0), {
        let mut inner = create_approx_circle(0.0, 0.0, 7.0);
        inner.invert_direction_mut();
        inner
    }];
    arc_ring_plines.extend([
        create_rectangle(-35.0, -2.0, -18.0, 2.0),
        create_rectangle(18.0, -2.0, 35.0, 2.0),
    ]);

    let cases = vec![
        (
            "deep island-lake stack against shifted stack",
            create_deeper_nested_rect_shape(-60.0, -60.0, 120.0, 9),
            create_deeper_nested_rect_shape(-46.0, -37.0, 94.0, 8),
        ),
        (
            "staggered hole grid against material bars",
            Shape::from_plines([
                create_rectangle(0.0, 0.0, 90.0, 70.0),
                create_cw_rectangle(6.0, 5.0, 18.0, 20.0),
                create_cw_rectangle(26.0, 5.0, 38.0, 20.0),
                create_cw_rectangle(46.0, 5.0, 58.0, 20.0),
                create_cw_rectangle(14.0, 30.0, 28.0, 54.0),
                create_cw_rectangle(40.0, 28.0, 55.0, 58.0),
                create_cw_rectangle(66.0, 18.0, 82.0, 62.0),
            ]),
            Shape::from_plines([
                create_rectangle(-4.0, 12.0, 94.0, 24.0),
                create_rectangle(-4.0, 46.0, 94.0, 58.0),
                create_rectangle(20.0, -6.0, 32.0, 76.0),
                create_rectangle(60.0, -6.0, 72.0, 76.0),
            ]),
        ),
        (
            "arc donut tangent to chordal slivers",
            Shape::from_plines(arc_ring_plines),
            Shape::from_plines([
                create_rectangle(-25.0, -1.0, 25.0, 1.0),
                create_rectangle(-1.0, -25.0, 1.0, 25.0),
                create_rectangle(-8.0, -8.0, 8.0, 8.0),
            ]),
        ),
        (
            "interlocking sawtooth shells",
            Shape::from_plines([{
                let mut shell = create_sawtooth_loop(-40.0, -20.0, 80.0, 40.0, 11);
                shell.invert_direction_mut();
                shell
            }]),
            Shape::from_plines([{
                let mut shell = create_sawtooth_loop(-37.5, -18.0, 80.0, 40.0, 13);
                shell.invert_direction_mut();
                shell
            }]),
        ),
        (
            "near-coincident shared edges and epsilon slivers",
            Shape::from_plines([
                create_rectangle(0.0, 0.0, 20.0, 20.0),
                create_rectangle(20.000001, 0.0, 40.0, 20.0),
                create_cw_rectangle(8.0, 8.0, 12.0, 12.0),
            ]),
            Shape::from_plines([
                create_rectangle(10.0, -5.0, 30.000001, 25.0),
                create_cw_rectangle(18.0, 6.0, 22.0, 14.0),
            ]),
        ),
        (
            "ui multipolyline demo against translated clone",
            create_multi_pline_boolean_scene_default_shape(),
            {
                let mut shape = create_multi_pline_boolean_scene_default_shape();
                shape.translate_mut(17.0, -23.0);
                shape.rotate_mut(0.08);
                shape
            },
        ),
        (
            "nested arc rings against deep rectangular islands",
            {
                let ring = create_arc_ring(0.0, 0.0, 40.0, 28.0);
                Shape::from_plines(
                    ring.ccw_plines
                        .iter()
                        .chain(ring.cw_plines.iter())
                        .map(|ip| ip.polyline.clone())
                        .chain([create_approx_circle(0.0, 0.0, 14.0), {
                            let mut lake = create_approx_circle(0.0, 0.0, 6.0);
                            lake.invert_direction_mut();
                            lake
                        }]),
                )
            },
            create_deeper_nested_rect_shape(-30.0, -30.0, 60.0, 7),
        ),
    ];

    for (name, a, b) in cases {
        let samples = dense_samples_for_shapes(&[&a, &b], 14, 14);
        for op in [
            BooleanOp::Or,
            BooleanOp::And,
            BooleanOp::Not,
            BooleanOp::Xor,
        ] {
            let result = a.boolean(&b, op);
            assert_shape_output_finite(&result);
            assert_shape_valid(&result);
            assert_boolean_samples(&a, &b, &result, op, &samples);

            let reverse = b.boolean(&a, op);
            assert_shape_output_finite(&reverse);
            assert_shape_valid(&reverse);
            assert_boolean_samples(&b, &a, &reverse, op, &samples);
        }
        eprintln!("checked adversarial closed-loop corpus case: {name}");
    }
}

#[test]
fn shape_boolean_ludicrous_open_line_clipping_corpus_all_ops() {
    let cases = vec![
        (
            "hole maze with diagonal scribbles",
            Shape::from_plines([
                create_rectangle(0.0, 0.0, 80.0, 80.0),
                create_cw_rectangle(10.0, 10.0, 25.0, 70.0),
                create_cw_rectangle(34.0, 5.0, 48.0, 36.0),
                create_cw_rectangle(52.0, 42.0, 72.0, 74.0),
                create_line_segment(-10.0, 6.0, 90.0, 76.0),
                create_line_segment(-10.0, 74.0, 90.0, 4.0),
                create_line_segment(40.0, -10.0, 40.0, 90.0),
            ]),
            Shape::from_plines([
                create_rectangle(20.0, -8.0, 88.0, 52.0),
                create_cw_rectangle(30.0, 8.0, 64.0, 22.0),
                create_line_segment(0.0, 18.0, 92.0, 18.0),
                create_line_segment(0.0, 40.0, 92.0, 40.0),
                create_line_segment(0.0, 62.0, 92.0, 62.0),
            ]),
        ),
        (
            "open triangle fan buried in filled material",
            Shape::from_plines([
                create_rectangle(-30.0, -30.0, 30.0, 30.0),
                pline_open![
                    (-25.0, -20.0, 0.0),
                    (20.0, 0.0, 0.0),
                    (-25.0, 20.0, 0.0),
                    (-25.0, -20.0, 0.0)
                ],
                create_line_segment(-40.0, 0.0, 40.0, 0.0),
                create_line_segment(0.0, -40.0, 0.0, 40.0),
            ]),
            Shape::from_plines([
                create_rectangle(-10.0, -35.0, 38.0, 35.0),
                create_cw_rectangle(2.0, -10.0, 22.0, 10.0),
                create_line_segment(-35.0, -35.0, 35.0, 35.0),
            ]),
        ),
    ];

    for (name, a, b) in cases {
        let samples = dense_samples_for_shapes(&[&a, &b], 16, 16);
        for op in [
            BooleanOp::Or,
            BooleanOp::And,
            BooleanOp::Not,
            BooleanOp::Xor,
        ] {
            let result = a.boolean(&b, op);
            assert_shape_output_finite(&result);
            assert_open_linework_outside_closed_material(&result, &format!("{name} a {op:?} b"));
            assert_boolean_samples(&a, &b, &result, op, &samples);

            let reverse = b.boolean(&a, op);
            assert_shape_output_finite(&reverse);
            assert_open_linework_outside_closed_material(&reverse, &format!("{name} b {op:?} a"));
            assert_boolean_samples(&b, &a, &reverse, op, &samples);
        }
    }
}

#[test]
fn shape_boolean_deep_island_lake_nesting_survives_all_ops() {
    let nested = create_deep_nested_rect_shape();
    let rect_in_deep_lake = create_shape_rects(&[(35.0, 35.0, 38.0, 38.0)]);
    let samples = [
        (5.0, 5.0),
        (15.0, 15.0),
        (25.0, 25.0),
        (36.0, 36.0),
        (45.0, 45.0),
        (65.0, 65.0),
        (85.0, 85.0),
        (95.0, 95.0),
    ];

    assert_shape_valid(&nested);
    assert_fuzzy_eq!(shape_signed_area(&nested), 6000.0);
    assert_boolean_result(
        &nested,
        &rect_in_deep_lake,
        BooleanOp::Or,
        6009.0,
        4,
        2,
        &samples,
    );
    assert_boolean_result(
        &nested,
        &rect_in_deep_lake,
        BooleanOp::And,
        0.0,
        0,
        0,
        &samples,
    );
    assert_boolean_result(
        &nested,
        &rect_in_deep_lake,
        BooleanOp::Not,
        6000.0,
        3,
        2,
        &samples,
    );
    assert_boolean_area_and_samples(
        &nested,
        &rect_in_deep_lake,
        BooleanOp::Xor,
        6009.0,
        &samples,
    );
}

#[test]
fn shape_boolean_deep_nesting_clips_middle_island_and_lake() {
    let nested = create_deep_nested_rect_shape();
    let cutter = create_shape_rects(&[(25.0, 25.0, 75.0, 75.0)]);
    let samples = dense_sample_grid(2.0, 98.0, 4.0);

    assert_boolean_result(&nested, &cutter, BooleanOp::And, 1300.0, 2, 1, &samples);
}

#[test]
fn shape_boolean_fuzz_deep_nested_difference_rejects_detached_holes() {
    let mut a_plines = Vec::new();
    let a_min_x = -47.75540768489133;
    let a_min_y = -47.99904467165447;
    let a_size = 95.4210800220161;
    let a_step = a_size / 12.0;
    for i in 0..5 {
        let inset = a_step * i as f64;
        if i % 2 == 0 {
            a_plines.push(create_rectangle(
                a_min_x + inset,
                a_min_y + inset,
                a_min_x + a_size - inset,
                a_min_y + a_size - inset,
            ));
        } else {
            a_plines.push(create_cw_rectangle(
                a_min_x + inset,
                a_min_y + inset,
                a_min_x + a_size - inset,
                a_min_y + a_size - inset,
            ));
        }
    }
    let a = Shape::from_plines(a_plines);

    let b = Shape::from_plines([
        create_rectangle(-48.0, -48.0, -32.0, -32.0),
        create_cw_rectangle(-46.0, -46.0, -34.0, -34.0),
        create_rectangle(-44.0, -44.0, -36.0, -36.0),
    ]);

    let samples = [
        (-34.89342136580437, -34.92680462965341),
        (-34.89342136580437, -18.322892261475996),
    ];

    let not_result = a.boolean(&b, BooleanOp::Not);
    assert_shape_valid(&not_result);
    assert!(shape_signed_area(&not_result) > 0.0);
    assert_boolean_samples(&a, &b, &not_result, BooleanOp::Not, &samples);
}

#[test]
fn shape_boolean_ui_vertex_clock_drag_or_keeps_existing_material() {
    let (shape1, mut shape2) = multi_pline_ui_default_pair();
    let extents = shape1.plines_index.bounds().unwrap();
    let extents2 = shape2.plines_index.bounds().unwrap();
    let min_x = extents.min_x.min(extents2.min_x);
    let min_y = extents.min_y.min(extents2.min_y);
    let max_x = extents.max_x.max(extents2.max_x);
    let max_y = extents.max_y.max(extents2.max_y);
    let center = Vector2::new((min_x + max_x) * 0.5, (min_y + max_y) * 0.5);
    let span = (max_x - min_x).abs().max((max_y - min_y).abs()).max(1.0);
    let radius = span * 0.35;
    let radial_jitter = -0.1 * span;

    for step in 0..8 {
        let angle = -PI + 2.0 * PI * (step as f64 / 8.0);
        let point = Vector2::new(
            center.x + angle.cos() * radius + radial_jitter * (3.0 * angle).sin(),
            center.y + angle.sin() * radius + radial_jitter * (5.0 * angle).cos(),
        );
        let rpline = &mut shape2.ccw_plines[0];
        let v = rpline.polyline.at(0);
        rpline.polyline.set(0, point.x, point.y, v.bulge);
        rpline.spatial_index = rpline.polyline.create_aabb_index();
        shape2 = Shape::from_plines(
            shape2
                .ccw_plines
                .iter()
                .chain(shape2.cw_plines.iter())
                .map(|ip| ip.polyline.clone()),
        );

        let result = shape1.boolean(&shape2, BooleanOp::Or);
        assert_shape_valid(&result);
        assert!(
            shape_contains(&result, 85.87352676212761, 26.745000000000005),
            "OR should preserve existing material while a dragged shell is transiently degenerate"
        );
    }
}

#[test]
fn shape_parallel_offset_outward() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![rect]);
    let offset_result = shape.parallel_offset(-1.0, Default::default());
    // Expect single loop with bigger bounding box
    assert_eq!(offset_result.ccw_plines.len(), 1);
    let out_pline = &offset_result.ccw_plines[0].polyline;
    assert!(out_pline.extents().unwrap().max_x > 4.9);
    assert!(out_pline.extents().unwrap().max_y > 2.9);
}

#[test]
fn shape_parallel_offset_inward_collapses() {
    // Inward offset that collapses rectangle
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![rect]);
    let offset_result = shape.parallel_offset(2.0, Default::default());
    // Collapsed => empty
    assert!(offset_result.ccw_plines.is_empty());
    assert!(offset_result.cw_plines.is_empty());
}

#[test]
fn shape_translate_works() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.translate_mut(5.0, 5.0);
    // bounding box now from (5,5) to (15,15)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 5.0);
    assert_fuzzy_eq!(aabb.min_y, 5.0);
    assert_fuzzy_eq!(aabb.max_x, 15.0);
    assert_fuzzy_eq!(aabb.max_y, 15.0);
}

#[test]
fn shape_scale_works() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.scale_mut(2.0);
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 0.0);
    assert_fuzzy_eq!(aabb.min_y, 0.0);
    assert_fuzzy_eq!(aabb.max_x, 20.0);
    assert_fuzzy_eq!(aabb.max_y, 20.0);
}

#[test]
fn shape_mirror_x_works() {
    // Mirror about X (meaning y -> -y)
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.mirror_x_mut();
    // bounding box now from (0, -2) to (2, 0)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 0.0);
    assert_fuzzy_eq!(aabb.min_y, -2.0);
    assert_fuzzy_eq!(aabb.max_x, 2.0);
    assert_fuzzy_eq!(aabb.max_y, 0.0);
}

#[test]
fn shape_center_works() {
    // shape is from (0,0) to (10,10)
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.center_mut();
    let aabb = shape.plines_index.bounds().unwrap();
    // bounding box is now from (-5,-5) to (5,5)
    assert_fuzzy_eq!(aabb.min_x, -5.0);
    assert_fuzzy_eq!(aabb.min_y, -5.0);
    assert_fuzzy_eq!(aabb.max_x, 5.0);
    assert_fuzzy_eq!(aabb.max_y, 5.0);
}

#[test]
fn shape_transform_mut() {
    // Apply a 2D transform [a b; c d] plus translation.
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    // Matrix [2 0; 0 2] plus (10,10) translation.
    shape.transform_mut(2.0, 0.0, 0.0, 2.0, 10.0, 10.0);

    // bounding box now from (10,10) to (12,12)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 10.0);
    assert_fuzzy_eq!(aabb.min_y, 10.0);
    assert_fuzzy_eq!(aabb.max_x, 12.0);
    assert_fuzzy_eq!(aabb.max_y, 12.0);
}

#[test]
fn transform_builder_scale_test() {
    // Make a rectangle from (0,0) to (10,5)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 5.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // scale by factor of 2 about origin => extents from (0,0) to (20,10)
    let mut shape_copy = shape.clone();
    shape_copy.scale_mut(2.0);
    assert_extents_fuzzy_eq(&shape_copy, 0.0, 0.0, 20.0, 10.0, 1e-5);
}

#[test]
fn transform_builder_translate_test() {
    // Make a rectangle from (0,0) to (10,5)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 5.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // translate by (15, -2)
    let mut shape_copy = shape.clone();
    shape_copy.translate_mut(15.0, -2.0);
    // new bounding box => from (15,-2) to (25,3)
    assert_extents_fuzzy_eq(&shape_copy, 15.0, -2.0, 25.0, 3.0, 1e-5);
}

#[test]
fn transform_builder_center_test() {
    // Make a rectangle from (0,0) to (10,10) => extents from (0,0) to (10,10)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 10.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // center => bounding box is from (-5,-5) to (5,5)
    let mut shape_copy = shape.clone();
    shape_copy.center_mut();
    assert_extents_fuzzy_eq(&shape_copy, -5.0, -5.0, 5.0, 5.0, 1e-5);
}

#[test]
fn transform_builder_mirror_x_test() {
    // rectangle from (1.0,1.0) to (4.0,3.0)
    let rect_pline = create_rectangle(1.0, 1.0, 4.0, 3.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // mirror_x => flip in place (y -> -y), so bounding box => min_y is -3.0, max_y is -1.0
    let mut shape_copy = shape.clone();
    shape_copy.mirror_x_mut();
    // bounding box => x from (1,4) to (1,4) unchanged, y from ( -3, -1)
    assert_extents_fuzzy_eq(&shape_copy, 1.0, -3.0, 4.0, -1.0, 1e-5);
}

#[test]
fn transform_builder_rotate_test() {
    // rectangle from (0.0,0.0) to (10.0, 0.0) => extents from (0,0) to (10,0) plus the small y range
    let mut rect_pline = Polyline::new_closed();
    rect_pline.add(0.0, 0.0, 0.0);
    rect_pline.add(10.0, 0.0, 0.0);
    rect_pline.add(10.0, 2.0, 0.0);
    rect_pline.add(0.0, 2.0, 0.0);

    let shape = Shape::from_plines(vec![rect_pline]);
    // rotate 90 degrees about (0,0)
    let mut shape_copy = shape.clone();
    shape_copy.rotate_mut(PI * 0.5);

    // after 90 deg rotation:
    //  originally x from [0..10], y from [0..2]
    //  becomes x from [-2..0], y from [0..10]
    assert_extents_fuzzy_eq(&shape_copy, -2.0, 0.0, 0.0, 10.0, 1e-5);
}

#[test]
fn shape_offset_simple_test() {
    // Make a single circle-like shape from an approximate circle.
    let circle_pline = create_approx_circle(0.0, 0.0, 5.0);
    let shape = Shape::from_plines(vec![circle_pline]);

    // Offset outward by 2; the two-arc circle should expand to roughly radius 7.
    let offset_opts = ShapeOffsetOptions::default();
    let result = shape.parallel_offset(-2.0, offset_opts);
    assert_eq!(result.ccw_plines.len(), 1);
    let final_bb = result.plines_index.bounds().unwrap();
    let min_x = final_bb.min_x;
    assert!(
        min_x < -6.95 && min_x > -7.05,
        "expected min_x ~ -7.0, got {min_x}"
    );
}
