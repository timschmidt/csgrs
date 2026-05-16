//! Shared shape-boolean fuzz helpers.
//!
//! The fuzz targets intentionally generate small but relationship-rich inputs, then assert the
//! same invariants as the integration tests: valid signed shape bins, fresh spatial indexes, and
//! sampled set-membership semantics. The byte reader returns deterministic defaults when input is
//! exhausted so minimized cases remain short and replayable.

#![allow(dead_code)]

use cavalier_contours::core::{math::Vector2, traits::FuzzyEq};
use cavalier_contours::polyline::{
    BooleanOp, PlineInversionView, PlineOrientation, PlineSource, PlineSourceMut, Polyline,
};
use cavalier_contours::shape_algorithms::Shape;
use cavalier_contours::static_aabb2d_index::AABB;
use hypercurve::{
    Aabb2 as HAabb2, ArcArcIntersection as HArcArcIntersection, BooleanOp as HBooleanOp,
    BulgeVertex2 as HBulgeVertex2, Classification as HClassification, Contour2 as HContour2,
    ContourOperand as HContourOperand, ContourPointLocation as HContourPointLocation,
    ContourSplitMap as HContourSplitMap, CurvePolicy as HCurvePolicy,
    CurveString2 as HCurveString2, DefaultBackend as HBackend, FillRule as HFillRule,
    LineArcIntersection as HLineArcIntersection, LineLineIntersection as HLineLineIntersection,
    LineSeg2 as HLineSeg2, OffsetCap as HOffsetCap, Point2 as HPoint2, Region2 as HRegion2,
    RegionPointLocation as HRegionPointLocation, Scalar as HScalar, Segment2 as HSegment2,
    SegmentIntersection as HSegmentIntersection, Tolerance as HTolerance,
};
use std::f64::consts::PI;

const EPS: f64 = 1e-7;

/// Deterministic byte reader for lightweight, shrinkable fuzz input decoding.
pub struct ByteReader<'a> {
    data: &'a [u8],
    index: usize,
}

impl<'a> ByteReader<'a> {
    /// Create a reader over the current fuzz input.
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, index: 0 }
    }

    /// Read one byte, returning zero after the input is exhausted.
    pub fn byte(&mut self) -> u8 {
        let byte = self.data.get(self.index).copied().unwrap_or(0);
        self.index = self.index.saturating_add(1);
        byte
    }

    /// Decode a boolean from the next byte.
    pub fn bool(&mut self) -> bool {
        self.byte() & 1 == 1
    }

    /// Decode an integer in an inclusive range.
    pub fn usize_range(&mut self, min: usize, max: usize) -> usize {
        min + usize::from(self.byte()) % (max - min + 1)
    }

    /// Decode an integer in an inclusive range.
    pub fn i32_range(&mut self, min: i32, max: i32) -> i32 {
        min + i32::from(self.byte()) % (max - min + 1)
    }

    /// Decode a bounded finite float from four bytes.
    pub fn f64_range(&mut self, min: f64, max: f64) -> f64 {
        let raw = u32::from_le_bytes([self.byte(), self.byte(), self.byte(), self.byte()]);
        let t = f64::from(raw) / f64::from(u32::MAX);
        min + (max - min) * t
    }
}

/// Decode one of the four boolean operations.
pub fn boolean_op(reader: &mut ByteReader<'_>) -> BooleanOp {
    match reader.byte() & 3 {
        0 => BooleanOp::Or,
        1 => BooleanOp::And,
        2 => BooleanOp::Not,
        _ => BooleanOp::Xor,
    }
}

/// Create a closed CCW rectangle.
pub fn rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pline = Polyline::new_closed();
    pline.add(xmin, ymin, 0.0);
    pline.add(xmax, ymin, 0.0);
    pline.add(xmax, ymax, 0.0);
    pline.add(xmin, ymax, 0.0);
    pline
}

/// Generate a rectangle with bounded positive width and height.
pub fn rectangle_from_bytes(reader: &mut ByteReader<'_>) -> Polyline<f64> {
    let x = reader.f64_range(-64.0, 64.0);
    let y = reader.f64_range(-64.0, 64.0);
    let width = reader.f64_range(0.01, 32.0);
    let height = reader.f64_range(0.01, 32.0);
    rectangle(x, y, x + width, y + height)
}

/// Generate a multi-rectangle shape for broad-phase and unused-loop coverage.
pub fn rectangle_shape(reader: &mut ByteReader<'_>, max_rects: usize) -> Shape<f64> {
    let count = reader.usize_range(1, max_rects);
    Shape::from_plines((0..count).map(|_| rectangle_from_bytes(reader)))
}

/// Generate a single rectangular ring so fuzzing repeatedly exercises CW hole handling.
pub fn donut_shape(reader: &mut ByteReader<'_>) -> Shape<f64> {
    let x = reader.f64_range(-40.0, 40.0);
    let y = reader.f64_range(-40.0, 40.0);
    let width = reader.f64_range(4.0, 36.0);
    let height = reader.f64_range(4.0, 36.0);
    let margin_x = reader.f64_range(width * 0.1, width * 0.4);
    let margin_y = reader.f64_range(height * 0.1, height * 0.4);

    let outer = rectangle(x, y, x + width, y + height);
    let mut inner = rectangle(
        x + margin_x,
        y + margin_y,
        x + width - margin_x,
        y + height - margin_y,
    );
    inner.invert_direction_mut();
    Shape::from_plines([outer, inner])
}

/// Generate alternating island/lake/island/lake rectangle stacks.
pub fn nested_rect_shape(reader: &mut ByteReader<'_>, max_depth: usize) -> Shape<f64> {
    let depth = reader.usize_range(3, max_depth.max(3));
    let x = reader.f64_range(-48.0, 48.0);
    let y = reader.f64_range(-48.0, 48.0);
    let size = reader.f64_range(16.0, 96.0);
    let inset_step = (size / (2.0 * depth as f64 + 2.0)).max(0.5);

    Shape::from_plines((0..depth).map(|i| {
        let inset = inset_step * i as f64;
        let mut rect = rectangle(x + inset, y + inset, x + size - inset, y + size - inset);
        if i % 2 == 1 {
            rect.invert_direction_mut();
        }
        rect
    }))
}

/// Generate a deliberately adversarial nested rectangle stack with deterministic geometry.
pub fn fixed_nested_rect_shape(
    origin_x: f64,
    origin_y: f64,
    size: f64,
    depth: usize,
) -> Shape<f64> {
    let inset_step = size / (2.0 * depth as f64 + 2.0);
    Shape::from_plines((0..depth).map(|i| {
        let inset = inset_step * i as f64;
        let mut rect = rectangle(
            origin_x + inset,
            origin_y + inset,
            origin_x + size - inset,
            origin_y + size - inset,
        );
        if i % 2 == 1 {
            rect.invert_direction_mut();
        }
        rect
    }))
}

/// Generate a closed sawtooth loop used to stress many near-collinear intersections.
pub fn sawtooth_loop(
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

/// Select one high-value deterministic adversarial shape pair, then optionally perturb it.
pub fn adversarial_corpus_pair(reader: &mut ByteReader<'_>) -> (Shape<f64>, Shape<f64>) {
    let mut pair = match reader.byte() % 6 {
        0 => (
            fixed_nested_rect_shape(-60.0, -60.0, 120.0, 9),
            fixed_nested_rect_shape(-46.0, -37.0, 94.0, 8),
        ),
        1 => (
            Shape::from_plines([
                rectangle(0.0, 0.0, 90.0, 70.0),
                {
                    let mut h = rectangle(6.0, 5.0, 18.0, 20.0);
                    h.invert_direction_mut();
                    h
                },
                {
                    let mut h = rectangle(26.0, 5.0, 38.0, 20.0);
                    h.invert_direction_mut();
                    h
                },
                {
                    let mut h = rectangle(46.0, 5.0, 58.0, 20.0);
                    h.invert_direction_mut();
                    h
                },
                {
                    let mut h = rectangle(66.0, 18.0, 82.0, 62.0);
                    h.invert_direction_mut();
                    h
                },
            ]),
            Shape::from_plines([
                rectangle(-4.0, 12.0, 94.0, 24.0),
                rectangle(-4.0, 46.0, 94.0, 58.0),
                rectangle(20.0, -6.0, 32.0, 76.0),
                rectangle(60.0, -6.0, 72.0, 76.0),
            ]),
        ),
        2 => (
            {
                let mut inner = circle(0.0, 0.0, 7.0);
                inner.invert_direction_mut();
                Shape::from_plines([
                    circle(0.0, 0.0, 18.0),
                    inner,
                    rectangle(-35.0, -2.0, -18.0, 2.0),
                    rectangle(18.0, -2.0, 35.0, 2.0),
                ])
            },
            Shape::from_plines([
                rectangle(-25.0, -1.0, 25.0, 1.0),
                rectangle(-1.0, -25.0, 1.0, 25.0),
                rectangle(-8.0, -8.0, 8.0, 8.0),
            ]),
        ),
        3 => (
            Shape::from_plines([sawtooth_loop(-40.0, -20.0, 80.0, 40.0, 11)]),
            Shape::from_plines([sawtooth_loop(-37.5, -18.0, 80.0, 40.0, 13)]),
        ),
        4 => (ui_multi_boolean_shape(0.0, 0.0), {
            let mut shape = ui_multi_boolean_shape(17.0, -23.0);
            shape.rotate_mut(0.08);
            shape
        }),
        _ => (
            Shape::from_plines([
                rectangle(0.0, 0.0, 20.0, 20.0),
                rectangle(20.000001, 0.0, 40.0, 20.0),
                {
                    let mut h = rectangle(8.0, 8.0, 12.0, 12.0);
                    h.invert_direction_mut();
                    h
                },
            ]),
            Shape::from_plines([rectangle(10.0, -5.0, 30.000001, 25.0), {
                let mut h = rectangle(18.0, 6.0, 22.0, 14.0);
                h.invert_direction_mut();
                h
            }]),
        ),
    };

    if reader.bool() {
        maybe_transform(&mut pair.0, reader);
    }
    if reader.bool() {
        maybe_transform(&mut pair.1, reader);
    }
    pair
}

/// Select a pair from a deterministic corpus of singularity-style polygon booleans.
///
/// These are the fuzz equivalents of the `geo`/`i_overlay`-inspired integration tests: point-only
/// contact, shared edges, epsilon-width overlaps, hole-edge tangencies, collinear notches, and
/// large-coordinate precision stress.
pub fn singularity_corpus_pair(reader: &mut ByteReader<'_>) -> (Shape<f64>, Shape<f64>) {
    let mut pair = match reader.byte() % 6 {
        0 => (Shape::from_plines([rectangle(0.0, 0.0, 10.0, 10.0)]), {
            let mut tri = Polyline::new_closed();
            tri.add(10.0, 10.0, 0.0);
            tri.add(16.0, 10.0, 0.0);
            tri.add(13.0, 15.0, 0.0);
            Shape::from_plines([tri])
        }),
        1 => (
            Shape::from_plines([rectangle(0.0, 0.0, 10.0, 10.0)]),
            Shape::from_plines([rectangle(10.0, 0.0, 20.0, 10.0)]),
        ),
        2 => (
            Shape::from_plines([rectangle(0.0, 0.0, 10.0, 10.0)]),
            Shape::from_plines([rectangle(9.999_999, -1.0, 20.0, 11.0)]),
        ),
        3 => (
            {
                let mut hole = rectangle(10.0, 10.0, 20.0, 20.0);
                hole.invert_direction_mut();
                Shape::from_plines([rectangle(0.0, 0.0, 30.0, 30.0), hole])
            },
            Shape::from_plines([rectangle(20.0, 12.0, 27.0, 18.0)]),
        ),
        4 => (
            {
                let mut notch = Polyline::new_closed();
                for (x, y) in [
                    (0.0, 0.0),
                    (4.0, 0.0),
                    (8.0, 0.0),
                    (8.0, 8.0),
                    (5.0, 8.0),
                    (5.0, 3.0),
                    (3.0, 3.0),
                    (3.0, 8.0),
                    (0.0, 8.0),
                ] {
                    notch.add(x, y, 0.0);
                }
                Shape::from_plines([notch])
            },
            Shape::from_plines([rectangle(-1.0, 2.0, 9.0, 6.0)]),
        ),
        _ => (
            Shape::from_plines([rectangle(
                1_000_000_000.0,
                1_000_000_000.0,
                1_000_000_010.0,
                1_000_000_010.0,
            )]),
            Shape::from_plines([rectangle(
                1_000_000_009.5,
                999_999_999.5,
                1_000_000_020.0,
                1_000_000_010.5,
            )]),
        ),
    };

    if reader.bool() {
        maybe_transform(&mut pair.0, reader);
    }
    if reader.bool() {
        maybe_transform(&mut pair.1, reader);
    }
    pair
}

/// Create a full circle represented by two half-circle bulge segments.
pub fn circle(center_x: f64, center_y: f64, radius: f64) -> Polyline<f64> {
    let mut pline = Polyline::new_closed();
    pline.add(center_x - radius, center_y, 1.0);
    pline.add(center_x + radius, center_y, 1.0);
    pline
}

/// Generate arc-bearing shapes, including full circles and capsule-like loops.
pub fn arc_shape(reader: &mut ByteReader<'_>) -> Shape<f64> {
    let count = reader.usize_range(1, 3);
    Shape::from_plines((0..count).map(|_| {
        let x = reader.f64_range(-48.0, 48.0);
        let y = reader.f64_range(-48.0, 48.0);
        let radius = reader.f64_range(0.1, 24.0);
        if reader.bool() {
            circle(x, y, radius)
        } else {
            let mut capsule = Polyline::new_closed();
            capsule.add(x - radius, y - radius * 0.5, 0.0);
            capsule.add(x + radius, y - radius * 0.5, reader.f64_range(-1.5, 1.5));
            capsule.add(x + radius, y + radius * 0.5, 0.0);
            capsule.add(x - radius, y + radius * 0.5, reader.f64_range(-1.5, 1.5));
            capsule
        }
    }))
}

/// Generate small star-like polygon loops from sorted polar angles.
pub fn polygon_shape(reader: &mut ByteReader<'_>) -> Shape<f64> {
    let count = reader.usize_range(1, 3);
    Shape::from_plines((0..count).map(|_| {
        let cx = reader.f64_range(-48.0, 48.0);
        let cy = reader.f64_range(-48.0, 48.0);
        let vertex_count = reader.usize_range(3, 8);
        let base_radius = reader.f64_range(0.25, 24.0);
        let mut pline = Polyline::new_closed();

        for i in 0..vertex_count {
            let angle = 2.0 * PI * (i as f64) / (vertex_count as f64);
            let radius = base_radius * reader.f64_range(0.55, 1.45);
            pline.add(cx + radius * angle.cos(), cy + radius * angle.sin(), 0.0);
        }

        pline
    }))
}

/// Recreate the complex multi-polyline boolean scene from the UI demo.
///
/// This gives fuzzing the same arc-heavy, multi-loop starting geometry that users manipulate in
/// the demo, instead of only synthetic rectangles.
pub fn ui_multi_boolean_shape(translate_x: f64, translate_y: f64) -> Shape<f64> {
    let mut shape = Shape::from_plines([
        {
            let mut pline = Polyline::new_closed();
            pline.add(100.0, 100.0, -0.5);
            pline.add(80.0, 90.0, 0.374794619217547);
            pline.add(210.0, 0.0, 0.0);
            pline.add(230.0, 0.0, 1.0);
            pline.add(320.0, 0.0, -0.5);
            pline.add(280.0, 0.0, 0.5);
            pline.add(390.0, 210.0, 0.0);
            pline.add(280.0, 120.0, 0.5);
            pline
        },
        {
            let mut pline = Polyline::new_closed();
            pline.add(150.0, 50.0, 0.0);
            pline.add(150.0, 100.0, 0.0);
            pline.add(223.74732137849435, 142.16931273980475, 0.0);
            pline.add(199.491310072685, 52.51543504258919, 0.5);
            pline
        },
        {
            let mut pline = Polyline::new_closed();
            pline.add(261.11232783167395, 35.79686193615828, -1.0);
            pline.add(250.0, 100.0, -1.0);
            pline
        },
        {
            let mut pline = Polyline::new_closed();
            pline.add(320.2986109239592, 103.52378781211337, 0.0);
            pline.add(320.5065990423979, 76.14222955572362, -1.0);
            pline
        },
        {
            let mut pline = Polyline::new_closed();
            pline.add(273.6131273938006, -13.968608715397636, -0.3);
            pline.add(256.61336060995995, -25.49387433156079, 0.0);
            pline.add(249.69820124026208, 27.234215862385582, 0.0);
            pline
        },
    ]);

    shape.translate_mut(translate_x, translate_y);
    shape
}

/// Optionally transform a shape before booleaning to exercise transformed indexes and geometry.
pub fn maybe_transform(shape: &mut Shape<f64>, reader: &mut ByteReader<'_>) {
    if reader.bool() {
        shape.translate_mut(reader.f64_range(-12.0, 12.0), reader.f64_range(-12.0, 12.0));
    }
    if reader.bool() {
        shape.scale_mut(reader.f64_range(0.1, 3.0));
    }
    if reader.bool() {
        shape.rotate_mut(reader.f64_range(-PI, PI));
    }
}

/// Rebuild a shape from its current loops, refreshing both child and top-level indexes.
pub fn rebuild_shape(shape: &Shape<f64>) -> Shape<f64> {
    Shape::from_plines(
        shape
            .ccw_plines
            .iter()
            .chain(shape.cw_plines.iter())
            .map(|ip| ip.polyline.clone()),
    )
}

/// Mirror the shape membership convention used by the integration tests.
fn shape_material_depth(shape: &Shape<f64>, point: Vector2<f64>) -> isize {
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

/// Return true when a point is inside material under signed-bin non-zero semantics.
fn shape_contains(shape: &Shape<f64>, point: Vector2<f64>) -> bool {
    shape_material_depth(shape, point) > 0
}

/// Sum signed shape area so fuzz targets can reject non-finite output.
pub fn shape_signed_area(shape: &Shape<f64>) -> f64 {
    shape
        .ccw_plines
        .iter()
        .map(|ip| ip.polyline.area())
        .chain(shape.cw_plines.iter().map(|ip| ip.polyline.area()))
        .sum()
}

/// Assert reusable shape invariants after each generated boolean operation.
pub fn assert_shape_valid(shape: &Shape<f64>) {
    for ip in &shape.ccw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::CounterClockwise);
        assert_matching_bounds(&ip.polyline, ip.spatial_index.bounds());
    }
    for ip in &shape.cw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::Clockwise);
        assert_matching_bounds(&ip.polyline, ip.spatial_index.bounds());
    }

    assert!(shape_signed_area(shape).is_finite());
    assert_shape_index_bounds(shape);
}

/// Validate one signed loop's orientation and numeric sanity.
fn assert_loop_valid(pline: &Polyline<f64>, expected_orientation: PlineOrientation) {
    assert!(pline.is_closed());
    assert_eq!(pline.orientation(), expected_orientation);
    assert!(pline.remove_repeat_pos(EPS).is_none());
    for vertex in pline.iter_vertexes() {
        assert!(vertex.x.is_finite());
        assert!(vertex.y.is_finite());
        assert!(vertex.bulge.is_finite());
    }
}

/// Ensure a loop's cached spatial index still matches its polyline extents.
fn assert_matching_bounds(
    pline: &Polyline<f64>,
    index_bounds: Option<cavalier_contours::static_aabb2d_index::AABB<f64>>,
) {
    let pline_extents = pline.extents().expect("non-empty closed loop");
    let index_bounds = index_bounds.expect("non-empty closed loop index");
    assert!(
        pline_extents.min_x.fuzzy_eq_eps(index_bounds.min_x, EPS),
        "loop index min_x mismatch: pline={}, index={}",
        pline_extents.min_x,
        index_bounds.min_x
    );
    assert!(
        pline_extents.min_y.fuzzy_eq_eps(index_bounds.min_y, EPS),
        "loop index min_y mismatch: pline={}, index={}",
        pline_extents.min_y,
        index_bounds.min_y
    );
    assert!(
        pline_extents.max_x.fuzzy_eq_eps(index_bounds.max_x, EPS),
        "loop index max_x mismatch: pline={}, index={}",
        pline_extents.max_x,
        index_bounds.max_x
    );
    assert!(
        pline_extents.max_y.fuzzy_eq_eps(index_bounds.max_y, EPS),
        "loop index max_y mismatch: pline={}, index={}",
        pline_extents.max_y,
        index_bounds.max_y
    );
}

/// Ensure the top-level shape index matches the union of child loop indexes.
fn assert_shape_index_bounds(shape: &Shape<f64>) {
    let child_bounds: Option<cavalier_contours::static_aabb2d_index::AABB<f64>> = shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .filter_map(|ip| ip.spatial_index.bounds())
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => bounds,
                Some(curr) => cavalier_contours::static_aabb2d_index::AABB::new(
                    curr.min_x.min(bounds.min_x),
                    curr.min_y.min(bounds.min_y),
                    curr.max_x.max(bounds.max_x),
                    curr.max_y.max(bounds.max_y),
                ),
            })
        });

    match (child_bounds, shape.plines_index.bounds()) {
        (None, None) => {}
        (Some(expected), Some(actual)) => {
            assert!(expected.min_x.fuzzy_eq_eps(actual.min_x, EPS));
            assert!(expected.min_y.fuzzy_eq_eps(actual.min_y, EPS));
            assert!(expected.max_x.fuzzy_eq_eps(actual.max_x, EPS));
            assert!(expected.max_y.fuzzy_eq_eps(actual.max_y, EPS));
        }
        _ => panic!("shape top-level bounds do not match child bounds"),
    }
}

/// Compute a sampling envelope that covers both inputs and the result.
fn combined_extents(shapes: &[&Shape<f64>]) -> Option<AABB<f64>> {
    shapes
        .iter()
        .filter_map(|shape| shape.plines_index.bounds())
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => bounds,
                Some(curr) => cavalier_contours::static_aabb2d_index::AABB::new(
                    curr.min_x.min(bounds.min_x),
                    curr.min_y.min(bounds.min_y),
                    curr.max_x.max(bounds.max_x),
                    curr.max_y.max(bounds.max_y),
                ),
            })
        })
}

/// Compute extents from child loops directly, ignoring the top-level shape index.
///
/// The UI currently updates the dragged loop's index immediately while the shape-level index may
/// lag until the shape is rebuilt. Drag fuzzing uses this helper so semantic samples still cover
/// the actual geometry in both stale-index and rebuilt-index modes.
fn combined_loop_extents(shapes: &[&Shape<f64>]) -> Option<AABB<f64>> {
    shapes
        .iter()
        .flat_map(|shape| shape.ccw_plines.iter().chain(shape.cw_plines.iter()))
        .filter_map(|ip| ip.polyline.extents())
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => AABB::new(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y),
                Some(curr) => AABB::new(
                    curr.min_x.min(bounds.min_x),
                    curr.min_y.min(bounds.min_y),
                    curr.max_x.max(bounds.max_x),
                    curr.max_y.max(bounds.max_y),
                ),
            })
        })
}

/// Check sampled membership semantics for one generated boolean case.
///
/// This oracle intentionally samples interior-biased fractions instead of boundaries because
/// boundary winding semantics are not the fuzz target's responsibility.
pub fn assert_boolean_semantics(a: &Shape<f64>, b: &Shape<f64>, op: BooleanOp) {
    assert_shape_valid(a);
    assert_shape_valid(b);
    let result = a.boolean(b, op);
    assert_shape_valid(&result);

    let Some(extents) = combined_extents(&[a, b, &result]) else {
        return;
    };

    let width = (extents.max_x - extents.min_x).max(1.0);
    let height = (extents.max_y - extents.min_y).max(1.0);
    let fractions = [0.137, 0.311, 0.587, 0.829];

    for fx in fractions {
        for fy in fractions {
            let point = Vector2::new(extents.min_x + width * fx, extents.min_y + height * fy);
            let in_a = shape_contains(a, point);
            let in_b = shape_contains(b, point);
            let expected = match op {
                BooleanOp::Or => in_a || in_b,
                BooleanOp::And => in_a && in_b,
                BooleanOp::Not => in_a && !in_b,
                BooleanOp::Xor => in_a != in_b,
            };
            let actual = shape_contains(&result, point);
            assert_eq!(
                actual,
                expected,
                "boolean semantic mismatch: op={op:?}, point=({}, {}), in_a={in_a}, in_b={in_b}, result_ccw={}, result_cw={}, area={}",
                point.x,
                point.y,
                result.ccw_plines.len(),
                result.cw_plines.len(),
                shape_signed_area(&result)
            );
        }
    }
}

/// Check result validity after UI-like vertex dragging.
///
/// Unlike `assert_boolean_semantics`, this deliberately does not validate the input top-level
/// indexes because one fuzz mode mimics the UI path where a dragged loop is updated in place.
/// Dragging can also create transient self-crossing shell/hole combinations where exact sampled
/// set semantics require a planar subdivision that `Shape` does not currently expose. This target
/// therefore focuses on the UI failure mode: no panics, finite loops, and reusable result indexes.
pub fn assert_boolean_semantics_after_drag(a: &Shape<f64>, b: &Shape<f64>, op: BooleanOp) {
    let result = a.boolean(b, op);
    assert_shape_valid(&result);
}

fn shape_loop_count(shape: &Shape<f64>) -> usize {
    shape.ccw_plines.len() + shape.cw_plines.len()
}

fn update_loop_vertex_like_ui(
    shape: &mut Shape<f64>,
    flat_loop_index: usize,
    vertex_index: usize,
    point: Vector2<f64>,
) {
    let ccw_count = shape.ccw_plines.len();
    if flat_loop_index < ccw_count {
        let rpline = &mut shape.ccw_plines[flat_loop_index];
        let v_idx = vertex_index % rpline.polyline.vertex_count().max(1);
        if v_idx < rpline.polyline.vertex_count() {
            let v = rpline.polyline.at(v_idx);
            rpline.polyline.set(v_idx, point.x, point.y, v.bulge);
            rpline.spatial_index = rpline.polyline.create_aabb_index();
        }
    } else {
        let rpline = &mut shape.cw_plines[flat_loop_index - ccw_count];
        let v_idx = vertex_index % rpline.polyline.vertex_count().max(1);
        if v_idx < rpline.polyline.vertex_count() {
            let v = rpline.polyline.at(v_idx);
            rpline.polyline.set(v_idx, point.x, point.y, v.bulge);
            rpline.spatial_index = rpline.polyline.create_aabb_index();
        }
    }
}

fn vertex_count_for_flat_loop(shape: &Shape<f64>, flat_loop_index: usize) -> usize {
    let ccw_count = shape.ccw_plines.len();
    if flat_loop_index < ccw_count {
        shape.ccw_plines[flat_loop_index].polyline.vertex_count()
    } else {
        shape.cw_plines[flat_loop_index - ccw_count]
            .polyline
            .vertex_count()
    }
}

fn drag_scene_from_bytes(reader: &mut ByteReader<'_>) -> (Shape<f64>, Shape<f64>) {
    let jitter_x = reader.f64_range(-8.0, 8.0);
    let jitter_y = reader.f64_range(-8.0, 8.0);
    (
        ui_multi_boolean_shape(-20.0 + jitter_x, -20.0 - jitter_y),
        ui_multi_boolean_shape(20.0 - jitter_x, 20.0 + jitter_y),
    )
}

/// Simulate a UI drag where one vertex is swept around the whole scene like a clock hand.
pub fn assert_vertex_drag_boolean_sequence(reader: &mut ByteReader<'_>, rebuild_each_step: bool) {
    assert_vertex_drag_boolean_sequence_impl(reader, rebuild_each_step, None);
}

/// Simulate the same UI drag path, but exercise only XOR.
pub fn assert_vertex_drag_xor_sequence(reader: &mut ByteReader<'_>, rebuild_each_step: bool) {
    assert_vertex_drag_boolean_sequence_impl(reader, rebuild_each_step, Some(BooleanOp::Xor));
}

fn assert_vertex_drag_boolean_sequence_impl(
    reader: &mut ByteReader<'_>,
    rebuild_each_step: bool,
    forced_op: Option<BooleanOp>,
) {
    let (mut a, mut b) = drag_scene_from_bytes(reader);
    maybe_transform(&mut a, reader);
    maybe_transform(&mut b, reader);

    let drag_a = reader.bool();
    let loop_count = if drag_a {
        shape_loop_count(&a)
    } else {
        shape_loop_count(&b)
    };
    if loop_count == 0 {
        return;
    }

    let loop_index = reader.usize_range(0, loop_count - 1);
    let vertex_count = if drag_a {
        vertex_count_for_flat_loop(&a, loop_index)
    } else {
        vertex_count_for_flat_loop(&b, loop_index)
    };
    if vertex_count == 0 {
        return;
    }

    let vertex_index = reader.usize_range(0, vertex_count - 1);
    let steps = reader.usize_range(8, 40);
    let op_mode = forced_op
        .map(|op| match op {
            BooleanOp::Or => 0,
            BooleanOp::And => 1,
            BooleanOp::Not => 2,
            BooleanOp::Xor => 3,
        })
        .unwrap_or_else(|| reader.byte() % 5);

    let Some(extents) = combined_loop_extents(&[&a, &b]) else {
        return;
    };
    let center = Vector2::new(
        (extents.min_x + extents.max_x) * 0.5,
        (extents.min_y + extents.max_y) * 0.5,
    );
    let span = (extents.max_x - extents.min_x)
        .abs()
        .max((extents.max_y - extents.min_y).abs())
        .max(1.0);
    let radius_x = span * reader.f64_range(0.35, 1.75);
    let radius_y = span * reader.f64_range(0.35, 1.75);
    let start_angle = reader.f64_range(-PI, PI);
    let radial_jitter = reader.f64_range(-0.1, 0.1) * span;

    for step in 0..steps {
        let t = step as f64 / steps as f64;
        let angle = start_angle + 2.0 * PI * t;
        let point = Vector2::new(
            center.x + angle.cos() * radius_x + radial_jitter * (3.0 * angle).sin(),
            center.y + angle.sin() * radius_y + radial_jitter * (5.0 * angle).cos(),
        );

        if drag_a {
            update_loop_vertex_like_ui(&mut a, loop_index, vertex_index, point);
            if rebuild_each_step {
                a = rebuild_shape(&a);
            }
        } else {
            update_loop_vertex_like_ui(&mut b, loop_index, vertex_index, point);
            if rebuild_each_step {
                b = rebuild_shape(&b);
            }
        }

        match op_mode {
            0 => assert_boolean_semantics_after_drag(&a, &b, BooleanOp::Or),
            1 => assert_boolean_semantics_after_drag(&a, &b, BooleanOp::And),
            2 => assert_boolean_semantics_after_drag(&a, &b, BooleanOp::Not),
            3 => assert_boolean_semantics_after_drag(&a, &b, BooleanOp::Xor),
            _ => {
                assert_boolean_semantics_after_drag(&a, &b, BooleanOp::Or);
                assert_boolean_semantics_after_drag(&a, &b, BooleanOp::And);
                assert_boolean_semantics_after_drag(&a, &b, BooleanOp::Not);
                assert_boolean_semantics_after_drag(&a, &b, BooleanOp::Xor);
            }
        }
    }
}

/// Check all boolean modes for one generated pair.
pub fn assert_boolean_semantics_all_ops(a: &Shape<f64>, b: &Shape<f64>) {
    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        assert_boolean_semantics(a, b, op);
    }
}

/// Check algebraic identities that do not require exact output topology.
pub fn assert_boolean_identities(shape: &Shape<f64>) {
    assert_shape_valid(shape);
    let empty = Shape::<f64>::empty();
    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        assert_boolean_semantics(shape, &empty, op);
        assert_boolean_semantics(shape, shape, op);
    }
}

/// Fuzz `PlineInversionView` directly because shape holes depend on it for lower-level booleans.
pub fn assert_inversion_boolean(reader: &mut ByteReader<'_>) {
    let pline1 = if reader.bool() {
        rectangle_from_bytes(reader)
    } else {
        circle(
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(0.1, 24.0),
        )
    };
    let pline2 = if reader.bool() {
        rectangle_from_bytes(reader)
    } else {
        circle(
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(0.1, 24.0),
        )
    };
    let inverted = PlineInversionView::new(&pline1);
    let result = inverted.boolean(&pline2, boolean_op(reader));

    for result_pline in result
        .pos_plines
        .into_iter()
        .chain(result.neg_plines.into_iter())
    {
        for vertex in result_pline.pline.iter_vertexes() {
            assert!(vertex.x.is_finite());
            assert!(vertex.y.is_finite());
            assert!(vertex.bulge.is_finite());
        }
        assert!(result_pline.pline.area().is_finite());
    }
}

type HPoint = HPoint2<HBackend>;
type HScalarValue = HScalar<HBackend>;
type HContour = HContour2<HBackend>;
type HRegion = HRegion2<HBackend>;
type HSegment = HSegment2<HBackend>;

#[derive(Clone, Copy, Debug)]
struct HRect {
    xmin: f64,
    ymin: f64,
    xmax: f64,
    ymax: f64,
}

impl HRect {
    fn inset(self, x_margin: f64, y_margin: f64) -> Self {
        Self {
            xmin: self.xmin + x_margin,
            ymin: self.ymin + y_margin,
            xmax: self.xmax - x_margin,
            ymax: self.ymax - y_margin,
        }
    }

    fn width(self) -> f64 {
        self.xmax - self.xmin
    }

    fn height(self) -> f64 {
        self.ymax - self.ymin
    }
}

fn h_scalar(value: f64) -> HScalarValue {
    HScalar::<HBackend>::try_from(value).unwrap()
}

fn h_scalar_i32(value: i32) -> HScalarValue {
    value.into()
}

fn h_point(x: f64, y: f64) -> HPoint {
    HPoint2::new(h_scalar(x), h_scalar(y))
}

fn h_point_i32(x: i32, y: i32) -> HPoint {
    HPoint2::new(h_scalar_i32(x), h_scalar_i32(y))
}

fn h_vertex(x: f64, y: f64, bulge: f64) -> HBulgeVertex2<HBackend> {
    HBulgeVertex2::new(h_point(x, y), h_scalar(bulge))
}

fn h_vertex_i32(x: i32, y: i32, bulge: i32) -> HBulgeVertex2<HBackend> {
    HBulgeVertex2::new(h_point_i32(x, y), h_scalar_i32(bulge))
}

fn h_policy() -> HCurvePolicy {
    HCurvePolicy::certified()
}

fn h_offset_policy() -> HCurvePolicy {
    HCurvePolicy::edge_preview(HTolerance::new(1e-8, 1e-8))
}

fn h_boolean_op(reader: &mut ByteReader<'_>) -> HBooleanOp {
    match reader.byte() & 3 {
        0 => HBooleanOp::Union,
        1 => HBooleanOp::Intersection,
        2 => HBooleanOp::Difference,
        _ => HBooleanOp::Xor,
    }
}

fn h_rect_from_bytes(reader: &mut ByteReader<'_>) -> HRect {
    let x = reader.f64_range(-64.0, 64.0);
    let y = reader.f64_range(-64.0, 64.0);
    let width = reader.f64_range(0.25, 40.0);
    let height = reader.f64_range(0.25, 40.0);
    HRect {
        xmin: x,
        ymin: y,
        xmax: x + width,
        ymax: y + height,
    }
}

fn h_rectangle_contour(rect: HRect) -> HContour {
    HContour2::from_bulge_vertices_with_fill_rule(
        &[
            h_vertex(rect.xmin, rect.ymin, 0.0),
            h_vertex(rect.xmax, rect.ymin, 0.0),
            h_vertex(rect.xmax, rect.ymax, 0.0),
            h_vertex(rect.xmin, rect.ymax, 0.0),
        ],
        HFillRule::NonZero,
    )
    .unwrap()
}

fn h_region_from_bytes(
    reader: &mut ByteReader<'_>,
    max_materials: usize,
    max_holes: usize,
) -> HRegion {
    let material_count = reader.usize_range(1, max_materials.max(1));
    let mut rects = Vec::with_capacity(material_count);
    let mut materials = Vec::with_capacity(material_count);
    for _ in 0..material_count {
        let rect = h_rect_from_bytes(reader);
        rects.push(rect);
        materials.push(h_rectangle_contour(rect));
    }

    let mut holes = Vec::new();
    if max_holes > 0 {
        let hole_count = reader.usize_range(0, max_holes);
        let outer = rects[0];
        for _ in 0..hole_count {
            let margin_x = reader.f64_range(outer.width() * 0.12, outer.width() * 0.42);
            let margin_y = reader.f64_range(outer.height() * 0.12, outer.height() * 0.42);
            let hole = outer.inset(margin_x, margin_y);
            if hole.width() > 1e-9 && hole.height() > 1e-9 {
                holes.push(h_rectangle_contour(hole));
            }
        }
    }

    HRegion2::new(materials, holes)
}

fn h_region_bounds(regions: &[&HRegion]) -> Option<(f64, f64, f64, f64)> {
    let policy = h_policy();
    regions
        .iter()
        .filter_map(|region| match HAabb2::from_region(region, &policy) {
            Ok(HClassification::Decided(bounds)) => Some((
                bounds.min_x().to_f64_approx()?,
                bounds.min_y().to_f64_approx()?,
                bounds.max_x().to_f64_approx()?,
                bounds.max_y().to_f64_approx()?,
            )),
            Ok(HClassification::Uncertain(_)) | Err(_) => None,
        })
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => bounds,
                Some((min_x, min_y, max_x, max_y)) => (
                    min_x.min(bounds.0),
                    min_y.min(bounds.1),
                    max_x.max(bounds.2),
                    max_y.max(bounds.3),
                ),
            })
        })
}

fn h_region_membership(region: &HRegion, x: f64, y: f64) -> Option<bool> {
    match region.classify_point(&h_point(x, y), &h_policy()) {
        HClassification::Decided(HRegionPointLocation::Inside) => Some(true),
        HClassification::Decided(HRegionPointLocation::Outside) => Some(false),
        HClassification::Decided(HRegionPointLocation::Boundary)
        | HClassification::Uncertain(_) => None,
    }
}

fn h_expected_boolean(in_a: bool, in_b: bool, op: HBooleanOp) -> bool {
    match op {
        HBooleanOp::Union => in_a || in_b,
        HBooleanOp::Intersection => in_a && in_b,
        HBooleanOp::Difference => in_a && !in_b,
        HBooleanOp::Xor => in_a != in_b,
    }
}

fn h_assert_point_finite(point: &HPoint) {
    assert!(point.x().to_f64_approx().is_some_and(f64::is_finite));
    assert!(point.y().to_f64_approx().is_some_and(f64::is_finite));
}

fn h_assert_scalar_unit_interval(value: &HScalarValue) {
    let value = value
        .to_f64_approx()
        .expect("fuzz scalar should be approximable");
    assert!(
        (-1e-8..=1.0 + 1e-8).contains(&value),
        "segment parameter out of range: {value}"
    );
}

fn h_assert_segment_finite(segment: &HSegment) {
    match segment {
        HSegment2::Line(line) => {
            h_assert_point_finite(line.start());
            h_assert_point_finite(line.end());
        }
        HSegment2::Arc(arc) => {
            h_assert_point_finite(arc.start());
            h_assert_point_finite(arc.end());
            h_assert_point_finite(arc.center());
            assert!(
                arc.radius_squared()
                    .to_f64_approx()
                    .is_some_and(f64::is_finite)
            );
        }
    }
}

fn h_assert_curve_string_finite(curve: &HCurveString2<HBackend>) {
    for segment in curve.segments() {
        h_assert_segment_finite(segment);
    }
}

fn h_assert_contour_finite(contour: &HContour) {
    h_assert_curve_string_finite(contour.curve_string());
}

fn h_assert_region_semantics(a: &HRegion, b: &HRegion, result: &HRegion, op: HBooleanOp) {
    let Some((min_x, min_y, max_x, max_y)) = h_region_bounds(&[a, b, result]) else {
        return;
    };
    let width = (max_x - min_x).abs().max(1.0);
    let height = (max_y - min_y).abs().max(1.0);
    let fractions = [0.137, 0.311, 0.587, 0.829];

    for fx in fractions {
        for fy in fractions {
            let x = min_x + width * fx;
            let y = min_y + height * fy;
            let (Some(in_a), Some(in_b), Some(actual)) = (
                h_region_membership(a, x, y),
                h_region_membership(b, x, y),
                h_region_membership(result, x, y),
            ) else {
                continue;
            };
            assert_eq!(
                actual,
                h_expected_boolean(in_a, in_b, op),
                "hypercurve region boolean semantic mismatch: op={op:?}, point=({x}, {y})"
            );
        }
    }
}

fn h_line_from_i32(start: (i32, i32), end: (i32, i32)) -> HLineSeg2<HBackend> {
    HLineSeg2::try_new(h_point_i32(start.0, start.1), h_point_i32(end.0, end.1)).unwrap()
}

fn h_random_line(reader: &mut ByteReader<'_>) -> HSegment {
    let x = reader.i32_range(-64, 64);
    let y = reader.i32_range(-64, 64);
    let mut dx = reader.i32_range(-32, 32);
    let dy = reader.i32_range(-32, 32);
    if dx == 0 && dy == 0 {
        dx = 1;
    }
    HSegment2::Line(h_line_from_i32((x, y), (x + dx, y + dy)))
}

fn h_semicircle(reader: &mut ByteReader<'_>) -> HSegment {
    let cx = reader.i32_range(-32, 32);
    let cy = reader.i32_range(-32, 32);
    let radius = reader.i32_range(1, 24);
    let clockwise = reader.bool();
    HSegment2::from_bulge(
        h_point_i32(cx - radius, cy),
        h_point_i32(cx + radius, cy),
        h_scalar_i32(if clockwise { -1 } else { 1 }),
    )
    .unwrap()
}

fn h_segment_from_bytes(reader: &mut ByteReader<'_>) -> HSegment {
    if reader.bool() {
        h_random_line(reader)
    } else {
        h_semicircle(reader)
    }
}

fn h_validate_line_line(result: &HLineLineIntersection<HBackend>) {
    match result {
        HLineLineIntersection::None | HLineLineIntersection::Uncertain { .. } => {}
        HLineLineIntersection::Point {
            point,
            a_param,
            b_param,
            ..
        } => {
            h_assert_point_finite(point);
            h_assert_scalar_unit_interval(a_param);
            h_assert_scalar_unit_interval(b_param);
        }
        HLineLineIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } => {
            h_assert_point_finite(segment.start());
            h_assert_point_finite(segment.end());
            h_assert_scalar_unit_interval(a_range.start());
            h_assert_scalar_unit_interval(a_range.end());
            h_assert_scalar_unit_interval(b_range.start());
            h_assert_scalar_unit_interval(b_range.end());
        }
    }
}

fn h_validate_line_arc(result: &HLineArcIntersection<HBackend>) {
    let validate_point = |point: &hypercurve::LineArcIntersectionPoint<HBackend>| {
        h_assert_point_finite(&point.point);
        h_assert_scalar_unit_interval(&point.line_param);
    };

    match result {
        HLineArcIntersection::None | HLineArcIntersection::Uncertain { .. } => {}
        HLineArcIntersection::Point(point) => validate_point(point),
        HLineArcIntersection::TwoPoints { first, second } => {
            validate_point(first);
            validate_point(second);
        }
    }
}

fn h_validate_arc_arc(result: &HArcArcIntersection<HBackend>) {
    let validate_point = |point: &hypercurve::ArcArcIntersectionPoint<HBackend>| {
        h_assert_point_finite(&point.point);
    };

    match result {
        HArcArcIntersection::None | HArcArcIntersection::Uncertain { .. } => {}
        HArcArcIntersection::Point(point) => validate_point(point),
        HArcArcIntersection::TwoPoints { first, second } => {
            validate_point(first);
            validate_point(second);
        }
        HArcArcIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } => {
            h_assert_point_finite(segment.start());
            h_assert_point_finite(segment.end());
            h_assert_point_finite(segment.center());
            h_assert_scalar_unit_interval(a_range.start());
            h_assert_scalar_unit_interval(a_range.end());
            h_assert_scalar_unit_interval(b_range.start());
            h_assert_scalar_unit_interval(b_range.end());
        }
    }
}

fn h_validate_segment_intersection(result: &HSegmentIntersection<HBackend>) {
    match result {
        HSegmentIntersection::LineLine(result) => h_validate_line_line(result),
        HSegmentIntersection::LineArc { result, .. } => h_validate_line_arc(result),
        HSegmentIntersection::ArcArc(result) => h_validate_arc_arc(result),
    }
}

/// Fuzz native line and circular-arc intersection dispatch.
pub fn h_assert_segment_intersections(reader: &mut ByteReader<'_>) {
    let first = h_segment_from_bytes(reader);
    let second = match reader.byte() % 4 {
        0 => h_segment_from_bytes(reader),
        1 => h_random_line(reader),
        2 => h_semicircle(reader),
        _ => HSegment2::Line(h_line_from_i32((-64, 0), (64, 0))),
    };
    let policy = h_policy();
    let forward = first.intersect_segment(&second, &policy).unwrap();
    let reverse = second.intersect_segment(&first, &policy).unwrap();

    assert_eq!(
        forward.is_none(),
        reverse.is_none(),
        "segment intersection none-ness should be symmetric"
    );
    h_validate_segment_intersection(&forward);
    h_validate_segment_intersection(&reverse);
}

/// Fuzz contour and region point classification, including explicit boundaries and holes.
pub fn h_assert_contour_region_classification(reader: &mut ByteReader<'_>) {
    let outer = h_rect_from_bytes(reader);
    let margin_x = reader.f64_range(outer.width() * 0.15, outer.width() * 0.35);
    let margin_y = reader.f64_range(outer.height() * 0.15, outer.height() * 0.35);
    let hole = outer.inset(margin_x, margin_y);
    let contour = h_rectangle_contour(outer);
    let region = HRegion2::new(vec![contour.clone()], vec![h_rectangle_contour(hole)]);
    let prepared = region.prepare_topology_queries(&h_policy());

    let samples = [
        (
            (outer.xmin + outer.xmax) * 0.5,
            outer.ymin + outer.height() * 0.05,
            HRegionPointLocation::Inside,
        ),
        (
            (hole.xmin + hole.xmax) * 0.5,
            (hole.ymin + hole.ymax) * 0.5,
            HRegionPointLocation::Outside,
        ),
        (
            outer.xmax + outer.width().max(1.0),
            outer.ymax + outer.height().max(1.0),
            HRegionPointLocation::Outside,
        ),
        (
            outer.xmin,
            (outer.ymin + outer.ymax) * 0.5,
            HRegionPointLocation::Boundary,
        ),
    ];

    for (x, y, expected) in samples {
        let point = h_point(x, y);
        assert_eq!(
            region.classify_point(&point, &h_policy()),
            HClassification::Decided(expected)
        );
        assert_eq!(
            prepared.classify_point(&point, &h_policy()),
            HClassification::Decided(expected)
        );
    }

    assert_eq!(
        contour.classify_point(
            &h_point(
                (outer.xmin + outer.xmax) * 0.5,
                (outer.ymin + outer.ymax) * 0.5
            ),
            &h_policy(),
        ),
        HClassification::Decided(HContourPointLocation::Inside)
    );
}

/// Fuzz native region booleans and prepared/ordinary consistency.
pub fn h_assert_region_boolean(reader: &mut ByteReader<'_>) {
    let a = h_region_from_bytes(reader, 3, 2);
    let b = h_region_from_bytes(reader, 3, 2);
    let op = h_boolean_op(reader);
    let policy = h_policy();
    let fill_rule = HFillRule::NonZero;
    let prepared_a = a.prepare_topology_queries(&policy);
    let prepared_b = b.prepare_topology_queries(&policy);

    let plain = a.boolean_region(&b, op, fill_rule, &policy).unwrap();
    assert_eq!(
        prepared_a
            .boolean_region(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain
    );
    assert_eq!(
        prepared_a
            .boolean_region_against_region(&b.as_view(), op, fill_rule, &policy)
            .unwrap(),
        plain
    );
    assert_eq!(
        a.as_view()
            .boolean_region_against_prepared_region(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain
    );

    let plain_contours = a
        .boolean_boundary_contours(&b, op, fill_rule, &policy)
        .unwrap();
    assert_eq!(
        prepared_a
            .boolean_boundary_contours(&prepared_b, op, fill_rule, &policy)
            .unwrap(),
        plain_contours
    );

    if let HClassification::Decided(result) = &plain {
        for contour in result
            .material_contours()
            .iter()
            .chain(result.hole_contours().iter())
        {
            h_assert_contour_finite(contour);
        }
        h_assert_region_semantics(&a, &b, result, op);
    }

    if let HClassification::Decided(contours) = plain_contours {
        for contour in &contours {
            h_assert_contour_finite(contour);
        }
    }
}

/// Fuzz contour/region event collection and split-map consistency.
pub fn h_assert_events_and_fragments(reader: &mut ByteReader<'_>) {
    let a = h_rectangle_contour(h_rect_from_bytes(reader));
    let b = h_rectangle_contour(h_rect_from_bytes(reader));
    let policy = h_policy();
    let prepared_a = a.prepare_topology_queries(&policy);
    let prepared_b = b.prepare_topology_queries(&policy);
    let events = a.intersect_contour(&b, &policy).unwrap();

    assert_eq!(
        prepared_a
            .intersect_prepared_contour(&prepared_b, &policy)
            .unwrap(),
        events
    );
    assert_eq!(prepared_a.intersect_contour(&b, &policy).unwrap(), events);

    for operand in [HContourOperand::First, HContourOperand::Second] {
        let source = if matches!(operand, HContourOperand::First) {
            &a
        } else {
            &b
        };
        if let HClassification::Decided(fragments) = source
            .split_at_intersections(&events, operand, &policy)
            .unwrap()
        {
            for fragment in fragments.fragments() {
                h_assert_segment_finite(&fragment.segment);
                h_assert_scalar_unit_interval(fragment.source_range.start());
                h_assert_scalar_unit_interval(fragment.source_range.end());
            }
        }
    }

    let region_a = HRegion2::new(vec![a], vec![]);
    let region_b = HRegion2::new(vec![b], vec![]);
    let region_events = region_a.intersect_region(&region_b, &policy).unwrap();
    let prepared_region_a = region_a.prepare_topology_queries(&policy);
    let prepared_region_b = region_b.prepare_topology_queries(&policy);
    assert_eq!(
        prepared_region_a
            .intersect_prepared_region(&prepared_region_b, &policy)
            .unwrap(),
        region_events
    );
}

fn h_curve_string_from_bytes(reader: &mut ByteReader<'_>) -> HCurveString2<HBackend> {
    let count = reader.usize_range(2, 6);
    let mut x = reader.f64_range(-48.0, 48.0);
    let mut y = reader.f64_range(-48.0, 48.0);
    let mut vertices = Vec::with_capacity(count);
    for index in 0..count {
        let bulge = if index + 1 < count {
            match reader.byte() % 5 {
                0 => -1.0,
                1 => -0.5,
                2 => 0.0,
                3 => 0.5,
                _ => 1.0,
            }
        } else {
            0.0
        };
        vertices.push(h_vertex(x, y, bulge));
        if index + 1 < count {
            x += reader.f64_range(0.25, 24.0);
            y += reader.f64_range(-12.0, 12.0);
        }
    }
    HCurveString2::from_bulge_vertices(&vertices).unwrap()
}

fn h_assert_aabb_finite(bbox: &HAabb2<HBackend>) {
    h_assert_point_finite(bbox.min());
    h_assert_point_finite(bbox.max());
    let min_x = bbox.min_x().to_f64_approx().unwrap();
    let min_y = bbox.min_y().to_f64_approx().unwrap();
    let max_x = bbox.max_x().to_f64_approx().unwrap();
    let max_y = bbox.max_y().to_f64_approx().unwrap();
    assert!(min_x <= max_x, "aabb x coordinates are inverted");
    assert!(min_y <= max_y, "aabb y coordinates are inverted");
}

fn h_assert_aabb_contains_point(bbox: &HAabb2<HBackend>, point: &HPoint, policy: &HCurvePolicy) {
    assert_eq!(
        bbox.contains_point(point, policy),
        HClassification::Decided(true)
    );
}

fn h_assert_aabb_contains_segment_endpoints(
    bbox: &HAabb2<HBackend>,
    segment: &HSegment,
    policy: &HCurvePolicy,
) {
    match segment {
        HSegment2::Line(line) => {
            h_assert_aabb_contains_point(bbox, line.start(), policy);
            h_assert_aabb_contains_point(bbox, line.end(), policy);
        }
        HSegment2::Arc(arc) => {
            h_assert_aabb_contains_point(bbox, arc.start(), policy);
            h_assert_aabb_contains_point(bbox, arc.end(), policy);
        }
    }
}

fn h_validate_split_map(map: &HContourSplitMap<HBackend>) {
    assert_eq!(
        map.split_points().first().map(|point| point.segment_index),
        Some(0)
    );
    let mut previous_segment = 0;
    let mut previous_param = f64::NEG_INFINITY;

    for point in map.split_points() {
        h_assert_scalar_unit_interval(&point.param);
        let param = point.param.to_f64_approx().unwrap();
        assert!(
            point.segment_index >= previous_segment,
            "split points should be sorted by segment index"
        );
        if point.segment_index == previous_segment {
            assert!(
                param + 1e-8 >= previous_param,
                "split parameters should be sorted within a segment"
            );
        } else {
            previous_segment = point.segment_index;
        }
        previous_param = param;
    }

    for segment_index in 0..map.segment_count() {
        let params = map
            .params_for_segment(segment_index)
            .expect("split map should expose every source segment");
        assert_eq!(params.first(), Some(&h_scalar_i32(0)));
        assert_eq!(params.last(), Some(&h_scalar_i32(1)));
        let mut previous = f64::NEG_INFINITY;
        for param in params {
            h_assert_scalar_unit_interval(param);
            let value = param.to_f64_approx().unwrap();
            assert!(
                value + 1e-8 >= previous,
                "split map parameters should be sorted"
            );
            previous = value;
        }
    }
}

/// Fuzz bbox, prepared curve-string, and split-map invariants.
pub fn h_assert_bboxes_curve_strings_and_splits(reader: &mut ByteReader<'_>) {
    let policy = h_policy();
    let first = h_segment_from_bytes(reader);
    let second = h_segment_from_bytes(reader);

    let first_box = HAabb2::from_segment(&first, &policy).unwrap();
    let second_box = HAabb2::from_segment(&second, &policy).unwrap();
    if let HClassification::Decided(first_box) = &first_box {
        h_assert_aabb_finite(first_box);
        h_assert_aabb_contains_segment_endpoints(first_box, &first, &policy);
    }
    if let HClassification::Decided(second_box) = &second_box {
        h_assert_aabb_finite(second_box);
        h_assert_aabb_contains_segment_endpoints(second_box, &second, &policy);
    }
    if let (HClassification::Decided(first_box), HClassification::Decided(second_box)) =
        (&first_box, &second_box)
    {
        assert_eq!(
            first_box.overlaps(second_box, &policy),
            second_box.overlaps(first_box, &policy),
            "aabb overlap must be symmetric"
        );
        if let HClassification::Decided(union) = first_box.union(second_box, &policy) {
            h_assert_aabb_finite(&union);
            h_assert_aabb_contains_segment_endpoints(&union, &first, &policy);
            h_assert_aabb_contains_segment_endpoints(&union, &second, &policy);
        }
    }

    let curve_a = h_curve_string_from_bytes(reader);
    let curve_b = h_curve_string_from_bytes(reader);
    if let HClassification::Decided(curve_box) =
        HAabb2::from_curve_string(&curve_a, &policy).unwrap()
    {
        h_assert_aabb_finite(&curve_box);
        for segment in curve_a.segments() {
            h_assert_aabb_contains_segment_endpoints(&curve_box, segment, &policy);
        }
    }

    let prepared_a = curve_a.prepare_topology_queries(&policy);
    let prepared_b = curve_b.prepare_topology_queries(&policy);
    assert_eq!(prepared_a.curve_string(), &curve_a);
    assert_eq!(prepared_a.segment_boxes().len(), curve_a.segments().len());
    if let Some(curve_box) = prepared_a.curve_box() {
        h_assert_aabb_finite(curve_box);
    }
    for bbox in prepared_a.segment_boxes().iter().flatten() {
        h_assert_aabb_finite(bbox);
    }

    let plain_events = curve_a.intersect_curve_string(&curve_b, &policy).unwrap();
    assert_eq!(
        prepared_a
            .intersect_prepared_curve_string(&prepared_b, &policy)
            .unwrap(),
        plain_events
    );
    assert_eq!(
        prepared_a
            .intersect_curve_string(&curve_b, &policy)
            .unwrap(),
        plain_events
    );
    for event in &plain_events {
        h_validate_segment_intersection(&event.relation);
    }

    let contour_a = h_rectangle_contour(h_rect_from_bytes(reader));
    let contour_b = h_rectangle_contour(h_rect_from_bytes(reader));
    let contour_events = contour_a.intersect_contour(&contour_b, &policy).unwrap();
    for (operand, segment_count) in [
        (HContourOperand::First, contour_a.len()),
        (HContourOperand::Second, contour_b.len()),
    ] {
        if let HClassification::Decided(split_map) =
            HContourSplitMap::from_intersections(segment_count, &contour_events, operand, &policy)
        {
            h_validate_split_map(&split_map);
        }
    }
}

/// Fuzz primitive, curve-string, contour, and outline offsets.
pub fn h_assert_offsets_and_self_contacts(reader: &mut ByteReader<'_>) {
    let policy = h_offset_policy();
    let HSegment2::Line(line) = h_random_line(reader) else {
        unreachable!("h_random_line always returns a line segment");
    };
    let distance = h_scalar(reader.f64_range(-8.0, 8.0));
    if let HClassification::Decided(offset) = HSegment2::Line(line.clone())
        .offset_left(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_segment_finite(&offset);
    }

    let curve = h_curve_string_from_bytes(reader);
    let _ = curve.has_self_contacts(&policy).unwrap();
    if let HClassification::Decided(offset) = curve
        .offset_left_with_line_joins(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_curve_string_finite(&offset);
    }
    if let HClassification::Decided(offset) = curve
        .offset_left_checked(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_curve_string_finite(&offset);
    }

    let outline_distance = h_scalar(reader.f64_range(0.01, 8.0));
    let cap = match reader.byte() % 3 {
        0 => HOffsetCap::Round,
        1 => HOffsetCap::Butt,
        _ => HOffsetCap::Square,
    };
    if let HClassification::Decided(outline) = curve
        .offset_outline(outline_distance.clone(), cap, &policy)
        .unwrap()
    {
        h_assert_contour_finite(&outline);
    }

    let rect = h_rectangle_contour(h_rect_from_bytes(reader));
    let _ = rect.has_self_contacts(&policy).unwrap();
    if let HClassification::Decided(offset) = rect
        .offset_left_with_line_joins(distance.clone(), &policy)
        .unwrap()
    {
        h_assert_contour_finite(&offset);
    }
    if let HClassification::Decided(offset) = rect.offset_left_checked(distance, &policy).unwrap() {
        h_assert_contour_finite(&offset);
    }
}
