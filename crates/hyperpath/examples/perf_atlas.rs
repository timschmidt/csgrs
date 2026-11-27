//! Performance atlas for Hyperpath: exact CAM toolpath generation, offsets,
//! arrangements, feed scheduling and PCB trace checks.
//!
//! Groups and axes:
//! - `cam`: pocket rings, pocket link graphs, additive beads and serpentine
//!   infill swept over the stepover or bead spacing `2^-k` of a unit-size
//!   region (so path count grows as `2^k`); rest-material graphs over cutter
//!   count; region booleans over coordinate bits.
//! - `offset`: line, profile, arc and Bézier offsets over offset distance
//!   `2^-k`, profile segment count, coordinate bits, distance to arc collapse
//!   and Bézier control-point bits (quadratic to quintic) or sample count.
//! - `arrangement`: line, arc, line/arc and line/cubic arrangements over
//!   primitive count, for general, axis-aligned and degenerate inputs.
//! - `routing`: feed-time certification, lookahead and jerk-feasible feed
//!   planning, axis projection, tangent chains, differential-pair skew and
//!   meanders over path length (segment, span or bump count).
//! - `trace`: PCB trace clearance over gap `2^-k`, all-pairs design-rule
//!   checks over trace count, and Specctra parse, import and rule audits over
//!   record count.
//!
//! Input kinds compare integer, rational (non-dyadic) and irrational
//! (square-root) coordinates, and axis-aligned against general directions.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperpath --all-features --example perf_atlas -- --out atlas.json`.
//! With `--resources`, path-producing operations report their path or
//! fragment count (`nodes`), the expression depth of their deepest generated
//! coordinate (`depth`) and its largest stored rational (`bits`). With
//! `--counters` and `dispatch-trace`, Hyperreal/Hyperlimit dispatch,
//! predicate and approximation events are recorded per call; Hyperpath has no
//! instrumentation of its own.
use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    AffineSpanAxisProjection, ArcDirection, AxisMotionLimits, BeadFillAxis, BezierParameter,
    CardinalPoint, CircularArc, ClearanceStatus, CubicBezier, ExplicitCircularArc, FeedPathElement,
    HigherOrderBezier, LinePathSegment, LookaheadFeedPlanningLimits, NetId, OffsetSide,
    PcbCircularPad, PcbRectPad, PcbTrace, PcbViaStack, QuadraticBezier, RectangularPocket,
    SpecctraGridArcWireRecord, SpecctraGridRouteRecords, SpecctraGridRouteRuleRecord,
    SpecctraGridTraceRecord, SpecctraGridViaRecord, SweptLineSegment, TangentSpan,
    TraceClearanceReport, TraceLayer, ViaDrillIntent, alternating_detour_meander,
    arrange_explicit_arcs, arrange_line_segments, arrange_line_segments_with_cubic_beziers,
    arrange_line_segments_with_explicit_arcs, audit_specctra_trace_rule_clearances,
    certify_acceleration_limited_feed_time_for_path, certify_constant_feed_time_for_path,
    certify_differential_pair_skew, certify_g1_chain, check_trace_clearance,
    check_trace_pad_clearance, check_trace_rect_pad_clearance, check_trace_via_clearance,
    classify_tangent_chain, classify_tangent_join, import_specctra_text_route,
    intersect_rectangular_regions, multi_detour_meander, nonuniform_detour_meander,
    offset_axis_aligned_segment, offset_cardinal_arc, offset_cubic_bezier_sample,
    offset_explicit_arc, offset_higher_order_bezier_sample, offset_quadratic_bezier_sample,
    parse_specctra_grid_route_records, plan_axis_projected_motion_limits,
    plan_jerk_feasible_lookahead_schedule, plan_lookahead_feed_schedule, rectangular_beads,
    rectangular_pocket_link_graph, rectangular_pocket_rings, rectangular_rest_material_graph,
    rectangular_serpentine_infill_graph, serialize_specctra_grid_route_records,
    specctra_grid_route_rule_record, specctra_grid_trace_record, subtract_rectangular_region,
};
use hyperreal::{Rational, Real};
use perf_atlas::{Atlas, Checked, Rng, Shape, Shaped, ladder, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

/// Side of the integer coordinate frame used by generated routes and boards.
const FRAME: i64 = 1 << 16;

fn int(v: i64) -> Real {
    Real::from(v)
}

fn ratio(n: i64, d: i64) -> Real {
    Real::new(Rational::new(n) / Rational::new(d))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(int(x), int(y))
}

fn pt(x: Real, y: Real) -> Point2 {
    Point2::new(x, y)
}

fn seg(a: Point2, b: Point2) -> LinePathSegment {
    LinePathSegment::new(a, b, STRICT).expect("non-degenerate generated segment")
}

fn sqrt2() -> Real {
    int(2).sqrt().expect("sqrt(2)")
}

/// `2^-k` as an exact rational, for any `k >= 0`.
fn half_pow(k: u32) -> Real {
    let mut denominator = Rational::new(1);
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        denominator *= Rational::new(1 << step);
        left -= step;
    }
    Real::new(Rational::new(1) / denominator)
}

/// A random positive integer with exactly `bits` bits.
fn big(bits: u32, rng: &mut Rng) -> Real {
    let value: Rational = rng.decimal(bits).parse().expect("decimal digits");
    Real::new(value)
}

/// Expression nodes, depth and largest stored rational of one coordinate.
fn real_shape(value: &Real) -> Shape {
    let size = value.representation_size(1 << 16);
    Shape {
        nodes: size.nodes as u64,
        depth: size.depth as u64,
        bits: size.max_rational_bits,
    }
}

/// `count` generated paths whose deepest coordinate has the given shape.
fn paths_shape<'a>(count: usize, coordinates: impl IntoIterator<Item = &'a Real>) -> Shape {
    let deepest = coordinates
        .into_iter()
        .map(real_shape)
        .fold(Shape::default(), |a, b| Shape {
            nodes: 0,
            depth: a.depth.max(b.depth),
            bits: a.bits.max(b.bits),
        });
    Shape {
        nodes: count as u64,
        ..deepest
    }
}

fn segment_coordinates(segment: &LinePathSegment) -> [&Real; 4] {
    [
        &segment.start().x,
        &segment.start().y,
        &segment.end().x,
        &segment.end().y,
    ]
}

/// A certification report as an outcome: `Err` names the failing stage.
fn certified<T>(value: Result<T, impl std::fmt::Debug>, ok: impl Fn(&T) -> bool) -> Checked<T> {
    match value {
        Ok(value) if ok(&value) => Ok(value),
        Ok(_) => Err("certification unsatisfied".to_owned()),
        Err(error) => Err(format!("{error:?}")),
    }
}

#[cfg(feature = "dispatch-trace")]
fn correlation() -> hyperreal::dispatch_trace::TraceCorrelationSummary {
    hyperreal::dispatch_trace::snapshot_trace().correlation_summary()
}

#[cfg(feature = "dispatch-trace")]
fn dispatch_events() -> u64 {
    correlation().dispatch_events
}

#[cfg(feature = "dispatch-trace")]
fn predicate_events() -> u64 {
    correlation().predicate_events
}

#[cfg(feature = "dispatch-trace")]
fn approximation_events() -> u64 {
    correlation().approximation_events
}

// ---------------------------------------------------------------- CAM inputs

/// A unit-scale region of the given kind; the stepover or spacing for
/// exponent `k` is returned with it.
fn cam_region(kind: &str, k: u32) -> (RectangularPocket, Real) {
    let step = half_pow(k);
    let (min, max, step) = match kind {
        "square" => (p(0, 0), p(1, 1), step),
        "8:1 slot" => (p(0, 0), p(8, 1), step),
        "rational frame" => (
            pt(ratio(1, 3), ratio(1, 7)),
            pt(ratio(4, 3), ratio(8, 7)),
            step * ratio(5, 7),
        ),
        "irrational step" => (p(0, 0), p(1, 1), step * sqrt2() * ratio(1, 2)),
        other => unreachable!("unknown CAM kind {other}"),
    };
    (
        RectangularPocket::new(min, max, STRICT).expect("ordered region"),
        step,
    )
}

const CAM_KINDS: [&str; 4] = ["square", "8:1 slot", "rational frame", "irrational step"];

/// Cutters for the rest-material graph over a `FRAME`-sized stock.
fn rest_cutters(kind: &str, n: usize, rng: &mut Rng) -> Vec<RectangularPocket> {
    let rect = |x0: Real, y0: Real, x1: Real, y1: Real| {
        RectangularPocket::new(pt(x0, y0), pt(x1, y1), STRICT).expect("ordered cutter")
    };
    match kind {
        "random overlapping" => (0..n)
            .map(|_| {
                let (x, y) = (rng.range(-FRAME / 8, FRAME), rng.range(-FRAME / 8, FRAME));
                let (w, h) = (rng.range(1, FRAME / 4), rng.range(1, FRAME / 4));
                rect(int(x), int(y), int(x + w), int(y + h))
            })
            .collect(),
        "grid tiles" => {
            let side = (n as f64).sqrt().ceil() as i64;
            let cell = FRAME / side.max(1);
            (0..n as i64)
                .map(|i| {
                    let (x, y) = ((i % side) * cell, (i / side) * cell);
                    rect(int(x), int(y), int(x + cell / 2), int(y + cell / 2))
                })
                .collect()
        }
        "full-height strips" => (0..n as i64)
            .map(|i| {
                let x = (2 * i + 1) * FRAME / (2 * n as i64 + 1);
                rect(int(x), int(-1), int(x + 1), int(FRAME + 1))
            })
            .collect(),
        "rational overlapping" => (0..n)
            .map(|_| {
                let d = 2 * rng.range(1, 1 << 10) + 1;
                let (x, y) = (rng.range(-FRAME / 8, FRAME), rng.range(-FRAME / 8, FRAME));
                let (w, h) = (rng.range(1, FRAME / 4), rng.range(1, FRAME / 4));
                rect(
                    ratio(x * d + rng.range(0, d - 1), d),
                    ratio(y * d + rng.range(0, d - 1), d),
                    ratio((x + w) * d, d),
                    ratio((y + h) * d, d),
                )
            })
            .collect(),
        other => unreachable!("unknown rest kind {other}"),
    }
}

/// Two overlapping rectangles whose coordinates carry `bits`-bit fractions.
fn bits_rects(kind: &str, bits: u32, rng: &mut Rng) -> (RectangularPocket, RectangularPocket) {
    let fraction = |rng: &mut Rng| -> Real {
        match kind {
            "dyadic" => big(bits, rng) * half_pow(bits + 1),
            "rational" => (big(bits, rng) / (big(bits, rng) + big(bits, rng)))
                .unwrap_or_else(|_| unreachable!("positive denominator")),
            "irrational" => big(bits, rng) * half_pow(bits + 1) * sqrt2() * ratio(1, 2),
            other => unreachable!("unknown bits kind {other}"),
        }
    };
    let corner = |base: i64, rng: &mut Rng| int(base) + fraction(rng);
    let first = RectangularPocket::new(
        pt(corner(0, rng), corner(0, rng)),
        pt(corner(8, rng), corner(8, rng)),
        STRICT,
    )
    .expect("ordered first rectangle");
    let second = RectangularPocket::new(
        pt(corner(2, rng), corner(-2, rng)),
        pt(corner(5, rng), corner(4, rng)),
        STRICT,
    )
    .expect("ordered second rectangle");
    (first, second)
}

// ------------------------------------------------------------- path inputs

/// Kinds of generated polyline routes.
const ROUTE_KINDS: [&str; 4] = [
    "collinear",
    "axis staircase",
    "3-4-5 zigzag",
    "unit diagonals",
];

/// A connected polyline of `n` segments; returns its points and exact length.
fn route_points(kind: &str, n: usize, rng: &mut Rng) -> (Vec<Point2>, Real) {
    let mut points = vec![p(0, 0)];
    let (mut x, mut y) = (0i64, 0i64);
    let mut units = 0i64;
    for i in 0..n {
        let len = rng.range(1, 8);
        match kind {
            "collinear" => x += len,
            "axis staircase" => {
                if i % 2 == 0 {
                    x += len
                } else {
                    y += len
                }
            }
            "3-4-5 zigzag" => {
                x += 3 * len;
                y += if i % 2 == 0 { 4 * len } else { -4 * len };
            }
            "unit diagonals" => {
                x += len;
                y += if i % 2 == 0 { len } else { -len };
            }
            "3-4-5 collinear" => {
                x += 3 * len;
                y += 4 * len;
            }
            other => unreachable!("unknown route kind {other}"),
        }
        units += match kind {
            "3-4-5 zigzag" | "3-4-5 collinear" => 5 * len,
            _ => len,
        };
        points.push(p(x, y));
    }
    let length = if kind == "unit diagonals" {
        int(units) * sqrt2()
    } else {
        int(units)
    };
    (points, length)
}

fn route(kind: &str, n: usize, rng: &mut Rng) -> (Vec<LinePathSegment>, Real) {
    let (points, length) = route_points(kind, n, rng);
    let segments = points
        .windows(2)
        .map(|w| seg(w[0].clone(), w[1].clone()))
        .collect();
    (segments, length)
}

/// A route for the lookahead planners: elements, spans and node limits.
fn lookahead_input(
    kind: &str,
    n: usize,
    rng: &mut Rng,
) -> (
    Vec<FeedPathElement>,
    Vec<TangentSpan>,
    LookaheadFeedPlanningLimits,
) {
    let (segments, _) = route(
        if kind == "collinear" {
            "collinear"
        } else {
            "axis staircase"
        },
        n,
        rng,
    );
    let spans = segments
        .iter()
        .map(TangentSpan::from_line_segment)
        .collect();
    let radius = match kind {
        "collinear" | "staircase, sharp corners" => Real::zero(),
        "staircase, corner radius 1/4" => ratio(1, 4),
        other => unreachable!("unknown lookahead kind {other}"),
    };
    let limits = LookaheadFeedPlanningLimits {
        maximum_entry_feed: Real::zero(),
        maximum_corner_feeds: vec![int(8); n - 1],
        corner_radii: vec![radius; n - 1],
        maximum_exit_feed: Real::zero(),
    };
    (
        segments.into_iter().map(FeedPathElement::Line).collect(),
        spans,
        limits,
    )
}

const LOOKAHEAD_KINDS: [&str; 3] = [
    "collinear",
    "staircase, sharp corners",
    "staircase, corner radius 1/4",
];

/// One of the two exact arc carriers; inputs are built once per call, so the
/// size difference between variants does not matter.
#[allow(clippy::large_enum_variant, reason = "short-lived atlas inputs")]
enum AnyArc {
    Cardinal(CircularArc),
    Explicit(ExplicitCircularArc),
}

/// A quadratic or cubic Bézier.
enum Curve {
    Quadratic(QuadraticBezier),
    Cubic(CubicBezier),
}

/// A half-circle explicit arc of radius `5s` through the Pythagorean points
/// `c ± (3s, 4s)`, counter-clockwise.
fn half_arc(cx: Real, cy: Real, s: Real) -> ExplicitCircularArc {
    let (dx, dy) = (int(3) * s.clone(), int(4) * s.clone());
    ExplicitCircularArc::new(
        pt(cx.clone(), cy.clone()),
        int(5) * s,
        pt(cx.clone() + dx.clone(), cy.clone() + dy.clone()),
        pt(cx - dx, cy - dy),
        ArcDirection::Ccw,
        STRICT,
    )
    .expect("arc endpoints on the circle")
}

const ARC_KINDS: [&str; 3] = ["random half-arcs", "concentric", "common chord pencil"];

fn arcs_of(kind: &str, n: usize, rng: &mut Rng) -> Vec<ExplicitCircularArc> {
    (0..n)
        .map(|i| match kind {
            "random half-arcs" => half_arc(
                int(rng.range(0, FRAME)),
                int(rng.range(0, FRAME)),
                int(rng.range(64, FRAME / 16)),
            ),
            "concentric" => half_arc(int(0), int(0), int(i as i64 + 1)),
            // Circles through (±3, 0): centres on the y axis at heights 4i/3,
            // radius 5 scaled to keep the endpoints rational.
            "common chord pencil" => {
                let s = int(i as i64 + 1);
                half_arc(int(0), int(4) * s.clone(), s)
            }
            other => unreachable!("unknown arc kind {other}"),
        })
        .collect()
}

const LINE_KINDS: [&str; 4] = [
    "random",
    "axis grid",
    "concurrent star",
    "near-parallel fan",
];

fn lines_of(kind: &str, n: usize, rng: &mut Rng) -> Vec<LinePathSegment> {
    let half = (n / 2).max(1) as i64;
    (0..n as i64)
        .map(|i| match kind {
            "random" => seg(
                p(rng.range(0, FRAME), rng.range(0, FRAME)),
                p(rng.range(0, FRAME), rng.range(0, FRAME) + 1),
            ),
            "axis grid" => {
                let t = (i / 2 + 1) * FRAME / (half + 1);
                if i % 2 == 0 {
                    seg(p(0, t), p(FRAME, t))
                } else {
                    seg(p(t, 0), p(t, FRAME))
                }
            }
            // Every segment passes through the frame centre.
            "concurrent star" => {
                let (dx, dy) = (rng.range(-FRAME / 2, FRAME / 2), rng.range(1, FRAME / 2));
                let c = FRAME / 2;
                seg(p(c - dx, c - dy), p(c + dx, c + dy))
            }
            // Slopes i/FRAME: neighbours meet far outside, at small angles.
            "near-parallel fan" => seg(p(0, i), p(FRAME, i + i * i % 7 + 1)),
            other => unreachable!("unknown line kind {other}"),
        })
        .collect()
}

fn random_cubic(rng: &mut Rng) -> CubicBezier {
    let mut q = || p(rng.range(0, FRAME), rng.range(0, FRAME));
    CubicBezier::new(q(), q(), q(), q())
}

// ------------------------------------------------------------ trace inputs

fn trace(net: u32, a: Point2, b: Point2, width: Real) -> PcbTrace {
    PcbTrace::new(
        NetId(net),
        TraceLayer(0),
        SweptLineSegment::new(seg(a, b), width, STRICT).expect("positive width"),
        STRICT,
    )
}

/// Two parallel width-2 traces whose copper gap exceeds the required
/// clearance of 1 by `2^-k` (an irrational multiple for that kind).
fn trace_pair(kind: &str, k: u32) -> (PcbTrace, PcbTrace) {
    let excess = half_pow(k);
    let width = int(2);
    match kind {
        "axis-aligned" => {
            let y = int(3) + excess;
            (
                trace(1, p(0, 0), p(FRAME, 0), width.clone()),
                trace(2, pt(int(0), y.clone()), pt(int(FRAME), y), width),
            )
        }
        "irrational gap" => {
            let y = int(3) + excess * sqrt2();
            (
                trace(1, p(0, 0), p(FRAME, 0), width.clone()),
                trace(2, pt(int(0), y.clone()), pt(int(FRAME), y), width),
            )
        }
        "rational vertical" => {
            let x = ratio(1, 3) + int(3) + excess;
            let (y0, y1) = (ratio(1, 7), ratio(FRAME * 7 + 1, 7));
            (
                trace(
                    1,
                    pt(ratio(1, 3), y0.clone()),
                    pt(ratio(1, 3), y1.clone()),
                    width.clone(),
                ),
                trace(2, pt(x.clone(), y0), pt(x, y1), width),
            )
        }
        other => unreachable!("unknown trace kind {other}"),
    }
}

fn undecided(report: &TraceClearanceReport) -> bool {
    report.status == ClearanceStatus::Unknown
}

const DRC_KINDS: [&str; 3] = ["parallel bus", "rational parallel bus", "axis-aligned grid"];

fn drc_traces(kind: &str, n: usize, rng: &mut Rng) -> Vec<PcbTrace> {
    (0..n as u32)
        .map(|i| {
            let (x, y) = (rng.range(0, FRAME), rng.range(0, FRAME));
            let len = rng.range(16, FRAME / 8);
            match kind {
                "axis-aligned grid" => {
                    let end = if rng.coin() {
                        p(x + len, y)
                    } else {
                        p(x, y + len)
                    };
                    trace(i, p(x, y), end, int(4))
                }
                // Horizontal traces over one shared x span, on distinct rows.
                "parallel bus" => {
                    let y = i64::from(i) * 16 + rng.range(0, 8);
                    trace(i, p(0, y), p(FRAME, y), int(4))
                }
                "rational parallel bus" => {
                    let y = ratio(i64::from(i) * 160 + rng.range(0, 80), 10);
                    trace(
                        i,
                        pt(ratio(3, 10), y.clone()),
                        pt(int(FRAME), y),
                        ratio(7, 10),
                    )
                }
                other => unreachable!("unknown DRC kind {other}"),
            }
        })
        .collect()
}

fn grid_trace(i: i64, rng: &mut Rng) -> SpecctraGridTraceRecord {
    let (x, y) = (rng.range(0, FRAME), rng.range(0, FRAME));
    SpecctraGridTraceRecord {
        net: NetId((i % 64) as u32),
        layer: TraceLayer((i % 4) as u16),
        start_x: x,
        start_y: y,
        end_x: x + rng.range(1, 1000),
        end_y: if rng.coin() {
            y
        } else {
            y + rng.range(1, 1000)
        },
        width: 8,
        grid_denominator: 10,
    }
}

fn specctra_records(kind: &str, n: usize, rng: &mut Rng) -> SpecctraGridRouteRecords {
    let mut records = SpecctraGridRouteRecords::default();
    for i in 0..n as i64 {
        match (kind, i % 3) {
            ("traces", _) | ("mixed", 0) => records.traces.push(grid_trace(i, rng)),
            ("mixed", 1) => {
                let (x, y) = (rng.range(0, FRAME), rng.range(0, FRAME));
                records.vias.push(SpecctraGridViaRecord {
                    net: NetId((i % 64) as u32),
                    start_layer: TraceLayer(0),
                    end_layer: TraceLayer(3),
                    x,
                    y,
                    land_diameter: 24,
                    drill_diameter: 10,
                    drill_intent: ViaDrillIntent::Plated,
                    grid_denominator: 10,
                });
            }
            ("mixed", _) => {
                let (x, y) = (rng.range(0, FRAME), rng.range(0, FRAME));
                let s = rng.range(1, 64);
                records.arcs.push(SpecctraGridArcWireRecord {
                    net: NetId((i % 64) as u32),
                    layer: TraceLayer(1),
                    center_x: x,
                    center_y: y,
                    start_x: x + 3 * s,
                    start_y: y + 4 * s,
                    end_x: x - 3 * s,
                    end_y: y - 4 * s,
                    radius: 5 * s,
                    direction: ArcDirection::Ccw,
                    width: 8,
                    grid_denominator: 10,
                });
            }
            (other, _) => unreachable!("unknown Specctra kind {other}"),
        }
    }
    records
}

fn main() {
    let mut atlas = Atlas::from_args("hyperpath");
    // Recording dispatch labels allocates, so it runs only when asked for.
    #[cfg(feature = "dispatch-trace")]
    let _recording = atlas.config().counters.then(|| {
        atlas.counter("dispatch events", dispatch_events);
        atlas.counter("predicate events", predicate_events);
        atlas.counter("approximation events", approximation_events);
        hyperreal::dispatch_trace::recording_scope()
    });

    cam(&mut atlas);
    offsets(&mut atlas);
    arrangements(&mut atlas);
    routing(&mut atlas);
    traces(&mut atlas);
    atlas.finish();
}

fn cam(atlas: &mut Atlas) {
    let steps = powers_of_two(0, 4)
        .into_iter()
        .map(f64::log2)
        .chain((5..=14).map(f64::from))
        .collect::<Vec<_>>();
    let steps = {
        let mut s = steps;
        s.dedup();
        s
    };
    let max = usize::MAX;

    atlas.operation(
        "cam",
        "pocket rings",
        "stepover exponent k (stepover 2^-k of a unit pocket)",
        "Contour-parallel offset rings of a rectangular pocket, tool radius half the stepover; ring count grows as 2^k.",
    );
    for kind in CAM_KINDS {
        atlas.sweep(
            kind,
            &steps,
            |k, _| cam_region(kind, k as u32),
            |(pocket, step)| {
                let radius = step.clone() * ratio(1, 2);
                let report = rectangular_pocket_rings(pocket, radius, step.clone(), max, STRICT);
                Shaped::new(report, |r| {
                    r.as_ref().ok().map(|r| {
                        paths_shape(
                            r.rings.len(),
                            r.rings.last().into_iter().flat_map(|ring| {
                                [&ring.inset, &ring.min.x, &ring.max.x, &ring.max.y]
                            }),
                        )
                    })
                })
            },
        );
    }

    atlas.operation(
        "cam",
        "pocket link graph",
        "stepover exponent k (stepover 2^-k of a unit pocket)",
        "Pocket rings plus their exact side segments and ring-to-ring dogleg links.",
    );
    for kind in CAM_KINDS {
        atlas.sweep(
            kind,
            &steps,
            |k, _| cam_region(kind, k as u32),
            |(pocket, step)| {
                let radius = step.clone() * ratio(1, 2);
                let graph = rectangular_pocket_link_graph(
                    pocket.clone(),
                    radius,
                    step.clone(),
                    max,
                    STRICT,
                );
                Shaped::new(graph, |g| {
                    g.as_ref().ok().map(|g| {
                        paths_shape(
                            g.ring_segments.len() + g.links.len(),
                            g.links
                                .last()
                                .into_iter()
                                .flat_map(|l| segment_coordinates(&l.segment)),
                        )
                    })
                })
            },
        );
    }

    atlas.operation(
        "cam",
        "additive beads",
        "spacing exponent k (bead spacing 2^-k of a unit region)",
        "Exact additive bead centerlines across a rectangular region; bead width equals spacing.",
    );
    for kind in CAM_KINDS {
        atlas.sweep(
            kind,
            &steps,
            |k, _| cam_region(kind, k as u32),
            |(region, step)| {
                let report = rectangular_beads(
                    region,
                    BeadFillAxis::Horizontal,
                    step.clone(),
                    step.clone(),
                    max,
                    STRICT,
                );
                Shaped::new(report, |r| {
                    r.as_ref().ok().map(|r| {
                        paths_shape(
                            r.beads.len(),
                            r.beads.last().into_iter().map(|b| &b.pitch_position),
                        )
                    })
                })
            },
        );
    }

    atlas.operation(
        "cam",
        "serpentine infill graph",
        "spacing exponent k (bead spacing 2^-k of a unit region)",
        "Alternating-direction bead centerlines plus exact connectors, validated endpoint by endpoint.",
    );
    for kind in CAM_KINDS {
        atlas.sweep(
            kind,
            &steps,
            |k, _| cam_region(kind, k as u32),
            |(region, step)| {
                let graph = rectangular_serpentine_infill_graph(
                    region.clone(),
                    BeadFillAxis::Vertical,
                    step.clone(),
                    step.clone(),
                    max,
                    STRICT,
                );
                Shaped::new(graph, |g| {
                    g.as_ref().ok().map(|g| {
                        paths_shape(
                            g.deposition_segments.len() + g.links.len(),
                            g.links
                                .last()
                                .into_iter()
                                .flat_map(|l| segment_coordinates(&l.connector)),
                        )
                    })
                })
            },
        );
    }

    let cutters = powers_of_two(0, 9);
    atlas.operation(
        "cam",
        "rest material graph",
        "cutter count",
        "Staged rest-material subtraction of rectangular cutters from a 2^16 stock, with exact area-conservation replay per stage.",
    );
    for kind in [
        "random overlapping",
        "grid tiles",
        "full-height strips",
        "rational overlapping",
    ] {
        atlas.sweep(
            kind,
            &cutters,
            |n, rng| {
                let stock =
                    RectangularPocket::new(p(0, 0), p(FRAME, FRAME), STRICT).expect("stock");
                (stock, rest_cutters(kind, n as usize, rng))
            },
            |(stock, cutters)| {
                let graph = certified(
                    rectangular_rest_material_graph(stock.clone(), cutters.clone(), STRICT),
                    |g| g.all_area_certified(),
                );
                Shaped::new(graph, |g| {
                    g.as_ref().ok().map(|g| {
                        paths_shape(
                            g.remaining.len(),
                            g.remaining.iter().flat_map(|r| [&r.min().x, &r.max().y]),
                        )
                    })
                })
            },
        );
    }
    atlas.fuzz(
        "random overlapping",
        &powers_of_two(0, 7),
        |n, rng| {
            let n = rng.range(1, n as i64) as usize;
            let stock = RectangularPocket::new(p(0, 0), p(FRAME, FRAME), STRICT).expect("stock");
            (stock, rest_cutters("random overlapping", n, rng))
        },
        |(stock, cutters)| {
            certified(
                rectangular_rest_material_graph(stock.clone(), cutters.clone(), STRICT),
                |g| g.all_area_certified(),
            )
        },
    );

    let bits = powers_of_two(0, 12);
    for (name, description) in [
        (
            "region intersection",
            "Exact intersection of two overlapping rectangles whose coordinates carry fractions of the given size.",
        ),
        (
            "region subtraction",
            "Exact difference (up to four remainder rectangles) of two overlapping rectangles.",
        ),
    ] {
        atlas.operation("cam", name, "coordinate fraction bits", description);
        for kind in ["dyadic", "rational", "irrational"] {
            atlas.sweep(
                kind,
                &bits,
                |b, rng| bits_rects(kind, b as u32, rng),
                |(a, b)| {
                    if name == "region intersection" {
                        intersect_rectangular_regions(a.clone(), b.clone(), STRICT)
                            .map(|r| usize::from(r.intersection.is_some()))
                    } else {
                        subtract_rectangular_region(a.clone(), b.clone(), STRICT)
                            .map(|r| r.remainder.len())
                    }
                    .map_err(|e| format!("{e:?}"))
                },
            );
        }
    }
}

fn offsets(atlas: &mut Atlas) {
    let exponents = powers_of_two(0, 12);
    atlas.operation(
        "offset",
        "line offset by distance",
        "offset exponent k (distance 2^-k)",
        "Exact offset of an axis-aligned segment by a tiny distance; the offset coordinate's denominator grows with k.",
    );
    for kind in [
        "integer horizontal",
        "rational vertical",
        "irrational distance",
    ] {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| {
                let k = k as u32;
                match kind {
                    "integer horizontal" => (seg(p(0, 0), p(FRAME, 0)), half_pow(k)),
                    "rational vertical" => (
                        seg(pt(ratio(1, 3), ratio(1, 7)), pt(ratio(1, 3), ratio(9, 7))),
                        half_pow(k),
                    ),
                    _ => (seg(p(0, 0), p(FRAME, 0)), half_pow(k) * sqrt2()),
                }
            },
            |(segment, distance)| {
                let offset = offset_axis_aligned_segment(
                    segment,
                    distance.clone(),
                    OffsetSide::Left,
                    STRICT,
                );
                Shaped::new(offset, |o| {
                    o.as_ref()
                        .ok()
                        .map(|o| paths_shape(1, segment_coordinates(&o.segment)))
                })
            },
        );
    }

    let counts = powers_of_two(0, 12);
    atlas.operation(
        "offset",
        "rectilinear profile offset",
        "profile segment count",
        "Offset every segment of an axis-aligned staircase profile by 1/4 (a contour pass); path length grows with the count.",
    );
    for kind in ["integer steps", "rational steps", "irrational distance"] {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let (points, _) = route_points("axis staircase", n as usize, rng);
                let scale = if kind == "rational steps" {
                    ratio(5, 3)
                } else {
                    int(1)
                };
                let segments: Vec<_> = points
                    .windows(2)
                    .map(|w| {
                        let s = |q: &Point2| {
                            pt(q.x.clone() * scale.clone(), q.y.clone() * scale.clone())
                        };
                        seg(s(&w[0]), s(&w[1]))
                    })
                    .collect();
                let distance = if kind == "irrational distance" {
                    sqrt2() * ratio(1, 4)
                } else {
                    ratio(1, 4)
                };
                (segments, distance)
            },
            |(segments, distance)| {
                segments
                    .iter()
                    .map(|s| {
                        offset_axis_aligned_segment(s, distance.clone(), OffsetSide::Left, STRICT)
                            .is_ok()
                    })
                    .collect::<Vec<_>>()
            },
        );
    }

    let bits = powers_of_two(0, 10);
    atlas.operation(
        "offset",
        "arc offset by coordinate bits",
        "centre and radius bits",
        "Exact outward offset of a circular arc whose centre and radius scale carry the given bits; endpoints are rebuilt by exact radial scaling.",
    );
    for kind in [
        "cardinal arc",
        "explicit 3-4-5 arc",
        "explicit rational centre",
    ] {
        atlas.sweep(
            kind,
            &bits,
            |b, rng| {
                let b = b as u32;
                let (cx, cy, s) = (big(b, rng), big(b, rng), big(b, rng));
                match kind {
                    "cardinal arc" => AnyArc::Cardinal(
                        CircularArc::cardinal(
                            pt(cx, cy),
                            s,
                            CardinalPoint::East,
                            CardinalPoint::West,
                            ArcDirection::Ccw,
                            STRICT,
                        )
                        .expect("cardinal arc"),
                    ),
                    "explicit 3-4-5 arc" => AnyArc::Explicit(half_arc(cx, cy, s)),
                    _ => {
                        let d = big(b, rng) + int(1);
                        let cx = (cx / d.clone()).unwrap_or_else(|_| unreachable!("positive"));
                        let cy = (cy / d).unwrap_or_else(|_| unreachable!("positive"));
                        AnyArc::Explicit(half_arc(cx, cy, s))
                    }
                }
            },
            |arc| match arc {
                AnyArc::Cardinal(cardinal) => {
                    offset_cardinal_arc(cardinal, int(1), OffsetSide::Left, STRICT).is_ok()
                }
                AnyArc::Explicit(explicit) => {
                    offset_explicit_arc(explicit, int(1), OffsetSide::Left, STRICT).is_ok()
                }
            },
        );
    }

    atlas.operation(
        "offset",
        "arc inward offset near collapse",
        "collapse exponent k (offset = radius - 2^-k)",
        "Inward offset of a radius-5 arc that leaves a 2^-k radius: distance to the degenerate (collapsed) offset.",
    );
    for kind in ["cardinal arc", "explicit 3-4-5 arc", "irrational radius"] {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| {
                let left = half_pow(k as u32);
                match kind {
                    "cardinal arc" => (
                        AnyArc::Cardinal(
                            CircularArc::cardinal(
                                p(0, 0),
                                int(5),
                                CardinalPoint::East,
                                CardinalPoint::West,
                                ArcDirection::Ccw,
                                STRICT,
                            )
                            .expect("cardinal arc"),
                        ),
                        int(5) - left,
                    ),
                    "explicit 3-4-5 arc" => (
                        AnyArc::Explicit(half_arc(int(0), int(0), int(1))),
                        int(5) - left,
                    ),
                    _ => {
                        let r = sqrt2();
                        (
                            AnyArc::Cardinal(
                                CircularArc::cardinal(
                                    p(0, 0),
                                    r.clone(),
                                    CardinalPoint::East,
                                    CardinalPoint::West,
                                    ArcDirection::Ccw,
                                    STRICT,
                                )
                                .expect("cardinal arc"),
                            ),
                            r - left,
                        )
                    }
                }
            },
            |(arc, distance)| match arc {
                AnyArc::Cardinal(cardinal) => {
                    offset_cardinal_arc(cardinal, distance.clone(), OffsetSide::Right, STRICT)
                        .map(|_| ())
                        .map_err(|e| format!("{e:?}"))
                }
                AnyArc::Explicit(explicit) => {
                    offset_explicit_arc(explicit, distance.clone(), OffsetSide::Right, STRICT)
                        .map(|_| ())
                        .map_err(|e| format!("{e:?}"))
                }
            },
        );
    }

    atlas.operation(
        "offset",
        "Bézier offset sample by coordinate bits",
        "control-point bits",
        "Offset point of a Bézier at t = 1/3 by distance 1 (exact normal through a square root), approximated to f64 as a G-code emitter would.",
    );
    for (kind, degree) in [
        ("quadratic", 2),
        ("cubic", 3),
        ("quartic", 4),
        ("quintic", 5),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            |b, rng| {
                (0..=degree)
                    .map(|i| pt(big(b as u32, rng) + int(i), big(b as u32, rng) + int(i * i)))
                    .collect::<Vec<_>>()
            },
            |controls| {
                let t = BezierParameter::new(1, 3).expect("t in [0, 1]");
                let c = controls;
                match degree {
                    2 => offset_quadratic_bezier_sample(
                        &QuadraticBezier::new(c[0].clone(), c[1].clone(), c[2].clone()),
                        t,
                        int(1),
                        OffsetSide::Left,
                        STRICT,
                    ),
                    3 => offset_cubic_bezier_sample(
                        &CubicBezier::new(c[0].clone(), c[1].clone(), c[2].clone(), c[3].clone()),
                        t,
                        int(1),
                        OffsetSide::Left,
                        STRICT,
                    ),
                    _ => offset_higher_order_bezier_sample(
                        &HigherOrderBezier::try_new(c.clone()).expect("quartic or quintic"),
                        t,
                        int(1),
                        OffsetSide::Left,
                        STRICT,
                    ),
                }
                .ok()
                .and_then(|o| Some((o.point.x.to_f64_lossy()?, o.point.y.to_f64_lossy()?)))
            },
        );
    }

    atlas.operation(
        "offset",
        "Bézier offset samples along a curve",
        "sample count",
        "Offset points at n evenly spaced parameters along one curve, each approximated to f64.",
    );
    for kind in ["quadratic", "cubic"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            |n, rng| {
                let mut q = || p(rng.range(0, FRAME), rng.range(0, FRAME));
                let curve = if kind == "quadratic" {
                    Curve::Quadratic(QuadraticBezier::new(q(), q(), q()))
                } else {
                    Curve::Cubic(CubicBezier::new(q(), q(), q(), q()))
                };
                (curve, n as u64)
            },
            |(curve, n)| {
                (0..=*n)
                    .map(|i| {
                        let t = BezierParameter::new(i as i64, *n).expect("t in [0, 1]");
                        match curve {
                            Curve::Quadratic(q) => offset_quadratic_bezier_sample(
                                q,
                                t,
                                int(1),
                                OffsetSide::Left,
                                STRICT,
                            ),
                            Curve::Cubic(c) => {
                                offset_cubic_bezier_sample(c, t, int(1), OffsetSide::Left, STRICT)
                            }
                        }
                        .ok()
                        .and_then(|o| o.point.x.to_f64_lossy())
                    })
                    .collect::<Vec<_>>()
            },
        );
    }
}

fn arrangements(atlas: &mut Atlas) {
    let counts = powers_of_two(1, 9);
    atlas.operation(
        "arrangement",
        "line arrangement",
        "segment count",
        "Exact line-segment arrangement: events, split fragments and the retained cell graph.",
    );
    for kind in LINE_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| lines_of(kind, n as usize, rng),
            |lines| {
                Shaped::new(arrange_line_segments(lines, STRICT), |r| {
                    r.as_ref().ok().map(|r| {
                        paths_shape(
                            r.fragments.len() + r.cell_graph.faces.len(),
                            r.fragments
                                .iter()
                                .flat_map(|f| segment_coordinates(&f.segment)),
                        )
                    })
                })
            },
        );
    }

    let arcs = powers_of_two(1, 8);
    atlas.operation(
        "arrangement",
        "arc arrangement",
        "arc count",
        "Exact explicit-arc arrangement: arc/arc events, split sub-arcs and the curve cell graph.",
    );
    for kind in ARC_KINDS {
        atlas.sweep(
            kind,
            &arcs,
            |n, rng| arcs_of(kind, n as usize, rng),
            |arcs| {
                Shaped::new(arrange_explicit_arcs(arcs, STRICT), |r| {
                    r.as_ref().ok().map(|r| {
                        paths_shape(
                            r.fragments.len() + r.cell_graph.faces.len(),
                            r.fragments
                                .iter()
                                .flat_map(|f| [&f.arc.start().x, &f.arc.start().y]),
                        )
                    })
                })
            },
        );
    }

    let few = powers_of_two(0, 6);
    atlas.operation(
        "arrangement",
        "line and arc arrangement",
        "line count",
        "Mixed line/arc arrangement of random segments and random half-arcs.",
    );
    atlas.grid(
        "random",
        ("line count", &few),
        ("arc count", &few),
        |n, m, rng| {
            (
                lines_of("random", n as usize, rng),
                arcs_of("random half-arcs", m as usize, rng),
            )
        },
        |(lines, arcs)| arrange_line_segments_with_explicit_arcs(lines, arcs, STRICT).is_ok(),
    );
    for kind in ["random", "axis grid"] {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                (
                    lines_of(kind, n as usize, rng),
                    arcs_of("random half-arcs", 4, rng),
                )
            },
            |(lines, arcs)| {
                Shaped::new(
                    arrange_line_segments_with_explicit_arcs(lines, arcs, STRICT),
                    |r| {
                        r.as_ref().ok().map(|r| {
                            paths_shape(
                                r.line_fragments.len() + r.arc_fragments.len(),
                                r.arc_fragments
                                    .iter()
                                    .flat_map(|f| [&f.arc.start().x, &f.arc.start().y]),
                            )
                        })
                    },
                )
            },
        );
    }

    atlas.operation(
        "arrangement",
        "line and cubic arrangement",
        "line count",
        "Lines against cubic Béziers with algebraic intersection evidence and breakpoint promotion.",
    );
    for kind in ["random", "axis grid"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 7),
            |n, rng| (lines_of(kind, n as usize, rng), vec![random_cubic(rng)]),
            |(lines, curves)| {
                Shaped::new(
                    arrange_line_segments_with_cubic_beziers(lines, curves, STRICT),
                    |r| {
                        r.as_ref().ok().map(|r| Shape {
                            nodes: r.events.len() as u64,
                            ..Shape::default()
                        })
                    },
                )
            },
        );
    }
    atlas.grid(
        "random",
        ("line count", &powers_of_two(0, 5)),
        ("cubic count", &powers_of_two(0, 4)),
        |n, m, rng| {
            (
                lines_of("random", n as usize, rng),
                (0..m as usize)
                    .map(|_| random_cubic(rng))
                    .collect::<Vec<_>>(),
            )
        },
        |(lines, curves)| arrange_line_segments_with_cubic_beziers(lines, curves, STRICT).is_ok(),
    );
}

fn routing(atlas: &mut Atlas) {
    let lengths = powers_of_two(0, 12);
    atlas.operation(
        "routing",
        "constant feed time",
        "segment count",
        "Certify the traversal time of an n-segment route at constant feed; unit diagonals have irrational lengths.",
    );
    for kind in ROUTE_KINDS {
        atlas.sweep(
            kind,
            &lengths,
            |n, rng| route(kind, n as usize, rng),
            |(segments, length)| {
                let time = (length.clone() / int(2)).map_err(|e| format!("{e:?}"))?;
                let path: Vec<_> = segments
                    .iter()
                    .cloned()
                    .map(FeedPathElement::Line)
                    .collect();
                certified(
                    certify_constant_feed_time_for_path(&path, int(2), time, STRICT),
                    |r| r.certification.all_satisfied(),
                )
                .map(|_| ())
            },
        );
    }

    atlas.operation(
        "routing",
        "acceleration-limited feed time",
        "segment count",
        "Certify a trapezoidal feed profile (feed 1, acceleration 1) over the route length.",
    );
    for kind in ROUTE_KINDS {
        atlas.sweep(
            kind,
            &lengths,
            |n, rng| route(kind, n as usize, rng),
            |(segments, length)| {
                let path: Vec<_> = segments
                    .iter()
                    .cloned()
                    .map(FeedPathElement::Line)
                    .collect();
                certified(
                    certify_acceleration_limited_feed_time_for_path(
                        &path,
                        int(1),
                        int(1),
                        length.clone() + int(1),
                        STRICT,
                    ),
                    |r| r.certification.all_satisfied(),
                )
                .map(|_| ())
            },
        );
    }

    let spans = powers_of_two(1, 10);
    atlas.operation(
        "routing",
        "lookahead feed plan",
        "span count",
        "Forward/reverse exact lookahead feed planning with Hypersolve replay of every node and span.",
    );
    for kind in LOOKAHEAD_KINDS {
        atlas.sweep(
            kind,
            &spans,
            |n, rng| lookahead_input(kind, n as usize, rng),
            |(route, spans, limits)| {
                certified(
                    plan_lookahead_feed_schedule(route, spans, limits, int(8), int(4), STRICT),
                    |p| p.all_satisfied(),
                )
                .map(|p| p.schedule.corner_feeds.len())
            },
        );
    }

    atlas.operation(
        "routing",
        "jerk-feasible lookahead plan",
        "span count",
        "Lookahead planning refined by exact halving until every monotonic jerk transition certifies (at most 16 halvings).",
    );
    for kind in LOOKAHEAD_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(1, 8),
            |n, rng| lookahead_input(kind, n as usize, rng),
            |(route, spans, limits)| {
                certified(
                    plan_jerk_feasible_lookahead_schedule(
                        route,
                        spans,
                        limits,
                        int(8),
                        int(64),
                        int(1),
                        16,
                        STRICT,
                    ),
                    |p| p.all_satisfied(),
                )
                .map(|p| p.positive_node_components.len())
            },
        );
    }

    let axis_limits = |count: usize| {
        (0..count)
            .map(|i| AxisMotionLimits {
                maximum_velocity: int(100 + i as i64),
                maximum_acceleration: int(10 + i as i64),
                maximum_jerk: int(1 + i as i64),
            })
            .collect::<Vec<_>>()
    };
    let projections = |spans: usize, axes: usize, rational: bool, rng: &mut Rng| {
        (0..spans)
            .map(|_| AffineSpanAxisProjection {
                absolute_axis_derivatives: (0..axes)
                    .map(|_| {
                        if rational {
                            ratio(rng.range(0, 1000), 2 * rng.range(1, 500) + 1)
                        } else {
                            ratio(rng.range(0, 1 << 10), 1 << 10)
                        }
                    })
                    .collect(),
            })
            .collect::<Vec<_>>()
    };
    atlas.operation(
        "routing",
        "axis-projected motion limits",
        "span count",
        "Convert per-axis velocity/acceleration/jerk limits into route-wide limits over n affine spans, replaying every span/axis inequality.",
    );
    for (kind, axes, rational) in [
        ("3 axes, dyadic", 3, false),
        ("3 axes, rational", 3, true),
        ("5 axes, rational", 5, true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 11),
            |n, rng| {
                (
                    projections(n as usize, axes, rational, rng),
                    axis_limits(axes),
                )
            },
            |(projections, limits)| {
                certified(
                    plan_axis_projected_motion_limits(projections, limits, STRICT),
                    |p| p.all_satisfied(),
                )
                .map(|_| ())
            },
        );
    }
    atlas.grid(
        "rational",
        ("span count", &powers_of_two(0, 8)),
        ("axis count", &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]),
        |n, a, rng| {
            (
                projections(n as usize, a as usize, true, rng),
                axis_limits(a as usize),
            )
        },
        |(projections, limits)| {
            plan_axis_projected_motion_limits(projections, limits, STRICT).is_ok()
        },
    );

    atlas.operation(
        "routing",
        "tangent chain classification",
        "span count",
        "Exact tangent-join classification across an n-span chain (collinear joins are G1, staircase joins are corners).",
    );
    for kind in ROUTE_KINDS {
        atlas.sweep(
            kind,
            &lengths,
            |n, rng| {
                route(kind, n as usize, rng)
                    .0
                    .iter()
                    .map(TangentSpan::from_line_segment)
                    .collect::<Vec<_>>()
            },
            |spans| classify_tangent_chain(spans, STRICT).joins.len(),
        );
    }

    atlas.operation(
        "routing",
        "G1 chain certification",
        "span count",
        "Hypersolve certification of G1 continuity over every join of an n-span collinear chain.",
    );
    for kind in ["collinear", "3-4-5 collinear"] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 10),
            |n, rng| {
                route(kind, n as usize, rng)
                    .0
                    .iter()
                    .map(TangentSpan::from_line_segment)
                    .collect::<Vec<_>>()
            },
            |spans| certify_g1_chain(spans).all_certified(),
        );
    }

    atlas.operation(
        "routing",
        "tangent join near alignment",
        "angle exponent k (direction (1, 2^-k))",
        "Classify the join of two spans whose directions differ by about 2^-k radians: distance to an exact G1 join.",
    );
    for kind in ["dyadic", "rational", "irrational"] {
        atlas.sweep(
            kind,
            &exponents_for_join(),
            |k, _| {
                let tilt = match kind {
                    "dyadic" => half_pow(k as u32),
                    "rational" => half_pow(k as u32) * ratio(5, 7),
                    _ => half_pow(k as u32) * sqrt2(),
                };
                (p(1, 0), pt(int(1), tilt))
            },
            |(first, second)| {
                classify_tangent_join(&p(1, 0), first, &p(1, 0), second, STRICT)
                    .alignment
                    .is_some()
            },
        );
    }

    atlas.operation(
        "routing",
        "differential pair skew",
        "segment count per route",
        "Certify the exact length skew between two n-segment axis-aligned routes (the only geometry it accepts).",
    );
    for kind in ["collinear", "axis staircase"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 11),
            |n, rng| {
                let (a, la) = route(kind, n as usize, rng);
                let (b, lb) = route(kind, n as usize, rng);
                (a, b, la - lb)
            },
            |(a, b, skew)| {
                certified(
                    certify_differential_pair_skew(a, b, skew.clone(), STRICT),
                    |r| r.certification.all_satisfied(),
                )
                .map(|_| ())
            },
        );
    }

    atlas.operation(
        "routing",
        "length-tuning meander",
        "bump count",
        "Build an exact length-tuning meander on a 2^16 trace adding length 2^12.",
    );
    for kind in ["uniform", "alternating", "nonuniform"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 12),
            |n, rng| {
                let amplitudes = (0..n as usize)
                    .map(|_| int(rng.range(1, 64)))
                    .collect::<Vec<_>>();
                (seg(p(0, 0), p(FRAME, 0)), n as u64, amplitudes)
            },
            |(source, bumps, amplitudes)| {
                let segments = match kind {
                    "uniform" => {
                        multi_detour_meander(source, int(4096), *bumps, OffsetSide::Left, STRICT)
                            .map(|m| m.segments.len())
                    }
                    "alternating" => alternating_detour_meander(
                        source,
                        int(4096),
                        *bumps,
                        OffsetSide::Left,
                        STRICT,
                    )
                    .map(|m| m.segments.len()),
                    _ => nonuniform_detour_meander(
                        source,
                        amplitudes.clone(),
                        OffsetSide::Left,
                        STRICT,
                    )
                    .map(|m| m.segments.len()),
                };
                segments.map_err(|e| format!("{e:?}"))
            },
        );
    }
}

fn exponents_for_join() -> Vec<f64> {
    powers_of_two(0, 12)
}

fn traces(atlas: &mut Atlas) {
    let exponents = powers_of_two(0, 12);
    atlas.operation(
        "trace",
        "trace clearance near the limit",
        "gap exponent k (gap exceeds clearance by 2^-k)",
        "Certify clearance between two parallel width-2 traces whose copper gap exceeds the required clearance by 2^-k.",
    );
    for kind in ["axis-aligned", "rational vertical", "irrational gap"] {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| trace_pair(kind, k as u32),
            |(a, b)| check_trace_clearance(a, b, &int(1), STRICT).is_certified_clear(),
        );
    }

    atlas.operation(
        "trace",
        "trace to pad clearance near the limit",
        "gap exponent k (gap exceeds clearance by 2^-k)",
        "Certify clearance between a trace and a pad or via whose copper gap exceeds the required clearance by 2^-k.",
    );
    for kind in ["circular pad", "via land", "rectangular pad"] {
        atlas.sweep(
            kind,
            &exponents,
            |k, _| {
                let t = trace(1, p(0, 0), p(FRAME, 0), int(2));
                // Pad edge at y = 1 + 1 + 2^-k above the trace edge y = 1.
                let y = int(4) + half_pow(k as u32);
                (t, y)
            },
            |(t, y)| {
                let centre = pt(int(FRAME / 2), y.clone());
                let report = match kind {
                    "circular pad" => {
                        let pad =
                            PcbCircularPad::new(NetId(2), TraceLayer(0), centre, int(4), STRICT)
                                .ok()?;
                        check_trace_pad_clearance(t, &pad, &int(1), STRICT)
                    }
                    "via land" => {
                        let via = PcbViaStack::new(
                            NetId(2),
                            TraceLayer(0),
                            TraceLayer(3),
                            centre,
                            int(4),
                            STRICT,
                        )
                        .ok()?;
                        check_trace_via_clearance(t, &via, &int(1), STRICT)
                    }
                    _ => {
                        let pad = PcbRectPad::new(
                            NetId(2),
                            TraceLayer(0),
                            centre,
                            int(8),
                            int(4),
                            STRICT,
                        )
                        .ok()?;
                        check_trace_rect_pad_clearance(t, &pad, &int(1), STRICT)
                    }
                };
                Some(report.is_certified_clear())
            },
        );
    }

    atlas.operation(
        "trace",
        "all-pairs trace clearance",
        "trace count",
        "Design-rule check of every trace pair (n^2/2 clearance reports) on one layer; a decided violation counts as success, an undecided pair as failure.",
    );
    for kind in DRC_KINDS {
        atlas.sweep(
            kind,
            &ladder(2, 512, 10),
            |n, rng| drc_traces(kind, n as usize, rng),
            |traces| {
                let (mut pairs, mut unknown) = (0usize, 0usize);
                for (i, a) in traces.iter().enumerate() {
                    for b in &traces[i + 1..] {
                        pairs += 1;
                        let report = check_trace_clearance(a, b, &int(2), STRICT);
                        unknown += usize::from(undecided(&report));
                    }
                }
                if unknown == 0 {
                    Ok(pairs)
                } else {
                    Err(format!("{unknown} of {pairs} pairs undecided"))
                }
            },
        );
    }

    let records = powers_of_two(0, 12);
    atlas.operation(
        "trace",
        "Specctra route parse",
        "record count",
        "Parse fixed-grid Specctra route text (serialized outside the timer) into grid records.",
    );
    for kind in ["traces", "mixed"] {
        atlas.sweep(
            kind,
            &records,
            |n, rng| {
                serialize_specctra_grid_route_records(&specctra_records(kind, n as usize, rng))
            },
            |text| parse_specctra_grid_route_records(text).map(|r| r.traces.len()),
        );
    }

    atlas.operation(
        "trace",
        "Specctra route import",
        "record count",
        "Parse Specctra route text and lift every trace, via and arc to exact path values.",
    );
    for kind in ["traces", "mixed"] {
        atlas.sweep(
            kind,
            &records,
            |n, rng| {
                serialize_specctra_grid_route_records(&specctra_records(kind, n as usize, rng))
            },
            |text| {
                Shaped::new(import_specctra_text_route(text, STRICT), |r| {
                    r.as_ref().ok().map(|r| Shape {
                        nodes: (r.traces().len() + r.vias().len() + r.arcs().len()) as u64,
                        ..Shape::default()
                    })
                })
            },
        );
    }

    atlas.operation(
        "trace",
        "Specctra rule clearance audit",
        "trace count",
        "Audit every same-layer trace pair against a fixed-grid clearance rule.",
    );
    atlas.sweep(
        "one global rule",
        &ladder(2, 512, 10),
        |n, rng| {
            let traces = (0..n as i64)
                .map(|i| specctra_grid_trace_record(grid_trace(i, rng)).expect("grid trace"))
                .collect::<Vec<_>>();
            let rule = specctra_grid_route_rule_record(SpecctraGridRouteRuleRecord {
                net: None,
                layer: None,
                clearance: 6,
                width: 8,
                grid_denominator: 10,
            })
            .expect("grid rule");
            (traces, vec![rule])
        },
        |(traces, rules)| {
            Shaped::new(
                audit_specctra_trace_rule_clearances(traces, rules, STRICT),
                |r| {
                    r.as_ref().ok().map(|r| Shape {
                        nodes: r.pairs.len() as u64,
                        ..Shape::default()
                    })
                },
            )
        },
    );
}
