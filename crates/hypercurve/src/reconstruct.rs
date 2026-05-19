//! Polyline-to-curve reconstruction.
//!
//! The routines in this module are intentionally lossy import helpers. They
//! live at the IO boundary: sampled points are inspected through finite `f64`
//! approximations, then promoted back into native line and circular-arc
//! segments once a run has been classified.
//!
//! Arc promotion uses local finite-difference curvature witnesses. The
//! three-point circle behind that witness is the reciprocal-radius idea of
//! Menger curvature; see Menger, "Untersuchungen über allgemeine Metrik"
//! (*Mathematische Annalen* 100, 75-163, 1928). The code chooses a deterministic
//! streaming circumcircle instead of a multi-point least-squares fit; Kåsa,
//! "A Circle Fitting Procedure and Its Error Analysis" (*IEEE Transactions on
//! Instrumentation and Measurement* IM-25(1), 8-14, 1976), is cited near the
//! circumcircle construction as the relevant least-squares alternative.

use std::f64::consts::PI;

use hyperreal::Real;

use crate::{BulgeVertex2, Contour2, CurveError, CurveResult, CurveString2, FillRule, Point2};

const DEFAULT_DISTANCE_TOLERANCE: f64 = 1e-6;
const DEFAULT_RELATIVE_TOLERANCE: f64 = 1e-9;
const DEFAULT_COLLINEAR_TOLERANCE: f64 = 1e-7;
const DEFAULT_DUPLICATE_POINT_TOLERANCE: f64 = 1e-12;
const DEFAULT_MIN_ARC_POINTS: usize = 4;
const MIN_ARC_POINTS: usize = 3;

/// Controls reconstruction of line and circular-arc segments from sampled
/// polyline points.
///
/// The defaults are conservative: nearly-collinear samples are merged into
/// long line segments, while arc promotion requires at least four points so a
/// single corner triplet is not interpreted as curvature.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PolylineReconstructionOptions {
    /// Maximum absolute point-to-line or radial point-to-circle error accepted
    /// while extending a candidate run.
    pub distance_tolerance: f64,
    /// Relative tolerance scaled by the candidate chord length or radius.
    pub relative_tolerance: f64,
    /// Dimensionless signed-area tolerance used to treat a three-point finite
    /// difference as collinear.
    pub collinear_tolerance: f64,
    /// Distance below which adjacent input samples are treated as duplicate
    /// polyline points.
    pub duplicate_point_tolerance: f64,
    /// Minimum number of sampled points required before a run can be promoted
    /// to a circular arc.
    pub min_arc_points: usize,
}

/// Certificate for importing finite `f64` polyline/ring coordinates.
///
/// This certificate records the API-boundary promotion from primitive floats
/// into native curve objects. It does not make the input samples authoritative
/// for topology; it records how many finite points were accepted, whether a
/// closing ring sample was consumed, and how many native line segments were
/// emitted. That separation follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>): exact topology starts
/// after finite adapter data has crossed into exact-aware objects.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FiniteImportCertificate {
    input_point_count: usize,
    retained_point_count: usize,
    skipped_duplicate_edge_count: usize,
    repeated_closing_point: bool,
    output_segment_count: usize,
    closed: bool,
}

/// Result of importing a finite open line string.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteCurveStringImport2 {
    curve: CurveString2,
    certificate: FiniteImportCertificate,
}

/// Result of importing a finite closed line ring.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteContourImport2 {
    contour: Contour2,
    certificate: FiniteImportCertificate,
}

impl PolylineReconstructionOptions {
    /// Constructs conservative reconstruction options with a custom absolute
    /// distance tolerance.
    pub const fn new(distance_tolerance: f64) -> Self {
        Self {
            distance_tolerance,
            ..Self::DEFAULT
        }
    }

    /// Default reconstruction options.
    pub const DEFAULT: Self = Self {
        distance_tolerance: DEFAULT_DISTANCE_TOLERANCE,
        relative_tolerance: DEFAULT_RELATIVE_TOLERANCE,
        collinear_tolerance: DEFAULT_COLLINEAR_TOLERANCE,
        duplicate_point_tolerance: DEFAULT_DUPLICATE_POINT_TOLERANCE,
        min_arc_points: DEFAULT_MIN_ARC_POINTS,
    };

    fn validate(self) -> CurveResult<Self> {
        let finite_nonnegative = [
            self.distance_tolerance,
            self.relative_tolerance,
            self.collinear_tolerance,
            self.duplicate_point_tolerance,
        ]
        .into_iter()
        .all(|value| value.is_finite() && value >= 0.0);

        if !finite_nonnegative || self.min_arc_points < MIN_ARC_POINTS {
            return Err(CurveError::InvalidReconstructionOptions);
        }
        Ok(self)
    }

    fn distance_limit(self, scale: f64) -> f64 {
        self.distance_tolerance + self.relative_tolerance * scale.abs()
    }
}

impl FiniteImportCertificate {
    /// Returns the number of finite points supplied at the API boundary.
    pub const fn input_point_count(&self) -> usize {
        self.input_point_count
    }

    /// Returns the number of finite points retained after boundary normalization.
    pub const fn retained_point_count(&self) -> usize {
        self.retained_point_count
    }

    /// Returns the number of adjacent duplicate open-line edges skipped.
    pub const fn skipped_duplicate_edge_count(&self) -> usize {
        self.skipped_duplicate_edge_count
    }

    /// Returns true when a repeated final ring point was consumed.
    pub const fn repeated_closing_point(&self) -> bool {
        self.repeated_closing_point
    }

    /// Returns the number of native segments emitted.
    pub const fn output_segment_count(&self) -> usize {
        self.output_segment_count
    }

    /// Returns true when the import target was a closed contour.
    pub const fn is_closed(&self) -> bool {
        self.closed
    }
}

impl FiniteCurveStringImport2 {
    /// Returns the imported native curve string.
    pub const fn curve_string(&self) -> &CurveString2 {
        &self.curve
    }

    /// Consumes the report and returns the imported native curve string.
    pub fn into_curve_string(self) -> CurveString2 {
        self.curve
    }

    /// Returns the import certificate.
    pub const fn certificate(&self) -> &FiniteImportCertificate {
        &self.certificate
    }
}

impl FiniteContourImport2 {
    /// Returns the imported native contour.
    pub const fn contour(&self) -> &Contour2 {
        &self.contour
    }

    /// Consumes the report and returns the imported native contour.
    pub fn into_contour(self) -> Contour2 {
        self.contour
    }

    /// Returns the import certificate.
    pub const fn certificate(&self) -> &FiniteImportCertificate {
        &self.certificate
    }
}

impl Default for PolylineReconstructionOptions {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BulgeVertex2 {
    /// Reconstructs exact bulge vertices from an open sampled polyline.
    ///
    /// Flat runs are collapsed to one zero-bulge line segment. Runs with
    /// consistent three-point finite curvature are represented as circular
    /// arcs with `|bulge| <= 1`, splitting naturally at semicircle boundaries.
    pub fn reconstruct_polyline(
        points: &[Point2],
        options: PolylineReconstructionOptions,
    ) -> CurveResult<Vec<Self>> {
        let options = options.validate()?;
        let samples = sample_open_points(points, options)?;
        if samples.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let spans = reconstruct_spans(&samples, options)?;
        let mut vertices = Vec::with_capacity(spans.len() + 1);
        for span in &spans {
            vertices.push(BulgeVertex2::new(
                samples[span.start].point.clone(),
                real_from_f64(span.bulge)?,
            ));
        }
        vertices.push(BulgeVertex2::new(
            samples[samples.len() - 1].point.clone(),
            Real::zero(),
        ));
        Ok(vertices)
    }
}

impl CurveString2 {
    /// Constructs an open line-segment curve string from finite `f64` points.
    ///
    /// This is an API-boundary import adapter: primitive floats are accepted at
    /// the boundary, immediately promoted to [`Real`], and stored as native
    /// line geometry before any topology-sensitive operation runs. That follows
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    /// Unlike [`CurveString2::reconstruct_from_polyline`], this constructor
    /// makes no attempt to infer arcs from samples.
    pub fn from_finite_line_string(points: &[[f64; 2]]) -> CurveResult<Self> {
        Self::import_finite_line_string(points).map(FiniteCurveStringImport2::into_curve_string)
    }

    /// Constructs an open line-segment curve string from an iterator of finite
    /// `f64` points.
    ///
    /// This is the ownership-friendly counterpart to
    /// [`CurveString2::from_finite_line_string`] for callers that generate
    /// finite boundary samples lazily. The samples are still a boundary import:
    /// they are collected, promoted to [`Real`], and stored as native line
    /// geometry before topology-sensitive work proceeds. This follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
    /// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_finite_point_iter<I>(points: I) -> CurveResult<Self>
    where
        I: IntoIterator<Item = [f64; 2]>,
    {
        let points = points.into_iter().collect::<Vec<_>>();
        Self::from_finite_line_string(&points)
    }

    /// Imports an open finite line string and returns an audit certificate.
    ///
    /// Adjacent duplicate finite points are not represented as zero-length
    /// native segments; they are counted in
    /// [`FiniteImportCertificate::skipped_duplicate_edge_count`]. All
    /// non-duplicate points are promoted through [`Real`] before constructing
    /// exact-aware line segments.
    pub fn import_finite_line_string(points: &[[f64; 2]]) -> CurveResult<FiniteCurveStringImport2> {
        if points.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(points.len() - 1);
        let mut skipped_duplicate_edge_count = 0;
        for edge in points.windows(2) {
            let start = point_from_finite_xy(edge[0])?;
            let end = point_from_finite_xy(edge[1])?;
            if let Ok(line) = crate::LineSeg2::try_new(start, end) {
                segments.push(crate::Segment2::Line(line));
            } else {
                skipped_duplicate_edge_count += 1;
            }
        }
        let curve = Self::try_new(segments)?;
        let certificate = FiniteImportCertificate {
            input_point_count: points.len(),
            retained_point_count: points.len(),
            skipped_duplicate_edge_count,
            repeated_closing_point: false,
            output_segment_count: curve.len(),
            closed: false,
        };
        Ok(FiniteCurveStringImport2 { curve, certificate })
    }

    /// Reconstructs an open curve string from sampled polyline points.
    ///
    /// This is a finite-precision import helper. It is useful after tracing,
    /// digitizing, tessellating, or user-editing a dense point polyline and
    /// before running exact topology on the reconstructed line/arc model.
    pub fn reconstruct_from_polyline(
        points: &[Point2],
        options: PolylineReconstructionOptions,
    ) -> CurveResult<Self> {
        let vertices = BulgeVertex2::reconstruct_polyline(points, options)?;
        Self::from_bulge_vertices(&vertices)
    }
}

impl Contour2 {
    /// Constructs a closed straight-segment contour from finite `f64` ring points.
    ///
    /// A repeated final point equal to the first point is accepted and removed
    /// before native contour construction. This is the closed-ring counterpart
    /// to [`CurveString2::from_finite_line_string`]; it imports finite boundary
    /// coordinates as exact-aware line topology without fitting arcs.
    pub fn from_finite_ring(points: &[[f64; 2]]) -> CurveResult<Self> {
        Self::import_finite_ring(points).map(FiniteContourImport2::into_contour)
    }

    /// Constructs a closed straight-segment contour from finite `f64` ring
    /// points and an explicit fill rule.
    pub fn from_finite_ring_with_fill_rule(
        points: &[[f64; 2]],
        fill_rule: FillRule,
    ) -> CurveResult<Self> {
        Self::import_finite_ring_with_fill_rule(points, fill_rule)
            .map(FiniteContourImport2::into_contour)
    }

    /// Imports a closed finite line ring and returns an audit certificate.
    ///
    /// A repeated final point equal to the first point is accepted as finite
    /// file-format closure metadata and recorded in the certificate. The
    /// emitted native contour remains the exact topology carrier.
    pub fn import_finite_ring(points: &[[f64; 2]]) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring_with_fill_rule(points, FillRule::NonZero)
    }

    /// Imports a closed finite line ring with an explicit fill rule and returns
    /// an audit certificate.
    pub fn import_finite_ring_with_fill_rule(
        points: &[[f64; 2]],
        fill_rule: FillRule,
    ) -> CurveResult<FiniteContourImport2> {
        if points.len() < 3 {
            return Err(CurveError::InsufficientVertices);
        }

        let repeated_closing_point = points.len() > 1 && points.first() == points.last();
        let end = if repeated_closing_point {
            points.len() - 1
        } else {
            points.len()
        };
        if end < 3 {
            return Err(CurveError::InsufficientVertices);
        }

        let vertices = points
            .iter()
            .take(end)
            .map(|point| {
                Ok(BulgeVertex2::new(
                    point_from_finite_xy(*point)?,
                    Real::zero(),
                ))
            })
            .collect::<CurveResult<Vec<_>>>()?;
        let contour = Self::from_bulge_vertices_with_fill_rule(&vertices, fill_rule)?;
        let certificate = FiniteImportCertificate {
            input_point_count: points.len(),
            retained_point_count: end,
            skipped_duplicate_edge_count: 0,
            repeated_closing_point,
            output_segment_count: contour.len(),
            closed: true,
        };
        Ok(FiniteContourImport2 {
            contour,
            certificate,
        })
    }

    /// Reconstructs a closed contour from sampled polyline points using the
    /// non-zero fill rule.
    ///
    /// The input may include or omit a repeated final point equal to the first
    /// point. Reconstruction is performed on the explicit closed sample chain.
    pub fn reconstruct_from_closed_polyline(
        points: &[Point2],
        options: PolylineReconstructionOptions,
    ) -> CurveResult<Self> {
        Self::reconstruct_from_closed_polyline_with_fill_rule(points, options, FillRule::NonZero)
    }

    /// Reconstructs a closed contour from sampled polyline points with an
    /// explicit fill rule.
    pub fn reconstruct_from_closed_polyline_with_fill_rule(
        points: &[Point2],
        options: PolylineReconstructionOptions,
        fill_rule: FillRule,
    ) -> CurveResult<Self> {
        let options = options.validate()?;
        let mut samples = sample_open_points(points, options)?;
        if samples.len() >= 2
            && distance(&samples[0], &samples[samples.len() - 1])
                <= options.duplicate_point_tolerance
        {
            samples.pop();
        }
        if samples.len() < 3 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut closed_points: Vec<_> = samples.iter().map(|sample| sample.point.clone()).collect();
        closed_points.push(samples[0].point.clone());
        let curve = CurveString2::reconstruct_from_polyline(&closed_points, options)?;
        Self::try_new_with_fill_rule(curve.into_segments(), fill_rule)
    }
}

#[derive(Clone, Debug)]
struct SamplePoint {
    point: Point2,
    x: f64,
    y: f64,
}

#[derive(Clone, Copy, Debug)]
struct Span {
    start: usize,
    end: usize,
    bulge: f64,
}

#[derive(Clone, Copy, Debug)]
struct Circle {
    cx: f64,
    cy: f64,
    radius: f64,
    sign: f64,
}

#[derive(Clone, Copy, Debug)]
struct ArcCandidate {
    end: usize,
    bulge: f64,
}

fn sample_open_points(
    points: &[Point2],
    options: PolylineReconstructionOptions,
) -> CurveResult<Vec<SamplePoint>> {
    let mut samples = Vec::with_capacity(points.len());
    for point in points {
        let sample = sample_point(point)?;
        if samples.last().is_some_and(|previous| {
            distance(previous, &sample) <= options.duplicate_point_tolerance
        }) {
            continue;
        }
        samples.push(sample);
    }
    Ok(samples)
}

fn point_from_finite_xy(point: [f64; 2]) -> CurveResult<Point2> {
    if !point[0].is_finite() || !point[1].is_finite() {
        return Err(CurveError::NonFiniteReconstructionPoint);
    }
    Ok(Point2::new(
        real_from_f64(point[0])?,
        real_from_f64(point[1])?,
    ))
}

fn sample_point(point: &Point2) -> CurveResult<SamplePoint> {
    let Some(x) = point.x().to_f64_lossy() else {
        return Err(CurveError::NonFiniteReconstructionPoint);
    };
    let Some(y) = point.y().to_f64_lossy() else {
        return Err(CurveError::NonFiniteReconstructionPoint);
    };
    if !x.is_finite() || !y.is_finite() {
        return Err(CurveError::NonFiniteReconstructionPoint);
    }
    Ok(SamplePoint {
        point: point.clone(),
        x,
        y,
    })
}

fn real_from_f64(value: f64) -> CurveResult<Real> {
    if !value.is_finite() {
        return Err(CurveError::NonFiniteReconstructionPoint);
    }
    Real::try_from(value).map_err(CurveError::from)
}

fn reconstruct_spans(
    samples: &[SamplePoint],
    options: PolylineReconstructionOptions,
) -> CurveResult<Vec<Span>> {
    let mut spans = Vec::new();
    let mut start = 0;

    while start + 1 < samples.len() {
        let line_end = line_run_end(samples, start, options);
        let arc = arc_run(samples, start, options);
        let span = if let Some(arc) = arc {
            if arc.end > line_end {
                Span {
                    start,
                    end: arc.end,
                    bulge: arc.bulge,
                }
            } else {
                Span {
                    start,
                    end: line_end,
                    bulge: 0.0,
                }
            }
        } else {
            Span {
                start,
                end: line_end,
                bulge: 0.0,
            }
        };

        if span.end <= start {
            return Err(CurveError::Topology(
                "polyline reconstruction made no forward progress".to_owned(),
            ));
        }
        start = span.end;
        spans.push(span);
    }

    Ok(spans)
}

fn line_run_end(
    samples: &[SamplePoint],
    start: usize,
    options: PolylineReconstructionOptions,
) -> usize {
    let mut end = start + 1;
    for candidate in (start + 2)..samples.len() {
        if line_span_ok(samples, start, candidate, options) {
            end = candidate;
        } else {
            break;
        }
    }
    end
}

fn line_span_ok(
    samples: &[SamplePoint],
    start: usize,
    end: usize,
    options: PolylineReconstructionOptions,
) -> bool {
    let scale = distance(&samples[start], &samples[end]);
    if scale <= options.duplicate_point_tolerance {
        return false;
    }
    let limit = options.distance_limit(scale);
    samples[(start + 1)..end]
        .iter()
        .all(|point| point_line_distance(point, &samples[start], &samples[end]) <= limit)
}

fn arc_run(
    samples: &[SamplePoint],
    start: usize,
    options: PolylineReconstructionOptions,
) -> Option<ArcCandidate> {
    if start + 2 >= samples.len() {
        return None;
    }

    let p0 = &samples[start];
    let p1 = &samples[start + 1];
    let p2 = &samples[start + 2];
    if point_line_distance(p1, p0, p2) <= options.distance_limit(distance(p0, p2)) {
        return None;
    }

    let circle = circumcircle(p0, p1, p2, options)?;
    let mut previous_sweep = directed_sweep(&circle, p0, p1)?;
    let mut final_sweep = directed_sweep(&circle, p0, p2)?;
    if previous_sweep <= options.collinear_tolerance
        || final_sweep <= previous_sweep + options.collinear_tolerance
        || final_sweep > PI + options.collinear_tolerance
    {
        return None;
    }

    let mut end = start + 2;
    for candidate in (start + 3)..samples.len() {
        let point = &samples[candidate];
        let radial_error = (distance_to_center(point, &circle) - circle.radius).abs();
        if radial_error > options.distance_limit(circle.radius) {
            break;
        }

        let Some(sweep) = directed_sweep(&circle, p0, point) else {
            break;
        };
        if sweep <= previous_sweep + options.collinear_tolerance {
            break;
        }
        if sweep > PI + options.collinear_tolerance {
            break;
        }

        // The local signed area is the finite-difference curvature witness.
        // Requiring a stable sign follows Menger's point-triple curvature idea:
        // Menger, "Untersuchungen über allgemeine Metrik" (Mathematische
        // Annalen 100, 75-163, 1928).
        let local_turn = signed_area2(&samples[candidate - 2], &samples[candidate - 1], point);
        if local_turn.signum() != circle.sign
            && local_turn.abs()
                > options.collinear_tolerance
                    * distance(&samples[candidate - 2], &samples[candidate - 1])
                    * distance(&samples[candidate - 1], point)
        {
            break;
        }

        end = candidate;
        previous_sweep = sweep;
        final_sweep = sweep;
    }

    if end - start + 1 < options.min_arc_points {
        return None;
    }

    let sweep = final_sweep.min(PI);
    let mut bulge = circle.sign * (sweep * 0.25).tan();
    if bulge.abs() > 1.0 && bulge.abs() <= 1.0 + options.collinear_tolerance {
        bulge = circle.sign;
    }
    if bulge.abs() > 1.0 {
        return None;
    }

    Some(ArcCandidate { end, bulge })
}

fn circumcircle(
    a: &SamplePoint,
    b: &SamplePoint,
    c: &SamplePoint,
    options: PolylineReconstructionOptions,
) -> Option<Circle> {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let acx = c.x - a.x;
    let acy = c.y - a.y;
    let cross = abx * acy - aby * acx;
    let scale = abx.hypot(aby) * acx.hypot(acy);
    if scale <= options.duplicate_point_tolerance
        || cross.abs() <= options.collinear_tolerance * scale
    {
        return None;
    }

    // This is the three-point circumcircle behind the finite curvature test.
    // Algebraic multi-point fits such as Kåsa's method are better for noisy
    // whole-run least squares, but this local formula keeps reconstruction
    // streaming and deterministic. See Kåsa, "A Circle Fitting Procedure and
    // Its Error Analysis" (IEEE Transactions on Instrumentation and
    // Measurement IM-25(1), 8-14, 1976).
    let ab2 = abx * abx + aby * aby;
    let ac2 = acx * acx + acy * acy;
    let denom = 2.0 * cross;
    let cx = a.x + (acy * ab2 - aby * ac2) / denom;
    let cy = a.y + (abx * ac2 - acx * ab2) / denom;
    let radius = (a.x - cx).hypot(a.y - cy);
    if !cx.is_finite() || !cy.is_finite() || !radius.is_finite() || radius <= 0.0 {
        return None;
    }

    Some(Circle {
        cx,
        cy,
        radius,
        sign: cross.signum(),
    })
}

fn directed_sweep(circle: &Circle, start: &SamplePoint, point: &SamplePoint) -> Option<f64> {
    let start_x = start.x - circle.cx;
    let start_y = start.y - circle.cy;
    let point_x = point.x - circle.cx;
    let point_y = point.y - circle.cy;
    let cross = start_x * point_y - start_y * point_x;
    let dot = start_x * point_x + start_y * point_y;
    let raw = cross.atan2(dot);
    if !raw.is_finite() {
        return None;
    }

    let sweep = if circle.sign >= 0.0 {
        if raw < 0.0 { raw + 2.0 * PI } else { raw }
    } else if raw > 0.0 {
        -raw + 2.0 * PI
    } else {
        -raw
    };
    if sweep.is_finite() { Some(sweep) } else { None }
}

fn point_line_distance(point: &SamplePoint, start: &SamplePoint, end: &SamplePoint) -> f64 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length = dx.hypot(dy);
    if length == 0.0 {
        return f64::INFINITY;
    }
    ((point.x - start.x) * dy - (point.y - start.y) * dx).abs() / length
}

fn distance(left: &SamplePoint, right: &SamplePoint) -> f64 {
    (left.x - right.x).hypot(left.y - right.y)
}

fn distance_to_center(point: &SamplePoint, circle: &Circle) -> f64 {
    (point.x - circle.cx).hypot(point.y - circle.cy)
}

fn signed_area2(a: &SamplePoint, b: &SamplePoint, c: &SamplePoint) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}
