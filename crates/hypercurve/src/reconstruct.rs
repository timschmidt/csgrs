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

use crate::{
    BulgeVertex2, Contour2, CurveError, CurveResult, CurveString2, FillRule, Point2,
    RetainedImportFormat2, RetainedImportRecord2, RetainedSourceTolerance2, RetainedTopologyStatus,
    Segment2, SegmentKind, SegmentKindCounts,
};

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

/// Result of importing a finite open line string.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteCurveStringImport2 {
    curve: CurveString2,
    record: RetainedImportRecord2,
}

/// Result of importing a finite closed line ring.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteContourImport2 {
    contour: Contour2,
    record: RetainedImportRecord2,
}

/// Source sample provenance for one reconstructed output segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PolylineReconstructionSegmentReport2 {
    output_segment_index: usize,
    output_segment_kind: SegmentKind,
    retained_start_sample_index: usize,
    retained_end_sample_index: usize,
    retained_sample_count: usize,
    output_start_point: Point2,
    output_end_point: Point2,
    status: RetainedTopologyStatus,
}

/// Report for finite polyline reconstruction into native line/arc topology.
#[derive(Clone, Debug, PartialEq)]
pub struct PolylineReconstructionReport2 {
    closed: bool,
    fill_rule: Option<FillRule>,
    input_point_count: usize,
    retained_sample_count: usize,
    discarded_duplicate_count: usize,
    output_segment_count: Option<usize>,
    output_segment_kind_counts: Option<SegmentKindCounts>,
    segment_reports: Vec<PolylineReconstructionSegmentReport2>,
    options: PolylineReconstructionOptions,
    lossy_boundary: bool,
    status: RetainedTopologyStatus,
}

/// Result of report-bearing open polyline reconstruction.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringPolylineReconstructionResult2 {
    curve_string: CurveString2,
    report: PolylineReconstructionReport2,
}

/// Result of report-bearing closed polyline reconstruction.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourPolylineReconstructionResult2 {
    contour: Contour2,
    report: PolylineReconstructionReport2,
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

impl FiniteCurveStringImport2 {
    /// Returns the imported native curve string.
    pub const fn curve_string(&self) -> &CurveString2 {
        &self.curve
    }

    /// Returns retained import evidence for the finite source.
    pub const fn report(&self) -> &RetainedImportRecord2 {
        &self.record
    }

    /// Returns retained import evidence for the finite source.
    pub const fn record(&self) -> &RetainedImportRecord2 {
        &self.record
    }

    /// Consumes the import result and returns the native curve string.
    pub fn into_curve_string(self) -> CurveString2 {
        self.curve
    }

    /// Consumes the import result and returns both native geometry and evidence.
    pub fn into_parts(self) -> (CurveString2, RetainedImportRecord2) {
        (self.curve, self.record)
    }
}

impl FiniteContourImport2 {
    /// Returns the imported native contour.
    pub const fn contour(&self) -> &Contour2 {
        &self.contour
    }

    /// Returns retained import evidence for the finite source.
    pub const fn report(&self) -> &RetainedImportRecord2 {
        &self.record
    }

    /// Returns retained import evidence for the finite source.
    pub const fn record(&self) -> &RetainedImportRecord2 {
        &self.record
    }

    /// Consumes the import result and returns the native contour.
    pub fn into_contour(self) -> Contour2 {
        self.contour
    }

    /// Consumes the import result and returns both native geometry and evidence.
    pub fn into_parts(self) -> (Contour2, RetainedImportRecord2) {
        (self.contour, self.record)
    }
}

impl PolylineReconstructionSegmentReport2 {
    /// Returns the reconstructed output segment index.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns the primitive family of the reconstructed output segment.
    pub const fn output_segment_kind(&self) -> SegmentKind {
        self.output_segment_kind
    }

    /// Returns the first retained source sample index consumed by this segment.
    pub const fn retained_start_sample_index(&self) -> usize {
        self.retained_start_sample_index
    }

    /// Returns the final retained source sample index consumed by this segment.
    pub const fn retained_end_sample_index(&self) -> usize {
        self.retained_end_sample_index
    }

    /// Returns retained source samples spanned by this output segment.
    pub const fn retained_sample_count(&self) -> usize {
        self.retained_sample_count
    }

    /// Returns the exact output segment start point.
    pub const fn output_start_point(&self) -> &Point2 {
        &self.output_start_point
    }

    /// Returns the exact output segment end point.
    pub const fn output_end_point(&self) -> &Point2 {
        &self.output_end_point
    }

    /// Returns retained reconstruction status for this output segment.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl PolylineReconstructionReport2 {
    /// Returns whether the source samples were interpreted as a closed ring.
    pub const fn closed(&self) -> bool {
        self.closed
    }

    /// Returns the fill rule for closed reconstruction results.
    pub const fn fill_rule(&self) -> Option<FillRule> {
        self.fill_rule
    }

    /// Returns the number of source sample points supplied.
    pub const fn input_point_count(&self) -> usize {
        self.input_point_count
    }

    /// Returns the number of samples retained after duplicate/closure filtering.
    pub const fn retained_sample_count(&self) -> usize {
        self.retained_sample_count
    }

    /// Returns finite duplicate samples consumed as reconstruction metadata.
    pub const fn discarded_duplicate_count(&self) -> usize {
        self.discarded_duplicate_count
    }

    /// Returns output segment count when reconstruction materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns primitive-family counts for materialized output segments.
    pub const fn output_segment_kind_counts(&self) -> Option<SegmentKindCounts> {
        self.output_segment_kind_counts
    }

    /// Returns per-output-segment source sample provenance.
    pub fn segment_reports(&self) -> &[PolylineReconstructionSegmentReport2] {
        &self.segment_reports
    }

    /// Returns the reconstruction options used for this finite source.
    pub const fn options(&self) -> PolylineReconstructionOptions {
        self.options
    }

    /// Returns true because reconstruction crosses a finite lossy boundary.
    pub const fn lossy_boundary(&self) -> bool {
        self.lossy_boundary
    }

    /// Returns retained topology status for the reconstructed carrier.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringPolylineReconstructionResult2 {
    /// Returns the reconstructed native curve string.
    pub const fn curve_string(&self) -> &CurveString2 {
        &self.curve_string
    }

    /// Consumes this result and returns the reconstructed curve string.
    pub fn into_curve_string(self) -> CurveString2 {
        self.curve_string
    }

    /// Returns retained reconstruction evidence.
    pub const fn report(&self) -> &PolylineReconstructionReport2 {
        &self.report
    }
}

impl ContourPolylineReconstructionResult2 {
    /// Returns the reconstructed native contour.
    pub const fn contour(&self) -> &Contour2 {
        &self.contour
    }

    /// Consumes this result and returns the reconstructed contour.
    pub fn into_contour(self) -> Contour2 {
        self.contour
    }

    /// Returns retained reconstruction evidence.
    pub const fn report(&self) -> &PolylineReconstructionReport2 {
        &self.report
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
        bulge_vertices_from_reconstruction_spans(&samples, &spans)
    }
}

impl CurveString2 {
    /// Constructs an open line-segment curve string from hyperreal points.
    ///
    /// This is the native counterpart to [`CurveString2::from_finite_line_string`].
    /// It keeps already-promoted coordinates in `Real` form and builds exact-aware
    /// line segments directly.
    pub fn from_real_line_string(points: &[[Real; 2]]) -> CurveResult<Self> {
        if points.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(points.len() - 1);
        for edge in points.windows(2) {
            let start = point_from_real_xy(&edge[0]);
            let end = point_from_real_xy(&edge[1]);
            let line = crate::LineSeg2::try_new(start, end)?;
            segments.push(crate::Segment2::Line(line));
        }
        Self::try_new(segments)
    }

    /// Constructs an open line-segment curve string from an iterator of
    /// hyperreal points.
    pub fn from_real_point_iter<I>(points: I) -> CurveResult<Self>
    where
        I: IntoIterator<Item = [Real; 2]>,
    {
        let points = points.into_iter().collect::<Vec<_>>();
        Self::from_real_line_string(&points)
    }

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

    /// Imports an open finite line string as native line geometry.
    ///
    /// Adjacent duplicate finite points are not represented as zero-length
    /// native segments. All non-duplicate points are promoted through [`Real`]
    /// before constructing exact-aware line segments.
    pub fn import_finite_line_string(points: &[[f64; 2]]) -> CurveResult<FiniteCurveStringImport2> {
        Self::import_finite_line_string_with_source(
            points,
            RetainedImportFormat2::FinitePolyline,
            0,
            None,
        )
    }

    /// Imports an open finite line string as native line geometry and returns
    /// retained source evidence.
    pub fn import_finite_line_string_with_report(
        points: &[[f64; 2]],
    ) -> CurveResult<FiniteCurveStringImport2> {
        Self::import_finite_line_string(points)
    }

    /// Imports an open finite line string with retained source metadata.
    ///
    /// This method is the adapter hook for STEP/DXF readers that already
    /// decoded a finite polyline or faceted curve preview. The source handle
    /// and tolerance are retained as lossy import evidence; the emitted line
    /// segments are still the only native geometry, and the record keeps
    /// [`RetainedTopologyStatus::ImportedLossy`](crate::RetainedTopologyStatus::ImportedLossy)
    /// visible to callers.
    pub fn import_finite_line_string_with_source(
        points: &[[f64; 2]],
        format: RetainedImportFormat2,
        source_index: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteCurveStringImport2> {
        Self::import_finite_line_string_with_source_version(
            points,
            format,
            source_index,
            0,
            source_tolerance,
        )
    }

    /// Imports an open finite line string with retained source metadata and a
    /// source version/revision.
    pub fn import_finite_line_string_with_source_version(
        points: &[[f64; 2]],
        format: RetainedImportFormat2,
        source_index: u64,
        source_version: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteCurveStringImport2> {
        if points.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(points.len() - 1);
        let mut discarded_duplicate_count = 0_usize;
        for edge in points.windows(2) {
            let start = point_from_finite_xy(edge[0])?;
            let end = point_from_finite_xy(edge[1])?;
            if let Ok(line) = crate::LineSeg2::try_new(start, end) {
                segments.push(crate::Segment2::Line(line));
            } else {
                discarded_duplicate_count += 1;
            }
        }
        let curve = Self::try_new(segments)?;
        let record = RetainedImportRecord2::try_new_open_line_string_with_source_version(
            format,
            source_index,
            source_version,
            source_tolerance,
            points.len(),
            curve.len(),
            discarded_duplicate_count,
        )?;
        Ok(FiniteCurveStringImport2 { curve, record })
    }

    /// Imports an open finite line string with retained source metadata and
    /// report-bearing naming.
    pub fn import_finite_line_string_with_source_report(
        points: &[[f64; 2]],
        format: RetainedImportFormat2,
        source_index: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteCurveStringImport2> {
        Self::import_finite_line_string_with_source(points, format, source_index, source_tolerance)
    }

    /// Imports an open finite line string with retained source metadata,
    /// version evidence, and report-bearing naming.
    pub fn import_finite_line_string_with_source_version_report(
        points: &[[f64; 2]],
        format: RetainedImportFormat2,
        source_index: u64,
        source_version: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteCurveStringImport2> {
        Self::import_finite_line_string_with_source_version(
            points,
            format,
            source_index,
            source_version,
            source_tolerance,
        )
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
        Self::reconstruct_from_polyline_with_report(points, options)
            .map(CurveStringPolylineReconstructionResult2::into_curve_string)
    }

    /// Reconstructs an open curve string from sampled polyline points and
    /// retains finite reconstruction evidence.
    pub fn reconstruct_from_polyline_with_report(
        points: &[Point2],
        options: PolylineReconstructionOptions,
    ) -> CurveResult<CurveStringPolylineReconstructionResult2> {
        let options = options.validate()?;
        let samples = sample_open_points(points, options)?;
        if samples.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let spans = reconstruct_spans(&samples, options)?;
        let vertices = bulge_vertices_from_reconstruction_spans(&samples, &spans)?;
        let curve_string = Self::from_bulge_vertices(&vertices)?;
        let report = polyline_reconstruction_report(
            false,
            None,
            points.len(),
            samples.len(),
            points.len().saturating_sub(samples.len()),
            &spans,
            curve_string.segments(),
            options,
        );
        Ok(CurveStringPolylineReconstructionResult2 {
            curve_string,
            report,
        })
    }
}

impl Contour2 {
    /// Constructs a closed straight-segment contour from hyperreal ring points.
    ///
    /// A repeated final point equal to the first point is accepted and removed
    /// before native contour construction. Unlike [`Contour2::from_finite_ring`],
    /// this constructor does not cross a primitive-float boundary.
    pub fn from_real_ring(points: &[[Real; 2]]) -> CurveResult<Self> {
        Self::from_real_ring_with_fill_rule(points, FillRule::NonZero)
    }

    /// Constructs a closed straight-segment contour from hyperreal ring points
    /// and an explicit fill rule.
    pub fn from_real_ring_with_fill_rule(
        points: &[[Real; 2]],
        fill_rule: FillRule,
    ) -> CurveResult<Self> {
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
            .map(|point| BulgeVertex2::new(point_from_real_xy(point), Real::zero()))
            .collect::<Vec<_>>();
        Self::from_bulge_vertices_with_fill_rule(&vertices, fill_rule)
    }

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

    /// Imports a closed finite line ring as native contour geometry.
    ///
    /// A repeated final point equal to the first point is accepted as finite
    /// file-format closure metadata and removed before native contour
    /// construction. The emitted native contour remains the exact topology
    /// carrier.
    pub fn import_finite_ring(points: &[[f64; 2]]) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring_with_source(
            points,
            FillRule::NonZero,
            RetainedImportFormat2::FinitePolyline,
            0,
            None,
        )
    }

    /// Imports a closed finite line ring as native contour geometry and
    /// returns retained source evidence.
    pub fn import_finite_ring_with_report(
        points: &[[f64; 2]],
    ) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring(points)
    }

    /// Imports a closed finite line ring with an explicit fill rule.
    pub fn import_finite_ring_with_fill_rule(
        points: &[[f64; 2]],
        fill_rule: FillRule,
    ) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring_with_source(
            points,
            fill_rule,
            RetainedImportFormat2::FinitePolyline,
            0,
            None,
        )
    }

    /// Imports a closed finite line ring with an explicit fill rule and
    /// report-bearing naming.
    pub fn import_finite_ring_with_fill_rule_report(
        points: &[[f64; 2]],
        fill_rule: FillRule,
    ) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring_with_fill_rule(points, fill_rule)
    }

    /// Imports a closed finite ring with retained source metadata.
    ///
    /// Repeated finite source points, including a repeated closing point, are
    /// counted as discarded source-edge metadata in the retained record. They
    /// are not allowed to become zero-length native edges, matching the
    /// exact-object boundary described by Yap.
    pub fn import_finite_ring_with_source(
        points: &[[f64; 2]],
        fill_rule: FillRule,
        format: RetainedImportFormat2,
        source_index: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring_with_source_version(
            points,
            fill_rule,
            format,
            source_index,
            0,
            source_tolerance,
        )
    }

    /// Imports a closed finite line ring with retained source metadata and
    /// report-bearing naming.
    pub fn import_finite_ring_with_source_report(
        points: &[[f64; 2]],
        fill_rule: FillRule,
        format: RetainedImportFormat2,
        source_index: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring_with_source(
            points,
            fill_rule,
            format,
            source_index,
            source_tolerance,
        )
    }

    /// Imports a closed finite ring with retained source metadata and a source
    /// version/revision.
    pub fn import_finite_ring_with_source_version(
        points: &[[f64; 2]],
        fill_rule: FillRule,
        format: RetainedImportFormat2,
        source_index: u64,
        source_version: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteContourImport2> {
        if points.len() < 3 {
            return Err(CurveError::InsufficientVertices);
        }

        let (vertices, discarded_duplicate_count) = finite_ring_vertices(points)?;
        let contour = Self::from_bulge_vertices_with_fill_rule(&vertices, fill_rule)?;
        let record = RetainedImportRecord2::try_new_closed_ring_with_source_version(
            format,
            source_index,
            source_version,
            source_tolerance,
            points.len(),
            contour.len(),
            discarded_duplicate_count,
        )?;
        Ok(FiniteContourImport2 { contour, record })
    }

    /// Imports a closed finite line ring with retained source metadata,
    /// version evidence, and report-bearing naming.
    pub fn import_finite_ring_with_source_version_report(
        points: &[[f64; 2]],
        fill_rule: FillRule,
        format: RetainedImportFormat2,
        source_index: u64,
        source_version: u64,
        source_tolerance: Option<RetainedSourceTolerance2>,
    ) -> CurveResult<FiniteContourImport2> {
        Self::import_finite_ring_with_source_version(
            points,
            fill_rule,
            format,
            source_index,
            source_version,
            source_tolerance,
        )
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
        Self::reconstruct_from_closed_polyline_with_fill_rule_and_report(points, options, fill_rule)
            .map(ContourPolylineReconstructionResult2::into_contour)
    }

    /// Reconstructs a closed contour from sampled polyline points with an
    /// explicit fill rule and retained finite reconstruction evidence.
    pub fn reconstruct_from_closed_polyline_with_fill_rule_and_report(
        points: &[Point2],
        options: PolylineReconstructionOptions,
        fill_rule: FillRule,
    ) -> CurveResult<ContourPolylineReconstructionResult2> {
        let options = options.validate()?;
        let mut samples = sample_open_points(points, options)?;
        let duplicate_count = points.len().saturating_sub(samples.len());
        let mut discarded_duplicate_count = duplicate_count;
        if samples.len() >= 2
            && distance(&samples[0], &samples[samples.len() - 1])
                <= options.duplicate_point_tolerance
        {
            samples.pop();
            discarded_duplicate_count += 1;
        }
        if samples.len() < 3 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut closed_samples = samples.clone();
        closed_samples.push(samples[0].clone());
        let spans = reconstruct_spans(&closed_samples, options)?;
        let vertices = bulge_vertices_from_reconstruction_spans(&closed_samples, &spans)?;
        let curve = CurveString2::from_bulge_vertices(&vertices)?;
        let contour = Self::try_new_with_fill_rule(curve.into_segments(), fill_rule)?;
        let report = polyline_reconstruction_report(
            true,
            Some(fill_rule),
            points.len(),
            samples.len(),
            discarded_duplicate_count,
            &spans,
            contour.segments(),
            options,
        );
        Ok(ContourPolylineReconstructionResult2 { contour, report })
    }

    /// Reconstructs a closed contour from sampled polyline points using the
    /// non-zero fill rule and retains finite reconstruction evidence.
    pub fn reconstruct_from_closed_polyline_with_report(
        points: &[Point2],
        options: PolylineReconstructionOptions,
    ) -> CurveResult<ContourPolylineReconstructionResult2> {
        Self::reconstruct_from_closed_polyline_with_fill_rule_and_report(
            points,
            options,
            FillRule::NonZero,
        )
    }
}

fn finite_ring_vertices(points: &[[f64; 2]]) -> CurveResult<(Vec<BulgeVertex2>, usize)> {
    let source_points = points
        .iter()
        .map(|point| point_from_finite_xy(*point))
        .collect::<CurveResult<Vec<_>>>()?;
    let discarded_duplicate_count = cyclic_duplicate_edge_count(&source_points);

    let mut unique_points: Vec<Point2> = Vec::with_capacity(source_points.len());
    for point in source_points {
        if unique_points
            .last()
            .is_some_and(|previous| previous == &point)
        {
            continue;
        }
        unique_points.push(point);
    }
    if unique_points.len() > 1 && unique_points.first() == unique_points.last() {
        unique_points.pop();
    }
    if unique_points.len() < 3 {
        return Err(CurveError::InsufficientVertices);
    }

    let vertices = unique_points
        .into_iter()
        .map(|point| BulgeVertex2::new(point, Real::zero()))
        .collect();
    Ok((vertices, discarded_duplicate_count))
}

fn bulge_vertices_from_reconstruction_spans(
    samples: &[SamplePoint],
    spans: &[Span],
) -> CurveResult<Vec<BulgeVertex2>> {
    let mut vertices = Vec::with_capacity(spans.len() + 1);
    for span in spans {
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

fn polyline_reconstruction_report(
    closed: bool,
    fill_rule: Option<FillRule>,
    input_point_count: usize,
    retained_sample_count: usize,
    discarded_duplicate_count: usize,
    spans: &[Span],
    output_segments: &[Segment2],
    options: PolylineReconstructionOptions,
) -> PolylineReconstructionReport2 {
    PolylineReconstructionReport2 {
        closed,
        fill_rule,
        input_point_count,
        retained_sample_count,
        discarded_duplicate_count,
        output_segment_count: Some(output_segments.len()),
        output_segment_kind_counts: Some(segment_kind_counts(output_segments)),
        segment_reports: polyline_reconstruction_segment_reports(
            closed,
            retained_sample_count,
            spans,
            output_segments,
        ),
        options,
        lossy_boundary: true,
        status: RetainedTopologyStatus::ImportedLossy,
    }
}

fn polyline_reconstruction_segment_reports(
    closed: bool,
    retained_sample_count: usize,
    spans: &[Span],
    output_segments: &[Segment2],
) -> Vec<PolylineReconstructionSegmentReport2> {
    spans
        .iter()
        .zip(output_segments.iter())
        .enumerate()
        .map(|(output_segment_index, (span, segment))| {
            let retained_start_sample_index = if closed && retained_sample_count > 0 {
                span.start % retained_sample_count
            } else {
                span.start
            };
            let retained_end_sample_index = if closed && retained_sample_count > 0 {
                span.end % retained_sample_count
            } else {
                span.end
            };
            PolylineReconstructionSegmentReport2 {
                output_segment_index,
                output_segment_kind: segment.structural_facts().kind,
                retained_start_sample_index,
                retained_end_sample_index,
                retained_sample_count: span.end - span.start + 1,
                output_start_point: segment.start().clone(),
                output_end_point: segment.end().clone(),
                status: RetainedTopologyStatus::ImportedLossy,
            }
        })
        .collect()
}

fn segment_kind_counts(segments: &[Segment2]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        match segment {
            Segment2::Line(_) => counts.lines += 1,
            Segment2::Arc(_) => counts.arcs += 1,
        }
    }
    counts
}

fn cyclic_duplicate_edge_count(points: &[Point2]) -> usize {
    if points.is_empty() {
        return 0;
    }

    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .filter(|(start, end)| crate::LineSeg2::try_new((*start).clone(), (*end).clone()).is_err())
        .count()
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

fn point_from_real_xy(point: &[Real; 2]) -> Point2 {
    Point2::new(point[0].clone(), point[1].clone())
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
