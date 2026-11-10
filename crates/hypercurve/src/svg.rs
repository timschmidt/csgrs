//! Strict SVG document and path-data import/export.
//!
//! SVG syntax is parsed at this feature-gated interchange boundary and
//! materialized directly into Hypercurve topology. Filled geometry remains a
//! [`CurveRegion2`], line/circular-arc strokes remain [`CurveString2`] values,
//! and higher-order strokes remain [`CurvePath2`] values. Unsupported paint or
//! viewport behavior is rejected explicitly instead of being approximated as
//! different topology.
//!
//! Exported stroke paths use standard SVG line, circular-arc, quadratic, and
//! cubic commands where those commands can represent the source family. Every
//! path also carries a bounded, versioned `data-hypercurve-path` extension so
//! rational quadratics, arbitrary rational Beziers, polynomial B-splines,
//! NURBS, periodicity, and non-decimal [`Real`] values round-trip exactly
//! through Hypercurve. Retained authored-source restrictions and procedural
//! quadratic, cubic and rational Bezier parallels also preserve selected
//! algebraic parameters, source charts and traversal. Parallel definitions retain
//! their shared sources and signed distances; replay rechecks each finite range's
//! regularity. Selected straight chords retain their support, coordinate chart
//! and directed endpoints. Those endpoints may be exact coordinates, rational
//! source images, rational expressions or native parallel evaluations at retained
//! Bezier parameters; shared curve-endpoint references preserve incidence.
//! Polynomial fibers retain the base parameter, equation and singleton selector.
//! Rational images retain original denominator guards; similarities and unique
//! supporting-line intersections retain their checked point dependencies.
//! Replay rechecks point definitions, support incidence and strict chart order.
//! Chord-normal selected circles retain their center, directed frame chord,
//! nonzero signed radius, selected angular contacts and finite oriented ranges.
//! Independent incident points re-prove circle incidence and the selected half.
//! Non-cardinal transported chord directions retain checked unit-vector witnesses;
//! replay proves unit length, parallelism and orientation before retaining them.
//! Other circle frames, general contact charts and auxiliary chord contacts
//! remain unsupported.
//! Chord displacements retain their selected source, origin, signed normal or
//! tangent displacement and translation, sharing one recipe for paired endpoints.
//! Point-definition nesting and extension bytes have separate configurable limits;
//! neither is a total arithmetic-work, memory or cancellation bound.
//! Filled regions retain their regularized exact boundaries in a version 5
//! compound graph, sharing definitions across all boundary paths. Import validates
//! each definition and closed path, then uses the existing region arrangement and
//! declared SVG fill rule. Other SVG consumers ignore this `data-*` attribute and
//! render the finite standard-SVG projection in `d`.

mod exact;
mod path_numbers;
pub use exact::NativeParallelFidelityShape2;
use path_numbers::PathNumbers;

use crate::{
    Aabb2, BezierSubcurve2, BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2,
    CurveContext, CurveGeometry2, CurveOperation2, CurveOutcome, CurvePath2, CurveRegion2,
    CurveString2, ExactCurveError, FillRule, FiniteProjectionOptions, LineSeg2, NurbsCurve2,
    Point2, PolynomialSplineCurve2, QuadraticBezier2, RationalQuadraticBezier2, Real, RealSign,
    Segment2, Similarity2,
};
use std::fmt::{self, Write};

const EXACT_PATH_ATTRIBUTE: &str = "data-hypercurve-path";

fn svg_real_sign(value: &Real) -> Option<RealSign> {
    crate::classify::real_sign(value, &CurveContext::STRICT)
}

/// Error produced by strict SVG geometry import or export.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SvgError {
    /// Syntactically malformed SVG input or invalid finite attributes.
    MalformedInput(String),
    /// Valid SVG behavior that cannot be represented by Hypercurve topology.
    Unsupported(String),
    /// Exact geometry construction, transformation, or Boolean failure.
    Geometry(String),
    /// Exact arrangement, geometry operation or native replay failed, retaining its cause.
    Exact(ExactCurveError),
    /// Polynomial topology reconstruction failed before publishing an exact path.
    PolynomialTopology(hypersolve::bivariate_topology::BivariateTopologyError),
    /// An exact coordinate has an undecided certificate or exhausted its budget.
    Scalar(hyperreal::Problem),
    /// A configured projection, extension-byte or point-definition-depth bound was exceeded.
    SizeOverflow {
        /// Name of the exceeded bound.
        limit: &'static str,
    },
}

impl fmt::Display for SvgError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MalformedInput(detail) => write!(formatter, "malformed SVG input: {detail}"),
            Self::Unsupported(detail) => write!(formatter, "unsupported SVG input: {detail}"),
            Self::Geometry(detail) => write!(formatter, "SVG geometry conversion failed: {detail}"),
            Self::Exact(error) => write!(formatter, "SVG exact geometry failed: {error}"),
            Self::Scalar(error) => write!(formatter, "SVG exact coordinate failed: {error}"),
            Self::PolynomialTopology(error) => {
                write!(formatter, "SVG polynomial topology failed: {error:?}")
            }
            Self::SizeOverflow { limit } => {
                write!(
                    formatter,
                    "SVG conversion exceeds the supported {limit} limit"
                )
            }
        }
    }
}

impl std::error::Error for SvgError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Scalar(error) => Some(error),
            Self::Exact(error) => Some(error),
            _ => None,
        }
    }
}

impl From<std::num::ParseFloatError> for SvgError {
    fn from(error: std::num::ParseFloatError) -> Self {
        Self::MalformedInput(error.to_string())
    }
}

impl From<svg::parser::Error> for SvgError {
    fn from(error: svg::parser::Error) -> Self {
        Self::MalformedInput(error.to_string())
    }
}

impl From<std::io::Error> for SvgError {
    fn from(error: std::io::Error) -> Self {
        Self::MalformedInput(error.to_string())
    }
}

/// Result type for SVG import and export.
pub type SvgResult<T> = Result<T, SvgError>;

/// Controls finite SVG shape sampling, export projection, and exact extensions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SvgOptions {
    /// Maximum allowed geometric error at finite SVG output boundaries.
    pub curve_tolerance: f64,
    /// Maximum points admitted for one sampled primitive or projected path.
    pub max_curve_segments: usize,
    /// Maximum decoded byte length of one exact Hypercurve path extension.
    pub max_extension_bytes: usize,
    /// Maximum nested rational-image, similarity, chord-displacement, supporting-line point
    /// or selected-circle frame constructions and parallel-fidelity dependencies
    /// in one exact path graph. Leaf definitions have depth zero. This is distinct
    /// from arithmetic work and memory limits.
    pub max_point_definition_depth: usize,
    /// Maximum bisection depth of each source-fidelity verification on import.
    /// Serialized construction budgets cannot increase this replay limit.
    /// This bounds neither scalar arithmetic, total nodes nor heap bytes.
    pub max_parallel_verification_depth: usize,
}

impl Default for SvgOptions {
    fn default() -> Self {
        Self {
            curve_tolerance: 0.1,
            max_curve_segments: 4096,
            max_extension_bytes: 16 * 1024 * 1024,
            max_point_definition_depth: 256,
            max_parallel_verification_depth: 64,
        }
    }
}

impl SvgOptions {
    fn validate(self) -> SvgResult<Self> {
        if !self.curve_tolerance.is_finite()
            || self.curve_tolerance <= 0.0
            || self.max_curve_segments < 8
            || self.max_extension_bytes < 64
            || self.max_parallel_verification_depth == 0
        {
            return Err(SvgError::MalformedInput(
                "SVG options require a positive finite tolerance, at least 8 segments, at least 64 extension bytes, and a positive parallel-verification depth".into(),
            ));
        }
        Ok(self)
    }

    fn segments_for_radius(self, radius: f64) -> SvgResult<usize> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(SvgError::MalformedInput(
                "radius must be finite and positive".into(),
            ));
        }
        let requested = (std::f64::consts::TAU * radius / self.curve_tolerance)
            .ceil()
            .max(16.0);
        if requested > self.max_curve_segments as f64 {
            return Err(SvgError::SizeOverflow {
                limit: "configured curve-segment count",
            });
        }
        Ok(requested as usize)
    }
}

/// Complete Hypercurve topology represented by one SVG document.
#[derive(Clone, Debug)]
pub struct SvgGeometry2 {
    region: CurveRegion2,
    wires: Vec<CurveString2>,
    paths: Vec<CurvePath2>,
}

impl SvgGeometry2 {
    /// Constructs SVG geometry from complete native Hypercurve topology.
    pub const fn new(
        region: CurveRegion2,
        wires: Vec<CurveString2>,
        paths: Vec<CurvePath2>,
    ) -> Self {
        Self {
            region,
            wires,
            paths,
        }
    }

    /// Constructs empty SVG geometry.
    pub fn empty() -> Self {
        Self::new(CurveRegion2::empty(), Vec::new(), Vec::new())
    }

    /// Returns true when the document contains no filled or stroked topology.
    pub fn is_empty(&self) -> bool {
        self.region.is_empty() && self.wires.is_empty() && self.paths.is_empty()
    }

    /// Borrows the unified filled region.
    pub const fn region(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Borrows native line/circular-arc strokes.
    pub fn wires(&self) -> &[CurveString2] {
        &self.wires
    }

    /// Borrows higher-order stroked paths.
    pub fn paths(&self) -> &[CurvePath2] {
        &self.paths
    }

    /// Consumes the geometry into its native topology components.
    pub fn into_parts(self) -> (CurveRegion2, Vec<CurveString2>, Vec<CurvePath2>) {
        (self.region, self.wires, self.paths)
    }

    /// Parses a complete SVG document with default options.
    pub fn from_svg(document: &str) -> SvgResult<Self> {
        import_svg_document(document)
    }

    /// Parses a complete SVG document with explicit options.
    pub fn from_svg_with_options(document: &str, options: SvgOptions) -> SvgResult<Self> {
        import_svg_document_with_options(document, options)
    }

    /// Serializes this topology as a complete SVG document with default options.
    pub fn to_svg(&self) -> SvgResult<String> {
        export_svg_document(self)
    }

    /// Serializes this topology as a complete SVG document with explicit options.
    pub fn to_svg_with_options(&self, options: SvgOptions) -> SvgResult<String> {
        export_svg_document_with_options(self, options)
    }

    fn append(&mut self, mut other: Self) -> SvgResult<()> {
        if self.region.is_empty() {
            self.region = other.region;
        } else if !other.region.is_empty() {
            self.region = self
                .region
                .boolean_region_raw(&other.region, BooleanOp::Union, &CurveContext::STRICT)
                .map_err(SvgError::Exact)?;
        }
        self.wires.append(&mut other.wires);
        self.paths.append(&mut other.paths);
        Ok(())
    }
}

/// One parsed SVG path-data subpath.
#[derive(Clone, Debug)]
pub struct SvgSubpath2 {
    path: CurvePath2,
    closed: bool,
}

impl SvgSubpath2 {
    /// Borrows the exact curve path.
    pub const fn path(&self) -> &CurvePath2 {
        &self.path
    }

    /// Returns whether the source subpath ended with a close-path command.
    pub const fn is_closed(&self) -> bool {
        self.closed
    }

    /// Consumes the record into its exact curve path.
    pub fn into_path(self) -> CurvePath2 {
        self.path
    }
}

fn svg_geometry_error(error: impl fmt::Display) -> SvgError {
    SvgError::Geometry(error.to_string())
}

fn svg_projection_error(error: crate::CurveError) -> SvgError {
    match error {
        crate::CurveError::UncertainFiniteCurveProjection { reason } => SvgError::Exact(
            ExactCurveError::blocked_unattributed(CurveOperation2::Evaluation, reason),
        ),
        other => svg_geometry_error(other),
    }
}

fn real(value: f64, name: &'static str) -> SvgResult<Real> {
    Real::try_from(value)
        .map_err(|error| SvgError::MalformedInput(format!("{name} is invalid: {error}")))
}

fn svg_path_segments(data: &str) -> impl Iterator<Item = SvgResult<svgtypes::PathSegment>> + '_ {
    svgtypes::PathParser::from(data).map(|segment| {
        segment.map_err(|error| SvgError::MalformedInput(format!("invalid path data: {error}")))
    })
}

/// Parses all SVG path commands into exact Hypercurve subpaths.
///
/// Accepted decimal and scientific number tokens retain their exact rational
/// values, including values that binary64 would round or underflow. The
/// underlying SVG grammar validator still rejects tokens outside its finite
/// numeric range. Numeric attributes and transform strings use separate ingress.
///
/// Lines, circular arcs, quadratic Beziers, and cubic Beziers remain native
/// curves. SVG elliptical arcs are accepted only when `rx == ry`; a genuine
/// ellipse has no single native circular-arc representation and is rejected.
pub fn parse_svg_path_data(data: &str) -> SvgResult<Vec<SvgSubpath2>> {
    use svgtypes::PathSegment;

    let mut output = Vec::new();
    let mut curves = Vec::new();
    let mut current = None::<Point2>;
    let mut start = None::<Point2>;
    let mut previous_cubic_control = None::<Point2>;
    let mut previous_quadratic_control = None::<Point2>;
    let mut numbers = PathNumbers::new(data);

    for segment in svg_path_segments(data) {
        let segment = segment?;
        numbers.begin_segment();
        match segment {
            PathSegment::MoveTo { abs, .. } => {
                finish_svg_subpath(&mut output, &mut curves, false)?;
                let point = numbers.point(abs, current.as_ref())?;
                current = Some(point.clone());
                start = Some(point);
                previous_cubic_control = None;
                previous_quadratic_control = None;
            }
            PathSegment::LineTo { abs, .. } => {
                let end = numbers.point(abs, current.as_ref())?;
                push_svg_line(&mut curves, &mut current, end)?;
                previous_cubic_control = None;
                previous_quadratic_control = None;
            }
            PathSegment::HorizontalLineTo { abs, .. } => {
                let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
                let x = numbers.number()?;
                let end = Point2::new(if abs { x } else { from.x() + x }, from.y().clone());
                push_svg_line(&mut curves, &mut current, end)?;
                previous_cubic_control = None;
                previous_quadratic_control = None;
            }
            PathSegment::VerticalLineTo { abs, .. } => {
                let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
                let y = numbers.number()?;
                let end = Point2::new(from.x().clone(), if abs { y } else { from.y() + y });
                push_svg_line(&mut curves, &mut current, end)?;
                previous_cubic_control = None;
                previous_quadratic_control = None;
            }
            PathSegment::CurveTo { abs, .. } => {
                let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
                let control1 = numbers.point(abs, Some(from))?;
                let control2 = numbers.point(abs, Some(from))?;
                let end = numbers.point(abs, Some(from))?;
                curves.push(Curve2::from(CubicBezier2::new(
                    from.clone(),
                    control1,
                    control2.clone(),
                    end.clone(),
                )));
                current = Some(end);
                previous_cubic_control = Some(control2);
                previous_quadratic_control = None;
            }
            PathSegment::SmoothCurveTo { abs, .. } => {
                let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
                let control1 = previous_cubic_control
                    .as_ref()
                    .map(|control| {
                        Point2::new(
                            Real::from(2) * from.x() - control.x(),
                            Real::from(2) * from.y() - control.y(),
                        )
                    })
                    .unwrap_or_else(|| from.clone());
                let control2 = numbers.point(abs, Some(from))?;
                let end = numbers.point(abs, Some(from))?;
                curves.push(Curve2::from(CubicBezier2::new(
                    from.clone(),
                    control1,
                    control2.clone(),
                    end.clone(),
                )));
                current = Some(end);
                previous_cubic_control = Some(control2);
                previous_quadratic_control = None;
            }
            PathSegment::Quadratic { abs, .. } => {
                let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
                let control = numbers.point(abs, Some(from))?;
                let end = numbers.point(abs, Some(from))?;
                curves.push(Curve2::from(QuadraticBezier2::new(
                    from.clone(),
                    control.clone(),
                    end.clone(),
                )));
                current = Some(end);
                previous_cubic_control = None;
                previous_quadratic_control = Some(control);
            }
            PathSegment::SmoothQuadratic { abs, .. } => {
                let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
                let control = previous_quadratic_control
                    .as_ref()
                    .map(|control| {
                        Point2::new(
                            Real::from(2) * from.x() - control.x(),
                            Real::from(2) * from.y() - control.y(),
                        )
                    })
                    .unwrap_or_else(|| from.clone());
                let end = numbers.point(abs, Some(from))?;
                curves.push(Curve2::from(QuadraticBezier2::new(
                    from.clone(),
                    control.clone(),
                    end.clone(),
                )));
                current = Some(end);
                previous_cubic_control = None;
                previous_quadratic_control = Some(control);
            }
            PathSegment::EllipticalArc { abs, .. } => {
                let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
                let rx = numbers.magnitude()?;
                let ry = numbers.magnitude()?;
                let rotation = numbers.number()?;
                let large_arc = numbers.flag()?;
                let sweep = numbers.flag()?;
                let end = numbers.point(abs, Some(from))?;
                if svg_real_sign(&rx) == Some(RealSign::Zero)
                    || svg_real_sign(&ry) == Some(RealSign::Zero)
                {
                    push_svg_line(&mut curves, &mut current, end)?;
                    previous_cubic_control = None;
                    previous_quadratic_control = None;
                    continue;
                }
                if let Some(arc) = svg_circular_arc(
                    from.clone(),
                    end.clone(),
                    rx,
                    ry,
                    rotation,
                    large_arc,
                    sweep,
                )? {
                    curves.push(Curve2::from(arc));
                }
                current = Some(end);
                previous_cubic_control = None;
                previous_quadratic_control = None;
            }
            PathSegment::ClosePath { .. } => {
                let first = start.as_ref().ok_or_else(svg_path_requires_move)?.clone();
                push_svg_line(&mut curves, &mut current, first)?;
                finish_svg_subpath(&mut output, &mut curves, true)?;
                start = None;
                previous_cubic_control = None;
                previous_quadratic_control = None;
            }
        }
    }
    finish_svg_subpath(&mut output, &mut curves, false)?;
    if output.is_empty() {
        return Err(SvgError::MalformedInput(
            "path contains no drawable subpath".into(),
        ));
    }
    Ok(output)
}

fn push_svg_line(
    curves: &mut Vec<Curve2>,
    current: &mut Option<Point2>,
    end: Point2,
) -> SvgResult<()> {
    let from = current.as_ref().ok_or_else(svg_path_requires_move)?;
    if from != &end {
        curves.push(Curve2::from(
            LineSeg2::try_new(from.clone(), end.clone()).map_err(svg_geometry_error)?,
        ));
    }
    *current = Some(end);
    Ok(())
}

fn finish_svg_subpath(
    output: &mut Vec<SvgSubpath2>,
    curves: &mut Vec<Curve2>,
    closed: bool,
) -> SvgResult<()> {
    if curves.is_empty() {
        return Ok(());
    }
    let path = CurvePath2::try_new(std::mem::take(curves)).map_err(svg_geometry_error)?;
    output.push(SvgSubpath2 { path, closed });
    Ok(())
}

fn svg_path_requires_move() -> SvgError {
    SvgError::MalformedInput("path drawing command precedes move-to".into())
}

fn svg_circular_arc(
    start: Point2,
    end: Point2,
    rx: Real,
    ry: Real,
    _rotation: Real,
    large_arc: bool,
    sweep: bool,
) -> SvgResult<Option<CircularArc2>> {
    if rx != ry {
        return Err(SvgError::Unsupported(
            "non-circular elliptical path arc".into(),
        ));
    }
    if svg_real_sign(&rx) != Some(RealSign::Positive) {
        return Err(SvgError::MalformedInput(
            "path arc radius must be positive".into(),
        ));
    }
    if start == end {
        return Ok(None);
    }

    let chord_squared = start.distance_squared(&end);
    let radius_squared = &rx * &rx;
    let quarter_chord_squared = (&chord_squared / &Real::from(4)).map_err(svg_geometry_error)?;
    let center_offset_squared = &radius_squared - quarter_chord_squared;
    let center_offset_scale = match svg_real_sign(&center_offset_squared) {
        // SVG scales an insufficient radius to the unique half-chord
        // semicircle. The corresponding center offset is exactly zero.
        Some(RealSign::Negative | RealSign::Zero) => Real::zero(),
        Some(RealSign::Positive) => {
            let ratio = (center_offset_squared / &chord_squared).map_err(svg_geometry_error)?;
            ratio.sqrt().map_err(svg_geometry_error)?
        }
        None => {
            return Err(SvgError::Geometry(
                "circular path arc center sign is undecidable".into(),
            ));
        }
    };
    let midpoint = Point2::new(
        ((start.x() + end.x()) / &Real::from(2)).map_err(svg_geometry_error)?,
        ((start.y() + end.y()) / &Real::from(2)).map_err(svg_geometry_error)?,
    );
    let dx = end.x() - start.x();
    let dy = end.y() - start.y();
    let offset_x = -(&dy * &center_offset_scale);
    let offset_y = &dx * &center_offset_scale;
    let centers = [
        Point2::new(midpoint.x() + &offset_x, midpoint.y() + &offset_y),
        Point2::new(midpoint.x() - &offset_x, midpoint.y() - &offset_y),
    ];
    for center in centers {
        let start_x = start.x() - center.x();
        let start_y = start.y() - center.y();
        let end_x = end.x() - center.x();
        let end_y = end.y() - center.y();
        let cross = start_x * end_y - start_y * end_x;
        let cross_sign = svg_real_sign(&cross);
        let is_major = match cross_sign {
            Some(RealSign::Zero) => false,
            Some(RealSign::Positive) => sweep,
            Some(RealSign::Negative) => !sweep,
            None => continue,
        };
        if is_major == large_arc || cross_sign == Some(RealSign::Zero) {
            return CircularArc2::try_from_center(start, end, center, sweep)
                .map(Some)
                .map_err(svg_geometry_error);
        }
    }
    Err(SvgError::Geometry(
        "circular path arc flags do not select a certified center".into(),
    ))
}

#[derive(Clone, Debug)]
struct ExactAffine2 {
    m00: Real,
    m01: Real,
    m10: Real,
    m11: Real,
    tx: Real,
    ty: Real,
}

impl ExactAffine2 {
    fn identity() -> Self {
        Self::from_coefficients([
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
        ])
    }

    fn from_coefficients([a, b, c, d, e, f]: [Real; 6]) -> Self {
        Self {
            m00: a,
            m01: c,
            m10: b,
            m11: d,
            tx: e,
            ty: f,
        }
    }

    fn translation(tx: Real, ty: Real) -> Self {
        Self {
            tx,
            ty,
            ..Self::identity()
        }
    }

    fn then(self, rhs: Self) -> Self {
        Self {
            m00: &self.m00 * &rhs.m00 + &self.m01 * &rhs.m10,
            m01: &self.m00 * &rhs.m01 + &self.m01 * &rhs.m11,
            m10: &self.m10 * &rhs.m00 + &self.m11 * &rhs.m10,
            m11: &self.m10 * &rhs.m01 + &self.m11 * &rhs.m11,
            tx: &self.m00 * &rhs.tx + &self.m01 * &rhs.ty + self.tx,
            ty: &self.m10 * &rhs.tx + &self.m11 * &rhs.ty + self.ty,
        }
    }

    fn is_identity(&self) -> bool {
        self.m00.definitely_one()
            && self.m10.definitely_zero()
            && self.m01.definitely_zero()
            && self.m11.definitely_one()
            && self.tx.definitely_zero()
            && self.ty.definitely_zero()
    }

    fn transform_point(&self, point: &Point2) -> Point2 {
        Point2::new(
            &self.m00 * point.x() + &self.m01 * point.y() + self.tx.clone(),
            &self.m10 * point.x() + &self.m11 * point.y() + self.ty.clone(),
        )
    }

    fn similarity(&self) -> Option<Similarity2> {
        Similarity2::try_from_real_affine(
            self.m00.clone(),
            self.m01.clone(),
            self.m10.clone(),
            self.m11.clone(),
            self.tx.clone(),
            self.ty.clone(),
        )
        .ok()
    }
}

#[derive(Clone, Debug)]
struct StyleContext {
    transform: Option<ExactAffine2>,
    fill: bool,
    stroke: bool,
    displayed: bool,
    visibility: bool,
    // Paint values select geometry; this importer does not retain appearance
    // or construct stroke thickness. After exact nonnegative range validation,
    // positivity is the sufficient retained fact, including for tiny values.
    opacity_positive: bool,
    fill_opacity_positive: bool,
    stroke_opacity_positive: bool,
    stroke_width_positive: bool,
    fill_rule: FillRule,
}

impl Default for StyleContext {
    fn default() -> Self {
        Self {
            transform: None,
            fill: true,
            stroke: false,
            displayed: true,
            visibility: true,
            opacity_positive: true,
            fill_opacity_positive: true,
            stroke_opacity_positive: true,
            stroke_width_positive: true,
            fill_rule: FillRule::NonZero,
        }
    }
}

impl StyleContext {
    fn visible(&self) -> bool {
        self.displayed && self.visibility && self.opacity_positive
    }

    fn fills(&self) -> bool {
        self.fill && self.fill_opacity_positive
    }

    fn strokes(&self) -> bool {
        self.stroke && self.stroke_opacity_positive && self.stroke_width_positive
    }
}

fn parse_numbers(value: &str) -> SvgResult<Vec<f64>> {
    svgtypes::NumberListParser::from(value)
        .map(|number| {
            let number = number.map_err(|error| {
                SvgError::MalformedInput(format!("invalid number list: {error}"))
            })?;
            if number.is_finite() {
                Ok(number)
            } else {
                Err(SvgError::MalformedInput(
                    "number list contains a non-finite value".into(),
                ))
            }
        })
        .collect()
}

fn parse_transform(value: &str) -> SvgResult<ExactAffine2> {
    // Keep svgtypes as the grammar authority, including optional arguments,
    // compact signed numbers and the existing finite-token input envelope.
    for token in svgtypes::TransformListParser::from(value) {
        token.map_err(|error| {
            SvgError::MalformedInput(format!("invalid transform {value:?}: {error}"))
        })?;
    }
    let mut transform = ExactAffine2::identity();
    // Transform arguments cannot contain nested parentheses. Grammar validation
    // above precedes extraction; only the original decimal spans supply values.
    for definition in value.split(')') {
        let definition = definition.trim_matches(|c: char| c.is_ascii_whitespace() || c == ',');
        if definition.is_empty() {
            continue;
        }
        let (name, arguments) = definition
            .split_once('(')
            .ok_or_else(|| SvgError::MalformedInput(format!("invalid transform {value:?}")))?;
        let mut numbers = PathNumbers::new(arguments);
        let arguments = svgtypes::NumberListParser::from(arguments)
            .map(|number| {
                number.map_err(|error| SvgError::MalformedInput(error.to_string()))?;
                numbers.number()
            })
            .collect::<SvgResult<Vec<_>>>()?;
        let next = match (name.trim(), arguments.as_slice()) {
            ("matrix", [a, b, c, d, e, f]) => ExactAffine2::from_coefficients([
                a.clone(),
                b.clone(),
                c.clone(),
                d.clone(),
                e.clone(),
                f.clone(),
            ]),
            ("translate", [x]) => ExactAffine2::translation(x.clone(), Real::zero()),
            ("translate", [x, y]) => ExactAffine2::translation(x.clone(), y.clone()),
            ("scale", [x]) => ExactAffine2 {
                m00: x.clone(),
                m11: x.clone(),
                ..ExactAffine2::identity()
            },
            ("scale", [x, y]) => ExactAffine2 {
                m00: x.clone(),
                m11: y.clone(),
                ..ExactAffine2::identity()
            },
            ("rotate", [angle]) => rotation(angle.clone())?,
            ("rotate", [angle, x, y]) => {
                // Preserve SVG's translate/rotate/translate sequence, retaining
                // the authored center rather than svgtypes' rounded token view.
                transform = transform.then(ExactAffine2::translation(x.clone(), y.clone()));
                transform = transform.then(rotation(angle.clone())?);
                transform = transform.then(ExactAffine2::translation(-x, -y));
                continue;
            }
            ("skewX", [angle]) => ExactAffine2 {
                m01: radians(angle.clone())?.tan().map_err(SvgError::Scalar)?,
                ..ExactAffine2::identity()
            },
            ("skewY", [angle]) => ExactAffine2 {
                m10: radians(angle.clone())?.tan().map_err(SvgError::Scalar)?,
                ..ExactAffine2::identity()
            },
            _ => {
                return Err(SvgError::MalformedInput(format!(
                    "invalid transform {value:?}"
                )));
            }
        };
        transform = transform.then(next);
    }
    Ok(transform)
}

fn radians(degrees: Real) -> SvgResult<Real> {
    (degrees * Real::pi() / Real::from(180)).map_err(SvgError::Scalar)
}

fn rotation(degrees: Real) -> SvgResult<ExactAffine2> {
    let angle = radians(degrees)?;
    let sin = angle.clone().sin();
    let cos = angle.cos();
    Ok(ExactAffine2::from_coefficients([
        cos.clone(),
        sin.clone(),
        -sin,
        cos,
        Real::zero(),
        Real::zero(),
    ]))
}

fn apply_style(
    mut context: StyleContext,
    attrs: &svg::node::Attributes,
) -> SvgResult<StyleContext> {
    let mut properties = Vec::<(&str, &str)>::new();
    for name in [
        "fill",
        "stroke",
        "fill-rule",
        "display",
        "visibility",
        "opacity",
        "fill-opacity",
        "stroke-opacity",
        "stroke-width",
        "clip-path",
        "mask",
        "stroke-dasharray",
        "stroke-dashoffset",
        "stroke-linecap",
        "stroke-linejoin",
        "stroke-miterlimit",
        "vector-effect",
        "marker-start",
        "marker-mid",
        "marker-end",
    ] {
        if let Some(value) = attrs.get(name) {
            properties.push((name, value));
        }
    }
    if let Some(style) = attrs.get("style") {
        for declaration in style.split(';').filter(|part| !part.trim().is_empty()) {
            let (name, value) = declaration.split_once(':').ok_or_else(|| {
                SvgError::MalformedInput(format!("invalid style declaration {declaration:?}"))
            })?;
            properties.push((name.trim(), value.trim()));
        }
    }
    let mut local_opacity_positive = true;
    let mut local_display = true;
    for (name, value) in properties {
        match name {
            "fill" => context.fill = value != "none",
            "stroke" => context.stroke = value != "none",
            "fill-rule" => {
                context.fill_rule = match value {
                    "nonzero" => FillRule::NonZero,
                    "evenodd" => FillRule::EvenOdd,
                    _ => {
                        return Err(SvgError::MalformedInput(format!(
                            "invalid fill-rule {value:?}"
                        )));
                    }
                }
            }
            "display" => local_display = value != "none",
            "visibility" => context.visibility = !matches!(value, "hidden" | "collapse"),
            "opacity" => local_opacity_positive = unit_interval_positive(value, "opacity")?,
            "fill-opacity" => {
                context.fill_opacity_positive = unit_interval_positive(value, "fill-opacity")?;
            }
            "stroke-opacity" => {
                context.stroke_opacity_positive = unit_interval_positive(value, "stroke-opacity")?;
            }
            "stroke-width" => {
                context.stroke_width_positive = style_number_positive(
                    value,
                    None,
                    "stroke-width must be finite and non-negative",
                )?;
            }
            "clip-path" | "mask" | "stroke-dasharray" | "stroke-dashoffset" | "stroke-linecap"
            | "stroke-linejoin" | "stroke-miterlimit" | "vector-effect" | "marker-start"
            | "marker-mid" | "marker-end" => {
                return Err(SvgError::Unsupported(format!("style property {name}")));
            }
            _ => {}
        }
    }
    // Every factor is nonnegative: a finite product is positive exactly when
    // every factor is positive. Retain that proof without product underflow.
    context.opacity_positive &= local_opacity_positive;
    context.displayed &= local_display;
    if let Some(value) = attrs.get("transform") {
        let local = parse_transform(value)?;
        context.transform = Some(match context.transform {
            Some(parent) => parent.then(local),
            None => local,
        });
    }
    Ok(context)
}

fn unit_interval_positive(value: &str, name: &str) -> SvgResult<bool> {
    let (value, upper) = if let Some(percent) = value.strip_suffix('%') {
        (percent, 100)
    } else {
        (value, 1)
    };
    // The sign and interval are equivalent before dividing a percentage by
    // 100. Avoid materializing a normalized value that selection never uses.
    style_number_positive(
        value,
        Some(upper),
        &format!("{name} must be between zero and one"),
    )
}

fn style_number_positive(value: &str, upper: Option<u8>, invalid: &str) -> SvgResult<bool> {
    // Preserve the previous finite numeric syntax envelope. The validator's
    // rounded value never supplies a sign, range decision or retained scalar.
    if !value.parse::<f64>()?.is_finite() {
        return Err(SvgError::MalformedInput(invalid.into()));
    }
    let value: Real = value.parse().map_err(SvgError::Scalar)?;
    let sign = svg_real_sign(&value).ok_or_else(|| {
        SvgError::Exact(ExactCurveError::blocked_unattributed(
            CurveOperation2::Classification,
            crate::UncertaintyReason::RealSign,
        ))
    })?;
    if sign == RealSign::Negative {
        return Err(SvgError::MalformedInput(invalid.into()));
    }
    if let Some(upper) = upper {
        let order =
            crate::classify::compare_reals(&value, &Real::from(upper), &CurveContext::STRICT)
                .ok_or_else(|| {
                    SvgError::Exact(ExactCurveError::blocked_unattributed(
                        CurveOperation2::Classification,
                        crate::UncertaintyReason::Ordering,
                    ))
                })?;
        if order == std::cmp::Ordering::Greater {
            return Err(SvgError::MalformedInput(invalid.into()));
        }
    }
    Ok(sign == RealSign::Positive)
}

fn number(attrs: &svg::node::Attributes, name: &str, default: Option<f64>) -> SvgResult<f64> {
    let value = match attrs.get(name) {
        Some(value) => value.parse::<f64>()?,
        None => {
            default.ok_or_else(|| SvgError::MalformedInput(format!("missing attribute {name}")))?
        }
    };
    if !value.is_finite() {
        return Err(SvgError::MalformedInput(format!(
            "attribute {name} must be finite"
        )));
    }
    Ok(value)
}

fn exact_number(
    attrs: &svg::node::Attributes,
    name: &str,
    default: Option<f64>,
) -> SvgResult<Real> {
    // Reuse the existing syntax, finite-range and missing-attribute checks.
    // Only an internal default may supply a binary value; authored attributes
    // retain the original decimal/scientific token through the shared cursor.
    let validated = number(attrs, name, default)?;
    match attrs.get(name) {
        Some(value) => PathNumbers::new(value).number(),
        None => real(validated, "default SVG attribute"),
    }
}

fn points(value: &str) -> SvgResult<Vec<Point2>> {
    let numbers = parse_numbers(value)?;
    if numbers.len() < 4 || numbers.len() % 2 != 0 {
        return Err(SvgError::MalformedInput(
            "points require coordinate pairs".into(),
        ));
    }
    // Complete grammar/finite and arity validation still precedes exact scalar
    // conversion. The validator's rounded values supply only the token count.
    let mut exact = PathNumbers::new(value);
    numbers
        .as_chunks::<2>()
        .0
        .iter()
        .map(|_| Ok(Point2::new(exact.number()?, exact.number()?)))
        .collect()
}

fn transform_path(path: &CurvePath2, transform: &ExactAffine2) -> SvgResult<CurvePath2> {
    let mut curves = Vec::new();
    for curve in path.curves() {
        curves.extend(transform_curve(curve, transform)?);
    }
    CurvePath2::try_new(curves).map_err(svg_geometry_error)
}

fn transform_curve(curve: &Curve2, transform: &ExactAffine2) -> SvgResult<Vec<Curve2>> {
    if curve.parallel_fidelity().is_some() {
        if let Some(similarity) = transform.similarity() {
            return curve
                .transform_similarity_with_policy(&similarity, &CurveContext::STRICT)
                .map(|curve| vec![curve.into_value()])
                .map_err(svg_geometry_error);
        }
        if let Some((source, range, reversed)) = curve.source_restriction() {
            let mut sources = transform_curve(source, transform)?;
            if sources.len() != 1 {
                return Err(SvgError::Unsupported(
                    "affine transport of a selected multichart fidelity source".into(),
                ));
            }
            let curve = sources
                .pop()
                .expect("one source")
                .restrict_source(range.clone())
                .map_err(SvgError::Exact)?;
            return Ok(vec![if reversed {
                curve.reversed().map_err(SvgError::Exact)?
            } else {
                curve
            }]);
        }
        if matches!(curve.geometry(), Some(CurveGeometry2::CircularArc(_))) {
            let mut curves = Vec::new();
            for span in curve
                .native_bezier_fragments_with_policy(&CurveContext::STRICT)
                .map_err(SvgError::Exact)?
                .into_value()
            {
                curves.extend(transform_curve(&span.clone().into_curve(), transform)?);
            }
            return Ok(curves);
        }
        let bare = Curve2::new(curve.geometry().expect("native fidelity source").clone());
        return transform_curve(&bare, transform).map(|curves| {
            curves
                .into_iter()
                .map(|result| {
                    curve.retain_parallel_affine_image(
                        result,
                        [
                            transform.m00.clone(),
                            transform.m01.clone(),
                            transform.m10.clone(),
                            transform.m11.clone(),
                            transform.tx.clone(),
                            transform.ty.clone(),
                        ],
                    )
                })
                .collect()
        });
    }
    let invalid = |cause| ExactCurveError::Invalid {
        operation: CurveOperation2::Transformation,
        family: Some(curve.family()),
        cause,
    };
    let transformed = match curve.geometry() {
        None => {
            let Some(similarity) = transform.similarity() else {
                return Err(SvgError::Geometry(
                    "this retained curve requires a similarity transform".into(),
                ));
            };
            return curve
                .transform_similarity_with_policy(&similarity, &CurveContext::STRICT)
                .map(|curve| vec![curve.into_value()])
                .map_err(svg_geometry_error);
        }
        Some(CurveGeometry2::Line(line)) => vec![Curve2::from(
            LineSeg2::try_new(
                transform.transform_point(line.start()),
                transform.transform_point(line.end()),
            )
            .map_err(invalid)
            .map_err(svg_geometry_error)?,
        )],
        Some(CurveGeometry2::CircularArc(_)) => {
            if let Some(similarity) = transform.similarity() {
                return curve
                    .transform_similarity_with_policy(&similarity, &CurveContext::STRICT)
                    .map(|curve| vec![curve.into_value()])
                    .map_err(svg_geometry_error);
            }
            curve
                .native_bezier_fragments_with_policy(&CurveContext::STRICT)
                .map_err(svg_geometry_error)?
                .into_value()
                .iter()
                .map(|fragment| {
                    transform_bezier_subcurve(fragment.native_curve(), transform).map(Curve2::from)
                })
                .collect::<SvgResult<Vec<_>>>()?
        }
        Some(CurveGeometry2::QuadraticBezier(curve)) => vec![Curve2::from(QuadraticBezier2::new(
            transform.transform_point(curve.start()),
            transform.transform_point(curve.control()),
            transform.transform_point(curve.end()),
        ))],
        Some(CurveGeometry2::CubicBezier(curve)) => vec![Curve2::from(CubicBezier2::new(
            transform.transform_point(curve.start()),
            transform.transform_point(curve.control1()),
            transform.transform_point(curve.control2()),
            transform.transform_point(curve.end()),
        ))],
        Some(CurveGeometry2::RationalQuadraticBezier(curve)) => vec![Curve2::from(
            RationalQuadraticBezier2::try_new(
                transform.transform_point(curve.start()),
                transform.transform_point(curve.control()),
                transform.transform_point(curve.end()),
                curve.start_weight().clone(),
                curve.control_weight().clone(),
                curve.end_weight().clone(),
            )
            .map_err(invalid)
            .map_err(svg_geometry_error)?,
        )],
        Some(CurveGeometry2::RationalBezier(curve)) => {
            vec![Curve2::from(curve.transformed_affine([
                &transform.m00,
                &transform.m01,
                &transform.m10,
                &transform.m11,
                &transform.tx,
                &transform.ty,
            ]))]
        }
        Some(CurveGeometry2::PolynomialBSpline(curve)) => {
            vec![Curve2::from(
                PolynomialSplineCurve2::try_new_raw(
                    curve.degree(),
                    curve
                        .control_points()
                        .iter()
                        .map(|point| transform.transform_point(point))
                        .collect(),
                    curve.knots().to_vec(),
                    &CurveContext::STRICT,
                )
                .map_err(svg_geometry_error)?,
            )]
        }
        Some(CurveGeometry2::Nurbs(curve)) => vec![Curve2::from(
            NurbsCurve2::from_homogeneous_raw(
                curve.degree(),
                curve
                    .homogeneous_controls()
                    .iter()
                    .map(|control| {
                        crate::HomogeneousControl2::new(
                            &transform.m00 * control.x()
                                + &transform.m01 * control.y()
                                + &transform.tx * control.weight(),
                            &transform.m10 * control.x()
                                + &transform.m11 * control.y()
                                + &transform.ty * control.weight(),
                            control.weight().clone(),
                        )
                    })
                    .collect(),
                curve.knots().to_vec(),
                curve.periodicity().clone(),
                &CurveContext::STRICT,
            )
            .map_err(svg_geometry_error)?,
        )],
    };
    Ok(transformed)
}

fn transform_bezier_subcurve(
    curve: &BezierSubcurve2,
    transform: &ExactAffine2,
) -> SvgResult<BezierSubcurve2> {
    let family = Curve2::from(curve.clone()).family();
    let invalid = |cause| ExactCurveError::Invalid {
        operation: CurveOperation2::Transformation,
        family: Some(family),
        cause,
    };
    match curve {
        BezierSubcurve2::Quadratic(curve) => Ok(BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            transform.transform_point(curve.start()),
            transform.transform_point(curve.control()),
            transform.transform_point(curve.end()),
        ))),
        BezierSubcurve2::Cubic(curve) => Ok(BezierSubcurve2::Cubic(CubicBezier2::new(
            transform.transform_point(curve.start()),
            transform.transform_point(curve.control1()),
            transform.transform_point(curve.control2()),
            transform.transform_point(curve.end()),
        ))),
        BezierSubcurve2::RationalQuadratic(curve) => Ok(BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(
                transform.transform_point(curve.start()),
                transform.transform_point(curve.control()),
                transform.transform_point(curve.end()),
                curve.start_weight().clone(),
                curve.control_weight().clone(),
                curve.end_weight().clone(),
            )
            .map_err(invalid)
            .map_err(svg_geometry_error)?,
        )),
        BezierSubcurve2::Rational(curve) => {
            Ok(BezierSubcurve2::Rational(curve.transformed_affine([
                &transform.m00,
                &transform.m01,
                &transform.m10,
                &transform.m11,
                &transform.tx,
                &transform.ty,
            ])))
        }
    }
}

fn open_path(points: &[Point2]) -> SvgResult<CurvePath2> {
    let curves = points
        .windows(2)
        .filter(|points| points[0] != points[1])
        .map(|points| {
            LineSeg2::try_new(points[0].clone(), points[1].clone())
                .map(Curve2::from)
                .map_err(svg_geometry_error)
        })
        .collect::<SvgResult<Vec<_>>>()?;
    if curves.is_empty() {
        return Err(SvgError::MalformedInput(
            "shape contains no nondegenerate edge".into(),
        ));
    }
    CurvePath2::try_new(curves).map_err(svg_geometry_error)
}

fn closed_path(points: &[Point2]) -> SvgResult<CurvePath2> {
    let mut points = points.to_vec();
    points.dedup();
    if points.first() == points.last() {
        points.pop();
    }
    if points.len() < 3 {
        return Err(SvgError::MalformedInput(
            "closed shape requires at least three distinct points".into(),
        ));
    }
    let mut curves = points
        .windows(2)
        .map(|points| {
            LineSeg2::try_new(points[0].clone(), points[1].clone())
                .map(Curve2::from)
                .map_err(svg_geometry_error)
        })
        .collect::<SvgResult<Vec<_>>>()?;
    curves.push(
        LineSeg2::try_new(
            points.last().expect("closed path has points").clone(),
            points[0].clone(),
        )
        .map(Curve2::from)
        .map_err(svg_geometry_error)?,
    );
    CurvePath2::try_new(curves).map_err(svg_geometry_error)
}

fn sampled_ellipse_path(
    cx: f64,
    cy: f64,
    rx: f64,
    ry: f64,
    segments: usize,
) -> SvgResult<CurvePath2> {
    if ![cx, cy, rx, ry].into_iter().all(f64::is_finite) || rx <= 0.0 || ry <= 0.0 {
        return Err(SvgError::MalformedInput(
            "ellipse center and radii must be finite, with positive radii".into(),
        ));
    }
    let points = (0..segments)
        .map(|index| {
            let angle = std::f64::consts::TAU * index as f64 / segments as f64;
            Ok(Point2::new(
                real(cx + rx * angle.cos(), "ellipse x")?,
                real(cy + ry * angle.sin(), "ellipse y")?,
            ))
        })
        .collect::<SvgResult<Vec<_>>>()?;
    closed_path(&points)
}

fn ellipse_path(
    attrs: &svg::node::Attributes,
    radius_attributes: (&str, Option<&str>),
    options: SvgOptions,
    materialize: bool,
) -> SvgResult<Option<CurvePath2>> {
    let mut zero_radius = false;
    for attribute in [Some(radius_attributes.0), radius_attributes.1]
        .into_iter()
        .flatten()
    {
        let value = attrs
            .get(attribute)
            .ok_or_else(|| SvgError::MalformedInput(format!("missing attribute {attribute}")))?;
        zero_radius |=
            !style_number_positive(value, None, "ellipse radii must be finite and non-negative")?;
    }
    let cx = number(attrs, "cx", Some(0.0))?;
    let cy = number(attrs, "cy", Some(0.0))?;
    if !materialize {
        return Ok(None);
    }
    // Preserve existing hidden zero-radius empty selection and the visible
    // sampler's positive-radius requirement. Visible zero semantics are not
    // extended here.
    if zero_radius {
        return Err(SvgError::MalformedInput(
            "ellipse radii must be positive".into(),
        ));
    }
    // Keep the existing visible sampled model. Exact source validation above
    // distinguishes a positive value lost at this binary64 boundary from an
    // invalid authored radius; unused geometry never needs this conversion.
    let rx = number(attrs, radius_attributes.0, None)?;
    let ry = radius_attributes
        .1
        .map(|attribute| number(attrs, attribute, None))
        .transpose()?
        .unwrap_or(rx);
    if rx <= 0.0 || ry <= 0.0 {
        return Err(SvgError::Unsupported(
            "ellipse cannot be sampled with binary64 radii".into(),
        ));
    }
    let segments = options.segments_for_radius(rx.max(ry))?;
    sampled_ellipse_path(cx, cy, rx, ry, segments).map(Some)
}

fn rectangle_path(
    attrs: &svg::node::Attributes,
    options: SvgOptions,
    materialize: bool,
) -> SvgResult<Option<CurvePath2>> {
    let x = exact_number(attrs, "x", Some(0.0))?;
    let y = exact_number(attrs, "y", Some(0.0))?;
    let width = exact_number(attrs, "width", None)?;
    let height = exact_number(attrs, "height", None)?;
    let rx_attr = attrs
        .get("rx")
        .map(|_| exact_number(attrs, "rx", None))
        .transpose()?;
    let ry_attr = attrs
        .get("ry")
        .map(|_| exact_number(attrs, "ry", None))
        .transpose()?;
    let (rx, ry) = match (rx_attr, ry_attr) {
        (None, None) => (Real::zero(), Real::zero()),
        (Some(rx), None) => (rx.clone(), rx),
        (None, Some(ry)) => (ry.clone(), ry),
        (Some(rx), Some(ry)) => (rx, ry),
    };
    let mut unrounded = false;
    for (value, dimension) in [(&width, true), (&height, true), (&rx, false), (&ry, false)] {
        let sign = svg_real_sign(value).ok_or_else(|| {
            SvgError::Exact(ExactCurveError::blocked_unattributed(
                CurveOperation2::Classification,
                crate::UncertaintyReason::RealSign,
            ))
        })?;
        if sign == RealSign::Negative || (dimension && sign == RealSign::Zero) {
            return Err(SvgError::MalformedInput(
                if dimension {
                    "rectangle dimensions must be positive"
                } else {
                    "rectangle corner radii must be finite and non-negative"
                }
                .into(),
            ));
        }
        unrounded |= !dimension && sign == RealSign::Zero;
    }
    if !materialize {
        return Ok(None);
    }
    if unrounded {
        let right = &x + &width;
        let top = &y + &height;
        return closed_path(&[
            Point2::new(x.clone(), y.clone()),
            Point2::new(right.clone(), y),
            Point2::new(right, top.clone()),
            Point2::new(x, top),
        ])
        .map(Some);
    }

    // Retain the existing sampled rounded-shape calculation. Its source
    // fidelity is separate from the exact native unrounded boundary above.
    let x = number(attrs, "x", Some(0.0))?;
    let y = number(attrs, "y", Some(0.0))?;
    let width = number(attrs, "width", None)?;
    let height = number(attrs, "height", None)?;
    if width <= 0.0 || height <= 0.0 || !(x + width).is_finite() || !(y + height).is_finite() {
        return Err(SvgError::Unsupported(
            "rounded rectangle cannot be sampled with binary64 dimensions and bounds".into(),
        ));
    }
    let rx_attr = attrs
        .get("rx")
        .map(|value| value.parse::<f64>())
        .transpose()?;
    let ry_attr = attrs
        .get("ry")
        .map(|value| value.parse::<f64>())
        .transpose()?;
    let (rx, ry) = match (rx_attr, ry_attr) {
        (None, None) => (0.0, 0.0),
        (Some(rx), None) => (rx, rx),
        (None, Some(ry)) => (ry, ry),
        (Some(rx), Some(ry)) => (rx, ry),
    };
    if !rx.is_finite() || !ry.is_finite() || rx < 0.0 || ry < 0.0 {
        return Err(SvgError::MalformedInput(
            "rectangle corner radii must be finite and non-negative".into(),
        ));
    }
    let rx = rx.clamp(0.0, width / 2.0);
    let ry = ry.clamp(0.0, height / 2.0);
    if rx == 0.0 || ry == 0.0 {
        return closed_path(&[
            Point2::new(real(x, "rectangle x")?, real(y, "rectangle y")?),
            Point2::new(real(x + width, "rectangle x")?, real(y, "rectangle y")?),
            Point2::new(
                real(x + width, "rectangle x")?,
                real(y + height, "rectangle y")?,
            ),
            Point2::new(real(x, "rectangle x")?, real(y + height, "rectangle y")?),
        ])
        .map(Some);
    }

    let segments = (options.segments_for_radius(rx.max(ry))? / 4).max(1);
    let mut boundary = Vec::with_capacity(4 * (segments + 1));
    for (center_x, center_y, start) in [
        (x + width - rx, y + ry, -std::f64::consts::FRAC_PI_2),
        (x + width - rx, y + height - ry, 0.0),
        (x + rx, y + height - ry, std::f64::consts::FRAC_PI_2),
        (x + rx, y + ry, std::f64::consts::PI),
    ] {
        for index in 0..=segments {
            let angle = start + std::f64::consts::FRAC_PI_2 * index as f64 / segments as f64;
            boundary.push(Point2::new(
                real(center_x + rx * angle.cos(), "rounded rectangle x")?,
                real(center_y + ry * angle.sin(), "rounded rectangle y")?,
            ));
        }
    }
    closed_path(&boundary).map(Some)
}

fn path_as_native_wire(path: &CurvePath2) -> Option<CurveString2> {
    if path
        .curves()
        .iter()
        .any(|curve| curve.parallel_fidelity().is_some())
    {
        return None;
    }
    let segments = path
        .curves()
        .iter()
        .map(|curve| match curve.geometry() {
            Some(CurveGeometry2::Line(line)) => Some(Segment2::Line(line.clone())),
            Some(CurveGeometry2::CircularArc(arc)) => Some(Segment2::Arc(arc.clone())),
            _ => None,
        })
        .collect::<Option<Vec<_>>>()?;
    CurveString2::try_new(segments).ok()
}

fn region_from_paths(
    paths: &[CurvePath2],
    fill_rule: FillRule,
    preserve_native_lines: bool,
) -> SvgResult<CurveRegion2> {
    let construct = if preserve_native_lines {
        CurveRegion2::replay_boundary_paths_with_policy
    } else {
        CurveRegion2::try_from_boundary_paths_with_policy
    };
    construct(paths, fill_rule, &CurveContext::STRICT)
        .map(CurveOutcome::into_value)
        .map_err(SvgError::Exact)
}

fn geometry_from_paths(
    fill_paths: &[CurvePath2],
    stroke_paths: &[CurvePath2],
    context: StyleContext,
) -> SvgResult<SvgGeometry2> {
    geometry_from_paths_preserving_native_lines(fill_paths, stroke_paths, context, false)
}

fn geometry_from_paths_preserving_native_lines(
    fill_paths: &[CurvePath2],
    stroke_paths: &[CurvePath2],
    context: StyleContext,
    preserve_native_lines: bool,
) -> SvgResult<SvgGeometry2> {
    if !context.visible() || (!context.fills() && !context.strokes()) {
        return Ok(SvgGeometry2::empty());
    }
    let transform = context
        .transform
        .as_ref()
        .filter(|transform| !transform.is_identity());
    let transformed_fill = if context.fills() {
        match &transform {
            Some(transform) => fill_paths
                .iter()
                .map(|path| transform_path(path, transform))
                .collect::<SvgResult<Vec<_>>>()?,
            None => fill_paths.to_vec(),
        }
    } else {
        Vec::new()
    };
    let transformed_strokes = if context.strokes() {
        match &transform {
            Some(transform) => stroke_paths
                .iter()
                .map(|path| transform_path(path, transform))
                .collect::<SvgResult<Vec<_>>>()?,
            None => stroke_paths.to_vec(),
        }
    } else {
        Vec::new()
    };
    let region = region_from_paths(&transformed_fill, context.fill_rule, preserve_native_lines)?;
    let mut wires = Vec::new();
    let mut paths = Vec::new();
    for path in transformed_strokes {
        if let Some(wire) = path_as_native_wire(&path) {
            wires.push(wire);
        } else {
            paths.push(path);
        }
    }
    Ok(SvgGeometry2::new(region, wires, paths))
}

fn path_geometry(
    data: &str,
    exact_path: Option<&str>,
    context: StyleContext,
    options: SvgOptions,
) -> SvgResult<SvgGeometry2> {
    if let Some(encoded) = exact_path {
        let paths = exact::decode_paths(
            encoded,
            options.max_extension_bytes,
            options.max_point_definition_depth,
            options.max_parallel_verification_depth,
        )?;
        if context.visible() && context.fills() {
            for path in &paths {
                let connected = crate::curve::validate_closed_curve_path_connectivity(
                    path,
                    &CurveContext::STRICT,
                )
                .map_err(SvgError::Exact)?;
                ExactCurveError::decided(crate::CurveOperation2::Construction, connected)
                    .map_err(SvgError::Exact)?;
            }
        }
        return geometry_from_paths_preserving_native_lines(&paths, &paths, context, true);
    }

    if !context.visible() {
        // Hidden standard paths still have to satisfy the command grammar.
        // Constructing their geometry would impose visible carrier limits on
        // unselected curves, including valid noncircular elliptical arcs.
        for segment in svg_path_segments(data) {
            segment?;
        }
        return Ok(SvgGeometry2::empty());
    }

    let subpaths = parse_svg_path_data(data)?;
    if context.fills() && subpaths.iter().any(|subpath| !subpath.closed) {
        return Err(SvgError::Geometry(
            "filled path contains an explicitly open subpath".into(),
        ));
    }
    let paths = subpaths
        .iter()
        .map(|subpath| subpath.path.clone())
        .collect::<Vec<_>>();
    geometry_from_paths(&paths, &paths, context)
}

/// Parses a complete SVG document with default projection and sampling options.
pub fn import_svg_document(document: &str) -> SvgResult<SvgGeometry2> {
    import_svg_document_with_options(document, SvgOptions::default())
}

/// Parses a complete SVG document into native Hypercurve topology.
pub fn import_svg_document_with_options(
    document: &str,
    options: SvgOptions,
) -> SvgResult<SvgGeometry2> {
    use svg::node::element::tag;
    use svg::node::element::tag::Type::{Empty, End, Start};
    use svg::parser::Event;

    let options = options.validate()?;
    let mut contexts = vec![StyleContext::default()];
    let mut output = SvgGeometry2::empty();
    for event in svg::read(document)? {
        let Event::Tag(name, kind, attrs) = event else {
            if let Event::Error(error) = event {
                return Err(error.into());
            }
            continue;
        };
        if matches!(name, tag::Group | tag::SVG) {
            match kind {
                Start => {
                    if name == tag::SVG
                        && contexts.len() > 1
                        && ["x", "y", "viewBox", "preserveAspectRatio"]
                            .iter()
                            .any(|attribute| attrs.contains_key(*attribute))
                    {
                        return Err(SvgError::Unsupported(
                            "nested SVG viewport transforms".into(),
                        ));
                    }
                    let parent = contexts.last().cloned().ok_or_else(|| {
                        SvgError::MalformedInput("style context stack is empty".into())
                    })?;
                    contexts.push(apply_style(parent, &attrs)?);
                }
                End => {
                    if contexts.len() > 1 {
                        contexts.pop();
                    }
                }
                Empty => {}
            }
            continue;
        }
        if matches!(kind, End) || matches!(name, tag::Description | tag::Title) {
            continue;
        }
        let parent = contexts
            .last()
            .cloned()
            .ok_or_else(|| SvgError::MalformedInput("style context stack is empty".into()))?;
        let context = apply_style(parent, &attrs)?;
        // Validate paths and primitive attributes before selecting empty
        // geometry. The handlers skip transforms and arrangement when hidden.
        if !context.visible()
            && !matches!(
                name,
                tag::Path
                    | tag::Line
                    | tag::Polygon
                    | tag::Polyline
                    | tag::Rectangle
                    | tag::Circle
                    | tag::Ellipse
            )
        {
            continue;
        }

        let shape =
            match name {
                tag::Path => {
                    let data = attrs.get("d").ok_or_else(|| {
                        SvgError::MalformedInput("missing path d attribute".into())
                    })?;
                    path_geometry(
                        data,
                        attrs.get(EXACT_PATH_ATTRIBUTE).map(|value| &**value),
                        context,
                        options,
                    )?
                }
                tag::Circle => {
                    let materialize = context.visible() && (context.fills() || context.strokes());
                    match ellipse_path(&attrs, ("r", None), options, materialize)? {
                        Some(path) => geometry_from_paths(
                            std::slice::from_ref(&path),
                            std::slice::from_ref(&path),
                            context,
                        )?,
                        None => SvgGeometry2::empty(),
                    }
                }
                tag::Ellipse => {
                    let materialize = context.visible() && (context.fills() || context.strokes());
                    match ellipse_path(&attrs, ("rx", Some("ry")), options, materialize)? {
                        Some(path) => geometry_from_paths(
                            std::slice::from_ref(&path),
                            std::slice::from_ref(&path),
                            context,
                        )?,
                        None => SvgGeometry2::empty(),
                    }
                }
                tag::Rectangle => {
                    let materialize = context.visible() && (context.fills() || context.strokes());
                    match rectangle_path(&attrs, options, materialize)? {
                        Some(path) => geometry_from_paths(
                            std::slice::from_ref(&path),
                            std::slice::from_ref(&path),
                            context,
                        )?,
                        None => SvgGeometry2::empty(),
                    }
                }
                tag::Line => {
                    let path = open_path(&[
                        Point2::new(
                            exact_number(&attrs, "x1", Some(0.0))?,
                            exact_number(&attrs, "y1", Some(0.0))?,
                        ),
                        Point2::new(
                            exact_number(&attrs, "x2", Some(0.0))?,
                            exact_number(&attrs, "y2", Some(0.0))?,
                        ),
                    ])?;
                    geometry_from_paths(&[], std::slice::from_ref(&path), context)?
                }
                tag::Polygon => {
                    let polygon_points = points(attrs.get("points").ok_or_else(|| {
                        SvgError::MalformedInput("missing polygon points".into())
                    })?)?;
                    if polygon_points.len() < 3 {
                        return Err(SvgError::MalformedInput(
                            "polygon requires at least three points".into(),
                        ));
                    }
                    let path = closed_path(&polygon_points)?;
                    geometry_from_paths(
                        std::slice::from_ref(&path),
                        std::slice::from_ref(&path),
                        context,
                    )?
                }
                tag::Polyline => {
                    let polyline_points = points(attrs.get("points").ok_or_else(|| {
                        SvgError::MalformedInput("missing polyline points".into())
                    })?)?;
                    let stroke_path = open_path(&polyline_points)?;
                    let fill_path = (polyline_points.len() >= 3)
                        .then(|| closed_path(&polyline_points))
                        .transpose()?;
                    geometry_from_paths(
                        fill_path.as_slice(),
                        std::slice::from_ref(&stroke_path),
                        context,
                    )?
                }
                other => {
                    return Err(SvgError::Unsupported(format!("element {other}")));
                }
            };
        output.append(shape)?;
    }
    Ok(output)
}

/// Serializes native Hypercurve topology as a complete SVG document.
///
/// Paths and filled regions carry a versioned exact Hypercurve extension in
/// addition to their interoperable standard-SVG `d` projection. Region boundaries
/// share one checked definition graph and re-enter the existing arrangement on
/// import. Native [`CurveGeometry2`] families and exact [`Real`] payloads retain
/// their definitions. Unsupported retained definitions fail explicitly.
/// An exact empty document carries a zero-root native graph and uses a unit
/// display viewport. The viewport adds no filled geometry.
pub fn export_svg_document(geometry: &SvgGeometry2) -> SvgResult<String> {
    export_svg_document_with_options(geometry, SvgOptions::default())
}

/// Serializes native Hypercurve topology with explicit finite projection and
/// exact-extension bounds.
pub fn export_svg_document_with_options(
    geometry: &SvgGeometry2,
    options: SvgOptions,
) -> SvgResult<String> {
    let options = options.validate()?;
    let projection =
        FiniteProjectionOptions::try_new(options.curve_tolerance).map_err(svg_geometry_error)?;
    let mut body = String::new();
    // Even an empty boundary can depend on qualified topology decisions.
    // Revalidate before choosing a display viewport or omitting its fill.
    let region = geometry
        .region
        .regularized_region_raw(&CurveContext::STRICT)
        .map_err(SvgError::Exact)?;
    let bounds = exact_finite_bounds(geometry, &region)?;

    if !region.is_empty() || (geometry.wires.is_empty() && geometry.paths.is_empty()) {
        let boundaries = ExactCurveError::decided(
            crate::CurveOperation2::NativeTopology,
            region
                .boundary_paths_with_policy(&CurveContext::STRICT)
                .map_err(SvgError::Exact)?
                .into_value(),
        )
        .map_err(SvgError::Exact)?;
        let exact = exact::encode_paths(
            &boundaries,
            options.max_extension_bytes,
            options.max_point_definition_depth,
        )?;
        let profiles = match region
            .project_to_finite_profiles_with_policy(&projection, &CurveContext::STRICT)
            .map_err(svg_projection_error)?
            .into_value()
        {
            Classification::Decided(profiles) => profiles,
            Classification::Uncertain(reason) => {
                return Err(SvgError::Geometry(format!(
                    "region projection is uncertain: {reason:?}"
                )));
            }
        };
        write!(
            &mut body,
            "<path fill=\"black\" fill-rule=\"evenodd\" stroke=\"none\" {EXACT_PATH_ATTRIBUTE}=\"{exact}\" d=\""
        )
        .expect("writing SVG path element to String cannot fail");
        for profile in profiles {
            append_finite_path(&mut body, profile.material().points(), true, options)?;
            for hole in profile.holes() {
                append_finite_path(&mut body, hole.points(), true, options)?;
            }
        }
        body.push_str("\"/>\n");
    }

    for wire in &geometry.wires {
        let path = curve_path_from_wire(wire)?;
        let exact = exact::encode_path(
            &path,
            options.max_extension_bytes,
            options.max_point_definition_depth,
        )?;
        write!(
            &mut body,
            "<path fill=\"none\" stroke=\"black\" {EXACT_PATH_ATTRIBUTE}=\"{exact}\" d=\""
        )
        .expect("writing SVG path element to String cannot fail");
        append_native_path(&mut body, &path, &projection, options)?;
        body.push_str("\"/>\n");
    }
    for path in &geometry.paths {
        let exact = exact::encode_path(
            path,
            options.max_extension_bytes,
            options.max_point_definition_depth,
        )?;
        write!(
            &mut body,
            "<path fill=\"none\" stroke=\"black\" {EXACT_PATH_ATTRIBUTE}=\"{exact}\" d=\""
        )
        .expect("writing SVG path element to String cannot fail");
        append_native_path(&mut body, path, &projection, options)?;
        body.push_str("\"/>\n");
    }

    Ok(format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{:.17} {:.17} {:.17} {:.17}\">\n{body}</svg>",
        bounds[0],
        bounds[1],
        bounds[2] - bounds[0],
        bounds[3] - bounds[1],
    ))
}

fn curve_path_from_wire(wire: &CurveString2) -> SvgResult<CurvePath2> {
    let curves = wire
        .segments()
        .iter()
        .map(|segment| match segment {
            Segment2::Line(line) => Curve2::from(line.clone()),
            Segment2::Arc(arc) => Curve2::from(arc.clone()),
        })
        .collect();
    CurvePath2::try_new(curves).map_err(svg_geometry_error)
}

fn append_native_path(
    output: &mut String,
    path: &CurvePath2,
    projection: &FiniteProjectionOptions,
    options: SvgOptions,
) -> SvgResult<()> {
    if path.curves().iter().any(|curve| curve.geometry().is_none()) {
        let projected = path
            .project_to_finite_polyline_with_policy(projection, &CurveContext::STRICT)
            .map_err(svg_projection_error)?
            .into_value();
        if projected.points().len().saturating_sub(1) > options.max_curve_segments {
            return Err(SvgError::SizeOverflow {
                limit: "configured curve-segment count",
            });
        }
        for (index, point) in projected.points().iter().enumerate() {
            write!(
                output,
                "{} {:.17} {:.17}",
                if index == 0 { "M" } else { " L" },
                point[0],
                point[1]
            )
            .expect("writing SVG path data to String cannot fail");
        }
        return Ok(());
    }
    let start = finite_point(path.curves()[0].geometry().expect("native path").start())?;
    write!(output, "M {:.17} {:.17}", start[0], start[1])
        .expect("writing SVG path data to String cannot fail");
    let mut emitted_segments = 0_usize;

    for curve in path.curves() {
        match curve.geometry() {
            None => unreachable!("retained paths projected above"),
            Some(CurveGeometry2::Line(line)) => {
                append_line_command(output, line.end())?;
                emitted_segments += 1;
            }
            Some(CurveGeometry2::CircularArc(arc)) => {
                let radius = arc
                    .radius_squared_ref()
                    .clone()
                    .sqrt()
                    .map_err(svg_geometry_error)?
                    .to_f64_lossy()
                    .filter(|radius| radius.is_finite() && *radius > 0.0)
                    .ok_or_else(|| {
                        SvgError::Geometry(
                            "circular arc radius is not representable as finite f64".into(),
                        )
                    })?;
                let sweep = u8::from(arc.is_clockwise());
                match crate::arc_bezier::classify_sweep(arc).map_err(svg_geometry_error)? {
                    crate::arc_bezier::ArcSweepKind::FullCircle => {
                        let midpoint = match arc
                            .representative_point_with_policy(&CurveContext::STRICT)
                            .map_err(svg_geometry_error)?
                        {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Err(SvgError::Geometry(format!(
                                    "full-circle SVG midpoint is uncertain: {reason:?}"
                                )));
                            }
                        };
                        append_arc_command(output, radius, false, sweep, &midpoint)?;
                        append_arc_command(output, radius, false, sweep, arc.end())?;
                        emitted_segments += 2;
                    }
                    kind => {
                        append_arc_command(
                            output,
                            radius,
                            kind == crate::arc_bezier::ArcSweepKind::Major,
                            sweep,
                            arc.end(),
                        )?;
                        emitted_segments += 1;
                    }
                }
            }
            Some(CurveGeometry2::QuadraticBezier(curve)) => {
                let control = finite_point(curve.control())?;
                let end = finite_point(curve.end())?;
                write!(
                    output,
                    " Q {:.17} {:.17} {:.17} {:.17}",
                    control[0], control[1], end[0], end[1]
                )
                .expect("writing SVG quadratic command to String cannot fail");
                emitted_segments += 1;
            }
            Some(CurveGeometry2::CubicBezier(curve)) => {
                let control1 = finite_point(curve.control1())?;
                let control2 = finite_point(curve.control2())?;
                let end = finite_point(curve.end())?;
                write!(
                    output,
                    " C {:.17} {:.17} {:.17} {:.17} {:.17} {:.17}",
                    control1[0], control1[1], control2[0], control2[1], end[0], end[1]
                )
                .expect("writing SVG cubic command to String cannot fail");
                emitted_segments += 1;
            }
            Some(CurveGeometry2::RationalQuadraticBezier(_))
            | Some(CurveGeometry2::RationalBezier(_))
            | Some(CurveGeometry2::PolynomialBSpline(_))
            | Some(CurveGeometry2::Nurbs(_)) => {
                let one_curve =
                    CurvePath2::try_new(vec![curve.clone()]).map_err(svg_geometry_error)?;
                let polyline = one_curve
                    .project_to_finite_polyline_with_policy(projection, &CurveContext::STRICT)
                    .map_err(svg_projection_error)?
                    .into_value();
                if polyline.points().len() < 2 {
                    return Err(SvgError::Geometry(
                        "finite projection emitted no curve continuation".into(),
                    ));
                }
                for point in &polyline.points()[1..] {
                    if point.iter().any(|coordinate| !coordinate.is_finite()) {
                        return Err(SvgError::Geometry(
                            "finite projection emitted a non-finite coordinate".into(),
                        ));
                    }
                    write!(output, " L {:.17} {:.17}", point[0], point[1])
                        .expect("writing SVG path data to String cannot fail");
                }
                emitted_segments += polyline.points().len() - 1;
            }
        }
        if emitted_segments > options.max_curve_segments {
            return Err(SvgError::SizeOverflow {
                limit: "configured curve-segment count",
            });
        }
    }
    Ok(())
}

fn append_line_command(output: &mut String, end: &Point2) -> SvgResult<()> {
    let end = finite_point(end)?;
    write!(output, " L {:.17} {:.17}", end[0], end[1])
        .expect("writing SVG line command to String cannot fail");
    Ok(())
}

fn append_arc_command(
    output: &mut String,
    radius: f64,
    large_arc: bool,
    sweep: u8,
    end: &Point2,
) -> SvgResult<()> {
    let end = finite_point(end)?;
    write!(
        output,
        " A {radius:.17} {radius:.17} 0 {} {sweep} {:.17} {:.17}",
        u8::from(large_arc),
        end[0],
        end[1]
    )
    .expect("writing SVG arc command to String cannot fail");
    Ok(())
}

fn finite_point(point: &Point2) -> SvgResult<[f64; 2]> {
    [point.x(), point.y()]
        .map(|coordinate| {
            coordinate
                .to_f64_lossy()
                .filter(|value| value.is_finite())
                .ok_or_else(|| {
                    SvgError::Geometry(
                        "exact SVG coordinate is not representable as finite f64".into(),
                    )
                })
        })
        .into_iter()
        .collect::<SvgResult<Vec<_>>>()
        .map(|point| [point[0], point[1]])
}

fn exact_finite_bounds(geometry: &SvgGeometry2, region: &CurveRegion2) -> SvgResult<[f64; 4]> {
    let policy = CurveContext::STRICT;
    let mut bounds = if region.is_empty() {
        None
    } else {
        match region.bounds_raw(&policy).map_err(svg_geometry_error)? {
            Classification::Decided(bounds) => Some(bounds),
            Classification::Uncertain(reason) => {
                return Err(SvgError::Geometry(format!(
                    "region bounds are uncertain: {reason:?}"
                )));
            }
        }
    };
    for wire in &geometry.wires {
        let next = match Aabb2::from_curve_string(wire).map_err(svg_geometry_error)? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Err(SvgError::Geometry(format!(
                    "wire bounds are uncertain: {reason:?}"
                )));
            }
        };
        merge_bounds(&mut bounds, next)?;
    }
    for path in &geometry.paths {
        merge_bounds(
            &mut bounds,
            path.bounds().map_err(svg_geometry_error)?.clone(),
        )?;
    }
    let Some(bounds) = bounds else {
        // All carriers were checked above. This viewport is a display
        // convention for the empty set, not an added boundary or region.
        return Ok([0.0, 0.0, 1.0, 1.0]);
    };
    [
        bounds.min_x(),
        bounds.min_y(),
        bounds.max_x(),
        bounds.max_y(),
    ]
    .map(|coordinate| {
        coordinate
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .ok_or_else(|| {
                SvgError::Geometry("exact SVG bound is not representable as finite f64".into())
            })
    })
    .into_iter()
    .collect::<SvgResult<Vec<_>>>()
    .map(|bounds| [bounds[0], bounds[1], bounds[2], bounds[3]])
}

fn merge_bounds(bounds: &mut Option<Aabb2>, next: Aabb2) -> SvgResult<()> {
    *bounds = Some(match bounds.take() {
        Some(current) => match current.union(&next) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Err(SvgError::Geometry(format!(
                    "combined SVG bounds are uncertain: {reason:?}"
                )));
            }
        },
        None => next,
    });
    Ok(())
}

fn append_finite_path(
    output: &mut String,
    points: &[[f64; 2]],
    closed: bool,
    options: SvgOptions,
) -> SvgResult<()> {
    if points.is_empty() {
        return Err(SvgError::Geometry(
            "finite projection emitted an empty path".into(),
        ));
    }
    if points.len().saturating_sub(1) > options.max_curve_segments {
        return Err(SvgError::SizeOverflow {
            limit: "configured curve-segment count",
        });
    }
    if points
        .iter()
        .flatten()
        .any(|coordinate| !coordinate.is_finite())
    {
        return Err(SvgError::Geometry(
            "finite projection emitted a non-finite coordinate".into(),
        ));
    }
    let first = points[0];
    write!(output, "M {:.17} {:.17}", first[0], first[1])
        .expect("writing SVG path data to String cannot fail");
    for point in &points[1..] {
        write!(output, " L {:.17} {:.17}", point[0], point[1])
            .expect("writing SVG path data to String cannot fail");
    }
    if closed {
        output.push_str(" Z ");
    }
    Ok(())
}
