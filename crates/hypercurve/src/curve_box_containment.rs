//! Exact containment of curve images in closed axis-aligned boxes.
//!
//! [`Curve2::bounds`] is a cached conservative enclosure. For polynomial
//! Bezier spans it is the control-hull box, which can exceed the curve image
//! and changes when the same image is split differently. The cubic with
//! controls `(8, 0), (9, 3), (11, 3), (12, 0)` traces `y = 9t(1 - t)`: its
//! hull reaches `y = 3`, the curve only `y = 9/4`, and the two halves split at
//! `t = 1/2` have hulls reaching exactly `9/4`. A containment decision read
//! from the enclosure would therefore depend on the representation.
//!
//! The queries here decide the image itself. The cached enclosure stays the
//! fast path: a side of the box that contains the enclosure needs no further
//! work. Each remaining side is decided from the exact extremum candidates of
//! its coordinate: span endpoints, derivative roots of polynomial and rational
//! Bezier spans, and cardinal points inside a circular arc's sweep. Retained
//! rational spans filter derivative roots through their active source range
//! and prove that range has no denominator pole. A differentiable coordinate attains its extrema at
//! these points, and every candidate is an exact point of the curve, so a
//! candidate outside the box is a witness and no candidate outside proves
//! containment. Neither answer depends on how the image is split.

use std::cmp::Ordering;

use crate::classify::compare_reals;
use crate::{
    Aabb2, Axis2, BezierSubcurve2, CircularArc2, Classification, Curve2, CurveContext,
    CurveGeometry2, CurveOperation2, CurvePath2, CurvePoint2, ExactCurveError, ExactCurveResult,
    NativeBezierFragment2, Point2, UncertaintyReason,
};

/// One side of a closed axis-aligned box.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoxBoundary2 {
    /// The side at the axis minimum.
    Minimum,
    /// The side at the axis maximum.
    Maximum,
}

/// An exact point of a curve that lies strictly outside a closed box.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveBoxEscape2 {
    curve_index: usize,
    axis: Axis2,
    boundary: BoxBoundary2,
    point: CurvePoint2,
}

impl CurveBoxEscape2 {
    /// Index of the curve that leaves the box; zero for a single curve.
    pub const fn curve_index(&self) -> usize {
        self.curve_index
    }

    /// Axis on which the witness lies outside the box.
    pub const fn axis(&self) -> Axis2 {
        self.axis
    }

    /// Side of the box that the witness lies beyond.
    pub const fn boundary(&self) -> BoxBoundary2 {
        self.boundary
    }

    /// The witness, an exact point of the curve.
    pub const fn point(&self) -> &CurvePoint2 {
        &self.point
    }
}

/// Relation between a curve image and a closed axis-aligned box.
///
/// An undecided relation is not a variant: it is reported as
/// [`ExactCurveError::Blocked`], with [`UncertaintyReason::Unsupported`] when
/// the enclosure crosses the box and the curve has no exact extremum
/// candidates, and an ordering or sign reason when a comparison could not be
/// decided.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveBoxContainment2 {
    /// Every point of the curve lies in the closed box.
    Contained,
    /// The witness is a point of the curve strictly outside the box.
    Escapes(CurveBoxEscape2),
}

impl Curve2 {
    /// Decides whether this curve lies in the closed box `envelope`.
    ///
    /// See [`CurvePath2::box_containment`].
    pub fn box_containment(&self, envelope: &Aabb2) -> ExactCurveResult<CurveBoxContainment2> {
        let policy = crate::policy::principal_context();
        Ok(match curve_box_escape(self, 0, envelope, &policy)? {
            Some(escape) => CurveBoxContainment2::Escapes(escape),
            None => CurveBoxContainment2::Contained,
        })
    }
}

impl CurvePath2 {
    /// Decides whether every curve of this path lies in the closed box `envelope`.
    ///
    /// [`Self::bounds`] is a conservative enclosure: when it lies in the box
    /// the path does, but when it crosses the box the path may still be
    /// contained. This query decides the image. Sides of the box that contain
    /// a curve's cached bounds are accepted without further work; the others
    /// are decided from that curve's exact extremum candidates. A decided
    /// result depends only on the image, not on how the path's curves were
    /// split or traversed.
    ///
    /// Exact candidates exist for lines, circular arcs, polynomial quadratic
    /// and cubic Beziers, and rational Beziers and NURBS spans whose quotient
    /// derivative roots can be isolated under the selected policy and budget.
    /// Retained rational/source-range spans admit only roots certified in their
    /// active interval within the unit source chart; the denominator must be
    /// nonzero throughout that interval. Selected coordinates are compared
    /// through retained evidence, without mandatory Cartesian reconstruction.
    /// Analytic parallels and implicit-polynomial carriers contribute only
    /// their endpoints. When such a curve's enclosure crosses
    /// the box and no candidate escapes, the query reports a typed blocker. A
    /// definite escape on any curve takes precedence over a blocker on
    /// another; the witness is the first escape in path order. Invalid curve
    /// state is reported at once.
    ///
    /// Decisions follow the principal predicate policy, so the query composes
    /// with [`crate::evaluate_under`] and [`crate::evaluate_with_budget`].
    pub fn box_containment(&self, envelope: &Aabb2) -> ExactCurveResult<CurveBoxContainment2> {
        let policy = crate::policy::principal_context();
        let mut blocker = None;
        for (index, curve) in self.curves().iter().enumerate() {
            match curve_box_escape(curve, index, envelope, &policy) {
                Ok(Some(escape)) => return Ok(CurveBoxContainment2::Escapes(escape)),
                Ok(None) => {}
                Err(error @ ExactCurveError::Blocked(_)) => {
                    blocker.get_or_insert(error);
                }
                Err(error) => return Err(error),
            }
        }
        match blocker {
            Some(error) => Err(error),
            None => Ok(CurveBoxContainment2::Contained),
        }
    }
}

const SIDES: [(Axis2, BoxBoundary2); 4] = [
    (Axis2::X, BoxBoundary2::Minimum),
    (Axis2::X, BoxBoundary2::Maximum),
    (Axis2::Y, BoxBoundary2::Minimum),
    (Axis2::Y, BoxBoundary2::Maximum),
];

/// Sides of the envelope not yet proved to contain the curve, in `SIDES` order.
type OpenSides = [bool; 4];

struct EscapeSearch<'a> {
    curve_index: usize,
    envelope: &'a Aabb2,
    policy: &'a CurveContext,
    undecided: Option<UncertaintyReason>,
}

impl EscapeSearch<'_> {
    /// Whether `value` lies on the inner side of `side`, or `None` if undecided.
    fn inside(&self, side: usize, value: &crate::Real) -> Option<bool> {
        let (axis, boundary) = SIDES[side];
        let bound = coordinate(
            match boundary {
                BoxBoundary2::Minimum => self.envelope.min(),
                BoxBoundary2::Maximum => self.envelope.max(),
            },
            axis,
        );
        let ordering = compare_reals(value, bound, self.policy)?;
        Some(match boundary {
            BoxBoundary2::Minimum => ordering != Ordering::Less,
            BoxBoundary2::Maximum => ordering != Ordering::Greater,
        })
    }

    /// Sides of `open` on which `bounds` is not proved inside the envelope.
    fn open_sides(&self, bounds: &Aabb2, open: &OpenSides) -> OpenSides {
        let mut result = [false; 4];
        for (side, &(axis, boundary)) in SIDES.iter().enumerate() {
            let corner = match boundary {
                BoxBoundary2::Minimum => bounds.min(),
                BoxBoundary2::Maximum => bounds.max(),
            };
            result[side] = open[side] && self.inside(side, coordinate(corner, axis)) != Some(true);
        }
        result
    }

    /// Tests one exact curve point against the open sides.
    fn test(&mut self, point: &Point2, open: &OpenSides) -> Option<CurveBoxEscape2> {
        for (side, &(axis, boundary)) in SIDES.iter().enumerate() {
            if !open[side] {
                continue;
            }
            match self.inside(side, coordinate(point, axis)) {
                Some(true) => {}
                Some(false) => {
                    return Some(CurveBoxEscape2 {
                        curve_index: self.curve_index,
                        axis,
                        boundary,
                        point: CurvePoint2::from(point.clone()),
                    });
                }
                None => self.undecide(UncertaintyReason::Ordering),
            }
        }
        None
    }

    /// Tests one retained curve endpoint, which need not have stored coordinates.
    fn test_endpoint(
        &mut self,
        point: CurvePoint2,
        open: &OpenSides,
    ) -> crate::CurveResult<Option<CurveBoxEscape2>> {
        if let Some(coordinates) = point.coordinates() {
            return Ok(self.test(coordinates, open));
        }
        for (side, &(axis, boundary)) in SIDES.iter().enumerate() {
            if !open[side] {
                continue;
            }
            let corner = CurvePoint2::from(match boundary {
                BoxBoundary2::Minimum => self.envelope.min().clone(),
                BoxBoundary2::Maximum => self.envelope.max().clone(),
            });
            match point
                .compare_coordinate_with_policy(&corner, axis, self.policy)?
                .into_value()
            {
                Classification::Decided(ordering) => {
                    let outside = match boundary {
                        BoxBoundary2::Minimum => ordering == Ordering::Less,
                        BoxBoundary2::Maximum => ordering == Ordering::Greater,
                    };
                    if outside {
                        return Ok(Some(CurveBoxEscape2 {
                            curve_index: self.curve_index,
                            axis,
                            boundary,
                            point,
                        }));
                    }
                }
                Classification::Uncertain(reason) => self.undecide(reason),
            }
        }
        Ok(None)
    }

    fn undecide(&mut self, reason: UncertaintyReason) {
        self.undecided.get_or_insert(reason);
    }
}

/// Returns a witness outside `envelope`, `None` when `curve` is contained, or
/// a typed blocker.
fn curve_box_escape(
    curve: &Curve2,
    curve_index: usize,
    envelope: &Aabb2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveBoxEscape2>> {
    let mut search = EscapeSearch {
        curve_index,
        envelope,
        policy,
        undecided: None,
    };
    // An undecided enclosure leaves every side to the exact candidates.
    let open = match curve.bounds() {
        Ok(bounds) => search.open_sides(bounds, &[true; 4]),
        Err(ExactCurveError::Blocked(_)) => [true; 4],
        Err(error) => return Err(error),
    };
    if !open.contains(&true) {
        return Ok(None);
    }
    let escape = match curve.geometry() {
        Some(geometry) => {
            if let Some(escape) = search
                .test(geometry.start(), &open)
                .or_else(|| search.test(geometry.end(), &open))
            {
                return Ok(Some(escape));
            }
            match geometry {
                CurveGeometry2::Line(_) => None,
                CurveGeometry2::CircularArc(arc) => {
                    arc_escape(arc, &open, &mut search).map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Classification,
                            curve.family(),
                            cause,
                        )
                    })?
                }
                _ => {
                    let fragments = curve.native_bezier_fragments_for_operation(
                        policy,
                        CurveOperation2::Classification,
                    )?;
                    fragments_escape(fragments, &open, &mut search).map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Classification,
                            curve.family(),
                            cause,
                        )
                    })?
                }
            }
        }
        None => {
            let mut escape = None;
            for endpoint in [curve.start(), curve.end()] {
                escape = search.test_endpoint(endpoint, &open).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Classification, curve.family(), cause)
                })?;
                if escape.is_some() {
                    break;
                }
            }
            if escape.is_none() {
                if let Some(spans) =
                    curve.restricted_source_spans(policy, CurveOperation2::Classification)?
                {
                    for span in spans {
                        escape = retained_fragment_escape(&span.fragment, true, &open, &mut search)
                            .map_err(|cause| {
                                ExactCurveError::invalid(
                                    CurveOperation2::Classification,
                                    curve.family(),
                                    cause,
                                )
                            })?;
                        if escape.is_some() {
                            break;
                        }
                    }
                } else if let Some(fragment) = curve.retained_fragment() {
                    // The top-level endpoints were already tested above.
                    escape = retained_fragment_escape(fragment, false, &open, &mut search)
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Classification,
                                curve.family(),
                                cause,
                            )
                        })?;
                } else {
                    search.undecide(UncertaintyReason::Unsupported);
                }
            }
            escape
        }
    };
    match (escape, search.undecided) {
        (Some(escape), _) => Ok(Some(escape)),
        (None, Some(reason)) => Err(ExactCurveError::blocked(
            CurveOperation2::Classification,
            curve.family(),
            reason,
        )),
        (None, None) => Ok(None),
    }
}

/// Tests the cardinal points that lie in the arc's sweep; the endpoints were
/// tested by the caller.
fn arc_escape(
    arc: &CircularArc2,
    open: &OpenSides,
    search: &mut EscapeSearch<'_>,
) -> crate::CurveResult<Option<CurveBoxEscape2>> {
    let radius = arc.radius_squared().sqrt()?;
    let center = arc.center();
    for (side, &(axis, boundary)) in SIDES.iter().enumerate() {
        if !open[side] {
            continue;
        }
        let offset = match boundary {
            BoxBoundary2::Minimum => -radius.clone(),
            BoxBoundary2::Maximum => radius.clone(),
        };
        let cardinal = match axis {
            Axis2::X => Point2::new(center.x() + &offset, center.y().clone()),
            Axis2::Y => Point2::new(center.x().clone(), center.y() + &offset),
        };
        match arc.contains_sweep_point_with_policy(&cardinal, search.policy) {
            Classification::Decided(true) => {
                let mut single = [false; 4];
                single[side] = true;
                if let Some(escape) = search.test(&cardinal, &single) {
                    return Ok(Some(escape));
                }
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => search.undecide(reason),
        }
    }
    Ok(None)
}

/// Tests interior fragment boundaries and coordinate extrema; the curve
/// endpoints were tested by the caller.
fn fragments_escape(
    fragments: &[NativeBezierFragment2],
    open: &OpenSides,
    search: &mut EscapeSearch<'_>,
) -> crate::CurveResult<Option<CurveBoxEscape2>> {
    let last = fragments.len().saturating_sub(1);
    for (index, fragment) in fragments.iter().enumerate() {
        let span = fragment.native_curve();
        // One span's enclosure is the curve's, which already crossed the box.
        let open = if fragments.len() > 1 {
            match span_bounds(span) {
                Classification::Decided(bounds) => search.open_sides(&bounds, open),
                Classification::Uncertain(_) => *open,
            }
        } else {
            *open
        };
        if !open.contains(&true) {
            continue;
        }
        if index < last
            && let Some(escape) = search.test(span.end(), &open)
        {
            return Ok(Some(escape));
        }
        for axis in [Axis2::X, Axis2::Y] {
            let mut axis_open = open;
            for (side, &(side_axis, _)) in SIDES.iter().enumerate() {
                axis_open[side] &= side_axis == axis;
            }
            if !axis_open.contains(&true) {
                continue;
            }
            match span_axis_extrema(span, axis, None, search.policy)? {
                Classification::Decided(points) => {
                    for point in points {
                        if let Some(escape) = search.test_endpoint(point, &axis_open)? {
                            return Ok(Some(escape));
                        }
                    }
                }
                Classification::Uncertain(reason) => search.undecide(reason),
            }
        }
    }
    Ok(None)
}

/// Retained boundaries keep their source chart. Only a stationary parameter
/// certified inside that chart's active range can be an interior witness.
fn retained_fragment_escape(
    fragment: &crate::BezierSplitFragment2,
    test_endpoints: bool,
    open: &OpenSides,
    search: &mut EscapeSearch<'_>,
) -> crate::CurveResult<Option<CurveBoxEscape2>> {
    if test_endpoints {
        let curve = Curve2::from_retained_fragment(fragment.clone());
        for endpoint in [curve.start(), curve.end()] {
            if let Some(escape) = search.test_endpoint(endpoint, open)? {
                return Ok(Some(escape));
            }
        }
    }
    let (span, range) = match fragment {
        crate::BezierSplitFragment2::Materialized { curve, .. } => (curve.clone(), None),
        crate::BezierSplitFragment2::RetainedBezier {
            source_curve,
            start,
            end,
            ..
        } => (
            source_curve.clone(),
            Some(crate::CurveParameterRange2::new_validated(
                start.clone().into(),
                end.clone().into(),
            )),
        ),
        crate::BezierSplitFragment2::SelectedFiber(fragment) => {
            let Some(source) = fragment.rational_curve() else {
                search.undecide(UncertaintyReason::Unsupported);
                return Ok(None);
            };
            (
                BezierSubcurve2::Rational(source.clone()),
                Some(fragment.range().clone()),
            )
        }
        _ => {
            search.undecide(UncertaintyReason::Unsupported);
            return Ok(None);
        }
    };
    for axis in [Axis2::X, Axis2::Y] {
        let mut axis_open = *open;
        for (side, &(side_axis, _)) in SIDES.iter().enumerate() {
            axis_open[side] &= side_axis == axis;
        }
        if !axis_open.contains(&true) {
            continue;
        }
        match span_axis_extrema(&span, axis, range.as_ref(), search.policy)? {
            Classification::Decided(points) => {
                for point in points {
                    if let Some(escape) = search.test_endpoint(point, &axis_open)? {
                        return Ok(Some(escape));
                    }
                }
            }
            Classification::Uncertain(reason) => search.undecide(reason),
        }
    }
    Ok(None)
}

fn span_bounds(span: &BezierSubcurve2) -> Classification<Aabb2> {
    match span {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(),
    }
}

/// Exact span points, other than its endpoints, where `axis` can attain an
/// extremum: the points at the roots of the coordinate's derivative.
fn span_axis_extrema(
    span: &BezierSubcurve2,
    axis: Axis2,
    range: Option<&crate::CurveParameterRange2>,
    policy: &CurveContext,
) -> crate::CurveResult<Classification<Vec<CurvePoint2>>> {
    if let Some(range) = range {
        // These derivative enumerators cover the authored unit chart.
        // An extension beyond it needs its own complete root interval.
        match crate::bezier_split::CurveParameterDomain2::new(
            &crate::CurveParameterRange2::unit(),
            None,
        )
        .contains_finite_range(range, policy)?
        {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    let roots = match span {
        BezierSubcurve2::Quadratic(curve) => {
            curve.axis_monotone_parameters_with_policy(axis, policy)
        }
        BezierSubcurve2::Cubic(curve) => curve.axis_monotone_parameters_with_policy(axis, policy),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.axis_monotone_parameters_with_policy(axis, policy)
        }
        BezierSubcurve2::Rational(curve) => {
            let unit = crate::CurveParameterRange2::unit();
            let parameters = match curve.axis_extremum_parameters_classified(
                axis,
                range.unwrap_or(&unit),
                policy,
            )? {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if parameters.is_empty() {
                return Ok(Classification::Decided(Vec::new()));
            }
            // The owner proved the denominator on the active range. Only
            // candidates certified in that range may become deferred images.
            let mut points = Vec::with_capacity(parameters.len());
            for parameter in parameters {
                match extremum_in_range(&parameter, range, policy)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match parameter {
                    crate::BezierParameter2::Exact(parameter) => {
                        match curve.point_at_classified(&parameter, policy) {
                            Classification::Decided(point) => points.push(point.into()),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    crate::BezierParameter2::Algebraic(parameter) => {
                        points.push(
                            crate::RationalBezierAlgebraicPointImage2::from_parametric_source(
                                curve.clone(),
                                parameter,
                                policy,
                            )
                            .into(),
                        );
                    }
                }
            }
            return Ok(Classification::Decided(points));
        }
    };
    let roots = match roots {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut points = Vec::with_capacity(roots.len());
    for root in roots {
        match extremum_in_range(&crate::BezierParameter2::Exact(root.clone()), range, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => continue,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        match span_point_at(span, root, policy) {
            Classification::Decided(point) => points.push(point.into()),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(points))
}

fn extremum_in_range(
    parameter: &crate::BezierParameter2,
    range: Option<&crate::CurveParameterRange2>,
    policy: &CurveContext,
) -> crate::CurveResult<Classification<bool>> {
    let Some(range) = range else {
        return Ok(Classification::Decided(true));
    };
    let parameter = crate::CurveParameter2::from(parameter.clone());
    for (first, second) in [(range.start(), &parameter), (&parameter, range.end())] {
        match first.cmp_by_refinement(second, policy)? {
            Classification::Decided(Ordering::Greater) => {
                return Ok(Classification::Decided(false));
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(true))
}

fn span_point_at(
    span: &BezierSubcurve2,
    parameter: crate::Real,
    policy: &CurveContext,
) -> Classification<Point2> {
    match span {
        BezierSubcurve2::Quadratic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::Cubic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::RationalQuadratic(curve) => curve.point_at_with_policy(parameter, policy),
        BezierSubcurve2::Rational(curve) => curve.point_at_classified(&parameter, policy),
    }
}

fn coordinate(point: &Point2, axis: Axis2) -> &crate::Real {
    match axis {
        Axis2::X => point.x(),
        Axis2::Y => point.y(),
    }
}
