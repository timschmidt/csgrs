//! Mixed exact arrangement cleanup for retained lines and rational quadratics.
//!
//! This module is the conic companion to the mixed line/quadratic-Bezier
//! scheduler. It promotes certified line/conic event witnesses into exact line
//! breakpoints and exact rational-quadratic breakpoints, then emits retained
//! homogeneous conic fragments. It does not construct planar cells or perform
//! boolean materialization; those are downstream responsibilities.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::{Real, RealExactSetFacts};
use hypersolve::AlgebraicRootRepresentation;

use crate::bezier::RationalQuadraticBezier;
use crate::bezier_arrangement::{
    HomogeneousPoint2, LineRationalQuadraticBezierIntersection,
    LineRationalQuadraticBezierIntersectionClass, LineRationalQuadraticBezierIntersectionReport,
    LineRationalQuadraticBezierInverseBoundarySource, LineRationalQuadraticBezierInverseRootDomain,
    LineRationalQuadraticBezierSupportOverlap,
    intersect_axis_aligned_line_rational_quadratic_bezier,
};
use crate::provenance::PathProvenance;
use crate::segment::{Axis, LinePathSegment};

/// Errors that prevent a trusted line/rational-quadratic split schedule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierArrangementError {
    /// A retained line segment is degenerate and cannot carry an ordered split set.
    DegenerateLine { line: usize },
    /// Exact comparison of line split parameters was undecidable.
    UndecidableLineOrder { line: usize },
    /// Exact comparison of conic split parameters was undecidable.
    UndecidableConicOrder { curve: usize },
    /// The same geometric point could not be de-duplicated exactly.
    UndecidablePointEquality,
}

/// Exact event between one retained line segment and one rational quadratic.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierArrangementEvent {
    /// Line segment index.
    pub line: usize,
    /// Rational quadratic index.
    pub curve: usize,
    /// Certified intersection class.
    pub class: LineRationalQuadraticBezierIntersectionClass,
    /// Raw exact line/conic predicate report.
    pub intersection: LineRationalQuadraticBezierIntersectionReport,
}

/// Retained same-support line/conic overlap candidate.
///
/// These candidates are copied from the predicate report whenever a rational
/// quadratic conic is certified to lie on an axis-aligned line support. They
/// are retained even when monotonicity is non-certified and the event remains
/// [`LineRationalQuadraticBezierIntersectionClass::Unknown`]. This keeps the
/// exact homogeneous support and hodograph-numerator evidence available to
/// later algebraic ordering work, following Yap, "Towards Exact Geometric
/// Computation" (1997), instead of collapsing nonmonotone conic overlaps into
/// a lossy sampled approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierSupportOverlapCandidate {
    /// Line segment index.
    pub line: usize,
    /// Rational quadratic conic index.
    pub curve: usize,
    /// Retained same-support overlap evidence.
    pub overlap: LineRationalQuadraticBezierSupportOverlap,
}

/// Certified domain status for a retained algebraic line/conic breakpoint candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicBreakpointDomain {
    /// Conic parameter and line boundary source are certified inside the retained pair domains.
    InsideLineAndCurve,
    /// The retained conic parameter is certified outside `[0, 1]`.
    OutsideConic,
    /// Exact interval comparison did not decide.
    Unknown,
}

/// Retained algebraic breakpoint candidate for a nonmonotone line/conic overlap boundary.
///
/// This is the mixed-scheduler counterpart to
/// [`crate::bezier_arrangement::LineRationalQuadraticBezierAlgebraicInverseRoot`].
/// The predicate layer retains represented roots of
/// `N_v(t) - value * W(t) == 0`; this scheduler attaches each finite root to
/// the exact line endpoint that induced the boundary value.
///
/// Following Yap, "Towards Exact Geometric Computation" (1997), these
/// candidates are exact replay evidence, not inserted topology. They remain
/// separate from [`RationalQuadraticBezierRealBreakpoint`] until a later
/// ordering/materialization pass can compare represented conic parameters and
/// split homogeneous rational quadratics without sampling. The homogeneous
/// rational equation follows Farouki, *Pythagorean Hodograph Curves* (2008),
/// while the represented root itself is isolated by the Sturm/Collins-Loos
/// univariate root discipline used by `hypersolve`.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierAlgebraicBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Rational quadratic conic index.
    pub curve: usize,
    /// Line endpoint that supplied the retained boundary value.
    pub boundary_source: LineRationalQuadraticBezierInverseBoundarySource,
    /// Exact varying-coordinate boundary value on the line support.
    pub boundary_value: Real,
    /// Exact point on the retained line support.
    pub point: Point2,
    /// Exact line parameter for the retained endpoint boundary (`0` or `1`).
    pub line_parameter: Real,
    /// Represented algebraic conic parameter.
    pub conic_parameter: AlgebraicRootRepresentation,
    /// Certified relation of the represented conic parameter to `[0, 1]`.
    pub conic_parameter_domain: LineRationalQuadraticBezierInverseRootDomain,
    /// Certified relation of the candidate to both source domains.
    pub domain: LineRationalQuadraticBezierAlgebraicBreakpointDomain,
}

/// Certified order relation between two represented conic breakpoint candidates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicBreakpointOrderClass {
    /// The left breakpoint parameter is certified before the right parameter.
    Before,
    /// The represented parameters are certified equal.
    Equal,
    /// The left breakpoint parameter is certified after the right parameter.
    After,
    /// The isolating intervals overlap or exact comparison did not decide.
    Unknown,
}

/// Pairwise ordering evidence for retained algebraic conic breakpoint candidates.
///
/// The order is certified only from exact root witnesses or from separated
/// Sturm isolating intervals. It is deliberately not used to mutate
/// [`LineRationalQuadraticBezierArrangementReport::conic_breakpoints`], because
/// represented conic parameters still need exact homogeneous subcurve
/// materialization before they can become topology. This is the same
/// object/predicate boundary advocated by Yap, "Towards Exact Geometric
/// Computation" (1997): report exact ordering evidence when available, and
/// keep uncertainty explicit. The isolating-interval comparison follows the
/// Sturm/Collins-Loos univariate root model used by `hypersolve`.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierAlgebraicBreakpointOrder {
    /// Rational quadratic conic index shared by both candidates.
    pub curve: usize,
    /// Index in [`LineRationalQuadraticBezierArrangementReport::algebraic_breakpoints`].
    pub left: usize,
    /// Index in [`LineRationalQuadraticBezierArrangementReport::algebraic_breakpoints`].
    pub right: usize,
    /// Certified order relation between the represented conic parameters.
    pub order: LineRationalQuadraticBezierAlgebraicBreakpointOrderClass,
}

/// Exact breakpoint on one arranged rational quadratic conic.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezierRealBreakpoint {
    /// Rational quadratic index.
    pub curve: usize,
    /// Exact source parameter in `[0, 1]`.
    pub parameter: Real,
    /// Exact affine point image at `parameter`.
    pub point: Point2,
}

/// Exact line breakpoint used by the mixed line/conic scheduler.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedConicLineArrangementBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Exact point on the retained line segment.
    pub point: Point2,
    /// Numerator of the retained parameter `dot(point-start, end-start) / |end-start|^2`.
    pub parameter_numerator: Real,
    /// Positive denominator of the retained line parameter.
    pub parameter_denominator: Real,
}

/// Exact retained line fragment induced by mixed line/conic events.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedConicLineArrangementFragment {
    /// Source line segment index.
    pub source_line: usize,
    /// Fragment start witness.
    pub start: MixedConicLineArrangementBreakpoint,
    /// Fragment end witness.
    pub end: MixedConicLineArrangementBreakpoint,
    /// Retained exact line fragment.
    pub segment: LinePathSegment,
}

/// Exact homogeneous rational-quadratic fragment induced by mixed line/conic events.
///
/// The fragment stores homogeneous Bernstein controls `(X, Y, W)` directly.
/// That is the exact object produced by rational de Casteljau restriction.
/// Keeping it homogeneous follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), and the rational-curve treatment in
/// Farouki, *Pythagorean Hodograph Curves* (2008): topology is certified before
/// any affine denominator division is trusted.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezierRealFragment {
    /// Source rational quadratic index.
    pub source_curve: usize,
    /// Fragment start witness.
    pub start: RationalQuadraticBezierRealBreakpoint,
    /// Fragment end witness.
    pub end: RationalQuadraticBezierRealBreakpoint,
    /// Homogeneous start control.
    pub start_control: HomogeneousPoint2,
    /// Homogeneous middle control.
    pub control: HomogeneousPoint2,
    /// Homogeneous end control.
    pub end_control: HomogeneousPoint2,
}

/// Cached exact facts for a mixed line/conic arrangement schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierArrangementFacts {
    /// Exact-set facts across retained line endpoints and conic controls/weights.
    pub input_exact: RealExactSetFacts,
    /// Exact-set facts across emitted line and homogeneous conic fragment controls.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the arrangement schedule.
    pub provenance: PathProvenance,
}

/// Retained mixed line/rational-quadratic arrangement schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierArrangementReport {
    /// Retained input line segments.
    pub lines: Vec<LinePathSegment>,
    /// Retained input rational quadratic conics.
    pub curves: Vec<RationalQuadraticBezier>,
    /// Certified or unknown pairwise events.
    pub events: Vec<LineRationalQuadraticBezierArrangementEvent>,
    /// Retained same-support conic overlap candidates.
    pub support_overlaps: Vec<LineRationalQuadraticBezierSupportOverlapCandidate>,
    /// Algebraic conic breakpoint candidates retained from nonmonotone overlap boundaries.
    pub algebraic_breakpoints: Vec<LineRationalQuadraticBezierAlgebraicBreakpoint>,
    /// Pairwise exact order evidence for retained algebraic conic breakpoints.
    pub algebraic_breakpoint_orders: Vec<LineRationalQuadraticBezierAlgebraicBreakpointOrder>,
    /// Sorted line breakpoints induced by line endpoints and certified events.
    pub line_breakpoints: Vec<Vec<MixedConicLineArrangementBreakpoint>>,
    /// Sorted conic breakpoints induced by endpoints and certified events.
    pub conic_breakpoints: Vec<Vec<RationalQuadraticBezierRealBreakpoint>>,
    /// Positive-length line fragments.
    pub line_fragments: Vec<MixedConicLineArrangementFragment>,
    /// Positive-length homogeneous conic fragments.
    pub conic_fragments: Vec<RationalQuadraticBezierRealFragment>,
    /// Cached exact facts for the retained schedule.
    pub facts: LineRationalQuadraticBezierArrangementFacts,
}

/// Arrange retained line segments against retained rational quadratic conics.
pub fn arrange_line_segments_with_rational_quadratic_beziers(
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
) -> Result<LineRationalQuadraticBezierArrangementReport, LineRationalQuadraticBezierArrangementError>
{
    arrange_line_segments_with_rational_quadratic_beziers_and_provenance(
        lines,
        curves,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange retained line segments against retained rational quadratics with provenance.
pub fn arrange_line_segments_with_rational_quadratic_beziers_and_provenance(
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineRationalQuadraticBezierArrangementReport, LineRationalQuadraticBezierArrangementError>
{
    reject_degenerate_lines(lines, policy)?;
    let mut line_breakpoints = seed_line_breakpoints(lines);
    let mut conic_breakpoints = seed_conic_breakpoints(curves);
    let mut events = Vec::new();
    let mut support_overlaps = Vec::new();
    let mut algebraic_breakpoints = Vec::new();

    for (line_index, line) in lines.iter().enumerate() {
        for (curve_index, curve) in curves.iter().enumerate() {
            let intersection =
                intersect_axis_aligned_line_rational_quadratic_bezier(line, curve, policy);
            if intersection.class != LineRationalQuadraticBezierIntersectionClass::Unknown {
                for event in &intersection.intersections {
                    insert_line_breakpoint(
                        &mut line_breakpoints[line_index],
                        line_index,
                        line,
                        event.point.clone(),
                        policy,
                    )?;
                    insert_conic_breakpoint(
                        &mut conic_breakpoints[curve_index],
                        curve_index,
                        event,
                        policy,
                    )?;
                }
            }
            if let Some(overlap) = &intersection.support_overlap {
                algebraic_breakpoints.extend(retained_algebraic_conic_breakpoints(
                    line_index,
                    curve_index,
                    overlap,
                ));
                support_overlaps.push(LineRationalQuadraticBezierSupportOverlapCandidate {
                    line: line_index,
                    curve: curve_index,
                    overlap: overlap.clone(),
                });
            }
            events.push(LineRationalQuadraticBezierArrangementEvent {
                line: line_index,
                curve: curve_index,
                class: intersection.class,
                intersection,
            });
        }
    }

    sort_and_dedup_line_breakpoints(&mut line_breakpoints, policy)?;
    sort_and_dedup_conic_breakpoints(&mut conic_breakpoints, policy)?;
    let algebraic_breakpoint_orders =
        algebraic_conic_breakpoint_orders(&algebraic_breakpoints, policy);
    let line_fragments = build_line_fragments(&line_breakpoints, policy)?;
    let conic_fragments = build_conic_fragments(&conic_breakpoints, curves, policy)?;
    let facts = LineRationalQuadraticBezierArrangementFacts {
        input_exact: input_exact_facts(lines, curves),
        fragment_exact: fragment_exact_facts(&line_fragments, &conic_fragments),
        provenance,
    };

    Ok(LineRationalQuadraticBezierArrangementReport {
        lines: lines.to_vec(),
        curves: curves.to_vec(),
        events,
        support_overlaps,
        algebraic_breakpoints,
        algebraic_breakpoint_orders,
        line_breakpoints,
        conic_breakpoints,
        line_fragments,
        conic_fragments,
        facts,
    })
}

fn reject_degenerate_lines(
    lines: &[LinePathSegment],
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for (index, line) in lines.iter().enumerate() {
        if line.facts().known_degenerate == Some(true)
            || compare_reals_with_policy(&line.length_squared(), &Real::zero(), policy).value()
                == Some(Ordering::Equal)
        {
            return Err(
                LineRationalQuadraticBezierArrangementError::DegenerateLine { line: index },
            );
        }
    }
    Ok(())
}

fn seed_line_breakpoints(
    lines: &[LinePathSegment],
) -> Vec<Vec<MixedConicLineArrangementBreakpoint>> {
    lines
        .iter()
        .enumerate()
        .map(|(line_index, line)| {
            vec![
                line_breakpoint(line_index, line, line.start().clone()),
                line_breakpoint(line_index, line, line.end().clone()),
            ]
        })
        .collect()
}

fn seed_conic_breakpoints(
    curves: &[RationalQuadraticBezier],
) -> Vec<Vec<RationalQuadraticBezierRealBreakpoint>> {
    curves
        .iter()
        .enumerate()
        .map(|(curve_index, curve)| {
            vec![
                RationalQuadraticBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::zero(),
                    point: curve.start().clone(),
                },
                RationalQuadraticBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::one(),
                    point: curve.end().clone(),
                },
            ]
        })
        .collect()
}

fn retained_algebraic_conic_breakpoints(
    line_index: usize,
    curve_index: usize,
    overlap: &LineRationalQuadraticBezierSupportOverlap,
) -> Vec<LineRationalQuadraticBezierAlgebraicBreakpoint> {
    let mut retained = Vec::new();
    for boundary in &overlap.inverse_boundary_roots {
        let point = point_from_axis(overlap.axis, overlap.fixed.clone(), boundary.value.clone());
        let line_parameter = match boundary.source {
            LineRationalQuadraticBezierInverseBoundarySource::SegmentStart => Real::zero(),
            LineRationalQuadraticBezierInverseBoundarySource::SegmentEnd => Real::one(),
        };
        for root in &boundary.roots {
            retained.push(LineRationalQuadraticBezierAlgebraicBreakpoint {
                line: line_index,
                curve: curve_index,
                boundary_source: boundary.source,
                boundary_value: boundary.value.clone(),
                point: point.clone(),
                line_parameter: line_parameter.clone(),
                conic_parameter: root.parameter.clone(),
                conic_parameter_domain: root.parameter_domain,
                domain: classify_algebraic_conic_breakpoint_domain(root.parameter_domain),
            });
        }
    }
    retained
}

fn classify_algebraic_conic_breakpoint_domain(
    conic_domain: LineRationalQuadraticBezierInverseRootDomain,
) -> LineRationalQuadraticBezierAlgebraicBreakpointDomain {
    match conic_domain {
        LineRationalQuadraticBezierInverseRootDomain::InsideUnitInterval => {
            LineRationalQuadraticBezierAlgebraicBreakpointDomain::InsideLineAndCurve
        }
        LineRationalQuadraticBezierInverseRootDomain::OutsideUnitInterval => {
            LineRationalQuadraticBezierAlgebraicBreakpointDomain::OutsideConic
        }
        LineRationalQuadraticBezierInverseRootDomain::Unknown => {
            LineRationalQuadraticBezierAlgebraicBreakpointDomain::Unknown
        }
    }
}

fn point_from_axis(axis: Axis, fixed: Real, varying: Real) -> Point2 {
    match axis {
        Axis::X => Point2::new(varying, fixed),
        Axis::Y => Point2::new(fixed, varying),
    }
}

fn algebraic_conic_breakpoint_orders(
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    policy: PredicatePolicy,
) -> Vec<LineRationalQuadraticBezierAlgebraicBreakpointOrder> {
    let mut orders = Vec::new();
    for left in 0..breakpoints.len() {
        for right in (left + 1)..breakpoints.len() {
            if breakpoints[left].curve != breakpoints[right].curve {
                continue;
            }
            orders.push(LineRationalQuadraticBezierAlgebraicBreakpointOrder {
                curve: breakpoints[left].curve,
                left,
                right,
                order: compare_algebraic_conic_parameters(
                    &breakpoints[left].conic_parameter,
                    &breakpoints[right].conic_parameter,
                    policy,
                ),
            });
        }
    }
    orders
}

fn compare_algebraic_conic_parameters(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> LineRationalQuadraticBezierAlgebraicBreakpointOrderClass {
    if let (Some(left_exact), Some(right_exact)) =
        (&left.interval.exact_root, &right.interval.exact_root)
    {
        return match compare_reals_with_policy(left_exact, right_exact, policy).value() {
            Some(Ordering::Less) => {
                LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before
            }
            Some(Ordering::Equal) => {
                LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Equal
            }
            Some(Ordering::Greater) => {
                LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After
            }
            None => LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown,
        };
    }
    match compare_reals_with_policy(&left.interval.upper, &right.interval.lower, policy).value() {
        Some(Ordering::Less) => {
            return LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before;
        }
        Some(Ordering::Equal | Ordering::Greater) | None => {}
    }
    match compare_reals_with_policy(&right.interval.upper, &left.interval.lower, policy).value() {
        Some(Ordering::Less) => LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After,
        Some(Ordering::Equal | Ordering::Greater) | None => {
            LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown
        }
    }
}

fn insert_line_breakpoint(
    breakpoints: &mut Vec<MixedConicLineArrangementBreakpoint>,
    line_index: usize,
    line: &LinePathSegment,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for existing in breakpoints.iter() {
        match point2_equal_with_policy(&existing.point, &point, policy).value() {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => {
                return Err(LineRationalQuadraticBezierArrangementError::UndecidablePointEquality);
            }
        }
    }
    breakpoints.push(line_breakpoint(line_index, line, point));
    Ok(())
}

fn insert_conic_breakpoint(
    breakpoints: &mut Vec<RationalQuadraticBezierRealBreakpoint>,
    curve_index: usize,
    event: &LineRationalQuadraticBezierIntersection,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for existing in breakpoints.iter() {
        match compare_reals_with_policy(&existing.parameter, &event.parameter, policy).value() {
            Some(Ordering::Equal) => return Ok(()),
            Some(Ordering::Less | Ordering::Greater) => {}
            None => {
                return Err(
                    LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
                        curve: curve_index,
                    },
                );
            }
        }
    }
    breakpoints.push(RationalQuadraticBezierRealBreakpoint {
        curve: curve_index,
        parameter: event.parameter.clone(),
        point: event.point.clone(),
    });
    Ok(())
}

fn line_breakpoint(
    line_index: usize,
    line: &LinePathSegment,
    point: Point2,
) -> MixedConicLineArrangementBreakpoint {
    let dx = line.end().x.clone() - line.start().x.clone();
    let dy = line.end().y.clone() - line.start().y.clone();
    let px = point.x.clone() - line.start().x.clone();
    let py = point.y.clone() - line.start().y.clone();
    let parameter_numerator = px * dx.clone() + py * dy.clone();
    let parameter_denominator = dx.clone() * dx + dy.clone() * dy;
    MixedConicLineArrangementBreakpoint {
        line: line_index,
        point,
        parameter_numerator,
        parameter_denominator,
    }
}

fn sort_and_dedup_line_breakpoints(
    breakpoints: &mut [Vec<MixedConicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for (line_index, points) in breakpoints.iter_mut().enumerate() {
        certify_line_orders(points, line_index, policy)?;
        points.sort_by(|left, right| {
            compare_line_parameters(left, right, policy)
                .expect("line breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<MixedConicLineArrangementBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match point2_equal_with_policy(&last.point, &point.point, policy).value() {
                    Some(true) => continue,
                    Some(false) => {}
                    None => {
                        return Err(
                            LineRationalQuadraticBezierArrangementError::UndecidablePointEquality,
                        );
                    }
                }
            }
            deduped.push(point);
        }
        *points = deduped;
    }
    Ok(())
}

fn certify_line_orders(
    points: &[MixedConicLineArrangementBreakpoint],
    line_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_line_parameters(&points[left], &points[right], policy).ok_or(
                LineRationalQuadraticBezierArrangementError::UndecidableLineOrder {
                    line: line_index,
                },
            )?;
        }
    }
    Ok(())
}

fn compare_line_parameters(
    left: &MixedConicLineArrangementBreakpoint,
    right: &MixedConicLineArrangementBreakpoint,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    compare_reals_with_policy(
        &(left.parameter_numerator.clone() * right.parameter_denominator.clone()),
        &(right.parameter_numerator.clone() * left.parameter_denominator.clone()),
        policy,
    )
    .value()
}

fn sort_and_dedup_conic_breakpoints(
    breakpoints: &mut [Vec<RationalQuadraticBezierRealBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for (curve_index, points) in breakpoints.iter_mut().enumerate() {
        certify_conic_orders(points, curve_index, policy)?;
        points.sort_by(|left, right| {
            compare_reals_with_policy(&left.parameter, &right.parameter, policy)
                .value()
                .expect("conic breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<RationalQuadraticBezierRealBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match compare_reals_with_policy(&last.parameter, &point.parameter, policy).value() {
                    Some(Ordering::Equal) => continue,
                    Some(Ordering::Less | Ordering::Greater) => {}
                    None => {
                        return Err(
                            LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
                                curve: curve_index,
                            },
                        );
                    }
                }
            }
            deduped.push(point);
        }
        *points = deduped;
    }
    Ok(())
}

fn certify_conic_orders(
    points: &[RationalQuadraticBezierRealBreakpoint],
    curve_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_reals_with_policy(&points[left].parameter, &points[right].parameter, policy)
                .value()
                .ok_or(
                    LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
                        curve: curve_index,
                    },
                )?;
        }
    }
    Ok(())
}

fn build_line_fragments(
    breakpoints: &[Vec<MixedConicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<MixedConicLineArrangementFragment>, LineRationalQuadraticBezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_line_parameters(&window[0], &window[1], policy) == Some(Ordering::Equal) {
                continue;
            }
            fragments.push(MixedConicLineArrangementFragment {
                source_line: window[0].line,
                start: window[0].clone(),
                end: window[1].clone(),
                segment: LinePathSegment::new(window[0].point.clone(), window[1].point.clone()),
            });
        }
    }
    Ok(fragments)
}

fn build_conic_fragments(
    breakpoints: &[Vec<RationalQuadraticBezierRealBreakpoint>],
    curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
) -> Result<Vec<RationalQuadraticBezierRealFragment>, LineRationalQuadraticBezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            match compare_reals_with_policy(&window[0].parameter, &window[1].parameter, policy)
                .value()
            {
                Some(Ordering::Equal) => continue,
                Some(Ordering::Less | Ordering::Greater) => {}
                None => {
                    return Err(
                        LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
                            curve: window[0].curve,
                        },
                    );
                }
            }
            let fragment =
                rational_quadratic_subcurve_real(&curves[window[0].curve], &window[0], &window[1]);
            fragments.push(fragment);
        }
    }
    Ok(fragments)
}

fn rational_quadratic_subcurve_real(
    curve: &RationalQuadraticBezier,
    start: &RationalQuadraticBezierRealBreakpoint,
    end: &RationalQuadraticBezierRealBreakpoint,
) -> RationalQuadraticBezierRealFragment {
    let start_control = homogeneous_eval_real(curve, &start.parameter);
    let end_control = homogeneous_eval_real(curve, &end.parameter);
    let delta = end.parameter.clone() - start.parameter.clone();
    let derivative = homogeneous_derivative_real(curve, &start.parameter);
    let half = Real::from(2);
    let control = HomogeneousPoint2 {
        x: start_control.x.clone()
            + (delta.clone() * derivative.x / half.clone()).expect("nonzero two"),
        y: start_control.y.clone()
            + (delta.clone() * derivative.y / half.clone()).expect("nonzero two"),
        w: start_control.w.clone() + (delta * derivative.w / half).expect("nonzero two"),
    };
    RationalQuadraticBezierRealFragment {
        source_curve: start.curve,
        start: start.clone(),
        end: end.clone(),
        start_control,
        control,
        end_control,
    }
}

fn homogeneous_eval_real(curve: &RationalQuadraticBezier, parameter: &Real) -> HomogeneousPoint2 {
    let one_minus_t = Real::one() - parameter.clone();
    let b0 = one_minus_t.clone() * one_minus_t.clone();
    let b1 = Real::from(2) * one_minus_t * parameter.clone();
    let b2 = parameter.clone() * parameter.clone();
    let weighted_b1 = b1 * curve.control_weight().clone();
    HomogeneousPoint2 {
        x: curve.start().x.clone() * b0.clone()
            + curve.control().x.clone() * weighted_b1.clone()
            + curve.end().x.clone() * b2.clone(),
        y: curve.start().y.clone() * b0.clone()
            + curve.control().y.clone() * weighted_b1.clone()
            + curve.end().y.clone() * b2.clone(),
        w: b0 + weighted_b1 + b2,
    }
}

fn homogeneous_derivative_real(
    curve: &RationalQuadraticBezier,
    parameter: &Real,
) -> HomogeneousPoint2 {
    let db0 = -Real::from(2) * (Real::one() - parameter.clone());
    let db1 = Real::from(2) * (Real::one() - Real::from(2) * parameter.clone());
    let db2 = Real::from(2) * parameter.clone();
    let weighted_db1 = db1 * curve.control_weight().clone();
    HomogeneousPoint2 {
        x: curve.start().x.clone() * db0.clone()
            + curve.control().x.clone() * weighted_db1.clone()
            + curve.end().x.clone() * db2.clone(),
        y: curve.start().y.clone() * db0.clone()
            + curve.control().y.clone() * weighted_db1.clone()
            + curve.end().y.clone() * db2.clone(),
        w: db0 + weighted_db1 + db2,
    }
}

fn input_exact_facts(
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
) -> RealExactSetFacts {
    let mut values = Vec::new();
    for line in lines {
        values.extend([
            &line.start().x,
            &line.start().y,
            &line.end().x,
            &line.end().y,
        ]);
    }
    for curve in curves {
        values.extend([
            &curve.start().x,
            &curve.start().y,
            &curve.control().x,
            &curve.control().y,
            &curve.end().x,
            &curve.end().y,
            curve.control_weight(),
        ]);
    }
    Real::exact_set_facts(values)
}

fn fragment_exact_facts(
    lines: &[MixedConicLineArrangementFragment],
    curves: &[RationalQuadraticBezierRealFragment],
) -> RealExactSetFacts {
    let mut values = Vec::new();
    for fragment in lines {
        values.extend([
            &fragment.segment.start().x,
            &fragment.segment.start().y,
            &fragment.segment.end().x,
            &fragment.segment.end().y,
        ]);
    }
    for fragment in curves {
        values.extend([
            &fragment.start_control.x,
            &fragment.start_control.y,
            &fragment.start_control.w,
            &fragment.control.x,
            &fragment.control.y,
            &fragment.control.w,
            &fragment.end_control.x,
            &fragment.end_control.y,
            &fragment.end_control.w,
        ]);
    }
    Real::exact_set_facts(values)
}
