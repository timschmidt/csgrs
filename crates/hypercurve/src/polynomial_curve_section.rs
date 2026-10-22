//! Sections of retained polynomial graph branches by authored vertical lines.
//!
//! A graph branch is parameterized by the first coordinate of its source
//! polynomial. In that plane, a vertical line `x = c` meets the branch at most
//! once, at the point selected by the branch's own fiber authority. The line
//! parameter is the affine form `(y - y0) / dy` of that correlated point, so
//! evaluating the line there replays the polynomial point instead of building
//! independent coordinates. Neither operand is promoted to a Bezier carrier.

use std::cmp::Ordering;

use super::*;
use crate::CurveFamily2;

/// An authored rational vertical line, or a retained restriction of one, in
/// the source chart shared by its public parameter.
///
/// Only a native line evaluates an affine-form parameter by replaying its
/// source point, which the split of the line operand relies on. A degree-
/// elevated line image keeps its existing dispatch.
fn rational_vertical_line(curve: &Curve2) -> Option<&crate::LineSeg2> {
    let source = curve
        .source_restriction()
        .map_or(curve, |(source, _, _)| source);
    let Some(CurveGeometry2::Line(line)) = source.geometry() else {
        return None;
    };
    let (start, end) = (line.start(), line.end());
    let [x0, x1] = [start.x(), end.x()].map(|value| value.exact_rational());
    let rational = [start.y(), end.y()]
        .iter()
        .all(|value| value.exact_rational_ref().is_some());
    (rational && x0.is_some() && x0 == x1).then_some(line)
}

fn decided<T>(value: CurveResult<Classification<T>>) -> ExactCurveResult<T> {
    let family = CurveFamily2::ImplicitPolynomial;
    match value
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Intersection, family, cause))?
    {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Intersection,
            family,
            reason,
        )),
    }
}

fn publish(contacts: Vec<CurveIntersectionContact2>) -> CurveIntersectionResult2 {
    CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count: 1,
            contacts: contacts.into(),
            overlaps: Arc::from([]),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    }
}

/// Admits a line parameter in the active domain of `curve`.
///
/// A boundary keeps its retained parameter. The line chart is injective, so
/// when an order is undecided, a proved equal endpoint point identifies the
/// boundary parameter. Any other undecided order is a typed blocker.
fn admit_line_parameter(
    curve: &Curve2,
    parameter: CurveParameter2,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveParameter2>> {
    let domain = curve.parameter_domain();
    let endpoints = if curve.source_traversal_is_reversed() {
        [curve.end(), curve.start()]
    } else {
        [curve.start(), curve.end()]
    };
    let mut uncertainty = None;
    for (boundary, endpoint, outside) in [
        (domain.start(), &endpoints[0], Ordering::Less),
        (domain.end(), &endpoints[1], Ordering::Greater),
    ] {
        let reason = match parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Intersection, curve.family(), cause)
            })? {
            Classification::Decided(Ordering::Equal) => return Ok(Some(boundary.clone())),
            Classification::Decided(order) if order == outside => return Ok(None),
            Classification::Decided(_) => continue,
            Classification::Uncertain(reason) => reason,
        };
        if point.same_point(endpoint, policy) == Classification::Decided(true) {
            return Ok(Some(boundary.clone()));
        }
        uncertainty.get_or_insert(reason);
    }
    match uncertainty {
        Some(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Intersection,
            curve.family(),
            reason,
        )),
        None => Ok(Some(parameter)),
    }
}

/// Intersects an unmapped polynomial graph branch with an authored rational
/// vertical line in either operand order.
///
/// Returns `None` for every other pair, which keeps its existing dispatch.
/// Both active domains are checked. A contact is certified transverse only at
/// a regular branch point; its tangent cross sign then follows from traversal
/// alone, because the branch tangent has nonzero source-x component and the
/// line tangent has none.
pub(super) fn polynomial_line_section(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionResult2>> {
    let (branch, line_curve, branch_first) =
        match (first.polynomial_branch(), second.polynomial_branch()) {
            (Some(branch), None) => (branch, second, true),
            (None, Some(branch)) => (branch, first, false),
            _ => return Ok(None),
        };
    if !branch.is_source_image() {
        return Ok(None);
    }
    let Some(line) = rational_vertical_line(line_curve) else {
        return Ok(None);
    };
    let spans = line_curve.source_spans(policy, CurveOperation2::Intersection)?;
    let [span] = &spans[..] else {
        return Ok(None);
    };
    let chart = span.chart();
    if chart.endpoints() != (&Real::zero(), &Real::one()) {
        // Transporting a selected parameter into another local chart would
        // drop the affine source that lets the line replay the branch point.
        return Ok(None);
    }
    let Some(section) = branch
        .vertical_section(line.start().x(), policy)
        .map_err(|error| error.with_operation(CurveOperation2::Intersection))?
    else {
        return Ok(Some(publish(Vec::new())));
    };
    let dy = line.end().y() - line.start().y();
    let scale = (Real::one() / &dy).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Intersection,
            line_curve.family(),
            cause.into(),
        )
    })?;
    let offset = -(line.start().y() * &scale);
    let parameter = decided(crate::bezier_offset::point_affine_form_parameter(
        &section.point,
        &[Real::zero(), scale, offset],
        policy,
    ))?;
    let Some(parameter) = admit_line_parameter(line_curve, parameter, &section.point, policy)?
    else {
        return Ok(Some(publish(Vec::new())));
    };
    let upward = dy
        .exact_rational_ref()
        .is_some_and(|value| value.is_positive());
    let line_y = upward != line_curve.source_traversal_is_reversed();
    let tangent_cross_sign = section.regular.then(|| {
        // cross((s, *), (0, t)) = s * t with the branch first; swapping negates.
        if (line_y != branch.is_reversed()) == branch_first {
            hyperreal::RealSign::Positive
        } else {
            hyperreal::RealSign::Negative
        }
    });
    let branch_location = CurveLocation2 {
        span_index: 0,
        span_range: CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
        local_parameter: section.parameter,
    };
    let line_location = CurveLocation2 {
        span_index: 0,
        span_range: chart,
        local_parameter: parameter,
    };
    let (first, second) = if branch_first {
        (branch_location, line_location)
    } else {
        (line_location, branch_location)
    };
    Ok(Some(publish(vec![CurveIntersectionContact2 {
        first,
        second,
        point: section.point,
        certified_transverse: section.regular,
        tangent_cross_sign,
    }])))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hypersolve::bivariate_topology::{
        BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
    };

    fn quintic() -> Curve2 {
        let topology = decompose_bivariate_curve(
            &hypersolve::BivariatePolynomial::new(vec![
                vec![5.into(), 5.into(), 0.into(), 0.into(), 0.into(), 1.into()],
                vec![(-1).into()],
            ]),
            &BivariateCurveDomain {
                first: [(-1).into(), 1.into()],
                second: [(-2).into(), 2.into()],
            },
            BivariateTopologyConfig::default(),
        )
        .unwrap();
        crate::PolynomialCurveSet2::from_topology(
            topology,
            hypersolve::AlgebraicFiberRootIsolationConfig::default(),
        )
        .unwrap()
        .curves()[0]
            .clone()
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    #[test]
    fn polynomial_cuts_stay_in_the_source_chart() {
        let source = quintic();
        let policy = CurveContext::STRICT;
        for curve in [source.clone(), source.reversed().unwrap()] {
            let cut = |cuts: &[(usize, Real)]| {
                split_curve(
                    &curve,
                    cuts.iter()
                        .map(|(span, value)| (*span, value.clone().into())),
                    &policy,
                )
            };
            let whole = std::slice::from_ref(&curve);
            assert_eq!(cut(&[]).unwrap(), whole);
            // Endpoint and repeated cuts do not create empty pieces.
            assert_eq!(cut(&[(0, (-1).into()), (0, 1.into())]).unwrap(), whole);
            let pieces = cut(&[(0, q(1, 2)), (0, q(-1, 2)), (0, q(1, 2)), (0, 1.into())]).unwrap();
            assert_eq!(pieces.len(), 3);
            let reversed = curve.polynomial_branch().unwrap().is_reversed();
            let bounds = [(-1).into(), q(-1, 2), q(1, 2), 1.into()];
            for (index, piece) in pieces.iter().enumerate() {
                let index = if reversed { 2 - index } else { index };
                let domain = piece.parameter_domain();
                assert_eq!(domain.start().scalar(), Some(&bounds[index]));
                assert_eq!(domain.end().scalar(), Some(&bounds[index + 1]));
                assert_eq!(piece.polynomial_branch().unwrap().is_reversed(), reversed);
            }
            for pair in pieces.windows(2) {
                assert!(pair[0].end().coincides_with(&pair[1].start()).unwrap());
            }
            // A span other than the graph chart, or a cut outside the active
            // range, is malformed rather than ignored.
            for malformed in [
                &[(1, Real::zero())][..],
                &[(0, Real::zero()), (0, 2.into())],
            ] {
                assert!(matches!(
                    cut(malformed),
                    Err(ExactCurveError::Invalid {
                        operation: CurveOperation2::Arrangement,
                        ..
                    })
                ));
            }
        }
    }
}
