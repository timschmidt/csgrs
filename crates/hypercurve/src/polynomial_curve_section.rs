//! Sections of retained polynomial graph branches by authored rational lines.
//!
//! A graph branch is parameterized by the first coordinate of its source
//! polynomial. The branch's own fiber authority selects every point it shares
//! with a line of that plane. The line parameter is an affine form of that
//! correlated point. Except on a horizontal line, where it is the abscissa's
//! own root, evaluating the line there replays the polynomial point instead of
//! building independent coordinates. Neither operand is promoted to a Bezier
//! carrier.

use std::cmp::Ordering;

use super::*;
use crate::CurveFamily2;
use crate::curve::PolynomialLineSections2;

/// An authored rational line, or a retained restriction of one, in the source
/// chart shared by its public parameter.
///
/// Only a native line evaluates an affine-form parameter by replaying its
/// source point, which the split of the line operand relies on. A degree-
/// elevated line image keeps its existing dispatch.
fn rational_line(curve: &Curve2) -> Option<&crate::LineSeg2> {
    let source = curve
        .source_restriction()
        .map_or(curve, |(source, _, _)| source);
    let Some(CurveGeometry2::Line(line)) = source.geometry() else {
        return None;
    };
    let (start, end) = (line.start(), line.end());
    [start.x(), start.y(), end.x(), end.y()]
        .iter()
        .all(|value| value.exact_rational_ref().is_some())
        .then_some(line)
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

fn publish(
    contacts: Vec<CurveIntersectionContact2>,
    overlaps: Vec<CurveIntersectionOverlap2>,
) -> CurveIntersectionResult2 {
    CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count: 1,
            contacts: contacts.into(),
            overlaps: overlaps.into(),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    }
}

/// Clips a caller-certified common supporting line in the original source
/// chart. Reuse the same correspondence authority as native line overlaps;
/// source abscissas and native parameters are never rebased on the trim.
fn shared_line_section(
    branch_curve: &Curve2,
    line_curve: &Curve2,
    branch_first: bool,
    abscissas: [Real; 2],
    chart: &CurveSpanRange2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let [x0, x1] = abscissas;
    let reversed_map = compare_reals(&x0, &x1, &policy.strict_counterpart()).ok_or_else(|| {
        ExactCurveError::blocked(
            CurveOperation2::Intersection,
            CurveFamily2::ImplicitPolynomial,
            UncertaintyReason::Ordering,
        )
    })? == Ordering::Greater;
    let correspondence = CurveOverlapCorrespondence2::affine(
        &ParamRange::new(x0.clone(), x1.clone()),
        &ParamRange::new(Real::zero(), Real::one()),
    );
    let ranges = decided(correspondence.clipped_ranges(
        branch_curve.parameter_domain(),
        line_curve.parameter_domain(),
        policy,
    ))?;
    if let Some((mut branch_range, mut line_range)) = ranges {
        let branch_reversed = branch_curve.source_traversal_is_reversed();
        let line_reversed = line_curve.source_traversal_is_reversed();
        let reverse_pair = if branch_first {
            branch_reversed
        } else {
            reversed_map != line_reversed
        };
        if reverse_pair {
            branch_range = CurveParameterRange2::new_validated(
                branch_range.end().clone(),
                branch_range.start().clone(),
            );
            line_range = CurveParameterRange2::new_validated(
                line_range.end().clone(),
                line_range.start().clone(),
            );
        }
        let (first_range, second_range, parameter_correspondence) = if branch_first {
            (branch_range, line_range, correspondence)
        } else {
            let CurveOverlapCorrespondence2::Rational { source, .. } = correspondence else {
                unreachable!("the certified affine constructor returns a rational correspondence")
            };
            (
                line_range,
                branch_range,
                CurveOverlapCorrespondence2::Rational {
                    source,
                    swapped: true,
                },
            )
        };
        return Ok(publish(
            Vec::new(),
            vec![CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                first_range,
                second_range,
                orientation: if reversed_map ^ branch_reversed ^ line_reversed {
                    CurveOverlapOrientation2::Reversed
                } else {
                    CurveOverlapOrientation2::Same
                },
                endpoint_inclusion: [true, true],
                parameter_correspondence,
            }],
        ));
    }

    // Positive-length clipping deliberately omits singleton intersections.
    // Two closed nonempty intervals meet at one point only when the lower
    // end of one equals the upper end of the other.
    let dx = &x1 - &x0;
    let source_start = decided(
        line_curve
            .parameter_domain()
            .start()
            .affine_image_unbounded(&dx, &x0, policy),
    )?;
    let source_end = decided(
        line_curve
            .parameter_domain()
            .end()
            .affine_image_unbounded(&dx, &x0, policy),
    )?;
    let line_source_range = CurveParameterRange2::new_validated(source_start, source_end);
    let [line_low, line_high] = decided(line_source_range.ordered_endpoints(policy))?;
    let [branch_low, branch_high] =
        decided(branch_curve.parameter_domain().ordered_endpoints(policy))?;
    for (branch_parameter, line_boundary) in [(branch_low, line_high), (branch_high, line_low)] {
        if decided(branch_parameter.cmp_by_refinement(line_boundary, policy))? != Ordering::Equal {
            continue;
        }
        let point = branch_curve
            .point_at_with_policy(branch_parameter, policy)?
            .into_value();
        let parameter = decided(correspondence.map_parameter(branch_parameter, true, policy))?
            .ok_or_else(|| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    CurveFamily2::Line,
                    CurveError::InvalidCurveParameter,
                )
            })?;
        let parameter =
            admit_line_parameter(line_curve, parameter, &point, policy)?.ok_or_else(|| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    CurveFamily2::Line,
                    CurveError::InvalidCurveParameter,
                )
            })?;
        let branch_location = CurveLocation2 {
            span_index: 0,
            span_range: CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
            local_parameter: branch_parameter.clone(),
        };
        let line_location = CurveLocation2 {
            span_index: 0,
            span_range: chart.clone(),
            local_parameter: parameter,
        };
        let (first, second) = if branch_first {
            (branch_location, line_location)
        } else {
            (line_location, branch_location)
        };
        return Ok(publish(
            vec![CurveIntersectionContact2 {
                first,
                second,
                point,
                certified_transverse: false,
                tangent_cross_sign: Some(hyperreal::RealSign::Zero),
            }],
            Vec::new(),
        ));
    }
    Ok(publish(Vec::new(), Vec::new()))
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

fn negate(sign: hyperreal::RealSign) -> hyperreal::RealSign {
    match sign {
        hyperreal::RealSign::Positive => hyperreal::RealSign::Negative,
        hyperreal::RealSign::Negative => hyperreal::RealSign::Positive,
        hyperreal::RealSign::Zero => hyperreal::RealSign::Zero,
    }
}

/// Intersects a polynomial graph branch, including a retained rational
/// similarity, with an authored rational line in either operand order.
///
/// Returns `None` for every other pair, which keeps its existing dispatch.
/// Both active domains are checked. The line parameter is `(y - y0) / dy`
/// of the branch point, or `(x - x0) / dx` for a horizontal line. A contact
/// is certified transverse only when the branch classifies a nonzero tangent
/// cross sign; a certified tangency publishes a zero sign. Reversing either
/// traversal or swapping the operands negates the sign.
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
    let Some(line) = rational_line(line_curve) else {
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
    let Some(sections) = branch
        .line_sections(line, policy)
        .map_err(|error| error.with_operation(CurveOperation2::Intersection))?
    else {
        return Ok(None);
    };
    let sections = match sections {
        PolynomialLineSections2::Isolated(points) => points,
        PolynomialLineSections2::SharedLine(abscissas) => {
            return shared_line_section(
                if branch_first { first } else { second },
                line_curve,
                branch_first,
                abscissas,
                &chart,
                policy,
            )
            .map(Some);
        }
    };
    let (dx, dy) = line.delta();
    let inverse = |delta: &Real| {
        (Real::one() / delta).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Intersection,
                line_curve.family(),
                cause.into(),
            )
        })
    };
    let form = if dy.definitely_zero() {
        let scale = inverse(&dx)?;
        let offset = -(line.start().x() * &scale);
        [scale, Real::zero(), offset]
    } else {
        let scale = inverse(&dy)?;
        let offset = -(line.start().y() * &scale);
        [Real::zero(), scale, offset]
    };
    // The branch reports the sign toward increasing abscissa, along the
    // line's source direction, with the branch first. Each reversed traversal
    // and the swapped operand order negates it.
    let flip = (branch.is_reversed() != line_curve.source_traversal_is_reversed()) == branch_first;
    let mut contacts = Vec::with_capacity(sections.len());
    for section in sections {
        let parameter = decided(crate::bezier_offset::point_affine_form_parameter(
            &section.point,
            &form,
            policy,
        ))?;
        let Some(parameter) = admit_line_parameter(line_curve, parameter, &section.point, policy)?
        else {
            continue;
        };
        let tangent_cross_sign = section
            .cross
            .map(|sign| if flip { negate(sign) } else { sign });
        let branch_location = CurveLocation2 {
            span_index: 0,
            span_range: CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
            local_parameter: section.parameter,
        };
        let line_location = CurveLocation2 {
            span_index: 0,
            span_range: chart.clone(),
            local_parameter: parameter,
        };
        let (first, second) = if branch_first {
            (branch_location, line_location)
        } else {
            (line_location, branch_location)
        };
        contacts.push(CurveIntersectionContact2 {
            first,
            second,
            point: section.point,
            certified_transverse: tangent_cross_sign
                .is_some_and(|sign| sign != hyperreal::RealSign::Zero),
            tangent_cross_sign,
        });
    }
    Ok(Some(publish(contacts, Vec::new())))
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
