//! Polynomial sections retain generated chords' point-based parameters.

use super::*;
use crate::{Axis2, BezierAlgebraicChord2, BezierSplitFragment2, LineSeg2, Point2};

#[derive(Clone, Debug)]
pub(crate) struct PolynomialChordCorrespondence2 {
    branch: Curve2,
    chord: BezierAlgebraicChord2,
    source_x: [Real; 3],
    chord_first: bool,
    first_range: CurveParameterRange2,
    second_range: CurveParameterRange2,
}

impl PolynomialChordCorrespondence2 {
    pub(crate) fn map_parameter(
        &self,
        parameter: &CurveParameter2,
        forward: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        if forward == self.chord_first {
            let point = parameter
                .as_algebraic_chord()
                .ok_or(CurveError::InvalidCurveParameter)?
                .point();
            return Ok(crate::bezier_offset::point_affine_form_parameter(
                point,
                &self.source_x,
                policy,
            )?
            .map(Some));
        }
        let point = match self.branch.point_at_with_policy(parameter, policy) {
            Ok(point) => point.into_value(),
            Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
        };
        self.chord
            .parameter_at_certified_support_point(point, policy)
            .map(|parameter| {
                Classification::Decided(Some(CurveParameter2::from_algebraic_chord(parameter)))
            })
    }

    pub(crate) fn clipped_ranges(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        crate::bezier_split::clip_corresponding_parameter_ranges(
            &self.first_range,
            &self.second_range,
            first,
            second,
            policy,
            |parameter| self.map_parameter(parameter, true, policy),
            |parameter| self.map_parameter(parameter, false, policy),
        )
    }
}

fn coordinate(
    point: &CurvePoint2,
    form: &[Real; 3],
    policy: &CurveContext,
) -> ExactCurveResult<CurveParameter2> {
    decided(crate::bezier_offset::point_affine_form_parameter(
        point, form, policy,
    ))
}

fn order(
    first: &CurveParameter2,
    second: &CurveParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    decided(first.cmp_by_refinement(second, policy))
}

fn invalid(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Intersection,
        CurveFamily2::ImplicitPolynomial,
        cause,
    )
}

fn source_forms(
    branch: &crate::PolynomialCurveBranch2,
) -> ExactCurveResult<Option<[[Real; 3]; 2]>> {
    if !branch.rational_maps().is_empty() {
        return Ok(None);
    }
    let Some(image) = branch.image_transform() else {
        return Ok(Some([
            [Real::one(), Real::zero(), Real::zero()],
            [Real::zero(), Real::one(), Real::zero()],
        ]));
    };
    let inverse = image.inverse().map_err(invalid)?;
    let (a, b, d, e, x, y) = inverse.affine_components();
    if [a, b, d, e, x, y]
        .iter()
        .any(|v| v.exact_rational_ref().is_none())
    {
        return Ok(None);
    }
    Ok(Some([
        [a.clone(), b.clone(), x.clone()],
        [d.clone(), e.clone(), y.clone()],
    ]))
}

/// A native support view does not replace this chord's selected endpoints.
fn supporting_line(
    chord: &BezierAlgebraicChord2,
    branch: &crate::PolynomialCurveBranch2,
    source_x: &[Real; 3],
    source_y: &[Real; 3],
    policy: &CurveContext,
) -> ExactCurveResult<Option<LineSeg2>> {
    chord.validate_policy(policy).map_err(invalid)?;
    if let Some(mut line) = chord
        .exact_line()
        .or_else(|| chord.retained_support().exact_line())
    {
        let (dx, dy) = line.delta();
        let (axis, delta) =
            if order(&dx.clone().into(), &Real::zero().into(), policy)? != Ordering::Equal {
                (Axis2::X, dx)
            } else {
                (Axis2::Y, dy)
            };
        let sign = decided(chord.tangent_axis_sign(axis, policy))?;
        let positive = order(&delta.into(), &Real::zero().into(), policy)? == Ordering::Greater;
        if positive != (sign == hyperreal::RealSign::Positive) {
            line = line.reversed();
        }
        return Ok(Some(line));
    }
    // A represented source abscissa can certify an entire vertical support
    // while both ordinates remain selected fibers. No ordinate is projected.
    let x_form = [Real::one(), Real::zero(), Real::zero()];
    let x = coordinate(chord.start(), &x_form, policy)?;
    if let Some(value) = x.scalar()
        && order(&x, &coordinate(chord.end(), &x_form, policy)?, policy)? == Ordering::Equal
    {
        let sign = decided(chord.tangent_axis_sign(Axis2::Y, policy))?;
        let dy = match sign {
            hyperreal::RealSign::Positive => Real::one(),
            hyperreal::RealSign::Negative => -Real::one(),
            hyperreal::RealSign::Zero => return Err(invalid(CurveError::InvalidCurveParameter)),
        };
        return LineSeg2::try_new(
            Point2::new(value.clone(), Real::zero()),
            Point2::new(value.clone(), dy),
        )
        .map(Some)
        .map_err(invalid);
    }
    let Some(transform) = branch.image_transform() else {
        return Ok(None);
    };
    // The same certificate can live in the retained source chart. Pull back
    // the coordinate forms, then transport a support through the similarity;
    // do not materialize the selected endpoint ordinates in either plane.
    let x = coordinate(chord.start(), source_x, policy)?;
    let Some(value) = x.scalar() else {
        return Ok(None);
    };
    if order(&x, &coordinate(chord.end(), source_x, policy)?, policy)? != Ordering::Equal {
        return Ok(None);
    }
    let dy = match order(
        &coordinate(chord.start(), source_y, policy)?,
        &coordinate(chord.end(), source_y, policy)?,
        policy,
    )? {
        Ordering::Less => Real::one(),
        Ordering::Greater => -Real::one(),
        Ordering::Equal => return Err(invalid(CurveError::InvalidCurveParameter)),
    };
    let line = LineSeg2::try_new(
        Point2::new(value.clone(), Real::zero()),
        Point2::new(value.clone(), dy),
    )
    .map_err(invalid)?;
    line.transform_similarity(transform)
        .map(Some)
        .map_err(invalid)
}

fn contact(
    branch_parameter: CurveParameter2,
    chord_parameter: CurveParameter2,
    point: CurvePoint2,
    cross: Option<hyperreal::RealSign>,
    branch_first: bool,
) -> CurveIntersectionContact2 {
    let location = |local_parameter| CurveLocation2 {
        span_index: 0,
        span_range: CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
        local_parameter,
    };
    let (first, second) = if branch_first {
        (branch_parameter, chord_parameter)
    } else {
        (chord_parameter, branch_parameter)
    };
    CurveIntersectionContact2 {
        first: location(first),
        second: location(second),
        point,
        tangent_cross_sign: cross,
        certified_transverse: cross.is_some_and(|sign| sign != hyperreal::RealSign::Zero),
    }
}

fn shared(
    branch_curve: &Curve2,
    line_curve: &Curve2,
    chord: &BezierAlgebraicChord2,
    source_x: [Real; 3],
    branch_first: bool,
    policy: &CurveContext,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let (first_range, second_range) = if branch_first {
        (
            branch_curve.parameter_domain().clone(),
            line_curve.parameter_domain().clone(),
        )
    } else {
        (
            line_curve.parameter_domain().clone(),
            branch_curve.parameter_domain().clone(),
        )
    };
    let source = Arc::new(PolynomialChordCorrespondence2 {
        branch: branch_curve.clone(),
        chord: chord.clone(),
        source_x,
        chord_first: !branch_first,
        first_range: first_range.clone(),
        second_range: second_range.clone(),
    });
    let correspondence = CurveOverlapCorrespondence2::PolynomialChord(source.clone());
    if let Some((mut first, mut second)) =
        decided(correspondence.clipped_ranges(&first_range, &second_range, policy))?
    {
        if if branch_first {
            branch_curve.source_traversal_is_reversed()
        } else {
            line_curve.source_traversal_is_reversed()
        } {
            first = CurveParameterRange2::new_validated(first.end().clone(), first.start().clone());
            second =
                CurveParameterRange2::new_validated(second.end().clone(), second.start().clone());
        }
        let start = coordinate(chord.start(), &source.source_x, policy)?;
        let end = coordinate(chord.end(), &source.source_x, policy)?;
        let reversed = (order(&start, &end, policy)? == Ordering::Greater)
            ^ branch_curve.source_traversal_is_reversed()
            ^ line_curve.source_traversal_is_reversed();
        return Ok(publish(
            Vec::new(),
            vec![CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                first_range: first,
                second_range: second,
                orientation: if reversed {
                    CurveOverlapOrientation2::Reversed
                } else {
                    CurveOverlapOrientation2::Same
                },
                endpoint_inclusion: [true, true],
                parameter_correspondence: correspondence,
            }],
        ));
    }
    for parameter in [
        branch_curve.parameter_domain().start(),
        branch_curve.parameter_domain().end(),
    ] {
        for boundary in [
            line_curve.parameter_domain().start(),
            line_curve.parameter_domain().end(),
        ] {
            let Some(mapped) = decided(source.map_parameter(boundary, !branch_first, policy))?
            else {
                continue;
            };
            if order(parameter, &mapped, policy)? == Ordering::Equal {
                let point = branch_curve
                    .point_at_with_policy(parameter, policy)?
                    .into_value();
                return Ok(publish(
                    vec![contact(
                        parameter.clone(),
                        boundary.clone(),
                        point,
                        Some(hyperreal::RealSign::Zero),
                        branch_first,
                    )],
                    Vec::new(),
                ));
            }
        }
    }
    Ok(publish(Vec::new(), Vec::new()))
}

pub(super) fn intersect(
    branch_curve: &Curve2,
    line_curve: &Curve2,
    branch_first: bool,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionResult2>> {
    let source = line_curve
        .source_restriction()
        .map_or(line_curve, |(source, _, _)| source);
    let Some(BezierSplitFragment2::AlgebraicChord(chord)) = source.retained_fragment() else {
        return Ok(None);
    };
    let branch = branch_curve
        .polynomial_branch()
        .expect("polynomial section dispatch");
    let Some([source_x, source_y]) = source_forms(branch)? else {
        return Ok(None);
    };
    let strict = policy.strict_counterpart();
    let policy = &strict;
    let sections = if let Some(line) = supporting_line(chord, branch, &source_x, &source_y, policy)?
    {
        branch.line_sections(&line, policy)?
    } else if branch.has_constant_source_ordinate(policy)? {
        // Independence of x proves every selected source graph is a straight
        // support. Endpoint equality selects that constant branch; it is not
        // used as a substitute for the support proof.
        let expected = coordinate(&branch_curve.start(), &source_y, policy)?;
        let start = coordinate(chord.start(), &source_y, policy)?;
        let end = coordinate(chord.end(), &source_y, policy)?;
        if order(&start, &end, policy)? == Ordering::Equal {
            return if order(&start, &expected, policy)? == Ordering::Equal {
                shared(
                    branch_curve,
                    line_curve,
                    chord,
                    source_x,
                    branch_first,
                    policy,
                )
                .map(Some)
            } else {
                // Distinct selected constant ordinates give disjoint entire
                // supports, regardless of the finite interval overlap.
                Ok(Some(publish(Vec::new(), Vec::new())))
            };
        }
        return Ok(None);
    } else {
        return Ok(None);
    };
    let Some(sections) = sections else {
        return Ok(None);
    };
    let points = match sections {
        PolynomialLineSections2::SharedLine(_) => {
            return shared(
                branch_curve,
                line_curve,
                chord,
                source_x,
                branch_first,
                policy,
            )
            .map(Some);
        }
        PolynomialLineSections2::Isolated(points) => points,
    };
    let flip = (branch.is_reversed() != line_curve.source_traversal_is_reversed()) == branch_first;
    let mut contacts = Vec::new();
    for section in points {
        let Some(parameter) =
            decided(chord.parameter_at_certified_point(section.point.clone(), policy))?
        else {
            continue;
        };
        let parameter = CurveParameter2::from_algebraic_chord(parameter);
        let Some(parameter) = admit_line_parameter(line_curve, parameter, &section.point, policy)?
        else {
            continue;
        };
        contacts.push(contact(
            section.parameter,
            parameter,
            section.point,
            section
                .cross
                .map(|sign| if flip { negate(sign) } else { sign }),
            branch_first,
        ));
    }
    Ok(Some(publish(contacts, Vec::new())))
}
