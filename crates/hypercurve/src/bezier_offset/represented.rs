//! Exact represented coordinates, ratios, tensor images and circle
//! predicates built from selected algebraic roots.

use super::*;

pub(super) fn represented_similarity_point(
    point: &[AlgebraicRootRepresentation; 2],
    transform: &Similarity2,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let (a, b, d, e, xoff, yoff) = transform.affine_components();
    let x = represented_affine_coordinate(&[(&point[0], a), (&point[1], b)], xoff);
    let y = represented_affine_coordinate(&[(&point[0], d), (&point[1], e)], yoff);
    match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => Classification::Decided([x, y]),
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    }
}

pub(super) fn represented_similarity_vector(
    vector: &[AlgebraicRootRepresentation; 2],
    transform: &Similarity2,
    output_scale: &Real,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let (a, b, d, e, _, _) = transform.affine_components();
    let [a, b, d, e] = [a, b, d, e].map(|coefficient| coefficient * output_scale);
    let x = represented_affine_coordinate(&[(&vector[0], &a), (&vector[1], &b)], &Real::zero());
    let y = represented_affine_coordinate(&[(&vector[0], &d), (&vector[1], &e)], &Real::zero());
    match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => Classification::Decided([x, y]),
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    }
}

pub(super) fn represented_point_bounds_refined(
    x: &AlgebraicRootRepresentation,
    y: &AlgebraicRootRepresentation,
    refinement_steps: usize,
) -> Classification<Aabb2> {
    if !x.is_valid() || !y.is_valid() {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    let x = refined_represented_root(x, refinement_steps);
    let y = refined_represented_root(y, refinement_steps);
    Classification::Decided(Aabb2::new_unchecked(
        Point2::new(x.interval.lower, y.interval.lower),
        Point2::new(x.interval.upper, y.interval.upper),
    ))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn represented_tensor_circle_contact_location_parameter(
    unit_radial: &[DenseTensorPolynomial; 2],
    radial_retained: &[DenseTensorPolynomial; 2],
    radial_candidate: &[DenseTensorPolynomial; 2],
    common_denominator: &DenseTensorPolynomial,
    discriminant: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
    signed_radical: &AlgebraicRootRepresentation,
    signed_radius: &Real,
    turn: &Real,
    radius_squared: &Real,
) -> CurveResult<
    Classification<
        Option<(
            BezierAlgebraicCuspSemicircleContactLocation2,
            BezierParameter2,
        )>,
    >,
> {
    let dot = |first: &[DenseTensorPolynomial; 2], second: &[DenseTensorPolynomial; 2]| {
        first[0]
            .multiply(&second[0])?
            .add(&first[1].multiply(&second[1])?)
    };
    let cross = |first: &[DenseTensorPolynomial; 2], second: &[DenseTensorPolynomial; 2]| {
        first[0]
            .multiply(&second[1])?
            .subtract(&first[1].multiply(&second[0])?)
    };
    let Some((dot_retained, dot_candidate, cross_retained, cross_candidate)) = (|| {
        let dot_retained = dot(unit_radial, radial_retained)?.scale(signed_radius)?;
        let dot_candidate = dot(unit_radial, radial_candidate)?.scale(signed_radius)?;
        let cross_scale = signed_radius * turn;
        let cross_retained = cross(unit_radial, radial_retained)?.scale(&cross_scale)?;
        let cross_candidate = cross(unit_radial, radial_candidate)?.scale(&cross_scale)?;
        Some((dot_retained, dot_candidate, cross_retained, cross_candidate))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let rank = sources.len() + 1;
    let Some(zero) =
        DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(&Real::zero()))
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let dot_value = represented_tensor_nested_ratio(
        &dot_retained,
        &dot_candidate,
        common_denominator,
        &zero,
        discriminant,
        sources,
        signed_radical,
    );
    let cross_value = represented_tensor_nested_ratio(
        &cross_retained,
        &cross_candidate,
        common_denominator,
        &zero,
        discriminant,
        sources,
        signed_radical,
    );
    let (Classification::Decided(dot_value), Classification::Decided(cross_value)) =
        (dot_value, cross_value)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    let Some(cross_sign) = represented_strict_sign(&cross_value) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    if cross_sign == RealSign::Negative {
        return Ok(Classification::Decided(None));
    }
    if cross_sign == RealSign::Zero {
        let Some(dot_sign) = represented_strict_sign(&dot_value) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        return match dot_sign {
            RealSign::Positive => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::Start,
                BezierParameter2::Exact(Real::zero()),
            )))),
            RealSign::Negative => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::End,
                BezierParameter2::Exact(Real::one()),
            )))),
            RealSign::Zero => Err(CurveError::Topology(
                "a represented nonzero circle contact had zero diameter coordinates".into(),
            )),
        };
    }
    let radial_complement =
        match represented_affine_coordinate(&[(&dot_value, &Real::one())], radius_squared) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if represented_strict_sign(&radial_complement) != Some(RealSign::Positive) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some((parameter_denominator_retained, parameter_denominator_candidate)) = (|| {
        Some((
            common_denominator
                .scale(radius_squared)?
                .add(&dot_retained)?
                .add(&cross_retained)?,
            dot_candidate.add(&cross_candidate)?,
        ))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let parameter = match represented_tensor_nested_ratio(
        &cross_retained,
        &cross_candidate,
        &parameter_denominator_retained,
        &parameter_denominator_candidate,
        discriminant,
        sources,
        signed_radical,
    ) {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let parameter = match represented_strict_interior_bezier_parameter(&parameter)? {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(Some((
        BezierAlgebraicCuspSemicircleContactLocation2::Interior,
        parameter,
    ))))
}

/// Converts a root already proved to lie strictly between zero and one into a
/// Bezier parameter. A tensor image may initially isolate that root in a wider
/// rational interval; intersecting the isolator with the unit interval keeps
/// the same singleton root and avoids treating harmless bracket slack as an
/// invalid parameter.
pub(super) fn represented_strict_interior_bezier_parameter(
    representation: &AlgebraicRootRepresentation,
) -> CurveResult<Classification<BezierParameter2>> {
    if representation.exact_point_witness().is_some() {
        return BezierParameter2::from_algebraic_root_representation(
            representation,
            &CurveContext::STRICT,
        );
    }
    let mut clipped = representation.clone();
    let zero = Real::zero();
    let one = Real::one();
    match compare_reals(&clipped.interval.lower, &zero, &CurveContext::STRICT) {
        Some(std::cmp::Ordering::Less) => clipped.interval.lower = zero,
        Some(_) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
    }
    match compare_reals(&clipped.interval.upper, &one, &CurveContext::STRICT) {
        Some(std::cmp::Ordering::Greater) => clipped.interval.upper = one,
        Some(_) => {}
        None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
    }
    clipped.validation =
        validate_algebraic_root_representation(&clipped, hypersolve::PredicatePolicy::STRICT);
    if !clipped.is_valid() {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    BezierParameter2::from_algebraic_root_representation(&clipped, &CurveContext::STRICT)
}

pub(super) fn represented_circle_contact_location_parameter_from_dot_cross(
    dot: AlgebraicRootRepresentation,
    oriented_cross: AlgebraicRootRepresentation,
    radius_squared: &Real,
) -> CurveResult<
    Classification<
        Option<(
            BezierAlgebraicCuspSemicircleContactLocation2,
            BezierParameter2,
        )>,
    >,
> {
    let Some(cross_sign) = represented_strict_sign(&oriented_cross) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    if cross_sign == RealSign::Negative {
        return Ok(Classification::Decided(None));
    }
    if cross_sign == RealSign::Zero {
        let Some(dot_sign) = represented_strict_sign(&dot) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        return match dot_sign {
            RealSign::Positive => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::Start,
                BezierParameter2::Exact(Real::zero()),
            )))),
            RealSign::Negative => Ok(Classification::Decided(Some((
                BezierAlgebraicCuspSemicircleContactLocation2::End,
                BezierParameter2::Exact(Real::one()),
            )))),
            RealSign::Zero => Err(CurveError::Topology(
                "a represented nonzero circle contact had zero diameter coordinates".into(),
            )),
        };
    }
    let radial_complement =
        match represented_affine_coordinate(&[(&dot, &Real::one())], radius_squared) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if represented_strict_sign(&radial_complement) != Some(RealSign::Positive) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let denominator = match represented_affine_coordinate(
        &[(&dot, &Real::one()), (&oriented_cross, &Real::one())],
        radius_squared,
    ) {
        Classification::Decided(denominator) => denominator,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if represented_strict_sign(&denominator) != Some(RealSign::Positive) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let parameter = match represented_ratio(&oriented_cross, &denominator) {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let parameter = match represented_strict_interior_bezier_parameter(&parameter)? {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(Some((
        BezierAlgebraicCuspSemicircleContactLocation2::Interior,
        parameter,
    ))))
}

pub(super) fn represented_circle_diameter_predicate_sign(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    curve: &RationalBezier2,
    curve_parameter: &BezierParameter2,
    radial_coefficient: &Real,
    parameter_denominator: &Real,
    cache: &BezierAlgebraicCuspSemicircleParameterCache2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let dot = if let Some(dot) =
        cache.cached_represented_diameter_coordinate(curve_parameter, policy)
    {
        dot
    } else {
        let point = match rational_point_evidence_at_parameter(curve, curve_parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let point = match represented_point_evidence_coordinates(&point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let difference = |point: &AlgebraicRootRepresentation,
                          center: &AlgebraicRootRepresentation| {
            represented_affine_coordinate(
                &[(point, &Real::one()), (center, &Real::from(-1_i8))],
                &Real::zero(),
            )
        };
        let (Classification::Decided(dx), Classification::Decided(dy)) = (
            difference(&point[0], &frame.center[0]),
            difference(&point[1], &frame.center[1]),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let dot = match represented_vector_dot_cross(&frame.unit_radial, &[dx, dy]) {
            Classification::Decided([dot, _]) => dot,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        cache.retain_represented_diameter_coordinate(curve_parameter.clone(), dot.clone(), policy);
        dot
    };
    let diameter_scale = parameter_denominator * &frame.signed_radius;
    let radius_squared = &frame.signed_radius * &frame.signed_radius;
    let predicate = match represented_affine_coordinate(
        &[(&dot, &diameter_scale)],
        &(-radial_coefficient * radius_squared),
    ) {
        Classification::Decided(predicate) => predicate,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(represented_policy_sign(&predicate, policy))
}

/// Materializes dot and cross for two scaled unit circle radials while
/// retaining their circle correlation. Independently eliminated coordinates
/// can still certify an orthogonal signed permutation exactly. The
/// orientation-preserving permutations reduce to `(+-scale_product, 0)` or
/// `(0, +-scale_product)` without inventing independent coordinate sheets.
pub(super) fn represented_scaled_unit_radial_dot_cross(
    first: &[AlgebraicRootRepresentation; 2],
    second: &[AlgebraicRootRepresentation; 2],
    scale_product: &Real,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let direct_x = represented_zero_offset_unit_scales(&first[0], &second[0]);
    let direct_y = represented_zero_offset_unit_scales(&first[1], &second[1]);
    let zero = || AlgebraicRootRepresentation::from_exact_value(&Real::zero());
    if direct_x & POSITIVE_UNIT_SCALE != 0 && direct_y & POSITIVE_UNIT_SCALE != 0 {
        return Classification::Decided([
            AlgebraicRootRepresentation::from_exact_value(scale_product),
            zero(),
        ]);
    }
    if direct_x & NEGATIVE_UNIT_SCALE != 0 && direct_y & NEGATIVE_UNIT_SCALE != 0 {
        return Classification::Decided([
            AlgebraicRootRepresentation::from_exact_value(&(-scale_product)),
            zero(),
        ]);
    }

    let second_x_from_first_y = represented_zero_offset_unit_scales(&first[1], &second[0]);
    let second_y_from_first_x = represented_zero_offset_unit_scales(&first[0], &second[1]);
    if second_x_from_first_y & NEGATIVE_UNIT_SCALE != 0
        && second_y_from_first_x & POSITIVE_UNIT_SCALE != 0
    {
        return Classification::Decided([
            zero(),
            AlgebraicRootRepresentation::from_exact_value(scale_product),
        ]);
    }
    if second_x_from_first_y & POSITIVE_UNIT_SCALE != 0
        && second_y_from_first_x & NEGATIVE_UNIT_SCALE != 0
    {
        return Classification::Decided([
            zero(),
            AlgebraicRootRepresentation::from_exact_value(&(-scale_product)),
        ]);
    }
    match represented_vector_dot_cross(first, second) {
        Classification::Decided([dot, cross]) => {
            let dot = represented_affine_coordinate(&[(&dot, scale_product)], &Real::zero());
            let cross = represented_affine_coordinate(&[(&cross, scale_product)], &Real::zero());
            match (dot, cross) {
                (Classification::Decided(dot), Classification::Decided(cross)) => {
                    Classification::Decided([dot, cross])
                }
                (Classification::Uncertain(UncertaintyReason::Unsupported), _)
                | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                _ => Classification::Uncertain(UncertaintyReason::Predicate),
            }
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

pub(super) fn represented_circle_dot_cross_from_exact_radial(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    contact_radial: &[Real; 2],
    turn: &Real,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let x_scale = &frame.signed_radius * contact_radial[0].clone();
    let y_scale = &frame.signed_radius * contact_radial[1].clone();
    let dot = represented_affine_coordinate(
        &[
            (&frame.unit_radial[0], &x_scale),
            (&frame.unit_radial[1], &y_scale),
        ],
        &Real::zero(),
    );
    let cross_x_scale = &frame.signed_radius * contact_radial[1].clone();
    let cross_y_scale = -(&frame.signed_radius * contact_radial[0].clone());
    let cross = represented_affine_coordinate(
        &[
            (&frame.unit_radial[0], &cross_x_scale),
            (&frame.unit_radial[1], &cross_y_scale),
        ],
        &Real::zero(),
    );
    let (Classification::Decided(dot), Classification::Decided(cross)) = (dot, cross) else {
        return Classification::Uncertain(UncertaintyReason::Predicate);
    };
    let oriented_cross = match represented_affine_coordinate(&[(&cross, turn)], &Real::zero()) {
        Classification::Decided(cross) => cross,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    Classification::Decided([dot, oriented_cross])
}

pub(super) fn represented_exact_radial_linear_sign(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    x_scale: &Real,
    y_scale: &Real,
    offset: &Real,
) -> Option<RealSign> {
    let x_scale = x_scale * &frame.signed_radius;
    let y_scale = y_scale * &frame.signed_radius;
    for refinement_steps in [0, 2, 4, 8] {
        let x = refined_represented_root(&frame.unit_radial[0], refinement_steps);
        let y = refined_represented_root(&frame.unit_radial[1], refinement_steps);
        let interval = |source: &AlgebraicRootRepresentation, scale: &Real| {
            RealInterval {
                lower: source.interval.lower.clone(),
                upper: source.interval.upper.clone(),
            }
            .multiply(&RealInterval {
                lower: scale.clone(),
                upper: scale.clone(),
            })
        };
        let value = RealInterval {
            lower: offset.clone(),
            upper: offset.clone(),
        }
        .add(&interval(&x, &x_scale)?)
        .add(&interval(&y, &y_scale)?);
        if compare_reals(&value.upper, &Real::zero(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Some(RealSign::Negative);
        }
        if compare_reals(&value.lower, &Real::zero(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            return Some(RealSign::Positive);
        }
        if value.lower == Real::zero() && value.upper == Real::zero() {
            return Some(RealSign::Zero);
        }
    }
    None
}

pub(super) fn represented_circle_contact_location_from_exact_radial(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    contact_radial: &[Real; 2],
    radius_squared: &Real,
    turn: &Real,
) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
    let cross_x = &contact_radial[1] * turn;
    let cross_y = -(&contact_radial[0] * turn);
    match represented_exact_radial_linear_sign(frame, &cross_x, &cross_y, &Real::zero()) {
        Some(RealSign::Positive) => {
            return Ok(Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )));
        }
        Some(RealSign::Negative) => return Ok(Classification::Decided(None)),
        Some(RealSign::Zero) | None => {}
    }
    let [dot, cross] =
        match represented_circle_dot_cross_from_exact_radial(frame, contact_radial, turn) {
            Classification::Decided(values) => values,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    Ok(
        represented_circle_contact_location_parameter_from_dot_cross(dot, cross, radius_squared)?
            .map(|contact| contact.map(|(location, _)| location)),
    )
}
