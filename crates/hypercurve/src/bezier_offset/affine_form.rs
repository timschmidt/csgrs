//! Affine forms reuse retained point fields before Cartesian elimination.

use super::*;

/// Use the same checked source recipe at construction and cold replay.
/// A similarity can be pulled into an affine query without replacing either
/// selected coordinate or introducing an independent root projection.
pub(super) fn canonical_affine_form_definition(
    mut form: crate::CurvePointAffineForm2,
) -> crate::CurvePointAffineForm2 {
    loop {
        if let CurvePointData2::Similarity(image) = &form.source().0 {
            if !CurveContext::STRICT.accepts_retained_policy(image.data.policy) {
                return form;
            }
            let [a, b, c] = form.coefficients();
            let (m00, m01, m10, m11, tx, ty) = image.data.transform.affine_components();
            form = image.data.source.affine_form(
                Real::dot2_refs([a, b], [m00, m10]),
                Real::dot2_refs([a, b], [m01, m11]),
                Real::dot2_refs([a, b], [tx, ty]) + c,
            );
            continue;
        }
        // A scalar fiber image may itself be lifted to an affine point before
        // another line chart consumes it. Flatten that exact recipe at both
        // construction and replay; otherwise loading alone changes archives.
        let CurvePointData2::AnalyticParallel(point) = &form.source().0 else {
            return form;
        };
        let Some((parallel, parameter)) = point.native_parallel_evaluation() else {
            return form;
        };
        let Some(source) = parameter
            .as_selected_fiber()
            .and_then(|value| value.affine_form_source())
        else {
            return form;
        };
        if parallel.distance().zero_status() != ZeroKnowledge::Zero {
            return form;
        }
        let BezierParallelSource2::Rational(curve) = parallel.source() else {
            return form;
        };
        let Ok(basis) = curve.homogeneous_power_basis() else {
            return form;
        };
        let Ok(Some((scale, offset))) = affine_image_coefficients(
            &basis.x_numerator,
            &basis.y_numerator,
            &basis.weight,
            form.coefficients(),
        ) else {
            return form;
        };
        let [a, b, c] = source.coefficients();
        form = source
            .source()
            .affine_form(&scale * a, &scale * b, &scale * c + offset);
    }
}

fn affine_image_coefficients(
    x: &[Real],
    y: &[Real],
    w: &[Real],
    [a, b, c]: &[Real; 3],
) -> CurveResult<Option<(Real, Real)>> {
    let zero = Real::zero();
    if w.is_empty()
        || w.iter()
            .skip(1)
            .any(|v| v.zero_status() != ZeroKnowledge::Zero)
    {
        return Ok(None);
    }
    let coefficient = |i| {
        Real::dot2_refs(
            [a, b],
            [x.get(i).unwrap_or(&zero), y.get(i).unwrap_or(&zero)],
        )
    };
    if (2..x.len().max(y.len())).any(|i| coefficient(i).zero_status() != ZeroKnowledge::Zero) {
        return Ok(None);
    }
    Ok(Some((
        (coefficient(1) / &w[0])?,
        (coefficient(0) / &w[0])? + c,
    )))
}

pub(crate) fn point_affine_form_parameter(
    source: &CurvePoint2,
    coefficients: &[Real; 3],
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    if let Classification::Uncertain(reason) = source.validate_rational_image(policy)? {
        return Ok(Classification::Uncertain(reason));
    }
    let [a, b, c] = coefficients;
    if a.zero_status() == ZeroKnowledge::Zero && b.zero_status() == ZeroKnowledge::Zero {
        return Ok(Classification::Decided(c.clone().into()));
    }
    if let Some(point) = source.coordinates() {
        return Ok(Classification::Decided(
            (Real::dot2_refs([a, b], [point.x(), point.y()]) + c).into(),
        ));
    }
    // An affine image of one retained scalar already owns its parameter.
    // Recover that chart before creating a recursive coordinate expression.
    // This also keeps a constant coordinate authored, even when the other
    // coordinate depends on selected roots.
    let affine = |x: &[Real],
                  y: &[Real],
                  w: &[Real],
                  parameter: &CurveParameter2|
     -> CurveResult<Option<Classification<CurveParameter2>>> {
        let Some((scale, offset)) = affine_image_coefficients(x, y, w, coefficients)? else {
            return Ok(None);
        };
        if scale.zero_status() == ZeroKnowledge::Zero {
            return Ok(Some(Classification::Decided(offset.into())));
        }
        if scale == Real::one() && offset.zero_status() == ZeroKnowledge::Zero {
            return Ok(Some(Classification::Decided(parameter.clone())));
        }
        Ok(Some(
            parameter.affine_image_unbounded(&scale, &offset, policy)?,
        ))
    };
    match &source.0 {
        CurvePointData2::RationalImage(_) => {
            return point_rational_form_parameter(
                source,
                &BivariatePolynomial::new(vec![vec![c.clone(), b.clone()], vec![a.clone()]]),
                &BivariatePolynomial::new(vec![vec![Real::one()]]),
                policy,
            );
        }
        CurvePointData2::AlgebraicChordParallel(point) if point.uses_selected_field() => {
            return point_rational_form_parameter(
                source,
                &BivariatePolynomial::new(vec![vec![c.clone(), b.clone()], vec![a.clone()]]),
                &BivariatePolynomial::new(vec![vec![Real::one()]]),
                policy,
            );
        }
        CurvePointData2::Polynomial(point) => {
            return Ok(point
                .affine_parameter(&[a.clone(), b.clone(), c.clone()], policy)?
                .map(|parameter| {
                    parameter.with_selected_fiber_affine_form(source.affine_form(
                        a.clone(),
                        b.clone(),
                        c.clone(),
                    ))
                }));
        }
        CurvePointData2::Algebraic(point) => {
            if let Some((x, y, w)) = point.retained_coordinate_polynomials() {
                let parameter = match point.source_parameter(policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if let Some(result) = affine(x, y, w, &parameter.into())? {
                    return Ok(result);
                }
            }
        }
        CurvePointData2::AnalyticParallel(point) => {
            if let Some((parallel, parameter)) = point.native_parallel_evaluation()
                && parallel.distance().zero_status() == ZeroKnowledge::Zero
                && let BezierParallelSource2::Rational(curve) = parallel.source()
            {
                let basis = curve.homogeneous_power_basis()?;
                if let Some(result) = affine(
                    &basis.x_numerator,
                    &basis.y_numerator,
                    &basis.weight,
                    &parameter,
                )? {
                    return Ok(result);
                }
            }
        }
        CurvePointData2::Endpoint(endpoint) => {
            return match endpoint.resolve(policy)? {
                Classification::Decided(Some(point)) => {
                    point_affine_form_parameter(&point, &[a.clone(), b.clone(), c.clone()], policy)
                }
                Classification::Decided(None) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        CurvePointData2::Similarity(image) => {
            if !policy.accepts_retained_policy(image.data.policy) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let (m00, m01, m10, m11, tx, ty) = image.data.transform.affine_components();
            return point_affine_form_parameter(
                &image.data.source,
                &[
                    Real::dot2_refs([a, b], [m00, m10]),
                    Real::dot2_refs([a, b], [m01, m11]),
                    Real::dot2_refs([a, b], [tx, ty]) + c,
                ],
                policy,
            );
        }
        _ => {}
    }
    let points = match recursive_projective_evidence_points(&[source], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let point = match positive_recursive_projective_point(
        points
            .into_iter()
            .next()
            .expect("one source point supplies one projective image"),
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let Some(numerator) = point
        .x
        .scale(a)
        .and_then(|x| x.add(&point.y.scale(b)?))
        .and_then(|sum| sum.add(&point.denominator.scale(c)?))
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    // Chord-intersection charts retain both supporting-line authorities even
    // when their current endpoint fields happen to yield scalar expressions.
    // The chord chart constructor uses this same retained affine identity.
    // Promoting only during cold replay discards that identity and expands
    // every later curve/vertex occurrence into a different scalar expression.
    if !matches!(source.0, CurvePointData2::AlgebraicChordPair(_))
        && let (Some(numerator), Some(denominator)) = (
            numerator.exact_real_value_with_retained_witnesses(),
            point.denominator.exact_real_value_with_retained_witnesses(),
        )
    {
        return Ok(Classification::Decided((numerator / denominator)?.into()));
    }
    Ok(BezierRecursiveProjectiveParameter2::new(
        RecursiveQuadraticProjectiveScalar {
            numerator,
            denominator: point.denominator,
        },
        policy,
    )?
    .map(|parameter| {
        CurveParameter2::from_recursive_projective(
            parameter.with_affine_form_identity(source.affine_form(
                a.clone(),
                b.clone(),
                c.clone(),
            )),
        )
    }))
}

pub(crate) fn point_affine_form_order_to_real(
    point: &CurvePoint2,
    a: &Real,
    b: &Real,
    value: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    if let Classification::Uncertain(reason) = point.validate_rational_image(policy)? {
        return Ok(Classification::Uncertain(reason));
    }
    use std::cmp::Ordering;
    let order = |sign| match sign {
        RealSign::Negative => Ordering::Less,
        RealSign::Zero => Ordering::Equal,
        RealSign::Positive => Ordering::Greater,
    };
    if a.zero_status() == ZeroKnowledge::Zero && b.zero_status() == ZeroKnowledge::Zero {
        return Ok(compare_reals(&Real::zero(), value, policy).map_or(
            Classification::Uncertain(UncertaintyReason::Ordering),
            Classification::Decided,
        ));
    }
    match &point.0 {
        CurvePointData2::RationalImage(_) => point_rational_form_sign(
            point,
            &BivariatePolynomial::new(vec![vec![-value.clone(), b.clone()], vec![a.clone()]]),
            &BivariatePolynomial::new(vec![vec![Real::one()]]),
            policy,
        )
        .map(|sign| sign.map(order)),
        CurvePointData2::Polynomial(point) => point.affine_order(a, b, value, policy),
        CurvePointData2::Exact(point) => Ok(compare_reals(
            &Real::dot2_refs([a, b], [point.x(), point.y()]),
            value,
            policy,
        )
        .map_or(
            Classification::Uncertain(UncertaintyReason::Ordering),
            Classification::Decided,
        )),
        CurvePointData2::Algebraic(point) => match point.predicate_evaluator(policy)? {
            Classification::Decided(predicate) => {
                predicate.linear_order_to_real(a, b, value, policy)
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        },
        CurvePointData2::AnalyticParallel(point) => {
            Ok(point.linear_residual_sign(a, b, value, policy)?.map(order))
        }
        CurvePointData2::AlgebraicCuspChord(point) => {
            point.linear_order_to_real(a, b, value, policy)
        }
        CurvePointData2::Endpoint(endpoint) => match endpoint.resolve(policy)? {
            Classification::Decided(Some(point)) => {
                point_affine_form_order_to_real(&point, a, b, value, policy)
            }
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        },
        CurvePointData2::Similarity(image) => {
            if !policy.accepts_retained_policy(image.data.policy) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let (m00, m01, m10, m11, tx, ty) = image.data.transform.affine_components();
            point_affine_form_order_to_real(
                &image.data.source,
                &Real::dot2_refs([a, b], [m00, m10]),
                &Real::dot2_refs([a, b], [m01, m11]),
                &(value - Real::dot2_refs([a, b], [tx, ty])),
                policy,
            )
        }
        _ => {
            // Existing correlated chord predicates already own these families.
            // A finite point on the requested level line supplies their origin.
            let strict = policy.strict_counterpart();
            let origin = if matches!(
                real_sign(a, &strict),
                Some(RealSign::Positive | RealSign::Negative)
            ) {
                Point2::new((value / a)?, Real::zero())
            } else if matches!(
                real_sign(b, &strict),
                Some(RealSign::Positive | RealSign::Negative)
            ) {
                Point2::new(Real::zero(), (value / b)?)
            } else {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            };
            algebraic_chord_point_linear_order_to_exact(point, &origin, a, b, policy)
        }
    }
}

pub(crate) fn point_affine_form_image(
    source: &CurvePoint2,
    [a, b, c]: &[Real; 3],
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    let strict = policy.strict_counterpart();
    match real_sign(&Real::dot2_refs([a, b], [a, b]), &strict) {
        Some(RealSign::Zero) => Ok(Classification::Decided(
            Point2::new(c.clone(), Real::zero()).into(),
        )),
        Some(RealSign::Positive) => {
            // Completing one nonzero row to a similarity does not reconstruct
            // either coordinate or change the point's original source field.
            let transform = Similarity2::try_from_real_affine(
                a.clone(),
                b.clone(),
                -b,
                a.clone(),
                c.clone(),
                Real::zero(),
            )?;
            Ok(Classification::Decided(
                BezierSimilarityPoint2::new(source.clone(), transform, &strict).into(),
            ))
        }
        _ => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

pub(crate) fn point_affine_form_bounds(
    source: &CurvePoint2,
    [a, b, c]: &[Real; 3],
    policy: &CurveContext,
) -> CurveResult<Classification<(Real, Real)>> {
    if a.zero_status() == ZeroKnowledge::Zero && b.zero_status() == ZeroKnowledge::Zero {
        return Ok(Classification::Decided((c.clone(), c.clone())));
    }
    let bounds = match algebraic_chord_endpoint_bounds_refined(source, 0, policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let exact =
        |value: &Real| RealInterval::from_values([value.clone()]).expect("one scalar interval");
    let interval = real_interval_from_axis(&bounds, Axis2::X)
        .multiply(&exact(a))
        .and_then(|x| {
            real_interval_from_axis(&bounds, Axis2::Y)
                .multiply(&exact(b))
                .map(|y| x.add(&y).add(&exact(c)))
        });
    Ok(interval.map_or(
        Classification::Uncertain(UncertaintyReason::Ordering),
        |interval| Classification::Decided((interval.lower, interval.upper)),
    ))
}
