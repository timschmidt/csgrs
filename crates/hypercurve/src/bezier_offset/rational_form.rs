//! Rational queries dispatch through the original joint point authority.

use super::*;
use hypersolve::bivariate_arithmetic::{
    bivariate_add, bivariate_compose, bivariate_multiply, bivariate_scale, bivariate_subtract,
};

#[derive(Clone, Debug)]
pub(crate) struct PointFormSource2 {
    pub(crate) source: CurvePoint2,
    pub(crate) numerator: BivariatePolynomial,
    pub(crate) denominator: BivariatePolynomial,
}

impl PointFormSource2 {
    pub(crate) fn new(
        source: CurvePoint2,
        numerator: BivariatePolynomial,
        denominator: BivariatePolynomial,
    ) -> Self {
        Self {
            source,
            numerator,
            denominator,
        }
    }
}

/// Similarities and represented procedural displacements can transport a
/// joint point field without separating its coordinates. This is only a
/// dispatch hint; the queried determinant still
/// validates the retained maps, guards and policy in that field.
pub(crate) fn point_retains_joint_field(point: &CurvePoint2) -> bool {
    match &point.0 {
        CurvePointData2::Polynomial(_) | CurvePointData2::RationalImage(_) => true,
        CurvePointData2::Similarity(image) => point_retains_joint_field(&image.data.source),
        CurvePointData2::Endpoint(endpoint) => match endpoint.fragment.as_ref() {
            crate::BezierSplitFragment2::AlgebraicChord(chord) => {
                point_retains_joint_field(if endpoint.start {
                    chord.start()
                } else {
                    chord.end()
                })
            }
            crate::BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                let frame = &fragment.data.semicircle.data.frame;
                frame
                    .chord_normal()
                    .is_some_and(|frame| point_retains_joint_field(&frame.center))
                    || frame.selected_radial().is_some_and(|frame| {
                        frame
                            .center_parameter
                            .retained_point_evidence()
                            .is_some_and(point_retains_joint_field)
                    })
            }
            _ => false,
        },
        CurvePointData2::AlgebraicChordParallel(point)
            if point.data.source.certified_unit_tangent().is_some() =>
        {
            point_retains_joint_field(point.source_endpoint())
        }
        CurvePointData2::AlgebraicCuspChordDerived(point) => {
            point
                .data
                .source
                .retained_point()
                .as_ref()
                .is_some_and(point_retains_joint_field)
                || point
                    .data
                    .source
                    .semicircle()
                    .data
                    .frame
                    .chord_normal()
                    .is_some_and(|frame| point_retains_joint_field(&frame.center))
        }
        _ => false,
    }
}

/// Aligns source-bound queries using their retained coordinate maps. The
/// caller keeps the original immutable inputs and denominator guards; these
/// pulled-back forms are temporary predicate views, not replacement points.
pub(crate) fn common_point_form_sources(
    mut first: PointFormSource2,
    mut second: PointFormSource2,
    policy: &CurveContext,
) -> CurveResult<Classification<[PointFormSource2; 2]>> {
    if first.source.shares_storage(&second.source) || first.source == second.source {
        return Ok(Classification::Decided([first, second]));
    }
    let original = [first.clone(), second.clone()];
    for form in [&mut first, &mut second] {
        *form = match point_form_source(&form.source, &form.numerator, &form.denominator, policy)? {
            Classification::Decided(form) => form,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if let Some(point) = form.source.coordinates() {
            form.numerator = BivariatePolynomial::new(vec![vec![bivariate_evaluate_exact(
                &form.numerator,
                point.x(),
                point.y(),
            )]]);
            form.denominator = BivariatePolynomial::new(vec![vec![bivariate_evaluate_exact(
                &form.denominator,
                point.x(),
                point.y(),
            )]]);
        }
    }
    let constant = |polynomial: &BivariatePolynomial| {
        polynomial.coefficients.iter().enumerate().all(|(i, row)| {
            row.iter()
                .enumerate()
                .all(|(j, coefficient)| (i == 0 && j == 0) || coefficient.definitely_zero())
        })
    };
    if constant(&first.numerator) && constant(&first.denominator) {
        first.source = second.source.clone();
    } else if constant(&second.numerator) && constant(&second.denominator) {
        second.source = first.source.clone();
    }
    if first.source.shares_storage(&second.source) || first.source == second.source {
        return Ok(Classification::Decided([first, second]));
    }
    // Equality is only an alignment shortcut here. Query-pair coordinates
    // already have a common selected-field route; comparing both projected
    // coordinates first can rebuild an unnecessary tower of root decisions.
    let query_pair = |point: &CurvePoint2| matches!(&point.0, CurvePointData2::Polynomial(point) if point.has_query_authority());
    let same_point = if query_pair(&first.source) || query_pair(&second.source) {
        None
    } else {
        Some(first.source.same_point(&second.source, policy))
    };
    if same_point == Some(Classification::Decided(true)) {
        return Ok(Classification::Decided([first, second]));
    }
    if same_point
        == Some(Classification::Uncertain(
            UncertaintyReason::BudgetExhausted,
        ))
    {
        return Ok(Classification::Uncertain(
            UncertaintyReason::BudgetExhausted,
        ));
    }
    match crate::polynomial_point::PolynomialPoint2::from_queries(original, policy)? {
        Classification::Decided(source) => {
            let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
            return Ok(Classification::Decided([
                PointFormSource2::new(
                    source.clone(),
                    BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
                    one.clone(),
                ),
                PointFormSource2::new(
                    source,
                    BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
                    one,
                ),
            ]));
        }
        Classification::Uncertain(UncertaintyReason::Unsupported) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    Ok(
        match same_point.unwrap_or_else(|| first.source.same_point(&second.source, policy)) {
            Classification::Decided(true) => Classification::Decided([first, second]),
            Classification::Decided(false) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn point_form_difference(
    first: PointFormSource2,
    second: PointFormSource2,
    policy: &CurveContext,
) -> CurveResult<Classification<PointFormSource2>> {
    Ok(
        common_point_form_sources(first, second, policy)?.map(|[first, second]| {
            let (numerator, denominator) = if first.denominator == second.denominator {
                (
                    bivariate_subtract(&first.numerator, &second.numerator),
                    first.denominator,
                )
            } else {
                (
                    bivariate_subtract(
                        &bivariate_multiply(&first.numerator, &second.denominator),
                        &bivariate_multiply(&second.numerator, &first.denominator),
                    ),
                    bivariate_multiply(&first.denominator, &second.denominator),
                )
            };
            PointFormSource2::new(first.source, numerator, denominator)
        }),
    )
}

/// Recovers a represented displacement when common-field cancellation makes
/// both differences constant. This never materializes the original coordinates
/// or substitutes an enclosure for a nonconstant selected value.
pub(super) fn common_point_constant_displacement(
    first: &CurvePoint2,
    second: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<(Real, Real)>>> {
    let mut values = [Real::zero(), Real::zero()];
    for (value, numerator) in values.iter_mut().zip([
        BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
        BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
    ]) {
        let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let difference = match point_form_difference(
            PointFormSource2::new(first.clone(), numerator.clone(), one.clone()),
            PointFormSource2::new(second.clone(), numerator, one),
            policy,
        )? {
            Classification::Decided(form) => form,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let constant = |polynomial: &BivariatePolynomial| {
            polynomial
                .coefficients
                .iter()
                .enumerate()
                .all(|(i, row)| {
                    row.iter()
                        .enumerate()
                        .all(|(j, coefficient)| (i == 0 && j == 0) || coefficient.definitely_zero())
                })
                .then(|| {
                    polynomial
                        .coefficients
                        .first()
                        .and_then(|row| row.first())
                        .cloned()
                        .unwrap_or_else(Real::zero)
                })
        };
        let (Some(numerator), Some(denominator)) = (
            constant(&difference.numerator),
            constant(&difference.denominator),
        ) else {
            return Ok(Classification::Decided(None));
        };
        if let Classification::Uncertain(reason) =
            point_rational_denominator_sign(&difference.source, &difference.denominator, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }
        *value = (numerator / denominator)?;
    }
    let [x, y] = values;
    Ok(Classification::Decided(Some((x, y))))
}

/// Compares a linear projection before separating correlated point coordinates.
pub(super) fn common_point_linear_order(
    first: &CurvePoint2,
    second: &CurvePoint2,
    a: &Real,
    b: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    let numerator = BivariatePolynomial::new(vec![vec![Real::zero(), b.clone()], vec![a.clone()]]);
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let form = match point_form_difference(
        PointFormSource2::new(first.clone(), numerator.clone(), one.clone()),
        PointFormSource2::new(second.clone(), numerator, one),
        policy,
    )? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(
        point_rational_form_sign(&form.source, &form.numerator, &form.denominator, policy)?.map(
            |sign| match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            },
        ),
    )
}

/// Retains both coordinates of P-C in one shared query field. The caller
/// keeps the original point and center, including their denominator guards.
pub(super) fn common_point_displacement(
    point: &CurvePoint2,
    center: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<[PointFormSource2; 2]>> {
    let coordinate_difference = |numerator: BivariatePolynomial| {
        let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
        point_form_difference(
            PointFormSource2::new(point.clone(), numerator.clone(), one.clone()),
            PointFormSource2::new(center.clone(), numerator, one),
            policy,
        )
    };
    let dx = match coordinate_difference(BivariatePolynomial::new(vec![
        vec![Real::zero()],
        vec![Real::one()],
    ]))? {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let dy = match coordinate_difference(BivariatePolynomial::new(vec![vec![
        Real::zero(),
        Real::one(),
    ]]))? {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    common_point_form_sources(dx, dy, policy)
}

/// Intersects a line with a fixed coordinate in the endpoints' shared field.
/// The caller owns the finite placement on the line's injective axis. Original
/// coordinate denominators remain guards even when division moves a factor
/// into a numerator.
pub(super) fn common_point_line_at_coordinate(
    start: &CurvePoint2,
    end: &CurvePoint2,
    axis: Axis2,
    coordinate: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    let [dx, dy] = match common_point_displacement(end, start, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (direction, other_direction, axis_polynomial, other_polynomial) = match axis {
        Axis2::X => (
            dx,
            dy,
            BivariatePolynomial::new(vec![vec![-coordinate], vec![Real::one()]]),
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
        ),
        Axis2::Y => (
            dy,
            dx,
            BivariatePolynomial::new(vec![vec![-coordinate, Real::one()]]),
            BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
        ),
    };
    match point_rational_form_sign(
        &direction.source,
        &direction.numerator,
        &direction.denominator,
        policy,
    )? {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let axis_difference = match point_form_source(start, &axis_polynomial, &one, policy)? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [axis_difference, other_direction] =
        match common_point_form_sources(axis_difference, other_direction, policy)? {
            Classification::Decided(forms) => forms,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let product = PointFormSource2::new(
        axis_difference.source,
        bivariate_multiply(&axis_difference.numerator, &other_direction.numerator),
        bivariate_multiply(&axis_difference.denominator, &other_direction.denominator),
    );
    let [product, direction] = match common_point_form_sources(product, direction, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let quotient = PointFormSource2::new(
        product.source,
        bivariate_multiply(
            &bivariate_multiply(&product.numerator, &direction.denominator),
            &direction.denominator,
        ),
        bivariate_multiply(
            &bivariate_multiply(&product.denominator, &direction.numerator),
            &direction.denominator,
        ),
    );
    let other = match point_form_difference(
        PointFormSource2::new(start.clone(), other_polynomial, one),
        quotient,
        policy,
    )? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let fixed = bivariate_scale(other.denominator.clone(), coordinate);
    let numerators = match axis {
        Axis2::X => [fixed, other.numerator],
        Axis2::Y => [other.numerator, fixed],
    };
    crate::ExactCurveError::classify_result(other.source.rational_image_with_policy(
        &crate::RationalMap2::new(numerators, other.denominator),
        policy,
    ))
}

/// A linear query of two already aligned field coordinates. Keep a shared
/// denominator instead of multiplying it into itself on every angular query.
pub(super) fn point_form_linear_combination(
    [x, y]: &[PointFormSource2; 2],
    a: &Real,
    b: &Real,
) -> PointFormSource2 {
    let (numerator, denominator) = if x.denominator == y.denominator {
        (
            bivariate_add(
                &bivariate_scale(x.numerator.clone(), a),
                &bivariate_scale(y.numerator.clone(), b),
            ),
            x.denominator.clone(),
        )
    } else {
        (
            bivariate_add(
                &bivariate_scale(bivariate_multiply(&x.numerator, &y.denominator), a),
                &bivariate_scale(bivariate_multiply(&y.numerator, &x.denominator), b),
            ),
            bivariate_multiply(&x.denominator, &y.denominator),
        )
    };
    PointFormSource2::new(x.source.clone(), numerator, denominator)
}

fn point_form_product(
    first: PointFormSource2,
    second: PointFormSource2,
    policy: &CurveContext,
) -> CurveResult<Classification<PointFormSource2>> {
    Ok(
        common_point_form_sources(first, second, policy)?.map(|[first, second]| {
            PointFormSource2::new(
                first.source,
                bivariate_multiply(&first.numerator, &second.numerator),
                bivariate_multiply(&first.denominator, &second.denominator),
            )
        }),
    )
}

fn point_form_dot(
    first: &[PointFormSource2; 2],
    second: &[PointFormSource2; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<PointFormSource2>> {
    let mut products = Vec::with_capacity(2);
    for (first, second) in first.iter().zip(second) {
        match point_form_product(first.clone(), second.clone(), policy)? {
            Classification::Decided(product) => products.push(product),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    let [x, y] = products.try_into().expect("two vector components");
    Ok(common_point_form_sources(x, y, policy)?
        .map(|forms| point_form_linear_combination(&forms, &Real::one(), &Real::one())))
}

/// Given an incident point A, the second contact on direction D is
/// A - 2 ((A-C).D) D / (D.D). No new root or coordinate projection is needed.
/// The dot sign also gives tangency and the two opposite transverse signs.
pub(super) fn common_point_circle_second_line_contact(
    incident: &CurvePoint2,
    center: &CurvePoint2,
    start: &CurvePoint2,
    end: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<(CurvePoint2, RealSign)>> {
    let radial = match common_point_displacement(incident, center, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let direction = match common_point_displacement(end, start, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let norm = match point_form_dot(&direction, &direction, policy)? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match point_rational_form_sign(&norm.source, &norm.numerator, &norm.denominator, policy)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(_) => {
            return Err(CurveError::Topology(
                "a circle contact line must have positive squared direction".into(),
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let dot = match point_form_dot(&radial, &direction, policy)? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let sign =
        match point_rational_form_sign(&dot.source, &dot.numerator, &dot.denominator, policy)? {
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided((incident.clone(), RealSign::Zero)));
            }
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let [dot, norm] = match common_point_form_sources(dot, norm, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    // Division moves the norm denominator upstairs. Keep it as an original
    // nonzero guard as well, including after native replay of the final map.
    let scale = PointFormSource2::new(
        dot.source,
        bivariate_scale(
            bivariate_multiply(
                &bivariate_multiply(&dot.numerator, &norm.denominator),
                &norm.denominator,
            ),
            &Real::from(2),
        ),
        bivariate_multiply(
            &bivariate_multiply(&dot.denominator, &norm.numerator),
            &norm.denominator,
        ),
    );
    let coordinates = [
        BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
        BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
    ];
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let mut result = Vec::with_capacity(2);
    for (direction, coordinate) in direction.into_iter().zip(coordinates) {
        let delta = match point_form_product(scale.clone(), direction, policy)? {
            Classification::Decided(form) => form,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match point_form_difference(
            PointFormSource2::new(incident.clone(), coordinate, one.clone()),
            delta,
            policy,
        )? {
            Classification::Decided(form) => result.push(form),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    let [x, y] = result.try_into().expect("two contact coordinates");
    let [x, y] = match common_point_form_sources(x, y, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (numerators, denominator) = if x.denominator == y.denominator {
        ([x.numerator, y.numerator], x.denominator)
    } else {
        (
            [
                bivariate_multiply(&x.numerator, &y.denominator),
                bivariate_multiply(&y.numerator, &x.denominator),
            ],
            bivariate_multiply(&x.denominator, &y.denominator),
        )
    };
    Ok(crate::ExactCurveError::classify_result(
        x.source
            .rational_image_with_policy(&crate::RationalMap2::new(numerators, denominator), policy),
    )?
    .map(|point| (point, sign)))
}

/// Signs |P-C|^2-r^2 in one certified common coordinate field. The squared
/// denominator is positive only after both original difference denominators
/// have been proved nonzero under the consuming policy.
pub(super) fn common_point_circle_residual_sign(
    point: &CurvePoint2,
    center: &CurvePoint2,
    radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    // A native circle contact already retains a compact common field. Use
    // that authority before constructing general query-pair coordinates.
    match recursive_projective_point_evidence_circle_residual_sign(
        point,
        center,
        radius_squared,
        policy,
    )? {
        Classification::Decided(Some(sign)) => return Ok(Classification::Decided(sign)),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
            return Ok(Classification::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        Classification::Decided(None) | Classification::Uncertain(_) => {}
    }
    let [dx, dy] = match common_point_displacement(point, center, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for form in [&dx, &dy] {
        if let Classification::Uncertain(reason) =
            point_rational_denominator_sign(&form.source, &form.denominator, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let (x, y, denominator) = if dx.denominator == dy.denominator {
        (dx.numerator, dy.numerator, dx.denominator)
    } else {
        (
            bivariate_multiply(&dx.numerator, &dy.denominator),
            bivariate_multiply(&dy.numerator, &dx.denominator),
            bivariate_multiply(&dx.denominator, &dy.denominator),
        )
    };
    let residual = bivariate_subtract(
        &bivariate_add(&bivariate_multiply(&x, &x), &bivariate_multiply(&y, &y)),
        &bivariate_scale(
            bivariate_multiply(&denominator, &denominator),
            radius_squared,
        ),
    );
    point_polynomial_sign(&dx.source, &residual, policy)
}

/// Signs the circle-pair discriminant in the centers' shared point field.
/// For q=|C2-C1|², real contacts require 4*q*r1²-(q+r1²-r2²)² >= 0.
/// Keep the original coordinate guards; clearing the final squared
/// denominator preserves the sign without materializing either center.
pub(super) fn common_point_circle_pair_discriminant_sign(
    first: &CurvePoint2,
    second: &CurvePoint2,
    first_radius_squared: &Real,
    second_radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let displacement = match common_point_displacement(second, first, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for form in &displacement {
        if let Classification::Uncertain(reason) =
            point_rational_denominator_sign(&form.source, &form.denominator, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let norm = match point_form_dot(&displacement, &displacement, policy)? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let line = bivariate_add(
        &norm.numerator,
        &bivariate_scale(
            norm.denominator.clone(),
            &(first_radius_squared - second_radius_squared),
        ),
    );
    let discriminant = bivariate_subtract(
        &bivariate_scale(
            bivariate_multiply(&norm.numerator, &norm.denominator),
            &(Real::from(4) * first_radius_squared),
        ),
        &bivariate_multiply(&line, &line),
    );
    point_polynomial_sign(&norm.source, &discriminant, policy)
}

/// Signs the oriented area of three points in their retained common field.
/// Each original coordinate denominator remains a guard of the determinant.
pub(super) fn common_point_oriented_side(
    start: &CurvePoint2,
    end: &CurvePoint2,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::classify::LineSide>> {
    // Preserve the original native field, including shared radicals and
    // compact scalar witnesses, before forming new query-pair coordinates.
    // General polynomial fields still use the source-bound fallback below.
    match recursive_projective_point_evidence_oriented_side(start, end, point, false, policy)? {
        Classification::Decided(Some(side)) => return Ok(Classification::Decided(side)),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
            return Ok(Classification::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        Classification::Decided(None) | Classification::Uncertain(_) => {}
    }
    let [ux, uy] = match common_point_displacement(end, start, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [vx, vy] = match common_point_displacement(point, start, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let first = match point_form_product(ux, vy, policy)? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second = match point_form_product(uy, vx, policy)? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let determinant = match point_form_difference(first, second, policy)? {
        Classification::Decided(form) => form,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(point_rational_form_sign(
        &determinant.source,
        &determinant.numerator,
        &determinant.denominator,
        policy,
    )?
    .map(|sign| match sign {
        RealSign::Negative => crate::classify::LineSide::Right,
        RealSign::Zero => crate::classify::LineSide::On,
        RealSign::Positive => crate::classify::LineSide::Left,
    }))
}

/// Reuses the original common field for Q=C+a(P-C)+b*J(P-C)+T.
/// This temporary rational map is only a query view; Q keeps its circle and
/// selected contact authority for reconstruction and native replay.
fn derived_circle_point_form(
    point: &BezierAlgebraicCuspChordDerivedPoint2,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<PointFormSource2>>> {
    point.data.source.validate_policy(policy)?;
    let Some(source) = point.data.source.retained_point() else {
        return Ok(Classification::Decided(None));
    };
    if matches!(&source.0, CurvePointData2::AlgebraicCuspChordDerived(other) if Arc::ptr_eq(&point.data, &other.data))
    {
        return Ok(Classification::Decided(None));
    }
    let center = match point
        .data
        .source
        .semicircle()
        .center_point_evidence(policy)?
    {
        Classification::Decided(center) => center,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let radial = match common_point_displacement(&source, &center, policy)? {
        Classification::Decided(radial) => radial,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for form in &radial {
        if let Classification::Uncertain(reason) =
            point_rational_denominator_sign(&form.source, &form.denominator, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let displacement = [
        point_form_linear_combination(
            &radial,
            &point.data.radial_scale,
            &(-point.data.perpendicular_scale.clone()),
        ),
        point_form_linear_combination(
            &radial,
            &point.data.perpendicular_scale,
            &point.data.radial_scale,
        ),
    ];
    let mut coordinates = Vec::with_capacity(2);
    let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
    for ((displacement, coordinate), translation) in displacement
        .into_iter()
        .zip([
            BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
        ])
        .zip([&point.data.translation_x, &point.data.translation_y])
    {
        let center_coordinate = match point_form_source(&center, &coordinate, &one, policy)? {
            Classification::Decided(form) => form,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if let Classification::Uncertain(reason) = point_rational_denominator_sign(
            &center_coordinate.source,
            &center_coordinate.denominator,
            policy,
        )? {
            return Ok(Classification::Uncertain(reason));
        }
        let forms = match common_point_form_sources(displacement, center_coordinate, policy)? {
            Classification::Decided(forms) => forms,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut coordinate = point_form_linear_combination(&forms, &Real::one(), &Real::one());
        coordinate.numerator = bivariate_add(
            &coordinate.numerator,
            &bivariate_scale(coordinate.denominator.clone(), translation),
        );
        coordinates.push(coordinate);
    }
    let [x, y] = coordinates
        .try_into()
        .expect("two affine-derived coordinates were constructed");
    let [x, y] = match common_point_form_sources(x, y, policy)? {
        Classification::Decided(forms) => forms,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (coordinates, common_denominator) = if x.denominator == y.denominator {
        ([x.numerator, y.numerator], x.denominator)
    } else {
        (
            [
                bivariate_multiply(&x.numerator, &y.denominator),
                bivariate_multiply(&y.numerator, &x.denominator),
            ],
            bivariate_multiply(&x.denominator, &y.denominator),
        )
    };
    if let Classification::Uncertain(reason) =
        point_rational_denominator_sign(&x.source, &common_denominator, policy)?
    {
        return Ok(Classification::Uncertain(reason));
    }
    Ok(crate::RationalMap2::new(coordinates, common_denominator)
        .pullback(numerator, denominator)
        .map(|(numerator, denominator)| {
            Classification::Decided(Some(PointFormSource2::new(
                x.source,
                numerator,
                denominator,
            )))
        })
        .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)))
}

/// Pulls a query into its original coordinate field. The public form retains
/// its original operands/guards; this is not a replacement point definition.
pub(crate) fn point_form_source(
    source: &CurvePoint2,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<PointFormSource2>> {
    match &source.0 {
        CurvePointData2::RationalImage(image) => {
            match image.pullback(numerator, denominator, policy)? {
                Classification::Decided((numerator, denominator)) => {
                    point_form_source(&image.source, &numerator, &denominator, policy)
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        CurvePointData2::Polynomial(point) => {
            Ok(match point.source_query(numerator, denominator) {
                Some(query) => Classification::Decided(query),
                None => Classification::Uncertain(UncertaintyReason::Unsupported),
            })
        }
        CurvePointData2::Algebraic(image) => {
            image.rational_query_source(numerator, denominator, policy)
        }
        CurvePointData2::Similarity(image) => {
            if !policy.accepts_retained_policy(image.data.policy) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            point_form_source(
                &image.data.source,
                &pullback(numerator, &image.data.transform),
                &pullback(denominator, &image.data.transform),
                policy,
            )
        }
        CurvePointData2::AlgebraicChordParallel(point)
            if let Some((tangent_x, tangent_y)) = point.data.source.certified_unit_tangent() =>
        {
            if !policy.accepts_retained_policy(point.data.policy) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            point.data.source.validate_policy(policy)?;
            let (x, y) = match point.data.direction {
                BezierAlgebraicChordUnitDisplacement2::Tangent => (tangent_x, tangent_y),
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => (-tangent_y, tangent_x),
            };
            // This is a temporary query in the displaced point's original
            // field. Keep the procedural point and its source-normal relation
            // authoritative for composition and native replay.
            let transform = Similarity2::try_from_real_affine(
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                x * &point.data.distance + &point.data.translation_x,
                y * &point.data.distance + &point.data.translation_y,
            )?;
            point_form_source(
                point.source_endpoint(),
                &pullback(numerator, &transform),
                &pullback(denominator, &transform),
                policy,
            )
        }
        CurvePointData2::AlgebraicCuspChordDerived(point) => {
            match derived_circle_point_form(point, numerator, denominator, policy)? {
                Classification::Decided(Some(form)) => Ok(Classification::Decided(form)),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted) => Ok(
                    Classification::Uncertain(UncertaintyReason::BudgetExhausted),
                ),
                Classification::Decided(None) | Classification::Uncertain(_) => {
                    Ok(Classification::Decided(PointFormSource2::new(
                        source.clone(),
                        numerator.clone(),
                        denominator.clone(),
                    )))
                }
            }
        }
        CurvePointData2::Endpoint(endpoint) => match endpoint.resolve(policy)? {
            Classification::Decided(Some(point)) => {
                point_form_source(&point, numerator, denominator, policy)
            }
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        },
        _ => Ok(Classification::Decided(PointFormSource2::new(
            source.clone(),
            numerator.clone(),
            denominator.clone(),
        ))),
    }
}

pub(crate) fn point_rational_denominator_sign(
    source: &CurvePoint2,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    PointSignScope::new(policy).denominator_sign(source, denominator)
}

pub(crate) fn point_rational_form_sign(
    source: &CurvePoint2,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    PointSignScope::new(policy).rational_sign(source, numerator, denominator)
}

fn point_polynomial_sign(
    source: &CurvePoint2,
    polynomial: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    PointSignScope::new(policy).polynomial_sign(source, polynomial)
}

/// Successful signs belong to one query and one policy. Owned immutable points
/// keep storage identities alive; exact polynomial structure completes the key.
/// Distinct storage never proves inequality. No failure or exhausted attempt is
/// retained, and dropping the scope releases every entry. Retention follows the
/// visited query graph, not a fixed heap-byte or total-work bound.
struct PointSignScope<'a> {
    policy: &'a CurveContext,
    memoize: bool,
    decided: Vec<(CurvePoint2, BivariatePolynomial, RealSign)>,
}

impl<'a> PointSignScope<'a> {
    fn new(policy: &'a CurveContext) -> Self {
        Self {
            policy,
            memoize: false,
            decided: Vec::new(),
        }
    }

    fn denominator_sign(
        &mut self,
        source: &CurvePoint2,
        denominator: &BivariatePolynomial,
    ) -> CurveResult<Classification<RealSign>> {
        match self.polynomial_sign(source, denominator)? {
            Classification::Decided(RealSign::Zero) => {
                Err(CurveError::ZeroRationalPointDenominator)
            }
            sign => Ok(sign),
        }
    }

    fn rational_sign(
        &mut self,
        source: &CurvePoint2,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
    ) -> CurveResult<Classification<RealSign>> {
        let denominator = match self.denominator_sign(source, denominator)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(self
            .polynomial_sign(source, numerator)?
            .map(|sign| product_sign(sign, denominator)))
    }

    fn polynomial_sign(
        &mut self,
        source: &CurvePoint2,
        polynomial: &BivariatePolynomial,
    ) -> CurveResult<Classification<RealSign>> {
        if hypersolve::work_budget::exhaustion().is_some() {
            return Ok(Classification::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        // Ordinary leaf queries allocate no memo. A retained rational image
        // introduces the repeated guard/numerator/denominator dependencies.
        self.memoize |= matches!(&source.0, CurvePointData2::RationalImage(_));
        if self.memoize
            && let Some((_, _, sign)) = self
                .decided
                .iter()
                .find(|(point, query, _)| point.shares_storage(source) && query == polynomial)
        {
            return Ok(Classification::Decided(*sign));
        }
        let result = self.dispatch(source, polynomial)?;
        if hypersolve::work_budget::exhaustion().is_some() {
            return Ok(Classification::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        if self.memoize
            && let Classification::Decided(sign) = &result
        {
            self.decided
                .push((source.clone(), polynomial.clone(), *sign));
        }
        Ok(result)
    }

    fn dispatch(
        &mut self,
        source: &CurvePoint2,
        polynomial: &BivariatePolynomial,
    ) -> CurveResult<Classification<RealSign>> {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record("hypercurve", "point-polynomial-sign", "dispatch");
        let policy = self.policy;
        match &source.0 {
            CurvePointData2::RationalImage(image) => {
                let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
                if let Classification::Uncertain(reason) = image.validate_policy(policy) {
                    return Ok(Classification::Uncertain(reason));
                }
                // The guard, pulled-back numerator and denominator share one
                // sign scope; validating just the guard still duplicates recursion.
                if let Classification::Uncertain(reason) =
                    self.denominator_sign(&image.source, image.map.denominator())?
                {
                    return Ok(Classification::Uncertain(reason));
                }
                let Some((numerator, denominator)) = image.map.pullback(polynomial, &one) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                self.rational_sign(&image.source, &numerator, &denominator)
            }
            CurvePointData2::Exact(point) => Ok(real_sign(
                &bivariate_evaluate_exact(polynomial, point.x(), point.y()),
                policy,
            )
            .map_or(
                Classification::Uncertain(UncertaintyReason::Predicate),
                Classification::Decided,
            )),
            CurvePointData2::Polynomial(point) => point.polynomial_sign(polynomial, policy),
            CurvePointData2::Endpoint(endpoint) => match endpoint.resolve(policy)? {
                Classification::Decided(Some(point)) => self.polynomial_sign(&point, polynomial),
                Classification::Decided(None) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            },
            CurvePointData2::Similarity(image) => {
                if !policy.accepts_retained_policy(image.data.policy) {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                self.polynomial_sign(
                    &image.data.source,
                    &pullback(polynomial, &image.data.transform),
                )
            }
            _ => {
                let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
                match recursive_rational_value(source, polynomial, &one, policy)? {
                    Classification::Decided(Some(value)) => value.numerator.sign(policy),
                    Classification::Decided(None) => {
                        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
        }
    }
}

fn pullback(polynomial: &BivariatePolynomial, transform: &Similarity2) -> BivariatePolynomial {
    let (a, b, c, d, x, y) = transform.affine_components();
    bivariate_compose(
        polynomial,
        &BivariatePolynomial::new(vec![vec![x.clone(), b.clone()], vec![a.clone()]]),
        &BivariatePolynomial::new(vec![vec![y.clone(), d.clone()], vec![c.clone()]]),
    )
}

pub(crate) fn point_rational_form_parameter(
    source: &CurvePoint2,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    let sign = match point_rational_denominator_sign(source, denominator, policy)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match &source.0 {
        CurvePointData2::RationalImage(image) => {
            match image.pullback(numerator, denominator, policy)? {
                Classification::Decided((numerator, denominator)) => {
                    point_rational_form_parameter(&image.source, &numerator, &denominator, policy)
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        CurvePointData2::Exact(point) => Ok(Classification::Decided(
            (bivariate_evaluate_exact(numerator, point.x(), point.y())
                / bivariate_evaluate_exact(denominator, point.x(), point.y()))?
            .into(),
        )),
        CurvePointData2::Polynomial(point) => {
            point.rational_parameter(numerator, denominator, policy)
        }
        CurvePointData2::Endpoint(endpoint) => match endpoint.resolve(policy)? {
            Classification::Decided(Some(point)) => {
                point_rational_form_parameter(&point, numerator, denominator, policy)
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
            point_rational_form_parameter(
                &image.data.source,
                &pullback(numerator, &image.data.transform),
                &pullback(denominator, &image.data.transform),
                policy,
            )
        }
        _ => {
            let mut value = match recursive_rational_value(source, numerator, denominator, policy)?
            {
                Classification::Decided(Some(value)) => value,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            if sign == RealSign::Negative {
                let Some(numerator) = value.numerator.scale(&Real::from(-1)) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                let Some(denominator) = value.denominator.scale(&Real::from(-1)) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                value = RecursiveQuadraticProjectiveScalar {
                    numerator,
                    denominator,
                };
            }
            Ok(BezierRecursiveProjectiveParameter2::new(value, policy)?
                .map(CurveParameter2::from_recursive_projective))
        }
    }
}

pub(super) fn recursive_rational_value(
    source: &CurvePoint2,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RecursiveQuadraticProjectiveScalar>>> {
    let points = match recursive_projective_evidence_points(&[source], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let point = match positive_recursive_projective_point(
        points.into_iter().next().expect("one source point"),
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let degree = [numerator, denominator]
        .into_iter()
        .flat_map(|p| {
            p.coefficients.iter().enumerate().flat_map(|(i, row)| {
                row.iter()
                    .enumerate()
                    .filter(|(_, c)| !c.definitely_zero())
                    .map(move |(j, _)| i.checked_add(j))
            })
        })
        .collect::<Option<Vec<_>>>()
        .and_then(|degrees| degrees.into_iter().max().or(Some(0)));
    let Some(degree) = degree else {
        return Ok(Classification::Decided(None));
    };
    let evaluate = |p| {
        evaluate_bivariate_homogeneous_polynomial(p, &point.x, &point.y, &point.denominator, degree)
    };
    Ok(Classification::Decided(
        evaluate(numerator)
            .zip(evaluate(denominator))
            .map(
                |(numerator, denominator)| RecursiveQuadraticProjectiveScalar {
                    numerator,
                    denominator,
                },
            ),
    ))
}
