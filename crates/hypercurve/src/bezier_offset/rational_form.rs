//! Rational queries dispatch through the original joint point authority.

use super::*;
use hypersolve::bivariate_arithmetic::{
    bivariate_add, bivariate_compose, bivariate_multiply, bivariate_scale, bivariate_subtract,
};

#[derive(Debug)]
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
    Ok(match first.source.same_point(&second.source, policy) {
        Classification::Decided(true) => Classification::Decided([first, second]),
        Classification::Decided(false) => Classification::Uncertain(UncertaintyReason::Unsupported),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
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

/// Signs |P-C|^2-r^2 in one certified common coordinate field. The squared
/// denominator is positive only after both original difference denominators
/// have been proved nonzero under the consuming policy.
pub(super) fn common_point_circle_residual_sign(
    point: &CurvePoint2,
    center: &CurvePoint2,
    radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
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
    match point_polynomial_sign(source, denominator, policy)? {
        Classification::Decided(RealSign::Zero) => Err(CurveError::ZeroRationalPointDenominator),
        sign => Ok(sign),
    }
}

pub(crate) fn point_rational_form_sign(
    source: &CurvePoint2,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let denominator = match point_rational_denominator_sign(source, denominator, policy)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(point_polynomial_sign(source, numerator, policy)?
        .map(|sign| product_sign(sign, denominator)))
}

fn point_polynomial_sign(
    source: &CurvePoint2,
    polynomial: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match &source.0 {
        CurvePointData2::RationalImage(image) => {
            let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
            match image.pullback(polynomial, &one, policy)? {
                Classification::Decided((numerator, denominator)) => {
                    point_rational_form_sign(&image.source, &numerator, &denominator, policy)
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
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
            Classification::Decided(Some(point)) => {
                point_polynomial_sign(&point, polynomial, policy)
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
            point_polynomial_sign(
                &image.data.source,
                &pullback(polynomial, &image.data.transform),
                policy,
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

fn recursive_rational_value(
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
