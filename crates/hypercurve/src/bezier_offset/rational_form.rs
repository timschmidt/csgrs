//! Rational queries dispatch through the original joint point authority.

use super::*;
use hypersolve::bivariate_arithmetic::bivariate_compose;

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

/// Pulls a query into its original coordinate field. The public form retains
/// its original operands/guards; this is not a replacement point definition.
pub(crate) fn point_form_source(
    source: &CurvePoint2,
    numerator: &BivariatePolynomial,
    denominator: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<PointFormSource2>> {
    match &source.0 {
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
