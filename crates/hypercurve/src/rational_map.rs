//! Rational plane images keep the joint source and the original pole guards.

use hypersolve::bivariate_arithmetic::{
    bivariate_active_total_degree, bivariate_add, bivariate_homogeneous_compose,
    bivariate_multiply, bivariate_parameter_derivative, bivariate_scale, bivariate_subtract,
};
use hypersolve::{BivariatePolynomial, CurveResultantParameter, RealInterval};

use crate::{
    Aabb2, CurveContext, CurveError, CurveFamily2, CurveOperation2, CurvePoint2, CurvePointData2,
    ExactCurveError, ExactCurveResult, Point2, Real, Similarity2, UncertaintyReason,
};

/// A rational map `(x(u,v)/w(u,v), y(u,v)/w(u,v))`.
///
/// This is an immutable definition, not an injectivity or nonzero-denominator
/// certificate. Applying it checks the original denominator before retaining
/// its image. Coefficients are ascending powers `[i][j] = u^i v^j`.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalMap2 {
    numerators: [BivariatePolynomial; 2],
    denominator: BivariatePolynomial,
}

fn blocked() -> ExactCurveError {
    ExactCurveError::blocked(
        CurveOperation2::Construction,
        CurveFamily2::ImplicitPolynomial,
        UncertaintyReason::Unsupported,
    )
}

impl RationalMap2 {
    /// Retains the definition without cancelling any denominator factor.
    pub fn new(numerators: [BivariatePolynomial; 2], denominator: BivariatePolynomial) -> Self {
        Self {
            numerators,
            denominator,
        }
    }

    /// Both original coordinate numerators.
    pub const fn numerators(&self) -> &[BivariatePolynomial; 2] {
        &self.numerators
    }

    /// Original shared denominator, including removable pole factors.
    pub const fn denominator(&self) -> &BivariatePolynomial {
        &self.denominator
    }

    pub(crate) fn from_similarity(map: &Similarity2) -> Self {
        let (a, b, c, d, x, y) = map.affine_components();
        Self::new(
            [
                BivariatePolynomial::new(vec![vec![x.clone(), b.clone()], vec![a.clone()]]),
                BivariatePolynomial::new(vec![vec![y.clone(), d.clone()], vec![c.clone()]]),
            ],
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        )
    }

    pub(crate) fn pullback(
        &self,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
    ) -> Option<(BivariatePolynomial, BivariatePolynomial)> {
        let degree = bivariate_active_total_degree(numerator)?
            .max(bivariate_active_total_degree(denominator)?);
        let compose = |p| {
            bivariate_homogeneous_compose(
                p,
                &self.numerators[0],
                &self.numerators[1],
                &self.denominator,
                degree,
            )
        };
        Some((compose(numerator)?, compose(denominator)?))
    }

    /// A conservative image enclosure, after proving the denominator nonzero
    /// over the whole input box. Failure to enclose is a typed blocker; it is
    /// not evidence that the map has a pole. No subdivision is performed.
    pub fn bounds(&self, domain: &Aabb2) -> ExactCurveResult<Aabb2> {
        let axes = axes(domain);
        let evaluate = |p| RealInterval::evaluate_bivariate_power_basis(p, &axes[0], &axes[1]);
        let weight = evaluate(&self.denominator).ok_or_else(blocked)?;
        if weight.lower.definitely_zero() && weight.upper.definitely_zero() {
            return Err(ExactCurveError::invalid_unattributed(
                CurveOperation2::Construction,
                CurveError::ZeroRationalPointDenominator,
            ));
        }
        weight.strict_nonzero_sign().ok_or_else(blocked)?;
        let x = evaluate(&self.numerators[0])
            .and_then(|v| v.divide(&weight))
            .ok_or_else(blocked)?;
        let y = evaluate(&self.numerators[1])
            .and_then(|v| v.divide(&weight))
            .ok_or_else(blocked)?;
        Ok(Aabb2::new_unchecked(
            Point2::new(x.lower, y.lower),
            Point2::new(x.upper, y.upper),
        ))
    }

    /// Proves injectivity on a convex rectangle by strict monotonicity after
    /// a constant invertible linear change of image coordinates.
    ///
    /// The inverse homogeneous Jacobian numerator at the midpoint normalizes
    /// the map up to a positive constant factor.
    /// Exact interval bounds prove that the symmetric part of its Jacobian
    /// is positive definite everywhere. Integrating the positive quadratic
    /// form along the segment between distinct points proves distinct images.
    /// A blocker means this sufficient certificate was unavailable; it does
    /// not assert noninjectivity. Precision is inherited from strict scalar
    /// predicates; coefficient growth and total work have no additional bound.
    pub fn certify_injective_on(&self, domain: &Aabb2) -> ExactCurveResult<()> {
        self.bounds(domain)?;
        let axes = axes(domain);
        let center = axes.clone().map(|axis| {
            let mid = ((axis.lower + axis.upper) / Real::from(2)).expect("nonzero divisor");
            RealInterval {
                lower: mid.clone(),
                upper: mid,
            }
        });
        let derivative = |p, axis| bivariate_parameter_derivative(p, axis);
        let jacobian: [[BivariatePolynomial; 2]; 2] = std::array::from_fn(|i| {
            [
                CurveResultantParameter::First,
                CurveResultantParameter::Second,
            ]
            .map(|axis| {
                bivariate_subtract(
                    &bivariate_multiply(&derivative(&self.numerators[i], axis), &self.denominator),
                    &bivariate_multiply(&self.numerators[i], &derivative(&self.denominator, axis)),
                )
            })
        });
        let mut midpoint = [[Real::zero(), Real::zero()], [Real::zero(), Real::zero()]];
        for i in 0..2 {
            for j in 0..2 {
                midpoint[i][j] = RealInterval::evaluate_bivariate_power_basis(
                    &jacobian[i][j],
                    &center[0],
                    &center[1],
                )
                .ok_or_else(blocked)?
                .lower;
            }
        }
        let determinant = &midpoint[0][0] * &midpoint[1][1] - &midpoint[0][1] * &midpoint[1][0];
        RealInterval {
            lower: determinant.clone(),
            upper: determinant.clone(),
        }
        .strict_nonzero_sign()
        .ok_or_else(blocked)?;
        let reciprocal = (Real::one() / determinant).map_err(|_| blocked())?;
        let inverse = [
            [
                &midpoint[1][1] * &reciprocal,
                -(&midpoint[0][1] * &reciprocal),
            ],
            [
                -(&midpoint[1][0] * &reciprocal),
                &midpoint[0][0] * &reciprocal,
            ],
        ];
        let normalized: [[BivariatePolynomial; 2]; 2] = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                bivariate_add(
                    &bivariate_scale(jacobian[0][j].clone(), &inverse[i][0]),
                    &bivariate_scale(jacobian[1][j].clone(), &inverse[i][1]),
                )
            })
        });
        let off_diagonal = bivariate_add(&normalized[0][1], &normalized[1][0]);
        let determinant = bivariate_subtract(
            &bivariate_scale(
                bivariate_multiply(&normalized[0][0], &normalized[1][1]),
                &Real::from(4),
            ),
            &bivariate_multiply(&off_diagonal, &off_diagonal),
        );
        for polynomial in [&normalized[0][0], &normalized[1][1], &determinant] {
            if RealInterval::evaluate_bivariate_power_basis(polynomial, &axes[0], &axes[1])
                .and_then(|interval| interval.strict_nonzero_sign())
                != Some(hyperreal::RealSign::Positive)
            {
                return Err(blocked());
            }
        }
        Ok(())
    }
}

fn axes(domain: &Aabb2) -> [RealInterval; 2] {
    [
        RealInterval {
            lower: domain.min().x().clone(),
            upper: domain.max().x().clone(),
        },
        RealInterval {
            lower: domain.min().y().clone(),
            upper: domain.max().y().clone(),
        },
    ]
}

impl CurvePoint2 {
    /// Applies a rational map while preserving the joint polynomial point.
    /// Authored points and similarities of polynomial points are also accepted.
    /// Original poles remain invalid even if the image admits cancellation.
    pub fn rational_image(&self, map: &RationalMap2) -> ExactCurveResult<Self> {
        self.rational_form(
            BivariatePolynomial::new(Vec::new()),
            map.denominator.clone(),
        )?;
        match &self.0 {
            CurvePointData2::Polynomial(point) => Ok(point.mapped(map)),
            CurvePointData2::Exact(point) => {
                let axes = [point.x(), point.y()].map(|value| RealInterval {
                    lower: value.clone(),
                    upper: value.clone(),
                });
                let evaluate = |p| {
                    RealInterval::evaluate_bivariate_power_basis(p, &axes[0], &axes[1])
                        .ok_or_else(blocked)
                        .map(|value| value.lower)
                };
                let weight = evaluate(&map.denominator)?;
                let coordinate = |p| {
                    (evaluate(p)? / &weight).map_err(|cause| {
                        ExactCurveError::invalid_unattributed(
                            CurveOperation2::Construction,
                            CurveError::from(cause),
                        )
                    })
                };
                Ok(Point2::new(
                    coordinate(&map.numerators[0])?,
                    coordinate(&map.numerators[1])?,
                )
                .into())
            }
            CurvePointData2::Similarity(image) => {
                let (source, transform) = image.strict_definition().ok_or_else(blocked)?;
                source
                    .rational_image(&RationalMap2::from_similarity(transform))?
                    .rational_image(map)
            }
            CurvePointData2::Endpoint(endpoint) => {
                let source = ExactCurveError::decided(
                    CurveOperation2::Construction,
                    endpoint.resolve(&CurveContext::STRICT).map_err(|cause| {
                        ExactCurveError::invalid_unattributed(CurveOperation2::Construction, cause)
                    })?,
                )?
                .ok_or_else(blocked)?;
                source.rational_image(map)
            }
            _ => Err(blocked()),
        }
    }
}

#[cfg(test)]
mod tests;
