//! A scalar query retains its correlated point and checked field enclosure.

use crate::bezier_offset::PointFormSource2;
use crate::selected_field::{self as field, Enclosure, Value};
use crate::selected_scalar::SelectedScalar2;
use crate::{Classification, CurveContext, CurveResult, Real, UncertaintyReason};
use hypersolve::bivariate_arithmetic::{bivariate_add, bivariate_scale, bivariate_subtract};
use hypersolve::{BivariatePolynomial, RealInterval};
use std::{cmp::Ordering, sync::Arc};

#[derive(Clone)]
pub(crate) struct SelectedQueryScalar2 {
    source: Arc<PointFormSource2>,
    enclosure: Enclosure,
    complement: bool,
    bounds: RealInterval,
}

impl std::fmt::Debug for SelectedQueryScalar2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SelectedQueryScalar2")
            .field("bounds", &self.bounds)
            .field("complement", &self.complement)
            .finish_non_exhaustive()
    }
}

impl PartialEq for SelectedQueryScalar2 {
    fn eq(&self, other: &Self) -> bool {
        self.complement == other.complement && Arc::ptr_eq(&self.source, &other.source)
    }
}

impl SelectedQueryScalar2 {
    pub(crate) fn new(
        source: PointFormSource2,
        value: Value,
        policy: &CurveContext,
    ) -> field::Result<Self> {
        let enclosure = field::decided(value.enclosed(policy)?)?;
        Ok(Self {
            source: Arc::new(source),
            bounds: enclosure.interval().clone(),
            enclosure,
            complement: false,
        })
    }

    pub(crate) fn replay(
        source: PointFormSource2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let strict = policy.strict_counterpart();
        field::finish(|| {
            let value = crate::bezier_offset::selected_field_query(&source, &strict)?;
            Self::new(source, value, &strict)
        })
    }

    #[cfg(feature = "svg")]
    pub(crate) fn replay_storage_key(&self) -> (bool, *const ()) {
        (self.complement, Arc::as_ptr(&self.source).cast())
    }

    pub(crate) fn value(&self) -> field::Result<Value> {
        let value = self.enclosure.value();
        Ok(if self.complement {
            value.field().constant(Real::one())?.subtract(value)?
        } else {
            value.clone()
        })
    }

    pub(crate) fn query(&self) -> PointFormSource2 {
        let mut query = self.source.as_ref().clone();
        if self.complement {
            query.numerator = bivariate_subtract(&query.denominator, &query.numerator);
        }
        query
    }

    pub(crate) fn polynomial_sign(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<hyperreal::RealSign>> {
        field::finish(|| {
            field::decided(field::polynomial(coefficients, &self.value()?)?.sign(policy)?)
        })
    }

    pub(crate) fn compare(
        &self,
        other: &SelectedScalar2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        field::finish(|| {
            let [first, second] = field::joined(
                &self.value()?,
                &crate::bezier_offset::selected_field_scalar(other, policy)?,
                policy,
            )?;
            Ok(
                match field::decided(first.subtract(&second)?.sign(policy)?)? {
                    hyperreal::RealSign::Negative => Ordering::Less,
                    hyperreal::RealSign::Zero => Ordering::Equal,
                    hyperreal::RealSign::Positive => Ordering::Greater,
                },
            )
        })
    }

    pub(crate) fn isolating_bounds(&self) -> (&Real, &Real) {
        (&self.bounds.lower, &self.bounds.upper)
    }

    pub(crate) fn unit_complement(&self) -> Self {
        Self {
            complement: !self.complement,
            bounds: RealInterval {
                lower: Real::one() - &self.bounds.upper,
                upper: Real::one() - &self.bounds.lower,
            },
            ..self.clone()
        }
    }

    pub(crate) fn refined(
        &self,
        steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        field::finish(|| {
            let enclosure = field::decided(self.enclosure.refined(steps, policy)?)?;
            let bounds = enclosure.interval().clone();
            let value = Self {
                source: Arc::clone(&self.source),
                enclosure,
                bounds,
                complement: false,
            };
            Ok(if self.complement {
                value.unit_complement()
            } else {
                value
            })
        })
    }

    pub(crate) fn affine_image_unbounded(
        &self,
        scale: &Real,
        offset: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        self.projective_image_unbounded(
            &[offset.clone(), scale.clone()],
            &[Real::one(), Real::zero()],
            policy,
        )
    }

    pub(crate) fn projective_image_unbounded(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        field::finish(|| {
            let value = self.value()?;
            let image = field::decided(
                field::polynomial(numerator, &value)?
                    .divide(&field::polynomial(denominator, &value)?, policy)?,
            )?;
            let query = self.query();
            let map = |coefficients: &[Real; 2]| {
                bivariate_add(
                    &bivariate_scale(query.denominator.clone(), &coefficients[0]),
                    &bivariate_scale(query.numerator.clone(), &coefficients[1]),
                )
            };
            // Keep the original query denominator as a guard even if the chart
            // moves it to the numerator or the image is identically constant.
            let guard = &query.denominator;
            let source = PointFormSource2::new(
                query.source.clone(),
                hypersolve::bivariate_arithmetic::bivariate_multiply(&map(numerator), guard),
                hypersolve::bivariate_arithmetic::bivariate_multiply(&map(denominator), guard),
            );
            Self::new(source, image, policy)
        })
    }

    pub(crate) fn promoted_bezier_parameter_complete(
        &self,
        _policy: &CurveContext,
    ) -> CurveResult<Classification<crate::BezierParameter2>> {
        Ok(if self.bounds.lower == self.bounds.upper {
            Classification::Decided(crate::BezierParameter2::Exact(self.bounds.lower.clone()))
        } else {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        })
    }

    /// Evaluates a rational curve in this query's original joint point field.
    pub(crate) fn rational_point(
        &self,
        curve: &crate::RationalBezier2,
        policy: &CurveContext,
    ) -> crate::ExactCurveResult<crate::CurvePoint2> {
        let basis = curve.homogeneous_power_basis().map_err(|cause| {
            crate::ExactCurveError::invalid_unattributed(crate::CurveOperation2::Evaluation, cause)
        })?;
        self.rational_image(
            &basis.x_numerator,
            &basis.y_numerator,
            &basis.weight,
            policy,
        )
    }

    /// Shares the original rational-jet recurrence; only the final values are
    /// pulled into the correlated query field. The source polynomials being
    /// differentiated are never reduced by a relation at the selected point.
    pub(crate) fn rational_derivatives(
        &self,
        curve: &crate::RationalBezier2,
        max_order: usize,
        chart_scale: &Real,
        policy: &CurveContext,
    ) -> crate::ExactCurveResult<Vec<crate::CurveVector2>> {
        let basis = curve.homogeneous_power_basis().map_err(|cause| {
            crate::ExactCurveError::invalid_unattributed(crate::CurveOperation2::Evaluation, cause)
        })?;
        let mut factor = Real::one();
        let images = crate::bezier_algebraic_image::map_rational_derivatives_from_power_basis(
            basis.x_numerator.clone(),
            basis.y_numerator.clone(),
            basis.weight.clone(),
            max_order,
            Ok,
            |image| {
                factor = &factor * chart_scale;
                let scale = |coefficients: &[Real]| {
                    coefficients
                        .iter()
                        .map(|value| value * &factor)
                        .collect::<Vec<_>>()
                };
                self.rational_image(
                    &scale(&image.dx_numerator),
                    &scale(&image.dy_numerator),
                    &image.denominator,
                    policy,
                )
                .map(crate::CurveVector2::joint)
                .map(Classification::Decided)
            },
        )?;
        crate::ExactCurveError::decided(crate::CurveOperation2::Evaluation, images)
    }

    fn rational_image(
        &self,
        x_numerator: &[Real],
        y_numerator: &[Real],
        denominator: &[Real],
        policy: &CurveContext,
    ) -> crate::ExactCurveResult<crate::CurvePoint2> {
        let query = self.query();
        let chart = crate::RationalMap2::new(
            [
                query.numerator.clone(),
                BivariatePolynomial::new(vec![vec![Real::zero()]]),
            ],
            query.denominator.clone(),
        );
        let pullback = |coefficients: &[Real]| {
            chart.pullback(
                &BivariatePolynomial::new(coefficients.iter().map(|c| vec![c.clone()]).collect()),
                &BivariatePolynomial::new(denominator.iter().map(|c| vec![c.clone()]).collect()),
            )
        };
        let Some((x, xw)) = pullback(x_numerator) else {
            return Err(crate::ExactCurveError::blocked(
                crate::CurveOperation2::Evaluation,
                crate::CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            ));
        };
        let Some((y, yw)) = pullback(y_numerator) else {
            return Err(crate::ExactCurveError::blocked(
                crate::CurveOperation2::Evaluation,
                crate::CurveFamily2::RationalBezier,
                UncertaintyReason::Unsupported,
            ));
        };
        use hypersolve::bivariate_arithmetic::bivariate_multiply as mul;
        let guard = &query.denominator;
        query.source.rational_image_with_policy(
            &crate::RationalMap2::new(
                [mul(&mul(&x, &yw), guard), mul(&mul(&y, &xw), guard)],
                mul(&mul(&xw, &yw), guard),
            ),
            policy,
        )
    }
}
