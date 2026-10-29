//! Rational query forms preserve the joint point on which they are evaluated.

use std::cmp::Ordering;

use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::{bivariate_multiply, bivariate_scale, bivariate_subtract};

use crate::{
    Classification, CurveContext, CurveOperation2, CurvePoint2, CurveResult, ExactCurveError,
    ExactCurveResult, Real,
};

mod arithmetic;

/// The source-bound rational query `numerator(x,y) / denominator(x,y)`.
///
/// Both polynomials are evaluated at the same retained point. In particular,
/// a selected fiber keeps its base root and fiber selector. The original
/// denominator must be nonzero even when the quotient admits cancellation.
/// This query view does not reconstruct independent coordinate roots.
#[derive(Clone)]
pub struct CurvePointRationalForm2 {
    source: CurvePoint2,
    numerator: BivariatePolynomial,
    denominator: BivariatePolynomial,
    // A derived common-field query keeps both original source/denominator
    // authorities. Their checked immutable guards must not disappear when a
    // coordinate map is pulled back or a constant operand is embedded.
    operands: Option<std::sync::Arc<[Self; 2]>>,
}

impl std::fmt::Debug for CurvePointRationalForm2 {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The operand DAG can share the same child many times. A diagnostic
        // must not recursively expand it into an exponentially larger tree.
        // Callers can inspect original guards explicitly through operands().
        formatter
            .debug_struct("CurvePointRationalForm2")
            .field("source", &self.source)
            .field("numerator", &self.numerator)
            .field("denominator", &self.denominator)
            .field("has_operands", &self.operands.is_some())
            .finish()
    }
}

impl CurvePoint2 {
    /// Retains a rational query after checking its original denominator.
    ///
    /// Coefficients use ascending powers: `[i][j]` multiplies `x^i*y^j`.
    /// Poles return [`crate::CurveError::ZeroRationalPointDenominator`], and unavailable
    /// selected-field proofs remain typed. No cancellation removes a pole.
    pub fn rational_form(
        &self,
        numerator: BivariatePolynomial,
        denominator: BivariatePolynomial,
    ) -> ExactCurveResult<CurvePointRationalForm2> {
        let form = CurvePointRationalForm2 {
            source: self.clone(),
            numerator,
            denominator,
            operands: None,
        };
        resolve(CurveOperation2::Construction, |policy| {
            crate::bezier_offset::point_rational_denominator_sign(
                &form.source,
                &form.denominator,
                policy,
            )
        })?;
        Ok(form)
    }
}

impl CurvePointRationalForm2 {
    /// Original operands of a binary query, when this form was constructed by
    /// arithmetic. Their source points and original pole guards remain owned.
    pub fn operands(&self) -> Option<(&Self, &Self)> {
        self.operands.as_deref().map(|pair| (&pair[0], &pair[1]))
    }
    /// Borrows the complete correlated source point.
    pub const fn source(&self) -> &CurvePoint2 {
        &self.source
    }

    /// Borrows the original numerator, before any field reduction.
    pub const fn numerator(&self) -> &BivariatePolynomial {
        &self.numerator
    }

    /// Borrows the original denominator whose nonzero value is required.
    pub const fn denominator(&self) -> &BivariatePolynomial {
        &self.denominator
    }

    /// Compares with an authored value without discarding source correlation.
    pub fn compare_to_real(&self, value: &Real) -> ExactCurveResult<Ordering> {
        resolve(CurveOperation2::Classification, |policy| {
            let residual = bivariate_subtract(
                &self.numerator,
                &bivariate_scale(self.denominator.clone(), value),
            );
            crate::bezier_offset::point_rational_form_sign(
                &self.source,
                &residual,
                &self.denominator,
                policy,
            )
            .map(|sign| sign.map(sign_order))
        })
    }

    /// Compares two exact queries, retaining both original source authorities.
    pub fn compare(&self, other: &Self) -> ExactCurveResult<Ordering> {
        resolve(CurveOperation2::Classification, |policy| {
            if self.source.shares_storage(&other.source) || self.source == other.source {
                // Subtract in the joint field before constructing any image.
                let residual = bivariate_subtract(
                    &bivariate_multiply(&self.numerator, &other.denominator),
                    &bivariate_multiply(&other.numerator, &self.denominator),
                );
                return crate::bezier_offset::point_rational_form_sign(
                    &self.source,
                    &residual,
                    &bivariate_multiply(&self.denominator, &other.denominator),
                    policy,
                )
                .map(|sign| sign.map(sign_order));
            }
            let first = match self.parameter_with_policy(policy)? {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let second = match other.parameter_with_policy(policy)? {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            first.cmp_by_refinement(&second, policy)
        })
    }

    /// Retains this rational value as a source-bound exact scalar parameter.
    ///
    /// The original point and denominator remain available for native replay.
    /// A zero denominator stays invalid even if algebraic cancellation is possible.
    pub fn parameter(&self) -> ExactCurveResult<crate::CurveParameter2> {
        resolve(CurveOperation2::Evaluation, |policy| {
            crate::selected_query_scalar::SelectedQueryScalar2::replay(
                crate::bezier_offset::PointFormSource2::new(
                    self.source.clone(),
                    self.numerator.clone(),
                    self.denominator.clone(),
                ),
                policy,
            )
            .map(|value| value.map(crate::CurveParameter2::from_polynomial_query))
        })
    }

    pub(crate) fn retained_query(source: crate::bezier_offset::PointFormSource2) -> Self {
        Self {
            source: source.source,
            numerator: source.numerator,
            denominator: source.denominator,
            operands: None,
        }
    }

    /// Returns a conservative exact enclosure, with no promised width.
    ///
    /// An image relation, when needed, is selected using this same source.
    /// The enclosure never replaces that source or becomes point authority.
    pub fn bounds(&self) -> ExactCurveResult<(Real, Real)> {
        resolve(CurveOperation2::Evaluation, |policy| {
            Ok(match self.parameter_with_policy(policy)? {
                Classification::Decided(parameter) => match parameter.scalar_bounds() {
                    Some((lower, upper)) => Classification::Decided((lower.clone(), upper.clone())),
                    None => Classification::Uncertain(crate::UncertaintyReason::Unsupported),
                },
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            })
        })
    }

    fn parameter_with_policy(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::CurveParameter2>> {
        crate::bezier_offset::point_rational_form_parameter(
            &self.source,
            &self.numerator,
            &self.denominator,
            policy,
        )
    }
}

fn sign_order(sign: hyperreal::RealSign) -> Ordering {
    match sign {
        hyperreal::RealSign::Negative => Ordering::Less,
        hyperreal::RealSign::Zero => Ordering::Equal,
        hyperreal::RealSign::Positive => Ordering::Greater,
    }
}

fn resolve<T>(
    kind: CurveOperation2,
    operation: impl Fn(&CurveContext) -> CurveResult<Classification<T>>,
) -> ExactCurveResult<T> {
    crate::policy::resolve_certified_operation(&crate::policy::principal_context(), operation)
        .map(crate::CurveOutcome::into_value)
        .map_err(|cause| ExactCurveError::invalid_unattributed(kind, cause))
        .and_then(|value| ExactCurveError::decided(kind, value))
}
