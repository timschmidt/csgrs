//! Affine query forms retain the correlation of a point's selected coordinates.

use std::cmp::Ordering;

use crate::{
    Axis2, Classification, CurveContext, CurveOperation2, CurvePoint2, CurveResult,
    ExactCurveError, ExactCurveResult, Real,
};

/// The exact query `a*x + b*y + c` of one retained planar point.
///
/// This is a source-bound query view, not a coefficient scalar or independent
/// coordinate reconstruction. Comparisons reuse the original point's roots and
/// normal-sheet evidence. Unavailable proofs remain typed; an enclosure does
/// not replace the exact value. The view owns a clone of its immutable source.
#[derive(Clone, Debug)]
pub struct CurvePointAffineForm2 {
    source: CurvePoint2,
    coefficients: [Real; 3],
}

impl CurvePoint2 {
    /// Retains the affine query `a*x + b*y + c` without evaluating coordinates.
    pub fn affine_form(&self, a: Real, b: Real, c: Real) -> CurvePointAffineForm2 {
        CurvePointAffineForm2 {
            source: self.clone(),
            coefficients: [a, b, c],
        }
    }
}

impl CurvePointAffineForm2 {
    /// Retains this affine value as a parameter in its existing selected field.
    ///
    /// Authored constants remain authored; other values retain their defining
    /// field and selected sources. Polynomial-fiber points construct a local
    /// image relation and select its root from source enclosures, without a
    /// global norm through the base root. The affine recipe remains available
    /// for replay. Unsupported fields fail explicitly.
    /// A retained supporting-line intersection preserves both source lines
    /// even when their endpoints also admit scalar coordinate expressions.
    pub fn parameter(&self) -> ExactCurveResult<crate::CurveParameter2> {
        resolve(CurveOperation2::Evaluation, || {
            crate::policy::resolve_certified_operation(
                &crate::policy::principal_context(),
                |policy| {
                    crate::bezier_offset::point_affine_form_parameter(
                        &self.source,
                        &self.coefficients,
                        policy,
                    )
                },
            )
            .map(crate::CurveOutcome::into_value)
        })
    }

    /// Borrows the complete correlated source point.
    pub const fn source(&self) -> &CurvePoint2 {
        &self.source
    }

    /// Borrows `[a, b, c]` in `a*x + b*y + c`.
    pub const fn coefficients(&self) -> &[Real; 3] {
        &self.coefficients
    }

    /// Compares this exact form with an authored scalar under the active policy.
    pub fn compare_to_real(&self, value: &Real) -> ExactCurveResult<Ordering> {
        resolve(CurveOperation2::Classification, || {
            crate::policy::resolve_certified_operation(
                &crate::policy::principal_context(),
                |policy| self.order_to_real(value, policy),
            )
            .map(crate::CurveOutcome::into_value)
        })
    }

    /// Compares two forms without forgetting either source's selected evidence.
    pub fn compare(&self, other: &Self) -> ExactCurveResult<Ordering> {
        resolve(CurveOperation2::Classification, || {
            crate::policy::resolve_certified_operation(
                &crate::policy::principal_context(),
                |policy| self.order(other, policy),
            )
            .map(crate::CurveOutcome::into_value)
        })
    }

    /// Returns a conservative closed interval containing the exact form.
    ///
    /// No width or independent scalar view is promised. Refinement, source
    /// failures and instrumented solver-work exhaustion retain their meaning.
    pub fn bounds(&self) -> ExactCurveResult<(Real, Real)> {
        resolve(CurveOperation2::Evaluation, || {
            crate::policy::resolve_certified_operation(
                &crate::policy::principal_context(),
                |policy| {
                    crate::bezier_offset::point_affine_form_bounds(
                        &self.source,
                        &self.coefficients,
                        policy,
                    )
                },
            )
            .map(crate::CurveOutcome::into_value)
        })
    }

    fn order_to_real(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        let [a, b, c] = &self.coefficients;
        crate::bezier_offset::point_affine_form_order_to_real(
            &self.source,
            a,
            b,
            &(value - c),
            policy,
        )
    }

    fn order(&self, other: &Self, policy: &CurveContext) -> CurveResult<Classification<Ordering>> {
        let [a, b, c] = &self.coefficients;
        let [d, e, f] = &other.coefficients;
        if self.source.shares_storage(&other.source) || self.source == other.source {
            // Subtract before elimination so correlated cancellation survives.
            return crate::bezier_offset::point_affine_form_order_to_real(
                &self.source,
                &(a - d),
                &(b - e),
                &(f - c),
                policy,
            );
        }
        if let Some(point) = other.source.coordinates() {
            return self.order_to_real(
                &Real::dot3_refs([d, e, f], [point.x(), point.y(), &Real::one()]),
                policy,
            );
        }
        if let Some(point) = self.source.coordinates() {
            return Ok(other
                .order_to_real(
                    &Real::dot3_refs([a, b, c], [point.x(), point.y(), &Real::one()]),
                    policy,
                )?
                .map(Ordering::reverse));
        }
        if self.coefficients == other.coefficients {
            return crate::bezier_offset::algebraic_chord_points_linear_order(
                &self.source,
                &other.source,
                a,
                b,
                policy,
            );
        }
        let first = match crate::bezier_offset::point_affine_form_image(
            &self.source,
            &self.coefficients,
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let second = match crate::bezier_offset::point_affine_form_image(
            &other.source,
            &other.coefficients,
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        crate::bezier_offset::algebraic_chord_point_coordinate_order(
            &first,
            &second,
            Axis2::X,
            policy,
        )
    }
}

fn resolve<T>(
    kind: CurveOperation2,
    operation: impl FnOnce() -> CurveResult<Classification<T>>,
) -> ExactCurveResult<T> {
    operation()
        .map_err(|cause| ExactCurveError::invalid_unattributed(kind, cause))
        .and_then(|value| ExactCurveError::decided(kind, value))
}
