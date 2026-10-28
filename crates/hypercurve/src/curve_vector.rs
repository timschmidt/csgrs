//! Queries of correlated vector coordinates reuse the point-form owner.

use super::*;
use std::cmp::Ordering;

pub(super) fn order_sign(order: Ordering) -> RealSign {
    match order {
        Ordering::Less => RealSign::Negative,
        Ordering::Equal => RealSign::Zero,
        Ordering::Greater => RealSign::Positive,
    }
}

impl CurveVector2 {
    /// Returns coordinates of a vector positively proportional to this vector.
    ///
    /// This preserves orientation and zeros, but does not promise the original
    /// derivative magnitude. For a selected parallel, the certified nonzero
    /// speed-ratio sign selects the orientation of its source velocity; only a
    /// positive factor is discarded. Other carriers retain their coordinates.
    pub fn direction_image(&self) -> ExactCurveResult<CurvePoint2> {
        let CurveVectorData2::SelectedParallel(parallel) = &self.data else {
            return self.coordinate_image();
        };
        let sign = Real::from(if parallel.ratio_sign == RealSign::Negative {
            -1
        } else {
            1
        });
        let result = resolve_certified_operation(&crate::policy::principal_context(), |policy| {
            parallel.velocity.coordinate_image(&sign, policy)
        })
        .map(crate::CurveOutcome::into_value)
        .map_err(|cause| {
            ExactCurveError::invalid_unattributed(CurveOperation2::Evaluation, cause)
        })?;
        ExactCurveError::decided(CurveOperation2::Evaluation, result)
    }

    /// Returns this vector's coordinates as one exact point image.
    ///
    /// This owns the original joint source, not two independently reconstructed
    /// roots. Selected native rational derivatives retain their parameter field.
    /// Radical parallel speed ratios currently support signs but not this view.
    pub fn coordinate_image(&self) -> ExactCurveResult<CurvePoint2> {
        match &self.data {
            CurveVectorData2::Represented(value) => {
                Ok(Point2::new(value.dx().clone(), value.dy().clone()).into())
            }
            CurveVectorData2::Joint(point) => Ok(point.as_ref().clone()),
            CurveVectorData2::Selected {
                vector,
                chart_factor,
            } => {
                let result =
                    resolve_certified_operation(&crate::policy::principal_context(), |policy| {
                        vector.coordinate_image(chart_factor, policy)
                    })
                    .map(crate::CurveOutcome::into_value)
                    .map_err(|cause| {
                        ExactCurveError::invalid_unattributed(CurveOperation2::Evaluation, cause)
                    })?;
                ExactCurveError::decided(CurveOperation2::Evaluation, result)
            }
            CurveVectorData2::SelectedParallel(_) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                crate::UncertaintyReason::Unsupported,
            )),
        }
    }

    /// Compares `a*dx + b*dy` with an exact authored value.
    /// Original joint coordinates and denominator guards remain authoritative.
    /// Selected parallel vectors currently admit only a zero right-hand side.
    pub fn compare_linear_form(
        &self,
        a: &Real,
        b: &Real,
        rhs: &Real,
    ) -> ExactCurveResult<Ordering> {
        if let CurveVectorData2::SelectedParallel(parallel) = &self.data {
            if !rhs.definitely_zero() {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Classification,
                    CurveFamily2::RationalBezier,
                    crate::UncertaintyReason::Unsupported,
                ));
            }
            let vector = Self::selected(parallel.velocity.clone());
            let order = vector.compare_linear_form(a, b, rhs)?;
            return Ok(if parallel.ratio_sign == RealSign::Negative {
                order.reverse()
            } else {
                order
            });
        }
        self.coordinate_image()?
            .affine_form(a.clone(), b.clone(), Real::zero())
            .compare_to_real(rhs)
    }

    /// Constructs a rational query of both vector coordinates in their common
    /// source field. Original zero denominators are rejected before reduction.
    /// The selected-parallel radical carrier has no such view yet.
    pub fn rational_form(
        &self,
        numerator: hypersolve::BivariatePolynomial,
        denominator: hypersolve::BivariatePolynomial,
    ) -> ExactCurveResult<crate::CurvePointRationalForm2> {
        self.coordinate_image()?
            .rational_form(numerator, denominator)
    }

    pub(crate) fn joint(point: CurvePoint2) -> Self {
        Self {
            data: CurveVectorData2::Joint(Box::new(point)),
        }
    }
}
