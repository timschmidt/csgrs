//! Shared rational point images keep construction and pole evidence together.

use hypersolve::BivariatePolynomial;

use crate::{
    Aabb2, Classification, CurveContext, CurveOperation2, CurvePoint2, CurvePointData2,
    CurveResult, ExactCurveError, ExactCurveResult, RationalMap2, UncertaintyReason,
};

#[derive(Debug, PartialEq)]
pub(crate) struct RationalPointImage2 {
    pub(crate) source: CurvePoint2,
    pub(crate) map: RationalMap2,
    policy: CurveContext,
}

impl RationalPointImage2 {
    pub(crate) fn construct(
        source: &CurvePoint2,
        map: &RationalMap2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        crate::policy::resolve_certified_operation(policy, |policy| {
            Self::validate_map(source, map, policy).map(|sign| {
                sign.map(|_| {
                    CurvePoint2(CurvePointData2::RationalImage(std::sync::Arc::new(Self {
                        source: source.clone(),
                        map: map.clone(),
                        policy: policy.retained_object_policy(),
                    })))
                })
            })
        })
        .map_err(|cause| {
            ExactCurveError::invalid_unattributed(CurveOperation2::Construction, cause)
        })
        .and_then(|outcome| {
            ExactCurveError::decided(CurveOperation2::Construction, outcome.into_value())
        })
    }

    pub(crate) fn validate_map(
        source: &CurvePoint2,
        map: &RationalMap2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let sign = crate::bezier_offset::point_rational_denominator_sign(
            source,
            map.denominator(),
            policy,
        )?;
        // The shared scope also reports exhaustion, but a constructor must
        // not publish a point between the reached limit and that boundary.
        Ok(if hypersolve::work_budget::exhaustion().is_some() {
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        } else {
            sign.map(|_| ())
        })
    }

    /// Every consuming query checks the original guard before cancellation or
    /// field reduction. A failed attempt never becomes retained evidence.
    pub(crate) fn validate(&self, policy: &CurveContext) -> CurveResult<Classification<()>> {
        if !policy.accepts_retained_policy(self.policy) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        if self.policy.selects_approximate_512() {
            policy.observe_approximate_512();
        }
        Self::validate_map(&self.source, &self.map, policy)
    }

    pub(crate) fn pullback(
        &self,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(BivariatePolynomial, BivariatePolynomial)>> {
        if let Classification::Uncertain(reason) = self.validate(policy)? {
            return Ok(Classification::Uncertain(reason));
        }
        Ok(self.map.pullback(numerator, denominator).map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            Classification::Decided,
        ))
    }

    pub(crate) fn bounds(&self, steps: usize, policy: &CurveContext) -> Classification<Aabb2> {
        match self.validate(policy) {
            Ok(Classification::Decided(())) => {}
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Predicate),
        }
        let source = match crate::bezier_offset::algebraic_chord_endpoint_local_bounds_refined(
            &self.source,
            steps,
            policy,
        ) {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        self.map.bounds(&source).map_or(
            Classification::Uncertain(UncertaintyReason::Predicate),
            Classification::Decided,
        )
    }
}

impl CurvePoint2 {
    /// Identity and constant queries still consume the image's admission.
    pub(crate) fn validate_rational_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        match &self.0 {
            CurvePointData2::RationalImage(image) => image.validate(policy),
            _ => Ok(Classification::Decided(())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Axis2, BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
        BezierParameterPolynomial, CurveCertainty, Real,
    };
    use std::cmp::Ordering;

    fn source() -> CurvePoint2 {
        let alpha = BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new((Real::from(3) / Real::from(4)).unwrap(), 1.into())
                .unwrap(),
        )
        .unwrap();
        crate::CurveParameter2::from(BezierParameter2::Algebraic(alpha))
            .affine_point(1.into(), 0.into(), 0.into(), 0.into())
            .unwrap()
    }
    fn identity() -> RationalMap2 {
        RationalMap2::new(
            [
                BivariatePolynomial::new(vec![vec![0.into()], vec![1.into()]]),
                BivariatePolynomial::new(vec![vec![0.into(), 1.into()]]),
            ],
            BivariatePolynomial::new(vec![vec![1.into()]]),
        )
    }
    #[test]
    fn rational_point_images_retain_qualification_through_identity_queries() {
        let source = source();
        // A synthetic already-qualified dependency exercises provenance even
        // when the local map and consumed equality are exact identities.
        let qualified = CurvePoint2(CurvePointData2::RationalImage(std::sync::Arc::new(
            RationalPointImage2 {
                source: source.clone(),
                map: identity(),
                policy: CurveContext::APPROXIMATE_512,
            },
        )));
        assert!(qualified.definition().is_err());
        for query in [
            qualified
                .coincides_with_with_policy(&qualified, &CurveContext::STRICT)
                .map(|v| v.map(|_| ())),
            qualified
                .compare_coordinate_with_policy(&qualified, Axis2::X, &CurveContext::STRICT)
                .unwrap()
                .map(|v| v.map(|_| ())),
        ] {
            assert!(matches!(
                query.into_value(),
                Classification::Uncertain(UncertaintyReason::Unsupported)
            ));
        }
        let exact = crate::provisional(|| source.rational_image(&identity()).unwrap())
            .certified()
            .unwrap();
        assert!(exact.definition().is_ok());
        let approved = qualified
            .compare_coordinate_with_policy(&qualified, Axis2::X, &CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(approved.value, Classification::Decided(Ordering::Equal));
        assert_eq!(approved.certainty, CurveCertainty::Approximate512Consumed);
        let fresh = exact
            .compare_coordinate_with_policy(&exact, Axis2::X, &CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(fresh.certainty, CurveCertainty::Certified);
    }
    #[test]
    fn rational_point_native_replay_stays_strict_inside_provisional() {
        let qualified = CurvePoint2(CurvePointData2::RationalImage(std::sync::Arc::new(
            RationalPointImage2 {
                source: source(),
                map: identity(),
                policy: CurveContext::APPROXIMATE_512,
            },
        )));
        let replay = crate::provisional(|| {
            crate::CurvePointDefinition2::RationalImage {
                source: qualified,
                map: identity(),
            }
            .into_point()
        });
        assert!(replay.as_unverified().is_err());
    }
    #[test]
    fn rational_point_constant_affine_queries_keep_qualification() {
        let qualified = CurvePoint2(CurvePointData2::RationalImage(std::sync::Arc::new(
            RationalPointImage2 {
                source: source(),
                map: identity(),
                policy: CurveContext::APPROXIMATE_512,
            },
        )));
        let zero = Real::zero();
        let order = crate::bezier_offset::point_affine_form_order_to_real(
            &qualified,
            &zero,
            &zero,
            &zero,
            &CurveContext::STRICT,
        )
        .unwrap();
        assert!(matches!(
            order,
            Classification::Uncertain(UncertaintyReason::Unsupported)
        ));
    }
}
