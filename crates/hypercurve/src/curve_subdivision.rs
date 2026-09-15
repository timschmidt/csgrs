//! Exact restrictions in an authored curve's parameter chart.

use super::*;
use crate::curve_support::CurveSupport2;
use crate::policy::{PolicyClassificationCache, resolve_cached_classification};
use crate::{BezierSplitFragment2, CurveParameterRange2, UncertaintyReason};
use std::cmp::Ordering;

/// A flat restriction of one authored source. Repeated restrictions share
/// that source, its span preparation and the surviving endpoint witnesses.
#[derive(Debug)]
pub(super) struct CurveSourceRange2 {
    pub(super) source: Curve2,
    pub(super) range: CurveParameterRange2,
    /// Ascending source order, independent of traversal.
    pub(super) endpoints: [CurvePoint2; 2],
    pub(super) reversed: bool,
    validation: PolicyClassificationCache<()>,
    spans: PolicyEvaluationCache<Vec<CurveSourceSpan2>>,
}

impl PartialEq for CurveSourceRange2 {
    fn eq(&self, other: &Self) -> bool {
        self.source == other.source && self.range == other.range && self.reversed == other.reversed
    }
}

/// One retained span and the affine map from its support parameter to the
/// complete authored source chart. A selected endpoint does not redefine it.
#[derive(Clone, Debug)]
pub(crate) struct CurveSourceSpan2 {
    pub(crate) fragment: BezierSplitFragment2,
    pub(crate) source_scale: Real,
    pub(crate) source_offset: Real,
}

fn subdivision_error(family: CurveFamily2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Subdivision, family, cause)
}

fn decided<T>(value: Classification<T>, family: CurveFamily2) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            reason,
        )),
    }
}

fn compare(
    first: &CurveParameter2,
    second: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    decided(
        first
            .cmp_by_refinement(second, policy)
            .map_err(|cause| subdivision_error(family, cause))?,
        family,
    )
}

impl Curve2 {
    pub(super) fn split_at_parameter(
        &self,
        parameter: CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<(Self, Self)> {
        if self.geometry().is_some()
            && let Some(BezierParameter2::Exact(parameter)) = parameter.as_bezier_parameter()
        {
            return self.split_at_raw(parameter.clone(), policy);
        }
        let parameter = self.subdivision_parameter(parameter);
        let domain = self.parameter_domain();
        let family = self.family();
        if compare(domain.start(), &parameter, family, policy)? != Ordering::Less
            || compare(&parameter, domain.end(), family, policy)? != Ordering::Less
        {
            return Err(subdivision_error(family, CurveError::InvalidCurveParameter));
        }
        let lower =
            self.subcurve_at_parameters(domain.start().clone(), parameter.clone(), policy)?;
        let upper = self.subcurve_at_parameters(parameter, domain.end().clone(), policy)?;
        let reversed = self.source_range().map_or_else(
            || {
                self.retained_fragment()
                    .is_some_and(BezierSplitFragment2::source_is_reversed)
            },
            |range| range.reversed,
        );
        Ok(if reversed {
            (upper, lower)
        } else {
            (lower, upper)
        })
    }

    pub(super) fn subcurve_at_parameters(
        &self,
        start: CurveParameter2,
        end: CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if self.geometry().is_some()
            && let (Some(BezierParameter2::Exact(start)), Some(BezierParameter2::Exact(end))) =
                (start.as_bezier_parameter(), end.as_bezier_parameter())
        {
            return self.subcurve_with_policy(start.clone(), end.clone(), policy);
        }
        let Some(fragment) = self.retained_fragment() else {
            return self.restrict_authored_source(start, end, policy);
        };
        let start = self.subdivision_parameter(start);
        let end = self.subdivision_parameter(end);
        let domain = self.parameter_domain();
        let family = self.family();
        let start_order = compare(&start, domain.start(), family, policy)?;
        let end_order = compare(&end, domain.end(), family, policy)?;
        if start_order.is_lt()
            || end_order.is_gt()
            || compare(&start, &end, family, policy)? != Ordering::Less
        {
            return Err(subdivision_error(family, CurveError::InvalidCurveRange));
        }
        if start_order.is_eq() && end_order.is_eq() {
            return Ok(self.clone());
        }
        let start_point = self
            .point_at_parameter_with_policy(&start, CurveParameterSide2::Right, policy)
            .map_err(|error| error.with_operation(CurveOperation2::Subdivision))?;
        let end_point = self
            .point_at_parameter_with_policy(&end, CurveParameterSide2::Left, policy)
            .map_err(|error| error.with_operation(CurveOperation2::Subdivision))?;
        CurveSupport2::from_fragment(fragment)
            .restrict_certified(
                CurveParameterRange2::new_validated(start, end),
                Some([start_point, end_point]),
                fragment.source_is_reversed(),
                policy,
            )
            .map(Self::from_retained_fragment)
            .map_err(|cause| subdivision_error(family, cause))
    }

    fn subdivision_parameter(&self, parameter: CurveParameter2) -> CurveParameter2 {
        if matches!(
            self.retained_fragment(),
            Some(BezierSplitFragment2::AlgebraicCuspSemicircle(_))
        ) && let Some(BezierParameter2::Exact(value)) = parameter.as_bezier_parameter()
        {
            return CurveParameter2::from_algebraic_cusp(
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(value.clone()),
            );
        }
        parameter
    }

    pub(super) fn source_range(&self) -> Option<&CurveSourceRange2> {
        match &self.data.carrier {
            CurveCarrier2::SourceRange(range) => Some(range),
            CurveCarrier2::Native(_) | CurveCarrier2::Restricted(_) => None,
        }
    }

    pub(super) fn restrict_authored_source(
        &self,
        start: CurveParameter2,
        end: CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let family = self.family();
        let domain = self.parameter_domain();
        let start_order = compare(&start, domain.start(), family, policy)?;
        let end_order = compare(&end, domain.end(), family, policy)?;
        if start_order.is_lt()
            || end_order.is_gt()
            || compare(&start, &end, family, policy)? != Ordering::Less
        {
            return Err(subdivision_error(family, CurveError::InvalidCurveRange));
        }
        if start_order.is_eq() && end_order.is_eq() {
            return Ok(self.clone());
        }
        let (source, reversed) = self
            .source_range()
            .map_or((self, false), |range| (&range.source, range.reversed));
        debug_assert!(
            source.geometry().is_some(),
            "restrictions share an authored root"
        );
        let endpoints = [
            self.point_at_parameter_with_policy(&start, CurveParameterSide2::Right, policy),
            self.point_at_parameter_with_policy(&end, CurveParameterSide2::Left, policy),
        ];
        let [start_point, end_point] = endpoints;
        let endpoints = [
            start_point.map_err(|error| error.with_operation(CurveOperation2::Subdivision))?,
            end_point.map_err(|error| error.with_operation(CurveOperation2::Subdivision))?,
        ];
        let validation = PolicyClassificationCache::new();
        if CurveContext::STRICT.accepts_retained_policy(policy.retained_object_policy()) {
            validation.certify(());
        }
        Ok(Self::from_source_range(CurveSourceRange2 {
            source: source.clone(),
            range: CurveParameterRange2::new_validated(start, end),
            endpoints,
            reversed,
            validation,
            spans: PolicyEvaluationCache::new(),
        }))
    }

    pub(super) fn reverse_source_range(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        let range = self.source_range().expect("source range");
        range.validate(policy)?;
        let validation = PolicyClassificationCache::new();
        if CurveContext::STRICT.accepts_retained_policy(policy.retained_object_policy()) {
            validation.certify(());
        }
        Ok(Self::from_source_range(CurveSourceRange2 {
            source: range.source.clone(),
            range: range.range.clone(),
            endpoints: range.endpoints.clone(),
            reversed: !range.reversed,
            validation,
            spans: PolicyEvaluationCache::new(),
        }))
    }

    pub(super) fn source_range_point_at(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        let range = self.source_range().expect("source range");
        range.validate(policy)?;
        let family = self.family();
        let start_order = compare(parameter, range.range.start(), family, policy)?;
        let end_order = compare(parameter, range.range.end(), family, policy)?;
        if start_order.is_lt() || end_order.is_gt() {
            return Err(subdivision_error(family, CurveError::InvalidCurveParameter));
        }
        if start_order.is_eq() {
            return Ok(range.endpoints[0].clone());
        }
        if end_order.is_eq() {
            return Ok(range.endpoints[1].clone());
        }
        range
            .source
            .point_at_parameter_with_policy(parameter, side, policy)
    }

    pub(super) fn transform_source_range(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let range = self.source_range().expect("source range");
        range.validate(policy)?;
        let source = range.source.transform_similarity_raw(transform, policy)?;
        let transformed = source.restrict_authored_source(
            range.range.start().clone(),
            range.range.end().clone(),
            policy,
        )?;
        if range.reversed {
            transformed.reverse_source_range(policy)
        } else {
            Ok(transformed)
        }
    }

    fn from_source_range(range: CurveSourceRange2) -> Self {
        Self {
            data: Arc::new(CurveData2 {
                carrier: CurveCarrier2::SourceRange(Box::new(range)),
                lineage: None,
                parameter_domain: OnceLock::new(),
                native_bezier_fragments: PolicyEvaluationCache::new(),
                rational_evaluators: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        }
    }

    pub(crate) fn restricted_source_spans(
        &self,
        policy: &CurveContext,
        operation: CurveOperation2,
    ) -> ExactCurveResult<Option<&[CurveSourceSpan2]>> {
        let Some(range) = self.source_range() else {
            return Ok(None);
        };
        let spans = resolve_cached_evaluation(&range.spans, policy, |attempt| {
            range.lower_spans(attempt).map(Classification::Decided)
        })
        .map_err(|error| error.with_operation(operation))?;
        match spans {
            Classification::Decided(spans) => Ok(Some(spans)),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, self.family(), reason))
            }
        }
    }
}

impl CurveSourceRange2 {
    fn validate(&self, policy: &CurveContext) -> ExactCurveResult<()> {
        let family = self.source.family();
        let validation = resolve_cached_classification(&self.validation, policy, |attempt| {
            let domain = self.source.parameter_domain();
            if compare(self.range.start(), domain.start(), family, attempt)?.is_lt()
                || compare(self.range.end(), domain.end(), family, attempt)?.is_gt()
                || compare(self.range.start(), self.range.end(), family, attempt)? != Ordering::Less
            {
                return Err(subdivision_error(family, CurveError::InvalidCurveRange));
            }
            for (parameter, side, point) in [
                (
                    self.range.start(),
                    CurveParameterSide2::Right,
                    &self.endpoints[0],
                ),
                (
                    self.range.end(),
                    CurveParameterSide2::Left,
                    &self.endpoints[1],
                ),
            ] {
                let replay = self
                    .source
                    .point_at_parameter_with_policy(parameter, side, attempt)?;
                match replay.same_point(point, attempt) {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        return Err(subdivision_error(family, CurveError::InvalidCurveParameter));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(()))
        })?;
        decided(validation, family).map(|_| ())
    }

    fn lower_spans(&self, policy: &CurveContext) -> ExactCurveResult<Vec<CurveSourceSpan2>> {
        self.validate(policy)?;
        let family = self.source.family();
        let native = self
            .source
            .native_bezier_fragments_for_operation(policy, CurveOperation2::Subdivision)?;
        let mut spans = Vec::new();
        for native in native {
            let (chart_start, chart_end) = native.parameter_range();
            let chart_start_parameter = CurveParameter2::from(chart_start.clone());
            let chart_end_parameter = CurveParameter2::from(chart_end.clone());
            if compare(self.range.end(), &chart_start_parameter, family, policy)?
                != Ordering::Greater
                || compare(self.range.start(), &chart_end_parameter, family, policy)?
                    != Ordering::Less
            {
                continue;
            }
            let cuts_start =
                compare(self.range.start(), &chart_start_parameter, family, policy)?.is_gt();
            let cuts_end = compare(self.range.end(), &chart_end_parameter, family, policy)?.is_lt();
            let width = chart_end - chart_start;
            let mut span = if !cuts_start && !cuts_end {
                CurveSourceSpan2 {
                    fragment: BezierSplitFragment2::Materialized {
                        start: BezierParameter2::Exact(Real::zero()),
                        end: BezierParameter2::Exact(Real::one()),
                        curve: native.curve().clone(),
                    },
                    source_scale: width.clone(),
                    source_offset: chart_start.clone(),
                }
            } else {
                let inverse = (Real::one() / &width)
                    .map_err(|cause| subdivision_error(family, cause.into()))?;
                let local = |parameter: &CurveParameter2| {
                    decided(
                        parameter
                            .affine_image_unbounded(&inverse, &(-chart_start * &inverse), policy)
                            .map_err(|cause| subdivision_error(family, cause))?,
                        family,
                    )
                };
                let (native_start, native_end) = native.curve().endpoint_refs();
                let (start, start_point) = if cuts_start {
                    (local(self.range.start())?, self.endpoints[0].clone())
                } else {
                    (Real::zero().into(), CurvePoint2::from(native_start.clone()))
                };
                let (end, end_point) = if cuts_end {
                    (local(self.range.end())?, self.endpoints[1].clone())
                } else {
                    (Real::one().into(), CurvePoint2::from(native_end.clone()))
                };
                CurveSourceSpan2 {
                    fragment: CurveSupport2::Bezier(native.curve().clone())
                        .restrict_certified(
                            CurveParameterRange2::new_validated(start, end),
                            Some([start_point, end_point]),
                            false,
                            policy,
                        )
                        .map_err(|cause| subdivision_error(family, cause))?,
                    source_scale: width.clone(),
                    source_offset: chart_start.clone(),
                }
            };
            if self.reversed {
                if matches!(span.fragment, BezierSplitFragment2::Materialized { .. }) {
                    span.source_scale = -span.source_scale;
                    span.source_offset = chart_end.clone();
                }
                span.fragment = span
                    .fragment
                    .reversed()
                    .map_err(|cause| subdivision_error(family, cause))?;
            }
            spans.push(span);
        }
        if self.reversed {
            spans.reverse();
        }
        if spans.is_empty() {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Subdivision,
                family,
                UncertaintyReason::Boundary,
            ));
        }
        Ok(spans)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CurveCertainty, PolynomialSplineCurve2};

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32) -> Point2 {
        Point2::from_values(x, y)
    }

    fn selected_parameters(policy: &CurveContext) -> [CurveParameter2; 3] {
        let mut parameters =
            [(q(1, 2), 32_768), (q(1, 4), 1_024), (q(1, 3), 64)].map(|(constant, scale)| {
                let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                    constant, scale, policy,
                );
                assert!(matches!(
                    selected.promoted_bezier_parameter(policy).unwrap(),
                    Classification::Uncertain(_)
                ));
                CurveParameter2::from_selected_fiber(selected)
            });
        parameters[0] = decided(
            parameters[0]
                .affine_image_unbounded(&q(1, 2), &Real::zero(), policy)
                .unwrap(),
            CurveFamily2::RationalBezier,
        )
        .unwrap();
        parameters
    }

    fn assert_same(actual: &CurvePoint2, expected: &CurvePoint2, policy: &CurveContext) {
        let equality = actual.coincides_with(expected, policy);
        assert_eq!(equality.certainty, CurveCertainty::Certified);
        assert_eq!(equality.value, Classification::Decided(true));
    }

    #[test]
    fn native_span_publication_preserves_geometry_and_parameter_lineage() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for original in authored_sources(&policy) {
                for reversed in [false, true] {
                    let domain = original.native_parameter_domain().unwrap();
                    let parameter =
                        |unit: Real| domain.start() + (domain.end() - domain.start()) * unit;
                    let source = original
                        .subcurve(
                            parameter(q(1, 8)).into(),
                            parameter(q(7, 8)).into(),
                            &policy,
                        )
                        .unwrap()
                        .value;
                    let source = if reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source
                    };
                    let owner = Arc::downgrade(&source.data);
                    let source_lineage = source.data.lineage.as_ref().unwrap();
                    let spans = source.native_bezier_fragments(&policy).unwrap();
                    assert_eq!(spans.certainty, CurveCertainty::Certified);
                    let mut published = Vec::new();
                    for span in spans.value.iter().cloned() {
                        let (start, end) = span.parameter_range();
                        let (start, end) = (start.clone(), end.clone());
                        let curve = span.into_curve();
                        assert!(Arc::ptr_eq(
                            &curve.data.lineage.as_ref().unwrap().root,
                            &source_lineage.root
                        ));
                        assert_eq!(
                            curve.parameter_domain().scalar_endpoints(),
                            Some((&Real::zero(), &Real::one()))
                        );
                        for unit in [Real::zero(), q(1, 4), q(1, 2), q(3, 4), Real::one()] {
                            let parameter = &start + (&end - &start) * &unit;
                            assert_eq!(
                                curve.lineage_parameter_at(&unit).unwrap(),
                                source.lineage_parameter_at(&parameter).unwrap()
                            );
                            assert_same(
                                &curve.point_at(&unit.into(), &policy).unwrap().value,
                                &source.point_at(&parameter.into(), &policy).unwrap().value,
                                &policy,
                            );
                        }
                        let middle = curve
                            .subcurve(q(1, 4).into(), q(3, 4).into(), &policy)
                            .unwrap()
                            .value;
                        assert!(Arc::ptr_eq(
                            &middle.data.lineage.as_ref().unwrap().root,
                            &source_lineage.root
                        ));
                        assert_same(
                            &middle.point_at(&q(1, 2).into(), &policy).unwrap().value,
                            &curve.point_at(&q(1, 2).into(), &policy).unwrap().value,
                            &policy,
                        );
                        published.push(curve);
                    }
                    drop(source);
                    assert!(
                        owner.upgrade().is_none(),
                        "span publication must not retain its former curve owner"
                    );
                    assert!(!published.is_empty());
                }
            }
        }
    }

    #[test]
    fn transformed_parameter_lineage_does_not_certify_unrelated_curve_overlap() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0)));
            let restricted = source
                .subcurve(q(1, 8).into(), q(7, 8).into(), &policy)
                .unwrap()
                .value;
            let translation = Similarity2::try_from_real_affine(
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::zero(),
                q(1, 10),
            )
            .unwrap();
            let translated = restricted
                .transform_similarity(&translation, &policy)
                .unwrap()
                .value;
            // Both are graphs over the same x interval, separated everywhere
            // by exactly 1/10. Their bounding boxes overlap.
            let intersections = restricted.intersect_curve(&translated, &policy).unwrap();
            assert_eq!(intersections.certainty, CurveCertainty::Certified);
            assert!(intersections.value.blockers().is_empty());
            assert!(intersections.value.contacts().is_empty());
            assert!(
                intersections.value.overlaps().is_empty(),
                "a shared parameter source is not a shared geometric image"
            );

            let half_translation = Similarity2::try_from_real_affine(
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::zero(),
                q(1, 20),
            )
            .unwrap();
            let composed = restricted
                .transform_similarity(&half_translation, &policy)
                .unwrap()
                .value
                .transform_similarity(&half_translation, &policy)
                .unwrap()
                .value;
            let rotation = Similarity2::try_from_real_affine(
                Real::zero(),
                -Real::one(),
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
            )
            .unwrap();
            let rotated = translated
                .transform_similarity(&rotation, &policy)
                .unwrap()
                .value;
            let direct = restricted
                .transform_similarity(&translation.then(&rotation), &policy)
                .unwrap()
                .value;
            for (first, second) in [(&translated, &composed), (&rotated, &direct)] {
                assert!(first.shares_certified_parameter_lineage(second));
                let span = first.native_bezier_fragments(&policy).unwrap().value[0]
                    .clone()
                    .into_curve();
                assert!(span.shares_certified_parameter_lineage(second));
                for reversed in [false, true] {
                    let other = if reversed {
                        second.reversed(&policy).unwrap().value
                    } else {
                        second.clone()
                    };
                    let overlap = span.intersect_curve(&other, &policy).unwrap();
                    assert_eq!(overlap.certainty, CurveCertainty::Certified);
                    assert!(overlap.value.blockers().is_empty());
                    assert_eq!(overlap.value.overlaps().len(), 1);
                }
            }
        }
    }

    fn authored_sources(policy: &CurveContext) -> [Curve2; 8] {
        let controls = vec![p(0, 0), p(0, 1), p(1, 2), p(3, 3)];
        let knots = [3, 3, 3, 5, 7, 7, 7]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>();
        [
            Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
            Curve2::from(CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), true).unwrap()),
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0))),
            Curve2::from(CubicBezier2::new(p(0, 0), p(1, 1), p(2, -1), p(3, 0))),
            Curve2::from(
                RationalQuadraticBezier2::try_new(
                    p(0, 0),
                    p(1, 1),
                    p(2, 0),
                    Real::one(),
                    Real::from(2),
                    Real::one(),
                )
                .unwrap(),
            ),
            Curve2::from(
                RationalBezier2::try_new(
                    vec![p(0, 0), p(1, 1), p(2, 2), p(3, 1), p(4, 0)],
                    vec![
                        Real::one(),
                        Real::from(2),
                        Real::from(3),
                        Real::from(2),
                        Real::one(),
                    ],
                )
                .unwrap(),
            ),
            Curve2::from(
                PolynomialSplineCurve2::try_new(2, controls.clone(), knots.clone(), policy)
                    .unwrap()
                    .value,
            ),
            Curve2::from(
                NurbsCurve2::try_new(
                    2,
                    controls,
                    vec![Real::one(), Real::from(2), Real::from(3), Real::one()],
                    knots,
                    policy,
                )
                .unwrap()
                .value,
            ),
        ]
    }

    #[test]
    fn selected_subdivision_keeps_all_authored_families_and_flat_source_charts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let unit_parameters = selected_parameters(&policy);
            let sources = authored_sources(&policy);
            for source in sources {
                let domain = source.native_parameter_domain().unwrap();
                let [start, probe, end] = unit_parameters.clone().map(|parameter| {
                    decided(
                        parameter
                            .affine_image_unbounded(
                                &(domain.end() - domain.start()),
                                domain.start(),
                                &policy,
                            )
                            .unwrap(),
                        source.family(),
                    )
                    .unwrap()
                });
                let restricted = source
                    .subcurve(start.clone(), end.clone(), &policy)
                    .unwrap();
                assert_eq!(restricted.certainty, CurveCertainty::Certified);
                let restricted = restricted.value;
                assert_eq!(restricted.family(), source.family());
                assert_eq!(restricted.parameter_domain().start(), &start);
                assert_eq!(restricted.parameter_domain().end(), &end);
                assert!(Arc::ptr_eq(
                    &restricted.source_range().unwrap().source.data,
                    &source.data
                ));
                let expected = source.point_at(&probe, &policy).unwrap().value;
                for reversed in [false, true] {
                    let restricted = if reversed {
                        restricted.reversed(&policy).unwrap().value
                    } else {
                        restricted.clone()
                    };
                    assert_same(
                        &restricted.point_at(&probe, &policy).unwrap().value,
                        &expected,
                        &policy,
                    );
                    let (first, second) =
                        restricted.split_at(probe.clone(), &policy).unwrap().value;
                    assert_same(&first.start(), &restricted.start(), &policy);
                    assert_same(&first.end(), &expected, &policy);
                    assert_same(&second.start(), &expected, &policy);
                    assert_same(&second.end(), &restricted.end(), &policy);
                    for part in [&first, &second] {
                        assert!(Arc::ptr_eq(
                            &part.source_range().unwrap().source.data,
                            &source.data
                        ));
                        assert!(part.source_range().unwrap().source.source_range().is_none());
                    }
                    let repeated = restricted
                        .subcurve(start.clone(), probe.clone(), &policy)
                        .unwrap()
                        .value;
                    assert!(
                        repeated.source_range().unwrap().endpoints[0]
                            .shares_storage(&restricted.source_range().unwrap().endpoints[0])
                    );
                    let spans = restricted
                        .restricted_source_spans(&policy, CurveOperation2::Subdivision)
                        .unwrap()
                        .unwrap();
                    if source.family() == CurveFamily2::CircularArc {
                        assert_eq!(
                            spans.len(),
                            3,
                            "the complete major-arc chart cover survives"
                        );
                    }
                    if matches!(
                        source.family(),
                        CurveFamily2::PolynomialBSpline | CurveFamily2::Nurbs
                    ) {
                        assert_eq!(spans.len(), 2, "both authored spline spans survive");
                    }
                    for span in spans {
                        let curve = Curve2::from_retained_fragment(span.fragment.clone());
                        let local = decided(
                            curve
                                .parameter_domain()
                                .start()
                                .strict_scalar_between_ordered(
                                    curve.parameter_domain().end(),
                                    &policy,
                                )
                                .unwrap(),
                            source.family(),
                        )
                        .unwrap();
                        let authored = &span.source_scale * &local + &span.source_offset;
                        let actual = curve.point_at(&local.into(), &policy).unwrap().value;
                        let expected = source.point_at(&authored.into(), &policy).unwrap().value;
                        assert_same(&actual, &expected, &policy);
                        if let BezierSplitFragment2::SelectedFiber(fragment) = &span.fragment {
                            let root = Curve2::from(fragment.rational_curve().unwrap().clone());
                            let points = if fragment.is_reversed() {
                                [fragment.end_point(), fragment.start_point()]
                            } else {
                                [fragment.start_point(), fragment.end_point()]
                            };
                            for (parameter, point) in
                                [fragment.range().start(), fragment.range().end()]
                                    .into_iter()
                                    .zip(points)
                            {
                                assert_same(
                                    &root.point_at(parameter, &policy).unwrap().value,
                                    point,
                                    &policy,
                                );
                            }
                        }
                    }
                    restricted.bounds().unwrap();
                    let transform = Similarity2::try_from_real_affine(
                        Real::zero(),
                        Real::from(-2),
                        Real::from(2),
                        Real::zero(),
                        Real::from(3),
                        Real::from(-1),
                    )
                    .unwrap();
                    let transformed = restricted
                        .transform_similarity(&transform, &policy)
                        .unwrap()
                        .value;
                    assert_eq!(
                        transformed.parameter_domain(),
                        restricted.parameter_domain()
                    );
                    assert_eq!(transformed.family(), source.family());
                    let expected = CurvePoint2::from(crate::BezierSimilarityPoint2::new(
                        expected.clone(),
                        transform,
                        &policy,
                    ));
                    assert_same(
                        &transformed.point_at(&probe, &policy).unwrap().value,
                        &expected,
                        &policy,
                    );
                    let translated = transformed
                        .transform_similarity(
                            &Similarity2::try_from_real_affine(
                                Real::one(),
                                Real::zero(),
                                Real::zero(),
                                Real::one(),
                                Real::from(100),
                                Real::zero(),
                            )
                            .unwrap(),
                            &policy,
                        )
                        .unwrap()
                        .value;
                    assert_eq!(
                        translated
                            .point_at(&probe, &policy)
                            .unwrap()
                            .value
                            .coincides_with(&expected, &policy)
                            .value,
                        Classification::Decided(false)
                    );
                }
            }
        }
    }

    #[test]
    fn selected_analytic_subdivision_reuses_the_original_parallel() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 2))
                .parallel_left(q(1, 8))
                .unwrap();
            let original = decided(
                crate::BezierParallelFragment2::try_new(
                    parallel.clone(),
                    BezierParameterRange2::new_validated(
                        BezierParameter2::Exact(Real::zero()),
                        BezierParameter2::Exact(Real::one()),
                    ),
                    &policy,
                )
                .unwrap(),
                CurveFamily2::AnalyticParallel,
            )
            .unwrap();
            let [start, probe, end] = selected_parameters(&policy);
            for reversed in [false, true] {
                let source = Curve2::from_retained_fragment(
                    BezierSplitFragment2::AnalyticParallel(if reversed {
                        original.reversed()
                    } else {
                        original.clone()
                    }),
                );
                let restricted = source
                    .subcurve(start.clone(), end.clone(), &policy)
                    .unwrap()
                    .value;
                let expected = source.point_at(&probe, &policy).unwrap().value;
                let (first, second) = restricted.split_at(probe.clone(), &policy).unwrap().value;
                assert_same(&first.end(), &expected, &policy);
                assert_same(&second.start(), &expected, &policy);
                for part in [first, second] {
                    let Some(BezierSplitFragment2::SelectedFiber(fragment)) =
                        part.retained_fragment()
                    else {
                        panic!("analytic restrictions keep common selected parameters");
                    };
                    assert_eq!(fragment.analytic_parallel(), Some(&parallel));
                }
            }
        }
    }

    #[test]
    fn selected_spline_subdivision_preserves_jumps_and_one_sided_cuts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let controls = vec![p(0, 0), p(1, 1), p(2, 0), p(10, 0), p(11, 1), p(12, 0)];
            let knots = [-2, -1, 0, 1, 1, 1, 2, 3, 4]
                .into_iter()
                .map(Real::from)
                .collect::<Vec<_>>();
            let polynomial =
                PolynomialSplineCurve2::try_new(2, controls.clone(), knots.clone(), &policy)
                    .unwrap()
                    .value;
            let rational = NurbsCurve2::try_new(
                2,
                controls,
                [1, 2, 3, 5, 7, 11].into_iter().map(Real::from).collect(),
                knots,
                &policy,
            )
            .unwrap()
            .value;
            let [start, _, end] = selected_parameters(&policy).map(|parameter| {
                decided(
                    parameter
                        .affine_image_unbounded(&Real::from(2), &Real::zero(), &policy)
                        .unwrap(),
                    CurveFamily2::Nurbs,
                )
                .unwrap()
            });
            for source in [Curve2::from(polynomial), Curve2::from(rational)] {
                let restricted = source
                    .subcurve(start.clone(), end.clone(), &policy)
                    .unwrap()
                    .value;
                let left = source
                    .point_at_side(&Real::one().into(), CurveParameterSide2::Left, &policy)
                    .unwrap()
                    .value;
                let right = source
                    .point_at_side(&Real::one().into(), CurveParameterSide2::Right, &policy)
                    .unwrap()
                    .value;
                assert_eq!(
                    left.coincides_with(&right, &policy).value,
                    Classification::Decided(false)
                );
                for reversed in [false, true] {
                    let restricted = if reversed {
                        restricted.reversed(&policy).unwrap().value
                    } else {
                        restricted.clone()
                    };
                    assert!(matches!(restricted.point_at(&Real::one().into(), &policy),
                        Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::Boundary));
                    let (first, second) = restricted
                        .split_at(Real::one().into(), &policy)
                        .unwrap()
                        .value;
                    assert_same(&first.end(), if reversed { &right } else { &left }, &policy);
                    assert_same(
                        &second.start(),
                        if reversed { &left } else { &right },
                        &policy,
                    );
                    for part in [&first, &second] {
                        assert_eq!(
                            part.restricted_source_spans(&policy, CurveOperation2::Subdivision)
                                .unwrap()
                                .unwrap()
                                .len(),
                            1
                        );
                    }
                    // Both endpoints are selected, but their x envelopes lie
                    // on opposite sides of the jump, certifying distinctness.
                    let closing = decided(
                        crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                            restricted.end(),
                            restricted.start(),
                            &policy,
                        )
                        .unwrap(),
                        CurveFamily2::Line,
                    )
                    .unwrap();
                    let path = CurvePath2::try_new(vec![
                        restricted,
                        Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(
                            closing,
                        )),
                    ])
                    .unwrap();
                    assert!(matches!(
                        path.boundary_loop(&policy),
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::DisconnectedCurvePath,
                            ..
                        })
                    ));
                }
            }
        }
    }

    #[test]
    fn selected_source_ranges_reenter_public_path_corner_operations() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let [start, _, end] = selected_parameters(&policy);
            let previous = Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap())
                .subcurve(start, Real::one().into(), &policy)
                .unwrap()
                .value;
            let next = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)))
                .subcurve(Real::zero().into(), end, &policy)
                .unwrap()
                .value;
            let path = CurvePath2::try_new(vec![previous, next]).unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let results = [
                    path.chamfer_vertex_by_setbacks(
                        1,
                        Real::one(),
                        Real::one(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    ),
                    path.fillet_vertex_by_radius(
                        1,
                        Real::one(),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    ),
                ];
                for (operation, result) in results.into_iter().enumerate() {
                    let result = result.unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let CurveCornerSolutions2::Unique(edited) = result.value else {
                        panic!("one exact corner solution")
                    };
                    assert_same(&edited.start(), &path.start(), &policy);
                    assert_same(&edited.end(), &path.end(), &policy);
                    for pair in edited.curves().windows(2) {
                        assert_same(&pair[0].end(), &pair[1].start(), &policy);
                    }
                    let again = edited
                        .chamfer_vertex_by_setbacks(
                            1,
                            q(1, 64),
                            q(1, 64),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap_or_else(|error| panic!("second chamfer after operation {operation}, reversed={reversed}, policy={policy:?}: {error:?}"));
                    assert_eq!(again.certainty, CurveCertainty::Certified);
                    let CurveCornerSolutions2::Unique(again) = again.value else {
                        panic!("the selected result supports another corner edit")
                    };
                    assert_same(&again.start(), &path.start(), &policy);
                    assert_same(&again.end(), &path.end(), &policy);
                    for pair in again.curves().windows(2) {
                        assert_same(&pair[0].end(), &pair[1].start(), &policy);
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_chamfers_preserve_all_selected_major_arc_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let end = decided(
                selected_parameters(&policy)[0]
                    .affine_image_unbounded(&q(1, 64), &q(63, 64), &policy)
                    .unwrap(),
                CurveFamily2::CircularArc,
            )
            .unwrap();
            let arc = Curve2::from(
                CircularArc2::try_from_center(
                    p(1, 0),
                    Point2::new(q(3, 5), q(4, 5)),
                    p(0, 0),
                    true,
                )
                .unwrap(),
            )
            .subcurve(Real::zero().into(), end, &policy)
            .unwrap()
            .value;
            let path = CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(p(1, -2), p(1, 0)).unwrap()),
                arc,
            ])
            .unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                for setback in [1, 2] {
                    let setbacks = if reversed { [setback, 1] } else { [1, setback] };
                    let outcome = path
                        .chamfer_vertex_by_setbacks(
                            1,
                            Real::from(setbacks[0]),
                            Real::from(setbacks[1]),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let candidates = match outcome.value {
                        CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                        CurveCornerSolutions2::Multiple(candidates) => candidates,
                        CurveCornerSolutions2::NoSolution(reason) => {
                            panic!("lost selected major-arc contacts: {reason:?}")
                        }
                    };
                    assert_eq!(candidates.len(), if setback == 1 { 2 } else { 1 });
                    let expected = if setback == 2 {
                        vec![p(-1, 0)]
                    } else {
                        let y = (Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap();
                        vec![Point2::new(q(1, 2), y.clone()), Point2::new(q(1, 2), -y)]
                    };
                    for candidate in &candidates {
                        assert_same(&candidate.start(), &path.start(), &policy);
                        assert_same(&candidate.end(), &path.end(), &policy);
                        for pair in candidate.curves().windows(2) {
                            assert_same(&pair[0].end(), &pair[1].start(), &policy);
                        }
                    }
                    for point in expected {
                        assert!(
                            candidates.iter().any(|candidate| {
                                candidate.curves().windows(2).any(|pair| {
                                    pair[0].end().same_point(&point.clone().into(), &policy)
                                        == Classification::Decided(true)
                                })
                            }),
                            "missing {point:?}, reversed={reversed}, policy={policy:?}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_circle_extensions_do_not_republish_authored_chamfer_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = Curve2::from(
                CircularArc2::try_from_center(
                    p(1, 0),
                    Point2::new(q(3, 5), q(4, 5)),
                    p(0, 0),
                    true,
                )
                .unwrap(),
            );
            let end = decided(
                selected_parameters(&policy)[0]
                    .affine_image_unbounded(&q(1, 64), &q(63, 64), &policy)
                    .unwrap(),
                source.family(),
            )
            .unwrap();
            let selected = source
                .subcurve(Real::zero().into(), end, &policy)
                .unwrap()
                .value;
            for reversed in [false, true] {
                let solve = |arc: Curve2| {
                    let path = CurvePath2::try_new(vec![
                        Curve2::from(LineSeg2::try_new(p(1, -2), p(1, 0)).unwrap()),
                        arc,
                    ])
                    .unwrap();
                    let path = if reversed {
                        path.reversed(&policy).unwrap().value
                    } else {
                        path
                    };
                    let outcome = path
                        .chamfer_vertex_by_setbacks(
                            1,
                            Real::one(),
                            Real::one(),
                            CurveCornerMode2::TrimOrExtend,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let candidates = match outcome.value {
                        CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                        CurveCornerSolutions2::Multiple(candidates) => candidates,
                        CurveCornerSolutions2::NoSolution(reason) => {
                            panic!("the authored circular contacts were lost: {reason:?}")
                        }
                    };
                    candidates
                        .into_iter()
                        .map(|candidate| {
                            assert_same(&candidate.start(), &path.start(), &policy);
                            assert_same(&candidate.end(), &path.end(), &policy);
                            for pair in candidate.curves().windows(2) {
                                assert_same(&pair[0].end(), &pair[1].start(), &policy);
                            }
                            let index = if reversed {
                                candidate.curves().len() - 2
                            } else {
                                1
                            };
                            let chamfer = &candidate.curves()[index];
                            [chamfer.start(), chamfer.end()]
                        })
                        .collect::<Vec<_>>()
                };
                let direct = solve(source.clone());
                let retained = solve(selected.clone());
                assert_eq!(
                    retained.len(),
                    direct.len(),
                    "reversed={reversed}, policy={policy:?}"
                );
                for expected in &direct {
                    assert_eq!(
                        retained
                            .iter()
                            .filter(|actual| {
                                actual.iter().zip(expected).all(|(actual, expected)| {
                                    actual.same_point(expected, &policy)
                                        == Classification::Decided(true)
                                })
                            })
                            .count(),
                        1,
                        "each authored circle contact must have one owner",
                    );
                }
            }
        }
    }

    #[test]
    fn circular_spline_extensions_preserve_complements_and_closed_endpoint_ownership() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let weight = q(1, 2).sqrt().unwrap();
            let source = Curve2::from(
                NurbsCurve2::try_new(
                    2,
                    vec![
                        p(1, 0),
                        p(1, -1),
                        p(0, -1),
                        p(-1, -1),
                        p(-1, 0),
                        p(-1, 1),
                        p(0, 1),
                    ],
                    vec![
                        Real::one(),
                        weight.clone(),
                        Real::one(),
                        weight.clone(),
                        Real::one(),
                        weight,
                        Real::one(),
                    ],
                    [0, 0, 0, 1, 1, 2, 2, 3, 3, 3]
                        .into_iter()
                        .map(Real::from)
                        .collect(),
                    &policy,
                )
                .unwrap()
                .value,
            );
            let end = decided(
                selected_parameters(&policy)[0]
                    .affine_image_unbounded(&q(3, 64), &q(189, 64), &policy)
                    .unwrap(),
                source.family(),
            )
            .unwrap();
            let selected_source = source
                .subcurve(Real::zero().into(), end, &policy)
                .unwrap()
                .value;
            for selected in [false, true] {
                for reversed in [false, true] {
                    let path = CurvePath2::try_new(vec![
                        Curve2::from(LineSeg2::try_new(p(1, -2), p(1, 0)).unwrap()),
                        if selected {
                            selected_source.clone()
                        } else {
                            source.clone()
                        },
                    ])
                    .unwrap();
                    let path = if reversed {
                        path.reversed(&policy).unwrap().value
                    } else {
                        path
                    };
                    for endpoint_contact in [false, true] {
                        let arc_setback = if endpoint_contact {
                            Real::from(2).sqrt().unwrap()
                        } else {
                            Real::one()
                        };
                        let setbacks = if reversed {
                            [arc_setback, Real::one()]
                        } else {
                            [Real::one(), arc_setback]
                        };
                        let [first_setback, second_setback] = setbacks;
                        let outcome = path.chamfer_vertex_by_setbacks(
                            1, first_setback, second_setback, CurveCornerMode2::TrimOrExtend, &policy,
                        ).unwrap_or_else(|error| panic!("selected={selected}, reversed={reversed}, endpoint_contact={endpoint_contact}: {error:?}"));
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        let candidates = match outcome.value {
                            CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                            CurveCornerSolutions2::Multiple(candidates) => candidates,
                            CurveCornerSolutions2::NoSolution(reason) => {
                                panic!("circular spline contacts were lost: {reason:?}")
                            }
                        };
                        // North is the untrimmed spline's far endpoint. It
                        // becomes a valid complementary extension only after
                        // the selected trim removes it from the authored set.
                        assert_eq!(
                            candidates.len(),
                            if endpoint_contact && !selected { 2 } else { 4 }
                        );
                        let y = (Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap();
                        let expected = if endpoint_contact {
                            [p(0, -1), p(0, 1)]
                        } else {
                            [Point2::new(q(1, 2), -&y), Point2::new(q(1, 2), y)]
                        };
                        let contacts = candidates
                            .into_iter()
                            .map(|candidate| {
                                assert_same(&candidate.start(), &path.start(), &policy);
                                assert_same(&candidate.end(), &path.end(), &policy);
                                for pair in candidate.curves().windows(2) {
                                    assert_same(&pair[0].end(), &pair[1].start(), &policy);
                                }
                                let index = if reversed {
                                    candidate.curves().len() - 2
                                } else {
                                    1
                                };
                                if reversed {
                                    candidate.curves()[index].start()
                                } else {
                                    candidate.curves()[index].end()
                                }
                            })
                            .collect::<Vec<_>>();
                        for (index, point) in expected.into_iter().enumerate() {
                            let comparisons = contacts
                                .iter()
                                .map(|contact| contact.same_point(&point.clone().into(), &policy))
                                .collect::<Vec<_>>();
                            assert_eq!(
                                comparisons
                                    .iter()
                                    .filter(|result| **result == Classification::Decided(true))
                                    .count(),
                                if index == 1 && endpoint_contact && !selected {
                                    0
                                } else {
                                    2
                                },
                                "selected={selected}, reversed={reversed}, endpoint_contact={endpoint_contact}, expected_index={index}, comparisons={comparisons:?}",
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_fillets_preserve_all_selected_major_arc_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = Curve2::from(
                CircularArc2::try_from_center(
                    p(1, 0),
                    Point2::new(q(3, 5), q(4, 5)),
                    p(0, 0),
                    true,
                )
                .unwrap(),
            );
            let end = decided(
                selected_parameters(&policy)[0]
                    .affine_image_unbounded(&q(1, 64), &q(63, 64), &policy)
                    .unwrap(),
                source.family(),
            )
            .unwrap();
            let selected_source = source
                .subcurve(Real::zero().into(), end, &policy)
                .unwrap()
                .value;
            for selected in [false, true] {
                let arc = if selected {
                    selected_source.clone()
                } else {
                    source.clone()
                };
                let path = CurvePath2::try_new(vec![
                    Curve2::from(LineSeg2::try_new(p(-3, 0), p(1, 0)).unwrap()),
                    arc,
                ])
                .unwrap();
                for reversed in [false, true] {
                    let path = if reversed {
                        path.reversed(&policy).unwrap().value
                    } else {
                        path.clone()
                    };
                    let result = path.fillet_vertex_by_radius(1, q(1, 4), CurveCornerMode2::TrimOnly, &policy)
                        .unwrap_or_else(|error| panic!("selected={selected}, reversed={reversed}, policy={policy:?}: {error:?}"));
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let candidates = match result.value {
                        CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                        CurveCornerSolutions2::Multiple(candidates) => candidates,
                        CurveCornerSolutions2::NoSolution(reason) => {
                            panic!("major-arc fillets were lost: {reason:?}")
                        }
                    };
                    assert_eq!(
                        candidates.len(),
                        3,
                        "selected={selected}, reversed={reversed}, policy={policy:?}"
                    );
                    let inward = q(1, 2).sqrt().unwrap();
                    let outward = q(3, 2).sqrt().unwrap();
                    for expected in [
                        Point2::new(q(4, 3) * &inward, -q(1, 3)),
                        Point2::new(-q(4, 3) * &inward, -q(1, 3)),
                        Point2::new(-q(4, 5) * outward, q(1, 5)),
                    ] {
                        assert!(
                            candidates
                                .iter()
                                .any(|candidate| candidate.curves().windows(2).any(|pair| {
                                    pair[0].end().same_point(&expected.clone().into(), &policy)
                                        == Classification::Decided(true)
                                })),
                            "missing exact arc contact {expected:?}"
                        );
                    }
                    for candidate in candidates {
                        assert_same(&candidate.start(), &path.start(), &policy);
                        assert_same(&candidate.end(), &path.end(), &policy);
                        for pair in candidate.curves().windows(2) {
                            assert_same(&pair[0].end(), &pair[1].start(), &policy);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_fillets_own_surviving_spline_tangents_and_reenter_chamfers() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for rational in [false, true] {
                for sharp_knot in [false, true] {
                    // The unit fillet touches (0,1) exactly at a knot. A
                    // smooth partition owns it once. Turning right at that
                    // knot discards the vertical tangent, so that circle is
                    // inadmissible. The later contact (-3,1) always survives.
                    let controls = if sharp_knot {
                        vec![p(0, 0), p(0, 1), p(2, 1), p(2, 3), p(-3, 3), p(-3, 0)]
                    } else {
                        vec![p(0, 0), p(0, 1), p(0, 3), p(-3, 3), p(-3, 0)]
                    };
                    let last = controls.len() as i32 + 2;
                    let knots = std::iter::once(3)
                        .chain(3..=last)
                        .chain(std::iter::once(last))
                        .map(Real::from)
                        .collect();
                    let weights = (0..controls.len())
                        .map(|index| Real::from(1 + (index % 3) as i32))
                        .collect();
                    let source = if rational {
                        Curve2::try_nurbs(1, controls, weights, knots, &policy)
                    } else {
                        Curve2::try_polynomial_bspline(1, controls, knots, &policy)
                    }
                    .unwrap()
                    .value;
                    for selected in [false, true] {
                        let source = if selected {
                            let end = decided(
                                selected_parameters(&policy)[0]
                                    .affine_image_unbounded(
                                        &q(1, 64),
                                        &(Real::from(last) - q(1, 16)),
                                        &policy,
                                    )
                                    .unwrap(),
                                source.family(),
                            )
                            .unwrap();
                            source
                                .subcurve(Real::from(3).into(), end, &policy)
                                .unwrap()
                                .value
                        } else {
                            source.clone()
                        };
                        let path = CurvePath2::try_new(vec![
                            Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
                            source,
                        ])
                        .unwrap();
                        for reversed in [false, true] {
                            let path = if reversed {
                                path.reversed(&policy).unwrap().value
                            } else {
                                path.clone()
                            };
                            let outcome = path.fillet_vertex_by_radius(1, Real::one(), CurveCornerMode2::TrimOnly, &policy)
                                .unwrap_or_else(|error| panic!("rational={rational}, selected={selected}, sharp={sharp_knot}, reversed={reversed}, policy={policy:?}: {error:?}"));
                            assert_eq!(outcome.certainty, CurveCertainty::Certified);
                            let candidates = match outcome.value {
                                CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                                CurveCornerSolutions2::Multiple(candidates) => candidates,
                                CurveCornerSolutions2::NoSolution(reason) => {
                                    panic!("lost spline fillets: {reason:?}")
                                }
                            };
                            assert_eq!(candidates.len(), if sharp_knot { 1 } else { 2 });
                            let expected = if sharp_knot {
                                vec![p(-3, 1)]
                            } else {
                                vec![p(0, 1), p(-3, 1)]
                            };
                            for point in expected {
                                assert!(
                                    candidates.iter().any(|candidate| candidate
                                        .curves()
                                        .windows(2)
                                        .any(|pair| pair[0]
                                            .end()
                                            .same_point(&point.clone().into(), &policy)
                                            == Classification::Decided(true))),
                                    "missing {point:?}"
                                );
                            }
                            for candidate in candidates {
                                assert_same(&candidate.start(), &path.start(), &policy);
                                assert_same(&candidate.end(), &path.end(), &policy);
                                for pair in candidate.curves().windows(2) {
                                    assert_same(&pair[0].end(), &pair[1].start(), &policy);
                                }
                                let edited = candidate
                                    .chamfer_vertex_by_setbacks(
                                        1,
                                        q(1, 64),
                                        q(1, 64),
                                        CurveCornerMode2::TrimOnly,
                                        &policy,
                                    )
                                    .unwrap();
                                assert_eq!(edited.certainty, CurveCertainty::Certified);
                                let CurveCornerSolutions2::Unique(edited) = edited.value else {
                                    panic!("a spline fillet must reenter chamfering")
                                };
                                assert_same(&edited.start(), &path.start(), &policy);
                                assert_same(&edited.end(), &path.end(), &policy);
                                for pair in edited.curves().windows(2) {
                                    assert_same(&pair[0].end(), &pair[1].start(), &policy);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_fillets_preserve_one_closed_authored_interval() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for rational in [false, true] {
                let controls = vec![p(0, 0), p(4, 0), p(4, 4), p(0, 4), p(0, 0)];
                let knots = [3, 3, 4, 5, 6, 7, 7].into_iter().map(Real::from).collect();
                let source = if rational {
                    Curve2::try_nurbs(1, controls, vec![Real::one(); 5], knots, &policy)
                } else {
                    Curve2::try_polynomial_bspline(1, controls, knots, &policy)
                }
                .unwrap()
                .value;
                let path = CurvePath2::try_new(vec![source]).unwrap();
                for reversed in [false, true] {
                    let path = if reversed {
                        path.reversed(&policy).unwrap().value
                    } else {
                        path.clone()
                    };
                    let result = path
                        .fillet_vertex_by_radius(
                            0,
                            Real::one(),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let CurveCornerSolutions2::Multiple(candidates) = result.value else {
                        panic!("four exact closed-interval fillets")
                    };
                    assert_eq!(candidates.len(), 4);
                    for candidate in candidates {
                        assert_same(&candidate.start(), &candidate.end(), &policy);
                        for pair in candidate.curves().windows(2) {
                            assert_same(&pair[0].end(), &pair[1].start(), &policy);
                        }
                        assert_eq!(candidate.curves().len(), 2);
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_fillets_preserve_unmarked_affine_bezier_contacts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let arc = Curve2::from(
                CircularArc2::try_from_center(
                    p(1, 0),
                    Point2::new(q(3, 5), q(4, 5)),
                    p(0, 0),
                    true,
                )
                .unwrap(),
            );
            let end = decided(
                selected_parameters(&policy)[0]
                    .affine_image_unbounded(&q(1, 64), &q(63, 64), &policy)
                    .unwrap(),
                arc.family(),
            )
            .unwrap();
            let arc = arc
                .subcurve(Real::zero().into(), end, &policy)
                .unwrap()
                .value;
            // No line-construction witness: these ordinary Bezier controls
            // must describe the same exact path as the native line above.
            let path = CurvePath2::try_new(vec![
                Curve2::from(QuadraticBezier2::new(p(-3, 0), p(-1, 0), p(1, 0))),
                arc,
            ])
            .unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let outcome = path
                    .fillet_vertex_by_radius(1, q(1, 4), CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let CurveCornerSolutions2::Multiple(candidates) = outcome.value else {
                    panic!(
                        "lost affine-Bezier fillets, reversed={reversed}, policy={policy:?}: {:?}",
                        outcome.value
                    )
                };
                assert_eq!(candidates.len(), 3);
                for candidate in candidates {
                    assert_same(&candidate.start(), &path.start(), &policy);
                    assert_same(&candidate.end(), &path.end(), &policy);
                    for pair in candidate.curves().windows(2) {
                        assert_same(&pair[0].end(), &pair[1].start(), &policy);
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_fillets_map_selected_line_contacts_into_spline_charts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let arc = Curve2::from(
                CircularArc2::try_from_center(
                    p(1, 0),
                    Point2::new(q(3, 5), q(4, 5)),
                    p(0, 0),
                    true,
                )
                .unwrap(),
            );
            let end = decided(
                selected_parameters(&policy)[0]
                    .affine_image_unbounded(&q(1, 64), &q(63, 64), &policy)
                    .unwrap(),
                arc.family(),
            )
            .unwrap();
            let arc = arc
                .subcurve(Real::zero().into(), end, &policy)
                .unwrap()
                .value;
            let line = Curve2::try_polynomial_bspline(
                1,
                vec![p(-4, 0), p(-3, 0), p(1, 0)],
                [3, 3, 4, 5, 5].into_iter().map(Real::from).collect(),
                &policy,
            )
            .unwrap()
            .value;
            let path = CurvePath2::try_new(vec![line, arc]).unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let outcome = path
                    .fillet_vertex_by_radius(1, q(1, 4), CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let CurveCornerSolutions2::Multiple(candidates) = outcome.value else {
                    panic!(
                        "all three contacts must retain authored parameters, reversed={reversed}, policy={policy:?}: {:?}",
                        outcome.value
                    )
                };
                assert_eq!(candidates.len(), 3);
                for candidate in candidates {
                    assert_same(&candidate.start(), &path.start(), &policy);
                    assert_same(&candidate.end(), &path.end(), &policy);
                    for pair in candidate.curves().windows(2) {
                        assert_same(&pair[0].end(), &pair[1].start(), &policy);
                    }
                }
            }
        }
    }

    #[test]
    fn source_domain_chamfers_cover_spline_knots_and_distinct_source_locations() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for rational in [false, true] {
                // The setback circle meets both the first and last spans.
                // At distance two its contacts are internal knots; each knot
                // must be published once, on the side retained by the trim.
                let controls = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2), p(0, 0)];
                let knots = [3, 3, 4, 5, 6, 7, 7].into_iter().map(Real::from).collect();
                let source = if rational {
                    Curve2::try_nurbs(
                        1,
                        controls,
                        vec![
                            Real::one(),
                            Real::from(2),
                            Real::from(3),
                            Real::from(2),
                            Real::one(),
                        ],
                        knots,
                        &policy,
                    )
                } else {
                    Curve2::try_polynomial_bspline(1, controls, knots, &policy)
                }
                .unwrap()
                .value;
                for selected in [false, true] {
                    let source = if selected {
                        let end = decided(
                            selected_parameters(&policy)[0]
                                .affine_image_unbounded(&q(1, 16), &q(111, 16), &policy)
                                .unwrap(),
                            source.family(),
                        )
                        .unwrap();
                        source
                            .subcurve(Real::from(3).into(), end, &policy)
                            .unwrap()
                            .value
                    } else {
                        source.clone()
                    };
                    let path = CurvePath2::try_new(vec![
                        Curve2::from(LineSeg2::try_new(p(0, -3), p(0, 0)).unwrap()),
                        source,
                    ])
                    .unwrap();
                    for reversed in [false, true] {
                        let path = if reversed {
                            path.reversed(&policy).unwrap().value
                        } else {
                            path.clone()
                        };
                        for setback in [1, 2] {
                            let setbacks = if reversed { [setback, 1] } else { [1, setback] };
                            let result = path
                                .chamfer_vertex_by_setbacks(
                                    1,
                                    Real::from(setbacks[0]),
                                    Real::from(setbacks[1]),
                                    CurveCornerMode2::TrimOnly,
                                    &policy,
                                )
                                .unwrap();
                            assert_eq!(result.certainty, CurveCertainty::Certified);
                            let CurveCornerSolutions2::Multiple(candidates) = result.value else {
                                panic!(
                                    "both spline contacts must survive, rational={rational}, selected={selected}, reversed={reversed}, setback={setback}"
                                )
                            };
                            assert_eq!(candidates.len(), 2);
                            for expected in [p(setback, 0), p(0, setback)] {
                                assert!(candidates.iter().any(|candidate| {
                                    candidate.curves().windows(2).any(|pair| {
                                        pair[0].end().same_point(&expected.clone().into(), &policy)
                                            == Classification::Decided(true)
                                    })
                                }));
                            }
                            for candidate in candidates {
                                assert_same(&candidate.start(), &path.start(), &policy);
                                assert_same(&candidate.end(), &path.end(), &policy);
                                for pair in candidate.curves().windows(2) {
                                    assert_same(&pair[0].end(), &pair[1].start(), &policy);
                                }
                            }
                        }
                    }
                }
            }
            // The same Cartesian contact at three distinct source locations
            // represents three different surviving path suffixes.
            let source = Curve2::try_polynomial_bspline(
                1,
                vec![p(0, 0), p(2, 0), p(0, 0), p(2, 0)],
                [0, 0, 1, 2, 3, 3].into_iter().map(Real::from).collect(),
                &policy,
            )
            .unwrap()
            .value;
            let path = CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(p(0, -2), p(0, 0)).unwrap()),
                source,
            ])
            .unwrap();
            let result = path
                .chamfer_vertex_by_setbacks(
                    1,
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Multiple(candidates) = result.value else {
                panic!("three distinct source locations")
            };
            assert_eq!(candidates.len(), 3);
            for start in [q(1, 2), q(3, 2), q(5, 2)] {
                assert!(candidates.iter().any(|candidate| {
                    let range = candidate.curves().last().unwrap().parameter_domain();
                    range
                        .start()
                        .cmp_by_refinement(&start.clone().into(), &policy)
                        .unwrap()
                        == Classification::Decided(Ordering::Equal)
                }));
            }
        }
    }

    #[test]
    fn source_domain_chamfers_retain_selected_centers_across_different_charts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let center =
                QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap())
                    .parallel_left(Real::zero())
                    .unwrap();
            let candidate =
                QuadraticBezier2::from_line_segment(LineSeg2::try_new(p(0, 1), p(4, 1)).unwrap())
                    .parallel_left(Real::zero())
                    .unwrap();
            let parameter = selected_parameters(&policy)[0].clone();
            // P(alpha)=(2 alpha,0), Q(v)=(4v,1). Unit distance
            // is tangent at v=alpha/2. Applying a same-chart translation
            // shortcut would return alpha +/- 1/4 and lose this contact.
            let expected = decided(
                parameter
                    .affine_image_unbounded(&q(1, 2), &Real::zero(), &policy)
                    .unwrap(),
                CurveFamily2::RationalBezier,
            )
            .unwrap();
            let result = resolve_certified_operation(&policy, |attempt| {
                decided(
                    candidate
                        .fixed_distance_incidence(
                            &center,
                            &parameter,
                            &Real::one(),
                            &CurveParameterRange2::new_validated(
                                Real::zero().into(),
                                Real::one().into(),
                            ),
                            None,
                            attempt,
                        )
                        .unwrap(),
                    CurveFamily2::RationalBezier,
                )
            })
            .unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(result.value.len(), 1);
            let actual = result.value.into_iter().next().unwrap();
            assert_eq!(
                actual.cmp_by_refinement(&expected, &policy).unwrap(),
                Classification::Decided(Ordering::Equal)
            );
            // A smaller radius misses the other support entirely.
            let result = resolve_certified_operation(&policy, |attempt| {
                decided(
                    candidate
                        .fixed_distance_incidence(
                            &center,
                            &parameter,
                            &q(1, 2),
                            &CurveParameterRange2::new_validated(
                                Real::zero().into(),
                                Real::one().into(),
                            ),
                            None,
                            attempt,
                        )
                        .unwrap(),
                    CurveFamily2::RationalBezier,
                )
            })
            .unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert!(result.value.is_empty());
        }
    }

    #[test]
    fn source_domain_chamfers_reenter_paths_with_a_selected_spline_corner() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = Curve2::try_polynomial_bspline(
                1,
                vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2), p(0, 0)],
                [3, 3, 4, 5, 6, 7, 7].into_iter().map(Real::from).collect(),
                &policy,
            )
            .unwrap()
            .value;
            let start = decided(
                selected_parameters(&policy)[0]
                    .affine_image_unbounded(&Real::one(), &Real::from(3), &policy)
                    .unwrap(),
                source.family(),
            )
            .unwrap();
            let source = source
                .subcurve(start, Real::from(7).into(), &policy)
                .unwrap()
                .value;
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(p(-2, 0).into(), source.start(), &policy)
                    .unwrap(),
                CurveFamily2::Line,
            )
            .unwrap();
            let path = CurvePath2::try_new(vec![
                Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(chord)),
                source,
            ])
            .unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                for (setback, expected_count) in [(Real::one(), 2), (q(1, 4), 1)] {
                    let setbacks = if reversed {
                        [setback, Real::zero()]
                    } else {
                        [Real::zero(), setback]
                    };
                    let result = path
                        .chamfer_vertex_by_setbacks(
                            1,
                            setbacks[0].clone(),
                            setbacks[1].clone(),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    let candidates = match result.value {
                        CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                        CurveCornerSolutions2::Multiple(candidates) => candidates,
                        CurveCornerSolutions2::NoSolution(reason) => {
                            panic!("selected spline corner lost its contacts: {reason:?}")
                        }
                    };
                    assert_eq!(candidates.len(), expected_count);
                    for candidate in candidates {
                        assert_same(&candidate.start(), &path.start(), &policy);
                        assert_same(&candidate.end(), &path.end(), &policy);
                        for pair in candidate.curves().windows(2) {
                            assert_same(&pair[0].end(), &pair[1].start(), &policy);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selected_circle_subdivision_preserves_frame_and_outer_tangency() {
        use crate::bezier_offset::{
            BezierAlgebraicCuspSemicircle2, BezierAlgebraicCuspSemicircleFragment2,
        };
        use crate::{
            BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let family = CurveFamily2::CircularArc;
            let polynomial = decided(
                BezierParameterPolynomial::try_new_power_basis(
                    vec![Real::from(-1), Real::zero(), Real::from(2)],
                    &policy,
                )
                .unwrap(),
                family,
            )
            .unwrap();
            let interval = decided(
                BezierParameterInterval::try_new(q(2, 3), q(3, 4), &policy).unwrap(),
                family,
            )
            .unwrap();
            let parameter = decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
                family,
            )
            .unwrap();
            let half = q(1, 2);
            let support = QuadraticBezier2::new(
                Point2::new(half.clone(), half.clone()),
                Point2::new(half.clone(), half.clone()),
                Point2::new(half.clone(), -half),
            )
            .parallel_left(Real::zero())
            .unwrap();
            let circle = decided(
                BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    support,
                    BezierParameter2::algebraic(parameter),
                    Real::one(),
                    true,
                    &policy,
                )
                .unwrap(),
                family,
            )
            .unwrap()
            .unwrap();
            let full = BezierAlgebraicCuspSemicircleFragment2::full(circle.clone(), &policy)
                .with_certified_tangent_endpoints();
            for reversed in [false, true] {
                let original = if reversed {
                    full.reversed()
                } else {
                    full.clone()
                };
                let source = Curve2::from_retained_fragment(
                    BezierSplitFragment2::AlgebraicCuspSemicircle(original.clone()),
                );
                let (first, second) = source.split_at(q(1, 2).into(), &policy).unwrap().value;
                let expected =
                    decided(circle.point_evidence_at(&q(1, 2), &policy).unwrap(), family).unwrap();
                assert_same(&first.end(), &expected, &policy);
                assert_same(&second.start(), &expected, &policy);
                for (part, tangency) in [(&first, [true, false]), (&second, [false, true])] {
                    let Some(BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)) =
                        part.retained_fragment()
                    else {
                        panic!("retained circle")
                    };
                    assert_eq!(fragment.semicircle(), &circle);
                    assert_eq!(
                        [
                            fragment.certified_tangent_endpoint(true),
                            fragment.certified_tangent_endpoint(false)
                        ],
                        tangency
                    );
                }
                let middle = source
                    .subcurve(q(1, 4).into(), q(3, 4).into(), &policy)
                    .unwrap()
                    .value;
                assert_same(
                    &middle.point_at(&q(1, 2).into(), &policy).unwrap().value,
                    &expected,
                    &policy,
                );
                // Setback points at represented angular parameters need the
                // same general frame as selected angular endpoints.
                for endpoint in [true, false] {
                    assert!(matches!(
                        original
                            .endpoint_chord_setback_cut(endpoint, &q(1, 64), false, &policy)
                            .unwrap(),
                        Classification::Decided(Some(_))
                    ));
                }
            }
        }
    }

    #[test]
    fn selected_chord_subdivision_reuses_incidence_and_traversal() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let line = Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap());
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                    line.start(),
                    line.end(),
                    &policy,
                )
                .unwrap(),
                CurveFamily2::Line,
            )
            .unwrap();
            for reversed in [false, true] {
                let chord = if reversed {
                    chord.reversed()
                } else {
                    chord.clone()
                };
                let source = Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(
                    chord.clone(),
                ));
                let [start, probe, end] = selected_parameters(&policy).map(|parameter| {
                    let point = line.point_at(&parameter, &policy).unwrap().value;
                    CurveParameter2::from_algebraic_chord(
                        chord.parameter_at_certified_interior_point(point),
                    )
                });
                let (start, end) = if reversed { (end, start) } else { (start, end) };
                let restricted = source
                    .subcurve(start.clone(), end.clone(), &policy)
                    .unwrap()
                    .value;
                let expected = source.point_at(&probe, &policy).unwrap().value;
                let (first, second) = restricted.split_at(probe.clone(), &policy).unwrap().value;
                assert_same(&first.start(), &restricted.start(), &policy);
                assert_same(&first.end(), &expected, &policy);
                assert_same(&second.start(), &expected, &policy);
                assert_same(&second.end(), &restricted.end(), &policy);
                let repeated = restricted.subcurve(start, probe, &policy).unwrap().value;
                assert_same(&repeated.start(), &restricted.start(), &policy);
                assert_same(&repeated.end(), &expected, &policy);
            }
        }
    }

    #[test]
    fn selected_source_range_regions_close_through_boolean_offset_and_boolean() {
        use crate::{
            BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial,
            CurveRegion2, OffsetCornerStyle2, RegionPointLocation,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let family = CurveFamily2::QuadraticBezier;
            let polynomial = decided(
                BezierParameterPolynomial::try_new_power_basis(
                    vec![Real::from(-1), Real::zero(), Real::from(2)],
                    &policy,
                )
                .unwrap(),
                family,
            )
            .unwrap();
            let interval = decided(
                BezierParameterInterval::try_new(q(2, 3), q(3, 4), &policy).unwrap(),
                family,
            )
            .unwrap();
            let parameter = decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
                family,
            )
            .unwrap();
            let start = decided(
                CurveParameter2::from(BezierParameter2::algebraic(parameter))
                    .affine_image_unbounded(&q(1, 4), &Real::zero(), &policy)
                    .unwrap(),
                family,
            )
            .unwrap();
            let end = decided(
                start
                    .affine_image_unbounded(&Real::from(-1), &Real::one(), &policy)
                    .unwrap(),
                family,
            )
            .unwrap();
            let source = Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0)))
                .subcurve(start, end, &policy)
                .unwrap()
                .value;
            assert!(source.source_range().is_some());
            let closing = decided(
                crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                    source.end(),
                    source.start(),
                    &policy,
                )
                .unwrap(),
                CurveFamily2::Line,
            )
            .unwrap();
            let path = CurvePath2::try_new(vec![
                source,
                Curve2::from_retained_fragment(BezierSplitFragment2::AlgebraicChord(closing)),
            ])
            .unwrap();
            let region = CurveRegion2::try_from_boundary_paths(&[path], &policy)
                .unwrap()
                .value;
            let corners = [p(2, 0), p(4, 0), p(4, 3), p(2, 3)];
            let cutter = CurvePath2::try_new(
                (0..4)
                    .map(|index| {
                        Curve2::from(
                            LineSeg2::try_new(
                                corners[index].clone(),
                                corners[(index + 1) % 4].clone(),
                            )
                            .unwrap(),
                        )
                    })
                    .collect(),
            )
            .unwrap();
            let cutter = CurveRegion2::try_from_boundary_paths(&[cutter], &policy)
                .unwrap()
                .value;
            let result = region.boolean_regions(&cutter, &policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            for (region, expected) in [
                (result.value.union(), [true, true, true, false]),
                (result.value.intersection(), [false, true, false, false]),
                (result.value.difference(), [true, false, false, false]),
                (result.value.xor(), [true, false, true, false]),
            ] {
                for (query, inside) in [
                    Point2::new(q(3, 2), q(3, 2)),
                    Point2::new(q(5, 2), q(3, 2)),
                    p(3, 1),
                    p(0, 0),
                ]
                .into_iter()
                .zip(expected)
                {
                    let actual = region.classify_point(&query, &policy).unwrap();
                    assert_eq!(actual.certainty, CurveCertainty::Certified);
                    assert_eq!(
                        actual.value,
                        Classification::Decided(if inside {
                            RegionPointLocation::Inside
                        } else {
                            RegionPointLocation::Outside
                        })
                    );
                }
            }
            #[cfg(feature = "dispatch-trace")]
            let _trace = hyperreal::dispatch_trace::recording_scope();
            let expanded = result
                .value
                .difference()
                .offset(q(1, 16), &OffsetCornerStyle2::Round, &policy)
                .unwrap_or_else(|error| {
                    #[cfg(feature = "dispatch-trace")]
                    for entry in hyperreal::dispatch_trace::take()
                        .into_iter()
                        .filter(|entry| entry.layer == "hypercurve")
                    {
                        eprintln!("{entry:?}");
                    }
                    panic!("offset after selected subdivision and Boolean: {error:?}");
                });
            assert_eq!(expanded.certainty, CurveCertainty::Certified);
            let final_result = expanded.value.boolean_regions(&cutter, &policy).unwrap();
            assert_eq!(final_result.certainty, CurveCertainty::Certified);
            for (query, expected) in [
                (Point2::new(q(65, 32), q(3, 2)), RegionPointLocation::Inside),
                (Point2::new(q(3, 2), q(3, 2)), RegionPointLocation::Outside),
            ] {
                let actual = final_result
                    .value
                    .intersection()
                    .classify_point(&query, &policy)
                    .unwrap();
                assert_eq!(actual.certainty, CurveCertainty::Certified);
                assert_eq!(actual.value, Classification::Decided(expected));
            }
        }
    }
}
