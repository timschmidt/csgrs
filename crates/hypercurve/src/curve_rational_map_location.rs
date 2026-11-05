//! Query retained rational maps without replacing their source geometry.
use super::*;
use crate::curve_support::CurveSupport2;
use crate::{BezierSplitFragment2, RationalMap2};

impl Curve2 {
    /// All preimages of `point` through `map(self(t))`, in this curve's
    /// original parameter chart. Original source and map denominator zeros
    /// remain excluded, including removable poles. No image curve is fitted
    /// or published. Native rational/spline spans and retained polynomial
    /// branches share their incidence authority with ordinary point location.
    /// Continuous preimages covering the complete defined source return
    /// `EntireCurve` only after a proof that no pole punctures the domain.
    /// Discontinuous knot sides keep distinct source-chart locations even if
    /// their mapped point and authored parameter agree; continuous seams merge.
    /// Proper continuous and mixed preimages retain exact span charts and
    /// endpoint inclusion in `Components`; original poles remain open ends.
    pub fn rational_image_point_locations(
        &self,
        map: &RationalMap2,
        point: &CurvePoint2,
    ) -> ExactCurveResult<CurvePointLocations2> {
        resolve_certified_operation(&crate::policy::principal_context(), |policy| {
            self.rational_image_point_locations_raw(map, point, policy)
        })
        .map(CurveOutcome::into_value)
    }

    pub(super) fn identity_point_locations_raw(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePointLocations2> {
        let identity = crate::RationalMap2::new(
            [
                hypersolve::BivariatePolynomial::new(vec![vec![], vec![Real::one()]]),
                hypersolve::BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
            ],
            hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
        );
        self.rational_image_point_locations_raw(&identity, point, policy)
    }

    pub(super) fn rational_image_point_locations_raw(
        &self,
        map: &RationalMap2,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePointLocations2> {
        let mut zero_denominator = true;
        for row in &map.denominator().coefficients {
            if !crate::curve::polynomial_is_zero(row, policy)? {
                zero_denominator = false;
                break;
            }
        }
        if zero_denominator {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                self.family(),
                CurveError::ZeroRationalPointDenominator,
            ));
        }
        if let Some(branch) = self.polynomial_branch() {
            return Ok(match branch.mapped_point_parameters(point, map, policy)? {
                None => CurvePointLocations2::EntireCurve,
                Some(parameters) => CurvePointLocations2::Locations(
                    parameters
                        .into_iter()
                        .map(|p| {
                            CurveLocation2::new(
                                0,
                                CurveSpanRange2::from_affine_chart(&Real::one(), &Real::zero()),
                                p,
                            )
                        })
                        .collect(),
                ),
            });
        }
        let blocked = || {
            ExactCurveError::blocked(
                CurveOperation2::Intersection,
                self.family(),
                UncertaintyReason::Unsupported,
            )
        };
        let spans = self.source_spans(policy, CurveOperation2::Intersection)?;
        let mut components = Vec::new();
        let mut entire = 0;
        for (index, span) in spans.iter().enumerate() {
            let CurveSupport2::Bezier(source) = CurveSupport2::from_fragment(&span.fragment) else {
                return Err(blocked());
            };
            let source = RationalBezier2::try_from_subcurve(&source).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Intersection, self.family(), cause)
            })?;
            let range = if matches!(span.fragment, BezierSplitFragment2::Materialized { .. }) {
                CurveParameterRange2::unit()
            } else {
                span.fragment.curve_region_parameter_range()
            };
            let parameters = curve_rational_location::mapped_parameters(
                &source,
                &range,
                point,
                self.family(),
                policy,
                map,
            )?;
            let ordered = ExactCurveError::decided(
                CurveOperation2::Intersection,
                range.ordered_endpoints(policy).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Intersection, self.family(), cause)
                })?,
            )?;
            if parameters.len() == 1
                && parameters[0].1 == [true, true]
                && matches!(&parameters[0].0,CurveParameterSet2::Range(r) if r.start()==ordered[0] && r.end()==ordered[1])
            {
                entire += 1;
            }
            components.extend(parameters.into_iter().map(|(p, inclusion)| {
                CurveLocationComponent2::new(index, span.chart(), p, inclusion)
            }));
        }
        if entire == spans.len() {
            return Ok(CurvePointLocations2::EntireCurve);
        }
        self.normalize_point_components(components, policy)
    }
}
