//! Minimal staged offset support for Bezier and rational-conic curves.
//!
//! The only constructed offset primitive here is the proven line-image case.
//! Free-form Bezier and conic offsets remain explicit unresolved candidates
//! until analytic/fitted offset machinery can provide its own error and
//! trimming proof.

use hyperreal::{RealSign, ZeroKnowledge as ZeroStatus};

use crate::classify::real_sign;
use crate::{
    BezierCuspClassification, BezierDegree, BezierEndpoint, BezierInflectionClassification,
    BezierLineImageFitRelation, CertifiedBezierLineImageOffset2, Classification, CubicBezier2,
    CurveError, CurvePolicy, CurveResult, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real,
    UncertaintyReason,
};

/// Exact source-curve hazard that must be resolved before a Bezier offset is
/// treated as a topology product.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BezierOffsetRisk {
    /// The entire source curve is certified to be one point.
    DegeneratePoint,
    /// The source has at least one certified cusp where the normal is undefined.
    Cusp,
    /// A cubic has certified inflection parameters where the normal field can flip.
    Inflection,
    /// The curvature numerator is structurally zero over the whole cubic.
    AllCurvatureZero,
    /// The first derivative is certified zero at the given endpoint.
    UndefinedEndpointNormal {
        /// Endpoint whose first derivative is zero.
        endpoint: BezierEndpoint,
    },
    /// Structural inspection could not prove whether the endpoint derivative is nonzero.
    UnresolvedEndpointNormal {
        /// Endpoint whose first derivative status is unknown.
        endpoint: BezierEndpoint,
    },
    /// The source endpoints are structurally coincident.
    CoincidentEndpoints,
    /// A rational Bezier denominator can cross or touch zero on the affine interval.
    ProjectiveDenominatorBoundary,
}

/// Exact source analysis retained before a staged Bezier offset candidate is built.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierOffsetPreflight2 {
    degree: BezierDegree,
    cusp_classification: BezierCuspClassification,
    inflection_classification: BezierInflectionClassification,
    start_tangent_status: ZeroStatus,
    end_tangent_status: ZeroStatus,
    endpoint_coincidence: ZeroStatus,
    risks: Vec<BezierOffsetRisk>,
    construction_policy: CurvePolicy,
}

/// Result of a staged Bezier/conic offset attempt.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierOffsetCandidate2 {
    /// The source was certified to be one endpoint line image and offset exactly.
    ExactLineImage {
        /// Exact primitive offset of the certified endpoint line image.
        offset: CertifiedBezierLineImageOffset2,
        /// Exact source analysis retained from the staged preflight.
        preflight: BezierOffsetPreflight2,
    },
    /// The source is not yet supported by a certified analytic/fitted offset.
    Unresolved {
        /// Exact source analysis for the unresolved curve.
        preflight: BezierOffsetPreflight2,
        /// Signed distance along the curve's left normal.
        distance: Real,
    },
}

impl BezierOffsetPreflight2 {
    /// Returns the Bezier degree covered by this preflight.
    pub const fn degree(&self) -> BezierDegree {
        self.degree
    }

    /// Returns the exact cusp classification used by offset preflight.
    pub const fn cusp_classification(&self) -> &BezierCuspClassification {
        &self.cusp_classification
    }

    /// Returns the exact inflection classification used by offset preflight.
    pub const fn inflection_classification(&self) -> &BezierInflectionClassification {
        &self.inflection_classification
    }

    /// Returns structural zero knowledge for the start endpoint derivative.
    pub const fn start_tangent_status(&self) -> ZeroStatus {
        self.start_tangent_status
    }

    /// Returns structural zero knowledge for the end endpoint derivative.
    pub const fn end_tangent_status(&self) -> ZeroStatus {
        self.end_tangent_status
    }

    /// Returns structural zero knowledge for source endpoint coincidence.
    pub const fn endpoint_coincidence(&self) -> ZeroStatus {
        self.endpoint_coincidence
    }

    /// Returns the exact or unresolved risks detected before offset fitting.
    pub fn risks(&self) -> &[BezierOffsetRisk] {
        &self.risks
    }

    /// Returns true when no currently implemented exact preflight risk remains.
    pub fn is_clear(&self) -> bool {
        self.risks.is_empty()
    }

    /// Returns the policy used to prove this preflight.
    pub const fn construction_policy(&self) -> &CurvePolicy {
        &self.construction_policy
    }
}

impl BezierOffsetCandidate2 {
    /// Returns the preflight retained by this staged candidate.
    pub const fn preflight(&self) -> &BezierOffsetPreflight2 {
        match self {
            Self::ExactLineImage { preflight, .. } | Self::Unresolved { preflight, .. } => {
                preflight
            }
        }
    }

    /// Returns the exact primitive offset, if one was constructed.
    pub const fn exact_line_image_offset(&self) -> Option<&CertifiedBezierLineImageOffset2> {
        match self {
            Self::ExactLineImage { offset, .. } => Some(offset),
            Self::Unresolved { .. } => None,
        }
    }

    /// Returns the unresolved preflight when no primitive offset was constructed.
    pub const fn unresolved_preflight(&self) -> Option<&BezierOffsetPreflight2> {
        match self {
            Self::ExactLineImage { .. } => None,
            Self::Unresolved { preflight, .. } => Some(preflight),
        }
    }

    /// Returns the signed distance along the curve's left normal.
    pub const fn distance(&self) -> &Real {
        match self {
            Self::ExactLineImage { offset, .. } => offset.distance(),
            Self::Unresolved { distance, .. } => distance,
        }
    }
}

impl QuadraticBezier2 {
    /// Runs exact source analysis for later offset adapters.
    pub fn offset_preflight(&self, policy: &CurvePolicy) -> Classification<BezierOffsetPreflight2> {
        let cusp_classification = match self.cusp_classification(policy) {
            Classification::Decided(classification) => classification,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let inflection_classification = self.inflection_classification();
        let start_tangent_status = self.endpoint_tangent(BezierEndpoint::Start).zero_status();
        let end_tangent_status = self.endpoint_tangent(BezierEndpoint::End).zero_status();
        let endpoint_coincidence = self.endpoints_coincident_status();
        Classification::Decided(build_preflight(
            BezierDegree::Quadratic,
            cusp_classification,
            inflection_classification,
            start_tangent_status,
            end_tangent_status,
            endpoint_coincidence,
            policy,
        ))
    }

    /// Attempts a staged certified left offset of this quadratic Bezier.
    pub fn offset_left_staged(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, distance, policy)
    }

    /// Attempts a staged certified right offset of this quadratic Bezier.
    pub fn offset_right_staged(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, -distance, policy)
    }
}

impl CubicBezier2 {
    /// Runs exact source analysis for later offset adapters.
    pub fn offset_preflight(&self, policy: &CurvePolicy) -> Classification<BezierOffsetPreflight2> {
        let cusp_classification = match self.cusp_classification(policy) {
            Classification::Decided(classification) => classification,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let inflection_classification = match self.inflection_classification(policy) {
            Classification::Decided(classification) => classification,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let start_tangent_status = self.endpoint_tangent(BezierEndpoint::Start).zero_status();
        let end_tangent_status = self.endpoint_tangent(BezierEndpoint::End).zero_status();
        let endpoint_coincidence = self.endpoints_coincident_status();
        Classification::Decided(build_preflight(
            BezierDegree::Cubic,
            cusp_classification,
            inflection_classification,
            start_tangent_status,
            end_tangent_status,
            endpoint_coincidence,
            policy,
        ))
    }

    /// Attempts a staged certified left offset of this cubic Bezier.
    pub fn offset_left_staged(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, distance, policy)
    }

    /// Attempts a staged certified right offset of this cubic Bezier.
    pub fn offset_right_staged(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, -distance, policy)
    }
}

impl RationalQuadraticBezier2 {
    /// Runs exact source analysis for later rational-conic offset adapters.
    pub fn offset_preflight(&self, policy: &CurvePolicy) -> Classification<BezierOffsetPreflight2> {
        let denominator_risk =
            match weights_known_same_nonzero_sign(self.weights().as_slice(), policy) {
                Some(true) => false,
                Some(false) => true,
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            };
        let start_tangent_status = rational_endpoint_delta_status(self.start(), self.control());
        let end_tangent_status = rational_endpoint_delta_status(self.control(), self.end());
        let endpoint_coincidence = self.start().distance_squared(self.end()).zero_status();
        let mut preflight = build_preflight(
            BezierDegree::Quadratic,
            BezierCuspClassification::None,
            BezierInflectionClassification::NotApplicable,
            start_tangent_status,
            end_tangent_status,
            endpoint_coincidence,
            policy,
        );
        if denominator_risk {
            preflight
                .risks
                .push(BezierOffsetRisk::ProjectiveDenominatorBoundary);
        }
        if rational_collapsed_point_status(self) == ZeroStatus::Zero
            && !preflight.risks.contains(&BezierOffsetRisk::DegeneratePoint)
        {
            preflight.risks.insert(0, BezierOffsetRisk::DegeneratePoint);
        }
        Classification::Decided(preflight)
    }

    /// Attempts a staged certified left offset of this rational quadratic conic.
    pub fn offset_left_staged(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, distance, policy)
    }

    /// Attempts a staged certified right offset of this rational quadratic conic.
    pub fn offset_right_staged(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, -distance, policy)
    }
}

trait StagedBezierOffset {
    fn offset_preflight(&self, policy: &CurvePolicy) -> Classification<BezierOffsetPreflight2>;
    fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>>;
}

impl StagedBezierOffset for QuadraticBezier2 {
    fn offset_preflight(&self, policy: &CurvePolicy) -> Classification<BezierOffsetPreflight2> {
        QuadraticBezier2::offset_preflight(self, policy)
    }

    fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        QuadraticBezier2::fit_exact_line_image(self, policy)
    }
}

impl StagedBezierOffset for CubicBezier2 {
    fn offset_preflight(&self, policy: &CurvePolicy) -> Classification<BezierOffsetPreflight2> {
        CubicBezier2::offset_preflight(self, policy)
    }

    fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        CubicBezier2::fit_exact_line_image(self, policy)
    }
}

impl StagedBezierOffset for RationalQuadraticBezier2 {
    fn offset_preflight(&self, policy: &CurvePolicy) -> Classification<BezierOffsetPreflight2> {
        RationalQuadraticBezier2::offset_preflight(self, policy)
    }

    fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        RationalQuadraticBezier2::fit_exact_line_image(self, policy)
    }
}

fn staged_offset_left<C>(
    curve: &C,
    distance: Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BezierOffsetCandidate2>>
where
    C: StagedBezierOffset,
{
    let preflight = match curve.offset_preflight(policy) {
        Classification::Decided(preflight) => preflight,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let line_image_fit = match curve.fit_exact_line_image(policy) {
        Ok(relation) => relation,
        Err(CurveError::ZeroLengthLine)
            if preflight.risks.contains(&BezierOffsetRisk::DegeneratePoint) =>
        {
            return Ok(Classification::Decided(
                BezierOffsetCandidate2::Unresolved {
                    preflight,
                    distance,
                },
            ));
        }
        Err(error) => return Err(error),
    };
    match line_image_fit {
        Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => Ok(
            Classification::Decided(BezierOffsetCandidate2::ExactLineImage {
                offset: fit.offset_left_exact(distance)?,
                preflight,
            }),
        ),
        Classification::Decided(BezierLineImageFitRelation::NotLine) => Ok(
            Classification::Decided(BezierOffsetCandidate2::Unresolved {
                preflight,
                distance,
            }),
        ),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn rational_endpoint_delta_status(first: &Point2, second: &Point2) -> ZeroStatus {
    first.distance_squared(second).zero_status()
}

fn rational_collapsed_point_status(curve: &RationalQuadraticBezier2) -> ZeroStatus {
    let start_control = curve
        .start()
        .distance_squared(curve.control())
        .zero_status();
    let control_end = curve.control().distance_squared(curve.end()).zero_status();
    match (start_control, control_end) {
        (ZeroStatus::Zero, ZeroStatus::Zero) => ZeroStatus::Zero,
        (ZeroStatus::NonZero, _) | (_, ZeroStatus::NonZero) => ZeroStatus::NonZero,
        _ => ZeroStatus::Unknown,
    }
}

fn weights_known_same_nonzero_sign(weights: &[&Real], policy: &CurvePolicy) -> Option<bool> {
    let mut expected = None;
    for weight in weights {
        let sign = real_sign(weight, policy)?;
        match sign {
            RealSign::Positive | RealSign::Negative => {
                if let Some(expected) = expected {
                    if expected != sign {
                        return Some(false);
                    }
                } else {
                    expected = Some(sign);
                }
            }
            RealSign::Zero => return Some(false),
        }
    }
    Some(expected.is_some())
}

fn build_preflight(
    degree: BezierDegree,
    cusp_classification: BezierCuspClassification,
    inflection_classification: BezierInflectionClassification,
    start_tangent_status: ZeroStatus,
    end_tangent_status: ZeroStatus,
    endpoint_coincidence: ZeroStatus,
    policy: &CurvePolicy,
) -> BezierOffsetPreflight2 {
    let mut risks = Vec::new();
    match &cusp_classification {
        BezierCuspClassification::DegeneratePoint => risks.push(BezierOffsetRisk::DegeneratePoint),
        BezierCuspClassification::Cusps { .. } => risks.push(BezierOffsetRisk::Cusp),
        BezierCuspClassification::Unresolved => risks.push(BezierOffsetRisk::Cusp),
        BezierCuspClassification::None => {}
    }
    match &inflection_classification {
        BezierInflectionClassification::Inflections { .. } => {
            risks.push(BezierOffsetRisk::Inflection);
        }
        BezierInflectionClassification::AllCurvatureZero => {
            risks.push(BezierOffsetRisk::AllCurvatureZero);
        }
        BezierInflectionClassification::Unresolved => risks.push(BezierOffsetRisk::Inflection),
        BezierInflectionClassification::NotApplicable | BezierInflectionClassification::None => {}
    }
    push_endpoint_normal_risk(&mut risks, BezierEndpoint::Start, start_tangent_status);
    push_endpoint_normal_risk(&mut risks, BezierEndpoint::End, end_tangent_status);
    if endpoint_coincidence == ZeroStatus::Zero {
        risks.push(BezierOffsetRisk::CoincidentEndpoints);
    }
    BezierOffsetPreflight2 {
        degree,
        cusp_classification,
        inflection_classification,
        start_tangent_status,
        end_tangent_status,
        endpoint_coincidence,
        risks,
        construction_policy: policy.clone(),
    }
}

fn push_endpoint_normal_risk(
    risks: &mut Vec<BezierOffsetRisk>,
    endpoint: BezierEndpoint,
    zero_status: ZeroStatus,
) {
    match zero_status {
        ZeroStatus::Zero => risks.push(BezierOffsetRisk::UndefinedEndpointNormal { endpoint }),
        ZeroStatus::Unknown => risks.push(BezierOffsetRisk::UnresolvedEndpointNormal { endpoint }),
        ZeroStatus::NonZero => {}
    }
}
