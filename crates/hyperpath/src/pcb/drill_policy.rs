//! Exact retained via fabrication policy reports.
//!
//! Via fabrication rules mix discrete topology, drill process intent, and
//! exact geometry. This module keeps those pieces explicit instead of folding
//! them into one tolerance-driven accept/reject flag. It retains exact via and
//! policy data, then exposes every predicate used to accept or reject the candidate. The
//! individual checks mirror IPC-2221-style PCB design-rule practice, where
//! annular ring, drill aspect ratio, plated/non-plated intent, and blind/
//! buried/through transition capability are independent fabrication rules.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealSign};

use crate::pcb::{
    PcbViaStack, ViaAnnularRingReport, ViaDrillIntent, ViaDrillPolicyClass, ViaDrillPolicyReport,
    ViaLayerTransitionClass, ViaLayerTransitionReport,
};

/// Exact via-fabrication envelope for a board and process capability.
///
/// `maximum_aspect_ratio` is the process limit for `board_thickness / drill`.
/// The comparison is replayed denominator-free as
/// `board_thickness <= drill_diameter * maximum_aspect_ratio`, so exact
/// rational rules do not need a floating aspect-ratio division.
#[derive(Clone, Debug, PartialEq)]
pub struct ViaFabricationPolicy {
    /// Total copper layer count in the board stackup.
    pub board_layer_count: u16,
    /// Exact finished board thickness in the same length unit as drill diameter.
    pub board_thickness: Real,
    /// Exact minimum annular ring for plated drills.
    pub minimum_annular_ring: Real,
    /// Maximum allowed drill aspect ratio.
    pub maximum_aspect_ratio: Real,
    /// Whether layer-transition vias must have plated drill intent.
    pub require_plated_for_layer_transition: bool,
    /// Whether single-layer lands are permitted.
    pub allow_single_layer_land: bool,
    /// Whether blind vias are permitted.
    pub allow_blind: bool,
    /// Whether buried vias are permitted.
    pub allow_buried: bool,
    /// Whether through vias are permitted.
    pub allow_through: bool,
}

impl ViaFabricationPolicy {
    /// Construct a conservative policy that permits through vias only.
    pub fn through_only(
        board_layer_count: u16,
        board_thickness: Real,
        minimum_annular_ring: Real,
        maximum_aspect_ratio: Real,
    ) -> Self {
        Self {
            board_layer_count,
            board_thickness,
            minimum_annular_ring,
            maximum_aspect_ratio,
            require_plated_for_layer_transition: true,
            allow_single_layer_land: false,
            allow_blind: false,
            allow_buried: false,
            allow_through: true,
        }
    }

    /// Construct a policy that permits every retained via transition class.
    pub fn all_transitions(
        board_layer_count: u16,
        board_thickness: Real,
        minimum_annular_ring: Real,
        maximum_aspect_ratio: Real,
    ) -> Self {
        Self {
            board_layer_count,
            board_thickness,
            minimum_annular_ring,
            maximum_aspect_ratio,
            require_plated_for_layer_transition: true,
            allow_single_layer_land: true,
            allow_blind: true,
            allow_buried: true,
            allow_through: true,
        }
    }
}

/// Exact drill aspect-ratio certification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaAspectRatioReport {
    /// `board_thickness <= drill_diameter * maximum_aspect_ratio` was certified.
    Certified,
    /// The retained drill is too small for the board thickness/process ratio.
    Violation,
    /// No drill diameter was retained.
    UnknownNoDrill,
    /// Exact comparison could not decide the ratio predicate.
    Unknown,
}

/// Exact transition-policy certification for one via.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ViaTransitionPolicyReport {
    /// Discrete layer-transition classification.
    pub transition: ViaLayerTransitionReport,
    /// Whether the policy allows this transition class.
    pub allowed: bool,
}

/// Overall fabrication-envelope acceptance class.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaFabricationAcceptance {
    /// Every retained rule was certified as acceptable.
    Accepted,
    /// At least one retained rule was certified as rejected.
    Rejected,
    /// No rejection was certified, but at least one required rule is unknown.
    Unknown,
}

/// Exact retained via fabrication report.
#[derive(Clone, Debug, PartialEq)]
pub struct ViaFabricationReport {
    /// Discrete layer-transition policy result.
    pub transition_policy: ViaTransitionPolicyReport,
    /// Retained drill intent and annular-ring policy result.
    pub drill_policy: ViaDrillPolicyReport,
    /// Exact board-thickness/drill aspect-ratio result.
    pub aspect_ratio: ViaAspectRatioReport,
    /// Final acceptance class derived from the retained subreports.
    pub acceptance: ViaFabricationAcceptance,
}

/// Errors while constructing via fabrication-envelope reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaFabricationError {
    /// The board layer count was zero.
    InvalidBoardLayerCount,
    /// The via span was outside the board layer count.
    ViaOutsideBoardStackup,
    /// Board thickness must be strictly positive.
    NonPositiveBoardThickness,
    /// Maximum aspect ratio must be strictly positive.
    NonPositiveAspectRatio,
    /// Minimum annular ring must be nonnegative.
    NegativeAnnularRing,
}

/// Certify one retained via stack against a fabrication policy.
///
/// This function is deliberately stricter than basic clearance predicates. A
/// via can be geometrically clear yet fail fabrication because the process
/// forbids blind/buried structures, lacks plated intent for a layer
/// transition, violates annular ring, or exceeds the drill aspect ratio. Each
/// predicate remains visible in the returned report so importers and rule
/// checkers can distinguish hard rejections from unknown exact comparisons.
pub fn certify_via_fabrication_policy(
    via: &PcbViaStack,
    policy: &ViaFabricationPolicy,
    predicate_policy: PredicatePolicy,
) -> Result<ViaFabricationReport, ViaFabricationError> {
    validate_policy(policy, predicate_policy)?;
    let transition = via
        .classify_layer_transition(policy.board_layer_count)
        .map_err(|error| match error {
            "board layer count must be positive" => ViaFabricationError::InvalidBoardLayerCount,
            "via layer span exceeds board layer count" => {
                ViaFabricationError::ViaOutsideBoardStackup
            }
            _ => ViaFabricationError::ViaOutsideBoardStackup,
        })?;
    let transition_policy = ViaTransitionPolicyReport {
        allowed: transition_allowed(transition.class, policy),
        transition,
    };
    let drill_policy = via.classify_drill_policy(&policy.minimum_annular_ring, predicate_policy);
    let aspect_ratio = certify_aspect_ratio(via, policy, predicate_policy);
    let acceptance =
        classify_acceptance(via, policy, &transition_policy, &drill_policy, aspect_ratio);

    Ok(ViaFabricationReport {
        transition_policy,
        drill_policy,
        aspect_ratio,
        acceptance,
    })
}

fn validate_policy(
    policy: &ViaFabricationPolicy,
    predicate_policy: PredicatePolicy,
) -> Result<(), ViaFabricationError> {
    if policy.board_layer_count == 0 {
        return Err(ViaFabricationError::InvalidBoardLayerCount);
    }
    if compare_reals_with_policy(&policy.board_thickness, &Real::zero(), predicate_policy).value()
        != Some(Ordering::Greater)
    {
        return Err(ViaFabricationError::NonPositiveBoardThickness);
    }
    if compare_reals_with_policy(
        &policy.maximum_aspect_ratio,
        &Real::zero(),
        predicate_policy,
    )
    .value()
        != Some(Ordering::Greater)
    {
        return Err(ViaFabricationError::NonPositiveAspectRatio);
    }
    if policy.minimum_annular_ring.structural_facts().sign == Some(RealSign::Negative) {
        return Err(ViaFabricationError::NegativeAnnularRing);
    }
    Ok(())
}

fn certify_aspect_ratio(
    via: &PcbViaStack,
    policy: &ViaFabricationPolicy,
    predicate_policy: PredicatePolicy,
) -> ViaAspectRatioReport {
    let Some(drill) = via.drill_diameter() else {
        return ViaAspectRatioReport::UnknownNoDrill;
    };
    let allowed_thickness = drill.clone() * policy.maximum_aspect_ratio.clone();
    match compare_reals_with_policy(
        &policy.board_thickness,
        &allowed_thickness,
        predicate_policy,
    )
    .value()
    {
        Some(Ordering::Less | Ordering::Equal) => ViaAspectRatioReport::Certified,
        Some(Ordering::Greater) => ViaAspectRatioReport::Violation,
        None => ViaAspectRatioReport::Unknown,
    }
}

fn classify_acceptance(
    via: &PcbViaStack,
    policy: &ViaFabricationPolicy,
    transition_policy: &ViaTransitionPolicyReport,
    drill_policy: &ViaDrillPolicyReport,
    aspect_ratio: ViaAspectRatioReport,
) -> ViaFabricationAcceptance {
    if !transition_policy.allowed
        || matches!(aspect_ratio, ViaAspectRatioReport::Violation)
        || matches!(
            drill_policy.annular_ring,
            Some(ViaAnnularRingReport::Violation | ViaAnnularRingReport::InvalidMinimum)
        )
        || (policy.require_plated_for_layer_transition
            && transition_policy.transition.class != ViaLayerTransitionClass::SingleLayerLand
            && via.drill_intent() == ViaDrillIntent::NonPlated)
    {
        return ViaFabricationAcceptance::Rejected;
    }
    if matches!(
        aspect_ratio,
        ViaAspectRatioReport::Unknown | ViaAspectRatioReport::UnknownNoDrill
    ) || matches!(
        drill_policy.class,
        ViaDrillPolicyClass::MissingDrill | ViaDrillPolicyClass::UnspecifiedDrilledHole
    ) || matches!(
        drill_policy.annular_ring,
        Some(ViaAnnularRingReport::Unknown)
    ) {
        ViaFabricationAcceptance::Unknown
    } else {
        ViaFabricationAcceptance::Accepted
    }
}

fn transition_allowed(class: ViaLayerTransitionClass, policy: &ViaFabricationPolicy) -> bool {
    match class {
        ViaLayerTransitionClass::SingleLayerLand => policy.allow_single_layer_land,
        ViaLayerTransitionClass::BlindVia => policy.allow_blind,
        ViaLayerTransitionClass::BuriedVia => policy.allow_buried,
        ViaLayerTransitionClass::ThroughVia => policy.allow_through,
    }
}
