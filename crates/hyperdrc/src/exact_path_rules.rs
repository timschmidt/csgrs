//! Exact PCB path rule wrappers backed by `hyperpath` geometry.
//!
//! `hyperpath` owns traces, pads, vias, board outlines, and exact geometric
//! helper predicates. This module owns fabrication-readiness policy labels that
//! interpret those exact path objects as DRC/DFM evidence.

use core::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperpath::{PcbViaStack, ViaDrillIntent};
use hyperreal::{Real, RealSign};

/// Exact annular-ring certification result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AnnularRingStatus {
    /// The via land is certified large enough for drill plus minimum ring.
    Certified,
    /// The via land is certified too small.
    Violation,
    /// No drill diameter was available.
    UnknownNoDrill,
    /// The minimum annular ring was invalid.
    InvalidMinimum,
    /// Exact comparison could not decide.
    Unknown,
}

/// Exact fabrication-policy class for a retained via drill.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaDrillPolicyClass {
    /// No drill diameter was available.
    MissingDrill,
    /// A plated drill is present; annular-ring certification applies.
    PlatedCopperVia,
    /// A non-plated drill is present; annular-ring certification is not applicable.
    NonPlatedMechanicalHole,
    /// A drill exists, but the plating intent was not retained.
    UnspecifiedDrilledHole,
}

/// Exact via drill fabrication-policy report.
#[derive(Clone, Debug, PartialEq)]
pub struct ViaDrillPolicyReport {
    /// Classified drill policy.
    pub class: ViaDrillPolicyClass,
    /// Retained drill intent.
    pub intent: ViaDrillIntent,
    /// Exact drill diameter when present.
    pub drill_diameter: Option<Real>,
    /// Annular-ring certification for plated drills.
    pub annular_ring: Option<AnnularRingStatus>,
}

/// Certify a via annular ring against an exact minimum requirement.
pub fn certify_annular_ring(
    via: &PcbViaStack,
    minimum: &Real,
    policy: PredicatePolicy,
) -> AnnularRingStatus {
    let Some(drill) = via.drill_diameter() else {
        return AnnularRingStatus::UnknownNoDrill;
    };
    if real_sign(minimum, policy) == Some(RealSign::Negative) {
        return AnnularRingStatus::InvalidMinimum;
    }
    let required = drill.clone() + minimum.clone() * Real::from(2);
    match compare_reals(via.land_diameter(), &required, policy).value() {
        Some(Ordering::Less) => AnnularRingStatus::Violation,
        Some(Ordering::Equal | Ordering::Greater) => AnnularRingStatus::Certified,
        None => AnnularRingStatus::Unknown,
    }
}

/// Classify retained drill fabrication policy exactly.
pub fn classify_via_drill_policy(
    via: &PcbViaStack,
    minimum_annular_ring: &Real,
    policy: PredicatePolicy,
) -> ViaDrillPolicyReport {
    let Some(drill_diameter) = via.drill_diameter() else {
        return ViaDrillPolicyReport {
            class: ViaDrillPolicyClass::MissingDrill,
            intent: via.drill_intent(),
            drill_diameter: None,
            annular_ring: None,
        };
    };
    match via.drill_intent() {
        ViaDrillIntent::Plated => ViaDrillPolicyReport {
            class: ViaDrillPolicyClass::PlatedCopperVia,
            intent: via.drill_intent(),
            drill_diameter: Some(drill_diameter.clone()),
            annular_ring: Some(certify_annular_ring(via, minimum_annular_ring, policy)),
        },
        ViaDrillIntent::NonPlated => ViaDrillPolicyReport {
            class: ViaDrillPolicyClass::NonPlatedMechanicalHole,
            intent: via.drill_intent(),
            drill_diameter: Some(drill_diameter.clone()),
            annular_ring: None,
        },
        ViaDrillIntent::Unspecified => ViaDrillPolicyReport {
            class: ViaDrillPolicyClass::UnspecifiedDrilledHole,
            intent: via.drill_intent(),
            drill_diameter: Some(drill_diameter.clone()),
            annular_ring: None,
        },
    }
}

fn real_sign(value: &Real, policy: PredicatePolicy) -> Option<RealSign> {
    match compare_reals(value, &Real::zero(), policy).value()? {
        Ordering::Less => Some(RealSign::Negative),
        Ordering::Equal => Some(RealSign::Zero),
        Ordering::Greater => Some(RealSign::Positive),
    }
}

#[cfg(test)]
mod tests {
    use super::real_sign;
    use hyperlimit::PredicatePolicy;
    use hyperreal::{Real, RealSign};

    #[test]
    fn scalar_guard_honors_the_callers_certainty_policy() {
        // cos(pi/5) = (1 + sqrt 5)/4 is a trigonometric identity that no
        // structural rule proves: strict refinement cannot certify the zero,
        // while the approximate policy accepts it below 2^-512. (Cancelling
        // sums such as (pi + e) - (e + pi) are now proved zero structurally
        // and no longer exercise this guard.)
        let fifth_pi = (Real::pi() / Real::from(5_i32)).unwrap();
        let golden_quarter =
            ((Real::one() + Real::from(5_i32).sqrt().unwrap()) / Real::from(4_i32)).unwrap();
        let unresolved_zero = fifth_pi.cos() - golden_quarter;

        assert_eq!(real_sign(&unresolved_zero, PredicatePolicy::STRICT), None);
        assert_eq!(
            real_sign(&unresolved_zero, PredicatePolicy::APPROXIMATE_512),
            Some(RealSign::Zero)
        );
    }
}
