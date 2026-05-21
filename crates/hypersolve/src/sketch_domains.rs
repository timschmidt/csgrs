//! Sketch parameter-domain preflight certificates.
//!
//! This module checks retained sketch parameter domains before residual
//! lowering or numerical iteration. The checks deliberately return report rows
//! instead of mutating values or applying epsilon repairs: Yap's Exact
//! Geometric Computation model requires certified decisions or explicit
//! uncertainty; see Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997). The separation between domain
//! preconditions and residual certificates is also consistent with Moore's
//! interval-analysis discipline for validated numerical computation.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::sketch::{SketchParameterDomain, SketchParameterHandle, SketchSolveProblem};

/// Stable family name for a sketch parameter-domain check.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchParameterDomainKind {
    /// Exact lock value.
    Locked,
    /// Closed lower/upper bounds.
    Bounded,
    /// Nonnegative scalar.
    Nonnegative,
    /// Strictly positive scalar.
    Positive,
    /// Nonzero scalar.
    Nonzero,
    /// Closed angular range.
    AngularRange,
    /// Strictly positive period.
    Periodic,
}

/// Proof status for one sketch parameter-domain check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchParameterDomainStatus {
    /// The domain condition was proved exactly under the supplied policy.
    CertifiedValid,
    /// The domain condition was proved false.
    CertifiedInvalid,
    /// Exact comparison was not decided under the supplied policy.
    Unknown,
}

impl SketchParameterDomainStatus {
    /// Returns whether this status proves the domain condition.
    pub const fn is_certified_valid(&self) -> bool {
        matches!(self, Self::CertifiedValid)
    }

    /// Returns whether this status proves a domain violation.
    pub const fn is_certified_invalid(&self) -> bool {
        matches!(self, Self::CertifiedInvalid)
    }
}

/// One exact preflight check for a retained sketch parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchParameterDomainCheck {
    /// Source parameter handle.
    pub parameter: SketchParameterHandle,
    /// Source parameter name.
    pub parameter_name: String,
    /// Domain ordinal in the retained parameter's domain list.
    pub domain_index: usize,
    /// Checked domain payload.
    pub domain: SketchParameterDomain,
    /// Compact domain family.
    pub kind: SketchParameterDomainKind,
    /// Exact retained parameter value checked by this row.
    pub value: Real,
    /// Proof status.
    pub status: SketchParameterDomainStatus,
}

/// Report for sketch parameter-domain preflight.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchParameterDomainReport {
    /// Per-domain proof rows.
    pub checks: Vec<SketchParameterDomainCheck>,
    /// Number of retained parameters examined.
    pub parameter_count: usize,
    /// Number of domain checks proved valid.
    pub certified_valid_checks: usize,
    /// Number of domain checks proved invalid.
    pub certified_invalid_checks: usize,
    /// Number of domain checks left unresolved.
    pub unknown_checks: usize,
}

impl SketchParameterDomainReport {
    /// Returns true when at least one domain check proves invalid retained data.
    pub fn has_certified_invalid_domain(&self) -> bool {
        self.certified_invalid_checks > 0
    }

    /// Returns true when every retained domain check has a validity proof.
    pub fn all_certified_valid(&self) -> bool {
        self.certified_invalid_checks == 0 && self.unknown_checks == 0
    }
}

/// Certify retained sketch parameter-domain obligations with the default
/// predicate policy.
pub fn preflight_sketch_parameter_domains(
    sketch: &SketchSolveProblem,
) -> SketchParameterDomainReport {
    preflight_sketch_parameter_domains_with_policy(sketch, PredicatePolicy::default())
}

/// Certify retained sketch parameter-domain obligations with an explicit
/// predicate policy.
///
/// The checked value is the exact retained parameter value. Solver iteration
/// domains for later candidate values should be represented as residual/domain
/// certificates as those candidates are produced.
pub fn preflight_sketch_parameter_domains_with_policy(
    sketch: &SketchSolveProblem,
    policy: PredicatePolicy,
) -> SketchParameterDomainReport {
    let mut checks = Vec::new();
    for parameter in sketch.parameters() {
        for (domain_index, domain) in parameter.domains.iter().cloned().enumerate() {
            let status = classify_domain(&parameter.value, &domain, policy);
            checks.push(SketchParameterDomainCheck {
                parameter: parameter.handle,
                parameter_name: parameter.name.clone(),
                domain_index,
                kind: domain_kind(&domain),
                domain,
                value: parameter.value.clone(),
                status,
            });
        }
    }

    let certified_valid_checks = checks
        .iter()
        .filter(|check| check.status.is_certified_valid())
        .count();
    let certified_invalid_checks = checks
        .iter()
        .filter(|check| check.status.is_certified_invalid())
        .count();
    let unknown_checks = checks
        .iter()
        .filter(|check| check.status == SketchParameterDomainStatus::Unknown)
        .count();

    SketchParameterDomainReport {
        checks,
        parameter_count: sketch.parameters().len(),
        certified_valid_checks,
        certified_invalid_checks,
        unknown_checks,
    }
}

fn domain_kind(domain: &SketchParameterDomain) -> SketchParameterDomainKind {
    match domain {
        SketchParameterDomain::Locked { .. } => SketchParameterDomainKind::Locked,
        SketchParameterDomain::Bounded { .. } => SketchParameterDomainKind::Bounded,
        SketchParameterDomain::Nonnegative => SketchParameterDomainKind::Nonnegative,
        SketchParameterDomain::Positive => SketchParameterDomainKind::Positive,
        SketchParameterDomain::Nonzero => SketchParameterDomainKind::Nonzero,
        SketchParameterDomain::AngularRange { .. } => SketchParameterDomainKind::AngularRange,
        SketchParameterDomain::Periodic { .. } => SketchParameterDomainKind::Periodic,
    }
}

fn classify_domain(
    value: &Real,
    domain: &SketchParameterDomain,
    policy: PredicatePolicy,
) -> SketchParameterDomainStatus {
    match domain {
        SketchParameterDomain::Locked { value: locked } => {
            match compare_reals_with_policy(value, locked, policy).value() {
                Some(Ordering::Equal) => SketchParameterDomainStatus::CertifiedValid,
                Some(Ordering::Less | Ordering::Greater) => {
                    SketchParameterDomainStatus::CertifiedInvalid
                }
                None => SketchParameterDomainStatus::Unknown,
            }
        }
        SketchParameterDomain::Bounded { lower, upper } => {
            classify_closed_bounds(value, lower.as_ref(), upper.as_ref(), policy)
        }
        SketchParameterDomain::Nonnegative => match compare_zero(value, policy) {
            Some(Ordering::Equal | Ordering::Greater) => {
                SketchParameterDomainStatus::CertifiedValid
            }
            Some(Ordering::Less) => SketchParameterDomainStatus::CertifiedInvalid,
            None => SketchParameterDomainStatus::Unknown,
        },
        SketchParameterDomain::Positive => match compare_zero(value, policy) {
            Some(Ordering::Greater) => SketchParameterDomainStatus::CertifiedValid,
            Some(Ordering::Less | Ordering::Equal) => SketchParameterDomainStatus::CertifiedInvalid,
            None => SketchParameterDomainStatus::Unknown,
        },
        SketchParameterDomain::Nonzero => match compare_zero(value, policy) {
            Some(Ordering::Less | Ordering::Greater) => SketchParameterDomainStatus::CertifiedValid,
            Some(Ordering::Equal) => SketchParameterDomainStatus::CertifiedInvalid,
            None => SketchParameterDomainStatus::Unknown,
        },
        SketchParameterDomain::AngularRange { lower, upper } => {
            classify_closed_bounds(value, Some(lower), Some(upper), policy)
        }
        SketchParameterDomain::Periodic { period } => match compare_zero(period, policy) {
            Some(Ordering::Greater) => SketchParameterDomainStatus::CertifiedValid,
            Some(Ordering::Less | Ordering::Equal) => SketchParameterDomainStatus::CertifiedInvalid,
            None => SketchParameterDomainStatus::Unknown,
        },
    }
}

fn classify_closed_bounds(
    value: &Real,
    lower: Option<&Real>,
    upper: Option<&Real>,
    policy: PredicatePolicy,
) -> SketchParameterDomainStatus {
    if let (Some(lower), Some(upper)) = (lower, upper) {
        match compare_reals_with_policy(lower, upper, policy).value() {
            Some(Ordering::Greater) => return SketchParameterDomainStatus::CertifiedInvalid,
            Some(Ordering::Less | Ordering::Equal) => {}
            None => return SketchParameterDomainStatus::Unknown,
        }
    }

    let lower_ok = match lower {
        Some(lower) => match compare_reals_with_policy(value, lower, policy).value() {
            Some(Ordering::Greater | Ordering::Equal) => Some(true),
            Some(Ordering::Less) => Some(false),
            None => None,
        },
        None => Some(true),
    };
    let upper_ok = match upper {
        Some(upper) => match compare_reals_with_policy(value, upper, policy).value() {
            Some(Ordering::Less | Ordering::Equal) => Some(true),
            Some(Ordering::Greater) => Some(false),
            None => None,
        },
        None => Some(true),
    };

    match (lower_ok, upper_ok) {
        (Some(true), Some(true)) => SketchParameterDomainStatus::CertifiedValid,
        (Some(false), _) | (_, Some(false)) => SketchParameterDomainStatus::CertifiedInvalid,
        _ => SketchParameterDomainStatus::Unknown,
    }
}

fn compare_zero(value: &Real, policy: PredicatePolicy) -> Option<Ordering> {
    compare_reals_with_policy(value, &Real::zero(), policy).value()
}
