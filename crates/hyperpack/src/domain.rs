//! Report-bearing domain handoffs for Hyper ecosystem integrations.
//!
//! `hyperpack` should not silently own facts that belong to `hyperparts`,
//! `hyperphysics`, `hyperpath`, `hypervoxel`, `hypercurve`, `hypermesh`,
//! `csgrs`, `hyperdrc`, or `hypercircuit`. This module provides small adapter
//! records with provenance and explicit unknowns. That follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): exact facts can cross the
//! boundary as exact values, while uncertified or lossy facts must be reported
//! as such instead of being folded into geometric acceptance.

use crate::{AxisBox3, Bin3, Item3, ItemId, PackError, PackResult};
use hyperreal::{Real, RealSign};

/// Hyper ecosystem domain that produced a handoff fact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DomainCrate {
    /// Part/package/container facts.
    Hyperparts,
    /// Weight, load, material, support, and physics facts.
    Hyperphysics,
    /// Conservative occupancy and process-grid facts.
    Hypervoxel,
    /// Exact or bounded curve/nesting shape facts.
    Hypercurve,
    /// Exact or bounded mesh broad-phase facts.
    Hypermesh,
    /// Constructive solid geometry shape facts.
    Csgrs,
    /// Routing and path-clearance facts.
    Hyperpath,
    /// Design-rule and fabrication-process facts.
    Hyperdrc,
    /// Circuit, harness, thermal, and electrical facts.
    Hypercircuit,
}

/// Certification status of an imported or delegated fact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DomainFactStatus {
    /// Fact is exact for the target `hyperpack` model.
    Exact,
    /// Fact is conservative and safe for broad-phase proposal generation only.
    Conservative,
    /// Fact is approximate/lossy and cannot certify feasibility.
    Lossy,
    /// Fact is intentionally not certified by `hyperpack`.
    Unknown,
}

/// Handoff result for a non-owned domain constraint.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DomainHandoffStatus {
    /// Owning domain certified the constraint.
    Satisfied,
    /// Owning domain certified a violation.
    Violated,
    /// Constraint remains unknown to `hyperpack`.
    Unknown,
    /// Adapter was lossy and cannot be used as proof.
    Lossy,
}

/// Exact item/bin fact imported from a domain crate.
#[derive(Clone, Debug, PartialEq)]
pub struct DomainBoxFact3 {
    /// Owning domain.
    pub source: DomainCrate,
    /// Stable source id.
    pub id: String,
    /// Exact x dimension.
    pub x: Real,
    /// Exact y dimension.
    pub y: Real,
    /// Exact z dimension.
    pub z: Real,
    /// Provenance string supplied by the source crate or adapter.
    pub provenance: String,
    /// Certification status for this dimensional fact.
    pub status: DomainFactStatus,
}

/// Report from importing domain box facts into exact packing carriers.
#[derive(Clone, Debug, PartialEq)]
pub struct DomainImportReport3 {
    /// Domain source.
    pub source: DomainCrate,
    /// Imported exact items.
    pub items: Vec<Item3>,
    /// Imported exact bin, if a bin fact was supplied.
    pub bin: Option<Bin3>,
    /// Number of facts accepted as exact.
    pub exact_facts: usize,
    /// Number of conservative facts skipped from exact carriers.
    pub conservative_facts: usize,
    /// Number of lossy facts skipped from exact carriers.
    pub lossy_facts: usize,
    /// Number of unknown facts skipped from exact carriers.
    pub unknown_facts: usize,
    /// Human-readable evidence.
    pub facts: Vec<String>,
}

/// Non-owned domain constraint handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct DomainConstraintHandoff {
    /// Owning domain.
    pub source: DomainCrate,
    /// Constraint name.
    pub constraint: String,
    /// Handoff status.
    pub status: DomainHandoffStatus,
    /// Provenance or adapter evidence.
    pub provenance: String,
}

/// Aggregated handoff report from domain constraints not owned by `hyperpack`.
#[derive(Clone, Debug, PartialEq)]
pub struct DomainHandoffReport {
    /// Constraint handoffs.
    pub handoffs: Vec<DomainConstraintHandoff>,
    /// Overall status.
    pub status: DomainHandoffStatus,
    /// Human-readable facts.
    pub facts: Vec<String>,
}

impl DomainBoxFact3 {
    /// Creates a domain box fact after validating positive exact dimensions.
    pub fn new(
        source: DomainCrate,
        id: impl Into<String>,
        size: AxisBox3,
        provenance: impl Into<String>,
        status: DomainFactStatus,
    ) -> PackResult<Self> {
        let id = id.into();
        if id.is_empty() {
            return Err(PackError::EmptyIdentifier);
        }
        Ok(Self {
            source,
            id,
            x: size.x,
            y: size.y,
            z: size.z,
            provenance: provenance.into(),
            status,
        })
    }
}

/// Imports exact domain box facts into `Item3` carriers.
///
/// Only [`DomainFactStatus::Exact`] facts are converted to items. Conservative,
/// lossy, and unknown facts remain evidence because using them as exact
/// dimensions would collapse the domain boundary.
pub fn import_domain_items_3d(
    source: DomainCrate,
    facts: &[DomainBoxFact3],
) -> PackResult<DomainImportReport3> {
    let mut report = DomainImportReport3 {
        source,
        items: Vec::new(),
        bin: None,
        exact_facts: 0,
        conservative_facts: 0,
        lossy_facts: 0,
        unknown_facts: 0,
        facts: Vec::new(),
    };
    for fact in facts {
        count_status(fact.status, &mut report);
        if fact.status != DomainFactStatus::Exact {
            report.facts.push(format!(
                "{} skipped because status is {:?}",
                fact.id, fact.status
            ));
            continue;
        }
        report.items.push(Item3 {
            id: ItemId::new(fact.id.clone())?,
            size: exact_box_from_fact(fact)?,
        });
    }
    Ok(report)
}

/// Imports one exact domain box fact into a `Bin3` carrier.
pub fn import_domain_bin_3d(fact: &DomainBoxFact3) -> PackResult<DomainImportReport3> {
    let mut report = DomainImportReport3 {
        source: fact.source.clone(),
        items: Vec::new(),
        bin: None,
        exact_facts: 0,
        conservative_facts: 0,
        lossy_facts: 0,
        unknown_facts: 0,
        facts: Vec::new(),
    };
    count_status(fact.status, &mut report);
    if fact.status == DomainFactStatus::Exact {
        report.bin = Some(Bin3 {
            size: exact_box_from_fact(fact)?,
        });
    } else {
        report.facts.push(format!(
            "{} skipped because status is {:?}",
            fact.id, fact.status
        ));
    }
    Ok(report)
}

/// Aggregates non-owned domain constraint handoffs.
///
/// `Violated` dominates all other statuses, then `Lossy`, then `Unknown`, then
/// `Satisfied`. This prevents lossy or unknown domain reports from being
/// mistaken for feasibility proof.
pub fn summarize_domain_handoffs(handoffs: Vec<DomainConstraintHandoff>) -> DomainHandoffReport {
    let mut status = DomainHandoffStatus::Satisfied;
    let mut facts = Vec::new();
    for handoff in &handoffs {
        status = match (status, handoff.status) {
            (DomainHandoffStatus::Violated, _) | (_, DomainHandoffStatus::Violated) => {
                DomainHandoffStatus::Violated
            }
            (DomainHandoffStatus::Lossy, _) | (_, DomainHandoffStatus::Lossy) => {
                DomainHandoffStatus::Lossy
            }
            (DomainHandoffStatus::Unknown, _) | (_, DomainHandoffStatus::Unknown) => {
                DomainHandoffStatus::Unknown
            }
            _ => DomainHandoffStatus::Satisfied,
        };
        facts.push(format!(
            "{:?}:{} => {:?}",
            handoff.source, handoff.constraint, handoff.status
        ));
    }
    DomainHandoffReport {
        handoffs,
        status,
        facts,
    }
}

fn exact_box_from_fact(fact: &DomainBoxFact3) -> PackResult<AxisBox3> {
    validate_exact(&fact.x)?;
    validate_exact(&fact.y)?;
    validate_exact(&fact.z)?;
    AxisBox3::new(fact.x.clone(), fact.y.clone(), fact.z.clone())
}

fn validate_exact(value: &Real) -> PackResult<()> {
    match value.refine_sign_until(-64) {
        Some(RealSign::Positive) => Ok(()),
        Some(RealSign::Zero | RealSign::Negative) | None => Err(PackError::NonPositiveDimension),
    }
}

fn count_status(status: DomainFactStatus, report: &mut DomainImportReport3) {
    match status {
        DomainFactStatus::Exact => report.exact_facts += 1,
        DomainFactStatus::Conservative => report.conservative_facts += 1,
        DomainFactStatus::Lossy => report.lossy_facts += 1,
        DomainFactStatus::Unknown => report.unknown_facts += 1,
    }
}
