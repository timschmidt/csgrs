//! Sketch unit and declared-tolerance audits.
//!
//! Source units and fabrication/design tolerances are useful only when they are
//! explicit data. This module audits retained sketch metadata without using the
//! declared tolerance as a residual acceptance threshold. Exact comparisons
//! certify whether a declared tolerance is nonnegative; later design-rule
//! constraints can consume the same value as an ordinary exact bound. This is
//! the exact-computation boundary applied to units and tolerances: decisions
//! consume certified facts, not hidden epsilons.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::sketch::{
    SketchConstraintHandle, SketchEntityHandle, SketchParameterHandle, SketchRoundTripMetadata,
    SketchSolveProblem,
};

/// Retained object family for a unit/tolerance audit row.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchUnitToleranceObjectKind {
    /// Scalar parameter metadata.
    Parameter,
    /// Retained entity metadata.
    Entity,
    /// Retained high-level constraint metadata.
    Constraint,
}

/// Audit status for one retained unit/tolerance declaration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchUnitToleranceStatus {
    /// No unit or tolerance was declared.
    NoDeclaration,
    /// A source unit was declared without an exact tolerance value.
    UnitOnly,
    /// A nonnegative exact tolerance was declared with a source unit.
    CertifiedDeclaredTolerance,
    /// A tolerance was declared without a source unit.
    ToleranceWithoutUnit,
    /// The declared tolerance was proved negative.
    CertifiedInvalidNegativeTolerance,
    /// The declared tolerance sign could not be decided under the policy.
    UnknownToleranceSign,
}

impl SketchUnitToleranceStatus {
    /// Returns whether this row proves invalid metadata.
    pub const fn is_invalid(&self) -> bool {
        matches!(
            self,
            Self::ToleranceWithoutUnit | Self::CertifiedInvalidNegativeTolerance
        )
    }
}

/// One retained object's unit/tolerance audit row.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchUnitToleranceRow {
    /// Retained object family.
    pub object_kind: SketchUnitToleranceObjectKind,
    /// Parameter handle when this row describes a parameter.
    pub parameter: Option<SketchParameterHandle>,
    /// Entity handle when this row describes an entity.
    pub entity: Option<SketchEntityHandle>,
    /// Constraint handle when this row describes a constraint.
    pub constraint: Option<SketchConstraintHandle>,
    /// Source object name.
    pub name: String,
    /// Retained source unit label.
    pub source_unit: Option<String>,
    /// Retained exact tolerance value.
    pub declared_tolerance: Option<Real>,
    /// Audit status.
    pub status: SketchUnitToleranceStatus,
}

/// Report for retained unit/tolerance declarations.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchUnitToleranceReport {
    /// Per-object audit rows in parameter/entity/constraint order.
    pub rows: Vec<SketchUnitToleranceRow>,
    /// Number of retained objects examined.
    pub object_count: usize,
    /// Number of rows with both unit and certified nonnegative tolerance.
    pub certified_declared_tolerances: usize,
    /// Number of rows proving invalid metadata.
    pub invalid_declarations: usize,
    /// Number of rows whose tolerance sign remained unresolved.
    pub unknown_declarations: usize,
}

impl SketchUnitToleranceReport {
    /// Returns true when at least one unit/tolerance declaration is invalid.
    pub fn has_invalid_declaration(&self) -> bool {
        self.invalid_declarations > 0
    }
}

/// Audit retained unit/tolerance metadata with the default predicate policy.
pub fn audit_sketch_unit_tolerances(sketch: &SketchSolveProblem) -> SketchUnitToleranceReport {
    audit_sketch_unit_tolerances_with_policy(sketch, PredicatePolicy)
}

/// Audit retained unit/tolerance metadata with an explicit predicate policy.
pub fn audit_sketch_unit_tolerances_with_policy(
    sketch: &SketchSolveProblem,
    policy: PredicatePolicy,
) -> SketchUnitToleranceReport {
    let mut rows = Vec::new();
    rows.extend(sketch.parameters().iter().map(|parameter| {
        row_from_metadata(
            SketchUnitToleranceObjectKind::Parameter,
            Some(parameter.handle),
            None,
            None,
            parameter.name.clone(),
            &parameter.metadata,
            policy,
        )
    }));
    rows.extend(sketch.entities().iter().map(|entity| {
        row_from_metadata(
            SketchUnitToleranceObjectKind::Entity,
            None,
            Some(entity.handle),
            None,
            entity.name.clone(),
            &entity.metadata,
            policy,
        )
    }));
    rows.extend(sketch.constraints().iter().map(|constraint| {
        row_from_metadata(
            SketchUnitToleranceObjectKind::Constraint,
            None,
            None,
            Some(constraint.handle),
            constraint.name.clone(),
            &constraint.metadata,
            policy,
        )
    }));

    let certified_declared_tolerances = rows
        .iter()
        .filter(|row| row.status == SketchUnitToleranceStatus::CertifiedDeclaredTolerance)
        .count();
    let invalid_declarations = rows.iter().filter(|row| row.status.is_invalid()).count();
    let unknown_declarations = rows
        .iter()
        .filter(|row| row.status == SketchUnitToleranceStatus::UnknownToleranceSign)
        .count();

    SketchUnitToleranceReport {
        object_count: rows.len(),
        rows,
        certified_declared_tolerances,
        invalid_declarations,
        unknown_declarations,
    }
}

fn row_from_metadata(
    object_kind: SketchUnitToleranceObjectKind,
    parameter: Option<SketchParameterHandle>,
    entity: Option<SketchEntityHandle>,
    constraint: Option<SketchConstraintHandle>,
    name: String,
    metadata: &SketchRoundTripMetadata,
    policy: PredicatePolicy,
) -> SketchUnitToleranceRow {
    let status = classify_metadata(metadata, policy);
    SketchUnitToleranceRow {
        object_kind,
        parameter,
        entity,
        constraint,
        name,
        source_unit: metadata.source_unit.clone(),
        declared_tolerance: metadata.declared_tolerance.clone(),
        status,
    }
}

fn classify_metadata(
    metadata: &SketchRoundTripMetadata,
    policy: PredicatePolicy,
) -> SketchUnitToleranceStatus {
    match (&metadata.source_unit, &metadata.declared_tolerance) {
        (None, None) => SketchUnitToleranceStatus::NoDeclaration,
        (Some(_), None) => SketchUnitToleranceStatus::UnitOnly,
        (None, Some(tolerance)) => match compare_tolerance_sign(tolerance, policy) {
            Some(Ordering::Less) => SketchUnitToleranceStatus::CertifiedInvalidNegativeTolerance,
            Some(Ordering::Equal | Ordering::Greater) => {
                SketchUnitToleranceStatus::ToleranceWithoutUnit
            }
            None => SketchUnitToleranceStatus::UnknownToleranceSign,
        },
        (Some(_), Some(tolerance)) => match compare_tolerance_sign(tolerance, policy) {
            Some(Ordering::Less) => SketchUnitToleranceStatus::CertifiedInvalidNegativeTolerance,
            Some(Ordering::Equal | Ordering::Greater) => {
                SketchUnitToleranceStatus::CertifiedDeclaredTolerance
            }
            None => SketchUnitToleranceStatus::UnknownToleranceSign,
        },
    }
}

fn compare_tolerance_sign(tolerance: &Real, policy: PredicatePolicy) -> Option<Ordering> {
    compare_reals_with_policy(tolerance, &Real::zero(), policy).value()
}
