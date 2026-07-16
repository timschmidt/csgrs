//! Source-attributed assertions.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::{ManufacturerPartNumber, PartsError, PartsResult, SupplierSku};

/// Provenance for an imported or authored part assertion.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceRef {
    authority: String,
    locator: String,
}

/// Source revision, date, or version tag.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceRevision {
    /// Source revision text.
    pub revision: String,
    /// Optional source date.
    pub date: Option<String>,
}

/// Whether an assertion is known or explicitly unknown.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssertionStatus {
    /// A source supplied a value.
    Known,
    /// A source was checked and did not provide this field.
    Unknown,
}

/// Confidence/review state for a source assertion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssertionConfidence {
    /// Imported but not reviewed.
    Imported,
    /// Reviewed by a user or maintainer.
    Reviewed,
    /// Measured from a fixture.
    Measured,
    /// Source claims conflict with other evidence.
    Conflicting,
}

/// Condition attached to an assertion.
#[derive(Clone, Debug, PartialEq)]
pub struct AssertionCondition {
    /// Condition label, such as temperature, wavelength, load, or package.
    pub label: String,
    /// Exact or textual condition value.
    pub value: AssertionValue,
}

/// Provenance-preserving assertion value.
#[derive(Clone, Debug, PartialEq)]
pub enum AssertionValue {
    /// Exact scalar value.
    ExactScalar(Box<Real>),
    /// Exact closed interval.
    ExactInterval {
        /// Lower bound.
        min: Box<Real>,
        /// Upper bound.
        max: Box<Real>,
    },
    /// Text/enumerated value.
    Text(String),
    /// Explicit lossy adapter/import value.
    Lossy(String),
    /// Source was inspected and did not provide a value.
    Unknown,
}

/// General source-attributed assertion payload.
#[derive(Clone, Debug, PartialEq)]
pub struct GeneralPartAssertion {
    /// Assertion key.
    pub key: String,
    /// Assertion value.
    pub value: AssertionValue,
    /// Unit label.
    pub unit: Option<String>,
    /// Applicability conditions.
    pub conditions: Vec<AssertionCondition>,
    /// Confidence/review state.
    pub confidence: AssertionConfidence,
    /// Source evidence.
    pub source: SourceRef,
    /// Optional source revision/date.
    pub revision: Option<SourceRevision>,
}

/// A source-attributed value or explicit unknown.
#[derive(Clone, Debug, PartialEq)]
pub struct Assertion<T> {
    status: AssertionStatus,
    value: Option<T>,
    source: SourceRef,
}

/// Domain assertions indexed by a part variant.
#[derive(Clone, Debug, PartialEq)]
pub enum PartAssertion {
    /// Link to a CAD/CSG/curve handle owned by another crate.
    ShapeHandle(Assertion<String>),
    /// Link to a voxel/grid artifact owned by `hypervoxel`.
    VoxelArtifact(Assertion<String>),
    /// Link to a physics material id or model owned by `hyperphysics`.
    PhysicsMaterial(Assertion<String>),
    /// Link to an electrical model or symbol owned by `hypercircuit`.
    CircuitModel(Assertion<String>),
    /// Catalog lifecycle state such as active, NRND, or obsolete.
    Lifecycle(Assertion<String>),
    /// General provenance-rich scalar/text/range assertion.
    General(Box<GeneralPartAssertion>),
}

/// Lifecycle status for a part.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PartLifecycle {
    /// Active production.
    Active,
    /// Not recommended for new designs.
    NotRecommendedForNewDesigns,
    /// Obsolete/end-of-life.
    Obsolete,
    /// Unknown lifecycle.
    Unknown,
    /// Source-specific lifecycle label.
    Custom(String),
}

/// Source-attributed compliance claim.
#[derive(Clone, Debug, PartialEq)]
pub struct ComplianceClaim {
    /// Compliance scheme, such as RoHS or REACH.
    pub scheme: String,
    /// Claim value.
    pub value: AssertionValue,
    /// Source evidence.
    pub source: SourceRef,
    /// Optional revision or consolidation date of the compliance source.
    pub revision: Option<SourceRevision>,
}

/// Source-attributed procurement offer.
#[derive(Clone, Debug, PartialEq)]
pub struct ProcurementOffer {
    /// Supplier SKU.
    pub sku: SupplierSku,
    /// Manufacturer part number when known.
    pub mpn: Option<ManufacturerPartNumber>,
    /// Available quantity.
    pub quantity: AssertionValue,
    /// Unit price.
    pub unit_price: AssertionValue,
    /// Currency code.
    pub currency: Option<String>,
    /// Source evidence.
    pub source: SourceRef,
}

/// Knowledge report for a part query/import summary.
#[derive(Clone, Debug, PartialEq)]
pub struct PartKnowledgeReport {
    /// Report status.
    pub status: String,
    /// Evidence notes.
    pub evidence: Vec<String>,
    /// Explicit unknowns.
    pub unknowns: Vec<String>,
    /// Conflicts.
    pub conflicts: Vec<String>,
}

impl SourceRef {
    /// Creates a non-empty source reference.
    pub fn new(authority: impl Into<String>, locator: impl Into<String>) -> PartsResult<Self> {
        let authority = authority.into();
        let locator = locator.into();
        if authority.is_empty() || locator.is_empty() {
            return Err(PartsError::EmptySource);
        }
        Ok(Self { authority, locator })
    }

    /// Returns the source authority.
    pub fn authority(&self) -> &str {
        &self.authority
    }

    /// Returns the source locator.
    pub fn locator(&self) -> &str {
        &self.locator
    }
}

impl SourceRevision {
    /// Creates a non-empty source revision/date tag.
    pub fn new(revision: impl Into<String>, date: Option<String>) -> PartsResult<Self> {
        let revision = revision.into();
        if revision.is_empty() {
            return Err(PartsError::EmptySource);
        }
        Ok(Self { revision, date })
    }
}

impl AssertionValue {
    /// Creates an exact scalar assertion value.
    pub fn exact_scalar(value: Real) -> Self {
        Self::ExactScalar(Box::new(value))
    }

    /// Creates a validated exact interval assertion value.
    pub fn interval(min: Real, max: Real) -> PartsResult<Self> {
        match min.partial_cmp(&max) {
            Some(Ordering::Less | Ordering::Equal) => Ok(Self::ExactInterval {
                min: Box::new(min),
                max: Box::new(max),
            }),
            Some(Ordering::Greater) | None => Err(PartsError::InvalidAssertionRange),
        }
    }
}

impl<T> Assertion<T> {
    /// Creates a source-attributed known assertion.
    pub fn known(value: T, source: SourceRef) -> Self {
        Self {
            status: AssertionStatus::Known,
            value: Some(value),
            source,
        }
    }

    /// Creates an explicit unknown after a source was inspected.
    pub fn unknown(source: SourceRef) -> Self {
        Self {
            status: AssertionStatus::Unknown,
            value: None,
            source,
        }
    }

    /// Returns the assertion status.
    pub const fn status(&self) -> AssertionStatus {
        self.status
    }

    /// Returns the known value, if present.
    pub const fn value(&self) -> Option<&T> {
        self.value.as_ref()
    }

    /// Returns the source evidence.
    pub const fn source(&self) -> &SourceRef {
        &self.source
    }
}
