//! Physical-property handles and handoff reports.
//!
//! `hyperparts` records what a part source claims about material, thermal,
//! electrical, mass, and load requirements, but it does not implement physical
//! laws. The owning semantics live in `hyperphysics`; this module carries
//! source-attributed handles and query reports that say whether a fact is exact,
//! certified, conditional, lossy, or unknown.
//!
//! Imported metadata remains exact or certified facts, or explicit uncertainty,
//! instead of collapsing into numeric defaults. Material and transport facts
//! are handoff records because constitutive interpretation belongs to a
//! physical model.

use crate::{
    AssertionCondition, AssertionValue, PartId, PartQueryEvidence, PartsError, PartsResult,
};

/// Review status for a physical/material fact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhysicalFactStatus {
    /// Exact source value or exact downstream handle.
    Exact,
    /// Certified by a reviewed adapter or bounded derivation.
    Certified,
    /// Valid only under stated conditions.
    Conditional,
    /// Imported through a lossy adapter and not a source of exact truth.
    Lossy,
    /// Source was inspected and did not provide the fact.
    Unknown,
}

/// Handle to a physical property or material model owned by `hyperphysics`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicalPropertyHandle {
    /// Owning crate or external source namespace.
    pub owner: String,
    /// Stable handle in the owner namespace.
    pub handle: String,
    /// Property or model name, such as density, conductivity, or contact law.
    pub property: String,
    /// Source unit label when the handle denotes a scalar/range property.
    pub units: Option<String>,
    /// Review/exactness status for this handle.
    pub status: PhysicalFactStatus,
}

/// Material or property requirement needed before a downstream physics model is
/// allowed to claim a certified result.
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialRequirement {
    /// Required material/property key.
    pub key: String,
    /// Exact, interval, textual, lossy, or unknown value.
    pub value: AssertionValue,
    /// Source unit label.
    pub units: Option<String>,
    /// Applicability conditions such as temperature, frequency, or load.
    pub conditions: Vec<AssertionCondition>,
    /// Review/exactness status.
    pub status: PhysicalFactStatus,
}

/// Environmental operating envelope for a part, fixture, or material.
#[derive(Clone, Debug, PartialEq)]
pub struct EnvironmentalEnvelope {
    /// Temperature range or explicit unknown.
    pub temperature: AssertionValue,
    /// Humidity range or explicit unknown.
    pub humidity: AssertionValue,
    /// Pressure range or explicit unknown.
    pub pressure: AssertionValue,
    /// Additional source-specific conditions.
    pub conditions: Vec<AssertionCondition>,
    /// Review/exactness status.
    pub status: PhysicalFactStatus,
}

/// Thermal adjacency or conduction path claim.
#[derive(Clone, Debug, PartialEq)]
pub struct ThermalPath {
    /// Source aspect, terminal, face, region, or part handle.
    pub from: String,
    /// Destination aspect, terminal, face, region, or part handle.
    pub to: String,
    /// Thermal property handle or explicit requirement.
    pub property: Option<PhysicalPropertyHandle>,
    /// Source-attributed resistance/conductance value when known.
    pub value: AssertionValue,
    /// Review/exactness status.
    pub status: PhysicalFactStatus,
}

/// Mechanical load path claim between part aspects.
#[derive(Clone, Debug, PartialEq)]
pub struct MechanicalLoadPath {
    /// Source aspect, terminal, face, region, or part handle.
    pub from: String,
    /// Destination aspect, terminal, face, region, or part handle.
    pub to: String,
    /// Load direction, axis, or semantic label.
    pub direction: String,
    /// Load value/range or explicit unknown.
    pub load: AssertionValue,
    /// Review/exactness status.
    pub status: PhysicalFactStatus,
}

/// Mass-property query need for a downstream `hyperphysics` report.
#[derive(Clone, Debug, PartialEq)]
pub struct MassPropertyNeed {
    /// Shape, aspect, or part handle whose mass properties are requested.
    pub target: String,
    /// Density/material handle to use, when known.
    pub material: Option<PhysicalPropertyHandle>,
    /// Requested outputs such as mass, center_of_mass, inertia_tensor.
    pub outputs: Vec<String>,
    /// Review/exactness status for the request inputs.
    pub status: PhysicalFactStatus,
}

/// Handoff report for physical setup or property queries.
#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsHandoffReport {
    /// Part associated with this handoff.
    pub part: PartId,
    /// Physical handles discovered for the part.
    pub handles: Vec<PhysicalPropertyHandle>,
    /// Material/property requirements that remain visible to `hyperphysics`.
    pub requirements: Vec<MaterialRequirement>,
    /// Environmental envelopes carried into the downstream model.
    pub environments: Vec<EnvironmentalEnvelope>,
    /// Thermal path claims.
    pub thermal_paths: Vec<ThermalPath>,
    /// Mechanical load path claims.
    pub mechanical_load_paths: Vec<MechanicalLoadPath>,
    /// Mass-property requests.
    pub mass_property_needs: Vec<MassPropertyNeed>,
    /// Overall report status.
    pub status: PhysicalFactStatus,
    /// Evidence and source facts used to construct the report.
    pub evidence: PartQueryEvidence,
    /// Explicit gaps that downstream code must not fill with defaults.
    pub unknowns: Vec<String>,
}

impl PhysicalPropertyHandle {
    /// Creates a validated handle to a downstream physical property/model.
    pub fn new(
        owner: impl Into<String>,
        handle: impl Into<String>,
        property: impl Into<String>,
        units: Option<String>,
        status: PhysicalFactStatus,
    ) -> PartsResult<Self> {
        let owner = owner.into();
        let handle = handle.into();
        let property = property.into();
        if owner.is_empty() || handle.is_empty() || property.is_empty() {
            return Err(PartsError::EmptyIdentifier);
        }
        Ok(Self {
            owner,
            handle,
            property,
            units,
            status,
        })
    }
}

impl PhysicsHandoffReport {
    /// Returns true only when no explicit unknowns remain and every carried fact
    /// is exact or certified.
    pub fn is_certified_ready(&self) -> bool {
        self.unknowns.is_empty()
            && matches!(
                self.status,
                PhysicalFactStatus::Exact | PhysicalFactStatus::Certified
            )
            && self.handles.iter().all(|handle| {
                matches!(
                    handle.status,
                    PhysicalFactStatus::Exact | PhysicalFactStatus::Certified
                )
            })
            && self.requirements.iter().all(|requirement| {
                matches!(
                    requirement.status,
                    PhysicalFactStatus::Exact | PhysicalFactStatus::Certified
                )
            })
    }
}
