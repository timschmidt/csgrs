//! Source-attributed material and property-resolution graph.
//!
//! This module is the crate-model layer before solvers. Material properties are
//! represented as exact scalars, exact intervals, explicit unknowns, or
//! temporary external proposals with replacement status. Resolution reports keep
//! provenance and conflicts visible. That is the data-model analogue of Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>): downstream physics
//! should consume certified facts or explicit uncertainty, not hidden defaults.
//!
//! The first exact relationship is isotropic elasticity. For an isotropic
//! linear elastic material, shear modulus is `G = E / (2(1 + nu))` when Young's
//! modulus `E` and Poisson ratio `nu` are the stated assumptions. This is the
//! classical Hooke-law relationship for isotropic solids; see Landau and
//! Lifshitz, *Theory of Elasticity*, 3rd ed., 1986. The derivation function
//! records that assumption and source values instead of silently inventing a
//! missing property.

use std::cmp::Ordering;

use hyperlattice::Vector3;
use hyperreal::{Real, RealSign};

use crate::{PhysicsError, PhysicsResult};

/// Source/provenance reference for property data.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SourceSpec {
    /// Source authority, such as a datasheet, database, fixture, or calibration.
    pub authority: String,
    /// Locator within the authority.
    pub locator: String,
    /// Freshness or version tag.
    pub freshness: Option<String>,
}

/// Material state or process condition.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MaterialState {
    /// Solid material.
    Solid,
    /// Liquid material.
    Liquid,
    /// Gas material.
    Gas,
    /// Cured polymer or resin.
    Cured,
    /// Uncured or partially cured resin.
    Uncured,
    /// Custom source-specific state.
    Custom(String),
}

/// Material property kind.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub enum MaterialPropertyKind {
    /// Density.
    Density,
    /// Thermal conductivity.
    ThermalConductivity,
    /// Electrical conductivity.
    ElectricalConductivity,
    /// Refractive index.
    RefractiveIndex,
    /// Dynamic viscosity.
    Viscosity,
    /// Young's modulus.
    YoungModulus,
    /// Poisson ratio.
    PoissonRatio,
    /// Shear modulus.
    ShearModulus,
    /// Cure conversion.
    CureConversion,
    /// Custom source-specific property.
    Custom(String),
}

/// Replacement status for an external proposal value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExternalReplacementStatus {
    /// Temporary compatibility value.
    Temporary,
    /// Being replaced by a Hyper-native exact/certified path.
    ReplacementPlanned,
    /// Accepted only as a lossy display or adapter value.
    LossyAdapterOnly,
}

/// Property value with exact uncertainty semantics.
#[derive(Clone, Debug, PartialEq)]
pub enum PropertyValue {
    /// Exact scalar value.
    ExactScalar(Box<Real>),
    /// Exact interval enclosure.
    Interval {
        /// Lower bound.
        lower: Box<Real>,
        /// Upper bound.
        upper: Box<Real>,
    },
    /// Source was inspected but did not provide a value.
    Unknown,
    /// Temporary external proposal.
    ExternalProposal {
        /// Proposed scalar value.
        value: Box<Real>,
        /// Replacement status.
        status: ExternalReplacementStatus,
    },
}

/// Property tensor carrier for anisotropic or matrix-valued data.
#[derive(Clone, Debug, PartialEq)]
pub struct PropertyTensor {
    /// Tensor rows in 3x3 order.
    pub rows: [Vector3; 3],
    /// Unit label.
    pub unit: String,
    /// Source.
    pub source: SourceSpec,
}

/// Source-attributed property assertion.
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialAssertion {
    /// Property kind.
    pub kind: MaterialPropertyKind,
    /// Value.
    pub value: PropertyValue,
    /// Unit label.
    pub unit: String,
    /// Material state under which this assertion applies.
    pub state: MaterialState,
    /// Optional condition, such as temperature or wavelength.
    pub condition: Option<String>,
    /// Source provenance.
    pub source: SourceSpec,
}

/// Query port for material/circuit/thermal/optical handoff.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicalPort {
    /// Port handle.
    pub handle: String,
    /// Domain name.
    pub domain: String,
}

/// High-level certification report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicsCertificationReport {
    /// Report status.
    pub status: String,
    /// Evidence labels.
    pub evidence: Vec<String>,
}

/// Resolution status for a property query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PropertyResolutionStatus {
    /// All exact assertions agree.
    ExactKnown,
    /// A valid interval was returned.
    Interval,
    /// Source assertions conflict.
    Conflict,
    /// Only external proposal values are available.
    ExternalProposal,
    /// Only unknown or no assertions are available.
    Unknown,
}

/// Property-resolution report.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedPropertyReport {
    /// Property kind.
    pub kind: MaterialPropertyKind,
    /// Resolution status.
    pub status: PropertyResolutionStatus,
    /// Resolved value, if any.
    pub value: Option<PropertyValue>,
    /// Sources considered.
    pub sources: Vec<SourceSpec>,
    /// Human-readable evidence and conflict notes.
    pub evidence: Vec<String>,
}

/// Exact elastic derivation report.
#[derive(Clone, Debug, PartialEq)]
pub struct ElasticDerivationReport {
    /// Derived property kind.
    pub kind: MaterialPropertyKind,
    /// Derived exact value.
    pub value: Real,
    /// Source assertions used.
    pub sources: Vec<SourceSpec>,
    /// Assumption text.
    pub assumption: String,
}

/// Source-attributed material property graph.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MaterialPropertyGraph {
    assertions: Vec<MaterialAssertion>,
}

impl SourceSpec {
    /// Creates a source/provenance reference.
    pub fn new(authority: impl Into<String>, locator: impl Into<String>) -> Self {
        Self {
            authority: authority.into(),
            locator: locator.into(),
            freshness: None,
        }
    }
}

impl PropertyValue {
    /// Creates an exact scalar property value.
    pub fn exact_scalar(value: Real) -> Self {
        Self::ExactScalar(Box::new(value))
    }

    /// Creates a validated exact interval.
    pub fn interval(lower: Real, upper: Real) -> PhysicsResult<Self> {
        match lower.partial_cmp(&upper) {
            Some(Ordering::Less | Ordering::Equal) => Ok(Self::Interval {
                lower: Box::new(lower),
                upper: Box::new(upper),
            }),
            Some(Ordering::Greater) | None => Err(PhysicsError::InvalidPropertyInterval),
        }
    }

    /// Creates an external proposal property value with visible replacement status.
    pub fn external_proposal(value: Real, status: ExternalReplacementStatus) -> Self {
        Self::ExternalProposal {
            value: Box::new(value),
            status,
        }
    }
}

impl MaterialPropertyGraph {
    /// Adds a source-attributed assertion.
    pub fn push(&mut self, assertion: MaterialAssertion) {
        self.assertions.push(assertion);
    }

    /// Returns all assertions.
    pub fn assertions(&self) -> &[MaterialAssertion] {
        &self.assertions
    }

    /// Resolves a property without guessing missing values.
    pub fn resolve(&self, kind: &MaterialPropertyKind) -> ResolvedPropertyReport {
        let matching = self
            .assertions
            .iter()
            .filter(|assertion| &assertion.kind == kind)
            .collect::<Vec<_>>();
        let sources = matching
            .iter()
            .map(|assertion| assertion.source.clone())
            .collect::<Vec<_>>();
        if matching.is_empty() {
            return ResolvedPropertyReport {
                kind: kind.clone(),
                status: PropertyResolutionStatus::Unknown,
                value: None,
                sources,
                evidence: vec!["no assertions".into()],
            };
        }

        let exact_values = matching
            .iter()
            .filter_map(|assertion| match &assertion.value {
                PropertyValue::ExactScalar(value) => Some(value),
                _ => None,
            })
            .collect::<Vec<_>>();
        if let Some(first) = exact_values.first() {
            if exact_values.iter().all(|value| *value == *first) {
                return ResolvedPropertyReport {
                    kind: kind.clone(),
                    status: PropertyResolutionStatus::ExactKnown,
                    value: Some(PropertyValue::ExactScalar((*first).clone())),
                    sources,
                    evidence: vec!["all exact assertions agree".into()],
                };
            }
            return ResolvedPropertyReport {
                kind: kind.clone(),
                status: PropertyResolutionStatus::Conflict,
                value: None,
                sources,
                evidence: vec!["conflicting exact assertions".into()],
            };
        }

        if let Some(interval) = matching
            .iter()
            .find_map(|assertion| match &assertion.value {
                PropertyValue::Interval { lower, upper } => Some(PropertyValue::Interval {
                    lower: lower.clone(),
                    upper: upper.clone(),
                }),
                _ => None,
            })
        {
            return ResolvedPropertyReport {
                kind: kind.clone(),
                status: PropertyResolutionStatus::Interval,
                value: Some(interval),
                sources,
                evidence: vec!["interval assertion selected".into()],
            };
        }

        if let Some(proposal) = matching
            .iter()
            .find_map(|assertion| match &assertion.value {
                PropertyValue::ExternalProposal { value, status } => {
                    Some(PropertyValue::ExternalProposal {
                        value: value.clone(),
                        status: *status,
                    })
                }
                _ => None,
            })
        {
            return ResolvedPropertyReport {
                kind: kind.clone(),
                status: PropertyResolutionStatus::ExternalProposal,
                value: Some(proposal),
                sources,
                evidence: vec!["external proposal only".into()],
            };
        }

        ResolvedPropertyReport {
            kind: kind.clone(),
            status: PropertyResolutionStatus::Unknown,
            value: None,
            sources,
            evidence: vec!["only explicit unknown assertions".into()],
        }
    }

    /// Derives isotropic shear modulus `G = E / (2(1 + nu))` from exact inputs.
    pub fn derive_isotropic_shear_modulus(&self) -> PhysicsResult<Option<ElasticDerivationReport>> {
        let young = self.resolve(&MaterialPropertyKind::YoungModulus);
        let poisson = self.resolve(&MaterialPropertyKind::PoissonRatio);
        let Some(PropertyValue::ExactScalar(young_value)) = young.value.clone() else {
            return Ok(None);
        };
        let Some(PropertyValue::ExactScalar(poisson_value)) = poisson.value.clone() else {
            return Ok(None);
        };
        require_positive(young_value.as_ref(), PhysicsError::InvalidElasticConstant)?;
        let denominator = Real::from(2) * (Real::one() + poisson_value.as_ref().clone());
        require_positive(&denominator, PhysicsError::InvalidElasticConstant)?;
        let value = (young_value.as_ref() / &denominator)
            .map_err(|_| PhysicsError::InvalidElasticConstant)?;
        Ok(Some(ElasticDerivationReport {
            kind: MaterialPropertyKind::ShearModulus,
            value,
            sources: young.sources.into_iter().chain(poisson.sources).collect(),
            assumption: "isotropic linear elasticity: G = E / (2(1 + nu))".into(),
        }))
    }
}

fn require_positive(value: &Real, error: PhysicsError) -> PhysicsResult<()> {
    match value.refine_sign_until(-64) {
        Some(RealSign::Positive) => Ok(()),
        Some(RealSign::Negative | RealSign::Zero) | None => Err(error),
    }
}
