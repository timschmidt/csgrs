//! Numeric contracts for import, export, and preview adapters.
//!
//! Adapter outputs are often operationally useful: OBJ triangle voxelizers,
//! image stacks, glTF/Bevy previews, and SDF-like displays can all move data
//! between tools. They are not exact Hyper geometry unless their numeric
//! contract says how source units were scaled, how scalar values were produced,
//! and whether any epsilon/tolerance policy was explicitly bounded.
//!
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997, pp. 3-23: approximate numerical stages can propose
//! work, but exact combinatorial decisions must be replayed or certified rather
//! than inferred from primitive-float tolerances.

use hyperreal::{Real, RealSign};

use crate::LegacyAdapterStatus;

/// Scalar representation used at an adapter boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AdapterScalarPrecision {
    /// Scalars remain exact `Real` values or exact integer grid addresses.
    Exact,
    /// Scalars are approximate but carry a certified bound.
    CertifiedBounded,
    /// Scalars were lowered to primitive floats.
    PrimitiveFloat,
    /// The adapter did not declare scalar precision.
    Unknown,
}

/// Epsilon/tolerance status declared by an adapter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AdapterToleranceStatus {
    /// No epsilon or tolerance participates in the adapter's decisions.
    NotApplicable,
    /// Epsilon and/or tolerance values are explicit and bounded.
    Explicit,
    /// A tolerance-like decision is known to exist, but the value is missing.
    Missing,
    /// The adapter used an implicit lossy tolerance such as a display epsilon.
    LossyImplicit,
}

/// Numeric boundary declaration for a named adapter.
#[derive(Clone, Debug, PartialEq)]
pub struct AdapterNumericContract {
    /// Adapter family and replay status.
    pub adapter: LegacyAdapterStatus,
    /// Exact scale from source units into the voxel/grid contract.
    pub source_scale: Option<Real>,
    /// Declared scalar precision at this boundary.
    pub scalar_precision: AdapterScalarPrecision,
    /// Explicit epsilon, when the adapter names one.
    pub epsilon: Option<Real>,
    /// Explicit tolerance, when the adapter names one.
    pub tolerance: Option<Real>,
    /// Declared epsilon/tolerance status.
    pub tolerance_status: AdapterToleranceStatus,
}

/// Derived audit report for an [`AdapterNumericContract`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdapterNumericReport {
    /// Adapter family and replay status.
    pub adapter: LegacyAdapterStatus,
    /// Whether the adapter supplied non-empty policy/provenance text.
    ///
    /// Yap's EGC separation treats approximate stages as auditable proposals.
    /// An exact replay flag without its boundary policy is therefore
    /// insufficient evidence for exact topology.
    pub adapter_policy_ready: bool,
    /// Declared scalar precision at this boundary.
    pub scalar_precision: AdapterScalarPrecision,
    /// Declared epsilon/tolerance status.
    pub tolerance_status: AdapterToleranceStatus,
    /// Whether a source-to-grid scale is present.
    pub has_explicit_scale: bool,
    /// Whether the declared scale is structurally positive.
    pub scale_is_positive: bool,
    /// Whether epsilon, if present, is structurally non-negative.
    pub epsilon_is_non_negative: bool,
    /// Whether tolerance, if present, is structurally non-negative.
    pub tolerance_is_non_negative: bool,
    /// Whether the adapter supplied at least one numeric error bound.
    ///
    /// This is evidence, not a request to interpret that bound as geometry.
    /// Yap's exact-geometric-computation separation requires approximate
    /// proposals to expose their numerical envelope before an exact layer can
    /// decide whether the proposal is admissible.
    pub has_explicit_error_bound: bool,
    /// Whether the declared tolerance status is internally coherent.
    ///
    /// `Explicit` means an epsilon or tolerance value is actually present and
    /// non-negative. `NotApplicable` means no tolerance value participates.
    /// Missing or implicit lossy tolerances are deliberately incomplete.
    pub tolerance_declaration_complete: bool,
    /// Whether decisions include missing or implicit tolerance uncertainty.
    pub has_unbounded_tolerance: bool,
    /// Whether this adapter can contribute certified metric values.
    pub can_contribute_certified_values: bool,
    /// Whether this adapter can drive exact Hyper topology.
    pub can_drive_exact_topology: bool,
}

impl AdapterNumericContract {
    /// Creates an exact adapter contract with a positive source scale.
    pub fn exact(adapter: LegacyAdapterStatus, source_scale: Real) -> Self {
        Self {
            adapter,
            source_scale: Some(source_scale),
            scalar_precision: AdapterScalarPrecision::Exact,
            epsilon: None,
            tolerance: None,
            tolerance_status: AdapterToleranceStatus::NotApplicable,
        }
    }

    /// Creates a primitive-float adapter contract with optional tolerance data.
    pub fn primitive_float(
        adapter: LegacyAdapterStatus,
        source_scale: Option<Real>,
        epsilon: Option<Real>,
        tolerance: Option<Real>,
        tolerance_status: AdapterToleranceStatus,
    ) -> Self {
        Self {
            adapter,
            source_scale,
            scalar_precision: AdapterScalarPrecision::PrimitiveFloat,
            epsilon,
            tolerance,
            tolerance_status,
        }
    }

    /// Builds a conservative report from declared numeric facts.
    pub fn report(&self) -> AdapterNumericReport {
        let scale_is_positive = self
            .source_scale
            .as_ref()
            .is_some_and(|scale| scale.structural_facts().sign == Some(RealSign::Positive));
        let epsilon_is_non_negative = non_negative_or_absent(self.epsilon.as_ref());
        let tolerance_is_non_negative = non_negative_or_absent(self.tolerance.as_ref());
        let has_explicit_error_bound = self.epsilon.is_some() || self.tolerance.is_some();
        let tolerance_declaration_complete = match self.tolerance_status {
            AdapterToleranceStatus::NotApplicable => !has_explicit_error_bound,
            AdapterToleranceStatus::Explicit => {
                has_explicit_error_bound && epsilon_is_non_negative && tolerance_is_non_negative
            }
            AdapterToleranceStatus::Missing | AdapterToleranceStatus::LossyImplicit => false,
        };
        let has_unbounded_tolerance = matches!(
            self.tolerance_status,
            AdapterToleranceStatus::Missing | AdapterToleranceStatus::LossyImplicit
        );
        let exact_scalar = self.scalar_precision == AdapterScalarPrecision::Exact;
        let certified_scalar = matches!(
            self.scalar_precision,
            AdapterScalarPrecision::Exact | AdapterScalarPrecision::CertifiedBounded
        );
        let adapter_policy_ready = self.adapter.has_policy();
        let bounded_tolerance = !has_unbounded_tolerance
            && tolerance_declaration_complete
            && epsilon_is_non_negative
            && tolerance_is_non_negative
            && scale_is_positive;

        AdapterNumericReport {
            adapter: self.adapter.clone(),
            adapter_policy_ready,
            scalar_precision: self.scalar_precision,
            tolerance_status: self.tolerance_status,
            has_explicit_scale: self.source_scale.is_some(),
            scale_is_positive,
            epsilon_is_non_negative,
            tolerance_is_non_negative,
            has_explicit_error_bound,
            tolerance_declaration_complete,
            has_unbounded_tolerance,
            can_contribute_certified_values: certified_scalar && bounded_tolerance,
            can_drive_exact_topology: self.adapter.exact_replay_ready()
                && exact_scalar
                && tolerance_declaration_complete
                && self.tolerance_status == AdapterToleranceStatus::NotApplicable
                && scale_is_positive,
        }
    }
}

fn non_negative_or_absent(value: Option<&Real>) -> bool {
    match value.map(|value| value.structural_facts().sign) {
        None => true,
        Some(Some(RealSign::Zero | RealSign::Positive)) => true,
        Some(Some(RealSign::Negative)) | Some(None) => false,
    }
}
