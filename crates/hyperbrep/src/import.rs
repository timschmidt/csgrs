//! Lossy primitive-float BREP import audit.
//!
//! External CAD, viewer, and interchange sources often arrive as primitive
//! floats plus surface-family names. This module only audits that input. It
//! does not promote imported topology to trusted BREP evidence; exact topology
//! still has to replay through `hyperlimit` predicates and `hyperbrep`
//! validation reports.

use std::collections::BTreeSet;

use hyperreal::Real;

/// Surface family declared by a lossy/import adapter.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepImportedSurfaceFamily {
    /// Plane surface.
    Plane,
    /// Cylinder surface, unsupported until analytic frame facts exist.
    Cylinder,
    /// Cone surface, unsupported until analytic frame facts exist.
    Cone,
    /// Sphere surface, unsupported until analytic frame facts exist.
    Sphere,
    /// General NURBS surface, unsupported in the current exact BREP core.
    Nurbs,
    /// Named source-specific surface family.
    Other(String),
    /// Surface family was not declared.
    Unknown,
}

impl BrepImportedSurfaceFamily {
    /// Returns whether this imported surface family is currently exact-core supported.
    pub const fn is_supported_now(&self) -> bool {
        matches!(self, Self::Plane)
    }
}

/// Source precision declared by an import adapter.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepPrimitiveFloatPrecision {
    /// 32-bit primitive float source.
    F32,
    /// 64-bit primitive float source.
    F64,
    /// Source precision was not declared.
    Unknown,
}

/// One unsupported imported surface-family record.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepUnsupportedSurfaceRecord {
    /// Surface index in the source adapter payload.
    pub surface_index: usize,
    /// Declared family.
    pub family: BrepImportedSurfaceFamily,
}

/// Explicit blocker for primitive-float BREP import readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepLossyImportBlocker {
    /// Source precision was not declared.
    UnknownPrecision,
    /// Coordinate buffer length is not a multiple of 3.
    InvalidCoordinateArity,
    /// At least one coordinate is NaN or infinite.
    NonFiniteCoordinate,
    /// At least one finite coordinate could not be lifted as an exact dyadic.
    FailedDyadicLift,
    /// Adapter did not provide topology evidence.
    MissingTopologyEvidence,
    /// Adapter did not declare source tolerances.
    MissingTolerance,
    /// Adapter declared unsupported surface families.
    UnsupportedSurfaceKind,
    /// At least one surface family was undeclared.
    UnknownSurfaceKind,
}

/// Audit report for finite primitive-float BREP import.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepLossyFloatImportReport {
    /// Source adapter or file-family label.
    pub source: String,
    /// Declared source precision.
    pub precision: BrepPrimitiveFloatPrecision,
    /// Number of primitive coordinates supplied.
    pub coordinate_count: usize,
    /// Number of complete point triplets.
    pub point_count: usize,
    /// Number of finite coordinates.
    pub finite_coordinate_count: usize,
    /// Number of coordinates lifted as exact dyadics.
    pub exact_dyadic_lift_count: usize,
    /// Number of coordinates lifted from exact decimal text.
    pub exact_decimal_lift_count: usize,
    /// Indexes of non-finite coordinates.
    pub non_finite_coordinate_indexes: Vec<usize>,
    /// Whether source topology evidence was supplied by the adapter.
    pub topology_evidence_declared: bool,
    /// Whether source tolerance metadata was supplied by the adapter.
    pub tolerance_declared: bool,
    /// Number of imported surface families.
    pub surface_count: usize,
    /// Unsupported imported surface families.
    pub unsupported_surfaces: Vec<BrepUnsupportedSurfaceRecord>,
    /// Number of unknown imported surface families.
    pub unknown_surface_count: usize,
    /// Explicit blockers.
    pub blockers: Vec<BrepLossyImportBlocker>,
    /// Whether this import is ready to be passed to exact validation/replay.
    pub adapter_replay_ready: bool,
    /// Whether this report is still a lossy adapter boundary.
    pub lossy_adapter_only: bool,
}

impl BrepLossyFloatImportReport {
    /// Inspect primitive-float BREP adapter inputs without trusting topology.
    ///
    /// Finite IEEE-754 values can be represented exactly as dyadic rationals in
    /// `hyperreal::Real`, so the coordinate lift itself is exact. The imported
    /// geometry is still a lossy adapter artifact because the source topology,
    /// tolerances, and surface semantics came from an external system. This is
    /// the same separation required by Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): exact scalar
    /// encodings do not by themselves certify BREP combinatorics.
    pub fn inspect_f64(
        source: impl Into<String>,
        coordinates: &[f64],
        surfaces: &[BrepImportedSurfaceFamily],
        topology_evidence_declared: bool,
        tolerance_declared: bool,
    ) -> Self {
        let mut finite_coordinate_count = 0_usize;
        let mut exact_dyadic_lift_count = 0_usize;
        let mut non_finite_coordinate_indexes = Vec::new();
        for (index, value) in coordinates.iter().copied().enumerate() {
            if value.is_finite() {
                finite_coordinate_count += 1;
                if Real::try_from(value).is_ok_and(|lifted| lifted.is_exact_dyadic_rational()) {
                    exact_dyadic_lift_count += 1;
                }
            } else {
                non_finite_coordinate_indexes.push(index);
            }
        }

        let unsupported_surfaces = surfaces
            .iter()
            .enumerate()
            .filter(|(_, family)| !family.is_supported_now())
            .map(|(surface_index, family)| BrepUnsupportedSurfaceRecord {
                surface_index,
                family: family.clone(),
            })
            .collect::<Vec<_>>();
        let unknown_surface_count = surfaces
            .iter()
            .filter(|family| matches!(family, BrepImportedSurfaceFamily::Unknown))
            .count();

        let mut blockers = BTreeSet::new();
        if !coordinates.len().is_multiple_of(3) {
            blockers.insert(BrepLossyImportBlocker::InvalidCoordinateArity);
        }
        if !non_finite_coordinate_indexes.is_empty() {
            blockers.insert(BrepLossyImportBlocker::NonFiniteCoordinate);
        }
        if exact_dyadic_lift_count != finite_coordinate_count {
            blockers.insert(BrepLossyImportBlocker::FailedDyadicLift);
        }
        if !topology_evidence_declared {
            blockers.insert(BrepLossyImportBlocker::MissingTopologyEvidence);
        }
        if !tolerance_declared {
            blockers.insert(BrepLossyImportBlocker::MissingTolerance);
        }
        if unsupported_surfaces
            .iter()
            .any(|record| !matches!(record.family, BrepImportedSurfaceFamily::Unknown))
        {
            blockers.insert(BrepLossyImportBlocker::UnsupportedSurfaceKind);
        }
        if unknown_surface_count > 0 {
            blockers.insert(BrepLossyImportBlocker::UnknownSurfaceKind);
        }

        let blockers = blockers.into_iter().collect::<Vec<_>>();
        Self {
            source: source.into(),
            precision: BrepPrimitiveFloatPrecision::F64,
            coordinate_count: coordinates.len(),
            point_count: coordinates.len() / 3,
            finite_coordinate_count,
            exact_dyadic_lift_count,
            exact_decimal_lift_count: 0,
            non_finite_coordinate_indexes,
            topology_evidence_declared,
            tolerance_declared,
            surface_count: surfaces.len(),
            unsupported_surfaces,
            unknown_surface_count,
            adapter_replay_ready: blockers.is_empty(),
            blockers,
            lossy_adapter_only: true,
        }
    }
}
