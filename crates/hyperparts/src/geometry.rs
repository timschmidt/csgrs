//! Geometry handle and handoff reports.
//!
//! `hyperparts` indexes geometry provenance but does not own CAD, CSG, curve,
//! mesh, voxel, or footprint kernels. Geometry handoff reports name the owning
//! crate/artifact and whether the geometry is exact, certified, lossy, display
//! only, or missing. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): exact downstream code
//! should receive explicit provenance and uncertainty rather than unreviewed
//! mesh/display artifacts posing as certified geometry.

use crate::{PartId, PartQueryEvidence};

/// Geometry status for handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeometryStatus {
    /// Exact native geometry handle.
    Exact,
    /// Certified bounded/imported geometry.
    Certified,
    /// Lossy imported mesh.
    LossyMesh,
    /// Display-only artifact.
    DisplayOnly,
    /// Missing geometry.
    Missing,
}

/// Generic geometry handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GeometryHandle {
    /// Owning crate or external format.
    pub owner: String,
    /// Handle within the owner.
    pub handle: String,
    /// Units/provenance label.
    pub units: Option<String>,
}

/// Source family for geometry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ShapeSource {
    /// `csgrs` or CSG solid.
    Csg,
    /// Future `hypercad` solid/model.
    HyperCad,
    /// `hypercurve` 2D profile.
    HyperCurve,
    /// KiCad/LibrePCB/Altium footprint.
    Footprint,
    /// STEP/WRL/SCAD/STL artifact.
    ModelArtifact,
    /// Replimat/Gridbeam grid definition.
    Grid,
    /// Source-specific geometry.
    Custom(String),
}

/// Model artifact reference.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModelArtifact {
    /// Artifact handle or path.
    pub handle: String,
    /// Artifact format.
    pub format: String,
}

/// Footprint handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FootprintHandle {
    /// Library handle.
    pub library: String,
    /// Footprint name.
    pub name: String,
}

/// Package body handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PackageBodyHandle {
    /// Body handle.
    pub handle: String,
}

/// Curve/profile handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CurveProfileHandle {
    /// Profile handle.
    pub handle: String,
}

/// Modular grid system.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GridSystem {
    /// Grid name.
    pub name: String,
    /// Exact or textual spacing descriptor.
    pub spacing: String,
}

/// Feature tied to a grid.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GridFeature {
    /// Feature handle.
    pub handle: String,
    /// Grid system.
    pub grid: GridSystem,
}

/// Mounting pattern.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MountingPattern {
    /// Pattern handle.
    pub handle: String,
    /// Features in the pattern.
    pub features: Vec<GridFeature>,
}

/// Geometry handoff report.
#[derive(Clone, Debug, PartialEq)]
pub struct GeometryHandoffReport {
    /// Part associated with this handoff.
    pub part: PartId,
    /// Shape source.
    pub source: ShapeSource,
    /// Geometry handle.
    pub geometry: Option<GeometryHandle>,
    /// Handoff status.
    pub status: GeometryStatus,
    /// Evidence.
    pub evidence: PartQueryEvidence,
}
