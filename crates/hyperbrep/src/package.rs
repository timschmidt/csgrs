//! Aggregated exact BREP handoff package reports.
//!
//! This module does not add a new geometry algorithm. It packages existing
//! retained-surface, retained-solid, exact-triangle, physics, and voxel
//! preflight reports into one replayed boundary so downstream crates can ask a
//! single object which exact surfaces are ready and which requested routes are
//! unsupported or blocked.

use hyperreal::Real;
use hypervoxel::{GridFrame, GridSource};

use crate::handoff::{BrepExactSolidHandoffReport, BrepExactSurfaceHandoffReport};
use crate::physics::BrepPhysicsMassHandoffReport;
use crate::provenance::BrepConstructionManifest;
use crate::topology::BrepShell;
use crate::triangle::BrepExactTriangleMeshHandoffReport;
use crate::voxel::BrepVoxelHandoffReport;

/// Optional voxel handoff request carried by a package manifest.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepVoxelPackageRequest {
    /// Grid frame supplied by the voxel owner.
    pub frame: GridFrame,
    /// Expected source/version for frame freshness replay.
    pub expected_source: Option<GridSource>,
    /// Whether the caller requires full exact triangle voxelization now.
    ///
    /// The BREP package can provide exact AABB evidence and, for exact-ready
    /// retained solids, a prepared `hypervoxel` triangle-solid schedule.
    /// Keeping this as an explicit request flag follows Yap's
    /// exact-computation boundary: a blocked voxel route remains named
    /// evidence, not a silent fallback.
    pub require_triangle_voxelization: bool,
}

/// Request manifest for a consolidated BREP handoff package.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepHandoffPackageManifest {
    /// Optional uniform density for exact physics mass-property replay.
    pub physics_density: Option<Real>,
    /// Optional voxel preflight request.
    pub voxel: Option<BrepVoxelPackageRequest>,
}

/// Explicit blocker for a consolidated BREP handoff package.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepHandoffPackageBlocker {
    /// Retained surface evidence was not exact-ready.
    SurfaceHandoffNotReady,
    /// Retained solid evidence was not exact-ready.
    SolidHandoffNotReady,
    /// Retained exact triangle lowering was not exact-ready.
    TriangleMeshNotReady,
    /// Requested physics mass-property replay was not exact-ready.
    PhysicsMassNotReady,
    /// Requested voxel AABB preflight was not exact-ready.
    VoxelAabbNotReady,
    /// Requested full exact triangle voxelization is unavailable or blocked.
    TriangleVoxelizationUnavailable,
}

/// Consolidated exact handoff package for one retained BREP shell.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepHandoffPackageReport {
    /// Replayed retained-surface handoff.
    pub surface: BrepExactSurfaceHandoffReport,
    /// Replayed retained-solid handoff.
    pub solid: BrepExactSolidHandoffReport,
    /// Replayed retained exact triangle handoff.
    pub triangle_mesh: BrepExactTriangleMeshHandoffReport,
    /// Optional requested physics mass-property report.
    pub physics_mass: Option<BrepPhysicsMassHandoffReport>,
    /// Optional requested voxel preflight report.
    pub voxel: Option<BrepVoxelHandoffReport>,
    /// Whether exact retained surface evidence is ready.
    pub exact_surface_ready: bool,
    /// Whether exact retained solid evidence is ready.
    pub exact_solid_ready: bool,
    /// Whether exact retained triangles are ready.
    pub exact_triangle_mesh_ready: bool,
    /// Whether requested physics evidence is absent or exact-ready.
    pub requested_physics_ready: bool,
    /// Whether requested voxel AABB evidence is absent or exact-ready.
    pub requested_voxel_aabb_ready: bool,
    /// Whether requested exact triangle voxelization is absent or exact-ready.
    pub requested_triangle_voxelization_ready: bool,
    /// Explicit blockers across requested handoff domains.
    pub blockers: Vec<BrepHandoffPackageBlocker>,
    /// Whether every requested exact handoff in this package is ready.
    pub all_requested_exact_ready: bool,
}

impl BrepHandoffPackageManifest {
    /// Build a manifest with no optional domain requests.
    pub const fn basic() -> Self {
        Self {
            physics_density: None,
            voxel: None,
        }
    }

    /// Request exact uniform-density physics mass properties.
    pub fn with_physics_density(mut self, density: Real) -> Self {
        self.physics_density = Some(density);
        self
    }

    /// Request a voxel preflight package for the supplied grid frame.
    pub fn with_voxel(mut self, request: BrepVoxelPackageRequest) -> Self {
        self.voxel = Some(request);
        self
    }
}

impl BrepHandoffPackageReport {
    /// Build a consolidated package from current retained BREP evidence.
    ///
    /// The package follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): every domain handoff is replayed
    /// from the current object and every missing or unavailable route remains a
    /// named blocker. It is a convenience envelope over the lower reports, not
    /// a substitute for the detailed evidence those reports carry.
    pub fn from_shell_manifest(
        shell: &BrepShell,
        construction: Option<&BrepConstructionManifest>,
        manifest: BrepHandoffPackageManifest,
    ) -> Self {
        let surface = shell.exact_surface_handoff();
        let solid = shell.exact_solid_handoff(construction);
        let triangle_mesh = shell.exact_triangle_mesh_handoff_report();
        let physics_mass = manifest
            .physics_density
            .map(|density| shell.physics_mass_handoff_report(density));
        let voxel = manifest.voxel.as_ref().map(|request| {
            shell.voxel_handoff_report(request.frame.clone(), request.expected_source.clone())
        });

        let exact_surface_ready = surface.exact_surface_handoff_ready;
        let exact_solid_ready = solid.exact_solid_handoff_ready;
        let exact_triangle_mesh_ready = triangle_mesh.exact_triangle_mesh_ready;
        let requested_physics_ready = physics_mass
            .as_ref()
            .is_none_or(|report| report.exact_physics_mass_ready);
        let requested_voxel_aabb_ready = voxel
            .as_ref()
            .is_none_or(|report| report.exact_aabb_handoff_ready);
        let requested_triangle_voxelization_ready = match (&manifest.voxel, &voxel) {
            (Some(request), Some(report)) if request.require_triangle_voxelization => {
                report.exact_triangle_voxelization_ready
            }
            _ => true,
        };

        let mut blockers = Vec::new();
        if !exact_surface_ready {
            blockers.push(BrepHandoffPackageBlocker::SurfaceHandoffNotReady);
        }
        if !exact_solid_ready {
            blockers.push(BrepHandoffPackageBlocker::SolidHandoffNotReady);
        }
        if !exact_triangle_mesh_ready {
            blockers.push(BrepHandoffPackageBlocker::TriangleMeshNotReady);
        }
        if !requested_physics_ready {
            blockers.push(BrepHandoffPackageBlocker::PhysicsMassNotReady);
        }
        if !requested_voxel_aabb_ready {
            blockers.push(BrepHandoffPackageBlocker::VoxelAabbNotReady);
        }
        if !requested_triangle_voxelization_ready {
            blockers.push(BrepHandoffPackageBlocker::TriangleVoxelizationUnavailable);
        }

        let all_requested_exact_ready = blockers.is_empty();
        Self {
            surface,
            solid,
            triangle_mesh,
            physics_mass,
            voxel,
            exact_surface_ready,
            exact_solid_ready,
            exact_triangle_mesh_ready,
            requested_physics_ready,
            requested_voxel_aabb_ready,
            requested_triangle_voxelization_ready,
            blockers,
            all_requested_exact_ready,
        }
    }
}

impl BrepShell {
    /// Build a consolidated exact handoff package from this shell.
    pub fn handoff_package_report(
        &self,
        construction: Option<&BrepConstructionManifest>,
        manifest: BrepHandoffPackageManifest,
    ) -> BrepHandoffPackageReport {
        BrepHandoffPackageReport::from_shell_manifest(self, construction, manifest)
    }
}
