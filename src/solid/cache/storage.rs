//! Typed keys and completed results for the grammar cache policy.

use super::{CachePolicy, CacheUsage, STATE};
use crate::errors::ValidationError;
use crate::{GeometryContext, GeometryOutcome};
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::{Triangle, TriangleMesh};
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq)]
pub(in crate::solid) enum PrimitiveParameters {
    Cuboid(Real, Real, Real),
    Sphere(Real, usize, usize),
    Ellipsoid(Real, Real, Real, usize, usize),
    FrustumBetween(Box<Point3>, Box<Point3>, Real, Real, usize),
    Octahedron(Real),
    Icosahedron(Real),
    #[cfg(feature = "curve")]
    Torus(Real, Real, usize, usize),
}

#[derive(Clone, Debug, PartialEq)]
pub(in crate::solid) enum DistributionParameters {
    Linear {
        count: usize,
        direction: Vector3,
        spacing: Real,
    },
    Grid {
        rows: usize,
        columns: usize,
        dx: Real,
        dy: Real,
    },
    Arc {
        count: usize,
        radius: Real,
        start_angle_degrees: Real,
        end_angle_degrees: Real,
    },
}

struct MeshStorage {
    positions: Arc<[Point3]>,
    triangles: Arc<[Triangle]>,
    construction: hypermesh::ConstructionEvidence,
}

impl MeshStorage {
    fn new(mesh: &TriangleMesh) -> Self {
        Self {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            construction: mesh.construction_evidence(),
        }
    }

    fn matches(&self, mesh: &TriangleMesh) -> bool {
        Arc::ptr_eq(&self.positions, &mesh.positions)
            && Arc::ptr_eq(&self.triangles, &mesh.triangles)
            && self.construction == mesh.construction_evidence()
    }
}

struct CachedDistribution {
    source: MeshStorage,
    parameters: DistributionParameters,
    context: GeometryContext,
    result: GeometryOutcome<TriangleMesh>,
}

struct CachedMerge {
    sources: Vec<MeshStorage>,
    result: TriangleMesh,
}

#[cfg(feature = "curve")]
struct CachedFlatten {
    source: MeshStorage,
    context: GeometryContext,
    region: GeometryOutcome<hypercurve::CurveRegion2>,
}

#[cfg(feature = "curve")]
struct CachedSlice {
    source: MeshStorage,
    z: Real,
    context: GeometryContext,
    result: GeometryOutcome<crate::solid::SliceResult>,
}

pub(super) struct CacheState {
    pub(super) policy: CachePolicy,
    primitives: Vec<(PrimitiveParameters, TriangleMesh)>,
    distributions: Vec<CachedDistribution>,
    merges: Vec<CachedMerge>,
    #[cfg(feature = "curve")]
    flattenings: Vec<CachedFlatten>,
    #[cfg(feature = "curve")]
    slices: Vec<CachedSlice>,
}

fn trim<T>(entries: &mut Vec<T>, limit: usize) {
    if limit == 0 {
        *entries = Vec::new();
    } else if entries.len() > limit {
        entries.drain(..entries.len() - limit);
    }
}

fn retain<T>(entries: &mut Vec<T>, limit: usize, build: impl FnOnce() -> T) {
    if limit != 0 {
        if entries.len() >= limit {
            entries.remove(0);
        }
        entries.push(build());
    }
}

impl CacheState {
    pub(super) const fn new() -> Self {
        Self {
            policy: CachePolicy::DEFAULT,
            primitives: Vec::new(),
            distributions: Vec::new(),
            merges: Vec::new(),
            #[cfg(feature = "curve")]
            flattenings: Vec::new(),
            #[cfg(feature = "curve")]
            slices: Vec::new(),
        }
    }

    pub(super) fn set_policy(&mut self, policy: CachePolicy) {
        trim(&mut self.primitives, policy.primitives);
        trim(&mut self.distributions, policy.distributions);
        trim(&mut self.merges, policy.merges);
        #[cfg(feature = "curve")]
        {
            trim(&mut self.flattenings, policy.flattenings);
            trim(&mut self.slices, policy.slices);
        }
        self.policy = policy;
    }

    pub(super) const fn usage(&self) -> CacheUsage {
        CacheUsage {
            primitives: self.primitives.len(),
            distributions: self.distributions.len(),
            merges: self.merges.len(),
            #[cfg(feature = "curve")]
            flattenings: self.flattenings.len(),
            #[cfg(not(feature = "curve"))]
            flattenings: 0,
            #[cfg(feature = "curve")]
            slices: self.slices.len(),
            #[cfg(not(feature = "curve"))]
            slices: 0,
        }
    }
}

#[inline]
pub(in crate::solid) fn retained_primitive(
    parameters: PrimitiveParameters,
    build: impl FnOnce() -> Result<TriangleMesh, ValidationError>,
) -> Result<TriangleMesh, ValidationError> {
    if let Some(mesh) = STATE.with_borrow(|state| {
        state
            .primitives
            .iter()
            .find_map(|(cached, mesh)| (cached == &parameters).then(|| mesh.clone()))
    }) {
        return Ok(mesh);
    }
    let mesh = build()?;
    STATE.with_borrow_mut(|state| {
        retain(&mut state.primitives, state.policy.primitives, || {
            (parameters, mesh.clone())
        });
    });
    Ok(mesh)
}

pub(in crate::solid) fn retained_distribution(
    mesh: &TriangleMesh,
    parameters: &DistributionParameters,
    context: &GeometryContext,
) -> Option<GeometryOutcome<TriangleMesh>> {
    STATE.with_borrow(|state| {
        state.distributions.iter().find_map(|entry| {
            (entry.source.matches(mesh)
                && &entry.parameters == parameters
                && &entry.context == context)
                .then(|| entry.result.clone())
        })
    })
}

pub(in crate::solid) fn retain_distribution(
    mesh: &TriangleMesh,
    parameters: DistributionParameters,
    context: &GeometryContext,
    result: &GeometryOutcome<TriangleMesh>,
) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.distributions, state.policy.distributions, || {
            CachedDistribution {
                source: MeshStorage::new(mesh),
                parameters,
                context: *context,
                result: result.clone(),
            }
        });
    });
}

pub(in crate::solid) fn retained_merge(meshes: &[TriangleMesh]) -> Option<TriangleMesh> {
    STATE.with_borrow(|state| {
        state.merges.iter().find_map(|entry| {
            (entry.sources.len() == meshes.len()
                && entry
                    .sources
                    .iter()
                    .zip(meshes)
                    .all(|(source, mesh)| source.matches(mesh)))
            .then(|| entry.result.clone())
        })
    })
}

pub(in crate::solid) fn retain_merge(meshes: &[TriangleMesh], result: &TriangleMesh) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.merges, state.policy.merges, || CachedMerge {
            sources: meshes.iter().map(MeshStorage::new).collect(),
            result: result.clone(),
        });
    });
}

#[cfg(feature = "curve")]
pub(in crate::solid) fn retained_flatten(
    mesh: &TriangleMesh,
    context: &GeometryContext,
) -> Option<GeometryOutcome<hypercurve::CurveRegion2>> {
    STATE.with_borrow(|state| {
        state.flattenings.iter().find_map(|entry| {
            (entry.source.matches(mesh) && entry.context == *context)
                .then(|| entry.region.clone())
        })
    })
}

#[cfg(feature = "curve")]
pub(in crate::solid) fn retain_flatten(
    mesh: &TriangleMesh,
    context: &GeometryContext,
    region: &GeometryOutcome<hypercurve::CurveRegion2>,
) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.flattenings, state.policy.flattenings, || {
            CachedFlatten {
                source: MeshStorage::new(mesh),
                context: *context,
                region: region.clone(),
            }
        });
    });
}

#[cfg(feature = "curve")]
pub(in crate::solid) fn retained_slice(
    mesh: &TriangleMesh,
    z: &Real,
    context: &GeometryContext,
) -> Option<GeometryOutcome<crate::solid::SliceResult>> {
    STATE.with_borrow(|state| {
        state.slices.iter().find_map(|entry| {
            (entry.source.matches(mesh) && &entry.z == z && entry.context == *context)
                .then(|| entry.result.clone())
        })
    })
}

#[cfg(feature = "curve")]
pub(in crate::solid) fn retain_slice(
    mesh: &TriangleMesh,
    z: Real,
    context: &GeometryContext,
    result: &GeometryOutcome<crate::solid::SliceResult>,
) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.slices, state.policy.slices, || CachedSlice {
            source: MeshStorage::new(mesh),
            z,
            context: *context,
            result: result.clone(),
        });
    });
}
