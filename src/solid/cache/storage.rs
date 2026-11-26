//! Typed keys and completed results for the grammar cache policy.

use super::{CachePolicy, CacheUsage, STATE};
use crate::errors::ValidationError;
use crate::{GeometryContext, GeometryOutcome};
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::{Triangle, TriangleMesh};
use std::sync::Arc;

/// Structural constructor key for strict primitive builders.
/// Equality compares stored scalar representations and sample counts; it neither
/// solves mathematical scalar equality nor validates constructor arguments.
#[derive(Clone, Debug, PartialEq)]
pub(in crate::solid) enum PrimitiveParameters {
    /// Origin-anchored box dimensions.
    Cuboid(
        /// X dimension.
        Real,
        /// Y dimension.
        Real,
        /// Z dimension.
        Real,
    ),
    /// Sampled sphere parameters.
    Sphere(
        /// Radius.
        Real,
        /// Longitude segment count.
        usize,
        /// Latitude stack count.
        usize,
    ),
    /// Sampled ellipsoid parameters.
    Ellipsoid(
        /// X radius.
        Real,
        /// Y radius.
        Real,
        /// Z radius.
        Real,
        /// Longitude segment count.
        usize,
        /// Latitude stack count.
        usize,
    ),
    /// Directed endpoint-defined frustum parameters.
    FrustumBetween(
        /// Start center.
        Box<Point3>,
        /// End center.
        Box<Point3>,
        /// Radius at the start center.
        Real,
        /// Radius at the end center.
        Real,
        /// Circumferential segment count.
        usize,
    ),
    /// Octahedron constructor key.
    Octahedron(
        /// Requested radius.
        Real,
    ),
    /// Icosahedron constructor key.
    Icosahedron(
        /// Requested radius.
        Real,
    ),
    #[cfg(feature = "curve")]
    /// Sampled torus parameters.
    Torus(
        /// Major radius.
        Real,
        /// Minor radius.
        Real,
        /// Major-ring segment count.
        usize,
        /// Tube segment count.
        usize,
    ),
}

/// Authored layout key, compared structurally before normalized geometry is built.
/// Equivalent geometric layouts with different parameter representations need not hit.
#[derive(Clone, Debug, PartialEq)]
pub(in crate::solid) enum DistributionParameters {
    /// Copies along a normalized direction with signed spacing.
    Linear {
        /// Number of requested copies.
        count: usize,
        /// Authored direction, retained before normalization.
        direction: Vector3,
        /// Signed distance between successive copies.
        spacing: Real,
    },
    /// Copies on an XY grid.
    Grid {
        /// Number of Y rows.
        rows: usize,
        /// Number of X columns.
        columns: usize,
        /// Signed X spacing.
        dx: Real,
        /// Signed Y spacing.
        dy: Real,
    },
    /// Copies placed and rotated along an XY arc.
    Arc {
        /// Number of copies; one uses the angular midpoint.
        count: usize,
        /// Directed radial displacement before rotation.
        radius: Real,
        /// Starting angle in degrees.
        start_angle_degrees: Real,
        /// Ending angle in degrees.
        end_angle_degrees: Real,
    },
}

/// Owned source identities and evidence snapshot used by mesh-dependent caches.
/// Holding both `Arc`s prevents address reuse while the key is retained; equality
/// of distinct buffers is deliberately not a cache hit.
struct MeshStorage {
    /// Retained position allocation identity.
    positions: Arc<[Point3]>,
    /// Retained triangle-index allocation identity.
    triangles: Arc<[Triangle]>,
    /// Effective construction qualifications at insertion, including invalidated history.
    construction: hypermesh::ConstructionEvidence,
}

impl MeshStorage {
    /// Retain both source allocations and snapshot their effective construction evidence.
    fn new(mesh: &TriangleMesh) -> Self {
        Self {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            construction: mesh.construction_evidence(),
        }
    }

    /// Require both pointer identities and current effective construction evidence to match.
    fn matches(&self, mesh: &TriangleMesh) -> bool {
        Arc::ptr_eq(&self.positions, &mesh.positions)
            && Arc::ptr_eq(&self.triangles, &mesh.triangles)
            && self.construction == mesh.construction_evidence()
    }
}

/// Completed distribution with its source, layout and policy key.
struct CachedDistribution {
    /// Source allocations and qualifications.
    source: MeshStorage,
    /// Structural authored layout.
    parameters: DistributionParameters,
    /// Exact selected predicate policy; stronger/weaker policies are not interchanged.
    context: GeometryContext,
    /// Completed mesh and its operation-local receipt.
    result: GeometryOutcome<TriangleMesh>,
}

/// Completed concatenation keyed by ordered source storage.
struct CachedMerge {
    /// Source identities and qualifications in concatenation order.
    sources: Vec<MeshStorage>,
    /// Completed concatenated mesh and retained construction evidence.
    result: TriangleMesh,
}

#[cfg(feature = "curve")]
/// Completed XY projection under a selected predicate policy.
struct CachedFlatten {
    /// Input allocations and qualifications.
    source: MeshStorage,
    /// Policy required for a hit.
    context: GeometryContext,
    /// Completed region with its local predicate receipt.
    region: GeometryOutcome<hypercurve::CurveRegion2>,
}

#[cfg(feature = "curve")]
/// Completed horizontal slice with height and policy key.
struct CachedSlice {
    /// Input allocations and qualifications.
    source: MeshStorage,
    /// Structural scalar height, not a rounded coordinate.
    z: Real,
    /// Policy required for a hit.
    context: GeometryContext,
    /// Filled region, open strings and paths with their aggregate local receipt.
    result: GeometryOutcome<crate::solid::SliceResult>,
}

/// Current thread's limits and FIFO entry collections.
/// Entry counts do not bound retained source/result bytes; cleared or evicted
/// values can also remain alive through copies held outside this state.
pub(super) struct CacheState {
    /// Active entry limits, independent of predicate policy.
    pub(super) policy: CachePolicy,
    /// Completed strict primitives in insertion order.
    primitives: Vec<(PrimitiveParameters, TriangleMesh)>,
    /// Completed distributions in insertion order.
    distributions: Vec<CachedDistribution>,
    /// Completed concatenations in insertion order.
    merges: Vec<CachedMerge>,
    #[cfg(feature = "curve")]
    /// Completed XY projections in insertion order.
    flattenings: Vec<CachedFlatten>,
    #[cfg(feature = "curve")]
    /// Completed horizontal slices in insertion order.
    slices: Vec<CachedSlice>,
}

/// Drop oldest entries until the limit holds; zero also releases vector capacity.
/// Positive-limit shrinking leaves the vector allocation available for reuse.
fn trim<T>(entries: &mut Vec<T>, limit: usize) {
    if limit == 0 {
        *entries = Vec::new();
    } else if entries.len() > limit {
        entries.drain(..entries.len() - limit);
    }
}

/// Append a lazily built entry if enabled, evicting the oldest at capacity.
/// Requires the existing length not to exceed `limit`; policy changes establish
/// that invariant. A zero limit never invokes `build`. Does not deduplicate keys.
fn retain<T>(entries: &mut Vec<T>, limit: usize, build: impl FnOnce() -> T) {
    if limit != 0 {
        if entries.len() >= limit {
            entries.remove(0);
        }
        entries.push(build());
    }
}

impl CacheState {
    /// Create empty collections under the default entry limits.
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

    /// Trim each enabled collection before publishing the new limits.
    /// Increasing a limit does not reserve capacity or restore evicted entries.
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

    /// Snapshot collection lengths, reporting zero for compiled-out curve caches.
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
/// Return the first structural-key hit, or build and retain a successful primitive.
/// Runs `build` outside the state borrow, then uses the policy active on insertion.
/// Errors are not retained; hits clone the mesh without refreshing FIFO order.
/// Callers supply the strict primitive implementation corresponding to the key.
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

/// Clone the first matching source/layout/context result without refreshing FIFO order.
pub(in crate::solid) fn retained_distribution(
    mesh: &TriangleMesh,
    parameters: &DistributionParameters,
    context: GeometryContext,
) -> Option<GeometryOutcome<TriangleMesh>> {
    STATE.with_borrow(|state| {
        state.distributions.iter().find_map(|entry| {
            (entry.source.matches(mesh)
                && &entry.parameters == parameters
                && entry.context == context)
                .then(|| entry.result.clone())
        })
    })
}

/// Retain a caller-completed distribution and its receipt under the active entry limit.
/// No validation is performed here; source, parameters and context must describe that result.
pub(in crate::solid) fn retain_distribution(
    mesh: &TriangleMesh,
    parameters: DistributionParameters,
    context: GeometryContext,
    result: &GeometryOutcome<TriangleMesh>,
) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.distributions, state.policy.distributions, || {
            CachedDistribution {
                source: MeshStorage::new(mesh),
                parameters,
                context,
                result: result.clone(),
            }
        });
    });
}

/// Clone the first concatenation whose source identities/evidence match in order.
/// No predicate context is keyed because this cache stores buffer concatenation.
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

/// Retain a caller-completed concatenation with ordered source identities and evidence.
pub(in crate::solid) fn retain_merge(meshes: &[TriangleMesh], result: &TriangleMesh) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.merges, state.policy.merges, || CachedMerge {
            sources: meshes.iter().map(MeshStorage::new).collect(),
            result: result.clone(),
        });
    });
}

#[cfg(feature = "curve")]
/// Clone the first XY projection matching source identity, qualifications and context.
pub(in crate::solid) fn retained_flatten(
    mesh: &TriangleMesh,
    context: GeometryContext,
) -> Option<GeometryOutcome<hypercurve::CurveRegion2>> {
    STATE.with_borrow(|state| {
        state.flattenings.iter().find_map(|entry| {
            (entry.source.matches(mesh) && entry.context == context)
                .then(|| entry.region.clone())
        })
    })
}

#[cfg(feature = "curve")]
/// Retain a completed XY projection and receipt without revalidating it.
pub(in crate::solid) fn retain_flatten(
    mesh: &TriangleMesh,
    context: GeometryContext,
    region: &GeometryOutcome<hypercurve::CurveRegion2>,
) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.flattenings, state.policy.flattenings, || {
            CachedFlatten {
                source: MeshStorage::new(mesh),
                context,
                region: region.clone(),
            }
        });
    });
}

#[cfg(feature = "curve")]
/// Clone the first slice matching source identity/evidence, structural height and context.
pub(in crate::solid) fn retained_slice(
    mesh: &TriangleMesh,
    z: &Real,
    context: GeometryContext,
) -> Option<GeometryOutcome<crate::solid::SliceResult>> {
    STATE.with_borrow(|state| {
        state.slices.iter().find_map(|entry| {
            (entry.source.matches(mesh) && &entry.z == z && entry.context == context)
                .then(|| entry.result.clone())
        })
    })
}

#[cfg(feature = "curve")]
/// Retain a completed slice and receipt under its source, height and context key.
pub(in crate::solid) fn retain_slice(
    mesh: &TriangleMesh,
    z: Real,
    context: GeometryContext,
    result: &GeometryOutcome<crate::solid::SliceResult>,
) {
    STATE.with_borrow_mut(|state| {
        retain(&mut state.slices, state.policy.slices, || CachedSlice {
            source: MeshStorage::new(mesh),
            z,
            context,
            result: result.clone(),
        });
    });
}
