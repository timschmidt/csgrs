//! Exact mesh-boolean handoff records for path-domain swept volumes.
//!
//! `hyperpath` owns CAM and EDA path objects, but accepted mesh topology belongs
//! to `hypermesh`. This module is the narrow handoff layer between them: a
//! retained rectangular path/CAM footprint is extruded into an exact triangular
//! prism, `hypermesh` runs exact boolean preflight and materialization, and the
//! result is replayed against the retained source prisms before it is exposed.
//! That object/predicate boundary follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): a path-domain
//! candidate may propose a swept volume, while exact mesh topology is accepted
//! only by a proof-producing geometry owner.
//!
//! The current source object is deliberately bounded to axis-aligned
//! rectangular prisms because it is the common CAM pocket/support and PCB
//! keepout fixture already represented by this crate. The boolean itself is
//! not reimplemented here. `hypermesh` remains the acceptance boundary, which
//! matches regularized-solid modeling practice in Requicha, "Representations
//! for Rigid Solids: Theory, Methods, and Systems," *ACM Computing Surveys*
//! 12.4 (1980).

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hypermesh::exact::{
    ExactBooleanOperation, ExactBooleanPreflight, ExactBooleanResult, ExactBoundaryBooleanPolicy,
    ExactMesh, ValidationPolicy, boolean_exact_with_boundary_policy, preflight_boolean_exact,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::cam::{PocketPlanError, RectangularPocket};
use crate::provenance::PathProvenance;

/// Exact rectangular prism swept from a path-domain rectangular footprint.
///
/// The footprint remains a `hyperpath` object; the 3D mesh is a derived
/// handoff. Bounds are required to have positive X, Y, and Z extent so the
/// derived mesh is a closed solid rather than a degenerate surface. Degenerate
/// surface/curve cases should use an explicit lower-dimensional artifact, not
/// a mesh-boolean shortcut.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularPrism {
    footprint: RectangularPocket,
    z_min: Real,
    z_max: Real,
    provenance: PathProvenance,
    exact: RealExactSetFacts,
}

/// Named path-domain mesh boolean operation.
///
/// The variants intentionally mirror `hypermesh` named operations without
/// exposing `SelectedRegions`; path-domain callers request regularized
/// union/intersection/difference over retained swept-volume objects, while the
/// mesh owner decides which exact shortcut or winding materializer can prove
/// the result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PathMeshBooleanOperation {
    /// Regularized solid union.
    Union,
    /// Regularized solid intersection.
    Intersection,
    /// Regularized solid difference.
    Difference,
}

/// Errors from the path-to-mesh boolean handoff.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PathMeshBooleanError {
    /// At least one boolean step and two effective sources are required.
    NotEnoughSources,
    /// A swept path source was not certified axis-aligned.
    NonAxisAlignedSweep,
    /// A swept path source did not have certified positive width.
    NonPositiveSweepWidth,
    /// A PCB layer model did not have certified positive pitch.
    NonPositiveLayerPitch,
    /// A PCB layer model did not have certified positive copper thickness.
    NonPositiveCopperThickness,
    /// PCB copper thickness exceeded the layer pitch.
    CopperThicknessExceedsPitch,
    /// A rectangular PCB pad did not have certified positive width and height.
    NonPositivePadExtent,
    /// A polygonal source had fewer than three retained vertices.
    TooFewPolygonVertices,
    /// A polygonal source had zero exact area.
    DegeneratePolygon,
    /// A polygonal source had undecidable orientation or convexity.
    UnknownPolygonOrientation,
    /// A polygonal source was not strictly convex.
    NonConvexPolygon,
    /// A PCB copper boolean program mixed multiple nets.
    MixedPcbNets,
    /// A PCB copper boolean program mixed multiple layers.
    MixedPcbLayers,
    /// Footprint bounds did not have strictly positive area.
    DegenerateFootprint,
    /// Z bounds were unordered, equal, or undecidable.
    DegenerateHeight,
    /// A negative or unsupported scalar domain was supplied.
    InvalidScalar,
    /// `hypermesh` rejected the derived exact prism mesh.
    MeshConstruction(String),
    /// `hypermesh` could not produce exact boolean preflight evidence.
    Preflight(String),
    /// `hypermesh` could not materialize the exact boolean.
    Boolean(String),
    /// Replaying the retained boolean report against the source prisms failed.
    Replay(String),
}

/// Source-bound exact mesh-boolean handoff report.
///
/// The report retains both path-domain prisms and the exact mesh report that
/// accepted topology. [`Self::validate_replay`] reconstructs both prism meshes
/// from the retained path objects, reruns `hypermesh` preflight/materialization,
/// and validates the retained result against those exact sources. This is the
/// local form of Yap's EGC discipline: cached mesh topology is not proof unless
/// it still replays from the current source objects.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanReport {
    /// Left/source prism.
    pub left: RectangularPrism,
    /// Right/cutter prism.
    pub right: RectangularPrism,
    /// Requested regularized boolean operation.
    pub operation: PathMeshBooleanOperation,
    /// Explicit boundary-contact projection policy passed to `hypermesh`.
    pub boundary_policy: ExactBoundaryBooleanPolicy,
    /// Exact preflight report from `hypermesh`.
    pub preflight: ExactBooleanPreflight,
    /// Materialized exact boolean result from `hypermesh`.
    pub result: ExactBooleanResult,
}

/// One certified step in a multi-prism boolean chain.
///
/// The left operand is implicit: step zero uses the first source prism, and
/// every later step uses the accepted mesh produced by the previous step. This
/// makes the accumulator explicit during replay without requiring `hyperpath`
/// to reinterpret an intermediate mesh as a path primitive. The arrangement is
/// a direct application of Yap's exact object model: each topology-changing
/// step carries the exact `hypermesh` report that accepted it.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanStep {
    /// Zero-based step index.
    pub index: usize,
    /// Retained right-hand prism consumed by this step.
    pub right: RectangularPrism,
    /// Exact preflight report for this accumulator/right pair.
    pub preflight: ExactBooleanPreflight,
    /// Accepted exact boolean result for this step.
    pub result: ExactBooleanResult,
}

/// Source-bound exact boolean chain over several path-domain prisms.
///
/// This is the CAM/rest-material and multi-obstacle companion to
/// [`PathMeshBooleanReport`]. It never assumes that a later accumulator is a
/// rectangular prism. Instead, every intermediate mesh remains an accepted
/// `hypermesh` object, and [`Self::validate_replay`] rebuilds the whole chain
/// from the retained path prisms before accepting the final mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanChainReport {
    /// Retained path-domain prism sources. The first source seeds the chain.
    pub sources: Vec<RectangularPrism>,
    /// Named operation applied left-associatively through the chain.
    pub operation: PathMeshBooleanOperation,
    /// Boundary-contact projection policy passed to every `hypermesh` step.
    pub boundary_policy: ExactBoundaryBooleanPolicy,
    /// Per-step exact preflight and accepted mesh result.
    pub steps: Vec<PathMeshBooleanStep>,
}

impl RectangularPrism {
    /// Construct a positive-height rectangular prism from a retained footprint.
    pub fn new(
        footprint: RectangularPocket,
        z_min: Real,
        z_max: Real,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        if compare_reals_with_policy(&footprint.min().x, &footprint.max().x, policy).value()
            != Some(Ordering::Less)
            || compare_reals_with_policy(&footprint.min().y, &footprint.max().y, policy).value()
                != Some(Ordering::Less)
        {
            return Err(PathMeshBooleanError::DegenerateFootprint);
        }
        if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
            return Err(PathMeshBooleanError::DegenerateHeight);
        }
        let provenance = footprint.provenance();
        let exact = Real::exact_set_facts([
            &footprint.min().x,
            &footprint.min().y,
            &footprint.max().x,
            &footprint.max().y,
            &z_min,
            &z_max,
        ]);
        Ok(Self {
            footprint,
            z_min,
            z_max,
            provenance,
            exact,
        })
    }

    /// Return the retained 2D footprint.
    pub const fn footprint(&self) -> &RectangularPocket {
        &self.footprint
    }

    /// Return exact minimum Z.
    pub const fn z_min(&self) -> &Real {
        &self.z_min
    }

    /// Return exact maximum Z.
    pub const fn z_max(&self) -> &Real {
        &self.z_max
    }

    /// Return source provenance inherited from the footprint.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return exact-set facts for all six prism bounds.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Derive the exact `hypermesh` solid used for boolean certification.
    ///
    /// The triangle winding is the same outward box winding used by
    /// `hypermesh`'s exact AABB fixtures. Coordinates are cloned as `Real`
    /// values rather than exported through primitive floats, keeping the
    /// handoff on the exact-object side of Yap's EGC boundary.
    pub fn to_exact_mesh(&self) -> Result<ExactMesh, PathMeshBooleanError> {
        let min = self.footprint.min();
        let max = self.footprint.max();
        let positions = vec![
            min.x.clone(),
            min.y.clone(),
            self.z_min.clone(),
            max.x.clone(),
            min.y.clone(),
            self.z_min.clone(),
            max.x.clone(),
            max.y.clone(),
            self.z_min.clone(),
            min.x.clone(),
            max.y.clone(),
            self.z_min.clone(),
            min.x.clone(),
            min.y.clone(),
            self.z_max.clone(),
            max.x.clone(),
            min.y.clone(),
            self.z_max.clone(),
            max.x.clone(),
            max.y.clone(),
            self.z_max.clone(),
            min.x.clone(),
            max.y.clone(),
            self.z_max.clone(),
        ];
        let indices = box_triangle_indices();
        ExactMesh::from_real_triangles(&positions, &indices)
            .map_err(|error| PathMeshBooleanError::MeshConstruction(format!("{error:?}")))
    }
}

impl PathMeshBooleanOperation {
    /// Convert to the owning mesh crate's named boolean operation.
    pub const fn to_hypermesh(self) -> ExactBooleanOperation {
        match self {
            Self::Union => ExactBooleanOperation::Union,
            Self::Intersection => ExactBooleanOperation::Intersection,
            Self::Difference => ExactBooleanOperation::Difference,
        }
    }
}

impl PathMeshBooleanReport {
    /// Recompute and validate this report from retained path-domain sources.
    pub fn validate_replay(&self) -> Result<(), PathMeshBooleanError> {
        let left_mesh = self.left.to_exact_mesh()?;
        let right_mesh = self.right.to_exact_mesh()?;
        let operation = self.operation.to_hypermesh();
        let preflight = preflight_boolean_exact(&left_mesh, &right_mesh, operation)
            .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
        preflight
            .validate_against_sources(&left_mesh, &right_mesh)
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        if preflight != self.preflight {
            return Err(PathMeshBooleanError::Replay(
                "retained preflight no longer matches source prisms".into(),
            ));
        }
        self.result
            .validate_operation_against_sources(
                &left_mesh,
                &right_mesh,
                operation,
                ValidationPolicy::CLOSED,
                self.boundary_policy,
            )
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        Ok(())
    }

    /// Return the accepted exact output mesh.
    pub const fn mesh(&self) -> &ExactMesh {
        &self.result.mesh
    }
}

impl PathMeshBooleanChainReport {
    /// Rebuild and validate every retained boolean step from source prisms.
    ///
    /// Replay is left-associative: `(((source[0] op source[1]) op source[2])
    /// ...)`. This order is a report contract, not an algebraic claim about
    /// associativity. Regularized union/intersection are mathematically
    /// associative, but mesh materializers may choose different certified
    /// shortcuts; retaining the order keeps the proof boundary auditable, as
    /// Yap requires in "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997).
    pub fn validate_replay(&self) -> Result<(), PathMeshBooleanError> {
        if self.sources.len() < 2 || self.steps.len() != self.sources.len() - 1 {
            return Err(PathMeshBooleanError::NotEnoughSources);
        }
        let operation = self.operation.to_hypermesh();
        let mut accumulator = self.sources[0].to_exact_mesh()?;
        for (expected_index, step) in self.steps.iter().enumerate() {
            if step.index != expected_index || step.right != self.sources[expected_index + 1] {
                return Err(PathMeshBooleanError::Replay(
                    "retained boolean-chain step no longer matches source order".into(),
                ));
            }
            let right_mesh = step.right.to_exact_mesh()?;
            let preflight = preflight_boolean_exact(&accumulator, &right_mesh, operation)
                .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
            preflight
                .validate_against_sources(&accumulator, &right_mesh)
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            if preflight != step.preflight {
                return Err(PathMeshBooleanError::Replay(
                    "retained boolean-chain preflight no longer matches replay".into(),
                ));
            }
            step.result
                .validate_operation_against_sources(
                    &accumulator,
                    &right_mesh,
                    operation,
                    ValidationPolicy::CLOSED,
                    self.boundary_policy,
                )
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            accumulator = step.result.mesh.clone();
        }
        Ok(())
    }

    /// Return the final accepted exact output mesh.
    pub fn mesh(&self) -> Option<&ExactMesh> {
        self.steps.last().map(|step| &step.result.mesh)
    }
}

/// Run an exact mesh boolean over two retained rectangular path prisms.
pub fn boolean_rectangular_prisms(
    left: RectangularPrism,
    right: RectangularPrism,
    operation: PathMeshBooleanOperation,
) -> Result<PathMeshBooleanReport, PathMeshBooleanError> {
    boolean_rectangular_prisms_with_boundary_policy(
        left,
        right,
        operation,
        ExactBoundaryBooleanPolicy::Reject,
    )
}

/// Run an exact mesh boolean over two retained rectangular path prisms.
///
/// Boundary-only contacts are passed through as an explicit policy because a
/// triangle mesh cannot represent lower-dimensional intersections. That is
/// the same projection boundary documented by `hypermesh`; `hyperpath` only
/// records the choice beside the path-domain source objects.
pub fn boolean_rectangular_prisms_with_boundary_policy(
    left: RectangularPrism,
    right: RectangularPrism,
    operation: PathMeshBooleanOperation,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<PathMeshBooleanReport, PathMeshBooleanError> {
    let left_mesh = left.to_exact_mesh()?;
    let right_mesh = right.to_exact_mesh()?;
    let mesh_operation = operation.to_hypermesh();
    let preflight = preflight_boolean_exact(&left_mesh, &right_mesh, mesh_operation)
        .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
    preflight
        .validate_against_sources(&left_mesh, &right_mesh)
        .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
    let result = boolean_exact_with_boundary_policy(
        &left_mesh,
        &right_mesh,
        mesh_operation,
        ValidationPolicy::CLOSED,
        boundary_policy,
    )
    .map_err(|error| PathMeshBooleanError::Boolean(format!("{error:?}")))?;
    result
        .validate_operation_against_sources(
            &left_mesh,
            &right_mesh,
            mesh_operation,
            ValidationPolicy::CLOSED,
            boundary_policy,
        )
        .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
    Ok(PathMeshBooleanReport {
        left,
        right,
        operation,
        boundary_policy,
        preflight,
        result,
    })
}

/// Run a left-associative exact boolean chain over retained rectangular prisms.
///
/// This is intentionally stricter than a generic mesh batch API. Every input is
/// a path-domain prism with exact source coordinates, but after the first step
/// the left operand is the previously accepted `hypermesh` result. That allows
/// multi-cutter difference and multi-obstacle union/intersection without
/// inventing path-local mesh topology. The boolean materialization idea follows
/// the regularized set-operation model summarized by Requicha,
/// "Representations for Rigid Solids: Theory, Methods, and Systems,"
/// *ACM Computing Surveys* 12.4 (1980), while the source replay follows Yap's
/// exact-geometric-computation boundary.
pub fn boolean_rectangular_prism_chain(
    sources: Vec<RectangularPrism>,
    operation: PathMeshBooleanOperation,
) -> Result<PathMeshBooleanChainReport, PathMeshBooleanError> {
    boolean_rectangular_prism_chain_with_boundary_policy(
        sources,
        operation,
        ExactBoundaryBooleanPolicy::Reject,
    )
}

/// Run a left-associative exact boolean chain with explicit boundary policy.
pub fn boolean_rectangular_prism_chain_with_boundary_policy(
    sources: Vec<RectangularPrism>,
    operation: PathMeshBooleanOperation,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<PathMeshBooleanChainReport, PathMeshBooleanError> {
    if sources.len() < 2 {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    let mesh_operation = operation.to_hypermesh();
    let mut accumulator = sources[0].to_exact_mesh()?;
    let mut steps = Vec::with_capacity(sources.len() - 1);
    for (index, right) in sources.iter().enumerate().skip(1) {
        let right_mesh = right.to_exact_mesh()?;
        let preflight = preflight_boolean_exact(&accumulator, &right_mesh, mesh_operation)
            .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
        preflight
            .validate_against_sources(&accumulator, &right_mesh)
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        let result = boolean_exact_with_boundary_policy(
            &accumulator,
            &right_mesh,
            mesh_operation,
            ValidationPolicy::CLOSED,
            boundary_policy,
        )
        .map_err(|error| PathMeshBooleanError::Boolean(format!("{error:?}")))?;
        result
            .validate_operation_against_sources(
                &accumulator,
                &right_mesh,
                mesh_operation,
                ValidationPolicy::CLOSED,
                boundary_policy,
            )
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        accumulator = result.mesh.clone();
        steps.push(PathMeshBooleanStep {
            index: index - 1,
            right: right.clone(),
            preflight,
            result,
        });
    }

    Ok(PathMeshBooleanChainReport {
        sources,
        operation,
        boundary_policy,
        steps,
    })
}

/// Convenience constructor for a prism from exact integer bounds.
pub fn rectangular_prism_from_i64_bounds(
    min: [i64; 3],
    max: [i64; 3],
    policy: PredicatePolicy,
) -> Result<RectangularPrism, PathMeshBooleanError> {
    let footprint = RectangularPocket::new(
        Point2::new(Real::from(min[0]), Real::from(min[1])),
        Point2::new(Real::from(max[0]), Real::from(max[1])),
    )
    .map_err(|error| match error {
        PocketPlanError::UnorderedBounds => PathMeshBooleanError::DegenerateFootprint,
        _ => PathMeshBooleanError::InvalidScalar,
    })?;
    RectangularPrism::new(footprint, Real::from(min[2]), Real::from(max[2]), policy)
}

const fn box_triangle_indices() -> [usize; 36] {
    [
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6,
        3, 0, 4, 3, 4, 7,
    ]
}
