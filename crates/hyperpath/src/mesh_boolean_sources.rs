//! Retained path-domain sources for exact mesh booleans.
//!
//! This module extends the rectangular-prism handoff with path-native swept
//! segment slabs. `hyperpath` still does not own mesh topology: every retained
//! source can derive an exact `hypermesh` mesh, and boolean reports are accepted
//! only when that derivation replays. This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): exact
//! objects and predicates remain the authority, while cached topology is only a
//! replayable certificate.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hypermesh::exact::{
    ExactBooleanPreflight, ExactBooleanResult, ExactBoundaryBooleanPolicy, ExactMesh,
    ValidationPolicy, boolean_exact_with_boundary_policy, preflight_boolean_exact,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::cam::{PocketPlanError, RectangularPocket};
use crate::mesh_boolean::{PathMeshBooleanError, PathMeshBooleanOperation, RectangularPrism};
use crate::mesh_boolean_polygon::{ConvexPolygonPrism, OrthogonalPolygonPrism};
use crate::provenance::PathProvenance;
use crate::segment::Axis;
use crate::swept::SweptLineSegment;

/// Exact slab swept by a certified axis-aligned line segment and positive width.
///
/// The source remains a [`SweptLineSegment`], not a rectangle pretending to be a
/// path. For now the slab is intentionally limited to axis-aligned segments,
/// because those can be lowered to exact rectangular footprints without square
/// roots or approximate normals. General segment and curve sweeps should carry
/// their own offset/arrangement evidence before becoming mesh-boolean sources.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisAlignedSweptSegmentPrism {
    swept: SweptLineSegment,
    z_min: Real,
    z_max: Real,
    axis: Axis,
    prism: RectangularPrism,
    exact: RealExactSetFacts,
}

/// A retained hyperpath source that can derive an exact mesh-boolean operand.
///
/// The variants are source-domain objects, not mesh aliases. This keeps
/// heterogeneous chains auditable: replay rebuilds each mesh from its path/CAM
/// source and then asks `hypermesh` to validate the boolean step.
#[derive(Clone, Debug, PartialEq)]
pub enum PathMeshBooleanSource {
    /// Rectangular CAM/support/keepout prism.
    RectangularPrism(RectangularPrism),
    /// Strictly convex polygonal prism.
    ConvexPolygonPrism(ConvexPolygonPrism),
    /// Simple orthogonal polygonal prism.
    OrthogonalPolygonPrism(OrthogonalPolygonPrism),
    /// Axis-aligned swept trace/tool slab.
    AxisAlignedSweptSegmentPrism(AxisAlignedSweptSegmentPrism),
}

/// One certified step in a heterogeneous source boolean chain.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanSourceStep {
    /// Zero-based step index.
    pub index: usize,
    /// Retained right-hand source consumed by this step.
    pub right: PathMeshBooleanSource,
    /// Exact preflight report for this accumulator/right pair.
    pub preflight: ExactBooleanPreflight,
    /// Accepted exact boolean result for this step.
    pub result: ExactBooleanResult,
}

/// Source-bound exact boolean chain over heterogeneous path-domain sources.
///
/// Replay is left-associative and intentionally order-preserving. Requicha,
/// "Representations for Rigid Solids: Theory, Methods, and Systems,"
/// *ACM Computing Surveys* 12.4 (1980), supplies the regularized solid
/// operation model, while Yap's exact-geometric-computation model explains why
/// this report stores sources plus replayable evidence instead of exposing
/// unproven cached mesh topology as path output.
#[derive(Clone, Debug, PartialEq)]
pub struct PathMeshBooleanSourceChainReport {
    /// Retained path/CAM source objects. The first source seeds the chain.
    pub sources: Vec<PathMeshBooleanSource>,
    /// Named operation applied left-associatively through the chain.
    pub operation: PathMeshBooleanOperation,
    /// Boundary-contact projection policy passed to every `hypermesh` step.
    pub boundary_policy: ExactBoundaryBooleanPolicy,
    /// Per-step exact preflight and accepted mesh result.
    pub steps: Vec<PathMeshBooleanSourceStep>,
}

impl AxisAlignedSweptSegmentPrism {
    /// Build an exact slab from a certified axis-aligned swept line segment.
    ///
    /// The derived footprint is the closed rectangular butt-cap sweep of the
    /// centerline by half the retained width. That construction is exact only
    /// for axis-aligned segments in this API: the normal offset is an addition
    /// or subtraction of `width / 2`, so no approximate normalization crosses
    /// Yap's object boundary.
    pub fn new(
        swept: SweptLineSegment,
        z_min: Real,
        z_max: Real,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        let axis = swept
            .centerline()
            .facts()
            .axis_aligned
            .ok_or(PathMeshBooleanError::NonAxisAlignedSweep)?;
        if compare_reals_with_policy(swept.width(), &Real::zero(), policy).value()
            != Some(Ordering::Greater)
        {
            return Err(PathMeshBooleanError::NonPositiveSweepWidth);
        }
        if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
            return Err(PathMeshBooleanError::DegenerateHeight);
        }
        let length = swept
            .centerline()
            .axis_length(policy)
            .ok_or(PathMeshBooleanError::DegenerateFootprint)?;
        if compare_reals_with_policy(&length, &Real::zero(), policy).value()
            != Some(Ordering::Greater)
        {
            return Err(PathMeshBooleanError::DegenerateFootprint);
        }

        let half_width = (swept.width().clone() / Real::from(2))
            .map_err(|_| PathMeshBooleanError::InvalidScalar)?;
        let bounds_min = swept.centerline().bounds_min();
        let bounds_max = swept.centerline().bounds_max();
        let (min, max) = match axis {
            Axis::X => (
                Point2::new(
                    bounds_min.x.clone(),
                    bounds_min.y.clone() - half_width.clone(),
                ),
                Point2::new(bounds_max.x.clone(), bounds_max.y.clone() + half_width),
            ),
            Axis::Y => (
                Point2::new(
                    bounds_min.x.clone() - half_width.clone(),
                    bounds_min.y.clone(),
                ),
                Point2::new(bounds_max.x.clone() + half_width, bounds_max.y.clone()),
            ),
        };
        let footprint =
            RectangularPocket::with_provenance(min, max, swept.provenance()).map_err(|error| {
                match error {
                    PocketPlanError::UnorderedBounds => PathMeshBooleanError::DegenerateFootprint,
                    _ => PathMeshBooleanError::InvalidScalar,
                }
            })?;
        let exact = Real::exact_set_facts([
            &swept.centerline().start().x,
            &swept.centerline().start().y,
            &swept.centerline().end().x,
            &swept.centerline().end().y,
            swept.width(),
            &z_min,
            &z_max,
        ]);
        let prism = RectangularPrism::new(footprint, z_min.clone(), z_max.clone(), policy)?;
        Ok(Self {
            swept,
            z_min,
            z_max,
            axis,
            prism,
            exact,
        })
    }

    /// Return the retained swept path source.
    pub const fn swept(&self) -> &SweptLineSegment {
        &self.swept
    }

    /// Return the certified centerline axis.
    pub const fn axis(&self) -> Axis {
        self.axis
    }

    /// Return exact minimum Z.
    pub const fn z_min(&self) -> &Real {
        &self.z_min
    }

    /// Return exact maximum Z.
    pub const fn z_max(&self) -> &Real {
        &self.z_max
    }

    /// Return the exact rectangular prism derived from the swept segment.
    pub const fn derived_prism(&self) -> &RectangularPrism {
        &self.prism
    }

    /// Return source provenance inherited from the swept segment.
    pub fn provenance(&self) -> PathProvenance {
        self.swept.provenance()
    }

    /// Return exact-set facts for the swept source and Z bounds.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Derive the exact `hypermesh` solid used for boolean certification.
    pub fn to_exact_mesh(&self) -> Result<ExactMesh, PathMeshBooleanError> {
        self.prism.to_exact_mesh()
    }
}

impl From<RectangularPrism> for PathMeshBooleanSource {
    fn from(value: RectangularPrism) -> Self {
        Self::RectangularPrism(value)
    }
}

impl From<AxisAlignedSweptSegmentPrism> for PathMeshBooleanSource {
    fn from(value: AxisAlignedSweptSegmentPrism) -> Self {
        Self::AxisAlignedSweptSegmentPrism(value)
    }
}

impl From<ConvexPolygonPrism> for PathMeshBooleanSource {
    fn from(value: ConvexPolygonPrism) -> Self {
        Self::ConvexPolygonPrism(value)
    }
}

impl From<OrthogonalPolygonPrism> for PathMeshBooleanSource {
    fn from(value: OrthogonalPolygonPrism) -> Self {
        Self::OrthogonalPolygonPrism(value)
    }
}

impl PathMeshBooleanSource {
    /// Derive the exact `hypermesh` operand for this retained source.
    pub fn to_exact_mesh(&self) -> Result<ExactMesh, PathMeshBooleanError> {
        match self {
            Self::RectangularPrism(source) => source.to_exact_mesh(),
            Self::ConvexPolygonPrism(source) => source.to_exact_mesh(),
            Self::OrthogonalPolygonPrism(source) => source.to_exact_mesh(),
            Self::AxisAlignedSweptSegmentPrism(source) => source.to_exact_mesh(),
        }
    }
}

impl PathMeshBooleanSourceChainReport {
    /// Rebuild and validate every retained boolean step from heterogeneous sources.
    pub fn validate_replay(&self) -> Result<(), PathMeshBooleanError> {
        if self.sources.len() < 2 || self.steps.len() != self.sources.len() - 1 {
            return Err(PathMeshBooleanError::NotEnoughSources);
        }
        let operation = self.operation.to_hypermesh();
        let mut accumulator = self.sources[0].to_exact_mesh()?;
        for (expected_index, step) in self.steps.iter().enumerate() {
            if step.index != expected_index || step.right != self.sources[expected_index + 1] {
                return Err(PathMeshBooleanError::Replay(
                    "retained heterogeneous boolean-chain step no longer matches source order"
                        .into(),
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
                    "retained heterogeneous boolean-chain preflight no longer matches replay"
                        .into(),
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

/// Run a left-associative exact boolean chain over heterogeneous retained sources.
pub fn boolean_path_mesh_sources(
    sources: Vec<PathMeshBooleanSource>,
    operation: PathMeshBooleanOperation,
) -> Result<PathMeshBooleanSourceChainReport, PathMeshBooleanError> {
    boolean_path_mesh_sources_with_boundary_policy(
        sources,
        operation,
        ExactBoundaryBooleanPolicy::Reject,
    )
}

/// Run a heterogeneous exact boolean chain with explicit boundary policy.
pub fn boolean_path_mesh_sources_with_boundary_policy(
    sources: Vec<PathMeshBooleanSource>,
    operation: PathMeshBooleanOperation,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<PathMeshBooleanSourceChainReport, PathMeshBooleanError> {
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
        steps.push(PathMeshBooleanSourceStep {
            index: index - 1,
            right: right.clone(),
            preflight,
            result,
        });
    }

    Ok(PathMeshBooleanSourceChainReport {
        sources,
        operation,
        boundary_policy,
        steps,
    })
}
