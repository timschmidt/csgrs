//! CAM rest-material mesh-boolean programs over retained path sources.
//!
//! Subtractive toolpaths are not just arbitrary mesh differences: a stock
//! volume, cutter paths, and pocket cutters are CAM-domain objects that need to
//! remain replayable. This module builds exact rest-material programs from
//! retained rectangular stock and exact cutter sources, then delegates topology
//! acceptance to `hypermesh` through [`crate::mesh_boolean_program`].
//!
//! The boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): stock and cutter objects are kept as
//! exact sources, and the accepted mesh topology is trusted only while it can be
//! replayed from those sources. The regularized difference semantics follow
//! Requicha, "Representations for Rigid Solids: Theory, Methods, and Systems,"
//! *ACM Computing Surveys* 12.4 (1980), while the staged stock/cutter split is
//! the same evidence boundary used by contour-parallel CAM pipelines before
//! gouge, linking, and engagement checks are accepted.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealExactSetFacts};

use crate::cam::{
    PocketPlanError, RectangularInfillGraph, RectangularPocket, RectangularSupportPlan,
};
use crate::mesh_boolean::{PathMeshBooleanError, PathMeshBooleanOperation, RectangularPrism};
use crate::mesh_boolean_holes::validate_strict_orthogonal_holes;
use crate::mesh_boolean_polygon::{ConvexPolygonPrism, OrthogonalPolygonPrism, SimplePolygonPrism};
use crate::mesh_boolean_program::{
    PathMeshBooleanProgramReport, PathMeshBooleanProgramStep, boolean_path_mesh_program,
};
use crate::mesh_boolean_sources::{AxisAlignedSweptSegmentPrism, PathMeshBooleanSource};
use crate::provenance::PathProvenance;
use crate::swept::SweptLineSegment;

/// Retained subtractive CAM cutter source.
///
/// Rectangular pocket cutters represent exact box removals. Axis-aligned sweep
/// cutters represent retained straight tool-center passes with exact width.
/// Non-axis-aligned and curved cutters remain future arrangement evidence
/// rather than approximate mesh sources.
#[derive(Clone, Debug, PartialEq)]
pub enum CamRestMaterialCutter {
    /// Exact rectangular pocket cutter footprint.
    RectangularPocket(RectangularPocket),
    /// Exact axis-aligned swept cutter path.
    AxisAlignedSweep(SweptLineSegment),
    /// Exact orthogonal pocket boundary with retained material islands.
    OrthogonalIslandPocket(CamOrthogonalIslandPocketCutter),
}

/// Retained orthogonal CAM pocket cutter with exact material islands.
///
/// This source models a common 2.5D rest-material operation: remove the outer
/// pocket region while preserving one or more island regions. It is deliberately
/// not flattened into a holed mesh. Instead, [`build_cam_rest_material_program`]
/// expands it into `stock - outer + island_0 + ...`, preserving each loop as a
/// replayable object. The set-operation semantics follow Requicha,
/// "Representations for Rigid Solids: Theory, Methods, and Systems,"
/// *ACM Computing Surveys* 12.4 (1980), and the retained-loop validation uses
/// the exact object/predicate boundary described by Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct CamOrthogonalIslandPocketCutter {
    outer: Vec<Point2>,
    islands: Vec<Vec<Point2>>,
    provenance: PathProvenance,
    exact: RealExactSetFacts,
}

/// Exact rest-material boolean program for rectangular stock.
///
/// The retained sources and coarse exact-set facts are part of the public
/// report because the mesh is an accepted certificate, not the source of truth.
/// That mirrors Yap's EGC separation between exact objects and derived
/// predicates near the point where we hand topology off to `hypermesh`.
#[derive(Clone, Debug, PartialEq)]
pub struct CamRestMaterialProgramReport {
    /// Retained rectangular stock footprint.
    pub stock: RectangularPocket,
    /// Exact lower stock/cutter Z face.
    pub z_min: Real,
    /// Exact upper stock/cutter Z face.
    pub z_max: Real,
    /// Retained cutter sources in program order.
    ///
    /// Simple cutters subtract one source each. Island pockets expand to one
    /// outer subtraction followed by island union steps during replay.
    pub cutters: Vec<CamRestMaterialCutter>,
    /// Exact-set facts for retained stock, cutter, and Z scalar inputs.
    pub exact: RealExactSetFacts,
    /// Accepted exact mesh-boolean program.
    pub program: PathMeshBooleanProgramReport,
}

/// Retained straight-edge clipping boundary for additive support footprints.
///
/// The boundary is a 2D CAM object, not a cached mesh. It is lowered into a
/// layer slab only when an exact support-clip program is replayed. This keeps
/// the support pipeline in the exact-object discipline advocated by Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), while the clipping operation itself remains a Requicha regularized
/// solid intersection.
#[derive(Clone, Debug, PartialEq)]
pub enum CamSupportClipBoundary {
    /// Strictly convex straight-edge clip boundary.
    Convex {
        /// Retained boundary vertices.
        vertices: Vec<Point2>,
        /// Source provenance shared by the retained loop.
        provenance: PathProvenance,
        /// Exact-set facts for retained coordinates.
        exact: RealExactSetFacts,
    },
    /// Simple orthogonal clip boundary.
    Orthogonal {
        /// Retained boundary vertices.
        vertices: Vec<Point2>,
        /// Source provenance shared by the retained loop.
        provenance: PathProvenance,
        /// Exact-set facts for retained coordinates.
        exact: RealExactSetFacts,
    },
    /// Simple hole-free straight-edge clip boundary.
    Simple {
        /// Retained boundary vertices.
        vertices: Vec<Point2>,
        /// Source provenance shared by the retained loop.
        provenance: PathProvenance,
        /// Exact-set facts for retained coordinates.
        exact: RealExactSetFacts,
    },
}

/// Exact additive support-footprint clipping program.
///
/// A rectangular support plan is retained as the source request, then its
/// expanded footprint is intersected with a retained straight-edge boundary.
/// This is the bounded support counterpart to rest-material programs: topology
/// belongs to `hypermesh`, and this report remains valid only while replay can
/// regenerate the same exact boolean evidence from the retained support plan.
#[derive(Clone, Debug, PartialEq)]
pub struct CamSupportClipProgramReport {
    /// Retained support footprint request.
    pub support: RectangularSupportPlan,
    /// Exact lower support slab face.
    pub z_min: Real,
    /// Exact upper support slab face.
    pub z_max: Real,
    /// Retained clipping boundary.
    pub boundary: CamSupportClipBoundary,
    /// Accepted exact support/boundary intersection program.
    pub program: PathMeshBooleanProgramReport,
}

/// Exact additive infill clipping program.
///
/// The retained serpentine graph supplies deposition bead centerlines and
/// connector centerlines. Each centerline is swept by the exact bead width,
/// unioned in graph order, and then intersected with a retained straight-edge
/// boundary. This mirrors Zhao et al., "Continuous toolpath planning in a
/// graphical framework for sparse infill additive manufacturing," at the graph
/// generation layer, while applying Yap's exact-geometric-computation rule:
/// graph topology may propose material paths, but accepted swept topology must
/// replay through exact source objects and `hypermesh` evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct CamInfillClipProgramReport {
    /// Retained infill graph.
    pub graph: RectangularInfillGraph,
    /// Exact lower infill slab face.
    pub z_min: Real,
    /// Exact upper infill slab face.
    pub z_max: Real,
    /// Retained clipping boundary.
    pub boundary: CamSupportClipBoundary,
    /// Exact-set facts for retained graph, boundary, bead width, and Z inputs.
    pub exact: RealExactSetFacts,
    /// Accepted exact infill union and boundary-intersection program.
    pub program: PathMeshBooleanProgramReport,
}

impl CamRestMaterialCutter {
    /// Lower a single-operation cutter into a generic path mesh-boolean source.
    ///
    /// Island pockets deliberately do not implement this one-source lowering:
    /// their exact meaning is a mixed `Difference` followed by one or more
    /// `Union` steps, so callers must route them through
    /// [`build_cam_rest_material_program`] to preserve retained islands.
    pub fn to_path_source(
        &self,
        z_min: Real,
        z_max: Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        match self {
            Self::RectangularPocket(pocket) => {
                let prism = RectangularPrism::new(pocket.clone(), z_min, z_max, policy)?;
                Ok(prism.into())
            }
            Self::AxisAlignedSweep(swept) => {
                Ok(AxisAlignedSweptSegmentPrism::new(swept.clone(), z_min, z_max, policy)?.into())
            }
            Self::OrthogonalIslandPocket(_) => Err(PathMeshBooleanError::Replay(
                "orthogonal island pocket requires mixed-operation rest-material replay".into(),
            )),
        }
    }
}

impl CamOrthogonalIslandPocketCutter {
    /// Construct a retained orthogonal pocket with strict material islands.
    ///
    /// The outer and island loops must each be simple orthogonal polygons.
    /// Every island must be strictly inside the outer loop, and islands may not
    /// touch, overlap, or nest. That predicate stage follows the Shimrat/Haines
    /// crossing-test tradition in [`crate::mesh_boolean_holes`] and refuses
    /// ambiguous boundary contact before mesh booleans are requested.
    pub fn new(
        outer: Vec<Point2>,
        islands: Vec<Vec<Point2>>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::with_provenance(outer, islands, PathProvenance::native(), policy)
    }

    /// Construct a retained orthogonal pocket with explicit provenance.
    pub fn with_provenance(
        outer: Vec<Point2>,
        islands: Vec<Vec<Point2>>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        validate_orthogonal_loop_shape(&outer, provenance, policy)?;
        for island in &islands {
            validate_orthogonal_loop_shape(island, provenance, policy)?;
        }
        validate_strict_orthogonal_holes(&outer, &islands, policy)?;
        let exact = island_pocket_exact_facts(&outer, &islands);
        Ok(Self {
            outer,
            islands,
            provenance,
            exact,
        })
    }

    /// Return retained outer pocket loop.
    pub fn outer(&self) -> &[Point2] {
        &self.outer
    }

    /// Return retained material island loops.
    pub fn islands(&self) -> &[Vec<Point2>] {
        &self.islands
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return exact-set facts for all retained loop coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    fn outer_path_source(
        &self,
        z_min: Real,
        z_max: Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        orthogonal_loop_path_source(&self.outer, z_min, z_max, self.provenance, policy)
    }

    fn island_path_sources(
        &self,
        z_min: Real,
        z_max: Real,
        policy: PredicatePolicy,
    ) -> Result<Vec<PathMeshBooleanSource>, PathMeshBooleanError> {
        self.islands
            .iter()
            .map(|island| {
                orthogonal_loop_path_source(
                    island,
                    z_min.clone(),
                    z_max.clone(),
                    self.provenance,
                    policy,
                )
            })
            .collect()
    }
}

impl CamSupportClipBoundary {
    /// Construct a retained strictly convex support clipping boundary.
    pub fn convex(
        vertices: Vec<Point2>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::convex_with_provenance(vertices, PathProvenance::native(), policy)
    }

    /// Construct a retained strictly convex support clipping boundary with provenance.
    pub fn convex_with_provenance(
        vertices: Vec<Point2>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        ConvexPolygonPrism::new(
            vertices.clone(),
            Real::zero(),
            Real::one(),
            provenance,
            policy,
        )?;
        let exact = loop_exact_facts(&vertices);
        Ok(Self::Convex {
            vertices,
            provenance,
            exact,
        })
    }

    /// Construct a retained simple orthogonal support clipping boundary.
    pub fn orthogonal(
        vertices: Vec<Point2>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::orthogonal_with_provenance(vertices, PathProvenance::native(), policy)
    }

    /// Construct a retained simple orthogonal support clipping boundary with provenance.
    pub fn orthogonal_with_provenance(
        vertices: Vec<Point2>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        OrthogonalPolygonPrism::new(
            vertices.clone(),
            Real::zero(),
            Real::one(),
            provenance,
            policy,
        )?;
        let exact = loop_exact_facts(&vertices);
        Ok(Self::Orthogonal {
            vertices,
            provenance,
            exact,
        })
    }

    /// Construct a retained simple straight-edge support clipping boundary.
    ///
    /// This accepts nonconvex, non-orthogonal, hole-free loops after exact
    /// simplicity and triangulation checks. The derived prism uses the
    /// Meisters ear theorem during replay, but the retained loop remains the
    /// authoritative CAM object as required by Yap's EGC model.
    pub fn simple(
        vertices: Vec<Point2>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::simple_with_provenance(vertices, PathProvenance::native(), policy)
    }

    /// Construct a retained simple straight-edge clipping boundary with provenance.
    pub fn simple_with_provenance(
        vertices: Vec<Point2>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        SimplePolygonPrism::new(
            vertices.clone(),
            Real::zero(),
            Real::one(),
            provenance,
            policy,
        )?;
        let exact = loop_exact_facts(&vertices);
        Ok(Self::Simple {
            vertices,
            provenance,
            exact,
        })
    }

    /// Return retained boundary vertices.
    pub fn vertices(&self) -> &[Point2] {
        match self {
            Self::Convex { vertices, .. }
            | Self::Orthogonal { vertices, .. }
            | Self::Simple { vertices, .. } => vertices,
        }
    }

    /// Return retained source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        match self {
            Self::Convex { provenance, .. }
            | Self::Orthogonal { provenance, .. }
            | Self::Simple { provenance, .. } => *provenance,
        }
    }

    /// Return exact-set facts for retained boundary coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        match self {
            Self::Convex { exact, .. }
            | Self::Orthogonal { exact, .. }
            | Self::Simple { exact, .. } => exact,
        }
    }

    fn to_path_source(
        &self,
        z_min: Real,
        z_max: Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        match self {
            Self::Convex {
                vertices,
                provenance,
                ..
            } => Ok(
                ConvexPolygonPrism::new(vertices.clone(), z_min, z_max, *provenance, policy)?
                    .into(),
            ),
            Self::Orthogonal {
                vertices,
                provenance,
                ..
            } => orthogonal_loop_path_source(vertices, z_min, z_max, *provenance, policy),
            Self::Simple {
                vertices,
                provenance,
                ..
            } => Ok(
                SimplePolygonPrism::new(vertices.clone(), z_min, z_max, *provenance, policy)?
                    .into(),
            ),
        }
    }
}

impl CamRestMaterialProgramReport {
    /// Rebuild stock/cutter lowering and exact boolean evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed = build_cam_rest_material_program(
            self.stock.clone(),
            self.z_min.clone(),
            self.z_max.clone(),
            self.cutters.clone(),
            policy,
        )?;
        if replayed.program != self.program || replayed.exact != self.exact {
            return Err(PathMeshBooleanError::Replay(
                "retained CAM rest-material program no longer matches replay".into(),
            ));
        }
        self.program.validate_replay()
    }
}

impl CamSupportClipProgramReport {
    /// Rebuild support-footprint lowering, clip-boundary lowering, and exact evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed = build_cam_support_clip_program(
            self.support.clone(),
            self.z_min.clone(),
            self.z_max.clone(),
            self.boundary.clone(),
            policy,
        )?;
        if replayed.program != self.program {
            return Err(PathMeshBooleanError::Replay(
                "retained CAM support clip program no longer matches replay".into(),
            ));
        }
        self.program.validate_replay()
    }

    /// Return the final accepted exact clipped support mesh.
    pub fn mesh(&self) -> Option<&hypermesh::exact::ExactMesh> {
        self.program.mesh()
    }
}

impl CamInfillClipProgramReport {
    /// Rebuild infill sweep lowering, boundary lowering, and exact boolean evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed = build_cam_infill_clip_program(
            self.graph.clone(),
            self.z_min.clone(),
            self.z_max.clone(),
            self.boundary.clone(),
            policy,
        )?;
        if replayed.program != self.program || replayed.exact != self.exact {
            return Err(PathMeshBooleanError::Replay(
                "retained CAM infill clip program no longer matches replay".into(),
            ));
        }
        self.program.validate_replay()
    }

    /// Return the final accepted exact clipped infill mesh.
    pub fn mesh(&self) -> Option<&hypermesh::exact::ExactMesh> {
        self.program.mesh()
    }
}

/// Build an exact rest-material difference program for rectangular stock.
///
/// Simple cutters are subtracted from the current accumulator in the provided
/// order. Orthogonal island pockets expand to a subtraction of the outer loop
/// and a union for every retained island. The stock and all cutters share one
/// exact Z interval in this bounded API, which matches 2.5D pocket/rest
/// fixtures without pretending to solve general 5-axis swept volumes.
pub fn build_cam_rest_material_program(
    stock: RectangularPocket,
    z_min: Real,
    z_max: Real,
    cutters: Vec<CamRestMaterialCutter>,
    policy: PredicatePolicy,
) -> Result<CamRestMaterialProgramReport, PathMeshBooleanError> {
    if cutters.is_empty() {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
        return Err(PathMeshBooleanError::DegenerateHeight);
    }
    let stock_prism = RectangularPrism::new(stock.clone(), z_min.clone(), z_max.clone(), policy)?;
    let mut steps = Vec::new();
    for cutter in &cutters {
        match cutter {
            CamRestMaterialCutter::OrthogonalIslandPocket(source) => {
                let outer = source.outer_path_source(z_min.clone(), z_max.clone(), policy)?;
                steps.push(PathMeshBooleanProgramStep::new(
                    PathMeshBooleanOperation::Difference,
                    outer,
                ));
                for island in source.island_path_sources(z_min.clone(), z_max.clone(), policy)? {
                    steps.push(PathMeshBooleanProgramStep::new(
                        PathMeshBooleanOperation::Union,
                        island,
                    ));
                }
            }
            _ => {
                let right = cutter.to_path_source(z_min.clone(), z_max.clone(), policy)?;
                steps.push(PathMeshBooleanProgramStep::new(
                    PathMeshBooleanOperation::Difference,
                    right,
                ));
            }
        }
    }
    let program = boolean_path_mesh_program(stock_prism.into(), steps)?;
    let exact = rest_material_exact_facts(&stock, &z_min, &z_max, &cutters);
    Ok(CamRestMaterialProgramReport {
        stock,
        z_min,
        z_max,
        cutters,
        exact,
        program,
    })
}

/// Build an exact additive support clipping program.
///
/// The expanded rectangular support footprint seeds the accumulator. The
/// retained boundary is then intersected as a second source. This is a bounded
/// first step toward arbitrary polygon additive support clipping: no raster
/// mask or tolerance polygon is introduced, and replay must rebuild both
/// operands exactly before the accepted mesh can be reused.
pub fn build_cam_support_clip_program(
    support: RectangularSupportPlan,
    z_min: Real,
    z_max: Real,
    boundary: CamSupportClipBoundary,
    policy: PredicatePolicy,
) -> Result<CamSupportClipProgramReport, PathMeshBooleanError> {
    if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
        return Err(PathMeshBooleanError::DegenerateHeight);
    }
    let support_prism = RectangularPrism::new(
        support.footprint.clone(),
        z_min.clone(),
        z_max.clone(),
        policy,
    )?;
    let boundary_source = boundary.to_path_source(z_min.clone(), z_max.clone(), policy)?;
    let program = boolean_path_mesh_program(
        support_prism.into(),
        vec![PathMeshBooleanProgramStep::new(
            PathMeshBooleanOperation::Intersection,
            boundary_source,
        )],
    )?;
    Ok(CamSupportClipProgramReport {
        support,
        z_min,
        z_max,
        boundary,
        program,
    })
}

/// Build an exact additive infill clipping program.
///
/// Deposition centerlines are swept first; connector centerlines are swept
/// after them, preserving the retained graph order. The union accumulator is
/// then clipped by the retained boundary. Non-axis-aligned graph edges are
/// rejected by [`AxisAlignedSweptSegmentPrism`], so the function does not
/// smuggle approximate normals or curve offsets through the mesh handoff.
pub fn build_cam_infill_clip_program(
    graph: RectangularInfillGraph,
    z_min: Real,
    z_max: Real,
    boundary: CamSupportClipBoundary,
    policy: PredicatePolicy,
) -> Result<CamInfillClipProgramReport, PathMeshBooleanError> {
    if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
        return Err(PathMeshBooleanError::DegenerateHeight);
    }
    let mut sources = infill_graph_path_sources(&graph, z_min.clone(), z_max.clone(), policy)?;
    if sources.is_empty() {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    let initial = sources.remove(0);
    let mut steps = sources
        .into_iter()
        .map(|source| PathMeshBooleanProgramStep::new(PathMeshBooleanOperation::Union, source))
        .collect::<Vec<_>>();
    steps.push(PathMeshBooleanProgramStep::new(
        PathMeshBooleanOperation::Intersection,
        boundary.to_path_source(z_min.clone(), z_max.clone(), policy)?,
    ));
    let exact = infill_clip_exact_facts(&graph, &boundary, &z_min, &z_max);
    let program = boolean_path_mesh_program(initial, steps)?;
    Ok(CamInfillClipProgramReport {
        graph,
        z_min,
        z_max,
        boundary,
        exact,
        program,
    })
}

/// Convenience constructor for a rectangular pocket cutter from integer bounds.
pub fn cam_rectangular_pocket_cutter_from_i64_bounds(
    min: [i64; 2],
    max: [i64; 2],
) -> Result<CamRestMaterialCutter, PathMeshBooleanError> {
    let pocket = RectangularPocket::new(
        hyperlimit::Point2::new(Real::from(min[0]), Real::from(min[1])),
        hyperlimit::Point2::new(Real::from(max[0]), Real::from(max[1])),
    )
    .map_err(|error| match error {
        PocketPlanError::UnorderedBounds => PathMeshBooleanError::DegenerateFootprint,
        _ => PathMeshBooleanError::InvalidScalar,
    })?;
    Ok(CamRestMaterialCutter::RectangularPocket(pocket))
}

fn rest_material_exact_facts(
    stock: &RectangularPocket,
    z_min: &Real,
    z_max: &Real,
    cutters: &[CamRestMaterialCutter],
) -> RealExactSetFacts {
    let mut values = vec![
        &stock.min().x,
        &stock.min().y,
        &stock.max().x,
        &stock.max().y,
        z_min,
        z_max,
    ];
    for cutter in cutters {
        match cutter {
            CamRestMaterialCutter::RectangularPocket(pocket) => {
                values.extend([
                    &pocket.min().x,
                    &pocket.min().y,
                    &pocket.max().x,
                    &pocket.max().y,
                ]);
            }
            CamRestMaterialCutter::AxisAlignedSweep(swept) => {
                values.extend([
                    &swept.centerline().start().x,
                    &swept.centerline().start().y,
                    &swept.centerline().end().x,
                    &swept.centerline().end().y,
                    swept.width(),
                ]);
            }
            CamRestMaterialCutter::OrthogonalIslandPocket(source) => {
                values.extend(source.outer.iter().flat_map(|point| [&point.x, &point.y]));
                for island in &source.islands {
                    values.extend(island.iter().flat_map(|point| [&point.x, &point.y]));
                }
            }
        }
    }
    Real::exact_set_facts(values)
}

fn validate_orthogonal_loop_shape(
    vertices: &[Point2],
    provenance: PathProvenance,
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    OrthogonalPolygonPrism::new(
        vertices.to_vec(),
        Real::zero(),
        Real::one(),
        provenance,
        policy,
    )
    .map(|_| ())
}

fn orthogonal_loop_path_source(
    vertices: &[Point2],
    z_min: Real,
    z_max: Real,
    provenance: PathProvenance,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    Ok(OrthogonalPolygonPrism::new(vertices.to_vec(), z_min, z_max, provenance, policy)?.into())
}

fn infill_graph_path_sources(
    graph: &RectangularInfillGraph,
    z_min: Real,
    z_max: Real,
    policy: PredicatePolicy,
) -> Result<Vec<PathMeshBooleanSource>, PathMeshBooleanError> {
    let mut sources = Vec::with_capacity(graph.deposition_segments.len() + graph.links.len());
    for segment in &graph.deposition_segments {
        sources.push(infill_segment_path_source(
            segment,
            graph.plan.bead_width.clone(),
            z_min.clone(),
            z_max.clone(),
            policy,
        )?);
    }
    for link in &graph.links {
        sources.push(infill_segment_path_source(
            &link.connector,
            graph.plan.bead_width.clone(),
            z_min.clone(),
            z_max.clone(),
            policy,
        )?);
    }
    Ok(sources)
}

fn infill_segment_path_source(
    segment: &crate::segment::LinePathSegment,
    bead_width: Real,
    z_min: Real,
    z_max: Real,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    let swept = SweptLineSegment::new(segment.clone(), bead_width)
        .map_err(|_| PathMeshBooleanError::NonPositiveSweepWidth)?;
    Ok(AxisAlignedSweptSegmentPrism::new(swept, z_min, z_max, policy)?.into())
}

fn island_pocket_exact_facts(outer: &[Point2], islands: &[Vec<Point2>]) -> RealExactSetFacts {
    let mut values = outer
        .iter()
        .flat_map(|point| [&point.x, &point.y])
        .collect::<Vec<_>>();
    for island in islands {
        values.extend(island.iter().flat_map(|point| [&point.x, &point.y]));
    }
    Real::exact_set_facts(values)
}

fn loop_exact_facts(vertices: &[Point2]) -> RealExactSetFacts {
    Real::exact_set_facts(vertices.iter().flat_map(|point| [&point.x, &point.y]))
}

fn infill_clip_exact_facts(
    graph: &RectangularInfillGraph,
    boundary: &CamSupportClipBoundary,
    z_min: &Real,
    z_max: &Real,
) -> RealExactSetFacts {
    let mut values = vec![&graph.plan.bead_width, z_min, z_max];
    for segment in &graph.deposition_segments {
        values.extend([
            &segment.start().x,
            &segment.start().y,
            &segment.end().x,
            &segment.end().y,
        ]);
    }
    for link in &graph.links {
        values.extend([
            &link.connector.start().x,
            &link.connector.start().y,
            &link.connector.end().x,
            &link.connector.end().y,
        ]);
    }
    values.extend(
        boundary
            .vertices()
            .iter()
            .flat_map(|point| [&point.x, &point.y]),
    );
    Real::exact_set_facts(values)
}
