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
use crate::mesh_boolean_handoff::PathExactMeshHandoffSource;
use crate::mesh_boolean_holes::validate_strict_orthogonal_holes;
use crate::mesh_boolean_polygon::{
    ConvexPolygonPrism, OrthogonalPolygonPrism, SimplePolygonPrism,
    validate_strict_simple_polygon_holes,
};
use crate::mesh_boolean_program::{
    PathMeshBooleanProgramReport, PathMeshBooleanProgramStep, boolean_path_mesh_program,
};
use crate::mesh_boolean_sources::{AxisAlignedSweptSegmentPrism, PathMeshBooleanSource};
use crate::provenance::PathProvenance;
use crate::swept::SweptLineSegment;

/// Retained subtractive CAM cutter whose exact topology is owned by a `hypermesh` handoff.
///
/// Use this for non-axis or curved cutter sweeps emitted by a dedicated cutter
/// or arrangement producer. The source is opaque to `hyperpath`; replay checks
/// the exact handoff package and the requested rest-material Z slab before the
/// cutter can participate in a regularized solid difference. This keeps the
/// implementation aligned with Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): exact object evidence may cross
/// crate boundaries, but topology ownership remains with `hypermesh`.
#[derive(Clone, Debug, PartialEq)]
pub struct CamExactRestMaterialCutterHandoff {
    handoff: PathExactMeshHandoffSource,
    exact: RealExactSetFacts,
}

/// Retained material island whose exact topology is owned by a `hypermesh` handoff.
///
/// This is the additive half of an island-pocket rest-material operation. The
/// outer pocket is still subtracted by `hyperpath`, but curved or otherwise
/// non-orthogonal retained islands may be supplied as exact closed-solid
/// handoffs and unioned back during replay. Following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), the handoff
/// remains the exact object evidence; `hyperpath` only performs domain
/// preflight and asks `hypermesh` to own topology acceptance.
#[derive(Clone, Debug, PartialEq)]
pub struct CamExactRestMaterialIslandHandoff {
    handoff: PathExactMeshHandoffSource,
    exact: RealExactSetFacts,
}

/// Retained CAM rest-material stock whose exact topology is owned by a `hypermesh` handoff.
///
/// Rectangular stock remains the common 2.5D fixture path, but rough castings,
/// imported setup stock, and curved stock envelopes need a topology-safe intake
/// that does not move mesh ownership into `hyperpath`. This handoff is the
/// stock-side companion to [`CamExactRestMaterialCutterHandoff`]. It follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): the exact stock object is retained with package evidence, and the
/// accepted boolean topology is trusted only while replay can validate the
/// handoff and requested Z slab.
#[derive(Clone, Debug, PartialEq)]
pub struct CamExactRestMaterialStockHandoff {
    handoff: PathExactMeshHandoffSource,
    exact: RealExactSetFacts,
}

/// Retained subtractive CAM cutter source.
///
/// Rectangular pocket cutters represent exact box removals. Axis-aligned sweep
/// cutters represent retained straight tool-center passes with exact width.
/// Non-axis-aligned and curved cutters enter through [`Self::ExactHandoff`]
/// after a topology-owning producer has emitted an accepted exact mesh handoff;
/// they are never approximated locally from primitive floating geometry.
#[derive(Clone, Debug, PartialEq)]
pub enum CamRestMaterialCutter {
    /// Exact rectangular pocket cutter footprint.
    RectangularPocket(RectangularPocket),
    /// Exact axis-aligned swept cutter path.
    AxisAlignedSweep(SweptLineSegment),
    /// Exact orthogonal pocket boundary with retained material islands.
    OrthogonalIslandPocket(CamOrthogonalIslandPocketCutter),
    /// Exact closed-solid cutter package produced outside `hyperpath`.
    ExactHandoff(CamExactRestMaterialCutterHandoff),
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
    exact_islands: Vec<CamExactRestMaterialIslandHandoff>,
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

/// Exact rest-material boolean program for opaque exact stock.
///
/// This is the retained-source equivalent of [`CamRestMaterialProgramReport`]
/// when stock topology comes from a `hypermesh` handoff instead of a rectangular
/// pocket. Cutter replay remains identical: simple cutters subtract one source,
/// and island pockets expand to `difference outer` followed by `union islands`.
#[derive(Clone, Debug, PartialEq)]
pub struct CamExactStockRestMaterialProgramReport {
    /// Retained exact stock handoff.
    pub stock: CamExactRestMaterialStockHandoff,
    /// Exact lower stock/cutter Z face.
    pub z_min: Real,
    /// Exact upper stock/cutter Z face.
    pub z_max: Real,
    /// Retained cutter sources in program order.
    pub cutters: Vec<CamRestMaterialCutter>,
    /// Exact-set facts for retained stock, cutter, and Z scalar inputs.
    pub exact: RealExactSetFacts,
    /// Accepted exact mesh-boolean program.
    pub program: PathMeshBooleanProgramReport,
}

/// Retained CAM clip boundary whose exact topology is owned by a `hypermesh` handoff.
///
/// This is the CAM-side intake for curved additive support/infill clipping
/// envelopes produced by a curve or arrangement owner outside `hyperpath`.
/// `hyperpath` retains the handoff as an opaque exact closed solid and checks
/// its Z slab at replay time. That follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): cached topology is
/// useful only while the exact object and package evidence still replay, and
/// curve flattening must not be smuggled into a path-domain boolean API.
#[derive(Clone, Debug, PartialEq)]
pub struct CamExactClipBoundaryHandoff {
    handoff: PathExactMeshHandoffSource,
    exact: RealExactSetFacts,
}

/// Retained CAM clip cutout whose exact topology is owned by a `hypermesh` handoff.
///
/// Use this for curved support/infill voids or non-straight internal clipping
/// islands emitted by an exact arrangement producer. The cutout remains opaque
/// to `hyperpath`; replay validates the handoff package and requested Z slab
/// before subtracting it from the clipped additive material. This follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): exact object evidence may cross crate boundaries, but topology
/// ownership stays with `hypermesh`.
#[derive(Clone, Debug, PartialEq)]
pub struct CamExactClipCutoutHandoff {
    handoff: PathExactMeshHandoffSource,
    exact: RealExactSetFacts,
}

/// Retained internal cutout for additive CAM clip replay.
#[derive(Clone, Debug, PartialEq)]
pub enum CamSupportClipCutout {
    /// Simple straight-edge cutout loop retained by `hyperpath`.
    Simple {
        /// Retained cutout vertices.
        vertices: Vec<Point2>,
        /// Source provenance for the retained loop.
        provenance: PathProvenance,
    },
    /// Exact cutout package produced outside `hyperpath`.
    ExactHandoff(CamExactClipCutoutHandoff),
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
    /// Simple hole-free straight-edge outer clip boundary with retained void loops.
    HoledSimple {
        /// Retained outer boundary vertices.
        outer: Vec<Point2>,
        /// Retained void-loop vertices.
        holes: Vec<Vec<Point2>>,
        /// Retained exact cutout packages.
        exact_cutouts: Vec<CamExactClipCutoutHandoff>,
        /// Source provenance shared by all retained loops.
        provenance: PathProvenance,
        /// Exact-set facts for retained coordinates.
        exact: RealExactSetFacts,
    },
    /// Exact closed-solid clipping envelope produced outside `hyperpath`.
    ExactHandoff(CamExactClipBoundaryHandoff),
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
    /// Construct a retained exact cutter from a `hypermesh` handoff.
    ///
    /// This is the subtractive companion to
    /// [`CamSupportClipBoundary::exact_handoff`]. It lets non-axis and curved
    /// cutter producers feed exact closed-solid packages into rest-material
    /// programs without adding mesh construction or curve flattening to
    /// `hyperpath`.
    pub fn exact_handoff(
        handoff: PathExactMeshHandoffSource,
    ) -> Result<Self, PathMeshBooleanError> {
        Ok(Self::ExactHandoff(CamExactRestMaterialCutterHandoff::new(
            handoff,
        )?))
    }

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
            Self::ExactHandoff(source) => source.to_path_source(&z_min, &z_max, policy),
        }
    }
}

impl CamExactRestMaterialCutterHandoff {
    /// Construct a retained exact cutter from a `hypermesh` handoff.
    pub fn new(handoff: PathExactMeshHandoffSource) -> Result<Self, PathMeshBooleanError> {
        let exact = handoff_mesh_exact_facts(&handoff)?;
        Ok(Self { handoff, exact })
    }

    /// Return the retained exact mesh handoff.
    pub const fn handoff(&self) -> &PathExactMeshHandoffSource {
        &self.handoff
    }

    /// Return exact-set facts for retained mesh coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Lower this exact handoff into a generic path mesh-boolean source.
    pub fn to_path_source(
        &self,
        z_min: &Real,
        z_max: &Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        validate_handoff_z_slab(&self.handoff, z_min, z_max, policy)?;
        Ok(self.handoff.clone().into())
    }
}

impl CamExactRestMaterialIslandHandoff {
    /// Construct a retained exact material island from a `hypermesh` handoff.
    ///
    /// The package is checked immediately so stale or malformed handoffs cannot
    /// enter the retained CAM model. [`CamOrthogonalIslandPocketCutter`] then
    /// performs the path-domain footprint preflight, and final program replay
    /// checks the requested Z slab. That staged acceptance mirrors Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997): exact sources are retained, while derived topology is accepted
    /// only when every replay boundary still validates.
    pub fn new(handoff: PathExactMeshHandoffSource) -> Result<Self, PathMeshBooleanError> {
        let exact = handoff_mesh_exact_facts(&handoff)?;
        Ok(Self { handoff, exact })
    }

    /// Return the retained exact mesh handoff.
    pub const fn handoff(&self) -> &PathExactMeshHandoffSource {
        &self.handoff
    }

    /// Return exact-set facts for retained mesh coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Lower this exact island into a generic path mesh-boolean source.
    pub fn to_path_source(
        &self,
        z_min: &Real,
        z_max: &Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        validate_handoff_z_slab(&self.handoff, z_min, z_max, policy)?;
        Ok(self.handoff.clone().into())
    }
}

impl CamExactRestMaterialStockHandoff {
    /// Construct retained exact stock from a `hypermesh` handoff.
    ///
    /// The handoff is package-validated immediately and its exact mesh
    /// coordinates are retained as source facts. Program replay still validates
    /// the requested Z slab before this stock can seed a rest-material boolean.
    pub fn new(handoff: PathExactMeshHandoffSource) -> Result<Self, PathMeshBooleanError> {
        let exact = handoff_mesh_exact_facts(&handoff)?;
        Ok(Self { handoff, exact })
    }

    /// Return the retained exact mesh handoff.
    pub const fn handoff(&self) -> &PathExactMeshHandoffSource {
        &self.handoff
    }

    /// Return exact-set facts for retained stock mesh coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Lower this exact stock into a generic path mesh-boolean source.
    pub fn to_path_source(
        &self,
        z_min: &Real,
        z_max: &Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        validate_handoff_z_slab(&self.handoff, z_min, z_max, policy)?;
        Ok(self.handoff.clone().into())
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
        Self::with_exact_islands(outer, islands, Vec::new(), policy)
    }

    /// Construct a retained orthogonal pocket with strict straight and exact islands.
    ///
    /// Exact island handoffs are conservatively preflighted by their exact mesh
    /// XY bounding boxes: every box must be a strict non-overlapping hole
    /// inside the orthogonal outer loop alongside the retained straight islands.
    /// This intentionally rejects some valid curved islands whose bounding boxes
    /// overlap. The rejection is the price of keeping `hyperpath` from
    /// re-implementing mesh arrangements while still enforcing the CAM contract
    /// before `hypermesh` receives a union step. The regularized
    /// `stock - outer + islands` semantics are Requicha solids; the exact
    /// evidence boundary is Yap's EGC discipline.
    pub fn with_exact_islands(
        outer: Vec<Point2>,
        islands: Vec<Vec<Point2>>,
        exact_islands: Vec<CamExactRestMaterialIslandHandoff>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::with_provenance_and_exact_islands(
            outer,
            islands,
            exact_islands,
            PathProvenance::native(),
            policy,
        )
    }

    /// Construct a retained orthogonal pocket with explicit provenance.
    pub fn with_provenance(
        outer: Vec<Point2>,
        islands: Vec<Vec<Point2>>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::with_provenance_and_exact_islands(outer, islands, Vec::new(), provenance, policy)
    }

    /// Construct a retained orthogonal pocket with explicit provenance and exact islands.
    pub fn with_provenance_and_exact_islands(
        outer: Vec<Point2>,
        islands: Vec<Vec<Point2>>,
        exact_islands: Vec<CamExactRestMaterialIslandHandoff>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        validate_orthogonal_loop_shape(&outer, provenance, policy)?;
        for island in &islands {
            validate_orthogonal_loop_shape(island, provenance, policy)?;
        }
        let exact_island_footprints = exact_island_footprint_loops(&exact_islands, policy)?;
        let all_island_footprints = islands
            .iter()
            .cloned()
            .chain(exact_island_footprints)
            .collect::<Vec<_>>();
        validate_strict_orthogonal_holes(&outer, &all_island_footprints, policy)?;
        let exact = island_pocket_exact_facts(&outer, &islands, &exact_islands);
        Ok(Self {
            outer,
            islands,
            exact_islands,
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

    /// Return retained exact material island handoffs.
    pub fn exact_islands(&self) -> &[CamExactRestMaterialIslandHandoff] {
        &self.exact_islands
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
        let mut sources = self
            .islands
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
            .collect::<Result<Vec<_>, _>>()?;
        for exact_island in &self.exact_islands {
            sources.push(exact_island.to_path_source(&z_min, &z_max, policy)?);
        }
        Ok(sources)
    }
}

impl CamExactClipBoundaryHandoff {
    /// Construct a retained exact clip boundary from a `hypermesh` handoff.
    ///
    /// The constructor validates the generic exact mesh handoff immediately and
    /// records exact-set facts for the retained mesh coordinates. The final
    /// support/infill program still checks the requested Z interval when it
    /// lowers the boundary, because a valid clip envelope for one layer slab is
    /// not automatically valid for another.
    pub fn new(handoff: PathExactMeshHandoffSource) -> Result<Self, PathMeshBooleanError> {
        let exact = handoff_mesh_exact_facts(&handoff)?;
        Ok(Self { handoff, exact })
    }

    /// Return the retained exact mesh handoff.
    pub const fn handoff(&self) -> &PathExactMeshHandoffSource {
        &self.handoff
    }

    /// Return exact-set facts for retained mesh coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Lower this exact handoff into a generic path mesh-boolean source.
    pub fn to_path_source(
        &self,
        z_min: &Real,
        z_max: &Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        validate_handoff_z_slab(&self.handoff, z_min, z_max, policy)?;
        Ok(self.handoff.clone().into())
    }
}

impl CamExactClipCutoutHandoff {
    /// Construct a retained exact clip cutout from a `hypermesh` handoff.
    pub fn new(handoff: PathExactMeshHandoffSource) -> Result<Self, PathMeshBooleanError> {
        let exact = handoff_mesh_exact_facts(&handoff)?;
        Ok(Self { handoff, exact })
    }

    /// Return the retained exact mesh handoff.
    pub const fn handoff(&self) -> &PathExactMeshHandoffSource {
        &self.handoff
    }

    /// Return exact-set facts for retained cutout mesh coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Lower this exact cutout into a generic path mesh-boolean source.
    pub fn to_path_source(
        &self,
        z_min: &Real,
        z_max: &Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        validate_handoff_z_slab(&self.handoff, z_min, z_max, policy)?;
        Ok(self.handoff.clone().into())
    }
}

impl CamSupportClipCutout {
    /// Lower this retained cutout into a subtractive source over the requested Z slab.
    pub fn to_path_source(
        &self,
        z_min: &Real,
        z_max: &Real,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        match self {
            Self::Simple {
                vertices,
                provenance,
            } => Ok(SimplePolygonPrism::new(
                vertices.clone(),
                z_min.clone(),
                z_max.clone(),
                *provenance,
                policy,
            )?
            .into()),
            Self::ExactHandoff(source) => source.to_path_source(z_min, z_max, policy),
        }
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

    /// Construct a retained holed straight-edge clipping boundary.
    ///
    /// A holed boundary is not lowered as one approximate mesh. It replays as
    /// `material ∩ outer - hole_0 - ...`, preserving each loop as an exact
    /// source object. This follows Requicha's regularized set semantics for
    /// solids and Yap's EGC discipline: the accepted topology is reusable only
    /// while the outer and every void loop replay through exact predicates.
    pub fn holed_simple(
        outer: Vec<Point2>,
        holes: Vec<Vec<Point2>>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::holed_simple_with_provenance(outer, holes, PathProvenance::native(), policy)
    }

    /// Construct a retained holed straight-edge clipping boundary with provenance.
    pub fn holed_simple_with_provenance(
        outer: Vec<Point2>,
        holes: Vec<Vec<Point2>>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::holed_simple_with_exact_cutouts(outer, holes, Vec::new(), provenance, policy)
    }

    /// Construct a retained holed clip boundary with exact cutout packages.
    ///
    /// Straight-edge holes and exact cutout XY bounding boxes are proven
    /// strictly inside the retained outer loop and disjoint from each other.
    /// Exact cutouts are package-validated here and Z-slab checked at replay,
    /// because their topology is owned by the producer that emitted the
    /// `hypermesh` handoff. This admits curved or arrangement-owned internal
    /// support/infill clipping voids without moving mesh topology into
    /// `hyperpath`. The conservative bounding-footprint check follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997): reject ambiguous retained-source placement before asking
    /// `hypermesh` to accept derived topology.
    pub fn holed_simple_with_exact_cutouts(
        outer: Vec<Point2>,
        holes: Vec<Vec<Point2>>,
        exact_cutouts: Vec<CamExactClipCutoutHandoff>,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        if holes.is_empty() && exact_cutouts.is_empty() {
            return Err(PathMeshBooleanError::EmptyPolygonHoles);
        }
        validate_clip_cutouts_strictly_inside_outer(&outer, &holes, &exact_cutouts, policy)?;
        let exact = clip_boundary_exact_facts(&outer, &holes, &exact_cutouts);
        Ok(Self::HoledSimple {
            outer,
            holes,
            exact_cutouts,
            provenance,
            exact,
        })
    }

    /// Construct a retained exact clip boundary from a `hypermesh` handoff.
    ///
    /// Use this for curved or arrangement-produced clipping envelopes whose
    /// topology has already been accepted by `hypermesh`. The handoff is
    /// opaque to `hyperpath`; replay only validates the package and Z slab
    /// before intersecting support or infill material with it.
    pub fn exact_handoff(
        handoff: PathExactMeshHandoffSource,
    ) -> Result<Self, PathMeshBooleanError> {
        Ok(Self::ExactHandoff(CamExactClipBoundaryHandoff::new(
            handoff,
        )?))
    }

    /// Return retained boundary vertices.
    pub fn vertices(&self) -> &[Point2] {
        match self {
            Self::Convex { vertices, .. }
            | Self::Orthogonal { vertices, .. }
            | Self::Simple { vertices, .. } => vertices,
            Self::HoledSimple { outer, .. } => outer,
            Self::ExactHandoff(_) => &[],
        }
    }

    /// Return retained void-loop vertices for holed boundaries.
    pub fn hole_vertices(&self) -> &[Vec<Point2>] {
        match self {
            Self::HoledSimple { holes, .. } => holes,
            _ => &[],
        }
    }

    /// Return retained exact cutout packages for holed boundaries.
    pub fn exact_cutouts(&self) -> &[CamExactClipCutoutHandoff] {
        match self {
            Self::HoledSimple { exact_cutouts, .. } => exact_cutouts,
            _ => &[],
        }
    }

    /// Return retained cutouts in replay order.
    pub fn cutouts(&self) -> Vec<CamSupportClipCutout> {
        match self {
            Self::HoledSimple {
                holes,
                exact_cutouts,
                provenance,
                ..
            } => holes
                .iter()
                .cloned()
                .map(|vertices| CamSupportClipCutout::Simple {
                    vertices,
                    provenance: *provenance,
                })
                .chain(
                    exact_cutouts
                        .iter()
                        .cloned()
                        .map(CamSupportClipCutout::ExactHandoff),
                )
                .collect(),
            _ => Vec::new(),
        }
    }

    /// Return retained source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        match self {
            Self::Convex { provenance, .. }
            | Self::Orthogonal { provenance, .. }
            | Self::Simple { provenance, .. }
            | Self::HoledSimple { provenance, .. } => *provenance,
            Self::ExactHandoff(_) => PathProvenance::native(),
        }
    }

    /// Return exact-set facts for retained boundary coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        match self {
            Self::Convex { exact, .. }
            | Self::Orthogonal { exact, .. }
            | Self::Simple { exact, .. }
            | Self::HoledSimple { exact, .. } => exact,
            Self::ExactHandoff(source) => source.exact_facts(),
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
            Self::HoledSimple { .. } => Err(PathMeshBooleanError::Replay(
                "holed CAM clip boundaries require mixed intersection/difference replay".into(),
            )),
            Self::ExactHandoff(source) => source.to_path_source(&z_min, &z_max, policy),
        }
    }

    fn to_program_steps(
        &self,
        z_min: Real,
        z_max: Real,
        policy: PredicatePolicy,
    ) -> Result<Vec<PathMeshBooleanProgramStep>, PathMeshBooleanError> {
        match self {
            Self::HoledSimple {
                outer, provenance, ..
            } => {
                let cutouts = self.cutouts();
                let mut steps = Vec::with_capacity(cutouts.len() + 1);
                steps.push(PathMeshBooleanProgramStep::new(
                    PathMeshBooleanOperation::Intersection,
                    SimplePolygonPrism::new(
                        outer.clone(),
                        z_min.clone(),
                        z_max.clone(),
                        *provenance,
                        policy,
                    )?
                    .into(),
                ));
                for cutout in cutouts {
                    steps.push(PathMeshBooleanProgramStep::new(
                        PathMeshBooleanOperation::Difference,
                        cutout.to_path_source(&z_min, &z_max, policy)?,
                    ));
                }
                Ok(steps)
            }
            _ => Ok(vec![PathMeshBooleanProgramStep::new(
                PathMeshBooleanOperation::Intersection,
                self.to_path_source(z_min, z_max, policy)?,
            )]),
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

impl CamExactStockRestMaterialProgramReport {
    /// Rebuild exact-stock lowering, cutter lowering, and boolean evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed = build_cam_exact_stock_rest_material_program(
            self.stock.clone(),
            self.z_min.clone(),
            self.z_max.clone(),
            self.cutters.clone(),
            policy,
        )?;
        if replayed.program != self.program || replayed.exact != self.exact {
            return Err(PathMeshBooleanError::Replay(
                "retained CAM exact-stock rest-material program no longer matches replay".into(),
            ));
        }
        self.program.validate_replay()
    }

    /// Return the final accepted exact rest-material mesh.
    pub fn mesh(&self) -> Option<&hypermesh::exact::ExactMesh> {
        self.program.mesh()
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
    let steps = cam_rest_material_program_steps(&cutters, z_min.clone(), z_max.clone(), policy)?;
    let program = boolean_path_mesh_program(stock_prism.into(), steps)?;
    let exact = rest_material_exact_facts(&stock, &z_min, &z_max, &cutters)?;
    Ok(CamRestMaterialProgramReport {
        stock,
        z_min,
        z_max,
        cutters,
        exact,
        program,
    })
}

/// Build an exact rest-material difference program from opaque exact stock.
///
/// This is the handoff-stock companion to [`build_cam_rest_material_program`].
/// It admits curved or imported setup stock as an exact closed-solid operand
/// while keeping all cutter semantics in retained `hyperpath` sources. The
/// initial stock handoff is validated against the requested Z slab before any
/// boolean step is attempted; cutter handoffs and island handoffs are validated
/// by the same replay path as rectangular stock programs.
pub fn build_cam_exact_stock_rest_material_program(
    stock: CamExactRestMaterialStockHandoff,
    z_min: Real,
    z_max: Real,
    cutters: Vec<CamRestMaterialCutter>,
    policy: PredicatePolicy,
) -> Result<CamExactStockRestMaterialProgramReport, PathMeshBooleanError> {
    if cutters.is_empty() {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
        return Err(PathMeshBooleanError::DegenerateHeight);
    }
    let initial = stock.to_path_source(&z_min, &z_max, policy)?;
    let steps = cam_rest_material_program_steps(&cutters, z_min.clone(), z_max.clone(), policy)?;
    let program = boolean_path_mesh_program(initial, steps)?;
    let exact = exact_stock_rest_material_exact_facts(&stock, &z_min, &z_max, &cutters)?;
    Ok(CamExactStockRestMaterialProgramReport {
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
    let steps = boundary.to_program_steps(z_min.clone(), z_max.clone(), policy)?;
    let program = boolean_path_mesh_program(support_prism.into(), steps)?;
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
    steps.extend(boundary.to_program_steps(z_min.clone(), z_max.clone(), policy)?);
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
) -> Result<RealExactSetFacts, PathMeshBooleanError> {
    let mut refs = vec![
        &stock.min().x,
        &stock.min().y,
        &stock.max().x,
        &stock.max().y,
        z_min,
        z_max,
    ];
    let handoff_values = rest_material_cutter_exact_values(&mut refs, cutters)?;
    refs.extend(handoff_values.iter());
    Ok(Real::exact_set_facts(refs))
}

fn exact_stock_rest_material_exact_facts(
    stock: &CamExactRestMaterialStockHandoff,
    z_min: &Real,
    z_max: &Real,
    cutters: &[CamRestMaterialCutter],
) -> Result<RealExactSetFacts, PathMeshBooleanError> {
    let mut stock_values = Vec::new();
    let stock_mesh = stock.handoff().to_exact_mesh()?;
    for point in stock_mesh.vertices() {
        let coordinates = &point.coordinates().0;
        stock_values.extend([
            coordinates[0].clone(),
            coordinates[1].clone(),
            coordinates[2].clone(),
        ]);
    }
    let mut refs = vec![z_min, z_max];
    refs.extend(stock_values.iter());
    let handoff_values = rest_material_cutter_exact_values(&mut refs, cutters)?;
    refs.extend(handoff_values.iter());
    Ok(Real::exact_set_facts(refs))
}

fn rest_material_cutter_exact_values<'a>(
    refs: &mut Vec<&'a Real>,
    cutters: &'a [CamRestMaterialCutter],
) -> Result<Vec<Real>, PathMeshBooleanError> {
    let mut handoff_values = Vec::new();
    for cutter in cutters {
        match cutter {
            CamRestMaterialCutter::RectangularPocket(pocket) => {
                refs.extend([
                    &pocket.min().x,
                    &pocket.min().y,
                    &pocket.max().x,
                    &pocket.max().y,
                ]);
            }
            CamRestMaterialCutter::AxisAlignedSweep(swept) => {
                refs.extend([
                    &swept.centerline().start().x,
                    &swept.centerline().start().y,
                    &swept.centerline().end().x,
                    &swept.centerline().end().y,
                    swept.width(),
                ]);
            }
            CamRestMaterialCutter::OrthogonalIslandPocket(source) => {
                refs.extend(source.outer.iter().flat_map(|point| [&point.x, &point.y]));
                for island in &source.islands {
                    refs.extend(island.iter().flat_map(|point| [&point.x, &point.y]));
                }
                for exact_island in &source.exact_islands {
                    let mesh = exact_island.handoff().to_exact_mesh()?;
                    for point in mesh.vertices() {
                        let coordinates = &point.coordinates().0;
                        handoff_values.extend([
                            coordinates[0].clone(),
                            coordinates[1].clone(),
                            coordinates[2].clone(),
                        ]);
                    }
                }
            }
            CamRestMaterialCutter::ExactHandoff(source) => {
                let mesh = source.handoff().to_exact_mesh()?;
                for point in mesh.vertices() {
                    let coordinates = &point.coordinates().0;
                    handoff_values.extend([
                        coordinates[0].clone(),
                        coordinates[1].clone(),
                        coordinates[2].clone(),
                    ]);
                }
            }
        }
    }
    Ok(handoff_values)
}

fn cam_rest_material_program_steps(
    cutters: &[CamRestMaterialCutter],
    z_min: Real,
    z_max: Real,
    policy: PredicatePolicy,
) -> Result<Vec<PathMeshBooleanProgramStep>, PathMeshBooleanError> {
    let mut steps = Vec::new();
    for cutter in cutters {
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
    Ok(steps)
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

fn island_pocket_exact_facts(
    outer: &[Point2],
    islands: &[Vec<Point2>],
    exact_islands: &[CamExactRestMaterialIslandHandoff],
) -> RealExactSetFacts {
    let mut handoff_values = Vec::new();
    let mut values = outer
        .iter()
        .flat_map(|point| [&point.x, &point.y])
        .collect::<Vec<_>>();
    for island in islands {
        values.extend(island.iter().flat_map(|point| [&point.x, &point.y]));
    }
    for exact_island in exact_islands {
        if let Ok(mesh) = exact_island.handoff().to_exact_mesh() {
            for point in mesh.vertices() {
                let coordinates = &point.coordinates().0;
                handoff_values.extend([
                    coordinates[0].clone(),
                    coordinates[1].clone(),
                    coordinates[2].clone(),
                ]);
            }
        }
    }
    values.extend(handoff_values.iter());
    Real::exact_set_facts(values)
}

fn exact_island_footprint_loops(
    exact_islands: &[CamExactRestMaterialIslandHandoff],
    policy: PredicatePolicy,
) -> Result<Vec<Vec<Point2>>, PathMeshBooleanError> {
    exact_islands
        .iter()
        .map(|island| exact_handoff_footprint_loop(island.handoff(), policy))
        .collect()
}

fn exact_clip_cutout_footprint_loops(
    exact_cutouts: &[CamExactClipCutoutHandoff],
    policy: PredicatePolicy,
) -> Result<Vec<Vec<Point2>>, PathMeshBooleanError> {
    exact_cutouts
        .iter()
        .map(|cutout| exact_handoff_footprint_loop(cutout.handoff(), policy))
        .collect()
}

fn exact_handoff_footprint_loop(
    handoff: &PathExactMeshHandoffSource,
    policy: PredicatePolicy,
) -> Result<Vec<Point2>, PathMeshBooleanError> {
    let mesh = handoff.to_exact_mesh()?;
    let Some(bounds) = &mesh.bounds().mesh else {
        return Err(PathMeshBooleanError::MeshHandoff(
            "CAM exact cutout/island handoff has no mesh bounds".into(),
        ));
    };
    if compare_reals_with_policy(&bounds.min.x, &bounds.max.x, policy).value()
        != Some(Ordering::Less)
        || compare_reals_with_policy(&bounds.min.y, &bounds.max.y, policy).value()
            != Some(Ordering::Less)
    {
        return Err(PathMeshBooleanError::DegenerateFootprint);
    }
    Ok(vec![
        Point2::new(bounds.min.x.clone(), bounds.min.y.clone()),
        Point2::new(bounds.max.x.clone(), bounds.min.y.clone()),
        Point2::new(bounds.max.x.clone(), bounds.max.y.clone()),
        Point2::new(bounds.min.x.clone(), bounds.max.y.clone()),
    ])
}

fn loop_exact_facts(vertices: &[Point2]) -> RealExactSetFacts {
    loops_exact_facts(vertices, &[])
}

fn loops_exact_facts(outer: &[Point2], holes: &[Vec<Point2>]) -> RealExactSetFacts {
    let mut values = outer
        .iter()
        .flat_map(|point| [&point.x, &point.y])
        .collect::<Vec<_>>();
    for hole in holes {
        values.extend(hole.iter().flat_map(|point| [&point.x, &point.y]));
    }
    Real::exact_set_facts(values)
}

fn validate_clip_cutouts_strictly_inside_outer(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    exact_cutouts: &[CamExactClipCutoutHandoff],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    let exact_cutout_footprints = exact_clip_cutout_footprint_loops(exact_cutouts, policy)?;
    let all_cutout_footprints = holes
        .iter()
        .cloned()
        .chain(exact_cutout_footprints)
        .collect::<Vec<_>>();
    validate_strict_simple_polygon_holes(outer, &all_cutout_footprints, policy)
}

fn clip_boundary_exact_facts(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    exact_cutouts: &[CamExactClipCutoutHandoff],
) -> RealExactSetFacts {
    let mut values = Vec::new();
    let mut refs = outer
        .iter()
        .flat_map(|point| [&point.x, &point.y])
        .collect::<Vec<_>>();
    for hole in holes {
        refs.extend(hole.iter().flat_map(|point| [&point.x, &point.y]));
    }
    for exact_cutout in exact_cutouts {
        if let Ok(mesh) = exact_cutout.handoff().to_exact_mesh() {
            for point in mesh.vertices() {
                let coordinates = &point.coordinates().0;
                values.extend([
                    coordinates[0].clone(),
                    coordinates[1].clone(),
                    coordinates[2].clone(),
                ]);
            }
        }
    }
    refs.extend(values.iter());
    Real::exact_set_facts(refs)
}

fn handoff_mesh_exact_facts(
    handoff: &PathExactMeshHandoffSource,
) -> Result<RealExactSetFacts, PathMeshBooleanError> {
    let mesh = handoff.to_exact_mesh()?;
    let values = mesh
        .vertices()
        .iter()
        .flat_map(|point| {
            let coordinates = &point.coordinates().0;
            [
                coordinates[0].clone(),
                coordinates[1].clone(),
                coordinates[2].clone(),
            ]
        })
        .collect::<Vec<_>>();
    Ok(Real::exact_set_facts(values.iter().collect::<Vec<_>>()))
}

fn validate_handoff_z_slab(
    handoff: &PathExactMeshHandoffSource,
    z_min: &Real,
    z_max: &Real,
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    let mesh = handoff.to_exact_mesh()?;
    let Some(bounds) = &mesh.bounds().mesh else {
        return Err(PathMeshBooleanError::MeshHandoff(
            "CAM exact clip handoff has no mesh bounds".into(),
        ));
    };
    if compare_reals_with_policy(&bounds.min.z, z_min, policy).value() != Some(Ordering::Equal)
        || compare_reals_with_policy(&bounds.max.z, z_max, policy).value() != Some(Ordering::Equal)
    {
        return Err(PathMeshBooleanError::MeshHandoff(
            "CAM exact clip handoff does not match the requested Z slab".into(),
        ));
    }
    Ok(())
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
    for hole in boundary.hole_vertices() {
        values.extend(hole.iter().flat_map(|point| [&point.x, &point.y]));
    }
    let mut exact_cutout_values = Vec::new();
    for exact_cutout in boundary.exact_cutouts() {
        if let Ok(mesh) = exact_cutout.handoff().to_exact_mesh() {
            for point in mesh.vertices() {
                let coordinates = &point.coordinates().0;
                exact_cutout_values.extend([
                    coordinates[0].clone(),
                    coordinates[1].clone(),
                    coordinates[2].clone(),
                ]);
            }
        }
    }
    values.extend(exact_cutout_values.iter());
    Real::exact_set_facts(values)
}
