//! PCB-specific retained sources for exact mesh booleans.
//!
//! PCB routing objects are not generic boxes: they carry net, layer, source
//! provenance, and exact swept geometry. This module lowers the subset that is
//! already exact in `hyperpath`--axis-aligned traces, rectangular/cardinal
//! rectangular pads, strictly convex polygonal copper, and simple orthogonal
//! polygonal copper--into retained mesh-boolean sources. The lowering is
//! layer-aware through [`PcbLayerZModel`], so boolean programs can operate over
//! copper solids while replay still starts from PCB path objects.
//!
//! The split follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): PCB import/routing objects propose
//! geometry, exact predicates certify the source-to-solid lowering, and
//! `hypermesh` remains the only owner of accepted mesh topology. It also
//! matches the Lee/Hightower routing tradition: graph/layer routing candidates
//! are separate from final geometric certification.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hypermesh::exact::{
    ExactBooleanPreflight, ExactBooleanResult, ExactBoundaryBooleanPolicy, ExactMesh,
    ValidationPolicy, boolean_exact_with_boundary_policy, preflight_boolean_exact,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::cam::{PocketPlanError, RectangularPocket};
use crate::mesh_boolean::{PathMeshBooleanError, PathMeshBooleanOperation, RectangularPrism};
use crate::mesh_boolean_handoff::PathExactMeshHandoffSource;
use crate::mesh_boolean_holes::validate_strict_orthogonal_holes;
use crate::mesh_boolean_polygon::{ConvexPolygonPrism, OrthogonalPolygonPrism};
use crate::mesh_boolean_program::{
    PathMeshBooleanProgramReport, PathMeshBooleanProgramStep, boolean_path_mesh_program,
};
use crate::mesh_boolean_sources::{AxisAlignedSweptSegmentPrism, PathMeshBooleanSource};
use crate::pcb::{
    BoardContourError, NetId, PcbCardinalRectPad, PcbConvexBoardOutline, PcbConvexPolyPad,
    PcbOrthogonalBoardOutline, PcbOrthogonalPolyPad, PcbRectPad, PcbTrace, TraceLayer,
};

/// Exact Z mapping for discrete PCB copper layers.
///
/// `z_origin + layer * layer_pitch` is the lower copper face, and
/// `copper_thickness` is added to obtain the upper face. The model is
/// deliberately simple and exact: it describes uniform layer spacing for
/// mesh-boolean source construction, not the complete fabrication stackup.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbLayerZModel {
    z_origin: Real,
    layer_pitch: Real,
    copper_thickness: Real,
    exact: RealExactSetFacts,
}

/// Exact Z interval for one PCB copper layer.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbLayerSlab {
    /// Layer mapped into this slab.
    pub layer: TraceLayer,
    /// Exact lower Z face.
    pub z_min: Real,
    /// Exact upper Z face.
    pub z_max: Real,
}

/// Retained PCB copper whose exact topology is owned by a `hypermesh` handoff.
///
/// This is the PCB-facing admission path for rounded pads, curved pads, and
/// other copper producers that can emit exact closed-solid mesh packages before
/// `hyperpath` has native arrangement carriers for their source geometry. The
/// carrier retains net/layer semantics in `hyperpath`, while
/// [`PathExactMeshHandoffSource`] retains the package replay evidence. Boolean
/// lowering validates the handoff against the requested [`PcbLayerZModel`] so a
/// source produced for one PCB layer cannot be silently reused on another.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbExactCopperHandoffSource {
    net: NetId,
    layer: TraceLayer,
    handoff: PathExactMeshHandoffSource,
}

/// Retained board clipping solid whose topology is owned by a `hypermesh` handoff.
///
/// Straight-edge outlines remain represented by [`PcbCopperBoardClipOutline`]
/// variants above, but curved/non-orthogonal board producers can now hand over
/// an exact closed-solid board slab without giving `hyperpath` mesh topology
/// ownership. Replaying the board clip revalidates both the handoff package and
/// the exact layer Z interval before the board slab is intersected with copper.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbExactBoardHandoffOutline {
    layer: TraceLayer,
    handoff: PathExactMeshHandoffSource,
}

/// Retained board cutout whose exact topology is owned by a `hypermesh` handoff.
///
/// Use this for curved slots, routed mechanical openings, or other internal
/// board cutouts emitted by an exact topology producer outside `hyperpath`.
/// The handoff is opaque here: replay validates the closed-solid package and
/// the target PCB layer slab before the cutout can subtract copper. This is
/// the same exact-object boundary advocated by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), with regularized
/// subtraction following Requicha, "Representations for Rigid Solids: Theory,
/// Methods, and Systems," *ACM Computing Surveys* 12.4 (1980).
#[derive(Clone, Debug, PartialEq)]
pub struct PcbExactBoardCutoutHandoff {
    handoff: PathExactMeshHandoffSource,
    exact: RealExactSetFacts,
}

/// Retained orthogonal board outline with strict internal cutouts.
///
/// PCB board clipping is not always a single outer loop: slots, keep-out
/// windows, castellated void approximations, and panelized mechanical cutouts
/// can remove copper from the board interior. This carrier keeps that topology
/// as retained PCB board geometry. It is lowered as an exact outer
/// intersection followed by one exact difference per void, matching
/// Requicha's regularized set-operation model while preserving Yap's
/// object-before-topology discipline.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbHoledOrthogonalBoardClipOutline {
    outer: PcbOrthogonalBoardOutline,
    holes: Vec<PcbOrthogonalBoardOutline>,
    exact_cutouts: Vec<PcbExactBoardCutoutHandoff>,
    exact: RealExactSetFacts,
}

/// Retained board cutout used by board-clip replay.
#[derive(Clone, Debug, PartialEq)]
pub enum PcbBoardClipCutout {
    /// Exact orthogonal cutout loop retained by `hyperpath`.
    Orthogonal(PcbOrthogonalBoardOutline),
    /// Exact cutout package produced outside `hyperpath`.
    ExactHandoff(PcbExactBoardCutoutHandoff),
}

/// Retained PCB copper source supported by the exact mesh-boolean handoff.
///
/// This enum keeps PCB semantics attached until the final lowering step. It is
/// intentionally narrower than all PCB geometry: circular/rounded pads need
/// native curved arrangements or an exact [`PcbExactCopperHandoffSource`]
/// package before they can become mesh-boolean operands without approximation.
#[derive(Clone, Debug, PartialEq)]
pub enum PcbCopperBooleanSource {
    /// Axis-aligned retained trace.
    Trace(PcbTrace),
    /// Axis-aligned rectangular pad.
    RectPad(PcbRectPad),
    /// Cardinally rotated rectangular pad.
    CardinalRectPad(PcbCardinalRectPad),
    /// Strictly convex polygonal pad or copper zone.
    ConvexPolyPad(PcbConvexPolyPad),
    /// Simple orthogonal polygonal pad or copper zone.
    OrthogonalPolyPad(PcbOrthogonalPolyPad),
    /// Exact closed-solid copper package produced outside `hyperpath`.
    ExactHandoff(PcbExactCopperHandoffSource),
}

/// Exact PCB copper-union program for one net on one layer.
///
/// The accepted [`PathMeshBooleanProgramReport`] is retained rather than
/// flattened to a mesh. Replay therefore revalidates source lowering, net/layer
/// grouping, and `hypermesh` boolean evidence before the output topology is
/// trusted.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCopperBooleanProgramReport {
    /// Net shared by every retained copper source.
    pub net: NetId,
    /// Layer shared by every retained copper source.
    pub layer: TraceLayer,
    /// Exact layer-to-Z model used for lowering.
    pub z_model: PcbLayerZModel,
    /// Retained PCB copper sources in union order.
    pub sources: Vec<PcbCopperBooleanSource>,
    /// Accepted exact mesh-boolean program.
    pub program: PathMeshBooleanProgramReport,
}

/// Retained orthogonal copper source with strict orthogonal voids.
///
/// A holed pour is not a primitive source in the same sense as a trace or a
/// simple polygon. It is an outer copper object plus subtractive interior
/// objects, so this module lowers it as a replayable difference program. That
/// follows Yap's exact-object boundary directly: the outer loop and every hole
/// loop remain retained PCB objects, and `hypermesh` only owns the accepted
/// topology produced by replaying their boolean differences.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbHoledOrthogonalCopperSource {
    net: NetId,
    layer: TraceLayer,
    outer: PcbOrthogonalPolyPad,
    holes: Vec<PcbOrthogonalPolyPad>,
    exact: RealExactSetFacts,
}

/// Exact PCB copper program for one holed orthogonal copper source.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbHoledCopperBooleanProgramReport {
    /// Net owned by the retained copper source.
    pub net: NetId,
    /// Layer owned by the retained copper source.
    pub layer: TraceLayer,
    /// Exact layer-to-Z model used for lowering.
    pub z_model: PcbLayerZModel,
    /// Retained holed orthogonal source.
    pub source: PcbHoledOrthogonalCopperSource,
    /// Accepted exact outer-minus-holes boolean program.
    pub program: PathMeshBooleanProgramReport,
}

/// Retained PCB copper operand for composite same-net union programs.
///
/// Solid sources lower directly to one exact mesh. Holed orthogonal sources
/// first replay their retained outer-minus-holes program and then become a
/// union operand. Keeping these cases distinct follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): topology
/// produced by a boolean is accepted only with the replayable object/predicate
/// evidence that generated it.
#[derive(Clone, Debug, PartialEq)]
pub enum PcbCompositeCopperBooleanSource {
    /// Direct solid PCB copper source.
    Solid(PcbCopperBooleanSource),
    /// Retained orthogonal copper source with strict voids.
    HoledOrthogonal(PcbHoledOrthogonalCopperSource),
}

/// Materialized composite operand with retained provenance.
///
/// The `mesh` itself is intentionally not stored here. A direct solid can be
/// rederived from the source, while a holed source replays through
/// [`PcbHoledCopperBooleanProgramReport`]. This prevents the report from
/// treating an intermediate mesh as a new canonical PCB object.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCompositeCopperMaterialization {
    /// Retained composite source.
    pub source: PcbCompositeCopperBooleanSource,
    /// Replayable holed-source program, present only for holed operands.
    pub holed_program: Option<PcbHoledCopperBooleanProgramReport>,
}

/// Accepted evidence for one composite PCB copper union step.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCompositeCopperBooleanStepReport {
    /// Zero-based union step index.
    pub index: usize,
    /// Right-hand retained operand materialized for this step.
    pub right: PcbCompositeCopperMaterialization,
    /// Exact `hypermesh` preflight report for accumulator/right.
    pub preflight: ExactBooleanPreflight,
    /// Accepted exact union result for accumulator/right.
    pub result: ExactBooleanResult,
}

/// Exact same-net PCB copper-union program over solid and holed operands.
///
/// This is the next retained-source layer above
/// [`PcbHoledCopperBooleanProgramReport`]. The first operand seeds the
/// accumulator; every following operand is unioned with `hypermesh` and the
/// exact result is replayed against the original retained PCB source. The
/// regularized-solid interpretation is the one described by Requicha,
/// "Representations for Rigid Solids: Theory, Methods, and Systems,"
/// *ACM Computing Surveys* 12.4 (1980), while the replay boundary follows
/// Yap's exact geometric computation discipline.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCompositeCopperBooleanProgramReport {
    /// Net shared by every retained copper source.
    pub net: NetId,
    /// Layer shared by every retained copper source.
    pub layer: TraceLayer,
    /// Exact layer-to-Z model used for lowering.
    pub z_model: PcbLayerZModel,
    /// Retained composite sources in union order.
    pub sources: Vec<PcbCompositeCopperBooleanSource>,
    /// Accepted materialization of the first source.
    pub initial: PcbCompositeCopperMaterialization,
    /// Accepted per-step exact union evidence.
    pub steps: Vec<PcbCompositeCopperBooleanStepReport>,
}

/// Retained straight-edge board outline used to clip PCB copper solids.
///
/// Board clipping is an intersection, not a copper-source union. This enum
/// keeps board geometry in the PCB domain until the final exact boolean step,
/// following Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): the retained board object remains the authority,
/// while accepted mesh topology is replayable evidence.
#[derive(Clone, Debug, PartialEq)]
pub enum PcbCopperBoardClipOutline {
    /// Strictly convex straight-edge board outline.
    Convex(PcbConvexBoardOutline),
    /// Simple orthogonal board outline, including rectilinear notches.
    Orthogonal(PcbOrthogonalBoardOutline),
    /// Simple orthogonal board outline with strict interior cutouts.
    HoledOrthogonal(PcbHoledOrthogonalBoardClipOutline),
    /// Exact closed-solid board slab package produced outside `hyperpath`.
    ExactHandoff(PcbExactBoardHandoffOutline),
}

/// Accepted evidence for the final copper/board intersection step.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCopperBoardClipStepReport {
    /// Retained board outline consumed by this intersection.
    pub outline: PcbCopperBoardClipOutline,
    /// Exact `hypermesh` preflight report for copper accumulator/board slab.
    pub preflight: ExactBooleanPreflight,
    /// Accepted exact clipped copper result.
    pub result: ExactBooleanResult,
}

/// Accepted evidence for one internal board cutout subtraction.
///
/// The step index follows the retained hole order. Each subtraction is checked
/// against the accumulator produced by the previous clip operation and the
/// exact orthogonal hole slab. This keeps internal board voids auditable as
/// first-class boolean evidence instead of hiding them in an unowned mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCopperBoardClipVoidStepReport {
    /// Zero-based retained board-hole index.
    pub index: usize,
    /// Retained board cutout source.
    pub cutout: PcbBoardClipCutout,
    /// Exact `hypermesh` preflight report for accumulator/hole slab.
    pub preflight: ExactBooleanPreflight,
    /// Accepted exact copper-minus-board-void result.
    pub result: ExactBooleanResult,
}

/// Exact same-net PCB copper program clipped to a retained board outline.
///
/// The report first materializes retained copper sources exactly, then clips
/// the accepted copper accumulator to a layer slab of the retained board
/// outline. The operation is a Requicha regularized solid intersection
/// ("Representations for Rigid Solids: Theory, Methods, and Systems,"
/// *ACM Computing Surveys* 12.4 (1980)); the source retention and replay
/// boundary follow Yap's exact-geometric-computation model.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCopperBoardClipProgramReport {
    /// Net shared by every retained copper source.
    pub net: NetId,
    /// Layer shared by every retained copper source and board slab.
    pub layer: TraceLayer,
    /// Exact layer-to-Z model used for lowering.
    pub z_model: PcbLayerZModel,
    /// Retained copper sources in union order before board clipping.
    pub sources: Vec<PcbCompositeCopperBooleanSource>,
    /// Accepted materialization of the first copper source.
    pub initial: PcbCompositeCopperMaterialization,
    /// Accepted same-net copper union evidence before clipping.
    pub union_steps: Vec<PcbCompositeCopperBooleanStepReport>,
    /// Accepted exact board-intersection evidence.
    pub clip_step: PcbCopperBoardClipStepReport,
    /// Accepted exact board-cutout subtraction evidence.
    pub void_steps: Vec<PcbCopperBoardClipVoidStepReport>,
}

impl PcbLayerZModel {
    /// Construct a uniform exact PCB layer-to-Z model.
    ///
    /// Pitch and copper thickness must be strictly positive, and copper
    /// thickness may not exceed pitch. That guard prevents adjacent copper
    /// layers from silently overlapping before any `hypermesh` boolean proof is
    /// requested.
    pub fn new(
        z_origin: Real,
        layer_pitch: Real,
        copper_thickness: Real,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        if compare_reals_with_policy(&layer_pitch, &Real::zero(), policy).value()
            != Some(Ordering::Greater)
        {
            return Err(PathMeshBooleanError::NonPositiveLayerPitch);
        }
        if compare_reals_with_policy(&copper_thickness, &Real::zero(), policy).value()
            != Some(Ordering::Greater)
        {
            return Err(PathMeshBooleanError::NonPositiveCopperThickness);
        }
        if compare_reals_with_policy(&copper_thickness, &layer_pitch, policy).value()
            == Some(Ordering::Greater)
        {
            return Err(PathMeshBooleanError::CopperThicknessExceedsPitch);
        }
        let exact = Real::exact_set_facts([&z_origin, &layer_pitch, &copper_thickness]);
        Ok(Self {
            z_origin,
            layer_pitch,
            copper_thickness,
            exact,
        })
    }

    /// Return exact model origin.
    pub const fn z_origin(&self) -> &Real {
        &self.z_origin
    }

    /// Return exact layer pitch.
    pub const fn layer_pitch(&self) -> &Real {
        &self.layer_pitch
    }

    /// Return exact copper thickness.
    pub const fn copper_thickness(&self) -> &Real {
        &self.copper_thickness
    }

    /// Return exact-set facts for the layer model scalars.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Return the exact Z slab for a PCB copper layer.
    pub fn slab_for_layer(&self, layer: TraceLayer) -> PcbLayerSlab {
        let z_min = self.z_origin.clone() + self.layer_pitch.clone() * Real::from(layer.0);
        let z_max = z_min.clone() + self.copper_thickness.clone();
        PcbLayerSlab {
            layer,
            z_min,
            z_max,
        }
    }
}

impl PcbHoledOrthogonalCopperSource {
    /// Construct a retained orthogonal copper source with strict holes.
    ///
    /// Hole containment is checked by exact ray-crossing predicates in the
    /// tradition of Shimrat's point-in-polygon algorithm and Haines' survey of
    /// crossing tests. Hole/hole ambiguity is rejected before boolean lowering:
    /// every hole must be strictly inside the outer loop, and no hole may
    /// intersect, touch, contain, or be contained by another hole.
    pub fn new(
        net: NetId,
        layer: TraceLayer,
        outer_vertices: Vec<Point2>,
        hole_vertices: Vec<Vec<Point2>>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        if hole_vertices.is_empty() {
            return Err(PathMeshBooleanError::EmptyPolygonHoles);
        }
        let outer = PcbOrthogonalPolyPad::new(net, layer, outer_vertices)
            .map_err(path_error_from_board_contour_error)?;
        let holes = hole_vertices
            .into_iter()
            .map(|vertices| {
                PcbOrthogonalPolyPad::new(net, layer, vertices)
                    .map_err(path_error_from_board_contour_error)
            })
            .collect::<Result<Vec<_>, _>>()?;
        validate_holes_strictly_inside_outer(&outer, &holes, policy)?;
        let exact = holed_orthogonal_exact_facts(&outer, &holes);
        Ok(Self {
            net,
            layer,
            outer,
            holes,
            exact,
        })
    }

    /// Return source net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return source layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return retained outer copper loop.
    pub const fn outer(&self) -> &PcbOrthogonalPolyPad {
        &self.outer
    }

    /// Return retained strict hole loops.
    pub fn holes(&self) -> &[PcbOrthogonalPolyPad] {
        &self.holes
    }

    /// Return exact-set facts for all retained loop coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }
}

impl PcbExactCopperHandoffSource {
    /// Construct a retained PCB copper operand from an exact mesh handoff.
    ///
    /// The handoff itself has already proven closed-solid package readiness.
    /// Net and layer are PCB semantics, so they are retained here and checked
    /// by same-net/layer union builders before any mesh boolean is requested.
    pub const fn new(net: NetId, layer: TraceLayer, handoff: PathExactMeshHandoffSource) -> Self {
        Self {
            net,
            layer,
            handoff,
        }
    }

    /// Return source net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return source layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return the retained exact mesh handoff.
    pub const fn handoff(&self) -> &PathExactMeshHandoffSource {
        &self.handoff
    }

    /// Lower this exact handoff into a generic path mesh-boolean source.
    ///
    /// This validates both replayability and the exact PCB layer slab before
    /// exposing the source to boolean code. That follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): cached
    /// topology is admissible only while the retained exact object facts still
    /// replay, and domain metadata cannot be inferred from a mesh by convention.
    pub fn to_path_source(
        &self,
        z_model: &PcbLayerZModel,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        validate_handoff_layer_slab(&self.handoff, &z_model.slab_for_layer(self.layer), policy)?;
        Ok(self.handoff.clone().into())
    }
}

impl PcbExactBoardHandoffOutline {
    /// Construct a retained board clipping slab from an exact mesh handoff.
    ///
    /// The layer is explicit because board clipping in this module intersects
    /// copper with a layer-local board slab. Full stack-height board bodies can
    /// be represented by producer crates, but this path admits only the exact
    /// slab matching the copper layer being clipped.
    pub const fn new(layer: TraceLayer, handoff: PathExactMeshHandoffSource) -> Self {
        Self { layer, handoff }
    }

    /// Return the copper layer this board slab clips.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return the retained exact mesh handoff.
    pub const fn handoff(&self) -> &PathExactMeshHandoffSource {
        &self.handoff
    }

    /// Lower this exact board handoff into a generic path mesh-boolean source.
    pub fn to_path_source(
        &self,
        z_model: &PcbLayerZModel,
        layer: TraceLayer,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        if self.layer != layer {
            return Err(PathMeshBooleanError::MixedPcbLayers);
        }
        validate_handoff_layer_slab(&self.handoff, &z_model.slab_for_layer(layer), policy)?;
        Ok(self.handoff.clone().into())
    }
}

impl PcbExactBoardCutoutHandoff {
    /// Construct a retained exact board cutout from a `hypermesh` handoff.
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
        slab: &PcbLayerSlab,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        validate_handoff_layer_slab(&self.handoff, slab, policy)?;
        Ok(self.handoff.clone().into())
    }
}

impl PcbHoledOrthogonalBoardClipOutline {
    /// Construct a retained orthogonal board clip outline with strict cutouts.
    ///
    /// The constructor validates every loop as simple orthogonal geometry, then
    /// proves each void is strictly inside the outer board and disjoint from
    /// every other void. This uses the same exact retained-hole predicate
    /// discipline as holed copper pours, following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): board
    /// cutouts are accepted from retained loops and replayed boolean evidence,
    /// not from tolerance-classified mesh fragments.
    pub fn new(
        outer_vertices: Vec<Point2>,
        hole_vertices: Vec<Vec<Point2>>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        Self::with_exact_cutouts(outer_vertices, hole_vertices, Vec::new(), policy)
    }

    /// Construct a retained orthogonal board clip outline with exact cutouts.
    ///
    /// Orthogonal holes are proven strictly inside the outer loop here. Exact
    /// handoff cutouts are package-validated at construction and layer-slab
    /// validated when the board clip is replayed; their internal topology is
    /// owned by the producing crate and `hypermesh`, not by `hyperpath`.
    pub fn with_exact_cutouts(
        outer_vertices: Vec<Point2>,
        hole_vertices: Vec<Vec<Point2>>,
        exact_cutouts: Vec<PcbExactBoardCutoutHandoff>,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        if hole_vertices.is_empty() && exact_cutouts.is_empty() {
            return Err(PathMeshBooleanError::EmptyPolygonHoles);
        }
        let outer = PcbOrthogonalBoardOutline::new(outer_vertices)
            .map_err(path_error_from_board_contour_error)?;
        let holes = hole_vertices
            .into_iter()
            .map(|vertices| {
                PcbOrthogonalBoardOutline::new(vertices)
                    .map_err(path_error_from_board_contour_error)
            })
            .collect::<Result<Vec<_>, _>>()?;
        if !holes.is_empty() {
            validate_board_cutouts_strictly_inside_outer(&outer, &holes, policy)?;
        }
        let exact = holed_board_exact_facts(&outer, &holes, &exact_cutouts);
        Ok(Self {
            outer,
            holes,
            exact_cutouts,
            exact,
        })
    }

    /// Return the retained outer board loop.
    pub const fn outer(&self) -> &PcbOrthogonalBoardOutline {
        &self.outer
    }

    /// Return retained strict board cutout loops.
    pub fn holes(&self) -> &[PcbOrthogonalBoardOutline] {
        &self.holes
    }

    /// Return retained exact handoff board cutouts.
    pub fn exact_cutouts(&self) -> &[PcbExactBoardCutoutHandoff] {
        &self.exact_cutouts
    }

    /// Return all retained board cutouts in replay order.
    pub fn cutouts(&self) -> Vec<PcbBoardClipCutout> {
        self.holes
            .iter()
            .cloned()
            .map(PcbBoardClipCutout::Orthogonal)
            .chain(
                self.exact_cutouts
                    .iter()
                    .cloned()
                    .map(PcbBoardClipCutout::ExactHandoff),
            )
            .collect()
    }

    /// Return exact-set facts for all retained board loop coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }
}

impl PcbBoardClipCutout {
    /// Lower this retained cutout into a layer-aware subtractive source.
    pub fn to_path_source(
        &self,
        slab: &PcbLayerSlab,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        match self {
            Self::Orthogonal(outline) => pcb_orthogonal_board_outline_source(outline, slab, policy),
            Self::ExactHandoff(handoff) => handoff.to_path_source(slab, policy),
        }
    }
}

impl PcbHoledCopperBooleanProgramReport {
    /// Rebuild hole containment, source lowering, and boolean evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed = build_pcb_holed_orthogonal_copper_program(
            self.source.clone(),
            self.z_model.clone(),
            policy,
        )?;
        if replayed.net != self.net
            || replayed.layer != self.layer
            || replayed.program != self.program
        {
            return Err(PathMeshBooleanError::Replay(
                "retained PCB holed copper boolean program no longer matches replay".into(),
            ));
        }
        self.program.validate_replay()
    }
}

impl PcbCompositeCopperBooleanSource {
    /// Return the source net.
    pub const fn net(&self) -> NetId {
        match self {
            Self::Solid(source) => source.net(),
            Self::HoledOrthogonal(source) => source.net(),
        }
    }

    /// Return the source layer.
    pub const fn layer(&self) -> TraceLayer {
        match self {
            Self::Solid(source) => source.layer(),
            Self::HoledOrthogonal(source) => source.layer(),
        }
    }
}

impl From<PcbCopperBooleanSource> for PcbCompositeCopperBooleanSource {
    fn from(source: PcbCopperBooleanSource) -> Self {
        Self::Solid(source)
    }
}

impl PcbCompositeCopperMaterialization {
    /// Rebuild the exact mesh represented by this materialized operand.
    ///
    /// This method is deliberately a replay operation, not a mesh cache. Direct
    /// solids lower from their retained source; holed operands ask their nested
    /// boolean program for the accepted final mesh. That keeps the canonical
    /// source on the PCB side of Yap's EGC boundary.
    pub fn to_exact_mesh(
        &self,
        z_model: &PcbLayerZModel,
        policy: PredicatePolicy,
    ) -> Result<ExactMesh, PathMeshBooleanError> {
        match (&self.source, &self.holed_program) {
            (PcbCompositeCopperBooleanSource::Solid(source), None) => {
                source.to_path_source(z_model, policy)?.to_exact_mesh()
            }
            (PcbCompositeCopperBooleanSource::HoledOrthogonal(_), Some(program)) => program
                .program
                .mesh()
                .cloned()
                .ok_or(PathMeshBooleanError::NotEnoughSources),
            _ => Err(PathMeshBooleanError::Replay(
                "composite copper materialization does not match retained source kind".into(),
            )),
        }
    }
}

impl PcbCompositeCopperBooleanProgramReport {
    /// Return the final accepted exact output mesh.
    pub fn mesh(&self) -> Option<&ExactMesh> {
        self.steps.last().map(|step| &step.result.mesh)
    }

    /// Rebuild source materialization, same-net grouping, and union evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed = build_pcb_composite_copper_union_program(
            self.sources.clone(),
            self.z_model.clone(),
            policy,
        )?;
        if replayed.net != self.net
            || replayed.layer != self.layer
            || replayed.initial != self.initial
            || replayed.steps != self.steps
        {
            return Err(PathMeshBooleanError::Replay(
                "retained PCB composite copper boolean program no longer matches replay".into(),
            ));
        }
        if let Some(program) = &self.initial.holed_program {
            program.validate_replay(policy)?;
        }
        let mut accumulator = self.initial.to_exact_mesh(&self.z_model, policy)?;
        for (expected_index, step) in self.steps.iter().enumerate() {
            if step.index != expected_index {
                return Err(PathMeshBooleanError::Replay(
                    "retained PCB composite copper step index no longer matches order".into(),
                ));
            }
            match &step.right.holed_program {
                Some(program) => program.validate_replay(policy)?,
                None => {
                    if !matches!(step.right.source, PcbCompositeCopperBooleanSource::Solid(_)) {
                        return Err(PathMeshBooleanError::Replay(
                            "composite holed operand is missing its nested replay program".into(),
                        ));
                    }
                }
            }
            let operation = PathMeshBooleanOperation::Union.to_hypermesh();
            let right_mesh = step.right.to_exact_mesh(&self.z_model, policy)?;
            step.preflight
                .validate_against_sources(&accumulator, &right_mesh)
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            step.result
                .validate_operation_against_sources(
                    &accumulator,
                    &right_mesh,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            accumulator = step.result.mesh.clone();
        }
        Ok(())
    }
}

impl PcbCopperBoardClipOutline {
    /// Lower this retained board outline into a layer-aware clipping source.
    pub fn to_path_source(
        &self,
        z_model: &PcbLayerZModel,
        layer: TraceLayer,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        let slab = z_model.slab_for_layer(layer);
        match self {
            Self::Convex(outline) => Ok(ConvexPolygonPrism::new(
                outline.vertices().to_vec(),
                slab.z_min,
                slab.z_max,
                outline.provenance(),
                policy,
            )?
            .into()),
            Self::Orthogonal(outline) => Ok(OrthogonalPolygonPrism::new(
                outline.vertices().to_vec(),
                slab.z_min,
                slab.z_max,
                outline.provenance(),
                policy,
            )?
            .into()),
            Self::HoledOrthogonal(outline) => Ok(OrthogonalPolygonPrism::new(
                outline.outer().vertices().to_vec(),
                slab.z_min,
                slab.z_max,
                outline.outer().provenance(),
                policy,
            )?
            .into()),
            Self::ExactHandoff(outline) => outline.to_path_source(z_model, layer, policy),
        }
    }

    /// Return retained board cutout sources when this outline carries holes.
    pub fn cutouts(&self) -> Vec<PcbBoardClipCutout> {
        match self {
            Self::HoledOrthogonal(outline) => outline.cutouts(),
            _ => Vec::new(),
        }
    }
}

impl PcbCopperBoardClipProgramReport {
    /// Return the final accepted exact clipped copper mesh.
    pub fn mesh(&self) -> &ExactMesh {
        self.void_steps
            .last()
            .map(|step| &step.result.mesh)
            .unwrap_or(&self.clip_step.result.mesh)
    }

    /// Rebuild copper materialization, board lowering, and clipping evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed = build_pcb_copper_board_clip_program(
            self.sources.clone(),
            self.clip_step.outline.clone(),
            self.z_model.clone(),
            policy,
        )?;
        if replayed.net != self.net
            || replayed.layer != self.layer
            || replayed.initial != self.initial
            || replayed.union_steps != self.union_steps
            || replayed.clip_step != self.clip_step
            || replayed.void_steps != self.void_steps
        {
            return Err(PathMeshBooleanError::Replay(
                "retained PCB copper board-clip program no longer matches replay".into(),
            ));
        }
        if let Some(program) = &self.initial.holed_program {
            program.validate_replay(policy)?;
        }
        let mut accumulator = self.initial.to_exact_mesh(&self.z_model, policy)?;
        for (expected_index, step) in self.union_steps.iter().enumerate() {
            if step.index != expected_index {
                return Err(PathMeshBooleanError::Replay(
                    "retained PCB board-clip union step index no longer matches order".into(),
                ));
            }
            let right_mesh = step.right.to_exact_mesh(&self.z_model, policy)?;
            step.result
                .validate_operation_against_sources(
                    &accumulator,
                    &right_mesh,
                    PathMeshBooleanOperation::Union.to_hypermesh(),
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            accumulator = step.result.mesh.clone();
        }
        let board_source =
            self.clip_step
                .outline
                .to_path_source(&self.z_model, self.layer, policy)?;
        let board_mesh = board_source.to_exact_mesh()?;
        self.clip_step
            .result
            .validate_operation_against_sources(
                &accumulator,
                &board_mesh,
                PathMeshBooleanOperation::Intersection.to_hypermesh(),
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        accumulator = self.clip_step.result.mesh.clone();
        let slab = self.z_model.slab_for_layer(self.layer);
        for (expected_index, step) in self.void_steps.iter().enumerate() {
            if step.index != expected_index {
                return Err(PathMeshBooleanError::Replay(
                    "retained PCB board-cutout step index no longer matches order".into(),
                ));
            }
            let cutout_source = step.cutout.to_path_source(&slab, policy)?;
            let hole_mesh = cutout_source.to_exact_mesh()?;
            step.preflight
                .validate_against_sources(&accumulator, &hole_mesh)
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            step.result
                .validate_operation_against_sources(
                    &accumulator,
                    &hole_mesh,
                    PathMeshBooleanOperation::Difference.to_hypermesh(),
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
            accumulator = step.result.mesh.clone();
        }
        Ok(())
    }
}

impl PcbCopperBooleanSource {
    /// Return the source net.
    pub const fn net(&self) -> NetId {
        match self {
            Self::Trace(source) => source.net(),
            Self::RectPad(source) => source.net(),
            Self::CardinalRectPad(source) => source.net(),
            Self::ConvexPolyPad(source) => source.net(),
            Self::OrthogonalPolyPad(source) => source.net(),
            Self::ExactHandoff(source) => source.net(),
        }
    }

    /// Return the source layer.
    pub const fn layer(&self) -> TraceLayer {
        match self {
            Self::Trace(source) => source.layer(),
            Self::RectPad(source) => source.layer(),
            Self::CardinalRectPad(source) => source.layer(),
            Self::ConvexPolyPad(source) => source.layer(),
            Self::OrthogonalPolyPad(source) => source.layer(),
            Self::ExactHandoff(source) => source.layer(),
        }
    }

    /// Lower this retained PCB source into a generic path mesh-boolean source.
    pub fn to_path_source(
        &self,
        z_model: &PcbLayerZModel,
        policy: PredicatePolicy,
    ) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
        match self {
            Self::Trace(source) => pcb_trace_mesh_boolean_source(source, z_model, policy),
            Self::RectPad(source) => pcb_rect_pad_mesh_boolean_source(source, z_model, policy),
            Self::CardinalRectPad(source) => {
                pcb_cardinal_rect_pad_mesh_boolean_source(source, z_model, policy)
            }
            Self::ConvexPolyPad(source) => {
                pcb_convex_poly_pad_mesh_boolean_source(source, z_model, policy)
            }
            Self::OrthogonalPolyPad(source) => {
                pcb_orthogonal_poly_pad_mesh_boolean_source(source, z_model, policy)
            }
            Self::ExactHandoff(source) => source.to_path_source(z_model, policy),
        }
    }
}

impl PcbCopperBooleanProgramReport {
    /// Rebuild source lowering, net/layer grouping, and boolean evidence.
    pub fn validate_replay(&self, policy: PredicatePolicy) -> Result<(), PathMeshBooleanError> {
        let replayed =
            build_pcb_copper_union_program(self.sources.clone(), self.z_model.clone(), policy)?;
        if replayed.net != self.net
            || replayed.layer != self.layer
            || replayed.program != self.program
        {
            return Err(PathMeshBooleanError::Replay(
                "retained PCB copper boolean program no longer matches replay".into(),
            ));
        }
        self.program.validate_replay()
    }
}

/// Build an exact union program for retained PCB copper on one net/layer.
///
/// This is the PCB artwork companion to generic boolean programs. It rejects
/// mixed nets and mixed layers before mesh construction, because those are
/// routing semantics rather than geometric facts. The resulting boolean steps
/// are all regularized unions over exact source lowerings.
pub fn build_pcb_copper_union_program(
    sources: Vec<PcbCopperBooleanSource>,
    z_model: PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PcbCopperBooleanProgramReport, PathMeshBooleanError> {
    if sources.len() < 2 {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    let net = sources[0].net();
    let layer = sources[0].layer();
    if sources.iter().any(|source| source.net() != net) {
        return Err(PathMeshBooleanError::MixedPcbNets);
    }
    if sources.iter().any(|source| source.layer() != layer) {
        return Err(PathMeshBooleanError::MixedPcbLayers);
    }
    let mut lowered = sources
        .iter()
        .map(|source| source.to_path_source(&z_model, policy))
        .collect::<Result<Vec<_>, _>>()?;
    let initial = lowered.remove(0);
    let steps = lowered
        .into_iter()
        .map(|right| PathMeshBooleanProgramStep::new(PathMeshBooleanOperation::Union, right))
        .collect::<Vec<_>>();
    let program = boolean_path_mesh_program(initial, steps)?;
    Ok(PcbCopperBooleanProgramReport {
        net,
        layer,
        z_model,
        sources,
        program,
    })
}

/// Build an exact outer-minus-holes program for retained orthogonal copper.
///
/// This is the holed-pour companion to [`build_pcb_copper_union_program`].
/// Requicha's regularized solid difference model supplies the set-operation
/// semantics, while the retained outer/hole loops keep Yap's exact geometric
/// computation discipline visible at the API boundary.
pub fn build_pcb_holed_orthogonal_copper_program(
    source: PcbHoledOrthogonalCopperSource,
    z_model: PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PcbHoledCopperBooleanProgramReport, PathMeshBooleanError> {
    validate_holes_strictly_inside_outer(source.outer(), source.holes(), policy)?;
    let initial = pcb_orthogonal_poly_pad_mesh_boolean_source(source.outer(), &z_model, policy)?;
    let steps = source
        .holes()
        .iter()
        .map(|hole| {
            pcb_orthogonal_poly_pad_mesh_boolean_source(hole, &z_model, policy).map(|right| {
                PathMeshBooleanProgramStep::new(PathMeshBooleanOperation::Difference, right)
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let program = boolean_path_mesh_program(initial, steps)?;
    Ok(PcbHoledCopperBooleanProgramReport {
        net: source.net(),
        layer: source.layer(),
        z_model,
        source,
        program,
    })
}

/// Build an exact same-net union over solid and retained holed PCB copper.
///
/// Single holed pours are first accepted as outer-minus-holes programs. This
/// function then unions those accepted operands with ordinary solid copper,
/// preserving nested replay evidence rather than converting a holed pour into
/// an unowned mesh. The sequence is left-associative because exact mesh
/// booleans are proof-bearing operations with accumulator topology, not a
/// commutative bag rewrite.
pub fn build_pcb_composite_copper_union_program(
    sources: Vec<PcbCompositeCopperBooleanSource>,
    z_model: PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PcbCompositeCopperBooleanProgramReport, PathMeshBooleanError> {
    if sources.len() < 2 {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    let net = sources[0].net();
    let layer = sources[0].layer();
    if sources.iter().any(|source| source.net() != net) {
        return Err(PathMeshBooleanError::MixedPcbNets);
    }
    if sources.iter().any(|source| source.layer() != layer) {
        return Err(PathMeshBooleanError::MixedPcbLayers);
    }

    let mut materialized = sources
        .iter()
        .cloned()
        .map(|source| materialize_composite_copper_source(source, &z_model, policy))
        .collect::<Result<Vec<_>, _>>()?;
    let initial = materialized.remove(0);
    let mut accumulator = initial.to_exact_mesh(&z_model, policy)?;
    let mut steps = Vec::with_capacity(materialized.len());

    for (index, right) in materialized.into_iter().enumerate() {
        let right_mesh = right.to_exact_mesh(&z_model, policy)?;
        let operation = PathMeshBooleanOperation::Union.to_hypermesh();
        let preflight = preflight_boolean_exact(&accumulator, &right_mesh, operation)
            .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
        preflight
            .validate_against_sources(&accumulator, &right_mesh)
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        let result = boolean_exact_with_boundary_policy(
            &accumulator,
            &right_mesh,
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .map_err(|error| PathMeshBooleanError::Boolean(format!("{error:?}")))?;
        result
            .validate_operation_against_sources(
                &accumulator,
                &right_mesh,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        accumulator = result.mesh.clone();
        steps.push(PcbCompositeCopperBooleanStepReport {
            index,
            right,
            preflight,
            result,
        });
    }

    Ok(PcbCompositeCopperBooleanProgramReport {
        net,
        layer,
        z_model,
        sources,
        initial,
        steps,
    })
}

/// Build an exact same-net copper program clipped to a retained board outline.
///
/// Copper sources are first materialized and unioned left-to-right. The final
/// accumulator is intersected with the retained board outline extruded over the
/// same copper layer slab. This handles the common PCB manufacturing boundary
/// where copper artwork may be imported or routed beyond the board edge but
/// the accepted output topology must replay against the exact board contour.
pub fn build_pcb_copper_board_clip_program(
    sources: Vec<PcbCompositeCopperBooleanSource>,
    outline: PcbCopperBoardClipOutline,
    z_model: PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PcbCopperBoardClipProgramReport, PathMeshBooleanError> {
    if sources.is_empty() {
        return Err(PathMeshBooleanError::NotEnoughSources);
    }
    let net = sources[0].net();
    let layer = sources[0].layer();
    if sources.iter().any(|source| source.net() != net) {
        return Err(PathMeshBooleanError::MixedPcbNets);
    }
    if sources.iter().any(|source| source.layer() != layer) {
        return Err(PathMeshBooleanError::MixedPcbLayers);
    }

    let mut materialized = sources
        .iter()
        .cloned()
        .map(|source| materialize_composite_copper_source(source, &z_model, policy))
        .collect::<Result<Vec<_>, _>>()?;
    let initial = materialized.remove(0);
    let mut accumulator = initial.to_exact_mesh(&z_model, policy)?;
    let mut union_steps = Vec::with_capacity(materialized.len());

    for (index, right) in materialized.into_iter().enumerate() {
        let right_mesh = right.to_exact_mesh(&z_model, policy)?;
        let operation = PathMeshBooleanOperation::Union.to_hypermesh();
        let preflight = preflight_boolean_exact(&accumulator, &right_mesh, operation)
            .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
        preflight
            .validate_against_sources(&accumulator, &right_mesh)
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        let result = boolean_exact_with_boundary_policy(
            &accumulator,
            &right_mesh,
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .map_err(|error| PathMeshBooleanError::Boolean(format!("{error:?}")))?;
        result
            .validate_operation_against_sources(
                &accumulator,
                &right_mesh,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        accumulator = result.mesh.clone();
        union_steps.push(PcbCompositeCopperBooleanStepReport {
            index,
            right,
            preflight,
            result,
        });
    }

    let board_source = outline.to_path_source(&z_model, layer, policy)?;
    let board_mesh = board_source.to_exact_mesh()?;
    let operation = PathMeshBooleanOperation::Intersection.to_hypermesh();
    let preflight = preflight_boolean_exact(&accumulator, &board_mesh, operation)
        .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
    preflight
        .validate_against_sources(&accumulator, &board_mesh)
        .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
    let result = boolean_exact_with_boundary_policy(
        &accumulator,
        &board_mesh,
        operation,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .map_err(|error| PathMeshBooleanError::Boolean(format!("{error:?}")))?;
    result
        .validate_operation_against_sources(
            &accumulator,
            &board_mesh,
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
    let clip_step = PcbCopperBoardClipStepReport {
        outline: outline.clone(),
        preflight,
        result,
    };
    accumulator = clip_step.result.mesh.clone();

    let slab = z_model.slab_for_layer(layer);
    let cutouts = outline.cutouts();
    let mut void_steps = Vec::with_capacity(cutouts.len());
    for (index, cutout) in cutouts.into_iter().enumerate() {
        let hole_source = cutout.to_path_source(&slab, policy)?;
        let hole_mesh = hole_source.to_exact_mesh()?;
        let operation = PathMeshBooleanOperation::Difference.to_hypermesh();
        let preflight = preflight_boolean_exact(&accumulator, &hole_mesh, operation)
            .map_err(|error| PathMeshBooleanError::Preflight(format!("{error:?}")))?;
        preflight
            .validate_against_sources(&accumulator, &hole_mesh)
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        let result = boolean_exact_with_boundary_policy(
            &accumulator,
            &hole_mesh,
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .map_err(|error| PathMeshBooleanError::Boolean(format!("{error:?}")))?;
        result
            .validate_operation_against_sources(
                &accumulator,
                &hole_mesh,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .map_err(|error| PathMeshBooleanError::Replay(format!("{error:?}")))?;
        accumulator = result.mesh.clone();
        void_steps.push(PcbCopperBoardClipVoidStepReport {
            index,
            cutout,
            preflight,
            result,
        });
    }

    Ok(PcbCopperBoardClipProgramReport {
        net,
        layer,
        z_model,
        sources,
        initial,
        union_steps,
        clip_step,
        void_steps,
    })
}

/// Lower a retained PCB trace into an exact mesh-boolean source.
///
/// Only axis-aligned traces are accepted here because the existing
/// [`AxisAlignedSweptSegmentPrism`] lowering is exact without approximate
/// normal vectors. General trace/path arrangements remain separate future
/// evidence rather than being smuggled through approximate mesh construction.
pub fn pcb_trace_mesh_boolean_source(
    trace: &PcbTrace,
    z_model: &PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    let slab = z_model.slab_for_layer(trace.layer());
    Ok(
        AxisAlignedSweptSegmentPrism::new(trace.swept().clone(), slab.z_min, slab.z_max, policy)?
            .into(),
    )
}

/// Lower an axis-aligned rectangular PCB pad into an exact mesh-boolean source.
pub fn pcb_rect_pad_mesh_boolean_source(
    pad: &PcbRectPad,
    z_model: &PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    let prism = pcb_rect_pad_prism(pad, z_model, policy)?;
    Ok(prism.into())
}

/// Lower a cardinally rotated rectangular PCB pad into an exact source.
///
/// Cardinal rotations are lowered through [`PcbCardinalRectPad::effective_rect`],
/// which is an exact extent swap rather than a trigonometric rotation.
pub fn pcb_cardinal_rect_pad_mesh_boolean_source(
    pad: &PcbCardinalRectPad,
    z_model: &PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    let effective = pad
        .effective_rect()
        .map_err(|_| PathMeshBooleanError::InvalidScalar)?;
    pcb_rect_pad_mesh_boolean_source(&effective, z_model, policy)
}

/// Lower a retained strictly convex polygonal PCB pad into an exact source.
///
/// The prism triangulation is a convex fan over retained vertices. This is
/// intentionally not a general pour/zone arrangement: nonconvex and holed
/// copper need a retained planar-cell decomposition before mesh topology is
/// accepted, matching Yap's object-before-predicate guidance.
pub fn pcb_convex_poly_pad_mesh_boolean_source(
    pad: &PcbConvexPolyPad,
    z_model: &PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    let slab = z_model.slab_for_layer(pad.layer());
    Ok(ConvexPolygonPrism::new(
        pad.vertices().to_vec(),
        slab.z_min,
        slab.z_max,
        pad.provenance(),
        policy,
    )?
    .into())
}

/// Lower a retained simple orthogonal PCB pad into an exact source.
///
/// Orthogonal nonconvex copper is triangulated from retained vertices with
/// exact orientation predicates before `hypermesh` sees any topology. This
/// follows the same Yap boundary as the convex source, but uses the
/// two-ears theorem cited by [`OrthogonalPolygonPrism`] instead of a convex
/// fan. Holes and self-overlapping pours remain planar-arrangement work.
pub fn pcb_orthogonal_poly_pad_mesh_boolean_source(
    pad: &PcbOrthogonalPolyPad,
    z_model: &PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    let slab = z_model.slab_for_layer(pad.layer());
    Ok(OrthogonalPolygonPrism::new(
        pad.vertices().to_vec(),
        slab.z_min,
        slab.z_max,
        pad.provenance(),
        policy,
    )?
    .into())
}

fn pcb_orthogonal_board_outline_source(
    outline: &PcbOrthogonalBoardOutline,
    slab: &PcbLayerSlab,
    policy: PredicatePolicy,
) -> Result<PathMeshBooleanSource, PathMeshBooleanError> {
    Ok(OrthogonalPolygonPrism::new(
        outline.vertices().to_vec(),
        slab.z_min.clone(),
        slab.z_max.clone(),
        outline.provenance(),
        policy,
    )?
    .into())
}

/// Lower an axis-aligned rectangular PCB pad into an exact rectangular prism.
pub fn pcb_rect_pad_prism(
    pad: &PcbRectPad,
    z_model: &PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<RectangularPrism, PathMeshBooleanError> {
    if compare_reals_with_policy(pad.width(), &Real::zero(), policy).value()
        != Some(Ordering::Greater)
        || compare_reals_with_policy(pad.height(), &Real::zero(), policy).value()
            != Some(Ordering::Greater)
    {
        return Err(PathMeshBooleanError::NonPositivePadExtent);
    }
    let half_width =
        (pad.width().clone() / Real::from(2)).map_err(|_| PathMeshBooleanError::InvalidScalar)?;
    let half_height =
        (pad.height().clone() / Real::from(2)).map_err(|_| PathMeshBooleanError::InvalidScalar)?;
    let min = Point2::new(
        pad.center().x.clone() - half_width.clone(),
        pad.center().y.clone() - half_height.clone(),
    );
    let max = Point2::new(
        pad.center().x.clone() + half_width,
        pad.center().y.clone() + half_height,
    );
    let footprint = RectangularPocket::with_provenance(min, max, pad.provenance()).map_err(
        |error| match error {
            PocketPlanError::UnorderedBounds => PathMeshBooleanError::DegenerateFootprint,
            _ => PathMeshBooleanError::InvalidScalar,
        },
    )?;
    let slab = z_model.slab_for_layer(pad.layer());
    RectangularPrism::new(footprint, slab.z_min, slab.z_max, policy)
}

fn path_error_from_board_contour_error(error: BoardContourError) -> PathMeshBooleanError {
    match error {
        BoardContourError::TooFewVertices => PathMeshBooleanError::TooFewPolygonVertices,
        BoardContourError::DegenerateArea | BoardContourError::CollinearEdge => {
            PathMeshBooleanError::DegeneratePolygon
        }
        BoardContourError::UnknownOrientation => PathMeshBooleanError::UnknownPolygonOrientation,
        BoardContourError::NonConvex | BoardContourError::NonOrthogonal => {
            PathMeshBooleanError::NonConvexPolygon
        }
        BoardContourError::SelfIntersecting => PathMeshBooleanError::PolygonTriangulationFailed,
    }
}

fn holed_orthogonal_exact_facts(
    outer: &PcbOrthogonalPolyPad,
    holes: &[PcbOrthogonalPolyPad],
) -> RealExactSetFacts {
    let mut values = outer
        .vertices()
        .iter()
        .flat_map(|point| [&point.x, &point.y])
        .collect::<Vec<_>>();
    for hole in holes {
        values.extend(
            hole.vertices()
                .iter()
                .flat_map(|point| [&point.x, &point.y]),
        );
    }
    Real::exact_set_facts(values)
}

fn holed_board_exact_facts(
    outer: &PcbOrthogonalBoardOutline,
    holes: &[PcbOrthogonalBoardOutline],
    exact_cutouts: &[PcbExactBoardCutoutHandoff],
) -> RealExactSetFacts {
    let mut values = Vec::new();
    let mut refs = outer
        .vertices()
        .iter()
        .flat_map(|point| [&point.x, &point.y])
        .collect::<Vec<_>>();
    for hole in holes {
        refs.extend(
            hole.vertices()
                .iter()
                .flat_map(|point| [&point.x, &point.y]),
        );
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

fn materialize_composite_copper_source(
    source: PcbCompositeCopperBooleanSource,
    z_model: &PcbLayerZModel,
    policy: PredicatePolicy,
) -> Result<PcbCompositeCopperMaterialization, PathMeshBooleanError> {
    let holed_program = match &source {
        PcbCompositeCopperBooleanSource::Solid(_) => None,
        PcbCompositeCopperBooleanSource::HoledOrthogonal(source) => Some(
            build_pcb_holed_orthogonal_copper_program(source.clone(), z_model.clone(), policy)?,
        ),
    };
    Ok(PcbCompositeCopperMaterialization {
        source,
        holed_program,
    })
}

fn validate_handoff_layer_slab(
    handoff: &PathExactMeshHandoffSource,
    slab: &PcbLayerSlab,
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    let mesh = handoff.to_exact_mesh()?;
    let Some(bounds) = &mesh.bounds().mesh else {
        return Err(PathMeshBooleanError::MeshHandoff(
            "PCB exact handoff source has no mesh bounds".into(),
        ));
    };
    if compare_reals_with_policy(&bounds.min.z, &slab.z_min, policy).value()
        != Some(Ordering::Equal)
        || compare_reals_with_policy(&bounds.max.z, &slab.z_max, policy).value()
            != Some(Ordering::Equal)
    {
        return Err(PathMeshBooleanError::MeshHandoff(
            "PCB exact handoff source does not match the requested layer slab".into(),
        ));
    }
    Ok(())
}

fn validate_holes_strictly_inside_outer(
    outer: &PcbOrthogonalPolyPad,
    holes: &[PcbOrthogonalPolyPad],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    let hole_vertices = holes
        .iter()
        .map(|hole| hole.vertices().to_vec())
        .collect::<Vec<_>>();
    validate_strict_orthogonal_holes(outer.vertices(), &hole_vertices, policy)
}

fn validate_board_cutouts_strictly_inside_outer(
    outer: &PcbOrthogonalBoardOutline,
    holes: &[PcbOrthogonalBoardOutline],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    let hole_vertices = holes
        .iter()
        .map(|hole| hole.vertices().to_vec())
        .collect::<Vec<_>>();
    validate_strict_orthogonal_holes(outer.vertices(), &hole_vertices, policy)
}
