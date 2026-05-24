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
use crate::mesh_boolean_holes::validate_strict_orthogonal_holes;
use crate::mesh_boolean_polygon::{ConvexPolygonPrism, OrthogonalPolygonPrism};
use crate::mesh_boolean_program::{
    PathMeshBooleanProgramReport, PathMeshBooleanProgramStep, boolean_path_mesh_program,
};
use crate::mesh_boolean_sources::{AxisAlignedSweptSegmentPrism, PathMeshBooleanSource};
use crate::pcb::{
    BoardContourError, NetId, PcbCardinalRectPad, PcbConvexPolyPad, PcbOrthogonalPolyPad,
    PcbRectPad, PcbTrace, TraceLayer,
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

/// Retained PCB copper source supported by the exact mesh-boolean handoff.
///
/// This enum keeps PCB semantics attached until the final lowering step. It is
/// intentionally narrower than all PCB geometry: circular/rounded pads and
/// holed copper pours need curved or arrangement evidence before they can
/// become mesh-boolean operands without approximation.
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

impl PcbCopperBooleanSource {
    /// Return the source net.
    pub const fn net(&self) -> NetId {
        match self {
            Self::Trace(source) => source.net(),
            Self::RectPad(source) => source.net(),
            Self::CardinalRectPad(source) => source.net(),
            Self::ConvexPolyPad(source) => source.net(),
            Self::OrthogonalPolyPad(source) => source.net(),
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
