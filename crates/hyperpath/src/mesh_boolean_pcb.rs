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
use hyperreal::{Real, RealExactSetFacts};

use crate::cam::{PocketPlanError, RectangularPocket};
use crate::mesh_boolean::{PathMeshBooleanError, PathMeshBooleanOperation, RectangularPrism};
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
        validate_holes_pairwise_disjoint(&holes, policy)?;
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
    validate_holes_pairwise_disjoint(source.holes(), policy)?;
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

fn validate_holes_strictly_inside_outer(
    outer: &PcbOrthogonalPolyPad,
    holes: &[PcbOrthogonalPolyPad],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    if holes.is_empty() {
        return Err(PathMeshBooleanError::EmptyPolygonHoles);
    }
    for hole in holes {
        for point in hole.vertices() {
            if classify_point_in_orthogonal_loop(point, outer.vertices(), policy)?
                != OrthogonalLoopPointLocation::Inside
            {
                return Err(PathMeshBooleanError::PolygonHoleOutsideOuter);
            }
        }
        if loops_have_edge_intersection(outer.vertices(), hole.vertices(), policy)? {
            return Err(PathMeshBooleanError::PolygonHoleOutsideOuter);
        }
    }
    Ok(())
}

fn validate_holes_pairwise_disjoint(
    holes: &[PcbOrthogonalPolyPad],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    for left in 0..holes.len() {
        for right in left + 1..holes.len() {
            if loops_have_edge_intersection(
                holes[left].vertices(),
                holes[right].vertices(),
                policy,
            )? {
                return Err(PathMeshBooleanError::PolygonHoleOverlap);
            }
            if holes[left].vertices().iter().any(|point| {
                classify_point_in_orthogonal_loop(point, holes[right].vertices(), policy)
                    == Ok(OrthogonalLoopPointLocation::Inside)
            }) || holes[right].vertices().iter().any(|point| {
                classify_point_in_orthogonal_loop(point, holes[left].vertices(), policy)
                    == Ok(OrthogonalLoopPointLocation::Inside)
            }) {
                return Err(PathMeshBooleanError::PolygonHoleOverlap);
            }
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OrthogonalLoopPointLocation {
    Inside,
    Boundary,
    Outside,
}

fn classify_point_in_orthogonal_loop(
    point: &Point2,
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Result<OrthogonalLoopPointLocation, PathMeshBooleanError> {
    let mut inside = false;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        if point_on_axis_aligned_segment(point, start, end, policy)? {
            return Ok(OrthogonalLoopPointLocation::Boundary);
        }
        if compare_reals_with_policy(&start.x, &end.x, policy).value() != Some(Ordering::Equal) {
            continue;
        }
        let y_min = real_min(&start.y, &end.y, policy)?;
        let y_max = real_max(&start.y, &end.y, policy)?;
        let crosses_lower =
            compare_reals_with_policy(&point.y, y_min, policy).value() != Some(Ordering::Less);
        let crosses_upper =
            compare_reals_with_policy(&point.y, y_max, policy).value() == Some(Ordering::Less);
        let right_of_point = compare_reals_with_policy(&start.x, &point.x, policy).value()
            == Some(Ordering::Greater);
        if crosses_lower && crosses_upper && right_of_point {
            inside = !inside;
        }
    }
    Ok(if inside {
        OrthogonalLoopPointLocation::Inside
    } else {
        OrthogonalLoopPointLocation::Outside
    })
}

fn loops_have_edge_intersection(
    left: &[Point2],
    right: &[Point2],
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    for left_index in 0..left.len() {
        let left_next = (left_index + 1) % left.len();
        for right_index in 0..right.len() {
            let right_next = (right_index + 1) % right.len();
            if orthogonal_segments_intersect(
                &left[left_index],
                &left[left_next],
                &right[right_index],
                &right[right_next],
                policy,
            )? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn point_on_axis_aligned_segment(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let same_x = compare_reals_with_policy(&start.x, &end.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    let same_y = compare_reals_with_policy(&start.y, &end.y, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    if same_x {
        let point_same_x = compare_reals_with_policy(&point.x, &start.x, policy)
            .value()
            .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
            == Ordering::Equal;
        return Ok(point_same_x && interval_contains_point(&start.y, &end.y, &point.y, policy)?);
    }
    if same_y {
        let point_same_y = compare_reals_with_policy(&point.y, &start.y, policy)
            .value()
            .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
            == Ordering::Equal;
        return Ok(point_same_y && interval_contains_point(&start.x, &end.x, &point.x, policy)?);
    }
    Ok(false)
}

fn orthogonal_segments_intersect(
    a0: &Point2,
    a1: &Point2,
    b0: &Point2,
    b1: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let a_vertical = compare_reals_with_policy(&a0.x, &a1.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    let b_vertical = compare_reals_with_policy(&b0.x, &b1.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    match (a_vertical, b_vertical) {
        (true, true) => Ok(real_equal(&a0.x, &b0.x, policy)?
            && ranges_overlap(&a0.y, &a1.y, &b0.y, &b1.y, policy)?),
        (false, false) => Ok(real_equal(&a0.y, &b0.y, policy)?
            && ranges_overlap(&a0.x, &a1.x, &b0.x, &b1.x, policy)?),
        (true, false) => Ok(real_between_closed(&a0.x, &b0.x, &b1.x, policy)?
            && real_between_closed(&b0.y, &a0.y, &a1.y, policy)?),
        (false, true) => Ok(real_between_closed(&b0.x, &a0.x, &a1.x, policy)?
            && real_between_closed(&a0.y, &b0.y, &b1.y, policy)?),
    }
}

fn interval_contains_point(
    a0: &Real,
    a1: &Real,
    point: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let min = real_min(a0, a1, policy)?;
    let max = real_max(a0, a1, policy)?;
    Ok(
        compare_reals_with_policy(point, min, policy).value() != Some(Ordering::Less)
            && compare_reals_with_policy(point, max, policy).value() != Some(Ordering::Greater),
    )
}

fn ranges_overlap(
    a0: &Real,
    a1: &Real,
    b0: &Real,
    b1: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let a_min = real_min(a0, a1, policy)?;
    let a_max = real_max(a0, a1, policy)?;
    let b_min = real_min(b0, b1, policy)?;
    let b_max = real_max(b0, b1, policy)?;
    Ok(
        compare_reals_with_policy(a_min, b_max, policy).value() != Some(Ordering::Greater)
            && compare_reals_with_policy(b_min, a_max, policy).value() != Some(Ordering::Greater),
    )
}

fn real_between_closed(
    value: &Real,
    a: &Real,
    b: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let min = real_min(a, b, policy)?;
    let max = real_max(a, b, policy)?;
    Ok(
        compare_reals_with_policy(min, value, policy).value() != Some(Ordering::Greater)
            && compare_reals_with_policy(value, max, policy).value() != Some(Ordering::Greater),
    )
}

fn real_min<'a>(
    a: &'a Real,
    b: &'a Real,
    policy: PredicatePolicy,
) -> Result<&'a Real, PathMeshBooleanError> {
    match compare_reals_with_policy(a, b, policy).value() {
        Some(Ordering::Less | Ordering::Equal) => Ok(a),
        Some(Ordering::Greater) => Ok(b),
        None => Err(PathMeshBooleanError::UnknownPolygonOrientation),
    }
}

fn real_max<'a>(
    a: &'a Real,
    b: &'a Real,
    policy: PredicatePolicy,
) -> Result<&'a Real, PathMeshBooleanError> {
    match compare_reals_with_policy(a, b, policy).value() {
        Some(Ordering::Greater | Ordering::Equal) => Ok(a),
        Some(Ordering::Less) => Ok(b),
        None => Err(PathMeshBooleanError::UnknownPolygonOrientation),
    }
}

fn real_equal(a: &Real, b: &Real, policy: PredicatePolicy) -> Result<bool, PathMeshBooleanError> {
    compare_reals_with_policy(a, b, policy)
        .value()
        .map(|ordering| ordering == Ordering::Equal)
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)
}
