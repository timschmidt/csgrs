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
    NetId, PcbCardinalRectPad, PcbConvexPolyPad, PcbOrthogonalPolyPad, PcbRectPad, PcbTrace,
    TraceLayer,
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
