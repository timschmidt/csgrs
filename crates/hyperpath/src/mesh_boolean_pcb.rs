//! PCB-specific retained sources for exact mesh booleans.
//!
//! PCB routing objects are not generic boxes: they carry net, layer, source
//! provenance, and exact swept geometry. This module lowers the subset that is
//! already exact in `hyperpath`--axis-aligned traces and rectangular/cardinal
//! rectangular pads--into retained mesh-boolean sources. The lowering is
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
use crate::mesh_boolean::{PathMeshBooleanError, RectangularPrism};
use crate::mesh_boolean_sources::{AxisAlignedSweptSegmentPrism, PathMeshBooleanSource};
use crate::pcb::{PcbCardinalRectPad, PcbRectPad, PcbTrace, TraceLayer};

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
