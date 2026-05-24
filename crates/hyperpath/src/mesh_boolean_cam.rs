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

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealExactSetFacts};

use crate::cam::{PocketPlanError, RectangularPocket};
use crate::mesh_boolean::{PathMeshBooleanError, PathMeshBooleanOperation, RectangularPrism};
use crate::mesh_boolean_program::{
    PathMeshBooleanProgramReport, PathMeshBooleanProgramStep, boolean_path_mesh_program,
};
use crate::mesh_boolean_sources::{AxisAlignedSweptSegmentPrism, PathMeshBooleanSource};
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
    /// Retained cutter sources in subtraction order.
    pub cutters: Vec<CamRestMaterialCutter>,
    /// Exact-set facts for retained stock, cutter, and Z scalar inputs.
    pub exact: RealExactSetFacts,
    /// Accepted exact mesh-boolean program.
    pub program: PathMeshBooleanProgramReport,
}

impl CamRestMaterialCutter {
    /// Lower this retained cutter into a generic path mesh-boolean source.
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

/// Build an exact rest-material difference program for rectangular stock.
///
/// Every cutter is subtracted from the current accumulator in the provided
/// order. The stock and all cutters share one exact Z interval in this bounded
/// API, which matches 2.5D rectangular pocket/rest fixtures without pretending
/// to solve general 5-axis swept volumes.
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
    let steps = cutters
        .iter()
        .map(|cutter| {
            cutter
                .to_path_source(z_min.clone(), z_max.clone(), policy)
                .map(|right| {
                    PathMeshBooleanProgramStep::new(PathMeshBooleanOperation::Difference, right)
                })
        })
        .collect::<Result<Vec<_>, _>>()?;
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
        }
    }
    Real::exact_set_facts(values)
}
