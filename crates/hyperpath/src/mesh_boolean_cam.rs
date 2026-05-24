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

use crate::cam::{PocketPlanError, RectangularPocket};
use crate::mesh_boolean::{PathMeshBooleanError, PathMeshBooleanOperation, RectangularPrism};
use crate::mesh_boolean_holes::validate_strict_orthogonal_holes;
use crate::mesh_boolean_polygon::OrthogonalPolygonPrism;
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
