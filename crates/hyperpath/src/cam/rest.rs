//! Exact retained rest-material graph for rectangular CAM stock.
//!
//! Rest machining is a material-domain question, but `hyperpath` should not
//! hide that question inside tolerance-driven booleans. This module retains
//! exact stock/cutter records, emits the exact
//! stage schedule, and replays area-conservation predicates before a later
//! `hypermesh` or process planner accepts material removal. The separation is
//! also consistent with contour-parallel pocketing discussions such as Held,
//! "On the Computational Geometry of Pocket Machining" (1991): offset/link
//! candidates and rest-material state are distinct algorithmic objects.

use hypersolve::{
    CandidateCertificationReport, Constraint, Expr, Problem, certify_candidate,
    context_from_problem,
};

use crate::cam::{
    RectangularPocket, RectangularRegionDifference, RegionBooleanError, subtract_rectangular_region,
};
use crate::solve::RectangularRegion;
use hyperlimit::PredicatePolicy;
use hyperreal::Real;

/// One exact subtraction of one active stock piece by one rectangular cutter.
///
/// This is a retained source record, not a merged polygon. `removed` is the
/// positive-area intersection that was actually consumed from `subject`;
/// touching or disjoint cuts leave it `None` and keep the subject in
/// `remainder`.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularRestCutRecord {
    /// Zero-based active-piece index at the start of the stage.
    pub subject_index: usize,
    /// Exact subtraction report for that active piece and cutter.
    pub difference: RectangularRegionDifference,
    /// Positive-area material removed from this subject by the cutter.
    pub removed: Option<RectangularPocket>,
}

/// One retained rest-material update caused by a single rectangular cutter.
///
/// The stage keeps both the before/after active stock sets and per-subject
/// subtraction records. Its `area_certification` replays the exact invariant
/// `area(before) = area(removed) + area(after)` with constant exact
/// `hypersolve` rows rather than trusting the construction procedure.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularRestMaterialStage {
    /// Zero-based cutter index in the input sequence.
    pub cutter_index: usize,
    /// Exact cutter applied at this stage.
    pub cutter: RectangularPocket,
    /// Active rest-material pieces before the cutter is applied.
    pub before: Vec<RectangularPocket>,
    /// One retained subtraction record per `before` piece.
    pub cuts: Vec<RectangularRestCutRecord>,
    /// Active rest-material pieces after the cutter is applied.
    pub after: Vec<RectangularPocket>,
    /// Exact area-conservation certification for this stage.
    pub area_certification: CandidateCertificationReport,
}

/// Retained exact rest-material graph for rectangular stock and cutters.
///
/// `remaining` is the final positive-area rectangular cover after applying the
/// cutters in input order. The graph deliberately does not canonicalize that
/// cover into an arbitrary polygon, because source identity and exact replay
/// are the useful handoff to downstream arrangement, linking, and stock
/// materialization stages.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularRestMaterialGraph {
    /// Original exact stock rectangle.
    pub stock: RectangularPocket,
    /// Exact cutter rectangles in application order.
    pub cutters: Vec<RectangularPocket>,
    /// Per-cutter retained material updates.
    pub stages: Vec<RectangularRestMaterialStage>,
    /// Final positive-area rest-material cover.
    pub remaining: Vec<RectangularPocket>,
    /// Exact total area-conservation certification over the whole graph.
    pub total_area_certification: CandidateCertificationReport,
}

impl RectangularRestMaterialGraph {
    /// Return true when every retained area replay row is exactly satisfied.
    pub fn all_area_certified(&self) -> bool {
        self.total_area_certification.all_satisfied()
            && self
                .stages
                .iter()
                .all(|stage| stage.area_certification.all_satisfied())
    }

    /// Return the exact total area removed by all positive-area cut records.
    pub fn removed_area(&self) -> Real {
        self.stages
            .iter()
            .flat_map(|stage| stage.cuts.iter())
            .filter_map(|cut| cut.removed.as_ref())
            .fold(Real::zero(), |sum, region| sum + area(region))
    }

    /// Return the exact area of the final rest-material cover.
    pub fn remaining_area(&self) -> Real {
        self.remaining
            .iter()
            .fold(Real::zero(), |sum, region| sum + area(region))
    }
}

/// Errors while constructing retained rectangular rest-material graphs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RectangularRestMaterialError {
    /// At least one cutter must be supplied so a rest-material schedule exists.
    EmptyCutterSet,
    /// A rectangular subtraction predicate could not be certified.
    RegionBoolean(RegionBooleanError),
    /// A stage or total area replay row failed exact certification.
    AreaCertificationFailed,
}

/// Build an exact retained rest-material graph for rectangular stock.
///
/// The builder applies cutters in deterministic input order. Each cutter is
/// subtracted from every active positive-area rest piece; all positive-area
/// remainders are retained for the next stage. Every stage and the final graph
/// then receive exact area-conservation replay rows. This gives CAM and
/// `hypermesh` callers a material-source graph without pretending that
/// `hyperpath` has already performed general polygon booleans, cutter
/// engagement validation, or linking.
pub fn build_rectangular_rest_material_graph(
    stock: RectangularPocket,
    cutters: impl IntoIterator<Item = RectangularPocket>,
    policy: PredicatePolicy,
) -> Result<RectangularRestMaterialGraph, RectangularRestMaterialError> {
    let cutters: Vec<_> = cutters.into_iter().collect();
    if cutters.is_empty() {
        return Err(RectangularRestMaterialError::EmptyCutterSet);
    }

    let mut stages = Vec::with_capacity(cutters.len());
    let mut active = vec![stock.clone()];
    for (cutter_index, cutter) in cutters.iter().cloned().enumerate() {
        let before = active.clone();
        let mut cuts = Vec::with_capacity(before.len());
        let mut after = Vec::new();
        for (subject_index, subject) in before.iter().cloned().enumerate() {
            let difference = subtract_rectangular_region(subject, cutter.clone(), policy)
                .map_err(RectangularRestMaterialError::RegionBoolean)?;
            let removed = difference.intersection.clone().filter(|_| {
                difference.relation == crate::cam::RectangularRegionRelation::AreaOverlap
            });
            after.extend(difference.remainder.iter().cloned());
            cuts.push(RectangularRestCutRecord {
                subject_index,
                difference,
                removed,
            });
        }
        let area_certification = certify_area_balance(
            format!("rectangular rest stage {cutter_index} area"),
            before.iter(),
            cuts.iter().filter_map(|cut| cut.removed.as_ref()),
            after.iter(),
        );
        if !area_certification.all_satisfied() {
            return Err(RectangularRestMaterialError::AreaCertificationFailed);
        }
        stages.push(RectangularRestMaterialStage {
            cutter_index,
            cutter,
            before,
            cuts,
            after: after.clone(),
            area_certification,
        });
        active = after;
    }

    let total_area_certification = certify_area_balance(
        "rectangular rest total area",
        std::iter::once(&stock),
        stages
            .iter()
            .flat_map(|stage| stage.cuts.iter())
            .filter_map(|cut| cut.removed.as_ref()),
        active.iter(),
    );
    if !total_area_certification.all_satisfied() {
        return Err(RectangularRestMaterialError::AreaCertificationFailed);
    }

    Ok(RectangularRestMaterialGraph {
        stock,
        cutters,
        stages,
        remaining: active,
        total_area_certification,
    })
}

fn certify_area_balance<'a>(
    name: impl Into<String>,
    before: impl IntoIterator<Item = &'a RectangularPocket>,
    removed: impl IntoIterator<Item = &'a RectangularPocket>,
    after: impl IntoIterator<Item = &'a RectangularPocket>,
) -> CandidateCertificationReport {
    let mut residual = sum_area_expr(before);
    residual = residual - sum_area_expr(removed);
    residual = residual - sum_area_expr(after);
    let mut problem = Problem::default();
    problem.add_constraint(Constraint::equality(name, residual));
    let analysis = problem.analyze();
    let context = context_from_problem(&problem);
    certify_candidate(&analysis, &context)
}

fn sum_area_expr<'a>(regions: impl IntoIterator<Item = &'a RectangularPocket>) -> Expr {
    regions
        .into_iter()
        .map(to_solver_region)
        .fold(Expr::real(Real::zero()), |sum, region| {
            sum + region.area_expr()
        })
}

fn to_solver_region(region: &RectangularPocket) -> RectangularRegion {
    RectangularRegion::new(region.min().clone(), region.max().clone())
}

fn area(region: &RectangularPocket) -> Real {
    region.width() * region.height()
}
