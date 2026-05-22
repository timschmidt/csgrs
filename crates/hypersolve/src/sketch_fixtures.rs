//! License-clean sketch compatibility fixtures.
//!
//! These fixtures are Hyper-authored reference scenarios for the
//! SolveSpace-level semantic surface. They intentionally do not copy
//! SolveSpaceLib implementation code or fixture files. Instead, each fixture
//! builds a [`SketchSolveProblem`] through the public Hyper API, lowers it to a
//! generic residual [`crate::Problem`], and certifies the candidate through
//! exact replay. That keeps Yap's proof boundary intact: external solvers may
//! later be compared as proposal engines, but exact Hyper replay is the oracle.
//! See Yap, "Towards Exact Geometric Computation" (1997), and the public
//! SolveSpaceLib `slvs.h` vocabulary for the behavioral coverage target.

use crate::certification::{CandidateCertificationReport, certify_candidate};
use crate::eval::context_from_problem;
use crate::prepared::PreparedProblem;
use crate::sketch::{
    SketchEntityKind, SketchGeneratedRowStatus, SketchLoweringReport, SketchSolveProblem,
};
use crate::sketch_builders::{distance, incidence, orientation, symmetry};

/// Compatibility fixture family.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchCompatibilityFixtureKind {
    /// Retains every currently modeled public entity carrier without requiring
    /// residual rows.
    EntityCoverage,
    /// Covers current 2D constraint builders and exact replay.
    ConstraintCoverage2D,
    /// Covers free-3D point coincidence lowering and exact replay.
    Free3DPointCoincidence,
    /// Covers retained 3D workplane symmetry lowering and exact replay.
    WorkplaneSymmetry3D,
}

/// One license-clean sketch compatibility fixture.
#[derive(Clone, Debug)]
pub struct SketchCompatibilityFixture {
    /// Stable fixture name.
    pub name: &'static str,
    /// Fixture family.
    pub kind: SketchCompatibilityFixtureKind,
    /// Human-readable source/reference label.
    pub source: &'static str,
    /// Sketch model built through public APIs.
    pub sketch: SketchSolveProblem,
    /// Expected generated residual rows after lowering.
    pub expected_generated_rows: usize,
}

/// Exact replay report for a compatibility fixture.
#[derive(Clone, Debug)]
pub struct SketchCompatibilityReplayReport {
    /// Fixture name.
    pub name: &'static str,
    /// Fixture family.
    pub kind: SketchCompatibilityFixtureKind,
    /// Lowering report.
    pub lowering: SketchLoweringReport,
    /// Exact candidate certification report.
    pub certification: CandidateCertificationReport,
}

impl SketchCompatibilityReplayReport {
    /// Returns true when lowering generated every expected row and exact replay
    /// accepted all active residuals.
    pub fn is_certified_fixture(&self, expected_generated_rows: usize) -> bool {
        self.lowering.rows.len() == expected_generated_rows
            && self
                .lowering
                .rows
                .iter()
                .all(|row| row.status == SketchGeneratedRowStatus::Generated)
            && if expected_generated_rows == 0 {
                self.certification.rows.is_empty()
            } else {
                self.certification.all_satisfied()
            }
    }
}

/// Return the current built-in sketch compatibility fixture catalog.
pub fn sketch_compatibility_fixtures() -> Vec<SketchCompatibilityFixture> {
    vec![
        entity_coverage_fixture(),
        constraint_coverage_2d_fixture(),
        free_3d_point_coincidence_fixture(),
        workplane_symmetry_3d_fixture(),
    ]
}

/// Lower and exactly replay one compatibility fixture.
pub fn replay_sketch_compatibility_fixture(
    fixture: &SketchCompatibilityFixture,
) -> SketchCompatibilityReplayReport {
    let lowering = fixture.sketch.lower_to_problem();
    let certification = {
        let prepared = PreparedProblem::new(&lowering.problem);
        let context = context_from_problem(&lowering.problem);
        certify_candidate(&prepared, &context)
    };
    SketchCompatibilityReplayReport {
        name: fixture.name,
        kind: fixture.kind,
        lowering,
        certification,
    }
}

fn entity_coverage_fixture() -> SketchCompatibilityFixture {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", 0.into(), 0.into(), 0.into());
    let normal = sketch.add_normal3d("normal", 1.into(), 0.into(), 0.into(), 0.into());
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let p0 = sketch.add_point2d("p0", 0.into(), 0.into());
    let p1 = sketch.add_point2d("p1", 1.into(), 0.into());
    let p2 = sketch.add_point2d("p2", 2.into(), 1.into());
    let p3 = sketch.add_point2d("p3", 3.into(), 1.into());
    let radius = sketch.add_distance("radius", 1.into());
    let line = sketch.add_line_segment2("line", p0, p1);
    let circle = sketch.add_circle2("circle", p0, radius);
    let arc = sketch.add_arc_of_circle2("arc", p0, p1, p2, radius);
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let normal2 = sketch.add_normal2d("direction", 1.into(), 0.into());

    debug_assert!(matches!(
        &sketch.entities()[workplane.0 as usize].kind,
        SketchEntityKind::Workplane(_)
    ));
    debug_assert!(matches!(
        &sketch.entities()[line.0 as usize].kind,
        SketchEntityKind::LineSegment2(_)
    ));
    debug_assert!(matches!(
        &sketch.entities()[circle.0 as usize].kind,
        SketchEntityKind::Circle2(_)
    ));
    debug_assert!(matches!(
        &sketch.entities()[arc.0 as usize].kind,
        SketchEntityKind::ArcOfCircle2(_)
    ));
    debug_assert!(matches!(
        &sketch.entities()[cubic.0 as usize].kind,
        SketchEntityKind::Cubic2(_)
    ));
    debug_assert!(matches!(
        &sketch.entities()[normal2.0 as usize].kind,
        SketchEntityKind::Normal2D(_)
    ));

    SketchCompatibilityFixture {
        name: "entity_coverage",
        kind: SketchCompatibilityFixtureKind::EntityCoverage,
        source: "Hyper-authored SolveSpaceLib public entity vocabulary fixture",
        sketch,
        expected_generated_rows: 0,
    }
}

fn constraint_coverage_2d_fixture() -> SketchCompatibilityFixture {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", 0.into(), 0.into());
    let on_x = sketch.add_point2d("on_x", 5.into(), 0.into());
    let on_y = sketch.add_point2d("on_y", 0.into(), 5.into());
    let radius = sketch.add_distance("five", 5.into());
    let horizontal = sketch.add_line_segment2("horizontal", origin, on_x);
    let vertical = sketch.add_line_segment2("vertical", origin, on_y);
    let circle = sketch.add_circle2("circle", origin, radius);

    incidence::points_coincident(&mut sketch, "origin coincident", origin, origin);
    distance::point_point_distance(&mut sketch, "horizontal length", origin, on_x, radius);
    orientation::horizontal(&mut sketch, "horizontal line", horizontal);
    orientation::vertical(&mut sketch, "vertical line", vertical);
    incidence::point_on_circle(&mut sketch, "point on circle", on_x, circle);

    SketchCompatibilityFixture {
        name: "constraint_coverage_2d",
        kind: SketchCompatibilityFixtureKind::ConstraintCoverage2D,
        source: "Hyper-authored 2D SolveSpace-style constraint fixture",
        sketch,
        expected_generated_rows: 6,
    }
}

fn free_3d_point_coincidence_fixture() -> SketchCompatibilityFixture {
    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point3d("a", 1.into(), 2.into(), 3.into());
    let b = sketch.add_point3d("b", 1.into(), 2.into(), 3.into());
    incidence::points_coincident(&mut sketch, "free 3d coincidence", a, b);

    SketchCompatibilityFixture {
        name: "free_3d_point_coincidence",
        kind: SketchCompatibilityFixtureKind::Free3DPointCoincidence,
        source: "Hyper-authored free-3D point coincidence fixture",
        sketch,
        expected_generated_rows: 3,
    }
}

fn workplane_symmetry_3d_fixture() -> SketchCompatibilityFixture {
    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", 0.into(), 0.into(), 0.into());
    let normal = sketch.add_normal3d("normal", 1.into(), 0.into(), 0.into(), 0.into());
    let workplane = sketch.add_workplane("xy", origin, normal);
    let top = sketch.add_point3d("top", 2.into(), 3.into(), 5.into());
    let bottom = sketch.add_point3d("bottom", 2.into(), 3.into(), (-5).into());
    symmetry::symmetric_workplane3(&mut sketch, "xy workplane symmetry", top, bottom, workplane);

    SketchCompatibilityFixture {
        name: "workplane_symmetry_3d",
        kind: SketchCompatibilityFixtureKind::WorkplaneSymmetry3D,
        source: "Hyper-authored 3D workplane symmetry fixture",
        sketch,
        expected_generated_rows: 5,
    }
}
