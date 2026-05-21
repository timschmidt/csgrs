//! Typed sketch constraint builders grouped by geometric relation family.
//!
//! These builders are a semantic layer over [`SketchSolveProblem`]. They record
//! whether a relation is incidence, distance, or orientation before lowering to
//! generic residuals, so exact replay can later choose the correct proof
//! package. This follows Yap's "Towards Exact Geometric Computation" (1997):
//! the builder creates proof obligations, but certification remains the trust
//! boundary. The family split mirrors geometric-constraint-solver practice as
//! surveyed by Bouma et al., "Geometric Constraint Solver" (1995), while the
//! public entity names are SolveSpaceLib-compatible reference vocabulary.

use crate::sketch::{
    SketchConstraintHandle, SketchConstraintKind, SketchEntityHandle, SketchResidualStrategy,
    SketchSolveProblem,
};

/// High-level constraint family retained before residual lowering.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchConstraintFamily {
    /// Incidence or positional relation, such as coincident points.
    Incidence,
    /// Dimensional distance relation.
    Distance,
    /// Orientation relation, such as horizontal or vertical.
    Orientation,
}

/// Report emitted when a typed sketch constraint builder records a relation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchConstraintBuildReport {
    /// Handle assigned to the retained high-level constraint.
    pub handle: SketchConstraintHandle,
    /// Semantic family used for diagnostics and later proof selection.
    pub family: SketchConstraintFamily,
    /// Expected residual strategy after lowering.
    pub strategy: SketchResidualStrategy,
    /// Retained high-level constraint payload.
    pub kind: SketchConstraintKind,
}

/// Incidence and positional relation builders.
pub mod incidence {
    use super::*;

    /// Add a point coincidence relation.
    ///
    /// The residual builder keeps each coordinate equality as a separate exact
    /// row. This is the same construction/proof separation emphasized by Yap
    /// (1997): generated equations are not trusted until ordinary exact
    /// residual replay certifies them.
    pub fn points_coincident(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointsCoincident { a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::CoordinateEquality,
            kind,
        }
    }

    /// Add a point-on-circle relation represented by squared incidence.
    ///
    /// The squared-radius residual avoids introducing a square root into the
    /// proof obligation; nonnegative-radius policy remains an explicit domain
    /// or sketch constraint instead of a hidden tolerance test.
    pub fn point_on_circle(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        point: SketchEntityHandle,
        circle: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointOnCircle { point, circle };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::SquaredIncidence,
            kind,
        }
    }
}

/// Dimensional relation builders.
pub mod distance {
    use super::*;

    /// Add a point-to-point distance relation.
    ///
    /// The builder records `|a-b|^2 - d^2` as the expected proof strategy. This
    /// is proposal-compatible with CAD distance constraints while preserving a
    /// polynomial exact replay path for `hypersolve`.
    pub fn point_point_distance(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointPointDistance { a, b, distance };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredDistance,
            kind,
        }
    }
}

/// Orientation relation builders.
pub mod orientation {
    use super::*;

    /// Add a horizontal 2D line relation.
    pub fn horizontal(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::Horizontal { line };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::CoordinateEquality,
            kind,
        }
    }

    /// Add a vertical 2D line relation.
    pub fn vertical(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::Vertical { line };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::CoordinateEquality,
            kind,
        }
    }
}
