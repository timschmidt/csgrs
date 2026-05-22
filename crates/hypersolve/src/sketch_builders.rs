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

use hyperreal::Real;

use crate::sketch::{
    SketchConstraintHandle, SketchConstraintKind, SketchEntityHandle, SketchParameterHandle,
    SketchResidualStrategy, SketchSolveProblem,
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
    /// Symmetry relation, such as a point constrained to a midpoint.
    Symmetry,
    /// Inequality/domain range relation.
    Range,
    /// Soft objective relation.
    Objective,
}

/// Report emitted when a typed sketch constraint builder records a relation.
#[derive(Clone, Debug, PartialEq)]
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

    /// Add an exact bounded point-to-point distance relation.
    ///
    /// Bounds lower to squared-distance inequalities only after exact
    /// nonnegative-bound validation. This covers clearance and bounded-distance
    /// use cases while avoiding the classic error of squaring an invalid
    /// negative distance. The proof boundary remains exact residual replay; see
    /// Yap, "Towards Exact Geometric Computation" (1997).
    pub fn point_point_distance_range(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointPointDistanceRange { a, b, lower, upper };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::BoundedSquaredDistance,
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

    /// Add a 2D line parallelism relation.
    ///
    /// The lowered row is the exact direction cross product. This is the same
    /// algebraic predicate package used for G1 tangent parallelism; see Yap,
    /// "Towards Exact Geometric Computation" (1997), for the separation
    /// between constructing this proof obligation and certifying a candidate.
    pub fn parallel_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ParallelLines2 { a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::DirectionCrossProduct,
            kind,
        }
    }

    /// Add a 2D line perpendicularity relation.
    ///
    /// The lowered row is the exact direction dot product. Degenerate line
    /// carriers should be reported by retained-entity preflight rather than by
    /// an epsilon-based normalization step.
    pub fn perpendicular_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PerpendicularLines2 { a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::DirectionDotProduct,
            kind,
        }
    }

    /// Add a 2D line same-orientation relation.
    ///
    /// The lowered package uses exact unnormalized predicates: cross product
    /// equality for parallel support and dot product nonnegativity for shared
    /// orientation. This keeps the orientation decision report-bearing, as
    /// required by Yap's "Towards Exact Geometric Computation" (1997), and
    /// avoids importing any SolveSpace implementation code.
    pub fn same_direction_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::SameDirectionLines2 { a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::DirectionSameOrientation,
            kind,
        }
    }
}

/// Symmetry and midpoint relation builders.
pub mod symmetry {
    use super::*;

    /// Add a 2D point-at-midpoint relation.
    ///
    /// Lowering emits the two exact linear rows `2*point.x - a.x - b.x == 0`
    /// and `2*point.y - a.y - b.y == 0`. Keeping midpoint as a retained
    /// semantic object, rather than only as anonymous coordinate equations,
    /// follows Yap's construction/proof split in "Towards Exact Geometric
    /// Computation" (1997) and preserves SolveSpace-style constraint
    /// provenance for diagnostics.
    pub fn at_midpoint2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        point: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::AtMidpoint2 { point, a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Symmetry,
            strategy: SketchResidualStrategy::MidpointCoordinateEquality,
            kind,
        }
    }
}

/// Inequality and parameter-domain builders.
pub mod ranges {
    use super::*;

    /// Add an exact scalar parameter range.
    ///
    /// The lower bound lowers to `parameter - lower >= 0`; the upper bound
    /// lowers to `parameter - upper <= 0`. Bounds are exact `Real` values, not
    /// solver-private tolerances, preserving Yap's report-bearing decision
    /// boundary.
    pub fn parameter_range(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        parameter: SketchParameterHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ParameterRange {
            parameter,
            lower,
            upper,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Range,
            strategy: SketchResidualStrategy::ParameterRange,
            kind,
        }
    }

    /// Add an exact nondecreasing relation between two scalar parameters.
    ///
    /// The relation lowers to `upper - lower >= 0`, giving generic
    /// monotonicity checks a retained sketch object while preserving the
    /// Yap-style exact replay boundary for the actual sign decision.
    pub fn parameter_ordering(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        lower: SketchParameterHandle,
        upper: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ParameterOrdering { lower, upper };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Range,
            strategy: SketchResidualStrategy::ParameterOrdering,
            kind,
        }
    }

    /// Add an exact minimum margin between two ordered scalar parameters.
    ///
    /// The relation lowers to `upper - lower - margin >= 0` after exact
    /// nonnegative-margin validation. This keeps generic design-rule margins
    /// report-bearing without embedding fabrication or EDA semantics in
    /// `hypersolve`.
    pub fn parameter_margin(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        lower: SketchParameterHandle,
        upper: SketchParameterHandle,
        margin: Real,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ParameterMargin {
            lower,
            upper,
            margin,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Range,
            strategy: SketchResidualStrategy::ParameterMargin,
            kind,
        }
    }
}

/// Soft objective builders.
pub mod objective {
    use super::*;

    /// Add a soft stay-near objective for one parameter.
    ///
    /// Soft rows remain ordinary residual rows with [`crate::ConstraintKind::Soft`].
    /// They can guide proposal engines, but exact certification still reports
    /// whether the active candidate satisfies the row exactly, is uncertain, or
    /// only carries proposal evidence.
    pub fn stay_near_parameter(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        parameter: SketchParameterHandle,
        target: Real,
        weight: Real,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::StayNearParameter {
            parameter,
            target,
            weight,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Objective,
            strategy: SketchResidualStrategy::SoftObjective,
            kind,
        }
    }
}
