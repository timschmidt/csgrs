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
    /// Angle relation, such as equal unsigned angle between line pairs.
    Angle,
    /// Tangency relation, such as same-direction G1 tangent carriers.
    Tangency,
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

/// Angle relation builders.
pub mod angle {
    use super::*;

    /// Add an unsigned 2D equal-angle relation between two line pairs.
    ///
    /// Lowering compares squared cosines:
    /// `dot(a,b)^2*|c|^2*|d|^2 - dot(c,d)^2*|a|^2*|b|^2 == 0`.
    /// This is the algebraic proof package for equal unsigned angles; any
    /// oriented-angle or trigonometric proposal form should be retained
    /// separately. The split follows Yap's "Towards Exact Geometric
    /// Computation" (1997): exact replay, not a floating angle computation,
    /// decides acceptance.
    pub fn equal_angle_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualAngleLines2 { a, b, c, d };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Angle,
            strategy: SketchResidualStrategy::SquaredCosineAngleEquality,
            kind,
        }
    }
}

/// Tangency relation builders.
pub mod tangency {
    use super::*;

    /// Add a 2D same-direction tangent-carrier relation.
    ///
    /// Lowering emits exact unnormalized G1 tangent predicates: a cross-product
    /// equality for common tangent support and a dot-product inequality for the
    /// same orientation branch. Degenerate tangent carriers remain explicit
    /// entity-domain obligations. This follows Yap's "Towards Exact Geometric
    /// Computation" (1997): tangent construction may be a proposal step, but
    /// candidate acceptance is exact predicate replay.
    pub fn tangent_same_direction_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        candidate: SketchEntityHandle,
        target: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::TangentSameDirectionLines2 { candidate, target };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::TangentSameDirection,
            kind,
        }
    }
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

    /// Add a retained workplane-projected point-to-point distance relation.
    ///
    /// This is the SolveSpace-style projected-distance row with Hyper's exact
    /// proof boundary: the workplane remains a retained object, lowering emits
    /// a unit-quaternion guard plus a squared projected-distance polynomial,
    /// and any square-root projected-distance form is only proposal/UI data.
    /// The split follows Yap, "Towards Exact Geometric Computation" (1997);
    /// the frame polynomial uses the unit-quaternion rotation matrix described
    /// by Shoemake, "Animating Rotation with Quaternion Curves" (1985).
    pub fn projected_point_point_distance(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointPointDistance {
            workplane,
            a,
            b,
            distance,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedDistance,
            kind,
        }
    }

    /// Add a 2D line equal-length relation.
    ///
    /// Lowering compares squared segment lengths exactly. This is the
    /// polynomial proof form preferred by Yap's "Towards Exact Geometric
    /// Computation" (1997); true-length proposal forms can be retained
    /// separately when a caller needs SolveSpace-style UI behavior.
    pub fn equal_length_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualLengthLines2 { a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredLineLengthEquality,
            kind,
        }
    }

    /// Add a 2D line length-ratio relation.
    ///
    /// The relation retains the exact ratio `numerator / denominator` and
    /// lowers to a squared polynomial equality only after exact sign
    /// validation. This follows Yap's "Towards Exact Geometric Computation"
    /// (1997): invalid semantic inputs become diagnostics, not silently
    /// accepted squared rows.
    pub fn length_ratio_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        numerator: Real,
        denominator: Real,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::LengthRatioLines2 {
            a,
            b,
            numerator,
            denominator,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredLineLengthRatio,
            kind,
        }
    }

    /// Add a 2D line length-difference relation.
    ///
    /// The retained relation is `length(longer) = length(shorter) +
    /// difference`. Lowering uses the exact squared polynomial
    /// `(A + B - d^2)^2 - 4AB == 0` plus the exact branch inequality
    /// `A - B - d^2 >= 0`. This directly follows Yap's "Towards Exact
    /// Geometric Computation" (1997): the algebraic predicate and its branch
    /// assumptions are explicit proof rows.
    pub fn length_difference_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        longer: SketchEntityHandle,
        shorter: SketchEntityHandle,
        difference: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::LengthDifferenceLines2 {
            longer,
            shorter,
            difference,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredLineLengthDifference,
            kind,
        }
    }

    /// Add a 2D point-to-line distance relation.
    ///
    /// Lowering uses the exact polynomial
    /// `cross(point-start, dir)^2 - distance^2*|dir|^2 == 0` so certification
    /// avoids square roots and division by line length. Nondegenerate-line and
    /// nonnegative-distance assumptions should be modeled as explicit domains.
    pub fn point_line_distance2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointLineDistance2 {
            point,
            line,
            distance,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredPointLineDistance,
            kind,
        }
    }

    /// Add a relation equating a 2D line length to a point-line distance.
    ///
    /// The lowered row uses the exact cross-multiplied squared form
    /// `|dir(length_line)|^2*|dir(distance_line)|^2 - cross^2 == 0`.
    /// This follows Yap's "Towards Exact Geometric Computation" (1997) by
    /// retaining the semantic relation while making exact replay the trust
    /// boundary.
    pub fn equal_length_point_line_distance2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        length_line: SketchEntityHandle,
        point: SketchEntityHandle,
        distance_line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualLengthPointLineDistance2 {
            length_line,
            point,
            distance_line,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredLineLengthPointLineDistance,
            kind,
        }
    }

    /// Add equality between two 2D point-line distances.
    ///
    /// Lowering cross-multiplies squared point-line distance expressions so
    /// exact replay avoids square roots and division. Degenerate line carriers
    /// remain explicit domain obligations instead of hidden epsilons.
    pub fn equal_point_line_distances2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a_point: SketchEntityHandle,
        a_line: SketchEntityHandle,
        b_point: SketchEntityHandle,
        b_line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualPointLineDistances2 {
            a_point,
            a_line,
            b_point,
            b_line,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredEqualPointLineDistances,
            kind,
        }
    }

    /// Add an exact equal-radius relation for 2D circles or circular arcs.
    ///
    /// The relation lowers to direct equality of retained radius carriers.
    /// Radius positivity is deliberately a separate entity-domain obligation,
    /// not an implicit solver tolerance.
    pub fn equal_radius2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualRadius2 { a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::RadiusEquality,
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

    /// Add a 2D horizontal-axis point symmetry relation.
    ///
    /// Lowering emits exact affine rows for `a.x == b.x` and
    /// `a.y + b.y == 2*axis_y`. The mirror axis is retained as exact data,
    /// matching Yap's "Towards Exact Geometric Computation" (1997): the
    /// builder records a proof obligation, and exact replay decides whether a
    /// candidate satisfies it.
    pub fn symmetric_horizontal2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        axis_y: Real,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::SymmetricHorizontal2 { a, b, axis_y };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Symmetry,
            strategy: SketchResidualStrategy::AxisSymmetryCoordinateEquality,
            kind,
        }
    }

    /// Add a 2D vertical-axis point symmetry relation.
    ///
    /// Lowering emits exact affine rows for `a.y == b.y` and
    /// `a.x + b.x == 2*axis_x`, preserving the original relation for
    /// diagnostics instead of lowering it to anonymous coordinate equations.
    pub fn symmetric_vertical2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        axis_x: Real,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::SymmetricVertical2 { a, b, axis_x };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Symmetry,
            strategy: SketchResidualStrategy::AxisSymmetryCoordinateEquality,
            kind,
        }
    }

    /// Add a 2D line-axis point symmetry relation.
    ///
    /// Lowering emits exact midpoint-on-axis and perpendicular-offset rows
    /// using the retained axis direction without normalization. This records
    /// the SolveSpace-style mirror relation as semantic source data while
    /// preserving Yap's proof boundary from "Towards Exact Geometric
    /// Computation" (1997): exact predicates, not rounded construction
    /// coordinates, decide candidate acceptance.
    pub fn symmetric_line2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        axis: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::SymmetricLine2 { a, b, axis };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Symmetry,
            strategy: SketchResidualStrategy::LineSymmetryPolynomial,
            kind,
        }
    }

    /// Add a 3D workplane point-symmetry relation.
    ///
    /// Lowering emits an exact unit-workplane guard, a midpoint-on-plane row,
    /// and normal-offset cross-product rows. This is the retained 3D
    /// plane-symmetry package requested by the SolveSpace coverage plan:
    /// reflection is proved by exact predicates over the retained workplane,
    /// not by constructing a rounded mirror point. The boundary follows Yap's
    /// "Towards Exact Geometric Computation" (1997); the workplane frame uses
    /// Shoemake's unit-quaternion rotation matrix.
    pub fn symmetric_workplane3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        workplane: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::SymmetricWorkplane3 { a, b, workplane };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Symmetry,
            strategy: SketchResidualStrategy::WorkplaneSymmetryPolynomial,
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
