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
    SketchArcEndpoint, SketchArcLengthSweep, SketchArcPointSweep, SketchArcTangencyBranch,
    SketchCircleTangencyBranch, SketchConstraintHandle, SketchConstraintKind, SketchEntityHandle,
    SketchLineEndpoint, SketchParameterHandle, SketchResidualStrategy, SketchSolveProblem,
    SketchTangentOrientation,
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

    /// Add an oriented 2D equal-angle relation between two line pairs.
    ///
    /// Lowering compares the exact angle vectors `(dot, cross)` for `(a,b)`
    /// and `(c,d)`, then adds a same-branch predicate. This is the proof row
    /// package for branch-sensitive `ANGLE` behavior: unsigned angle equality
    /// is not enough when a sketch distinguishes clockwise from
    /// counterclockwise turns. The package follows Yap's "Towards Exact
    /// Geometric Computation" (1997) by proving the branch with exact
    /// polynomials rather than by trusting an `atan2` proposal.
    pub fn equal_oriented_angle_lines2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualOrientedAngleLines2 { a, b, c, d };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Angle,
            strategy: SketchResidualStrategy::OrientedAngleEquality,
            kind,
        }
    }

    /// Add a workplane-projected oriented equal-angle relation for 3D lines.
    ///
    /// Lowering certifies the retained workplane unit frame, projects four
    /// 3D line directions into exact `U/V` coordinates, and compares the
    /// resulting oriented angle vectors `(dot, cross)` plus same-branch
    /// predicate. This is the exact 3D/workplane counterpart of
    /// [`equal_oriented_angle_lines2`]: Yap's "Towards Exact Geometric
    /// Computation" (1997) keeps trigonometric proposal values out of the
    /// proof path, and Shoemake's unit-quaternion frame supplies the
    /// projection polynomial.
    pub fn projected_equal_oriented_angle_lines3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedEqualOrientedAngleLines3 {
            workplane,
            a,
            b,
            c,
            d,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Angle,
            strategy: SketchResidualStrategy::ProjectedOrientedAngleEquality,
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

    /// Add a retained 2D circular-arc/line tangent relation.
    ///
    /// The caller selects both endpoints and the signed orientation branch.
    /// Lowering emits exact endpoint incidence, arc endpoint-on-radius,
    /// radius/tangent perpendicularity, and signed cross-product rows. This
    /// covers the first `ARC_LINE_TANGENT` slice while keeping curve ownership
    /// and degeneracy decisions report-bearing, matching Yap's "Towards Exact
    /// Geometric Computation" (1997) and the endpoint-aware constraint
    /// vocabulary in Bouma et al., "A Geometric Constraint Solver" (1995).
    pub fn arc_line_tangent2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ArcLineTangent2 {
            arc,
            arc_endpoint,
            line,
            line_endpoint,
            orientation,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ArcLineTangent,
            kind,
        }
    }

    /// Add a retained 2D circular-arc/circular-arc tangent relation.
    ///
    /// The selected endpoints are constrained to coincide, both selected
    /// endpoints are replayed against their retained radii, and the two radius
    /// vectors are constrained collinear with an explicit same/opposite branch.
    /// The branch is source evidence, not a floating contact classifier. This
    /// follows Yap's "Towards Exact Geometric Computation" (1997) and the
    /// endpoint-aware geometric-constraint vocabulary surveyed by Bouma et al.,
    /// "A Geometric Constraint Solver" (1995).
    pub fn arc_arc_tangent2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_endpoint: SketchArcEndpoint,
        second: SketchEntityHandle,
        second_endpoint: SketchArcEndpoint,
        branch: SketchArcTangencyBranch,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ArcArcTangent2 {
            first,
            first_endpoint,
            second,
            second_endpoint,
            branch,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ArcArcTangent,
            kind,
        }
    }

    /// Add a retained 2D circular-arc/cubic-Bezier tangent relation.
    ///
    /// The selected arc endpoint is constrained to the exact cubic point
    /// `B(t)`, and the arc radius vector is constrained perpendicular to the
    /// exact cubic derivative `B'(t)` with a retained orientation branch. This
    /// is the mixed conic/polynomial differential package: it preserves the
    /// arc endpoint, cubic parameter, and branch as source evidence, then uses
    /// exact replay for acceptance. The construction follows Yap's "Towards
    /// Exact Geometric Computation" (1997), with the cubic derivative supplied
    /// by Farin's Bernstein/de Casteljau control-net formula.
    pub fn arc_cubic_tangent2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ArcCubicTangent2 {
            arc,
            arc_endpoint,
            cubic,
            parameter,
            orientation,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ArcCubicTangent,
            kind,
        }
    }

    /// Add retained second-order contact between a circular arc and cubic.
    ///
    /// This extends arc/cubic tangency with the exact differentiated
    /// circle-incidence row
    /// `B'(t).B'(t) + (B(t)-C).B''(t) == 0`. It is a curvature-sensitive
    /// proof package expressed without square roots or curvature division.
    /// Degenerate arcs and stationary parameters remain explicit retained
    /// domain obligations, following Yap's "Towards Exact Geometric
    /// Computation" (1997), while Farin's Bernstein/de Casteljau derivative
    /// control nets supply `B'` and `B''`.
    pub fn arc_cubic_second_order_contact2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ArcCubicSecondOrderContact2 {
            arc,
            arc_endpoint,
            cubic,
            parameter,
            orientation,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ArcCubicSecondOrderContact,
            kind,
        }
    }

    /// Add retained 3D/workplane arc-line tangency with endpoint and branch data.
    ///
    /// This projected package mirrors [`arc_line_tangent2`]: the selected
    /// 3D line endpoint projects to the selected arc endpoint, while the
    /// projected outgoing line tangent certifies radius-perpendicularity and
    /// the signed orientation branch. The unit workplane guard is part of the
    /// proof package, following Yap, "Towards Exact Geometric Computation"
    /// (1997); endpoint/branch data follows Bouma et al. (1995), and the
    /// retained frame uses Shoemake's quaternion construction (1985).
    #[allow(clippy::too_many_arguments)]
    pub fn projected_arc_line_tangent3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedArcLineTangent3 {
            workplane,
            arc,
            arc_endpoint,
            line,
            line_endpoint,
            orientation,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedArcLineTangent,
            kind,
        }
    }

    /// Add retained 3D/workplane line/circle tangency.
    ///
    /// Lowering projects the 3D line into the retained workplane and replays
    /// the exact denominator-cleared circle-center-to-line distance equation
    /// `cross_uv^2 - radius^2*|line_uv|^2 == 0` beside the workplane unit
    /// guard. This follows Yap, "Towards Exact Geometric Computation" (1997),
    /// by keeping the projected tangent proof polynomial explicit and never
    /// normalizing the projected line through primitive floats. The workplane
    /// frame is Shoemake's unit-quaternion construction (1985).
    pub fn projected_line_circle_tangent3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        line: SketchEntityHandle,
        circle: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedLineCircleTangent3 {
            workplane,
            line,
            circle,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedLineCircleTangency,
            kind,
        }
    }

    /// Add retained 2D circle/circle tangency with explicit branch evidence.
    ///
    /// External tangency replays `|center_b-center_a|^2 == (r_a+r_b)^2`;
    /// internal tangency replays `|center_b-center_a|^2 == (r_a-r_b)^2`.
    /// The branch is retained by the caller instead of inferred from rounded
    /// center distances. This follows Yap, "Towards Exact Geometric
    /// Computation" (1997), and the explicit branch object mirrors Bouma et
    /// al.'s geometric-constraint-solver vocabulary (1995).
    pub fn circle_circle_tangent2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        first: SketchEntityHandle,
        second: SketchEntityHandle,
        branch: SketchCircleTangencyBranch,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::CircleCircleTangent2 {
            first,
            second,
            branch,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::CircleCircleTangency,
            kind,
        }
    }

    /// Add retained 3D/workplane arc-cubic tangency with endpoint and branch data.
    ///
    /// The projected 3D cubic point is constrained to the selected retained
    /// 2D arc endpoint, and the projected cubic derivative is checked with
    /// exact radius-perpendicularity and a signed orientation predicate. The
    /// workplane guard, endpoint-on-radius, incidence, perpendicularity, and
    /// branch rows follow Yap (1997), Shoemake's quaternion frame, and Farin's
    /// Bernstein/de Casteljau derivative construction.
    #[allow(clippy::too_many_arguments)]
    pub fn projected_arc_cubic_curve_tangent3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedArcCubicCurveTangent3 {
            workplane,
            arc,
            arc_endpoint,
            cubic,
            parameter,
            orientation,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedArcCubicCurveTangent,
            kind,
        }
    }

    /// Add retained 3D/workplane arc-cubic second-order contact.
    ///
    /// The projected 3D cubic point, first derivative, and second derivative
    /// are replayed against the selected retained 2D arc endpoint. The proof
    /// package extends projected arc/cubic tangency with the differentiated
    /// circle-incidence row `B'(t).B'(t) + (B(t)-C).B''(t) == 0`, keeping
    /// curvature-sensitive evidence polynomial and denominator-free. This
    /// follows Yap (1997), Shoemake's quaternion frame (1985), and Farin's
    /// Bernstein/de Casteljau derivative construction (2002).
    #[allow(clippy::too_many_arguments)]
    pub fn projected_arc_cubic_curve_second_order_contact3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedArcCubicCurveSecondOrderContact3 {
            workplane,
            arc,
            arc_endpoint,
            cubic,
            parameter,
            orientation,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedArcCubicCurveSecondOrderContact,
            kind,
        }
    }

    /// Add a retained 2D cubic-Bezier/line tangent relation.
    ///
    /// The selected line endpoint is constrained to the exact cubic point
    /// `B(t)`, and the line's outgoing tangent is constrained to the same
    /// direction as the exact cubic derivative `B'(t)`. The proof package uses
    /// unnormalized cross and dot predicates, so degenerate curve derivatives,
    /// degenerate line tangents, and segment-domain policy remain explicit
    /// report-bearing obligations. This follows Yap's "Towards Exact
    /// Geometric Computation" (1997), while the derivative control-net formula
    /// follows Farin's Bernstein/de Casteljau curve model.
    pub fn cubic_line_tangent2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::CubicLineTangent2 {
            cubic,
            parameter,
            line,
            line_endpoint,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::CubicLineTangent,
            kind,
        }
    }

    /// Add projected 3D line/cubic tangency at a retained cubic parameter.
    ///
    /// The selected 3D line endpoint is projected into the retained workplane
    /// and constrained to the exact cubic point `B(t)`. The projected outgoing
    /// line direction is constrained to share support and orientation with the
    /// exact cubic derivative `B'(t)`. The workplane unit guard, endpoint
    /// incidence, tangent cross product, and same-direction branch are all
    /// exact replay rows, following Yap (1997); the frame uses Shoemake's
    /// unit-quaternion rotation matrix and the curve derivative uses Farin's
    /// Bernstein/de Casteljau construction.
    pub fn projected_cubic_line_tangent3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedCubicLineTangent3 {
            workplane,
            cubic,
            parameter,
            line,
            line_endpoint,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedCubicLineTangent,
            kind,
        }
    }

    /// Add projected 3D line/3D-cubic tangency at a retained cubic parameter.
    ///
    /// The selected 3D line endpoint, outgoing line direction, and retained
    /// 3D cubic control net are projected into the workplane before exact
    /// Bernstein point/derivative replay. The package keeps the unit-frame
    /// guard, endpoint incidence, tangent cross product, and same-direction
    /// branch as exact proof rows, following Yap (1997), Shoemake's
    /// quaternion frame, and Farin's Bernstein/de Casteljau construction.
    pub fn projected_cubic_curve_line_tangent3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedCubicCurveLineTangent3 {
            workplane,
            cubic,
            parameter,
            line,
            line_endpoint,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedCubicCurveLineTangent,
            kind,
        }
    }

    /// Add projected G1 tangency between retained 3D cubic Bezier carriers.
    ///
    /// Both retained 3D control nets are projected into the workplane before
    /// exact Bernstein point/derivative replay. The package keeps the
    /// unit-frame guard, point incidence, derivative cross product, and
    /// same-direction branch as exact proof rows, following Yap (1997),
    /// Shoemake's quaternion frame, and Farin's Bernstein/de Casteljau
    /// construction.
    pub fn projected_cubic_curve_cubic_curve_tangent3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedCubicCurveCubicCurveTangent3 {
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedCubicCurveCubicCurveTangent,
            kind,
        }
    }

    /// Add projected parametric C2 continuity between retained 3D cubics.
    ///
    /// The retained 3D control nets are projected into the workplane before
    /// exact Bernstein point, first-derivative, and second-derivative replay.
    /// This is a strict parameterization-preserving package, not a geometric
    /// curvature-normalized G2 test. It follows Yap (1997), Shoemake's
    /// quaternion frame, and Farin's Bernstein/de Casteljau derivative
    /// construction.
    pub fn projected_cubic_curve_cubic_curve_c2_continuity3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedCubicCurveCubicCurveC2Continuity3 {
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedCubicCurveCubicCurveC2Continuity,
            kind,
        }
    }

    /// Add projected geometric G2 continuity between retained 3D cubics.
    ///
    /// The retained 3D control nets are projected into the workplane before
    /// exact Bernstein point, tangent, and second-derivative replay. Signed
    /// curvature is compared after clearing speed denominators, following Yap
    /// (1997), Shoemake's quaternion frame, and Farin's Bernstein/de Casteljau
    /// derivative construction.
    pub fn projected_cubic_curve_cubic_curve_g2_continuity3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedCubicCurveCubicCurveG2Continuity3 {
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::ProjectedCubicCurveCubicCurveG2Continuity,
            kind,
        }
    }

    /// Add a retained 2D cubic-Bezier/cubic-Bezier tangent relation.
    ///
    /// The two curve points `B_a(t_a)` and `B_b(t_b)` are constrained to
    /// coincide, and the exact derivatives are constrained to have common
    /// support and the same orientation branch. This is the curve/curve
    /// analogue of G1 tangency: no sampled nearest point, derivative
    /// normalization, or primitive-float angle is trusted as evidence. The
    /// package follows Yap's "Towards Exact Geometric Computation" (1997) and
    /// Farin's Bernstein derivative construction.
    pub fn cubic_cubic_tangent2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::CubicCubicTangent2 {
            first,
            first_parameter,
            second,
            second_parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::CubicCubicTangent,
            kind,
        }
    }

    /// Add retained geometric G2 curvature continuity between two cubics.
    ///
    /// The package keeps point coincidence and same-direction tangent replay,
    /// then compares signed curvature without dividing by speed:
    /// `cross(Ba',Ba'')^2 * |Bb'|^6 - cross(Bb',Bb'')^2 * |Ba'|^6 == 0`
    /// plus a same-sign branch predicate. This is geometric G2, not
    /// parametric C2. The proof split follows Yap's "Towards Exact Geometric
    /// Computation" (1997), and the first/second derivative vectors follow
    /// Farin's Bernstein/de Casteljau control-net formulas.
    pub fn cubic_cubic_g2_continuity2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::CubicCubicG2Continuity2 {
            first,
            first_parameter,
            second,
            second_parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::CubicCubicG2Continuity,
            kind,
        }
    }

    /// Add a retained parametric C2 continuity relation between two cubics.
    ///
    /// The generated proof package equates the exact cubic point, first
    /// derivative, and second derivative vectors at retained parameters. This
    /// is deliberately stronger than geometric G2 continuity: it preserves the
    /// parameterization as source evidence instead of normalizing speed or
    /// curvature through primitive floats. The package follows Yap's "Towards
    /// Exact Geometric Computation" (1997) and Farin's Bernstein derivative
    /// control-net formulas.
    pub fn cubic_cubic_c2_continuity2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::CubicCubicC2Continuity2 {
            first,
            first_parameter,
            second,
            second_parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Tangency,
            strategy: SketchResidualStrategy::CubicCubicC2Continuity,
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

    /// Add a 2D point-on-circular-arc incidence relation.
    ///
    /// This is stronger than point-on-circle: the point must satisfy the
    /// retained radius equation and the selected arc-sector branch. Major arcs
    /// carry an explicit point half-branch so exact replay stays conjunctive
    /// and reportable, following Yap's exact-geometric-computation boundary.
    pub fn point_on_arc2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        point: SketchEntityHandle,
        arc: SketchEntityHandle,
        sweep: SketchArcPointSweep,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointOnArc2 { point, arc, sweep };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::PointArcIncidence,
            kind,
        }
    }

    /// Add a projected 3D point-on-2D-circle incidence relation.
    ///
    /// The 3D point is projected through the retained workplane origin and
    /// unit frame before replaying the exact 2D circle incidence row. This is
    /// a 3D/workplane incidence package, not a sampled projection: the
    /// workplane unit guard is part of the proof boundary in Yap's sense, and
    /// the axes follow Shoemake's unit-quaternion rotation matrix.
    pub fn projected_point_on_circle3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        circle: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointOnCircle3 {
            workplane,
            point,
            circle,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::ProjectedSquaredIncidence,
            kind,
        }
    }

    /// Add a projected 3D point-on-2D-circular-arc incidence relation.
    ///
    /// This is the retained arc counterpart of projected point-on-circle: the
    /// projected point must lie on the parent circle and on the selected arc
    /// sweep branch. Major arcs carry an explicit point half-branch so exact
    /// replay remains a conjunction of reportable predicates, following Yap
    /// (1997). The workplane axes use Shoemake's unit-quaternion frame.
    pub fn projected_point_on_arc3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        arc: SketchEntityHandle,
        sweep: SketchArcPointSweep,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointOnArc3 {
            workplane,
            point,
            arc,
            sweep,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::ProjectedPointArcIncidence,
            kind,
        }
    }

    /// Add a 2D point-on-line incidence relation.
    ///
    /// Lowering emits the unnormalized exact row
    /// `cross(point - line_start, line_end - line_start) == 0`. Segment
    /// containment and nondegenerate-line policy remain explicit domain or
    /// range obligations. This follows Yap, "Towards Exact Geometric
    /// Computation" (1997), by keeping collinearity as exact replay evidence
    /// rather than a rounded projection tolerance, and follows Bouma et al.
    /// (1995) by retaining the point/line incidence relation semantically.
    pub fn point_on_line2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointOnLine2 { point, line };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::PointLineIncidence,
            kind,
        }
    }

    /// Add a projected 3D point-on-line incidence relation.
    ///
    /// The 3D point and 3D line direction are projected through a retained
    /// workplane and replayed by exact `U/V` collinearity. The workplane unit
    /// guard is part of the proof package, following Yap, "Towards Exact
    /// Geometric Computation" (1997); the retained frame uses Shoemake's
    /// unit-quaternion rotation matrix from "Animating Rotation with
    /// Quaternion Curves" (1985).
    pub fn projected_point_on_line3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointOnLine3 {
            workplane,
            point,
            line,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::ProjectedPointLineIncidence,
            kind,
        }
    }

    /// Add a point-on-cubic-Bezier incidence relation at a retained parameter.
    ///
    /// Lowering emits one exact Bernstein-coordinate equation per axis:
    /// `point - ((1-t)^3*p0 + 3(1-t)^2*t*p1 + 3(1-t)*t^2*p2 + t^3*p3)`.
    /// The builder deliberately does not clamp or sample `t`; callers express
    /// segment-domain policy with explicit parameter domains or range
    /// constraints. That separation follows Yap's "Towards Exact Geometric
    /// Computation" (1997) and de Casteljau/Farin's Bernstein curve model:
    /// retain the curve/parameter object, then certify the polynomial rows.
    pub fn point_on_cubic2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        point: SketchEntityHandle,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::PointOnCubic2 {
            point,
            cubic,
            parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::CubicBezierIncidence,
            kind,
        }
    }

    /// Add projected 3D point-on-cubic-Bezier incidence at a retained parameter.
    ///
    /// Lowering proves the workplane unit frame, projects the 3D point into
    /// exact `U/V` coordinates, and replays the retained 2D cubic's Bernstein
    /// point equations at the exact sketch parameter. This keeps projection
    /// and curve evaluation as explicit proof rows in Yap's sense; the frame
    /// follows Shoemake (1985), and the cubic basis follows de Casteljau/Farin.
    pub fn projected_point_on_cubic3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointOnCubic3 {
            workplane,
            point,
            cubic,
            parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::ProjectedCubicBezierIncidence,
            kind,
        }
    }

    /// Add projected 3D point-on-3D-cubic-Bezier incidence.
    ///
    /// The point and retained 3D cubic control net are projected through the
    /// workplane frame before exact Bernstein-coordinate replay. This keeps
    /// the 3D curve, point, parameter, and projection frame as retained source
    /// objects, following Yap (1997); Shoemake's unit-quaternion frame and
    /// Farin's Bernstein/de Casteljau curve construction supply the exact
    /// polynomial rows.
    pub fn projected_point_on_cubic_curve3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointOnCubicCurve3 {
            workplane,
            point,
            cubic,
            parameter,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Incidence,
            strategy: SketchResidualStrategy::ProjectedCubicCurveIncidence,
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

    /// Add a bounded projected point-to-point distance relation.
    ///
    /// Bounds are exact distances measured in the retained workplane metric.
    /// Lowering validates nonnegative ordered bounds before replaying the
    /// unit-quaternion guard and squared projected-distance inequalities.
    /// This follows Yap, "Towards Exact Geometric Computation" (1997), by
    /// rejecting invalid semantic bounds before squaring; the workplane frame
    /// is Shoemake's unit-quaternion rotation matrix (1985).
    pub fn projected_point_point_distance_range(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointPointDistanceRange {
            workplane,
            a,
            b,
            lower,
            upper,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::BoundedSquaredProjectedDistance,
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

    /// Add equality between two retained projected 3D point-to-point distances.
    ///
    /// Lowering emits the retained workplane unit guard plus
    /// `|a-b|_uv^2 - |c-d|_uv^2 == 0`. This is deliberately a point-pair
    /// relation, not an implicit line conversion: callers keep their source
    /// objects, and exact replay proves the projected metric under Yap,
    /// "Towards Exact Geometric Computation" (1997). The frame polynomial is
    /// Shoemake's retained unit-quaternion construction (1985).
    pub fn projected_equal_point_point_distances3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedEqualPointPointDistances3 {
            workplane,
            a,
            b,
            c,
            d,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedEqualPointPointDistances,
            kind,
        }
    }

    /// Add a retained workplane-projected point-to-line distance relation.
    ///
    /// Lowering projects the 3D point offset and 3D line direction into the
    /// retained workplane frame, emits a unit-quaternion guard, and replays
    /// `cross_uv^2 - distance^2 * |line_uv|^2 == 0` exactly. The row clears
    /// the line-length denominator rather than normalizing the line with
    /// primitive floats, following Yap, "Towards Exact Geometric Computation"
    /// (1997). The workplane axes are the unit-quaternion frame of Shoemake,
    /// "Animating Rotation with Quaternion Curves" (1985).
    pub fn projected_point_line_distance(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointLineDistance {
            workplane,
            point,
            line,
            distance,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedPointLineDistance,
            kind,
        }
    }

    /// Add a bounded retained workplane-projected point-line distance relation.
    ///
    /// Bounds are exact projected point-line distances. Lowering validates
    /// nonnegative ordered bounds and replays denominator-cleared inequalities
    /// over `cross_uv^2` and `|line_uv|^2`, plus the workplane unit guard.
    /// This follows Yap, "Towards Exact Geometric Computation" (1997), by
    /// certifying semantic bounds before squaring; the retained frame is the
    /// unit-quaternion construction of Shoemake (1985).
    pub fn projected_point_line_distance_range(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointLineDistanceRange {
            workplane,
            point,
            line,
            lower,
            upper,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::BoundedSquaredProjectedPointLineDistance,
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

    /// Add a 2D line length to circular minor-arc length relation.
    ///
    /// Lowering emits endpoint-on-radius polynomial proof rows for the arc and
    /// an exact symbolic squared row
    /// `|line|^2 - (radius * acos(cos(theta)))^2 == 0`, where `theta` is the
    /// principal unsigned angle between the retained arc endpoints. This is
    /// deliberately not advertised as a polynomial package: Yap's "Towards
    /// Exact Geometric Computation" (1997) requires the non-algebraic
    /// arc-length boundary to remain visible. Major or oriented sweeps need an
    /// explicit branch carrier instead of a hidden UI angle.
    pub fn equal_line_arc_length2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        line: SketchEntityHandle,
        arc: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualLineArcLength2 { line, arc };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::LineArcLength,
            kind,
        }
    }

    /// Add a branch-aware 2D line length to circular arc sweep length relation.
    ///
    /// The sweep branch records clockwise/counterclockwise and minor/major
    /// intent explicitly. Lowering emits endpoint-on-radius rows, a signed
    /// branch predicate, and the exact symbolic row
    /// `|line|^2 - (radius * sweep_angle)^2 == 0`, where `sweep_angle` is
    /// either `theta` or `2*pi-theta`. This is the Yap exact-replay boundary
    /// for arc-length branch decisions: no primitive sweep angle or UI
    /// direction flag is trusted unless this retained branch replays.
    pub fn equal_line_arc_sweep_length2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        line: SketchEntityHandle,
        arc: SketchEntityHandle,
        sweep: SketchArcLengthSweep,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::EqualLineArcSweepLength2 { line, arc, sweep };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::LineArcSweepLength,
            kind,
        }
    }

    /// Add a workplane-projected 3D line length to circular arc sweep relation.
    ///
    /// Lowering certifies the retained workplane unit frame, projects the 3D
    /// line direction into exact `U/V` coordinates, then reuses the branch
    /// aware retained arc sweep package. The generated proof rows make both
    /// the projection and the sweep branch explicit, following Yap's "Towards
    /// Exact Geometric Computation" (1997); the workplane axes use Shoemake's
    /// unit-quaternion frame formula.
    pub fn projected_equal_line_arc_sweep_length3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        line: SketchEntityHandle,
        arc: SketchEntityHandle,
        sweep: SketchArcLengthSweep,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedEqualLineArcSweepLength3 {
            workplane,
            line,
            arc,
            sweep,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::ProjectedLineArcSweepLength,
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

    /// Add an exact concentricity relation for 2D circles or circular arcs.
    ///
    /// Lowering replays equality between retained center coordinates. It does
    /// not imply equal radii and it does not derive topology from a rounded
    /// center-distance tolerance. This is the Yap (1997) retained-object
    /// boundary applied to the SolveSpace-style concentric constraint
    /// vocabulary described by Bouma et al. (1995).
    pub fn concentric2(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::Concentric2 { a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::Concentricity,
            kind,
        }
    }

    /// Add a projected 3D point-to-circle/arc center relation.
    ///
    /// The 3D point is projected through a retained workplane and replayed
    /// against the exact retained center coordinates of a circle or circular
    /// arc. The workplane unit guard is part of the generated proof package,
    /// following Yap (1997), and the U/V frame uses Shoemake's quaternion
    /// rotation matrix. The relation is kept distinct from point-on-circle and
    /// radius equality so the SolveSpace-style concentric intent described by
    /// Bouma et al. (1995) remains visible in diagnostics.
    pub fn projected_point_concentric3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        curve: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPointConcentric3 {
            workplane,
            point,
            curve,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::ProjectedConcentricity,
            kind,
        }
    }

    /// Add a retained workplane-projected equal-length relation for 3D lines.
    ///
    /// Lowering projects both retained 3D line directions into the workplane
    /// `U/V` frame and replays `|a_uv|^2 - |b_uv|^2 == 0` beside the
    /// unit-quaternion guard. This is the exact workplane counterpart of
    /// SolveSpace-style line length equality: no projected coordinates are
    /// rounded and no square root participates in proof. See Yap, "Towards
    /// Exact Geometric Computation" (1997), and Shoemake, "Animating Rotation
    /// with Quaternion Curves" (1985), for the retained frame polynomial.
    pub fn projected_equal_length_lines3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedEqualLengthLines3 { workplane, a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedLineLengthEquality,
            kind,
        }
    }

    /// Add a bounded retained workplane-projected line-length relation.
    ///
    /// Bounds are exact projected line lengths. Lowering validates
    /// nonnegative ordered bounds before replaying `|dir_uv|^2 - bound^2`
    /// inequalities beside the workplane unit guard. This follows Yap,
    /// "Towards Exact Geometric Computation" (1997), by keeping semantic
    /// range validation outside the squared polynomial rows; the retained
    /// workplane frame uses Shoemake's unit-quaternion construction (1985).
    pub fn projected_line_length_range3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        line: SketchEntityHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedLineLengthRange3 {
            workplane,
            line,
            lower,
            upper,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::BoundedSquaredProjectedLineLength,
            kind,
        }
    }

    /// Add a retained workplane-projected length-ratio relation for 3D lines.
    ///
    /// The exact ratio is retained as semantic data and checked before
    /// lowering to the squared projected-length polynomial. Negative
    /// numerators and nonpositive denominators therefore become diagnostics
    /// rather than false successes introduced by squaring. The proof boundary
    /// follows Yap, "Towards Exact Geometric Computation" (1997), with the
    /// workplane frame polynomial from Shoemake (1985).
    pub fn projected_length_ratio_lines3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        numerator: Real,
        denominator: Real,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedLengthRatioLines3 {
            workplane,
            a,
            b,
            numerator,
            denominator,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedLineLengthRatio,
            kind,
        }
    }

    /// Add a retained workplane-projected length-difference relation for 3D lines.
    ///
    /// The retained relation is `length(longer_uv) = length(shorter_uv) +
    /// difference`. Lowering emits the exact square-root-free polynomial and
    /// branch inequality used by the 2D length-difference package, but with
    /// both squared lengths computed in the certified workplane frame. This
    /// follows Yap, "Towards Exact Geometric Computation" (1997), by making
    /// the ordering branch explicit proof data.
    pub fn projected_length_difference_lines3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        longer: SketchEntityHandle,
        shorter: SketchEntityHandle,
        difference: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedLengthDifferenceLines3 {
            workplane,
            longer,
            shorter,
            difference,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedLineLengthDifference,
            kind,
        }
    }

    /// Add a retained projected line-length to point-line-distance equality.
    ///
    /// The retained relation is
    /// `length(length_line_uv) = distance(point_uv, distance_line_uv)`.
    /// Lowering clears the point-line distance denominator exactly:
    /// `|length_line_uv|^2 * |distance_line_uv|^2 - cross_uv^2 == 0`. This is
    /// the workplane counterpart of the 2D length/distance relation, and
    /// follows Yap, "Towards Exact Geometric Computation" (1997), by keeping
    /// the projected proof polynomial explicit. The `U/V` frame is the
    /// retained unit-quaternion frame described by Shoemake (1985).
    pub fn projected_equal_length_point_line_distance3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        length_line: SketchEntityHandle,
        point: SketchEntityHandle,
        distance_line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedEqualLengthPointLineDistance3 {
            workplane,
            length_line,
            point,
            distance_line,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance,
            kind,
        }
    }

    /// Add equality between two retained projected 3D point-line distances.
    ///
    /// Lowering projects both point/line pairs into the same retained
    /// workplane frame and replays
    /// `cross(a)^2 * |line_b_uv|^2 - cross(b)^2 * |line_a_uv|^2 == 0`.
    /// Cross-multiplication keeps denominator evidence explicit and follows
    /// Yap, "Towards Exact Geometric Computation" (1997): candidate
    /// acceptance rests on exact polynomial replay, while degenerate projected
    /// line carriers remain separate domain obligations. The frame is the
    /// unit-quaternion construction described by Shoemake (1985).
    pub fn projected_equal_point_line_distances3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a_point: SketchEntityHandle,
        a_line: SketchEntityHandle,
        b_point: SketchEntityHandle,
        b_line: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedEqualPointLineDistances3 {
            workplane,
            a_point,
            a_line,
            b_point,
            b_line,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Distance,
            strategy: SketchResidualStrategy::SquaredProjectedEqualPointLineDistances,
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

    /// Add a retained workplane-projected 3D line parallelism relation.
    ///
    /// Lowering projects both 3D line directions into the certified workplane
    /// frame and replays the exact 2D direction cross product beside the
    /// unit-quaternion guard. This is the projected analogue of
    /// [`parallel_lines2`], following Yap, "Towards Exact Geometric
    /// Computation" (1997), with the retained frame polynomial from Shoemake
    /// (1985).
    pub fn projected_parallel_lines3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedParallelLines3 { workplane, a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::ProjectedDirectionCrossProduct,
            kind,
        }
    }

    /// Add a retained workplane-projected 3D line perpendicularity relation.
    ///
    /// Lowering projects both 3D line directions into the certified workplane
    /// frame and replays the exact 2D direction dot product beside the
    /// unit-quaternion guard. This avoids normalizing projected directions or
    /// evaluating primitive-float angles, following Yap's exact predicate
    /// boundary and Shoemake's unit-quaternion frame.
    pub fn projected_perpendicular_lines3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedPerpendicularLines3 { workplane, a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::ProjectedDirectionDotProduct,
            kind,
        }
    }

    /// Add a retained workplane-projected 3D line same-direction relation.
    ///
    /// Lowering projects both 3D line directions into the certified workplane
    /// frame and replays exact unnormalized predicates: cross product equality
    /// for common support and dot product nonnegativity for the same branch.
    /// This follows Yap's exact-geometric-computation boundary; Shoemake's
    /// unit-quaternion frame supplies the projection polynomial.
    pub fn projected_same_direction_lines3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedSameDirectionLines3 { workplane, a, b };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Orientation,
            strategy: SketchResidualStrategy::ProjectedDirectionSameOrientation,
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

    /// Add a 3D/workplane projected line-axis point symmetry relation.
    ///
    /// Lowering proves the retained workplane unit frame and then replays the
    /// ordinary line-symmetry predicates after exact projection: doubled
    /// midpoint-on-axis and point-offset perpendicularity. This follows Yap,
    /// "Towards Exact Geometric Computation" (1997), by keeping reflection as
    /// retained source structure and accepting candidates only through exact
    /// replay. The retained frame uses Shoemake's unit-quaternion rotation
    /// matrix from "Animating Rotation with Quaternion Curves" (1985).
    pub fn projected_symmetric_line3(
        sketch: &mut SketchSolveProblem,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        axis: SketchEntityHandle,
    ) -> SketchConstraintBuildReport {
        let kind = SketchConstraintKind::ProjectedSymmetricLine3 {
            workplane,
            a,
            b,
            axis,
        };
        let handle = sketch.add_constraint(name, kind.clone(), false, true);
        SketchConstraintBuildReport {
            handle,
            family: SketchConstraintFamily::Symmetry,
            strategy: SketchResidualStrategy::ProjectedLineSymmetryPolynomial,
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
