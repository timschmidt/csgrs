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
    SketchArcEndpoint, SketchArcTangencyBranch, SketchConstraintHandle, SketchConstraintKind,
    SketchEntityHandle, SketchLineEndpoint, SketchParameterHandle, SketchResidualStrategy,
    SketchSolveProblem, SketchTangentOrientation,
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
