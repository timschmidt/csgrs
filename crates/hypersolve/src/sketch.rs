//! Sketch-level semantic problem carriers above generic residual rows.
//!
//! This module is intentionally a modeling layer, not a second solver. It
//! retains typed handles, entities, high-level constraints, and source-row
//! provenance, then lowers them into the existing [`Problem`] residual format.
//! That split follows Yap's exact geometric computation boundary: constructions
//! and proposal-friendly residuals may be convenient, but exact replay and
//! certificate reports decide trust. See C. K. Yap, "Towards Exact Geometric
//! Computation" (1997). The handle/entity vocabulary is also shaped by the
//! public SolveSpaceLib surface, but no GPL implementation code is copied; see
//! SolveSpace's public `slvs.h` API and SolveSpace technology notes for the
//! behavioral reference.

use hyperreal::{Real, RealSign};

use crate::model::{Constraint, ConstraintKind, Problem, VariableId};
use crate::sketch_arc_tangent::{
    ArcArcTangentExprs, ArcCubicSecondOrderContactExprs, ArcCubicTangentExprs, ArcLineTangentExprs,
    arc_arc_tangent_exprs, arc_cubic_second_order_contact_exprs, arc_cubic_tangent_exprs,
    arc_line_tangent_exprs,
};
use crate::sketch_builders::{
    angle, distance, incidence, objective, orientation, ranges, symmetry, tangency,
};
use crate::sketch_cubic_tangent::{CubicPointTangentExprs, cubic_point_tangent_exprs};
use crate::sketch_oriented_angle::{OrientedAngleExprs, oriented_angle_exprs};
use crate::sketch_projection::{
    projected_direction_squared_length, projected_direction2, projected_distance_squared,
    projected_point_line_distance_squared_parts, unit_quaternion_residual,
};
use crate::sketch_workplane_symmetry::{WorkplaneSymmetryExprs, workplane_point_symmetry_exprs};
use crate::symbolic::{Expr, SymbolId};

/// Stable caller-facing handle for a sketch parameter.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SketchParameterHandle(pub u32);

/// Stable caller-facing handle for a sketch entity.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SketchEntityHandle(pub u32);

/// Stable caller-facing handle for a sketch constraint.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SketchConstraintHandle(pub u32);

/// Stable caller-facing handle for a sketch group.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SketchGroupHandle(pub u32);

/// Editor/API metadata retained across sketch lowering.
///
/// These fields are descriptive data and explicit proof inputs, not hidden
/// solver tolerances. Source units, displayed labels, and declared tolerances
/// may guide UI round-tripping or design-rule checks, but exact residual
/// replay remains the proof boundary, following Yap, "Towards Exact Geometric
/// Computation" (1997). The metadata mirrors the SolveSpace-style need to
/// round-trip reference dimensions, construction geometry, and comments
/// without turning every annotation into an equation.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SketchRoundTripMetadata {
    /// Source unit label such as `"mm"`, `"inch"`, or `"rad"`.
    pub source_unit: Option<String>,
    /// Declared exact source tolerance, retained as data rather than a hidden
    /// residual acceptance threshold.
    pub declared_tolerance: Option<Real>,
    /// UI/display label supplied by an editor or importer.
    pub display_label: Option<String>,
    /// Human-readable comment that should not generate residual rows.
    pub comment: Option<String>,
    /// Whether this object is normal geometry, construction-only geometry, a
    /// reference dimension, or a diagnostic/comment object.
    pub role: SketchRoundTripRole,
    /// Optional lossy proposal adapter label for diagnostics.
    pub lossy_adapter_label: Option<String>,
}

/// Round-trip role for sketch objects.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum SketchRoundTripRole {
    /// Ordinary solving geometry or constraint data.
    #[default]
    Normal,
    /// Construction geometry retained for editing but not necessarily exported
    /// as manufacturing geometry.
    Construction,
    /// Reference/display dimension that should not create proof obligations.
    ReferenceDimension,
    /// Diagnostic/comment-only object.
    DiagnosticOnly,
}

/// A retained scalar parameter before lowering to a solver variable.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchParameter {
    /// Stable source handle.
    pub handle: SketchParameterHandle,
    /// Optional group ownership for group-scoped solves.
    pub group: Option<SketchGroupHandle>,
    /// Human-readable source name.
    pub name: String,
    /// Initial exact value.
    pub value: Real,
    /// Whether proposal engines should treat this parameter as fixed.
    pub fixed: bool,
    /// Exact parameter-domain obligations checked before numerical iteration.
    pub domains: Vec<SketchParameterDomain>,
    /// Editor/API round-trip metadata.
    pub metadata: SketchRoundTripMetadata,
}

/// First-class exact domain obligation for a scalar sketch parameter.
///
/// Domains are preflight checks, not solver tolerances. They let callers reject
/// invalid retained sketch state before numerical iteration while keeping
/// Yap's proof boundary explicit: each check must be certified exactly or
/// reported as unresolved. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, PartialEq)]
pub enum SketchParameterDomain {
    /// The parameter is intentionally locked to this exact value.
    Locked { value: Real },
    /// The parameter must stay within optional exact closed bounds.
    Bounded {
        /// Inclusive lower bound.
        lower: Option<Real>,
        /// Inclusive upper bound.
        upper: Option<Real>,
    },
    /// The parameter must be greater than or equal to zero.
    Nonnegative,
    /// The parameter must be strictly greater than zero.
    Positive,
    /// The parameter must be nonzero.
    Nonzero,
    /// Angular parameter constrained to an exact closed interval.
    AngularRange {
        /// Inclusive lower angle bound.
        lower: Real,
        /// Inclusive upper angle bound.
        upper: Real,
    },
    /// Periodic parameter with a strictly positive exact period.
    Periodic {
        /// Exact period that must be positive.
        period: Real,
    },
}

/// First-class exact domain obligation for a retained sketch entity.
///
/// Entity domains are semantic preflight checks, not hidden solver tolerances.
/// They keep SolveSpace-style retained objects such as normals, radii, lines,
/// arcs, and tangent carriers honest before numerical proposal engines run.
/// Each obligation must be certified exactly or reported as unresolved,
/// following Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchEntityDomain {
    /// A retained 2D or 3D normal/quaternion must have squared length one.
    UnitNormal,
    /// A retained distance, circle radius, or arc radius must be strictly
    /// positive.
    PositiveRadius,
    /// A retained 2D line segment must have nonzero exact length.
    NonzeroLengthLineSegment2,
    /// A retained 3D line segment must have nonzero exact length.
    NonzeroLengthLineSegment3,
    /// A retained 2D circular arc must have distinct start and end points.
    NondegenerateArc2,
    /// A retained 2D line segment used as a tangent carrier must have nonzero
    /// direction. This is deliberately separate from generic line length so
    /// future tangent reports can explain the source relation.
    NonzeroTangentLineSegment2,
}

/// Point entity in a two-dimensional workplane.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchPoint2 {
    /// X coordinate parameter.
    pub x: SketchParameterHandle,
    /// Y coordinate parameter.
    pub y: SketchParameterHandle,
}

/// Point entity in free three-dimensional space.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchPoint3 {
    /// X coordinate parameter.
    pub x: SketchParameterHandle,
    /// Y coordinate parameter.
    pub y: SketchParameterHandle,
    /// Z coordinate parameter.
    pub z: SketchParameterHandle,
}

/// Retained scalar distance parameter.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchDistance {
    /// Exact distance value parameter.
    pub value: SketchParameterHandle,
}

/// Retained two-dimensional normal/direction carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchNormal2 {
    /// X component parameter.
    pub x: SketchParameterHandle,
    /// Y component parameter.
    pub y: SketchParameterHandle,
}

/// Retained three-dimensional normal/quaternion carrier.
///
/// SolveSpace represents normals with quaternion-like parameters and derives
/// frame directions from that retained object. Hyper keeps the components as
/// exact parameters and leaves unit-length/frame certification to explicit
/// residual or predicate reports, matching Yap's construction/proof split.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchNormal3 {
    /// Scalar component.
    pub w: SketchParameterHandle,
    /// X component.
    pub x: SketchParameterHandle,
    /// Y component.
    pub y: SketchParameterHandle,
    /// Z component.
    pub z: SketchParameterHandle,
}

/// Retained 2D line segment carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchLineSegment2 {
    /// Start point entity.
    pub start: SketchEntityHandle,
    /// End point entity.
    pub end: SketchEntityHandle,
}

/// Retained 3D line segment carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchLineSegment3 {
    /// Start point entity.
    pub start: SketchEntityHandle,
    /// End point entity.
    pub end: SketchEntityHandle,
}

/// Retained 2D circle carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchCircle2 {
    /// Center point entity.
    pub center: SketchEntityHandle,
    /// Radius parameter or distance entity.
    pub radius: SketchEntityHandle,
}

/// Retained 2D circular-arc carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchArcOfCircle2 {
    /// Center point entity.
    pub center: SketchEntityHandle,
    /// Start point entity.
    pub start: SketchEntityHandle,
    /// End point entity.
    pub end: SketchEntityHandle,
    /// Radius parameter or distance entity.
    pub radius: SketchEntityHandle,
}

/// Retained 2D cubic Bezier carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchCubic2 {
    /// First control point.
    pub p0: SketchEntityHandle,
    /// Second control point.
    pub p1: SketchEntityHandle,
    /// Third control point.
    pub p2: SketchEntityHandle,
    /// Fourth control point.
    pub p3: SketchEntityHandle,
}

/// Retained workplane carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchWorkplane {
    /// Origin point entity.
    pub origin: SketchEntityHandle,
    /// Normal entity or construction handle.
    pub normal: SketchEntityHandle,
}

/// Retained sketch entity families.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchEntityKind {
    /// SolveSpace-style `POINT_IN_2D`.
    Point2D(SketchPoint2),
    /// SolveSpace-style `POINT_IN_3D`.
    Point3D(SketchPoint3),
    /// SolveSpace-style scalar `DISTANCE`.
    Distance(SketchDistance),
    /// SolveSpace-style `NORMAL_IN_2D`.
    Normal2D(SketchNormal2),
    /// SolveSpace-style `NORMAL_IN_3D`.
    Normal3D(SketchNormal3),
    /// SolveSpace-style `LINE_SEGMENT`.
    LineSegment2(SketchLineSegment2),
    /// SolveSpace-style 3D `LINE_SEGMENT`.
    LineSegment3(SketchLineSegment3),
    /// SolveSpace-style `CUBIC`.
    Cubic2(SketchCubic2),
    /// SolveSpace-style `CIRCLE`.
    Circle2(SketchCircle2),
    /// SolveSpace-style `ARC_OF_CIRCLE`.
    ArcOfCircle2(SketchArcOfCircle2),
    /// SolveSpace-style `WORKPLANE`.
    Workplane(SketchWorkplane),
}

/// Endpoint selector for retained circular-arc constraints.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchArcEndpoint {
    /// Use the arc's retained start point.
    Start,
    /// Use the arc's retained end point.
    End,
}

/// Endpoint selector for retained line-segment constraints.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchLineEndpoint {
    /// Use the line's retained start point and orient the tangent toward the
    /// line's end point.
    Start,
    /// Use the line's retained end point and orient the tangent toward the
    /// line's start point.
    End,
}

/// Orientation branch for retained 2D tangent constraints.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchTangentOrientation {
    /// The outgoing line tangent is counterclockwise from the radius vector,
    /// certified by `radius x tangent >= 0`.
    CounterClockwise,
    /// The outgoing line tangent is clockwise from the radius vector,
    /// certified by `radius x tangent <= 0`.
    Clockwise,
}

/// Radius-vector branch for retained 2D arc-arc tangency.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchArcTangencyBranch {
    /// The selected radius vectors point in the same direction, certified by
    /// `r_a . r_b >= 0`.
    SameRadiusDirection,
    /// The selected radius vectors point in opposite directions, certified by
    /// `r_a . r_b <= 0`.
    OppositeRadiusDirection,
}

/// A retained source entity.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchEntity {
    /// Stable source handle.
    pub handle: SketchEntityHandle,
    /// Optional group ownership.
    pub group: Option<SketchGroupHandle>,
    /// Human-readable source name.
    pub name: String,
    /// Entity payload.
    pub kind: SketchEntityKind,
    /// Exact entity-domain obligations checked before numerical iteration.
    pub domains: Vec<SketchEntityDomain>,
    /// Editor/API round-trip metadata.
    pub metadata: SketchRoundTripMetadata,
}

/// Retained high-level sketch constraint families.
#[derive(Clone, Debug, PartialEq)]
pub enum SketchConstraintKind {
    /// Coincident 2D or 3D points.
    PointsCoincident {
        /// First point entity.
        a: SketchEntityHandle,
        /// Second point entity.
        b: SketchEntityHandle,
    },
    /// Point-to-point squared-distance constraint.
    PointPointDistance {
        /// First point entity.
        a: SketchEntityHandle,
        /// Second point entity.
        b: SketchEntityHandle,
        /// Distance entity.
        distance: SketchEntityHandle,
    },
    /// Point-to-point bounded-distance relation.
    ///
    /// Bounds are exact distance values. Lowering uses squared-distance
    /// inequalities only after proving that the bounds are nonnegative, so a
    /// negative upper bound cannot be accidentally accepted by squaring. This
    /// follows the exact-geometric-computation boundary advocated by Yap,
    /// "Towards Exact Geometric Computation" (1997): retain the semantic
    /// object, then lower only to certified exact predicates.
    PointPointDistanceRange {
        /// First point entity.
        a: SketchEntityHandle,
        /// Second point entity.
        b: SketchEntityHandle,
        /// Optional exact lower distance bound.
        lower: Option<Real>,
        /// Optional exact upper distance bound.
        upper: Option<Real>,
    },
    /// Bounded point-to-point distance after projection into a workplane.
    ///
    /// Bounds are exact distance values in the retained workplane metric.
    /// Lowering first validates that the lower and upper bounds are
    /// nonnegative and ordered, then emits the workplane unit-quaternion guard
    /// and squared projected-distance inequalities. This is the range
    /// counterpart of [`SketchConstraintKind::ProjectedPointPointDistance`].
    /// It follows Yap, "Towards Exact Geometric Computation" (1997), by
    /// keeping invalid semantic bounds as diagnostics instead of accepting
    /// misleading squared rows. The retained frame uses Shoemake's
    /// unit-quaternion rotation matrix from "Animating Rotation with
    /// Quaternion Curves" (1985).
    ProjectedPointPointDistanceRange {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// First 3D point entity.
        a: SketchEntityHandle,
        /// Second 3D point entity.
        b: SketchEntityHandle,
        /// Optional exact lower projected distance bound.
        lower: Option<Real>,
        /// Optional exact upper projected distance bound.
        upper: Option<Real>,
    },
    /// Point-to-point distance after orthogonal projection into a workplane.
    ///
    /// The retained relation is SolveSpace-style projected distance: two 3D
    /// points are interpreted through a retained workplane and only their
    /// `U/V` displacement is compared to the exact distance carrier. Lowering
    /// emits a unit-quaternion guard for the workplane plus
    /// `(delta . U)^2 + (delta . V)^2 - d^2 == 0`. The guard is part of the
    /// generated proof package because the polynomial projection is valid only
    /// for a certified unit frame. This follows Yap, "Towards Exact Geometric
    /// Computation" (1997), by keeping projection as retained object
    /// structure and accepting candidates only through exact residual replay.
    /// The exact `U/V/N` axes use the standard unit-quaternion rotation matrix;
    /// see Shoemake, "Animating Rotation with Quaternion Curves" (1985).
    ProjectedPointPointDistance {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// First 3D point entity.
        a: SketchEntityHandle,
        /// Second 3D point entity.
        b: SketchEntityHandle,
        /// Distance entity.
        distance: SketchEntityHandle,
    },
    /// Point-to-line distance after orthogonal projection into a workplane.
    ///
    /// This retained 3D/workplane relation projects a 3D point and a retained
    /// 3D line segment into the workplane `U/V` frame, then lowers the usual
    /// point-line distance equation without division:
    /// `cross(P-A, B-A)_uv^2 - d^2 * |B-A|_uv^2 == 0`. A unit-quaternion
    /// guard is emitted beside the distance row because the projected metric
    /// is valid only for a certified workplane frame. The proof package keeps
    /// degeneracy and projection-domain assumptions explicit, following Yap,
    /// "Towards Exact Geometric Computation" (1997); the frame polynomial is
    /// the standard unit-quaternion rotation matrix from Shoemake, "Animating
    /// Rotation with Quaternion Curves" (1985).
    ProjectedPointLineDistance {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// 3D point entity.
        point: SketchEntityHandle,
        /// 3D line segment entity.
        line: SketchEntityHandle,
        /// Distance entity.
        distance: SketchEntityHandle,
    },
    /// Equality between two point-line distances after workplane projection.
    ///
    /// Lowering projects both 3D point/line pairs into the retained workplane
    /// frame and cross-multiplies their squared distance expressions:
    /// `cross(a)^2 * |line_b_uv|^2 - cross(b)^2 * |line_a_uv|^2 == 0`.
    /// This keeps both line-length denominators explicit and avoids square
    /// roots or normalized projected line coordinates. The proof boundary
    /// follows Yap, "Towards Exact Geometric Computation" (1997); the
    /// workplane axes use Shoemake's unit-quaternion rotation matrix from
    /// "Animating Rotation with Quaternion Curves" (1985).
    ProjectedEqualPointLineDistances3 {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// First 3D point entity.
        a_point: SketchEntityHandle,
        /// First 3D line segment entity.
        a_line: SketchEntityHandle,
        /// Second 3D point entity.
        b_point: SketchEntityHandle,
        /// Second 3D line segment entity.
        b_line: SketchEntityHandle,
    },
    /// Equal projected lengths for two 3D line segments in a retained workplane.
    ///
    /// Lowering projects both 3D line directions into the retained workplane
    /// `U/V` frame and emits `|dir(a)_uv|^2 - |dir(b)_uv|^2 == 0` plus the
    /// workplane unit-quaternion guard. This is the 3D/workplane counterpart
    /// of [`SketchConstraintKind::EqualLengthLines2`]. It follows Yap,
    /// "Towards Exact Geometric Computation" (1997), by retaining the
    /// workplane object and replaying a polynomial proof row instead of
    /// accepting lossy projected coordinates or square-root lengths.
    ProjectedEqualLengthLines3 {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// First 3D line segment.
        a: SketchEntityHandle,
        /// Second 3D line segment.
        b: SketchEntityHandle,
    },
    /// Projected 3D line length-ratio relation in a retained workplane.
    ///
    /// The retained relation is `length(a_uv) / length(b_uv) = numerator /
    /// denominator`, where `a_uv` and `b_uv` are the exact projections of 3D
    /// line directions into the workplane frame. Lowering emits
    /// `|a_uv|^2 * denominator^2 - |b_uv|^2 * numerator^2 == 0` after exact
    /// ratio sign validation, plus the workplane unit-quaternion guard. This
    /// follows Yap, "Towards Exact Geometric Computation" (1997): squaring is
    /// accepted only after semantic ratio-domain checks are explicit.
    ProjectedLengthRatioLines3 {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// First 3D line segment.
        a: SketchEntityHandle,
        /// Second 3D line segment.
        b: SketchEntityHandle,
        /// Exact nonnegative ratio numerator.
        numerator: Real,
        /// Exact strictly positive ratio denominator.
        denominator: Real,
    },
    /// Projected 3D line length-difference relation in a retained workplane.
    ///
    /// The retained relation is `length(longer_uv) = length(shorter_uv) +
    /// difference`, where line directions are projected into the workplane
    /// frame. Lowering reuses the square-root-free proof package
    /// `(L + S - d^2)^2 - 4LS == 0` plus `L - S - d^2 >= 0`, where `L` and
    /// `S` are projected squared lengths. The branch inequality keeps the
    /// intended ordering explicit rather than hidden in a proposal solver,
    /// following Yap, "Towards Exact Geometric Computation" (1997).
    ProjectedLengthDifferenceLines3 {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// Line expected to have the longer projected length.
        longer: SketchEntityHandle,
        /// Line expected to have the shorter projected length.
        shorter: SketchEntityHandle,
        /// Distance entity for the exact projected length difference.
        difference: SketchEntityHandle,
    },
    /// Equality between a projected 3D line length and projected point-line distance.
    ///
    /// The retained relation is
    /// `length(length_line_uv) = distance(point_uv, distance_line_uv)`.
    /// Lowering clears the point-line denominator and emits
    /// `|length_line_uv|^2 * |distance_line_uv|^2 - cross_uv^2 == 0` beside
    /// the workplane unit-quaternion guard. This is the workplane counterpart
    /// of [`SketchConstraintKind::EqualLengthPointLineDistance2`]. It follows
    /// Yap, "Towards Exact Geometric Computation" (1997), by proving the
    /// projected metric relation with polynomial replay rather than trusting
    /// normalized projected coordinates; the frame polynomial is Shoemake's
    /// unit-quaternion rotation matrix from "Animating Rotation with
    /// Quaternion Curves" (1985).
    ProjectedEqualLengthPointLineDistance3 {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// 3D line whose projected length is compared.
        length_line: SketchEntityHandle,
        /// 3D point used for the projected point-line distance.
        point: SketchEntityHandle,
        /// 3D line used as the projected distance carrier.
        distance_line: SketchEntityHandle,
    },
    /// 2D line-segment equal-length relation.
    ///
    /// Lowering emits `|dir(a)|^2 - |dir(b)|^2 == 0`, a polynomial exact
    /// replay row. True line lengths remain proposal/UI data for future
    /// multi-form packages; the proof row follows Yap, "Towards Exact
    /// Geometric Computation" (1997), by avoiding square-root decisions unless
    /// they are explicitly certified.
    EqualLengthLines2 {
        /// First line entity.
        a: SketchEntityHandle,
        /// Second line entity.
        b: SketchEntityHandle,
    },
    /// 2D line-segment length-ratio relation.
    ///
    /// The retained relation is `length(a) / length(b) = numerator /
    /// denominator`. Lowering emits
    /// `|dir(a)|^2 * denominator^2 - |dir(b)|^2 * numerator^2 == 0` after
    /// proving the ratio inputs are nonnegative and the denominator is
    /// positive. This is the squared algebraic proof form preferred by Yap,
    /// "Towards Exact Geometric Computation" (1997); true lengths remain a
    /// proposal/UI concern.
    LengthRatioLines2 {
        /// First line entity.
        a: SketchEntityHandle,
        /// Second line entity.
        b: SketchEntityHandle,
        /// Exact nonnegative ratio numerator.
        numerator: Real,
        /// Exact strictly positive ratio denominator.
        denominator: Real,
    },
    /// 2D line-segment length-difference relation.
    ///
    /// The retained relation is `length(longer) = length(shorter) +
    /// difference`. Lowering emits the exact polynomial
    /// `(A + B - d^2)^2 - 4AB == 0`, where `A = |dir(longer)|^2` and
    /// `B = |dir(shorter)|^2`, plus `A - B - d^2 >= 0` to reject the
    /// opposite ordering branch. This follows Yap, "Towards Exact Geometric
    /// Computation" (1997), by keeping square roots out of the proof row while
    /// making branch assumptions explicit.
    LengthDifferenceLines2 {
        /// Line expected to be longer.
        longer: SketchEntityHandle,
        /// Line expected to be shorter.
        shorter: SketchEntityHandle,
        /// Distance entity for the exact length difference.
        difference: SketchEntityHandle,
    },
    /// 2D point-to-line distance relation.
    ///
    /// Lowering emits `cross(point-start, dir)^2 - distance^2*|dir|^2 == 0`.
    /// This polynomial row avoids dividing by line length or taking square
    /// roots during certification, in the exact-replay style described by Yap,
    /// "Towards Exact Geometric Computation" (1997).
    PointLineDistance2 {
        /// Point entity.
        point: SketchEntityHandle,
        /// 2D line entity.
        line: SketchEntityHandle,
        /// Distance entity.
        distance: SketchEntityHandle,
    },
    /// 2D line length equals a 2D point-to-line distance.
    ///
    /// Lowering emits `|dir(length_line)|^2 * |dir(distance_line)|^2 -
    /// cross(point-start, dir(distance_line))^2 == 0`, avoiding division by
    /// line length. Degenerate-line assumptions remain explicit domain
    /// obligations, following Yap, "Towards Exact Geometric Computation"
    /// (1997): construction convenience is separate from certified predicates.
    EqualLengthPointLineDistance2 {
        /// Line whose length is compared.
        length_line: SketchEntityHandle,
        /// Point used by the point-line distance.
        point: SketchEntityHandle,
        /// Line used by the point-line distance.
        distance_line: SketchEntityHandle,
    },
    /// Equality between two 2D point-to-line distances.
    ///
    /// Lowering cross-multiplies squared point-line distance numerators and
    /// line squared norms, so exact replay does not need square roots,
    /// divisions, or primitive-float tolerances.
    EqualPointLineDistances2 {
        /// First point entity.
        a_point: SketchEntityHandle,
        /// First line entity.
        a_line: SketchEntityHandle,
        /// Second point entity.
        b_point: SketchEntityHandle,
        /// Second line entity.
        b_line: SketchEntityHandle,
    },
    /// 2D circle/arc equal-radius relation.
    ///
    /// Lowering emits direct exact radius-parameter equality. Positive-radius
    /// policy remains an explicit entity-domain obligation, not a hidden
    /// tolerance or normalization step.
    EqualRadius2 {
        /// First circle or circular-arc entity.
        a: SketchEntityHandle,
        /// Second circle or circular-arc entity.
        b: SketchEntityHandle,
    },
    /// 2D horizontal line constraint.
    Horizontal {
        /// Line entity.
        line: SketchEntityHandle,
    },
    /// 2D vertical line constraint.
    Vertical {
        /// Line entity.
        line: SketchEntityHandle,
    },
    /// 2D line parallelism constraint.
    ///
    /// Lowering emits the exact 2D direction cross product. The retained
    /// source relation stays visible for failed-constraint diagnostics, while
    /// ordinary exact replay remains the trust boundary; see Yap, "Towards
    /// Exact Geometric Computation" (1997).
    ParallelLines2 {
        /// First line entity.
        a: SketchEntityHandle,
        /// Second line entity.
        b: SketchEntityHandle,
    },
    /// 2D line perpendicularity constraint.
    ///
    /// Lowering emits the exact 2D direction dot product. Degenerate line
    /// carriers are represented by explicit entity-domain/degen preflight
    /// reports rather than hidden normalization tolerances.
    PerpendicularLines2 {
        /// First line entity.
        a: SketchEntityHandle,
        /// Second line entity.
        b: SketchEntityHandle,
    },
    /// 2D line same-orientation relation.
    ///
    /// Lowering emits exact, unnormalized direction predicates: cross product
    /// equality for parallel support and dot product nonnegativity for common
    /// orientation. This mirrors the exact predicate style advocated by Yap,
    /// "Towards Exact Geometric Computation" (1997), while leaving degenerate
    /// line carriers to explicit entity-domain preflight reports.
    SameDirectionLines2 {
        /// First line entity.
        a: SketchEntityHandle,
        /// Second line entity.
        b: SketchEntityHandle,
    },
    /// 2D same-direction G1 tangent-carrier relation.
    ///
    /// Lowering emits exact, unnormalized tangent predicates: cross-product
    /// equality for common tangent support and dot-product nonnegativity for
    /// the same-direction branch. Tangent construction and smoothing may be
    /// proposal behavior, but accepted candidates must pass exact replay in
    /// the style of Yap, "Towards Exact Geometric Computation" (1997).
    TangentSameDirectionLines2 {
        /// Candidate tangent-carrier line entity.
        candidate: SketchEntityHandle,
        /// Target tangent-carrier line entity.
        target: SketchEntityHandle,
    },
    /// 2D circular-arc endpoint tangent to a line endpoint.
    ///
    /// This is the first retained `ARC_LINE_TANGENT` package. The caller
    /// explicitly selects the arc endpoint, line endpoint, and orientation
    /// branch. Lowering emits endpoint incidence, an arc endpoint-on-radius
    /// row, radius/tangent perpendicularity, and a signed orientation
    /// inequality without normalizing the radius or tangent. Degenerate arc
    /// and line objects remain explicit entity-domain obligations, following
    /// Yap, "Towards Exact Geometric Computation" (1997), and the retained
    /// endpoint/orientation vocabulary follows Bouma et al., "A Geometric
    /// Constraint Solver" (1995).
    ArcLineTangent2 {
        /// Circular arc entity.
        arc: SketchEntityHandle,
        /// Endpoint of the arc where tangency is enforced.
        arc_endpoint: SketchArcEndpoint,
        /// Line segment entity.
        line: SketchEntityHandle,
        /// Endpoint of the line that must coincide with the arc endpoint.
        line_endpoint: SketchLineEndpoint,
        /// Signed orientation branch for the outgoing line tangent.
        orientation: SketchTangentOrientation,
    },
    /// 2D circular-arc endpoint tangent to another circular-arc endpoint.
    ///
    /// Lowering emits selected endpoint-on-radius rows for both arcs, endpoint
    /// coordinate coincidence, radius-vector collinearity, and an explicit
    /// same/opposite radius-vector branch. This covers retained arc-arc
    /// tangency without deriving contact topology from primitive floats.
    /// Degenerate arcs and ambiguous zero-radius branches remain explicit
    /// entity-domain obligations, following Yap, "Towards Exact Geometric
    /// Computation" (1997), and the endpoint-aware branch vocabulary follows
    /// Bouma et al., "A Geometric Constraint Solver" (1995).
    ArcArcTangent2 {
        /// First circular arc entity.
        first: SketchEntityHandle,
        /// Endpoint of the first arc where tangency is enforced.
        first_endpoint: SketchArcEndpoint,
        /// Second circular arc entity.
        second: SketchEntityHandle,
        /// Endpoint of the second arc where tangency is enforced.
        second_endpoint: SketchArcEndpoint,
        /// Same/opposite radius-vector branch.
        branch: SketchArcTangencyBranch,
    },
    /// 2D circular-arc endpoint tangent to a cubic Bezier at a retained parameter.
    ///
    /// Lowering evaluates the cubic point and derivative exactly in Bernstein
    /// form, then emits selected arc endpoint-on-radius, cubic/endpoint
    /// incidence, radius/cubic-derivative perpendicularity, and a signed
    /// orientation branch. No radius, speed, or curvature is normalized.
    /// Degenerate arcs, stationary cubic parameters, and segment-domain policy
    /// remain explicit report-bearing obligations, following Yap, "Towards
    /// Exact Geometric Computation" (1997). The derivative control net follows
    /// Farin's Bernstein/de Casteljau construction.
    ArcCubicTangent2 {
        /// Circular arc entity.
        arc: SketchEntityHandle,
        /// Endpoint of the arc where tangency is enforced.
        arc_endpoint: SketchArcEndpoint,
        /// Retained 2D cubic Bezier entity.
        cubic: SketchEntityHandle,
        /// Curve parameter used for point and derivative evaluation.
        parameter: SketchParameterHandle,
        /// Signed orientation branch for the outgoing cubic tangent.
        orientation: SketchTangentOrientation,
    },
    /// 2D circular-arc endpoint second-order contact with a cubic Bezier.
    ///
    /// Lowering extends [`SketchConstraintKind::ArcCubicTangent2`] with the
    /// exact circle-contact second derivative row
    /// `B'(t).B'(t) + (B(t)-C).B''(t) == 0`. This is a curvature-sensitive
    /// retained proof package, not a sampled osculating-circle heuristic. It
    /// avoids division by speed, radius, or curvature; stationary cubic
    /// parameters and degenerate arcs stay explicit domain obligations,
    /// following Yap, "Towards Exact Geometric Computation" (1997). The cubic
    /// derivative control nets follow Farin's Bernstein/de Casteljau model.
    ArcCubicSecondOrderContact2 {
        /// Circular arc entity.
        arc: SketchEntityHandle,
        /// Endpoint of the arc where contact is enforced.
        arc_endpoint: SketchArcEndpoint,
        /// Retained 2D cubic Bezier entity.
        cubic: SketchEntityHandle,
        /// Curve parameter used for point and derivative evaluation.
        parameter: SketchParameterHandle,
        /// Signed orientation branch for the outgoing cubic tangent.
        orientation: SketchTangentOrientation,
    },
    /// 2D cubic-Bezier tangent to a selected line endpoint at a retained parameter.
    ///
    /// Lowering evaluates the retained cubic point and derivative exactly in
    /// Bernstein form, constrains the selected line endpoint to the curve
    /// point, emits `B'(t) x line_tangent == 0`, and emits
    /// `B'(t) . line_tangent >= 0` for the same-direction tangent branch.
    /// No derivative or line vector is normalized. Degenerate tangents and the
    /// usual segment domain for `t` remain explicit preflight/domain
    /// obligations, following Yap, "Towards Exact Geometric Computation"
    /// (1997). The derivative control-net formula is the standard
    /// de Casteljau/Bernstein construction; see Farin, *Curves and Surfaces
    /// for CAGD*, 5th ed. (2002).
    CubicLineTangent2 {
        /// Retained 2D cubic Bezier entity.
        cubic: SketchEntityHandle,
        /// Curve parameter used for point and derivative evaluation.
        parameter: SketchParameterHandle,
        /// Line segment entity whose selected endpoint lies on the cubic.
        line: SketchEntityHandle,
        /// Endpoint of the line used for incidence and outgoing tangent.
        line_endpoint: SketchLineEndpoint,
    },
    /// 2D cubic-Bezier tangent to another cubic Bezier at retained parameters.
    ///
    /// Lowering evaluates both retained cubic points and derivatives exactly
    /// in Bernstein form, emits coordinate coincidence for the two curve
    /// points, and certifies a same-direction derivative branch with
    /// `B_a'(t_a) x B_b'(t_b) == 0` and
    /// `B_a'(t_a) . B_b'(t_b) >= 0`. This is a retained differential
    /// constraint, not a sampled curve heuristic. Degenerate derivative and
    /// segment-domain assumptions stay in explicit preflight/domain reports,
    /// following Yap, "Towards Exact Geometric Computation" (1997), while the
    /// derivative formula follows Farin's Bernstein/de Casteljau model.
    CubicCubicTangent2 {
        /// First retained 2D cubic Bezier entity.
        first: SketchEntityHandle,
        /// Parameter on the first cubic.
        first_parameter: SketchParameterHandle,
        /// Second retained 2D cubic Bezier entity.
        second: SketchEntityHandle,
        /// Parameter on the second cubic.
        second_parameter: SketchParameterHandle,
    },
    /// Geometric G2 curvature continuity between two retained 2D cubic Beziers.
    ///
    /// Lowering evaluates both cubic points, first derivatives, and second
    /// derivatives exactly in Bernstein form. It emits point coincidence,
    /// same-direction tangent branch rows, and a polynomial signed-curvature
    /// equality:
    /// `cross(Ba',Ba'')^2 * |Bb'|^6 - cross(Bb',Bb'')^2 * |Ba'|^6 == 0`,
    /// plus `cross(Ba',Ba'') * cross(Bb',Bb'') >= 0` for the same signed
    /// curvature branch. This is geometric G2, not parametric C2; no speed or
    /// curvature value is normalized through primitive floats. Degenerate
    /// derivative assumptions stay explicit, following Yap, "Towards Exact
    /// Geometric Computation" (1997), while the derivative control nets follow
    /// Farin's Bernstein/de Casteljau construction.
    CubicCubicG2Continuity2 {
        /// First retained 2D cubic Bezier entity.
        first: SketchEntityHandle,
        /// Parameter on the first cubic.
        first_parameter: SketchParameterHandle,
        /// Second retained 2D cubic Bezier entity.
        second: SketchEntityHandle,
        /// Parameter on the second cubic.
        second_parameter: SketchParameterHandle,
    },
    /// Parametric C2 continuity between two retained 2D cubic Beziers.
    ///
    /// Lowering evaluates both cubic points, first derivatives, and second
    /// derivatives exactly in Bernstein form, then emits coordinate equality
    /// rows for `B_a(t_a) = B_b(t_b)`, `B_a'(t_a) = B_b'(t_b)`, and
    /// `B_a''(t_a) = B_b''(t_b)`. This is intentionally parametric C2, not a
    /// sampled or normalized geometric-curvature test. Segment-domain and
    /// degeneracy assumptions remain explicit obligations, following Yap,
    /// "Towards Exact Geometric Computation" (1997), while the derivative
    /// control nets follow Farin's Bernstein/de Casteljau construction.
    CubicCubicC2Continuity2 {
        /// First retained 2D cubic Bezier entity.
        first: SketchEntityHandle,
        /// Parameter on the first cubic.
        first_parameter: SketchParameterHandle,
        /// Second retained 2D cubic Bezier entity.
        second: SketchEntityHandle,
        /// Parameter on the second cubic.
        second_parameter: SketchParameterHandle,
    },
    /// Unsigned equal-angle relation between two 2D line pairs.
    ///
    /// Lowering compares squared cosines, which certifies ordinary equal
    /// unsigned angles without evaluating `acos` or any primitive-float angle.
    /// Oriented-angle branch choices remain future explicit predicate packages.
    EqualAngleLines2 {
        /// First line in the first angle.
        a: SketchEntityHandle,
        /// Second line in the first angle.
        b: SketchEntityHandle,
        /// First line in the second angle.
        c: SketchEntityHandle,
        /// Second line in the second angle.
        d: SketchEntityHandle,
    },
    /// Oriented equality between two 2D line-pair angles.
    ///
    /// Lowering compares the exact angle vectors `(dot, cross)` for the two
    /// line pairs. A polynomial collinearity row proves equal tangent of the
    /// angles, while a dot-product inequality selects the same oriented branch
    /// rather than the supplemental branch. This is the `ANGLE` proof package
    /// used when branch direction matters: Yap, "Towards Exact Geometric
    /// Computation" (1997), motivates keeping the exact rows separate from
    /// any `atan2` proposal value, and Bouma et al., "A Geometric Constraint
    /// Solver" (1995), motivates retaining angle relations semantically.
    EqualOrientedAngleLines2 {
        /// First line in the first oriented angle.
        a: SketchEntityHandle,
        /// Second line in the first oriented angle.
        b: SketchEntityHandle,
        /// First line in the second oriented angle.
        c: SketchEntityHandle,
        /// Second line in the second oriented angle.
        d: SketchEntityHandle,
    },
    /// Oriented equality between two 3D line-pair angles projected to a workplane.
    ///
    /// Lowering first emits a unit-quaternion guard for the retained
    /// workplane, then projects all four 3D line directions into the exact
    /// `U/V` frame and reuses the oriented angle-vector package
    /// `(dot, cross)`. This is a 3D/workplane variant of
    /// [`SketchConstraintKind::EqualOrientedAngleLines2`]: no projected
    /// coordinates are rounded, no line directions are normalized, and no
    /// `atan2` value is trusted as evidence. Degenerate 3D lines and zero
    /// projected directions remain explicit domain/preflight obligations,
    /// following Yap, "Towards Exact Geometric Computation" (1997), with the
    /// workplane frame polynomial from Shoemake, "Animating Rotation with
    /// Quaternion Curves" (1985).
    ProjectedEqualOrientedAngleLines3 {
        /// Workplane entity that supplies origin and normal/quaternion.
        workplane: SketchEntityHandle,
        /// First 3D line in the first oriented angle.
        a: SketchEntityHandle,
        /// Second 3D line in the first oriented angle.
        b: SketchEntityHandle,
        /// First 3D line in the second oriented angle.
        c: SketchEntityHandle,
        /// Second 3D line in the second oriented angle.
        d: SketchEntityHandle,
    },
    /// 2D point-at-midpoint relation.
    ///
    /// Lowering emits exact linear coordinate equations without averaging in
    /// floating point: `2*point - a - b == 0` per axis. The retained midpoint
    /// object keeps SolveSpace-style provenance visible while ordinary exact
    /// residual replay remains the proof boundary, following Yap, "Towards
    /// Exact Geometric Computation" (1997).
    AtMidpoint2 {
        /// Point constrained to the midpoint.
        point: SketchEntityHandle,
        /// First endpoint entity.
        a: SketchEntityHandle,
        /// Second endpoint entity.
        b: SketchEntityHandle,
    },
    /// 2D points symmetric across the horizontal line `y = axis_y`.
    ///
    /// Lowering emits `a.x - b.x == 0` and `a.y + b.y - 2*axis_y == 0`.
    /// Retaining the axis value exactly keeps SolveSpace-style horizontal
    /// symmetry explicit while candidate acceptance follows Yap, "Towards
    /// Exact Geometric Computation" (1997), through exact replay.
    SymmetricHorizontal2 {
        /// First point entity.
        a: SketchEntityHandle,
        /// Second point entity.
        b: SketchEntityHandle,
        /// Exact horizontal mirror-axis coordinate.
        axis_y: Real,
    },
    /// 2D points symmetric across the vertical line `x = axis_x`.
    ///
    /// Lowering emits `a.y - b.y == 0` and `a.x + b.x - 2*axis_x == 0`.
    /// The mirror axis is retained as exact data rather than a hidden UI
    /// tolerance or floating construction.
    SymmetricVertical2 {
        /// First point entity.
        a: SketchEntityHandle,
        /// Second point entity.
        b: SketchEntityHandle,
        /// Exact vertical mirror-axis coordinate.
        axis_x: Real,
    },
    /// 2D points symmetric across a retained 2D line.
    ///
    /// Lowering emits two exact polynomial rows: the midpoint of `a` and `b`
    /// lies on the mirror line, and the segment from `a` to `b` is
    /// perpendicular to that line. The rows use the unnormalized line
    /// direction, so degenerate-axis handling remains an explicit
    /// entity-domain obligation. This follows Yap, "Towards Exact Geometric
    /// Computation" (1997), by retaining the construction object while using
    /// exact predicates as the certification boundary.
    SymmetricLine2 {
        /// First point entity.
        a: SketchEntityHandle,
        /// Second point entity.
        b: SketchEntityHandle,
        /// 2D line segment used as the mirror axis.
        axis: SketchEntityHandle,
    },
    /// 3D points symmetric across a retained workplane.
    ///
    /// Lowering emits a unit-quaternion guard for the workplane frame, a
    /// midpoint-on-plane row, and three normal-offset cross-product rows.
    /// This is the 3D analogue of retained line symmetry: no reflected point
    /// is rounded into existence, no normal is normalized by primitive floats,
    /// and exact replay certifies the relation. The construction/proof split
    /// follows Yap, "Towards Exact Geometric Computation" (1997), and the
    /// retained workplane normal uses Shoemake's unit-quaternion frame.
    SymmetricWorkplane3 {
        /// First 3D point entity.
        a: SketchEntityHandle,
        /// Second 3D point entity.
        b: SketchEntityHandle,
        /// Workplane used as the mirror plane.
        workplane: SketchEntityHandle,
    },
    /// Point-on-circle incidence using squared radius residual.
    PointOnCircle {
        /// Point entity.
        point: SketchEntityHandle,
        /// Circle entity.
        circle: SketchEntityHandle,
    },
    /// 2D point-on-cubic-Bezier incidence at a retained parameter.
    ///
    /// Lowering emits the two exact Bernstein-coordinate equations
    /// `point.axis - B_axis(t) == 0`, where `B(t)` is the retained cubic
    /// Bezier control net. The parameter is retained as ordinary sketch data;
    /// callers should attach a [`SketchParameterDomain::Bounded`] obligation
    /// when they need the usual segment domain `0 <= t <= 1`. This keeps the
    /// proof package aligned with Yap, "Towards Exact Geometric Computation"
    /// (1997): the curve object and parameter are preserved, while exact
    /// residual replay, not sampled floating geometry, accepts candidates.
    /// The Bernstein form follows de Casteljau's Bezier construction as
    /// presented in Farin, *Curves and Surfaces for CAGD* (5th ed., 2002).
    PointOnCubic2 {
        /// Point constrained to the curve.
        point: SketchEntityHandle,
        /// Retained 2D cubic Bezier entity.
        cubic: SketchEntityHandle,
        /// Curve parameter used by the incidence equations.
        parameter: SketchParameterHandle,
    },
    /// Scalar parameter range constraint.
    ParameterRange {
        /// Parameter constrained by the range.
        parameter: SketchParameterHandle,
        /// Optional exact lower bound.
        lower: Option<Real>,
        /// Optional exact upper bound.
        upper: Option<Real>,
    },
    /// Nondecreasing relation between two scalar parameters.
    ///
    /// Lowering emits `upper - lower >= 0` as an exact inequality row. This is
    /// the generic monotonicity primitive for sketch parameters; domain-specific
    /// clearance or manufacturing interpretations should retain their own
    /// source objects and feed this only as a scalar proof obligation.
    ParameterOrdering {
        /// Parameter that must not exceed `upper`.
        lower: SketchParameterHandle,
        /// Parameter that must be greater than or equal to `lower`.
        upper: SketchParameterHandle,
    },
    /// Minimum exact separation between two ordered scalar parameters.
    ///
    /// Lowering emits `upper - lower - margin >= 0` after proving `margin` is
    /// nonnegative. This is the generic sketch-level design-rule-margin
    /// primitive; domain crates should still own fabrication-specific rule
    /// names, units, and geometry.
    ParameterMargin {
        /// Lower-side parameter.
        lower: SketchParameterHandle,
        /// Upper-side parameter.
        upper: SketchParameterHandle,
        /// Required exact nonnegative margin.
        margin: Real,
    },
    /// Soft objective that keeps a parameter near a target value.
    StayNearParameter {
        /// Parameter to bias.
        parameter: SketchParameterHandle,
        /// Exact target value.
        target: Real,
        /// Exact objective row weight.
        weight: Real,
    },
}

/// A retained high-level sketch constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchConstraint {
    /// Stable source handle.
    pub handle: SketchConstraintHandle,
    /// Optional group ownership.
    pub group: Option<SketchGroupHandle>,
    /// Human-readable source name.
    pub name: String,
    /// Whether this row contributes equations, not just diagnostics.
    pub reference: bool,
    /// Whether this row is an active proof obligation.
    pub active: bool,
    /// High-level constraint payload.
    pub kind: SketchConstraintKind,
    /// Editor/API round-trip metadata.
    pub metadata: SketchRoundTripMetadata,
}

/// Residual proof strategy selected while lowering a semantic constraint.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchResidualStrategy {
    /// Linear coordinate equality.
    CoordinateEquality,
    /// Squared Euclidean distance polynomial.
    SquaredDistance,
    /// Squared Euclidean distance inequality for bounded-distance constraints.
    BoundedSquaredDistance,
    /// Squared projected-distance inequality for bounded projected-distance constraints.
    BoundedSquaredProjectedDistance,
    /// Unit-quaternion guard for retained workplane projection rows.
    WorkplaneUnitQuaternion,
    /// Squared distance after exact projection onto a retained workplane.
    SquaredProjectedDistance,
    /// Squared point-line distance after exact projection onto a workplane.
    SquaredProjectedPointLineDistance,
    /// Squared equality between 3D line lengths after exact workplane projection.
    SquaredProjectedLineLengthEquality,
    /// Squared ratio between 3D line lengths after exact workplane projection.
    SquaredProjectedLineLengthRatio,
    /// Squared 3D line length difference after exact workplane projection.
    SquaredProjectedLineLengthDifference,
    /// Squared equality between projected 3D line length and projected point-line distance.
    SquaredProjectedLineLengthPointLineDistance,
    /// Squared equality between two projected 3D point-line distances.
    SquaredProjectedEqualPointLineDistances,
    /// Squared 2D line length equality.
    SquaredLineLengthEquality,
    /// Squared 2D line length ratio equality.
    SquaredLineLengthRatio,
    /// Squared 2D line length difference equality/order package.
    SquaredLineLengthDifference,
    /// Squared 2D point-to-line distance equality.
    SquaredPointLineDistance,
    /// Squared 2D line-length-to-point-line-distance equality.
    SquaredLineLengthPointLineDistance,
    /// Squared equality between two 2D point-line distances.
    SquaredEqualPointLineDistances,
    /// Exact retained radius equality.
    RadiusEquality,
    /// Incidence represented as a squared-distance polynomial.
    SquaredIncidence,
    /// Exact cubic-Bezier coordinate incidence in Bernstein form.
    CubicBezierIncidence,
    /// Scalar parameter bound.
    ParameterRange,
    /// Scalar nondecreasing parameter relation.
    ParameterOrdering,
    /// Scalar ordered relation with an exact nonnegative margin.
    ParameterMargin,
    /// 2D direction cross-product equality.
    DirectionCrossProduct,
    /// 2D direction dot-product equality.
    DirectionDotProduct,
    /// Exact same-orientation relation for two retained 2D directions.
    DirectionSameOrientation,
    /// Exact same-direction predicate package for retained G1 tangent carriers.
    TangentSameDirection,
    /// Exact endpoint/radius/tangent predicate package for retained arc-line tangency.
    ArcLineTangent,
    /// Exact endpoint/radius/radius-branch package for retained arc-arc tangency.
    ArcArcTangent,
    /// Exact endpoint/radius/cubic-derivative package for retained arc-cubic tangency.
    ArcCubicTangent,
    /// Exact endpoint/radius/cubic first- and second-derivative contact package.
    ArcCubicSecondOrderContact,
    /// Exact point/derivative package for retained cubic-line tangency.
    CubicLineTangent,
    /// Exact point/derivative package for retained cubic-cubic tangency.
    CubicCubicTangent,
    /// Exact point/tangent/signed-curvature package for geometric cubic G2 continuity.
    CubicCubicG2Continuity,
    /// Exact point/first/second derivative rows for parametric cubic C2 continuity.
    CubicCubicC2Continuity,
    /// Squared-cosine equality for unsigned 2D line angles.
    SquaredCosineAngleEquality,
    /// Exact angle-vector package for oriented 2D line-pair angles.
    OrientedAngleEquality,
    /// Exact workplane-projected oriented angle package for 3D line-pair angles.
    ProjectedOrientedAngleEquality,
    /// Linear coordinate equality for a retained midpoint relation.
    MidpointCoordinateEquality,
    /// Linear coordinate equality for retained axis symmetry.
    AxisSymmetryCoordinateEquality,
    /// Exact midpoint-on-axis/perpendicular rows for retained line symmetry.
    LineSymmetryPolynomial,
    /// Exact midpoint-on-plane/normal-offset rows for retained workplane symmetry.
    WorkplaneSymmetryPolynomial,
    /// Soft stay-near objective.
    SoftObjective,
}

/// Status for one generated residual row.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchGeneratedRowStatus {
    /// The row was generated into the lowered [`Problem`].
    Generated,
    /// A referenced entity handle was not present.
    MissingEntity(SketchEntityHandle),
    /// A referenced parameter handle was not present.
    MissingParameter(SketchParameterHandle),
    /// The referenced entity family is incompatible with this constraint.
    WrongEntityKind {
        /// The rejected entity handle.
        handle: SketchEntityHandle,
        /// Expected entity family.
        expected: &'static str,
    },
    /// The source constraint is diagnostic/reference-only and emitted no row.
    ReferenceOnly,
    /// The source exact bound is invalid for this relation.
    InvalidExactBound,
    /// The source exact bound could not be certified enough for safe lowering.
    UnresolvedExactBound,
}

/// Provenance for one residual row produced from a high-level sketch
/// constraint.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SketchGeneratedRow {
    /// Source sketch constraint.
    pub constraint: SketchConstraintHandle,
    /// Residual index in the lowered [`Problem`], when generated.
    pub residual_index: Option<usize>,
    /// Source name assigned to the residual row.
    pub name: String,
    /// Selected proof strategy.
    pub strategy: Option<SketchResidualStrategy>,
    /// Generation status.
    pub status: SketchGeneratedRowStatus,
}

/// Residual form retained for one high-level sketch relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchResidualFormKind {
    /// `|a-b|^2 - d^2`, retained as a polynomial exact-replay form.
    SquaredDistancePolynomial,
    /// `sqrt(|a-b|^2) - d`, retained for proposal compatibility.
    TrueDistanceProposal,
    /// `w^2+x^2+y^2+z^2-1`, retained as a workplane-frame proof guard.
    WorkplaneUnitQuaternionPolynomial,
    /// Squared projected-distance polynomial proof row.
    SquaredProjectedDistancePolynomial,
    /// Squared projected-distance range inequality proof row.
    BoundedSquaredProjectedDistancePolynomial,
    /// True projected distance residual retained for proposal/UI parity.
    TrueProjectedDistanceProposal,
    /// Squared projected point-line distance polynomial proof row.
    SquaredProjectedPointLineDistancePolynomial,
    /// Squared projected line-length equality polynomial proof row.
    SquaredProjectedLineLengthEqualityPolynomial,
    /// Squared projected line-length ratio polynomial proof row.
    SquaredProjectedLineLengthRatioPolynomial,
    /// Squared projected line-length difference polynomial proof row.
    SquaredProjectedLineLengthDifferencePolynomial,
    /// Projected line-length difference branch predicate.
    ProjectedLineLengthDifferenceBranchPredicate,
    /// Squared projected line-length to point-line-distance proof row.
    SquaredProjectedLineLengthPointLineDistancePolynomial,
    /// Squared equality between two projected point-line distances.
    SquaredProjectedEqualPointLineDistancesPolynomial,
    /// Positive signed projected point-line distance proposal branch.
    ProjectedPointLineSignedDistancePositiveProposal,
    /// Negative signed projected point-line distance proposal branch.
    ProjectedPointLineSignedDistanceNegativeProposal,
    /// Squared point-line distance polynomial proof row.
    SquaredPointLineDistancePolynomial,
    /// Positive signed point-line distance proposal branch.
    PointLineSignedDistancePositiveProposal,
    /// Negative signed point-line distance proposal branch.
    PointLineSignedDistanceNegativeProposal,
    /// Squared circle-incidence polynomial proof row.
    SquaredCircleIncidencePolynomial,
    /// True radial circle-incidence proposal form.
    CircleRadialDistanceProposal,
    /// Squared-cosine equality for an unsigned angle proof row.
    SquaredCosineAnglePolynomial,
    /// `acos(cos(first)) - acos(cos(second))`, retained for proposal/UI parity.
    TrueAngleProposal,
    /// Exact angle-vector collinearity proof row for oriented angle equality.
    OrientedAngleVectorCollinearityPolynomial,
    /// Exact same-branch predicate for oriented angle equality.
    OrientedAngleSameBranchPredicate,
    /// Exact workplane-projected angle-vector collinearity proof row.
    ProjectedOrientedAngleVectorCollinearityPolynomial,
    /// Exact same-branch predicate for workplane-projected oriented angle equality.
    ProjectedOrientedAngleSameBranchPredicate,
    /// Exact cross-product equality for a G1 tangent support proof row.
    TangentCrossProductPredicate,
    /// Exact dot-product inequality for a same-direction tangent proof row.
    TangentDotProductPredicate,
    /// Exact selected arc endpoint-on-radius proof row for arc-line tangency.
    ArcLineTangentEndpointRadiusPolynomial,
    /// Exact selected endpoint coordinate-incidence proof row for arc-line tangency.
    ArcLineTangentEndpointIncidencePolynomial,
    /// Exact radius/tangent perpendicularity proof row for arc-line tangency.
    ArcLineTangentRadiusPerpendicularPolynomial,
    /// Exact signed orientation predicate for arc-line tangency.
    ArcLineTangentOrientationPredicate,
    /// Exact selected endpoint-on-radius proof row for arc-arc tangency.
    ArcArcTangentEndpointRadiusPolynomial,
    /// Exact selected endpoint coordinate-incidence proof row for arc-arc tangency.
    ArcArcTangentEndpointIncidencePolynomial,
    /// Exact radius-vector collinearity proof row for arc-arc tangency.
    ArcArcTangentRadiusCrossProductPredicate,
    /// Exact same/opposite radius branch predicate for arc-arc tangency.
    ArcArcTangentRadiusBranchPredicate,
    /// Exact selected arc endpoint-on-radius proof row for arc-cubic tangency.
    ArcCubicTangentEndpointRadiusPolynomial,
    /// Exact cubic point/arc endpoint coordinate-incidence proof row.
    ArcCubicTangentEndpointIncidencePolynomial,
    /// Exact radius/cubic-derivative perpendicularity proof row.
    ArcCubicTangentRadiusPerpendicularPolynomial,
    /// Exact signed orientation predicate for arc-cubic tangency.
    ArcCubicTangentOrientationPredicate,
    /// Exact selected arc endpoint-on-radius proof row for arc-cubic second-order contact.
    ArcCubicSecondOrderEndpointRadiusPolynomial,
    /// Exact cubic point/arc endpoint coordinate-incidence proof row for arc-cubic second-order contact.
    ArcCubicSecondOrderEndpointIncidencePolynomial,
    /// Exact radius/cubic-derivative perpendicularity proof row for arc-cubic second-order contact.
    ArcCubicSecondOrderRadiusPerpendicularPolynomial,
    /// Exact signed orientation predicate for arc-cubic second-order contact.
    ArcCubicSecondOrderOrientationPredicate,
    /// Exact differentiated circle-incidence proof row for arc-cubic second-order contact.
    ArcCubicSecondOrderContactPolynomial,
    /// Exact selected endpoint coordinate-incidence proof row for cubic-line tangency.
    CubicLineTangentEndpointIncidencePolynomial,
    /// Exact cubic derivative/line tangent cross-product proof row.
    CubicLineTangentCrossProductPredicate,
    /// Exact cubic derivative/line tangent dot-product branch predicate.
    CubicLineTangentDotProductPredicate,
    /// Exact cubic/cubic point coordinate-incidence proof row.
    CubicCubicTangentPointIncidencePolynomial,
    /// Exact cubic/cubic derivative cross-product proof row.
    CubicCubicTangentCrossProductPredicate,
    /// Exact cubic/cubic derivative dot-product branch predicate.
    CubicCubicTangentDotProductPredicate,
    /// Exact cubic/cubic G2 point coordinate-incidence proof row.
    CubicCubicG2PointIncidencePolynomial,
    /// Exact cubic/cubic G2 derivative cross-product proof row.
    CubicCubicG2TangentCrossProductPredicate,
    /// Exact cubic/cubic G2 derivative dot-product branch predicate.
    CubicCubicG2TangentDotProductPredicate,
    /// Exact cubic/cubic G2 signed-curvature magnitude proof row.
    CubicCubicG2CurvatureMagnitudePolynomial,
    /// Exact cubic/cubic G2 signed-curvature branch predicate.
    CubicCubicG2CurvatureSignPredicate,
    /// Exact cubic/cubic C2 point coordinate equality proof row.
    CubicCubicC2PointPolynomial,
    /// Exact cubic/cubic C2 first-derivative coordinate equality proof row.
    CubicCubicC2FirstDerivativePolynomial,
    /// Exact cubic/cubic C2 second-derivative coordinate equality proof row.
    CubicCubicC2SecondDerivativePolynomial,
    /// Exact midpoint-on-workplane proof row for 3D workplane symmetry.
    WorkplaneSymmetryMidpointPlanePolynomial,
    /// Exact normal-offset cross-product proof row for 3D workplane symmetry.
    WorkplaneSymmetryNormalOffsetPolynomial,
    /// Exact cubic-Bezier coordinate incidence proof row.
    CubicBezierIncidencePolynomial,
}

/// Proof role for a retained residual form.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SketchResidualFormRole {
    /// Exact replay may use this form as a proof obligation.
    ExactProof,
    /// The form is retained for proposal or UI compatibility only.
    ProposalOnly,
}

/// One retained residual form for a high-level sketch constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchResidualForm {
    /// Residual form kind.
    pub kind: SketchResidualFormKind,
    /// Trust role of this form.
    pub role: SketchResidualFormRole,
    /// Lowering strategy when this form is accepted as a proof row.
    pub strategy: Option<SketchResidualStrategy>,
    /// Symbolic residual expression.
    pub residual: Expr,
}

/// Status for retained residual-form extraction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SketchResidualFormsStatus {
    /// Forms were produced successfully.
    Generated,
    /// No constraint exists for the requested handle.
    MissingConstraint(SketchConstraintHandle),
    /// The constraint is reference-only and intentionally has no proof row.
    ReferenceOnly,
    /// This constraint family has no retained multi-form package yet.
    UnsupportedConstraint,
    /// Entity or parameter validation failed while building forms.
    InvalidInputs,
}

/// Report containing retained residual forms for one source constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchResidualFormsReport {
    /// Requested constraint handle.
    pub constraint: SketchConstraintHandle,
    /// Extraction status.
    pub status: SketchResidualFormsStatus,
    /// Retained residual forms.
    pub forms: Vec<SketchResidualForm>,
    /// Validation diagnostics, when form construction could not proceed.
    pub diagnostics: Vec<SketchGeneratedRow>,
}

/// Result of lowering a semantic sketch into generic residual rows.
#[derive(Clone, Debug)]
pub struct SketchLoweringReport {
    /// Lowered generic residual problem.
    pub problem: Problem,
    /// One status row per generated or attempted residual.
    pub rows: Vec<SketchGeneratedRow>,
}

impl SketchLoweringReport {
    /// Returns true when every non-reference source row was generated.
    pub fn all_generated(&self) -> bool {
        self.rows
            .iter()
            .all(|row| row.status == SketchGeneratedRowStatus::Generated)
    }
}

/// CAD/sketch-facing semantic layer above [`Problem`].
#[derive(Clone, Debug, Default)]
pub struct SketchSolveProblem {
    parameters: Vec<SketchParameter>,
    entities: Vec<SketchEntity>,
    constraints: Vec<SketchConstraint>,
}

impl SketchSolveProblem {
    /// Create an empty sketch problem.
    pub fn new() -> Self {
        Self::default()
    }

    /// Return retained parameters.
    pub fn parameters(&self) -> &[SketchParameter] {
        &self.parameters
    }

    /// Return retained entities.
    pub fn entities(&self) -> &[SketchEntity] {
        &self.entities
    }

    /// Return retained high-level constraints.
    pub fn constraints(&self) -> &[SketchConstraint] {
        &self.constraints
    }

    /// Add a scalar sketch parameter.
    pub fn add_parameter(&mut self, name: impl Into<String>, value: Real) -> SketchParameterHandle {
        self.add_parameter_in_group(name, value, None)
    }

    /// Add a scalar sketch parameter with optional group ownership.
    pub fn add_parameter_in_group(
        &mut self,
        name: impl Into<String>,
        value: Real,
        group: Option<SketchGroupHandle>,
    ) -> SketchParameterHandle {
        let handle = SketchParameterHandle(self.parameters.len() as u32);
        self.parameters.push(SketchParameter {
            handle,
            group,
            name: name.into(),
            value,
            fixed: false,
            domains: Vec::new(),
            metadata: SketchRoundTripMetadata::default(),
        });
        handle
    }

    /// Attach an exact preflight domain obligation to a retained parameter.
    ///
    /// Returns `false` for stale handles. The domain is stored at the sketch
    /// layer and does not generate a residual row by itself; callers can run
    /// [`crate::preflight_sketch_parameter_domains`] before lowering or
    /// numerical iteration.
    pub fn add_parameter_domain(
        &mut self,
        handle: SketchParameterHandle,
        domain: SketchParameterDomain,
    ) -> bool {
        let Some(parameter) = self.parameters.get_mut(handle.0 as usize) else {
            return false;
        };
        parameter.domains.push(domain);
        true
    }

    /// Lock a retained parameter to its current exact value.
    ///
    /// This marks the lowered solver variable as fixed and records a
    /// [`SketchParameterDomain::Locked`] preflight obligation using the current
    /// value.
    pub fn lock_parameter(&mut self, handle: SketchParameterHandle) -> bool {
        let Some(parameter) = self.parameters.get_mut(handle.0 as usize) else {
            return false;
        };
        parameter.fixed = true;
        parameter.domains.push(SketchParameterDomain::Locked {
            value: parameter.value.clone(),
        });
        true
    }

    /// Attach round-trip metadata to a retained parameter.
    ///
    /// Returns `false` when the handle is stale. Metadata is deliberately kept
    /// on the semantic sketch layer and is not interpreted as proof data during
    /// lowering.
    pub fn set_parameter_metadata(
        &mut self,
        handle: SketchParameterHandle,
        metadata: SketchRoundTripMetadata,
    ) -> bool {
        let Some(parameter) = self.parameters.get_mut(handle.0 as usize) else {
            return false;
        };
        parameter.metadata = metadata;
        true
    }

    /// Add a retained 2D point and its coordinate parameters.
    pub fn add_point2d(&mut self, name: impl Into<String>, x: Real, y: Real) -> SketchEntityHandle {
        let name = name.into();
        let x = self.add_parameter(format!("{name}.x"), x);
        let y = self.add_parameter(format!("{name}.y"), y);
        self.add_entity(name, SketchEntityKind::Point2D(SketchPoint2 { x, y }))
    }

    /// Add a retained 3D point and its coordinate parameters.
    pub fn add_point3d(
        &mut self,
        name: impl Into<String>,
        x: Real,
        y: Real,
        z: Real,
    ) -> SketchEntityHandle {
        let name = name.into();
        let x = self.add_parameter(format!("{name}.x"), x);
        let y = self.add_parameter(format!("{name}.y"), y);
        let z = self.add_parameter(format!("{name}.z"), z);
        self.add_entity(name, SketchEntityKind::Point3D(SketchPoint3 { x, y, z }))
    }

    /// Add a retained scalar distance entity.
    pub fn add_distance(&mut self, name: impl Into<String>, value: Real) -> SketchEntityHandle {
        let name = name.into();
        let value = self.add_parameter(format!("{name}.value"), value);
        self.add_entity(name, SketchEntityKind::Distance(SketchDistance { value }))
    }

    /// Add a retained 2D normal/direction and its component parameters.
    pub fn add_normal2d(
        &mut self,
        name: impl Into<String>,
        x: Real,
        y: Real,
    ) -> SketchEntityHandle {
        let name = name.into();
        let x = self.add_parameter(format!("{name}.x"), x);
        let y = self.add_parameter(format!("{name}.y"), y);
        self.add_entity(name, SketchEntityKind::Normal2D(SketchNormal2 { x, y }))
    }

    /// Add a retained 3D normal/quaternion and its component parameters.
    ///
    /// The values are retained exactly as caller data. Unit-normal,
    /// right-handed-frame, or quaternion normalization checks should be added
    /// as explicit residuals/certificates by callers instead of hidden epsilon
    /// tests.
    pub fn add_normal3d(
        &mut self,
        name: impl Into<String>,
        w: Real,
        x: Real,
        y: Real,
        z: Real,
    ) -> SketchEntityHandle {
        let name = name.into();
        let w = self.add_parameter(format!("{name}.w"), w);
        let x = self.add_parameter(format!("{name}.x"), x);
        let y = self.add_parameter(format!("{name}.y"), y);
        let z = self.add_parameter(format!("{name}.z"), z);
        self.add_entity(
            name,
            SketchEntityKind::Normal3D(SketchNormal3 { w, x, y, z }),
        )
    }

    /// Add a retained 2D line segment between two point entities.
    pub fn add_line_segment2(
        &mut self,
        name: impl Into<String>,
        start: SketchEntityHandle,
        end: SketchEntityHandle,
    ) -> SketchEntityHandle {
        self.add_entity(
            name,
            SketchEntityKind::LineSegment2(SketchLineSegment2 { start, end }),
        )
    }

    /// Add a retained 3D line segment between two 3D point entities.
    pub fn add_line_segment3(
        &mut self,
        name: impl Into<String>,
        start: SketchEntityHandle,
        end: SketchEntityHandle,
    ) -> SketchEntityHandle {
        self.add_entity(
            name,
            SketchEntityKind::LineSegment3(SketchLineSegment3 { start, end }),
        )
    }

    /// Add a retained 2D cubic Bezier carrier.
    pub fn add_cubic2(
        &mut self,
        name: impl Into<String>,
        p0: SketchEntityHandle,
        p1: SketchEntityHandle,
        p2: SketchEntityHandle,
        p3: SketchEntityHandle,
    ) -> SketchEntityHandle {
        self.add_entity(
            name,
            SketchEntityKind::Cubic2(SketchCubic2 { p0, p1, p2, p3 }),
        )
    }

    /// Add a retained 2D circle with a point center and distance radius.
    pub fn add_circle2(
        &mut self,
        name: impl Into<String>,
        center: SketchEntityHandle,
        radius: SketchEntityHandle,
    ) -> SketchEntityHandle {
        self.add_entity(
            name,
            SketchEntityKind::Circle2(SketchCircle2 { center, radius }),
        )
    }

    /// Add a retained 2D circular arc.
    pub fn add_arc_of_circle2(
        &mut self,
        name: impl Into<String>,
        center: SketchEntityHandle,
        start: SketchEntityHandle,
        end: SketchEntityHandle,
        radius: SketchEntityHandle,
    ) -> SketchEntityHandle {
        self.add_entity(
            name,
            SketchEntityKind::ArcOfCircle2(SketchArcOfCircle2 {
                center,
                start,
                end,
                radius,
            }),
        )
    }

    /// Add a retained workplane from an origin point and normal entity.
    pub fn add_workplane(
        &mut self,
        name: impl Into<String>,
        origin: SketchEntityHandle,
        normal: SketchEntityHandle,
    ) -> SketchEntityHandle {
        self.add_entity(
            name,
            SketchEntityKind::Workplane(SketchWorkplane { origin, normal }),
        )
    }

    /// Add a raw retained entity payload.
    pub fn add_entity(
        &mut self,
        name: impl Into<String>,
        kind: SketchEntityKind,
    ) -> SketchEntityHandle {
        let handle = SketchEntityHandle(self.entities.len() as u32);
        self.entities.push(SketchEntity {
            handle,
            group: None,
            name: name.into(),
            kind,
            domains: Vec::new(),
            metadata: SketchRoundTripMetadata::default(),
        });
        handle
    }

    /// Attach an exact preflight domain obligation to a retained entity.
    ///
    /// Returns `false` for stale handles. The obligation is stored at the
    /// semantic sketch layer and does not generate residual rows by itself;
    /// callers can run [`crate::preflight_sketch_entity_domains`] before
    /// lowering or numerical iteration.
    pub fn add_entity_domain(
        &mut self,
        handle: SketchEntityHandle,
        domain: SketchEntityDomain,
    ) -> bool {
        let Some(entity) = self.entities.get_mut(handle.0 as usize) else {
            return false;
        };
        entity.domains.push(domain);
        true
    }

    /// Attach round-trip metadata to a retained entity.
    ///
    /// Returns `false` when the handle is stale. Construction/reference roles
    /// remain descriptive until a typed constraint chooses to generate proof
    /// rows.
    pub fn set_entity_metadata(
        &mut self,
        handle: SketchEntityHandle,
        metadata: SketchRoundTripMetadata,
    ) -> bool {
        let Some(entity) = self.entities.get_mut(handle.0 as usize) else {
            return false;
        };
        entity.metadata = metadata;
        true
    }

    /// Add a point coincidence constraint.
    pub fn add_points_coincident(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        incidence::points_coincident(self, name, a, b).handle
    }

    /// Add a point-to-point distance constraint.
    ///
    /// The lowered residual uses `|a-b|^2 - d^2`, following the common CAD
    /// solver practice of keeping distance constraints polynomial when the
    /// sign of the distance parameter is separately modeled. Exact replay then
    /// certifies the polynomial residual instead of trusting a square-root
    /// approximation; see Yap (1997) for the construction/proof split.
    pub fn add_point_point_distance(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::point_point_distance(self, name, a, b, distance).handle
    }

    /// Add a bounded point-to-point distance constraint.
    ///
    /// Lowering emits `|a-b|^2 - lower^2 >= 0` and/or
    /// `|a-b|^2 - upper^2 <= 0` after exact bound validation. The validation is
    /// report-bearing because squaring an invalid negative bound would hide a
    /// modeling error.
    pub fn add_point_point_distance_range(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintHandle {
        distance::point_point_distance_range(self, name, a, b, lower, upper).handle
    }

    /// Add a bounded workplane-projected distance relation between two 3D points.
    ///
    /// Lowering validates exact nonnegative/ordered bounds before emitting the
    /// workplane unit guard and projected squared-distance inequalities.
    pub fn add_projected_point_point_distance_range(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintHandle {
        distance::projected_point_point_distance_range(self, name, workplane, a, b, lower, upper)
            .handle
    }

    /// Add a workplane-projected point-to-point distance constraint.
    ///
    /// The retained relation lowers to an exact unit-workplane guard and a
    /// squared projected-distance row. Candidate replay therefore proves the
    /// projection algebra instead of relying on lossy workplane coordinates.
    pub fn add_projected_point_point_distance(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::projected_point_point_distance(self, name, workplane, a, b, distance).handle
    }

    /// Add a workplane-projected point-to-line distance constraint.
    ///
    /// The retained relation lowers to a unit-workplane guard and the exact
    /// denominator-cleared projected point-line distance polynomial.
    pub fn add_projected_point_line_distance(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::projected_point_line_distance(self, name, workplane, point, line, distance).handle
    }

    /// Add a workplane-projected equal-length constraint for two 3D lines.
    ///
    /// The retained relation lowers to a unit-workplane guard and the exact
    /// squared projected line-length equality polynomial.
    pub fn add_projected_equal_length_lines3(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::projected_equal_length_lines3(self, name, workplane, a, b).handle
    }

    /// Add a workplane-projected length-ratio constraint for two 3D lines.
    ///
    /// The retained relation lowers to a unit-workplane guard and the exact
    /// squared projected line-length ratio polynomial after ratio-domain
    /// validation.
    pub fn add_projected_length_ratio_lines3(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        numerator: Real,
        denominator: Real,
    ) -> SketchConstraintHandle {
        distance::projected_length_ratio_lines3(self, name, workplane, a, b, numerator, denominator)
            .handle
    }

    /// Add a retained 2D line equal-length relation.
    ///
    /// The proof row compares squared lengths exactly, so candidate replay does
    /// not depend on a lossy square-root length computation.
    pub fn add_equal_length_lines2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::equal_length_lines2(self, name, a, b).handle
    }

    /// Add a retained 2D line length-ratio relation.
    ///
    /// The proof row compares squared lengths scaled by exact ratio terms.
    /// Invalid negative numerator or nonpositive denominator inputs are
    /// reported during lowering rather than hidden by squaring.
    pub fn add_length_ratio_lines2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        numerator: Real,
        denominator: Real,
    ) -> SketchConstraintHandle {
        distance::length_ratio_lines2(self, name, a, b, numerator, denominator).handle
    }

    /// Add a retained 2D line length-difference relation.
    ///
    /// Lowering emits an exact squared polynomial row and an exact ordering
    /// inequality that selects the `longer >= shorter` branch.
    pub fn add_length_difference_lines2(
        &mut self,
        name: impl Into<String>,
        longer: SketchEntityHandle,
        shorter: SketchEntityHandle,
        difference: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::length_difference_lines2(self, name, longer, shorter, difference).handle
    }

    /// Add a retained 2D point-to-line distance relation.
    pub fn add_point_line_distance2(
        &mut self,
        name: impl Into<String>,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        distance: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::point_line_distance2(self, name, point, line, distance).handle
    }

    /// Add a retained relation equating a 2D line length to a point-line
    /// distance.
    pub fn add_equal_length_point_line_distance2(
        &mut self,
        name: impl Into<String>,
        length_line: SketchEntityHandle,
        point: SketchEntityHandle,
        distance_line: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::equal_length_point_line_distance2(self, name, length_line, point, distance_line)
            .handle
    }

    /// Add a retained equality between two 2D point-line distances.
    pub fn add_equal_point_line_distances2(
        &mut self,
        name: impl Into<String>,
        a_point: SketchEntityHandle,
        a_line: SketchEntityHandle,
        b_point: SketchEntityHandle,
        b_line: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::equal_point_line_distances2(self, name, a_point, a_line, b_point, b_line).handle
    }

    /// Add a retained 2D equal-radius relation for circles or circular arcs.
    pub fn add_equal_radius2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::equal_radius2(self, name, a, b).handle
    }

    /// Add a horizontal 2D line constraint.
    pub fn add_horizontal(
        &mut self,
        name: impl Into<String>,
        line: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        orientation::horizontal(self, name, line).handle
    }

    /// Add a vertical 2D line constraint.
    pub fn add_vertical(
        &mut self,
        name: impl Into<String>,
        line: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        orientation::vertical(self, name, line).handle
    }

    /// Add a retained 2D line parallelism constraint.
    ///
    /// Lowering emits `dir(a) x dir(b) == 0` as an exact polynomial row.
    pub fn add_parallel_lines2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        orientation::parallel_lines2(self, name, a, b).handle
    }

    /// Add a retained 2D line perpendicularity constraint.
    ///
    /// Lowering emits `dir(a) . dir(b) == 0` without normalizing either vector.
    pub fn add_perpendicular_lines2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        orientation::perpendicular_lines2(self, name, a, b).handle
    }

    /// Add a retained 2D line same-orientation constraint.
    ///
    /// Lowering emits exact cross-product and dot-product rows without
    /// direction normalization. Degenerate line detection stays in explicit
    /// sketch-domain preflight rather than a hidden epsilon.
    pub fn add_same_direction_lines2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        orientation::same_direction_lines2(self, name, a, b).handle
    }

    /// Add a retained 2D same-direction tangent-carrier relation.
    ///
    /// Lowering emits a G1 tangent package: exact cross-product equality and
    /// exact dot-product nonnegativity, with degenerate tangent carriers left
    /// to explicit entity-domain preflight.
    pub fn add_tangent_same_direction_lines2(
        &mut self,
        name: impl Into<String>,
        candidate: SketchEntityHandle,
        target: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        tangency::tangent_same_direction_lines2(self, name, candidate, target).handle
    }

    /// Add a retained 2D arc-line tangent relation with explicit endpoint and
    /// orientation flags.
    ///
    /// The selected line endpoint is constrained to the selected arc endpoint,
    /// the arc endpoint is checked against the retained radius, and the
    /// outgoing line tangent is certified perpendicular to the radius with the
    /// requested signed branch.
    pub fn add_arc_line_tangent2(
        &mut self,
        name: impl Into<String>,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintHandle {
        tangency::arc_line_tangent2(
            self,
            name,
            arc,
            arc_endpoint,
            line,
            line_endpoint,
            orientation,
        )
        .handle
    }

    /// Add a workplane-projected length-difference constraint for two 3D lines.
    ///
    /// Lowering emits the unit-workplane guard, a square-root-free polynomial
    /// equality, and the exact branch inequality selecting the longer line.
    pub fn add_projected_length_difference_lines3(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        longer: SketchEntityHandle,
        shorter: SketchEntityHandle,
        difference: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::projected_length_difference_lines3(
            self, name, workplane, longer, shorter, difference,
        )
        .handle
    }

    /// Add a workplane-projected equality between line length and point-line distance.
    ///
    /// Lowering emits the unit-workplane guard and the exact denominator-cleared
    /// polynomial proof row over projected `U/V` coordinates.
    pub fn add_projected_equal_length_point_line_distance3(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        length_line: SketchEntityHandle,
        point: SketchEntityHandle,
        distance_line: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::projected_equal_length_point_line_distance3(
            self,
            name,
            workplane,
            length_line,
            point,
            distance_line,
        )
        .handle
    }

    /// Add a workplane-projected equality between two point-line distances.
    ///
    /// Lowering emits the unit-workplane guard plus a denominator-cleared
    /// polynomial equality between the two projected point-line distances.
    pub fn add_projected_equal_point_line_distances3(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a_point: SketchEntityHandle,
        a_line: SketchEntityHandle,
        b_point: SketchEntityHandle,
        b_line: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        distance::projected_equal_point_line_distances3(
            self, name, workplane, a_point, a_line, b_point, b_line,
        )
        .handle
    }

    /// Add a retained 2D arc/arc tangent relation with explicit endpoint and
    /// same/opposite radius-vector branch flags.
    ///
    /// The selected endpoints are constrained to coincide, both endpoints are
    /// checked against retained radii, and exact radius-vector predicates
    /// certify the chosen tangency branch.
    pub fn add_arc_arc_tangent2(
        &mut self,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_endpoint: SketchArcEndpoint,
        second: SketchEntityHandle,
        second_endpoint: SketchArcEndpoint,
        branch: SketchArcTangencyBranch,
    ) -> SketchConstraintHandle {
        tangency::arc_arc_tangent2(
            self,
            name,
            first,
            first_endpoint,
            second,
            second_endpoint,
            branch,
        )
        .handle
    }

    /// Add a retained 2D arc/cubic tangent relation with explicit endpoint,
    /// parameter, and orientation flags.
    ///
    /// The selected arc endpoint is constrained to the exact cubic point at
    /// `parameter`, while the retained radius vector is constrained
    /// perpendicular to the exact cubic derivative with the requested signed
    /// branch.
    pub fn add_arc_cubic_tangent2(
        &mut self,
        name: impl Into<String>,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintHandle {
        tangency::arc_cubic_tangent2(self, name, arc, arc_endpoint, cubic, parameter, orientation)
            .handle
    }

    /// Add retained second-order contact between a 2D arc endpoint and cubic.
    ///
    /// The proof package includes arc/cubic tangency plus the exact
    /// differentiated circle-incidence row for `B''(t)`.
    pub fn add_arc_cubic_second_order_contact2(
        &mut self,
        name: impl Into<String>,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
    ) -> SketchConstraintHandle {
        tangency::arc_cubic_second_order_contact2(
            self,
            name,
            arc,
            arc_endpoint,
            cubic,
            parameter,
            orientation,
        )
        .handle
    }

    /// Add a retained 2D cubic-Bezier/line tangent relation.
    ///
    /// The selected line endpoint is constrained to the exact cubic point at
    /// `parameter`, and the outgoing line direction is constrained to the same
    /// direction as the exact cubic derivative. Segment-domain and
    /// nondegenerate-tangent assumptions are explicit sketch obligations, not
    /// hidden normalizations.
    pub fn add_cubic_line_tangent2(
        &mut self,
        name: impl Into<String>,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
    ) -> SketchConstraintHandle {
        tangency::cubic_line_tangent2(self, name, cubic, parameter, line, line_endpoint).handle
    }

    /// Add a retained 2D cubic-Bezier/cubic-Bezier tangent relation.
    ///
    /// The two exact cubic points are constrained to coincide and their exact
    /// derivatives are constrained to the same tangent branch. Segment-domain
    /// and nonzero-derivative assumptions remain explicit obligations.
    pub fn add_cubic_cubic_tangent2(
        &mut self,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintHandle {
        tangency::cubic_cubic_tangent2(self, name, first, first_parameter, second, second_parameter)
            .handle
    }

    /// Add retained geometric G2 curvature continuity between two cubics.
    ///
    /// The proof package keeps point/tangent replay and adds exact
    /// signed-curvature equality after clearing speed denominators.
    pub fn add_cubic_cubic_g2_continuity2(
        &mut self,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintHandle {
        tangency::cubic_cubic_g2_continuity2(
            self,
            name,
            first,
            first_parameter,
            second,
            second_parameter,
        )
        .handle
    }

    /// Add a retained parametric C2 continuity relation between two cubics.
    ///
    /// The proof package equates exact point, first derivative, and second
    /// derivative coordinates. This keeps parameterization explicit instead of
    /// smuggling geometric curvature normalization into lowering.
    pub fn add_cubic_cubic_c2_continuity2(
        &mut self,
        name: impl Into<String>,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
    ) -> SketchConstraintHandle {
        tangency::cubic_cubic_c2_continuity2(
            self,
            name,
            first,
            first_parameter,
            second,
            second_parameter,
        )
        .handle
    }

    /// Add a retained unsigned 2D equal-angle relation between two line pairs.
    ///
    /// The lowered proof row compares squared cosines exactly, preserving the
    /// source angle relation for diagnostics while avoiding trig in replay.
    pub fn add_equal_angle_lines2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        angle::equal_angle_lines2(self, name, a, b, c, d).handle
    }

    /// Add a retained oriented 2D equal-angle relation between two line pairs.
    ///
    /// The lowered proof package compares the exact `(dot, cross)` angle
    /// vectors and adds a same-branch inequality. It therefore distinguishes
    /// equal unsigned angles from reversed oriented angles without using
    /// `atan2` during proof replay.
    pub fn add_equal_oriented_angle_lines2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        angle::equal_oriented_angle_lines2(self, name, a, b, c, d).handle
    }

    /// Add a workplane-projected retained oriented angle equality for 3D lines.
    pub fn add_projected_equal_oriented_angle_lines3(
        &mut self,
        name: impl Into<String>,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        angle::projected_equal_oriented_angle_lines3(self, name, workplane, a, b, c, d).handle
    }

    /// Add a retained 2D point-at-midpoint relation.
    ///
    /// Lowering emits one exact linear equality per coordinate. This mirrors
    /// SolveSpace's midpoint relation at the semantic layer while keeping
    /// candidate acceptance in Yap-style exact residual replay.
    pub fn add_at_midpoint2(
        &mut self,
        name: impl Into<String>,
        point: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        symmetry::at_midpoint2(self, name, point, a, b).handle
    }

    /// Add a retained 2D horizontal-axis point symmetry relation.
    pub fn add_symmetric_horizontal2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        axis_y: Real,
    ) -> SketchConstraintHandle {
        symmetry::symmetric_horizontal2(self, name, a, b, axis_y).handle
    }

    /// Add a retained 2D vertical-axis point symmetry relation.
    pub fn add_symmetric_vertical2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        axis_x: Real,
    ) -> SketchConstraintHandle {
        symmetry::symmetric_vertical2(self, name, a, b, axis_x).handle
    }

    /// Add a retained 2D line-axis point symmetry relation.
    ///
    /// Lowering emits exact rows for midpoint-on-axis and perpendicularity to
    /// the mirror-axis direction, avoiding normalized vectors or rounded
    /// reflected coordinates.
    pub fn add_symmetric_line2(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        axis: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        symmetry::symmetric_line2(self, name, a, b, axis).handle
    }

    /// Add a retained 3D workplane point-symmetry relation.
    ///
    /// Lowering emits exact rows for a certified unit workplane frame, the
    /// mirrored midpoint lying on the workplane, and the point offset being
    /// parallel to the workplane normal. This covers the SolveSpace-style
    /// plane-symmetry relation without constructing a reflected point through
    /// lossy coordinates.
    pub fn add_symmetric_workplane3(
        &mut self,
        name: impl Into<String>,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        workplane: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        symmetry::symmetric_workplane3(self, name, a, b, workplane).handle
    }

    /// Add a point-on-circle incidence constraint.
    pub fn add_point_on_circle(
        &mut self,
        name: impl Into<String>,
        point: SketchEntityHandle,
        circle: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        incidence::point_on_circle(self, name, point, circle).handle
    }

    /// Add a point-on-cubic-Bezier incidence constraint at a retained parameter.
    ///
    /// This emits exact Bernstein-coordinate proof rows. The curve parameter
    /// is not implicitly clamped; use [`Self::add_parameter_domain`] or
    /// [`Self::add_parameter_range`] to make the segment interval explicit.
    pub fn add_point_on_cubic2(
        &mut self,
        name: impl Into<String>,
        point: SketchEntityHandle,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
    ) -> SketchConstraintHandle {
        incidence::point_on_cubic2(self, name, point, cubic, parameter).handle
    }

    /// Add a scalar parameter range constraint.
    pub fn add_parameter_range(
        &mut self,
        name: impl Into<String>,
        parameter: SketchParameterHandle,
        lower: Option<Real>,
        upper: Option<Real>,
    ) -> SketchConstraintHandle {
        ranges::parameter_range(self, name, parameter, lower, upper).handle
    }

    /// Add a nondecreasing scalar parameter relation.
    ///
    /// Lowering emits the exact inequality `upper - lower >= 0`, matching Yap's
    /// construction/proof split: the retained relation records intent, while
    /// ordinary exact candidate certification decides whether a candidate
    /// satisfies it.
    pub fn add_parameter_ordering(
        &mut self,
        name: impl Into<String>,
        lower: SketchParameterHandle,
        upper: SketchParameterHandle,
    ) -> SketchConstraintHandle {
        ranges::parameter_ordering(self, name, lower, upper).handle
    }

    /// Add a minimum exact margin between two ordered scalar parameters.
    ///
    /// Lowering emits `upper - lower - margin >= 0` after certifying that the
    /// retained margin is nonnegative. Invalid or unresolved margins are
    /// reported as lowering rows instead of being folded into a misleading
    /// inequality.
    pub fn add_parameter_margin(
        &mut self,
        name: impl Into<String>,
        lower: SketchParameterHandle,
        upper: SketchParameterHandle,
        margin: Real,
    ) -> SketchConstraintHandle {
        ranges::parameter_margin(self, name, lower, upper, margin).handle
    }

    /// Add a soft stay-near objective for one parameter.
    pub fn add_stay_near_parameter(
        &mut self,
        name: impl Into<String>,
        parameter: SketchParameterHandle,
        target: Real,
        weight: Real,
    ) -> SketchConstraintHandle {
        objective::stay_near_parameter(self, name, parameter, target, weight).handle
    }

    /// Add a retained high-level sketch constraint.
    pub fn add_constraint(
        &mut self,
        name: impl Into<String>,
        kind: SketchConstraintKind,
        reference: bool,
        active: bool,
    ) -> SketchConstraintHandle {
        let handle = SketchConstraintHandle(self.constraints.len() as u32);
        self.constraints.push(SketchConstraint {
            handle,
            group: None,
            name: name.into(),
            reference,
            active,
            kind,
            metadata: SketchRoundTripMetadata::default(),
        });
        handle
    }

    /// Attach round-trip metadata to a retained high-level constraint.
    ///
    /// A [`SketchRoundTripRole::ReferenceDimension`] role also marks the
    /// constraint as reference-only so lowering records a diagnostic row rather
    /// than an equation. This keeps displayed dimensions round-trippable
    /// without smuggling UI annotations into Yap-style proof obligations.
    pub fn set_constraint_metadata(
        &mut self,
        handle: SketchConstraintHandle,
        metadata: SketchRoundTripMetadata,
    ) -> bool {
        let Some(constraint) = self.constraints.get_mut(handle.0 as usize) else {
            return false;
        };
        if metadata.role == SketchRoundTripRole::ReferenceDimension {
            constraint.reference = true;
        }
        constraint.metadata = metadata;
        true
    }

    /// Lower the semantic sketch layer into a generic residual [`Problem`].
    ///
    /// The returned [`SketchLoweringReport`] is deliberately report-bearing:
    /// failed handles and wrong entity families become explicit rows instead of
    /// panics or silently missing equations. This mirrors the exact-report
    /// discipline in Yap (1997) and leaves all candidate acceptance to ordinary
    /// `hypersolve` certification.
    pub fn lower_to_problem(&self) -> SketchLoweringReport {
        let mut problem = Problem::default();
        for parameter in &self.parameters {
            let id = problem.add_variable(parameter.name.clone(), parameter.value.clone());
            debug_assert_eq!(id, VariableId(parameter.handle.0));
            if let Some(variable) = problem.variables.last_mut() {
                variable.fixed = parameter.fixed;
            }
        }

        let mut rows = Vec::new();
        for constraint in &self.constraints {
            self.lower_constraint(constraint, &mut problem, &mut rows);
        }
        SketchLoweringReport { problem, rows }
    }

    /// Return retained residual forms for a high-level sketch constraint.
    ///
    /// This is the first multiple-form package requested by the SolveSpace
    /// coverage plan. For point-to-point distance it keeps both
    /// `|a-b|^2 - d^2`, which is the polynomial exact-replay form, and
    /// `sqrt(|a-b|^2) - d`, which is useful for proposal engines and UI
    /// parity. For workplane-projected point distance it keeps the unit-frame
    /// guard and squared projected-distance rows as exact proof forms, while a
    /// square-root projected-distance residual remains proposal-only. For
    /// point-line distance it keeps the exact squared polynomial
    /// proof form plus both oriented signed-distance proposal branches; the
    /// branch is data until an orientation-aware constraint or predicate
    /// package certifies it. For point-on-circle incidence it keeps squared
    /// radius equality as proof and the true radial residual only as a
    /// proposal/UI form. For equal unsigned line angles it similarly keeps
    /// a squared-cosine polynomial proof form and an `acos` proposal form.
    /// Tangency keeps exact cross/dot predicate forms because the orientation
    /// branch is itself proof data, not a proposal residual. Yap (1997) is the
    /// controlling rule here: proposal-compatible forms are retained as data,
    /// but only exact replay/certification turns a residual into evidence.
    pub fn residual_forms_for_constraint(
        &self,
        handle: SketchConstraintHandle,
    ) -> SketchResidualFormsReport {
        let Some(constraint) = self.constraints.get(handle.0 as usize) else {
            return SketchResidualFormsReport {
                constraint: handle,
                status: SketchResidualFormsStatus::MissingConstraint(handle),
                forms: Vec::new(),
                diagnostics: Vec::new(),
            };
        };
        if constraint.reference {
            return SketchResidualFormsReport {
                constraint: handle,
                status: SketchResidualFormsStatus::ReferenceOnly,
                forms: Vec::new(),
                diagnostics: Vec::new(),
            };
        }

        match constraint.kind {
            SketchConstraintKind::PointPointDistance { a, b, distance } => {
                let mut diagnostics = Vec::new();
                let (Some(a), Some(b), Some(distance)) = (
                    self.point_coordinates(a, constraint, &mut diagnostics),
                    self.point_coordinates(b, constraint, &mut diagnostics),
                    self.distance_expr(distance, constraint, &mut diagnostics),
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                if a.len() != b.len() {
                    diagnostics.push(wrong_entity_row(
                        constraint,
                        b.handle,
                        "matching point dimension",
                    ));
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                }
                let distance_squared = squared_distance(&a.exprs, &b.exprs);
                let squared = distance_squared.clone() - distance.clone() * distance.clone();
                let true_distance = distance_squared.sqrt() - distance;
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredDistancePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::SquaredDistance),
                            residual: squared,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::TrueDistanceProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: true_distance,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedPointPointDistance {
                workplane,
                a,
                b,
                distance,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.projected_point_distance_exprs(
                    workplane,
                    a,
                    b,
                    distance,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let unit_guard = unit_quaternion_residual(&parts.quaternion);
                let squared = parts.projected_squared.clone()
                    - parts.distance.clone() * parts.distance.clone();
                let true_distance = parts.projected_squared.sqrt() - parts.distance;
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_guard,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredProjectedDistancePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::SquaredProjectedDistance),
                            residual: squared,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::TrueProjectedDistanceProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: true_distance,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedPointPointDistanceRange {
                workplane,
                a,
                b,
                ref lower,
                ref upper,
            } => {
                let mut diagnostics = Vec::new();
                if !validate_distance_bounds_with_strategy(
                    constraint,
                    lower.as_ref(),
                    upper.as_ref(),
                    SketchResidualStrategy::BoundedSquaredProjectedDistance,
                    &mut diagnostics,
                ) {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                }
                let Some(parts) =
                    self.projected_point_range_exprs(workplane, a, b, constraint, &mut diagnostics)
                else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = Vec::new();
                if lower.is_some() || upper.is_some() {
                    forms.push(SketchResidualForm {
                        kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                        residual: unit_quaternion_residual(&parts.quaternion),
                    });
                }
                if let Some(lower) = lower {
                    forms.push(SketchResidualForm {
                        kind: SketchResidualFormKind::BoundedSquaredProjectedDistancePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::BoundedSquaredProjectedDistance),
                        residual: parts.projected_squared.clone()
                            - Expr::real(lower.clone() * lower.clone()),
                    });
                }
                if let Some(upper) = upper {
                    forms.push(SketchResidualForm {
                        kind: SketchResidualFormKind::BoundedSquaredProjectedDistancePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::BoundedSquaredProjectedDistance),
                        residual: parts.projected_squared
                            - Expr::real(upper.clone() * upper.clone()),
                    });
                }
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedPointLineDistance {
                workplane,
                point,
                line,
                distance,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.projected_point_line_distance_exprs(
                    workplane,
                    point,
                    line,
                    distance,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let unit_guard = unit_quaternion_residual(&parts.quaternion);
                let squared = parts.distance_numerator.clone()
                    - parts.distance.clone()
                        * parts.distance.clone()
                        * parts.distance_denominator.clone();
                let scaled_distance =
                    parts.distance.clone() * parts.distance_denominator.clone().sqrt();
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_guard,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredProjectedPointLineDistancePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(
                                SketchResidualStrategy::SquaredProjectedPointLineDistance,
                            ),
                            residual: squared,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::ProjectedPointLineSignedDistancePositiveProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: parts.signed_cross.clone() - scaled_distance.clone(),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::ProjectedPointLineSignedDistanceNegativeProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: parts.signed_cross + scaled_distance,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedEqualLengthLines3 { workplane, a, b } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.projected_line_length_equality_exprs(
                    workplane,
                    a,
                    b,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_quaternion_residual(&parts.quaternion),
                        },
                        SketchResidualForm {
                            kind:
                                SketchResidualFormKind::SquaredProjectedLineLengthEqualityPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(
                                SketchResidualStrategy::SquaredProjectedLineLengthEquality,
                            ),
                            residual: parts.first_squared - parts.second_squared,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedLengthRatioLines3 {
                workplane,
                a,
                b,
                ref numerator,
                ref denominator,
            } => {
                let mut diagnostics = Vec::new();
                if !validate_length_ratio(
                    constraint,
                    numerator,
                    denominator,
                    SketchResidualStrategy::SquaredProjectedLineLengthRatio,
                    &mut diagnostics,
                ) {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                }
                let Some(parts) = self.projected_line_length_equality_exprs(
                    workplane,
                    a,
                    b,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let numerator_squared = Expr::real(numerator.clone() * numerator.clone());
                let denominator_squared = Expr::real(denominator.clone() * denominator.clone());
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_quaternion_residual(&parts.quaternion),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredProjectedLineLengthRatioPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::SquaredProjectedLineLengthRatio),
                            residual: parts.first_squared * denominator_squared
                                - parts.second_squared * numerator_squared,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedLengthDifferenceLines3 {
                workplane,
                longer,
                shorter,
                difference,
            } => {
                let mut diagnostics = Vec::new();
                let Some((parts, difference)) = self.projected_line_length_difference_exprs(
                    workplane,
                    longer,
                    shorter,
                    difference,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let difference_squared = difference.clone() * difference;
                let sum_minus_difference = parts.first_squared.clone()
                    + parts.second_squared.clone()
                    - difference_squared.clone();
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_quaternion_residual(&parts.quaternion),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredProjectedLineLengthDifferencePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(
                                SketchResidualStrategy::SquaredProjectedLineLengthDifference,
                            ),
                            residual: sum_minus_difference.clone() * sum_minus_difference
                                - Expr::int(4)
                                    * parts.first_squared.clone()
                                    * parts.second_squared.clone(),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::ProjectedLineLengthDifferenceBranchPredicate,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(
                                SketchResidualStrategy::SquaredProjectedLineLengthDifference,
                            ),
                            residual: parts.first_squared - parts.second_squared - difference_squared,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedEqualLengthPointLineDistance3 {
                workplane,
                length_line,
                point,
                distance_line,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.projected_equal_length_point_line_distance_exprs(
                    workplane,
                    length_line,
                    point,
                    distance_line,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_quaternion_residual(&parts.quaternion),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredProjectedLineLengthPointLineDistancePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(
                                SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance,
                            ),
                            residual: parts.length_squared * parts.distance_denominator
                                - parts.distance_numerator,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedEqualPointLineDistances3 {
                workplane,
                a_point,
                a_line,
                b_point,
                b_line,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.projected_equal_point_line_distances_exprs(
                    workplane,
                    [(a_point, a_line), (b_point, b_line)],
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_quaternion_residual(&parts.quaternion),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredProjectedEqualPointLineDistancesPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(
                                SketchResidualStrategy::SquaredProjectedEqualPointLineDistances,
                            ),
                            residual: parts.a_numerator * parts.b_denominator
                                - parts.b_numerator * parts.a_denominator,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::SymmetricWorkplane3 { a, b, workplane } => {
                let mut diagnostics = Vec::new();
                let Some(parts) =
                    self.workplane_symmetry_exprs(workplane, a, b, constraint, &mut diagnostics)
                else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = vec![
                    SketchResidualForm {
                        kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                        residual: parts.unit_guard,
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::WorkplaneSymmetryMidpointPlanePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::WorkplaneSymmetryPolynomial),
                        residual: parts.midpoint_plane,
                    },
                ];
                forms.extend(parts.normal_offset_cross.into_iter().map(|residual| {
                    SketchResidualForm {
                        kind: SketchResidualFormKind::WorkplaneSymmetryNormalOffsetPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::WorkplaneSymmetryPolynomial),
                        residual,
                    }
                }));
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::PointLineDistance2 {
                point,
                line,
                distance,
            } => {
                let mut diagnostics = Vec::new();
                let (Some(point), Some((start, end)), Some(distance)) = (
                    self.point2_coordinates(point, constraint, &mut diagnostics),
                    self.line2_points(line, constraint, &mut diagnostics),
                    self.distance_expr(distance, constraint, &mut diagnostics),
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let direction = [
                    end.exprs[0].clone() - start.exprs[0].clone(),
                    end.exprs[1].clone() - start.exprs[1].clone(),
                ];
                let point_delta = [
                    point.exprs[0].clone() - start.exprs[0].clone(),
                    point.exprs[1].clone() - start.exprs[1].clone(),
                ];
                let cross = direction_cross2(&point_delta, &direction);
                let direction_norm_squared = squared_norm2(&direction);
                let scaled_distance = distance.clone() * direction_norm_squared.clone().sqrt();
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredPointLineDistancePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::SquaredPointLineDistance),
                            residual: cross.clone() * cross.clone()
                                - distance.clone() * distance * direction_norm_squared,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::PointLineSignedDistancePositiveProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: cross.clone() - scaled_distance.clone(),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::PointLineSignedDistanceNegativeProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: cross + scaled_distance,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::PointOnCircle { point, circle } => {
                let mut diagnostics = Vec::new();
                let Some(point) = self.point_coordinates(point, constraint, &mut diagnostics)
                else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let Some(circle_entity) = self.entity(circle) else {
                    diagnostics.push(missing_entity_row(constraint, circle));
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let SketchEntityKind::Circle2(circle) = &circle_entity.kind else {
                    diagnostics.push(wrong_entity_row(constraint, circle_entity.handle, "circle"));
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let (Some(center), Some(radius)) = (
                    self.point_coordinates(circle.center, constraint, &mut diagnostics),
                    self.distance_expr(circle.radius, constraint, &mut diagnostics),
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                if point.len() != 2 || center.len() != 2 {
                    diagnostics.push(wrong_entity_row(
                        constraint,
                        point.handle,
                        "2D point compatible with circle",
                    ));
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                }
                let radius_squared = radius.clone() * radius.clone();
                let point_center_squared = squared_distance(&point.exprs, &center.exprs);
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredCircleIncidencePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::SquaredIncidence),
                            residual: point_center_squared.clone() - radius_squared,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::CircleRadialDistanceProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: point_center_squared.sqrt() - radius,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::PointOnCubic2 {
                point,
                cubic,
                parameter,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.point_on_cubic2_exprs(
                    point,
                    cubic,
                    parameter,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: parts
                        .into_iter()
                        .map(|residual| SketchResidualForm {
                            kind: SketchResidualFormKind::CubicBezierIncidencePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::CubicBezierIncidence),
                            residual,
                        })
                        .collect(),
                    diagnostics,
                }
            }
            SketchConstraintKind::EqualAngleLines2 { a, b, c, d } => {
                let mut diagnostics = Vec::new();
                let (Some(a), Some(b), Some(c), Some(d)) = (
                    self.line2_direction(a, constraint, &mut diagnostics),
                    self.line2_direction(b, constraint, &mut diagnostics),
                    self.line2_direction(c, constraint, &mut diagnostics),
                    self.line2_direction(d, constraint, &mut diagnostics),
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let squared_cosine = squared_cosine_angle_residual(&a, &b, &c, &d);
                let first_angle = angle_cosine_expr(&a, &b).acos();
                let second_angle = angle_cosine_expr(&c, &d).acos();
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::SquaredCosineAnglePolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::SquaredCosineAngleEquality),
                            residual: squared_cosine,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::TrueAngleProposal,
                            role: SketchResidualFormRole::ProposalOnly,
                            strategy: None,
                            residual: first_angle - second_angle,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::EqualOrientedAngleLines2 { a, b, c, d } => {
                let mut diagnostics = Vec::new();
                let Some(parts) =
                    self.oriented_angle_exprs(a, b, c, d, constraint, &mut diagnostics)
                else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::OrientedAngleVectorCollinearityPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::OrientedAngleEquality),
                            residual: parts.angle_vector_collinear,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::OrientedAngleSameBranchPredicate,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::OrientedAngleEquality),
                            residual: parts.same_branch,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::ProjectedEqualOrientedAngleLines3 {
                workplane,
                a,
                b,
                c,
                d,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.projected_oriented_angle_exprs(
                    workplane,
                    a,
                    b,
                    c,
                    d,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::WorkplaneUnitQuaternion),
                            residual: unit_quaternion_residual(&parts.quaternion),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::ProjectedOrientedAngleVectorCollinearityPolynomial,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::ProjectedOrientedAngleEquality),
                            residual: parts.angle.angle_vector_collinear,
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::ProjectedOrientedAngleSameBranchPredicate,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::ProjectedOrientedAngleEquality),
                            residual: parts.angle.same_branch,
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::TangentSameDirectionLines2 { candidate, target } => {
                let mut diagnostics = Vec::new();
                let (Some(candidate), Some(target)) = (
                    self.line2_direction(candidate, constraint, &mut diagnostics),
                    self.line2_direction(target, constraint, &mut diagnostics),
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms: vec![
                        SketchResidualForm {
                            kind: SketchResidualFormKind::TangentCrossProductPredicate,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::TangentSameDirection),
                            residual: direction_cross2(&candidate, &target),
                        },
                        SketchResidualForm {
                            kind: SketchResidualFormKind::TangentDotProductPredicate,
                            role: SketchResidualFormRole::ExactProof,
                            strategy: Some(SketchResidualStrategy::TangentSameDirection),
                            residual: direction_dot2(&candidate, &target),
                        },
                    ],
                    diagnostics,
                }
            }
            SketchConstraintKind::CubicLineTangent2 {
                cubic,
                parameter,
                line,
                line_endpoint,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.cubic_line_tangent_exprs(
                    cubic,
                    parameter,
                    line,
                    line_endpoint,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = parts
                    .endpoint_incidence
                    .into_iter()
                    .map(|residual| SketchResidualForm {
                        kind: SketchResidualFormKind::CubicLineTangentEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::CubicLineTangent),
                        residual,
                    })
                    .collect::<Vec<_>>();
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicLineTangentCrossProductPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicLineTangent),
                    residual: parts.tangent_cross,
                });
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicLineTangentDotProductPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicLineTangent),
                    residual: parts.tangent_dot,
                });
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::ArcCubicTangent2 {
                arc,
                arc_endpoint,
                cubic,
                parameter,
                orientation,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.arc_cubic_tangent_exprs(
                    arc,
                    arc_endpoint,
                    cubic,
                    parameter,
                    orientation,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = vec![
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcCubicTangentEndpointRadiusPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicTangent),
                        residual: parts.arc_endpoint_radius,
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcCubicTangentEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicTangent),
                        residual: parts.endpoint_incidence[0].clone(),
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcCubicTangentEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicTangent),
                        residual: parts.endpoint_incidence[1].clone(),
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcCubicTangentRadiusPerpendicularPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicTangent),
                        residual: parts.radius_perpendicular,
                    },
                ];
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::ArcCubicTangentOrientationPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::ArcCubicTangent),
                    residual: parts.orientation,
                });
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::ArcArcTangent2 {
                first,
                first_endpoint,
                second,
                second_endpoint,
                branch,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.arc_arc_tangent_exprs(
                    first,
                    first_endpoint,
                    second,
                    second_endpoint,
                    branch,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = Vec::with_capacity(6);
                forms.extend(parts.endpoint_radius.into_iter().map(|residual| {
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcArcTangentEndpointRadiusPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcArcTangent),
                        residual,
                    }
                }));
                forms.extend(parts.endpoint_incidence.into_iter().map(|residual| {
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcArcTangentEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcArcTangent),
                        residual,
                    }
                }));
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::ArcArcTangentRadiusCrossProductPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::ArcArcTangent),
                    residual: parts.radius_cross,
                });
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::ArcArcTangentRadiusBranchPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::ArcArcTangent),
                    residual: parts.radius_branch,
                });
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::ArcCubicSecondOrderContact2 {
                arc,
                arc_endpoint,
                cubic,
                parameter,
                orientation,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.arc_cubic_second_order_contact_exprs(
                    arc,
                    arc_endpoint,
                    cubic,
                    parameter,
                    orientation,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = vec![
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcCubicSecondOrderEndpointRadiusPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicSecondOrderContact),
                        residual: parts.arc_endpoint_radius,
                    },
                    SketchResidualForm {
                        kind:
                            SketchResidualFormKind::ArcCubicSecondOrderEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicSecondOrderContact),
                        residual: parts.endpoint_incidence[0].clone(),
                    },
                    SketchResidualForm {
                        kind:
                            SketchResidualFormKind::ArcCubicSecondOrderEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicSecondOrderContact),
                        residual: parts.endpoint_incidence[1].clone(),
                    },
                    SketchResidualForm {
                        kind:
                            SketchResidualFormKind::ArcCubicSecondOrderRadiusPerpendicularPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicSecondOrderContact),
                        residual: parts.radius_perpendicular,
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcCubicSecondOrderOrientationPredicate,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcCubicSecondOrderContact),
                        residual: parts.orientation,
                    },
                ];
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::ArcCubicSecondOrderContactPolynomial,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::ArcCubicSecondOrderContact),
                    residual: parts.second_order_contact,
                });
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::CubicCubicTangent2 {
                first,
                first_parameter,
                second,
                second_parameter,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.cubic_cubic_tangent_exprs(
                    first,
                    first_parameter,
                    second,
                    second_parameter,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = parts
                    .point_incidence
                    .into_iter()
                    .map(|residual| SketchResidualForm {
                        kind: SketchResidualFormKind::CubicCubicTangentPointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::CubicCubicTangent),
                        residual,
                    })
                    .collect::<Vec<_>>();
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicCubicTangentCrossProductPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicCubicTangent),
                    residual: parts.tangent_cross,
                });
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicCubicTangentDotProductPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicCubicTangent),
                    residual: parts.tangent_dot,
                });
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::CubicCubicG2Continuity2 {
                first,
                first_parameter,
                second,
                second_parameter,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.cubic_cubic_g2_exprs(
                    first,
                    first_parameter,
                    second,
                    second_parameter,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = parts
                    .point_incidence
                    .into_iter()
                    .map(|residual| SketchResidualForm {
                        kind: SketchResidualFormKind::CubicCubicG2PointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::CubicCubicG2Continuity),
                        residual,
                    })
                    .collect::<Vec<_>>();
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicCubicG2TangentCrossProductPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicCubicG2Continuity),
                    residual: parts.tangent_cross,
                });
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicCubicG2TangentDotProductPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicCubicG2Continuity),
                    residual: parts.tangent_dot,
                });
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicCubicG2CurvatureMagnitudePolynomial,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicCubicG2Continuity),
                    residual: parts.curvature_magnitude,
                });
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::CubicCubicG2CurvatureSignPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicCubicG2Continuity),
                    residual: parts.curvature_sign,
                });
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::CubicCubicC2Continuity2 {
                first,
                first_parameter,
                second,
                second_parameter,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.cubic_cubic_c2_exprs(
                    first,
                    first_parameter,
                    second,
                    second_parameter,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = Vec::with_capacity(6);
                forms.extend(parts.point.into_iter().map(|residual| SketchResidualForm {
                    kind: SketchResidualFormKind::CubicCubicC2PointPolynomial,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::CubicCubicC2Continuity),
                    residual,
                }));
                forms.extend(parts.first_derivative.into_iter().map(|residual| {
                    SketchResidualForm {
                        kind: SketchResidualFormKind::CubicCubicC2FirstDerivativePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::CubicCubicC2Continuity),
                        residual,
                    }
                }));
                forms.extend(parts.second_derivative.into_iter().map(|residual| {
                    SketchResidualForm {
                        kind: SketchResidualFormKind::CubicCubicC2SecondDerivativePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::CubicCubicC2Continuity),
                        residual,
                    }
                }));
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            SketchConstraintKind::ArcLineTangent2 {
                arc,
                arc_endpoint,
                line,
                line_endpoint,
                orientation,
            } => {
                let mut diagnostics = Vec::new();
                let Some(parts) = self.arc_line_tangent_exprs(
                    arc,
                    arc_endpoint,
                    line,
                    line_endpoint,
                    orientation,
                    constraint,
                    &mut diagnostics,
                ) else {
                    return SketchResidualFormsReport {
                        constraint: handle,
                        status: SketchResidualFormsStatus::InvalidInputs,
                        forms: Vec::new(),
                        diagnostics,
                    };
                };
                let mut forms = vec![
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcLineTangentEndpointRadiusPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcLineTangent),
                        residual: parts.arc_endpoint_radius,
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcLineTangentEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcLineTangent),
                        residual: parts.endpoint_incidence[0].clone(),
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcLineTangentEndpointIncidencePolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcLineTangent),
                        residual: parts.endpoint_incidence[1].clone(),
                    },
                    SketchResidualForm {
                        kind: SketchResidualFormKind::ArcLineTangentRadiusPerpendicularPolynomial,
                        role: SketchResidualFormRole::ExactProof,
                        strategy: Some(SketchResidualStrategy::ArcLineTangent),
                        residual: parts.radius_perpendicular,
                    },
                ];
                forms.push(SketchResidualForm {
                    kind: SketchResidualFormKind::ArcLineTangentOrientationPredicate,
                    role: SketchResidualFormRole::ExactProof,
                    strategy: Some(SketchResidualStrategy::ArcLineTangent),
                    residual: parts.orientation,
                });
                SketchResidualFormsReport {
                    constraint: handle,
                    status: SketchResidualFormsStatus::Generated,
                    forms,
                    diagnostics,
                }
            }
            _ => SketchResidualFormsReport {
                constraint: handle,
                status: SketchResidualFormsStatus::UnsupportedConstraint,
                forms: Vec::new(),
                diagnostics: Vec::new(),
            },
        }
    }

    fn lower_constraint(
        &self,
        constraint: &SketchConstraint,
        problem: &mut Problem,
        rows: &mut Vec<SketchGeneratedRow>,
    ) {
        if constraint.reference {
            rows.push(SketchGeneratedRow {
                constraint: constraint.handle,
                residual_index: None,
                name: constraint.name.clone(),
                strategy: None,
                status: SketchGeneratedRowStatus::ReferenceOnly,
            });
            return;
        }

        match constraint.kind {
            SketchConstraintKind::PointsCoincident { a, b } => {
                let (Some(a), Some(b)) = (
                    self.point_coordinates(a, constraint, rows),
                    self.point_coordinates(b, constraint, rows),
                ) else {
                    return;
                };
                if a.len() != b.len() {
                    rows.push(wrong_entity_row(
                        constraint,
                        b.handle,
                        "matching point dimension",
                    ));
                    return;
                }
                for (axis, (a_coord, b_coord)) in a.exprs.iter().zip(b.exprs.iter()).enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} coordinate {axis}", constraint.name),
                        a_coord.clone() - b_coord.clone(),
                        SketchResidualStrategy::CoordinateEquality,
                    );
                }
            }
            SketchConstraintKind::PointPointDistance { a, b, distance } => {
                let (Some(a), Some(b), Some(distance)) = (
                    self.point_coordinates(a, constraint, rows),
                    self.point_coordinates(b, constraint, rows),
                    self.distance_expr(distance, constraint, rows),
                ) else {
                    return;
                };
                if a.len() != b.len() {
                    rows.push(wrong_entity_row(
                        constraint,
                        b.handle,
                        "matching point dimension",
                    ));
                    return;
                }
                let residual = squared_distance(&a.exprs, &b.exprs) - distance.clone() * distance;
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    residual,
                    SketchResidualStrategy::SquaredDistance,
                );
            }
            SketchConstraintKind::PointPointDistanceRange {
                a,
                b,
                ref lower,
                ref upper,
            } => {
                // Yap, "Towards Exact Geometric Computation" (1997), treats
                // exact predicates as the correctness boundary. For distance
                // ranges that means validating the retained distance bounds
                // before lowering to polynomial squared-distance inequalities.
                if !validate_distance_bounds(constraint, lower.as_ref(), upper.as_ref(), rows) {
                    return;
                }
                let (Some(a), Some(b)) = (
                    self.point_coordinates(a, constraint, rows),
                    self.point_coordinates(b, constraint, rows),
                ) else {
                    return;
                };
                if a.len() != b.len() {
                    rows.push(wrong_entity_row(
                        constraint,
                        b.handle,
                        "matching point dimension",
                    ));
                    return;
                }
                let squared_distance = squared_distance(&a.exprs, &b.exprs);
                if let Some(lower) = lower {
                    self.push_residual_with_kind(
                        problem,
                        rows,
                        constraint,
                        format!("{} lower distance", constraint.name),
                        squared_distance.clone() - Expr::real(lower.clone() * lower.clone()),
                        SketchResidualStrategy::BoundedSquaredDistance,
                        ConstraintKind::GreaterOrEqual,
                        Real::one(),
                    );
                }
                if let Some(upper) = upper {
                    self.push_residual_with_kind(
                        problem,
                        rows,
                        constraint,
                        format!("{} upper distance", constraint.name),
                        squared_distance - Expr::real(upper.clone() * upper.clone()),
                        SketchResidualStrategy::BoundedSquaredDistance,
                        ConstraintKind::LessOrEqual,
                        Real::one(),
                    );
                }
                if lower.is_none() && upper.is_none() {
                    rows.push(SketchGeneratedRow {
                        constraint: constraint.handle,
                        residual_index: None,
                        name: constraint.name.clone(),
                        strategy: Some(SketchResidualStrategy::BoundedSquaredDistance),
                        status: SketchGeneratedRowStatus::ReferenceOnly,
                    });
                }
            }
            SketchConstraintKind::ProjectedPointPointDistanceRange {
                workplane,
                a,
                b,
                ref lower,
                ref upper,
            } => {
                // This is the exact workplane-projected counterpart of
                // bounded point distance. Validate the retained bounds before
                // squaring, then replay the workplane unit predicate and
                // projected squared-distance inequalities. This keeps Yap's
                // exact predicate boundary explicit and avoids accepting a
                // negative or inverted range by accident.
                if !validate_distance_bounds_with_strategy(
                    constraint,
                    lower.as_ref(),
                    upper.as_ref(),
                    SketchResidualStrategy::BoundedSquaredProjectedDistance,
                    rows,
                ) {
                    return;
                }
                if lower.is_none() && upper.is_none() {
                    rows.push(SketchGeneratedRow {
                        constraint: constraint.handle,
                        residual_index: None,
                        name: constraint.name.clone(),
                        strategy: Some(SketchResidualStrategy::BoundedSquaredProjectedDistance),
                        status: SketchGeneratedRowStatus::ReferenceOnly,
                    });
                    return;
                }
                let Some(parts) =
                    self.projected_point_range_exprs(workplane, a, b, constraint, rows)
                else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                if let Some(lower) = lower {
                    self.push_residual_with_kind(
                        problem,
                        rows,
                        constraint,
                        format!("{} lower projected distance", constraint.name),
                        parts.projected_squared.clone() - Expr::real(lower.clone() * lower.clone()),
                        SketchResidualStrategy::BoundedSquaredProjectedDistance,
                        ConstraintKind::GreaterOrEqual,
                        Real::one(),
                    );
                }
                if let Some(upper) = upper {
                    self.push_residual_with_kind(
                        problem,
                        rows,
                        constraint,
                        format!("{} upper projected distance", constraint.name),
                        parts.projected_squared - Expr::real(upper.clone() * upper.clone()),
                        SketchResidualStrategy::BoundedSquaredProjectedDistance,
                        ConstraintKind::LessOrEqual,
                        Real::one(),
                    );
                }
            }
            SketchConstraintKind::ProjectedPointPointDistance {
                workplane,
                a,
                b,
                distance,
            } => {
                // This is the exact workplane-projected distance package.
                // Yap, "Towards Exact Geometric Computation" (1997), is the
                // reason the retained workplane emits its unit-frame guard as
                // a proof row instead of letting a proposal engine normalize
                // the quaternion with primitive floats. The `U/V` axes are the
                // polynomial unit-quaternion frame from Shoemake (1985).
                let Some(parts) = self
                    .projected_point_distance_exprs(workplane, a, b, distance, constraint, rows)
                else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected distance", constraint.name),
                    parts.projected_squared - parts.distance.clone() * parts.distance,
                    SketchResidualStrategy::SquaredProjectedDistance,
                );
            }
            SketchConstraintKind::ProjectedPointLineDistance {
                workplane,
                point,
                line,
                distance,
            } => {
                // This is the 3D/workplane analogue of the 2D point-line
                // distance row. The line direction is projected into the
                // retained workplane and the denominator is cleared exactly,
                // so candidate acceptance never depends on normalized float
                // line coordinates. Yap (1997) supplies the proof boundary;
                // Shoemake (1985) supplies the quaternion frame polynomial.
                let Some(parts) = self.projected_point_line_distance_exprs(
                    workplane, point, line, distance, constraint, rows,
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected point-line distance", constraint.name),
                    parts.distance_numerator
                        - parts.distance.clone() * parts.distance * parts.distance_denominator,
                    SketchResidualStrategy::SquaredProjectedPointLineDistance,
                );
            }
            SketchConstraintKind::ProjectedEqualLengthLines3 { workplane, a, b } => {
                // This is the retained 3D/workplane equal-length package. The
                // workplane unit guard is generated as proof because the U/V
                // projected metric is meaningful only for a certified frame.
                // The line directions themselves are never normalized; Yap
                // (1997) keeps the acceptance boundary at exact polynomial
                // replay, while Shoemake (1985) gives the quaternion frame.
                let Some(parts) =
                    self.projected_line_length_equality_exprs(workplane, a, b, constraint, rows)
                else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected line length equality", constraint.name),
                    parts.first_squared - parts.second_squared,
                    SketchResidualStrategy::SquaredProjectedLineLengthEquality,
                );
            }
            SketchConstraintKind::ProjectedLengthRatioLines3 {
                workplane,
                a,
                b,
                ref numerator,
                ref denominator,
            } => {
                // This extends projected line length from equality to exact
                // ratios. The ratio signs are certified before the polynomial
                // row is squared, so invalid semantic inputs cannot pass by
                // symmetry of squares. Yap (1997) keeps the ratio-domain and
                // workplane-frame obligations explicit proof rows.
                if !validate_length_ratio(
                    constraint,
                    numerator,
                    denominator,
                    SketchResidualStrategy::SquaredProjectedLineLengthRatio,
                    rows,
                ) {
                    return;
                }
                let Some(parts) =
                    self.projected_line_length_equality_exprs(workplane, a, b, constraint, rows)
                else {
                    return;
                };
                let numerator_squared = Expr::real(numerator.clone() * numerator.clone());
                let denominator_squared = Expr::real(denominator.clone() * denominator.clone());
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected line length ratio", constraint.name),
                    parts.first_squared * denominator_squared
                        - parts.second_squared * numerator_squared,
                    SketchResidualStrategy::SquaredProjectedLineLengthRatio,
                );
            }
            SketchConstraintKind::ProjectedLengthDifferenceLines3 {
                workplane,
                longer,
                shorter,
                difference,
            } => {
                // This is the projected counterpart of 2D length difference:
                // no projected square root is trusted. The equality row
                // proves the algebraic relation and the inequality selects
                // the intended longer-branch, as required by Yap's EGC
                // separation of construction and certified decision.
                let Some((parts, difference)) = self.projected_line_length_difference_exprs(
                    workplane, longer, shorter, difference, constraint, rows,
                ) else {
                    return;
                };
                let difference_squared = difference.clone() * difference;
                let sum_minus_difference = parts.first_squared.clone()
                    + parts.second_squared.clone()
                    - difference_squared.clone();
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected squared difference", constraint.name),
                    sum_minus_difference.clone() * sum_minus_difference
                        - Expr::int(4) * parts.first_squared.clone() * parts.second_squared.clone(),
                    SketchResidualStrategy::SquaredProjectedLineLengthDifference,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected longer branch", constraint.name),
                    parts.first_squared - parts.second_squared - difference_squared,
                    SketchResidualStrategy::SquaredProjectedLineLengthDifference,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::ProjectedEqualLengthPointLineDistance3 {
                workplane,
                length_line,
                point,
                distance_line,
            } => {
                // This is the retained workplane analogue of the 2D
                // line-length-to-point-line-distance relation. The projected
                // point-line denominator is cleared before replay, so no
                // normalized projected line or square root is part of the
                // trusted predicate. Yap (1997) supplies that proof boundary;
                // Shoemake (1985) supplies the quaternion frame polynomial.
                let Some(parts) = self.projected_equal_length_point_line_distance_exprs(
                    workplane,
                    length_line,
                    point,
                    distance_line,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected length point-line distance", constraint.name),
                    parts.length_squared * parts.distance_denominator - parts.distance_numerator,
                    SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance,
                );
            }
            SketchConstraintKind::ProjectedEqualPointLineDistances3 {
                workplane,
                a_point,
                a_line,
                b_point,
                b_line,
            } => {
                // This is the workplane counterpart of equality between two
                // point-line distances. Both projected line denominators are
                // cross-multiplied, so neither distance is normalized through
                // a lossy proposal coordinate. Yap (1997) keeps that exact
                // polynomial replay as the trust boundary; Shoemake (1985)
                // gives the retained quaternion frame polynomial.
                let Some(parts) = self.projected_equal_point_line_distances_exprs(
                    workplane,
                    [(a_point, a_line), (b_point, b_line)],
                    constraint,
                    rows,
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected equal point-line distances", constraint.name),
                    parts.a_numerator * parts.b_denominator
                        - parts.b_numerator * parts.a_denominator,
                    SketchResidualStrategy::SquaredProjectedEqualPointLineDistances,
                );
            }
            SketchConstraintKind::EqualLengthLines2 { a, b } => {
                // Equal segment length has a square-root proposal reading, but
                // Yap's exact-geometric-computation boundary is cleaner as
                // squared-length polynomial replay.
                let (Some(a), Some(b)) = (
                    self.line2_direction(a, constraint, rows),
                    self.line2_direction(b, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    squared_norm2(&a) - squared_norm2(&b),
                    SketchResidualStrategy::SquaredLineLengthEquality,
                );
            }
            SketchConstraintKind::LengthRatioLines2 {
                a,
                b,
                ref numerator,
                ref denominator,
            } => {
                // This keeps the exact proof in polynomial space. Squaring the
                // ratio is safe only after exact sign validation, otherwise a
                // negative ratio could be silently accepted.
                if !validate_length_ratio(
                    constraint,
                    numerator,
                    denominator,
                    SketchResidualStrategy::SquaredLineLengthRatio,
                    rows,
                ) {
                    return;
                }
                let (Some(a), Some(b)) = (
                    self.line2_direction(a, constraint, rows),
                    self.line2_direction(b, constraint, rows),
                ) else {
                    return;
                };
                let numerator_squared = Expr::real(numerator.clone() * numerator.clone());
                let denominator_squared = Expr::real(denominator.clone() * denominator.clone());
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    squared_norm2(&a) * denominator_squared - squared_norm2(&b) * numerator_squared,
                    SketchResidualStrategy::SquaredLineLengthRatio,
                );
            }
            SketchConstraintKind::LengthDifferenceLines2 {
                longer,
                shorter,
                difference,
            } => {
                // For lengths l, s and difference d, the equation
                // l = s + d can be replayed without square roots as
                // (l^2 + s^2 - d^2)^2 - 4*l^2*s^2 == 0. The companion
                // inequality l^2 - s^2 - d^2 >= 0 selects the intended
                // nonnegative branch. Yap (1997) is the reason the branch is
                // explicit instead of hidden in a floating proposal.
                let (Some(longer), Some(shorter), Some(difference)) = (
                    self.line2_direction(longer, constraint, rows),
                    self.line2_direction(shorter, constraint, rows),
                    self.distance_expr(difference, constraint, rows),
                ) else {
                    return;
                };
                let longer_squared = squared_norm2(&longer);
                let shorter_squared = squared_norm2(&shorter);
                let difference_squared = difference.clone() * difference;
                let sum_minus_difference =
                    longer_squared.clone() + shorter_squared.clone() - difference_squared.clone();
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} squared difference", constraint.name),
                    sum_minus_difference.clone() * sum_minus_difference
                        - Expr::int(4) * longer_squared.clone() * shorter_squared.clone(),
                    SketchResidualStrategy::SquaredLineLengthDifference,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} longer branch", constraint.name),
                    longer_squared - shorter_squared - difference_squared,
                    SketchResidualStrategy::SquaredLineLengthDifference,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::PointLineDistance2 {
                point,
                line,
                distance,
            } => {
                // The exact polynomial form is equivalent to the usual
                // point-line distance under explicit nondegenerate-line and
                // nonnegative-distance domain assumptions. Those assumptions
                // remain visible domain/preflight obligations.
                let (Some(point), Some((start, end)), Some(distance)) = (
                    self.point2_coordinates(point, constraint, rows),
                    self.line2_points(line, constraint, rows),
                    self.distance_expr(distance, constraint, rows),
                ) else {
                    return;
                };
                let direction = [
                    end.exprs[0].clone() - start.exprs[0].clone(),
                    end.exprs[1].clone() - start.exprs[1].clone(),
                ];
                let point_delta = [
                    point.exprs[0].clone() - start.exprs[0].clone(),
                    point.exprs[1].clone() - start.exprs[1].clone(),
                ];
                let cross = direction_cross2(&point_delta, &direction);
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    cross.clone() * cross - distance.clone() * distance * squared_norm2(&direction),
                    SketchResidualStrategy::SquaredPointLineDistance,
                );
            }
            SketchConstraintKind::EqualLengthPointLineDistance2 {
                length_line,
                point,
                distance_line,
            } => {
                // This is the algebraic proof package for SolveSpace's
                // equal-line-length-to-point-line-distance relation. Yap
                // (1997) keeps the division/square-root-free predicate as the
                // acceptance boundary; degenerate-line preconditions stay
                // reportable elsewhere.
                let (Some(length_direction), Some((distance_numerator, distance_denominator))) = (
                    self.line2_direction(length_line, constraint, rows),
                    self.point_line_distance_squared_parts(point, distance_line, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    squared_norm2(&length_direction) * distance_denominator - distance_numerator,
                    SketchResidualStrategy::SquaredLineLengthPointLineDistance,
                );
            }
            SketchConstraintKind::EqualPointLineDistances2 {
                a_point,
                a_line,
                b_point,
                b_line,
            } => {
                // Cross-multiplying squared distance forms keeps the proof
                // exact and avoids deciding a square root during replay.
                let (Some((a_numerator, a_denominator)), Some((b_numerator, b_denominator))) = (
                    self.point_line_distance_squared_parts(a_point, a_line, constraint, rows),
                    self.point_line_distance_squared_parts(b_point, b_line, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    a_numerator * b_denominator - b_numerator * a_denominator,
                    SketchResidualStrategy::SquaredEqualPointLineDistances,
                );
            }
            SketchConstraintKind::EqualRadius2 { a, b } => {
                // Radius equality is exact scalar equality over retained
                // circle/arc radius carriers. Positive-radius semantics remain
                // explicit preflight domains.
                let (Some(a), Some(b)) = (
                    self.circle_or_arc_radius_expr(a, constraint, rows),
                    self.circle_or_arc_radius_expr(b, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    a - b,
                    SketchResidualStrategy::RadiusEquality,
                );
            }
            SketchConstraintKind::Horizontal { line } => {
                let Some((start, end)) = self.line2_points(line, constraint, rows) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    start.exprs[1].clone() - end.exprs[1].clone(),
                    SketchResidualStrategy::CoordinateEquality,
                );
            }
            SketchConstraintKind::Vertical { line } => {
                let Some((start, end)) = self.line2_points(line, constraint, rows) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    start.exprs[0].clone() - end.exprs[0].clone(),
                    SketchResidualStrategy::CoordinateEquality,
                );
            }
            SketchConstraintKind::ParallelLines2 { a, b } => {
                let (Some(a), Some(b)) = (
                    self.line2_direction(a, constraint, rows),
                    self.line2_direction(b, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    direction_cross2(&a, &b),
                    SketchResidualStrategy::DirectionCrossProduct,
                );
            }
            SketchConstraintKind::PerpendicularLines2 { a, b } => {
                let (Some(a), Some(b)) = (
                    self.line2_direction(a, constraint, rows),
                    self.line2_direction(b, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    direction_dot2(&a, &b),
                    SketchResidualStrategy::DirectionDotProduct,
                );
            }
            SketchConstraintKind::SameDirectionLines2 { a, b } => {
                // Yap, "Towards Exact Geometric Computation" (1997), keeps
                // the exact predicate package explicit. Same-direction support
                // is certified by cross == 0, while orientation is certified
                // by dot >= 0 without lossy normalization.
                let (Some(a), Some(b)) = (
                    self.line2_direction(a, constraint, rows),
                    self.line2_direction(b, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} parallel support", constraint.name),
                    direction_cross2(&a, &b),
                    SketchResidualStrategy::DirectionSameOrientation,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} orientation", constraint.name),
                    direction_dot2(&a, &b),
                    SketchResidualStrategy::DirectionSameOrientation,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::TangentSameDirectionLines2 { candidate, target } => {
                // This is the retained G1 tangent-carrier package. Yap,
                // "Towards Exact Geometric Computation" (1997), puts trust in
                // exact predicates, so the tangent branch is certified by
                // cross == 0 and dot >= 0 without normalizing or dividing by
                // tangent length.
                let (Some(candidate), Some(target)) = (
                    self.line2_direction(candidate, constraint, rows),
                    self.line2_direction(target, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} tangent support", constraint.name),
                    direction_cross2(&candidate, &target),
                    SketchResidualStrategy::TangentSameDirection,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} tangent orientation", constraint.name),
                    direction_dot2(&candidate, &target),
                    SketchResidualStrategy::TangentSameDirection,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::CubicLineTangent2 {
                cubic,
                parameter,
                line,
                line_endpoint,
            } => {
                // This is a retained differential cubic tangency package.
                // The cubic point and derivative are exact Bernstein
                // expressions, while acceptance is exact replay of endpoint
                // incidence, tangent support, and same-direction branch. No
                // derivative or line tangent is normalized by a proposal path.
                let Some(parts) = self.cubic_line_tangent_exprs(
                    cubic,
                    parameter,
                    line,
                    line_endpoint,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                for (axis, residual) in parts.endpoint_incidence.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} cubic endpoint coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::CubicLineTangent,
                    );
                }
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} cubic tangent support", constraint.name),
                    parts.tangent_cross,
                    SketchResidualStrategy::CubicLineTangent,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} cubic tangent orientation", constraint.name),
                    parts.tangent_dot,
                    SketchResidualStrategy::CubicLineTangent,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::ArcCubicTangent2 {
                arc,
                arc_endpoint,
                cubic,
                parameter,
                orientation,
            } => {
                // This is the retained mixed arc/cubic differential package:
                // exact arc endpoint/radius validation plus exact Bernstein
                // point/derivative rows. No tangent vector is normalized, so
                // stationary parameters and degenerate arcs stay explicit
                // domain/preflight obligations.
                let Some(parts) = self.arc_cubic_tangent_exprs(
                    arc,
                    arc_endpoint,
                    cubic,
                    parameter,
                    orientation,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} arc endpoint radius", constraint.name),
                    parts.arc_endpoint_radius,
                    SketchResidualStrategy::ArcCubicTangent,
                );
                for (axis, residual) in parts.endpoint_incidence.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} cubic endpoint coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::ArcCubicTangent,
                    );
                }
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} radius cubic perpendicular", constraint.name),
                    parts.radius_perpendicular,
                    SketchResidualStrategy::ArcCubicTangent,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} cubic tangent orientation", constraint.name),
                    parts.orientation,
                    SketchResidualStrategy::ArcCubicTangent,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::ArcArcTangent2 {
                first,
                first_endpoint,
                second,
                second_endpoint,
                branch,
            } => {
                // Arc/arc tangency is a retained branch package. The same or
                // opposite radius-vector choice is replayed as an exact
                // inequality instead of being inferred from a floating circle
                // contact classifier.
                let Some(parts) = self.arc_arc_tangent_exprs(
                    first,
                    first_endpoint,
                    second,
                    second_endpoint,
                    branch,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                for (index, residual) in parts.endpoint_radius.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} arc endpoint radius {index}", constraint.name),
                        residual,
                        SketchResidualStrategy::ArcArcTangent,
                    );
                }
                for (axis, residual) in parts.endpoint_incidence.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} endpoint coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::ArcArcTangent,
                    );
                }
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} radius collinearity", constraint.name),
                    parts.radius_cross,
                    SketchResidualStrategy::ArcArcTangent,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} radius branch", constraint.name),
                    parts.radius_branch,
                    SketchResidualStrategy::ArcArcTangent,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::ArcCubicSecondOrderContact2 {
                arc,
                arc_endpoint,
                cubic,
                parameter,
                orientation,
            } => {
                // Second-order arc/cubic contact differentiates the retained
                // circle incidence equation instead of computing curvature
                // through primitive-float normalization.
                let Some(parts) = self.arc_cubic_second_order_contact_exprs(
                    arc,
                    arc_endpoint,
                    cubic,
                    parameter,
                    orientation,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} arc endpoint radius", constraint.name),
                    parts.arc_endpoint_radius,
                    SketchResidualStrategy::ArcCubicSecondOrderContact,
                );
                for (axis, residual) in parts.endpoint_incidence.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} cubic endpoint coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::ArcCubicSecondOrderContact,
                    );
                }
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} radius cubic perpendicular", constraint.name),
                    parts.radius_perpendicular,
                    SketchResidualStrategy::ArcCubicSecondOrderContact,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} cubic tangent orientation", constraint.name),
                    parts.orientation,
                    SketchResidualStrategy::ArcCubicSecondOrderContact,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} cubic second order circle contact", constraint.name),
                    parts.second_order_contact,
                    SketchResidualStrategy::ArcCubicSecondOrderContact,
                );
            }
            SketchConstraintKind::CubicCubicTangent2 {
                first,
                first_parameter,
                second,
                second_parameter,
            } => {
                // This is the curve/curve differential package: exact
                // Bernstein point coincidence plus unnormalized derivative
                // cross/dot predicates. Degenerate derivative handling stays
                // a reportable domain/preflight concern.
                let Some(parts) = self.cubic_cubic_tangent_exprs(
                    first,
                    first_parameter,
                    second,
                    second_parameter,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                for (axis, residual) in parts.point_incidence.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} cubic point coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::CubicCubicTangent,
                    );
                }
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} cubic tangent support", constraint.name),
                    parts.tangent_cross,
                    SketchResidualStrategy::CubicCubicTangent,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} cubic tangent orientation", constraint.name),
                    parts.tangent_dot,
                    SketchResidualStrategy::CubicCubicTangent,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::CubicCubicG2Continuity2 {
                first,
                first_parameter,
                second,
                second_parameter,
            } => {
                // Geometric G2 continuity compares signed curvature after
                // clearing speed denominators. This keeps the branch evidence
                // exact and avoids normalizing derivatives into primitive
                // curvature values.
                let Some(parts) = self.cubic_cubic_g2_exprs(
                    first,
                    first_parameter,
                    second,
                    second_parameter,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                for (axis, residual) in parts.point_incidence.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} g2 cubic point coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::CubicCubicG2Continuity,
                    );
                }
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} g2 cubic tangent support", constraint.name),
                    parts.tangent_cross,
                    SketchResidualStrategy::CubicCubicG2Continuity,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} g2 cubic tangent orientation", constraint.name),
                    parts.tangent_dot,
                    SketchResidualStrategy::CubicCubicG2Continuity,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} g2 curvature magnitude", constraint.name),
                    parts.curvature_magnitude,
                    SketchResidualStrategy::CubicCubicG2Continuity,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} g2 curvature sign", constraint.name),
                    parts.curvature_sign,
                    SketchResidualStrategy::CubicCubicG2Continuity,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::CubicCubicC2Continuity2 {
                first,
                first_parameter,
                second,
                second_parameter,
            } => {
                // Parametric C2 continuity is deliberately stricter than
                // geometric smoothness. It equates point, velocity, and
                // acceleration vectors exactly, preserving the retained
                // parameterization as proof input.
                let Some(parts) = self.cubic_cubic_c2_exprs(
                    first,
                    first_parameter,
                    second,
                    second_parameter,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                for (axis, residual) in parts.point.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} c2 point coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::CubicCubicC2Continuity,
                    );
                }
                for (axis, residual) in parts.first_derivative.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} c2 first derivative {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::CubicCubicC2Continuity,
                    );
                }
                for (axis, residual) in parts.second_derivative.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} c2 second derivative {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::CubicCubicC2Continuity,
                    );
                }
            }
            SketchConstraintKind::ArcLineTangent2 {
                arc,
                arc_endpoint,
                line,
                line_endpoint,
                orientation,
            } => {
                // This is the retained arc-line tangent package. Yap (1997)
                // is the reason endpoint incidence, endpoint-on-radius,
                // perpendicularity, and signed orientation are separate exact
                // rows rather than a normalized floating tangent test. The
                // endpoint/orientation flags mirror the public constraint
                // vocabulary discussed by Bouma et al. (1995).
                let Some(parts) = self.arc_line_tangent_exprs(
                    arc,
                    arc_endpoint,
                    line,
                    line_endpoint,
                    orientation,
                    constraint,
                    rows,
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} arc endpoint radius", constraint.name),
                    parts.arc_endpoint_radius,
                    SketchResidualStrategy::ArcLineTangent,
                );
                for (axis, residual) in parts.endpoint_incidence.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} endpoint coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::ArcLineTangent,
                    );
                }
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} radius perpendicular", constraint.name),
                    parts.radius_perpendicular,
                    SketchResidualStrategy::ArcLineTangent,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} tangent orientation", constraint.name),
                    parts.orientation,
                    SketchResidualStrategy::ArcLineTangent,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::EqualAngleLines2 { a, b, c, d } => {
                // Equal unsigned angle is certified by squared cosine equality
                // rather than by evaluating an inverse trig function. This is
                // Yap's exact-geometric-computation boundary applied to angle
                // constraints; branch-sensitive equality is handled by
                // `EqualOrientedAngleLines2`.
                let (Some(a), Some(b), Some(c), Some(d)) = (
                    self.line2_direction(a, constraint, rows),
                    self.line2_direction(b, constraint, rows),
                    self.line2_direction(c, constraint, rows),
                    self.line2_direction(d, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    squared_cosine_angle_residual(&a, &b, &c, &d),
                    SketchResidualStrategy::SquaredCosineAngleEquality,
                );
            }
            SketchConstraintKind::EqualOrientedAngleLines2 { a, b, c, d } => {
                // Branch-sensitive angle equality is certified by the exact
                // angle vector `(dot, cross)`. Yap (1997) motivates avoiding
                // `atan2` in the proof path; the second row is a predicate that
                // rejects the opposite/supplemental angle-vector branch.
                let Some(parts) = self.oriented_angle_exprs(a, b, c, d, constraint, rows) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} angle vector collinearity", constraint.name),
                    parts.angle_vector_collinear,
                    SketchResidualStrategy::OrientedAngleEquality,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} same angle branch", constraint.name),
                    parts.same_branch,
                    SketchResidualStrategy::OrientedAngleEquality,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::ProjectedEqualOrientedAngleLines3 {
                workplane,
                a,
                b,
                c,
                d,
            } => {
                // This is the workplane-projected 3D angle package. The
                // workplane unit guard is part of the proof because the U/V
                // metric is valid only for a certified frame. Once the 3D
                // directions are projected, the same exact `(dot, cross)`
                // angle-vector rows used by `EqualOrientedAngleLines2`
                // certify equality and branch without `atan2`.
                let Some(parts) =
                    self.projected_oriented_angle_exprs(workplane, a, b, c, d, constraint, rows)
                else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    unit_quaternion_residual(&parts.quaternion),
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected angle vector collinearity", constraint.name),
                    parts.angle.angle_vector_collinear,
                    SketchResidualStrategy::ProjectedOrientedAngleEquality,
                );
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    format!("{} projected same angle branch", constraint.name),
                    parts.angle.same_branch,
                    SketchResidualStrategy::ProjectedOrientedAngleEquality,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::AtMidpoint2 { point, a, b } => {
                // Yap, "Towards Exact Geometric Computation" (1997), makes
                // exact predicates the decision boundary. The midpoint
                // relation therefore lowers to integer-coefficient linear
                // equations instead of computing a rounded midpoint.
                let (Some(point), Some(a), Some(b)) = (
                    self.point2_coordinates(point, constraint, rows),
                    self.point2_coordinates(a, constraint, rows),
                    self.point2_coordinates(b, constraint, rows),
                ) else {
                    return;
                };
                for axis in 0..2 {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} coordinate {axis}", constraint.name),
                        Expr::int(2) * point.exprs[axis].clone()
                            - a.exprs[axis].clone()
                            - b.exprs[axis].clone(),
                        SketchResidualStrategy::MidpointCoordinateEquality,
                    );
                }
            }
            SketchConstraintKind::SymmetricHorizontal2 { a, b, ref axis_y } => {
                // Horizontal symmetry is exact affine replay: matching x
                // coordinates and a y-coordinate sum fixed to twice the exact
                // mirror axis. No midpoint rounding is introduced.
                let (Some(a), Some(b)) = (
                    self.point2_coordinates(a, constraint, rows),
                    self.point2_coordinates(b, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} x coordinate", constraint.name),
                    a.exprs[0].clone() - b.exprs[0].clone(),
                    SketchResidualStrategy::AxisSymmetryCoordinateEquality,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} y mirror", constraint.name),
                    a.exprs[1].clone() + b.exprs[1].clone()
                        - Expr::real(axis_y.clone() * Real::from(2)),
                    SketchResidualStrategy::AxisSymmetryCoordinateEquality,
                );
            }
            SketchConstraintKind::SymmetricVertical2 { a, b, ref axis_x } => {
                // Vertical symmetry mirrors the horizontal package with exact
                // affine rows. The retained axis value is proof input, not an
                // editor tolerance.
                let (Some(a), Some(b)) = (
                    self.point2_coordinates(a, constraint, rows),
                    self.point2_coordinates(b, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} y coordinate", constraint.name),
                    a.exprs[1].clone() - b.exprs[1].clone(),
                    SketchResidualStrategy::AxisSymmetryCoordinateEquality,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} x mirror", constraint.name),
                    a.exprs[0].clone() + b.exprs[0].clone()
                        - Expr::real(axis_x.clone() * Real::from(2)),
                    SketchResidualStrategy::AxisSymmetryCoordinateEquality,
                );
            }
            SketchConstraintKind::SymmetricLine2 { a, b, axis } => {
                // Yap, "Towards Exact Geometric Computation" (1997), keeps
                // geometric decisions at exact predicate replay. A reflected
                // pair across a line is therefore certified by midpoint
                // incidence and perpendicularity, both using the unnormalized
                // retained line direction.
                let (Some(a), Some(b), Some((start, end))) = (
                    self.point2_coordinates(a, constraint, rows),
                    self.point2_coordinates(b, constraint, rows),
                    self.line2_points(axis, constraint, rows),
                ) else {
                    return;
                };
                let axis_direction = [
                    end.exprs[0].clone() - start.exprs[0].clone(),
                    end.exprs[1].clone() - start.exprs[1].clone(),
                ];
                let doubled_midpoint_delta = [
                    a.exprs[0].clone() + b.exprs[0].clone() - Expr::int(2) * start.exprs[0].clone(),
                    a.exprs[1].clone() + b.exprs[1].clone() - Expr::int(2) * start.exprs[1].clone(),
                ];
                let point_delta = [
                    a.exprs[0].clone() - b.exprs[0].clone(),
                    a.exprs[1].clone() - b.exprs[1].clone(),
                ];
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} midpoint on axis", constraint.name),
                    direction_cross2(&doubled_midpoint_delta, &axis_direction),
                    SketchResidualStrategy::LineSymmetryPolynomial,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} perpendicular offset", constraint.name),
                    direction_dot2(&point_delta, &axis_direction),
                    SketchResidualStrategy::LineSymmetryPolynomial,
                );
            }
            SketchConstraintKind::SymmetricWorkplane3 { a, b, workplane } => {
                // This is the retained 3D mirror-plane package. Following Yap
                // (1997), the workplane is not normalized by a proposal engine:
                // exact replay first checks the unit-quaternion guard, then
                // proves midpoint incidence and normal offset using the
                // Shoemake unit-quaternion frame polynomial.
                let Some(parts) = self.workplane_symmetry_exprs(workplane, a, b, constraint, rows)
                else {
                    return;
                };
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} workplane unit", constraint.name),
                    parts.unit_guard,
                    SketchResidualStrategy::WorkplaneUnitQuaternion,
                );
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    format!("{} midpoint on workplane", constraint.name),
                    parts.midpoint_plane,
                    SketchResidualStrategy::WorkplaneSymmetryPolynomial,
                );
                for (axis, residual) in parts.normal_offset_cross.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} normal offset cross {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::WorkplaneSymmetryPolynomial,
                    );
                }
            }
            SketchConstraintKind::PointOnCircle { point, circle } => {
                let Some(point) = self.point_coordinates(point, constraint, rows) else {
                    return;
                };
                let Some(circle) = self.entity(circle) else {
                    rows.push(missing_entity_row(constraint, circle));
                    return;
                };
                let SketchEntityKind::Circle2(circle) = &circle.kind else {
                    rows.push(wrong_entity_row(constraint, circle.handle, "circle"));
                    return;
                };
                let (Some(center), Some(radius)) = (
                    self.point_coordinates(circle.center, constraint, rows),
                    self.distance_expr(circle.radius, constraint, rows),
                ) else {
                    return;
                };
                if point.len() != 2 || center.len() != 2 {
                    rows.push(wrong_entity_row(
                        constraint,
                        point.handle,
                        "2D point compatible with circle",
                    ));
                    return;
                }
                let residual =
                    squared_distance(&point.exprs, &center.exprs) - radius.clone() * radius;
                self.push_residual(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    residual,
                    SketchResidualStrategy::SquaredIncidence,
                );
            }
            SketchConstraintKind::PointOnCubic2 {
                point,
                cubic,
                parameter,
            } => {
                // This is retained cubic-Bezier incidence, not a sampled
                // curve test. Yap (1997) puts trust in exact replay, so the
                // de Casteljau/Bernstein curve equations are emitted per axis
                // and the segment-domain assumption for `t` remains an
                // explicit parameter-domain or range obligation.
                let Some(residuals) =
                    self.point_on_cubic2_exprs(point, cubic, parameter, constraint, rows)
                else {
                    return;
                };
                for (axis, residual) in residuals.into_iter().enumerate() {
                    self.push_residual(
                        problem,
                        rows,
                        constraint,
                        format!("{} cubic coordinate {axis}", constraint.name),
                        residual,
                        SketchResidualStrategy::CubicBezierIncidence,
                    );
                }
            }
            SketchConstraintKind::ParameterRange {
                parameter,
                ref lower,
                ref upper,
            } => {
                let Some(parameter_expr) = self.parameter_expr(parameter, constraint, rows) else {
                    return;
                };
                if let Some(lower) = lower {
                    self.push_residual_with_kind(
                        problem,
                        rows,
                        constraint,
                        format!("{} lower", constraint.name),
                        parameter_expr.clone() - Expr::real(lower.clone()),
                        SketchResidualStrategy::ParameterRange,
                        ConstraintKind::GreaterOrEqual,
                        Real::one(),
                    );
                }
                if let Some(upper) = upper {
                    self.push_residual_with_kind(
                        problem,
                        rows,
                        constraint,
                        format!("{} upper", constraint.name),
                        parameter_expr - Expr::real(upper.clone()),
                        SketchResidualStrategy::ParameterRange,
                        ConstraintKind::LessOrEqual,
                        Real::one(),
                    );
                }
                if lower.is_none() && upper.is_none() {
                    rows.push(SketchGeneratedRow {
                        constraint: constraint.handle,
                        residual_index: None,
                        name: constraint.name.clone(),
                        strategy: Some(SketchResidualStrategy::ParameterRange),
                        status: SketchGeneratedRowStatus::ReferenceOnly,
                    });
                }
            }
            SketchConstraintKind::ParameterOrdering { lower, upper } => {
                // Yap, "Towards Exact Geometric Computation" (1997), keeps
                // the semantic relation separate from the exact predicate
                // that certifies it. Monotonicity lowers to one scalar
                // inequality and lets candidate replay prove the sign.
                let (Some(lower_expr), Some(upper_expr)) = (
                    self.parameter_expr(lower, constraint, rows),
                    self.parameter_expr(upper, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    upper_expr - lower_expr,
                    SketchResidualStrategy::ParameterOrdering,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::ParameterMargin {
                lower,
                upper,
                ref margin,
            } => {
                // This is the scalar form of a design-rule margin. Following
                // Yap (1997), the margin is retained as exact input and checked
                // before it participates in the proof polynomial.
                if !validate_parameter_margin(constraint, margin, rows) {
                    return;
                }
                let (Some(lower_expr), Some(upper_expr)) = (
                    self.parameter_expr(lower, constraint, rows),
                    self.parameter_expr(upper, constraint, rows),
                ) else {
                    return;
                };
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    upper_expr - lower_expr - Expr::real(margin.clone()),
                    SketchResidualStrategy::ParameterMargin,
                    ConstraintKind::GreaterOrEqual,
                    Real::one(),
                );
            }
            SketchConstraintKind::StayNearParameter {
                parameter,
                ref target,
                ref weight,
            } => {
                let Some(parameter_expr) = self.parameter_expr(parameter, constraint, rows) else {
                    return;
                };
                self.push_residual_with_kind(
                    problem,
                    rows,
                    constraint,
                    constraint.name.clone(),
                    parameter_expr - Expr::real(target.clone()),
                    SketchResidualStrategy::SoftObjective,
                    ConstraintKind::Soft,
                    weight.clone(),
                );
            }
        }
    }

    fn push_residual(
        &self,
        problem: &mut Problem,
        rows: &mut Vec<SketchGeneratedRow>,
        constraint: &SketchConstraint,
        name: String,
        residual: Expr,
        strategy: SketchResidualStrategy,
    ) {
        self.push_residual_with_kind(
            problem,
            rows,
            constraint,
            name,
            residual,
            strategy,
            ConstraintKind::Equality,
            Real::one(),
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn push_residual_with_kind(
        &self,
        problem: &mut Problem,
        rows: &mut Vec<SketchGeneratedRow>,
        constraint: &SketchConstraint,
        name: String,
        residual: Expr,
        strategy: SketchResidualStrategy,
        kind: ConstraintKind,
        weight: Real,
    ) {
        let residual_index = problem.constraints.len();
        let mut lowered = Constraint::equality(name.clone(), residual);
        lowered.kind = kind;
        lowered.active = constraint.active;
        lowered.weight = weight;
        problem.add_constraint(lowered);
        rows.push(SketchGeneratedRow {
            constraint: constraint.handle,
            residual_index: Some(residual_index),
            name,
            strategy: Some(strategy),
            status: SketchGeneratedRowStatus::Generated,
        });
    }

    fn entity(&self, handle: SketchEntityHandle) -> Option<&SketchEntity> {
        self.entities.get(handle.0 as usize)
    }

    fn parameter_expr(
        &self,
        handle: SketchParameterHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<Expr> {
        let Some(parameter) = self.parameters.get(handle.0 as usize) else {
            rows.push(SketchGeneratedRow {
                constraint: constraint.handle,
                residual_index: None,
                name: constraint.name.clone(),
                strategy: None,
                status: SketchGeneratedRowStatus::MissingParameter(handle),
            });
            return None;
        };
        Some(Expr::symbol(
            SymbolId(parameter.handle.0),
            parameter.name.clone(),
        ))
    }

    fn point_coordinates(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CoordinateExprs> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let exprs = match &entity.kind {
            SketchEntityKind::Point2D(point) => vec![
                self.parameter_expr(point.x, constraint, rows)?,
                self.parameter_expr(point.y, constraint, rows)?,
            ],
            SketchEntityKind::Point3D(point) => vec![
                self.parameter_expr(point.x, constraint, rows)?,
                self.parameter_expr(point.y, constraint, rows)?,
                self.parameter_expr(point.z, constraint, rows)?,
            ],
            _ => {
                rows.push(wrong_entity_row(constraint, handle, "point"));
                return None;
            }
        };
        Some(CoordinateExprs { handle, exprs })
    }

    fn distance_expr(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<Expr> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let SketchEntityKind::Distance(distance) = &entity.kind else {
            rows.push(wrong_entity_row(constraint, handle, "distance"));
            return None;
        };
        self.parameter_expr(distance.value, constraint, rows)
    }

    fn circle_or_arc_radius_expr(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<Expr> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let radius = match &entity.kind {
            SketchEntityKind::Circle2(circle) => circle.radius,
            SketchEntityKind::ArcOfCircle2(arc) => arc.radius,
            _ => {
                rows.push(wrong_entity_row(
                    constraint,
                    handle,
                    "circle or circular arc",
                ));
                return None;
            }
        };
        self.distance_expr(radius, constraint, rows)
    }

    fn point2_coordinates(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CoordinateExprs> {
        let point = self.point_coordinates(handle, constraint, rows)?;
        if point.len() != 2 {
            rows.push(wrong_entity_row(constraint, handle, "2D point"));
            return None;
        }
        Some(point)
    }

    fn point3_coordinates(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CoordinateExprs> {
        let point = self.point_coordinates(handle, constraint, rows)?;
        if point.len() != 3 {
            rows.push(wrong_entity_row(constraint, handle, "3D point"));
            return None;
        }
        Some(point)
    }

    fn normal3_quaternion(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<[Expr; 4]> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let SketchEntityKind::Normal3D(normal) = &entity.kind else {
            rows.push(wrong_entity_row(constraint, handle, "3D normal"));
            return None;
        };
        Some([
            self.parameter_expr(normal.w, constraint, rows)?,
            self.parameter_expr(normal.x, constraint, rows)?,
            self.parameter_expr(normal.y, constraint, rows)?,
            self.parameter_expr(normal.z, constraint, rows)?,
        ])
    }

    fn workplane_quaternion(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<[Expr; 4]> {
        self.workplane_origin_and_quaternion(handle, constraint, rows)
            .map(|(_, quaternion)| quaternion)
    }

    fn workplane_origin_and_quaternion(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<(CoordinateExprs, [Expr; 4])> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let SketchEntityKind::Workplane(workplane) = &entity.kind else {
            rows.push(wrong_entity_row(constraint, handle, "workplane"));
            return None;
        };
        // The origin cancels out of point-to-point projected distance, but a
        // retained workplane with a broken origin is not a valid source object.
        // Validate it here so projected-distance rows cannot outlive stale
        // workplane references.
        let origin = self.point3_coordinates(workplane.origin, constraint, rows)?;
        let quaternion = self.normal3_quaternion(workplane.normal, constraint, rows)?;
        Some((origin, quaternion))
    }

    fn projected_point_distance_exprs(
        &self,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        distance: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ProjectedPointDistanceExprs> {
        let quaternion = self.workplane_quaternion(workplane, constraint, rows)?;
        let a = self.point3_coordinates(a, constraint, rows)?;
        let b = self.point3_coordinates(b, constraint, rows)?;
        let distance = self.distance_expr(distance, constraint, rows)?;
        let delta = [
            a.exprs[0].clone() - b.exprs[0].clone(),
            a.exprs[1].clone() - b.exprs[1].clone(),
            a.exprs[2].clone() - b.exprs[2].clone(),
        ];
        Some(ProjectedPointDistanceExprs {
            projected_squared: projected_distance_squared(&delta, &quaternion),
            quaternion,
            distance,
        })
    }

    fn projected_point_range_exprs(
        &self,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ProjectedPointRangeExprs> {
        let quaternion = self.workplane_quaternion(workplane, constraint, rows)?;
        let a = self.point3_coordinates(a, constraint, rows)?;
        let b = self.point3_coordinates(b, constraint, rows)?;
        let delta = [
            a.exprs[0].clone() - b.exprs[0].clone(),
            a.exprs[1].clone() - b.exprs[1].clone(),
            a.exprs[2].clone() - b.exprs[2].clone(),
        ];
        Some(ProjectedPointRangeExprs {
            projected_squared: projected_distance_squared(&delta, &quaternion),
            quaternion,
        })
    }

    fn projected_point_line_distance_exprs(
        &self,
        workplane: SketchEntityHandle,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        distance: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ProjectedPointLineDistanceExprs> {
        let quaternion = self.workplane_quaternion(workplane, constraint, rows)?;
        let point = self.point3_coordinates(point, constraint, rows)?;
        let (start, end) = self.line3_points(line, constraint, rows)?;
        let distance = self.distance_expr(distance, constraint, rows)?;
        let point_delta = [
            point.exprs[0].clone() - start.exprs[0].clone(),
            point.exprs[1].clone() - start.exprs[1].clone(),
            point.exprs[2].clone() - start.exprs[2].clone(),
        ];
        let line_direction = [
            end.exprs[0].clone() - start.exprs[0].clone(),
            end.exprs[1].clone() - start.exprs[1].clone(),
            end.exprs[2].clone() - start.exprs[2].clone(),
        ];
        let (signed_cross, distance_denominator) =
            projected_point_line_distance_squared_parts(&point_delta, &line_direction, &quaternion);
        Some(ProjectedPointLineDistanceExprs {
            quaternion,
            distance_numerator: signed_cross.clone() * signed_cross.clone(),
            distance_denominator,
            signed_cross,
            distance,
        })
    }

    fn projected_point_line_distance_squared_parts(
        &self,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        quaternion: &[Expr; 4],
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<(Expr, Expr)> {
        let point = self.point3_coordinates(point, constraint, rows)?;
        let (start, end) = self.line3_points(line, constraint, rows)?;
        let point_delta = [
            point.exprs[0].clone() - start.exprs[0].clone(),
            point.exprs[1].clone() - start.exprs[1].clone(),
            point.exprs[2].clone() - start.exprs[2].clone(),
        ];
        let line_direction = [
            end.exprs[0].clone() - start.exprs[0].clone(),
            end.exprs[1].clone() - start.exprs[1].clone(),
            end.exprs[2].clone() - start.exprs[2].clone(),
        ];
        let (signed_cross, denominator) =
            projected_point_line_distance_squared_parts(&point_delta, &line_direction, quaternion);
        Some((signed_cross.clone() * signed_cross, denominator))
    }

    fn projected_equal_point_line_distances_exprs(
        &self,
        workplane: SketchEntityHandle,
        pairs: [(SketchEntityHandle, SketchEntityHandle); 2],
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ProjectedEqualPointLineDistancesExprs> {
        let quaternion = self.workplane_quaternion(workplane, constraint, rows)?;
        let (a_numerator, a_denominator) = self.projected_point_line_distance_squared_parts(
            pairs[0].0,
            pairs[0].1,
            &quaternion,
            constraint,
            rows,
        )?;
        let (b_numerator, b_denominator) = self.projected_point_line_distance_squared_parts(
            pairs[1].0,
            pairs[1].1,
            &quaternion,
            constraint,
            rows,
        )?;
        Some(ProjectedEqualPointLineDistancesExprs {
            quaternion,
            a_numerator,
            a_denominator,
            b_numerator,
            b_denominator,
        })
    }

    fn projected_line_length_equality_exprs(
        &self,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ProjectedLineLengthEqualityExprs> {
        let quaternion = self.workplane_quaternion(workplane, constraint, rows)?;
        let first_direction = self.line3_direction(a, constraint, rows)?;
        let second_direction = self.line3_direction(b, constraint, rows)?;
        Some(ProjectedLineLengthEqualityExprs {
            first_squared: projected_direction_squared_length(&first_direction, &quaternion),
            second_squared: projected_direction_squared_length(&second_direction, &quaternion),
            quaternion,
        })
    }

    fn projected_line_length_difference_exprs(
        &self,
        workplane: SketchEntityHandle,
        longer: SketchEntityHandle,
        shorter: SketchEntityHandle,
        difference: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<(ProjectedLineLengthEqualityExprs, Expr)> {
        Some((
            self.projected_line_length_equality_exprs(
                workplane, longer, shorter, constraint, rows,
            )?,
            self.distance_expr(difference, constraint, rows)?,
        ))
    }

    fn projected_equal_length_point_line_distance_exprs(
        &self,
        workplane: SketchEntityHandle,
        length_line: SketchEntityHandle,
        point: SketchEntityHandle,
        distance_line: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ProjectedLineLengthPointLineDistanceExprs> {
        let quaternion = self.workplane_quaternion(workplane, constraint, rows)?;
        let length_direction = self.line3_direction(length_line, constraint, rows)?;
        let point = self.point3_coordinates(point, constraint, rows)?;
        let (start, end) = self.line3_points(distance_line, constraint, rows)?;
        let point_delta = [
            point.exprs[0].clone() - start.exprs[0].clone(),
            point.exprs[1].clone() - start.exprs[1].clone(),
            point.exprs[2].clone() - start.exprs[2].clone(),
        ];
        let distance_direction = [
            end.exprs[0].clone() - start.exprs[0].clone(),
            end.exprs[1].clone() - start.exprs[1].clone(),
            end.exprs[2].clone() - start.exprs[2].clone(),
        ];
        let (signed_cross, distance_denominator) = projected_point_line_distance_squared_parts(
            &point_delta,
            &distance_direction,
            &quaternion,
        );
        Some(ProjectedLineLengthPointLineDistanceExprs {
            length_squared: projected_direction_squared_length(&length_direction, &quaternion),
            distance_numerator: signed_cross.clone() * signed_cross,
            distance_denominator,
            quaternion,
        })
    }

    fn workplane_symmetry_exprs(
        &self,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<WorkplaneSymmetryExprs> {
        let (origin, quaternion) =
            self.workplane_origin_and_quaternion(workplane, constraint, rows)?;
        let a = self.point3_coordinates(a, constraint, rows)?;
        let b = self.point3_coordinates(b, constraint, rows)?;
        Some(workplane_point_symmetry_exprs(
            &coordinate_exprs3(&origin),
            &quaternion,
            &coordinate_exprs3(&a),
            &coordinate_exprs3(&b),
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn arc_line_tangent_exprs(
        &self,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
        orientation: SketchTangentOrientation,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ArcLineTangentExprs> {
        let arc = self.arc2_endpoint_parts(arc, arc_endpoint, constraint, rows)?;
        let line = self.line2_endpoint_tangent(line, line_endpoint, constraint, rows)?;
        Some(arc_line_tangent_exprs(
            &coordinate_exprs2(&arc.center),
            &coordinate_exprs2(&arc.endpoint),
            arc.radius,
            &coordinate_exprs2(&line.endpoint),
            &line.tangent,
            tangent_orientation_sign(orientation),
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn arc_cubic_tangent_exprs(
        &self,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ArcCubicTangentExprs> {
        let arc = self.arc2_endpoint_parts(arc, arc_endpoint, constraint, rows)?;
        let cubic = self.cubic2_point_tangent_exprs(cubic, parameter, constraint, rows)?;
        Some(arc_cubic_tangent_exprs(
            &coordinate_exprs2(&arc.center),
            &coordinate_exprs2(&arc.endpoint),
            arc.radius,
            &cubic.point,
            &cubic.tangent,
            tangent_orientation_sign(orientation),
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn arc_arc_tangent_exprs(
        &self,
        first: SketchEntityHandle,
        first_endpoint: SketchArcEndpoint,
        second: SketchEntityHandle,
        second_endpoint: SketchArcEndpoint,
        branch: SketchArcTangencyBranch,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ArcArcTangentExprs> {
        let first = self.arc2_endpoint_parts(first, first_endpoint, constraint, rows)?;
        let second = self.arc2_endpoint_parts(second, second_endpoint, constraint, rows)?;
        Some(arc_arc_tangent_exprs(
            &coordinate_exprs2(&first.center),
            &coordinate_exprs2(&first.endpoint),
            first.radius,
            &coordinate_exprs2(&second.center),
            &coordinate_exprs2(&second.endpoint),
            second.radius,
            arc_tangency_branch_sign(branch),
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn arc_cubic_second_order_contact_exprs(
        &self,
        arc: SketchEntityHandle,
        arc_endpoint: SketchArcEndpoint,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        orientation: SketchTangentOrientation,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ArcCubicSecondOrderContactExprs> {
        let arc = self.arc2_endpoint_parts(arc, arc_endpoint, constraint, rows)?;
        let cubic = self.cubic2_point_tangent_exprs(cubic, parameter, constraint, rows)?;
        Some(arc_cubic_second_order_contact_exprs(
            &coordinate_exprs2(&arc.center),
            &coordinate_exprs2(&arc.endpoint),
            arc.radius,
            &cubic.point,
            &cubic.tangent,
            &cubic.second_derivative,
            tangent_orientation_sign(orientation),
        ))
    }

    fn oriented_angle_exprs(
        &self,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<OrientedAngleExprs> {
        let a = self.line2_direction(a, constraint, rows)?;
        let b = self.line2_direction(b, constraint, rows)?;
        let c = self.line2_direction(c, constraint, rows)?;
        let d = self.line2_direction(d, constraint, rows)?;
        Some(oriented_angle_exprs(&a, &b, &c, &d))
    }

    #[allow(clippy::too_many_arguments)]
    fn projected_oriented_angle_exprs(
        &self,
        workplane: SketchEntityHandle,
        a: SketchEntityHandle,
        b: SketchEntityHandle,
        c: SketchEntityHandle,
        d: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ProjectedOrientedAngleExprs> {
        let quaternion = self.workplane_quaternion(workplane, constraint, rows)?;
        let a = projected_direction2(&self.line3_direction(a, constraint, rows)?, &quaternion);
        let b = projected_direction2(&self.line3_direction(b, constraint, rows)?, &quaternion);
        let c = projected_direction2(&self.line3_direction(c, constraint, rows)?, &quaternion);
        let d = projected_direction2(&self.line3_direction(d, constraint, rows)?, &quaternion);
        Some(ProjectedOrientedAngleExprs {
            angle: oriented_angle_exprs(&a, &b, &c, &d),
            quaternion,
        })
    }

    fn line2_points(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<(CoordinateExprs, CoordinateExprs)> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let SketchEntityKind::LineSegment2(line) = &entity.kind else {
            rows.push(wrong_entity_row(constraint, handle, "2D line segment"));
            return None;
        };
        let start = self.point_coordinates(line.start, constraint, rows)?;
        let end = self.point_coordinates(line.end, constraint, rows)?;
        if start.len() != 2 || end.len() != 2 {
            rows.push(wrong_entity_row(constraint, handle, "2D line segment"));
            return None;
        }
        Some((start, end))
    }

    fn line3_points(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<(CoordinateExprs, CoordinateExprs)> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let SketchEntityKind::LineSegment3(line) = &entity.kind else {
            rows.push(wrong_entity_row(constraint, handle, "3D line segment"));
            return None;
        };
        let start = self.point_coordinates(line.start, constraint, rows)?;
        let end = self.point_coordinates(line.end, constraint, rows)?;
        if start.len() != 3 || end.len() != 3 {
            rows.push(wrong_entity_row(constraint, handle, "3D line segment"));
            return None;
        }
        Some((start, end))
    }

    fn line3_direction(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<[Expr; 3]> {
        let (start, end) = self.line3_points(handle, constraint, rows)?;
        Some([
            end.exprs[0].clone() - start.exprs[0].clone(),
            end.exprs[1].clone() - start.exprs[1].clone(),
            end.exprs[2].clone() - start.exprs[2].clone(),
        ])
    }

    fn line2_endpoint_tangent(
        &self,
        handle: SketchEntityHandle,
        endpoint: SketchLineEndpoint,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<LineEndpointTangentExprs> {
        let (start, end) = self.line2_points(handle, constraint, rows)?;
        let tangent = match endpoint {
            SketchLineEndpoint::Start => [
                end.exprs[0].clone() - start.exprs[0].clone(),
                end.exprs[1].clone() - start.exprs[1].clone(),
            ],
            SketchLineEndpoint::End => [
                start.exprs[0].clone() - end.exprs[0].clone(),
                start.exprs[1].clone() - end.exprs[1].clone(),
            ],
        };
        let endpoint = match endpoint {
            SketchLineEndpoint::Start => start,
            SketchLineEndpoint::End => end,
        };
        Some(LineEndpointTangentExprs { endpoint, tangent })
    }

    fn line2_direction(
        &self,
        handle: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<[Expr; 2]> {
        let (start, end) = self.line2_points(handle, constraint, rows)?;
        Some([
            end.exprs[0].clone() - start.exprs[0].clone(),
            end.exprs[1].clone() - start.exprs[1].clone(),
        ])
    }

    fn point_on_cubic2_exprs(
        &self,
        point: SketchEntityHandle,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<[Expr; 2]> {
        let point = self.point2_coordinates(point, constraint, rows)?;
        let Some(entity) = self.entity(cubic) else {
            rows.push(missing_entity_row(constraint, cubic));
            return None;
        };
        let SketchEntityKind::Cubic2(cubic) = &entity.kind else {
            rows.push(wrong_entity_row(
                constraint,
                entity.handle,
                "2D cubic Bezier",
            ));
            return None;
        };
        let p0 = self.point2_coordinates(cubic.p0, constraint, rows)?;
        let p1 = self.point2_coordinates(cubic.p1, constraint, rows)?;
        let p2 = self.point2_coordinates(cubic.p2, constraint, rows)?;
        let p3 = self.point2_coordinates(cubic.p3, constraint, rows)?;
        let parameter = self.parameter_expr(parameter, constraint, rows)?;
        let curve = cubic_bezier_point2_exprs(
            &coordinate_exprs2(&p0),
            &coordinate_exprs2(&p1),
            &coordinate_exprs2(&p2),
            &coordinate_exprs2(&p3),
            parameter,
        );
        Some([
            point.exprs[0].clone() - curve[0].clone(),
            point.exprs[1].clone() - curve[1].clone(),
        ])
    }

    fn cubic_line_tangent_exprs(
        &self,
        cubic: SketchEntityHandle,
        parameter: SketchParameterHandle,
        line: SketchEntityHandle,
        line_endpoint: SketchLineEndpoint,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CubicLineTangentExprs> {
        let cubic = self.cubic2_point_tangent_exprs(cubic, parameter, constraint, rows)?;
        let line = self.line2_endpoint_tangent(line, line_endpoint, constraint, rows)?;
        Some(CubicLineTangentExprs {
            endpoint_incidence: [
                line.endpoint.exprs[0].clone() - cubic.point[0].clone(),
                line.endpoint.exprs[1].clone() - cubic.point[1].clone(),
            ],
            tangent_cross: direction_cross2(&cubic.tangent, &line.tangent),
            tangent_dot: direction_dot2(&cubic.tangent, &line.tangent),
        })
    }

    fn cubic_cubic_tangent_exprs(
        &self,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CubicCubicTangentExprs> {
        let first = self.cubic2_point_tangent_exprs(first, first_parameter, constraint, rows)?;
        let second = self.cubic2_point_tangent_exprs(second, second_parameter, constraint, rows)?;
        Some(CubicCubicTangentExprs {
            point_incidence: [
                first.point[0].clone() - second.point[0].clone(),
                first.point[1].clone() - second.point[1].clone(),
            ],
            tangent_cross: direction_cross2(&first.tangent, &second.tangent),
            tangent_dot: direction_dot2(&first.tangent, &second.tangent),
        })
    }

    fn cubic_cubic_c2_exprs(
        &self,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CubicCubicC2Exprs> {
        let first = self.cubic2_point_tangent_exprs(first, first_parameter, constraint, rows)?;
        let second = self.cubic2_point_tangent_exprs(second, second_parameter, constraint, rows)?;
        Some(CubicCubicC2Exprs {
            point: [
                first.point[0].clone() - second.point[0].clone(),
                first.point[1].clone() - second.point[1].clone(),
            ],
            first_derivative: [
                first.tangent[0].clone() - second.tangent[0].clone(),
                first.tangent[1].clone() - second.tangent[1].clone(),
            ],
            second_derivative: [
                first.second_derivative[0].clone() - second.second_derivative[0].clone(),
                first.second_derivative[1].clone() - second.second_derivative[1].clone(),
            ],
        })
    }

    fn cubic_cubic_g2_exprs(
        &self,
        first: SketchEntityHandle,
        first_parameter: SketchParameterHandle,
        second: SketchEntityHandle,
        second_parameter: SketchParameterHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CubicCubicG2Exprs> {
        let first = self.cubic2_point_tangent_exprs(first, first_parameter, constraint, rows)?;
        let second = self.cubic2_point_tangent_exprs(second, second_parameter, constraint, rows)?;
        let first_curvature = direction_cross2(&first.tangent, &first.second_derivative);
        let second_curvature = direction_cross2(&second.tangent, &second.second_derivative);
        let first_speed_squared = direction_dot2(&first.tangent, &first.tangent);
        let second_speed_squared = direction_dot2(&second.tangent, &second.tangent);
        let first_speed_sixth = expr_cube(first_speed_squared.clone());
        let second_speed_sixth = expr_cube(second_speed_squared.clone());
        Some(CubicCubicG2Exprs {
            point_incidence: [
                first.point[0].clone() - second.point[0].clone(),
                first.point[1].clone() - second.point[1].clone(),
            ],
            tangent_cross: direction_cross2(&first.tangent, &second.tangent),
            tangent_dot: direction_dot2(&first.tangent, &second.tangent),
            curvature_magnitude: first_curvature.clone()
                * first_curvature.clone()
                * second_speed_sixth
                - second_curvature.clone() * second_curvature.clone() * first_speed_sixth,
            curvature_sign: first_curvature * second_curvature,
        })
    }

    fn cubic2_point_tangent_exprs(
        &self,
        handle: SketchEntityHandle,
        parameter: SketchParameterHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<CubicPointTangentExprs> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let SketchEntityKind::Cubic2(cubic) = &entity.kind else {
            rows.push(wrong_entity_row(
                constraint,
                entity.handle,
                "2D cubic Bezier",
            ));
            return None;
        };
        let p0 = self.point2_coordinates(cubic.p0, constraint, rows)?;
        let p1 = self.point2_coordinates(cubic.p1, constraint, rows)?;
        let p2 = self.point2_coordinates(cubic.p2, constraint, rows)?;
        let p3 = self.point2_coordinates(cubic.p3, constraint, rows)?;
        let parameter = self.parameter_expr(parameter, constraint, rows)?;
        Some(cubic_point_tangent_exprs(
            &coordinate_exprs2(&p0),
            &coordinate_exprs2(&p1),
            &coordinate_exprs2(&p2),
            &coordinate_exprs2(&p3),
            parameter,
        ))
    }

    fn arc2_endpoint_parts(
        &self,
        handle: SketchEntityHandle,
        endpoint: SketchArcEndpoint,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<ArcEndpointExprs> {
        let Some(entity) = self.entity(handle) else {
            rows.push(missing_entity_row(constraint, handle));
            return None;
        };
        let SketchEntityKind::ArcOfCircle2(arc) = &entity.kind else {
            rows.push(wrong_entity_row(constraint, handle, "2D circular arc"));
            return None;
        };
        let center = self.point2_coordinates(arc.center, constraint, rows)?;
        let endpoint = match endpoint {
            SketchArcEndpoint::Start => self.point2_coordinates(arc.start, constraint, rows)?,
            SketchArcEndpoint::End => self.point2_coordinates(arc.end, constraint, rows)?,
        };
        let radius = self.distance_expr(arc.radius, constraint, rows)?;
        Some(ArcEndpointExprs {
            center,
            endpoint,
            radius,
        })
    }

    fn point_line_distance_squared_parts(
        &self,
        point: SketchEntityHandle,
        line: SketchEntityHandle,
        constraint: &SketchConstraint,
        rows: &mut Vec<SketchGeneratedRow>,
    ) -> Option<(Expr, Expr)> {
        let point = self.point2_coordinates(point, constraint, rows)?;
        let (start, end) = self.line2_points(line, constraint, rows)?;
        let direction = [
            end.exprs[0].clone() - start.exprs[0].clone(),
            end.exprs[1].clone() - start.exprs[1].clone(),
        ];
        let point_delta = [
            point.exprs[0].clone() - start.exprs[0].clone(),
            point.exprs[1].clone() - start.exprs[1].clone(),
        ];
        let cross = direction_cross2(&point_delta, &direction);
        Some((cross.clone() * cross, squared_norm2(&direction)))
    }
}

#[derive(Clone, Debug)]
struct CoordinateExprs {
    handle: SketchEntityHandle,
    exprs: Vec<Expr>,
}

impl CoordinateExprs {
    fn len(&self) -> usize {
        self.exprs.len()
    }
}

#[derive(Clone, Debug)]
struct ArcEndpointExprs {
    center: CoordinateExprs,
    endpoint: CoordinateExprs,
    radius: Expr,
}

#[derive(Clone, Debug)]
struct LineEndpointTangentExprs {
    endpoint: CoordinateExprs,
    tangent: [Expr; 2],
}

#[derive(Clone, Debug)]
struct CubicLineTangentExprs {
    endpoint_incidence: [Expr; 2],
    tangent_cross: Expr,
    tangent_dot: Expr,
}

#[derive(Clone, Debug)]
struct CubicCubicTangentExprs {
    point_incidence: [Expr; 2],
    tangent_cross: Expr,
    tangent_dot: Expr,
}

#[derive(Clone, Debug)]
struct CubicCubicG2Exprs {
    point_incidence: [Expr; 2],
    tangent_cross: Expr,
    tangent_dot: Expr,
    curvature_magnitude: Expr,
    curvature_sign: Expr,
}

#[derive(Clone, Debug)]
struct CubicCubicC2Exprs {
    point: [Expr; 2],
    first_derivative: [Expr; 2],
    second_derivative: [Expr; 2],
}

fn coordinate_exprs2(coordinates: &CoordinateExprs) -> [Expr; 2] {
    debug_assert_eq!(coordinates.len(), 2);
    [coordinates.exprs[0].clone(), coordinates.exprs[1].clone()]
}

fn coordinate_exprs3(coordinates: &CoordinateExprs) -> [Expr; 3] {
    debug_assert_eq!(coordinates.len(), 3);
    [
        coordinates.exprs[0].clone(),
        coordinates.exprs[1].clone(),
        coordinates.exprs[2].clone(),
    ]
}

fn expr_cube(value: Expr) -> Expr {
    value.clone() * value.clone() * value
}

fn cubic_bezier_point2_exprs(
    p0: &[Expr; 2],
    p1: &[Expr; 2],
    p2: &[Expr; 2],
    p3: &[Expr; 2],
    parameter: Expr,
) -> [Expr; 2] {
    let one_minus_t = Expr::int(1) - parameter.clone();
    let b0 = one_minus_t.clone() * one_minus_t.clone() * one_minus_t.clone();
    let b1 = Expr::int(3) * one_minus_t.clone() * one_minus_t * parameter.clone();
    let b2 =
        Expr::int(3) * (Expr::int(1) - parameter.clone()) * parameter.clone() * parameter.clone();
    let b3 = parameter.clone() * parameter.clone() * parameter;
    [
        p0[0].clone() * b0.clone()
            + p1[0].clone() * b1.clone()
            + p2[0].clone() * b2.clone()
            + p3[0].clone() * b3.clone(),
        p0[1].clone() * b0 + p1[1].clone() * b1 + p2[1].clone() * b2 + p3[1].clone() * b3,
    ]
}

fn tangent_orientation_sign(orientation: SketchTangentOrientation) -> i8 {
    match orientation {
        SketchTangentOrientation::CounterClockwise => 1,
        SketchTangentOrientation::Clockwise => -1,
    }
}

fn arc_tangency_branch_sign(branch: SketchArcTangencyBranch) -> i8 {
    match branch {
        SketchArcTangencyBranch::SameRadiusDirection => 1,
        SketchArcTangencyBranch::OppositeRadiusDirection => -1,
    }
}

#[derive(Clone, Debug)]
struct ProjectedPointDistanceExprs {
    quaternion: [Expr; 4],
    projected_squared: Expr,
    distance: Expr,
}

#[derive(Clone, Debug)]
struct ProjectedPointRangeExprs {
    quaternion: [Expr; 4],
    projected_squared: Expr,
}

#[derive(Clone, Debug)]
struct ProjectedPointLineDistanceExprs {
    quaternion: [Expr; 4],
    distance_numerator: Expr,
    distance_denominator: Expr,
    signed_cross: Expr,
    distance: Expr,
}

#[derive(Clone, Debug)]
struct ProjectedEqualPointLineDistancesExprs {
    quaternion: [Expr; 4],
    a_numerator: Expr,
    a_denominator: Expr,
    b_numerator: Expr,
    b_denominator: Expr,
}

#[derive(Clone, Debug)]
struct ProjectedLineLengthEqualityExprs {
    quaternion: [Expr; 4],
    first_squared: Expr,
    second_squared: Expr,
}

#[derive(Clone, Debug)]
struct ProjectedLineLengthPointLineDistanceExprs {
    quaternion: [Expr; 4],
    length_squared: Expr,
    distance_numerator: Expr,
    distance_denominator: Expr,
}

#[derive(Clone, Debug)]
struct ProjectedOrientedAngleExprs {
    quaternion: [Expr; 4],
    angle: OrientedAngleExprs,
}

fn missing_entity_row(
    constraint: &SketchConstraint,
    handle: SketchEntityHandle,
) -> SketchGeneratedRow {
    SketchGeneratedRow {
        constraint: constraint.handle,
        residual_index: None,
        name: constraint.name.clone(),
        strategy: None,
        status: SketchGeneratedRowStatus::MissingEntity(handle),
    }
}

fn wrong_entity_row(
    constraint: &SketchConstraint,
    handle: SketchEntityHandle,
    expected: &'static str,
) -> SketchGeneratedRow {
    SketchGeneratedRow {
        constraint: constraint.handle,
        residual_index: None,
        name: constraint.name.clone(),
        strategy: None,
        status: SketchGeneratedRowStatus::WrongEntityKind { handle, expected },
    }
}

fn invalid_bound_row(
    constraint: &SketchConstraint,
    name: impl Into<String>,
    strategy: SketchResidualStrategy,
    status: SketchGeneratedRowStatus,
) -> SketchGeneratedRow {
    SketchGeneratedRow {
        constraint: constraint.handle,
        residual_index: None,
        name: name.into(),
        strategy: Some(strategy),
        status,
    }
}

fn validate_distance_bounds(
    constraint: &SketchConstraint,
    lower: Option<&Real>,
    upper: Option<&Real>,
    rows: &mut Vec<SketchGeneratedRow>,
) -> bool {
    validate_distance_bounds_with_strategy(
        constraint,
        lower,
        upper,
        SketchResidualStrategy::BoundedSquaredDistance,
        rows,
    )
}

fn validate_distance_bounds_with_strategy(
    constraint: &SketchConstraint,
    lower: Option<&Real>,
    upper: Option<&Real>,
    strategy: SketchResidualStrategy,
    rows: &mut Vec<SketchGeneratedRow>,
) -> bool {
    for (label, bound) in [("lower", lower), ("upper", upper)] {
        if let Some(bound) = bound {
            match bound.structural_facts().sign {
                Some(RealSign::Negative) => {
                    rows.push(invalid_bound_row(
                        constraint,
                        format!("{} {label} distance", constraint.name),
                        strategy,
                        SketchGeneratedRowStatus::InvalidExactBound,
                    ));
                    return false;
                }
                Some(RealSign::Zero | RealSign::Positive) => {}
                None => {
                    rows.push(invalid_bound_row(
                        constraint,
                        format!("{} {label} distance", constraint.name),
                        strategy,
                        SketchGeneratedRowStatus::UnresolvedExactBound,
                    ));
                    return false;
                }
            }
        }
    }
    if let (Some(lower), Some(upper)) = (lower, upper) {
        match (upper.clone() - lower.clone()).structural_facts().sign {
            Some(RealSign::Negative) => {
                rows.push(invalid_bound_row(
                    constraint,
                    constraint.name.clone(),
                    strategy,
                    SketchGeneratedRowStatus::InvalidExactBound,
                ));
                return false;
            }
            Some(RealSign::Zero | RealSign::Positive) => {}
            None => {
                rows.push(invalid_bound_row(
                    constraint,
                    constraint.name.clone(),
                    strategy,
                    SketchGeneratedRowStatus::UnresolvedExactBound,
                ));
                return false;
            }
        }
    }
    true
}

fn invalid_parameter_margin_row(
    constraint: &SketchConstraint,
    status: SketchGeneratedRowStatus,
) -> SketchGeneratedRow {
    SketchGeneratedRow {
        constraint: constraint.handle,
        residual_index: None,
        name: constraint.name.clone(),
        strategy: Some(SketchResidualStrategy::ParameterMargin),
        status,
    }
}

fn validate_parameter_margin(
    constraint: &SketchConstraint,
    margin: &Real,
    rows: &mut Vec<SketchGeneratedRow>,
) -> bool {
    match margin.structural_facts().sign {
        Some(RealSign::Negative) => {
            rows.push(invalid_parameter_margin_row(
                constraint,
                SketchGeneratedRowStatus::InvalidExactBound,
            ));
            false
        }
        Some(RealSign::Zero | RealSign::Positive) => true,
        None => {
            rows.push(invalid_parameter_margin_row(
                constraint,
                SketchGeneratedRowStatus::UnresolvedExactBound,
            ));
            false
        }
    }
}

fn invalid_length_ratio_row(
    constraint: &SketchConstraint,
    strategy: SketchResidualStrategy,
    status: SketchGeneratedRowStatus,
) -> SketchGeneratedRow {
    SketchGeneratedRow {
        constraint: constraint.handle,
        residual_index: None,
        name: constraint.name.clone(),
        strategy: Some(strategy),
        status,
    }
}

fn validate_length_ratio(
    constraint: &SketchConstraint,
    numerator: &Real,
    denominator: &Real,
    strategy: SketchResidualStrategy,
    rows: &mut Vec<SketchGeneratedRow>,
) -> bool {
    let numerator_ok = match numerator.structural_facts().sign {
        Some(RealSign::Zero | RealSign::Positive) => true,
        Some(RealSign::Negative) => {
            rows.push(invalid_length_ratio_row(
                constraint,
                strategy,
                SketchGeneratedRowStatus::InvalidExactBound,
            ));
            false
        }
        None => {
            rows.push(invalid_length_ratio_row(
                constraint,
                strategy,
                SketchGeneratedRowStatus::UnresolvedExactBound,
            ));
            false
        }
    };
    if !numerator_ok {
        return false;
    }
    match denominator.structural_facts().sign {
        Some(RealSign::Positive) => true,
        Some(RealSign::Negative | RealSign::Zero) => {
            rows.push(invalid_length_ratio_row(
                constraint,
                strategy,
                SketchGeneratedRowStatus::InvalidExactBound,
            ));
            false
        }
        None => {
            rows.push(invalid_length_ratio_row(
                constraint,
                strategy,
                SketchGeneratedRowStatus::UnresolvedExactBound,
            ));
            false
        }
    }
}

fn squared_distance(a: &[Expr], b: &[Expr]) -> Expr {
    a.iter()
        .zip(b)
        .fold(Expr::zero(), |sum, (a_coord, b_coord)| {
            let delta = a_coord.clone() - b_coord.clone();
            sum + delta.clone() * delta
        })
}

fn direction_cross2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[1].clone() - a[1].clone() * b[0].clone()
}

fn direction_dot2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone()
}

fn squared_norm2(direction: &[Expr; 2]) -> Expr {
    direction[0].clone() * direction[0].clone() + direction[1].clone() * direction[1].clone()
}

fn squared_cosine_angle_residual(
    a: &[Expr; 2],
    b: &[Expr; 2],
    c: &[Expr; 2],
    d: &[Expr; 2],
) -> Expr {
    let first_dot = direction_dot2(a, b);
    let second_dot = direction_dot2(c, d);
    first_dot.clone() * first_dot * squared_norm2(c) * squared_norm2(d)
        - second_dot.clone() * second_dot * squared_norm2(a) * squared_norm2(b)
}

fn angle_cosine_expr(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    direction_dot2(a, b) / (squared_norm2(a).sqrt() * squared_norm2(b).sqrt())
}
