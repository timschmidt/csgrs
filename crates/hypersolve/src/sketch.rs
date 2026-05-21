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
use crate::sketch_builders::{distance, incidence, objective, orientation, ranges};
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
    /// SolveSpace-style `CUBIC`.
    Cubic2(SketchCubic2),
    /// SolveSpace-style `CIRCLE`.
    Circle2(SketchCircle2),
    /// SolveSpace-style `ARC_OF_CIRCLE`.
    ArcOfCircle2(SketchArcOfCircle2),
    /// SolveSpace-style `WORKPLANE`.
    Workplane(SketchWorkplane),
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
    /// Point-on-circle incidence using squared radius residual.
    PointOnCircle {
        /// Point entity.
        point: SketchEntityHandle,
        /// Circle entity.
        circle: SketchEntityHandle,
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
    /// Incidence represented as a squared-distance polynomial.
    SquaredIncidence,
    /// Scalar parameter bound.
    ParameterRange,
    /// Scalar nondecreasing parameter relation.
    ParameterOrdering,
    /// Scalar ordered relation with an exact nonnegative margin.
    ParameterMargin,
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
            metadata: SketchRoundTripMetadata::default(),
        });
        handle
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

    /// Add a point-on-circle incidence constraint.
    pub fn add_point_on_circle(
        &mut self,
        name: impl Into<String>,
        point: SketchEntityHandle,
        circle: SketchEntityHandle,
    ) -> SketchConstraintHandle {
        incidence::point_on_circle(self, name, point, circle).handle
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
    /// parity. Yap (1997) is the controlling rule here: proposal-compatible
    /// forms are retained as data, but only exact replay/certification turns a
    /// residual into evidence.
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
    status: SketchGeneratedRowStatus,
) -> SketchGeneratedRow {
    SketchGeneratedRow {
        constraint: constraint.handle,
        residual_index: None,
        name: name.into(),
        strategy: Some(SketchResidualStrategy::BoundedSquaredDistance),
        status,
    }
}

fn validate_distance_bounds(
    constraint: &SketchConstraint,
    lower: Option<&Real>,
    upper: Option<&Real>,
    rows: &mut Vec<SketchGeneratedRow>,
) -> bool {
    for (label, bound) in [("lower", lower), ("upper", upper)] {
        if let Some(bound) = bound {
            match bound.structural_facts().sign {
                Some(RealSign::Negative) => {
                    rows.push(invalid_bound_row(
                        constraint,
                        format!("{} {label} distance", constraint.name),
                        SketchGeneratedRowStatus::InvalidExactBound,
                    ));
                    return false;
                }
                Some(RealSign::Zero | RealSign::Positive) => {}
                None => {
                    rows.push(invalid_bound_row(
                        constraint,
                        format!("{} {label} distance", constraint.name),
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
                    SketchGeneratedRowStatus::InvalidExactBound,
                ));
                return false;
            }
            Some(RealSign::Zero | RealSign::Positive) => {}
            None => {
                rows.push(invalid_bound_row(
                    constraint,
                    constraint.name.clone(),
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

fn squared_distance(a: &[Expr], b: &[Expr]) -> Expr {
    a.iter()
        .zip(b)
        .fold(Expr::zero(), |sum, (a_coord, b_coord)| {
            let delta = a_coord.clone() - b_coord.clone();
            sum + delta.clone() * delta
        })
}
