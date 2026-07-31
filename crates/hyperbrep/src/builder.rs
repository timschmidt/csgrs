//! Conventional exact solid constructors.

use std::collections::HashMap;
use std::fmt;

use hypercurve::{
    Aabb2, BezierSubcurve2, CircularArc2, Classification, Contour2, ContourPointLocation, Curve2,
    CurveGeometry2, CurvePath2, CurvePolicy, LineSeg2, Point2 as CurvePoint2, RationalBezier2,
    Segment2,
};
use hyperlattice::{Point2, Point3, Real, Vector2, Vector3};
use hyperlimit::{PredicateOutcome, compare_reals, point3_equal};

use crate::geometry::Curve3ExactData;
use crate::model::CertifiedSpherePairKind;
use crate::{
    BuildError, Curve3, Direction, EdgeId, FaceId, GeometryError, Model, ModelBuilder, Orientation,
    ParameterCorrespondence, ParameterDomain, Pcurve, SolidId, Surface, ValidationReport, VertexId,
};

/// Coordinate axis involved in a construction error.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Axis {
    /// Model-space x axis.
    X,
    /// Model-space y axis.
    Y,
    /// Model-space z axis.
    Z,
}

/// Failure while constructing a conventional solid.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ConstructionError {
    /// A lower bound was not certified strictly below its upper bound.
    InvalidBounds(Axis),
    /// A planar extrusion profile has fewer than three vertices.
    ProfileTooSmall,
    /// Cone-frustum radii are not strictly positive with base above top.
    InvalidFrustumRadii,
    /// A planar extrusion profile has zero certified signed area.
    DegenerateProfile,
    /// A radial/axial revolution profile reaches or crosses its axis.
    ProfileCrossesRevolutionAxis,
    /// A curved revolution profile lacks a required exact topology or integral certificate.
    UnsupportedRevolutionProfile,
    /// A planar path lacks a required exact simplicity, intersection, or persistence certificate.
    UnsupportedPlanarProfile,
    /// One finite revolution patch must span strictly less than a full period.
    RevolutionPatchSweepTooLarge,
    /// Loft construction requires at least two ordered sections.
    LoftNeedsAtLeastTwoSections,
    /// Loft sections do not have a certifiable positive correspondence.
    IncompatibleLoftSections,
    /// A curve sweep currently requires a rational Bézier path.
    UnsupportedSweepPath,
    /// A curve sweep path is not an exact positive affine graph through the
    /// initial profile plane.
    NonMonotoneSweepPath,
    /// A rational Bézier moving frame has inconsistent control counts or
    /// invalid projective data.
    InvalidSweepFrame,
    /// A moving frame leaves the parallel section-plane family established
    /// by its initial axes.
    NonPlanarSweepFrame,
    /// A polynomial moving frame reaches or crosses zero oriented section
    /// area somewhere in its complete Bernstein certificate.
    NonPositiveSweepFrameArea,
    /// A nonconstant rational section-area law has no active exact integration
    /// certificate.
    UnsupportedRationalSweepFrameArea,
    /// A tensor patch shell requires at least one face.
    EmptyPatchShell,
    /// A planar extrusion profile crosses or overlaps itself.
    SelfIntersectingProfile,
    /// A hole is not strictly inside the outer profile.
    HoleOutside,
    /// Two region profile loops cross or overlap.
    IntersectingProfiles,
    /// Two holes are nested rather than disjoint.
    NestedHoles,
    /// Incremental geometry or topology construction failed.
    Build(BuildError),
    /// Whole-model validation rejected the completed construction.
    Validation(ValidationReport),
}

/// One closed prismatic cavity used by [`extrude_with_voids`].
#[derive(Clone, Debug)]
pub struct ExtrusionVoid {
    /// Counterclockwise or clockwise simple planar cavity profile.
    pub profile: Vec<Point2>,
    /// Exact lower z coordinate, strictly inside the outer extrusion.
    pub z_min: Real,
    /// Exact upper z coordinate, strictly inside the outer extrusion.
    pub z_max: Real,
}

/// One exact spherical cavity used by [`sphere_with_voids`].
#[derive(Clone, Debug)]
pub struct SphereVoid {
    /// Exact cavity center in model space.
    pub center: Point3,
    /// Exact positive cavity radius.
    pub radius: Real,
}

/// One planar polygon section supplied to [`loft`].
#[derive(Clone, Debug)]
pub struct LoftSection {
    /// Exact simple polygon in the common loft x/y chart.
    pub profile: Vec<Point2>,
    /// Exact section height; sections are ordered by increasing height.
    pub z: Real,
}

/// One explicitly authored rational Bézier frame for an exact curved sweep.
///
/// At parameter `t`, a profile point `(x, y)` maps to
/// `origin(t) + u(t)*x + v(t)*y`. All three vector-valued paths use the same
/// positive rational Bézier weights. The frame must remain in the initial
/// section-plane family and have a certified strictly positive oriented area
/// law; this is a complete Bernstein proof, not a sampled condition.
#[derive(Clone, Debug)]
pub struct RationalBezierSweepFrame {
    origins: Vec<Point3>,
    u_axes: Vec<Vector3>,
    v_axes: Vec<Vector3>,
    weights: Vec<Real>,
}

impl RationalBezierSweepFrame {
    /// Constructs and certifies one authored moving frame.
    pub fn try_new(
        origins: Vec<Point3>,
        u_axes: Vec<Vector3>,
        v_axes: Vec<Vector3>,
        weights: Vec<Real>,
    ) -> Result<Self, ConstructionError> {
        certify_sweep_frame(&origins, &u_axes, &v_axes, &weights)?;
        Ok(Self {
            origins,
            u_axes,
            v_axes,
            weights,
        })
    }

    /// Returns the authored origin controls.
    pub fn origins(&self) -> &[Point3] {
        &self.origins
    }

    /// Returns the authored first-axis controls.
    pub fn u_axes(&self) -> &[Vector3] {
        &self.u_axes
    }

    /// Returns the authored second-axis controls.
    pub fn v_axes(&self) -> &[Vector3] {
        &self.v_axes
    }

    /// Returns the shared positive rational Bézier weights.
    pub fn weights(&self) -> &[Real] {
        &self.weights
    }
}

/// One exact tensor patch supplied to [`tensor_patch_shell`].
#[derive(Clone, Debug)]
pub enum TensorPatch {
    /// Tensor-product rational Bézier patch.
    RationalBezier {
        /// Rectangular row-major control net.
        control_points: Vec<Vec<Point3>>,
        /// Positive projective weight at every control point.
        weights: Vec<Vec<Real>>,
    },
    /// Finite nonperiodic tensor-product NURBS patch.
    Nurbs {
        /// Degree in the `u` direction.
        u_degree: usize,
        /// Degree in the `v` direction.
        v_degree: usize,
        /// Rectangular row-major control net.
        control_points: Vec<Vec<Point3>>,
        /// Positive projective weight at every control point.
        weights: Vec<Vec<Real>>,
        /// Authored nondecreasing `u` knot vector.
        u_knots: Vec<Real>,
        /// Authored nondecreasing `v` knot vector.
        v_knots: Vec<Real>,
    },
}

impl fmt::Display for ConstructionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidBounds(axis) => {
                write!(formatter, "{axis:?} bounds are not strictly increasing")
            }
            Self::ProfileTooSmall => {
                formatter.write_str("extrusion profile has fewer than 3 vertices")
            }
            Self::InvalidFrustumRadii => {
                formatter.write_str("frustum requires base_radius > top_radius > 0")
            }
            Self::DegenerateProfile => formatter.write_str("extrusion profile has zero area"),
            Self::ProfileCrossesRevolutionAxis => {
                formatter.write_str("revolution profile must stay strictly off the axis")
            }
            Self::UnsupportedRevolutionProfile => formatter.write_str(
                "curved revolution profile lacks a required exact topology or integral certificate",
            ),
            Self::UnsupportedPlanarProfile => formatter.write_str(
                "planar path lacks a required exact simplicity, intersection, or persistence certificate",
            ),
            Self::RevolutionPatchSweepTooLarge => {
                formatter.write_str("one revolution patch must span less than one full period")
            }
            Self::LoftNeedsAtLeastTwoSections => {
                formatter.write_str("loft requires at least two sections")
            }
            Self::IncompatibleLoftSections => formatter
                .write_str("loft sections require a positive homothetic or convex correspondence"),
            Self::UnsupportedSweepPath => {
                formatter.write_str("curve sweep requires a rational Bézier path")
            }
            Self::NonMonotoneSweepPath => formatter.write_str(
                "curve sweep path must advance affinely and positively through the profile plane",
            ),
            Self::InvalidSweepFrame => formatter.write_str(
                "moving sweep frame requires matching valid rational Bezier control data",
            ),
            Self::NonPlanarSweepFrame => formatter.write_str(
                "moving sweep frame axes must remain in the initial section-plane family",
            ),
            Self::NonPositiveSweepFrameArea => formatter.write_str(
                "moving sweep frame must retain strictly positive oriented section area",
            ),
            Self::UnsupportedRationalSweepFrameArea => formatter.write_str(
                "nonconstant rational moving-frame area has no exact integration certificate",
            ),
            Self::EmptyPatchShell => {
                formatter.write_str("tensor patch shell requires at least one patch")
            }
            Self::SelfIntersectingProfile => {
                formatter.write_str("extrusion profile intersects itself")
            }
            Self::HoleOutside => formatter.write_str("extrusion hole lies outside its outer loop"),
            Self::IntersectingProfiles => formatter.write_str("extrusion profile loops intersect"),
            Self::NestedHoles => formatter.write_str("extrusion holes are nested"),
            Self::Build(error) => write!(formatter, "solid construction failed: {error}"),
            Self::Validation(report) => write!(formatter, "solid construction failed: {report}"),
        }
    }
}

impl std::error::Error for ConstructionError {}

impl From<BuildError> for ConstructionError {
    fn from(value: BuildError) -> Self {
        Self::Build(value)
    }
}

impl From<GeometryError> for ConstructionError {
    fn from(value: GeometryError) -> Self {
        Self::Build(BuildError::Geometry(value))
    }
}

impl From<ValidationReport> for ConstructionError {
    fn from(value: ValidationReport) -> Self {
        Self::Validation(value)
    }
}

/// Constructs one exact axis-aligned cuboid as a validated canonical model.
///
/// The returned solid is bounded by six outward-oriented parameterized planes.
/// Its twelve topological edges are shared by twenty-four independent face-local
/// pcurves.
pub fn cuboid(min: Point3, max: Point3) -> Result<(Model, SolidId), ConstructionError> {
    require_increasing(&min.x, &max.x, Axis::X)?;
    require_increasing(&min.y, &max.y, Axis::Y)?;
    let profile = [
        Point2::new(min.x.clone(), min.y.clone()),
        Point2::new(max.x.clone(), min.y.clone()),
        Point2::new(max.x, max.y.clone()),
        Point2::new(min.x, max.y),
    ];
    extrude(&profile, min.z, max.z)
}

/// Constructs one exact trimmed planar face in an authored parameter frame.
///
/// The outer [`CurvePath2`] is normalized counterclockwise and every hole is
/// normalized clockwise. Complete path simplicity, pairwise non-contact, hole
/// containment, and non-nesting are certified by Hypercurve before topology is
/// authored. Line, rational Bézier, and non-periodic NURBS carriers retain
/// their native persistent families and domains; other exact path families
/// are promoted without approximation to persistent rational Bézier or NURBS
/// carriers. The returned model contains one validated open shell and no
/// solid.
pub fn planar_face(
    outer: &CurvePath2,
    holes: &[CurvePath2],
    origin: Point3,
    u: Vector3,
    v: Vector3,
) -> Result<(Model, FaceId), ConstructionError> {
    let outer = normalize_planar_path(outer, true)?;
    let holes = holes
        .iter()
        .map(|hole| normalize_planar_path(hole, false))
        .collect::<Result<Vec<_>, _>>()?;
    validate_planar_path_nesting(&outer, &holes)?;

    let surface = Surface::plane(origin, u, v)?;
    let mut builder = ModelBuilder::new();
    let surface_id = builder.surface(surface.clone())?;
    let outer_wire = add_planar_path_wire(&mut builder, &outer, &surface)?;
    let inner_wires = holes
        .iter()
        .map(|hole| add_planar_path_wire(&mut builder, hole, &surface))
        .collect::<Result<Vec<_>, _>>()?;
    let face = builder.face(surface_id, Orientation::Forward, outer_wire, inner_wires)?;
    builder.shell(vec![face])?;
    Ok((builder.finish()?, face))
}

/// Constructs one finite rectangular patch of an exact extrusion surface.
///
/// The profile parameter is retained as `u`; `v_start` and `v_end` are
/// strictly ordered signed coefficients of `direction`. The two profile edges
/// retain the profile's exact curve family and native domain, while the two
/// connector edges are exact lines. The returned model contains one validated
/// open shell and no solid.
pub fn extrusion_patch(
    profile: Curve3,
    direction: Vector3,
    v_start: Real,
    v_end: Real,
) -> Result<(Model, FaceId), ConstructionError> {
    let v_domain = ParameterDomain::new(v_start, v_end)?;
    let u_start = profile.domain().start().clone();
    let u_end = profile.domain().end().clone();
    let surface = Surface::extrusion(profile.clone(), direction.clone())?;
    let lower_offset = direction.clone() * v_domain.start();
    let upper_offset = direction * v_domain.end();
    let lower_profile = translated_curve(&profile, &lower_offset)?;
    let upper_profile = translated_curve(&profile, &upper_offset)?;
    let lower_start = lower_profile.point_at(lower_profile.domain().start())?;
    let lower_end = lower_profile.point_at(lower_profile.domain().end())?;
    let upper_start = upper_profile.point_at(upper_profile.domain().start())?;
    let upper_end = upper_profile.point_at(upper_profile.domain().end())?;
    let boundaries = [
        lower_profile,
        Curve3::line(lower_end, upper_end)?,
        upper_profile,
        Curve3::line(lower_start, upper_start)?,
    ];
    build_rectangular_face_patch(
        surface,
        boundaries,
        u_start,
        u_end,
        v_domain.start().clone(),
        v_domain.end().clone(),
    )
}

/// Constructs one finite rectangular patch of an exact revolution surface.
///
/// `angle_start` and `angle_end` are strictly ordered and must span less than
/// one full period; complete revolutions require explicit periodic
/// subdivision. The profile's exact curve family and native parameter domain
/// are retained on both meridians. Its complete interval must carry an exact
/// strict axis-clearance certificate; unsupported higher-degree contact
/// problems return [`ConstructionError::UnsupportedRevolutionProfile`]. The
/// returned model contains one validated open shell and no solid.
pub fn revolution_patch(
    profile: Curve3,
    axis_origin: Point3,
    axis: Vector3,
    angle_start: Real,
    angle_end: Real,
) -> Result<(Model, FaceId), ConstructionError> {
    let angle_domain = ParameterDomain::new(angle_start, angle_end)?;
    let angle_span = angle_domain.end() - angle_domain.start();
    match compare_reals(&angle_span, &Real::tau(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Less,
            ..
        } => {}
        PredicateOutcome::Decided { .. } => {
            return Err(ConstructionError::RevolutionPatchSweepTooLarge);
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(GeometryError::PredicateUnresolved { needed, stage }.into());
        }
    }
    let profile_domain = profile.domain().clone();
    let surface = Surface::revolution(profile, axis_origin, axis)?;
    if !surface
        .revolution_profile_is_strictly_off_axis()
        .map_err(revolution_patch_geometry_error)?
    {
        return Err(ConstructionError::ProfileCrossesRevolutionAxis);
    }
    let lower_latitude = surface
        .revolution_latitude_curve(
            profile_domain.start(),
            angle_domain.start().clone(),
            angle_domain.end().clone(),
        )
        .map_err(revolution_patch_geometry_error)?;
    let upper_latitude = surface
        .revolution_latitude_curve(
            profile_domain.end(),
            angle_domain.start().clone(),
            angle_domain.end().clone(),
        )
        .map_err(revolution_patch_geometry_error)?;
    let boundaries = [
        lower_latitude,
        surface.revolution_meridian_curve(angle_domain.end())?,
        upper_latitude,
        surface.revolution_meridian_curve(angle_domain.start())?,
    ];
    build_rectangular_face_patch(
        surface,
        boundaries,
        angle_domain.start().clone(),
        angle_domain.end().clone(),
        profile_domain.start().clone(),
        profile_domain.end().clone(),
    )
}

fn revolution_patch_geometry_error(error: GeometryError) -> ConstructionError {
    match error {
        GeometryError::SingularSurfaceParameter => ConstructionError::ProfileCrossesRevolutionAxis,
        GeometryError::UnsupportedIntersection => ConstructionError::UnsupportedRevolutionProfile,
        error => error.into(),
    }
}

/// Constructs one validated trimmed tensor-product rational Bézier patch.
///
/// The four boundary edges are exact rational Bézier restrictions of the
/// surface control net. The returned model contains one open shell and no
/// solid.
pub fn rational_bezier_patch(
    control_points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
) -> Result<(Model, crate::FaceId), ConstructionError> {
    let surface = Surface::rational_bezier(control_points.clone(), weights.clone())?;
    let u_last = control_points[0].len() - 1;
    let v_last = control_points.len() - 1;
    let boundaries = [
        Curve3::rational_bezier(control_points[0].clone(), weights[0].clone())?,
        Curve3::rational_bezier(
            control_points
                .iter()
                .map(|row| row[u_last].clone())
                .collect(),
            weights.iter().map(|row| row[u_last].clone()).collect(),
        )?,
        Curve3::rational_bezier(control_points[v_last].clone(), weights[v_last].clone())?,
        Curve3::rational_bezier(
            control_points.iter().map(|row| row[0].clone()).collect(),
            weights.iter().map(|row| row[0].clone()).collect(),
        )?,
    ];
    build_rectangular_face_patch(
        surface,
        boundaries,
        Real::zero(),
        Real::one(),
        Real::zero(),
        Real::one(),
    )
}

/// Constructs one validated trimmed finite tensor-product NURBS patch.
///
/// Each boundary edge retains the corresponding clamped NURBS row or column,
/// including its authored degree, knot vector, control points, and weights.
/// The returned model contains one open shell and no solid.
#[allow(clippy::too_many_arguments)]
pub fn nurbs_patch(
    u_degree: usize,
    v_degree: usize,
    control_points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
    u_knots: Vec<Real>,
    v_knots: Vec<Real>,
) -> Result<(Model, crate::FaceId), ConstructionError> {
    let u_count = control_points.first().map_or(0, Vec::len);
    let v_count = control_points.len();
    let surface = Surface::nurbs(
        u_degree,
        v_degree,
        control_points.clone(),
        weights.clone(),
        u_knots.clone(),
        v_knots.clone(),
    )?;
    let u_last = u_count - 1;
    let v_last = v_count - 1;
    let boundaries = [
        Curve3::nurbs(
            u_degree,
            control_points[0].clone(),
            weights[0].clone(),
            u_knots.clone(),
        )?,
        Curve3::nurbs(
            v_degree,
            control_points
                .iter()
                .map(|row| row[u_last].clone())
                .collect(),
            weights.iter().map(|row| row[u_last].clone()).collect(),
            v_knots.clone(),
        )?,
        Curve3::nurbs(
            u_degree,
            control_points[v_last].clone(),
            weights[v_last].clone(),
            u_knots.clone(),
        )?,
        Curve3::nurbs(
            v_degree,
            control_points.iter().map(|row| row[0].clone()).collect(),
            weights.iter().map(|row| row[0].clone()).collect(),
            v_knots.clone(),
        )?,
    ];
    build_rectangular_face_patch(
        surface,
        boundaries,
        u_knots[u_degree].clone(),
        u_knots[u_count].clone(),
        v_knots[v_degree].clone(),
        v_knots[v_count].clone(),
    )
}

/// Builds one validated open shell from exact tensor patches.
///
/// Boundary curves with the same exact endpoint vertices are stitched only
/// when their complete homogeneous representations are projectively
/// identical, including reversed traversal. Every face retains its own
/// pcurves and parameter correspondence; the shared model edge is authored
/// once and used in opposite directions by adjacent consistently oriented
/// patches. Unmatched boundaries remain open-shell boundaries.
pub fn tensor_patch_shell(
    patches: Vec<TensorPatch>,
) -> Result<(Model, Vec<FaceId>), ConstructionError> {
    if patches.is_empty() {
        return Err(ConstructionError::EmptyPatchShell);
    }
    let patches = patches
        .into_iter()
        .map(tensor_patch_build_data)
        .collect::<Result<Vec<_>, _>>()?;
    let mut builder = ModelBuilder::new();
    let mut vertices = Vec::<(Point3, VertexId)>::new();
    let mut shared_edges = Vec::<StitchedTensorEdge>::new();
    let mut faces = Vec::with_capacity(patches.len());
    for patch in patches {
        let patch_vertices = patch
            .points
            .iter()
            .map(|point| exact_patch_vertex(&mut builder, &mut vertices, point))
            .collect::<Result<Vec<_>, _>>()?;
        let surface = builder.surface(patch.surface)?;
        let natural_endpoints = [(0, 1), (1, 2), (3, 2), (0, 3)];
        let face_endpoints = [(0, 1), (1, 2), (2, 3), (3, 0)];
        let mut edges = Vec::with_capacity(4);
        for (index, curve) in patch.boundaries.into_iter().enumerate() {
            let (natural_start, natural_end) = natural_endpoints[index];
            let start = patch_vertices[natural_start];
            let end = patch_vertices[natural_end];
            let mut stitched = None;
            for existing in &shared_edges {
                let reversed = if existing.start == start && existing.end == end {
                    false
                } else if existing.start == end && existing.end == start {
                    true
                } else {
                    continue;
                };
                if exact_tensor_boundary_equal(&curve, &existing.curve, reversed)? {
                    stitched = Some((existing.edge, existing.domain.clone()));
                    break;
                }
            }
            let (edge, domain) = match stitched {
                Some(stitched) => stitched,
                None => {
                    let domain = curve.domain().clone();
                    let curve_id = builder.curve(curve.clone())?;
                    let edge = builder.edge(start, end, curve_id, domain.clone())?;
                    shared_edges.push(StitchedTensorEdge {
                        start,
                        end,
                        curve,
                        edge,
                        domain: domain.clone(),
                    });
                    (edge, domain)
                }
            };
            edges.push((edge, domain));
        }

        let mut uses = Vec::with_capacity(4);
        for index in 0..4 {
            let (face_start, face_end) = face_endpoints[index];
            let desired_start = patch_vertices[face_start];
            let desired_end = patch_vertices[face_end];
            let (edge, domain) = &edges[index];
            let edge_record = shared_edges
                .iter()
                .find(|candidate| candidate.edge == *edge)
                .expect("every stitched edge has one canonical record");
            let direction = if edge_record.start == desired_start && edge_record.end == desired_end
            {
                Direction::Forward
            } else if edge_record.start == desired_end && edge_record.end == desired_start {
                Direction::Reversed
            } else {
                unreachable!("patch edge endpoints were resolved from the same corners");
            };
            let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(
                    patch.parameters[face_start].clone(),
                    patch.parameters[face_end].clone(),
                )
                .map_err(GeometryError::from)?,
            )))?;
            let correspondence = match direction {
                Direction::Forward => ParameterCorrespondence::affine(
                    domain.end() - domain.start(),
                    domain.start().clone(),
                )?,
                Direction::Reversed => ParameterCorrespondence::affine(
                    domain.start() - domain.end(),
                    domain.end().clone(),
                )?,
            };
            uses.push(builder.edge_use(*edge, direction, pcurve, correspondence)?);
        }
        let wire = builder.wire(uses)?;
        faces.push(builder.face(surface, Orientation::Forward, wire, Vec::new())?);
    }
    builder.shell(faces.clone())?;
    Ok((builder.finish()?, faces))
}

struct TensorPatchBuildData {
    surface: Surface,
    boundaries: [Curve3; 4],
    parameters: [CurvePoint2; 4],
    points: [Point3; 4],
}

struct StitchedTensorEdge {
    start: VertexId,
    end: VertexId,
    curve: Curve3,
    edge: EdgeId,
    domain: ParameterDomain,
}

fn tensor_patch_build_data(patch: TensorPatch) -> Result<TensorPatchBuildData, ConstructionError> {
    let (surface, boundaries, u_start, u_end, v_start, v_end) = match patch {
        TensorPatch::RationalBezier {
            control_points,
            weights,
        } => {
            let surface = Surface::rational_bezier(control_points.clone(), weights.clone())?;
            let u_last = control_points[0].len() - 1;
            let v_last = control_points.len() - 1;
            let boundaries = [
                Curve3::rational_bezier(control_points[0].clone(), weights[0].clone())?,
                Curve3::rational_bezier(
                    control_points
                        .iter()
                        .map(|row| row[u_last].clone())
                        .collect(),
                    weights.iter().map(|row| row[u_last].clone()).collect(),
                )?,
                Curve3::rational_bezier(control_points[v_last].clone(), weights[v_last].clone())?,
                Curve3::rational_bezier(
                    control_points.iter().map(|row| row[0].clone()).collect(),
                    weights.iter().map(|row| row[0].clone()).collect(),
                )?,
            ];
            (
                surface,
                boundaries,
                Real::zero(),
                Real::one(),
                Real::zero(),
                Real::one(),
            )
        }
        TensorPatch::Nurbs {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        } => {
            let u_count = control_points.first().map_or(0, Vec::len);
            let v_count = control_points.len();
            let surface = Surface::nurbs(
                u_degree,
                v_degree,
                control_points.clone(),
                weights.clone(),
                u_knots.clone(),
                v_knots.clone(),
            )?;
            let u_last = u_count - 1;
            let v_last = v_count - 1;
            let boundaries = [
                Curve3::nurbs(
                    u_degree,
                    control_points[0].clone(),
                    weights[0].clone(),
                    u_knots.clone(),
                )?,
                Curve3::nurbs(
                    v_degree,
                    control_points
                        .iter()
                        .map(|row| row[u_last].clone())
                        .collect(),
                    weights.iter().map(|row| row[u_last].clone()).collect(),
                    v_knots.clone(),
                )?,
                Curve3::nurbs(
                    u_degree,
                    control_points[v_last].clone(),
                    weights[v_last].clone(),
                    u_knots.clone(),
                )?,
                Curve3::nurbs(
                    v_degree,
                    control_points.iter().map(|row| row[0].clone()).collect(),
                    weights.iter().map(|row| row[0].clone()).collect(),
                    v_knots.clone(),
                )?,
            ];
            (
                surface,
                boundaries,
                u_knots[u_degree].clone(),
                u_knots[u_count].clone(),
                v_knots[v_degree].clone(),
                v_knots[v_count].clone(),
            )
        }
    };
    let parameters = [
        CurvePoint2::new(u_start.clone(), v_start.clone()),
        CurvePoint2::new(u_end.clone(), v_start),
        CurvePoint2::new(u_end, v_end.clone()),
        CurvePoint2::new(u_start, v_end),
    ];
    let points = parameters
        .iter()
        .map(|parameter| {
            surface.point_at(&Point2::new(parameter.x().clone(), parameter.y().clone()))
        })
        .collect::<Result<Vec<_>, _>>()?
        .try_into()
        .expect("four tensor corners");
    Ok(TensorPatchBuildData {
        surface,
        boundaries,
        parameters,
        points,
    })
}

fn exact_patch_vertex(
    builder: &mut ModelBuilder,
    vertices: &mut Vec<(Point3, VertexId)>,
    point: &Point3,
) -> Result<VertexId, ConstructionError> {
    for (existing, vertex) in vertices.iter() {
        match point3_equal(existing, point, crate::STRICT_PREDICATES) {
            PredicateOutcome::Decided { value: true, .. } => return Ok(*vertex),
            PredicateOutcome::Decided { value: false, .. } => {}
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(GeometryError::PredicateUnresolved { needed, stage }.into());
            }
        }
    }
    let vertex = builder.vertex(point.clone())?;
    vertices.push((point.clone(), vertex));
    Ok(vertex)
}

fn exact_tensor_boundary_equal(
    candidate: &Curve3,
    existing: &Curve3,
    reversed: bool,
) -> Result<bool, ConstructionError> {
    let candidate = if reversed {
        candidate.reversed()?
    } else {
        candidate.clone()
    };
    if !exact_real_equal(candidate.domain().start(), existing.domain().start())?
        || !exact_real_equal(candidate.domain().end(), existing.domain().end())?
    {
        return Ok(false);
    }
    let unpack = |curve: &Curve3| match curve.exact_data() {
        Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } => Some((control_points, weights, None, None)),
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Some((control_points, weights, Some(degree), Some(knots))),
        _ => None,
    };
    let Some((candidate_points, candidate_weights, candidate_degree, candidate_knots)) =
        unpack(&candidate)
    else {
        return Ok(false);
    };
    let Some((existing_points, existing_weights, existing_degree, existing_knots)) =
        unpack(existing)
    else {
        return Ok(false);
    };
    if candidate_degree != existing_degree
        || candidate_points.len() != existing_points.len()
        || candidate_weights.len() != existing_weights.len()
    {
        return Ok(false);
    }
    match (candidate_knots.as_ref(), existing_knots.as_ref()) {
        (Some(candidate), Some(existing)) if candidate.len() == existing.len() => {
            for (candidate, existing) in candidate.iter().zip(existing) {
                if !exact_real_equal(candidate, existing)? {
                    return Ok(false);
                }
            }
        }
        (None, None) => {}
        _ => return Ok(false),
    }
    for (candidate, existing) in candidate_points.iter().zip(&existing_points) {
        match point3_equal(candidate, existing, crate::STRICT_PREDICATES) {
            PredicateOutcome::Decided { value: true, .. } => {}
            PredicateOutcome::Decided { value: false, .. } => return Ok(false),
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(GeometryError::PredicateUnresolved { needed, stage }.into());
            }
        }
    }
    let scale = (&candidate_weights[0] / &existing_weights[0])
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    for (candidate, existing) in candidate_weights.iter().zip(&existing_weights) {
        if !exact_real_equal(candidate, &(existing * &scale))? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn exact_real_equal(left: &Real, right: &Real) -> Result<bool, ConstructionError> {
    match compare_reals(left, right, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided { value, .. } => Ok(value == std::cmp::Ordering::Equal),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage }.into())
        }
    }
}

fn translated_curve(curve: &Curve3, offset: &Vector3) -> Result<Curve3, ConstructionError> {
    Ok(curve.transformed(&crate::Matrix4::affine_translation(offset.0.clone()))?)
}

fn build_rectangular_face_patch(
    surface: Surface,
    boundaries: [Curve3; 4],
    u_start: Real,
    u_end: Real,
    v_start: Real,
    v_end: Real,
) -> Result<(Model, crate::FaceId), ConstructionError> {
    let parameters = [
        CurvePoint2::new(u_start.clone(), v_start.clone()),
        CurvePoint2::new(u_end.clone(), v_start),
        CurvePoint2::new(u_end, v_end.clone()),
        CurvePoint2::new(u_start, v_end),
    ];
    let points = parameters
        .iter()
        .map(|parameter| {
            surface
                .point_at(&Point2::new(parameter.x().clone(), parameter.y().clone()))
                .map_err(ConstructionError::from)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut builder = ModelBuilder::new();
    let vertices = points
        .into_iter()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let surface = builder.surface(surface)?;
    let mut edges = Vec::with_capacity(4);
    let mut edge_domains = Vec::with_capacity(4);
    for (index, curve) in boundaries.into_iter().enumerate() {
        let (start, end) = match index {
            0 => (0, 1),
            1 => (1, 2),
            2 => (3, 2),
            3 => (0, 3),
            _ => unreachable!("four tensor boundaries"),
        };
        let domain = curve.domain().clone();
        let curve = builder.curve(curve)?;
        edges.push(builder.edge(vertices[start], vertices[end], curve, domain.clone())?);
        edge_domains.push(domain);
    }
    let specs = [
        (0, edges[0], Direction::Forward, 0, 1),
        (1, edges[1], Direction::Forward, 1, 2),
        (2, edges[2], Direction::Reversed, 2, 3),
        (3, edges[3], Direction::Reversed, 3, 0),
    ];
    let mut uses = Vec::with_capacity(4);
    for (index, edge, direction, start, end) in specs {
        let edge_domain = &edge_domains[index];
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(parameters[start].clone(), parameters[end].clone())
                .map_err(GeometryError::from)?,
        )))?;
        let correspondence = match direction {
            Direction::Forward => ParameterCorrespondence::affine(
                edge_domain.end() - edge_domain.start(),
                edge_domain.start().clone(),
            )?,
            Direction::Reversed => ParameterCorrespondence::affine(
                edge_domain.start() - edge_domain.end(),
                edge_domain.end().clone(),
            )?,
        };
        uses.push(builder.edge_use(edge, direction, pcurve, correspondence)?);
    }
    let wire = builder.wire(uses)?;
    let face = builder.face(surface, Orientation::Forward, wire, Vec::new())?;
    builder.shell(vec![face])?;
    Ok((builder.finish()?, face))
}

/// Constructs a right circular cylinder on the positive z axis.
///
/// The base center is the model origin. `radius` and `height` must be
/// certified positive. The result uses four exact quarter-circle edges on
/// each cap, four axial edges, two planar caps, and four trimmed faces sharing
/// one native cylindrical surface. No polygonal approximation is introduced.
pub fn cylinder(radius: Real, height: Real) -> Result<(Model, SolidId), ConstructionError> {
    require_increasing(&Real::zero(), &height, Axis::Z)?;
    let mut builder = ModelBuilder::new();
    let zero = Real::zero();
    let points_2d = [
        CurvePoint2::new(radius.clone(), zero.clone()),
        CurvePoint2::new(zero.clone(), radius.clone()),
        CurvePoint2::new(-radius.clone(), zero.clone()),
        CurvePoint2::new(zero.clone(), -radius.clone()),
    ];
    let mut points = Vec::with_capacity(8);
    points.extend(
        points_2d
            .iter()
            .map(|point| Point3::new(point.x().clone(), point.y().clone(), Real::zero())),
    );
    points.extend(
        points_2d
            .iter()
            .map(|point| Point3::new(point.x().clone(), point.y().clone(), height.clone())),
    );
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let half_pi = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let mut bottom_edges = Vec::with_capacity(4);
    let mut top_edges = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        let start_angle = &half_pi * Real::from(index as i32);
        let end_angle = &half_pi * Real::from(index as i32 + 1);
        let domain = ParameterDomain::new(start_angle.clone(), end_angle.clone())?;
        let bottom_curve = builder.curve(Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            radius.clone(),
            start_angle.clone(),
            end_angle.clone(),
        )?)?;
        bottom_edges.push(builder.edge(
            vertices[index],
            vertices[next],
            bottom_curve,
            domain.clone(),
        )?);
        let top_curve = builder.curve(Curve3::circle_arc(
            Point3::new(Real::zero(), Real::zero(), height.clone()),
            Vector3::x(),
            Vector3::y(),
            radius.clone(),
            start_angle,
            end_angle,
        )?)?;
        top_edges.push(builder.edge(vertices[index + 4], vertices[next + 4], top_curve, domain)?);
    }
    let mut axial_edges = Vec::with_capacity(4);
    for index in 0..4 {
        let curve = builder.curve(Curve3::line(
            points[index].clone(),
            points[index + 4].clone(),
        )?)?;
        axial_edges.push(builder.edge(
            vertices[index],
            vertices[index + 4],
            curve,
            ParameterDomain::unit(),
        )?);
    }

    let bottom_surface = builder.surface(Surface::plane(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
    )?)?;
    let mut bottom_uses = Vec::with_capacity(4);
    for index in (0..4).rev() {
        let next = (index + 1) % 4;
        let arc = CircularArc2::try_from_center(
            points_2d[next].clone(),
            points_2d[index].clone(),
            CurvePoint2::new(Real::zero(), Real::zero()),
            true,
        )
        .map_err(GeometryError::from)?;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(arc)))?;
        bottom_uses.push(builder.edge_use(
            bottom_edges[index],
            Direction::Reversed,
            pcurve,
            ParameterCorrespondence::angular_sweep(),
        )?);
    }
    let bottom_wire = builder.wire(bottom_uses)?;
    let bottom_face = builder.face(
        bottom_surface,
        Orientation::Reversed,
        bottom_wire,
        Vec::new(),
    )?;

    let top_surface = builder.surface(Surface::plane(
        Point3::new(Real::zero(), Real::zero(), height.clone()),
        Vector3::x(),
        Vector3::y(),
    )?)?;
    let mut top_uses = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        let arc = CircularArc2::try_from_center(
            points_2d[index].clone(),
            points_2d[next].clone(),
            CurvePoint2::new(Real::zero(), Real::zero()),
            false,
        )
        .map_err(GeometryError::from)?;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(arc)))?;
        top_uses.push(builder.edge_use(
            top_edges[index],
            Direction::Forward,
            pcurve,
            ParameterCorrespondence::angular_sweep(),
        )?);
    }
    let top_wire = builder.wire(top_uses)?;
    let top_face = builder.face(top_surface, Orientation::Forward, top_wire, Vec::new())?;

    let cylinder_surface = builder.surface(Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        radius,
    )?)?;
    let mut side_faces = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        let start_angle = &half_pi * Real::from(index as i32);
        let end_angle = &half_pi * Real::from(index as i32 + 1);
        let span = &end_angle - &start_angle;
        let specs = [
            (
                bottom_edges[index],
                Direction::Forward,
                CurvePoint2::new(start_angle.clone(), Real::zero()),
                CurvePoint2::new(end_angle.clone(), Real::zero()),
                ParameterCorrespondence::affine(span.clone(), start_angle.clone())?,
            ),
            (
                axial_edges[next],
                Direction::Forward,
                CurvePoint2::new(end_angle.clone(), Real::zero()),
                CurvePoint2::new(end_angle.clone(), height.clone()),
                ParameterCorrespondence::identity(),
            ),
            (
                top_edges[index],
                Direction::Reversed,
                CurvePoint2::new(end_angle.clone(), height.clone()),
                CurvePoint2::new(start_angle.clone(), height.clone()),
                ParameterCorrespondence::affine(-span.clone(), end_angle.clone())?,
            ),
            (
                axial_edges[index],
                Direction::Reversed,
                CurvePoint2::new(start_angle.clone(), height.clone()),
                CurvePoint2::new(start_angle, Real::zero()),
                ParameterCorrespondence::affine(-Real::one(), Real::one())?,
            ),
        ];
        let mut uses = Vec::with_capacity(4);
        for (edge, direction, start, end, correspondence) in specs {
            let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
            )))?;
            uses.push(builder.edge_use(edge, direction, pcurve, correspondence)?);
        }
        let wire = builder.wire(uses)?;
        side_faces.push(builder.face(cylinder_surface, Orientation::Forward, wire, Vec::new())?);
    }

    let mut faces = Vec::with_capacity(6);
    faces.push(bottom_face);
    faces.push(top_face);
    faces.extend(side_faces);
    let shell = builder.shell(faces)?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

/// Constructs one exact sphere centered at the model origin.
///
/// The sphere is represented by one complete closed-surface face. No
/// artificial longitude seam, collapsed pole edge, or tolerance sewing is
/// introduced.
pub fn sphere(radius: Real) -> Result<(Model, SolidId), ConstructionError> {
    sphere_with_voids(radius, &[])
}

/// Constructs one exact origin-centered sphere with disjoint spherical voids.
///
/// Every boundary is one complete closed-surface face. Void faces are
/// inward-oriented and must be strictly contained and pairwise non-contacting.
pub fn sphere_with_voids(
    radius: Real,
    voids: &[SphereVoid],
) -> Result<(Model, SolidId), ConstructionError> {
    let surface = Surface::sphere(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        radius,
    )?;
    let mut builder = ModelBuilder::new();
    let surface = builder.surface(surface)?;
    let face = builder.whole_face(surface, Orientation::Forward)?;
    let shell = builder.shell(vec![face])?;
    let mut void_shells = Vec::with_capacity(voids.len());
    for void in voids {
        let surface = builder.surface(Surface::sphere(
            void.center.clone(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            void.radius.clone(),
        )?)?;
        let face = builder.whole_face(surface, Orientation::Reversed)?;
        void_shells.push(builder.shell(vec![face])?);
    }
    let solid = builder.solid(shell, void_shells)?;
    Ok((builder.finish()?, solid))
}

pub(crate) fn sphere_pair_boolean(
    first_center: Point3,
    first_radius: Real,
    second_center: Point3,
    second_radius: Real,
    kind: CertifiedSpherePairKind,
) -> Result<(Model, SolidId), ConstructionError> {
    let displacement = &second_center - &first_center;
    let distance_squared = displacement.norm_squared();
    let distance = distance_squared
        .clone()
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let inverse_distance =
        (Real::one() / &distance).map_err(|_| GeometryError::ProjectiveDivision)?;
    let axis = displacement * inverse_distance;
    let (x, y) = axis
        .orthonormal_basis_checked()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let first_plane_distance = ((&first_radius * &first_radius - &second_radius * &second_radius
        + &distance_squared)
        / (Real::from(2) * &distance))
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let second_plane_distance = &distance - &first_plane_distance;
    let circle_radius = (&first_radius * &first_radius
        - &first_plane_distance * &first_plane_distance)
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let circle_center = first_center.clone() + axis.clone() * &first_plane_distance;
    let first_latitude = (first_plane_distance.clone() / &first_radius)
        .map_err(|_| GeometryError::ProjectiveDivision)?
        .asin()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let second_latitude = (second_plane_distance / &second_radius)
        .map_err(|_| GeometryError::ProjectiveDivision)?
        .asin()
        .map_err(|_| GeometryError::ElementaryFunction)?;

    let mut builder = ModelBuilder::new();
    let full_circle = Curve3::circle_arc(
        circle_center,
        x.clone(),
        y.clone(),
        circle_radius,
        Real::zero(),
        Real::tau(),
    )?;
    let quarter = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let vertices = (0..4)
        .map(|index| full_circle.point_at(&(&quarter * Real::from(index))))
        .collect::<Result<Vec<_>, GeometryError>>()?
        .into_iter()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, BuildError>>()?;
    let curve = builder.curve(full_circle)?;
    let edges = (0..4)
        .map(|index| {
            let start = &quarter * Real::from(index as i32);
            let end = &quarter * Real::from(index as i32 + 1);
            builder.edge(
                vertices[index],
                vertices[(index + 1) % 4],
                curve,
                ParameterDomain::new(start, end)?,
            )
        })
        .collect::<Result<Vec<_>, BuildError>>()?;

    let (first_upper, first_orientation, second_upper, second_orientation) = match kind {
        CertifiedSpherePairKind::Union => {
            (false, Orientation::Forward, false, Orientation::Forward)
        }
        CertifiedSpherePairKind::Intersection => {
            (true, Orientation::Forward, true, Orientation::Forward)
        }
        CertifiedSpherePairKind::Difference => {
            (false, Orientation::Forward, true, Orientation::Reversed)
        }
    };
    let first_surface = builder.surface(Surface::sphere(
        first_center,
        x.clone(),
        y.clone(),
        axis.clone(),
        first_radius,
    )?)?;
    let second_surface =
        builder.surface(Surface::sphere(second_center, x, -y, -axis, second_radius)?)?;
    let first_face = spherical_cap_face(
        &mut builder,
        first_surface,
        &edges,
        first_latitude,
        first_upper,
        first_orientation,
        1,
        &quarter,
    )?;
    let second_face = spherical_cap_face(
        &mut builder,
        second_surface,
        &edges,
        second_latitude,
        second_upper,
        second_orientation,
        -1,
        &quarter,
    )?;
    let shell = builder.shell(vec![first_face, second_face])?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

#[allow(clippy::too_many_arguments)]
fn spherical_cap_face(
    builder: &mut ModelBuilder,
    surface: crate::SurfaceId,
    edges: &[EdgeId],
    latitude: Real,
    upper: bool,
    orientation: Orientation,
    local_to_circle: i32,
    quarter: &Real,
) -> Result<crate::FaceId, ConstructionError> {
    let increasing = match orientation {
        Orientation::Forward => upper,
        Orientation::Reversed => !upper,
    };
    let circle_increasing = if local_to_circle > 0 {
        increasing
    } else {
        !increasing
    };
    let indices = if circle_increasing {
        vec![0, 1, 2, 3]
    } else {
        vec![3, 2, 1, 0]
    };
    let mut uses = Vec::with_capacity(4);
    for index in indices {
        let (direction, circle_start, circle_end) = if circle_increasing {
            (
                Direction::Forward,
                quarter * Real::from(index as i32),
                quarter * Real::from(index as i32 + 1),
            )
        } else {
            (
                Direction::Reversed,
                quarter * Real::from(index as i32 + 1),
                quarter * Real::from(index as i32),
            )
        };
        let local_start = Real::from(local_to_circle) * &circle_start;
        let local_end = Real::from(local_to_circle) * &circle_end;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(local_start, latitude.clone()),
                CurvePoint2::new(local_end, latitude.clone()),
            )
            .map_err(GeometryError::from)?,
        )))?;
        let correspondence = match direction {
            Direction::Forward => ParameterCorrespondence::affine(quarter.clone(), circle_start)?,
            Direction::Reversed => ParameterCorrespondence::affine(-quarter.clone(), circle_start)?,
        };
        uses.push(builder.edge_use(edges[index], direction, pcurve, correspondence)?);
    }
    let wire = builder.wire(uses)?;
    Ok(builder.face(surface, orientation, wire, Vec::new())?)
}

/// Builds a standard z-axis truncated cone with exact circular caps.
///
/// The base lies at `z = 0`, the top at `z = height`, and
/// `base_radius > top_radius > 0`.
pub fn cone_frustum(
    base_radius: Real,
    top_radius: Real,
    height: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    require_increasing(&Real::zero(), &height, Axis::Z)?;
    require_frustum_radii(&base_radius, &top_radius)?;
    let radial_drop = &base_radius - &top_radius;
    let apex_height =
        (&height * &base_radius / &radial_drop).map_err(|_| GeometryError::ProjectiveDivision)?;
    let semi_angle = crate::geometry::certified_atan2(radial_drop.clone(), height.clone())?;
    let sine = semi_angle.clone().sin();
    let cosine = semi_angle.clone().cos();
    let v_bottom = (&base_radius / &sine).map_err(|_| GeometryError::ProjectiveDivision)?;
    let v_top = (&top_radius / &sine).map_err(|_| GeometryError::ProjectiveDivision)?;
    let bottom_radius = &v_bottom * &sine;
    let top_radius_on_cone = &v_top * &sine;
    let bottom_center = Point3::new(
        Real::zero(),
        Real::zero(),
        &apex_height - &v_bottom * &cosine,
    );
    let top_center = Point3::new(Real::zero(), Real::zero(), &apex_height - &v_top * &cosine);
    let quarter = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let angles = (0..4)
        .map(|index| &quarter * Real::from(index))
        .collect::<Vec<_>>();
    let cone = Surface::cone(
        Point3::new(Real::zero(), Real::zero(), apex_height),
        Vector3::x(),
        -Vector3::y(),
        -Vector3::z(),
        semi_angle,
    )?;
    let mut builder = ModelBuilder::new();
    let cone_surface = builder.surface(cone.clone())?;
    // Author every incidence in the cone's retained meridian frame. Rebuilding
    // the cap centers and radii from the nominal inputs asks a general Real
    // equality predicate to rediscover inverse-trigonometric identities.
    // Sharing these exact expressions lets topology validate by identity while
    // preserving the mathematically identical requested dimensions.
    let mut points = Vec::with_capacity(8);
    for u in &angles {
        points.push(
            bottom_center.clone() + Vector3::x() * (&bottom_radius * u.clone().cos())
                - Vector3::y() * (&bottom_radius * u.clone().sin()),
        );
    }
    for u in &angles {
        points.push(
            top_center.clone() + Vector3::x() * (&top_radius_on_cone * u.clone().cos())
                - Vector3::y() * (&top_radius_on_cone * u.clone().sin()),
        );
    }
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;

    let mut bottom_edges = Vec::with_capacity(4);
    let mut top_edges = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        let start = angles[index].clone();
        let end = &quarter * Real::from((index + 1) as i32);
        let bottom_curve = builder.curve(Curve3::circle_arc(
            bottom_center.clone(),
            Vector3::x(),
            -Vector3::y(),
            bottom_radius.clone(),
            start.clone(),
            end.clone(),
        )?)?;
        bottom_edges.push(builder.edge(
            vertices[index],
            vertices[next],
            bottom_curve,
            ParameterDomain::new(start.clone(), end.clone())?,
        )?);
        let top_curve = builder.curve(Curve3::circle_arc(
            top_center.clone(),
            Vector3::x(),
            -Vector3::y(),
            top_radius_on_cone.clone(),
            start.clone(),
            end.clone(),
        )?)?;
        top_edges.push(builder.edge(
            vertices[index + 4],
            vertices[next + 4],
            top_curve,
            ParameterDomain::new(start, end)?,
        )?);
    }
    let mut generators = Vec::with_capacity(4);
    for index in 0..4 {
        let curve = builder.curve(Curve3::line(
            points[index].clone(),
            points[index + 4].clone(),
        )?)?;
        generators.push(builder.edge(
            vertices[index],
            vertices[index + 4],
            curve,
            ParameterDomain::unit(),
        )?);
    }

    let center = CurvePoint2::new(Real::zero(), Real::zero());
    let bottom_surface =
        builder.surface(Surface::plane(bottom_center, Vector3::x(), Vector3::y())?)?;
    let mut bottom_uses = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
            CircularArc2::try_from_center(
                CurvePoint2::new(points[index].x.clone(), points[index].y.clone()),
                CurvePoint2::new(points[next].x.clone(), points[next].y.clone()),
                center.clone(),
                true,
            )
            .map_err(GeometryError::from)?,
        )))?;
        bottom_uses.push(builder.edge_use(
            bottom_edges[index],
            Direction::Forward,
            pcurve,
            ParameterCorrespondence::angular_sweep(),
        )?);
    }
    let bottom_wire = builder.wire(bottom_uses)?;
    let bottom_face = builder.face(
        bottom_surface,
        Orientation::Reversed,
        bottom_wire,
        Vec::new(),
    )?;

    let top_surface = builder.surface(Surface::plane(top_center, Vector3::x(), Vector3::y())?)?;
    let mut top_uses = Vec::with_capacity(4);
    for index in (0..4).rev() {
        let next = (index + 1) % 4;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
            CircularArc2::try_from_center(
                CurvePoint2::new(points[next + 4].x.clone(), points[next + 4].y.clone()),
                CurvePoint2::new(points[index + 4].x.clone(), points[index + 4].y.clone()),
                center.clone(),
                false,
            )
            .map_err(GeometryError::from)?,
        )))?;
        top_uses.push(builder.edge_use(
            top_edges[index],
            Direction::Reversed,
            pcurve,
            ParameterCorrespondence::angular_sweep(),
        )?);
    }
    let top_wire = builder.wire(top_uses)?;
    let top_face = builder.face(top_surface, Orientation::Forward, top_wire, Vec::new())?;

    let mut side_faces = Vec::with_capacity(4);
    for index in 0..4 {
        let next = (index + 1) % 4;
        let u_start = angles[index].clone();
        let u_end = &quarter * Real::from((index + 1) as i32);
        let specs = [
            (
                top_edges[index],
                Direction::Forward,
                CurvePoint2::new(u_start.clone(), v_top.clone()),
                CurvePoint2::new(u_end.clone(), v_top.clone()),
                ParameterCorrespondence::affine(quarter.clone(), u_start.clone())?,
            ),
            (
                generators[next],
                Direction::Reversed,
                CurvePoint2::new(u_end.clone(), v_top.clone()),
                CurvePoint2::new(u_end.clone(), v_bottom.clone()),
                ParameterCorrespondence::affine(-Real::one(), Real::one())?,
            ),
            (
                bottom_edges[index],
                Direction::Reversed,
                CurvePoint2::new(u_end.clone(), v_bottom.clone()),
                CurvePoint2::new(u_start.clone(), v_bottom.clone()),
                ParameterCorrespondence::affine(-quarter.clone(), u_end)?,
            ),
            (
                generators[index],
                Direction::Forward,
                CurvePoint2::new(u_start.clone(), v_bottom.clone()),
                CurvePoint2::new(u_start, v_top.clone()),
                ParameterCorrespondence::identity(),
            ),
        ];
        let mut uses = Vec::with_capacity(4);
        for (edge, direction, start, end, correspondence) in specs {
            let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
            )))?;
            uses.push(builder.edge_use(edge, direction, pcurve, correspondence)?);
        }
        let wire = builder.wire(uses)?;
        side_faces.push(builder.face(cone_surface, Orientation::Forward, wire, Vec::new())?);
    }
    let mut faces = vec![bottom_face, top_face];
    faces.extend(side_faces);
    let shell = builder.shell(faces)?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

/// Builds a standard z-axis ring torus centered at the origin.
///
/// Sixteen exact parameter patches share 32 native circular edges on one
/// analytic torus surface. `major_radius` must be strictly greater than the
/// positive `minor_radius`.
pub fn torus(
    major_radius: Real,
    minor_radius: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    let surface = Surface::torus(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        major_radius.clone(),
        minor_radius.clone(),
    )?;
    let quarter = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let angles = (0..4)
        .map(|index| &quarter * Real::from(index))
        .collect::<Vec<_>>();
    let mut builder = ModelBuilder::new();
    let surface_id = builder.surface(surface.clone())?;
    let mut points = Vec::with_capacity(16);
    for u in &angles {
        for v in &angles {
            points.push(surface.point_at(&Point2::new(u.clone(), v.clone()))?);
        }
    }
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let index = |u: usize, v: usize| u * 4 + v;

    let mut u_edges = Vec::with_capacity(16);
    for u_index in 0..4 {
        let next_u = (u_index + 1) % 4;
        let u_start = angles[u_index].clone();
        let u_end = &quarter * Real::from((u_index + 1) as i32);
        for (v_index, v) in angles.iter().enumerate() {
            let center = Point3::origin() + Vector3::z() * (&minor_radius * v.clone().sin());
            let radius = &major_radius + &minor_radius * v.clone().cos();
            let curve = builder.curve(Curve3::circle_arc(
                center,
                Vector3::x(),
                Vector3::y(),
                radius,
                u_start.clone(),
                u_end.clone(),
            )?)?;
            u_edges.push(builder.edge(
                vertices[index(u_index, v_index)],
                vertices[index(next_u, v_index)],
                curve,
                ParameterDomain::new(u_start.clone(), u_end.clone())?,
            )?);
        }
    }

    let mut v_edges = Vec::with_capacity(16);
    for (u_index, u) in angles.iter().enumerate() {
        let radial = Vector3::x() * u.clone().cos() + Vector3::y() * u.clone().sin();
        let center = Point3::origin() + radial.clone() * &major_radius;
        for v_index in 0..4 {
            let next_v = (v_index + 1) % 4;
            let v_start = angles[v_index].clone();
            let v_end = &quarter * Real::from((v_index + 1) as i32);
            let curve = builder.curve(Curve3::circle_arc(
                center.clone(),
                radial.clone(),
                Vector3::z(),
                minor_radius.clone(),
                v_start.clone(),
                v_end.clone(),
            )?)?;
            v_edges.push(builder.edge(
                vertices[index(u_index, v_index)],
                vertices[index(u_index, next_v)],
                curve,
                ParameterDomain::new(v_start, v_end)?,
            )?);
        }
    }

    let mut faces = Vec::with_capacity(16);
    for u_index in 0..4 {
        let next_u = (u_index + 1) % 4;
        let u_start = angles[u_index].clone();
        let u_end = &quarter * Real::from((u_index + 1) as i32);
        for v_index in 0..4 {
            let next_v = (v_index + 1) % 4;
            let v_start = angles[v_index].clone();
            let v_end = &quarter * Real::from((v_index + 1) as i32);
            let specs = [
                (
                    u_edges[index(u_index, v_index)],
                    Direction::Forward,
                    CurvePoint2::new(u_start.clone(), v_start.clone()),
                    CurvePoint2::new(u_end.clone(), v_start.clone()),
                    ParameterCorrespondence::affine(quarter.clone(), u_start.clone())?,
                ),
                (
                    v_edges[index(next_u, v_index)],
                    Direction::Forward,
                    CurvePoint2::new(u_end.clone(), v_start.clone()),
                    CurvePoint2::new(u_end.clone(), v_end.clone()),
                    ParameterCorrespondence::affine(quarter.clone(), v_start.clone())?,
                ),
                (
                    u_edges[index(u_index, next_v)],
                    Direction::Reversed,
                    CurvePoint2::new(u_end.clone(), v_end.clone()),
                    CurvePoint2::new(u_start.clone(), v_end.clone()),
                    ParameterCorrespondence::affine(-quarter.clone(), u_end.clone())?,
                ),
                (
                    v_edges[index(u_index, v_index)],
                    Direction::Reversed,
                    CurvePoint2::new(u_start.clone(), v_end.clone()),
                    CurvePoint2::new(u_start.clone(), v_start.clone()),
                    ParameterCorrespondence::affine(-quarter.clone(), v_end.clone())?,
                ),
            ];
            let mut uses = Vec::with_capacity(4);
            for (edge, direction, start, end, correspondence) in specs {
                let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
                )))?;
                uses.push(builder.edge_use(edge, direction, pcurve, correspondence)?);
            }
            let wire = builder.wire(uses)?;
            faces.push(builder.face(surface_id, Orientation::Forward, wire, Vec::new())?);
        }
    }
    let shell = builder.shell(faces)?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

/// Extrudes one exact simple line/arc contour between two z coordinates.
///
/// Native circular segments remain circular on the caps and generate exact
/// extrusion surfaces on the sides. The contour is normalized to
/// counterclockwise orientation; no chords or sampled replacements are
/// authored.
pub fn extrude_contour(
    contour: &Contour2,
    z_min: Real,
    z_max: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    let (model, solids) = extrude_contour_regions(&[(contour.clone(), Vec::new())], z_min, z_max)?;
    Ok((model, solids[0]))
}

/// Extrudes disjoint exact line/arc regions into one validated model.
///
/// Each tuple contains one material contour and its disjoint hole contours.
/// Material contours are normalized counterclockwise and holes clockwise while
/// preserving every native segment family.
pub fn extrude_contour_regions(
    regions: &[(Contour2, Vec<Contour2>)],
    z_min: Real,
    z_max: Real,
) -> Result<(Model, Vec<SolidId>), ConstructionError> {
    require_increasing(&z_min, &z_max, Axis::Z)?;
    let normalized = regions
        .iter()
        .map(|(outer, holes)| {
            let outer = normalize_contour(outer, true)?;
            let holes = holes
                .iter()
                .map(|hole| normalize_contour(hole, false))
                .collect::<Result<Vec<_>, _>>()?;
            validate_contour_nesting(&outer, &holes)?;
            Ok((outer, holes))
        })
        .collect::<Result<Vec<_>, ConstructionError>>()?;
    let mut builder = ModelBuilder::new();
    let mut solids = Vec::with_capacity(normalized.len());
    for (outer, holes) in normalized {
        let mut loops = Vec::with_capacity(holes.len() + 1);
        loops.push(curve_path_from_contour(&outer)?);
        loops.extend(
            holes
                .iter()
                .map(curve_path_from_contour)
                .collect::<Result<Vec<_>, _>>()?,
        );
        solids.push(add_curve_path_region(
            &mut builder,
            &loops,
            z_min.clone(),
            z_max.clone(),
        )?);
    }
    Ok((builder.finish()?, solids))
}

/// Extrudes one exact simple [`CurvePath2`] between two z coordinates.
///
/// The path is normalized counterclockwise before its native line, circular,
/// rational Bézier, and NURBS carriers are authored. Polynomial carriers are
/// promoted exactly to persistence-supported rational Bézier or NURBS
/// carriers. No fitting, sampling, or direct arena manipulation is required.
pub fn extrude_path(
    path: &CurvePath2,
    z_min: Real,
    z_max: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    extrude_path_region(path, &[], z_min, z_max)
}

/// Extrudes one exact [`CurvePath2`] material region with disjoint holes.
pub fn extrude_path_region(
    outer: &CurvePath2,
    holes: &[CurvePath2],
    z_min: Real,
    z_max: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    let (model, solids) = extrude_path_regions(&[(outer.clone(), holes.to_vec())], z_min, z_max)?;
    Ok((model, solids[0]))
}

/// Extrudes disjoint exact [`CurvePath2`] regions into one validated model.
///
/// Returned solid IDs follow input order. Every outer path is normalized
/// counterclockwise, every hole clockwise, and all pairwise contact,
/// containment, and hole-nesting decisions are certified before topology is
/// authored.
pub fn extrude_path_regions(
    regions: &[(CurvePath2, Vec<CurvePath2>)],
    z_min: Real,
    z_max: Real,
) -> Result<(Model, Vec<SolidId>), ConstructionError> {
    require_increasing(&z_min, &z_max, Axis::Z)?;
    let normalized = regions
        .iter()
        .map(|(outer, holes)| {
            let outer = normalize_planar_path(outer, true)?;
            let holes = holes
                .iter()
                .map(|hole| normalize_planar_path(hole, false))
                .collect::<Result<Vec<_>, _>>()?;
            validate_planar_path_nesting(&outer, &holes)?;
            Ok((outer, holes))
        })
        .collect::<Result<Vec<_>, ConstructionError>>()?;
    let mut builder = ModelBuilder::new();
    let mut solids = Vec::with_capacity(normalized.len());
    for (outer, holes) in normalized {
        let mut loops = Vec::with_capacity(holes.len() + 1);
        loops.push(outer);
        loops.extend(holes);
        solids.push(add_curve_path_region(
            &mut builder,
            &loops,
            z_min.clone(),
            z_max.clone(),
        )?);
    }
    Ok((builder.finish()?, solids))
}

fn curve_path_from_contour(contour: &Contour2) -> Result<CurvePath2, ConstructionError> {
    CurvePath2::try_new(contour.segments().iter().map(curve2_from_segment).collect())
        .map_err(GeometryError::from)
        .map_err(Into::into)
}

fn normalize_contour(
    contour: &Contour2,
    counterclockwise: bool,
) -> Result<Contour2, ConstructionError> {
    if contour.segments().len() < 2 {
        return Err(ConstructionError::ProfileTooSmall);
    }
    if !contour
        .intersect_self(&CurvePolicy::STRICT)
        .map_err(GeometryError::from)?
        .is_empty()
    {
        return Err(ConstructionError::SelfIntersectingProfile);
    }
    let signed_area = contour
        .signed_area()
        .map_err(GeometryError::from)?
        .ok_or(ConstructionError::DegenerateProfile)?;
    let reverse = match compare_reals(&signed_area, &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Less,
            ..
        } => counterclockwise,
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } => !counterclockwise,
        PredicateOutcome::Decided { .. } => return Err(ConstructionError::DegenerateProfile),
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(
                BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into(),
            );
        }
    };
    if reverse {
        Contour2::try_new(
            contour
                .segments()
                .iter()
                .rev()
                .map(Segment2::reversed)
                .collect(),
        )
        .map_err(GeometryError::from)
        .map_err(ConstructionError::from)
    } else {
        Ok(contour.clone())
    }
}

fn validate_contour_nesting(outer: &Contour2, holes: &[Contour2]) -> Result<(), ConstructionError> {
    let policy = CurvePolicy::STRICT;
    for hole in holes {
        if !outer
            .intersect_contour(hole, &policy)
            .map_err(GeometryError::from)?
            .is_empty()
        {
            return Err(ConstructionError::IntersectingProfiles);
        }
        match outer.classify_point(hole.segments()[0].start(), &policy) {
            Classification::Decided(ContourPointLocation::Inside) => {}
            Classification::Decided(_) => return Err(ConstructionError::HoleOutside),
            Classification::Uncertain(reason) => {
                return Err(
                    BuildError::Geometry(GeometryError::PlanarClassificationUnresolved(reason))
                        .into(),
                );
            }
        }
    }
    for first in 0..holes.len() {
        for second in first + 1..holes.len() {
            if !holes[first]
                .intersect_contour(&holes[second], &policy)
                .map_err(GeometryError::from)?
                .is_empty()
            {
                return Err(ConstructionError::IntersectingProfiles);
            }
            for (container, point) in [
                (&holes[first], holes[second].segments()[0].start()),
                (&holes[second], holes[first].segments()[0].start()),
            ] {
                match container.classify_point(point, &policy) {
                    Classification::Decided(ContourPointLocation::Inside) => {
                        return Err(ConstructionError::NestedHoles);
                    }
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        return Err(BuildError::Geometry(
                            GeometryError::PlanarClassificationUnresolved(reason),
                        )
                        .into());
                    }
                }
            }
        }
    }
    Ok(())
}

fn add_curve_path_region(
    builder: &mut ModelBuilder,
    loops: &[CurvePath2],
    z_min: Real,
    z_max: Real,
) -> Result<SolidId, ConstructionError> {
    let loop_curves = loops
        .iter()
        .map(persistent_extrusion_path_curves)
        .collect::<Result<Vec<_>, _>>()?;
    let loop_offsets = loop_curves
        .iter()
        .scan(0_usize, |offset, curves| {
            let current = *offset;
            *offset += curves.len();
            Some(current)
        })
        .collect::<Vec<_>>();
    let count = loop_curves.iter().map(Vec::len).sum::<usize>();
    let mut next_indices = vec![0_usize; count];
    for (curves, offset) in loop_curves.iter().zip(&loop_offsets) {
        for local in 0..curves.len() {
            next_indices[offset + local] = offset + (local + 1) % curves.len();
        }
    }
    let curves = loop_curves.iter().flatten().cloned().collect::<Vec<_>>();
    let points_2d = curves
        .iter()
        .map(|curve| curve.start().clone())
        .collect::<Vec<_>>();
    let mut points = Vec::with_capacity(points_2d.len() * 2);
    points.extend(
        points_2d
            .iter()
            .map(|point| Point3::new(point.x().clone(), point.y().clone(), z_min.clone())),
    );
    points.extend(
        points_2d
            .iter()
            .map(|point| Point3::new(point.x().clone(), point.y().clone(), z_max.clone())),
    );
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let mut bottom_curves = Vec::with_capacity(count);
    let mut bottom_edges = Vec::with_capacity(count);
    let mut top_edges = Vec::with_capacity(count);
    let mut domains = Vec::with_capacity(count);
    for (index, pcurve) in curves.iter().enumerate() {
        let next = next_indices[index];
        let bottom_curve = spatial_extrusion_curve(pcurve, &z_min)?;
        let domain = bottom_curve.domain().clone();
        let bottom_curve_id = builder.curve(bottom_curve.clone())?;
        bottom_edges.push(builder.edge(
            vertices[index],
            vertices[next],
            bottom_curve_id,
            domain.clone(),
        )?);
        let top_curve = spatial_extrusion_curve(pcurve, &z_max)?;
        let top_domain = top_curve.domain().clone();
        let top_curve_id = builder.curve(top_curve)?;
        top_edges.push(builder.edge(
            vertices[index + count],
            vertices[next + count],
            top_curve_id,
            top_domain,
        )?);
        bottom_curves.push(bottom_curve);
        domains.push(domain);
    }
    let mut axial_edges = Vec::with_capacity(count);
    for index in 0..count {
        let curve = builder.curve(Curve3::line(
            points[index].clone(),
            points[index + count].clone(),
        )?)?;
        axial_edges.push(builder.edge(
            vertices[index],
            vertices[index + count],
            curve,
            ParameterDomain::unit(),
        )?);
    }

    let bottom_surface = builder.surface(Surface::plane(
        Point3::new(Real::zero(), Real::zero(), z_min.clone()),
        Vector3::x(),
        Vector3::y(),
    )?)?;
    let mut bottom_wires = Vec::with_capacity(loops.len());
    for (loop_curves, offset) in loop_curves.iter().zip(&loop_offsets) {
        let mut uses = Vec::with_capacity(loop_curves.len());
        for local in (0..loop_curves.len()).rev() {
            let index = offset + local;
            let pcurve = builder.pcurve(Pcurve::new(
                curves[index].reversed().map_err(GeometryError::from)?,
            ))?;
            let correspondence = planar_curve_correspondence(&curves[index], Direction::Reversed)?;
            uses.push(builder.edge_use(
                bottom_edges[index],
                Direction::Reversed,
                pcurve,
                correspondence,
            )?);
        }
        bottom_wires.push(builder.wire(uses)?);
    }
    let bottom_face = builder.face(
        bottom_surface,
        Orientation::Reversed,
        bottom_wires[0],
        bottom_wires[1..].to_vec(),
    )?;

    let top_surface = builder.surface(Surface::plane(
        Point3::new(Real::zero(), Real::zero(), z_max.clone()),
        Vector3::x(),
        Vector3::y(),
    )?)?;
    let mut top_wires = Vec::with_capacity(loops.len());
    for (loop_curves, offset) in loop_curves.iter().zip(&loop_offsets) {
        let mut uses = Vec::with_capacity(loop_curves.len());
        for local in 0..loop_curves.len() {
            let index = offset + local;
            let pcurve = builder.pcurve(Pcurve::new(curves[index].clone()))?;
            uses.push(builder.edge_use(
                top_edges[index],
                Direction::Forward,
                pcurve,
                planar_curve_correspondence(&curves[index], Direction::Forward)?,
            )?);
        }
        top_wires.push(builder.wire(uses)?);
    }
    let top_face = builder.face(
        top_surface,
        Orientation::Forward,
        top_wires[0],
        top_wires[1..].to_vec(),
    )?;

    let height = &z_max - &z_min;
    let mut side_faces = Vec::with_capacity(count);
    for index in 0..count {
        let next = next_indices[index];
        let domain = &domains[index];
        let surface = builder.surface(Surface::extrusion(
            bottom_curves[index].clone(),
            Vector3::z(),
        )?)?;
        let specs = [
            (
                bottom_edges[index],
                Direction::Forward,
                CurvePoint2::new(domain.start().clone(), Real::zero()),
                CurvePoint2::new(domain.end().clone(), Real::zero()),
                ParameterCorrespondence::affine(
                    domain.end() - domain.start(),
                    domain.start().clone(),
                )?,
            ),
            (
                axial_edges[next],
                Direction::Forward,
                CurvePoint2::new(domain.end().clone(), Real::zero()),
                CurvePoint2::new(domain.end().clone(), height.clone()),
                ParameterCorrespondence::identity(),
            ),
            (
                top_edges[index],
                Direction::Reversed,
                CurvePoint2::new(domain.end().clone(), height.clone()),
                CurvePoint2::new(domain.start().clone(), height.clone()),
                ParameterCorrespondence::affine(
                    domain.start() - domain.end(),
                    domain.end().clone(),
                )?,
            ),
            (
                axial_edges[index],
                Direction::Reversed,
                CurvePoint2::new(domain.start().clone(), height.clone()),
                CurvePoint2::new(domain.start().clone(), Real::zero()),
                ParameterCorrespondence::affine(-Real::one(), Real::one())?,
            ),
        ];
        let mut uses = Vec::with_capacity(4);
        for (edge, direction, start, end, correspondence) in specs {
            let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
            )))?;
            uses.push(builder.edge_use(edge, direction, pcurve, correspondence)?);
        }
        let wire = builder.wire(uses)?;
        side_faces.push(builder.face(surface, Orientation::Forward, wire, Vec::new())?);
    }
    let mut faces = vec![bottom_face, top_face];
    faces.extend(side_faces);
    let shell = builder.shell(faces)?;
    Ok(builder.solid(shell, Vec::new())?)
}

fn spatial_segment_curve(
    segment: &Segment2,
    z: &Real,
) -> Result<(Curve3, ParameterDomain), ConstructionError> {
    match segment {
        Segment2::Line(line) => Ok((
            Curve3::line(
                Point3::new(
                    line.start().x().clone(),
                    line.start().y().clone(),
                    z.clone(),
                ),
                Point3::new(line.end().x().clone(), line.end().y().clone(), z.clone()),
            )?,
            ParameterDomain::unit(),
        )),
        Segment2::Arc(arc) => {
            let radius = arc
                .radius_squared()
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let radial_x = arc.start().x() - arc.center().x();
            let radial_y = arc.start().y() - arc.center().y();
            let x = Vector3::from_xyz(
                (radial_x.clone() / &radius).map_err(|_| GeometryError::ProjectiveDivision)?,
                (radial_y.clone() / &radius).map_err(|_| GeometryError::ProjectiveDivision)?,
                Real::zero(),
            );
            let (tangent_x, tangent_y) = if arc.is_clockwise() {
                (radial_y, -radial_x)
            } else {
                (-radial_y, radial_x)
            };
            let y = Vector3::from_xyz(
                (tangent_x / &radius).map_err(|_| GeometryError::ProjectiveDivision)?,
                (tangent_y / &radius).map_err(|_| GeometryError::ProjectiveDivision)?,
                Real::zero(),
            );
            let sweep = match arc.directed_sweep_angle().map_err(GeometryError::from)? {
                Classification::Decided(sweep) => sweep,
                Classification::Uncertain(reason) => {
                    return Err(BuildError::Geometry(
                        GeometryError::PlanarClassificationUnresolved(reason),
                    )
                    .into());
                }
            };
            let domain = ParameterDomain::new(Real::zero(), sweep.clone())?;
            Ok((
                Curve3::circle_arc(
                    Point3::new(
                        arc.center().x().clone(),
                        arc.center().y().clone(),
                        z.clone(),
                    ),
                    x,
                    y,
                    radius,
                    Real::zero(),
                    sweep,
                )?,
                domain,
            ))
        }
    }
}

fn curve2_from_segment(segment: &Segment2) -> Curve2 {
    match segment {
        Segment2::Line(line) => Curve2::from(line.clone()),
        Segment2::Arc(arc) => Curve2::from(arc.clone()),
    }
}

/// Revolves one exact simple radial/axial polygon around the z axis.
///
/// Profile `x` is radius and must remain strictly positive; profile `y` is
/// axial position. Clockwise input is normalized. Every line segment owns one
/// revolution carrier split into four exact angular faces. The periodic seam
/// shares meridian edges by identity rather than tolerance sewing.
pub fn revolve(profile: &[Point2]) -> Result<(Model, SolidId), ConstructionError> {
    let profile = normalize_profile(profile, true)?;
    validate_revolution_profile_radius(&profile)?;
    let mut builder = ModelBuilder::new();
    let shell = add_normalized_revolution_shell(&mut builder, &profile, ShellDirection::Outward)?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

/// Revolves one exact simple radial/axial region around the z axis.
///
/// The outer loop and every hole must stay strictly at positive radius.
/// Holes become identity-independent inward shells and remain exact material
/// cavities through measurement, classification, transforms, and persistence.
pub fn revolve_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
) -> Result<(Model, SolidId), ConstructionError> {
    let outer = normalize_profile(outer, true)?;
    let holes = holes
        .iter()
        .map(|hole| normalize_profile(hole, true))
        .collect::<Result<Vec<_>, _>>()?;
    validate_revolution_profile_radius(&outer)?;
    for hole in &holes {
        validate_revolution_profile_radius(hole)?;
    }
    validate_profile_nesting(&outer, &holes)?;
    let mut builder = ModelBuilder::new();
    let outer_shell =
        add_normalized_revolution_shell(&mut builder, &outer, ShellDirection::Outward)?;
    let voids = holes
        .iter()
        .map(|hole| add_normalized_revolution_shell(&mut builder, hole, ShellDirection::Inward))
        .collect::<Result<Vec<_>, _>>()?;
    let solid = builder.solid(outer_shell, voids)?;
    Ok((builder.finish()?, solid))
}

/// Revolves one exact simple line/arc contour around the z axis.
///
/// Profile `x` is radius and the complete contour must remain strictly
/// positive. Every native line or circular-arc segment owns four periodic
/// revolution faces. Curved meridians remain native circles; no flattening or
/// tolerance sewing enters construction.
pub fn revolve_contour(profile: &Contour2) -> Result<(Model, SolidId), ConstructionError> {
    let profile = normalize_revolution_contour(profile)?;
    let mut builder = ModelBuilder::new();
    let shell =
        add_normalized_contour_revolution_shell(&mut builder, &profile, ShellDirection::Outward)?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

/// Revolves one exact closed curved path around the z axis.
///
/// Profile `x` is radius and must remain strictly positive. Every authored
/// line, circular arc, Bézier, polynomial B-spline, or finite NURBS carrier is
/// retained as the meridian of four periodic revolution faces. The preflight
/// requires a complete exact simple-loop and orientation certificate; it never
/// flattens the path to manufacture one.
pub fn revolve_path(profile: &CurvePath2) -> Result<(Model, SolidId), ConstructionError> {
    let profile = normalize_revolution_path(profile)?;
    let mut builder = ModelBuilder::new();
    let shell = add_normalized_curve_path_revolution_shell(
        &mut builder,
        &profile,
        ShellDirection::Outward,
    )?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

/// Revolves one exact curved-path region with inward profile cavities.
///
/// Every loop uses the same exact simple-path and positive-radius preflight as
/// [`revolve_path`]. Shell nesting is certified from the retained curved
/// boundaries before the solid is published.
pub fn revolve_path_region(
    outer: &CurvePath2,
    holes: &[CurvePath2],
) -> Result<(Model, SolidId), ConstructionError> {
    let outer = normalize_revolution_path(outer)?;
    let holes = holes
        .iter()
        .map(normalize_revolution_path)
        .collect::<Result<Vec<_>, _>>()?;
    let mut builder = ModelBuilder::new();
    let outer_shell =
        add_normalized_curve_path_revolution_shell(&mut builder, &outer, ShellDirection::Outward)?;
    let voids = holes
        .iter()
        .map(|hole| {
            add_normalized_curve_path_revolution_shell(&mut builder, hole, ShellDirection::Inward)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let solid = builder.solid(outer_shell, voids)?;
    Ok((builder.finish()?, solid))
}

/// Revolves one exact line/arc region with inward profile cavities.
pub fn revolve_contour_region(
    outer: &Contour2,
    holes: &[Contour2],
) -> Result<(Model, SolidId), ConstructionError> {
    let outer = normalize_revolution_contour(outer)?;
    let holes = holes
        .iter()
        .map(normalize_revolution_contour)
        .collect::<Result<Vec<_>, _>>()?;
    validate_contour_nesting(&outer, &holes)?;
    let mut builder = ModelBuilder::new();
    let outer_shell =
        add_normalized_contour_revolution_shell(&mut builder, &outer, ShellDirection::Outward)?;
    let voids = holes
        .iter()
        .map(|hole| {
            add_normalized_contour_revolution_shell(&mut builder, hole, ShellDirection::Inward)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let solid = builder.solid(outer_shell, voids)?;
    Ok((builder.finish()?, solid))
}

/// Sweeps one exact simple planar polygon along one exact linear path.
///
/// `origin + u*x + v*y` embeds the profile in model space and `path` carries
/// it to the terminal section. The three directions must be linearly
/// independent; shear and nonuniform scale are retained exactly. This is the
/// linear-path sweep contract: no implicit moving-frame or corner policy is
/// invented for a curved path.
pub fn sweep(
    profile: &[Point2],
    origin: Point3,
    u: Vector3,
    v: Vector3,
    path: Vector3,
) -> Result<(Model, SolidId), ConstructionError> {
    let (model, solid) = extrude(profile, Real::zero(), Real::one())?;
    place_linear_sweep(model, solid, origin, u, v, path)
}

/// Sweeps one exact planar polygonal region with through-holes along a line.
///
/// This has the same explicit affine frame and linear-path contract as
/// [`sweep`]. Hole topology remains part of the swept shell rather than being
/// approximated or Booleaned after construction.
pub fn sweep_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    origin: Point3,
    u: Vector3,
    v: Vector3,
    path: Vector3,
) -> Result<(Model, SolidId), ConstructionError> {
    let (model, solid) = extrude_region(outer, holes, Real::zero(), Real::one())?;
    place_linear_sweep(model, solid, origin, u, v, path)
}

/// Sweeps one exact polygon along a rational Bézier path in a fixed frame.
///
/// `path(t) + u*x + v*y` embeds every section. The path is the absolute locus
/// of the profile origin and must advance affinely and strictly positively
/// through the oriented profile plane: normalized plane progress is therefore
/// exactly the path's public parameter. Lateral path curvature is unrestricted.
/// Side faces are native tensor rational Bézier translation surfaces and no
/// moving-frame, sampling, or corner policy is inferred.
pub fn sweep_curve(
    profile: &[Point2],
    u: Vector3,
    v: Vector3,
    path: Curve3,
) -> Result<(Model, SolidId), ConstructionError> {
    sweep_curve_region(profile, &[], u, v, path)
}

/// Sweeps one exact polygonal region with through-holes along a rational
/// Bézier path in a fixed frame.
///
/// The outer loop and holes obey the same exact nesting contract as
/// [`extrude_region`]. All loops share the authored path and fixed `u`/`v`
/// frame. The result is one genus shell: cap holes are inner wires and their
/// tensor side walls face the removed material. No detached void shell,
/// moving frame, sampling, or corner policy is inferred.
pub fn sweep_curve_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    u: Vector3,
    v: Vector3,
    path: Curve3,
) -> Result<(Model, SolidId), ConstructionError> {
    let Curve3ExactData::RationalBezier {
        control_points: origins,
        weights,
    } = path.exact_data()
    else {
        return Err(ConstructionError::UnsupportedSweepPath);
    };
    let u_axes = vec![u; origins.len()];
    let v_axes = vec![v; origins.len()];
    sweep_rational_bezier_frame_region(outer, holes, origins, u_axes, v_axes, weights)
}

/// Sweeps one exact polygon through an explicitly authored rational Bézier
/// moving frame.
///
/// The frame supplies the complete origin and profile-axis motion. HyperBREP
/// infers no Frenet frame, corner transport, or sampling policy.
pub fn sweep_moving_frame(
    profile: &[Point2],
    frame: RationalBezierSweepFrame,
) -> Result<(Model, SolidId), ConstructionError> {
    sweep_moving_frame_region(profile, &[], frame)
}

/// Sweeps one exact polygonal region with through-holes through an explicitly
/// authored rational Bézier moving frame.
///
/// The authored frame is accepted only when its complete Bernstein form proves
/// parallel section planes, positive affine plane progress, and an exactly
/// strictly positive supported section-area law. Those restrictions make the
/// resulting shell globally injective without sampling while still permitting
/// exact shear, polynomial taper, and other non-rigid in-plane motion.
pub fn sweep_moving_frame_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    frame: RationalBezierSweepFrame,
) -> Result<(Model, SolidId), ConstructionError> {
    sweep_rational_bezier_frame_region(
        outer,
        holes,
        frame.origins,
        frame.u_axes,
        frame.v_axes,
        frame.weights,
    )
}

fn sweep_rational_bezier_frame_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    origins: Vec<Point3>,
    u_axes: Vec<Vector3>,
    v_axes: Vec<Vector3>,
    weights: Vec<Real>,
) -> Result<(Model, SolidId), ConstructionError> {
    let outer = normalize_profile(outer, true)?;
    let holes = holes
        .iter()
        .map(|hole| normalize_profile(hole, false))
        .collect::<Result<Vec<_>, _>>()?;
    validate_profile_nesting(&outer, &holes)?;
    let mut loops = Vec::with_capacity(holes.len() + 1);
    loops.push(outer);
    loops.extend(holes);
    let (normal, _) = certify_sweep_frame(&origins, &u_axes, &v_axes, &weights)?;
    certify_sweep_path_progress(&origins, &weights, &normal)?;

    let count = loops.iter().map(Vec::len).sum::<usize>();
    let mut loop_offsets = Vec::with_capacity(loops.len());
    let mut offset_index = 0;
    for profile in &loops {
        loop_offsets.push(offset_index);
        offset_index += profile.len();
    }
    let path_start = origins[0].clone();
    let path_end = origins[origins.len() - 1].clone();
    let lower_u = u_axes[0].clone();
    let lower_v = v_axes[0].clone();
    let upper_u = u_axes[u_axes.len() - 1].clone();
    let upper_v = v_axes[v_axes.len() - 1].clone();
    let lower_offset = |point: &Point2| lower_u.clone() * &point.x + lower_v.clone() * &point.y;
    let upper_offset = |point: &Point2| upper_u.clone() * &point.x + upper_v.clone() * &point.y;
    let lower_points = loops
        .iter()
        .flatten()
        .map(|point| path_start.clone() + lower_offset(point))
        .collect::<Vec<_>>();
    let upper_points = loops
        .iter()
        .flatten()
        .map(|point| path_end.clone() + upper_offset(point))
        .collect::<Vec<_>>();

    let mut builder = ModelBuilder::new();
    let lower_vertices = lower_points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let upper_vertices = upper_points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let mut lower_edges = Vec::with_capacity(count);
    let mut upper_edges = Vec::with_capacity(count);
    let mut path_edges = Vec::with_capacity(count);
    for (profile, profile_offset) in loops.iter().zip(&loop_offsets) {
        for local in 0..profile.len() {
            let index = profile_offset + local;
            let next = profile_offset + (local + 1) % profile.len();
            let lower_curve = builder.curve(Curve3::line(
                lower_points[index].clone(),
                lower_points[next].clone(),
            )?)?;
            lower_edges.push(builder.edge(
                lower_vertices[index],
                lower_vertices[next],
                lower_curve,
                ParameterDomain::unit(),
            )?);
            let upper_curve = builder.curve(Curve3::line(
                upper_points[index].clone(),
                upper_points[next].clone(),
            )?)?;
            upper_edges.push(builder.edge(
                upper_vertices[index],
                upper_vertices[next],
                upper_curve,
                ParameterDomain::unit(),
            )?);
            let translated_controls = origins
                .iter()
                .zip(u_axes.iter().zip(&v_axes))
                .map(|(origin, (u, v))| {
                    origin.clone() + u.clone() * &profile[local].x + v.clone() * &profile[local].y
                })
                .collect();
            let translated_path = builder.curve(Curve3::rational_bezier(
                translated_controls,
                weights.clone(),
            )?)?;
            path_edges.push(builder.edge(
                lower_vertices[index],
                upper_vertices[index],
                translated_path,
                ParameterDomain::unit(),
            )?);
        }
    }

    let lower_surface = builder.surface(Surface::plane(
        path_start,
        lower_u.clone(),
        lower_v.clone(),
    )?)?;
    let mut lower_wires = Vec::with_capacity(loops.len());
    for (profile, profile_offset) in loops.iter().zip(&loop_offsets) {
        let mut lower_uses = Vec::with_capacity(profile.len());
        for local in (0..profile.len()).rev() {
            let index = profile_offset + local;
            let next = (local + 1) % profile.len();
            let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(curve_point(&profile[next]), curve_point(&profile[local]))
                    .map_err(GeometryError::from)?,
            )))?;
            lower_uses.push(builder.edge_use(
                lower_edges[index],
                Direction::Reversed,
                pcurve,
                ParameterCorrespondence::affine(-Real::one(), Real::one())?,
            )?);
        }
        lower_wires.push(builder.wire(lower_uses)?);
    }
    let lower_outer = lower_wires.remove(0);
    let lower_face = builder.face(
        lower_surface,
        Orientation::Reversed,
        lower_outer,
        lower_wires,
    )?;

    let upper_surface =
        builder.surface(Surface::plane(path_end, upper_u.clone(), upper_v.clone())?)?;
    let mut upper_wires = Vec::with_capacity(loops.len());
    for (profile, profile_offset) in loops.iter().zip(&loop_offsets) {
        let mut upper_uses = Vec::with_capacity(profile.len());
        for local in 0..profile.len() {
            let index = profile_offset + local;
            let next = (local + 1) % profile.len();
            let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(curve_point(&profile[local]), curve_point(&profile[next]))
                    .map_err(GeometryError::from)?,
            )))?;
            upper_uses.push(builder.edge_use(
                upper_edges[index],
                Direction::Forward,
                pcurve,
                ParameterCorrespondence::identity(),
            )?);
        }
        upper_wires.push(builder.wire(upper_uses)?);
    }
    let upper_outer = upper_wires.remove(0);
    let upper_face = builder.face(
        upper_surface,
        Orientation::Forward,
        upper_outer,
        upper_wires,
    )?;

    let parameter_corners = [
        CurvePoint2::new(Real::zero(), Real::zero()),
        CurvePoint2::new(Real::one(), Real::zero()),
        CurvePoint2::new(Real::one(), Real::one()),
        CurvePoint2::new(Real::zero(), Real::one()),
    ];
    let mut faces = vec![lower_face, upper_face];
    for (profile, profile_offset) in loops.iter().zip(&loop_offsets) {
        for local in 0..profile.len() {
            let index = profile_offset + local;
            let next = profile_offset + (local + 1) % profile.len();
            let start = &profile[local];
            let end = &profile[(local + 1) % profile.len()];
            let surface_controls = origins
                .iter()
                .zip(u_axes.iter().zip(&v_axes))
                .map(|(origin, (u, v))| {
                    vec![
                        origin.clone() + u.clone() * &start.x + v.clone() * &start.y,
                        origin.clone() + u.clone() * &end.x + v.clone() * &end.y,
                    ]
                })
                .collect::<Vec<_>>();
            let surface_weights = weights
                .iter()
                .map(|weight| vec![weight.clone(), weight.clone()])
                .collect::<Vec<_>>();
            let surface =
                builder.surface(Surface::rational_bezier(surface_controls, surface_weights)?)?;
            let specs = [
                (lower_edges[index], Direction::Forward, 0, 1),
                (path_edges[next], Direction::Forward, 1, 2),
                (upper_edges[index], Direction::Reversed, 2, 3),
                (path_edges[index], Direction::Reversed, 3, 0),
            ];
            let mut uses = Vec::with_capacity(4);
            for (edge, direction, start, end) in specs {
                let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(
                        parameter_corners[start].clone(),
                        parameter_corners[end].clone(),
                    )
                    .map_err(GeometryError::from)?,
                )))?;
                uses.push(builder.edge_use(
                    edge,
                    direction,
                    pcurve,
                    match direction {
                        Direction::Forward => ParameterCorrespondence::identity(),
                        Direction::Reversed => {
                            ParameterCorrespondence::affine(-Real::one(), Real::one())?
                        }
                    },
                )?);
            }
            let wire = builder.wire(uses)?;
            faces.push(builder.face(surface, Orientation::Forward, wire, Vec::new())?);
        }
    }
    let shell = builder.shell(faces)?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

fn certify_sweep_frame(
    origins: &[Point3],
    u_axes: &[Vector3],
    v_axes: &[Vector3],
    weights: &[Real],
) -> Result<(Vector3, Real), ConstructionError> {
    if origins.len() < 2
        || origins.len() != u_axes.len()
        || origins.len() != v_axes.len()
        || origins.len() != weights.len()
    {
        return Err(ConstructionError::InvalidSweepFrame);
    }
    Curve3::rational_bezier(origins.to_vec(), weights.to_vec())?;

    let normal = u_axes[0].cross(&v_axes[0]);
    if decided_construction_order(compare_reals(
        &normal.norm_squared(),
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Err(BuildError::Geometry(GeometryError::DegeneratePlaneBasis).into());
    }
    for axis in u_axes.iter().chain(v_axes) {
        if decided_construction_order(compare_reals(
            &axis.dot(&normal),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Err(ConstructionError::NonPlanarSweepFrame);
        }
    }

    let degree = origins.len() - 1;
    let product_degree = degree
        .checked_mul(2)
        .ok_or(ConstructionError::InvalidSweepFrame)?;
    let normal_squared = normal.norm_squared();
    let mut constant_area = true;
    let mut determinant_numerators = Vec::with_capacity(product_degree + 1);
    for product_index in 0..=product_degree {
        let first_min = product_index.saturating_sub(degree);
        let first_max = degree.min(product_index);
        let mut cross_coefficient = Vector3::zero();
        let mut weight_coefficient = Real::zero();
        for first in first_min..=first_max {
            let second = product_index - first;
            let coefficient = bernstein_product_coefficient(degree, first, second)?;
            let weighted_u = u_axes[first].clone() * &weights[first];
            let weighted_v = v_axes[second].clone() * &weights[second];
            cross_coefficient = cross_coefficient + weighted_u.cross(&weighted_v) * &coefficient;
            weight_coefficient += &coefficient * &weights[first] * &weights[second];
        }
        let expected = normal.clone() * weight_coefficient;
        let mut this_constant = true;
        for component in 0..3 {
            if decided_construction_order(compare_reals(
                &cross_coefficient.0[component],
                &expected.0[component],
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
            {
                this_constant = false;
            }
        }
        constant_area &= this_constant;
        let determinant_numerator = (cross_coefficient.dot(&normal) / &normal_squared)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let parallel = normal.clone() * &determinant_numerator;
        for component in 0..3 {
            if decided_construction_order(compare_reals(
                &cross_coefficient.0[component],
                &parallel.0[component],
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
            {
                return Err(ConstructionError::NonPlanarSweepFrame);
            }
        }
        determinant_numerators.push(determinant_numerator);
    }
    if constant_area {
        return Ok((normal, Real::one()));
    }
    for weight in &weights[1..] {
        if decided_construction_order(compare_reals(weight, &weights[0], crate::STRICT_PREDICATES))?
            != std::cmp::Ordering::Equal
        {
            return Err(ConstructionError::UnsupportedRationalSweepFrameArea);
        }
    }
    let weight_squared = &weights[0] * &weights[0];
    let mut integral = Real::zero();
    for numerator in determinant_numerators {
        let coefficient =
            (numerator / &weight_squared).map_err(|_| GeometryError::ProjectiveDivision)?;
        if decided_construction_order(compare_reals(
            &coefficient,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Err(ConstructionError::NonPositiveSweepFrameArea);
        }
        integral += coefficient;
    }
    let basis_count =
        Real::from(u128::try_from(product_degree + 1).expect("usize is representable as u128"));
    Ok((
        normal,
        (integral / basis_count).map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

fn bernstein_product_coefficient(
    degree: usize,
    first: usize,
    second: usize,
) -> Result<Real, ConstructionError> {
    let numerator = binomial_real(degree, first)? * binomial_real(degree, second)?;
    (numerator
        / binomial_real(
            degree
                .checked_mul(2)
                .ok_or(ConstructionError::InvalidSweepFrame)?,
            first + second,
        )?)
    .map_err(|_| GeometryError::ProjectiveDivision.into())
}

fn binomial_real(n: usize, k: usize) -> Result<Real, ConstructionError> {
    let k = k.min(n - k);
    let mut result = Real::one();
    for index in 1..=k {
        result = (result
            * Real::from(u128::try_from(n + 1 - index).expect("usize is representable as u128"))
            / Real::from(u128::try_from(index).expect("usize is representable as u128")))
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    }
    Ok(result)
}

fn certify_sweep_path_progress(
    controls: &[Point3],
    weights: &[Real],
    normal: &Vector3,
) -> Result<Real, ConstructionError> {
    let degree = controls
        .len()
        .checked_sub(1)
        .ok_or(ConstructionError::UnsupportedSweepPath)?;
    let scalar = |point: &Point3| normal.dot(&Vector3::from(point.clone()));
    let start = scalar(&controls[0]);
    let end = scalar(&controls[degree]);
    let progress = &end - &start;
    if decided_construction_order(compare_reals(
        &progress,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? != std::cmp::Ordering::Greater
    {
        return Err(ConstructionError::NonMonotoneSweepPath);
    }
    let denominator =
        Real::from(u128::try_from(degree + 1).expect("usize is representable as u128"));
    let numerators = controls
        .iter()
        .zip(weights)
        .map(|(control, weight)| weight * scalar(control))
        .collect::<Vec<_>>();
    for index in 0..=degree + 1 {
        let lower_count =
            Real::from(u128::try_from(degree + 1 - index).expect("usize is representable as u128"));
        let upper_count =
            Real::from(u128::try_from(index).expect("usize is representable as u128"));
        let elevated_numerator = (if index <= degree {
            &lower_count * &numerators[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * &numerators[index - 1]
        } else {
            Real::zero()
        };
        let affine_times_weight = (if index <= degree {
            &lower_count * &start * &weights[index]
        } else {
            Real::zero()
        }) + if index > 0 {
            &upper_count * &end * &weights[index - 1]
        } else {
            Real::zero()
        };
        let difference = (elevated_numerator / &denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)?
            - (affine_times_weight / &denominator)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
        if decided_construction_order(compare_reals(
            &difference,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Err(ConstructionError::NonMonotoneSweepPath);
        }
    }
    Ok(progress)
}

fn place_linear_sweep(
    model: Model,
    solid: SolidId,
    origin: Point3,
    u: Vector3,
    v: Vector3,
    path: Vector3,
) -> Result<(Model, SolidId), ConstructionError> {
    let transform = crate::Matrix4::from_row_major([
        u.0[0].clone(),
        v.0[0].clone(),
        path.0[0].clone(),
        origin.x,
        u.0[1].clone(),
        v.0[1].clone(),
        path.0[1].clone(),
        origin.y,
        u.0[2].clone(),
        v.0[2].clone(),
        path.0[2].clone(),
        origin.z,
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ]);
    Ok((model.transformed(&transform)?, solid))
}

/// Lofts two or more exactly corresponding polygon sections.
///
/// Sections use one common x/y chart, strictly increasing `z`, and one
/// vertex-for-vertex correspondence. Every adjacent span is certified
/// independently. Positive homothetic spans retain planar ruled sides.
/// Otherwise both endpoints and the complete interpolation must pass the
/// sufficient exact strictly-convex certificate; each side is then one native
/// bilinear rational Bézier patch. Intermediate section rings are
/// identity-shared C⁰ seams, not hidden continuity assumptions. No
/// nearest-vertex correspondence, sampling, or tolerance sewing is used.
/// Exact solid volume and classification cover every span; exact area of a
/// general bilinear side remains unsupported.
pub fn loft(sections: &[LoftSection]) -> Result<(Model, SolidId), ConstructionError> {
    if sections.len() < 2 {
        return Err(ConstructionError::LoftNeedsAtLeastTwoSections);
    }
    for pair in sections.windows(2) {
        require_increasing(&pair[0].z, &pair[1].z, Axis::Z)?;
    }
    let profiles = sections
        .iter()
        .map(|section| normalize_profile(&section.profile, true))
        .collect::<Result<Vec<_>, _>>()?;
    let count = profiles[0].len();
    if profiles.iter().any(|profile| profile.len() != count) {
        return Err(ConstructionError::IncompatibleLoftSections);
    }
    let span_scales = profiles
        .windows(2)
        .map(|pair| match certified_loft_scale(&pair[0], &pair[1]) {
            Ok(scale) => Ok(Some(scale)),
            Err(ConstructionError::IncompatibleLoftSections) => {
                certify_convex_loft_correspondence(&pair[0], &pair[1])?;
                Ok(None)
            }
            Err(error) => Err(error),
        })
        .collect::<Result<Vec<_>, ConstructionError>>()?;

    let mut builder = ModelBuilder::new();
    let mut vertices = Vec::with_capacity(sections.len());
    for (section, profile) in sections.iter().zip(&profiles) {
        let mut ring = Vec::with_capacity(count);
        for point in profile {
            ring.push(builder.vertex(Point3::new(
                point.x.clone(),
                point.y.clone(),
                section.z.clone(),
            ))?);
        }
        vertices.push(ring);
    }

    let mut rings = Vec::with_capacity(sections.len());
    for (section_index, profile) in profiles.iter().enumerate() {
        let mut ring = Vec::with_capacity(count);
        for index in 0..count {
            let next = (index + 1) % count;
            let curve = builder.curve(Curve3::line(
                Point3::new(
                    profile[index].x.clone(),
                    profile[index].y.clone(),
                    sections[section_index].z.clone(),
                ),
                Point3::new(
                    profile[next].x.clone(),
                    profile[next].y.clone(),
                    sections[section_index].z.clone(),
                ),
            )?)?;
            ring.push(builder.edge(
                vertices[section_index][index],
                vertices[section_index][next],
                curve,
                ParameterDomain::unit(),
            )?);
        }
        rings.push(ring);
    }

    let mut connectors = Vec::with_capacity(sections.len() - 1);
    for span in 0..sections.len() - 1 {
        let mut span_connectors = Vec::with_capacity(count);
        for index in 0..count {
            let curve = builder.curve(Curve3::line(
                Point3::new(
                    profiles[span][index].x.clone(),
                    profiles[span][index].y.clone(),
                    sections[span].z.clone(),
                ),
                Point3::new(
                    profiles[span + 1][index].x.clone(),
                    profiles[span + 1][index].y.clone(),
                    sections[span + 1].z.clone(),
                ),
            )?)?;
            span_connectors.push(builder.edge(
                vertices[span][index],
                vertices[span + 1][index],
                curve,
                ParameterDomain::unit(),
            )?);
        }
        connectors.push(span_connectors);
    }

    let cap_surface = |builder: &mut ModelBuilder, z: &Real| {
        builder.surface(Surface::plane(
            Point3::new(Real::zero(), Real::zero(), z.clone()),
            Vector3::x(),
            Vector3::y(),
        )?)
    };
    let bottom_surface = cap_surface(&mut builder, &sections[0].z)?;
    let top_surface = cap_surface(&mut builder, &sections[sections.len() - 1].z)?;
    let mut bottom_uses = Vec::with_capacity(count);
    for index in (0..count).rev() {
        let next = (index + 1) % count;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(
                curve_point(&profiles[0][next]),
                curve_point(&profiles[0][index]),
            )
            .map_err(GeometryError::from)?,
        )))?;
        bottom_uses.push(builder.edge_use(
            rings[0][index],
            Direction::Reversed,
            pcurve,
            ParameterCorrespondence::affine(-Real::one(), Real::one())?,
        )?);
    }
    let bottom_wire = builder.wire(bottom_uses)?;
    let bottom_face = builder.face(
        bottom_surface,
        Orientation::Reversed,
        bottom_wire,
        Vec::new(),
    )?;
    let mut top_uses = Vec::with_capacity(count);
    let top = profiles.len() - 1;
    for index in 0..count {
        let next = (index + 1) % count;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(
                curve_point(&profiles[top][index]),
                curve_point(&profiles[top][next]),
            )
            .map_err(GeometryError::from)?,
        )))?;
        top_uses.push(builder.edge_use(
            rings[top][index],
            Direction::Forward,
            pcurve,
            ParameterCorrespondence::identity(),
        )?);
    }
    let top_wire = builder.wire(top_uses)?;
    let top_face = builder.face(top_surface, Orientation::Forward, top_wire, Vec::new())?;

    let mut faces = vec![bottom_face, top_face];
    for span in 0..sections.len() - 1 {
        for index in 0..count {
            let next = (index + 1) % count;
            let point = |section: usize, vertex: usize| {
                Point3::new(
                    profiles[section][vertex].x.clone(),
                    profiles[section][vertex].y.clone(),
                    sections[section].z.clone(),
                )
            };
            let lower_start = point(span, index);
            let lower_end = point(span, next);
            let upper_start = point(span + 1, index);
            let upper_end = point(span + 1, next);
            let (side_surface, parameter_points) = if let Some(scale) = &span_scales[span] {
                (
                    Surface::plane(
                        lower_start.clone(),
                        &lower_end - &lower_start,
                        &upper_start - &lower_start,
                    )?,
                    [
                        CurvePoint2::new(Real::zero(), Real::zero()),
                        CurvePoint2::new(Real::one(), Real::zero()),
                        CurvePoint2::new(scale.clone(), Real::one()),
                        CurvePoint2::new(Real::zero(), Real::one()),
                    ],
                )
            } else {
                (
                    Surface::rational_bezier(
                        vec![vec![lower_start, lower_end], vec![upper_start, upper_end]],
                        vec![vec![Real::one(), Real::one()]; 2],
                    )?,
                    [
                        CurvePoint2::new(Real::zero(), Real::zero()),
                        CurvePoint2::new(Real::one(), Real::zero()),
                        CurvePoint2::new(Real::one(), Real::one()),
                        CurvePoint2::new(Real::zero(), Real::one()),
                    ],
                )
            };
            let side_surface = builder.surface(side_surface)?;
            let side_specs = [
                (rings[span][index], Direction::Forward, 0, 1),
                (connectors[span][next], Direction::Forward, 1, 2),
                (rings[span + 1][index], Direction::Reversed, 2, 3),
                (connectors[span][index], Direction::Reversed, 3, 0),
            ];
            let mut uses = Vec::with_capacity(4);
            for (edge, direction, start, end) in side_specs {
                let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(
                        parameter_points[start].clone(),
                        parameter_points[end].clone(),
                    )
                    .map_err(GeometryError::from)?,
                )))?;
                uses.push(builder.edge_use(
                    edge,
                    direction,
                    pcurve,
                    match direction {
                        Direction::Forward => ParameterCorrespondence::identity(),
                        Direction::Reversed => {
                            ParameterCorrespondence::affine(-Real::one(), Real::one())?
                        }
                    },
                )?);
            }
            let wire = builder.wire(uses)?;
            faces.push(builder.face(side_surface, Orientation::Forward, wire, Vec::new())?);
        }
    }
    let shell = builder.shell(faces)?;
    let solid = builder.solid(shell, Vec::new())?;
    Ok((builder.finish()?, solid))
}

fn certified_loft_scale(lower: &[Point2], upper: &[Point2]) -> Result<Real, ConstructionError> {
    if lower.len() != upper.len() {
        return Err(ConstructionError::IncompatibleLoftSections);
    }
    let lower_delta = &lower[1] - &lower[0];
    let upper_delta = &upper[1] - &upper[0];
    let lower_x_is_zero =
        match compare_reals(&lower_delta.0[0], &Real::zero(), crate::STRICT_PREDICATES) {
            PredicateOutcome::Decided { value, .. } => value == std::cmp::Ordering::Equal,
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                })
                .into());
            }
        };
    let scale = if lower_x_is_zero {
        (&upper_delta.0[1] / &lower_delta.0[1]).map_err(|_| GeometryError::ProjectiveDivision)?
    } else {
        (&upper_delta.0[0] / &lower_delta.0[0]).map_err(|_| GeometryError::ProjectiveDivision)?
    };
    match compare_reals(&scale, &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } => {}
        PredicateOutcome::Decided { .. } => {
            return Err(ConstructionError::IncompatibleLoftSections);
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(
                BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into(),
            );
        }
    }
    for index in 0..lower.len() {
        let lower_delta = &lower[index] - &lower[0];
        let upper_delta = &upper[index] - &upper[0];
        for (actual, expected) in [
            (&upper_delta.0[0], &scale * &lower_delta.0[0]),
            (&upper_delta.0[1], &scale * &lower_delta.0[1]),
        ] {
            match compare_reals(actual, &expected, crate::STRICT_PREDICATES) {
                PredicateOutcome::Decided {
                    value: std::cmp::Ordering::Equal,
                    ..
                } => {}
                PredicateOutcome::Decided { .. } => {
                    return Err(ConstructionError::IncompatibleLoftSections);
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    return Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                        needed,
                        stage,
                    })
                    .into());
                }
            }
        }
    }
    Ok(scale)
}

fn certify_convex_loft_correspondence(
    lower: &[Point2],
    upper: &[Point2],
) -> Result<(), ConstructionError> {
    if lower.len() != upper.len() {
        return Err(ConstructionError::IncompatibleLoftSections);
    }
    let edge =
        |points: &[Point2], index: usize| &points[(index + 1) % points.len()] - &points[index];
    let cross =
        |first: &Vector2, second: &Vector2| &first.0[0] * &second.0[1] - &first.0[1] * &second.0[0];
    for index in 0..lower.len() {
        let next = (index + 1) % lower.len();
        let lower_edge = edge(lower, index);
        let lower_next = edge(lower, next);
        let upper_edge = edge(upper, index);
        let upper_next = edge(upper, next);
        let lower_turn = cross(&lower_edge, &lower_next);
        let upper_turn = cross(&upper_edge, &upper_next);
        let mixed_turn = cross(&lower_edge, &upper_next) + cross(&upper_edge, &lower_next);
        for turn in [&lower_turn, &upper_turn] {
            if decided_construction_order(compare_reals(
                turn,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            {
                return Err(ConstructionError::IncompatibleLoftSections);
            }
        }
        if decided_construction_order(compare_reals(
            &mixed_turn,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less
        {
            return Err(ConstructionError::IncompatibleLoftSections);
        }
    }
    Ok(())
}

fn decided_construction_order(
    outcome: PredicateOutcome<std::cmp::Ordering>,
) -> Result<std::cmp::Ordering, ConstructionError> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into())
        }
    }
}

fn validate_revolution_profile_radius(profile: &[Point2]) -> Result<(), ConstructionError> {
    for point in profile {
        match compare_reals(&point.x, &Real::zero(), crate::STRICT_PREDICATES) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Greater,
                ..
            } => {}
            PredicateOutcome::Decided { .. } => {
                return Err(ConstructionError::ProfileCrossesRevolutionAxis);
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                })
                .into());
            }
        }
    }
    Ok(())
}

fn normalize_revolution_contour(contour: &Contour2) -> Result<Contour2, ConstructionError> {
    let contour = normalize_contour(contour, true)?;
    let bounds = match Aabb2::from_contour(&contour, &CurvePolicy::STRICT)
        .map_err(GeometryError::from)?
    {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => {
            return Err(
                BuildError::Geometry(GeometryError::PlanarClassificationUnresolved(reason)).into(),
            );
        }
    };
    match compare_reals(bounds.min_x(), &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } => Ok(contour),
        PredicateOutcome::Decided { .. } => Err(ConstructionError::ProfileCrossesRevolutionAxis),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into())
        }
    }
}

fn curve2_from_bezier_subcurve(curve: &BezierSubcurve2) -> Curve2 {
    match curve {
        BezierSubcurve2::Quadratic(curve) => Curve2::from(curve.clone()),
        BezierSubcurve2::Cubic(curve) => Curve2::from(curve.clone()),
        BezierSubcurve2::RationalQuadratic(curve) => Curve2::from(curve.clone()),
        BezierSubcurve2::Rational(curve) => Curve2::from(curve.clone()),
    }
}

fn exact_parameter_is(parameter: Option<Real>, expected: &Real) -> Result<bool, ConstructionError> {
    parameter
        .map(|parameter| exact_real_equal(&parameter, expected))
        .transpose()
        .map(Option::unwrap_or_default)
}

fn normalize_planar_path(
    path: &CurvePath2,
    counterclockwise: bool,
) -> Result<CurvePath2, ConstructionError> {
    let path = partition_periodic_curve_path(path, ConstructionError::UnsupportedPlanarProfile)?;
    validate_simple_curve_path(&path, ConstructionError::UnsupportedPlanarProfile)?;
    let signed_area = path
        .bezier_boundary_loop()
        .map_err(GeometryError::from)?
        .boundary_loop()
        .signed_area()
        .map_err(GeometryError::from)?
        .ok_or(ConstructionError::UnsupportedPlanarProfile)?;
    match compare_reals(&signed_area, &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } if counterclockwise => Ok(path),
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Less,
            ..
        } if !counterclockwise => Ok(path),
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater | std::cmp::Ordering::Less,
            ..
        } => path
            .reversed()
            .map_err(GeometryError::from)
            .map_err(Into::into),
        PredicateOutcome::Decided { .. } => Err(ConstructionError::DegenerateProfile),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into())
        }
    }
}

fn validate_planar_path_nesting(
    outer: &CurvePath2,
    holes: &[CurvePath2],
) -> Result<(), ConstructionError> {
    let policy = CurvePolicy::STRICT;
    let paths_are_disjoint =
        |first: &CurvePath2, second: &CurvePath2| -> Result<bool, ConstructionError> {
            let result = first
                .intersect_path(second, &policy)
                .map_err(GeometryError::from)?;
            if !result.is_complete() {
                return Err(ConstructionError::UnsupportedPlanarProfile);
            }
            Ok(result.contacts().is_empty() && result.overlaps().is_empty())
        };
    let classify = |container: &CurvePath2,
                    point: &CurvePoint2|
     -> Result<ContourPointLocation, ConstructionError> {
        match container
            .classify_point(point, &policy)
            .map_err(GeometryError::from)?
        {
            Classification::Decided(location) => Ok(location),
            Classification::Uncertain(reason) => Err(BuildError::Geometry(
                GeometryError::PlanarClassificationUnresolved(reason),
            )
            .into()),
        }
    };

    for hole in holes {
        if !paths_are_disjoint(outer, hole)? {
            return Err(ConstructionError::IntersectingProfiles);
        }
        if classify(outer, hole.start())? != ContourPointLocation::Inside {
            return Err(ConstructionError::HoleOutside);
        }
    }
    for first in 0..holes.len() {
        for second in first + 1..holes.len() {
            if !paths_are_disjoint(&holes[first], &holes[second])? {
                return Err(ConstructionError::IntersectingProfiles);
            }
            if classify(&holes[first], holes[second].start())? == ContourPointLocation::Inside
                || classify(&holes[second], holes[first].start())? == ContourPointLocation::Inside
            {
                return Err(ConstructionError::NestedHoles);
            }
        }
    }
    Ok(())
}

fn persistent_rational_bezier(curve: &BezierSubcurve2) -> Result<Curve2, ConstructionError> {
    let (control_points, weights) = match curve {
        BezierSubcurve2::Quadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 3],
        ),
        BezierSubcurve2::Cubic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 4],
        ),
        BezierSubcurve2::RationalQuadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            curve.weights().into_iter().cloned().collect(),
        ),
        BezierSubcurve2::Rational(curve) => {
            (curve.control_points().to_vec(), curve.weights().to_vec())
        }
    };
    RationalBezier2::try_new(control_points, weights)
        .map(Curve2::from)
        .map_err(GeometryError::from)
        .map_err(Into::into)
}

fn persistent_planar_path_curves(path: &CurvePath2) -> Result<Vec<Curve2>, ConstructionError> {
    let mut persistent = Vec::new();
    for curve in path.curves() {
        match curve.geometry() {
            CurveGeometry2::Line(_)
            | CurveGeometry2::RationalBezier(_)
            | CurveGeometry2::Nurbs(_) => persistent.push(curve.clone()),
            CurveGeometry2::PolynomialBSpline(curve) => {
                persistent.push(
                    Curve2::try_nurbs(
                        curve.degree(),
                        curve.control_points().to_vec(),
                        vec![Real::one(); curve.control_points().len()],
                        curve.knots().to_vec(),
                    )
                    .map_err(GeometryError::from)?,
                );
            }
            CurveGeometry2::CircularArc(_)
            | CurveGeometry2::QuadraticBezier(_)
            | CurveGeometry2::CubicBezier(_)
            | CurveGeometry2::RationalQuadraticBezier(_) => {
                for fragment in curve
                    .native_bezier_fragments()
                    .map_err(GeometryError::from)?
                {
                    persistent.push(persistent_rational_bezier(fragment.curve())?);
                }
            }
        }
    }
    Ok(persistent)
}

fn persistent_extrusion_path_curves(path: &CurvePath2) -> Result<Vec<Curve2>, ConstructionError> {
    let mut persistent = Vec::new();
    for curve in path.curves() {
        match curve.geometry() {
            CurveGeometry2::Line(_)
            | CurveGeometry2::CircularArc(_)
            | CurveGeometry2::RationalBezier(_)
            | CurveGeometry2::Nurbs(_) => persistent.push(curve.clone()),
            CurveGeometry2::PolynomialBSpline(curve) => {
                persistent.push(
                    Curve2::try_nurbs(
                        curve.degree(),
                        curve.control_points().to_vec(),
                        vec![Real::one(); curve.control_points().len()],
                        curve.knots().to_vec(),
                    )
                    .map_err(GeometryError::from)?,
                );
            }
            CurveGeometry2::QuadraticBezier(_)
            | CurveGeometry2::CubicBezier(_)
            | CurveGeometry2::RationalQuadraticBezier(_) => {
                for fragment in curve
                    .native_bezier_fragments()
                    .map_err(GeometryError::from)?
                {
                    persistent.push(persistent_rational_bezier(fragment.curve())?);
                }
            }
        }
    }
    Ok(persistent)
}

fn spatial_extrusion_curve(curve: &Curve2, z: &Real) -> Result<Curve3, ConstructionError> {
    let lift = |point: &CurvePoint2| Point3::new(point.x().clone(), point.y().clone(), z.clone());
    match curve.geometry() {
        CurveGeometry2::Line(line) => Ok(Curve3::line(lift(line.start()), lift(line.end()))?),
        CurveGeometry2::CircularArc(arc) => {
            spatial_segment_curve(&Segment2::Arc(arc.clone()), z).map(|(curve, _)| curve)
        }
        CurveGeometry2::RationalBezier(curve) => Ok(Curve3::rational_bezier(
            curve.control_points().iter().map(lift).collect(),
            curve.weights().to_vec(),
        )?),
        CurveGeometry2::Nurbs(curve) => Ok(Curve3::nurbs(
            curve.degree(),
            curve.control_points().iter().map(lift).collect(),
            curve.weights().to_vec(),
            curve.knots().to_vec(),
        )?),
        CurveGeometry2::QuadraticBezier(_)
        | CurveGeometry2::CubicBezier(_)
        | CurveGeometry2::RationalQuadraticBezier(_)
        | CurveGeometry2::PolynomialBSpline(_) => Err(ConstructionError::UnsupportedPlanarProfile),
    }
}

fn planar_curve_correspondence(
    curve: &Curve2,
    direction: Direction,
) -> Result<ParameterCorrespondence, ConstructionError> {
    if matches!(curve.geometry(), CurveGeometry2::CircularArc(_)) {
        return Ok(ParameterCorrespondence::angular_sweep());
    }
    Ok(match direction {
        Direction::Forward => ParameterCorrespondence::identity(),
        Direction::Reversed => ParameterCorrespondence::affine(
            -Real::one(),
            curve.parameter_domain().start() + curve.parameter_domain().end(),
        )?,
    })
}

fn lift_planar_pcurve(curve: &Curve2, surface: &Surface) -> Result<Curve3, ConstructionError> {
    let lift =
        |point: &CurvePoint2| surface.point_at(&Point2::new(point.x().clone(), point.y().clone()));
    match curve.geometry() {
        CurveGeometry2::Line(line) => Ok(Curve3::line(lift(line.start())?, lift(line.end())?)?),
        CurveGeometry2::RationalBezier(curve) => Ok(Curve3::rational_bezier(
            curve
                .control_points()
                .iter()
                .map(&lift)
                .collect::<Result<Vec<_>, _>>()?,
            curve.weights().to_vec(),
        )?),
        CurveGeometry2::Nurbs(curve) => Ok(Curve3::nurbs(
            curve.degree(),
            curve
                .control_points()
                .iter()
                .map(lift)
                .collect::<Result<Vec<_>, _>>()?,
            curve.weights().to_vec(),
            curve.knots().to_vec(),
        )?),
        CurveGeometry2::CircularArc(_)
        | CurveGeometry2::QuadraticBezier(_)
        | CurveGeometry2::CubicBezier(_)
        | CurveGeometry2::RationalQuadraticBezier(_)
        | CurveGeometry2::PolynomialBSpline(_) => Err(ConstructionError::UnsupportedPlanarProfile),
    }
}

fn add_planar_path_wire(
    builder: &mut ModelBuilder,
    path: &CurvePath2,
    surface: &Surface,
) -> Result<crate::WireId, ConstructionError> {
    let curves = persistent_planar_path_curves(path)?;
    let points = curves
        .iter()
        .map(|curve| {
            surface.point_at(&Point2::new(
                curve.start().x().clone(),
                curve.start().y().clone(),
            ))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let vertices = points
        .into_iter()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let mut uses = Vec::with_capacity(curves.len());
    for (index, pcurve) in curves.into_iter().enumerate() {
        let spatial = lift_planar_pcurve(&pcurve, surface)?;
        let domain = spatial.domain().clone();
        let curve = builder.curve(spatial)?;
        let edge = builder.edge(
            vertices[index],
            vertices[(index + 1) % vertices.len()],
            curve,
            domain,
        )?;
        let pcurve = builder.pcurve(Pcurve::new(pcurve))?;
        uses.push(builder.edge_use(
            edge,
            Direction::Forward,
            pcurve,
            ParameterCorrespondence::identity(),
        )?);
    }
    Ok(builder.wire(uses)?)
}

fn validate_simple_curve_path(
    profile: &CurvePath2,
    unsupported: ConstructionError,
) -> Result<(), ConstructionError> {
    let policy = CurvePolicy::STRICT;
    let fragments = profile
        .native_bezier_fragments()
        .map_err(GeometryError::from)?;
    if fragments.len() < 2 {
        return Err(ConstructionError::ProfileTooSmall);
    }
    for fragment in fragments {
        if !fragment
            .has_certified_injective_axis(&policy)
            .map_err(GeometryError::from)?
        {
            return Err(unsupported.clone());
        }
    }
    let curves = fragments
        .iter()
        .map(|fragment| curve2_from_bezier_subcurve(fragment.curve()))
        .collect::<Vec<_>>();
    for first_index in 0..curves.len() {
        for second_index in first_index + 1..curves.len() {
            let result = curves[first_index]
                .intersect_curve(&curves[second_index], &policy)
                .map_err(GeometryError::from)?;
            if !result.blockers().is_empty() {
                return Err(unsupported.clone());
            }
            if !result.overlaps().is_empty() {
                return Err(ConstructionError::SelfIntersectingProfile);
            }
            for contact in result.contacts() {
                let forward_seam = second_index == first_index + 1
                    && exact_parameter_is(
                        contact.first().exact_curve_parameter(),
                        curves[first_index].parameter_domain().end(),
                    )?
                    && exact_parameter_is(
                        contact.second().exact_curve_parameter(),
                        curves[second_index].parameter_domain().start(),
                    )?;
                let closing_seam = first_index == 0
                    && second_index + 1 == curves.len()
                    && exact_parameter_is(
                        contact.first().exact_curve_parameter(),
                        curves[first_index].parameter_domain().start(),
                    )?
                    && exact_parameter_is(
                        contact.second().exact_curve_parameter(),
                        curves[second_index].parameter_domain().end(),
                    )?;
                if !forward_seam && !closing_seam {
                    return Err(ConstructionError::SelfIntersectingProfile);
                }
            }
        }
    }
    Ok(())
}

fn partition_periodic_curve_path(
    profile: &CurvePath2,
    unsupported: ConstructionError,
) -> Result<CurvePath2, ConstructionError> {
    let periodic_curves = profile
        .curves()
        .iter()
        .filter(|curve| curve.is_periodic())
        .count();
    if periodic_curves == 0 {
        return Ok(profile.clone());
    }
    let [curve] = profile.curves() else {
        return Err(unsupported.clone());
    };
    if !curve.is_periodic() {
        return Err(unsupported.clone());
    }
    let fragments = curve
        .native_bezier_fragments()
        .map_err(GeometryError::from)?;
    if fragments.len() < 2 {
        return Err(ConstructionError::ProfileTooSmall);
    }
    let curves = fragments
        .iter()
        .map(|fragment| {
            let (start, end) = fragment.parameter_range();
            curve
                .clamped_subcurve(start.clone(), end.clone())
                .map_err(GeometryError::from)
                .map_err(Into::into)
        })
        .collect::<Result<Vec<_>, ConstructionError>>()?;
    if curves.iter().any(Curve2::is_periodic) {
        return Err(unsupported);
    }
    CurvePath2::try_new(curves)
        .map_err(GeometryError::from)
        .map_err(Into::into)
}

fn normalize_revolution_path(profile: &CurvePath2) -> Result<CurvePath2, ConstructionError> {
    let profile =
        partition_periodic_curve_path(profile, ConstructionError::UnsupportedRevolutionProfile)?;
    validate_simple_curve_path(&profile, ConstructionError::UnsupportedRevolutionProfile)?;
    let bounds = profile.bounds().map_err(GeometryError::from)?;
    match compare_reals(bounds.min_x(), &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } => {}
        PredicateOutcome::Decided { .. } => {
            return Err(ConstructionError::ProfileCrossesRevolutionAxis);
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(
                BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into(),
            );
        }
    }
    let signed_area = profile
        .bezier_boundary_loop()
        .map_err(GeometryError::from)?
        .boundary_loop()
        .signed_area()
        .map_err(GeometryError::from)?
        .ok_or(ConstructionError::UnsupportedRevolutionProfile)?;
    match compare_reals(&signed_area, &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } => Ok(profile),
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Less,
            ..
        } => profile
            .reversed()
            .map_err(GeometryError::from)
            .map_err(Into::into),
        PredicateOutcome::Decided { .. } => Err(ConstructionError::DegenerateProfile),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into())
        }
    }
}

fn positive_projective_weights(weights: &[Real]) -> Result<Vec<Real>, ConstructionError> {
    let mut sign = None;
    for weight in weights {
        let this_sign = match compare_reals(weight, &Real::zero(), crate::STRICT_PREDICATES) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Greater,
                ..
            } => 1_i8,
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Less,
                ..
            } => -1_i8,
            PredicateOutcome::Decided { .. } => {
                return Err(ConstructionError::UnsupportedRevolutionProfile);
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                })
                .into());
            }
        };
        if sign
            .replace(this_sign)
            .is_some_and(|sign| sign != this_sign)
        {
            return Err(ConstructionError::UnsupportedRevolutionProfile);
        }
    }
    Ok(if sign == Some(-1) {
        weights.iter().map(|weight| -weight.clone()).collect()
    } else {
        weights.to_vec()
    })
}

fn spatial_revolution_curve(
    curve: &Curve2,
    angle: &Real,
) -> Result<(Curve3, ParameterDomain), ConstructionError> {
    if curve.is_periodic() {
        return Err(ConstructionError::UnsupportedRevolutionProfile);
    }
    let radial = Vector3::from_xyz(angle.clone().cos(), angle.clone().sin(), Real::zero());
    let lift = |point: &CurvePoint2| {
        Point3::new(
            point.x() * &radial.0[0],
            point.x() * &radial.0[1],
            point.y().clone(),
        )
    };
    let spatial = match curve.geometry() {
        CurveGeometry2::Line(line) => Curve3::line(lift(line.start()), lift(line.end()))?,
        CurveGeometry2::CircularArc(arc) => {
            return spatial_revolution_segment(&Segment2::Arc(arc.clone()), angle);
        }
        CurveGeometry2::QuadraticBezier(curve) => Curve3::rational_bezier(
            curve.control_points().into_iter().map(lift).collect(),
            vec![Real::one(); 3],
        )?,
        CurveGeometry2::CubicBezier(curve) => Curve3::rational_bezier(
            curve.control_points().into_iter().map(lift).collect(),
            vec![Real::one(); 4],
        )?,
        CurveGeometry2::RationalQuadraticBezier(curve) => Curve3::rational_bezier(
            curve.control_points().into_iter().map(lift).collect(),
            positive_projective_weights(&curve.weights().into_iter().cloned().collect::<Vec<_>>())?,
        )?,
        CurveGeometry2::RationalBezier(curve) => Curve3::rational_bezier(
            curve.control_points().iter().map(lift).collect(),
            positive_projective_weights(curve.weights())?,
        )?,
        CurveGeometry2::PolynomialBSpline(curve) => Curve3::nurbs(
            curve.degree(),
            curve.control_points().iter().map(lift).collect(),
            vec![Real::one(); curve.control_points().len()],
            curve.knots().to_vec(),
        )?,
        CurveGeometry2::Nurbs(curve) => Curve3::nurbs(
            curve.degree(),
            curve.control_points().iter().map(lift).collect(),
            positive_projective_weights(curve.weights())?,
            curve.knots().to_vec(),
        )?,
    };
    let domain = spatial.domain().clone();
    Ok((spatial, domain))
}

fn spatial_revolution_segment(
    segment: &Segment2,
    angle: &Real,
) -> Result<(Curve3, ParameterDomain), ConstructionError> {
    let radial = Vector3::from_xyz(angle.clone().cos(), angle.clone().sin(), Real::zero());
    match segment {
        Segment2::Line(line) => Ok((
            Curve3::line(
                Point3::new(
                    line.start().x() * &radial.0[0],
                    line.start().x() * &radial.0[1],
                    line.start().y().clone(),
                ),
                Point3::new(
                    line.end().x() * &radial.0[0],
                    line.end().x() * &radial.0[1],
                    line.end().y().clone(),
                ),
            )?,
            ParameterDomain::unit(),
        )),
        Segment2::Arc(arc) => {
            let radius = arc
                .radius_squared()
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let start_radial = arc.start().x() - arc.center().x();
            let start_axial = arc.start().y() - arc.center().y();
            let x = radial.clone()
                * ((start_radial.clone() / &radius)
                    .map_err(|_| GeometryError::ProjectiveDivision)?)
                + Vector3::z()
                    * ((start_axial.clone() / &radius)
                        .map_err(|_| GeometryError::ProjectiveDivision)?);
            let (tangent_radial, tangent_axial) = if arc.is_clockwise() {
                (start_axial, -start_radial)
            } else {
                (-start_axial, start_radial)
            };
            let y = radial.clone()
                * ((tangent_radial / &radius).map_err(|_| GeometryError::ProjectiveDivision)?)
                + Vector3::z()
                    * ((tangent_axial / &radius).map_err(|_| GeometryError::ProjectiveDivision)?);
            let sweep = match arc.directed_sweep_angle().map_err(GeometryError::from)? {
                Classification::Decided(sweep) => sweep,
                Classification::Uncertain(reason) => {
                    return Err(BuildError::Geometry(
                        GeometryError::PlanarClassificationUnresolved(reason),
                    )
                    .into());
                }
            };
            let domain = ParameterDomain::new(Real::zero(), sweep.clone())?;
            Ok((
                Curve3::circle_arc(
                    Point3::new(
                        arc.center().x() * &radial.0[0],
                        arc.center().x() * &radial.0[1],
                        arc.center().y().clone(),
                    ),
                    x,
                    y,
                    radius,
                    Real::zero(),
                    sweep,
                )?,
                domain,
            ))
        }
    }
}

fn add_normalized_curve_path_revolution_shell(
    builder: &mut ModelBuilder,
    profile: &CurvePath2,
    direction: ShellDirection,
) -> Result<crate::ShellId, ConstructionError> {
    let curves = profile.curves();
    let quarter = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let angles = (0..4)
        .map(|index| &quarter * Real::from(index))
        .collect::<Vec<_>>();
    let mut points = Vec::with_capacity(curves.len() * 4);
    for curve in curves {
        for angle in &angles {
            points.push(Point3::new(
                curve.start().x() * angle.clone().cos(),
                curve.start().x() * angle.clone().sin(),
                curve.start().y().clone(),
            ));
        }
    }
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let vertex =
        |profile_index: usize, angle_index: usize| vertices[profile_index * 4 + angle_index];

    let mut circles = vec![Vec::with_capacity(4); curves.len()];
    for (profile_index, curve) in curves.iter().enumerate() {
        for angle_index in 0..4 {
            let next_angle = (angle_index + 1) % 4;
            let start = &quarter * Real::from(angle_index as i32);
            let end = &quarter * Real::from(angle_index as i32 + 1);
            let circle = builder.curve(Curve3::circle_arc(
                Point3::new(Real::zero(), Real::zero(), curve.start().y().clone()),
                Vector3::x(),
                Vector3::y(),
                curve.start().x().clone(),
                start.clone(),
                end.clone(),
            )?)?;
            circles[profile_index].push(builder.edge(
                vertex(profile_index, angle_index),
                vertex(profile_index, next_angle),
                circle,
                ParameterDomain::new(start, end)?,
            )?);
        }
    }

    let mut meridians = vec![Vec::with_capacity(4); curves.len()];
    let mut profile_curves = Vec::with_capacity(curves.len());
    let mut domains = Vec::with_capacity(curves.len());
    for (profile_index, curve) in curves.iter().enumerate() {
        let next_profile = (profile_index + 1) % curves.len();
        let (profile_curve, domain) = spatial_revolution_curve(curve, &Real::zero())?;
        profile_curves.push(profile_curve);
        domains.push(domain);
        for (angle_index, angle) in angles.iter().enumerate() {
            let (meridian, curve_domain) = spatial_revolution_curve(curve, angle)?;
            let meridian = builder.curve(meridian)?;
            meridians[profile_index].push(builder.edge(
                vertex(profile_index, angle_index),
                vertex(next_profile, angle_index),
                meridian,
                curve_domain,
            )?);
        }
    }

    let mut faces = Vec::with_capacity(curves.len() * 4);
    for profile_index in 0..curves.len() {
        let next_profile = (profile_index + 1) % curves.len();
        let surface = builder.surface(Surface::revolution(
            profile_curves[profile_index].clone(),
            Point3::origin(),
            Vector3::z(),
        )?)?;
        let domain = &domains[profile_index];
        let v_min = domain.start();
        let v_max = domain.end();
        for angle_index in 0..4 {
            let next_angle = (angle_index + 1) % 4;
            let u_min = &quarter * Real::from(angle_index as i32);
            let u_max = &quarter * Real::from(angle_index as i32 + 1);
            let mut specs = vec![
                (
                    circles[profile_index][angle_index],
                    Direction::Forward,
                    CurvePoint2::new(u_min.clone(), v_min.clone()),
                    CurvePoint2::new(u_max.clone(), v_min.clone()),
                    ParameterCorrespondence::affine(&u_max - &u_min, u_min.clone())?,
                ),
                (
                    meridians[profile_index][next_angle],
                    Direction::Forward,
                    CurvePoint2::new(u_max.clone(), v_min.clone()),
                    CurvePoint2::new(u_max.clone(), v_max.clone()),
                    ParameterCorrespondence::affine(v_max - v_min, v_min.clone())?,
                ),
                (
                    circles[next_profile][angle_index],
                    Direction::Reversed,
                    CurvePoint2::new(u_max.clone(), v_max.clone()),
                    CurvePoint2::new(u_min.clone(), v_max.clone()),
                    ParameterCorrespondence::affine(&u_min - &u_max, u_max.clone())?,
                ),
                (
                    meridians[profile_index][angle_index],
                    Direction::Reversed,
                    CurvePoint2::new(u_min.clone(), v_max.clone()),
                    CurvePoint2::new(u_min, v_min.clone()),
                    ParameterCorrespondence::affine(v_min - v_max, v_max.clone())?,
                ),
            ];
            if matches!(direction, ShellDirection::Inward) {
                specs.reverse();
                for (_, use_direction, start, end, correspondence) in &mut specs {
                    *use_direction = use_direction.reversed();
                    std::mem::swap(start, end);
                    let ParameterCorrespondence::Affine { scale, offset, .. } = correspondence
                    else {
                        unreachable!("revolution pcurves use affine correspondence");
                    };
                    *correspondence = ParameterCorrespondence::affine(
                        -scale.clone(),
                        scale.clone() + offset.clone(),
                    )?;
                }
            }
            let mut uses = Vec::with_capacity(4);
            for (edge, use_direction, start, end, correspondence) in specs {
                let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
                )))?;
                uses.push(builder.edge_use(edge, use_direction, pcurve, correspondence)?);
            }
            let wire = builder.wire(uses)?;
            faces.push(builder.face(
                surface,
                match direction {
                    ShellDirection::Outward => Orientation::Forward,
                    ShellDirection::Inward => Orientation::Reversed,
                },
                wire,
                Vec::new(),
            )?);
        }
    }
    Ok(builder.shell(faces)?)
}

fn add_normalized_contour_revolution_shell(
    builder: &mut ModelBuilder,
    profile: &Contour2,
    direction: ShellDirection,
) -> Result<crate::ShellId, ConstructionError> {
    let segments = profile.segments();
    let quarter = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let angles = (0..4)
        .map(|index| &quarter * Real::from(index))
        .collect::<Vec<_>>();
    let mut points = Vec::with_capacity(segments.len() * 4);
    for segment in segments {
        for angle in &angles {
            points.push(Point3::new(
                segment.start().x() * angle.clone().cos(),
                segment.start().x() * angle.clone().sin(),
                segment.start().y().clone(),
            ));
        }
    }
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let vertex =
        |profile_index: usize, angle_index: usize| vertices[profile_index * 4 + angle_index];

    let mut circles = vec![Vec::with_capacity(4); segments.len()];
    for (profile_index, segment) in segments.iter().enumerate() {
        for angle_index in 0..4 {
            let next_angle = (angle_index + 1) % 4;
            let start = &quarter * Real::from(angle_index as i32);
            let end = &quarter * Real::from(angle_index as i32 + 1);
            let curve = builder.curve(Curve3::circle_arc(
                Point3::new(Real::zero(), Real::zero(), segment.start().y().clone()),
                Vector3::x(),
                Vector3::y(),
                segment.start().x().clone(),
                start.clone(),
                end.clone(),
            )?)?;
            circles[profile_index].push(builder.edge(
                vertex(profile_index, angle_index),
                vertex(profile_index, next_angle),
                curve,
                ParameterDomain::new(start, end)?,
            )?);
        }
    }

    let mut meridians = vec![Vec::with_capacity(4); segments.len()];
    let mut profile_curves = Vec::with_capacity(segments.len());
    let mut domains = Vec::with_capacity(segments.len());
    for (profile_index, segment) in segments.iter().enumerate() {
        let next_profile = (profile_index + 1) % segments.len();
        let (profile_curve, domain) = spatial_revolution_segment(segment, &Real::zero())?;
        profile_curves.push(profile_curve);
        domains.push(domain);
        for (angle_index, angle) in angles.iter().enumerate() {
            let (curve, curve_domain) = spatial_revolution_segment(segment, angle)?;
            let curve = builder.curve(curve)?;
            meridians[profile_index].push(builder.edge(
                vertex(profile_index, angle_index),
                vertex(next_profile, angle_index),
                curve,
                curve_domain,
            )?);
        }
    }

    let mut faces = Vec::with_capacity(segments.len() * 4);
    for profile_index in 0..segments.len() {
        let next_profile = (profile_index + 1) % segments.len();
        let surface = builder.surface(Surface::revolution(
            profile_curves[profile_index].clone(),
            Point3::origin(),
            Vector3::z(),
        )?)?;
        let domain = &domains[profile_index];
        let v_min = domain.start();
        let v_max = domain.end();
        for angle_index in 0..4 {
            let next_angle = (angle_index + 1) % 4;
            let u_min = &quarter * Real::from(angle_index as i32);
            let u_max = &quarter * Real::from(angle_index as i32 + 1);
            let mut specs = vec![
                (
                    circles[profile_index][angle_index],
                    Direction::Forward,
                    CurvePoint2::new(u_min.clone(), v_min.clone()),
                    CurvePoint2::new(u_max.clone(), v_min.clone()),
                    ParameterCorrespondence::affine(&u_max - &u_min, u_min.clone())?,
                ),
                (
                    meridians[profile_index][next_angle],
                    Direction::Forward,
                    CurvePoint2::new(u_max.clone(), v_min.clone()),
                    CurvePoint2::new(u_max.clone(), v_max.clone()),
                    ParameterCorrespondence::affine(v_max - v_min, v_min.clone())?,
                ),
                (
                    circles[next_profile][angle_index],
                    Direction::Reversed,
                    CurvePoint2::new(u_max.clone(), v_max.clone()),
                    CurvePoint2::new(u_min.clone(), v_max.clone()),
                    ParameterCorrespondence::affine(&u_min - &u_max, u_max.clone())?,
                ),
                (
                    meridians[profile_index][angle_index],
                    Direction::Reversed,
                    CurvePoint2::new(u_min.clone(), v_max.clone()),
                    CurvePoint2::new(u_min, v_min.clone()),
                    ParameterCorrespondence::affine(v_min - v_max, v_max.clone())?,
                ),
            ];
            if matches!(direction, ShellDirection::Inward) {
                specs.reverse();
                for (_, use_direction, start, end, correspondence) in &mut specs {
                    *use_direction = use_direction.reversed();
                    std::mem::swap(start, end);
                    let ParameterCorrespondence::Affine { scale, offset, .. } = correspondence
                    else {
                        unreachable!("revolution pcurves use affine correspondence");
                    };
                    *correspondence = ParameterCorrespondence::affine(
                        -scale.clone(),
                        scale.clone() + offset.clone(),
                    )?;
                }
            }
            let mut uses = Vec::with_capacity(4);
            for (edge, use_direction, start, end, correspondence) in specs {
                let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
                )))?;
                uses.push(builder.edge_use(edge, use_direction, pcurve, correspondence)?);
            }
            let wire = builder.wire(uses)?;
            faces.push(builder.face(
                surface,
                match direction {
                    ShellDirection::Outward => Orientation::Forward,
                    ShellDirection::Inward => Orientation::Reversed,
                },
                wire,
                Vec::new(),
            )?);
        }
    }
    Ok(builder.shell(faces)?)
}

fn add_normalized_revolution_shell(
    builder: &mut ModelBuilder,
    profile: &[Point2],
    direction: ShellDirection,
) -> Result<crate::ShellId, ConstructionError> {
    let quarter = (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
    let angles = (0..4)
        .map(|index| &quarter * Real::from(index))
        .collect::<Vec<_>>();
    let mut points = Vec::with_capacity(profile.len() * 4);
    for point in profile {
        for angle in &angles {
            points.push(Point3::new(
                &point.x * angle.clone().cos(),
                &point.x * angle.clone().sin(),
                point.y.clone(),
            ));
        }
    }
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;

    let vertex =
        |profile_index: usize, angle_index: usize| vertices[profile_index * 4 + angle_index];
    let mut circles = vec![Vec::with_capacity(4); profile.len()];
    for (profile_index, point) in profile.iter().enumerate() {
        for angle_index in 0..4 {
            let next_angle = (angle_index + 1) % 4;
            let start = &quarter * Real::from(angle_index as i32);
            let end = &quarter * Real::from(angle_index as i32 + 1);
            let curve = builder.curve(Curve3::circle_arc(
                Point3::new(Real::zero(), Real::zero(), point.y.clone()),
                Vector3::x(),
                Vector3::y(),
                point.x.clone(),
                start.clone(),
                end.clone(),
            )?)?;
            circles[profile_index].push(builder.edge(
                vertex(profile_index, angle_index),
                vertex(profile_index, next_angle),
                curve,
                ParameterDomain::new(start, end)?,
            )?);
        }
    }

    let mut meridians = vec![Vec::with_capacity(4); profile.len()];
    for profile_index in 0..profile.len() {
        let next_profile = (profile_index + 1) % profile.len();
        for angle_index in 0..4 {
            let curve = builder.curve(Curve3::line(
                points[profile_index * 4 + angle_index].clone(),
                points[next_profile * 4 + angle_index].clone(),
            )?)?;
            meridians[profile_index].push(builder.edge(
                vertex(profile_index, angle_index),
                vertex(next_profile, angle_index),
                curve,
                ParameterDomain::unit(),
            )?);
        }
    }

    let mut faces = Vec::with_capacity(profile.len() * 4);
    for profile_index in 0..profile.len() {
        let next_profile = (profile_index + 1) % profile.len();
        let profile_curve = Curve3::line(
            Point3::new(
                profile[profile_index].x.clone(),
                Real::zero(),
                profile[profile_index].y.clone(),
            ),
            Point3::new(
                profile[next_profile].x.clone(),
                Real::zero(),
                profile[next_profile].y.clone(),
            ),
        )?;
        let surface = builder.surface(Surface::revolution(
            profile_curve,
            Point3::origin(),
            Vector3::z(),
        )?)?;
        for angle_index in 0..4 {
            let next_angle = (angle_index + 1) % 4;
            let u_min = &quarter * Real::from(angle_index as i32);
            let u_max = &quarter * Real::from(angle_index as i32 + 1);
            let mut specs = vec![
                (
                    circles[profile_index][angle_index],
                    Direction::Forward,
                    CurvePoint2::new(u_min.clone(), Real::zero()),
                    CurvePoint2::new(u_max.clone(), Real::zero()),
                    ParameterCorrespondence::affine(&u_max - &u_min, u_min.clone())?,
                ),
                (
                    meridians[profile_index][next_angle],
                    Direction::Forward,
                    CurvePoint2::new(u_max.clone(), Real::zero()),
                    CurvePoint2::new(u_max.clone(), Real::one()),
                    ParameterCorrespondence::identity(),
                ),
                (
                    circles[next_profile][angle_index],
                    Direction::Reversed,
                    CurvePoint2::new(u_max.clone(), Real::one()),
                    CurvePoint2::new(u_min.clone(), Real::one()),
                    ParameterCorrespondence::affine(&u_min - &u_max, u_max.clone())?,
                ),
                (
                    meridians[profile_index][angle_index],
                    Direction::Reversed,
                    CurvePoint2::new(u_min.clone(), Real::one()),
                    CurvePoint2::new(u_min, Real::zero()),
                    ParameterCorrespondence::affine(-Real::one(), Real::one())?,
                ),
            ];
            if matches!(direction, ShellDirection::Inward) {
                specs.reverse();
                for (_, use_direction, start, end, correspondence) in &mut specs {
                    *use_direction = use_direction.reversed();
                    std::mem::swap(start, end);
                    let ParameterCorrespondence::Affine { scale, offset, .. } = correspondence
                    else {
                        unreachable!("revolution side pcurves use affine correspondence");
                    };
                    *correspondence = ParameterCorrespondence::affine(
                        -scale.clone(),
                        scale.clone() + offset.clone(),
                    )?;
                }
            }
            let mut uses = Vec::with_capacity(4);
            for (edge, direction, start, end, correspondence) in specs {
                let pcurve = builder.pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
                )))?;
                uses.push(builder.edge_use(edge, direction, pcurve, correspondence)?);
            }
            let wire = builder.wire(uses)?;
            faces.push(builder.face(
                surface,
                match direction {
                    ShellDirection::Outward => Orientation::Forward,
                    ShellDirection::Inward => Orientation::Reversed,
                },
                wire,
                Vec::new(),
            )?);
        }
    }
    Ok(builder.shell(faces)?)
}

/// Extrudes one exact simple polygon between two z coordinates.
///
/// Clockwise input is normalized to counterclockwise order. Self-intersections,
/// repeated adjacent points, zero area, and undecidable ordering are rejected
/// before any trusted topology is published.
pub fn extrude(
    profile: &[Point2],
    z_min: Real,
    z_max: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    extrude_region(profile, &[], z_min, z_max)
}

/// Extrudes one simple polygon and subtracts closed prismatic cavities.
///
/// Every cavity must be strictly contained by the outer extrusion. Cavity
/// shells are authored inward, retained as [`crate::Solid::voids`], and
/// certified for exact non-contact and non-overlap before publication.
pub fn extrude_with_voids(
    profile: &[Point2],
    z_min: Real,
    z_max: Real,
    voids: &[ExtrusionVoid],
) -> Result<(Model, SolidId), ConstructionError> {
    require_increasing(&z_min, &z_max, Axis::Z)?;
    let profile = normalize_profile(profile, true)?;
    let mut builder = ModelBuilder::new();
    let outer = add_normalized_extrusion_shell(
        &mut builder,
        &[profile],
        z_min,
        z_max,
        ShellDirection::Outward,
    )?;
    let mut void_shells = Vec::with_capacity(voids.len());
    for void_region in voids {
        require_increasing(&void_region.z_min, &void_region.z_max, Axis::Z)?;
        let profile = normalize_profile(&void_region.profile, true)?;
        void_shells.push(add_normalized_extrusion_shell(
            &mut builder,
            &[profile],
            void_region.z_min.clone(),
            void_region.z_max.clone(),
            ShellDirection::Inward,
        )?);
    }
    let solid = builder.solid(outer, void_shells)?;
    Ok((builder.finish()?, solid))
}

/// Extrudes one exact planar region with an outer loop and disjoint holes.
pub fn extrude_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    z_min: Real,
    z_max: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    require_increasing(&z_min, &z_max, Axis::Z)?;
    let profile = normalize_profile(outer, true)?;
    let holes = holes
        .iter()
        .map(|hole| normalize_profile(hole, false))
        .collect::<Result<Vec<_>, _>>()?;
    validate_profile_nesting(&profile, &holes)?;
    let mut loops = Vec::with_capacity(holes.len() + 1);
    loops.push(profile);
    loops.extend(holes);
    extrude_normalized_loops(&loops, z_min, z_max)
}

/// Extrudes multiple disjoint exact planar regions into one validated model.
///
/// Each tuple contains one outer loop and its hole loops. The returned solid
/// IDs follow input order.
pub fn extrude_regions(
    regions: &[(Vec<Point2>, Vec<Vec<Point2>>)],
    z_min: Real,
    z_max: Real,
) -> Result<(Model, Vec<SolidId>), ConstructionError> {
    require_increasing(&z_min, &z_max, Axis::Z)?;
    let mut builder = ModelBuilder::new();
    let mut solids = Vec::with_capacity(regions.len());
    for (outer, holes) in regions {
        let outer = normalize_profile(outer, true)?;
        let holes = holes
            .iter()
            .map(|hole| normalize_profile(hole, false))
            .collect::<Result<Vec<_>, _>>()?;
        validate_profile_nesting(&outer, &holes)?;
        let mut loops = Vec::with_capacity(holes.len() + 1);
        loops.push(outer);
        loops.extend(holes);
        solids.push(add_normalized_extrusion(
            &mut builder,
            &loops,
            z_min.clone(),
            z_max.clone(),
        )?);
    }
    Ok((builder.finish()?, solids))
}

fn normalize_profile(
    profile: &[Point2],
    counterclockwise: bool,
) -> Result<Vec<Point2>, ConstructionError> {
    if profile.len() < 3 {
        return Err(ConstructionError::ProfileTooSmall);
    }
    let mut profile = profile.to_vec();
    let mut contour_segments = Vec::with_capacity(profile.len());
    for index in 0..profile.len() {
        let next = (index + 1) % profile.len();
        contour_segments.push(Segment2::Line(
            LineSeg2::try_new(curve_point(&profile[index]), curve_point(&profile[next]))
                .map_err(GeometryError::from)?,
        ));
    }
    let contour = Contour2::try_new(contour_segments).map_err(GeometryError::from)?;
    if !contour
        .intersect_self(&CurvePolicy::STRICT)
        .map_err(GeometryError::from)?
        .is_empty()
    {
        return Err(ConstructionError::SelfIntersectingProfile);
    }
    let signed_area = contour
        .signed_area()
        .map_err(GeometryError::from)?
        .ok_or(ConstructionError::DegenerateProfile)?;
    match compare_reals(&signed_area, &Real::zero(), crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater,
            ..
        } if counterclockwise => {}
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Less,
            ..
        } if !counterclockwise => {}
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Greater | std::cmp::Ordering::Less,
            ..
        } => profile.reverse(),
        PredicateOutcome::Decided { .. } => return Err(ConstructionError::DegenerateProfile),
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(
                BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into(),
            );
        }
    }
    Ok(profile)
}

fn validate_profile_nesting(
    outer: &[Point2],
    holes: &[Vec<Point2>],
) -> Result<(), ConstructionError> {
    let policy = CurvePolicy::STRICT;
    let outer_contour = contour_from_profile(outer)?;
    let hole_contours = holes
        .iter()
        .map(|hole| contour_from_profile(hole))
        .collect::<Result<Vec<_>, _>>()?;
    for (hole, contour) in holes.iter().zip(&hole_contours) {
        if !outer_contour
            .intersect_contour(contour, &policy)
            .map_err(GeometryError::from)?
            .is_empty()
        {
            return Err(ConstructionError::IntersectingProfiles);
        }
        match outer_contour.classify_point(&curve_point(&hole[0]), &policy) {
            Classification::Decided(ContourPointLocation::Inside) => {}
            Classification::Decided(_) => return Err(ConstructionError::HoleOutside),
            Classification::Uncertain(reason) => {
                return Err(
                    BuildError::Geometry(GeometryError::PlanarClassificationUnresolved(reason))
                        .into(),
                );
            }
        }
    }
    for first in 0..holes.len() {
        for second in first + 1..holes.len() {
            if !hole_contours[first]
                .intersect_contour(&hole_contours[second], &policy)
                .map_err(GeometryError::from)?
                .is_empty()
            {
                return Err(ConstructionError::IntersectingProfiles);
            }
            for (container, point) in [
                (&hole_contours[first], &holes[second][0]),
                (&hole_contours[second], &holes[first][0]),
            ] {
                match container.classify_point(&curve_point(point), &policy) {
                    Classification::Decided(ContourPointLocation::Inside) => {
                        return Err(ConstructionError::NestedHoles);
                    }
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        return Err(BuildError::Geometry(
                            GeometryError::PlanarClassificationUnresolved(reason),
                        )
                        .into());
                    }
                }
            }
        }
    }
    Ok(())
}

fn contour_from_profile(profile: &[Point2]) -> Result<Contour2, ConstructionError> {
    let mut segments = Vec::with_capacity(profile.len());
    for index in 0..profile.len() {
        let next = (index + 1) % profile.len();
        segments.push(Segment2::Line(
            LineSeg2::try_new(curve_point(&profile[index]), curve_point(&profile[next]))
                .map_err(GeometryError::from)?,
        ));
    }
    Ok(Contour2::try_new(segments).map_err(GeometryError::from)?)
}

fn extrude_normalized_loops(
    loops: &[Vec<Point2>],
    z_min: Real,
    z_max: Real,
) -> Result<(Model, SolidId), ConstructionError> {
    let mut builder = ModelBuilder::new();
    let solid = add_normalized_extrusion(&mut builder, loops, z_min, z_max)?;
    Ok((builder.finish()?, solid))
}

fn add_normalized_extrusion(
    builder: &mut ModelBuilder,
    loops: &[Vec<Point2>],
    z_min: Real,
    z_max: Real,
) -> Result<SolidId, ConstructionError> {
    let shell =
        add_normalized_extrusion_shell(builder, loops, z_min, z_max, ShellDirection::Outward)?;
    Ok(builder.solid(shell, Vec::new())?)
}

#[derive(Clone, Copy)]
enum ShellDirection {
    Outward,
    Inward,
}

fn add_normalized_extrusion_shell(
    builder: &mut ModelBuilder,
    loops: &[Vec<Point2>],
    z_min: Real,
    z_max: Real,
    direction: ShellDirection,
) -> Result<crate::ShellId, ConstructionError> {
    let count = loops.iter().map(Vec::len).sum::<usize>();
    let mut loop_offsets = Vec::with_capacity(loops.len());
    let mut offset = 0;
    for profile in loops {
        loop_offsets.push(offset);
        offset += profile.len();
    }
    let mut points = Vec::with_capacity(2 * count);
    points.extend(loops.iter().flat_map(|profile| {
        profile
            .iter()
            .map(|point| Point3::new(point.x.clone(), point.y.clone(), z_min.clone()))
    }));
    points.extend(loops.iter().flat_map(|profile| {
        profile
            .iter()
            .map(|point| Point3::new(point.x.clone(), point.y.clone(), z_max.clone()))
    }));
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()?;
    let mut edges = HashMap::<(usize, usize), EdgeId>::new();
    let mut faces = Vec::with_capacity(count + 2);

    let reverse_bottom = matches!(direction, ShellDirection::Outward);
    let bottom_loops = loops
        .iter()
        .zip(&loop_offsets)
        .map(|(profile, offset)| {
            let mut indices = (*offset..(*offset + profile.len())).collect::<Vec<_>>();
            let mut parameters = profile.iter().map(curve_point).collect::<Vec<_>>();
            if reverse_bottom {
                indices.reverse();
                parameters.reverse();
            }
            (indices, parameters)
        })
        .collect::<Vec<_>>();
    faces.push(add_planar_region_face(
        builder,
        &points,
        &vertices,
        &mut edges,
        &bottom_loops,
        Point3::new(Real::zero(), Real::zero(), z_min),
        Vector3::x(),
        Vector3::y(),
        if reverse_bottom {
            Orientation::Reversed
        } else {
            Orientation::Forward
        },
    )?);

    let reverse_top = matches!(direction, ShellDirection::Inward);
    let top_loops = loops
        .iter()
        .zip(&loop_offsets)
        .map(|(profile, offset)| {
            let mut indices =
                ((count + offset)..(count + offset + profile.len())).collect::<Vec<_>>();
            let mut parameters = profile.iter().map(curve_point).collect::<Vec<_>>();
            if reverse_top {
                indices.reverse();
                parameters.reverse();
            }
            (indices, parameters)
        })
        .collect::<Vec<_>>();
    faces.push(add_planar_region_face(
        builder,
        &points,
        &vertices,
        &mut edges,
        &top_loops,
        Point3::new(Real::zero(), Real::zero(), z_max),
        Vector3::x(),
        Vector3::y(),
        if reverse_top {
            Orientation::Reversed
        } else {
            Orientation::Forward
        },
    )?);

    let unit_square = [
        CurvePoint2::new(Real::zero(), Real::zero()),
        CurvePoint2::new(Real::one(), Real::zero()),
        CurvePoint2::new(Real::one(), Real::one()),
        CurvePoint2::new(Real::zero(), Real::one()),
    ];
    for (profile, offset) in loops.iter().zip(&loop_offsets) {
        for local in 0..profile.len() {
            let index = offset + local;
            let next = offset + (local + 1) % profile.len();
            let origin = points[index].clone();
            let (side_loop, u, v) = match direction {
                ShellDirection::Outward => (
                    [index, next, count + next, count + index],
                    &points[next] - &origin,
                    &points[count + index] - &origin,
                ),
                ShellDirection::Inward => (
                    [index, count + index, count + next, next],
                    &points[count + index] - &origin,
                    &points[next] - &origin,
                ),
            };
            faces.push(add_planar_face(
                builder,
                &points,
                &vertices,
                &mut edges,
                &side_loop,
                &unit_square,
                origin.clone(),
                u,
                v,
                Orientation::Forward,
            )?);
        }
    }

    Ok(builder.shell(faces)?)
}

#[allow(clippy::too_many_arguments)]
fn add_planar_face(
    builder: &mut ModelBuilder,
    points: &[Point3],
    vertices: &[VertexId],
    edges: &mut HashMap<(usize, usize), EdgeId>,
    loop_vertices: &[usize],
    parameter_points: &[CurvePoint2],
    origin: Point3,
    u: Vector3,
    v: Vector3,
    orientation: Orientation,
) -> Result<crate::FaceId, ConstructionError> {
    add_planar_region_face(
        builder,
        points,
        vertices,
        edges,
        &[(loop_vertices.to_vec(), parameter_points.to_vec())],
        origin,
        u,
        v,
        orientation,
    )
}

#[allow(clippy::too_many_arguments)]
fn add_planar_region_face(
    builder: &mut ModelBuilder,
    points: &[Point3],
    vertices: &[VertexId],
    edges: &mut HashMap<(usize, usize), EdgeId>,
    loops: &[(Vec<usize>, Vec<CurvePoint2>)],
    origin: Point3,
    u: Vector3,
    v: Vector3,
    orientation: Orientation,
) -> Result<crate::FaceId, ConstructionError> {
    let surface = builder.surface(Surface::plane(origin, u, v)?)?;
    let mut wires = Vec::with_capacity(loops.len());
    for (loop_vertices, parameter_points) in loops {
        wires.push(add_planar_wire(
            builder,
            points,
            vertices,
            edges,
            loop_vertices,
            parameter_points,
        )?);
    }
    let outer = wires.remove(0);
    Ok(builder.face(surface, orientation, outer, wires)?)
}

fn add_planar_wire(
    builder: &mut ModelBuilder,
    points: &[Point3],
    vertices: &[VertexId],
    edges: &mut HashMap<(usize, usize), EdgeId>,
    loop_vertices: &[usize],
    parameter_points: &[CurvePoint2],
) -> Result<crate::WireId, ConstructionError> {
    let mut edge_uses = Vec::with_capacity(loop_vertices.len());
    for local in 0..loop_vertices.len() {
        let from = loop_vertices[local];
        let to = loop_vertices[(local + 1) % loop_vertices.len()];
        let key = if from < to { (from, to) } else { (to, from) };
        let edge = if let Some(edge) = edges.get(&key) {
            *edge
        } else {
            let curve =
                builder.curve(Curve3::line(points[key.0].clone(), points[key.1].clone())?)?;
            let edge = builder.edge(
                vertices[key.0],
                vertices[key.1],
                curve,
                ParameterDomain::unit(),
            )?;
            edges.insert(key, edge);
            edge
        };
        let direction = if from < to {
            Direction::Forward
        } else {
            Direction::Reversed
        };
        let line = LineSeg2::try_new(
            parameter_points[local].clone(),
            parameter_points[(local + 1) % parameter_points.len()].clone(),
        )
        .map_err(GeometryError::from)?;
        let pcurve = builder.pcurve(Pcurve::new(Curve2::from(line)))?;
        let parameter_correspondence = match direction {
            Direction::Forward => ParameterCorrespondence::identity(),
            Direction::Reversed => ParameterCorrespondence::affine(-Real::one(), Real::one())?,
        };
        edge_uses.push(builder.edge_use(edge, direction, pcurve, parameter_correspondence)?);
    }
    Ok(builder.wire(edge_uses)?)
}

fn curve_point(point: &Point2) -> CurvePoint2 {
    CurvePoint2::new(point.x.clone(), point.y.clone())
}

fn require_increasing(min: &Real, max: &Real, axis: Axis) -> Result<(), ConstructionError> {
    match compare_reals(min, max, crate::STRICT_PREDICATES) {
        PredicateOutcome::Decided {
            value: std::cmp::Ordering::Less,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(ConstructionError::InvalidBounds(axis)),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(BuildError::Geometry(GeometryError::PredicateUnresolved { needed, stage }).into())
        }
    }
}

fn require_frustum_radii(base_radius: &Real, top_radius: &Real) -> Result<(), ConstructionError> {
    for (left, right) in [(&Real::zero(), top_radius), (top_radius, base_radius)] {
        match compare_reals(left, right, crate::STRICT_PREDICATES) {
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Less,
                ..
            } => {}
            PredicateOutcome::Decided { .. } => {
                return Err(ConstructionError::InvalidFrustumRadii);
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(BuildError::Geometry(GeometryError::PredicateUnresolved {
                    needed,
                    stage,
                })
                .into());
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use hyperlimit::PredicateOutcome;

    use super::*;
    use crate::geometry::SurfaceExactData;
    use crate::{EdgeUseId, ModelCounts, SolidPointLocation};

    // Assertions compare independently constructed exact representations. Keep
    // production predicates strict while allowing the test oracle to finish a
    // comparison when structural certification alone cannot normalize them.
    fn compare_reals(
        left: &Real,
        right: &Real,
        _policy: hyperlimit::PredicatePolicy,
    ) -> hyperlimit::PredicateOutcome<std::cmp::Ordering> {
        hyperlimit::compare_reals(left, right, crate::TEST_ORACLE_PREDICATES)
    }

    fn point3_equal(
        left: &Point3,
        right: &Point3,
        _policy: hyperlimit::PredicatePolicy,
    ) -> hyperlimit::PredicateOutcome<bool> {
        hyperlimit::point3_equal(left, right, crate::TEST_ORACLE_PREDICATES)
    }

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn cuboid_uses_shared_edges_and_independent_face_local_pcurves() {
        let (model, solid) = cuboid(p(-2, -3, -5), p(7, 11, 13)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 8,
                curves: 12,
                pcurves: 24,
                surfaces: 6,
                edges: 12,
                edge_uses: 24,
                wires: 6,
                faces: 6,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(model.solid(solid).unwrap().voids(), &[]);
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &Real::from(2_268),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let total_area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&total_area, &Real::from(1_080), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );
        let bounds = model.bounds().unwrap().unwrap();
        assert_eq!(
            point3_equal(&bounds.mins, &p(-2, -3, -5), crate::STRICT_PREDICATES).value(),
            Some(true)
        );
        assert_eq!(
            point3_equal(&bounds.maxs, &p(7, 11, 13), crate::STRICT_PREDICATES).value(),
            Some(true)
        );
        for edge_index in 0..model.counts().edges {
            let uses = model
                .uses_of_edge(EdgeId::from_index(edge_index).unwrap())
                .unwrap();
            assert_eq!(uses.len(), 2);
            assert_ne!(
                model.edge_use(uses[0]).unwrap().direction(),
                model.edge_use(uses[1]).unwrap().direction()
            );
        }
    }

    #[test]
    fn planar_face_builds_native_spline_regions_in_authored_frames() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let line =
            |x0, y0, x1, y1| Curve2::from(LineSeg2::try_new(cp(x0, y0), cp(x1, y1)).unwrap());
        let outer = CurvePath2::try_new(vec![
            Curve2::try_nurbs(
                2,
                vec![cp(0, 0), cp(2, 0), cp(4, 0)],
                vec![Real::one(), r(2), r(3)],
                vec![r(2), r(2), r(2), r(5), r(5), r(5)],
            )
            .unwrap(),
            line(4, 0, 4, 4),
            line(4, 4, 0, 4),
            line(0, 4, 0, 0),
        ])
        .unwrap()
        .reversed()
        .unwrap();
        let hole_center = cp(2, 2);
        let hole = CurvePath2::try_new(vec![
            Curve2::from(
                CircularArc2::try_from_center(cp(3, 2), cp(1, 2), hole_center.clone(), false)
                    .unwrap(),
            ),
            Curve2::from(
                CircularArc2::try_from_center(cp(1, 2), cp(3, 2), hole_center, false).unwrap(),
            ),
        ])
        .unwrap();
        let (model, face) = planar_face(
            &outer,
            &[hole],
            p(5, -2, 7),
            Vector3::from_xyz(r(2), Real::zero(), Real::zero()),
            Vector3::from_xyz(Real::one(), r(3), Real::zero()),
        )
        .unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 8,
                curves: 8,
                pcurves: 8,
                surfaces: 1,
                edges: 8,
                edge_uses: 8,
                wires: 2,
                faces: 1,
                shells: 1,
                solids: 0,
            }
        );
        assert_eq!(
            model
                .curves()
                .filter(|(_, curve)| curve.kind() == crate::Curve3Kind::Nurbs)
                .count(),
            1
        );
        assert_eq!(
            model
                .pcurves()
                .filter(|(_, pcurve)| pcurve.kind() == hypercurve::CurveFamily2::Nurbs)
                .count(),
            1
        );
        assert_eq!(
            model
                .pcurves()
                .filter(|(_, pcurve)| pcurve.kind() == hypercurve::CurveFamily2::RationalBezier)
                .count(),
            4
        );
        let expected_area = r(96) - r(6) * Real::pi();
        assert_eq!(
            compare_reals(
                &model.face_area(face).unwrap(),
                &expected_area,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let transformed = model
            .transformed(&crate::Matrix4::affine_orthonormal(
                [
                    [Real::zero(), Real::zero(), Real::one()],
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), Real::one(), Real::zero()],
                ],
                [r(-3), r(11), r(2)],
            ))
            .unwrap();
        assert_eq!(
            compare_reals(
                &transformed.face_area(face).unwrap(),
                &expected_area,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = transformed.to_json().unwrap();
        let replayed = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(replayed.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &replayed.face_area(face).unwrap(),
                &expected_area,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let crossing = CurvePath2::try_new(vec![
            line(0, 0, 2, 2),
            line(2, 2, 0, 2),
            line(0, 2, 2, 0),
            line(2, 0, 0, 0),
        ])
        .unwrap();
        assert_eq!(
            planar_face(&crossing, &[], Point3::origin(), Vector3::x(), Vector3::y(),).unwrap_err(),
            ConstructionError::SelfIntersectingProfile
        );
        assert!(matches!(
            planar_face(&outer, &[], Point3::origin(), Vector3::x(), Vector3::x(),),
            Err(ConstructionError::Build(BuildError::Geometry(
                GeometryError::DegeneratePlaneBasis
            )))
        ));
    }

    #[test]
    fn path_extrusion_retains_native_splines_and_exact_through_holes() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let line =
            |x0, y0, x1, y1| Curve2::from(LineSeg2::try_new(cp(x0, y0), cp(x1, y1)).unwrap());
        let outer = CurvePath2::try_new(vec![
            Curve2::try_nurbs(
                2,
                vec![cp(0, 0), cp(2, 0), cp(4, 0)],
                vec![Real::one(), r(2), r(3)],
                vec![r(2), r(2), r(2), r(5), r(5), r(5)],
            )
            .unwrap(),
            line(4, 0, 4, 4),
            line(4, 4, 0, 4),
            line(0, 4, 0, 0),
        ])
        .unwrap()
        .reversed()
        .unwrap();
        let hole_center = cp(2, 2);
        let hole = CurvePath2::try_new(vec![
            Curve2::from(
                CircularArc2::try_from_center(cp(3, 2), cp(1, 2), hole_center.clone(), false)
                    .unwrap(),
            ),
            Curve2::from(
                CircularArc2::try_from_center(cp(1, 2), cp(3, 2), hole_center, false).unwrap(),
            ),
        ])
        .unwrap();
        let (model, solid) =
            extrude_path_region(&outer, &[hole], r(-1), r(2)).expect("exact path extrusion");
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 12,
                curves: 18,
                pcurves: 36,
                surfaces: 8,
                edges: 18,
                edge_uses: 36,
                wires: 10,
                faces: 8,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            model
                .curves()
                .filter(|(_, curve)| curve.kind() == crate::Curve3Kind::Nurbs)
                .count(),
            2
        );
        assert_eq!(
            model
                .curves()
                .filter(|(_, curve)| curve.kind() == crate::Curve3Kind::CircleArc)
                .count(),
            4
        );
        assert_eq!(
            model
                .pcurves()
                .filter(|(_, pcurve)| pcurve.kind() == hypercurve::CurveFamily2::Nurbs)
                .count(),
            2
        );
        let expected_volume = r(48) - r(3) * Real::pi();
        let expected_area = r(80) + r(4) * Real::pi();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected_volume,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&area, &expected_area, crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(1, 1, 0), SolidPointLocation::Inside),
            (p(2, 2, 0), SolidPointLocation::Outside),
            (p(3, 2, 0), SolidPointLocation::Boundary),
            (p(5, 2, 0), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }

        let transformed = model
            .transformed(&crate::Matrix4::affine_orthonormal(
                [
                    [Real::zero(), Real::zero(), Real::one()],
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), Real::one(), Real::zero()],
                ],
                [r(-3), r(11), r(2)],
            ))
            .unwrap();
        assert_eq!(
            compare_reals(
                &transformed.solid_volume(solid).unwrap(),
                &expected_volume,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = transformed.to_json().unwrap();
        let replayed = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(replayed.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &replayed.solid_volume(solid).unwrap(),
                &expected_volume,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let outside_hole = CurvePath2::try_new(vec![
            line(5, 1, 6, 1),
            line(6, 1, 6, 2),
            line(6, 2, 5, 2),
            line(5, 2, 5, 1),
        ])
        .unwrap();
        assert_eq!(
            extrude_path_region(&outer, &[outside_hole], Real::zero(), Real::one()).unwrap_err(),
            ConstructionError::HoleOutside
        );
    }

    #[test]
    fn cylinder_uses_native_shared_circles_and_one_analytic_side_surface() {
        let (model, solid) = cylinder(r(2), r(3)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 8,
                curves: 12,
                pcurves: 24,
                surfaces: 3,
                edges: 12,
                edge_uses: 24,
                wires: 6,
                faces: 6,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(model.solid(solid).unwrap().voids(), &[]);
        assert_eq!(
            model
                .surfaces()
                .filter(|(_, surface)| surface.kind() == crate::SurfaceKind::Cylinder)
                .count(),
            1
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(Real::from(12) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let faces = model.faces().map(|(id, _)| id).collect::<Vec<_>>();
        assert_eq!(
            compare_reals(
                &model.face_area(faces[0]).unwrap(),
                &(Real::from(4) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &model.face_area(faces[2]).unwrap(),
                &(Real::from(3) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(0, 0, 1), SolidPointLocation::Inside),
            (p(2, 0, 1), SolidPointLocation::Boundary),
            (p(0, 0, 0), SolidPointLocation::Boundary),
            (p(3, 0, 1), SolidPointLocation::Outside),
            (p(0, 0, 4), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let translated = model
            .transformed(&crate::Matrix4::affine_translation([r(3), r(-2), r(5)]))
            .unwrap();
        assert_eq!(
            translated.classify_point(solid, &p(3, -2, 6)).unwrap(),
            SolidPointLocation::Inside
        );
        let reflected = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-2, 0, 1)).unwrap(),
            SolidPointLocation::Boundary
        );
        crate::RawModel::from_json(&reflected.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        let json = model.to_json().unwrap();
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded.solid_volume(solid).unwrap(),
                &(Real::from(12) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn sphere_is_one_exact_closed_face_without_fake_seam_topology() {
        let (model, solid) = sphere(r(3)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 0,
                curves: 0,
                pcurves: 0,
                surfaces: 1,
                edges: 0,
                edge_uses: 0,
                wires: 0,
                faces: 1,
                shells: 1,
                solids: 1,
            }
        );
        let (face_id, face) = model.faces().next().unwrap();
        assert!(face.is_whole_surface());
        assert_eq!(face.outer(), None);
        assert_eq!(
            compare_reals(
                &model.face_area(face_id).unwrap(),
                &(r(36) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(36) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let bounds = model.bounds().unwrap().unwrap();
        assert_eq!(
            point3_equal(&bounds.mins, &p(-3, -3, -3), crate::STRICT_PREDICATES).value(),
            Some(true)
        );
        assert_eq!(
            point3_equal(&bounds.maxs, &p(3, 3, 3), crate::STRICT_PREDICATES).value(),
            Some(true)
        );
        for (point, expected) in [
            (p(0, 0, 0), SolidPointLocation::Inside),
            (p(3, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, -3), SolidPointLocation::Boundary),
            (p(4, 0, 0), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }

        let translated = model
            .transformed(&crate::Matrix4::affine_translation([r(5), r(-2), r(7)]))
            .unwrap();
        assert_eq!(
            translated.classify_point(solid, &p(5, -2, 7)).unwrap(),
            SolidPointLocation::Inside
        );
        let reflected = translated
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-8, -2, 7)).unwrap(),
            SolidPointLocation::Boundary
        );
        let decoded = crate::RawModel::from_json(&reflected.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded.solid_volume(solid).unwrap(),
                &(r(36) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn spherical_void_shells_are_exact_inward_and_strictly_nested() {
        let (model, solid) = sphere_with_voids(
            r(5),
            &[SphereVoid {
                center: p(1, 0, 0),
                radius: r(2),
            }],
        )
        .unwrap();
        assert_eq!(model.counts().faces, 2);
        assert_eq!(model.counts().shells, 2);
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(156) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(1, 0, 0)).unwrap(),
            SolidPointLocation::Outside
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 0)).unwrap(),
            SolidPointLocation::Boundary
        );
        assert_eq!(
            model.classify_point(solid, &p(-4, 0, 0)).unwrap(),
            SolidPointLocation::Inside
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        assert!(
            sphere_with_voids(
                r(5),
                &[SphereVoid {
                    center: p(3, 0, 0),
                    radius: r(2),
                }],
            )
            .is_err()
        );
        assert!(
            sphere_with_voids(
                r(5),
                &[
                    SphereVoid {
                        center: p(-1, 0, 0),
                        radius: r(2),
                    },
                    SphereVoid {
                        center: p(1, 0, 0),
                        radius: r(2),
                    },
                ],
            )
            .is_err()
        );
    }

    #[test]
    fn exact_edge_split_updates_affine_pcurves_and_wire_topology() {
        let mut builder = ModelBuilder::new();
        let points = [p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)];
        let vertices = points
            .iter()
            .cloned()
            .map(|point| builder.vertex(point))
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        let mut uses = Vec::new();
        for index in 0..4 {
            let next = (index + 1) % 4;
            let curve = builder
                .curve(Curve3::line(points[index].clone(), points[next].clone()).unwrap())
                .unwrap();
            let edge = builder
                .edge(
                    vertices[index],
                    vertices[next],
                    curve,
                    ParameterDomain::unit(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(
                        CurvePoint2::new(points[index].x.clone(), points[index].y.clone()),
                        CurvePoint2::new(points[next].x.clone(), points[next].y.clone()),
                    )
                    .unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edge,
                        Direction::Forward,
                        pcurve,
                        ParameterCorrespondence::identity(),
                    )
                    .unwrap(),
            );
        }
        let wire = builder.wire(uses).unwrap();
        let surface = builder
            .surface(Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        let face = builder
            .face(surface, Orientation::Forward, wire, Vec::new())
            .unwrap();
        builder.shell(vec![face]).unwrap();
        let model = builder.finish().unwrap();

        let half = (Real::one() / r(2)).unwrap();
        let (split_model, split) = model
            .split_edge(EdgeId::from_index(0).unwrap(), half.clone())
            .unwrap();
        assert_eq!(split.vertex.index(), 4);
        assert_eq!(split.first.index(), 0);
        assert_eq!(split.second.index(), 4);
        assert_eq!(split.edge_uses.len(), 1);
        assert_eq!(split_model.wire(wire).unwrap().edge_uses().len(), 5);
        assert_eq!(
            compare_reals(
                &split_model.vertex(split.vertex).unwrap().point().x,
                &Real::one(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let (first_use, second_use) = split.edge_uses[0];
        assert_eq!(
            compare_reals(
                &split_model
                    .edge_parameter_at(first_use, &Real::one())
                    .unwrap(),
                &half,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &split_model
                    .edge_parameter_at(second_use, &Real::zero())
                    .unwrap(),
                &half,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &split_model.face_area(face).unwrap(),
                &r(4),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&split_model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let reflected = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected
                .edge_use(EdgeUseId::from_index(0).unwrap())
                .unwrap()
                .direction(),
            Direction::Reversed
        );
        let (reversed_split, reversed_ids) = reflected
            .split_edge(EdgeId::from_index(0).unwrap(), half)
            .unwrap();
        assert_eq!(reversed_split.wire(wire).unwrap().edge_uses().len(), 5);
        assert_eq!(
            reversed_split
                .edge_use(reversed_ids.edge_uses[0].0)
                .unwrap()
                .edge(),
            reversed_ids.second
        );
        assert_eq!(
            reversed_split
                .edge_use(reversed_ids.edge_uses[0].1)
                .unwrap()
                .edge(),
            reversed_ids.first
        );
        assert_eq!(
            compare_reals(
                &reversed_split.face_area(face).unwrap(),
                &r(4),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn exact_planar_face_split_authors_one_shared_chord_and_two_valid_faces() {
        let mut builder = ModelBuilder::new();
        let points = [p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)];
        let vertices = points
            .iter()
            .cloned()
            .map(|point| builder.vertex(point))
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        let mut uses = Vec::new();
        for index in 0..4 {
            let next = (index + 1) % 4;
            let curve = builder
                .curve(Curve3::line(points[index].clone(), points[next].clone()).unwrap())
                .unwrap();
            let edge = builder
                .edge(
                    vertices[index],
                    vertices[next],
                    curve,
                    ParameterDomain::unit(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(
                        CurvePoint2::new(points[index].x.clone(), points[index].y.clone()),
                        CurvePoint2::new(points[next].x.clone(), points[next].y.clone()),
                    )
                    .unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edge,
                        Direction::Forward,
                        pcurve,
                        ParameterCorrespondence::identity(),
                    )
                    .unwrap(),
            );
        }
        let wire = builder.wire(uses).unwrap();
        let surface = builder
            .surface(Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        let face = builder
            .face(surface, Orientation::Forward, wire, Vec::new())
            .unwrap();
        let shell = builder.shell(vec![face]).unwrap();
        let model = builder.finish().unwrap();

        let (split_model, split) = model.split_face(face, vertices[0], vertices[2]).unwrap();
        assert_eq!(split.first_face, face);
        assert_eq!(split.first_wire, wire);
        assert_eq!(split_model.counts().edges, 5);
        assert_eq!(split_model.counts().edge_uses, 6);
        assert_eq!(split_model.counts().wires, 2);
        assert_eq!(split_model.counts().faces, 2);
        assert_eq!(
            split_model.shell(shell).unwrap().faces(),
            &[split.first_face, split.second_face]
        );
        assert_eq!(
            split_model
                .edge_use(split.edge_uses[0])
                .unwrap()
                .direction(),
            Direction::Reversed
        );
        assert_eq!(
            split_model
                .edge_use(split.edge_uses[1])
                .unwrap()
                .direction(),
            Direction::Forward
        );
        assert_eq!(
            compare_reals(
                &split_model.face_area(split.first_face).unwrap(),
                &r(2),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &split_model.face_area(split.second_face).unwrap(),
                &r(2),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&split_model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert!(matches!(
            model.split_face(face, vertices[0], vertices[1]),
            Err(crate::TopologyEditError::DegenerateFaceSplit)
        ));
        assert_eq!(model.counts().faces, 1);
    }

    #[test]
    fn exact_planar_face_split_preserves_certified_prisms_and_cylinders() {
        let (model, solid) = cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let face = crate::FaceId::from_index(1).unwrap();
        let (split_model, split) = model
            .split_face(
                face,
                VertexId::from_index(4).unwrap(),
                VertexId::from_index(6).unwrap(),
            )
            .unwrap();
        assert_eq!(split_model.counts().faces, 7);
        assert_eq!(
            compare_reals(
                &split_model.solid_volume(solid).unwrap(),
                &r(8),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &(split_model.face_area(split.first_face).unwrap()
                    + split_model.face_area(split.second_face).unwrap()),
                &r(4),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&split_model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let (cylinder, solid) = cylinder(r(2), r(3)).unwrap();
        let face = crate::FaceId::from_index(1).unwrap();
        let (split_cylinder, split) = cylinder
            .split_face(
                face,
                VertexId::from_index(4).unwrap(),
                VertexId::from_index(6).unwrap(),
            )
            .unwrap();
        assert_eq!(split_cylinder.counts().faces, 7);
        assert_eq!(
            compare_reals(
                &split_cylinder.solid_volume(solid).unwrap(),
                &(r(12) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &(split_cylinder.face_area(split.first_face).unwrap()
                    + split_cylinder.face_area(split.second_face).unwrap()),
                &(r(4) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&split_cylinder.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let outer =
            contour_from_profile(&[p2(0, 0), p2(3, 0), p2(6, 0), p2(6, 6), p2(3, 6), p2(0, 6)])
                .unwrap();
        let hole = contour_from_profile(&[p2(1, 1), p2(1, 2), p2(2, 2), p2(2, 1)]).unwrap();
        let (holed, solids) = extrude_contour_regions(&[(outer, vec![hole])], r(0), r(2)).unwrap();
        let face = crate::FaceId::from_index(1).unwrap();
        let (split_holed, split) = holed
            .split_face(
                face,
                VertexId::from_index(11).unwrap(),
                VertexId::from_index(14).unwrap(),
            )
            .unwrap();
        assert_eq!(
            split_holed.face(split.first_face).unwrap().inner().len()
                + split_holed.face(split.second_face).unwrap().inner().len(),
            1
        );
        assert_eq!(
            compare_reals(
                &split_holed.solid_volume(solids[0]).unwrap(),
                &r(70),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&split_holed.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn exact_curve_driven_face_split_attaches_to_boundary_edges_by_identity() {
        let (model, solid) = cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let face = crate::FaceId::from_index(1).unwrap();
        let outer = model.face(face).unwrap().outer().unwrap();
        let uses = model.wire(outer).unwrap().edge_uses();
        let directed_start = |use_id| {
            let edge_use = model.edge_use(use_id).unwrap();
            let edge = model.edge(edge_use.edge()).unwrap();
            match edge_use.direction() {
                Direction::Forward => edge.start(),
                Direction::Reversed => edge.end(),
            }
        };
        let midpoint = |use_id| {
            let edge = model.edge(model.edge_use(use_id).unwrap().edge()).unwrap();
            let parameter = (edge.domain().start() + edge.domain().end()) / r(2);
            model
                .curve(edge.curve())
                .unwrap()
                .point_at(&parameter.unwrap())
                .unwrap()
        };
        let start = midpoint(uses[0]);
        let end = midpoint(uses[2]);
        let fragment = Curve3::line(start.clone(), end.clone()).unwrap();

        let (split_model, split) = model.split_face_by_curve(face, &fragment).unwrap();
        assert!(split.start_edge.is_some());
        assert!(split.end_edge.is_some());
        assert_eq!(split_model.counts().edges, model.counts().edges + 3);
        assert_eq!(split_model.counts().faces, model.counts().faces + 1);
        assert_eq!(
            compare_reals(
                &split_model.solid_volume(solid).unwrap(),
                &r(8),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &(split_model.face_area(split.face.first_face).unwrap()
                    + split_model.face_area(split.face.second_face).unwrap()),
                &r(4),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&split_model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let vertex_fragment = Curve3::line(
            model
                .vertex(directed_start(uses[0]))
                .unwrap()
                .point()
                .clone(),
            model
                .vertex(directed_start(uses[2]))
                .unwrap()
                .point()
                .clone(),
        )
        .unwrap();
        let (_, vertex_split) = model.split_face_by_curve(face, &vertex_fragment).unwrap();
        assert!(vertex_split.start_edge.is_none());
        assert!(vertex_split.end_edge.is_none());

        let half = (Real::one() / r(2)).unwrap();
        let interior = fragment.point_at(&half).unwrap();
        let invalid = Curve3::line(start.clone(), interior).unwrap();
        assert!(matches!(
            model.split_face_by_curve(face, &invalid),
            Err(
                crate::TopologyEditError::FaceSplitEndpointNotOnOuterBoundary {
                    endpoint: crate::Endpoint::End,
                    ..
                }
            )
        ));
        let unsupported = Curve3::rational_bezier(
            vec![start, fragment.point_at(&half).unwrap(), end],
            vec![Real::one(); 3],
        )
        .unwrap();
        assert!(matches!(
            model.split_face_by_curve(face, &unsupported),
            Err(crate::TopologyEditError::UnsupportedFaceSplitCurve(
                crate::Curve3Kind::RationalBezier
            ))
        ));
    }

    #[test]
    fn exact_multi_trace_face_partition_is_order_and_direction_independent() {
        let (model, solid) = cuboid(p(0, 0, 0), p(3, 3, 2)).unwrap();
        let face = crate::FaceId::from_index(1).unwrap();
        let outer = model.face(face).unwrap().outer().unwrap();
        let uses = model.wire(outer).unwrap().edge_uses();
        let directed_point = |use_id, fraction: &Real| {
            let edge_use = model.edge_use(use_id).unwrap();
            let edge = model.edge(edge_use.edge()).unwrap();
            let span = edge.domain().end() - edge.domain().start();
            let parameter = match edge_use.direction() {
                Direction::Forward => edge.domain().start() + &span * fraction,
                Direction::Reversed => edge.domain().end() - &span * fraction,
            };
            model
                .curve(edge.curve())
                .unwrap()
                .point_at(&parameter)
                .unwrap()
        };
        let trace_at = |numerator: i32| {
            let fraction = (r(numerator) / r(3)).unwrap();
            let complement = Real::one() - &fraction;
            Curve3::line(
                directed_point(uses[0], &fraction),
                directed_point(uses[2], &complement),
            )
            .unwrap()
        };
        let first = trace_at(1);
        let second = trace_at(2);

        let (forward, partition) = model
            .split_face_by_curves(face, &[second.clone(), first.clone()])
            .unwrap();
        let (reordered, reordered_partition) = model
            .split_face_by_curves(
                face,
                &[first.reversed().unwrap(), second.reversed().unwrap()],
            )
            .unwrap();

        assert_eq!(partition.source_face, face);
        assert_eq!(partition.faces.len(), 3);
        assert_eq!(
            partition
                .traces
                .iter()
                .map(|trace| trace.source_index)
                .collect::<Vec<_>>(),
            vec![1, 0]
        );
        assert_eq!(
            reordered_partition
                .traces
                .iter()
                .map(|trace| trace.source_index)
                .collect::<Vec<_>>(),
            vec![0, 1]
        );
        assert!(
            partition
                .traces
                .iter()
                .all(|trace| trace.segments.len() == 1 && trace.splits.len() == 1)
        );
        assert_eq!(forward.to_json().unwrap(), reordered.to_json().unwrap());
        assert_eq!(forward.counts().faces, model.counts().faces + 2);
        assert_eq!(forward.counts().edges, model.counts().edges + 6);
        assert_eq!(
            compare_reals(
                &forward.solid_volume(solid).unwrap(),
                &r(18),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let area = partition
            .faces
            .iter()
            .map(|face| forward.face_area(*face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&area, &r(9), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&forward.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        assert!(matches!(
            model.split_face_by_curves(face, &[first.clone(), first.reversed().unwrap()]),
            Err(crate::TopologyEditError::DuplicateFaceSplitTrace { .. })
        ));

        let directed_vertex = |use_id| {
            let edge_use = model.edge_use(use_id).unwrap();
            let edge = model.edge(edge_use.edge()).unwrap();
            match edge_use.direction() {
                Direction::Forward => edge.start(),
                Direction::Reversed => edge.end(),
            }
        };
        let diagonal = |first_use, second_use| {
            Curve3::line(
                model
                    .vertex(directed_vertex(first_use))
                    .unwrap()
                    .point()
                    .clone(),
                model
                    .vertex(directed_vertex(second_use))
                    .unwrap()
                    .point()
                    .clone(),
            )
            .unwrap()
        };
        let diagonals = [diagonal(uses[0], uses[2]), diagonal(uses[1], uses[3])];
        let (crossed, crossed_partition) = model.split_face_by_curves(face, &diagonals).unwrap();
        let (crossed_reordered, crossed_reordered_partition) = model
            .split_face_by_curves(
                face,
                &[
                    diagonals[1].reversed().unwrap(),
                    diagonals[0].reversed().unwrap(),
                ],
            )
            .unwrap();
        assert_eq!(
            crossed.to_json().unwrap(),
            crossed_reordered.to_json().unwrap()
        );
        assert_eq!(crossed_partition.faces.len(), 4);
        assert_eq!(crossed_reordered_partition.faces.len(), 4);
        assert_eq!(
            crossed_partition
                .traces
                .iter()
                .map(|trace| trace.segments.len())
                .sum::<usize>(),
            3
        );
        assert_eq!(crossed.counts().vertices, model.counts().vertices + 1);
        assert_eq!(crossed.counts().edges, model.counts().edges + 4);
        assert_eq!(crossed.counts().faces, model.counts().faces + 3);
        assert_eq!(
            compare_reals(
                &crossed.solid_volume(solid).unwrap(),
                &r(18),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let crossed_area = crossed_partition
            .faces
            .iter()
            .map(|face| crossed.face_area(*face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&crossed_area, &r(9), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );

        let half = (r(3) / r(2)).unwrap();
        let center_trace = Curve3::line(
            Point3::new(half.clone(), r(0), r(2)),
            Point3::new(half.clone(), r(3), r(2)),
        )
        .unwrap();
        let concurrent = [
            diagonals[0].clone(),
            diagonals[1].clone(),
            center_trace.clone(),
        ];
        let (six_way, six_way_partition) = model.split_face_by_curves(face, &concurrent).unwrap();
        let (six_way_reordered, _) = model
            .split_face_by_curves(
                face,
                &[
                    center_trace.reversed().unwrap(),
                    diagonals[1].reversed().unwrap(),
                    diagonals[0].reversed().unwrap(),
                ],
            )
            .unwrap();
        assert_eq!(
            six_way.to_json().unwrap(),
            six_way_reordered.to_json().unwrap()
        );
        assert_eq!(six_way_partition.faces.len(), 6);
        assert_eq!(
            six_way_partition
                .traces
                .iter()
                .map(|trace| trace.segments.len())
                .sum::<usize>(),
            5
        );
        assert_eq!(six_way.counts().vertices, model.counts().vertices + 3);
        assert_eq!(six_way.counts().edges, model.counts().edges + 8);
        assert_eq!(six_way.counts().faces, model.counts().faces + 5);
        assert_eq!(
            compare_reals(
                &six_way.solid_volume(solid).unwrap(),
                &r(18),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let overlapping = [
            Curve3::line(
                Point3::new(r(0), half.clone(), r(2)),
                Point3::new(r(3), half.clone(), r(2)),
            )
            .unwrap(),
            Curve3::line(
                Point3::new(r(1), half.clone(), r(2)),
                Point3::new(r(2), half, r(2)),
            )
            .unwrap(),
        ];
        assert!(matches!(
            model.split_face_by_curves(face, &overlapping),
            Err(crate::TopologyEditError::OverlappingFaceSplitTraces { .. })
        ));
    }

    #[test]
    fn crossed_circular_face_partition_revalidates_radial_sector_wires() {
        let (model, solid) = cylinder(r(2), r(3)).unwrap();
        let (face, record) = model.faces().nth(1).unwrap();
        let outer = record.outer().unwrap();
        let uses = model.wire(outer).unwrap().edge_uses();
        let directed_vertex = |use_id| {
            let edge_use = model.edge_use(use_id).unwrap();
            let edge = model.edge(edge_use.edge()).unwrap();
            match edge_use.direction() {
                Direction::Forward => edge.start(),
                Direction::Reversed => edge.end(),
            }
        };
        let diagonal = |first_use, second_use| {
            Curve3::line(
                model
                    .vertex(directed_vertex(first_use))
                    .unwrap()
                    .point()
                    .clone(),
                model
                    .vertex(directed_vertex(second_use))
                    .unwrap()
                    .point()
                    .clone(),
            )
            .unwrap()
        };
        let traces = [diagonal(uses[1], uses[3]), diagonal(uses[0], uses[2])];
        let (edited, partition) = model.split_face_by_curves(face, &traces).unwrap();

        assert_eq!(partition.faces.len(), 4);
        assert_eq!(
            compare_reals(
                &edited.solid_volume(solid).unwrap(),
                &(r(12) * Real::pi()),
                crate::STRICT_PREDICATES,
            )
            .value(),
            Some(std::cmp::Ordering::Equal),
        );
        let json = edited.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json,
        );
    }

    #[test]
    fn nested_angular_pcurve_splits_persist_without_expression_blowup() {
        let (model, solid) = cylinder(r(65), r(11)).unwrap();
        let face = crate::FaceId::from_index(1).unwrap();
        let outer = model.face(face).unwrap().outer().unwrap();
        let uses = model.wire(outer).unwrap().edge_uses();
        let directed_point = |use_id, numerator: i32| {
            let edge_use = model.edge_use(use_id).unwrap();
            let edge = model.edge(edge_use.edge()).unwrap();
            let fraction = (r(numerator) / r(3)).unwrap();
            let span = edge.domain().end() - edge.domain().start();
            let offset = span * fraction;
            let parameter = match edge_use.direction() {
                Direction::Forward => edge.domain().start() + offset,
                Direction::Reversed => edge.domain().end() - offset,
            };
            model
                .curve(edge.curve())
                .unwrap()
                .point_at(&parameter)
                .unwrap()
        };
        let opposite = uses[uses.len() / 2];
        let trace = |numerator| {
            Curve3::line(
                directed_point(uses[0], numerator),
                directed_point(opposite, 3 - numerator),
            )
            .unwrap()
        };
        let (edited, partition) = model
            .split_face_by_curves(face, &[trace(2), trace(1)])
            .unwrap();
        assert_eq!(partition.faces.len(), 3);
        assert_eq!(
            compare_reals(
                &edited.solid_volume(solid).unwrap(),
                &(r(46_475) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let json = edited.to_json().unwrap();
        assert!(
            json.len() < 1_048_576,
            "two exact angular splits must retain a compact root-lineage representation"
        );
        let decoded = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(decoded.to_json().unwrap(), json);
    }

    #[test]
    fn tensor_patch_builders_certify_complete_exact_boundary_images() {
        let controls = vec![
            vec![p(0, 0, 0), p(1, 0, 1), p(2, 0, 0)],
            vec![p(0, 1, 1), p(1, 1, 2), p(2, 1, 1)],
            vec![p(0, 2, 0), p(1, 2, 1), p(2, 2, 0)],
        ];
        let weights = vec![
            vec![r(1), r(2), r(1)],
            vec![r(1), r(3), r(1)],
            vec![r(1), r(2), r(1)],
        ];
        let (bezier, face) = rational_bezier_patch(controls.clone(), weights.clone()).unwrap();
        assert_eq!(
            bezier.counts(),
            ModelCounts {
                vertices: 4,
                curves: 4,
                pcurves: 4,
                surfaces: 1,
                edges: 4,
                edge_uses: 4,
                wires: 1,
                faces: 1,
                shells: 1,
                solids: 0,
            }
        );
        assert_eq!(
            bezier
                .surface(bezier.face(face).unwrap().surface())
                .unwrap()
                .kind(),
            crate::SurfaceKind::RationalBezier
        );
        assert_eq!(
            bezier.face_area(face),
            Err(crate::QueryError::Geometry(
                GeometryError::UnsupportedMeasurement
            ))
        );
        crate::RawModel::from_json(&bezier.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        let (split_bezier, split) = bezier
            .split_edge(
                EdgeId::from_index(0).unwrap(),
                (Real::one() / r(2)).unwrap(),
            )
            .unwrap();
        assert_eq!(
            split_bezier
                .wire(bezier.face(face).unwrap().outer().unwrap())
                .unwrap()
                .edge_uses()
                .len(),
            5
        );
        assert_eq!(split.first, EdgeId::from_index(0).unwrap());
        crate::RawModel::from_json(&split_bezier.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        let transformed_bezier = bezier
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        crate::RawModel::from_json(&transformed_bezier.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let mut forged = bezier.edit();
        forged
            .replace_curve(
                crate::Curve3Id::from_index(0).unwrap(),
                Curve3::rational_bezier(
                    vec![p(0, 0, 0), p(1, 0, 9), p(2, 0, 0)],
                    vec![r(1), r(2), r(1)],
                )
                .unwrap(),
            )
            .unwrap();
        let crate::EditError::Validation(report) = forged.commit().unwrap_err() else {
            panic!("forged spline boundary must fail global image validation");
        };
        assert!(
            report
                .errors()
                .contains(&BuildError::EdgeUseSupportMismatch)
        );

        let knots = vec![r(2), r(2), r(2), r(5), r(5), r(5)];
        let (nurbs, face) = nurbs_patch(2, 2, controls, weights, knots.clone(), knots).unwrap();
        assert_eq!(
            nurbs
                .surface(nurbs.face(face).unwrap().surface())
                .unwrap()
                .kind(),
            crate::SurfaceKind::Nurbs
        );
        crate::RawModel::from_json(&nurbs.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        let (split_nurbs, _) = nurbs
            .split_edge(EdgeId::from_index(1).unwrap(), (r(7) / r(2)).unwrap())
            .unwrap();
        crate::RawModel::from_json(&split_nurbs.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        let transformed_nurbs = nurbs
            .transformed(&crate::Matrix4::affine_translation([r(7), r(-3), r(5)]))
            .unwrap();
        crate::RawModel::from_json(&transformed_nurbs.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        let reflected_nurbs = nurbs
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        crate::RawModel::from_json(&reflected_nurbs.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn affine_tensor_patches_have_exact_area_across_native_domains() {
        let controls = vec![
            vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)],
            vec![p(0, 3, 0), p(2, 3, 0), p(4, 3, 0)],
            vec![p(0, 6, 0), p(2, 6, 0), p(4, 6, 0)],
        ];
        let equivalent_one = Real::one() + Real::zero();
        let weights = vec![
            vec![Real::one(), equivalent_one.clone(), Real::one()],
            vec![equivalent_one.clone(), Real::one(), equivalent_one.clone()],
            vec![Real::one(), equivalent_one, Real::one()],
        ];
        let (bezier, bezier_face) =
            rational_bezier_patch(controls.clone(), weights.clone()).unwrap();
        assert_eq!(
            compare_reals(
                &bezier.face_area(bezier_face).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let (split_bezier, _) = bezier
            .split_edge(
                EdgeId::from_index(0).unwrap(),
                (Real::one() / r(2)).unwrap(),
            )
            .unwrap();
        assert_eq!(
            compare_reals(
                &split_bezier.face_area(bezier_face).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let scaled = bezier
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled.face_area(bezier_face).unwrap(),
                &r(144),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let rebuilt_bezier = crate::RawModel::from_json(&scaled.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt_bezier.face_area(bezier_face).unwrap(),
                &r(144),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let knots = vec![r(2), r(2), r(2), r(5), r(5), r(5)];
        let (nurbs, nurbs_face) =
            nurbs_patch(2, 2, controls, weights, knots.clone(), knots).unwrap();
        assert_eq!(
            compare_reals(
                &nurbs.face_area(nurbs_face).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let rebuilt_nurbs = crate::RawModel::from_json(&nurbs.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt_nurbs.face_area(nurbs_face).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn separably_reparameterized_complete_affine_tensor_patches_have_exact_area() {
        let bezier_controls = vec![
            vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)],
            vec![p(0, 3, 0), p(2, 3, 0), p(4, 3, 0)],
            vec![p(0, 6, 0), p(2, 6, 0), p(4, 6, 0)],
        ];
        let bezier_weights = vec![
            vec![r(2), r(4), r(6)],
            vec![r(5), r(10), r(15)],
            vec![r(7), r(14), r(21)],
        ];
        let (bezier, bezier_face) =
            rational_bezier_patch(bezier_controls.clone(), bezier_weights).unwrap();
        assert_eq!(
            compare_reals(
                &bezier.face_area(bezier_face).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let (split_bezier, _) = bezier
            .split_edge(
                EdgeId::from_index(0).unwrap(),
                (Real::one() / r(2)).unwrap(),
            )
            .unwrap();
        assert_eq!(
            compare_reals(
                &split_bezier.face_area(bezier_face).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let scaled_bezier = split_bezier
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled_bezier.face_area(bezier_face).unwrap(),
                &r(144),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let rebuilt_bezier = crate::RawModel::from_json(&scaled_bezier.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt_bezier.face_area(bezier_face).unwrap(),
                &r(144),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let nonseparable_weights = vec![
            vec![r(2), r(4), r(6)],
            vec![r(5), r(11), r(15)],
            vec![r(7), r(14), r(21)],
        ];
        let (nonseparable, nonseparable_face) =
            rational_bezier_patch(bezier_controls, nonseparable_weights).unwrap();
        assert_eq!(
            nonseparable.face_area(nonseparable_face),
            Err(crate::QueryError::Geometry(
                GeometryError::UnsupportedMeasurement
            ))
        );

        let nurbs_controls = vec![
            vec![p(0, 0, 0), p(1, 0, 0), p(4, 0, 0), p(6, 0, 0)],
            vec![p(0, 4, 0), p(1, 4, 0), p(4, 4, 0), p(6, 4, 0)],
            vec![p(0, 8, 0), p(1, 8, 0), p(4, 8, 0), p(6, 8, 0)],
        ];
        let nurbs_weights = vec![
            vec![r(2), r(4), r(10), r(6)],
            vec![r(7), r(14), r(35), r(21)],
            vec![r(4), r(8), r(20), r(12)],
        ];
        let u_knots = vec![r(2), r(2), r(2), r(3), r(5), r(5), r(5)];
        let v_knots = vec![r(7), r(7), r(7), r(11), r(11), r(11)];
        let (nurbs, nurbs_face) =
            nurbs_patch(2, 2, nurbs_controls, nurbs_weights, u_knots, v_knots).unwrap();
        assert_eq!(
            compare_reals(
                &nurbs.face_area(nurbs_face).unwrap(),
                &r(48),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let rebuilt_nurbs = crate::RawModel::from_json(&nurbs.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt_nurbs.face_area(nurbs_face).unwrap(),
                &r(48),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn tensor_patch_shell_identity_stitches_projectively_equal_boundaries() {
        assert_eq!(
            tensor_patch_shell(Vec::new()).unwrap_err(),
            ConstructionError::EmptyPatchShell
        );
        let first_controls = vec![vec![p(0, 0, 0), p(1, 0, 0)], vec![p(0, 1, 0), p(1, 1, 1)]];
        let first_weights = vec![vec![r(1), r(1)], vec![r(1), r(2)]];
        let second_controls = vec![vec![p(1, 0, 0), p(2, 0, 0)], vec![p(1, 1, 1), p(2, 1, 0)]];
        let second_weights = vec![vec![r(3), r(1)], vec![r(6), r(1)]];
        let assert_shell = |model: &Model, faces: &[FaceId], kind| {
            assert_eq!(faces.len(), 2);
            assert_eq!(
                model.counts(),
                ModelCounts {
                    vertices: 6,
                    curves: 7,
                    pcurves: 8,
                    surfaces: 2,
                    edges: 7,
                    edge_uses: 8,
                    wires: 2,
                    faces: 2,
                    shells: 1,
                    solids: 0,
                }
            );
            assert!(faces.iter().all(|face| {
                model
                    .surface(model.face(*face).unwrap().surface())
                    .unwrap()
                    .kind()
                    == kind
            }));
            let shared = model
                .edges()
                .find_map(|(edge, _)| {
                    (model.uses_of_edge(edge).unwrap().len() == 2).then_some(edge)
                })
                .expect("two adjacent tensor patches share one exact edge");
            let uses = model.uses_of_edge(shared).unwrap();
            assert_ne!(
                model.edge_use(uses[0]).unwrap().direction(),
                model.edge_use(uses[1]).unwrap().direction()
            );
            crate::RawModel::from_json(&model.to_json().unwrap())
                .unwrap()
                .validate()
                .unwrap();
        };

        let (bezier, bezier_faces) = tensor_patch_shell(vec![
            TensorPatch::RationalBezier {
                control_points: first_controls.clone(),
                weights: first_weights.clone(),
            },
            TensorPatch::RationalBezier {
                control_points: second_controls.clone(),
                weights: second_weights.clone(),
            },
        ])
        .unwrap();
        assert_shell(&bezier, &bezier_faces, crate::SurfaceKind::RationalBezier);

        let knots = vec![r(0), r(0), r(1), r(1)];
        let (nurbs, nurbs_faces) = tensor_patch_shell(vec![
            TensorPatch::Nurbs {
                u_degree: 1,
                v_degree: 1,
                control_points: first_controls,
                weights: first_weights,
                u_knots: knots.clone(),
                v_knots: knots.clone(),
            },
            TensorPatch::Nurbs {
                u_degree: 1,
                v_degree: 1,
                control_points: second_controls,
                weights: second_weights,
                u_knots: knots.clone(),
                v_knots: knots,
            },
        ])
        .unwrap();
        assert_shell(&nurbs, &nurbs_faces, crate::SurfaceKind::Nurbs);
    }

    #[test]
    fn exact_edge_split_inverts_angular_sweep_without_projection() {
        let mut builder = ModelBuilder::new();
        let radius = r(2);
        let points = [p(2, 0, 0), p(0, 2, 0), p(-2, 0, 0), p(0, -2, 0)];
        let planar = [
            CurvePoint2::new(r(2), r(0)),
            CurvePoint2::new(r(0), r(2)),
            CurvePoint2::new(r(-2), r(0)),
            CurvePoint2::new(r(0), r(-2)),
        ];
        let vertices = points
            .iter()
            .cloned()
            .map(|point| builder.vertex(point))
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        let quarter = (Real::pi() / r(2)).unwrap();
        let mut uses = Vec::new();
        for index in 0..4 {
            let next = (index + 1) % 4;
            let start = &quarter * Real::from(index as i32);
            let end = &quarter * Real::from(index as i32 + 1);
            let curve = builder
                .curve(
                    Curve3::circle_arc(
                        Point3::origin(),
                        Vector3::x(),
                        Vector3::y(),
                        radius.clone(),
                        start.clone(),
                        end.clone(),
                    )
                    .unwrap(),
                )
                .unwrap();
            let edge = builder
                .edge(
                    vertices[index],
                    vertices[next],
                    curve,
                    ParameterDomain::new(start, end).unwrap(),
                )
                .unwrap();
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    CircularArc2::try_from_center(
                        planar[index].clone(),
                        planar[next].clone(),
                        CurvePoint2::new(r(0), r(0)),
                        false,
                    )
                    .unwrap(),
                )))
                .unwrap();
            uses.push(
                builder
                    .edge_use(
                        edge,
                        Direction::Forward,
                        pcurve,
                        ParameterCorrespondence::angular_sweep(),
                    )
                    .unwrap(),
            );
        }
        let wire = builder.wire(uses).unwrap();
        let surface = builder
            .surface(Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap())
            .unwrap();
        let face = builder
            .face(surface, Orientation::Forward, wire, Vec::new())
            .unwrap();
        builder.shell(vec![face]).unwrap();
        let model = builder.finish().unwrap();

        let split_angle = (Real::pi() / r(4)).unwrap();
        let source_use = EdgeUseId::from_index(0).unwrap();
        let rational_parameter = model.pcurve_parameter_at(source_use, &split_angle).unwrap();
        let source_pcurve = model
            .pcurve(model.edge_use(source_use).unwrap().pcurve())
            .unwrap();
        let split_parameter_point = source_pcurve.point_at(&rational_parameter).unwrap();
        assert_eq!(
            compare_reals(
                &split_parameter_point.x,
                &(r(2).sqrt().unwrap()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let (split_model, split) = model
            .split_edge(EdgeId::from_index(0).unwrap(), split_angle)
            .unwrap();
        assert_eq!(split_model.wire(wire).unwrap().edge_uses().len(), 5);
        assert_eq!(
            compare_reals(
                &split_model.face_area(face).unwrap(),
                &(r(4) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(split.edge_uses.len(), 1);
        crate::RawModel::from_json(&split_model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn exact_edge_split_preserves_a_certified_solid() {
        let (model, solid) = cuboid(p(0, 0, 0), p(2, 3, 4)).unwrap();
        let (split, ids) = model
            .split_edge(
                EdgeId::from_index(0).unwrap(),
                (Real::one() / r(2)).unwrap(),
            )
            .unwrap();
        assert_eq!(split.uses_of_edge(ids.first).unwrap().len(), 2);
        assert_eq!(split.uses_of_edge(ids.second).unwrap().len(), 2);
        assert_eq!(
            compare_reals(
                &split.solid_volume(solid).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            split.classify_point(solid, &p(1, 1, 1)).unwrap(),
            SolidPointLocation::Inside
        );

        let (cylinder_model, cylinder_solid) = cylinder(r(2), r(3)).unwrap();
        let (split_cylinder, cylinder_ids) = cylinder_model
            .split_edge(EdgeId::from_index(0).unwrap(), (Real::pi() / r(4)).unwrap())
            .unwrap();
        assert_eq!(
            split_cylinder
                .uses_of_edge(cylinder_ids.first)
                .unwrap()
                .len(),
            2
        );
        assert_eq!(
            split_cylinder
                .uses_of_edge(cylinder_ids.second)
                .unwrap()
                .len(),
            2
        );
        assert_eq!(
            compare_reals(
                &split_cylinder.solid_volume(cylinder_solid).unwrap(),
                &(r(12) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            split_cylinder
                .classify_point(cylinder_solid, &p(0, 0, 1))
                .unwrap(),
            SolidPointLocation::Inside
        );
        crate::RawModel::from_json(&split_cylinder.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        let (frustum_model, frustum_solid) = cone_frustum(r(2), r(1), r(3)).unwrap();
        let (split_frustum, _) = frustum_model
            .split_edge(EdgeId::from_index(0).unwrap(), (Real::pi() / r(4)).unwrap())
            .unwrap();
        assert_eq!(
            compare_reals(
                &split_frustum.solid_volume(frustum_solid).unwrap(),
                &(r(7) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let (torus_model, torus_solid) = torus(r(3), r(1)).unwrap();
        let (split_torus, _) = torus_model
            .split_edge(EdgeId::from_index(0).unwrap(), (Real::pi() / r(4)).unwrap())
            .unwrap();
        assert_eq!(
            compare_reals(
                &split_torus.solid_volume(torus_solid).unwrap(),
                &(r(6) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&split_torus.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn cone_frustum_retains_native_conic_sides_and_exact_queries() {
        let (model, solid) = cone_frustum(r(2), r(1), r(3)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 8,
                curves: 12,
                pcurves: 24,
                surfaces: 3,
                edges: 12,
                edge_uses: 24,
                wires: 6,
                faces: 6,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(7) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let faces = model.faces().map(|(face, _)| face).collect::<Vec<_>>();
        assert_eq!(
            compare_reals(
                &model.face_area(faces[0]).unwrap(),
                &(r(4) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &model.face_area(faces[1]).unwrap(),
                &Real::pi(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let lateral_quarter = (r(3) * Real::pi() * r(10).sqrt().unwrap() / r(4)).unwrap();
        assert_eq!(
            compare_reals(
                &model.face_area(faces[2]).unwrap(),
                &lateral_quarter,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(0, 0, 1), SolidPointLocation::Inside),
            (p(2, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 3), SolidPointLocation::Boundary),
            (p(2, 0, 2), SolidPointLocation::Outside),
            (p(0, 0, 4), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let translated = model
            .transformed(&crate::Matrix4::affine_translation([r(5), r(-2), r(7)]))
            .unwrap();
        assert_eq!(
            translated.classify_point(solid, &p(5, -2, 8)).unwrap(),
            SolidPointLocation::Inside
        );
        let reflected = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-2, 0, 0)).unwrap(),
            SolidPointLocation::Boundary
        );
        let decoded = crate::RawModel::from_json(&reflected.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded.solid_volume(solid).unwrap(),
                &(r(7) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn torus_uses_native_periodic_patches_and_exact_analytic_queries() {
        let (model, solid) = torus(r(3), r(1)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 16,
                curves: 32,
                pcurves: 64,
                surfaces: 1,
                edges: 32,
                edge_uses: 64,
                wires: 16,
                faces: 16,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let total_area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(
                &total_area,
                &(r(12) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let first_face = crate::FaceId::from_index(0).unwrap();
        let quarter = (Real::pi() / r(2)).unwrap();
        let expected_face_area = &quarter * (r(3) * &quarter + Real::one());
        assert_eq!(
            compare_reals(
                &model.face_area(first_face).unwrap(),
                &expected_face_area,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let total_area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(
                &total_area,
                &(r(12) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(3, 0, 0), SolidPointLocation::Inside),
            (p(4, 0, 0), SolidPointLocation::Boundary),
            (p(2, 0, 0), SolidPointLocation::Boundary),
            (p(0, 0, 0), SolidPointLocation::Outside),
            (p(3, 0, 2), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }

        let translated = model
            .transformed(&crate::Matrix4::affine_translation([r(5), r(-2), r(7)]))
            .unwrap();
        assert_eq!(
            translated.classify_point(solid, &p(8, -2, 7)).unwrap(),
            SolidPointLocation::Inside
        );
        let reflected = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-4, 0, 0)).unwrap(),
            SolidPointLocation::Boundary
        );
        let decoded = crate::RawModel::from_json(&reflected.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_uses_shared_periodic_topology_and_exact_profile_queries() {
        let profile = [p2(1, 0), p2(3, 0), p2(3, 2), p2(1, 2)];
        let (model, solid) = revolve(&profile).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 16,
                curves: 32,
                pcurves: 64,
                surfaces: 4,
                edges: 32,
                edge_uses: 64,
                wires: 16,
                faces: 16,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(16) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let total_area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&total_area, &(r(32) * Real::pi()), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(0, 0, 1), SolidPointLocation::Outside),
            (p(2, 0, 1), SolidPointLocation::Inside),
            (p(3, 0, 1), SolidPointLocation::Boundary),
            (p(2, 0, 3), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }

        let reoriented = model
            .transformed(&crate::Matrix4::from_row_major([
                Real::zero(),
                Real::zero(),
                Real::one(),
                r(5),
                Real::one(),
                Real::zero(),
                Real::zero(),
                r(-3),
                Real::zero(),
                Real::one(),
                Real::zero(),
                r(7),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reoriented.classify_point(solid, &p(6, -1, 7)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            compare_reals(
                &reoriented.solid_volume(solid).unwrap(),
                &(r(16) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let reflected = reoriented
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-6, -1, 7)).unwrap(),
            SolidPointLocation::Inside
        );
        let rebuilt = crate::RawModel::from_json(&reflected.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &(r(16) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            rebuilt.classify_point(solid, &p(-6, -1, 7)).unwrap(),
            SolidPointLocation::Inside
        );
    }

    #[test]
    fn revolution_path_retains_exact_nurbs_profile_and_polynomial_volume() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::try_nurbs(
                2,
                vec![cp(4, 0), cp(5, 1), cp(4, 2)],
                vec![Real::one(); 3],
                vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            )
            .unwrap(),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();

        assert_eq!(model.faces().count(), 16);
        assert_eq!(
            model
                .faces()
                .filter(|(face, _)| matches!(
                    model.face_area(*face),
                    Err(crate::QueryError::Geometry(
                        GeometryError::UnsupportedMeasurement
                    ))
                ))
                .count(),
            4
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &((r(148) * Real::pi() / r(5)).unwrap()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert!(model.certified_revolution_profile(solid).is_none());
        for (point, expected) in [
            (p(3, 0, 1), SolidPointLocation::Inside),
            (p(5, 0, 1), SolidPointLocation::Outside),
            (
                Point3::new((r(9) / r(2)).unwrap(), Real::zero(), r(1)),
                SolidPointLocation::Boundary,
            ),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &((r(148) * Real::pi() / r(5)).unwrap()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_nonuniform_rational_line_images_exactly() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(
                hypercurve::RationalBezier2::try_new(
                    vec![cp(4, 0), cp(4, 1), cp(4, 2)],
                    vec![Real::one(), r(2), r(3)],
                )
                .unwrap(),
            ),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::try_nurbs(
                1,
                vec![cp(2, 2), cp(2, 0)],
                vec![r(2), r(5)],
                vec![r(0), r(0), r(1), r(1)],
            )
            .unwrap(),
        ])
        .unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            model
                .faces()
                .filter(|(face, _)| model.face_area(*face).is_err())
                .count(),
            0
        );
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&area, &(r(48) * Real::pi()), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(24) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &(r(24) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let rebuilt_area = rebuilt
            .faces()
            .map(|(face, _)| rebuilt.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(
                &rebuilt_area,
                &(r(48) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_partitions_periodic_spline_carriers_at_exact_native_spans() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let controls = vec![cp(3, 0), cp(5, 0), cp(5, 2), cp(3, 2)];
        let period_knots = (0..=4).map(r).collect::<Vec<_>>();
        let polynomial_profile = CurvePath2::try_new(vec![
            Curve2::try_periodic_polynomial_bspline(2, controls.clone(), period_knots.clone())
                .unwrap(),
        ])
        .unwrap();
        let (model, solid) = revolve_path(&polynomial_profile).unwrap();

        assert_eq!(model.faces().count(), 16);
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &((r(80) * Real::pi() / r(3)).unwrap()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(4, 0, 1), SolidPointLocation::Inside),
            (p(5, 0, 1), SolidPointLocation::Boundary),
            (p(2, 0, 1), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        let rational_profile = CurvePath2::try_new(vec![
            Curve2::try_periodic_nurbs(2, controls, vec![r(1), r(2), r(3), r(4)], period_knots)
                .unwrap(),
        ])
        .unwrap();
        let (rational_model, rational_solid) = revolve_path(&rational_profile).unwrap();
        assert_eq!(rational_model.faces().count(), 16);
        let rational_volume = rational_model.solid_volume(rational_solid).unwrap();
        assert_eq!(
            compare_reals(&rational_volume, &Real::zero(), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Greater)
        );
        let rational_json = rational_model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&rational_json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), rational_json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(rational_solid).unwrap(),
                &rational_volume,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_region_retains_exact_curved_profile_cavity() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let outer = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::try_nurbs(
                2,
                vec![cp(4, 0), cp(5, 1), cp(4, 2)],
                vec![Real::one(); 3],
                vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            )
            .unwrap(),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let half = (Real::one() / r(2)).unwrap();
        let three_halves = (r(3) / r(2)).unwrap();
        let hole_point = |x: i32, y: &Real| CurvePoint2::new(r(x), y.clone());
        let hole = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(hole_point(3, &half), hole_point(4, &half)).unwrap()),
            Curve2::from(
                LineSeg2::try_new(hole_point(4, &half), hole_point(4, &three_halves)).unwrap(),
            ),
            Curve2::from(
                LineSeg2::try_new(hole_point(4, &three_halves), hole_point(3, &three_halves))
                    .unwrap(),
            ),
            Curve2::from(
                LineSeg2::try_new(hole_point(3, &three_halves), hole_point(3, &half)).unwrap(),
            ),
        ])
        .unwrap();

        let (model, solid) = revolve_path_region(&outer, &[hole]).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &((r(113) * Real::pi() / r(5)).unwrap()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(2, 0, 1)).unwrap(),
            SolidPointLocation::Boundary
        );
        assert_eq!(
            model
                .classify_point(solid, &Point3::new(half + r(2), Real::zero(), r(1)))
                .unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model
                .classify_point(
                    solid,
                    &Point3::new((r(7) / r(2)).unwrap(), Real::zero(), r(1)),
                )
                .unwrap(),
            SolidPointLocation::Outside
        );
        let json = model.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn revolution_path_measures_genuinely_rational_profile_exactly() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let rational = hypercurve::RationalQuadraticBezier2::try_new(
            cp(4, 0),
            cp(5, 1),
            cp(4, 2),
            Real::one(),
            (Real::one() / r(2)).unwrap(),
            Real::one(),
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();

        let expected =
            (r(22) * Real::pi() * &(r(81) + r(4) * Real::from(3).sqrt().unwrap() * Real::pi())
                / r(81))
            .unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(5, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );
        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_recovers_degree_elevated_conic_moments_after_replay() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(5, 1), cp(4, 2)],
            vec![Real::one(), (Real::one() / r(2)).unwrap(), Real::one()],
        )
        .unwrap()
        .elevated_to_degree(7)
        .unwrap();
        assert_eq!(rational.degree(), 7);
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected =
            (r(22) * Real::pi() * &(r(81) + r(4) * Real::from(3).sqrt().unwrap() * Real::pi())
                / r(81))
            .unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_genuinely_cubic_quadratic_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let two_thirds = (r(2) / r(3)).unwrap();
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(6, 0), cp(6, 2), cp(4, 2)],
            vec![Real::one(), two_thirds.clone(), two_thirds, Real::one()],
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected =
            (r(8) * Real::pi() * &(r(16) * Real::from(3).sqrt().unwrap() * Real::pi() + r(351))
                / r(81))
            .unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_square_free_cubic_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let one_third = (Real::one() / r(3)).unwrap();
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(6, 0), cp(6, 2), cp(4, 2)],
            vec![Real::one(), Real::one() + &one_third, r(2), r(4)],
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected = r(4)
            * Real::pi()
            * &((r(104) / r(3)).unwrap() + r(16) * r(2).ln().unwrap() - r(11) * Real::pi());
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_repeated_root_cubic_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(6, 0), cp(6, 2), cp(4, 2)],
            vec![Real::one(), r(2), r(4), r(8)],
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected = (r(324) * Real::pi() / r(7)).unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_fully_split_quartic_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(6, 0), cp(6, 1), cp(6, 2), cp(4, 2)],
            vec![Real::one(), r(2), r(4), r(8), r(16)],
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected = (r(59_128) * Real::pi() / r(1_155)).unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_arbitrary_degree_split_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let controls = vec![
            cp(4, 0),
            cp(5, 0),
            cp(6, 0),
            cp(6, 0),
            cp(6, 1),
            cp(6, 1),
            cp(6, 2),
            cp(6, 2),
            cp(5, 2),
            cp(4, 2),
        ];
        let rational = hypercurve::RationalBezier2::try_new(
            controls.clone(),
            vec![
                Real::one(),
                r(2),
                r(4),
                r(8),
                r(16),
                r(32),
                r(64),
                r(128),
                r(256),
                r(512),
            ],
        )
        .unwrap();
        let polynomial_equivalent =
            hypercurve::RationalBezier2::try_new(controls, vec![Real::one(); 10]).unwrap();
        let profile = |curve| {
            CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
                Curve2::from(curve),
                Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
                Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
            ])
            .unwrap()
        };
        let (model, solid) = revolve_path(&profile(rational)).unwrap();
        let (reference, reference_solid) = revolve_path(&profile(polynomial_equivalent)).unwrap();
        let expected_volume = reference.solid_volume(reference_solid).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected_volume,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(7, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected_volume,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_mixed_factor_quartic_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(6, 0), cp(6, 1), cp(6, 2), cp(4, 2)],
            vec![
                Real::one(),
                (r(3) / r(2)).unwrap(),
                (r(7) / r(3)).unwrap(),
                r(4),
                r(8),
            ],
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected =
            (r(4) * Real::pi() * &(r(459) * Real::pi() - r(348) * r(2).ln().unwrap() - r(1_163))
                / r(3))
            .unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_repeated_quadratic_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(6, 0), cp(6, 1), cp(6, 2), cp(4, 2)],
            vec![Real::one(), Real::one(), (r(4) / r(3)).unwrap(), r(2), r(4)],
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected = r(8) * Real::pi() * &(r(3) + Real::pi());
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_measures_distinct_quadratic_weight_profile() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let three_halves = (r(3) / r(2)).unwrap();
        let rational = hypercurve::RationalBezier2::try_new(
            vec![cp(4, 0), cp(6, 0), cp(6, 1), cp(6, 2), cp(4, 2)],
            vec![
                r(2),
                three_halves.clone(),
                three_halves.clone(),
                three_halves,
                r(2),
            ],
        )
        .unwrap();
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
            Curve2::from(rational),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        let expected = (r(24)
            * Real::pi()
            * &(r(-2_605) + r(117) * Real::pi() + r(5_088) * r(2).ln().unwrap())
            / r(625))
        .unwrap();
        let (model, solid) = revolve_path(&profile).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 0, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(6, 0, 1)).unwrap(),
            SolidPointLocation::Outside
        );

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn revolution_path_rejects_exact_self_crossings_before_topology_build() {
        let cp = |x, y| CurvePoint2::new(r(x), r(y));
        let profile = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(2, 2), cp(4, 0)).unwrap()),
            Curve2::from(LineSeg2::try_new(cp(4, 0), cp(2, 0)).unwrap()),
        ])
        .unwrap();
        assert_eq!(
            revolve_path(&profile).unwrap_err(),
            ConstructionError::SelfIntersectingProfile
        );
    }

    #[test]
    fn revolution_rejects_axis_contact_and_self_intersection_exactly() {
        assert!(matches!(
            revolve(&[p2(0, 0), p2(2, 0), p2(2, 1), p2(0, 1)]),
            Err(ConstructionError::ProfileCrossesRevolutionAxis)
        ));
        assert!(matches!(
            revolve(&[p2(-1, 0), p2(2, 0), p2(2, 1), p2(-1, 1)]),
            Err(ConstructionError::ProfileCrossesRevolutionAxis)
        ));
        assert!(matches!(
            revolve(&[p2(1, 0), p2(4, 3), p2(1, 3), p2(3, 0)]),
            Err(ConstructionError::SelfIntersectingProfile)
        ));
    }

    #[test]
    fn revolution_region_retains_exact_toroidal_profile_cavities() {
        let outer = [p2(1, 0), p2(5, 0), p2(5, 4), p2(1, 4)];
        let hole = vec![p2(2, 1), p2(3, 1), p2(3, 2), p2(2, 2)];
        let (model, solid) = revolve_region(&outer, &[hole]).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 32,
                curves: 64,
                pcurves: 128,
                surfaces: 8,
                edges: 64,
                edge_uses: 128,
                wires: 32,
                faces: 32,
                shells: 2,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(91) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(4, 0, 2), SolidPointLocation::Inside),
            (p(2, 0, 1), SolidPointLocation::Boundary),
            (
                Point3::new((r(5) / r(2)).unwrap(), Real::zero(), (r(3) / r(2)).unwrap()),
                SolidPointLocation::Outside,
            ),
            (p(0, 0, 2), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let reflected = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-4, 0, 2)).unwrap(),
            SolidPointLocation::Inside
        );
        let rebuilt = crate::RawModel::from_json(&reflected.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &(r(91) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert!(matches!(
            revolve_region(&outer, &[vec![p2(5, 1), p2(6, 1), p2(6, 2), p2(5, 2)]],),
            Err(ConstructionError::IntersectingProfiles)
        ));
    }

    #[test]
    fn line_arc_profile_revolution_retains_native_curved_meridians() {
        let center = CurvePoint2::new(r(3), Real::zero());
        let profile = Contour2::try_new(vec![
            Segment2::Arc(
                CircularArc2::try_from_center(
                    CurvePoint2::new(r(4), Real::zero()),
                    CurvePoint2::new(r(2), Real::zero()),
                    center.clone(),
                    false,
                )
                .unwrap(),
            ),
            Segment2::Arc(
                CircularArc2::try_from_center(
                    CurvePoint2::new(r(2), Real::zero()),
                    CurvePoint2::new(r(4), Real::zero()),
                    center,
                    false,
                )
                .unwrap(),
            ),
        ])
        .unwrap();
        let (model, solid) = revolve_contour(&profile).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 8,
                curves: 16,
                pcurves: 32,
                surfaces: 2,
                edges: 16,
                edge_uses: 32,
                wires: 8,
                faces: 8,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            model
                .curves()
                .filter(|(_, curve)| curve.kind() == crate::Curve3Kind::CircleArc)
                .count(),
            16
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(3, 0, 0), SolidPointLocation::Inside),
            (p(4, 0, 0), SolidPointLocation::Boundary),
            (p(5, 0, 0), SolidPointLocation::Outside),
            (p(0, 0, 0), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let reflected = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-3, 0, 0)).unwrap(),
            SolidPointLocation::Inside
        );
        let json = reflected.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let circle_profile = |center_x: i32, radius: i32| {
            let center = CurvePoint2::new(r(center_x), Real::zero());
            Contour2::try_new(vec![
                Segment2::Arc(
                    CircularArc2::try_from_center(
                        CurvePoint2::new(r(center_x + radius), Real::zero()),
                        CurvePoint2::new(r(center_x - radius), Real::zero()),
                        center.clone(),
                        false,
                    )
                    .unwrap(),
                ),
                Segment2::Arc(
                    CircularArc2::try_from_center(
                        CurvePoint2::new(r(center_x - radius), Real::zero()),
                        CurvePoint2::new(r(center_x + radius), Real::zero()),
                        center,
                        false,
                    )
                    .unwrap(),
                ),
            ])
            .unwrap()
        };
        let (region, region_solid) =
            revolve_contour_region(&circle_profile(3, 2), &[circle_profile(3, 1)]).unwrap();
        assert_eq!(
            compare_reals(
                &region.solid_volume(region_solid).unwrap(),
                &(r(18) * Real::pi() * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            region.classify_point(region_solid, &p(3, 0, 0)).unwrap(),
            SolidPointLocation::Outside
        );
        assert_eq!(
            region
                .classify_point(
                    region_solid,
                    &Point3::new((r(9) / r(2)).unwrap(), Real::zero(), Real::zero()),
                )
                .unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            revolve_contour(&circle_profile(1, 1)).unwrap_err(),
            ConstructionError::ProfileCrossesRevolutionAxis
        );
    }

    #[test]
    fn line_arc_contour_extrusion_retains_curved_caps_and_side_surface() {
        let contour = Contour2::try_new(vec![
            Segment2::Arc(
                CircularArc2::try_from_center(
                    CurvePoint2::new(r(2), r(0)),
                    CurvePoint2::new(r(0), r(2)),
                    CurvePoint2::new(r(0), r(0)),
                    false,
                )
                .unwrap(),
            ),
            Segment2::Arc(
                CircularArc2::try_from_center(
                    CurvePoint2::new(r(0), r(2)),
                    CurvePoint2::new(r(-2), r(0)),
                    CurvePoint2::new(r(0), r(0)),
                    false,
                )
                .unwrap(),
            ),
            Segment2::Line(
                LineSeg2::try_new(CurvePoint2::new(r(-2), r(0)), CurvePoint2::new(r(2), r(0)))
                    .unwrap(),
            ),
        ])
        .unwrap();
        let (model, solid) = extrude_contour(&contour, r(0), r(3)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 6,
                curves: 9,
                pcurves: 18,
                surfaces: 5,
                edges: 9,
                edge_uses: 18,
                wires: 5,
                faces: 5,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let faces = model.faces().map(|(id, _)| id).collect::<Vec<_>>();
        for (face, expected) in [
            (faces[0], r(2) * Real::pi()),
            (faces[1], r(2) * Real::pi()),
            (faces[2], r(3) * Real::pi()),
            (faces[3], r(3) * Real::pi()),
            (faces[4], r(12)),
        ] {
            assert_eq!(
                compare_reals(
                    &model.face_area(face).unwrap(),
                    &expected,
                    crate::STRICT_PREDICATES
                )
                .value(),
                Some(std::cmp::Ordering::Equal)
            );
        }
        for (point, expected) in [
            (p(0, 1, 1), SolidPointLocation::Inside),
            (p(0, -1, 1), SolidPointLocation::Outside),
            (p(0, 2, 1), SolidPointLocation::Boundary),
            (p(0, 1, 0), SolidPointLocation::Boundary),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let decoded = crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let reoriented = model
            .transformed(&crate::Matrix4::from_row_major([
                Real::zero(),
                Real::zero(),
                Real::one(),
                r(5),
                Real::one(),
                Real::zero(),
                Real::zero(),
                r(-3),
                Real::zero(),
                Real::one(),
                Real::zero(),
                r(7),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &reoriented.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(6, -3, 8), SolidPointLocation::Inside),
            (p(6, -3, 6), SolidPointLocation::Outside),
            (p(6, -3, 9), SolidPointLocation::Boundary),
        ] {
            assert_eq!(reoriented.classify_point(solid, &point).unwrap(), expected);
        }
        let decoded_reoriented = crate::RawModel::from_json(&reoriented.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &decoded_reoriented.solid_volume(solid).unwrap(),
                &(r(6) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            decoded_reoriented
                .classify_point(solid, &p(6, -3, 8))
                .unwrap(),
            SolidPointLocation::Inside
        );
    }

    #[test]
    fn cuboid_rejects_flat_or_reversed_extents_exactly() {
        assert_eq!(
            cuboid(p(0, 0, 0), p(0, 1, 1)).unwrap_err(),
            ConstructionError::InvalidBounds(Axis::X)
        );
        assert_eq!(
            cuboid(p(0, 2, 0), p(1, 1, 1)).unwrap_err(),
            ConstructionError::InvalidBounds(Axis::Y)
        );
    }

    #[test]
    fn cuboid_accepts_representation_distinct_certified_bounds() {
        let two = Real::from(1) + Real::from(1);
        assert!(matches!(
            compare_reals(&two, &Real::from(2), crate::STRICT_PREDICATES),
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Equal,
                ..
            }
        ));
        cuboid(
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(two, Real::from(3), Real::from(4)),
        )
        .unwrap();
    }

    #[test]
    fn cuboid_transform_preserves_ids_and_scales_exact_measurements() {
        let (model, solid) = cuboid(p(0, 0, 0), p(2, 3, 5)).unwrap();
        let transform = crate::Matrix4::from_row_major([
            Real::from(2),
            Real::zero(),
            Real::zero(),
            Real::from(7),
            Real::zero(),
            Real::from(3),
            Real::zero(),
            Real::from(11),
            Real::zero(),
            Real::zero(),
            Real::from(4),
            Real::from(13),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]);
        let transformed = model.transformed(&transform).unwrap();
        assert_eq!(transformed.counts(), model.counts());
        assert_eq!(
            compare_reals(
                &transformed.solid_volume(solid).unwrap(),
                &Real::from(720),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let bounds = transformed.bounds().unwrap().unwrap();
        assert_eq!(
            point3_equal(&bounds.mins, &p(7, 11, 13), crate::STRICT_PREDICATES).value(),
            Some(true)
        );
        assert_eq!(
            point3_equal(&bounds.maxs, &p(11, 20, 33), crate::STRICT_PREDICATES).value(),
            Some(true)
        );
    }

    #[test]
    fn model_transform_rejects_singular_maps_and_repairs_reflected_orientation() {
        let (model, solid) = cuboid(p(0, 0, 0), p(2, 2, 2)).unwrap();
        let singular =
            crate::Matrix4::affine_nonuniform_scale([Real::one(), Real::zero(), Real::one()]);
        assert_eq!(
            model.transformed(&singular).unwrap_err(),
            GeometryError::SingularTransform
        );
        let reflection =
            crate::Matrix4::affine_nonuniform_scale([-Real::one(), Real::one(), Real::one()]);
        let reflected = model.transformed(&reflection).unwrap();
        assert_eq!(
            reflected.classify_point(solid, &p(-1, 1, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            compare_reals(
                &reflected.solid_volume(solid).unwrap(),
                &Real::from(8),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn edit_commit_is_transactional_and_preserves_the_source() {
        let (source, solid) = cuboid(p(0, 0, 0), p(2, 3, 5)).unwrap();
        let translation =
            crate::Matrix4::affine_translation([Real::from(7), Real::from(11), Real::from(13)]);
        let mut edit = source.edit();
        edit.transform(&translation).unwrap();
        let edited = edit.commit().unwrap();
        assert_eq!(
            point3_equal(
                &source.bounds().unwrap().unwrap().mins,
                &p(0, 0, 0),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(true)
        );
        assert_eq!(
            point3_equal(
                &edited.bounds().unwrap().unwrap().mins,
                &p(7, 11, 13),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(true)
        );
        assert_eq!(
            compare_reals(
                &source.solid_volume(solid).unwrap(),
                &edited.solid_volume(solid).unwrap(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn edit_replays_replacements_and_rejects_invalid_staged_geometry() {
        let (source, _) = cuboid(p(0, 0, 0), p(2, 3, 5)).unwrap();
        let (edge_id, edge) = source.edges().next().unwrap();
        let start = source.vertex(edge.start()).unwrap().point().clone();
        let end = source.vertex(edge.end()).unwrap().point().clone();
        let mut valid = source.edit();
        valid
            .replace_curve(edge.curve(), Curve3::line(start, end).unwrap())
            .unwrap();
        let edited = valid.commit().unwrap();
        assert_eq!(edited.counts(), source.counts());
        assert_eq!(
            point3_equal(
                edited
                    .vertex(edited.edge(edge_id).unwrap().start())
                    .unwrap()
                    .point(),
                source.vertex(edge.start()).unwrap().point(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(true)
        );

        let (vertex, _) = source.vertices().next().unwrap();
        let mut invalid = source.edit();
        invalid.replace_vertex(vertex, p(100, 100, 100)).unwrap();
        let error = invalid.commit().unwrap_err();
        let crate::EditError::Validation(report) = error else {
            panic!("invalid edit must fail model validation");
        };
        assert!(matches!(
            report.errors().first(),
            Some(BuildError::EdgeEndpointMismatch { .. })
        ));
        assert_eq!(
            point3_equal(
                &source.bounds().unwrap().unwrap().mins,
                &p(0, 0, 0),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(true)
        );
    }

    #[test]
    fn planar_rational_tensor_cap_retains_exact_prism_certificate() {
        let (source, solid) = cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let (cap_face, cap_surface, origin, u, v) = source
            .faces()
            .find_map(|(face_id, face)| {
                let surface = source.surface(face.surface()).unwrap();
                let SurfaceExactData::Plane { origin, u, v } = surface.exact_data() else {
                    return None;
                };
                (compare_reals(&origin.z, &Real::one(), crate::STRICT_PREDICATES).value()
                    == Some(std::cmp::Ordering::Equal))
                .then(|| {
                    (
                        face_id,
                        face.surface(),
                        origin.clone(),
                        u.clone(),
                        v.clone(),
                    )
                })
            })
            .expect("unit cuboid has an upper planar cap");
        let tensor = Surface::rational_bezier(
            vec![
                vec![origin.clone(), origin.clone() + u.clone()],
                vec![origin.clone() + v.clone(), origin + u + v],
            ],
            vec![vec![Real::one(), Real::one()]; 2],
        )
        .unwrap();
        let mut edit = source.edit();
        edit.replace_surface(cap_surface, tensor).unwrap();
        let edited = edit.commit().unwrap();

        assert_eq!(
            edited.surface(cap_surface).unwrap().kind(),
            crate::SurfaceKind::RationalBezier
        );
        assert_eq!(
            compare_reals(
                &edited.face_area(cap_face).unwrap(),
                &Real::one(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                &edited.solid_volume(solid).unwrap(),
                &Real::one(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            edited
                .classify_point(
                    solid,
                    &Point3::new(
                        (Real::one() / r(2)).unwrap(),
                        (Real::one() / r(2)).unwrap(),
                        (Real::one() / r(2)).unwrap(),
                    ),
                )
                .unwrap(),
            SolidPointLocation::Inside
        );
        let json = edited.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn planar_nurbs_tensor_cap_retains_exact_prism_certificate() {
        let (source, solid) = cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let (cap_surface, origin, u, v) = source
            .faces()
            .find_map(|(_, face)| {
                let surface = source.surface(face.surface()).unwrap();
                let SurfaceExactData::Plane { origin, u, v } = surface.exact_data() else {
                    return None;
                };
                (compare_reals(&origin.z, &Real::one(), crate::STRICT_PREDICATES).value()
                    == Some(std::cmp::Ordering::Equal))
                .then(|| (face.surface(), origin.clone(), u.clone(), v.clone()))
            })
            .expect("unit cuboid has an upper planar cap");
        let knots = vec![Real::zero(), Real::zero(), Real::one(), Real::one()];
        let tensor = Surface::nurbs(
            1,
            1,
            vec![
                vec![origin.clone(), origin.clone() + u.clone()],
                vec![origin.clone() + v.clone(), origin + u + v],
            ],
            vec![vec![Real::one(), Real::one()]; 2],
            knots.clone(),
            knots,
        )
        .unwrap();
        let mut edit = source.edit();
        edit.replace_surface(cap_surface, tensor).unwrap();
        let edited = edit.commit().unwrap();

        assert_eq!(
            edited.surface(cap_surface).unwrap().kind(),
            crate::SurfaceKind::Nurbs
        );
        assert_eq!(
            compare_reals(
                &edited.solid_volume(solid).unwrap(),
                &Real::one(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = edited.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn coplanar_non_affine_tensor_cap_remains_explicit() {
        let (source, _) = cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
        let cap_surface = source
            .faces()
            .find_map(|(_, face)| {
                let surface = source.surface(face.surface()).unwrap();
                let SurfaceExactData::Plane { origin, .. } = surface.exact_data() else {
                    return None;
                };
                (compare_reals(&origin.z, &Real::one(), crate::STRICT_PREDICATES).value()
                    == Some(std::cmp::Ordering::Equal))
                .then_some(face.surface())
            })
            .expect("unit cuboid has an upper planar cap");
        let half = (Real::one() / r(2)).unwrap();
        let three_quarters = (r(3) / r(4)).unwrap();
        let tensor = Surface::rational_bezier(
            vec![
                vec![
                    Point3::new(Real::zero(), Real::zero(), Real::one()),
                    Point3::new(half.clone(), Real::zero(), Real::one()),
                    Point3::new(Real::one(), Real::zero(), Real::one()),
                ],
                vec![
                    Point3::new(Real::zero(), half.clone(), Real::one()),
                    Point3::new(three_quarters, half.clone(), Real::one()),
                    Point3::new(Real::one(), half.clone(), Real::one()),
                ],
                vec![
                    Point3::new(Real::zero(), Real::one(), Real::one()),
                    Point3::new(half, Real::one(), Real::one()),
                    Point3::new(Real::one(), Real::one(), Real::one()),
                ],
            ],
            vec![vec![Real::one(); 3]; 3],
        )
        .unwrap();
        let mut edit = source.edit();
        edit.replace_surface(cap_surface, tensor).unwrap();
        let crate::EditError::Validation(report) = edit.commit().unwrap_err() else {
            panic!("non-affine coplanar tensor must remain explicit unsupported evidence");
        };
        assert!(
            matches!(
                report.errors(),
                [BuildError::EdgeUseSupportMismatch | BuildError::UnsupportedSolidShell(_)]
            ),
            "{report:?}"
        );
    }

    #[test]
    fn extrusion_patch_retains_native_profile_domain_and_validates_bounds() {
        let profile = Curve3::nurbs(
            2,
            vec![p(0, 0, 0), p(2, 1, 0), p(0, 2, 0)],
            vec![Real::one(), r(2), r(3)],
            vec![r(2), r(2), r(2), r(5), r(5), r(5)],
        )
        .unwrap();
        let (model, face) = extrusion_patch(profile.clone(), Vector3::x(), r(-1), r(2)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 4,
                curves: 4,
                pcurves: 4,
                surfaces: 1,
                edges: 4,
                edge_uses: 4,
                wires: 1,
                faces: 1,
                shells: 1,
                solids: 0,
            }
        );
        assert_eq!(
            model
                .curves()
                .map(|(_, curve)| curve.kind())
                .collect::<Vec<_>>(),
            vec![
                crate::Curve3Kind::Nurbs,
                crate::Curve3Kind::Line,
                crate::Curve3Kind::Nurbs,
                crate::Curve3Kind::Line,
            ]
        );
        assert_eq!(
            compare_reals(
                &model.face_area(face).unwrap(),
                &r(6),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();

        assert!(matches!(
            extrusion_patch(profile.clone(), Vector3::x(), r(2), r(2)),
            Err(ConstructionError::Build(BuildError::Geometry(
                GeometryError::InvalidParameterDomain
            )))
        ));
        assert!(matches!(
            extrusion_patch(profile, Vector3::zero(), r(-1), r(2)),
            Err(ConstructionError::Build(BuildError::Geometry(
                GeometryError::DegenerateExtrusionDirection
            )))
        ));
    }

    #[test]
    fn revolution_patch_retains_native_meridians_and_rejects_invalid_single_patches() {
        let profile = Curve3::nurbs(
            2,
            vec![p(2, 0, 0), p(3, 0, 1), p(4, 0, 2)],
            vec![Real::one(), r(2), r(3)],
            vec![r(2), r(2), r(2), r(5), r(5), r(5)],
        )
        .unwrap();
        let quarter = (Real::pi() / r(2)).unwrap();
        let (model, face) = revolution_patch(
            profile.clone(),
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            quarter,
        )
        .unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 4,
                curves: 4,
                pcurves: 4,
                surfaces: 1,
                edges: 4,
                edge_uses: 4,
                wires: 1,
                faces: 1,
                shells: 1,
                solids: 0,
            }
        );
        assert_eq!(
            model
                .curves()
                .map(|(_, curve)| curve.kind())
                .collect::<Vec<_>>(),
            vec![
                crate::Curve3Kind::CircleArc,
                crate::Curve3Kind::Nurbs,
                crate::Curve3Kind::CircleArc,
                crate::Curve3Kind::Nurbs,
            ]
        );
        let expected_area = r(3)
            * Real::pi()
            * r(2)
                .sqrt()
                .expect("positive integer has an exact square root expression");
        assert_eq!(
            compare_reals(
                &model.face_area(face).unwrap(),
                &expected_area,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let transformed = model
            .transformed(&crate::Matrix4::affine_orthonormal(
                [
                    [Real::zero(), Real::zero(), Real::one()],
                    [Real::one(), Real::zero(), Real::zero()],
                    [Real::zero(), Real::one(), Real::zero()],
                ],
                [r(5), r(-2), r(7)],
            ))
            .unwrap();
        assert_eq!(
            compare_reals(
                &transformed.face_area(face).unwrap(),
                &expected_area,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let replayed = crate::RawModel::from_json(&transformed.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &replayed.face_area(face).unwrap(),
                &expected_area,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        assert_eq!(
            revolution_patch(
                profile.clone(),
                Point3::origin(),
                Vector3::z(),
                Real::zero(),
                Real::tau(),
            )
            .unwrap_err(),
            ConstructionError::RevolutionPatchSweepTooLarge
        );
        assert!(matches!(
            revolution_patch(
                profile,
                Point3::origin(),
                Vector3::from_xyz(Real::zero(), Real::zero(), r(2)),
                Real::zero(),
                Real::one(),
            ),
            Err(ConstructionError::Build(BuildError::Geometry(
                GeometryError::InvalidRevolutionAxis
            )))
        ));
        let axis_contact = Curve3::line(Point3::origin(), p(2, 0, 1)).unwrap();
        assert_eq!(
            revolution_patch(
                axis_contact,
                Point3::origin(),
                Vector3::z(),
                Real::zero(),
                Real::one(),
            )
            .unwrap_err(),
            ConstructionError::ProfileCrossesRevolutionAxis
        );
        for interior_axis_contact in [
            Curve3::line(p(-1, 0, 0), p(1, 0, 2)).unwrap(),
            Curve3::rational_bezier(
                vec![p(1, 0, 0), p(-1, 0, 1), p(1, 0, 2)],
                vec![Real::one(); 3],
            )
            .unwrap(),
        ] {
            assert_eq!(
                revolution_patch(
                    interior_axis_contact,
                    Point3::origin(),
                    Vector3::z(),
                    Real::zero(),
                    Real::one(),
                )
                .unwrap_err(),
                ConstructionError::ProfileCrossesRevolutionAxis
            );
        }
    }

    #[test]
    fn concave_clockwise_profile_builds_an_exact_prism() {
        let profile = [p2(0, 4), p2(1, 4), p2(1, 1), p2(4, 1), p2(4, 0), p2(0, 0)];
        let (model, solid) = extrude(&profile, Real::from(2), Real::from(5)).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 12,
                curves: 18,
                pcurves: 36,
                surfaces: 8,
                edges: 18,
                edge_uses: 36,
                wires: 8,
                faces: 8,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &Real::from(21),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, face| sum + face);
        assert_eq!(
            compare_reals(&area, &Real::from(62), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn linear_sweep_retains_exact_affine_frame_and_path() {
        let profile = [p2(0, 0), p2(1, 0), p2(1, 1), p2(0, 1)];
        let (model, solid) = sweep(
            &profile,
            p(1, 2, 3),
            Vector3::from_xyz(r(2), Real::zero(), Real::zero()),
            Vector3::from_xyz(Real::zero(), r(3), Real::zero()),
            Vector3::from_xyz(Real::one(), Real::zero(), r(4)),
        )
        .unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let half = (Real::one() / r(2)).unwrap();
        assert_eq!(
            model
                .classify_point(solid, &Point3::new(r(2) + &half, r(3) + &half, r(5),),)
                .unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(1, 2, 3)).unwrap(),
            SolidPointLocation::Boundary
        );
        let total_area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(
                &total_area,
                &(r(28) + r(6) * r(17).sqrt().unwrap()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let rebuilt = crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &r(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert!(matches!(
            sweep(
                &profile,
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                Vector3::x(),
            ),
            Err(ConstructionError::Build(BuildError::Geometry(
                GeometryError::SingularTransform
            )))
        ));

        let region_outer = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let region_hole = vec![p2(1, 1), p2(3, 1), p2(3, 3), p2(1, 3)];
        let (region, region_solid) = sweep_region(
            &region_outer,
            &[region_hole],
            p(1, 2, 3),
            Vector3::from_xyz(r(2), Real::zero(), Real::zero()),
            Vector3::from_xyz(Real::zero(), r(3), Real::zero()),
            Vector3::from_xyz(Real::one(), Real::zero(), r(4)),
        )
        .unwrap();
        assert_eq!(
            compare_reals(
                &region.solid_volume(region_solid).unwrap(),
                &r(288),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            region
                .classify_point(region_solid, &Point3::new(r(5) + half, r(8), r(5),),)
                .unwrap(),
            SolidPointLocation::Outside
        );
    }

    #[test]
    fn curved_sweep_retains_exact_fixed_frame_path_and_certificate() {
        let profile = [p2(0, 0), p2(2, 0), p2(2, 2), p2(0, 2)];
        let path = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 1), p(0, 0, 4)],
            vec![Real::one(), r(2), r(3)],
        )
        .unwrap();
        let (model, solid) = sweep_curve(&profile, Vector3::x(), Vector3::y(), path).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 8,
                curves: 12,
                pcurves: 24,
                surfaces: 6,
                edges: 12,
                edge_uses: 24,
                wires: 6,
                faces: 6,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            model
                .faces()
                .filter(|(_, face)| {
                    model.surface(face.surface()).unwrap().kind()
                        == crate::SurfaceKind::RationalBezier
                })
                .count(),
            4
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &r(16),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let half = (Real::one() / r(2)).unwrap();
        for (point, expected) in [
            (
                Point3::new(r(1) + &half, Real::one(), r(2)),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(half.clone(), Real::one(), r(2)),
                SolidPointLocation::Boundary,
            ),
            (p(3, 1, 2), SolidPointLocation::Outside),
            (p(1, 1, 0), SolidPointLocation::Boundary),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }

        let scaled = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled.solid_volume(solid).unwrap(),
                &r(384),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            scaled
                .classify_point(solid, &Point3::new(r(3), r(3), r(8)),)
                .unwrap(),
            SolidPointLocation::Inside
        );
        let reflected = scaled
            .transformed(&crate::Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &reflected.solid_volume(solid).unwrap(),
                &r(384),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = reflected.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &r(384),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );

        let nonlinear_progress = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 1), p(0, 0, 4)],
            vec![Real::one(), Real::one(), Real::one()],
        )
        .unwrap();
        assert_eq!(
            sweep_curve(&profile, Vector3::x(), Vector3::y(), nonlinear_progress,).unwrap_err(),
            ConstructionError::NonMonotoneSweepPath
        );
        assert_eq!(
            sweep_curve(
                &profile,
                Vector3::x(),
                Vector3::y(),
                Curve3::line(p(0, 0, 0), p(0, 0, 4)).unwrap(),
            )
            .unwrap_err(),
            ConstructionError::UnsupportedSweepPath
        );
    }

    #[test]
    fn curved_sweep_region_retains_exact_through_hole_certificate() {
        let outer = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let hole = vec![p2(1, 1), p2(3, 1), p2(3, 3), p2(1, 3)];
        let path = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 1), p(0, 0, 4)],
            vec![Real::one(), r(2), r(3)],
        )
        .unwrap();
        let half = (Real::one() / r(2)).unwrap();
        let path_midpoint = path.point_at(&half).unwrap();
        let (model, solid) =
            sweep_curve_region(&outer, &[hole], Vector3::x(), Vector3::y(), path).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 16,
                curves: 24,
                pcurves: 48,
                surfaces: 10,
                edges: 24,
                edge_uses: 48,
                wires: 12,
                faces: 10,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &r(48),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (profile_point, expected) in [
            ((half.clone(), half.clone()), SolidPointLocation::Inside),
            ((r(2), r(2)), SolidPointLocation::Outside),
            ((r(1), r(2)), SolidPointLocation::Boundary),
            ((Real::zero(), r(2)), SolidPointLocation::Boundary),
        ] {
            let point = Point3::new(
                &path_midpoint.x + profile_point.0,
                &path_midpoint.y + profile_point.1,
                path_midpoint.z.clone(),
            );
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &r(48),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let scaled = rebuilt
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled.solid_volume(solid).unwrap(),
                &r(1152),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn moving_frame_sweep_retains_exact_authored_shear_and_region_certificate() {
        let outer = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let hole = vec![p2(1, 1), p2(3, 1), p2(3, 3), p2(1, 3)];
        let frame = RationalBezierSweepFrame::try_new(
            vec![p(0, 0, 0), p(1, 0, 1), p(0, 0, 4)],
            vec![
                Vector3::x(),
                Vector3::from_xyz(Real::one(), Real::one(), Real::zero()),
                Vector3::from_xyz(Real::one(), r(2), Real::zero()),
            ],
            vec![Vector3::y(), Vector3::y(), Vector3::y()],
            vec![Real::one(), r(2), r(3)],
        )
        .unwrap();
        assert_eq!(frame.origins().len(), 3);
        assert_eq!(frame.u_axes().len(), 3);
        assert_eq!(frame.v_axes().len(), 3);
        assert_eq!(frame.weights().len(), 3);

        let (without_hole, without_hole_solid) = sweep_moving_frame(&outer, frame.clone()).unwrap();
        assert_eq!(
            compare_reals(
                &without_hole.solid_volume(without_hole_solid).unwrap(),
                &r(64),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let (model, solid) = sweep_moving_frame_region(&outer, &[hole], frame).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 16,
                curves: 24,
                pcurves: 48,
                surfaces: 10,
                edges: 24,
                edge_uses: 48,
                wires: 12,
                faces: 10,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &r(48),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let half = (Real::one() / r(2)).unwrap();
        for (point, expected) in [
            (
                Point3::new(r(1), (r(9) / r(8)).unwrap(), r(2)),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new((r(5) / r(2)).unwrap(), (r(9) / r(2)).unwrap(), r(2)),
                SolidPointLocation::Outside,
            ),
            (
                Point3::new((r(3) / r(2)).unwrap(), (r(13) / r(4)).unwrap(), r(2)),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(half.clone(), r(2), r(2)),
                SolidPointLocation::Boundary,
            ),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }

        let json = model.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            rebuilt
                .classify_point(
                    solid,
                    &Point3::new((r(5) / r(2)).unwrap(), (r(9) / r(2)).unwrap(), r(2),),
                )
                .unwrap(),
            SolidPointLocation::Outside
        );
        let scaled = rebuilt
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled.solid_volume(solid).unwrap(),
                &r(1152),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            scaled
                .classify_point(solid, &Point3::new(r(2), (r(27) / r(8)).unwrap(), r(8)),)
                .unwrap(),
            SolidPointLocation::Inside
        );
    }

    #[test]
    fn moving_frame_integrates_exact_positive_polynomial_taper() {
        let profile = [p2(0, 0), p2(2, 0), p2(2, 2), p2(0, 2)];
        let frame = RationalBezierSweepFrame::try_new(
            vec![p(0, 0, 0), p(0, 0, 3)],
            vec![
                Vector3::x(),
                Vector3::from_xyz(r(2), Real::zero(), Real::zero()),
            ],
            vec![
                Vector3::y(),
                Vector3::from_xyz(Real::zero(), r(2), Real::zero()),
            ],
            vec![Real::one(), Real::one()],
        )
        .unwrap();
        let (model, solid) = sweep_moving_frame(&profile, frame).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &r(28),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let half = (Real::one() / r(2)).unwrap();
        for (point, expected) in [
            (
                Point3::new(
                    (r(3) / r(2)).unwrap(),
                    (r(3) / r(2)).unwrap(),
                    (r(3) / r(2)).unwrap(),
                ),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(Real::zero(), half, (r(3) / r(2)).unwrap()),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(r(4), r(4), (r(3) / r(2)).unwrap()),
                SolidPointLocation::Outside,
            ),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let rebuilt = crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &r(28),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let scaled = rebuilt
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled.solid_volume(solid).unwrap(),
                &r(672),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            scaled
                .classify_point(solid, &Point3::new(r(3), (r(9) / r(2)).unwrap(), r(6)),)
                .unwrap(),
            SolidPointLocation::Inside
        );
    }

    #[test]
    fn moving_frame_rejects_incomplete_nonplanar_and_uncertified_area_authorship() {
        assert_eq!(
            RationalBezierSweepFrame::try_new(
                vec![p(0, 0, 0), p(0, 0, 4)],
                vec![Vector3::x()],
                vec![Vector3::y(), Vector3::y()],
                vec![Real::one(), Real::one()],
            )
            .unwrap_err(),
            ConstructionError::InvalidSweepFrame
        );
        assert_eq!(
            RationalBezierSweepFrame::try_new(
                vec![p(0, 0, 0), p(0, 0, 4)],
                vec![
                    Vector3::x(),
                    Vector3::from_xyz(Real::one(), Real::zero(), Real::one()),
                ],
                vec![Vector3::y(), Vector3::y()],
                vec![Real::one(), Real::one()],
            )
            .unwrap_err(),
            ConstructionError::NonPlanarSweepFrame
        );
        assert_eq!(
            RationalBezierSweepFrame::try_new(
                vec![p(0, 0, 0), p(0, 0, 4)],
                vec![Vector3::x(), Vector3::zero()],
                vec![Vector3::y(), Vector3::y()],
                vec![Real::one(), Real::one()],
            )
            .unwrap_err(),
            ConstructionError::NonPositiveSweepFrameArea
        );
        assert_eq!(
            RationalBezierSweepFrame::try_new(
                vec![p(0, 0, 0), p(1, 0, 1), p(0, 0, 4)],
                vec![
                    Vector3::x(),
                    Vector3::from_xyz(r(2), Real::zero(), Real::zero()),
                    Vector3::from_xyz(r(3), Real::zero(), Real::zero()),
                ],
                vec![Vector3::y(), Vector3::y(), Vector3::y()],
                vec![Real::one(), r(2), r(3)],
            )
            .unwrap_err(),
            ConstructionError::UnsupportedRationalSweepFrameArea
        );
    }

    #[test]
    fn loft_builds_exact_homothetic_and_convex_corresponding_topology() {
        let sections = [
            LoftSection {
                profile: vec![p2(0, 0), p2(2, 0), p2(2, 2), p2(0, 2)],
                z: Real::zero(),
            },
            LoftSection {
                profile: vec![p2(1, 1), p2(5, 1), p2(5, 5), p2(1, 5)],
                z: r(3),
            },
        ];
        let (model, solid) = loft(&sections).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 8,
                curves: 12,
                pcurves: 24,
                surfaces: 6,
                edges: 12,
                edge_uses: 24,
                wires: 6,
                faces: 6,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &r(28),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(2, 2, 1), SolidPointLocation::Inside),
            (p(0, 0, 1), SolidPointLocation::Outside),
            (p(0, 0, 0), SolidPointLocation::Boundary),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let total_area = model
            .faces()
            .map(|(face, _)| model.face_area(face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(
                &total_area,
                &(r(20) + r(6) * r(10).sqrt().unwrap() + r(18) * r(2).sqrt().unwrap()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let scaled = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled.solid_volume(solid).unwrap(),
                &r(672),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            scaled.classify_point(solid, &p(4, 6, 4)).unwrap(),
            SolidPointLocation::Inside
        );
        let rebuilt = crate::RawModel::from_json(&scaled.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &r(672),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let non_homothetic = [
            sections[0].clone(),
            LoftSection {
                profile: vec![p2(1, 1), p2(5, 1), p2(4, 5), p2(1, 5)],
                z: r(3),
            },
        ];
        let (general, general_solid) = loft(&non_homothetic).unwrap();
        assert_eq!(
            general
                .faces()
                .filter(|(_, face)| {
                    general.surface(face.surface()).unwrap().kind()
                        == crate::SurfaceKind::RationalBezier
                })
                .count(),
            4
        );
        assert_eq!(
            compare_reals(
                &general.solid_volume(general_solid).unwrap(),
                &(r(51) / r(2)).unwrap(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (
                Point3::new(r(1), r(1), (r(3) / r(2)).unwrap()),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(
                    (r(7) / r(2)).unwrap(),
                    (r(1) / r(2)).unwrap(),
                    (r(3) / r(2)).unwrap(),
                ),
                SolidPointLocation::Boundary,
            ),
            (p(4, 4, 1), SolidPointLocation::Outside),
        ] {
            assert_eq!(
                general.classify_point(general_solid, &point).unwrap(),
                expected
            );
        }
        let general_json = general.to_json().unwrap();
        let general_rebuilt = crate::RawModel::from_json(&general_json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(general_rebuilt.to_json().unwrap(), general_json);
        assert_eq!(
            compare_reals(
                &general_rebuilt.solid_volume(general_solid).unwrap(),
                &(r(51) / r(2)).unwrap(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let general_scaled = general
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &general_scaled.solid_volume(general_solid).unwrap(),
                &r(612),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            general_scaled
                .classify_point(general_solid, &p(2, 3, 6))
                .unwrap(),
            SolidPointLocation::Inside
        );
        let incompatible = [
            sections[0].clone(),
            LoftSection {
                profile: vec![p2(1, 1), p2(5, 1), p2(2, 2), p2(1, 5)],
                z: r(3),
            },
        ];
        assert!(matches!(
            loft(&incompatible),
            Err(ConstructionError::IncompatibleLoftSections)
        ));
        assert!(matches!(
            loft(&sections[..1]),
            Err(ConstructionError::LoftNeedsAtLeastTwoSections)
        ));
    }

    #[test]
    fn multi_section_loft_retains_exact_c0_rings_and_piecewise_certificates() {
        let sections = [
            LoftSection {
                profile: vec![p2(0, 0), p2(2, 0), p2(2, 2), p2(0, 2)],
                z: Real::zero(),
            },
            LoftSection {
                profile: vec![p2(1, 1), p2(5, 1), p2(5, 5), p2(1, 5)],
                z: r(2),
            },
            LoftSection {
                profile: vec![p2(0, 0), p2(6, 0), p2(6, 3), p2(0, 3)],
                z: r(5),
            },
        ];
        let (model, solid) = loft(&sections).unwrap();
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 12,
                curves: 20,
                pcurves: 40,
                surfaces: 10,
                edges: 20,
                edge_uses: 40,
                wires: 10,
                faces: 10,
                shells: 1,
                solids: 1,
            }
        );
        assert_eq!(
            model
                .faces()
                .filter(|(_, face)| {
                    model.surface(face.surface()).unwrap().kind()
                        == crate::SurfaceKind::RationalBezier
                })
                .count(),
            4
        );
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &(r(212) / r(3)).unwrap(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (point, expected) in [
            (p(1, 1, 1), SolidPointLocation::Inside),
            (p(2, 2, 2), SolidPointLocation::Inside),
            (
                Point3::new((r(11) / r(2)).unwrap(), r(2), (r(7) / r(2)).unwrap()),
                SolidPointLocation::Boundary,
            ),
            (p(7, 2, 4), SolidPointLocation::Outside),
        ] {
            assert_eq!(model.classify_point(solid, &point).unwrap(), expected);
        }
        let scaled = model
            .transformed(&crate::Matrix4::affine_nonuniform_scale([r(2), r(3), r(4)]))
            .unwrap();
        assert_eq!(
            compare_reals(
                &scaled.solid_volume(solid).unwrap(),
                &r(1_696),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = scaled.to_json().unwrap();
        let rebuilt = crate::RawModel::from_json(&json)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(rebuilt.to_json().unwrap(), json);
        assert_eq!(
            compare_reals(
                &rebuilt.solid_volume(solid).unwrap(),
                &r(1_696),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn extrusion_with_a_through_hole_builds_one_valid_genus_shell() {
        let outer = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let hole = vec![p2(1, 1), p2(3, 1), p2(3, 3), p2(1, 3)];
        let (model, solid) = extrude_region(&outer, &[hole], Real::zero(), Real::from(2)).unwrap();
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &Real::from(24),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model
                .classify_point(
                    solid,
                    &Point3::new(
                        (Real::one() / Real::from(2)).unwrap(),
                        (Real::one() / Real::from(2)).unwrap(),
                        Real::one(),
                    ),
                )
                .unwrap(),
            crate::SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(2, 2, 1)).unwrap(),
            crate::SolidPointLocation::Outside
        );
        assert_eq!(
            model.counts(),
            ModelCounts {
                vertices: 16,
                curves: 24,
                pcurves: 48,
                surfaces: 10,
                edges: 24,
                edge_uses: 48,
                wires: 12,
                faces: 10,
                shells: 1,
                solids: 1,
            }
        );
    }

    #[test]
    fn extrusion_void_shells_are_nested_inward_and_change_material_queries() {
        let outer = [p2(0, 0), p2(10, 0), p2(10, 10), p2(0, 10)];
        let voids = [
            ExtrusionVoid {
                profile: vec![p2(2, 2), p2(4, 2), p2(4, 4), p2(2, 4)],
                z_min: Real::from(2),
                z_max: Real::from(8),
            },
            ExtrusionVoid {
                profile: vec![p2(6, 6), p2(8, 6), p2(8, 8), p2(6, 8)],
                z_min: Real::from(3),
                z_max: Real::from(7),
            },
        ];
        let (model, solid) =
            extrude_with_voids(&outer, Real::zero(), Real::from(10), &voids).unwrap();
        assert_eq!(model.solid(solid).unwrap().voids().len(), 2);
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).unwrap(),
                &Real::from(960),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            model.classify_point(solid, &p(1, 1, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 3, 5)).unwrap(),
            SolidPointLocation::Outside
        );
        assert_eq!(
            model.classify_point(solid, &p(2, 3, 5)).unwrap(),
            SolidPointLocation::Boundary
        );
        assert_eq!(
            model.classify_point(solid, &p(3, 3, 1)).unwrap(),
            SolidPointLocation::Inside
        );

        let reflection =
            crate::Matrix4::affine_nonuniform_scale([-Real::one(), Real::one(), Real::one()]);
        let reflected = model.transformed(&reflection).unwrap();
        assert_eq!(
            compare_reals(
                &reflected.solid_volume(solid).unwrap(),
                &Real::from(960),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            reflected.classify_point(solid, &p(-3, 3, 5)).unwrap(),
            SolidPointLocation::Outside
        );
    }

    #[test]
    fn separated_voids_may_share_a_planar_footprint_but_contact_is_rejected() {
        let outer = [p2(0, 0), p2(10, 0), p2(10, 10), p2(0, 10)];
        let profile = vec![p2(2, 2), p2(4, 2), p2(4, 4), p2(2, 4)];
        let separated = [
            ExtrusionVoid {
                profile: profile.clone(),
                z_min: Real::one(),
                z_max: Real::from(3),
            },
            ExtrusionVoid {
                profile: profile.clone(),
                z_min: Real::from(4),
                z_max: Real::from(6),
            },
        ];
        extrude_with_voids(&outer, Real::zero(), Real::from(10), &separated).unwrap();

        let touching = [
            separated[0].clone(),
            ExtrusionVoid {
                profile: profile.clone(),
                z_min: Real::from(3),
                z_max: Real::from(6),
            },
        ];
        assert!(matches!(
            extrude_with_voids(&outer, Real::zero(), Real::from(10), &touching),
            Err(ConstructionError::Build(
                BuildError::IntersectingVoidShells { .. }
            ))
        ));

        let outside = [ExtrusionVoid {
            profile,
            z_min: Real::zero(),
            z_max: Real::from(3),
        }];
        assert!(matches!(
            extrude_with_voids(&outer, Real::zero(), Real::from(10), &outside),
            Err(ConstructionError::Build(BuildError::VoidShellOutside(_)))
        ));
    }

    #[test]
    fn prism_rejects_small_and_self_intersecting_profiles() {
        assert_eq!(
            extrude(&[p2(0, 0), p2(1, 0)], Real::zero(), Real::one()).unwrap_err(),
            ConstructionError::ProfileTooSmall
        );
        assert_eq!(
            extrude(
                &[p2(0, 0), p2(2, 2), p2(0, 2), p2(2, 0)],
                Real::zero(),
                Real::one(),
            )
            .unwrap_err(),
            ConstructionError::SelfIntersectingProfile
        );
    }

    #[test]
    fn exact_solid_point_classification_covers_interior_exterior_and_boundary() {
        let (box_model, box_solid) = cuboid(p(0, 0, 0), p(4, 6, 8)).unwrap();
        assert_eq!(
            box_model.classify_point(box_solid, &p(1, 2, 3)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            box_model.classify_point(box_solid, &p(5, 2, 3)).unwrap(),
            SolidPointLocation::Outside
        );
        for boundary in [p(0, 2, 3), p(0, 0, 3), p(0, 0, 0)] {
            assert_eq!(
                box_model.classify_point(box_solid, &boundary).unwrap(),
                SolidPointLocation::Boundary
            );
        }

        let profile = [p2(0, 0), p2(8, 0), p2(8, 2), p2(2, 2), p2(2, 8), p2(0, 8)];
        let (concave, solid) = extrude(&profile, Real::zero(), Real::from(3)).unwrap();
        assert_eq!(
            concave.classify_point(solid, &p(1, 6, 1)).unwrap(),
            SolidPointLocation::Inside
        );
        assert_eq!(
            concave.classify_point(solid, &p(6, 6, 1)).unwrap(),
            SolidPointLocation::Outside
        );
    }
}
