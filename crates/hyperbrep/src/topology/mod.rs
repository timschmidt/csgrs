//! Immutable typed-arena topology and validated construction.

use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt;
use std::sync::{Arc, OnceLock};

use hypercurve::{
    BooleanOp, CircularArc2, Classification, Contour2, ContourPointLocation, CubicBezier2, Curve2,
    CurveFamily2, CurveGeometry2, CurvePath2, CurveRegion2, LineLineIntersection, LineSeg2,
    Point2 as CurvePoint2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    RegionPointLocation, Segment2,
};
use hyperlattice::{Aabb, Matrix4, Point2, Point3, Real, Vector3};
use hyperlimit::{PredicateOutcome, compare_point3_lexicographic, compare_reals, point3_equal};

use crate::error::GeometryError;
use crate::geometry::{
    Curve3, Curve3ExactData, Curve3Kind, CurveParameterLocation, EllipseArcExactData,
    ParameterDomain, Pcurve, Surface, SurfaceExactData, SurfaceIntersectionBoundaryContact,
    SurfaceIntersectionCurve, SurfaceIntersectionOperand, SurfaceIntersectionPcurve,
    SurfaceIsoAxis, SurfaceKind, SurfaceParameterDomain, SurfacePcurveCorrespondence,
    affine_transform_orientation, materialize_nurbs_parameter_graph,
    rational_bilinear_parameter_curve, resolve_planar_classification,
};
use crate::{GeometricPoint3, Parameter, ParameterRange};

mod builder;
mod correspondence;
mod edit;
mod error;
mod helpers;
mod ids;
mod measure;
mod model;
mod query;
mod records;
mod transfer;
mod transform;
mod validation;
mod wire_geometry;

pub use correspondence::ParameterCorrespondence;
pub use edit::*;
pub use error::*;
use helpers::*;
pub(crate) use helpers::{
    certified_sweep_progress, circle_data_parameterizations_equal, compare_curve3_exact_data,
};
pub use ids::*;
use records::FaceBoundary;
pub use records::*;

#[cfg(test)]
mod builder_tests;
#[cfg(test)]
mod tests;

enum FaceSplitEndpointLocation {
    Vertex(VertexId),
    Edge {
        edge: EdgeId,
        parameter: Real,
        pcurve_split: Option<Box<(EdgeUseId, SurfaceIntersectionBoundaryContact)>>,
    },
}

struct OrderedFaceSplitTrace {
    source_index: usize,
    curve: Curve3,
    lower: Point3,
    upper: Point3,
}

struct OrderedSurfaceCurveTrace {
    source_index: usize,
    intersection: SurfaceIntersectionCurve,
    exact_key: Curve3,
}

/// Immutable validated BREP arena.
#[derive(Clone, Debug)]
pub struct ArenaModel {
    data: Arc<ModelData>,
}

/// Transactional copy-on-write model edit.
///
/// Replacement operations may temporarily violate model invariants so related
/// records can be changed together. [`ArenaEdit::commit`] replays the entire staged
/// snapshot through [`ArenaModelBuilder`]; failure leaves the source model intact.
#[derive(Clone, Debug)]
pub struct ArenaEdit {
    staged: ArenaModel,
}

#[derive(Clone, Debug)]
struct ModelData {
    vertices: Vec<Vertex>,
    curves: Vec<Curve3>,
    pcurves: Vec<Pcurve>,
    surfaces: Vec<Surface>,
    edges: Vec<Edge>,
    edge_uses: Vec<EdgeUse>,
    wires: Vec<Wire>,
    faces: Vec<Face>,
    shells: Vec<Shell>,
    solids: Vec<Solid>,
    vertex_edges: Vec<Vec<EdgeId>>,
    edge_uses_by_edge: Vec<Vec<EdgeUseId>>,
    edge_use_wire: Vec<WireId>,
    wire_face: Vec<FaceId>,
    face_shell: Vec<ShellId>,
    shell_solid: Vec<Option<SolidId>>,
    solid_certificates_valid: Vec<bool>,
    certified_cylinders: Vec<Option<CertifiedCylinderShell>>,
    certified_spheres: Vec<Option<CertifiedSphereShell>>,
    certified_sphere_pairs: Vec<Option<CertifiedSpherePairShell>>,
    certified_cone_frustums: Vec<Option<CertifiedConeFrustumShell>>,
    certified_tori: Vec<Option<CertifiedTorusShell>>,
    certified_revolutions: Vec<Option<CertifiedRevolutionShell>>,
    certified_lofts: Vec<Option<CertifiedLoftShell>>,
    certified_curve_sweeps: Vec<Option<CertifiedCurveSweepShell>>,
    certified_prisms: Vec<Option<CertifiedPrismShell>>,
    certified_axial_shells: Vec<Option<CertifiedAxialShell>>,
    bounds: OnceLock<Option<Aabb>>,
    face_contours: Vec<OnceLock<Vec<Contour2>>>,
}

#[derive(Clone, Debug)]
struct CertifiedSolid {
    cylinder: Option<CertifiedCylinderShell>,
    sphere: Option<CertifiedSphereShell>,
    sphere_pair: Option<CertifiedSpherePairShell>,
    cone_frustum: Option<CertifiedConeFrustumShell>,
    torus: Option<CertifiedTorusShell>,
    revolution: Option<CertifiedRevolutionShell>,
    loft: Option<CertifiedLoftShell>,
    curve_sweep: Option<CertifiedCurveSweepShell>,
    prism: Option<CertifiedPrismShell>,
    axial: Option<CertifiedAxialShell>,
}

/// A complete oriented boundary proof, reconstructed from native faces. Each
/// open height band has this exact material section. The frame is orthonormal
/// on admission; validated transforms may carry an invertible affine frame.
#[derive(Clone, Debug)]
struct CertifiedAxialShell {
    origin: Point3,
    u: Vector3,
    v: Vector3,
    axis: Vector3,
    heights: Vec<Real>,
    bands: Vec<CertifiedAxialBand>,
}

#[derive(Clone, Debug)]
struct CertifiedAxialBand {
    region: CurveRegion2,
    area: Real,
}

/// Builder-owned proofs have two scopes: admission is partial, whereas a
/// validated transfer preserves its complete set of available certificates.
/// A missing admission proof must be retried, never cached as impossible.
#[derive(Debug)]
enum BuilderSolidCertificates {
    Admitted(CertifiedSolid),
    Transferred(CertifiedSolid),
}

impl BuilderSolidCertificates {
    fn reuse<T: Clone>(
        &self,
        family: impl FnOnce(&CertifiedSolid) -> &Option<T>,
        admissible: bool,
    ) -> Option<Option<T>> {
        match self {
            Self::Transferred(certificates) => Some(family(certificates).clone()),
            Self::Admitted(certificates) if admissible => family(certificates)
                .as_ref()
                .map(|proof| Some(proof.clone())),
            Self::Admitted(_) => None,
        }
    }
}

#[derive(Clone, Debug)]
struct CertifiedZPrismShell {
    contour: Contour2,
    z_min: Real,
    z_max: Real,
}

#[derive(Clone, Debug)]
struct CertifiedPrismShell {
    profile: CertifiedPrismProfile,
    origin: Point3,
    u: Vector3,
    v: Vector3,
    extrusion: Vector3,
    parameter_min: Real,
    parameter_max: Real,
}

#[derive(Clone, Debug)]
enum CertifiedPrismProfile {
    Paths {
        outer: CurvePath2,
        holes: Vec<CurvePath2>,
        // Profile-coordinate area derived from admitted cap geometry on every
        // validation, including archive replay; never trusted archive data.
        area: Option<Real>,
    },
    CurveRegion {
        region: CurveRegion2,
        area: Option<Real>,
    },
}

#[derive(Clone, Debug)]
struct CertifiedCylinderShell {
    origin: Point3,
    axis: Vector3,
    radius: Real,
    v_min: Real,
    v_max: Real,
    sphere_subtraction: Option<CertifiedCylinderSphereSubtraction>,
}

#[derive(Clone, Debug)]
enum CertifiedCylinderSphereSubtraction {
    Void {
        center: Point3,
        radius: Real,
    },
    Component {
        center: Point3,
        radius: Real,
        side: CertifiedCylinderSphereComponentSide,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CertifiedCylinderSphereComponentSide {
    Lower,
    Upper,
}

#[derive(Clone, Debug)]
struct CertifiedSphereShell {
    center: Point3,
    radius: Real,
    voids: Vec<CertifiedSphereVoid>,
    region: CertifiedSphereRegion,
}

#[derive(Clone, Debug)]
enum CertifiedSphereVoid {
    Sphere { center: Point3, radius: Real },
    Cylinder(Box<CertifiedSphereCylinderVoid>),
}

#[derive(Clone, Debug)]
struct CertifiedSphereCylinderVoid {
    origin: Point3,
    axis: Vector3,
    radius: Real,
    v_min: Real,
    v_max: Real,
}

#[derive(Clone, Debug)]
enum CertifiedSphereRegion {
    Whole,
    Axial(CertifiedSphereAxialClip),
    Radial(CertifiedSphereRadialClip),
    FiniteCylinder(CertifiedSphereFiniteCylinderRegion),
}

#[derive(Clone, Debug)]
struct CertifiedSphereAxialClip {
    axis: Vector3,
    min: Real,
    max: Real,
}

#[derive(Clone, Debug)]
struct CertifiedSphereRadialClip {
    axis: Vector3,
    radius: Real,
    side: CertifiedSphereRadialSide,
}

#[derive(Clone, Debug)]
struct CertifiedSphereFiniteCylinderRegion {
    origin: Point3,
    axis: Vector3,
    radius: Real,
    v_min: Real,
    v_max: Real,
    operation: CertifiedSphereFiniteCylinderOperation,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CertifiedSphereFiniteCylinderOperation {
    Union,
    Intersection,
    Difference,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CertifiedSphereRadialSide {
    Inside,
    Outside,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CertifiedSpherePairKind {
    Union,
    Intersection,
    Difference,
}

#[derive(Clone, Debug)]
struct CertifiedSpherePairShell {
    first_center: Point3,
    first_radius: Real,
    second_center: Point3,
    second_radius: Real,
    kind: CertifiedSpherePairKind,
}

#[derive(Clone, Debug)]
struct CertifiedSphericalCapFace {
    center: Point3,
    axis: Vector3,
    radius: Real,
    latitude: Real,
    upper: bool,
    orientation: Orientation,
}

#[derive(Clone, Debug)]
struct CertifiedConeFrustumShell {
    apex: Point3,
    axis: Vector3,
    semi_angle: Real,
    v_min: Real,
    v_max: Real,
    region: CertifiedConeFrustumRegion,
}

#[derive(Clone, Debug)]
enum CertifiedConeFrustumRegion {
    Whole,
    LongitudinalHalf { interior_normal: Vector3 },
    Sector(CertifiedAngularSector),
}

/// A convex angular sector, certified from complete periodic surface coverage.
#[derive(Clone, Debug)]
struct CertifiedAngularSector {
    interior_normals: Vec<Vector3>,
    sweep: Real,
}

#[derive(Clone, Debug)]
struct CertifiedTorusShell {
    center: Point3,
    axis: Vector3,
    major_radius: Real,
    minor_radius: Real,
    region: CertifiedTorusRegion,
}

#[derive(Clone, Debug)]
enum CertifiedTorusRegion {
    Whole,
    Axial {
        min: Real,
        max: Real,
    },
    LongitudinalHalf {
        interior_normal: Vector3,
    },
    SectorBand {
        angular: CertifiedAngularSector,
        min: Real,
        max: Real,
    },
}

#[derive(Clone, Debug)]
struct CertifiedRevolutionShell {
    axis_origin: Point3,
    axis: Vector3,
    profile: CertifiedRevolutionBoundary,
    voids: Vec<CertifiedRevolutionBoundary>,
}

#[derive(Clone, Debug)]
enum CertifiedRevolutionBoundary {
    Native(Contour2),
    Curved(CurvePath2),
}

impl CertifiedRevolutionBoundary {
    fn signed_x_first_moment(&self) -> Result<Option<Real>, GeometryError> {
        match self {
            Self::Native(contour) => contour.signed_x_first_moment().map_err(GeometryError::from),
            Self::Curved(path) => crate::error::classified(
                path.boundary_loop()
                    .map_err(GeometryError::from)?
                    .area_moments(),
            )
            .map_err(GeometryError::from)
            .and_then(resolve_planar_classification)
            .map(|moments| moments.map(|moments| moments.x_moment().clone())),
        }
    }

    fn classify_point(
        &self,
        point: &CurvePoint2,
    ) -> Result<Classification<ContourPointLocation>, GeometryError> {
        match self {
            Self::Native(contour) => {
                crate::error::classified(contour.classify_point(point)).map_err(GeometryError::from)
            }
            Self::Curved(path) => {
                crate::error::classified(path.classify_point(&point.clone().into()))
                    .map_err(GeometryError::from)
            }
        }
    }

    fn start(&self) -> hypercurve::CurvePoint2 {
        match self {
            Self::Native(contour) => contour.segments()[0].start().clone().into(),
            Self::Curved(path) => path.start(),
        }
    }

    fn classify_retained_point(
        &self,
        point: &hypercurve::CurvePoint2,
    ) -> Result<Classification<ContourPointLocation>, GeometryError> {
        if let Some(coordinates) = point.coordinates() {
            return self.classify_point(coordinates);
        }
        crate::error::classified(self.as_curve_path()?.classify_point(point))
            .map_err(GeometryError::from)
    }

    fn as_curve_path(&self) -> Result<CurvePath2, GeometryError> {
        match self {
            Self::Native(contour) => CurvePath2::try_new(
                contour
                    .segments()
                    .iter()
                    .cloned()
                    .map(curve2_from_segment)
                    .collect(),
            )
            .map_err(GeometryError::from),
            Self::Curved(path) => Ok(path.clone()),
        }
    }

    fn intersects(&self, other: &Self) -> Result<bool, GeometryError> {
        if let (Self::Native(first), Self::Native(second)) = (self, other) {
            return first
                .intersect_contour(second)
                .map(|intersections| !intersections.is_empty())
                .map_err(GeometryError::from);
        }
        let result = self
            .as_curve_path()?
            .intersect_path(&other.as_curve_path()?)
            .map_err(GeometryError::from)?;
        if !result.blockers().is_empty() {
            return Err(GeometryError::UnsupportedIntersection);
        }
        Ok(!result.contacts().is_empty() || !result.overlaps().is_empty())
    }

    const fn native_contour(&self) -> Option<&Contour2> {
        match self {
            Self::Native(contour) => Some(contour),
            Self::Curved(_) => None,
        }
    }
}

#[derive(Clone, Debug)]
struct CertifiedLoftShell {
    origin: Point3,
    u: Vector3,
    v: Vector3,
    height_axis: Vector3,
    spans: Vec<CertifiedLoftSpan>,
    parameter_volume: Real,
}

#[derive(Clone, Debug)]
struct CertifiedLoftSpan {
    start: Real,
    end: Real,
    interpolation: CertifiedLoftInterpolation,
}

#[derive(Clone, Debug)]
enum CertifiedLoftInterpolation {
    Homothetic {
        profile: Contour2,
        scale: Real,
        translation: CurvePoint2,
    },
    ConvexCorresponding {
        lower: Vec<CurvePoint2>,
        upper: Vec<CurvePoint2>,
    },
}

/// How a certified sweep path advances through its parallel section planes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum SweepProgress {
    /// Plane progress is an exact affine function of the path parameter, so
    /// one exact division gives the section parameter of a point.
    Affine,
    /// Plane progress has a certified positive derivative on the closed
    /// parameter domain without being affine. A point's section parameter is
    /// the path's unique exact contact with the point's section plane. This is
    /// admitted only for a fixed profile frame.
    Monotone,
}

#[derive(Clone, Debug)]
struct CertifiedCurveSweepShell {
    profile: Contour2,
    holes: Vec<Contour2>,
    path: Curve3,
    u_path: Curve3,
    v_path: Curve3,
    area_scale_integral: Real,
    progress: SweepProgress,
}

struct TensorPathChain {
    lower: VertexId,
    upper: VertexId,
    curve: Curve3,
    parameter_start: Real,
    parameter_end: Real,
}

pub(crate) struct CertifiedZPrismProfile {
    pub(crate) region: CurveRegion2,
    pub(crate) z_min: Real,
    pub(crate) z_max: Real,
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedSphereProfile {
    pub(crate) center: Point3,
    pub(crate) radius: Real,
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedCylinderProfile {
    pub(crate) origin: Point3,
    pub(crate) axis: Vector3,
    pub(crate) radius: Real,
    pub(crate) v_min: Real,
    pub(crate) v_max: Real,
}

impl CertifiedSphereProfile {
    pub(crate) fn strictly_contains_cylinder(
        &self,
        cylinder: &CertifiedCylinderProfile,
    ) -> Result<bool, GeometryError> {
        let offset = &self.center - &cylinder.origin;
        let center_parameter = offset.dot(&cylinder.axis);
        let radial = offset - cylinder.axis.clone() * &center_parameter;
        let radial_distance = radial
            .norm_squared()
            .sqrt()
            .map_err(GeometryError::Numeric)?;
        let maximum_radial = radial_distance + &cylinder.radius;
        let lower_height = (&cylinder.v_min - &center_parameter).abs();
        let upper_height = (&cylinder.v_max - center_parameter).abs();
        let maximum_height = if decided_model_order(compare_reals(
            &lower_height,
            &upper_height,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Greater
        {
            lower_height
        } else {
            upper_height
        };
        Ok(decided_model_order(compare_reals(
            &(&maximum_radial * &maximum_radial + &maximum_height * &maximum_height),
            &(&self.radius * &self.radius),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less)
    }
}

impl CertifiedCylinderProfile {
    pub(crate) fn strictly_contains_sphere(
        &self,
        sphere: &CertifiedSphereProfile,
    ) -> Result<bool, GeometryError> {
        let offset = &sphere.center - &self.origin;
        let center_parameter = offset.dot(&self.axis);
        let radial = offset - self.axis.clone() * &center_parameter;
        let radial_distance = radial
            .norm_squared()
            .sqrt()
            .map_err(GeometryError::Numeric)?;
        Ok(decided_model_order(compare_reals(
            &(&radial_distance + &sphere.radius),
            &self.radius,
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less
            && decided_model_order(compare_reals(
                &self.v_min,
                &(&center_parameter - &sphere.radius),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Less
            && decided_model_order(compare_reals(
                &(&center_parameter + &sphere.radius),
                &self.v_max,
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Less)
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedConeFrustumProfile {
    pub(crate) apex: Point3,
    pub(crate) axis: Vector3,
    pub(crate) semi_angle: Real,
    pub(crate) v_min: Real,
    pub(crate) v_max: Real,
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedRevolutionProfile {
    pub(crate) axis_origin: Point3,
    pub(crate) axis: Vector3,
    profile: CertifiedRevolutionBoundary,
    holes: Vec<CertifiedRevolutionBoundary>,
}

impl CertifiedRevolutionProfile {
    fn native_contours(&self) -> Option<(Contour2, Vec<Contour2>)> {
        Some((
            self.profile.native_contour()?.clone(),
            self.holes
                .iter()
                .map(|hole| hole.native_contour().cloned())
                .collect::<Option<Vec<_>>>()?,
        ))
    }

    /// Preserve the existing polygon construction path without projecting a
    /// curved or selected profile to scalar vertices.
    pub(crate) fn native_polygons(&self) -> Option<(Contour2, Vec<Contour2>)> {
        let (outer, holes) = self.native_contours()?;
        std::iter::once(&outer)
            .chain(&holes)
            .all(|contour| {
                contour
                    .segments()
                    .iter()
                    .all(|segment| matches!(segment, Segment2::Line(_)))
            })
            .then_some((outer, holes))
    }

    /// The solid's independently admitted shell/cavity roles establish this
    /// filled region. Hypercurve replays its planar winding and contact proof.
    pub(crate) fn region(&self) -> Result<CurveRegion2, GeometryError> {
        if let Some((outer, holes)) = self.native_contours() {
            return CurveRegion2::try_from_native_contours(vec![outer], holes)
                .map_err(GeometryError::from);
        }
        let paths = std::iter::once(&self.profile)
            .chain(&self.holes)
            .map(CertifiedRevolutionBoundary::as_curve_path)
            .collect::<Result<Vec<_>, _>>()?;
        let roles = std::iter::once(hypercurve::CurveRegionLoopRole::Material)
            .chain(
                self.holes
                    .iter()
                    .map(|_| hypercurve::CurveRegionLoopRole::Hole),
            )
            .collect::<Vec<_>>();
        CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &paths,
            &roles,
            &vec![hypercurve::FillRule::NonZero; paths.len()],
        )
        .map_err(GeometryError::from)
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CertifiedTorusProfile {
    pub(crate) center: Point3,
    pub(crate) axis: Vector3,
    pub(crate) major_radius: Real,
    pub(crate) minor_radius: Real,
}

fn single_validation_error(error: BuildError) -> ValidationReport {
    ValidationReport {
        errors: vec![error],
    }
}

fn reset_model_caches(data: &mut ModelData) {
    data.bounds = OnceLock::new();
    data.face_contours = (0..data.faces.len()).map(|_| OnceLock::new()).collect();
}

fn invalidate_model_certificates(data: &mut ModelData) {
    for edge in &mut data.edges {
        edge.endpoint_certificate = false;
    }
    for edge_use in &mut data.edge_uses {
        edge_use.image_certificate = false;
    }
    for wire in &mut data.wires {
        wire.simplicity_certificate = false;
    }
    data.solid_certificates_valid.fill(false);
    data.certified_cylinders.fill(None);
    data.certified_spheres.fill(None);
    data.certified_sphere_pairs.fill(None);
    data.certified_cone_frustums.fill(None);
    data.certified_tori.fill(None);
    data.certified_revolutions.fill(None);
    data.certified_lofts.fill(None);
    data.certified_curve_sweeps.fill(None);
    data.certified_prisms.fill(None);
    data.certified_axial_shells.fill(None);
}

fn compact_wire_arena(
    data: &mut ModelData,
    retired: &std::collections::HashSet<WireId>,
) -> Result<Vec<Option<WireId>>, BuildError> {
    let mut remap = vec![None; data.wires.len()];
    let mut compacted = Vec::with_capacity(data.wires.len().saturating_sub(retired.len()));
    for (index, wire) in std::mem::take(&mut data.wires).into_iter().enumerate() {
        let old =
            WireId::from_index(index).ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        if retired.contains(&old) {
            continue;
        }
        let new = WireId::from_index(compacted.len())
            .ok_or(BuildError::CapacityExceeded(EntityKind::Wire))?;
        remap[index] = Some(new);
        compacted.push(wire);
    }
    let remap_wire = |wire: WireId| {
        remap
            .get(wire.index())
            .copied()
            .flatten()
            .ok_or(BuildError::InvalidReference {
                kind: EntityKind::Wire,
                index: wire.index(),
            })
    };
    for face in &mut data.faces {
        let FaceBoundary::Trimmed { outer, inner } = &mut face.boundary else {
            continue;
        };
        *outer = remap_wire(*outer)?;
        for wire in inner {
            *wire = remap_wire(*wire)?;
        }
    }
    data.wires = compacted;
    Ok(remap)
}

struct FaceLineMaterial {
    intervals: Vec<(Real, Real)>,
    contacts: Vec<Real>,
}

/// Incremental constructor for a canonical validated model.
///
/// Successful solid admission retains strict proofs for the lifetime of this
/// builder. Finalization reuses those proofs and derives any remaining
/// applicable families. Adding records cannot change an admitted boundary;
/// model edits invalidate affected proof state before replay. Failed admission
/// publishes neither a solid nor its proofs.
///
/// Retention is one record per admitted solid, released when the builder is
/// consumed or dropped. This is not a heap-byte, total-work or cancellation
/// budget, and there is no global cache or configurable eviction policy here.
#[derive(Debug, Default)]
pub struct ArenaModelBuilder {
    vertices: Vec<Vertex>,
    curves: Vec<Curve3>,
    pcurves: Vec<Pcurve>,
    surfaces: Vec<Surface>,
    edges: Vec<Edge>,
    edge_uses: Vec<EdgeUse>,
    wires: Vec<Wire>,
    faces: Vec<Face>,
    shells: Vec<Shell>,
    solids: Vec<Solid>,
    vertex_edges: Vec<Vec<EdgeId>>,
    edge_uses_by_edge: Vec<Vec<EdgeUseId>>,
    edge_use_wire: Vec<Option<WireId>>,
    wire_face: Vec<Option<FaceId>>,
    face_shell: Vec<Option<ShellId>>,
    shell_solid: Vec<Option<SolidId>>,
    // One record per successfully admitted solid, owned until finish/drop.
    // Appended records cannot mutate that solid's geometry or boundary uses.
    // This is an entry-count bound, not a bound on retained expression bytes.
    solid_certificates: Vec<BuilderSolidCertificates>,
}
