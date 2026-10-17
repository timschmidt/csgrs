//! Exact constructors return owned bodies or sheets with bound entity access.

use super::*;
pub use crate::construction::{ExtrusionVoid, SphereVoid};
use hypercurve::{Contour2, CurvePath2, CurveRegion2};

/// Constructs one exact axis-aligned cuboid as a validated canonical model.
///
/// The returned solid is bounded by six outward-oriented parameterized planes.
/// Its twelve topological edges are shared by twenty-four independent face-local
/// pcurves.
pub fn cuboid(min: Point3, max: Point3) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::cuboid(min, max)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<SheetSet, ConstructionError> {
    let (model, faces) = crate::construction::planar_face(outer, holes, origin, u, v)?;
    Ok(SheetSet::from_parts(model, vec![faces]))
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
) -> Result<SheetSet, ConstructionError> {
    let (model, faces) = crate::construction::extrusion_patch(profile, direction, v_start, v_end)?;
    Ok(SheetSet::from_parts(model, vec![faces]))
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
) -> Result<SheetSet, ConstructionError> {
    let (model, faces) =
        crate::construction::revolution_patch(profile, axis_origin, axis, angle_start, angle_end)?;
    Ok(SheetSet::from_parts(model, vec![faces]))
}

/// Constructs one validated trimmed tensor-product rational Bézier patch.
///
/// The four boundary edges are exact rational Bézier restrictions of the
/// surface control net. The returned model contains one open shell and no
/// solid.
pub fn rational_bezier_patch(
    control_points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
) -> Result<SheetSet, ConstructionError> {
    let (model, faces) = crate::construction::rational_bezier_patch(control_points, weights)?;
    Ok(SheetSet::from_parts(model, vec![faces]))
}

/// Constructs a validated tensor-product NURBS sheet with exact boundaries.
pub fn nurbs_patch(
    u_degree: usize,
    v_degree: usize,
    control_points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
    u_knots: Vec<Real>,
    v_knots: Vec<Real>,
) -> Result<SheetSet, ConstructionError> {
    let (model, faces) = crate::construction::nurbs_patch(
        u_degree,
        v_degree,
        control_points,
        weights,
        u_knots,
        v_knots,
    )?;
    Ok(SheetSet::from_parts(model, vec![faces]))
}

/// Builds one validated open shell from exact tensor patches.
///
/// Boundary curves with the same exact endpoint vertices are stitched only
/// when their complete homogeneous representations are projectively
/// identical, including reversed traversal. Every face retains its own
/// pcurves and parameter correspondence; the shared model edge is authored
/// once and used in opposite directions by adjacent consistently oriented
/// patches. Unmatched boundaries remain open-shell boundaries.
pub fn tensor_patch_shell(patches: Vec<TensorPatch>) -> Result<SheetSet, ConstructionError> {
    let (model, faces) = crate::construction::tensor_patch_shell(patches)?;
    Ok(SheetSet::from_parts(model, faces))
}

/// Constructs a right circular cylinder on the positive z axis.
///
/// The base center is the model origin. `radius` and `height` must be
/// certified positive. The result uses four exact quarter-circle edges on
/// each cap, four axial edges, two planar caps, and four trimmed faces sharing
/// one native cylindrical surface. No polygonal approximation is introduced.
pub fn cylinder(radius: Real, height: Real) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::cylinder(radius, height)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Constructs one exact sphere centered at the model origin.
///
/// The sphere is represented by one complete closed-surface face. No
/// artificial longitude seam, collapsed pole edge, or tolerance sewing is
/// introduced.
pub fn sphere(radius: Real) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sphere(radius)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Constructs one exact origin-centered sphere with disjoint spherical voids.
///
/// Every boundary is one complete closed-surface face. Void faces are
/// inward-oriented and must be strictly contained and pairwise non-contacting.
pub fn sphere_with_voids(radius: Real, voids: &[SphereVoid]) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sphere_with_voids(radius, voids)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Builds a standard z-axis truncated cone with exact circular caps.
///
/// The base lies at `z = 0`, the top at `z = height`, and
/// `base_radius > top_radius > 0`.
pub fn cone_frustum(
    base_radius: Real,
    top_radius: Real,
    height: Real,
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::cone_frustum(base_radius, top_radius, height)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Builds a standard z-axis ring torus centered at the origin.
///
/// Sixteen exact parameter patches share 32 native circular edges on one
/// analytic torus surface. `major_radius` must be strictly greater than the
/// positive `minor_radius`.
pub fn torus(major_radius: Real, minor_radius: Real) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::torus(major_radius, minor_radius)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_contour(contour, z_min, z_max)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_contour_regions(regions, z_min, z_max)?;
    Ok(BodySet::from_parts(model, solids))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_path(path, z_min, z_max)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Extrudes one exact [`CurvePath2`] material region with disjoint holes.
pub fn extrude_path_region(
    outer: &CurvePath2,
    holes: &[CurvePath2],
    z_min: Real,
    z_max: Real,
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_path_region(outer, holes, z_min, z_max)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_path_regions(regions, z_min, z_max)?;
    Ok(BodySet::from_parts(model, solids))
}

/// Extrudes a regularized [`CurveRegion2`] between two z coordinates.
///
/// Hypercurve's retained material components determine the solids and their
/// holes. Its certified native line/arc view preserves circular side surfaces,
/// including circles represented by rational boundary spans. The original
/// component remains the solid's planar support proof; no fitting or display
/// geometry is used. Other persistent boundary families retain their existing
/// construction and measurement restrictions as typed errors.
///
/// `z_min` must be strictly less than `z_max`, including for an empty region.
/// Empty material returns an empty body set. Predicate decisions remain strict
/// inside an ambient curve approximation scope, and an enclosing
/// [`OperationContext`] continues to account for solver work.
pub fn extrude_curve_region(
    region: &CurveRegion2,
    z_min: Real,
    z_max: Real,
) -> Result<BodySet, ConstructionError> {
    crate::context::strict(|| {
        let (model, solids) = crate::construction::extrude_curve_region(region, z_min, z_max)?;
        Ok(BodySet::from_parts(model, solids))
    })
}

/// Revolves one exact simple radial/axial polygon around the z axis.
///
/// Profile `x` is radius and must remain strictly positive; profile `y` is
/// axial position. Clockwise input is normalized. Every line segment owns one
/// revolution carrier split into four exact angular faces. The periodic seam
/// shares meridian edges by identity rather than tolerance sewing.
pub fn revolve(profile: &[Point2]) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::revolve(profile)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Revolves one exact simple radial/axial region around the z axis.
///
/// The outer loop and every hole must stay strictly at positive radius.
/// Holes become identity-independent inward shells and remain exact material
/// cavities through measurement, classification, transforms, and persistence.
pub fn revolve_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::revolve_region(outer, holes)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Revolves one exact simple line/arc contour around the z axis.
///
/// Profile `x` is radius and the complete contour must remain strictly
/// positive. Every native line or circular-arc segment owns four periodic
/// revolution faces. Curved meridians remain native circles; no flattening or
/// tolerance sewing enters construction.
pub fn revolve_contour(profile: &Contour2) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::revolve_contour(profile)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Revolves one exact closed curved path around the z axis.
///
/// Profile `x` is radius and must remain strictly positive. Every authored
/// line, circular arc, Bézier, polynomial B-spline, or finite NURBS carrier is
/// retained as the meridian of four periodic revolution faces. The preflight
/// requires a complete exact simple-loop and orientation certificate; it never
/// flattens the path to manufacture one.
pub fn revolve_path(profile: &CurvePath2) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::revolve_path(profile)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Revolves one exact curved-path region with inward profile cavities.
///
/// Every loop uses the same exact simple-path and positive-radius preflight as
/// [`revolve_path`]. Shell nesting is certified from the retained curved
/// boundaries before the solid is published.
pub fn revolve_path_region(
    outer: &CurvePath2,
    holes: &[CurvePath2],
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::revolve_path_region(outer, holes)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Revolves one exact line/arc region with inward profile cavities.
pub fn revolve_contour_region(
    outer: &Contour2,
    holes: &[Contour2],
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::revolve_contour_region(outer, holes)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sweep(profile, origin, u, v, path)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sweep_region(outer, holes, origin, u, v, path)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sweep_curve(profile, u, v, path)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sweep_curve_region(outer, holes, u, v, path)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Sweeps one exact polygon through an explicitly authored rational Bézier
/// moving frame.
///
/// The frame supplies the complete origin and profile-axis motion. HyperBREP
/// infers no Frenet frame, corner transport, or sampling policy.
pub fn sweep_moving_frame(
    profile: &[Point2],
    frame: RationalBezierSweepFrame,
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sweep_moving_frame(profile, frame)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::sweep_moving_frame_region(outer, holes, frame)?;
    Ok(BodySet::from_parts(model, vec![solids]))
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
pub fn loft(sections: &[LoftSection]) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::loft(sections)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Extrudes one exact simple polygon between two z coordinates.
///
/// Clockwise input is normalized to counterclockwise order. Self-intersections,
/// repeated adjacent points, zero area, and undecidable ordering are rejected
/// before any trusted topology is published.
pub fn extrude(profile: &[Point2], z_min: Real, z_max: Real) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude(profile, z_min, z_max)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Extrudes one simple polygon and subtracts closed prismatic cavities.
///
/// Every cavity must be strictly contained by the outer extrusion. Cavity
/// shells are authored inward, retained as [`SolidRef::voids`], and
/// certified for exact non-contact and non-overlap before publication.
pub fn extrude_with_voids(
    profile: &[Point2],
    z_min: Real,
    z_max: Real,
    voids: &[ExtrusionVoid],
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_with_voids(profile, z_min, z_max, voids)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Extrudes one exact planar region with an outer loop and disjoint holes.
pub fn extrude_region(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    z_min: Real,
    z_max: Real,
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_region(outer, holes, z_min, z_max)?;
    Ok(BodySet::from_parts(model, vec![solids]))
}

/// Extrudes multiple disjoint exact planar regions into one validated model.
///
/// Each tuple contains one outer loop and its hole loops. The returned solid
/// IDs follow input order.
pub fn extrude_regions(
    regions: &[(Vec<Point2>, Vec<Vec<Point2>>)],
    z_min: Real,
    z_max: Real,
) -> Result<BodySet, ConstructionError> {
    let (model, solids) = crate::construction::extrude_regions(regions, z_min, z_max)?;
    Ok(BodySet::from_parts(model, solids))
}
