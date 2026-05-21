//! Public scalar type, tolerance configuration, and physics backend re-exports.

use hyperreal::RealSign;
use nalgebra::{Matrix4, Point3, Vector3};
#[cfg(feature = "wasm")]
use nalgebra::{Quaternion, UnitQuaternion};
use std::cmp::Ordering;

pub use parry3d_f64 as parry3d;
pub use rapier3d_f64 as rapier3d;

/// Public API scalar type.
///
/// This is the legacy f64 boundary scalar retained while the crate is ported
/// module-by-module to [`HReal`]. New internal geometry should use `HReal` and
/// convert primitive floats only at API and IO boundaries.
pub type Real = f64;

/// Primary internal scalar type for exact-aware geometry.
pub type HReal = hyperreal::Real;

/// Explicit f64 boundary scalar alias.
pub type F64 = f64;

/// Explicit f32 boundary scalar alias.
pub type F32 = f32;

/// Promote an f64 boundary value to the internal hyperreal scalar.
pub fn hreal_from_f64(value: F64) -> Result<HReal, hyperreal::Problem> {
    HReal::try_from(value)
}

/// Promote an f32 boundary value to the internal hyperreal scalar.
pub fn hreal_from_f32(value: F32) -> Result<HReal, hyperreal::Problem> {
    HReal::try_from(value)
}

/// Lossy f64 export for API, rendering, and IO boundaries.
pub fn hreal_to_f64(value: &HReal) -> Option<F64> {
    value.to_f64_lossy().filter(|value| value.is_finite())
}

/// Promote a finite f64 boundary point to a hyperreal lattice vector.
///
/// The checked primitive lift is owned by `hyperlattice`; `csgrs` only adapts
/// nalgebra's current mesh boundary type into that hyper geometry API.
pub(crate) fn hvector3_from_point3(point: &Point3<Real>) -> Option<hyperlattice::Vector3> {
    hyperlattice::Point3::try_from_f64_array([point.x, point.y, point.z])
        .ok()
        .map(Into::into)
}

/// Promote a finite f64 boundary vector to a hyperreal lattice vector.
///
/// Keeping the finite-array constructor in `hyperlattice` avoids duplicating
/// scalar promotion rules in CAD code, following Yap's exact-geometric-
/// computation split between primitive input boundaries and exact-aware
/// geometric objects (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hvector3_from_vector3(vector: &Vector3<Real>) -> Option<hyperlattice::Vector3> {
    hyperlattice::Vector3::try_from_f64_array([vector.x, vector.y, vector.z]).ok()
}

/// Return a finite unit vector using hyperlattice normalization.
///
/// This is the shared replacement for ad hoc nalgebra `.normalize()` calls in
/// topology-sensitive import and mesh paths. Invalid primitive inputs and
/// degenerate directions fail closed before they can become public CAD state,
/// following Yap's exact-geometric-computation boundary discipline
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hunit_vector3(vector: &Vector3<Real>) -> Option<Vector3<Real>> {
    let unit = hvector3_from_vector3(vector).and_then(|vector| {
        vector
            .normalize_checked()
            .ok()
            .and_then(|unit| unit.to_f64_array_lossy())
    })?;
    Some(Vector3::new(unit[0], unit[1], unit[2]))
}

/// Return a finite unit vector and magnitude using hyperlattice normalization.
///
/// This supports legacy transformation APIs that still need a primitive length
/// term, while keeping the zero/non-finite rejection and square root in
/// hyperreal-backed vector algebra. See Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hunit_vector3_and_magnitude(
    vector: &Vector3<Real>,
) -> Option<(Vector3<Real>, Real)> {
    let vector = hvector3_from_vector3(vector)?;
    let magnitude = vector
        .magnitude()
        .ok()
        .and_then(|value| hreal_to_f64(&value))?;
    let unit = vector
        .normalize_checked()
        .ok()
        .and_then(|unit| unit.to_f64_array_lossy())?;
    Some((Vector3::new(unit[0], unit[1], unit[2]), magnitude))
}

/// Return a finite Euclidean magnitude for a public boundary vector.
#[cfg(feature = "wasm")]
pub(crate) fn hvector3_magnitude(vector: &Vector3<Real>) -> Option<Real> {
    let vector = hvector3_from_vector3(vector)?;
    vector.magnitude().ok().and_then(|value| hreal_to_f64(&value))
}

/// Return a finite dot product for two public boundary vectors.
pub(crate) fn hvector3_dot(lhs: &Vector3<Real>, rhs: &Vector3<Real>) -> Option<Real> {
    let lhs = hvector3_from_vector3(lhs)?;
    let rhs = hvector3_from_vector3(rhs)?;
    hreal_to_f64(&lhs.dot(&rhs))
}

/// Return a finite cross product for two public boundary vectors.
///
/// The determinant is evaluated by `hyperlattice::Vector3`, keeping vector
/// algebra used by JS-facing CAD helpers on the hyperreal side of the primitive
/// boundary. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hvector3_cross(
    lhs: &Vector3<Real>,
    rhs: &Vector3<Real>,
) -> Option<Vector3<Real>> {
    let lhs = hvector3_from_vector3(lhs)?;
    let rhs = hvector3_from_vector3(rhs)?;
    let cross = lhs.cross(&rhs).to_f64_array_lossy()?;
    Some(Vector3::new(cross[0], cross[1], cross[2]))
}

/// Return the exact-aware orientation sign of three finite XY boundary points.
///
/// The determinant `(b - a) x (c - a)` is assembled in `hyperreal::Real` and
/// refined through [`hreal_sign`]. This gives finite import/export adapters a
/// shared 2D turn predicate instead of local f64 cross products, in line with
/// Yap's exact-geometric-computation split between primitive boundary data and
/// exact-aware predicates (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[cfg(feature = "gerber-io")]
pub(crate) fn hxy_orientation_sign(
    a: (Real, Real),
    b: (Real, Real),
    c: (Real, Real),
) -> Option<RealSign> {
    match hyperlimit::orient2d_f64([a.0, a.1], [b.0, b.1], [c.0, c.1]).value()? {
        hyperlimit::Sign::Positive => Some(RealSign::Positive),
        hyperlimit::Sign::Negative => Some(RealSign::Negative),
        hyperlimit::Sign::Zero => Some(RealSign::Zero),
    }
}

/// Return the finite Euclidean distance between two XY boundary coordinates.
///
/// Toolpath and 2D profile adapters receive primitive coordinates only after
/// hypercurve has projected finite machine-output polylines. The length itself
/// is still evaluated in `hyperlattice::Vector2`/`hyperreal::Real`, following
/// Yap's exact-geometric-computation split between primitive boundary data and
/// exact-aware predicates (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[cfg(any(test, feature = "gerber-io", feature = "offset"))]
pub(crate) fn hxy_distance(lhs: (Real, Real), rhs: (Real, Real)) -> Option<Real> {
    let lhs = hyperlattice::Vector2::try_from_f64_array([lhs.0, lhs.1]).ok()?;
    let rhs = hyperlattice::Vector2::try_from_f64_array([rhs.0, rhs.1]).ok()?;
    hreal_to_f64(&lhs.squared_distance(&rhs).sqrt().ok()?)
}

/// Return a finite unit direction from one XY boundary coordinate to another.
///
/// This is the shared 2D counterpart to [`hunit_vector3`], used by CAM lead-in
/// construction so cutter vectors are normalized by hyperlattice instead of
/// local primitive `sqrt`/division.
#[cfg(any(test, feature = "gerber-io", feature = "offset"))]
pub(crate) fn hxy_unit_direction(
    from: (Real, Real),
    to: (Real, Real),
) -> Option<(Real, Real)> {
    let from = hyperlattice::Vector2::try_from_f64_array([from.0, from.1]).ok()?;
    let to = hyperlattice::Vector2::try_from_f64_array([to.0, to.1]).ok()?;
    let unit = (to - from).normalize_checked().ok()?.to_f64_array_lossy()?;
    Some((unit[0], unit[1]))
}

/// Interpolate two XY boundary coordinates through hyperreal arithmetic.
///
/// This is the 2-D counterpart to [`hpoint_lerp`]. Profile constructors still
/// receive finite `f64` control points at the API edge, but curve evaluation
/// should happen after promotion to `hyperlattice::Vector2`, following Yap's
/// exact-geometric-computation boundary discipline
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[cfg(any(test, feature = "sketch"))]
pub(crate) fn hxy_lerp(from: (Real, Real), to: (Real, Real), t: Real) -> Option<(Real, Real)> {
    let from = hyperlattice::Vector2::try_from_f64_array([from.0, from.1]).ok()?;
    let to = hyperlattice::Vector2::try_from_f64_array([to.0, to.1]).ok()?;
    let t = hreal_from_f64(t).ok()?;
    let result = from.lerp(&to, &t);
    let coords = result.to_f64_array_lossy()?;
    Some((coords[0], coords[1]))
}

/// Step from an XY boundary coordinate along a finite unit direction.
#[cfg(any(test, feature = "offset"))]
pub(crate) fn hxy_step(
    origin: (Real, Real),
    direction: (Real, Real),
    distance: Real,
) -> Option<(Real, Real)> {
    let origin = hyperlattice::Vector2::try_from_f64_array([origin.0, origin.1]).ok()?;
    let direction =
        hyperlattice::Vector2::try_from_f64_array([direction.0, direction.1]).ok()?;
    let distance = hreal_from_f64(distance).ok()?;
    let stepped = origin.step(&direction, &distance);
    let coords = stepped.to_f64_array_lossy()?;
    Some((coords[0], coords[1]))
}

/// Return a finite checked unit cross product for two public boundary vectors.
///
/// This is the normal-construction path for ruled surfaces and other mesh
/// carriers that still expose `Vector3<f64>` normals. Both the determinant and
/// zero-vector rejection stay in `hyperlattice`, following Yap's exact-
/// geometric-computation split between finite boundary data and exact-aware
/// geometric objects (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hunit_cross_vector3(
    lhs: &Vector3<Real>,
    rhs: &Vector3<Real>,
) -> Option<Vector3<Real>> {
    let lhs = hvector3_from_vector3(lhs)?;
    let rhs = hvector3_from_vector3(rhs)?;
    let unit = lhs.unit_cross_checked(&rhs).ok()?.to_f64_array_lossy()?;
    Some(Vector3::new(unit[0], unit[1], unit[2]))
}

/// Return a finite homogeneous rotation matrix that maps `from` onto `to`.
///
/// Unit-vector checks, dot products, and cross products are delegated to
/// `hyperlattice`, then the finite boundary matrix is assembled with Rodrigues'
/// rotation formula. This keeps orientation construction in the hyper geometry
/// layer while preserving the transitional `Matrix4<f64>` transform API.
///
/// References:
/// - Yap, "Towards Exact Geometric Computation," *Computational Geometry*
///   7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
/// - Rodrigues, "Des lois géométriques qui régissent les déplacements d'un
///   système solide dans l'espace," *Journal de Mathématiques Pures et
///   Appliquées* 5, 1840.
pub(crate) fn hrotation_between_vectors(
    from: &Vector3<Real>,
    to: &Vector3<Real>,
) -> Option<Matrix4<Real>> {
    let from = hunit_vector3(from)?;
    let to = hunit_vector3(to)?;
    let dot = hvector3_dot(&from, &to)?;

    if dot >= 1.0 - tolerance() {
        return Some(Matrix4::identity());
    }

    let (axis, angle) = if dot <= -1.0 + tolerance() {
        let seed = if from.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        (hunit_vector3(&hvector3_cross(&from, &seed)?)?, PI)
    } else {
        (
            hunit_vector3(&hvector3_cross(&from, &to)?)?,
            hangle_between_vectors(&from, &to)?,
        )
    };

    hrotation_axis_angle(&axis, angle)
}

/// Build a finite homogeneous translation matrix from a public boundary vector.
///
/// The vector is first promoted through `hyperlattice` so non-finite primitive
/// inputs cannot become CAD transforms. The returned `Matrix4<f64>` is still
/// the transitional transform carrier, but callers no longer depend on
/// nalgebra's translation constructor for CAD API boundaries. This follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn htranslation_matrix(vector: &Vector3<Real>) -> Option<Matrix4<Real>> {
    hvector3_from_vector3(vector)?;
    Some(Matrix4::new(
        1.0, 0.0, 0.0, vector.x, 0.0, 1.0, 0.0, vector.y, 0.0, 0.0, 1.0, vector.z, 0.0, 0.0,
        0.0, 1.0,
    ))
}

/// Build a finite homogeneous non-uniform scale matrix.
///
/// Scale factors are promoted through the same hyperlattice boundary adapter
/// as vectors before matrix construction. This keeps non-finite primitive
/// values out of CAD transforms while the transitional API still transports
/// matrices as `Matrix4<f64>`, following Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hscale_matrix(sx: Real, sy: Real, sz: Real) -> Option<Matrix4<Real>> {
    hvector3_from_vector3(&Vector3::new(sx, sy, sz))?;
    Some(Matrix4::new(
        sx, 0.0, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 0.0, sz, 0.0, 0.0, 0.0, 0.0, 1.0,
    ))
}

/// Return a finite orthonormal basis perpendicular to a finite axis.
///
/// The input axis is normalized and both perpendicular axes are generated by
/// `hyperlattice` cross products before finite export. This gives mesh
/// primitive constructors one shared basis builder instead of local ad hoc
/// seed/cross code, following Yap's exact-geometric-computation boundary model
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hperpendicular_basis(
    axis: &Vector3<Real>,
) -> Option<(Vector3<Real>, Vector3<Real>)> {
    let axis = hvector3_from_vector3(axis)?;
    let (x, y) = axis.orthonormal_basis_checked().ok()?;
    let x = x.to_f64_array_lossy()?;
    let y = y.to_f64_array_lossy()?;
    Some((Vector3::new(x[0], x[1], x[2]), Vector3::new(y[0], y[1], y[2])))
}

fn hrotation_axis_angle(axis: &Vector3<Real>, angle: Real) -> Option<Matrix4<Real>> {
    let axis = hunit_vector3(axis)?;
    let (sin, cos) = hangle_sin_cos(angle)?;

    let one_minus_cos = 1.0 - cos;
    let x = axis.x;
    let y = axis.y;
    let z = axis.z;
    Some(Matrix4::new(
        cos + x * x * one_minus_cos,
        x * y * one_minus_cos - z * sin,
        x * z * one_minus_cos + y * sin,
        0.0,
        y * x * one_minus_cos + z * sin,
        cos + y * y * one_minus_cos,
        y * z * one_minus_cos - x * sin,
        0.0,
        z * x * one_minus_cos - y * sin,
        z * y * one_minus_cos + x * sin,
        cos + z * z * one_minus_cos,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ))
}

/// Return finite sine and cosine of a public angle in radians.
///
/// The trigonometric functions are evaluated on `hyperreal::Real`, then
/// exported once to the transitional scalar boundary. This is the shared path
/// for transform matrix construction, keeping Euler/Rodrigues rotation
/// carriers from using local primitive trig. See Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>) and Rodrigues, "Des lois
/// géométriques qui régissent les déplacements d'un système solide dans
/// l'espace," *Journal de Mathématiques Pures et Appliquées* 5, 1840.
pub(crate) fn hangle_sin_cos(angle_radians: Real) -> Option<(Real, Real)> {
    let angle = hreal_from_f64(angle_radians).ok()?;
    Some((
        hreal_to_f64(&angle.clone().sin())?,
        hreal_to_f64(&angle.cos())?,
    ))
}

/// Return a finite Euclidean distance between two public boundary vectors.
pub(crate) fn hvector3_distance(lhs: &Vector3<Real>, rhs: &Vector3<Real>) -> Option<Real> {
    let lhs = hvector3_from_vector3(lhs)?;
    let rhs = hvector3_from_vector3(rhs)?;
    hreal_to_f64(&lhs.squared_distance(&rhs).sqrt().ok()?)
}

/// Return the finite arithmetic mean of public boundary vectors.
///
/// Coordinate summation and count division are evaluated by
/// `hyperlattice::Vector3` over `hyperreal::Real`; only the returned boundary
/// vector is lossy-exported. This is the vector analogue of [`hpoint_centroid`]
/// for analysis paths that still expose `Vector3<f64>` normals, following
/// Yap's exact-geometric-computation boundary split
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hvector3_mean(vectors: &[Vector3<Real>]) -> Option<Vector3<Real>> {
    let vectors = vectors
        .iter()
        .map(hvector3_from_vector3)
        .collect::<Option<Vec<_>>>()?;
    let mean = hyperlattice::Vector3::mean(&vectors)?.to_f64_array_lossy()?;
    Some(Vector3::new(mean[0], mean[1], mean[2]))
}

/// Return a finite weighted sum of public boundary points.
///
/// The caller owns weight normalization semantics; this helper only guarantees
/// that finite coordinates and weights are promoted to hyperreal-backed
/// vectors before the affine sum crosses back to the transitional point type.
pub(crate) fn hpoint_weighted_sum(
    points: &[Point3<Real>],
    weights: &[Real],
) -> Option<Point3<Real>> {
    let points = points
        .iter()
        .map(|point| {
            hyperlattice::Point3::try_from_f64_array([point.x, point.y, point.z]).ok()
        })
        .collect::<Option<Vec<_>>>()?;
    let weights = weights
        .iter()
        .copied()
        .map(hreal_from_f64)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    let coords =
        hyperlattice::Point3::weighted_sum(&points, &weights)?.to_f64_array_lossy()?;
    Some(Point3::new(coords[0], coords[1], coords[2]))
}

/// Return a finite weighted sum of public boundary vectors.
///
/// This keeps normal blending and smoothing stencils in hyperlattice instead
/// of local primitive vector folds while csgrs still exposes `Vector3<f64>` at
/// API edges.
pub(crate) fn hvector3_weighted_sum(
    vectors: &[Vector3<Real>],
    weights: &[Real],
) -> Option<Vector3<Real>> {
    let vectors = vectors
        .iter()
        .map(hvector3_from_vector3)
        .collect::<Option<Vec<_>>>()?;
    let weights = weights
        .iter()
        .copied()
        .map(hreal_from_f64)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    let coords =
        hyperlattice::Vector3::weighted_sum(&vectors, &weights)?.to_f64_array_lossy()?;
    Some(Vector3::new(coords[0], coords[1], coords[2]))
}

/// Return whether two public boundary vectors are orthogonal within epsilon.
///
/// The dot product is evaluated in hyperreal space and only the tolerance is a
/// primitive API-boundary scalar. This follows Yap's exact-geometric-
/// computation boundary discipline
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[cfg(feature = "wasm")]
pub(crate) fn hvectors_orthogonal_within(
    lhs: &Vector3<Real>,
    rhs: &Vector3<Real>,
    epsilon: Real,
) -> bool {
    if !epsilon.is_finite() || epsilon < 0.0 {
        return false;
    }
    let Some(lhs) = hvector3_from_vector3(lhs) else {
        return false;
    };
    let Some(rhs) = hvector3_from_vector3(rhs) else {
        return false;
    };
    let dot = lhs.dot(&rhs);
    !hreal_gt_f64(&dot, epsilon) && !hreal_lt_f64(&dot, -epsilon)
}

/// Return the finite angle between two public boundary vectors in radians.
///
/// Unit-vector normalization, dot product, clamping, and arccos are delegated
/// to hyperreal-backed `hyperlattice`, with only the final radian value
/// exported to `f64`.
pub(crate) fn hangle_between_vectors(
    lhs: &Vector3<Real>,
    rhs: &Vector3<Real>,
) -> Option<Real> {
    let lhs = hvector3_from_vector3(lhs)?.normalize_checked().ok()?;
    let rhs = hvector3_from_vector3(rhs)?.normalize_checked().ok()?;
    let mut cos_angle = lhs.dot(&rhs);
    if hreal_gt_f64(&cos_angle, 1.0) {
        cos_angle = hreal_from_f64(1.0).ok()?;
    } else if hreal_lt_f64(&cos_angle, -1.0) {
        cos_angle = hreal_from_f64(-1.0).ok()?;
    }
    hreal_to_f64(&hyperlattice::acos(cos_angle).ok()?)
}

/// Return a finite unit quaternion from primitive boundary components.
///
/// Quaternion normalization is represented as a four-dimensional hyperlattice
/// vector normalization before crossing back into nalgebra's transform carrier.
/// This keeps invalid JS/API quaternion components out of CAD transforms and
/// follows Yap's exact-geometric-computation boundary model
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[cfg(feature = "wasm")]
pub(crate) fn hunit_quaternion(
    w: Real,
    x: Real,
    y: Real,
    z: Real,
) -> Option<UnitQuaternion<Real>> {
    let vector = hyperlattice::Vector4::try_from_f64_array([w, x, y, z]).ok()?;
    let unit = vector
        .normalize_checked()
        .ok()
        .and_then(|unit| unit.to_f64_array_lossy())?;
    Some(UnitQuaternion::new_unchecked(Quaternion::new(
        unit[0], unit[1], unit[2], unit[3],
    )))
}

/// Compare finite f64 boundary points by squared distance in hyperreal space.
///
/// This keeps tolerance predicates on the exact-aware side of the API boundary:
/// public callers still pass `Point3<f64>`, while topology-sensitive code gets
/// a `hyperlattice::Vector3` predicate that fails closed for non-finite inputs.
/// That mirrors Yap's exact-geometric-computation boundary discipline
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hpoints_within_epsilon(
    lhs: &Point3<Real>,
    rhs: &Point3<Real>,
    epsilon: Real,
) -> bool {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return false;
    }

    let Some(lhs) = hvector3_from_point3(lhs) else {
        return false;
    };
    let Some(rhs) = hvector3_from_point3(rhs) else {
        return false;
    };

    hreal_lt_f64(&lhs.squared_distance(&rhs), epsilon * epsilon)
}

/// Return the finite Euclidean distance between two public boundary points.
///
/// Distance construction and square root are delegated to `hyperlattice` /
/// `hyperreal`; only the final value crosses back to `f64`. Callers that need
/// topology-sensitive segment lengths should use this instead of nalgebra
/// vector norms, in line with Yap's exact-geometric-computation layering
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hpoint_distance(lhs: &Point3<Real>, rhs: &Point3<Real>) -> Option<Real> {
    let lhs = hvector3_from_point3(lhs)?;
    let rhs = hvector3_from_point3(rhs)?;
    hreal_to_f64(&lhs.squared_distance(&rhs).sqrt().ok()?)
}

/// Return the finite centroid of public boundary points using hyperlattice.
///
/// Count-to-scalar conversion is the only integer boundary here; coordinate
/// summation and division are evaluated as `hyperreal::Real` values inside
/// `hyperlattice::Vector3`. This is the shared smoothing/refinement helper for
/// avoiding local `Point3<f64>` accumulation in CAD algorithms, following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
/// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hpoint_centroid(points: &[Point3<Real>]) -> Option<Point3<Real>> {
    let points = points
        .iter()
        .map(|point| {
            hyperlattice::Point3::try_from_f64_array([point.x, point.y, point.z]).ok()
        })
        .collect::<Option<Vec<_>>>()?;
    let centroid = hyperlattice::Point3::centroid(&points)?.to_f64_array_lossy()?;
    Some(Point3::new(centroid[0], centroid[1], centroid[2]))
}

/// Interpolate two public boundary points through hyperreal arithmetic.
///
/// This keeps smoothing stencils and other topology-sensitive movement steps
/// from doing local f64 vector math after the current mesh carrier has crossed
/// into hyperlattice coordinates. The returned point is the transitional
/// boundary representation.
pub(crate) fn hpoint_lerp(
    from: &Point3<Real>,
    to: &Point3<Real>,
    t: Real,
) -> Option<Point3<Real>> {
    let from = hvector3_from_point3(from)?;
    let to = hvector3_from_point3(to)?;
    let t = hreal_from_f64(t).ok()?;
    let result = from.lerp(&to, &t);
    let coords = result.to_f64_array_lossy()?;
    Some(Point3::new(coords[0], coords[1], coords[2]))
}

/// Return the finite hyperreal area of a triangle from public boundary points.
///
/// The doubled-area determinant `(b-a) x (c-a)` and magnitude are evaluated in
/// `hyperlattice::Vector3`/`hyperreal::Real` and kept there for callers that
/// still need to compose additional CAD metrics. This avoids repeating local
/// nalgebra cross/dot area predicates in mesh quality and import paths,
/// following Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn htriangle_area_hreal(
    a: &Point3<Real>,
    b: &Point3<Real>,
    c: &Point3<Real>,
) -> Option<HReal> {
    let a = hvector3_from_point3(a)?;
    let b = hvector3_from_point3(b)?;
    let c = hvector3_from_point3(c)?;
    let area2 = (&b - &a).cross(&(&c - &a));
    let magnitude_squared = area2.dot(&area2);
    if !hreal_gt_f64(&magnitude_squared, tolerance() * tolerance()) {
        return None;
    }
    let twice_area = magnitude_squared.sqrt().ok()?;
    Some(twice_area * hreal_from_f64(0.5).ok()?)
}

/// Return true when a triangle's doubled area exceeds `epsilon`.
///
/// The public mesh carrier still stores nalgebra points during the transition,
/// but degenerate-triangle predicates should be evaluated after promotion to
/// `hyperlattice::Vector3`. This keeps topology-affecting area checks in the
/// exact-aware geometry layer, following Yap's exact-geometric-computation
/// boundary model (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[cfg(any(feature = "sdf", feature = "metaballs"))]
pub(crate) fn htriangle_area2_exceeds_epsilon(
    a: &Point3<Real>,
    b: &Point3<Real>,
    c: &Point3<Real>,
    epsilon: Real,
) -> bool {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return false;
    }
    let Some(a) = hvector3_from_point3(a) else {
        return false;
    };
    let Some(b) = hvector3_from_point3(b) else {
        return false;
    };
    let Some(c) = hvector3_from_point3(c) else {
        return false;
    };
    let area2 = (&b - &a).cross(&(&c - &a));
    hreal_gt_f64(&area2.dot(&area2), epsilon * epsilon)
}

/// Compare finite f64 boundary vectors by squared distance in hyperreal space.
///
/// This is the vector analogue of [`hpoints_within_epsilon`], used for normals
/// and directions that still cross public API or file-format boundaries as
/// primitive floats while topology-sensitive equality is evaluated with
/// `hyperreal::Real`.
pub(crate) fn hvectors_within_epsilon(
    lhs: &Vector3<Real>,
    rhs: &Vector3<Real>,
    epsilon: Real,
) -> bool {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return false;
    }

    let Some(lhs) = hvector3_from_vector3(lhs) else {
        return false;
    };
    let Some(rhs) = hvector3_from_vector3(rhs) else {
        return false;
    };

    hreal_lt_f64(&lhs.squared_distance(&rhs), epsilon * epsilon)
}

/// Refine the sign of a hyperreal expression for topology decisions.
pub(crate) fn hreal_sign(value: &HReal) -> Option<RealSign> {
    value.refine_sign_until(128)
}

/// Returns true when `value` is strictly greater than the finite f64 threshold.
pub(crate) fn hreal_gt_f64(value: &HReal, threshold: F64) -> bool {
    let Ok(threshold) = hreal_from_f64(threshold) else {
        return false;
    };
    match hyperlimit::compare_reals(value, &threshold).value() {
        Some(Ordering::Greater) => true,
        Some(_) => false,
        None => matches!(
            hreal_sign(&(value.clone() - threshold)),
            Some(RealSign::Positive)
        ),
    }
}

/// Returns true when `value` is strictly less than the finite f64 threshold.
pub(crate) fn hreal_lt_f64(value: &HReal, threshold: F64) -> bool {
    let Ok(threshold) = hreal_from_f64(threshold) else {
        return false;
    };
    match hyperlimit::compare_reals(value, &threshold).value() {
        Some(Ordering::Less) => true,
        Some(_) => false,
        None => matches!(
            hreal_sign(&(value.clone() - threshold)),
            Some(RealSign::Negative)
        ),
    }
}

/// Compare two finite f64 API-boundary scalar values in hyperreal space.
///
/// This is intentionally a boundary adapter, not a replacement for native
/// hyperreal scalar ownership. It is used when external libraries report
/// primitive parameters, but csgrs still needs topology-affecting ordering to
/// follow the same exact-geometric-computation discipline as point/vector
/// predicates (Yap, *Computational Geometry* 7(1-2), 1997,
/// <https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hreal_cmp_f64(lhs: F64, rhs: F64) -> Ordering {
    let (Ok(lhs), Ok(rhs)) = (hreal_from_f64(lhs), hreal_from_f64(rhs)) else {
        return Ordering::Equal;
    };
    hyperlimit::compare_reals(&lhs, &rhs)
        .value()
        .unwrap_or_else(|| match hreal_sign(&(lhs - rhs)) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            Some(RealSign::Zero) | None => Ordering::Equal,
        })
}

/// Clamp a finite public boundary scalar in hyperreal comparison space.
///
/// This is for transitional APIs that still receive and return `f64` scalars
/// but need branch decisions to use the same exact-aware sign refinement as
/// geometric predicates. It follows Yap's exact-geometric-computation split
/// between primitive boundaries and exact-aware decisions
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hreal_clamp_f64(value: F64, min: F64, max: F64) -> Option<F64> {
    let value_h = hreal_from_f64(value).ok()?;
    let min_h = hreal_from_f64(min).ok()?;
    let max_h = hreal_from_f64(max).ok()?;
    hreal_to_f64(&hyperlimit::real_clamp(value_h, &min_h, &max_h).value()?)
}

/// Return true when two finite f64 API-boundary scalars are within `epsilon`.
///
/// The squared difference is evaluated in `hyperreal::Real` so callers avoid
/// mixing f64 subtraction, absolute value, and tolerance logic in local CAD
/// code. This follows the same Yap exact-computation boundary model cited in
/// [`hreal_cmp_f64`].
pub(crate) fn hreal_f64s_within_epsilon(lhs: F64, rhs: F64, epsilon: F64) -> bool {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return false;
    }
    let (Ok(lhs), Ok(rhs)) = (hreal_from_f64(lhs), hreal_from_f64(rhs)) else {
        return false;
    };
    let delta = lhs - rhs;
    hreal_lt_f64(&(delta.clone() * delta), epsilon * epsilon)
}

/// Return the finite sum of public boundary scalars.
///
/// This helper is for aggregate reporting paths that still receive `f64`
/// slices. Values are promoted to `hyperreal::Real`, accumulated there, and
/// exported once at the boundary.
pub(crate) fn hreal_sum(values: &[Real]) -> Option<Real> {
    let values = values
        .iter()
        .copied()
        .map(hreal_from_f64)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    hreal_to_f64(&HReal::sum_refs(values.iter()))
}

/// Return the finite maximum of public boundary scalars in hyperreal order.
///
/// This keeps constructor clamping and sampling bounds in the exact-aware
/// scalar layer while `f64` remains only the current public/IO boundary. The
/// comparison discipline follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hreal_max(values: &[Real]) -> Option<Real> {
    let mut values = values
        .iter()
        .copied()
        .map(hreal_from_f64)
        .collect::<Result<Vec<_>, _>>()
        .ok()?
        .into_iter();
    let first = values.next()?;
    let max = values.try_fold(first, |acc, value| {
        hyperlimit::real_max(&acc, &value).value().cloned()
    })?;
    hreal_to_f64(&max)
}

/// Return the finite minimum of public boundary scalars in hyperreal order.
pub(crate) fn hreal_min(values: &[Real]) -> Option<Real> {
    let mut values = values
        .iter()
        .copied()
        .map(hreal_from_f64)
        .collect::<Result<Vec<_>, _>>()
        .ok()?
        .into_iter();
    let first = values.next()?;
    let min = values.try_fold(first, |acc, value| {
        hyperlimit::real_min(&acc, &value).value().cloned()
    })?;
    hreal_to_f64(&min)
}

/// Divide two finite public boundary scalars through hyperreal arithmetic.
pub(crate) fn hreal_div(lhs: Real, rhs: Real) -> Option<Real> {
    let lhs = hreal_from_f64(lhs).ok()?;
    let rhs = hreal_from_f64(rhs).ok()?;
    hreal_to_f64(&(lhs / rhs).ok()?)
}

/// Subtract two finite public boundary scalars through hyperreal arithmetic.
pub(crate) fn hreal_sub(lhs: Real, rhs: Real) -> Option<Real> {
    let lhs = hreal_from_f64(lhs).ok()?;
    let rhs = hreal_from_f64(rhs).ok()?;
    hreal_to_f64(&(lhs - rhs))
}

/// Multiply two finite public boundary scalars through hyperreal arithmetic.
pub(crate) fn hreal_mul(lhs: Real, rhs: Real) -> Option<Real> {
    let lhs = hreal_from_f64(lhs).ok()?;
    let rhs = hreal_from_f64(rhs).ok()?;
    hreal_to_f64(&(lhs * rhs))
}

/// Return the finite absolute value of a public boundary scalar.
pub(crate) fn hreal_abs(value: Real) -> Option<Real> {
    let value = hreal_from_f64(value).ok()?;
    hreal_to_f64(&value.abs())
}

/// Return the finite square root of a public boundary scalar.
pub(crate) fn hreal_sqrt(value: Real) -> Option<Real> {
    let value = hreal_from_f64(value).ok()?;
    hreal_to_f64(&value.sqrt().ok()?)
}

/// Return the finite arctangent of a public boundary scalar.
#[cfg(any(test, feature = "sketch"))]
pub(crate) fn hreal_atan(value: Real) -> Option<Real> {
    let value = hreal_from_f64(value).ok()?;
    hreal_to_f64(&value.atan().ok()?)
}

/// Return the finite tangent of a public boundary scalar.
#[cfg(any(test, feature = "sketch"))]
pub(crate) fn hreal_tan(value: Real) -> Option<Real> {
    let value = hreal_from_f64(value).ok()?;
    hreal_to_f64(&value.tan().ok()?)
}

/// Raise a finite public boundary scalar to a finite public boundary exponent.
#[cfg(any(test, feature = "sketch"))]
pub(crate) fn hreal_pow(base: Real, exponent: Real) -> Option<Real> {
    let base = hreal_from_f64(base).ok()?;
    let exponent = hreal_from_f64(exponent).ok()?;
    hreal_to_f64(&base.pow(exponent).ok()?)
}

/// Evaluate `origin + t * delta` through hyperreal arithmetic.
pub(crate) fn hreal_affine(origin: Real, t: Real, delta: Real) -> Option<Real> {
    let origin = hreal_from_f64(origin).ok()?;
    let t = hreal_from_f64(t).ok()?;
    let delta = hreal_from_f64(delta).ok()?;
    hreal_to_f64(&HReal::affine(&origin, &t, &delta))
}

/// Convert finite public degrees to radians through hyperreal arithmetic.
pub(crate) fn hdegrees_to_radians(degrees: Real) -> Option<Real> {
    let degrees = hreal_from_f64(degrees).ok()?;
    hreal_to_f64(&degrees.to_radians())
}

/// Convert finite public radians to degrees through hyperreal arithmetic.
pub(crate) fn hradians_to_degrees(radians: Real) -> Option<Real> {
    let radians = hreal_from_f64(radians).ok()?;
    hreal_to_f64(&radians.to_degrees())
}

/// Return the finite arithmetic mean of public boundary scalars.
///
/// This is a reporting-edge helper for aggregate metrics. The input scalars
/// are promoted into `hyperreal::Real`, accumulated there, divided by an
/// integer count promoted at the boundary, and only then exported to `f64`.
pub(crate) fn hreal_mean(values: &[Real]) -> Option<Real> {
    let values = values
        .iter()
        .copied()
        .map(hreal_from_f64)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    hreal_to_f64(&HReal::mean(&values)?)
}

/// Return the finite sample standard deviation of public boundary scalars.
///
/// Mean and squared-difference accumulation stay in hyperreal space; only the
/// final standard deviation crosses back to the transitional scalar type. This
/// follows Yap's exact-geometric-computation boundary split used throughout the
/// crate (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub(crate) fn hreal_sample_stddev(values: &[Real]) -> Option<Real> {
    let values = values
        .iter()
        .copied()
        .map(hreal_from_f64)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    hreal_to_f64(&HReal::sample_stddev(&values)?)
}

use std::sync::OnceLock;

/// Lazily-initialized tolerance used across the crate.
///
/// Defaults to `1e-6` and may be overridden once at runtime with
/// [`set_tolerance`]. Tolerance is deliberately not a Cargo/build-time knob:
/// primitive `f32`/`f64` values are API-boundary data, while internal geometry
/// promotes predicates into hyperreal space. Keeping that split explicit follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
static TOLERANCE_CELL: OnceLock<Real> = OnceLock::new();

#[inline]
const fn default_tolerance() -> Real {
    1e-6
}

/// Returns the current epsilon value.
/// If no runtime override was installed, this returns the crate default.
pub fn tolerance() -> Real {
    *TOLERANCE_CELL.get_or_init(default_tolerance)
}

/// Set epsilon programmatically once (subsequent calls are ignored).
///
/// Call near program start: `csgrs::float_types::set_tolerance(1e-6);`.
/// Non-finite values are rejected back to the default because only finite
/// primitive floats are accepted at csgrs API boundaries.
pub fn set_tolerance(value: Real) {
    let value = if value.is_finite() {
        value.max(Real::EPSILON)
    } else {
        default_tolerance()
    };
    let _ = TOLERANCE_CELL.set(value);
}

/// Archimedes' constant (π)
pub const PI: Real = core::f64::consts::PI;

/// π/2
pub const FRAC_PI_2: Real = core::f64::consts::FRAC_PI_2;

/// The full circle constant (τ)
pub const TAU: Real = core::f64::consts::TAU;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Unit conversion
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
pub const INCH: Real = 25.4;
pub const FOOT: Real = 25.4 * 12.0;
pub const YARD: Real = 25.4 * 36.0;
pub const MM: Real = 1.0;
pub const CM: Real = 10.0;
pub const METER: Real = 1000.0;

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn hpoints_within_epsilon_accepts_nearby_finite_points() {
        let epsilon = 1e-6;
        let lhs = Point3::new(1.0, 2.0, 3.0);
        let rhs = Point3::new(1.0 + epsilon * 0.25, 2.0, 3.0);

        assert!(hpoints_within_epsilon(&lhs, &rhs, epsilon));
    }

    #[test]
    fn hpoints_within_epsilon_rejects_distant_or_nonfinite_points() {
        let epsilon = 1e-6;
        let lhs = Point3::new(1.0, 2.0, 3.0);
        let distant = Point3::new(1.0 + epsilon * 2.0, 2.0, 3.0);
        let nonfinite = Point3::new(f64::NAN, 2.0, 3.0);

        assert!(!hpoints_within_epsilon(&lhs, &distant, epsilon));
        assert!(!hpoints_within_epsilon(&lhs, &nonfinite, epsilon));
        assert!(!hpoints_within_epsilon(&lhs, &lhs, 0.0));
    }

    #[test]
    fn hvectors_within_epsilon_uses_hyperreal_distance() {
        let epsilon = 1e-6;
        let lhs = Vector3::new(0.0, 1.0, 0.0);
        let near = Vector3::new(0.0, 1.0 + epsilon * 0.25, 0.0);
        let distant = Vector3::new(0.0, 1.0 + epsilon * 2.0, 0.0);
        let nonfinite = Vector3::new(0.0, f64::INFINITY, 0.0);

        assert!(hvectors_within_epsilon(&lhs, &near, epsilon));
        assert!(!hvectors_within_epsilon(&lhs, &distant, epsilon));
        assert!(!hvectors_within_epsilon(&lhs, &nonfinite, epsilon));
    }

    #[test]
    fn hunit_vector3_rejects_degenerate_and_nonfinite_inputs() {
        let unit = hunit_vector3(&Vector3::new(3.0, 4.0, 0.0)).unwrap();
        assert!(hreal_f64s_within_epsilon(unit.x, 0.6, tolerance()));
        assert!(hreal_f64s_within_epsilon(unit.y, 0.8, tolerance()));
        assert!(hreal_f64s_within_epsilon(unit.z, 0.0, tolerance()));

        assert!(hunit_vector3(&Vector3::zeros()).is_none());
        assert!(hunit_vector3(&Vector3::new(1.0, f64::NAN, 0.0)).is_none());
    }

    #[test]
    fn hunit_vector3_and_magnitude_exports_finite_length() {
        let (unit, magnitude) =
            hunit_vector3_and_magnitude(&Vector3::new(0.0, 0.0, -2.5)).unwrap();
        assert!(hreal_f64s_within_epsilon(unit.x, 0.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(unit.y, 0.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(unit.z, -1.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(magnitude, 2.5, tolerance()));

        assert!(hunit_vector3_and_magnitude(&Vector3::zeros()).is_none());
        assert!(
            hunit_vector3_and_magnitude(&Vector3::new(Real::INFINITY, 0.0, 0.0)).is_none()
        );
    }

    #[test]
    fn hrotation_between_vectors_maps_axes_and_rejects_hostile_inputs() {
        let x_to_y = hrotation_between_vectors(&Vector3::x(), &Vector3::y()).unwrap();
        let rotated =
            Point3::from_homogeneous(x_to_y * Point3::new(1.0, 0.0, 0.0).to_homogeneous())
                .unwrap();
        assert!(hreal_f64s_within_epsilon(rotated.x, 0.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(rotated.y, 1.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(rotated.z, 0.0, tolerance()));

        let opposite = hrotation_between_vectors(&Vector3::x(), &-Vector3::x()).unwrap();
        let rotated =
            Point3::from_homogeneous(opposite * Point3::new(1.0, 0.0, 0.0).to_homogeneous())
                .unwrap();
        assert!(hreal_f64s_within_epsilon(rotated.x, -1.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(rotated.y, 0.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(rotated.z, 0.0, tolerance()));

        assert!(
            hrotation_between_vectors(&Vector3::new(Real::NAN, 0.0, 0.0), &Vector3::z())
                .is_none()
        );
        assert!(hrotation_between_vectors(&Vector3::zeros(), &Vector3::z()).is_none());
    }

    #[test]
    fn htranslation_matrix_rejects_nonfinite_boundary_vectors() {
        let matrix = htranslation_matrix(&Vector3::new(1.0, -2.0, 3.0)).unwrap();
        let moved =
            Point3::from_homogeneous(matrix * Point3::new(4.0, 5.0, 6.0).to_homogeneous())
                .unwrap();
        assert!(hreal_f64s_within_epsilon(moved.x, 5.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(moved.y, 3.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(moved.z, 9.0, tolerance()));

        assert!(htranslation_matrix(&Vector3::new(0.0, Real::NAN, 0.0)).is_none());
    }

    #[test]
    fn hscale_matrix_rejects_nonfinite_scale_factors() {
        let matrix = hscale_matrix(2.0, 3.0, -4.0).unwrap();
        let scaled =
            Point3::from_homogeneous(matrix * Point3::new(1.0, 2.0, -3.0).to_homogeneous())
                .unwrap();
        assert!(hreal_f64s_within_epsilon(scaled.x, 2.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(scaled.y, 6.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(scaled.z, 12.0, tolerance()));

        assert!(hscale_matrix(1.0, Real::INFINITY, 1.0).is_none());
    }

    #[test]
    fn hperpendicular_basis_returns_unit_orthogonal_axes() {
        let axis = Vector3::new(1.0, 2.0, 3.0);
        let axis_unit = hunit_vector3(&axis).unwrap();
        let (x, y) = hperpendicular_basis(&axis).unwrap();

        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&x, &axis_unit).unwrap(),
            0.0,
            tolerance()
        ));
        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&y, &axis_unit).unwrap(),
            0.0,
            tolerance()
        ));
        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&x, &y).unwrap(),
            0.0,
            tolerance()
        ));
        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&x, &x).unwrap(),
            1.0,
            tolerance()
        ));
        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&y, &y).unwrap(),
            1.0,
            tolerance()
        ));
        let handed_normal = hunit_vector3(&hvector3_cross(&x, &y).unwrap()).unwrap();
        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&handed_normal, &axis_unit).unwrap(),
            1.0,
            tolerance()
        ));

        assert!(hperpendicular_basis(&Vector3::zeros()).is_none());
        assert!(hperpendicular_basis(&Vector3::new(Real::NAN, 0.0, 0.0)).is_none());
    }

    #[test]
    fn hunit_cross_vector3_rejects_degenerate_and_nonfinite_inputs() {
        let normal = hunit_cross_vector3(&Vector3::x(), &Vector3::y()).unwrap();
        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&normal, &Vector3::z()).unwrap(),
            1.0,
            tolerance()
        ));

        assert!(hunit_cross_vector3(&Vector3::x(), &Vector3::x()).is_none());
        assert!(
            hunit_cross_vector3(&Vector3::new(Real::NAN, 0.0, 0.0), &Vector3::y()).is_none()
        );
    }

    #[test]
    #[cfg(feature = "mesh")]
    fn htriangle_area_hreal_rejects_degenerate_and_nonfinite_boundary_points() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let c = Point3::new(0.0, 3.0, 0.0);

        assert!(hreal_f64s_within_epsilon(
            hreal_to_f64(&htriangle_area_hreal(&a, &b, &c).unwrap()).unwrap(),
            3.0,
            tolerance()
        ));
        assert!(htriangle_area_hreal(&a, &a, &c).is_none());
        assert!(htriangle_area_hreal(&a, &Point3::new(Real::NAN, 0.0, 0.0), &c).is_none());
    }

    #[test]
    fn hpoint_centroid_and_lerp_use_hyperreal_boundaries() {
        let points = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 4.0, 6.0),
            Point3::new(4.0, 8.0, 12.0),
        ];
        assert_eq!(hpoint_centroid(&points).unwrap(), Point3::new(2.0, 4.0, 6.0));
        assert!(hpoint_centroid(&[]).is_none());
        assert!(hpoint_centroid(&[Point3::new(Real::NAN, 0.0, 0.0)]).is_none());

        let interpolated = hpoint_lerp(&points[0], &points[2], 0.25).unwrap();
        assert_eq!(interpolated, Point3::new(1.0, 2.0, 3.0));
        assert!(hpoint_lerp(&points[0], &points[2], Real::INFINITY).is_none());
    }

    #[test]
    fn hreal_aggregate_helpers_use_hyperreal_boundaries() {
        let values = [1.0, 2.0, 3.0];
        assert_eq!(hreal_sum(&values).unwrap(), 6.0);
        assert_eq!(hreal_max(&values).unwrap(), 3.0);
        assert_eq!(hreal_min(&values).unwrap(), 1.0);
        assert_eq!(hreal_div(6.0, 3.0).unwrap(), 2.0);
        assert_eq!(hreal_sub(7.0, 4.0).unwrap(), 3.0);
        assert_eq!(hreal_mul(2.0, 3.0).unwrap(), 6.0);
        assert_eq!(hreal_abs(-3.0).unwrap(), 3.0);
        assert_eq!(hreal_abs(0.0).unwrap(), 0.0);
        assert_eq!(hreal_sqrt(25.0).unwrap(), 5.0);
        assert!(hreal_f64s_within_epsilon(
            hreal_atan(1.0).unwrap(),
            std::f64::consts::FRAC_PI_4,
            tolerance()
        ));
        assert!(hreal_f64s_within_epsilon(
            hreal_tan(std::f64::consts::FRAC_PI_4).unwrap(),
            1.0,
            tolerance()
        ));
        assert_eq!(hreal_pow(4.0, 0.5).unwrap(), 2.0);
        assert_eq!(hreal_pow(2.0, 3.0).unwrap(), 8.0);
        assert_eq!(hreal_clamp_f64(2.0, -1.0, 1.0).unwrap(), 1.0);
        assert_eq!(hreal_clamp_f64(-2.0, -1.0, 1.0).unwrap(), -1.0);
        assert_eq!(hreal_clamp_f64(0.5, -1.0, 1.0).unwrap(), 0.5);
        assert_eq!(hreal_affine(1.0, 0.25, 8.0).unwrap(), 3.0);
        assert!(hreal_f64s_within_epsilon(
            hdegrees_to_radians(180.0).unwrap(),
            PI,
            tolerance()
        ));
        assert!(hreal_f64s_within_epsilon(
            hradians_to_degrees(PI).unwrap(),
            180.0,
            tolerance()
        ));
        let (sin, cos) = hangle_sin_cos(FRAC_PI_2).unwrap();
        assert!(hreal_f64s_within_epsilon(sin, 1.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(cos, 0.0, tolerance()));
        assert_eq!(hreal_mean(&values).unwrap(), 2.0);
        assert!(hreal_sample_stddev(&values).unwrap() > 0.99);
        assert_eq!(
            hvector3_mean(&[Vector3::x(), Vector3::y()]).unwrap(),
            Vector3::new(0.5, 0.5, 0.0)
        );
        assert_eq!(
            hpoint_weighted_sum(
                &[Point3::origin(), Point3::new(2.0, 4.0, 6.0)],
                &[0.25, 0.75],
            )
            .unwrap(),
            Point3::new(1.5, 3.0, 4.5)
        );
        assert_eq!(
            hvector3_weighted_sum(&[Vector3::x(), Vector3::y()], &[0.25, 0.75]).unwrap(),
            Vector3::new(0.25, 0.75, 0.0)
        );
        assert!(hreal_mean(&[]).is_none());
        assert!(hreal_sample_stddev(&[1.0]).is_none());
        assert!(hvector3_mean(&[Vector3::new(Real::NAN, 0.0, 0.0)]).is_none());
        assert!(hreal_mean(&[1.0, Real::NAN]).is_none());
        assert!(hreal_sample_stddev(&[1.0, Real::INFINITY]).is_none());
        assert!(hreal_max(&[1.0, Real::NAN]).is_none());
        assert!(hreal_min(&[1.0, Real::INFINITY]).is_none());
        assert!(hangle_sin_cos(Real::INFINITY).is_none());
        assert!(hreal_abs(Real::NAN).is_none());
        assert!(hreal_sqrt(-1.0).is_none());
        assert!(hreal_atan(Real::INFINITY).is_none());
        assert!(hreal_tan(Real::INFINITY).is_none());
        assert!(hreal_pow(-1.0, 0.5).is_none());
        assert!(hreal_pow(Real::NAN, 2.0).is_none());
        assert!(hreal_clamp_f64(Real::NAN, -1.0, 1.0).is_none());
        assert!(hreal_clamp_f64(0.0, 1.0, -1.0).is_none());
        assert!(hdegrees_to_radians(Real::NAN).is_none());
        assert!(hradians_to_degrees(Real::NAN).is_none());
    }

    #[test]
    #[cfg(feature = "gerber-io")]
    fn hxy_orientation_sign_rejects_nonfinite_and_classifies_turns() {
        assert!(matches!(
            hxy_orientation_sign((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
            Some(RealSign::Positive)
        ));
        assert!(matches!(
            hxy_orientation_sign((0.0, 0.0), (0.0, 1.0), (1.0, 0.0)),
            Some(RealSign::Negative)
        ));
        assert!(matches!(
            hxy_orientation_sign((0.0, 0.0), (1.0, 1.0), (2.0, 2.0)),
            Some(RealSign::Zero)
        ));
        assert!(hxy_orientation_sign((0.0, 0.0), (Real::NAN, 1.0), (2.0, 2.0)).is_none());
    }

    #[test]
    fn hxy_distance_and_direction_use_hyperreal_boundaries() {
        assert!(hreal_f64s_within_epsilon(
            hxy_distance((0.0, 0.0), (3.0, 4.0)).unwrap(),
            5.0,
            tolerance()
        ));

        let direction = hxy_unit_direction((0.0, 0.0), (3.0, 4.0)).unwrap();
        assert!(hreal_f64s_within_epsilon(direction.0, 0.6, tolerance()));
        assert!(hreal_f64s_within_epsilon(direction.1, 0.8, tolerance()));

        let midpoint = hxy_lerp((0.0, 2.0), (4.0, 6.0), 0.5).unwrap();
        assert!(hreal_f64s_within_epsilon(midpoint.0, 2.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(midpoint.1, 4.0, tolerance()));

        let stepped = hxy_step((1.0, 2.0), direction, -5.0).unwrap();
        assert!(hreal_f64s_within_epsilon(stepped.0, -2.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(stepped.1, -2.0, tolerance()));

        assert!(hxy_distance((0.0, 0.0), (Real::NAN, 0.0)).is_none());
        assert!(hxy_unit_direction((0.0, 0.0), (0.0, 0.0)).is_none());
        assert!(hxy_lerp((0.0, f64::NAN), (1.0, 1.0), 0.5).is_none());
        assert!(hxy_step((0.0, 0.0), (1.0, 0.0), Real::INFINITY).is_none());
    }

    #[test]
    #[cfg(feature = "wasm")]
    fn hyper_vector_boundary_angle_and_dot_reject_nonfinite_inputs() {
        let x = Vector3::x();
        let y = Vector3::y();

        assert!(hreal_f64s_within_epsilon(
            hvector3_magnitude(&Vector3::new(0.0, 3.0, 4.0)).unwrap(),
            5.0,
            tolerance()
        ));
        assert!(hreal_f64s_within_epsilon(
            hvector3_dot(&x, &y).unwrap(),
            0.0,
            tolerance()
        ));
        assert!(hvectors_orthogonal_within(&x, &y, tolerance()));
        assert!(hreal_f64s_within_epsilon(
            hangle_between_vectors(&x, &y).unwrap(),
            FRAC_PI_2,
            tolerance()
        ));

        let hostile = Vector3::new(Real::NAN, Real::INFINITY, 0.0);
        assert!(hvector3_magnitude(&hostile).is_none());
        assert!(hvector3_dot(&x, &hostile).is_none());
        assert!(!hvectors_orthogonal_within(&x, &hostile, tolerance()));
        assert!(hangle_between_vectors(&x, &hostile).is_none());
    }

    #[test]
    #[cfg(feature = "wasm")]
    fn hunit_quaternion_rejects_degenerate_and_nonfinite_components() {
        let q = hunit_quaternion(2.0, 0.0, 0.0, 0.0).unwrap();
        assert!(hreal_f64s_within_epsilon(q.scalar(), 1.0, tolerance()));
        assert!(hreal_f64s_within_epsilon(q.vector().x, 0.0, tolerance()));

        assert!(hunit_quaternion(0.0, 0.0, 0.0, 0.0).is_none());
        assert!(hunit_quaternion(1.0, Real::NAN, 0.0, Real::INFINITY).is_none());
    }

    #[test]
    fn hreal_scalar_boundary_comparison_rejects_nonfinite_values() {
        let epsilon = 1e-6;

        assert_eq!(hreal_cmp_f64(1.0, 2.0), Ordering::Less);
        assert_eq!(hreal_cmp_f64(2.0, 1.0), Ordering::Greater);
        assert_eq!(hreal_cmp_f64(1.0, 1.0), Ordering::Equal);
        assert_eq!(hreal_cmp_f64(f64::NAN, 1.0), Ordering::Equal);

        assert!(hreal_f64s_within_epsilon(1.0, 1.0 + epsilon * 0.25, epsilon));
        assert!(!hreal_f64s_within_epsilon(1.0, 1.0 + epsilon * 2.0, epsilon));
        assert!(!hreal_f64s_within_epsilon(1.0, f64::INFINITY, epsilon));
        assert!(!hreal_f64s_within_epsilon(1.0, 1.0, 0.0));
    }
}
