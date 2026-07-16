//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::errors::ValidationError;
use crate::mesh::plane::Plane;

use crate::vertex::{Vertex, cache_position_f64, fresh_position_id};

#[cfg(feature = "sketch")]
use crate::sketch::Profile;

use crate::csg::{CSG, finite_reflection};
#[cfg(feature = "sketch")]
use hypercurve::{Classification, FiniteProjectionOptions};
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hyperphysics::{
    ClosedTriangleMesh3 as HyperClosedTriangleMesh3,
    MassPropertyReport3 as HyperMassPropertyReport3, Triangle3 as HyperTriangle3,
    Vector3 as HyperVector3,
};
use hyperreal::RealSign;
use std::{
    cell::RefCell, cmp::PartialEq, collections::HashMap, fmt::Debug, num::NonZeroU32,
    sync::OnceLock,
};

pub mod convex_hull;
#[cfg(feature = "sketch")]
pub mod flatten_slice;

pub mod connectivity;
pub mod hypermesh;
pub mod manifold;
#[cfg(feature = "metaballs")]
pub mod metaballs;
pub mod plane;
pub mod polygon;
pub use polygon::Polygon;
use polygon::{
    CertifiedF64Bounds, PreparedExactXAxisTriangle, PreparedTriangleQuery,
    finite_normalized_exact_rational, fresh_plane_id,
};

#[derive(Clone, Debug)]
struct CachedRigidTransform {
    source_geometry_identity: Vec<u64>,
    matrix: Matrix4,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct CachedGeneralTransform {
    source_geometry_identity: Vec<u64>,
    matrix: Matrix4,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct CachedRotation {
    source_geometry_identity: Vec<u64>,
    degrees: [Real; 3],
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct PendingTransform {
    source_geometry_identity: Vec<u64>,
    matrix: Matrix4,
}

#[derive(Clone, Debug)]
struct PendingRotation {
    source_geometry_identity: Vec<u64>,
    degrees: [Real; 3],
}

thread_local! {
    static LAST_RIGID_TRANSFORM: RefCell<Option<CachedRigidTransform>> = const { RefCell::new(None) };
    static LAST_GENERAL_TRANSFORM: RefCell<Option<CachedGeneralTransform>> = const { RefCell::new(None) };
    static LAST_ROTATION: RefCell<Option<CachedRotation>> = const { RefCell::new(None) };
    static PENDING_RIGID_TRANSFORM: RefCell<Option<PendingTransform>> = const { RefCell::new(None) };
    static PENDING_GENERAL_TRANSFORM: RefCell<Option<PendingTransform>> = const { RefCell::new(None) };
    static PENDING_ROTATION: RefCell<Option<PendingRotation>> = const { RefCell::new(None) };
}
#[cfg(feature = "sdf")]
pub mod sdf;
pub mod shapes;
pub mod smoothing;
#[cfg(feature = "sdf")]
pub mod tpms;
pub mod triangulated;

/// Stored as `(position: [Real; 3], normal: [Real; 3])`.
pub type GraphicsMeshVertex = ([Real; 3], [Real; 3]);

/// Mesh data laid out for renderers that want vertex buffers plus u32 indices.
#[derive(Debug, Clone)]
pub struct GraphicsMesh {
    pub vertices: Vec<GraphicsMeshVertex>,
    pub indices: Vec<u32>,
}

#[derive(Clone, Debug)]
pub struct Mesh<M: Clone + Send + Sync + Debug> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<M>>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,
}

#[derive(Clone, Debug)]
struct CachedMassProperties {
    geometry_identity: Vec<u64>,
    density: Real,
    report: HyperMassPropertyReport3,
}

#[derive(Clone, Debug)]
struct CachedBasicMassProperties {
    geometry_identity: Vec<u64>,
    density: Real,
    mass: Real,
    center: Point3,
}

#[derive(Clone, Debug)]
struct CachedRayIntersections {
    spatial_identity: Vec<u64>,
    origin: Point3,
    direction: Vector3,
    hits: Vec<(Point3, Real)>,
}

#[derive(Clone, Debug)]
struct CachedPolylineIntersections {
    spatial_identity: Vec<u64>,
    polyline: Vec<Point3>,
    hits: Vec<Point3>,
}

thread_local! {
    static LAST_MASS_PROPERTIES: RefCell<Option<CachedMassProperties>> = const { RefCell::new(None) };
    static LAST_BASIC_MASS_PROPERTIES: RefCell<Option<CachedBasicMassProperties>> = const { RefCell::new(None) };
    static LAST_RAY_INTERSECTIONS: RefCell<Option<CachedRayIntersections>> = const { RefCell::new(None) };
    static LAST_POLYLINE_INTERSECTIONS: RefCell<Option<CachedPolylineIntersections>> = const { RefCell::new(None) };
}

fn real_cmp(lhs: &Real, rhs: &Real) -> std::cmp::Ordering {
    if let (Some(lhs), Some(rhs)) = (lhs.exact_rational_ref(), rhs.exact_rational_ref()) {
        return lhs
            .partial_cmp(rhs)
            .expect("exact rationals are totally ordered");
    }
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(-128) {
            Some(RealSign::Positive) => std::cmp::Ordering::Greater,
            Some(RealSign::Negative) => std::cmp::Ordering::Less,
            Some(RealSign::Zero) | None => std::cmp::Ordering::Equal,
        })
}

fn real_gt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), std::cmp::Ordering::Greater)
}

fn real_lt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), std::cmp::Ordering::Less)
}

fn real_zero(value: &Real) -> bool {
    if let Some(value) = value.exact_rational_ref() {
        return value.is_zero();
    }
    matches!(value.refine_sign_until(-128), Some(RealSign::Zero))
}

fn real_sign(value: &Real) -> Option<RealSign> {
    if let Some(value) = value.exact_rational_ref() {
        return Some(if value.is_zero() {
            RealSign::Zero
        } else if value.is_positive() {
            RealSign::Positive
        } else {
            RealSign::Negative
        });
    }
    value.refine_sign_until(-128)
}

fn include_point3_bounds(bounds: &mut Option<(Point3, Point3)>, point: &Point3) {
    let Some((mins, maxs)) = bounds else {
        *bounds = Some((point.clone(), point.clone()));
        return;
    };
    if real_lt(&point.x, &mins.x) {
        mins.x = point.x.clone();
    }
    if real_lt(&point.y, &mins.y) {
        mins.y = point.y.clone();
    }
    if real_lt(&point.z, &mins.z) {
        mins.z = point.z.clone();
    }
    if real_gt(&point.x, &maxs.x) {
        maxs.x = point.x.clone();
    }
    if real_gt(&point.y, &maxs.y) {
        maxs.y = point.y.clone();
    }
    if real_gt(&point.z, &maxs.z) {
        maxs.z = point.z.clone();
    }
}

fn aabbs_decided_disjoint(left: &Aabb, right: &Aabb) -> bool {
    [
        (&left.maxs.x, &right.mins.x),
        (&right.maxs.x, &left.mins.x),
        (&left.maxs.y, &right.mins.y),
        (&right.maxs.y, &left.mins.y),
        (&left.maxs.z, &right.mins.z),
        (&right.maxs.z, &left.mins.z),
    ]
    .into_iter()
    .any(|(maximum, minimum)| {
        matches!(
            hyperlimit::compare_reals(maximum, minimum).value(),
            Some(std::cmp::Ordering::Less)
        )
    })
}

fn point_decided_outside_aabb(point: &Point3, bounds: &Aabb) -> bool {
    [
        (&point.x, &bounds.mins.x, &bounds.maxs.x),
        (&point.y, &bounds.mins.y, &bounds.maxs.y),
        (&point.z, &bounds.mins.z, &bounds.maxs.z),
    ]
    .into_iter()
    .any(|(coordinate, minimum, maximum)| {
        matches!(
            hyperlimit::compare_reals(coordinate, minimum).value(),
            Some(std::cmp::Ordering::Less)
        ) || matches!(
            hyperlimit::compare_reals(coordinate, maximum).value(),
            Some(std::cmp::Ordering::Greater)
        )
    })
}

fn exact_f64(value: &Real) -> Option<f64> {
    if !value.is_exact_dyadic_rational() {
        return None;
    }
    let approximate = value.to_f64_lossy()?;
    if !approximate.is_finite() {
        return None;
    }
    let promoted = Real::try_from(approximate).ok()?;
    (promoted.exact_rational_ref() == value.exact_rational_ref()).then_some(approximate)
}

fn point_exact_f64(point: &Point3) -> Option<[f64; 3]> {
    Some([
        exact_f64(&point.x)?,
        exact_f64(&point.y)?,
        exact_f64(&point.z)?,
    ])
}

fn vector_exact_f64(vector: &Vector3) -> Option<[f64; 3]> {
    Some([
        exact_f64(&vector.0[0])?,
        exact_f64(&vector.0[1])?,
        exact_f64(&vector.0[2])?,
    ])
}

fn matrix_f64_lossy(matrix: &Matrix4) -> Option<[[f64; 4]; 4]> {
    if (0..4)
        .all(|row| (0..4).all(|column| matrix[row][column].exact_rational_ref().is_some()))
    {
        return None;
    }
    let mut finite = [[0.0; 4]; 4];
    for row in 0..4 {
        for column in 0..4 {
            finite[row][column] = matrix[row][column].to_f64_lossy()?;
        }
    }
    Some(finite)
}

fn transform_point_f64_lossy(matrix: &[[f64; 4]; 4], point: [f64; 3]) -> Option<[f64; 3]> {
    let homogeneous = [point[0], point[1], point[2], 1.0];
    let mut transformed = [0.0; 4];
    for row in 0..4 {
        transformed[row] = matrix[row]
            .iter()
            .zip(homogeneous)
            .map(|(coefficient, coordinate)| coefficient * coordinate)
            .sum();
    }
    let w = transformed[3];
    if !w.is_finite() || w == 0.0 {
        return None;
    }
    let point = [transformed[0] / w, transformed[1] / w, transformed[2] / w];
    point
        .iter()
        .all(|coordinate| coordinate.is_finite())
        .then_some(point)
}

fn rotation_xyz_f64_lossy(degrees: &[Real; 3]) -> Option<[[f64; 4]; 4]> {
    let [x, y, z] = [
        degrees[0].to_f64_lossy()?.to_radians(),
        degrees[1].to_f64_lossy()?.to_radians(),
        degrees[2].to_f64_lossy()?.to_radians(),
    ];
    let (sin_x, cos_x) = x.sin_cos();
    let (sin_y, cos_y) = y.sin_cos();
    let (sin_z, cos_z) = z.sin_cos();
    Some([
        [
            cos_z * cos_y,
            cos_z * sin_y * sin_x - sin_z * cos_x,
            cos_z * sin_y * cos_x + sin_z * sin_x,
            0.0,
        ],
        [
            sin_z * cos_y,
            sin_z * sin_y * sin_x + cos_z * cos_x,
            sin_z * sin_y * cos_x - cos_z * sin_x,
            0.0,
        ],
        [-sin_y, cos_y * sin_x, cos_y * cos_x, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

fn point_decided_outside_certified_f64_bounds(
    point: [f64; 3],
    bounds: &CertifiedF64Bounds,
) -> bool {
    (0..3).any(|axis| point[axis] < bounds.min[axis] || point[axis] > bounds.max[axis])
}

fn ray_decided_miss_certified_f64_bounds(
    origin: [f64; 3],
    direction: [f64; 3],
    bounds: &CertifiedF64Bounds,
    parameter_maximum: Option<f64>,
) -> bool {
    let mut active_axis = None;
    for (axis, ray_direction) in direction.iter().copied().enumerate() {
        if ray_direction != 0.0 {
            if active_axis.is_some() {
                active_axis = None;
                break;
            }
            active_axis = Some(axis);
        }
    }
    if let Some(axis) = active_axis {
        for (fixed_axis, ray_origin) in origin.iter().copied().enumerate() {
            if fixed_axis != axis
                && (ray_origin < bounds.min[fixed_axis] || ray_origin > bounds.max[fixed_axis])
            {
                return true;
            }
        }
        if let Some(maximum) = parameter_maximum {
            let end = origin[axis] + direction[axis] * maximum;
            let segment_minimum = origin[axis].min(end).next_down();
            let segment_maximum = origin[axis].max(end).next_up();
            return bounds.max[axis] < segment_minimum || bounds.min[axis] > segment_maximum;
        }
        return if direction[axis].is_sign_positive() {
            bounds.max[axis] < origin[axis]
        } else {
            bounds.min[axis] > origin[axis]
        };
    }

    let mut interval_minimum = 0.0_f64;
    let mut interval_maximum = parameter_maximum.unwrap_or(f64::INFINITY);

    for (axis, ray_direction) in direction.iter().copied().enumerate() {
        let ray_origin = origin[axis];
        if ray_direction == 0.0 {
            if ray_origin < bounds.min[axis] || ray_origin > bounds.max[axis] {
                return true;
            }
            continue;
        }

        let numerator_minimum = (bounds.min[axis] - ray_origin).next_down();
        let numerator_maximum = (bounds.max[axis] - ray_origin).next_up();
        let (near, far) = if ray_direction.is_sign_positive() {
            (
                (numerator_minimum / ray_direction).next_down(),
                (numerator_maximum / ray_direction).next_up(),
            )
        } else {
            (
                (numerator_maximum / ray_direction).next_down(),
                (numerator_minimum / ray_direction).next_up(),
            )
        };
        if near.is_nan() || far.is_nan() {
            return false;
        }
        interval_minimum = interval_minimum.max(near);
        interval_maximum = interval_maximum.min(far);
        if interval_maximum < interval_minimum {
            return true;
        }
    }
    false
}

fn ray_decided_miss_aabb(
    origin: &Point3,
    direction: &Vector3,
    bounds: &Aabb,
    parameter_maximum: Option<&Real>,
) -> bool {
    let zero = Real::zero();
    let axes = [
        (&origin.x, &direction.0[0], &bounds.mins.x, &bounds.maxs.x),
        (&origin.y, &direction.0[1], &bounds.mins.y, &bounds.maxs.y),
        (&origin.z, &direction.0[2], &bounds.mins.z, &bounds.maxs.z),
    ];

    // Zero-direction slabs are cheap and commonly reject axis-aligned queries
    // before any exact division is needed.
    for (ray_origin, ray_direction, minimum, maximum) in axes {
        if matches!(
            hyperlimit::compare_reals(ray_direction, &zero).value(),
            Some(std::cmp::Ordering::Equal)
        ) && (matches!(
            hyperlimit::compare_reals(ray_origin, minimum).value(),
            Some(std::cmp::Ordering::Less)
        ) || matches!(
            hyperlimit::compare_reals(ray_origin, maximum).value(),
            Some(std::cmp::Ordering::Greater)
        )) {
            return true;
        }
    }

    let mut interval_minimum = zero.clone();
    let mut interval_maximum = parameter_maximum.cloned();
    for (ray_origin, ray_direction, minimum, maximum) in axes {
        let Some(direction_ordering) = hyperlimit::compare_reals(ray_direction, &zero).value()
        else {
            // An uncertified broad-phase fact may not reject an exact candidate.
            return false;
        };
        if direction_ordering == std::cmp::Ordering::Equal {
            continue;
        }

        let Ok(first) = (minimum.clone() - ray_origin.clone()) / ray_direction.clone() else {
            return false;
        };
        let Ok(second) = (maximum.clone() - ray_origin.clone()) / ray_direction.clone() else {
            return false;
        };
        let (near, far) = if direction_ordering == std::cmp::Ordering::Greater {
            (first, second)
        } else {
            (second, first)
        };
        if matches!(
            hyperlimit::compare_reals(&far, &zero).value(),
            Some(std::cmp::Ordering::Less)
        ) {
            return true;
        }
        if matches!(
            hyperlimit::compare_reals(&near, &interval_minimum).value(),
            Some(std::cmp::Ordering::Greater)
        ) {
            interval_minimum = near;
        }
        if let Some(current_maximum) = &interval_maximum
            && matches!(
                hyperlimit::compare_reals(&far, current_maximum).value(),
                Some(std::cmp::Ordering::Less)
            )
        {
            interval_maximum = Some(far);
        } else if interval_maximum.is_none() {
            interval_maximum = Some(far);
        }
        if interval_maximum.as_ref().is_some_and(|maximum| {
            matches!(
                hyperlimit::compare_reals(maximum, &interval_minimum).value(),
                Some(std::cmp::Ordering::Less)
            )
        }) {
            return true;
        }
    }
    false
}

fn concatenate_disjoint_meshes<M: Clone + Send + Sync + Debug>(
    left: &Mesh<M>,
    right: &Mesh<M>,
) -> Mesh<M> {
    let mut polygons = Vec::with_capacity(left.polygons.len() + right.polygons.len());
    polygons.extend(left.polygons.iter().cloned());
    polygons.extend(right.polygons.iter().cloned());
    Mesh::from_polygons(polygons)
}

fn hyperphysics_vector3_from_point3(point: &Point3) -> Result<HyperVector3, ValidationError> {
    Ok(HyperVector3::new([
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
    ]))
}

#[cfg(test)]
fn ray_triangle_intersection(
    origin: &Point3,
    direction: &Vector3,
    tri: &[Vertex; 3],
) -> Option<(Point3, Real)> {
    ray_triangle_positions_hyperlimit(
        origin,
        direction,
        [&tri[0].position, &tri[1].position, &tri[2].position],
    )
}

fn ray_triangle_positions(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
) -> Option<(Point3, Real)> {
    ray_triangle_positions_prepared(origin, direction, positions, None)
}

fn ray_triangle_positions_prepared(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<(Point3, Real)> {
    match ray_triangle_positions_moller_trumbore(origin, direction, positions, prepared) {
        Some(result) => result,
        None => ray_triangle_positions_hyperlimit(origin, direction, positions),
    }
}

fn ray_triangle_positions_moller_trumbore(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<Option<(Point3, Real)>> {
    if let Some(result) = ray_triangle_positions_x_axis(origin, direction, positions, prepared)
    {
        return Some(result);
    }

    let computed_edges;
    let (edge1, edge2) = if let Some(prepared) = prepared {
        (&prepared.edge1, &prepared.edge2)
    } else {
        computed_edges = (
            positions[1].clone() - positions[0].clone(),
            positions[2].clone() - positions[0].clone(),
        );
        (&computed_edges.0, &computed_edges.1)
    };
    let perpendicular = direction.cross(edge2);
    let determinant = edge1.dot(&perpendicular);
    let determinant_sign = real_sign(&determinant)?;
    if determinant_sign == RealSign::Zero {
        return Some(None);
    }

    let from_first = origin.clone() - positions[0].clone();
    let u_numerator = from_first.dot(&perpendicular);
    let u_sign = real_sign(&u_numerator)?;
    if (determinant_sign == RealSign::Positive && u_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && u_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let cross = from_first.cross(edge1);
    let v_numerator = direction.dot(&cross);
    let v_sign = real_sign(&v_numerator)?;
    if (determinant_sign == RealSign::Positive && v_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && v_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let barycentric_remainder =
        determinant.clone() - u_numerator.clone() - v_numerator.clone();
    let remainder_sign = real_sign(&barycentric_remainder)?;
    if (determinant_sign == RealSign::Positive && remainder_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && remainder_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let t_numerator = edge2.dot(&cross);
    let t_sign = real_sign(&t_numerator)?;
    if (determinant_sign == RealSign::Positive && t_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && t_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let parameter = (t_numerator / determinant).ok()?;
    let point = origin.clone() + direction.clone() * parameter.clone();
    Some(Some((point, parameter)))
}

fn ray_triangle_positions_x_axis(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<Option<(Point3, Real)>> {
    if let Some(result) =
        ray_triangle_positions_x_axis_exact_rational(origin, direction, positions, prepared)
    {
        return Some(result);
    }
    if real_sign(&direction.0[1])? != RealSign::Zero
        || real_sign(&direction.0[2])? != RealSign::Zero
        || real_sign(&direction.0[0])? == RealSign::Zero
    {
        return None;
    }

    let computed_edges;
    let (edge1, edge2) = if let Some(prepared) = prepared {
        (&prepared.edge1, &prepared.edge2)
    } else {
        computed_edges = (
            positions[1].clone() - positions[0].clone(),
            positions[2].clone() - positions[0].clone(),
        );
        (&computed_edges.0, &computed_edges.1)
    };
    let scale = &direction.0[0];
    let determinant = scale.clone()
        * Real::signed_product_sum(
            [true, false],
            [[&edge1.0[2], &edge2.0[1]], [&edge1.0[1], &edge2.0[2]]],
        );
    let determinant_sign = real_sign(&determinant)?;
    if determinant_sign == RealSign::Zero {
        return Some(None);
    }

    let from_first = origin.clone() - positions[0].clone();
    let u_numerator = scale.clone()
        * Real::signed_product_sum(
            [true, false],
            [
                [&from_first.0[2], &edge2.0[1]],
                [&from_first.0[1], &edge2.0[2]],
            ],
        );
    let u_sign = real_sign(&u_numerator)?;
    if (determinant_sign == RealSign::Positive && u_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && u_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let v_numerator = scale.clone()
        * Real::signed_product_sum(
            [true, false],
            [
                [&from_first.0[1], &edge1.0[2]],
                [&from_first.0[2], &edge1.0[1]],
            ],
        );
    let v_sign = real_sign(&v_numerator)?;
    if (determinant_sign == RealSign::Positive && v_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && v_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let barycentric_remainder =
        determinant.clone() - u_numerator.clone() - v_numerator.clone();
    let remainder_sign = real_sign(&barycentric_remainder)?;
    if (determinant_sign == RealSign::Positive && remainder_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && remainder_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let cross = from_first.cross(edge1);
    let t_numerator = edge2.dot(&cross);
    let t_sign = real_sign(&t_numerator)?;
    if (determinant_sign == RealSign::Positive && t_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && t_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let parameter = (t_numerator / determinant).ok()?;
    let point = origin.clone() + direction.clone() * parameter.clone();
    Some(Some((point, parameter)))
}

fn ray_triangle_positions_x_axis_exact_rational(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<Option<(Point3, Real)>> {
    let direction_x = direction.0[0].exact_rational_ref()?;
    if direction_x.is_zero()
        || !direction.0[1].exact_rational_ref()?.is_zero()
        || !direction.0[2].exact_rational_ref()?.is_zero()
    {
        return None;
    }
    let exact = prepared_exact_x_axis_query(prepared?, positions)?;
    let [_edge1_x, edge1_y, edge1_z] = exact.edge1.each_ref();
    let [_edge2_x, edge2_y, edge2_z] = exact.edge2.each_ref();
    let origin_x = origin.x.exact_rational_ref()?;
    let origin_y = origin.y.exact_rational_ref()?;
    let origin_z = origin.z.exact_rational_ref()?;
    let from_y = origin_y - &exact.first[1];
    let from_z = origin_z - &exact.first[2];

    let determinant = direction_x * &exact.determinant_unit;
    if determinant.is_zero() {
        return Some(None);
    }
    let conflicts = |value: &hyperreal::Rational| {
        (determinant.is_positive() && value.is_negative())
            || (determinant.is_negative() && value.is_positive())
    };
    let u_numerator = direction_x * &(from_z.clone() * edge2_y - from_y.clone() * edge2_z);
    if conflicts(&u_numerator) {
        return Some(None);
    }
    let v_numerator = direction_x * &(from_y.clone() * edge1_z - from_z.clone() * edge1_y);
    if conflicts(&v_numerator) {
        return Some(None);
    }
    let remainder = determinant.clone() - &u_numerator - &v_numerator;
    if conflicts(&remainder) {
        return Some(None);
    }

    let t_numerator = origin_x * &exact.normal[0]
        + origin_y * &exact.normal[1]
        + origin_z * &exact.normal[2]
        - &exact.first_dot_normal;
    if conflicts(&t_numerator) {
        return Some(None);
    }
    let parameter = &t_numerator / &determinant;
    let point_x = origin_x + direction_x * &parameter;
    Some(Some((
        Point3::new(
            Real::new(point_x),
            Real::new(origin_y.clone()),
            Real::new(origin_z.clone()),
        ),
        Real::new(parameter),
    )))
}

fn prepared_exact_x_axis_query<'a>(
    prepared: &'a PreparedTriangleQuery,
    positions: [&Point3; 3],
) -> Option<&'a PreparedExactXAxisTriangle> {
    prepared
        .exact_x_axis
        .get_or_init(|| {
            let first = [
                positions[0].x.exact_rational_ref()?.clone(),
                positions[0].y.exact_rational_ref()?.clone(),
                positions[0].z.exact_rational_ref()?.clone(),
            ];
            let edge1 = [
                prepared.edge1.0[0].exact_rational_ref()?.clone(),
                prepared.edge1.0[1].exact_rational_ref()?.clone(),
                prepared.edge1.0[2].exact_rational_ref()?.clone(),
            ];
            let edge2 = [
                prepared.edge2.0[0].exact_rational_ref()?.clone(),
                prepared.edge2.0[1].exact_rational_ref()?.clone(),
                prepared.edge2.0[2].exact_rational_ref()?.clone(),
            ];
            let normal = [
                &edge1[1] * &edge2[2] - &edge1[2] * &edge2[1],
                &edge1[2] * &edge2[0] - &edge1[0] * &edge2[2],
                &edge1[0] * &edge2[1] - &edge1[1] * &edge2[0],
            ];
            let determinant_unit = -normal[0].clone();
            let first_dot_normal =
                &first[0] * &normal[0] + &first[1] * &normal[1] + &first[2] * &normal[2];
            Some(PreparedExactXAxisTriangle {
                first,
                edge1,
                edge2,
                determinant_unit,
                normal,
                first_dot_normal,
            })
        })
        .as_ref()
}

fn ray_triangle_parallel_decided(
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<bool> {
    let computed_edges;
    let (edge1, edge2) = if let Some(prepared) = prepared {
        (&prepared.edge1, &prepared.edge2)
    } else {
        computed_edges = (
            positions[1].clone() - positions[0].clone(),
            positions[2].clone() - positions[0].clone(),
        );
        (&computed_edges.0, &computed_edges.1)
    };
    let determinant = edge1.dot(&direction.cross(edge2));
    real_sign(&determinant).map(|sign| sign == RealSign::Zero)
}

fn ray_triangle_positions_hyperlimit(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
) -> Option<(Point3, Real)> {
    let exact_origin = hyperlimit_point3(origin);
    let exact_direction = hyperlimit::Point3::new(
        direction.0[0].clone(),
        direction.0[1].clone(),
        direction.0[2].clone(),
    );
    let a = hyperlimit_point3(positions[0]);
    let b = hyperlimit_point3(positions[1]);
    let c = hyperlimit_point3(positions[2]);
    match hyperlimit::classify_ray_triangle3_intersection_report(
        &exact_origin,
        &exact_direction,
        &a,
        &b,
        &c,
    ) {
        hyperlimit::PredicateOutcome::Decided { value: report, .. } => {
            if matches!(
                report.relation,
                hyperlimit::RayTriangleIntersection::Disjoint
                    | hyperlimit::RayTriangleIntersection::Coplanar
            ) {
                return None;
            }
            let point = report.point?;
            let parameter = report.parameter?;
            Some((Point3::new(point.x, point.y, point.z), parameter))
        },
        hyperlimit::PredicateOutcome::Unknown { .. } => None,
    }
}

fn hyperlimit_point3(point: &Point3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone())
}

fn canonicalize_ray_hits(hits: &mut Vec<(Point3, Real)>) {
    hits.sort_by(|a, b| real_cmp(&a.1, &b.1));
    hits.dedup_by(|a, b| {
        if let (Some(a_parameter), Some(b_parameter)) =
            (a.1.exact_rational_ref(), b.1.exact_rational_ref())
        {
            return a_parameter == b_parameter;
        }
        if real_zero(&(a.1.clone() - b.1.clone())) {
            return true;
        }
        let a_point = hyperlimit::Point3::new(a.0.x.clone(), a.0.y.clone(), a.0.z.clone());
        let b_point = hyperlimit::Point3::new(b.0.x.clone(), b.0.y.clone(), b.0.z.clone());
        matches!(
            hyperlimit::point3_equal(&a_point, &b_point).value(),
            Some(true)
        )
    });
}

impl<M: Clone + Send + Sync + Debug + PartialEq> Mesh<M> {
    /// Example: retain only polygons whose metadata matches `needle`
    #[inline]
    pub fn filter_polygons_by_metadata(&self, needle: &M) -> Mesh<M> {
        let polys = self
            .polygons
            .iter()
            .filter(|&p| &p.metadata == needle)
            .cloned()
            .collect();

        Mesh {
            polygons: polys,
            bounding_box: std::sync::OnceLock::new(),
        }
    }
}

impl<M: Clone + Send + Sync + Debug> Mesh<M> {
    /// Return a new empty mesh.
    pub const fn empty() -> Self {
        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
        }
    }

    /// Build a Mesh from an existing polygon list
    pub const fn from_polygons(polygons: Vec<Polygon<M>>) -> Self {
        Mesh {
            polygons,
            bounding_box: OnceLock::new(),
        }
    }

    fn rigid_transform_owned(self, matrix: &Matrix4) -> Self {
        let finite_matrix = matrix_f64_lossy(matrix);
        self.rigid_transform_owned_with_finite(matrix, finite_matrix)
    }

    fn rigid_transform_owned_with_finite(
        mut self,
        matrix: &Matrix4,
        finite_matrix: Option<[[f64; 4]; 4]>,
    ) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let source_geometry_identity = self.geometry_identity();
        if let Some(cached_polygons) = LAST_RIGID_TRANSFORM.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.matrix == *matrix
                        && cached.source_geometry_identity == source_geometry_identity
                })
                .map(|cached| cached.polygons.clone())
        }) {
            for (polygon, cached) in self.polygons.iter_mut().zip(cached_polygons) {
                let metadata = polygon.metadata.clone();
                *polygon = cached.with_metadata(metadata);
            }
            self.bounding_box = OnceLock::new();
            if convex_pwn {
                self.cache_convex_pwn_fact();
            }
            return self;
        }
        let cache_on_completion = PENDING_RIGID_TRANSFORM.with_borrow_mut(|pending| {
            let repeated = pending.as_ref().is_some_and(|pending| {
                pending.matrix == *matrix
                    && pending.source_geometry_identity == source_geometry_identity
            });
            *pending = Some(PendingTransform {
                source_geometry_identity: source_geometry_identity.clone(),
                matrix: matrix.clone(),
            });
            repeated
        });

        let prepared_matrix = matrix.prepare();
        let mut transformed_positions = HashMap::<u64, (Point3, u64, Option<[f64; 3]>)>::new();
        let mut transformed_normals = HashMap::<u64, Vec<(Vector3, Vector3)>>::new();
        let matrix_facts = prepared_matrix.structural_facts();
        let mut transformed_coordinates: [HashMap<[Option<u64>; 3], u64>; 3] =
            std::array::from_fn(|_| HashMap::new());
        let mut transformed_planes = HashMap::new();

        for polygon in &mut self.polygons {
            polygon.plane_id = *transformed_planes
                .entry(polygon.plane_id)
                .or_insert_with(fresh_plane_id);
            for vertex in &mut polygon.vertices {
                let source_position_id = vertex.position_id;
                let source_coordinate_ids = vertex.coordinate_ids;
                for (row, ids) in transformed_coordinates.iter_mut().enumerate() {
                    let key = std::array::from_fn(|column| {
                        (matrix_facts.entry_known_zero(row, column) != Some(true))
                            .then_some(source_coordinate_ids[column])
                    });
                    vertex.coordinate_ids[row] =
                        *ids.entry(key).or_insert_with(fresh_position_id);
                }
                if let Some((position, position_id, position_f64)) =
                    transformed_positions.get(&source_position_id)
                {
                    vertex.position = position.clone();
                    vertex.position_id = *position_id;
                    cache_position_f64(*position_id, *position_f64);
                } else {
                    let position_f64 = finite_matrix.as_ref().and_then(|matrix| {
                        transform_point_f64_lossy(matrix, vertex.position_f64_lossy()?)
                    });
                    let position = prepared_matrix
                        .transform_point3(&vertex.position)
                        .expect("rigid transforms preserve affine points");
                    let position_id = fresh_position_id();
                    transformed_positions.insert(
                        source_position_id,
                        (position.clone(), position_id, position_f64),
                    );
                    vertex.position = position;
                    vertex.position_id = position_id;
                    cache_position_f64(position_id, position_f64);
                }
                let cached_normal =
                    transformed_normals
                        .get(&source_position_id)
                        .and_then(|entries| {
                            entries
                                .iter()
                                .find(|(source, _)| source == &vertex.normal)
                                .map(|(_, transformed)| transformed.clone())
                        });
                if let Some(normal) = cached_normal {
                    vertex.normal = normal;
                } else {
                    let source_normal = vertex.normal.clone();
                    let transformed_normal =
                        prepared_matrix.transform_direction3(&source_normal);
                    transformed_normals
                        .entry(source_position_id)
                        .or_default()
                        .push((source_normal, transformed_normal.clone()));
                    vertex.normal = transformed_normal;
                }
            }
            if polygon.vertices.len() == 3 {
                polygon.plane.point_a = polygon.vertices[0].position.clone();
                polygon.plane.point_b = polygon.vertices[1].position.clone();
                polygon.plane.point_c = polygon.vertices[2].position.clone();
            } else {
                assert!(
                    polygon.plane.transform_affine_in_place(matrix),
                    "rigid transforms preserve affine plane points"
                );
            }
            polygon.invalidate_bounding_box();
        }
        self.bounding_box = OnceLock::new();
        if cache_on_completion {
            LAST_RIGID_TRANSFORM.with_borrow_mut(|cached| {
                *cached = Some(CachedRigidTransform {
                    source_geometry_identity,
                    matrix: matrix.clone(),
                    polygons: self
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        if convex_pwn {
            self.cache_convex_pwn_fact();
        }
        self
    }

    fn translate_vector_owned(mut self, vector: Vector3) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let mut transformed = HashMap::<u64, (Point3, u64)>::new();
        let mut transformed_coordinates: [HashMap<u64, u64>; 3] =
            std::array::from_fn(|_| HashMap::new());
        let mut transformed_planes = HashMap::new();
        for polygon in &mut self.polygons {
            let plane_id = *transformed_planes
                .entry(polygon.plane_id)
                .or_insert_with(fresh_plane_id);
            for vertex in &mut polygon.vertices {
                for (axis, ids) in transformed_coordinates.iter_mut().enumerate() {
                    vertex.coordinate_ids[axis] = *ids
                        .entry(vertex.coordinate_ids[axis])
                        .or_insert_with(fresh_position_id);
                }
                if let Some((position, position_id)) = transformed.get(&vertex.position_id) {
                    vertex.position = position.clone();
                    vertex.position_id = *position_id;
                } else {
                    let source_id = vertex.position_id;
                    let position = vertex.position.clone() + vector.clone();
                    let position_id = fresh_position_id();
                    transformed.insert(source_id, (position.clone(), position_id));
                    vertex.position = position;
                    vertex.position_id = position_id;
                }
            }
            polygon.plane.translate_in_place(&vector);
            polygon.invalidate_bounding_box();
            polygon.plane_id = plane_id;
        }
        self.bounding_box = OnceLock::new();
        if convex_pwn {
            self.cache_convex_pwn_fact();
        }
        self
    }

    pub(crate) fn nonuniform_scale_owned(mut self, sx: Real, sy: Real, sz: Real) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let scales = [sx, sy, sz];
        let inverse_scales = [
            scales[0]
                .clone()
                .inverse()
                .expect("shape scale is certified nonzero"),
            scales[1]
                .clone()
                .inverse()
                .expect("shape scale is certified nonzero"),
            scales[2]
                .clone()
                .inverse()
                .expect("shape scale is certified nonzero"),
        ];
        let matrix = Matrix4::affine_nonuniform_scale(scales.clone());
        let mut transformed_positions = HashMap::<u64, (Point3, u64)>::new();
        let mut transformed_normals = HashMap::<u64, Vec<(Vector3, Vector3)>>::new();
        let mut transformed_coordinates: [HashMap<u64, u64>; 3] =
            std::array::from_fn(|_| HashMap::new());
        let mut transformed_planes = HashMap::new();

        for polygon in &mut self.polygons {
            polygon.plane_id = *transformed_planes
                .entry(polygon.plane_id)
                .or_insert_with(fresh_plane_id);
            for vertex in &mut polygon.vertices {
                for (axis, ids) in transformed_coordinates.iter_mut().enumerate() {
                    vertex.coordinate_ids[axis] = *ids
                        .entry(vertex.coordinate_ids[axis])
                        .or_insert_with(fresh_position_id);
                }
                let source_position_id = vertex.position_id;
                if let Some((position, position_id)) =
                    transformed_positions.get(&source_position_id)
                {
                    vertex.position = position.clone();
                    vertex.position_id = *position_id;
                } else {
                    let position = Point3::new(
                        vertex.position.x.clone() * scales[0].clone(),
                        vertex.position.y.clone() * scales[1].clone(),
                        vertex.position.z.clone() * scales[2].clone(),
                    );
                    let position_id = fresh_position_id();
                    transformed_positions
                        .insert(source_position_id, (position.clone(), position_id));
                    vertex.position = position;
                    vertex.position_id = position_id;
                }

                let cached_normal =
                    transformed_normals
                        .get(&source_position_id)
                        .and_then(|entries| {
                            entries
                                .iter()
                                .find(|(source, _)| source == &vertex.normal)
                                .map(|(_, transformed)| transformed.clone())
                        });
                if let Some(normal) = cached_normal {
                    vertex.normal = normal;
                } else {
                    let source_normal = vertex.normal.clone();
                    let scaled = Vector3::new([
                        source_normal.0[0].clone() * inverse_scales[0].clone(),
                        source_normal.0[1].clone() * inverse_scales[1].clone(),
                        source_normal.0[2].clone() * inverse_scales[2].clone(),
                    ]);
                    transformed_normals
                        .entry(source_position_id)
                        .or_default()
                        .push((source_normal, scaled.clone()));
                    vertex.normal = scaled;
                }
            }
            if polygon.vertices.len() == 3 {
                polygon.plane.point_a = polygon.vertices[0].position.clone();
                polygon.plane.point_b = polygon.vertices[1].position.clone();
                polygon.plane.point_c = polygon.vertices[2].position.clone();
            } else {
                assert!(
                    polygon.plane.transform_affine_in_place(&matrix),
                    "nonzero diagonal shape scales preserve affine plane points"
                );
            }
            polygon.invalidate_bounding_box();
        }
        self.bounding_box = OnceLock::new();
        if convex_pwn {
            self.cache_convex_pwn_fact();
        }
        self
    }

    /// Consume and translate this mesh while reusing its polygon storage.
    pub fn into_translated(self, x: Real, y: Real, z: Real) -> Self {
        self.translate_vector_owned(Vector3::new([x, y, z]))
    }

    /// Consume and rigidly rotate this mesh while reusing its polygon storage.
    pub fn into_rotated(mut self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let source_geometry_identity = self.geometry_identity();
        let degrees = [x_deg, y_deg, z_deg];
        if let Some(cached_polygons) = LAST_ROTATION.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.degrees == degrees
                        && cached.source_geometry_identity == source_geometry_identity
                })
                .map(|cached| cached.polygons.clone())
        }) {
            for (polygon, cached) in self.polygons.iter_mut().zip(cached_polygons) {
                let metadata = polygon.metadata.clone();
                *polygon = cached.with_metadata(metadata);
            }
            self.bounding_box = OnceLock::new();
            if convex_pwn {
                self.cache_convex_pwn_fact();
            }
            return self;
        }
        let cache_on_completion = PENDING_ROTATION.with_borrow_mut(|pending| {
            let repeated = pending.as_ref().is_some_and(|pending| {
                pending.degrees == degrees
                    && pending.source_geometry_identity == source_geometry_identity
            });
            *pending = Some(PendingRotation {
                source_geometry_identity: source_geometry_identity.clone(),
                degrees: degrees.clone(),
            });
            repeated
        });

        let x = degrees[0].clone().to_radians();
        let y = degrees[1].clone().to_radians();
        let z = degrees[2].clone().to_radians();
        let (sin_x, cos_x) = (x.clone().sin(), x.cos());
        let (sin_y, cos_y) = (y.clone().sin(), y.cos());
        let (sin_z, cos_z) = (z.clone().sin(), z.cos());
        let cos_z_sin_y = cos_z.clone() * sin_y.clone();
        let sin_z_sin_y = sin_z.clone() * sin_y.clone();
        let zero = Real::zero();
        let one = Real::one();
        // Rz * Ry * Rx, expanded once. Avoiding two generic 4x4 matrix
        // products keeps the six retained trigonometric objects shallow and
        // removes zero/identity arithmetic from first-use rotations.
        let rotation = Matrix4::from_row_major([
            cos_z.clone() * cos_y.clone(),
            cos_z_sin_y.clone() * sin_x.clone() - sin_z.clone() * cos_x.clone(),
            cos_z_sin_y * cos_x.clone() + sin_z.clone() * sin_x.clone(),
            zero.clone(),
            sin_z.clone() * cos_y.clone(),
            sin_z_sin_y.clone() * sin_x.clone() + cos_z.clone() * cos_x.clone(),
            sin_z_sin_y * cos_x.clone() - cos_z * sin_x.clone(),
            zero.clone(),
            -sin_y,
            cos_y.clone() * sin_x,
            cos_y * cos_x,
            zero.clone(),
            zero.clone(),
            zero.clone(),
            zero,
            one,
        ]);
        let finite_rotation = rotation_xyz_f64_lossy(&degrees);
        let rotated = self.rigid_transform_owned_with_finite(&rotation, finite_rotation);
        if cache_on_completion {
            LAST_ROTATION.with_borrow_mut(|cached| {
                *cached = Some(CachedRotation {
                    source_geometry_identity,
                    degrees,
                    polygons: rotated
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        rotated
    }

    /// Return this mesh with replacement metadata on the mesh and every polygon.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Mesh<NewM> {
        let polygons = self
            .polygons
            .into_iter()
            .map(|polygon| polygon.with_metadata(metadata.clone()))
            .collect();

        Mesh {
            polygons,
            bounding_box: OnceLock::new(),
        }
    }

    /// Map metadata on the mesh and every polygon while preserving geometry.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, mut f: F) -> Mesh<NewM>
    where
        F: FnMut(M) -> NewM,
    {
        let polygons = self
            .polygons
            .into_iter()
            .map(|polygon| polygon.map_metadata(&mut f))
            .collect();
        Mesh {
            polygons,
            bounding_box: OnceLock::new(),
        }
    }

    /// Helper to collect all vertices from the CSG.
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    pub fn triangulate(&self) -> Mesh<M> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.triangulate().into_iter().map(move |triangle| {
                    Polygon::new(triangle.to_vec(), poly.metadata.clone())
                        .with_plane_id(poly.plane_id)
                })
            })
            .collect::<Vec<_>>();

        Mesh::from_polygons(triangles)
    }

    /// Subdivide all polygons in this Mesh 'levels' times, returning a new Mesh.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Mesh<M> {
        let new_polygons: Vec<Polygon<M>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                sub_tris.into_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                    .with_plane_id(poly.plane_id)
                })
            })
            .collect();

        Mesh::from_polygons(new_polygons)
    }

    /// Subdivide all polygons in this Mesh 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```ignore
    /// use csgrs::mesh::Mesh;
    /// use core::num::NonZeroU32;
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, ());
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, ());
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: NonZeroU32) {
        self.polygons = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let polytri = poly.subdivide_triangles(levels);
                polytri.into_iter().map(move |tri| {
                    Polygon::new(tri.to_vec(), poly.metadata.clone())
                        .with_plane_id(poly.plane_id)
                })
            })
            .collect();
    }

    /// Renormalize all polygons in this Mesh by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
    }

    /// Sample every coordinate at an explicit finite application/output boundary.
    ///
    /// This is intended for callers whose source language or file format has
    /// finite-number semantics and which want to retry an operation that could
    /// not certify a symbolic predicate. Native mesh operations do not call it
    /// implicitly.
    pub fn materialize_finite_output(&self) -> Option<Self> {
        let polygons = self
            .polygons
            .iter()
            .map(|polygon| {
                let vertices = polygon
                    .vertices
                    .iter()
                    .map(|vertex| {
                        Some(Vertex::new(
                            Point3::new(
                                Real::try_from(vertex.position.x.to_f64_lossy()?).ok()?,
                                Real::try_from(vertex.position.y.to_f64_lossy()?).ok()?,
                                Real::try_from(vertex.position.z.to_f64_lossy()?).ok()?,
                            ),
                            Vector3::new([
                                Real::try_from(vertex.normal.0[0].to_f64_lossy()?).ok()?,
                                Real::try_from(vertex.normal.0[1].to_f64_lossy()?).ok()?,
                                Real::try_from(vertex.normal.0[2].to_f64_lossy()?).ok()?,
                            ]),
                        ))
                    })
                    .collect::<Option<Vec<_>>>()?;
                Some(Polygon::new(vertices, polygon.metadata.clone()))
            })
            .collect::<Option<Vec<_>>>()?;
        Some(Self::from_polygons(polygons))
    }

    /// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two polygons sharing an edge.
    /// The angle is computed as the angle between the normal vectors of the two polygons.
    /// Normalization, dot product, clamping, and arccos are delegated to
    /// `hyperlattice`/`hyperreal`, keeping the mesh query on the same exact-
    /// geometric-computation boundary as other normal-angle predicates. See
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// Returns the angle in radians.
    pub fn dihedral_angle(p1: &Polygon<M>, p2: &Polygon<M>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        n1.angle_to(&n2).unwrap_or_else(|_| Real::zero())
    }

    /// Converts this mesh into exact vertex/index buffers suitable for rendering adapters.
    pub fn build_graphics_mesh(&self) -> GraphicsMesh {
        let triangle_capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices.len().saturating_sub(2))
            .sum::<usize>();

        let mut indices = Vec::with_capacity(triangle_capacity * 3);
        let mut vertices = Vec::with_capacity(triangle_capacity * 3);

        for polygon in &self.polygons {
            for triangle in polygon.triangulate_indices() {
                for vertex_index in triangle {
                    let vertex = &polygon.vertices[vertex_index];
                    let position = [
                        vertex.position.x.clone(),
                        vertex.position.y.clone(),
                        vertex.position.z.clone(),
                    ];
                    let normal = [
                        vertex.normal.0[0].clone(),
                        vertex.normal.0[1].clone(),
                        vertex.normal.0[2].clone(),
                    ];

                    let index = vertices.len() as u32;
                    vertices.push((position, normal));
                    indices.push(index);
                }
            }
        }

        GraphicsMesh { vertices, indices }
    }

    /// Try to extract hyperreal vertices and triangle indices.
    ///
    /// This is the native mesh-buffer view. It does not cross a primitive-float
    /// boundary; coordinates stay in [`Real`] and only the
    /// topological index carrier is lowered to `u32`.
    pub fn try_get_vertices_and_indices(
        &self,
    ) -> Result<(Vec<Point3>, Vec<[u32; 3]>), ValidationError> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|polygon| polygon.triangulate())
            .collect::<Vec<_>>();
        let mut vertices = Vec::with_capacity(triangles.len() * 3);
        let mut indices = Vec::with_capacity(triangles.len());

        for triangle in triangles {
            let base = u32::try_from(vertices.len()).map_err(|_| {
                ValidationError::MeshBufferError("mesh vertex count exceeded u32".into())
            })?;
            vertices.push(triangle[0].position.clone());
            vertices.push(triangle[1].position.clone());
            vertices.push(triangle[2].position.clone());
            indices.push([base, base + 1, base + 2]);
        }

        Ok((vertices, indices))
    }

    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this Mesh and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray’s start point.
    /// - `direction`: The ray’s direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3, Real)` where:
    /// - `Point3` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    ///
    /// Triangle intersections are evaluated with hyperreal vector operations,
    /// keeping hit ordering and deduplication in the same scalar domain as the
    /// mesh coordinates.
    pub fn ray_intersections(
        &self,
        origin: &Point3,
        direction: &Vector3,
    ) -> Vec<(Point3, Real)> {
        if let Some(hits) = LAST_RAY_INTERSECTIONS.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.origin == *origin
                        && cached.direction == *direction
                        && self.spatial_identity_matches(&cached.spatial_identity)
                })
                .map(|cached| cached.hits.clone())
        }) {
            return hits;
        }

        let certified_query = point_exact_f64(origin).zip(vector_exact_f64(direction));
        let mut candidates = Vec::new();
        for (index, polygon) in self.polygons.iter().enumerate() {
            let misses_bounds = match (certified_query, polygon.certified_f64_bounds_ref()) {
                (Some((origin, direction)), Some(bounds)) => {
                    ray_decided_miss_certified_f64_bounds(origin, direction, bounds, None)
                },
                _ => {
                    ray_decided_miss_aabb(origin, direction, polygon.bounding_box_ref(), None)
                },
            };
            if !misses_bounds {
                candidates.push(index);
            }
        }
        let mut hits = Vec::new();
        for &candidate in &candidates {
            let polygon = &self.polygons[candidate];
            if polygon.vertices.len() == 3 {
                let prepared = polygon.prepared_triangle_query_ref();
                let positions = [
                    &polygon.vertices[0].position,
                    &polygon.vertices[1].position,
                    &polygon.vertices[2].position,
                ];
                if let Some(hit) =
                    ray_triangle_positions_prepared(origin, direction, positions, prepared)
                {
                    hits.push(hit);
                }
                continue;
            }
            for [a, b, c] in polygon.triangulate_indices() {
                if let Some(hit) = ray_triangle_positions(
                    origin,
                    direction,
                    [
                        &polygon.vertices[a].position,
                        &polygon.vertices[b].position,
                        &polygon.vertices[c].position,
                    ],
                ) {
                    hits.push(hit);
                }
            }
        }
        // Sort and deduplicate only when Hyper proves exact identity.
        canonicalize_ray_hits(&mut hits);

        LAST_RAY_INTERSECTIONS.with_borrow_mut(|cached| {
            *cached = Some(CachedRayIntersections {
                spatial_identity: self.spatial_identity(),
                origin: origin.clone(),
                direction: direction.clone(),
                hits: hits.clone(),
            });
        });
        hits
    }

    /// Find all intersection points between a polyline and this mesh's
    /// triangulated surface.
    ///
    /// Each consecutive pair of points defines one segment. Hits are deduplicated
    /// locally and returned in polyline order. Deduplication requires exact
    /// hit-point equality through `hyperlattice::Vector3` and `Real`,
    /// keeping this topology-affecting equality decision out of local f64
    /// tolerance arithmetic. This follows Yap's
    /// exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn intersect_polyline(&self, polyline: &[Point3]) -> Vec<Point3> {
        if polyline.len() < 2 {
            return Vec::new();
        }
        if let Some(hits) = LAST_POLYLINE_INTERSECTIONS.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.polyline == polyline
                        && self.spatial_identity_matches(&cached.spatial_identity)
                })
                .map(|cached| cached.hits.clone())
        }) {
            return hits;
        }

        let mut hits: Vec<Point3> = Vec::new();

        for seg in polyline.windows(2) {
            let seg_start = seg[0].clone();
            let seg_end = seg[1].clone();
            let seg_dir = &seg_end - &seg_start;
            if real_zero(&seg_dir.dot(&seg_dir)) {
                continue;
            }

            if let Some(mut seg_hits) = LAST_RAY_INTERSECTIONS.with_borrow(|cached| {
                let cached = cached.as_ref()?;
                if cached.origin != seg_start
                    || !self.spatial_identity_matches(&cached.spatial_identity)
                {
                    return None;
                }
                let axis = cached
                    .direction
                    .0
                    .iter()
                    .position(|component| !real_zero(component))?;
                let scale =
                    (seg_dir.0[axis].clone() / cached.direction.0[axis].clone()).ok()?;
                if !matches!(real_sign(&scale), Some(RealSign::Positive))
                    || (0..3).any(|component| {
                        !real_zero(
                            &(seg_dir.0[component].clone()
                                - cached.direction.0[component].clone() * scale.clone()),
                        )
                    })
                {
                    return None;
                }
                Some(
                    cached
                        .hits
                        .iter()
                        .filter_map(|(point, parameter)| {
                            let parameter = (parameter.clone() / scale.clone()).ok()?;
                            (!real_lt(&parameter, &Real::zero())
                                && !real_gt(&parameter, &Real::one()))
                            .then(|| (point.clone(), parameter))
                        })
                        .collect::<Vec<_>>(),
                )
            }) {
                seg_hits.sort_by(|a, b| real_cmp(&a.1, &b.1));
                for (point, _) in seg_hits {
                    if let Some(last) = hits.last() {
                        let point_h = hyperlimit_point3(&point);
                        let last_h = hyperlimit_point3(last);
                        if matches!(
                            hyperlimit::point3_equal(&point_h, &last_h).value(),
                            Some(true)
                        ) {
                            continue;
                        }
                    }
                    hits.push(point);
                }
                continue;
            }
            let certified_query = point_exact_f64(&seg_start).zip(vector_exact_f64(&seg_dir));

            let mut seg_hits: Vec<(Point3, Real)> = Vec::new();

            for polygon in &self.polygons {
                let misses_bounds = match (certified_query, polygon.certified_f64_bounds_ref())
                {
                    (Some((origin, direction)), Some(bounds)) => {
                        ray_decided_miss_certified_f64_bounds(
                            origin,
                            direction,
                            bounds,
                            Some(1.0),
                        )
                    },
                    _ => ray_decided_miss_aabb(
                        &seg_start,
                        &seg_dir,
                        polygon.bounding_box_ref(),
                        Some(&Real::one()),
                    ),
                };
                if misses_bounds {
                    continue;
                }
                for [a, b, c] in polygon.triangulate_indices() {
                    let prepared = polygon.prepared_triangle_query_ref();
                    let positions = [
                        &polygon.vertices[a].position,
                        &polygon.vertices[b].position,
                        &polygon.vertices[c].position,
                    ];
                    if let Some((point, t)) = ray_triangle_positions_prepared(
                        &seg_start, &seg_dir, positions, prepared,
                    ) && !real_lt(&t, &Real::zero())
                        && !real_gt(&t, &Real::one())
                    {
                        seg_hits.push((point, t));
                    }
                }
            }

            seg_hits.sort_by(|a, b| real_cmp(&a.1, &b.1));

            for (point, _) in seg_hits {
                if let Some(last) = hits.last() {
                    let point_h = hyperlimit::Point3::new(
                        point.x.clone(),
                        point.y.clone(),
                        point.z.clone(),
                    );
                    let last_h = hyperlimit::Point3::new(
                        last.x.clone(),
                        last.y.clone(),
                        last.z.clone(),
                    );
                    if matches!(
                        hyperlimit::point3_equal(&point_h, &last_h).value(),
                        Some(true)
                    ) {
                        continue;
                    }
                }
                hits.push(point);
            }
        }

        LAST_POLYLINE_INTERSECTIONS.with_borrow_mut(|cached| {
            *cached = Some(CachedPolylineIntersections {
                spatial_identity: self.spatial_identity(),
                polyline: polyline.to_vec(),
                hits: hits.clone(),
            });
        });
        hits
    }

    /// Uses hyperreal triangle ray intersections to check if a point is inside a `Mesh`.
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```ignore
    /// # use csgrs::mesh::Mesh;
    /// # use hyperlattice::Point3;
    /// # use hyperlattice::Vector3;
    /// let csg_cube = Mesh::<()>::cube(6.0, ());
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3) -> bool {
        if self.polygons.is_empty() {
            return false;
        }

        let bounds = self.bounding_box();
        if real_lt(&point.x, &bounds.mins.x)
            || real_gt(&point.x, &bounds.maxs.x)
            || real_lt(&point.y, &bounds.mins.y)
            || real_gt(&point.y, &bounds.maxs.y)
            || real_lt(&point.z, &bounds.mins.z)
            || real_gt(&point.z, &bounds.maxs.z)
        {
            return false;
        }

        let one = Real::one();
        let eight = Real::from(8_u8);
        let sixteen = Real::from(16_u8);
        let direction = Vector3::from_xyz(
            one.clone(),
            (Real::from(3_u8) / eight).unwrap_or_else(|_| one.clone()),
            (Real::from(5_u8) / sixteen).unwrap_or(one),
        );

        // A nonparallel surface contact is retained by the ray query at exact
        // parameter zero. Only parallel triangles need a separate containment
        // replay because a coplanar ray intentionally has no unique hit point.
        let query = hyperlimit_point3(point);
        let certified_point = point_exact_f64(point);
        for polygon in &self.polygons {
            let certified_bounds = polygon.certified_f64_bounds_ref();
            if let (Some(point), Some(bounds)) = (certified_point, certified_bounds)
                && point[1] >= bounds.min[1]
                && point[1] <= bounds.max[1]
                && point[2] >= bounds.min[2]
                && point[2] <= bounds.max[2]
                && polygon.vertices.len() == 3
                && let Some(prepared) = polygon.prepared_triangle_query_ref()
            {
                let _ = prepared_exact_x_axis_query(
                    prepared,
                    [
                        &polygon.vertices[0].position,
                        &polygon.vertices[1].position,
                        &polygon.vertices[2].position,
                    ],
                );
            }
            let outside_bounds = match (certified_point, certified_bounds) {
                (Some(point), Some(bounds)) => {
                    point_decided_outside_certified_f64_bounds(point, bounds)
                },
                _ => point_decided_outside_aabb(point, polygon.bounding_box_ref()),
            };
            if outside_bounds {
                continue;
            }
            for [a, b, c] in polygon.triangulate_indices() {
                let positions = [
                    &polygon.vertices[a].position,
                    &polygon.vertices[b].position,
                    &polygon.vertices[c].position,
                ];
                if matches!(
                    ray_triangle_parallel_decided(
                        &direction,
                        positions,
                        polygon.prepared_triangle_query_ref(),
                    ),
                    Some(false)
                ) {
                    continue;
                }
                let [a, b, c] = positions.map(hyperlimit_point3);
                if matches!(
                    hyperlimit::classify_point_triangle3(&a, &b, &c, &query).value(),
                    Some(
                        hyperlimit::Triangle3Location::Inside
                            | hyperlimit::Triangle3Location::OnEdge
                            | hyperlimit::Triangle3Location::OnVertex
                    )
                ) {
                    return false;
                }
            }
        }

        let hits = self.ray_intersections(point, &direction);
        if hits.iter().any(|(_, parameter)| real_zero(parameter)) {
            return false;
        }
        hits.len() % 2 == 1
    }

    /// Mass properties through the Hyper physics stack.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> Result<(Real, Point3, Matrix4), ValidationError> {
        let geometry_identity = self.geometry_identity();
        if let Some((mass, center)) = LAST_BASIC_MASS_PROPERTIES.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.density == density && cached.geometry_identity == geometry_identity
                })
                .map(|cached| (cached.mass.clone(), cached.center.clone()))
        }) {
            return Ok((mass, center, Matrix4::identity()));
        }
        if !matches!(real_sign(&density), Some(RealSign::Positive)) {
            return Err(ValidationError::Other(
                "mass density must be certified positive".to_owned(),
                None,
            ));
        }
        if let Some((mass, center)) = self.finite_basic_mass_properties(&density) {
            LAST_BASIC_MASS_PROPERTIES.with_borrow_mut(|cached| {
                *cached = Some(CachedBasicMassProperties {
                    geometry_identity,
                    density,
                    mass: mass.clone(),
                    center: center.clone(),
                });
            });
            return Ok((mass, center, Matrix4::identity()));
        }

        let mut signed_volume_numerator = Real::zero();
        let mut first_moment_numerator = [Real::zero(), Real::zero(), Real::zero()];
        for polygon in &self.polygons {
            for [a, b, c] in polygon.triangulate_indices() {
                let a = &polygon.vertices[a].position;
                let b = &polygon.vertices[b].position;
                let c = &polygon.vertices[c].position;
                let determinant = Real::signed_product_sum(
                    [true, false, false, true, true, false],
                    [
                        [&a.x, &b.y, &c.z],
                        [&a.x, &b.z, &c.y],
                        [&a.y, &b.x, &c.z],
                        [&a.y, &b.z, &c.x],
                        [&a.z, &b.x, &c.y],
                        [&a.z, &b.y, &c.x],
                    ],
                );
                signed_volume_numerator = Real::signed_product_sum(
                    [true, true],
                    [[&signed_volume_numerator], [&determinant]],
                );
                for (axis, accumulator) in first_moment_numerator.iter_mut().enumerate() {
                    let coordinates =
                        [[&a.x, &b.x, &c.x], [&a.y, &b.y, &c.y], [&a.z, &b.z, &c.z]][axis];
                    let vertex_sum =
                        Real::signed_product_sum([true, true, true], coordinates.map(|v| [v]));
                    *accumulator = Real::signed_product_sum(
                        [true, true],
                        [[accumulator, &Real::one()], [&vertex_sum, &determinant]],
                    );
                }
            }
        }
        let sign = real_sign(&signed_volume_numerator);
        if !matches!(sign, Some(RealSign::Positive | RealSign::Negative)) {
            return Err(ValidationError::Other(
                "mesh volume is zero or could not be certified".to_owned(),
                None,
            ));
        }
        let signed_volume = (signed_volume_numerator.clone() / Real::from(6_u8))
            .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?;
        let volume = if matches!(sign, Some(RealSign::Negative)) {
            -signed_volume
        } else {
            signed_volume
        };
        let center_denominator = signed_volume_numerator * Real::from(4_u8);
        let center = Point3::new(
            (first_moment_numerator[0].clone() / center_denominator.clone())
                .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?,
            (first_moment_numerator[1].clone() / center_denominator.clone())
                .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?,
            (first_moment_numerator[2].clone() / center_denominator)
                .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?,
        );
        let mass = density.clone() * volume;
        LAST_BASIC_MASS_PROPERTIES.with_borrow_mut(|cached| {
            *cached = Some(CachedBasicMassProperties {
                geometry_identity,
                density,
                mass: mass.clone(),
                center: center.clone(),
            });
        });
        Ok((mass, center, Matrix4::identity()))
    }

    fn finite_basic_mass_properties(&self, density: &Real) -> Option<(Real, Point3)> {
        let density = density.to_f64_lossy()?;
        if !density.is_finite() || density <= 0.0 {
            return None;
        }
        let mut sums = [0.0_f64; 4];
        let mut compensations = [0.0_f64; 4];
        let mut add = |slot: usize, value: f64| {
            let corrected = value - compensations[slot];
            let next = sums[slot] + corrected;
            compensations[slot] = (next - sums[slot]) - corrected;
            sums[slot] = next;
        };
        for polygon in &self.polygons {
            let mut accumulate_triangle = |[ia, ib, ic]: [usize; 3]| -> Option<()> {
                let points = [ia, ib, ic].map(|index| {
                    let point = &polygon.vertices[index].position;
                    Some([
                        point.x.to_f64_lossy()?,
                        point.y.to_f64_lossy()?,
                        point.z.to_f64_lossy()?,
                    ])
                });
                let [Some(a), Some(b), Some(c)] = points else {
                    return None;
                };
                let determinant = a[0] * (b[1] * c[2] - b[2] * c[1])
                    - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]);
                if !determinant.is_finite() {
                    return None;
                }
                add(0, determinant);
                add(1, (a[0] + b[0] + c[0]) * determinant);
                add(2, (a[1] + b[1] + c[1]) * determinant);
                add(3, (a[2] + b[2] + c[2]) * determinant);
                Some(())
            };
            if polygon.vertices.len() == 3 {
                accumulate_triangle([0, 1, 2])?;
            } else {
                for triangle in polygon.triangulate_indices() {
                    accumulate_triangle(triangle)?;
                }
            }
        }
        if !sums.iter().all(|value| value.is_finite()) || sums[0] == 0.0 {
            return None;
        }
        let volume = (sums[0] / 6.0).abs();
        let denominator = 4.0 * sums[0];
        let mass = Real::try_from(density * volume).ok()?;
        let center = Point3::new(
            Real::try_from(sums[1] / denominator).ok()?,
            Real::try_from(sums[2] / denominator).ok()?,
            Real::try_from(sums[3] / denominator).ok()?,
        );
        Some((mass, center))
    }

    /// Exact uniform-density mass properties through `hyperphysics`.
    ///
    /// This is the report-bearing counterpart to [`Mesh::mass_properties`]. Mesh
    /// coordinates are finite `csgrs` adapter scalars at this boundary; each
    /// coordinate and the density are lifted into [`hyperphysics::Real`] before
    /// volume, center of mass, and inertia are accumulated. The report-bearing
    /// path follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>) by keeping the lossy
    /// exact object layer.
    pub fn exact_mass_properties(
        &self,
        density: Real,
    ) -> Result<HyperMassPropertyReport3, ValidationError> {
        let geometry_identity = self.geometry_identity();
        if let Some(report) = LAST_MASS_PROPERTIES.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.density == density && cached.geometry_identity == geometry_identity
                })
                .map(|cached| cached.report.clone())
        }) {
            return Ok(report);
        }

        let mesh = self.to_hyperphysics_closed_triangle_mesh()?;
        let report = mesh
            .uniform_density_mass_properties(density.clone())
            .map_err(|err| ValidationError::Other(format!("{err:?}"), None))?;
        LAST_MASS_PROPERTIES.with_borrow_mut(|cached| {
            *cached = Some(CachedMassProperties {
                geometry_identity,
                density,
                report: report.clone(),
            });
        });
        Ok(report)
    }

    fn geometry_identity(&self) -> Vec<u64> {
        let capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices.len() + 1)
            .sum();
        let mut identity = Vec::with_capacity(capacity);
        for polygon in &self.polygons {
            identity.push(u64::try_from(polygon.vertices.len()).unwrap_or(u64::MAX));
            identity.extend(polygon.vertices.iter().map(|vertex| vertex.position_id));
        }
        identity
    }

    fn geometry_identity_matches(&self, identity: &[u64]) -> bool {
        let mut expected = identity.iter().copied();
        for polygon in &self.polygons {
            if expected.next()
                != Some(u64::try_from(polygon.vertices.len()).unwrap_or(u64::MAX))
            {
                return false;
            }
            for vertex in &polygon.vertices {
                if expected.next() != Some(vertex.position_id) {
                    return false;
                }
            }
        }
        expected.next().is_none()
    }

    fn spatial_identity(&self) -> Vec<u64> {
        self.polygons.iter().map(|polygon| polygon.plane_id).collect()
    }

    fn spatial_identity_matches(&self, identity: &[u64]) -> bool {
        self.polygons.len() == identity.len()
            && self
                .polygons
                .iter()
                .zip(identity)
                .all(|(polygon, plane_id)| polygon.plane_id == *plane_id)
    }

    /// Converts triangulated mesh polygons into a `hyperphysics` closed mesh carrier.
    pub fn to_hyperphysics_closed_triangle_mesh(
        &self,
    ) -> Result<HyperClosedTriangleMesh3, ValidationError> {
        let tri_mesh = self.triangulate();
        let triangles = tri_mesh
            .polygons
            .iter()
            .map(|polygon| {
                if polygon.vertices.len() != 3 {
                    return Err(ValidationError::TooFewPoints(Point3::origin()));
                }
                Ok(HyperTriangle3::new([
                    hyperphysics_vector3_from_point3(&polygon.vertices[0].position)?,
                    hyperphysics_vector3_from_point3(&polygon.vertices[1].position)?,
                    hyperphysics_vector3_from_point3(&polygon.vertices[2].position)?,
                ]))
            })
            .collect::<Result<Vec<_>, _>>()?;
        HyperClosedTriangleMesh3::new(triangles)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), None))
    }

    /// Convert a Mesh into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh, PrimitiveTopology};

        let triangulated_mesh = &self.triangulate();
        let polygons = &triangulated_mesh.polygons;

        // Prepare buffers
        let mut positions_32 = Vec::new();
        let mut normals_32 = Vec::new();
        let mut indices = Vec::with_capacity(polygons.len() * 3);

        let mut index_start = 0u32;

        // Each polygon is assumed to have exactly 3 vertices after tessellation.
        for poly in polygons {
            // skip any degenerate polygons
            if poly.vertices.len() != 3 {
                continue;
            }

            // push 3 positions/normals
            for v in &poly.vertices {
                positions_32.push(v.position.to_f32_array_lossy().unwrap_or([0.0; 3]));
                normals_32.push([
                    v.normal.0[0].to_f32_lossy().unwrap_or(0.0),
                    v.normal.0[1].to_f32_lossy().unwrap_or(0.0),
                    v.normal.0[2].to_f32_lossy().unwrap_or(0.0),
                ]);
            }

            // triangle indices
            indices.push(index_start);
            indices.push(index_start + 1);
            indices.push(index_start + 2);
            index_start += 3;
        }

        // Create the mesh with the new 2-argument constructor
        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());

        // Insert attributes. Note the `<Vec<[f32; 3]>>` usage.
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals_32);

        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));

        mesh
    }

    /// Return a new mesh representing the exact hypermesh union, or the reason
    /// hypermesh could not import, validate, or materialize the result.
    pub fn try_union(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Union)?
            .try_union()
    }

    /// Return a new mesh representing the exact hypermesh difference, or the
    /// typed reason hypermesh could not produce it.
    pub fn try_difference(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Difference)?
            .try_difference()
    }

    /// Return a new mesh representing the exact hypermesh intersection, or the
    /// typed reason hypermesh could not produce it.
    pub fn try_intersection(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Intersection)?
            .try_intersection()
    }

    /// Return a new mesh representing the exact hypermesh symmetric
    /// difference, or the typed reason hypermesh could not produce it.
    pub fn try_xor(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Xor)?
            .try_xor()
    }
}

impl Mesh<()> {
    /// Return a new empty mesh.
    pub const fn new() -> Self {
        Self::empty()
    }
}

impl<M: Clone + Send + Sync + Debug> CSG for Mesh<M> {
    fn union_distributed(copies: Vec<Self>) -> Self {
        let bounds = copies.iter().map(CSG::bounding_box).collect::<Vec<_>>();
        let pairwise_disjoint = bounds.iter().enumerate().all(|(left_index, left)| {
            bounds
                .iter()
                .skip(left_index + 1)
                .all(|right| aabbs_decided_disjoint(left, right))
        });

        if pairwise_disjoint {
            let polygon_count = copies.iter().map(|mesh| mesh.polygons.len()).sum();
            let mut polygons = Vec::with_capacity(polygon_count);
            for mesh in copies {
                polygons.extend(mesh.polygons);
            }
            return Mesh::from_polygons(polygons);
        }

        copies
            .into_iter()
            .reduce(|acc, mesh| acc.union(&mesh))
            .expect("distribution always produces at least one copy")
    }

    /// Return a new Mesh representing union of the two Meshes.
    ///
    /// ```text
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn union(&self, other: &Mesh<M>) -> Mesh<M> {
        if self.polygons.is_empty() {
            return other.clone();
        }
        if other.polygons.is_empty() {
            return self.clone();
        }
        self.try_union(other)
            .unwrap_or_else(|error| panic!("hypermesh union failed: {error}"))
    }

    /// Return a new Mesh representing diffarence of the two Meshes.
    ///
    /// ```text
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn difference(&self, other: &Mesh<M>) -> Mesh<M> {
        if self.polygons.is_empty() {
            return Mesh::empty();
        }
        if other.polygons.is_empty() {
            return self.clone();
        }
        self.try_difference(other)
            .unwrap_or_else(|error| panic!("hypermesh difference failed: {error}"))
    }

    /// Return a new CSG representing intersection of the two CSG's.
    ///
    /// ```text
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn intersection(&self, other: &Mesh<M>) -> Mesh<M> {
        if self.polygons.is_empty() || other.polygons.is_empty() {
            return Mesh::empty();
        }
        self.try_intersection(other)
            .unwrap_or_else(|error| panic!("hypermesh intersection failed: {error}"))
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
    ///
    /// ```text
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn xor(&self, other: &Mesh<M>) -> Mesh<M> {
        self.try_xor(other)
            .unwrap_or_else(|error| panic!("hypermesh xor failed: {error}"))
    }

    fn translate_vector(&self, vector: Vector3) -> Self {
        self.clone().translate_vector_owned(vector)
    }

    fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        self.clone().into_rotated(x_deg, y_deg, z_deg)
    }

    fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        let nonzero = [&sx, &sy, &sz]
            .into_iter()
            .all(|scale| !matches!(real_sign(scale), Some(RealSign::Zero) | None));
        if nonzero {
            return self.clone().nonuniform_scale_owned(sx, sy, sz);
        }
        self.transform(&Matrix4::affine_nonuniform_scale([sx, sy, sz]))
    }

    fn mirror(&self, plane: Plane) -> Self {
        let Some(matrix) = finite_reflection(&plane) else {
            return self.clone();
        };
        self.clone().rigid_transform_owned(&matrix).inverse()
    }

    /// **Mathematical Foundation: General 3D Transformations**
    ///
    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to Mesh.
    /// This implements the complete theory of affine transformations in homogeneous coordinates.
    ///
    /// ## **Transformation Mathematics**
    ///
    /// ### **Homogeneous Coordinates**
    /// Points and vectors are represented in 4D homogeneous coordinates:
    /// - **Point**: (x, y, z, 1)ᵀ → transforms as p' = Mp
    /// - **Vector**: (x, y, z, 0)ᵀ → transforms as v' = Mv
    /// - **Normal**: n'ᵀ = nᵀM⁻¹ (inverse transpose rule)
    ///
    /// ### **Normal Vector Transformation**
    /// Normals require special handling to remain perpendicular to surfaces:
    /// ```text
    /// If: T(p)·n = 0 (tangent perpendicular to normal)
    /// Then: T(p)·T(n) ≠ 0 in general
    /// But: T(p)·(M⁻¹)ᵀn = 0 ✓
    /// ```
    /// **Proof**: (Mp)ᵀ(M⁻¹)ᵀn = pᵀMᵀ(M⁻¹)ᵀn = pᵀ(M⁻¹M)ᵀn = pᵀn = 0
    ///
    /// ### **Numerical Stability**
    /// - **Degeneracy Detection**: Check determinant before inversion
    /// - **Homogeneous Division**: Validate w-coordinate after transformation
    /// - **Precision**: Maintain accuracy through matrix decomposition
    ///
    /// ## **Algorithm Complexity**
    /// - **Vertices**: O(n) matrix-vector multiplications
    /// - **Matrix Inversion**: O(1) for 4×4 matrices
    /// - **Plane Updates**: O(n) plane reconstructions from transformed vertices
    ///
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D
    ///
    /// Exact-rational transformed normals are normalized into finite dyadic
    /// shading attributes; symbolic normals retain checked `Real`
    /// normalization. Vertex normals are never consumed by topology or
    /// predicates. This keeps the inverse-transpose normal path on the same
    /// exact-aware boundary discipline as other finite mesh attributes; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn transform(&self, mat: &Matrix4) -> Mesh<M> {
        let source_geometry_identity = self.geometry_identity();
        if let Some(cached_polygons) = LAST_GENERAL_TRANSFORM.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.matrix == *mat
                        && cached.source_geometry_identity == source_geometry_identity
                })
                .map(|cached| cached.polygons.clone())
        }) {
            let mut mesh = self.clone();
            for (polygon, cached) in mesh.polygons.iter_mut().zip(cached_polygons) {
                let metadata = polygon.metadata.clone();
                *polygon = cached.with_metadata(metadata);
            }
            mesh.bounding_box = OnceLock::new();
            return mesh;
        }
        let cache_on_completion = PENDING_GENERAL_TRANSFORM.with_borrow_mut(|pending| {
            let repeated = pending.as_ref().is_some_and(|pending| {
                pending.matrix == *mat
                    && pending.source_geometry_identity == source_geometry_identity
            });
            *pending = Some(PendingTransform {
                source_geometry_identity: source_geometry_identity.clone(),
                matrix: mat.clone(),
            });
            repeated
        });

        let mut prepared_matrix = mat.prepare();
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match prepared_matrix.inverse() {
            Ok(inv) => inv.transpose(),
            Err(_) => {
                eprintln!(
                    "Warning: Transformation matrix is not invertible, using identity for normals"
                );
                Matrix4::identity()
            },
        };
        let prepared_inverse_transpose = mat_inv_transpose.prepare();

        let finite_matrix = matrix_f64_lossy(mat);
        let mut mesh = self.clone();
        let mut transformed_positions = HashMap::<u64, (Point3, u64, Option<[f64; 3]>)>::new();
        let mut transformed_normals = HashMap::<u64, Vec<(Vector3, Vector3)>>::new();
        let matrix_facts = prepared_matrix.structural_facts();
        let mut transformed_coordinates: [HashMap<[Option<u64>; 3], u64>; 3] =
            std::array::from_fn(|_| HashMap::new());
        let mut transformed_planes = HashMap::new();

        for poly in &mut mesh.polygons {
            let plane_id = *transformed_planes
                .entry(poly.plane_id)
                .or_insert_with(fresh_plane_id);
            let mut vertices = poly.vertices_mut_with_managed_identity();
            for vert in vertices.iter_mut() {
                let source_position_id = vert.position_id;
                let source_coordinate_ids = vert.coordinate_ids;
                for (row, ids) in transformed_coordinates.iter_mut().enumerate() {
                    let key = std::array::from_fn(|column| {
                        (matrix_facts.entry_known_zero(row, column) != Some(true))
                            .then_some(source_coordinate_ids[column])
                    });
                    vert.coordinate_ids[row] =
                        *ids.entry(key).or_insert_with(fresh_position_id);
                }
                if let Some((position, position_id, position_f64)) =
                    transformed_positions.get(&source_position_id)
                {
                    vert.position = position.clone();
                    vert.position_id = *position_id;
                    cache_position_f64(*position_id, *position_f64);
                } else {
                    let position_f64 = finite_matrix.as_ref().and_then(|matrix| {
                        transform_point_f64_lossy(matrix, vert.position_f64_lossy()?)
                    });
                    match prepared_matrix.transform_point3(&vert.position) {
                        Ok(position) => {
                            let position_id = fresh_position_id();
                            transformed_positions.insert(
                                source_position_id,
                                (position.clone(), position_id, position_f64),
                            );
                            vert.position = position;
                            vert.position_id = position_id;
                            cache_position_f64(position_id, position_f64);
                        },
                        Err(_) => {
                            eprintln!(
                                "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                            );
                            continue;
                        },
                    }
                }

                // Transform normal using inverse transpose rule
                let cached_normal =
                    transformed_normals
                        .get(&source_position_id)
                        .and_then(|entries| {
                            entries
                                .iter()
                                .find(|(source, _)| source == &vert.normal)
                                .map(|(_, transformed)| transformed.clone())
                        });
                if let Some(normal) = cached_normal {
                    vert.normal = normal;
                } else {
                    let source_normal = vert.normal.clone();
                    let transformed_normal =
                        prepared_inverse_transpose.transform_direction3(&source_normal);
                    if let Some(normal) = finite_normalized_exact_rational(&transformed_normal)
                        .or_else(|| transformed_normal.normalize_checked().ok())
                    {
                        transformed_normals
                            .entry(source_position_id)
                            .or_default()
                            .push((source_normal, normal.clone()));
                        vert.normal = normal;
                    }
                }
            }
            drop(vertices);
            poly.plane_id = plane_id;
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = OnceLock::new();

        if cache_on_completion {
            LAST_GENERAL_TRANSFORM.with_borrow_mut(|cached| {
                *cached = Some(CachedGeneralTransform {
                    source_geometry_identity,
                    matrix: mat.clone(),
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }

        mesh
    }

    /// Returns an axis-aligned bounding box indicating the 3D bounds of all
    /// `polygons`.
    fn bounding_box(&self) -> Aabb {
        self.bounding_box
            .get_or_init(|| {
                let mut bounds = None;
                for polygon in &self.polygons {
                    if let Some(polygon_bounds) = polygon.cached_bounding_box_ref() {
                        include_point3_bounds(&mut bounds, &polygon_bounds.mins);
                        include_point3_bounds(&mut bounds, &polygon_bounds.maxs);
                    } else {
                        for vertex in &polygon.vertices {
                            include_point3_bounds(&mut bounds, &vertex.position);
                        }
                    }
                }
                let Some((mins, maxs)) = bounds else {
                    return Aabb::origin();
                };
                Aabb::new(mins, maxs)
            })
            .clone()
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Mesh (flip inside vs. outside)
    fn inverse(&self) -> Mesh<M> {
        let mut mesh = self.clone();
        for p in &mut mesh.polygons {
            p.flip();
        }
        mesh
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Send + Sync + Debug> Mesh<M> {
    /// Convert a Profile into a Mesh.
    ///
    /// Closed area sketches are consumed only from hypercurve-owned finite
    /// [`hypercurve::FiniteRegionProfile2`] projections. `Region2` and
    /// `CurveString2` are the CAD source of truth. The grouping follows the
    /// point-in-polygon ownership structure surveyed by Hormann and Agathos,
    /// "The point in polygon problem for arbitrary polygons," *Computational
    /// Geometry* 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>). This keeps the
    /// conversion aligned with the exact-geometric-computation boundary
    /// discipline from Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>): topology lives in
    /// hyper geometry, while mesh vertices are the finite API-boundary
    /// realization.
    pub fn from_profile(sketch: Profile, metadata: M) -> Self {
        fn ring_to_vertices(ring: &[[f64; 2]]) -> Vec<Vertex> {
            let mut vertices: Vec<_> = ring
                .iter()
                .map(|p| {
                    Vertex::new(
                        Point3::new(
                            Real::try_from(p[0]).unwrap_or_else(|_| Real::zero()),
                            Real::try_from(p[1]).unwrap_or_else(|_| Real::zero()),
                            Real::zero(),
                        ),
                        Vector3::z(),
                    )
                })
                .collect();
            if vertices.first() == vertices.last() {
                vertices.pop();
            }
            vertices
        }

        fn ring_to_polygon<M: Clone + Send + Sync>(
            ring: &[[f64; 2]],
            metadata: &M,
        ) -> Option<Polygon<M>> {
            let vertices = ring_to_vertices(ring);
            (vertices.len() >= 3).then(|| Polygon::new(vertices, metadata.clone()))
        }

        let projection_options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let region_profiles = match sketch.project_region_profiles(&projection_options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        if !region_profiles.is_empty() {
            let final_polygons = region_profiles
                .iter()
                .flat_map(|profile| {
                    std::iter::once(profile.material().points())
                        .chain(profile.holes().iter().map(|hole| hole.points()))
                })
                .filter_map(|ring| ring_to_polygon(ring, &metadata))
                .collect();

            return Mesh {
                polygons: final_polygons,
                bounding_box: OnceLock::new(),
            };
        }

        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::{Real, hreal_from_f64, tolerance};

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn decided_disjoint_mesh_booleans_use_exact_set_identities() {
        let left = Mesh::cube(r(2.0), ());
        let right = Mesh::cube(r(2.0), ()).translate(r(10.0), Real::zero(), Real::zero());

        assert!(aabbs_decided_disjoint(
            &left.bounding_box(),
            &right.bounding_box()
        ));
        assert_eq!(left.try_union(&right).unwrap().polygons.len(), 12);
        assert_eq!(left.try_difference(&right).unwrap().polygons.len(), 6);
        assert!(left.try_intersection(&right).unwrap().polygons.is_empty());
        assert_eq!(left.try_xor(&right).unwrap().polygons.len(), 12);

        let touching = Mesh::cube(r(2.0), ()).translate(r(2.0), Real::zero(), Real::zero());
        assert!(!aabbs_decided_disjoint(
            &left.bounding_box(),
            &touching.bounding_box()
        ));
    }

    #[test]
    fn streaming_ray_intersections_match_materialized_triangle_enumeration() {
        let mesh = Mesh::sphere(r(10.0), 16, 8, ());
        let origin = p3(-20.0, 0.0, 0.0);
        let direction = Vector3::x();
        let mut materialized = mesh
            .polygons
            .iter()
            .flat_map(Polygon::triangulate)
            .filter_map(|triangle| ray_triangle_intersection(&origin, &direction, &triangle))
            .collect::<Vec<_>>();
        canonicalize_ray_hits(&mut materialized);

        assert_eq!(mesh.ray_intersections(&origin, &direction), materialized);
    }

    #[test]
    fn sphere_query_bounds_are_certified_and_selective() {
        let mesh = Mesh::sphere(r(10.0), 32, 16, ());
        let origin = p3(-20.0, 0.0, 0.0);
        let direction = Vector3::x();
        let certified_query = point_exact_f64(&origin)
            .zip(vector_exact_f64(&direction))
            .unwrap();
        let certified = mesh
            .polygons
            .iter()
            .filter_map(Polygon::certified_f64_bounds_ref)
            .count();
        let candidates = mesh
            .polygons
            .iter()
            .filter(|polygon| {
                let bounds = polygon.certified_f64_bounds_ref().unwrap();
                !ray_decided_miss_certified_f64_bounds(
                    certified_query.0,
                    certified_query.1,
                    bounds,
                    None,
                )
            })
            .count();

        assert_eq!(certified, mesh.polygons.len());
        assert!(candidates < mesh.polygons.len() / 8, "{candidates}");
    }

    #[test]
    fn query_memoization_tracks_public_geometry_edits() {
        let mut mesh = Mesh::cube(r(2.0), ());
        let origin = p3(-10.0, 0.0, 0.0);
        let direction = Vector3::x();
        let polyline = [origin.clone(), p3(10.0, 0.0, 0.0)];
        let first_ray = mesh.ray_intersections(&origin, &direction);
        let first_polyline = mesh.intersect_polyline(&polyline);
        assert_eq!(mesh.ray_intersections(&origin, &direction), first_ray);
        assert_eq!(mesh.intersect_polyline(&polyline), first_polyline);

        for polygon in &mut mesh.polygons {
            for vertex in polygon.vertices_mut().iter_mut() {
                vertex.position.x += Real::from(5_u8);
            }
        }

        let edited_ray = mesh.ray_intersections(&origin, &direction);
        let edited_polyline = mesh.intersect_polyline(&polyline);
        assert_ne!(edited_ray, first_ray);
        assert_ne!(edited_polyline, first_polyline);
    }

    #[test]
    fn rigid_transform_cache_is_invalidated_by_public_vertex_edits() {
        let mut mesh = Mesh::cube(r(2.0), ());
        let before = mesh
            .rotate(Real::zero(), Real::zero(), Real::zero())
            .bounding_box();

        for polygon in &mut mesh.polygons {
            for vertex in polygon.vertices_mut().iter_mut() {
                vertex.position.x += Real::one();
            }
        }

        let after = mesh
            .rotate(Real::zero(), Real::zero(), Real::zero())
            .bounding_box();
        assert_eq!(after.mins.x, before.mins.x + Real::one());
        assert_eq!(after.maxs.x, before.maxs.x + Real::one());
    }

    #[test]
    fn rigid_transform_cache_preserves_exact_geometry() {
        let mesh = Mesh::sphere(r(2.0), 8, 4, 17_u8);
        let first = mesh.rotate(Real::from(17_u8), Real::from(29_u8), Real::from(43_u8));
        let second = mesh.rotate(Real::from(17_u8), Real::from(29_u8), Real::from(43_u8));

        assert_eq!(first.polygons.len(), second.polygons.len());
        for (left, right) in first.polygons.iter().zip(&second.polygons) {
            assert_eq!(left.metadata, right.metadata);
            assert_eq!(left.plane, right.plane);
            assert_eq!(left.vertices, right.vertices);
        }
    }

    #[test]
    fn general_transform_cache_preserves_exact_geometry_and_current_metadata() {
        let quarter = (Real::one() / Real::from(4_u8)).unwrap();
        let matrix = Matrix4::from_row_major([
            Real::one(),
            quarter,
            Real::zero(),
            Real::from(2_u8),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::from(-3_i8),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::from(4_u8),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]);
        let mesh = Mesh::sphere(r(2.0), 8, 4, 17_u8);
        let first = mesh.transform(&matrix);
        let remapped = mesh.map_metadata(|_| 29_u8).transform(&matrix);

        assert_eq!(first.polygons.len(), remapped.polygons.len());
        for (left, right) in first.polygons.iter().zip(&remapped.polygons) {
            assert_eq!(left.plane, right.plane);
            assert_eq!(left.vertices, right.vertices);
            assert_eq!(left.metadata, 17);
            assert_eq!(right.metadata, 29);
        }
    }

    #[test]
    fn mesh_bounding_box_helpers_do_not_widen_by_tolerance() {
        let container = Aabb::new(Point3::origin(), p3(1.0, 1.0, 1.0));
        let exact_touch = Aabb::new(p3(1.0, 0.25, 0.25), p3(2.0, 0.75, 0.75));
        let just_outside = Aabb::new(
            Point3::new(r(1.0) + tolerance() * r(0.25), r(0.25), r(0.25)),
            p3(2.0, 0.75, 0.75),
        );
        let overhanging = Aabb::new(
            Point3::new(tolerance() * r(-0.25), r(0.25), r(0.25)),
            p3(0.75, 0.75, 0.75),
        );

        let intersecting = |lhs: &Aabb, rhs: &Aabb| {
            let lhs_min = hyperlimit::Point3::new(
                lhs.mins.x.clone(),
                lhs.mins.y.clone(),
                lhs.mins.z.clone(),
            );
            let lhs_max = hyperlimit::Point3::new(
                lhs.maxs.x.clone(),
                lhs.maxs.y.clone(),
                lhs.maxs.z.clone(),
            );
            let rhs_min = hyperlimit::Point3::new(
                rhs.mins.x.clone(),
                rhs.mins.y.clone(),
                rhs.mins.z.clone(),
            );
            let rhs_max = hyperlimit::Point3::new(
                rhs.maxs.x.clone(),
                rhs.maxs.y.clone(),
                rhs.maxs.z.clone(),
            );

            matches!(
                hyperlimit::aabb3s_intersect(&lhs_min, &lhs_max, &rhs_min, &rhs_max).value(),
                Some(true)
            )
        };
        let contains = |container: &Aabb, contained: &Aabb| {
            let container_min = hyperlimit::Point3::new(
                container.mins.x.clone(),
                container.mins.y.clone(),
                container.mins.z.clone(),
            );
            let container_max = hyperlimit::Point3::new(
                container.maxs.x.clone(),
                container.maxs.y.clone(),
                container.maxs.z.clone(),
            );
            let contained_min = hyperlimit::Point3::new(
                contained.mins.x.clone(),
                contained.mins.y.clone(),
                contained.mins.z.clone(),
            );
            let contained_max = hyperlimit::Point3::new(
                contained.maxs.x.clone(),
                contained.maxs.y.clone(),
                contained.maxs.z.clone(),
            );

            let container = hyperlimit::PreparedAabb3::new(&container_min, &container_max);
            matches!(container.contains_point(&contained_min).value(), Some(true))
                && matches!(container.contains_point(&contained_max).value(), Some(true))
        };

        assert!(intersecting(&container, &exact_touch));
        assert!(!intersecting(&container, &just_outside));
        assert!(!contains(&container, &overhanging));
    }

    #[test]
    fn mesh_triangulate_does_not_repair_cross_polygon_t_junctions() {
        let normal = Vector3::z();
        let polygons = vec![
            Polygon::new(
                vec![
                    Vertex::new(p3(0.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(2.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(2.0, 1.0, 0.0), normal.clone()),
                    Vertex::new(p3(0.0, 1.0, 0.0), normal.clone()),
                ],
                (),
            ),
            Polygon::new(
                vec![
                    Vertex::new(p3(1.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(1.5, -0.5, 0.0), normal.clone()),
                    Vertex::new(p3(0.5, -0.5, 0.0), normal),
                ],
                (),
            ),
        ];

        let triangulated = Mesh::from_polygons(polygons).triangulate();

        assert_eq!(triangulated.polygons.len(), 3);
    }

    #[test]
    fn graphics_mesh_streams_the_same_exact_triangle_rows_as_mesh_triangulation() {
        let mesh = Mesh::cube(r(2.0), ());
        let expected = mesh
            .triangulate()
            .polygons
            .into_iter()
            .flat_map(|polygon| polygon.vertices)
            .map(|vertex| {
                (
                    [vertex.position.x, vertex.position.y, vertex.position.z],
                    [
                        vertex.normal.0[0].clone(),
                        vertex.normal.0[1].clone(),
                        vertex.normal.0[2].clone(),
                    ],
                )
            })
            .collect::<Vec<_>>();

        let graphics = mesh.build_graphics_mesh();

        assert_eq!(graphics.vertices, expected);
        assert_eq!(
            graphics.indices,
            (0..u32::try_from(expected.len()).unwrap()).collect::<Vec<_>>()
        );
    }
}
