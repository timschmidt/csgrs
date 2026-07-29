//! CSG grammar over native [`hypermesh::TriangleMesh`] geometry.
//!
//! This module owns modeling vocabulary, not a second mesh carrier. Primitive
//! constructors, transforms, and Boolean composition all accept or return the
//! native Hypermesh type.

use crate::errors::ValidationError;
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hypermesh::{BooleanOp, EmberConfig, HypermeshResult, Plane, Triangle, TriangleMesh};
use hyperreal::RealSign;
use std::cell::RefCell;
use std::num::NonZeroU32;
use std::sync::{Arc, OnceLock};

#[derive(Clone, Debug, PartialEq)]
enum PrimitiveParameters {
    Cuboid(Real, Real, Real),
    Sphere(Real, usize, usize),
    Ellipsoid(Real, Real, Real, usize, usize),
    FrustumBetween(Box<Point3>, Box<Point3>, Real, Real, usize),
    Octahedron(Real),
    Icosahedron(Real),
    #[cfg(feature = "curve")]
    Torus(Real, Real, usize, usize),
}

thread_local! {
    static PRIMITIVE_CACHE: RefCell<Vec<(PrimitiveParameters, TriangleMesh)>> =
        const { RefCell::new(Vec::new()) };
}

fn retained_primitive(
    parameters: PrimitiveParameters,
    build: impl FnOnce() -> TriangleMesh,
) -> TriangleMesh {
    if let Some(mesh) = PRIMITIVE_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|(cached, _)| cached == &parameters)
            .map(|(_, mesh)| mesh.clone())
    }) {
        return mesh;
    }
    let mesh = build();
    PRIMITIVE_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 32;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push((parameters, mesh.clone()));
    });
    mesh
}

#[derive(Clone, Debug, PartialEq)]
enum DistributionParameters {
    Linear {
        count: usize,
        direction: Vector3,
        spacing: Real,
    },
    Grid {
        rows: usize,
        columns: usize,
        dx: Real,
        dy: Real,
    },
    Arc {
        count: usize,
        radius: Real,
        start_angle_degrees: Real,
        end_angle_degrees: Real,
    },
}

#[derive(Clone)]
struct CachedDistribution {
    positions: Arc<[Point3]>,
    triangles: Arc<[Triangle]>,
    parameters: DistributionParameters,
    result: TriangleMesh,
}

type MeshStorage = (Arc<[Point3]>, Arc<[Triangle]>);

#[derive(Clone)]
struct CachedMerge {
    sources: Vec<MeshStorage>,
    result: TriangleMesh,
}

thread_local! {
    static DISTRIBUTION_CACHE: RefCell<Vec<CachedDistribution>> =
        const { RefCell::new(Vec::new()) };
    static MERGE_CACHE: RefCell<Vec<CachedMerge>> = const { RefCell::new(Vec::new()) };
}

#[cfg(feature = "curve")]
#[derive(Clone)]
struct CachedFlatten {
    positions: Arc<[Point3]>,
    triangles: Arc<[Triangle]>,
    region: hypercurve::CurveRegion2,
}

#[cfg(feature = "curve")]
type SliceResult = (
    hypercurve::CurveRegion2,
    Vec<hypercurve::CurveString2>,
    Vec<hypercurve::CurvePath2>,
);

#[cfg(feature = "curve")]
#[derive(Clone)]
struct CachedSlice {
    positions: Arc<[Point3]>,
    triangles: Arc<[Triangle]>,
    z: Real,
    result: SliceResult,
}

#[cfg(feature = "curve")]
thread_local! {
    static FLATTEN_CACHE: RefCell<Vec<CachedFlatten>> = const { RefCell::new(Vec::new()) };
    static SLICE_CACHE: RefCell<Vec<CachedSlice>> = const { RefCell::new(Vec::new()) };
}

fn retained_distribution(
    mesh: &TriangleMesh,
    parameters: &DistributionParameters,
) -> Option<TriangleMesh> {
    DISTRIBUTION_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|entry| {
                Arc::ptr_eq(&entry.positions, &mesh.positions)
                    && Arc::ptr_eq(&entry.triangles, &mesh.triangles)
                    && &entry.parameters == parameters
            })
            .map(|entry| entry.result.clone())
    })
}

fn retain_distribution(
    mesh: &TriangleMesh,
    parameters: DistributionParameters,
    result: &TriangleMesh,
) {
    DISTRIBUTION_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 8;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push(CachedDistribution {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            parameters,
            result: result.clone(),
        });
    });
}

#[cfg(feature = "metaballs")]
pub use crate::implicit::metaballs::{MetaBall, MetaballDiagnostics};
#[cfg(feature = "sdf")]
pub use crate::implicit::sdf::SdfDiagnostics;

fn positive(value: &Real) -> bool {
    matches!(crate::hyper_math::hreal_sign(value), Some(RealSign::Positive))
}

fn nonnegative(value: &Real) -> bool {
    matches!(
        crate::hyper_math::hreal_sign(value),
        Some(RealSign::Positive | RealSign::Zero)
    )
}

fn sampled_circle(count: usize) -> Option<Vec<(Real, Real)>> {
    if count == 0 {
        return None;
    }
    let denominator = Real::from(u64::try_from(count).ok()?);
    (0..count)
        .map(|index| {
            let fraction = (Real::from(u64::try_from(index).ok()?) / &denominator).ok()?;
            let angle = Real::tau() * fraction;
            Some((angle.clone().sin(), angle.cos()))
        })
        .collect::<Option<Vec<_>>>()
}

fn indexed(
    positions: Vec<Point3>,
    triangles: impl IntoIterator<Item = [usize; 3]>,
) -> TriangleMesh {
    TriangleMesh::new(
        positions,
        triangles
            .into_iter()
            .map(|[a, b, c]| Triangle::new(a, b, c))
            .collect(),
    )
}

fn indexed_convex(
    positions: Vec<Point3>,
    triangles: impl IntoIterator<Item = [usize; 3]>,
) -> TriangleMesh {
    indexed(positions, triangles).with_certified_convexity()
}

/// Empty native triangle geometry.
pub fn empty() -> TriangleMesh {
    TriangleMesh::new(Vec::new(), Vec::new())
}

/// Axis-aligned cube with its minimum corner at the origin.
pub fn cube(width: Real) -> TriangleMesh {
    cuboid(width.clone(), width.clone(), width)
}

/// Axis-aligned cuboid with its minimum corner at the origin.
pub fn cuboid(width: Real, length: Real, height: Real) -> TriangleMesh {
    let parameters =
        PrimitiveParameters::Cuboid(width.clone(), length.clone(), height.clone());
    retained_primitive(parameters, || cuboid_uncached(width, length, height))
}

fn cuboid_uncached(width: Real, length: Real, height: Real) -> TriangleMesh {
    if !positive(&width) || !positive(&length) || !positive(&height) {
        return empty();
    }
    let zero = Real::zero();
    let positions = vec![
        Point3::new(zero.clone(), zero.clone(), zero.clone()),
        Point3::new(width.clone(), zero.clone(), zero.clone()),
        Point3::new(width.clone(), length.clone(), zero.clone()),
        Point3::new(zero.clone(), length.clone(), zero),
        Point3::new(Real::zero(), Real::zero(), height.clone()),
        Point3::new(width.clone(), Real::zero(), height.clone()),
        Point3::new(width, length.clone(), height.clone()),
        Point3::new(Real::zero(), length, height),
    ];
    indexed_convex(
        positions,
        [
            [0, 3, 2],
            [0, 2, 1],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ],
    )
}

/// Tessellated sphere centered at the origin.
pub fn sphere(radius: Real, segments: usize, stacks: usize) -> TriangleMesh {
    let parameters = PrimitiveParameters::Sphere(radius.clone(), segments, stacks);
    retained_primitive(parameters, || sphere_uncached(radius, segments, stacks))
}

fn sphere_uncached(radius: Real, segments: usize, stacks: usize) -> TriangleMesh {
    if !positive(&radius) || segments < 3 || stacks < 2 {
        return empty();
    }
    let Some(position_capacity) = segments
        .checked_mul(stacks - 1)
        .and_then(|count| count.checked_add(2))
    else {
        return empty();
    };
    let Some(triangle_capacity) = segments
        .checked_mul(stacks - 1)
        .and_then(|count| count.checked_mul(2))
    else {
        return empty();
    };
    let Some(longitudes) = sampled_circle(segments) else {
        return empty();
    };
    let Some(stack_count) = u64::try_from(stacks).ok().map(Real::from) else {
        return empty();
    };
    let Some(latitudes) = (1..stacks)
        .map(|stack| {
            let fraction = (Real::from(u64::try_from(stack).ok()?) / &stack_count).ok()?;
            let angle = Real::pi() * fraction;
            Some((angle.clone().sin(), angle.cos()))
        })
        .collect::<Option<Vec<_>>>()
    else {
        return empty();
    };

    let mut positions = Vec::with_capacity(position_capacity);
    positions.push(Point3::new(Real::zero(), radius.clone(), Real::zero()));
    for (longitude_sin, longitude_cos) in &longitudes {
        positions.extend(latitudes.iter().map(|(latitude_sin, latitude_cos)| {
            Point3::new(
                radius.clone() * longitude_cos.clone() * latitude_sin.clone(),
                radius.clone() * latitude_cos.clone(),
                radius.clone() * longitude_sin.clone() * latitude_sin.clone(),
            )
        }));
    }
    let south = positions.len();
    positions.push(Point3::new(Real::zero(), -radius, Real::zero()));
    let mut triangles = Vec::with_capacity(triangle_capacity);
    for longitude in 0..segments {
        let next = (longitude + 1) % segments;
        let slot = |longitude: usize, latitude: usize| {
            if latitude == 0 {
                0
            } else if latitude == stacks {
                south
            } else {
                1 + longitude * (stacks - 1) + latitude - 1
            }
        };
        triangles.push([slot(longitude, 0), slot(next, 1), slot(longitude, 1)]);
        for latitude in 1..stacks - 1 {
            triangles.push([
                slot(longitude, latitude),
                slot(next, latitude),
                slot(next, latitude + 1),
            ]);
            triangles.push([
                slot(longitude, latitude),
                slot(next, latitude + 1),
                slot(longitude, latitude + 1),
            ]);
        }
        triangles.push([
            slot(longitude, stacks - 1),
            slot(next, stacks - 1),
            slot(longitude, stacks),
        ]);
    }
    indexed_convex(positions, triangles)
}

/// Tessellated cylinder along +Z.
pub fn cylinder(radius: Real, height: Real, segments: usize) -> TriangleMesh {
    frustum(radius.clone(), radius, height, segments)
}

/// Tessellated ellipsoid centered at the origin.
pub fn ellipsoid(
    radius_x: Real,
    radius_y: Real,
    radius_z: Real,
    segments: usize,
    stacks: usize,
) -> TriangleMesh {
    let parameters = PrimitiveParameters::Ellipsoid(
        radius_x.clone(),
        radius_y.clone(),
        radius_z.clone(),
        segments,
        stacks,
    );
    retained_primitive(parameters, || {
        ellipsoid_uncached(radius_x, radius_y, radius_z, segments, stacks)
    })
}

fn ellipsoid_uncached(
    radius_x: Real,
    radius_y: Real,
    radius_z: Real,
    segments: usize,
    stacks: usize,
) -> TriangleMesh {
    if !positive(&radius_x) || !positive(&radius_y) || !positive(&radius_z) {
        return empty();
    }
    scale(
        &sphere(Real::one(), segments, stacks),
        radius_x,
        radius_y,
        radius_z,
    )
}

/// Tessellated conical frustum along +Z.
pub fn frustum(
    radius_bottom: Real,
    radius_top: Real,
    height: Real,
    segments: usize,
) -> TriangleMesh {
    frustum_between(
        Point3::origin(),
        Point3::new(Real::zero(), Real::zero(), height),
        radius_bottom,
        radius_top,
        segments,
    )
}

/// Tessellated conical frustum between two exact points.
pub fn frustum_between(
    start: Point3,
    end: Point3,
    start_radius: Real,
    end_radius: Real,
    segments: usize,
) -> TriangleMesh {
    let parameters = PrimitiveParameters::FrustumBetween(
        Box::new(start.clone()),
        Box::new(end.clone()),
        start_radius.clone(),
        end_radius.clone(),
        segments,
    );
    retained_primitive(parameters, || {
        frustum_between_uncached(start, end, start_radius, end_radius, segments)
    })
}

fn frustum_between_uncached(
    start: Point3,
    end: Point3,
    start_radius: Real,
    end_radius: Real,
    segments: usize,
) -> TriangleMesh {
    if segments < 3 || !nonnegative(&start_radius) || !nonnegative(&end_radius) {
        return empty();
    }
    let Some(position_capacity) =
        segments.checked_mul(2).and_then(|count| count.checked_add(2))
    else {
        return empty();
    };
    let Some(triangle_capacity) = segments.checked_mul(4) else {
        return empty();
    };
    let direction = &end - &start;
    let Ok(axis) = direction.normalize_checked() else {
        return empty();
    };
    if !positive(&direction.dot(&direction)) {
        return empty();
    }
    let Ok((axis_x, axis_y)) = axis.orthonormal_basis_checked() else {
        return empty();
    };
    let Some(samples) = sampled_circle(segments) else {
        return empty();
    };
    let bottom_degenerate = !positive(&start_radius);
    let top_degenerate = !positive(&end_radius);
    if bottom_degenerate && top_degenerate {
        return empty();
    }
    let mut positions = Vec::with_capacity(position_capacity);
    let bottom_center = positions.len();
    positions.push(start.clone());
    let top_center = positions.len();
    positions.push(end.clone());
    let bottom_base = positions.len();
    if !bottom_degenerate {
        positions.extend(samples.iter().map(|(sin, cos)| {
            start.clone()
                + axis_x.clone() * (start_radius.clone() * cos.clone())
                + axis_y.clone() * (start_radius.clone() * sin.clone())
        }));
    }
    let top_base = positions.len();
    if !top_degenerate {
        positions.extend(samples.iter().map(|(sin, cos)| {
            end.clone()
                + axis_x.clone() * (end_radius.clone() * cos.clone())
                + axis_y.clone() * (end_radius.clone() * sin.clone())
        }));
    }
    let mut triangles = Vec::with_capacity(triangle_capacity);
    for index in 0..segments {
        let next = (index + 1) % segments;
        if !bottom_degenerate {
            triangles.push([bottom_center, bottom_base + next, bottom_base + index]);
        }
        if !top_degenerate {
            triangles.push([top_center, top_base + index, top_base + next]);
        }
        match (bottom_degenerate, top_degenerate) {
            (true, false) => {
                triangles.push([bottom_center, top_base + next, top_base + index])
            },
            (false, true) => {
                triangles.push([bottom_base + index, bottom_base + next, top_center])
            },
            (false, false) => {
                triangles.push([bottom_base + index, bottom_base + next, top_base + next]);
                triangles.push([bottom_base + index, top_base + next, top_base + index]);
            },
            (true, true) => {},
        }
    }
    indexed_convex(positions, triangles)
}

/// Regular octahedron centered at the origin.
pub fn octahedron(radius: Real) -> TriangleMesh {
    let parameters = PrimitiveParameters::Octahedron(radius.clone());
    retained_primitive(parameters, || octahedron_uncached(radius))
}

fn octahedron_uncached(radius: Real) -> TriangleMesh {
    if !positive(&radius) {
        return empty();
    }
    let negative = -radius.clone();
    indexed_convex(
        vec![
            Point3::new(radius.clone(), Real::zero(), Real::zero()),
            Point3::new(negative.clone(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), radius.clone(), Real::zero()),
            Point3::new(Real::zero(), negative.clone(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), radius),
            Point3::new(Real::zero(), Real::zero(), negative),
        ],
        [
            [4, 0, 2],
            [4, 2, 1],
            [4, 1, 3],
            [4, 3, 0],
            [5, 2, 0],
            [5, 1, 2],
            [5, 3, 1],
            [5, 0, 3],
        ],
    )
}

/// Regular icosahedron centered at the origin.
pub fn icosahedron(radius: Real) -> TriangleMesh {
    let parameters = PrimitiveParameters::Icosahedron(radius.clone());
    retained_primitive(parameters, || icosahedron_uncached(radius))
}

fn icosahedron_uncached(radius: Real) -> TriangleMesh {
    if !positive(&radius) {
        return empty();
    }
    let phi =
        (Real::one() + Real::from(5_u8).sqrt().expect("five is positive")) / Real::from(2_u8);
    let Ok(phi) = phi else {
        return empty();
    };
    let raw = [
        [-Real::one(), phi.clone(), Real::zero()],
        [Real::one(), phi.clone(), Real::zero()],
        [-Real::one(), -phi.clone(), Real::zero()],
        [Real::one(), -phi.clone(), Real::zero()],
        [Real::zero(), -Real::one(), phi.clone()],
        [Real::zero(), Real::one(), phi.clone()],
        [Real::zero(), -Real::one(), -phi.clone()],
        [Real::zero(), Real::one(), -phi.clone()],
        [phi.clone(), Real::zero(), -Real::one()],
        [phi.clone(), Real::zero(), Real::one()],
        [-phi.clone(), Real::zero(), -Real::one()],
        [-phi, Real::zero(), Real::one()],
    ];
    let positions = raw
        .into_iter()
        .map(|[x, y, z]| {
            let vector = Vector3::from_xyz(x, y, z);
            let unit = vector
                .normalize_checked()
                .expect("canonical icosahedron vertices are nonzero");
            Point3::origin() + unit * radius.clone()
        })
        .collect();
    indexed_convex(
        positions,
        [
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ],
    )
}

/// Tessellated arrow from a start point and direction vector.
pub fn arrow(
    start: Point3,
    direction: Vector3,
    segments: usize,
    mirrored: bool,
) -> TriangleMesh {
    if segments < 3 {
        return empty();
    }
    let Some(position_capacity) =
        segments.checked_mul(3).and_then(|count| count.checked_add(2))
    else {
        return empty();
    };
    let Some(triangle_capacity) = segments.checked_mul(7) else {
        return empty();
    };
    let Ok(axis) = direction.normalize_checked() else {
        return empty();
    };
    let Ok((axis_x, axis_y)) = axis.orthonormal_basis_checked() else {
        return empty();
    };
    let Some(length) = direction.dot(&direction).sqrt().ok() else {
        return empty();
    };
    if !positive(&length) {
        return empty();
    }
    let ratio = |numerator: u8, denominator: u8| {
        (Real::from(numerator) / Real::from(denominator))
            .expect("arrow ratio denominator is nonzero")
    };
    let shaft_length = length.clone() * ratio(4, 5);
    let head_length = length.clone() - shaft_length.clone();
    let shaft_radius = length.clone() * ratio(3, 100);
    let head_radius = length * ratio(3, 50);
    let end = start.clone() + direction;
    let joint = if mirrored {
        start.clone() + axis.clone() * head_length
    } else {
        start.clone() + axis.clone() * shaft_length
    };
    let (shaft_start, shaft_end, tip, shoulder_faces_positive) = if mirrored {
        (joint.clone(), end, start, true)
    } else {
        (start, joint.clone(), end, false)
    };
    let Some(samples) = sampled_circle(segments) else {
        return empty();
    };
    let ring = |center: &Point3, radius: &Real| {
        samples
            .iter()
            .map(|(sin, cos)| {
                center.clone()
                    + axis_x.clone() * (radius.clone() * cos.clone())
                    + axis_y.clone() * (radius.clone() * sin.clone())
            })
            .collect::<Vec<_>>()
    };
    let mut positions = Vec::with_capacity(position_capacity);
    let shaft_start_center = positions.len();
    positions.push(shaft_start.clone());
    let tip_index = positions.len();
    positions.push(tip);
    let shaft_start_ring = positions.len();
    positions.extend(ring(&shaft_start, &shaft_radius));
    let shaft_joint_ring = positions.len();
    positions.extend(ring(&shaft_end, &shaft_radius));
    let head_joint_ring = positions.len();
    positions.extend(ring(&joint, &head_radius));
    let mut triangles = Vec::with_capacity(triangle_capacity);
    for index in 0..segments {
        let next = (index + 1) % segments;
        if mirrored {
            triangles.push([
                shaft_start_ring + index,
                shaft_start_ring + next,
                shaft_start_center,
            ]);
        } else {
            triangles.push([
                shaft_start_center,
                shaft_start_ring + next,
                shaft_start_ring + index,
            ]);
        }
        triangles.push([
            shaft_start_ring + index,
            shaft_start_ring + next,
            shaft_joint_ring + next,
        ]);
        triangles.push([
            shaft_start_ring + index,
            shaft_joint_ring + next,
            shaft_joint_ring + index,
        ]);
        let mut shoulder_a = [
            shaft_joint_ring + index,
            head_joint_ring + next,
            head_joint_ring + index,
        ];
        let mut shoulder_b = [
            shaft_joint_ring + index,
            shaft_joint_ring + next,
            head_joint_ring + next,
        ];
        if shoulder_faces_positive {
            shoulder_a.swap(1, 2);
            shoulder_b.swap(1, 2);
        }
        triangles.push(shoulder_a);
        triangles.push(shoulder_b);
        if mirrored {
            triangles.push([tip_index, head_joint_ring + index, head_joint_ring + next]);
        } else {
            triangles.push([head_joint_ring + index, tip_index, head_joint_ring + next]);
        }
    }
    let interior = shaft_start.clone() + (shaft_end - shaft_start) * ratio(1, 2);
    for triangle in &mut triangles {
        let [a, b, c] = triangle.map(|index| &positions[index]);
        let normal = (b - a).cross(&(c - a));
        let outward = a.to_vector() + b.to_vector() + c.to_vector()
            - interior.to_vector() * Real::from(3_u8);
        let orientation = normal.dot(&outward);
        let faces_inward = match crate::hyper_math::hreal_sign(&orientation) {
            Some(RealSign::Negative) => true,
            Some(RealSign::Positive | RealSign::Zero) => false,
            None => orientation.to_f64_lossy().is_some_and(|value| value < 0.0),
        };
        if faces_inward {
            triangle.swap(1, 2);
        }
    }
    let mut edge_uses = std::collections::HashMap::<(usize, usize), Vec<(usize, bool)>>::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        for (a, b) in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ] {
            edge_uses
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push((triangle_index, a < b));
        }
    }
    let mut adjacency = vec![Vec::<(usize, bool)>::new(); triangles.len()];
    for uses in edge_uses.values() {
        if let [(first, first_direction), (second, second_direction)] = uses.as_slice() {
            let toggle = first_direction == second_direction;
            adjacency[*first].push((*second, toggle));
            adjacency[*second].push((*first, toggle));
        }
    }
    let mut flips = vec![None; triangles.len()];
    for seed in 0..triangles.len() {
        if flips[seed].is_some() {
            continue;
        }
        flips[seed] = Some(false);
        let mut queue = std::collections::VecDeque::from([seed]);
        while let Some(current) = queue.pop_front() {
            for &(neighbor, toggle) in &adjacency[current] {
                let required = flips[current].expect("queued faces are oriented") ^ toggle;
                if flips[neighbor].is_none() {
                    flips[neighbor] = Some(required);
                    queue.push_back(neighbor);
                }
            }
        }
    }
    for (triangle, flip) in triangles.iter_mut().zip(flips) {
        if flip == Some(true) {
            triangle.swap(1, 2);
        }
    }
    let orientation_score = triangles
        .iter()
        .filter_map(|triangle| {
            let [a, b, c] = triangle.map(|index| &positions[index]);
            let normal = (b - a).cross(&(c - a));
            let outward = a.to_vector() + b.to_vector() + c.to_vector()
                - interior.to_vector() * Real::from(3_u8);
            normal.dot(&outward).to_f64_lossy()
        })
        .sum::<f64>();
    if orientation_score < 0.0 {
        for triangle in &mut triangles {
            triangle.swap(1, 2);
        }
    }
    indexed(positions, triangles)
}

/// Tessellated torus around the Z axis.
#[cfg(feature = "curve")]
pub fn torus(
    major_radius: Real,
    minor_radius: Real,
    major_segments: usize,
    minor_segments: usize,
) -> TriangleMesh {
    let parameters = PrimitiveParameters::Torus(
        major_radius.clone(),
        minor_radius.clone(),
        major_segments,
        minor_segments,
    );
    retained_primitive(parameters, || {
        torus_uncached(major_radius, minor_radius, major_segments, minor_segments)
    })
}

#[cfg(feature = "curve")]
fn torus_uncached(
    major_radius: Real,
    minor_radius: Real,
    major_segments: usize,
    minor_segments: usize,
) -> TriangleMesh {
    if !positive(&major_radius)
        || !positive(&minor_radius)
        || major_radius <= minor_radius
        || major_segments < 3
        || minor_segments < 3
    {
        return empty();
    }
    let Some(position_capacity) = major_segments.checked_mul(minor_segments) else {
        return empty();
    };
    let Some(triangle_capacity) = position_capacity.checked_mul(2) else {
        return empty();
    };
    let (Some(major), Some(minor)) =
        (sampled_circle(major_segments), sampled_circle(minor_segments))
    else {
        return empty();
    };
    let mut positions = Vec::with_capacity(position_capacity);
    for (major_sin, major_cos) in &major {
        for (minor_sin, minor_cos) in &minor {
            let radial = major_radius.clone() + minor_radius.clone() * minor_cos.clone();
            positions.push(Point3::new(
                radial.clone() * major_cos.clone(),
                radial * major_sin.clone(),
                minor_radius.clone() * minor_sin.clone(),
            ));
        }
    }
    let mut triangles = Vec::with_capacity(triangle_capacity);
    for major_index in 0..major_segments {
        let next_major = (major_index + 1) % major_segments;
        for minor_index in 0..minor_segments {
            let next_minor = (minor_index + 1) % minor_segments;
            let a = major_index * minor_segments + minor_index;
            let b = next_major * minor_segments + minor_index;
            let c = next_major * minor_segments + next_minor;
            let d = major_index * minor_segments + next_minor;
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    indexed(positions, triangles)
}

/// Extruded teardrop solid.
#[cfg(feature = "curve")]
pub fn teardrop_cylinder(
    width: Real,
    length: Real,
    height: Real,
    shape_segments: usize,
) -> TriangleMesh {
    crate::curve::extrude(&crate::curve::teardrop(width, length, shape_segments), height)
}

/// Extruded involute spur gear.
#[cfg(feature = "curve")]
pub fn spur_gear_involute(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    segments_per_flank: usize,
    thickness: Real,
) -> TriangleMesh {
    crate::curve::extrude(
        &crate::curve::involute_gear(
            module,
            teeth,
            pressure_angle_degrees,
            clearance,
            backlash,
            segments_per_flank,
        ),
        thickness,
    )
}

/// Extruded cycloidal spur gear.
#[cfg(feature = "curve")]
pub fn spur_gear_cycloid(
    module: Real,
    teeth: usize,
    generating_radius: Real,
    clearance: Real,
    segments_per_flank: usize,
    thickness: Real,
) -> TriangleMesh {
    crate::curve::extrude(
        &crate::curve::cycloidal_gear(
            module,
            teeth,
            generating_radius,
            clearance,
            segments_per_flank,
        ),
        thickness,
    )
}

/// Twisted extrusion of an involute gear region.
#[cfg(feature = "curve")]
#[allow(clippy::too_many_arguments)]
pub fn helical_involute_gear(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    segments_per_flank: usize,
    thickness: Real,
    helix_angle_degrees: Real,
    slices: usize,
) -> TriangleMesh {
    let region = crate::curve::involute_gear(
        module,
        teeth,
        pressure_angle_degrees,
        clearance,
        backlash,
        segments_per_flank,
    );
    crate::curve::extrude_twisted(
        &region,
        thickness,
        helix_angle_degrees,
        [Real::one(), Real::one()],
        slices,
    )
    .unwrap_or_else(|_| empty())
}

/// Builds native triangle geometry from polygon index rows.
pub fn polyhedron(
    points: &[[Real; 3]],
    faces: &[&[usize]],
) -> Result<TriangleMesh, ValidationError> {
    let positions = points
        .iter()
        .map(|[x, y, z]| Point3::new(x.clone(), y.clone(), z.clone()))
        .collect::<Vec<_>>();
    let mut triangles = Vec::new();
    for face in faces {
        if face.len() < 3
            || face
                .iter()
                .enumerate()
                .any(|(index, value)| face[index + 1..].contains(value))
        {
            return Err(ValidationError::InvalidArguments);
        }
        for &index in *face {
            if index >= positions.len() {
                return Err(ValidationError::IndexOutOfRangeWithLen {
                    index,
                    len: positions.len(),
                });
            }
        }
        let origin = &positions[face[0]];
        let support = (1..face.len() - 1).find_map(|left| {
            (left + 1..face.len()).find_map(|right| {
                let x = &positions[face[left]] - origin;
                let y = &positions[face[right]] - origin;
                let support_normal = x.cross(&y);
                support_normal
                    .normalize_checked()
                    .ok()
                    .map(|normal| (x, support_normal, normal))
            })
        });
        let Some((axis_x, support_normal, normal)) = support else {
            return Err(ValidationError::InvalidArguments);
        };
        for &index in &face[1..] {
            let offset = &positions[index] - origin;
            if !matches!(
                crate::hyper_math::hreal_sign(&support_normal.dot(&offset)),
                Some(RealSign::Zero)
            ) {
                return Err(ValidationError::InvalidArguments);
            }
        }
        let axis_x = axis_x
            .normalize_checked()
            .map_err(|_| ValidationError::InvalidArguments)?;
        let axis_y = normal.cross(&axis_x);
        let projected = face
            .iter()
            .map(|&index| {
                let offset = &positions[index] - origin;
                hypertri::Point2::new(axis_x.dot(&offset), axis_y.dot(&offset))
            })
            .collect::<Vec<_>>();
        let indices = hypertri::earcut(&projected, &[])
            .map_err(|error| ValidationError::Geometry(error.to_string()))?;
        triangles.extend(indices.chunks_exact(3).map(|triangle| {
            Triangle::new(face[triangle[0]], face[triangle[1]], face[triangle[2]])
        }));
    }
    Ok(TriangleMesh::new(positions, triangles))
}

/// Computes one exact regularized Boolean and returns reusable native geometry.
pub fn boolean(
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
) -> HypermeshResult<TriangleMesh> {
    hypermesh::boolean_triangle_meshes(left, right, operation, EmberConfig::default())
}

/// Applies a homogeneous transform and returns native geometry.
pub fn transform(mesh: &TriangleMesh, matrix: &Matrix4) -> TriangleMesh {
    let Some(orientation) = crate::hyper_math::hreal_sign(&matrix.determinant()) else {
        return empty();
    };
    let Some(transformed) = mesh.try_transformed(matrix) else {
        return empty();
    };
    match orientation {
        RealSign::Positive => transformed,
        RealSign::Negative => transformed.reversed_winding(),
        RealSign::Zero => empty(),
    }
}

/// Rotates native geometry by Euler angles in degrees.
pub fn rotate(mesh: &TriangleMesh, x: Real, y: Real, z: Real) -> TriangleMesh {
    mesh.rotated_xyz_degrees(x, y, z)
}

/// Scales native geometry independently along each axis.
pub fn scale(mesh: &TriangleMesh, x: Real, y: Real, z: Real) -> TriangleMesh {
    transform(mesh, &Matrix4::affine_nonuniform_scale([x, y, z]))
}

/// Reflects native geometry across a plane.
pub fn mirror(mesh: &TriangleMesh, plane: &Plane) -> TriangleMesh {
    plane
        .reflection_matrix()
        .map_or_else(|_| mesh.clone(), |matrix| transform(mesh, &matrix))
}

/// Uniformly subdivides every native triangle.
pub fn subdivide(mesh: &TriangleMesh, levels: NonZeroU32) -> TriangleMesh {
    mesh.subdivide_triangles(levels)
}

/// Rebuilds exact coordinates into their normalized representation.
pub fn renormalized(mesh: &TriangleMesh) -> TriangleMesh {
    // Native geometry does not store authored face normals or duplicated
    // polygon planes. Hyperreal coordinates normalize themselves, so there is
    // no mesh-side state to rebuild.
    mesh.clone()
}

/// Materializes every coordinate as finite exact geometry.
///
/// Returns `None` if any coordinate cannot be represented as a finite value.
pub fn materialize_finite(mesh: &TriangleMesh) -> Option<TriangleMesh> {
    mesh.materialize_finite()
}

/// Checks indexed edge pairing for a closed, consistently oriented manifold.
pub fn is_closed_manifold(mesh: &TriangleMesh) -> bool {
    mesh.is_closed_manifold()
}

/// Inverts triangle winding.
pub fn inverse(mesh: &TriangleMesh) -> TriangleMesh {
    mesh.reversed_winding()
}

/// Samples a native metaball surface.
#[cfg(feature = "metaballs")]
pub fn metaballs(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
) -> TriangleMesh {
    crate::implicit::metaballs::metaballs(balls, resolution, iso_value, padding)
}

/// Samples a native metaball surface and returns boundary diagnostics.
#[cfg(feature = "metaballs")]
pub fn metaballs_with_diagnostics(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
) -> (TriangleMesh, MetaballDiagnostics) {
    crate::implicit::metaballs::metaballs_with_diagnostics(
        balls, resolution, iso_value, padding,
    )
}

/// Samples an exact scalar field into reusable native triangle geometry.
#[cfg(feature = "sdf")]
pub fn sdf(
    field: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> TriangleMesh {
    crate::implicit::sdf::sdf(field, resolution, min, max, iso_value)
}

/// Samples an SDF into native triangle geometry and returns sampling diagnostics.
#[cfg(feature = "sdf")]
pub fn sdf_with_diagnostics(
    field: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> (TriangleMesh, SdfDiagnostics) {
    crate::implicit::sdf::sdf_with_diagnostics(field, resolution, min, max, iso_value)
}

/// Samples a Hypersdf expression into reusable native triangle geometry.
#[cfg(feature = "sdf")]
pub fn sdf_expr(
    expression: hypersdf::SdfExpr,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> TriangleMesh {
    crate::implicit::sdf::sdf_expr(expression, resolution, min, max, iso_value)
}

/// Samples a retained SDF expression and returns sampling diagnostics.
#[cfg(feature = "sdf")]
pub fn sdf_expr_with_diagnostics(
    expression: hypersdf::SdfExpr,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> (TriangleMesh, SdfDiagnostics) {
    crate::implicit::sdf::sdf_expr_with_diagnostics(
        expression, resolution, min, max, iso_value,
    )
}

/// Bounded gyroid solid sampled through the established SDF boundary.
#[cfg(feature = "sdf")]
pub fn gyroid_solid(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
    wall_thickness: Real,
) -> TriangleMesh {
    crate::implicit::tpms::gyroid_solid(
        bounding_box(bounds),
        resolution,
        scale,
        iso_value,
        wall_thickness,
    )
}

/// Bounded Schwarz-P solid sampled through the established SDF boundary.
#[cfg(feature = "sdf")]
pub fn schwarz_p_solid(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
    wall_thickness: Real,
) -> TriangleMesh {
    crate::implicit::tpms::schwarz_p_solid(
        bounding_box(bounds),
        resolution,
        scale,
        iso_value,
        wall_thickness,
    )
}

/// Bounded Schwarz-D solid sampled through the established SDF boundary.
#[cfg(feature = "sdf")]
pub fn schwarz_d_solid(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
    wall_thickness: Real,
) -> TriangleMesh {
    crate::implicit::tpms::schwarz_d_solid(
        bounding_box(bounds),
        resolution,
        scale,
        iso_value,
        wall_thickness,
    )
}

/// Samples a bounded gyroid level surface.
#[cfg(feature = "sdf")]
pub fn gyroid(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
) -> TriangleMesh {
    crate::implicit::tpms::gyroid(bounding_box(bounds), resolution, scale, iso_value)
}

/// Samples a bounded Schwarz-P level surface.
#[cfg(feature = "sdf")]
pub fn schwarz_p(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
) -> TriangleMesh {
    crate::implicit::tpms::schwarz_p(bounding_box(bounds), resolution, scale, iso_value)
}

/// Samples a bounded Schwarz-D level surface.
#[cfg(feature = "sdf")]
pub fn schwarz_d(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
) -> TriangleMesh {
    crate::implicit::tpms::schwarz_d(bounding_box(bounds), resolution, scale, iso_value)
}

/// Exact bounds of native triangle positions.
pub fn bounding_box(mesh: &TriangleMesh) -> &Aabb {
    static EMPTY_BOUNDS: OnceLock<Aabb> = OnceLock::new();
    mesh.exact_bounds()
        .unwrap_or_else(|| EMPTY_BOUNDS.get_or_init(Aabb::origin))
}

/// Translates the center of the exact bounds to the origin.
pub fn center(mesh: &TriangleMesh) -> TriangleMesh {
    let bounds = bounding_box(mesh);
    let two = Real::from(2_u8);
    let x = -((&bounds.mins.x + &bounds.maxs.x) / &two).expect("two is nonzero");
    let y = -((&bounds.mins.y + &bounds.maxs.y) / &two).expect("two is nonzero");
    let z = -((&bounds.mins.z + &bounds.maxs.z) / &two).expect("two is nonzero");
    mesh.translated(x, y, z)
}

/// Translates the minimum Z bound to zero.
pub fn float(mesh: &TriangleMesh) -> TriangleMesh {
    let bounds = bounding_box(mesh);
    mesh.translated(Real::zero(), Real::zero(), -bounds.mins.z.clone())
}

/// Concatenates native meshes without performing a Boolean operation.
pub fn merge(meshes: &[TriangleMesh]) -> TriangleMesh {
    if let Some(result) = MERGE_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|entry| {
                entry.sources.len() == meshes.len()
                    && entry.sources.iter().zip(meshes).all(
                        |((positions, triangles), mesh)| {
                            Arc::ptr_eq(positions, &mesh.positions)
                                && Arc::ptr_eq(triangles, &mesh.triangles)
                        },
                    )
            })
            .map(|entry| entry.result.clone())
    }) {
        return result;
    }
    let position_count = meshes.iter().map(|mesh| mesh.positions.len()).sum();
    let triangle_count = meshes.iter().map(|mesh| mesh.triangles.len()).sum();
    let mut positions = Vec::with_capacity(position_count);
    let mut triangles = Vec::with_capacity(triangle_count);
    for mesh in meshes {
        let base = positions.len();
        positions.extend(mesh.positions.iter().cloned());
        triangles.extend(mesh.triangles.iter().map(|triangle| {
            hypermesh::Triangle::new(
                base + triangle.v0,
                base + triangle.v1,
                base + triangle.v2,
            )
        }));
    }
    let result = TriangleMesh::new(positions, triangles);
    MERGE_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 8;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push(CachedMerge {
            sources: meshes
                .iter()
                .map(|mesh| (Arc::clone(&mesh.positions), Arc::clone(&mesh.triangles)))
                .collect(),
            result: result.clone(),
        });
    });
    result
}

fn union_copies(copies: Vec<TriangleMesh>) -> TriangleMesh {
    let pairwise_disjoint = copies
        .iter()
        .map(TriangleMesh::exact_bounds)
        .collect::<Option<Vec<_>>>()
        .is_some_and(|bounds| {
            bounds.iter().enumerate().all(|(left_index, left)| {
                bounds.iter().skip(left_index + 1).all(|right| {
                    matches!(
                        hyperlimit::ordered_aabb3s_intersect(
                            &left.mins,
                            &left.maxs,
                            &right.mins,
                            &right.maxs,
                        )
                        .value(),
                        Some(false)
                    )
                })
            })
        });
    if pairwise_disjoint {
        return merge(&copies);
    }
    copies
        .into_iter()
        .reduce(|left, right| {
            boolean(&left, &right, BooleanOp::Union).unwrap_or_else(|_| merge(&[left, right]))
        })
        .unwrap_or_else(empty)
}

/// Places `count` copies along a direction at exact spacing.
pub fn distribute_linear(
    mesh: &TriangleMesh,
    count: usize,
    direction: Vector3,
    spacing: Real,
) -> TriangleMesh {
    if count == 0 {
        return mesh.clone();
    }
    let parameters = DistributionParameters::Linear {
        count,
        direction: direction.clone(),
        spacing: spacing.clone(),
    };
    if let Some(result) = retained_distribution(mesh, &parameters) {
        return result;
    }
    let Ok(direction) = direction.normalize_checked() else {
        return mesh.clone();
    };
    let step = direction * spacing;
    let result = union_copies(
        (0..count)
            .map(|index| {
                let offset = step.clone() * Real::from(index as u64);
                mesh.translated(offset.0[0].clone(), offset.0[1].clone(), offset.0[2].clone())
            })
            .collect(),
    );
    retain_distribution(mesh, parameters, &result);
    result
}

/// Places copies in an XY grid.
pub fn distribute_grid(
    mesh: &TriangleMesh,
    rows: usize,
    columns: usize,
    dx: Real,
    dy: Real,
) -> TriangleMesh {
    if rows == 0 || columns == 0 {
        return mesh.clone();
    }
    let parameters = DistributionParameters::Grid {
        rows,
        columns,
        dx: dx.clone(),
        dy: dy.clone(),
    };
    if let Some(result) = retained_distribution(mesh, &parameters) {
        return result;
    }
    let mut copies = Vec::with_capacity(rows.saturating_mul(columns));
    for row in 0..rows {
        for column in 0..columns {
            copies.push(mesh.translated(
                dx.clone() * Real::from(column as u64),
                dy.clone() * Real::from(row as u64),
                Real::zero(),
            ));
        }
    }
    let result = union_copies(copies);
    retain_distribution(mesh, parameters, &result);
    result
}

/// Places copies along a circular arc in the XY plane.
pub fn distribute_arc(
    mesh: &TriangleMesh,
    count: usize,
    radius: Real,
    start_angle_degrees: Real,
    end_angle_degrees: Real,
) -> TriangleMesh {
    if count == 0 {
        return mesh.clone();
    }
    let parameters = DistributionParameters::Arc {
        count,
        radius: radius.clone(),
        start_angle_degrees: start_angle_degrees.clone(),
        end_angle_degrees: end_angle_degrees.clone(),
    };
    if let Some(result) = retained_distribution(mesh, &parameters) {
        return result;
    }
    let start = start_angle_degrees.to_radians();
    let sweep = end_angle_degrees.to_radians() - start.clone();
    let result = union_copies(
        (0..count)
            .map(|index| {
                let fraction = if count == 1 {
                    (Real::one() / Real::from(2_u8)).expect("two is nonzero")
                } else {
                    (Real::from(index as u64) / Real::from((count - 1) as u64))
                        .expect("count minus one is nonzero")
                };
                let angle = start.clone() + sweep.clone() * fraction;
                let translated = mesh.translated(radius.clone(), Real::zero(), Real::zero());
                transform(&translated, &Matrix4::rotation_z(angle))
            })
            .collect(),
    );
    retain_distribution(mesh, parameters, &result);
    result
}

/// Exact point containment through the retained native triangle query path.
pub fn contains_point(
    mesh: &TriangleMesh,
    point: &Point3,
) -> hypermesh::HypermeshResult<bool> {
    mesh.contains_point(point)
}

/// Exact ray intersections sorted by ray parameter.
pub fn ray_intersections(
    mesh: &TriangleMesh,
    origin: &Point3,
    direction: &Vector3,
) -> hypermesh::HypermeshResult<Vec<(Point3, Real)>> {
    mesh.ray_intersections(origin, direction)
}

/// Exact intersections between native triangle geometry and a polyline.
pub fn polyline_intersections(
    mesh: &TriangleMesh,
    polyline: &[Point3],
) -> hypermesh::HypermeshResult<Vec<Point3>> {
    mesh.polyline_intersections(polyline)
}

/// Exact dihedral angle between two indexed triangles.
///
/// The angle is measured between the triangles' oriented plane normals.
pub fn dihedral_angle(
    mesh: &TriangleMesh,
    first: hypermesh::Triangle,
    second: hypermesh::Triangle,
) -> Option<Real> {
    mesh.dihedral_angle(first, second)
}

/// Exact uniform-density mass properties through Hyperphysics.
pub fn exact_mass_properties(
    mesh: &TriangleMesh,
    density: Real,
) -> Result<hyperphysics::MassPropertyReport3, ValidationError> {
    hyperphysics::triangle_mesh_uniform_density_mass_properties(mesh, density)
        .map_err(|error| ValidationError::Geometry(format!("{error:?}")))
}

/// Flattens native triangle faces into native filled curve topology.
#[cfg(feature = "curve")]
pub fn flatten(mesh: &TriangleMesh) -> hypercurve::CurveRegion2 {
    use hypercurve::{BooleanOp as CurveBooleanOp, Contour2, CurvePolicy, CurveRegion2};

    if let Some(region) = FLATTEN_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|entry| {
                Arc::ptr_eq(&entry.positions, &mesh.positions)
                    && Arc::ptr_eq(&entry.triangles, &mesh.triangles)
            })
            .map(|entry| entry.region.clone())
    }) {
        return region;
    }
    let policy = CurvePolicy::certified();
    let mut output = CurveRegion2::empty();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        let (Some(a), Some(b), Some(c)) = (
            mesh.positions.get(a),
            mesh.positions.get(b),
            mesh.positions.get(c),
        ) else {
            continue;
        };
        let mut points = [
            [a.x.clone(), a.y.clone()],
            [b.x.clone(), b.y.clone()],
            [c.x.clone(), c.y.clone()],
        ];
        let limit = points
            .iter()
            .map(|[x, y]| hyperlimit::Point2::new(x.clone(), y.clone()))
            .collect::<Vec<_>>();
        match hyperlimit::ring_area_sign(&limit).value() {
            Some(hyperlimit::Sign::Negative) => points.swap(1, 2),
            Some(hyperlimit::Sign::Positive) => {},
            Some(hyperlimit::Sign::Zero) | None => continue,
        }
        let Ok(contour) = Contour2::from_real_ring(&points) else {
            continue;
        };
        let Ok(region) =
            CurveRegion2::try_from_native_material_contours(vec![contour], &policy)
        else {
            continue;
        };
        output = if output.is_empty() {
            region
        } else {
            output
                .boolean_region(&region, CurveBooleanOp::Union, &policy)
                .unwrap_or(output)
        };
    }
    FLATTEN_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 8;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push(CachedFlatten {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            region: output.clone(),
        });
    });
    output
}

/// Intersects native geometry with the XY plane at an exact Z coordinate.
#[cfg(feature = "curve")]
pub fn slice_z(mesh: &TriangleMesh, z: Real) -> SliceResult {
    use hypercurve::{
        BooleanOp as CurveBooleanOp, Contour2, CurvePolicy, CurveRegion2, CurveString2,
    };

    if let Some(result) = SLICE_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|entry| {
                Arc::ptr_eq(&entry.positions, &mesh.positions)
                    && Arc::ptr_eq(&entry.triangles, &mesh.triangles)
                    && entry.z == z
            })
            .map(|entry| entry.result.clone())
    }) {
        return result;
    }
    let equal = |left: &Point3, right: &Point3| {
        let left = hyperlimit::Point3::new(left.x.clone(), left.y.clone(), left.z.clone());
        let right = hyperlimit::Point3::new(right.x.clone(), right.y.clone(), right.z.clone());
        matches!(hyperlimit::point3_equal(&left, &right).value(), Some(true))
    };
    let mut edges = Vec::<[Point3; 2]>::new();
    let mut coplanar_positions = Vec::new();
    let mut coplanar_triangles = Vec::new();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        let Some(points) = [a, b, c]
            .into_iter()
            .map(|index| mesh.positions.get(index).cloned())
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        let signs = points
            .iter()
            .map(|point| crate::hyper_math::hreal_sign(&(point.z.clone() - z.clone())))
            .collect::<Vec<_>>();
        if signs.iter().all(|sign| *sign == Some(RealSign::Zero)) {
            let base = coplanar_positions.len();
            coplanar_positions.extend(points);
            coplanar_triangles.push(Triangle::new(base, base + 1, base + 2));
            continue;
        }
        let mut intersections = Vec::with_capacity(2);
        for [start, end] in [[0, 1], [1, 2], [2, 0]] {
            let (start_sign, end_sign) = (signs[start], signs[end]);
            let point = if start_sign == Some(RealSign::Zero) {
                Some(points[start].clone())
            } else if matches!(
                (start_sign, end_sign),
                (Some(RealSign::Negative), Some(RealSign::Positive))
                    | (Some(RealSign::Positive), Some(RealSign::Negative))
            ) {
                let denominator = points[end].z.clone() - points[start].z.clone();
                let Ok(parameter) = (z.clone() - points[start].z.clone()) / denominator else {
                    continue;
                };
                Some(Point3::new(
                    points[start].x.clone()
                        + (points[end].x.clone() - points[start].x.clone())
                            * parameter.clone(),
                    points[start].y.clone()
                        + (points[end].y.clone() - points[start].y.clone()) * parameter,
                    z.clone(),
                ))
            } else {
                None
            };
            if let Some(point) = point
                && !intersections.iter().any(|existing| equal(existing, &point))
            {
                intersections.push(point);
            }
        }
        if intersections.len() == 2 {
            edges.push([intersections.remove(0), intersections.remove(0)]);
        }
    }

    let mut chains = Vec::<Vec<Point3>>::new();
    let mut used = vec![false; edges.len()];
    for start in 0..edges.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let mut chain = vec![edges[start][0].clone(), edges[start][1].clone()];
        while let Some(last) = chain.last() {
            let mut next = None;
            for (index, edge) in edges.iter().enumerate() {
                if used[index] {
                    continue;
                }
                if equal(last, &edge[0]) {
                    next = Some((index, edge[1].clone()));
                    break;
                }
                if equal(last, &edge[1]) {
                    next = Some((index, edge[0].clone()));
                    break;
                }
            }
            let Some((index, point)) = next else {
                break;
            };
            used[index] = true;
            chain.push(point);
        }
        chains.push(chain);
    }

    let policy = CurvePolicy::certified();
    let mut region = if coplanar_triangles.is_empty() {
        CurveRegion2::empty()
    } else {
        flatten(&TriangleMesh::new(coplanar_positions, coplanar_triangles))
    };
    let mut wires = Vec::new();
    for chain in chains {
        let points = chain
            .iter()
            .map(|point| [point.x.clone(), point.y.clone()])
            .collect::<Vec<_>>();
        let closed = chain
            .first()
            .zip(chain.last())
            .is_some_and(|(first, last)| equal(first, last));
        if closed {
            let Ok(contour) = Contour2::from_real_ring(&points) else {
                continue;
            };
            let Ok(loop_region) =
                CurveRegion2::try_from_native_material_contours(vec![contour], &policy)
            else {
                continue;
            };
            region = if region.is_empty() {
                loop_region
            } else {
                region
                    .boolean_region(&loop_region, CurveBooleanOp::Union, &policy)
                    .unwrap_or(region)
            };
        } else if let Ok(wire) = CurveString2::from_real_point_iter(points) {
            wires.push(wire);
        }
    }
    let result = (region, wires, Vec::new());
    SLICE_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 8;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push(CachedSlice {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            z,
            result: result.clone(),
        });
    });
    result
}

/// Converts native geometry to a Bevy triangle mesh at the renderer boundary.
#[cfg(feature = "bevymesh")]
pub fn to_bevy_mesh(mesh: &TriangleMesh) -> bevy_mesh::Mesh {
    use bevy_asset::RenderAssetUsages;
    use bevy_mesh::{Indices, Mesh as BevyMesh, PrimitiveTopology};

    let gpu = mesh
        .try_to_gpu_mesh_f32()
        .expect("native GPU mesh approximation");
    let mut output =
        BevyMesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    output.insert_attribute(BevyMesh::ATTRIBUTE_POSITION, gpu.positions);
    output.insert_attribute(BevyMesh::ATTRIBUTE_NORMAL, gpu.normals);
    output.insert_indices(Indices::U32(gpu.indices));
    output
}

/// Exact convex hull of all native mesh positions.
pub fn convex_hull(mesh: &TriangleMesh) -> HypermeshResult<TriangleMesh> {
    mesh.convex_hull()
}

/// Exact Minkowski sum of two native triangle meshes.
pub fn minkowski_sum(
    left: &TriangleMesh,
    right: &TriangleMesh,
) -> Result<TriangleMesh, ValidationError> {
    if left.positions.is_empty() || right.positions.is_empty() {
        return Ok(empty());
    }
    let capacity = left
        .positions
        .len()
        .checked_mul(right.positions.len())
        .ok_or(ValidationError::InvalidArguments)?;
    let mut points = Vec::with_capacity(capacity);
    for left in left.positions.iter() {
        for right in right.positions.iter() {
            points.push(Point3::new(
                left.x.clone() + right.x.clone(),
                left.y.clone() + right.y.clone(),
                left.z.clone() + right.z.clone(),
            ));
        }
    }
    hypermesh::convex_hull(&points)
        .map_err(|error| ValidationError::Geometry(error.to_string()))
}

/// Lofts corresponding closed point loops into reusable native geometry.
pub fn loft(sections: &[Vec<Point3>]) -> Result<TriangleMesh, ValidationError> {
    if sections.len() < 2 {
        return Err(ValidationError::FieldLessThan {
            name: "sections",
            min: 2,
        });
    }
    let vertex_count = sections[0].len();
    if vertex_count < 3 {
        return Err(ValidationError::InvalidArguments);
    }
    if let Some(section) = sections.iter().find(|section| section.len() != vertex_count) {
        return Err(ValidationError::MismatchedVertexCount {
            left: vertex_count,
            right: section.len(),
        });
    }

    let section_basis =
        |section: &[Point3]| -> Result<(Vector3, Vector3, Vector3), ValidationError> {
            if section
                .iter()
                .zip(section.iter().cycle().skip(1))
                .any(|(left, right)| left == right)
            {
                return Err(ValidationError::InvalidArguments);
            }
            let origin = &section[0];
            let axis_x = (&section[1] - origin)
                .normalize_checked()
                .map_err(|_| ValidationError::InvalidArguments)?;
            let normal = section[2..]
                .iter()
                .find_map(|point| axis_x.cross(&(point - origin)).normalize_checked().ok())
                .ok_or(ValidationError::InvalidArguments)?;
            if section.iter().any(|point| {
                crate::hyper_math::hreal_sign(&normal.dot(&(point - origin)))
                    != Some(RealSign::Zero)
            }) {
                return Err(ValidationError::InvalidArguments);
            }
            let axis_y = normal
                .cross(&axis_x)
                .normalize_checked()
                .map_err(|_| ValidationError::InvalidArguments)?;
            Ok((axis_x, axis_y, normal))
        };
    let bases = sections
        .iter()
        .map(|section| section_basis(section))
        .collect::<Result<Vec<_>, _>>()?;
    let first = &sections[0];
    let normal = &bases[0].2;
    let travel = &sections[sections.len() - 1][0] - &first[0];
    let forward = matches!(
        crate::hyper_math::hreal_sign(&normal.dot(&travel)),
        Some(RealSign::Positive)
    );
    if !forward
        && !matches!(
            crate::hyper_math::hreal_sign(&normal.dot(&travel)),
            Some(RealSign::Negative)
        )
    {
        return Err(ValidationError::InvalidArguments);
    }

    let triangulate_cap = |section: &[Point3],
                           axis_x: &Vector3,
                           axis_y: &Vector3|
     -> Result<Vec<[usize; 3]>, ValidationError> {
        let origin = &section[0];
        let points = section
            .iter()
            .map(|point| {
                let offset = point - origin;
                hypertri::Point2::new(axis_x.dot(&offset), axis_y.dot(&offset))
            })
            .collect::<Vec<_>>();
        let flat = hypertri::earcut(&points, &[])
            .map_err(|error| ValidationError::Geometry(error.to_string()))?;
        Ok(flat
            .chunks_exact(3)
            .map(|triangle| [triangle[0], triangle[1], triangle[2]])
            .collect())
    };

    let position_capacity = sections
        .len()
        .checked_mul(vertex_count)
        .ok_or(ValidationError::InvalidArguments)?;
    let side_triangle_count = sections
        .len()
        .checked_sub(1)
        .and_then(|count| count.checked_mul(vertex_count))
        .and_then(|count| count.checked_mul(2))
        .ok_or(ValidationError::InvalidArguments)?;
    let cap_triangle_count = vertex_count
        .checked_sub(2)
        .and_then(|count| count.checked_mul(2))
        .ok_or(ValidationError::InvalidArguments)?;
    let triangle_capacity = side_triangle_count
        .checked_add(cap_triangle_count)
        .ok_or(ValidationError::InvalidArguments)?;
    let mut positions = Vec::with_capacity(position_capacity);
    positions.extend(sections.iter().flatten().cloned());
    let mut triangles = Vec::with_capacity(triangle_capacity);
    for mut triangle in triangulate_cap(first, &bases[0].0, &bases[0].1)? {
        if forward {
            triangle.swap(1, 2);
        }
        triangles.push(triangle);
    }
    let top_base = (sections.len() - 1) * vertex_count;
    for mut triangle in triangulate_cap(
        &sections[sections.len() - 1],
        &bases[bases.len() - 1].0,
        &bases[bases.len() - 1].1,
    )? {
        if !forward {
            triangle.swap(1, 2);
        }
        triangles.push([
            top_base + triangle[0],
            top_base + triangle[1],
            top_base + triangle[2],
        ]);
    }
    for section in 0..sections.len() - 1 {
        let bottom = section * vertex_count;
        let top = bottom + vertex_count;
        for index in 0..vertex_count {
            let next = (index + 1) % vertex_count;
            let mut first = [bottom + index, bottom + next, top + next];
            let mut second = [bottom + index, top + next, top + index];
            if !forward {
                first.swap(1, 2);
                second.swap(1, 2);
            }
            triangles.push(first);
            triangles.push(second);
        }
    }
    Ok(indexed(positions, triangles))
}

/// Fluent CSG grammar implemented directly for native Hypermesh geometry.
pub trait SolidExt: Sized {
    /// Exact regularized union.
    fn try_union(&self, other: &Self) -> HypermeshResult<Self>;
    /// Exact regularized difference.
    fn try_difference(&self, other: &Self) -> HypermeshResult<Self>;
    /// Exact regularized intersection.
    fn try_intersection(&self, other: &Self) -> HypermeshResult<Self>;
    /// Exact regularized symmetric difference.
    fn try_xor(&self, other: &Self) -> HypermeshResult<Self>;
    /// Apply a homogeneous transform.
    fn transformed(&self, matrix: &Matrix4) -> Self;
    /// Translate by an exact vector.
    fn translated(&self, x: Real, y: Real, z: Real) -> Self;
    /// Return exact bounds.
    fn exact_bounds(&self) -> Aabb;
    /// Visit native triangles without constructing a csgrs mesh facade.
    fn visit_native_triangles<F>(&self, visitor: F)
    where
        F: FnMut([Point3; 3]);
}

impl SolidExt for TriangleMesh {
    fn try_union(&self, other: &Self) -> HypermeshResult<Self> {
        boolean(self, other, BooleanOp::Union)
    }

    fn try_difference(&self, other: &Self) -> HypermeshResult<Self> {
        boolean(self, other, BooleanOp::Difference)
    }

    fn try_intersection(&self, other: &Self) -> HypermeshResult<Self> {
        boolean(self, other, BooleanOp::Intersection)
    }

    fn try_xor(&self, other: &Self) -> HypermeshResult<Self> {
        boolean(self, other, BooleanOp::SymmetricDifference)
    }

    fn transformed(&self, matrix: &Matrix4) -> Self {
        transform(self, matrix)
    }

    fn translated(&self, x: Real, y: Real, z: Real) -> Self {
        transform(self, &Matrix4::affine_translation([x, y, z]))
    }

    fn exact_bounds(&self) -> Aabb {
        bounding_box(self).clone()
    }

    fn visit_native_triangles<F>(&self, mut visitor: F)
    where
        F: FnMut([Point3; 3]),
    {
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices();
            visitor([
                self.positions[a].clone(),
                self.positions[b].clone(),
                self.positions[c].clone(),
            ]);
        }
    }
}

/// Converts native topology to the optional attributed boundary carrier.
#[cfg(feature = "attributed")]
pub fn with_face_metadata<M>(mesh: &TriangleMesh, metadata: M) -> crate::AttributedMesh<M>
where
    M: Clone,
{
    crate::AttributedMesh::from_uniform(mesh.clone(), metadata)
}

/// Places a native point at the origin of a translation matrix.
pub fn translation_to(point: &Point3) -> Matrix4 {
    Matrix4::affine_translation([point.x.clone(), point.y.clone(), point.z.clone()])
}

/// Constructs an exact translation matrix from a vector.
pub fn translation(vector: Vector3) -> Matrix4 {
    Matrix4::affine_translation(vector.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.triangles
            .iter()
            .filter_map(|triangle| {
                let a = mesh.positions[triangle.v0].to_vector();
                let b = mesh.positions[triangle.v1].to_vector();
                let c = mesh.positions[triangle.v2].to_vector();
                a.dot(&b.cross(&c)).to_f64_lossy()
            })
            .sum::<f64>()
            / 6.0
    }

    #[test]
    fn sampled_circle_retains_exact_cardinal_angles() {
        assert_eq!(
            sampled_circle(4).unwrap(),
            vec![
                (Real::zero(), Real::one()),
                (Real::one(), Real::zero()),
                (Real::zero(), -Real::one()),
                (-Real::one(), Real::zero()),
            ]
        );
    }

    #[test]
    fn constructors_return_native_hypermesh_geometry() {
        let mesh: TriangleMesh = cube(Real::from(2_u8));
        assert_eq!(mesh.triangles.len(), 12);
        hypermesh::polygon_soup(&[mesh.as_ref()])
            .expect("native cube must satisfy Hypermesh's input contract");
    }

    #[test]
    fn primitive_sample_count_arithmetic_is_checked() {
        assert!(sphere(Real::one(), usize::MAX, 2).triangles.is_empty());
        assert!(
            frustum(Real::one(), Real::one(), Real::one(), usize::MAX)
                .triangles
                .is_empty()
        );
        assert!(
            arrow(Point3::origin(), Vector3::z(), usize::MAX, false)
                .triangles
                .is_empty()
        );
        #[cfg(feature = "curve")]
        assert!(
            torus(Real::from(2_u8), Real::one(), usize::MAX, 3)
                .triangles
                .is_empty()
        );
    }

    #[test]
    fn nominal_direct_solid_catalog_is_closed_and_consistently_indexed() {
        let one = Real::one();
        let two = Real::from(2_u8);
        let start = Point3::origin();
        let end = Point3::new(Real::zero(), Real::zero(), two.clone());
        let shapes = vec![
            cube(two.clone()),
            cuboid(two.clone(), one.clone(), Real::from(3_u8)),
            sphere(one.clone(), 12, 6),
            cylinder(one.clone(), two.clone(), 12),
            ellipsoid(two.clone(), one.clone(), one.clone(), 12, 6),
            frustum(one.clone(), two.clone(), two.clone(), 12),
            frustum(Real::zero(), one.clone(), two.clone(), 12),
            frustum_between(start.clone(), end.clone(), one.clone(), one.clone(), 12),
            octahedron(one.clone()),
            icosahedron(one.clone()),
            arrow(start, end.to_vector(), 12, false),
        ];
        #[cfg(feature = "curve")]
        let shapes = {
            let mut shapes = shapes;
            shapes.extend([
                torus(two.clone(), one.clone(), 12, 8),
                teardrop_cylinder(two.clone(), Real::from(3_u8), one.clone(), 12),
                spur_gear_involute(
                    one.clone(),
                    16,
                    Real::from(20_u8),
                    Real::zero(),
                    Real::zero(),
                    3,
                    one.clone(),
                ),
                spur_gear_cycloid(
                    one.clone(),
                    16,
                    (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
                    Real::zero(),
                    3,
                    one.clone(),
                ),
                helical_involute_gear(
                    one,
                    16,
                    Real::from(20_u8),
                    Real::zero(),
                    Real::zero(),
                    3,
                    Real::from(2_u8),
                    Real::from(15_u8),
                    3,
                ),
            ]);
            shapes
        };
        for (index, shape) in shapes.into_iter().enumerate() {
            assert!(!shape.triangles.is_empty(), "solid catalog entry {index}");
            assert!(
                shape.is_closed_manifold(),
                "closed indexed solid catalog entry {index}"
            );
            assert!(
                shape.has_unique_nondegenerate_triangles(),
                "solid catalog entry {index} contains duplicate or degenerate exact triangles"
            );
            assert!(
                shape.is_closed_manifold_geometry(),
                "solid catalog entry {index} is not an exact geometric two-manifold"
            );
            assert!(
                signed_volume(&shape) > 0.0,
                "outward solid catalog entry {index}"
            );
        }
    }

    #[test]
    fn arrows_and_orientation_reversing_transforms_keep_outward_manifold_winding() {
        let defects = |mesh: &TriangleMesh| {
            let mut edges = std::collections::HashMap::<(usize, usize), (usize, isize)>::new();
            for triangle in mesh.triangles.iter() {
                for (a, b) in [
                    (triangle.v0, triangle.v1),
                    (triangle.v1, triangle.v2),
                    (triangle.v2, triangle.v0),
                ] {
                    let entry = edges.entry((a.min(b), a.max(b))).or_default();
                    entry.0 += 1;
                    entry.1 += if a < b { 1 } else { -1 };
                }
            }
            (
                edges.values().filter(|(count, _)| *count != 2).count(),
                edges
                    .values()
                    .filter(|(count, direction)| *count == 2 && *direction != 0)
                    .count(),
            )
        };
        let start = Point3::origin();
        let direction = Vector3::from_xyz(Real::zero(), Real::zero(), Real::from(5_u8));
        let ordinary = arrow(start.clone(), direction.clone(), 24, false);
        assert!(
            ordinary.is_closed_manifold(),
            "ordinary defects: {:?}",
            defects(&ordinary)
        );
        assert!(signed_volume(&ordinary) > 0.0);
        let reflected_arrow = arrow(start, direction, 24, true);
        assert!(
            reflected_arrow.is_closed_manifold(),
            "mirrored defects: {:?}",
            defects(&reflected_arrow)
        );
        assert!(signed_volume(&reflected_arrow) > 0.0);

        let cube = cube(Real::from(2_u8));
        let reflected = scale(&cube, -Real::one(), Real::one(), Real::one());
        assert!(reflected.is_closed_manifold());
        assert!(signed_volume(&reflected) > 0.0);
        assert_eq!(
            reflected.triangles[0],
            Triangle::new(
                cube.triangles[0].v2,
                cube.triangles[0].v1,
                cube.triangles[0].v0,
            )
        );
        let plane = Plane::axis_aligned(0, Real::zero());
        let mirrored = mirror(&cube, &plane);
        assert!(mirrored.is_closed_manifold());
        assert!(signed_volume(&mirrored) > 0.0);
    }

    #[test]
    fn fluent_boolean_returns_reusable_native_geometry() {
        let left = cube(Real::from(2_u8));
        let right =
            cube(Real::from(2_u8)).translated(Real::from(1_u8), Real::zero(), Real::zero());
        let result: TriangleMesh = left
            .try_union(&right)
            .expect("native cube union must be certified");
        assert!(!result.triangles.is_empty());
        assert!(result.has_unique_nondegenerate_triangles());
        assert!(result.is_closed_manifold_geometry());
        hypermesh::polygon_soup(&[result.as_ref()])
            .expect("native Boolean output must be reusable as Hypermesh input");
    }

    #[test]
    fn polyhedron_certifies_symbolically_planar_faces_before_normalization() {
        let phi = ((Real::one() + Real::from(5_u8).sqrt().expect("five is positive"))
            / Real::from(2_u8))
        .expect("two is nonzero");
        let points = [
            [Real::zero(), Real::zero(), Real::zero()],
            [Real::one(), Real::zero(), phi.clone()],
            [Real::one(), Real::one(), phi],
            [Real::zero(), Real::one(), Real::zero()],
        ];
        let face = [0, 1, 2, 3];

        let mesh = polyhedron(&points, &[&face])
            .expect("an exactly planar symbolic face must be accepted");

        assert_eq!(mesh.triangles.len(), 2);
    }

    #[test]
    fn loft_uses_each_cap_plane_and_rejects_nonplanar_sections() {
        let point = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
        let bottom = vec![
            point(0, 0, 0),
            point(1, 0, 0),
            point(1, 1, 0),
            point(0, 1, 0),
        ];
        let tilted = vec![
            point(0, 0, 1),
            point(1, 0, 2),
            point(1, 1, 2),
            point(0, 1, 1),
        ];
        let tilted_loft = loft(&[bottom.clone(), tilted]).expect("independently planar caps");
        assert!(tilted_loft.is_closed_manifold());
        assert!(tilted_loft.has_unique_nondegenerate_triangles());
        assert!(tilted_loft.is_closed_manifold_geometry());

        let nonplanar = vec![
            point(0, 0, 1),
            point(1, 0, 1),
            point(1, 1, 2),
            point(0, 1, 1),
        ];
        assert!(loft(&[bottom, nonplanar]).is_err());
    }
}
