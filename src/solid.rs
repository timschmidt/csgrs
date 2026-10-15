//! CSG grammar over native [`hypermesh::TriangleMesh`] geometry.
//!
//! This module owns modeling vocabulary, not a second mesh carrier. Primitive
//! constructors, transforms, and Boolean composition all accept or return the
//! native Hypermesh type.
//!
//! Primitive constructors return [`ValidationError`] on invalid parameters,
//! undecided strict predicates, unsupported representations, or checked buffer
//! capacity/reservation failures. Dimensions and radii are nonnegative; a
//! proven zero size yields canonical empty geometry after validating all inputs.
//! Frustum/cylinder height is directed and may be negative. Sample counts need
//! at least three segments and two sphere/ellipsoid stacks. Nonempty tori need
//! `major_radius > minor_radius`. Only successful results are cached.
//!
//! Exact sampled coordinates are retained; construction does not certify
//! analytic-surface fidelity or inherited provenance. Capacity checks are not
//! total-work or heap-byte budgets. Scalar growth, secondary allocations and
//! cancellation remain outside the constructor resource contract.

use crate::context::{GeometryContext, GeometryDecisions, GeometryOutcome};
use crate::errors::ValidationError;
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hypermesh::{
    BooleanExpression, BooleanOp, BooleanProgram, HypermeshError, HypermeshResult, Plane,
    Triangle, TriangleMesh,
};
use hyperreal::RealSign;
use std::num::NonZeroU32;

pub mod cache;

use cache::{
    DistributionParameters, PrimitiveParameters, retain_distribution, retain_merge,
    retained_distribution, retained_merge, retained_primitive,
};
#[cfg(feature = "curve")]
use cache::{retain_flatten, retain_slice, retained_flatten, retained_slice};

#[cfg(feature = "curve")]
/// Filled region, open strings, and open paths produced by a horizontal slice.
pub type SliceResult = (
    hypercurve::CurveRegion2,
    Vec<hypercurve::CurveString2>,
    Vec<hypercurve::CurvePath2>,
);

#[cfg(feature = "metaballs")]
pub use crate::implicit::metaballs::{MetaBall, MetaballDiagnostics};
#[cfg(feature = "sdf")]
pub use crate::implicit::sdf::SdfDiagnostics;

fn primitive_sign(value: &Real, predicate: &'static str) -> Result<RealSign, ValidationError> {
    GeometryDecisions::new(&GeometryContext::STRICT).sign(value, predicate)
}

fn nonnegative(value: &Real, name: &'static str) -> Result<RealSign, ValidationError> {
    match primitive_sign(value, name)? {
        RealSign::Negative => Err(ValidationError::NegativeParameter { name }),
        sign => Ok(sign),
    }
}

pub(crate) const fn sample_minimum(
    value: usize,
    min: usize,
    name: &'static str,
) -> Result<(), ValidationError> {
    if value < min {
        Err(ValidationError::InvalidResolution {
            name,
            min,
            actual: value,
        })
    } else {
        Ok(())
    }
}

pub(crate) fn capacity<T>(count: Option<usize>) -> Result<usize, ValidationError> {
    count
        .filter(|count| {
            count
                .checked_mul(std::mem::size_of::<T>())
                .is_some_and(|bytes| bytes <= isize::MAX as usize)
        })
        .ok_or(ValidationError::CapacityOverflow)
}

pub(crate) fn reserved<T>(count: usize) -> Result<Vec<T>, ValidationError> {
    let mut values = Vec::new();
    values
        .try_reserve_exact(capacity::<T>(Some(count))?)
        .map_err(|_| ValidationError::AllocationFailed { elements: count })?;
    Ok(values)
}

fn sampled_circle(count: usize) -> Result<Vec<(Real, Real)>, ValidationError> {
    sample_minimum(count, 1, "circle samples")?;
    let denominator =
        Real::from(u64::try_from(count).map_err(|_| ValidationError::CapacityOverflow)?);
    let mut samples = reserved(count)?;
    for index in 0..count {
        let fraction = (Real::from(
            u64::try_from(index).map_err(|_| ValidationError::CapacityOverflow)?,
        ) / &denominator)?;
        let angle = Real::tau() * fraction;
        samples.push((angle.clone().sin(), angle.cos()));
    }
    Ok(samples)
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
    indexed(positions, triangles)
}

/// Empty native triangle geometry.
pub fn empty() -> TriangleMesh {
    TriangleMesh::new(Vec::new(), Vec::new())
}

/// Axis-aligned cube with its minimum corner at the origin.
pub fn cube(width: Real) -> Result<TriangleMesh, ValidationError> {
    cuboid(width.clone(), width.clone(), width)
}

/// Axis-aligned cuboid with its minimum corner at the origin.
pub fn cuboid(
    width: Real,
    length: Real,
    height: Real,
) -> Result<TriangleMesh, ValidationError> {
    let parameters =
        PrimitiveParameters::Cuboid(width.clone(), length.clone(), height.clone());
    retained_primitive(parameters, || cuboid_uncached(width, length, height))
}

fn cuboid_uncached(
    width: Real,
    length: Real,
    height: Real,
) -> Result<TriangleMesh, ValidationError> {
    let signs = [
        nonnegative(&width, "width")?,
        nonnegative(&length, "length")?,
        nonnegative(&height, "height")?,
    ];
    if signs.contains(&RealSign::Zero) {
        return Ok(empty());
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
    Ok(indexed_convex(
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
    ))
}

/// Tessellated sphere centered at the origin.
pub fn sphere(
    radius: Real,
    segments: usize,
    stacks: usize,
) -> Result<TriangleMesh, ValidationError> {
    let parameters = PrimitiveParameters::Sphere(radius.clone(), segments, stacks);
    retained_primitive(parameters, || sphere_uncached(radius, segments, stacks))
}

fn sphere_uncached(
    radius: Real,
    segments: usize,
    stacks: usize,
) -> Result<TriangleMesh, ValidationError> {
    sample_minimum(segments, 3, "segments")?;
    sample_minimum(stacks, 2, "stacks")?;
    let radius_sign = nonnegative(&radius, "radius")?;
    let position_capacity = capacity::<Point3>(
        segments
            .checked_mul(stacks - 1)
            .and_then(|count| count.checked_add(2)),
    )?;
    let triangle_capacity = capacity::<[usize; 3]>(
        segments
            .checked_mul(stacks - 1)
            .and_then(|count| count.checked_mul(2)),
    )?;
    if radius_sign == RealSign::Zero {
        return Ok(empty());
    }
    let mut positions = reserved(position_capacity)?;
    let mut triangles = reserved(triangle_capacity)?;
    let longitudes = sampled_circle(segments)?;
    let stack_count =
        Real::from(u64::try_from(stacks).map_err(|_| ValidationError::CapacityOverflow)?);
    let mut latitudes = reserved(stacks - 1)?;
    for stack in 1..stacks {
        let fraction = (Real::from(
            u64::try_from(stack).map_err(|_| ValidationError::CapacityOverflow)?,
        ) / &stack_count)?;
        let angle = Real::pi() * fraction;
        latitudes.push((angle.clone().sin(), angle.cos()));
    }
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
    Ok(indexed_convex(positions, triangles))
}

/// Tessellated cylinder along +Z.
pub fn cylinder(
    radius: Real,
    height: Real,
    segments: usize,
) -> Result<TriangleMesh, ValidationError> {
    frustum(radius.clone(), radius, height, segments)
}

/// Tessellated ellipsoid centered at the origin.
pub fn ellipsoid(
    radius_x: Real,
    radius_y: Real,
    radius_z: Real,
    segments: usize,
    stacks: usize,
) -> Result<TriangleMesh, ValidationError> {
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
) -> Result<TriangleMesh, ValidationError> {
    sample_minimum(segments, 3, "segments")?;
    sample_minimum(stacks, 2, "stacks")?;
    let signs = [
        nonnegative(&radius_x, "radius_x")?,
        nonnegative(&radius_y, "radius_y")?,
        nonnegative(&radius_z, "radius_z")?,
    ];
    capacity::<Point3>(
        segments
            .checked_mul(stacks - 1)
            .and_then(|count| count.checked_add(2)),
    )?;
    capacity::<[usize; 3]>(
        segments
            .checked_mul(stacks - 1)
            .and_then(|count| count.checked_mul(2)),
    )?;
    if signs.contains(&RealSign::Zero) {
        return Ok(empty());
    }
    try_scale(
        &sphere(Real::one(), segments, stacks)?,
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
) -> Result<TriangleMesh, ValidationError> {
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
) -> Result<TriangleMesh, ValidationError> {
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
) -> Result<TriangleMesh, ValidationError> {
    sample_minimum(segments, 3, "segments")?;
    let bottom_degenerate = nonnegative(&start_radius, "start_radius")? == RealSign::Zero;
    let top_degenerate = nonnegative(&end_radius, "end_radius")? == RealSign::Zero;
    let position_capacity =
        capacity::<Point3>(segments.checked_mul(2).and_then(|count| count.checked_add(2)))?;
    let triangle_capacity = capacity::<[usize; 3]>(segments.checked_mul(4))?;
    let direction = &end - &start;
    let direction_sign = primitive_sign(&direction.dot(&direction), "frustum length squared")?;
    if direction_sign == RealSign::Zero || (bottom_degenerate && top_degenerate) {
        return Ok(empty());
    }
    let axis = direction.normalize_checked()?;
    let (axis_x, axis_y) = axis.orthonormal_basis_checked()?;
    let mut positions = reserved(position_capacity)?;
    let mut triangles = reserved(triangle_capacity)?;
    let samples = sampled_circle(segments)?;
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
    Ok(indexed_convex(positions, triangles))
}

/// Regular octahedron centered at the origin.
pub fn octahedron(radius: Real) -> Result<TriangleMesh, ValidationError> {
    let parameters = PrimitiveParameters::Octahedron(radius.clone());
    retained_primitive(parameters, || octahedron_uncached(radius))
}

fn octahedron_uncached(radius: Real) -> Result<TriangleMesh, ValidationError> {
    if nonnegative(&radius, "radius")? == RealSign::Zero {
        return Ok(empty());
    }
    let negative = -radius.clone();
    Ok(indexed_convex(
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
    ))
}

/// Regular icosahedron centered at the origin.
pub fn icosahedron(radius: Real) -> Result<TriangleMesh, ValidationError> {
    let parameters = PrimitiveParameters::Icosahedron(radius.clone());
    retained_primitive(parameters, || icosahedron_uncached(radius))
}

fn icosahedron_uncached(radius: Real) -> Result<TriangleMesh, ValidationError> {
    if nonnegative(&radius, "radius")? == RealSign::Zero {
        return Ok(empty());
    }
    let phi =
        (Real::one() + Real::from(5_u8).sqrt().expect("five is positive")) / Real::from(2_u8);
    let phi = phi?;
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
    Ok(indexed_convex(
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
    ))
}

/// Tessellated arrow from a start point and direction vector.
pub fn arrow(
    start: Point3,
    direction: Vector3,
    segments: usize,
    mirrored: bool,
) -> Result<TriangleMesh, ValidationError> {
    sample_minimum(segments, 3, "segments")?;
    let position_capacity =
        capacity::<Point3>(segments.checked_mul(3).and_then(|count| count.checked_add(2)))?;
    let triangle_capacity = capacity::<[usize; 3]>(segments.checked_mul(7))?;
    let squared_length = direction.dot(&direction);
    if primitive_sign(&squared_length, "arrow length squared")? == RealSign::Zero {
        return Ok(empty());
    }
    let axis = direction.normalize_checked()?;
    let (axis_x, axis_y) = axis.orthonormal_basis_checked()?;
    let length = squared_length.sqrt()?;
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
    let samples = sampled_circle(segments)?;
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
    let mut positions = reserved(position_capacity)?;
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
    let mut triangles = reserved(triangle_capacity)?;
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
        let faces_inward = match primitive_sign(&orientation, "arrow face orientation")? {
            RealSign::Negative => true,
            RealSign::Positive => false,
            RealSign::Zero => {
                return Err(ValidationError::UnsupportedPrimitive {
                    reason: "zero arrow face orientation",
                });
            },
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
    let orientation_score = triangles.iter().fold(Real::zero(), |score, triangle| {
        let [a, b, c] = triangle.map(|index| &positions[index]);
        let normal = (b - a).cross(&(c - a));
        let outward = a.to_vector() + b.to_vector() + c.to_vector()
            - interior.to_vector() * Real::from(3_u8);
        score + normal.dot(&outward)
    });
    match primitive_sign(&orientation_score, "arrow orientation")? {
        RealSign::Negative => {
            for triangle in &mut triangles {
                triangle.swap(1, 2);
            }
        },
        RealSign::Positive => {},
        RealSign::Zero => {
            return Err(ValidationError::UnsupportedPrimitive {
                reason: "zero arrow orientation",
            });
        },
    }
    Ok(indexed(positions, triangles))
}

/// Tessellated torus around the Z axis.
#[cfg(feature = "curve")]
pub fn torus(
    major_radius: Real,
    minor_radius: Real,
    major_segments: usize,
    minor_segments: usize,
) -> Result<TriangleMesh, ValidationError> {
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
) -> Result<TriangleMesh, ValidationError> {
    sample_minimum(major_segments, 3, "major_segments")?;
    sample_minimum(minor_segments, 3, "minor_segments")?;
    nonnegative(&major_radius, "major_radius")?;
    let minor_sign = nonnegative(&minor_radius, "minor_radius")?;
    let position_capacity = capacity::<Point3>(major_segments.checked_mul(minor_segments))?;
    let triangle_capacity = capacity::<[usize; 3]>(position_capacity.checked_mul(2))?;
    if minor_sign == RealSign::Zero {
        return Ok(empty());
    }
    if primitive_sign(
        &(&major_radius - &minor_radius),
        "torus major radius > minor radius",
    )? != RealSign::Positive
    {
        return Err(ValidationError::UnsupportedPrimitive {
            reason: "only ring tori (major_radius > minor_radius) are supported",
        });
    }
    let mut positions = reserved(position_capacity)?;
    let major = sampled_circle(major_segments)?;
    let minor = sampled_circle(minor_segments)?;
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
    let mut triangles = reserved(triangle_capacity)?;
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
    Ok(indexed(positions, triangles))
}

/// Extruded teardrop solid.
#[cfg(feature = "curve")]
pub fn teardrop_cylinder(
    width: Real,
    length: Real,
    height: Real,
    shape_segments: usize,
    context: &crate::GeometryContext,
) -> Result<crate::GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = crate::context::GeometryDecisions::new(context);
    let profile = decisions.consume_geometry(crate::curve::teardrop_with_context(
        width,
        length,
        shape_segments,
        context,
    )?);
    let mesh =
        decisions.consume_geometry(crate::curve::try_extrude(&profile, height, context)?);
    Ok(decisions.finish(mesh))
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
    context: &crate::GeometryContext,
) -> Result<crate::GeometryOutcome<TriangleMesh>, ValidationError> {
    crate::curve::try_extrude(
        &crate::curve::involute_gear(
            module,
            teeth,
            pressure_angle_degrees,
            clearance,
            backlash,
            segments_per_flank,
        ),
        thickness,
        context,
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
    context: &crate::GeometryContext,
) -> Result<crate::GeometryOutcome<TriangleMesh>, ValidationError> {
    crate::curve::try_extrude(
        &crate::curve::cycloidal_gear(
            module,
            teeth,
            generating_radius,
            clearance,
            segments_per_flank,
        ),
        thickness,
        context,
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
    context: &crate::GeometryContext,
) -> Result<crate::GeometryOutcome<TriangleMesh>, ValidationError> {
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
        context,
    )
}

/// Builds native triangle geometry from polygon index rows.
pub fn polyhedron(
    points: &[[Real; 3]],
    faces: &[&[usize]],
) -> Result<TriangleMesh, ValidationError> {
    polyhedron_with_context(points, faces, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Triangulates polyhedron faces with the selected policy and aggregate certainty.
pub fn polyhedron_with_context(
    points: &[[Real; 3]],
    faces: &[&[usize]],
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
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
        let points = face
            .iter()
            .map(|&index| &positions[index])
            .collect::<Vec<_>>();
        let (projected, _) = project_planar_ring(&points, &decisions)?;
        let indices = decisions.consume_triangulation(
            hypertri::earcut(&decisions.triangulation_context(), &projected, &[])
                .map_err(|error| ValidationError::Geometry(error.to_string()))?,
        );
        triangles.extend(indices.as_chunks::<3>().0.iter().map(|triangle| {
            Triangle::new(face[triangle[0]], face[triangle[1]], face[triangle[2]])
        }));
    }
    Ok(decisions.finish(TriangleMesh::new(positions, triangles)))
}

// A planar ring only needs an invertible coordinate projection for triangulation.
// Normalizing its support vectors adds square roots and strict scalar guards
// that do not contribute to topology or to the returned triangle coordinates.
fn project_planar_ring(
    points: &[&Point3],
    decisions: &GeometryDecisions,
) -> Result<(Vec<hypertri::Point2>, Vector3), ValidationError> {
    let origin = points[0];
    let mut normal = None;
    let mut blocker = None;
    'support: for left in 1..points.len() - 1 {
        for right in left + 1..points.len() {
            match hypermesh::Plane::points_are_nondegenerate(
                decisions.mesh_context(),
                origin,
                points[left],
                points[right],
            ) {
                Ok(outcome) => {
                    if decisions.consume_mesh(outcome) {
                        normal =
                            Some((points[left] - origin).cross(&(points[right] - origin)));
                        break 'support;
                    }
                },
                Err(error) => blocker = Some(ValidationError::Geometry(error.to_string())),
            }
        }
    }
    let normal = normal.ok_or_else(|| blocker.unwrap_or(ValidationError::InvalidArguments))?;
    for point in points {
        if decisions.sign(&normal.dot(&(*point - origin)), "ring planarity")? != RealSign::Zero
        {
            return Err(ValidationError::InvalidArguments);
        }
    }
    // Any decidably nonzero normal component gives an invertible projection.
    // An undecided component need not block another component's certified proof.
    let (axis, sign) = normal
        .0
        .iter()
        .enumerate()
        .find_map(|(axis, value)| {
            decisions
                .sign(value, "ring projection normal")
                .ok()
                .filter(|sign| *sign != RealSign::Zero)
                .map(|sign| (axis, sign))
        })
        .ok_or_else(|| {
            ValidationError::Geometry("ring has no decidably nonzero projection normal".into())
        })?;
    let projected = points
        .iter()
        .map(|point| {
            let (u, v) = match axis {
                0 => (point.y.clone(), point.z.clone()),
                1 => (point.z.clone(), point.x.clone()),
                _ => (point.x.clone(), point.y.clone()),
            };
            hypertri::Point2::new(u, if sign == RealSign::Negative { -v } else { v })
        })
        .collect();
    Ok((projected, normal))
}

/// Computes one regularized Boolean under [`GeometryContext::STRICT`].
///
/// Use [`boolean_with_context`] to select a predicate policy and retain the
/// operation's aggregate certainty.
pub fn boolean(
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
) -> HypermeshResult<TriangleMesh> {
    boolean_with_context(left, right, operation, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Computes one regularized Boolean with the selected predicate policy.
///
/// The context applies to input validation, the Boolean operation, and output
/// certification. The returned certainty records whether any predicate consumed
/// the approximate 512-bit terminal; choosing an approximate context does not
/// by itself make a certified result approximate.
///
/// Canonical empty geometry (no positions and no triangles) is a valid operand.
/// The other operand is still validated and regularized by Hypermesh, including
/// when the requested set is empty. A mesh missing only positions or only
/// triangles is not canonical empty geometry and remains a typed input error.
pub fn boolean_with_context(
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
    context: &GeometryContext,
) -> HypermeshResult<GeometryOutcome<TriangleMesh>> {
    let decisions = GeometryDecisions::new(context);
    let left_empty = left.positions.is_empty() && left.triangles.is_empty();
    let right_empty = right.positions.is_empty() && right.triangles.is_empty();
    let outcome = match (left_empty, right_empty) {
        (true, true) => return Ok(decisions.finish(empty())),
        (false, false) => hypermesh::boolean(
            decisions.mesh_context(),
            &[left.as_ref(), right.as_ref()],
            BooleanProgram::Operation(operation),
        )?,
        _ => {
            // Bridge the kernel's nonempty-input contract with its own truth
            // program. Returning the operand directly would skip validation
            // and leave repeated/canceling PWN boundaries unregularized.
            let (operand, mesh_index, winding) = if left_empty {
                (right, 1, [0, 1])
            } else {
                (left, 0, [1, 0])
            };
            let node = if operation.contains(&winding) {
                BooleanExpression::Operand(0)
            } else {
                BooleanExpression::False
            };
            hypermesh::boolean(
                decisions.mesh_context(),
                &[operand.as_ref()],
                BooleanProgram::Expressions {
                    nodes: &[node],
                    roots: &[0],
                },
            )
            .map_err(|error| match error {
                HypermeshError::EmptyMesh { .. } => HypermeshError::EmptyMesh { mesh_index },
                HypermeshError::DegenerateTriangle { triangle_index, .. } => {
                    HypermeshError::DegenerateTriangle {
                        mesh_index,
                        triangle_index,
                    }
                },
                HypermeshError::OpenInput { boundary_edges, .. } => {
                    HypermeshError::OpenInput {
                        mesh_index,
                        boundary_edges,
                    }
                },
                HypermeshError::NonPwnInput {
                    unbalanced_edges, ..
                } => HypermeshError::NonPwnInput {
                    mesh_index,
                    unbalanced_edges,
                },
                other => other,
            })?
        },
    };
    let batch = decisions.consume_mesh(outcome);
    let mut meshes = batch.into_triangle_meshes()?;
    let mesh = meshes
        .pop()
        .ok_or(hypermesh::HypermeshError::InvalidBooleanProgram {
            reason: "one CSG Boolean operation produced no result row",
        })?;
    Ok(decisions.finish(mesh))
}

/// Applies a homogeneous transform and returns native geometry.
pub fn try_transform(
    mesh: &TriangleMesh,
    matrix: &Matrix4,
) -> Result<TriangleMesh, ValidationError> {
    try_transform_with_context(mesh, matrix, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies a transform with the selected policy, preserving orientation and certainty.
pub fn try_transform_with_context(
    mesh: &TriangleMesh,
    matrix: &Matrix4,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let orientation = decisions.sign(&matrix.determinant(), "transform determinant")?;
    if orientation == RealSign::Zero {
        return Err(ValidationError::InvalidArguments);
    }
    let transformed = try_transform_with_known_orientation(
        mesh,
        matrix,
        orientation == RealSign::Negative,
        &decisions,
    )?;
    Ok(decisions.finish(transformed))
}

fn try_transform_with_known_orientation(
    mesh: &TriangleMesh,
    matrix: &Matrix4,
    reverse_winding: bool,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let transformed =
        decisions.consume_mesh(mesh.try_transformed(decisions.mesh_context(), matrix)?);
    Ok(if reverse_winding {
        transformed.reversed_winding()
    } else {
        transformed
    })
}

/// Applies a homogeneous transform, preserving typed failures.
///
/// Equivalent to [`try_transform`]; singular, undecided, and unbounded results fail.
pub fn transform(
    mesh: &TriangleMesh,
    matrix: &Matrix4,
) -> Result<TriangleMesh, ValidationError> {
    try_transform(mesh, matrix)
}

/// Rotates native geometry by Euler angles in degrees, reporting uncertainty.
pub fn try_rotate(
    mesh: &TriangleMesh,
    x: Real,
    y: Real,
    z: Real,
) -> Result<TriangleMesh, ValidationError> {
    try_rotate_with_context(mesh, x, y, z, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Rotates with the selected policy and reports aggregate certainty.
pub fn try_rotate_with_context(
    mesh: &TriangleMesh,
    x: Real,
    y: Real,
    z: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let rotated = decisions.consume_mesh(mesh.try_rotated_xyz_degrees(
        decisions.mesh_context(),
        x,
        y,
        z,
    )?);
    Ok(decisions.finish(rotated))
}

/// Rotates native geometry by Euler angles in degrees.
pub fn rotate(
    mesh: &TriangleMesh,
    x: Real,
    y: Real,
    z: Real,
) -> Result<TriangleMesh, ValidationError> {
    try_rotate(mesh, x, y, z)
}

/// Scales native geometry independently along each axis, reporting degeneracy.
pub fn try_scale(
    mesh: &TriangleMesh,
    x: Real,
    y: Real,
    z: Real,
) -> Result<TriangleMesh, ValidationError> {
    try_scale_with_context(mesh, x, y, z, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Scales with the selected policy, rejecting collapsed axes and retaining certainty.
pub fn try_scale_with_context(
    mesh: &TriangleMesh,
    x: Real,
    y: Real,
    z: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let signs = [&x, &y, &z]
        .map(|value| decisions.sign(value, "scale axis"))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
    if signs.contains(&RealSign::Zero) {
        return Err(ValidationError::InvalidArguments);
    }
    let reverse_winding = signs
        .iter()
        .filter(|sign| **sign == RealSign::Negative)
        .count()
        % 2
        == 1;
    let scaled = try_transform_with_known_orientation(
        mesh,
        &Matrix4::affine_nonuniform_scale([x, y, z]),
        reverse_winding,
        &decisions,
    )?;
    Ok(decisions.finish(scaled))
}

/// Scales native geometry independently along each axis.
pub fn scale(
    mesh: &TriangleMesh,
    x: Real,
    y: Real,
    z: Real,
) -> Result<TriangleMesh, ValidationError> {
    try_scale(mesh, x, y, z)
}

/// Reflects native geometry across a plane, reporting an invalid plane.
pub fn try_mirror(
    mesh: &TriangleMesh,
    plane: &Plane,
) -> Result<TriangleMesh, ValidationError> {
    try_mirror_with_context(mesh, plane, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Reflects across a plane with the selected policy and aggregate certainty.
pub fn try_mirror_with_context(
    mesh: &TriangleMesh,
    plane: &Plane,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let matrix = decisions.consume_mesh(plane.reflection_matrix(decisions.mesh_context())?);
    // A valid plane reflection always reverses orientation.
    let mirrored = try_transform_with_known_orientation(mesh, &matrix, true, &decisions)?;
    Ok(decisions.finish(mirrored))
}

/// Reflects native geometry across a plane.
pub fn mirror(mesh: &TriangleMesh, plane: &Plane) -> Result<TriangleMesh, ValidationError> {
    try_mirror(mesh, plane)
}

/// Uniformly subdivides every native triangle.
pub fn subdivide(
    mesh: &TriangleMesh,
    levels: NonZeroU32,
) -> hypermesh::HypermeshResult<TriangleMesh> {
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

/// Samples inverse-square metaballs through the native exact-coordinate sampler.
///
/// This uses the continuous normalized field `(iso-I)/(iso+I)`, extended to -1
/// at centers, with I the sum of exact inverse-square influences. Numerical
/// certainty concerns the sampled proposal, not its fidelity to the analytic
/// field. Radii and iso must be positive, padding nonnegative, and each grid
/// dimension at least two. Invalid or undecided input and sampling failures are
/// typed errors; an empty source list succeeds only after parameter validation.
#[cfg(feature = "metaballs")]
pub fn metaballs(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
) -> Result<TriangleMesh, ValidationError> {
    metaballs_with_context(
        balls,
        resolution,
        iso_value,
        padding,
        &GeometryContext::STRICT,
    )
    .map(|outcome| outcome.value)
}

/// Samples the normalized metaball field and returns diagnostics with typed failure.
#[cfg(feature = "metaballs")]
pub fn metaballs_with_diagnostics(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
) -> Result<(TriangleMesh, MetaballDiagnostics), ValidationError> {
    metaballs_with_diagnostics_and_context(
        balls,
        resolution,
        iso_value,
        padding,
        &GeometryContext::STRICT,
    )
    .map(|outcome| outcome.value)
}

/// Samples metaballs with the selected meshing policy, retaining local certainty.
/// Native field construction and its conservative bounds remain strict.
#[cfg(feature = "metaballs")]
pub fn metaballs_with_context(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    crate::implicit::metaballs::sample(balls, resolution, iso_value, padding, false, context)
        .map(|outcome| outcome.map(|(mesh, _)| mesh))
}

/// Samples metaballs and retains diagnostics together with consumed certainty.
#[cfg(feature = "metaballs")]
pub fn metaballs_with_diagnostics_and_context(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, MetaballDiagnostics)>, ValidationError> {
    crate::implicit::metaballs::sample(balls, resolution, iso_value, padding, true, context)
}

/// Samples an exact scalar field with native-real Surface Nets.
///
/// Coordinates remain [`Real`] throughout construction. The regular sample
/// grid still defines proposal topology; this is not a continuous-field
/// topology certificate. Each grid dimension must be at least two. Invalid
/// dimensions, undecided predicates, capacity failures and extraction errors
/// return `Err`; successful empty geometry remains `Ok`.
#[cfg(feature = "sdf")]
pub fn sdf(
    field: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    sdf_with_context(
        field,
        resolution,
        min,
        max,
        iso_value,
        &GeometryContext::STRICT,
    )
    .map(|outcome| outcome.value)
}

/// Samples an SDF into native triangle geometry and returns sampling diagnostics.
#[cfg(feature = "sdf")]
pub fn sdf_with_diagnostics(
    field: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> Result<(TriangleMesh, SdfDiagnostics), ValidationError> {
    sdf_with_diagnostics_and_context(
        field,
        resolution,
        min,
        max,
        iso_value,
        &GeometryContext::STRICT,
    )
    .map(|outcome| outcome.value)
}

/// Samples a Hypersdf expression with native-real Surface Nets.
///
/// Coordinates remain [`Real`] throughout construction. The regular sample
/// grid still defines proposal topology; this is not a continuous-field
/// topology certificate. Each grid dimension must be at least two. Invalid
/// dimensions, undecided predicates, capacity failures and extraction errors
/// return `Err`; successful empty geometry remains `Ok`.
#[cfg(feature = "sdf")]
pub fn sdf_expr(
    expression: hypersdf::SdfExpr,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    sdf_expr_with_context(
        expression,
        resolution,
        min,
        max,
        iso_value,
        &GeometryContext::STRICT,
    )
    .map(|outcome| outcome.value)
}

/// Samples a retained SDF expression and returns sampling diagnostics.
#[cfg(feature = "sdf")]
pub fn sdf_expr_with_diagnostics(
    expression: hypersdf::SdfExpr,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
) -> Result<(TriangleMesh, SdfDiagnostics), ValidationError> {
    sdf_expr_with_diagnostics_and_context(
        expression,
        resolution,
        min,
        max,
        iso_value,
        &GeometryContext::STRICT,
    )
    .map(|outcome| outcome.value)
}

/// Samples and meshes a scalar field with the selected policy, retaining errors and certainty.
/// As with [`sdf`], certification covers the sampled grid, not the continuous field.
#[cfg(feature = "sdf")]
pub fn sdf_with_context(
    field: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    crate::implicit::sdf::sample_with_context(
        |point| Ok(field(point)),
        resolution,
        min,
        max,
        iso_value,
        false,
        context,
    )
    .map(|outcome| outcome.map(|(mesh, _)| mesh))
}

/// Samples a field with the selected policy and retains sampling diagnostics.
#[cfg(feature = "sdf")]
pub fn sdf_with_diagnostics_and_context(
    field: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, SdfDiagnostics)>, ValidationError> {
    crate::implicit::sdf::sample_with_context(
        |point| Ok(field(point)),
        resolution,
        min,
        max,
        iso_value,
        true,
        context,
    )
}

/// Samples a Hypersdf expression and meshes its scalar values with the selected policy.
/// Expression domain and branch checks remain strict even when meshing permits
/// approximate predicates. Evaluation errors retain their cause and sample index.
#[cfg(feature = "sdf")]
pub fn sdf_expr_with_context(
    expression: hypersdf::SdfExpr,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    crate::implicit::sdf::sample_expr_with_context(
        expression, resolution, min, max, iso_value, false, context,
    )
    .map(|outcome| outcome.map(|(mesh, _)| mesh))
}

/// Samples an expression with the selected policy and retains diagnostics and certainty.
/// Expression domain and branch checks remain strict. Preview samples remain lossy,
/// and their sign counts include only strict proofs, even under an approximate policy.
#[cfg(feature = "sdf")]
pub fn sdf_expr_with_diagnostics_and_context(
    expression: hypersdf::SdfExpr,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, SdfDiagnostics)>, ValidationError> {
    crate::implicit::sdf::sample_expr_with_context(
        expression, resolution, min, max, iso_value, true, context,
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
) -> Result<TriangleMesh, ValidationError> {
    crate::implicit::tpms::gyroid_solid(
        bounding_box(bounds)?.as_ref(),
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
) -> Result<TriangleMesh, ValidationError> {
    crate::implicit::tpms::schwarz_p_solid(
        bounding_box(bounds)?.as_ref(),
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
) -> Result<TriangleMesh, ValidationError> {
    crate::implicit::tpms::schwarz_d_solid(
        bounding_box(bounds)?.as_ref(),
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
) -> Result<TriangleMesh, ValidationError> {
    crate::implicit::tpms::gyroid(bounding_box(bounds)?.as_ref(), resolution, scale, iso_value)
}

/// Samples a bounded Schwarz-P level surface.
#[cfg(feature = "sdf")]
pub fn schwarz_p(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    crate::implicit::tpms::schwarz_p(
        bounding_box(bounds)?.as_ref(),
        resolution,
        scale,
        iso_value,
    )
}

/// Samples a bounded Schwarz-D level surface.
#[cfg(feature = "sdf")]
pub fn schwarz_d(
    bounds: &TriangleMesh,
    resolution: usize,
    scale: Real,
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    crate::implicit::tpms::schwarz_d(
        bounding_box(bounds)?.as_ref(),
        resolution,
        scale,
        iso_value,
    )
}

/// Exact bounds of native triangle positions.
pub fn try_bounding_box(mesh: &TriangleMesh) -> Result<Option<Aabb>, ValidationError> {
    try_bounding_box_with_context(mesh, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Returns bounds with the selected policy and aggregate certainty.
pub fn try_bounding_box_with_context(
    mesh: &TriangleMesh,
    context: &GeometryContext,
) -> Result<GeometryOutcome<Option<Aabb>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let bounds = decisions.consume_mesh(mesh.exact_bounds(decisions.mesh_context())?);
    Ok(decisions.finish(bounds))
}

/// Exact position bounds, absent for canonical empty geometry; errors remain typed.
pub fn bounding_box(mesh: &TriangleMesh) -> Result<Option<Aabb>, ValidationError> {
    try_bounding_box(mesh)
}

/// Translates the center of the exact bounds to the origin.
pub fn try_center(mesh: &TriangleMesh) -> Result<TriangleMesh, ValidationError> {
    try_center_with_context(mesh, &GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Centers the selected-policy bounds, retaining certainty through the translation.
pub fn try_center_with_context(
    mesh: &TriangleMesh,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let Some(bounds) =
        decisions.consume_geometry(try_bounding_box_with_context(mesh, context)?)
    else {
        return Ok(decisions.finish(mesh.clone()));
    };
    let two = Real::from(2_u8);
    let x = -((&bounds.mins.x + &bounds.maxs.x) / &two)?;
    let y = -((&bounds.mins.y + &bounds.maxs.y) / &two)?;
    let z = -((&bounds.mins.z + &bounds.maxs.z) / &two)?;
    let centered = try_transform_with_known_orientation(
        mesh,
        &Matrix4::affine_translation([x, y, z]),
        false,
        &decisions,
    )?;
    Ok(decisions.finish(centered))
}

/// Translates the center of the exact bounds to the origin.
pub fn center(mesh: &TriangleMesh) -> Result<TriangleMesh, ValidationError> {
    try_center(mesh)
}

/// Translates the minimum Z bound to zero.
pub fn try_float(mesh: &TriangleMesh) -> Result<TriangleMesh, ValidationError> {
    try_float_with_context(mesh, &GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Moves the selected-policy minimum Z bound to zero and retains certainty.
pub fn try_float_with_context(
    mesh: &TriangleMesh,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let Some(bounds) =
        decisions.consume_geometry(try_bounding_box_with_context(mesh, context)?)
    else {
        return Ok(decisions.finish(mesh.clone()));
    };
    let floated = try_transform_with_known_orientation(
        mesh,
        &Matrix4::affine_translation([Real::zero(), Real::zero(), -bounds.mins.z]),
        false,
        &decisions,
    )?;
    Ok(decisions.finish(floated))
}

/// Translates the minimum Z bound to zero.
pub fn float(mesh: &TriangleMesh) -> Result<TriangleMesh, ValidationError> {
    try_float(mesh)
}

/// Concatenates native meshes without performing a Boolean operation.
pub fn merge(meshes: &[TriangleMesh]) -> TriangleMesh {
    if let Some(result) = retained_merge(meshes) {
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
    retain_merge(meshes, &result);
    result
}

fn union_copies(
    copies: Vec<TriangleMesh>,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let bounds = copies
        .iter()
        .map(|mesh| {
            mesh.exact_bounds(decisions.mesh_context())
                .ok()
                .and_then(|outcome| decisions.consume_mesh(outcome))
        })
        .collect::<Option<Vec<_>>>();
    let mut pairwise_disjoint = bounds.is_some();
    if let Some(bounds) = bounds {
        for (index, left) in bounds.iter().enumerate() {
            for right in &bounds[index + 1..] {
                let disjoint = decisions.decide(
                    hyperlimit::ordered_aabb3s_intersect(
                        &left.mins,
                        &left.maxs,
                        &right.mins,
                        &right.maxs,
                        decisions.predicate_policy(),
                    ),
                    "distribution bounds overlap",
                );
                if !matches!(disjoint, Ok(false)) {
                    pairwise_disjoint = false;
                    break;
                }
            }
            if !pairwise_disjoint {
                break;
            }
        }
    }
    if pairwise_disjoint {
        return Ok(merge(&copies));
    }
    let mut copies = copies.into_iter();
    let Some(first) = copies.next() else {
        return Ok(empty());
    };
    let context = GeometryContext::new(decisions.predicate_policy());
    copies.try_fold(first, |left, right| {
        boolean_with_context(&left, &right, BooleanOp::Union, &context)
            .map(|outcome| decisions.consume_geometry(outcome))
            .map_err(|error| ValidationError::Geometry(error.to_string()))
    })
}

/// Places `count` copies along a direction at exact spacing under strict policy.
pub fn distribute_linear(
    mesh: &TriangleMesh,
    count: usize,
    direction: Vector3,
    spacing: Real,
) -> TriangleMesh {
    distribute_linear_with_context(mesh, count, direction, spacing, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
        .unwrap_or_else(|_| empty())
}

/// Distributes and unions copies with the selected policy, preserving errors and certainty.
pub fn distribute_linear_with_context(
    mesh: &TriangleMesh,
    count: usize,
    direction: Vector3,
    spacing: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if count == 0 {
        return Ok(decisions.finish(empty()));
    }
    let parameters = DistributionParameters::Linear {
        count,
        direction: direction.clone(),
        spacing: spacing.clone(),
    };
    if let Some(result) = retained_distribution(mesh, &parameters, context) {
        return Ok(result);
    }
    if decisions.sign(&direction.dot(&direction), "distribution direction")?
        != RealSign::Positive
    {
        return Err(ValidationError::InvalidArguments);
    }
    let step = direction
        .normalize_checked()
        .map_err(|_| ValidationError::InvalidArguments)?
        * spacing;
    let copies = (0..count)
        .map(|index| {
            let offset = step.clone() * Real::from(index as u64);
            try_transform_with_known_orientation(
                mesh,
                &Matrix4::affine_translation(offset.0),
                false,
                &decisions,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let result = decisions.finish(union_copies(copies, &decisions)?);
    retain_distribution(mesh, parameters, context, &result);
    Ok(result)
}

/// Places copies in an XY grid under strict policy.
pub fn distribute_grid(
    mesh: &TriangleMesh,
    rows: usize,
    columns: usize,
    dx: Real,
    dy: Real,
) -> TriangleMesh {
    distribute_grid_with_context(mesh, rows, columns, dx, dy, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
        .unwrap_or_else(|_| empty())
}

/// Distributes and unions a grid with the selected policy and aggregate certainty.
pub fn distribute_grid_with_context(
    mesh: &TriangleMesh,
    rows: usize,
    columns: usize,
    dx: Real,
    dy: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if rows == 0 || columns == 0 {
        return Ok(decisions.finish(empty()));
    }
    let parameters = DistributionParameters::Grid {
        rows,
        columns,
        dx: dx.clone(),
        dy: dy.clone(),
    };
    if let Some(result) = retained_distribution(mesh, &parameters, context) {
        return Ok(result);
    }
    let capacity = rows
        .checked_mul(columns)
        .ok_or(ValidationError::InvalidArguments)?;
    let mut copies = Vec::with_capacity(capacity);
    for row in 0..rows {
        for column in 0..columns {
            copies.push(try_transform_with_known_orientation(
                mesh,
                &Matrix4::affine_translation([
                    dx.clone() * Real::from(column as u64),
                    dy.clone() * Real::from(row as u64),
                    Real::zero(),
                ]),
                false,
                &decisions,
            )?);
        }
    }
    let result = decisions.finish(union_copies(copies, &decisions)?);
    retain_distribution(mesh, parameters, context, &result);
    Ok(result)
}

/// Places copies along a circular arc in the XY plane under strict policy.
pub fn distribute_arc(
    mesh: &TriangleMesh,
    count: usize,
    radius: Real,
    start_angle_degrees: Real,
    end_angle_degrees: Real,
) -> TriangleMesh {
    distribute_arc_with_context(
        mesh,
        count,
        radius,
        start_angle_degrees,
        end_angle_degrees,
        &GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
    .unwrap_or_else(|_| empty())
}

/// Distributes and unions an arc with the selected policy and aggregate certainty.
pub fn distribute_arc_with_context(
    mesh: &TriangleMesh,
    count: usize,
    radius: Real,
    start_angle_degrees: Real,
    end_angle_degrees: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if count == 0 {
        return Ok(decisions.finish(empty()));
    }
    let parameters = DistributionParameters::Arc {
        count,
        radius: radius.clone(),
        start_angle_degrees: start_angle_degrees.clone(),
        end_angle_degrees: end_angle_degrees.clone(),
    };
    if let Some(result) = retained_distribution(mesh, &parameters, context) {
        return Ok(result);
    }
    let start = start_angle_degrees.to_radians();
    let sweep = end_angle_degrees.to_radians() - start.clone();
    let translated = try_transform_with_known_orientation(
        mesh,
        &Matrix4::affine_translation([radius, Real::zero(), Real::zero()]),
        false,
        &decisions,
    )?;
    let copies = (0..count)
        .map(|index| {
            let fraction = if count == 1 {
                (Real::one() / Real::from(2_u8)).expect("two is nonzero")
            } else {
                (Real::from(index as u64) / Real::from((count - 1) as u64))
                    .expect("count minus one is nonzero")
            };
            let angle = start.clone() + sweep.clone() * fraction;
            try_transform_with_known_orientation(
                &translated,
                &Matrix4::rotation_z(angle),
                false,
                &decisions,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let result = decisions.finish(union_copies(copies, &decisions)?);
    retain_distribution(mesh, parameters, context, &result);
    Ok(result)
}

/// Exact point containment through the retained native triangle query path.
pub fn contains_point(
    mesh: &TriangleMesh,
    point: &Point3,
) -> hypermesh::HypermeshResult<bool> {
    contains_point_with_context(mesh, point, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Classifies a point with the selected policy and reports aggregate certainty.
pub fn contains_point_with_context(
    mesh: &TriangleMesh,
    point: &Point3,
    context: &GeometryContext,
) -> HypermeshResult<GeometryOutcome<bool>> {
    let decisions = GeometryDecisions::new(context);
    let inside = decisions.consume_mesh(mesh.contains_point(decisions.mesh_context(), point)?);
    Ok(decisions.finish(inside))
}

/// Exact ray intersections sorted by ray parameter.
pub fn ray_intersections(
    mesh: &TriangleMesh,
    origin: &Point3,
    direction: &Vector3,
    context: &crate::GeometryContext,
) -> hypermesh::HypermeshResult<crate::GeometryOutcome<Vec<(Point3, Real)>>> {
    let decisions = GeometryDecisions::new(context);
    let hits = decisions.consume_mesh(mesh.ray_intersections(
        decisions.mesh_context(),
        origin,
        direction,
    )?);
    Ok(decisions.finish(hits))
}

/// Exact intersections between native triangle geometry and a polyline.
pub fn polyline_intersections(
    mesh: &TriangleMesh,
    polyline: &[Point3],
) -> hypermesh::HypermeshResult<Vec<Point3>> {
    polyline_intersections_with_context(mesh, polyline, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Intersects a polyline with the selected policy and reports aggregate certainty.
pub fn polyline_intersections_with_context(
    mesh: &TriangleMesh,
    polyline: &[Point3],
    context: &GeometryContext,
) -> HypermeshResult<GeometryOutcome<Vec<Point3>>> {
    let decisions = GeometryDecisions::new(context);
    let hits = decisions
        .consume_mesh(mesh.polyline_intersections(decisions.mesh_context(), polyline)?);
    Ok(decisions.finish(hits))
}

/// Exact dihedral angle between two indexed triangles.
///
/// The angle is measured between the triangles' oriented plane normals.
pub fn dihedral_angle(
    mesh: &TriangleMesh,
    first: hypermesh::Triangle,
    second: hypermesh::Triangle,
) -> Option<Real> {
    dihedral_angle_with_context(mesh, first, second, &GeometryContext::STRICT)
        .ok()
        .map(GeometryOutcome::into_value)
}

/// Measures a dihedral angle with the selected policy, preserving errors and certainty.
pub fn dihedral_angle_with_context(
    mesh: &TriangleMesh,
    first: Triangle,
    second: Triangle,
    context: &GeometryContext,
) -> HypermeshResult<GeometryOutcome<Real>> {
    let decisions = GeometryDecisions::new(context);
    let angle = decisions.consume_mesh(mesh.dihedral_angle(
        decisions.mesh_context(),
        first,
        second,
    )?);
    Ok(decisions.finish(angle))
}

/// Exact uniform-density mass properties through Hyperphysics.
pub fn exact_mass_properties(
    mesh: &TriangleMesh,
    density: Real,
) -> Result<hyperphysics::MassPropertyReport3, ValidationError> {
    hyperphysics::triangle_mesh_uniform_density_mass_properties(mesh, density)
        .map_err(|error| ValidationError::Geometry(format!("{error:?}")))
}

/// Projects triangle faces into their regularized union in the XY plane.
///
/// Zero-area projected faces are omitted. Invalid indices and undecided
/// orientation or region predicates remain typed errors. This projects faces,
/// not an inferred analytic source, and does not certify input manifoldness.
#[cfg(feature = "curve")]
pub fn flatten(mesh: &TriangleMesh) -> Result<hypercurve::CurveRegion2, ValidationError> {
    flatten_with_context(mesh, &GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Flattens faces with the selected policy, retaining errors and aggregate certainty.
#[cfg(feature = "curve")]
pub fn flatten_with_context(
    mesh: &TriangleMesh,
    context: &GeometryContext,
) -> Result<GeometryOutcome<hypercurve::CurveRegion2>, ValidationError> {
    use hypercurve::{BooleanOp as CurveBooleanOp, Contour2, CurveRegion2};

    if let Some(region) = retained_flatten(mesh, context) {
        return Ok(region);
    }
    let decisions = GeometryDecisions::new(context);
    let mut output = CurveRegion2::empty();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = mesh.triangle_points(*triangle)?;
        let mut points = [
            [a.x.clone(), a.y.clone()],
            [b.x.clone(), b.y.clone()],
            [c.x.clone(), c.y.clone()],
        ];
        let limit = points
            .iter()
            .map(|[x, y]| hyperlimit::Point2::new(x.clone(), y.clone()))
            .collect::<Vec<_>>();
        match decisions.decide(
            hyperlimit::ring_area_sign(&limit, decisions.predicate_policy()),
            "projected face orientation",
        )? {
            hyperlimit::Sign::Negative => points.swap(1, 2),
            hyperlimit::Sign::Positive => {},
            hyperlimit::Sign::Zero => continue,
        }
        let contour = Contour2::from_real_ring(&points)?;
        let region = decisions
            .exact_curve(|| CurveRegion2::try_from_native_material_contours(vec![contour]))?;
        output = if output.is_empty() {
            region
        } else {
            decisions.exact_curve(|| output.boolean_region(&region, CurveBooleanOp::Union))?
        };
    }
    let output = decisions.finish(output);
    retain_flatten(mesh, context, &output);
    Ok(output)
}

/// Intersects oriented native geometry with the XY plane at exact Z.
///
/// Closed sections use nonzero winding; holes and duplicate or opposite
/// traversals retain their meaning. The one-sided sections and signed
/// coplanar faces are regularized separately, then united. Open sheets retain
/// section wires; isolated point contacts are omitted. Invalid indices and
/// undecided predicates are errors, including when the geometric result would
/// otherwise be empty. This operation does not certify manifoldness or
/// inherited construction provenance, nor impose a total-work or byte bound.
#[cfg(feature = "curve")]
pub fn slice_z(mesh: &TriangleMesh, z: Real) -> Result<SliceResult, ValidationError> {
    slice_z_with_context(mesh, z, &GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Slices with the selected policy, retaining native diagnostics and certainty.
#[cfg(feature = "curve")]
pub fn slice_z_with_context(
    mesh: &TriangleMesh,
    z: Real,
    context: &GeometryContext,
) -> Result<GeometryOutcome<SliceResult>, ValidationError> {
    use hypercurve::{
        BooleanOp as CurveBooleanOp, Contour2, CurveRegion2, CurveString2, FillRule,
    };

    if let Some(result) = retained_slice(mesh, &z, context) {
        return Ok(result);
    }
    let decisions = GeometryDecisions::new(context);
    let section = decisions.consume_mesh(mesh.slice_z(decisions.mesh_context(), &z)?);
    let chain_points = |chain: &hypermesh::SectionChain3| {
        let mut points = chain
            .segments
            .iter()
            .map(|&index| {
                let point = &section.segments[index].endpoints[0].position;
                [point.x.clone(), point.y.clone()]
            })
            .collect::<Vec<_>>();
        if let Some(&index) = chain.segments.last() {
            let point = &section.segments[index].endpoints[1].position;
            points.push([point.x.clone(), point.y.clone()]);
        }
        points
    };
    // Equal directed segment records refer to the same native points and
    // sources under this policy. Reuse their winding proof: union(A, A) = A.
    let identical_limits = section.above.len() == section.below.len()
        && section
            .above
            .iter()
            .zip(&section.below)
            .all(|(above, below)| {
                above.closed == below.closed && above.segments == below.segments
            });
    let mut region = CurveRegion2::empty();
    for chains in [&section.above, &section.below]
        .into_iter()
        .take(if identical_limits { 1 } else { 2 })
    {
        let contours = chains
            .iter()
            .map(|chain| {
                // A directed cycle may retrace one edge. Native contour
                // construction accepts that zero-area winding contribution;
                // a polygon-ring adapter incorrectly requires three vertices.
                Contour2::try_new(
                    CurveString2::from_real_point_iter(chain_points(chain))?.into_segments(),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let filled = decisions.exact_curve(|| {
            CurveRegion2::try_from_native_boundary_contours(&contours, FillRule::NonZero)
        })?;
        region =
            decisions.exact_curve(|| region.boolean_region(&filled, CurveBooleanOp::Union))?;
    }
    let coplanar = section
        .coplanar_triangles
        .iter()
        .map(|&index| {
            let points = mesh
                .triangle_points(mesh.triangles[index])?
                .map(|point| [point.x.clone(), point.y.clone()]);
            Ok::<_, ValidationError>(Contour2::from_real_ring(&points)?)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let coplanar = decisions.exact_curve(|| {
        CurveRegion2::try_from_native_boundary_contours(&coplanar, FillRule::NonZero)
    })?;
    region =
        decisions.exact_curve(|| region.boolean_region(&coplanar, CurveBooleanOp::Union))?;
    let wires = section
        .wires
        .iter()
        .map(|chain| CurveString2::from_real_point_iter(chain_points(chain)))
        .collect::<Result<Vec<_>, _>>()?;
    let result = decisions.finish((region, wires, Vec::new()));
    retain_slice(mesh, z, context, &result);
    Ok(result)
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
    convex_hull_with_context(mesh, &GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Computes the convex hull with the selected policy and aggregate certainty.
pub fn convex_hull_with_context(
    mesh: &TriangleMesh,
    context: &GeometryContext,
) -> HypermeshResult<GeometryOutcome<TriangleMesh>> {
    let decisions = GeometryDecisions::new(context);
    let hull = decisions.consume_mesh(mesh.convex_hull(decisions.mesh_context())?);
    Ok(decisions.finish(hull))
}

/// Exact Minkowski sum of two native triangle meshes.
pub fn minkowski_sum(
    left: &TriangleMesh,
    right: &TriangleMesh,
) -> Result<TriangleMesh, ValidationError> {
    minkowski_sum_with_context(left, right, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Computes the Minkowski hull with the selected policy and aggregate certainty.
pub fn minkowski_sum_with_context(
    left: &TriangleMesh,
    right: &TriangleMesh,
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if left.positions.is_empty() || right.positions.is_empty() {
        return Ok(decisions.finish(empty()));
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
    let hull = decisions.consume_mesh(
        hypermesh::convex_hull(decisions.mesh_context(), &points)
            .map_err(|error| ValidationError::Geometry(error.to_string()))?,
    );
    Ok(decisions.finish(hull))
}

/// Lofts corresponding closed point loops into reusable native geometry.
pub fn loft(sections: &[Vec<Point3>]) -> Result<TriangleMesh, ValidationError> {
    loft_with_context(sections, &GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Lofts sections with the selected policy for validation and cap triangulation.
pub fn loft_with_context(
    sections: &[Vec<Point3>],
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
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

    let projections = sections
        .iter()
        .map(|section| {
            for (left, right) in section.iter().zip(section.iter().cycle().skip(1)) {
                if decisions.points_equal(left, right)? {
                    return Err(ValidationError::InvalidArguments);
                }
            }
            project_planar_ring(&section.iter().collect::<Vec<_>>(), &decisions)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let first = &sections[0];
    let normal = &projections[0].1;
    let travel = &sections[sections.len() - 1][0] - &first[0];
    let forward = match decisions.sign(&normal.dot(&travel), "loft orientation")? {
        RealSign::Positive => true,
        RealSign::Negative => false,
        RealSign::Zero => return Err(ValidationError::InvalidArguments),
    };

    let triangulate_cap =
        |points: &[hypertri::Point2]| -> Result<Vec<[usize; 3]>, ValidationError> {
            let flat = decisions.consume_triangulation(
                hypertri::earcut(&decisions.triangulation_context(), points, &[])
                    .map_err(|error| ValidationError::Geometry(error.to_string()))?,
            );
            Ok(flat
                .as_chunks::<3>()
                .0
                .iter()
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
    for mut triangle in triangulate_cap(&projections[0].0)? {
        if forward {
            triangle.swap(1, 2);
        }
        triangles.push(triangle);
    }
    let top_base = (sections.len() - 1) * vertex_count;
    for mut triangle in triangulate_cap(&projections[projections.len() - 1].0)? {
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
    Ok(decisions.finish(indexed(positions, triangles)))
}

/// Fluent CSG grammar implemented directly for native Hypermesh geometry.
pub trait SolidExt: Sized {
    /// Regularized union under [`GeometryContext::STRICT`].
    /// See [`Self::try_union_with_context`] to select a predicate policy.
    fn try_union(&self, other: &Self) -> HypermeshResult<Self>;
    /// Regularized difference under [`GeometryContext::STRICT`].
    /// See [`Self::try_difference_with_context`] to select a predicate policy.
    fn try_difference(&self, other: &Self) -> HypermeshResult<Self>;
    /// Regularized intersection under [`GeometryContext::STRICT`].
    /// See [`Self::try_intersection_with_context`] to select a predicate policy.
    fn try_intersection(&self, other: &Self) -> HypermeshResult<Self>;
    /// Regularized symmetric difference under [`GeometryContext::STRICT`].
    /// See [`Self::try_xor_with_context`] to select a predicate policy.
    fn try_xor(&self, other: &Self) -> HypermeshResult<Self>;
    /// Regularized union with the selected policy and aggregate certainty.
    fn try_union_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>>;
    /// Regularized difference with the selected policy and aggregate certainty.
    fn try_difference_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>>;
    /// Regularized intersection with the selected policy and aggregate certainty.
    fn try_intersection_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>>;
    /// Regularized symmetric difference with the selected policy and aggregate certainty.
    fn try_xor_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>>;
    /// Apply a homogeneous transform.
    fn transformed(&self, matrix: &Matrix4) -> Result<Self, ValidationError>;
    /// Translate by an exact vector.
    fn translated(&self, x: Real, y: Real, z: Real) -> Result<Self, ValidationError>;
    /// Return exact bounds.
    fn exact_bounds(&self) -> Result<Option<Aabb>, ValidationError>;
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

    fn try_union_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>> {
        boolean_with_context(self, other, BooleanOp::Union, context)
    }

    fn try_difference_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>> {
        boolean_with_context(self, other, BooleanOp::Difference, context)
    }

    fn try_intersection_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>> {
        boolean_with_context(self, other, BooleanOp::Intersection, context)
    }

    fn try_xor_with_context(
        &self,
        other: &Self,
        context: &GeometryContext,
    ) -> HypermeshResult<GeometryOutcome<Self>> {
        boolean_with_context(self, other, BooleanOp::SymmetricDifference, context)
    }

    fn transformed(&self, matrix: &Matrix4) -> Result<Self, ValidationError> {
        transform(self, matrix)
    }

    fn translated(&self, x: Real, y: Real, z: Real) -> Result<Self, ValidationError> {
        transform(self, &Matrix4::affine_translation([x, y, z]))
    }

    fn exact_bounds(&self) -> Result<Option<Aabb>, ValidationError> {
        bounding_box(self)
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
        let mesh: TriangleMesh = cube(Real::from(2_u8)).expect("valid primitive fixture");
        assert_eq!(mesh.triangles.len(), 12);
        hypermesh::polygon_soup(&crate::MESH_CONTEXT, &[mesh.as_ref()])
            .expect("native cube must satisfy Hypermesh's input contract");
    }

    #[test]
    fn primitive_sample_count_arithmetic_is_checked() {
        let mut results = vec![
            sphere(Real::one(), usize::MAX, 2),
            frustum(Real::one(), Real::one(), Real::one(), usize::MAX),
            arrow(Point3::origin(), Vector3::z(), usize::MAX, false),
        ];
        #[cfg(feature = "curve")]
        results.push(torus(Real::from(2_u8), Real::one(), usize::MAX, 3));
        for result in results.drain(..) {
            assert!(matches!(result, Err(ValidationError::CapacityOverflow)));
        }
    }

    #[test]
    fn nominal_direct_solid_catalog_is_closed_and_consistently_indexed() {
        let one = Real::one();
        let two = Real::from(2_u8);
        let start = Point3::origin();
        let end = Point3::new(Real::zero(), Real::zero(), two.clone());
        let shapes = vec![
            cube(two.clone()).expect("valid primitive fixture"),
            cuboid(two.clone(), one.clone(), Real::from(3_u8))
                .expect("valid primitive fixture"),
            sphere(one.clone(), 12, 6).expect("valid primitive fixture"),
            cylinder(one.clone(), two.clone(), 12).expect("valid primitive fixture"),
            ellipsoid(two.clone(), one.clone(), one.clone(), 12, 6)
                .expect("valid primitive fixture"),
            frustum(one.clone(), two.clone(), two.clone(), 12)
                .expect("valid primitive fixture"),
            frustum(Real::zero(), one.clone(), two.clone(), 12)
                .expect("valid primitive fixture"),
            frustum_between(start.clone(), end.clone(), one.clone(), one.clone(), 12)
                .expect("valid primitive fixture"),
            octahedron(one.clone()).expect("valid primitive fixture"),
            icosahedron(one.clone()).expect("valid primitive fixture"),
            arrow(start, end.to_vector(), 12, false).expect("valid primitive fixture"),
        ];
        #[cfg(feature = "curve")]
        let shapes = {
            let mut shapes = shapes;
            shapes.extend([
                torus(two.clone(), one.clone(), 12, 8).expect("valid primitive fixture"),
                teardrop_cylinder(
                    two.clone(),
                    Real::from(3_u8),
                    one.clone(),
                    12,
                    &crate::GeometryContext::STRICT,
                )
                .expect("teardrop extrusion")
                .into_value(),
                spur_gear_involute(
                    one.clone(),
                    16,
                    Real::from(20_u8),
                    Real::zero(),
                    Real::zero(),
                    3,
                    one.clone(),
                    &crate::GeometryContext::STRICT,
                )
                .expect("involute gear extrusion")
                .into_value(),
                spur_gear_cycloid(
                    one.clone(),
                    16,
                    Real::one(),
                    Real::zero(),
                    3,
                    one.clone(),
                    &crate::GeometryContext::STRICT,
                )
                .expect("cycloidal gear extrusion")
                .into_value(),
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
                    &crate::GeometryContext::STRICT,
                )
                .expect("helical gear extrusion")
                .into_value(),
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
                shape
                    .has_unique_nondegenerate_triangles(&crate::MESH_CONTEXT)
                    .expect("solid boundary predicate")
                    .into_value(),
                "solid catalog entry {index} contains duplicate or degenerate exact triangles"
            );
            assert!(
                shape
                    .is_closed_manifold_geometry(&crate::MESH_CONTEXT)
                    .expect("solid boundary predicate")
                    .into_value(),
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
        let ordinary = arrow(start.clone(), direction.clone(), 24, false)
            .expect("valid primitive fixture");
        assert!(
            ordinary.is_closed_manifold(),
            "ordinary defects: {:?}",
            defects(&ordinary)
        );
        assert!(signed_volume(&ordinary) > 0.0);
        let reflected_arrow =
            arrow(start, direction, 24, true).expect("valid primitive fixture");
        assert!(
            reflected_arrow.is_closed_manifold(),
            "mirrored defects: {:?}",
            defects(&reflected_arrow)
        );
        assert!(signed_volume(&reflected_arrow) > 0.0);

        let cube = cube(Real::from(2_u8)).expect("valid primitive fixture");
        assert!(try_scale(&cube, Real::zero(), Real::one(), Real::one()).is_err());
        assert!(matches!(
            scale(&cube, Real::zero(), Real::one(), Real::one()),
            Err(ValidationError::InvalidArguments)
        ));
        let reflected = scale(&cube, -Real::one(), Real::one(), Real::one()).unwrap();
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
        let mirrored = mirror(&cube, &plane).unwrap();
        assert!(mirrored.is_closed_manifold());
        assert!(signed_volume(&mirrored) > 0.0);
    }

    #[test]
    fn fluent_boolean_returns_reusable_native_geometry() {
        let left = cube(Real::from(2_u8)).expect("valid primitive fixture");
        let right = cube(Real::from(2_u8))
            .expect("valid primitive fixture")
            .translated(Real::from(1_u8), Real::zero(), Real::zero())
            .expect("valid transform fixture");
        let result: TriangleMesh = left
            .try_union(&right)
            .expect("native cube union must be certified");
        assert!(!result.triangles.is_empty());
        assert!(
            result
                .has_unique_nondegenerate_triangles(&crate::MESH_CONTEXT)
                .unwrap()
                .into_value()
        );
        assert!(
            result
                .is_closed_manifold_geometry(&crate::MESH_CONTEXT)
                .unwrap()
                .into_value()
        );
        hypermesh::polygon_soup(&crate::MESH_CONTEXT, &[result.as_ref()])
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
        assert!(
            tilted_loft
                .has_unique_nondegenerate_triangles(&crate::MESH_CONTEXT)
                .unwrap()
                .into_value()
        );
        assert!(
            tilted_loft
                .is_closed_manifold_geometry(&crate::MESH_CONTEXT)
                .unwrap()
                .into_value()
        );

        let nonplanar = vec![
            point(0, 0, 1),
            point(1, 0, 1),
            point(1, 1, 2),
            point(0, 1, 1),
        ];
        assert!(loft(&[bottom, nonplanar]).is_err());
    }
}
