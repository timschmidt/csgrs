//! Exact axis-aligned box solid certificates.
//!
//! This module is intentionally narrow. It recognizes closed triangular meshes
//! whose exact vertices are exactly the eight corners of their retained AABB.
//! Boolean support is handled by the orthogonal arrangement layer, which
//! replays occupancy on the merged exact grid instead of bypassing the
//! cell-complex pipeline with box-specific topology.

use core::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::mesh::ExactMesh;
use super::solid::certify_convex_solid;
use hyperreal::Real;

/// Certified exact AABB box bounds retained by the shortcut.
#[derive(Clone, Debug, PartialEq)]
struct AxisAlignedBox {
    min: Point3,
    max: Point3,
}

/// Coordinate axis used for exact retained AABB validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Axis {
    X,
    Y,
    Z,
}

/// Return whether one mesh certifies as a retained exact axis-aligned box.
///
/// Affine-normalized solid shortcuts use this as their local replay boundary:
/// a transformed mesh may enter the existing orthogonal cell materializer only
/// after its exact vertices, closed topology, and convexity certify as one
/// structure rule intact across the affine adapter instead of trusting a
/// coordinate transform alone.
pub(crate) fn is_axis_aligned_box(mesh: &ExactMesh) -> bool {
    matches!(try_certify_axis_aligned_box(mesh), Ok(Some(_)))
}

/// Return whether one mesh certifies as a retained exact axis-aligned box,
/// preserving retained-certificate and exact-predicate blockers.
pub(crate) fn try_is_axis_aligned_box(mesh: &ExactMesh) -> Result<bool, ExactMeshError> {
    try_certify_axis_aligned_box(mesh).map(|box_| box_.is_some())
}

/// Recognize a closed exact mesh as exactly its retained AABB.
fn try_certify_axis_aligned_box(
    mesh: &ExactMesh,
) -> Result<Option<AxisAlignedBox>, ExactMeshError> {
    if mesh.vertices().len() != 8 || mesh.triangles().len() != 12 {
        return Ok(None);
    }
    mesh.validate_retained_bounds_certificate()?;
    let Some(bounds) = mesh.bounds().mesh() else {
        return Ok(None);
    };
    let box_bounds = AxisAlignedBox {
        min: bounds.min.clone(),
        max: bounds.max.clone(),
    };
    let Some(box_bounds) = valid_box(box_bounds)? else {
        return Ok(None);
    };
    let corners = box_bounds.corners();
    for vertex in mesh.vertices() {
        let point = vertex.clone();
        if !points_equal_any(&corners, &point)? {
            return Ok(None);
        }
    }
    for corner in &corners {
        if !mesh_point_equal_any(mesh, corner)? {
            return Ok(None);
        }
    }
    let convex = certify_convex_solid(mesh);
    if convex.is_certified_convex() && convex.all_proof_producing() {
        Ok(Some(box_bounds))
    } else {
        Ok(None)
    }
}

impl AxisAlignedBox {
    fn corners(&self) -> [Point3; 8] {
        let min = &self.min;
        let max = &self.max;
        [
            Point3::new(min.x.clone(), min.y.clone(), min.z.clone()),
            Point3::new(max.x.clone(), min.y.clone(), min.z.clone()),
            Point3::new(max.x.clone(), max.y.clone(), min.z.clone()),
            Point3::new(min.x.clone(), max.y.clone(), min.z.clone()),
            Point3::new(min.x.clone(), min.y.clone(), max.z.clone()),
            Point3::new(max.x.clone(), min.y.clone(), max.z.clone()),
            Point3::new(max.x.clone(), max.y.clone(), max.z.clone()),
            Point3::new(min.x.clone(), max.y.clone(), max.z.clone()),
        ]
    }
}

fn valid_box(bounds: AxisAlignedBox) -> Result<Option<AxisAlignedBox>, ExactMeshError> {
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        if cmp(axis_min(&bounds.min, axis), axis_max(&bounds.max, axis))? != Ordering::Less {
            return Ok(None);
        }
    }
    Ok(Some(bounds))
}

fn axis_min(point: &Point3, axis: Axis) -> &Real {
    match axis {
        Axis::X => &point.x,
        Axis::Y => &point.y,
        Axis::Z => &point.z,
    }
}

fn axis_max(point: &Point3, axis: Axis) -> &Real {
    axis_min(point, axis)
}

fn cmp(left: &Real, right: &Real) -> Result<Ordering, ExactMeshError> {
    compare_reals(left, right).value().ok_or_else(|| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::UndecidablePredicate,
            "exact axis-aligned box certificate comparison was undecidable",
        ))
    })
}

fn real_eq(left: &Real, right: &Real) -> Result<bool, ExactMeshError> {
    Ok(cmp(left, right)? == Ordering::Equal)
}

fn points_equal(left: &Point3, right: &Point3) -> Result<bool, ExactMeshError> {
    Ok(real_eq(&left.x, &right.x)? && real_eq(&left.y, &right.y)? && real_eq(&left.z, &right.z)?)
}

fn points_equal_any(points: &[Point3], point: &Point3) -> Result<bool, ExactMeshError> {
    for candidate in points {
        if points_equal(candidate, point)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn mesh_point_equal_any(mesh: &ExactMesh, point: &Point3) -> Result<bool, ExactMeshError> {
    for vertex in mesh.vertices() {
        if points_equal(point, vertex)? {
            return Ok(true);
        }
    }
    Ok(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0],
                max[1], min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1],
                max[2], min[0], max[1], max[2],
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap()
    }

    #[test]
    fn fallible_axis_aligned_box_predicate_certifies_box_shape() {
        let box_mesh = axis_aligned_box_i64([0, 0, 0], [1, 1, 1]);
        assert!(try_is_axis_aligned_box(&box_mesh).unwrap());

        let tetrahedron = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        assert!(!try_is_axis_aligned_box(&tetrahedron).unwrap());
    }
}
