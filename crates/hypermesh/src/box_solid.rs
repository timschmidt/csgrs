//! Exact axis-aligned box solid certificates.
//!
//! This module is intentionally narrow. It recognizes closed triangular meshes
//! whose exact vertices are exactly the eight corners of their retained AABB.
//! Boolean support is handled by the orthogonal arrangement layer, which
//! replays occupancy on the merged exact grid instead of bypassing the
//! cell-complex pipeline with box-specific topology.

use core::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

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
    certify_axis_aligned_box(mesh).is_some()
}

/// Recognize a closed exact mesh as exactly its retained AABB.
fn certify_axis_aligned_box(mesh: &ExactMesh) -> Option<AxisAlignedBox> {
    if mesh.vertices().len() != 8 || mesh.triangles().len() != 12 {
        return None;
    }
    let bounds = mesh.bounds().mesh.as_ref()?;
    let box_bounds = AxisAlignedBox {
        min: bounds.min.clone(),
        max: bounds.max.clone(),
    };
    valid_box(box_bounds.clone())?;
    let corners = box_bounds.corners();
    for vertex in mesh.vertices() {
        let point = vertex.clone();
        if !corners.iter().any(|corner| points_equal(corner, &point)) {
            return None;
        }
    }
    for corner in &corners {
        if !mesh
            .vertices()
            .iter()
            .any(|vertex| points_equal(corner, &vertex.clone()))
        {
            return None;
        }
    }
    let convex = certify_convex_solid(mesh);
    if convex.is_certified_convex() && convex.all_proof_producing() {
        Some(box_bounds)
    } else {
        None
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

fn valid_box(bounds: AxisAlignedBox) -> Option<AxisAlignedBox> {
    let valid = [Axis::X, Axis::Y, Axis::Z].into_iter().all(|axis| {
        cmp(axis_min(&bounds.min, axis), axis_max(&bounds.max, axis)) == Some(Ordering::Less)
    });
    valid.then_some(bounds)
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

fn cmp(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn real_eq(left: &Real, right: &Real) -> bool {
    cmp(left, right) == Some(Ordering::Equal)
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    real_eq(&left.x, &right.x) && real_eq(&left.y, &right.y) && real_eq(&left.z, &right.z)
}
