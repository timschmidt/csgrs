//! Exact affine-box solid booleans.
//!
//! Axis-aligned box shortcuts already cover a useful coplanar-volumetric cell
//! family, but they are still tied to world axes. This module retains one exact
//! affine frame for two closed parallelepiped meshes, normalizes both solids
//! into `(u, v, w)` AABB boxes, reuses the orthogonal box/cell materializers,
//! and lifts the accepted output back to 3D. The basis, normalized replay, and
//! primitive-float fit or tolerance decides the topology.
//!
//! The selected cells are the affine image of the rectangular box subdivision
//! determinants, and every accepted source vertex must reconstruct exactly
//! from the retained frame before the orthogonal cell complex can be consumed.

use core::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use super::error::MeshError;
use super::mesh::{ExactMesh, Triangle};
use super::validation::ValidationPolicy;
use hyperlimit::SourceProvenance;
use hyperreal::Real;

/// Exact 3D affine frame for normalized box coordinates.
///
/// A normalized point `(u, v, w)` is interpreted as
/// `origin + u * basis_u + v * basis_v + w * basis_w`. The frame is part of
/// the certificate: every source and output vertex must replay through it
/// exactly before a copied boolean artifact is accepted.
#[derive(Clone, Debug, PartialEq)]
pub struct AffineBoxBasis {
    /// Exact 3D affine origin.
    pub origin: Point3,
    /// Exact vector for the normalized `u` axis.
    pub basis_u: Point3,
    /// Exact vector for the normalized `v` axis.
    pub basis_v: Point3,
    /// Exact vector for the normalized `w` axis.
    pub basis_w: Point3,
}

pub(crate) fn mesh_to_uvw(
    mesh: &ExactMesh,
    basis: &AffineBoxBasis,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    let vertices = mesh
        .vertices()
        .iter()
        .map(|point| {
            point_to_uvw_checked(&point.clone(), basis).map(|uvw| Point3::new(uvw.x, uvw.y, uvw.z))
        })
        .collect::<Option<Vec<_>>>()?;
    let triangles = triangles_for_affine_orientation(mesh, basis)?;
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact affine-normalized box solid"),
        validation,
    )
    .ok()
}

pub(crate) fn mesh_from_uvw(
    mesh: &ExactMesh,
    basis: &AffineBoxBasis,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let vertices = mesh
        .vertices()
        .iter()
        .map(|point| {
            let point = point.clone();
            let lifted = point_from_uvw(&point.x, &point.y, &point.z, basis);
            Point3::new(lifted.x, lifted.y, lifted.z)
        })
        .collect::<Vec<_>>();
    let triangles =
        if compare_reals(&basis.determinant(), &Real::from(0)).value() == Some(Ordering::Less) {
            mesh.triangles().iter().map(reverse_triangle).collect()
        } else {
            mesh.triangles().to_vec()
        };
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )
}

fn point_to_uvw_checked(point: &Point3, basis: &AffineBoxBasis) -> Option<Point3> {
    let uvw = point_to_uvw(point, basis)?;
    let replay = point_from_uvw(&uvw.x, &uvw.y, &uvw.z, basis);
    if points_equal(&replay, point) {
        Some(uvw)
    } else {
        None
    }
}

fn point_to_uvw(point: &Point3, basis: &AffineBoxBasis) -> Option<Point3> {
    let delta = sub3(point, &basis.origin);
    let denominator = basis.determinant();
    if compare_reals(&denominator, &Real::from(0)).value()? == Ordering::Equal {
        return None;
    }
    let u = (det3(&delta, &basis.basis_v, &basis.basis_w) / &denominator).ok()?;
    let v = (det3(&basis.basis_u, &delta, &basis.basis_w) / &denominator).ok()?;
    let w = (det3(&basis.basis_u, &basis.basis_v, &delta) / &denominator).ok()?;
    Some(Point3::new(u, v, w))
}

fn triangles_for_affine_orientation(
    mesh: &ExactMesh,
    basis: &AffineBoxBasis,
) -> Option<Vec<Triangle>> {
    // A negative determinant reverses orientation under the exact affine
    // coordinate map. Reversing triangle order keeps the normalized shell
    if compare_reals(&basis.determinant(), &Real::from(0)).value()? == Ordering::Less {
        Some(mesh.triangles().iter().map(reverse_triangle).collect())
    } else {
        Some(mesh.triangles().to_vec())
    }
}

fn reverse_triangle(triangle: &Triangle) -> Triangle {
    let [a, b, c] = triangle.0;
    Triangle([a, c, b])
}

fn point_from_uvw(u: &Real, v: &Real, w: &Real, basis: &AffineBoxBasis) -> Point3 {
    Point3::new(
        add(
            &basis.origin.x,
            &add(
                &mul(u, &basis.basis_u.x),
                &add(&mul(v, &basis.basis_v.x), &mul(w, &basis.basis_w.x)),
            ),
        ),
        add(
            &basis.origin.y,
            &add(
                &mul(u, &basis.basis_u.y),
                &add(&mul(v, &basis.basis_v.y), &mul(w, &basis.basis_w.y)),
            ),
        ),
        add(
            &basis.origin.z,
            &add(
                &mul(u, &basis.basis_u.z),
                &add(&mul(v, &basis.basis_v.z), &mul(w, &basis.basis_w.z)),
            ),
        ),
    )
}

impl AffineBoxBasis {
    pub(crate) fn determinant(&self) -> Real {
        det3(&self.basis_u, &self.basis_v, &self.basis_w)
    }
}

fn det3(a: &Point3, b: &Point3, c: &Point3) -> Real {
    let x_minor = sub(&mul(&b.y, &c.z), &mul(&b.z, &c.y));
    let y_minor = sub(&mul(&b.x, &c.z), &mul(&b.z, &c.x));
    let z_minor = sub(&mul(&b.x, &c.y), &mul(&b.y, &c.x));
    add(
        &sub(&mul(&a.x, &x_minor), &mul(&a.y, &y_minor)),
        &mul(&a.z, &z_minor),
    )
}

fn sub3(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub(&left.x, &right.x),
        sub(&left.y, &right.y),
        sub(&left.z, &right.z),
    )
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    real_eq(&left.x, &right.x) && real_eq(&left.y, &right.y) && real_eq(&left.z, &right.z)
}

fn real_eq(left: &Real, right: &Real) -> bool {
    compare_reals(left, right).value() == Some(Ordering::Equal)
}

fn add(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn sub(left: &Real, right: &Real) -> Real {
    left.clone() - right
}

fn mul(left: &Real, right: &Real) -> Real {
    left.clone() * right
}
