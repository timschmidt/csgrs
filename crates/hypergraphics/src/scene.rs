//! Exact scene construction helpers.

use hyperlattice::{Point3, Real};
use hypertri::ExactPoint;

use crate::error::Result;
use crate::geometry::{Color3, ExactMesh, ExactVertex, Primitive};

/// Build an exact RGB axis gizmo.
pub fn axes_mesh(length: Real, z_base: Real) -> Result<ExactMesh> {
    let red = Color3::new(1.0, 0.0, 0.0)?;
    let green = Color3::new(0.0, 1.0, 0.0)?;
    let blue = Color3::new(0.0, 0.0, 1.0)?;
    let zero = Real::zero();
    let mut mesh = ExactMesh::empty(Primitive::Lines);

    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero.clone(), z_base.clone()),
        red,
    ));
    mesh.push(ExactVertex::new(
        Point3::new(length.clone(), zero.clone(), z_base.clone()),
        red,
    ));

    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero.clone(), z_base.clone()),
        green,
    ));
    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), length.clone(), z_base.clone()),
        green,
    ));

    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero.clone(), z_base.clone()),
        blue,
    ));
    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero, length + z_base),
        blue,
    ));

    Ok(mesh)
}

/// Build exact XY grid lines centered at the origin.
///
/// `half_steps` controls loop cardinality, while `step` remains an exact
/// hyperreal spacing. This avoids deriving iteration counts from lossy floats.
pub fn grid_mesh(half_steps: i32, step: Real, color: Color3) -> ExactMesh {
    let vertex_capacity = usize::try_from(half_steps)
        .ok()
        .and_then(|count| count.checked_mul(2))
        .and_then(|count| count.checked_add(1))
        .and_then(|count| count.checked_mul(4))
        .unwrap_or(0);
    let mut mesh = ExactMesh::new(Primitive::Lines, Vec::with_capacity(vertex_capacity));
    let z = Real::zero();
    let extent = Real::from(half_steps) * step.clone();
    let coordinates = (-half_steps..=half_steps)
        .map(|index| Real::from(index) * step.clone())
        .collect::<Vec<_>>();

    for x in &coordinates {
        mesh.push(ExactVertex::new(
            Point3::new(x.clone(), -extent.clone(), z.clone()),
            color,
        ));
        mesh.push(ExactVertex::new(
            Point3::new(x.clone(), extent.clone(), z.clone()),
            color,
        ));
    }

    for y in &coordinates {
        mesh.push(ExactVertex::new(
            Point3::new(-extent.clone(), y.clone(), z.clone()),
            color,
        ));
        mesh.push(ExactVertex::new(
            Point3::new(extent.clone(), y.clone(), z.clone()),
            color,
        ));
    }

    mesh
}

/// Triangulate an exact 2D polygon with `hypertri` and lift it to a flat z plane.
pub fn polygon_surface_mesh(
    vertices: &[ExactPoint],
    hole_indices: &[usize],
    z: Real,
    color: Color3,
) -> Result<ExactMesh> {
    let indices = hypertri::earcut(vertices, hole_indices)?;
    let mut out = ExactMesh::new(Primitive::Triangles, Vec::with_capacity(indices.len()));
    for index in indices {
        let point = &vertices[index];
        out.push(ExactVertex::new(
            Point3::new(point.x.clone(), point.y.clone(), z.clone()),
            color,
        ));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use hyperreal::Real;
    use hypertri::Point2;

    use super::*;

    #[test]
    fn grid_keeps_exact_vertex_count() {
        let color = Color3::new(0.2, 0.2, 0.2).unwrap();
        let mesh = grid_mesh(2, Real::from(5), color);
        assert_eq!(mesh.vertex_count(), 20);
    }

    #[test]
    fn polygon_surface_uses_hypertri_indices() {
        let vertices = vec![
            Point2::new(Real::from(0), Real::from(0)),
            Point2::new(Real::from(1), Real::from(0)),
            Point2::new(Real::from(1), Real::from(1)),
            Point2::new(Real::from(0), Real::from(1)),
        ];
        let color = Color3::new(0.8, 0.4, 0.1).unwrap();
        let mesh = polygon_surface_mesh(&vertices, &[], Real::zero(), color).unwrap();
        assert_eq!(mesh.primitive(), Primitive::Triangles);
        assert_eq!(mesh.triangle_count(), 2);
    }
}
