//! Exact scene construction helpers.

use hyperlattice::{Point3, Real};
use hypermesh::TriangleMesh;
use hypertri::{ExactPoint, TriangulationContext, TriangulationOutcome};

use crate::error::{Error, Result};
use crate::geometry::{Color3, ExactMesh, ExactVertex, Primitive};

/// Expand a native exact Hypermesh triangle mesh into a flat colored scene mesh.
///
/// The source positions remain [`Real`] values. Triangle indexing is checked
/// before any scene value is produced, and primitive-float projection remains
/// deferred to the normal Hypergraphics render boundary.
pub fn triangle_mesh(mesh: &TriangleMesh, color: Color3) -> Result<ExactMesh> {
    let vertex_count = mesh
        .triangles
        .len()
        .checked_mul(3)
        .ok_or(Error::VertexCountOverflow { count: usize::MAX })?;
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(vertex_count)
        .map_err(|_| Error::VertexCountOverflow {
            count: vertex_count,
        })?;

    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        for (corner, index) in triangle.indices().into_iter().enumerate() {
            let position =
                mesh.positions
                    .get(index)
                    .ok_or(Error::SourceTriangleIndexOutOfBounds {
                        triangle: triangle_index,
                        corner,
                        index,
                        vertex_count: mesh.positions.len(),
                    })?;
            vertices.push(ExactVertex::new(position.clone(), color));
        }
    }

    Ok(ExactMesh::new(Primitive::Triangles, vertices))
}

/// Build an exact RGB axis gizmo.
pub fn axes_mesh(length: Real, z_base: Real) -> Result<ExactMesh> {
    let red = Color3::RED;
    let green = Color3::GREEN;
    let blue = Color3::BLUE;
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
/// `half_steps` is unsigned loop cardinality, while `step` remains an exact
/// hyperreal spacing. Allocation failure is reported instead of panicking.
pub fn grid_mesh(half_steps: u32, step: Real, color: Color3) -> Result<ExactMesh> {
    let coordinate_count_u64 = u64::from(half_steps) * 2 + 1;
    let vertex_count_u64 = coordinate_count_u64 * 4;
    let coordinate_count = usize::try_from(coordinate_count_u64)
        .map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let vertex_count =
        usize::try_from(vertex_count_u64).map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let mut coordinates = Vec::new();
    coordinates
        .try_reserve_exact(coordinate_count)
        .map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(vertex_count)
        .map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let mut mesh = ExactMesh::new(Primitive::Lines, vertices);
    let z = Real::zero();
    let half_steps = i64::from(half_steps);
    let extent = Real::from(half_steps) * step.clone();
    coordinates.extend((-half_steps..=half_steps).map(|index| Real::from(index) * step.clone()));

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

    Ok(mesh)
}

/// Triangulate an exact 2D polygon with `hypertri` and lift it to a flat z plane.
pub fn polygon_surface_mesh(
    context: &TriangulationContext,
    vertices: &[ExactPoint],
    hole_indices: &[usize],
    z: Real,
    color: Color3,
) -> Result<TriangulationOutcome<ExactMesh>> {
    Ok(
        hypertri::earcut(context, vertices, hole_indices)?.map(|indices| {
            let mut out = ExactMesh::new(Primitive::Triangles, Vec::with_capacity(indices.len()));
            for index in indices {
                let point = &vertices[index];
                out.push(ExactVertex::new(
                    Point3::new(point.x.clone(), point.y.clone(), z.clone()),
                    color,
                ));
            }
            out
        }),
    )
}

#[cfg(test)]
mod tests {
    use hyperlattice::Point3;
    use hyperlimit::PredicatePolicy;
    use hypermesh::{Triangle, TriangleMesh};
    use hyperreal::Real;
    use hypertri::{Point2, TriangulationContext};

    use super::*;

    #[test]
    fn native_triangle_mesh_stays_exact_until_render_export() {
        let one_third = Real::from(hyperreal::Rational::fraction(1, 3).unwrap());
        let source = TriangleMesh::new(
            vec![
                Point3::new(Real::zero(), Real::zero(), Real::zero()),
                Point3::new(one_third.clone(), Real::zero(), Real::zero()),
                Point3::new(Real::zero(), one_third.clone(), Real::zero()),
            ],
            vec![Triangle::new(0, 1, 2)],
        );
        let scene = triangle_mesh(&source, Color3::RED).unwrap();

        assert_eq!(scene.triangle_count(), 1);
        assert_eq!(scene.vertices()[1].position.x, one_third);
        assert_eq!(scene.to_render_vertices64().unwrap().len(), 3);
    }

    #[test]
    fn native_triangle_mesh_rejects_bad_source_indices() {
        let source = TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 0)]);

        assert!(matches!(
            triangle_mesh(&source, Color3::RED),
            Err(Error::SourceTriangleIndexOutOfBounds {
                triangle: 0,
                corner: 1,
                index: 1,
                vertex_count: 1,
            })
        ));
    }

    #[test]
    fn grid_keeps_exact_vertex_count() {
        let color = Color3::new(0.2, 0.2, 0.2).unwrap();
        let mesh = grid_mesh(2, Real::from(5), color).unwrap();
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
        let context = TriangulationContext::new(PredicatePolicy::STRICT);
        let mesh = polygon_surface_mesh(&context, &vertices, &[], Real::zero(), color)
            .unwrap()
            .into_value();
        assert_eq!(mesh.primitive(), Primitive::Triangles);
        assert_eq!(mesh.triangle_count(), 2);
    }
}
