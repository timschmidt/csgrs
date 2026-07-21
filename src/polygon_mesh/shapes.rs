//! Planar-faced shape constructors for [`PolygonMesh`](super::PolygonMesh).

use std::cmp::Ordering;
use std::fmt::Debug;

use hyperlattice::{Point3, Real, Vector3};

use crate::mesh::Polygon;
use crate::vertex::Vertex;

use super::PolygonMesh;

const CUBOID_FACES: [[usize; 4]; 6] = [
    [0, 3, 2, 1],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [3, 7, 6, 2],
    [0, 4, 7, 3],
    [1, 2, 6, 5],
];

impl<M: Clone + Debug + Send + Sync> PolygonMesh<M> {
    /// Construct a six-face planar cuboid from the origin to
    /// `(width, length, height)`.
    pub fn cuboid(width: Real, length: Real, height: Real, metadata: M) -> Self {
        if ![&width, &length, &height].into_iter().all(scalar_positive) {
            return Self::empty();
        }

        let zero = Real::zero();
        let positions = [
            Point3::new(zero.clone(), zero.clone(), zero.clone()),
            Point3::new(width.clone(), zero.clone(), zero.clone()),
            Point3::new(width.clone(), length.clone(), zero.clone()),
            Point3::new(zero.clone(), length.clone(), zero),
            Point3::new(Real::zero(), Real::zero(), height.clone()),
            Point3::new(width.clone(), Real::zero(), height.clone()),
            Point3::new(width, length.clone(), height.clone()),
            Point3::new(Real::zero(), length, height),
        ];
        let normals = [
            -Vector3::z(),
            Vector3::z(),
            -Vector3::y(),
            Vector3::y(),
            -Vector3::x(),
            Vector3::x(),
        ];
        let polygons = CUBOID_FACES
            .into_iter()
            .zip(normals)
            .map(|(face, normal)| {
                Polygon::new(
                    face.into_iter()
                        .map(|index| Vertex::new(positions[index].clone(), normal.clone()))
                        .collect(),
                    metadata.clone(),
                )
            })
            .collect();
        Self::from_polygons(polygons)
    }

    /// Construct a six-face planar cube from the origin.
    pub fn cube(width: Real, metadata: M) -> Self {
        Self::cuboid(width.clone(), width.clone(), width, metadata)
    }
}

fn scalar_positive(value: &Real) -> bool {
    if let Some(value) = value.exact_rational_ref() {
        return value.is_positive();
    }
    matches!(
        hyperlimit::compare_reals(value, &Real::zero()).value(),
        Some(Ordering::Greater)
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polygon_cuboid_retains_six_quads_and_converts_to_twelve_triangles() {
        let polygon_mesh =
            PolygonMesh::cuboid(Real::from(2), Real::from(3), Real::from(5), "cuboid");

        assert_eq!(polygon_mesh.polygons.len(), 6);
        assert!(
            polygon_mesh
                .polygons
                .iter()
                .all(|polygon| polygon.vertices().len() == 4)
        );

        let mesh = polygon_mesh.triangulate();
        assert_eq!(mesh.polygons.len(), 12);
        assert!(mesh.polygons.iter().all(|triangle| {
            triangle.vertices().len() == 3 && triangle.metadata() == &"cuboid"
        }));
    }

    #[test]
    fn polygon_cuboid_rejects_nonpositive_dimensions() {
        assert!(
            PolygonMesh::cuboid(Real::zero(), Real::from(3), Real::from(5), ())
                .polygons
                .is_empty()
        );
        assert!(PolygonMesh::cube(Real::from(-1), ()).polygons.is_empty());
    }
}
