//! [`Triangulated3D`] implementation for [`Mesh`].

use crate::mesh::Mesh;
use crate::triangulated::{IndexedTriangleMesh3D, IndexedTriangulated3D, Triangulated3D};
use crate::vertex::Vertex;
use hyperlattice::Vector3;
use std::collections::HashMap;

impl<M: Clone + Send + Sync + std::fmt::Debug> Triangulated3D for Mesh<M> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for poly in &self.polygons {
            let triangles = poly.triangulate();
            let normal = poly.plane.unit_hreal_normal().unwrap_or_else(Vector3::z);
            for tri in triangles {
                f([
                    Vertex::new(tri[0].position.clone(), normal.clone()),
                    Vertex::new(tri[1].position.clone(), normal.clone()),
                    Vertex::new(tri[2].position.clone(), normal.clone()),
                ]);
            }
        }
    }

    fn visit_triangle_facets<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3], Option<Vector3>),
    {
        for poly in &self.polygons {
            let normal = poly.plane.unit_hreal_normal();
            for triangle in poly.triangulate() {
                f(triangle, normal.clone());
            }
        }
    }
}

impl<M: Clone + Send + Sync + std::fmt::Debug> IndexedTriangulated3D for Mesh<M> {
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let position_capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices().len())
            .sum();
        let triangle_capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices().len().saturating_sub(2))
            .sum();
        let mut positions = Vec::with_capacity(position_capacity);
        let mut normals = Vec::with_capacity(self.polygons.len());
        let mut faces = Vec::with_capacity(triangle_capacity);
        let mut position_indices = HashMap::<u64, usize>::with_capacity(position_capacity);
        let mut plane_normals =
            HashMap::<u64, Vec<(Vector3, usize)>>::with_capacity(self.polygons.len());

        for polygon in &self.polygons {
            let triangles = polygon.triangulate_indices();
            if triangles.is_empty() {
                continue;
            }
            let normal = polygon.plane.unit_hreal_normal().unwrap_or_else(Vector3::z);
            let normal_index = plane_normals
                .get(&polygon.plane_id)
                .and_then(|variants| {
                    variants
                        .iter()
                        .find(|(existing, _)| existing == &normal)
                        .map(|(_, index)| *index)
                })
                .unwrap_or_else(|| {
                    let index = normals.len();
                    normals.push(normal.clone());
                    plane_normals
                        .entry(polygon.plane_id)
                        .or_default()
                        .push((normal, index));
                    index
                });

            for triangle in triangles {
                let mut face = [(0usize, normal_index); 3];
                for (slot, vertex_index) in triangle.into_iter().enumerate() {
                    let vertex = &polygon.vertices[vertex_index];
                    let position_index =
                        *position_indices.entry(vertex.position_id).or_insert_with(|| {
                            let index = positions.len();
                            positions.push(vertex.position.clone());
                            index
                        });
                    face[slot] = (position_index, normal_index);
                }
                faces.push(face);
            }
        }

        IndexedTriangleMesh3D {
            positions,
            normals,
            faces,
        }
    }
}
