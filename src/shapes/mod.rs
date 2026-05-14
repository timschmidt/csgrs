//! Legacy polygon collection wrapper for shape-like triangle sets.

use crate::polygon::Polygon;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;

use crate::float_types::Real;
use nalgebra::{Point3, Vector3};

use std::fmt::Debug;

#[derive(Clone, Debug)]
pub struct Shapes<M: Clone + Send + Sync + Debug> {
    pub polygons: Vec<Polygon<M>>,
    pub metadata: M,
}

impl<M: Clone + Send + Sync + Debug> Shapes<M> {
    #[inline]
    pub fn from_polygons(polygons: Vec<Polygon<M>>, metadata: M) -> Self {
        Self { polygons, metadata }
    }

    /// Convenience when `mesh` is enabled.
    #[cfg(feature = "mesh")]
    #[inline]
    pub fn into_mesh(self) -> crate::mesh::Mesh<M> {
        self.into()
    }

    /// Convenience when `bmesh` is enabled.
    #[cfg(feature = "bmesh")]
    #[inline]
    pub fn into_bmesh(self) -> crate::bmesh::BMesh<M> {
        self.into()
    }
}

/// So all triangle-based IO backends work on Shapes too.
impl<M: Clone + Send + Sync + Debug> Triangulated3D for Shapes<M> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for poly in &self.polygons {
            for tri in poly.triangulate() {
                // tri already contains positions+normals; keep normals flat-consistent if you want:
                // let n = poly.plane.normal().normalize();
                // f([Vertex{position: tri[0].position, normal: n}, ...]);
                f(tri);
            }
        }
    }
}

#[cfg(feature = "mesh")]
impl<M: Clone + Send + Sync + Debug> From<Shapes<M>> for crate::mesh::Mesh<M> {
    fn from(shapes: Shapes<M>) -> Self {
        crate::mesh::Mesh::from_polygons(&shapes.polygons, shapes.metadata)
    }
}

#[cfg(feature = "bmesh")]
impl<M: Clone + Send + Sync + Debug> From<Shapes<M>> for crate::bmesh::BMesh<M> {
    fn from(shapes: Shapes<M>) -> Self {
        // Build a triangle soup (duplicated vertices is OK; boolmesh will accept it).
        let mut points: Vec<Point3<Real>> = Vec::new();
        let mut indices: Vec<usize> = Vec::new();

        shapes.visit_triangles(|t| {
            let base = points.len();
            points.push(t[0].position);
            points.push(t[1].position);
            points.push(t[2].position);
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 2);
        });

        let metadata = shapes.metadata.clone();

        if indices.is_empty() {
            return crate::bmesh::BMesh::empty(metadata);
        }

        let mut pos_bool: Vec<boolmesh::Real> = Vec::with_capacity(points.len() * 3);
        for p in &points {
            pos_bool.push(p.x as boolmesh::Real);
            pos_bool.push(p.y as boolmesh::Real);
            pos_bool.push(p.z as boolmesh::Real);
        }

        let manifold = boolmesh::Manifold::new(&pos_bool, &indices)
            .expect("boolmesh::Manifold::new failed when converting from Shapes");

        crate::bmesh::BMesh::from_manifold(manifold, metadata)
    }
}
