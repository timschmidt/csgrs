//! Legacy polygon collection wrapper for shape-like triangle sets.

use crate::polygon::Polygon;
use crate::triangulated::{IndexedTriangulated3D, Triangulated3D};
use crate::vertex::Vertex;

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

}

/// So all triangle-based IO backends work on Shapes too.
impl<M: Clone + Send + Sync + Debug> Triangulated3D for Shapes<M> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for poly in &self.polygons {
            for tri in poly.triangulate() {
                // `Polygon::triangulate` keeps the hyper-backed winding and
                // normal decisions on the polygon path.
                f(tri);
            }
        }
    }
}

impl<M: Clone + Send + Sync + Debug> IndexedTriangulated3D for Shapes<M> {}

#[cfg(feature = "mesh")]
impl<M: Clone + Send + Sync + Debug> From<Shapes<M>> for crate::mesh::Mesh<M> {
    fn from(shapes: Shapes<M>) -> Self {
        crate::mesh::Mesh::from_polygons(&shapes.polygons, shapes.metadata)
    }
}
