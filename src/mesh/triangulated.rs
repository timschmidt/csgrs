//! [`Triangulated3D`] implementation for [`Mesh`].

use crate::mesh::Mesh;
use crate::triangulated::{IndexedTriangulated3D, Triangulated3D};
use crate::vertex::Vertex;
use hyperlattice::Vector3;

impl<M: Clone + Send + Sync + std::fmt::Debug> Triangulated3D for Mesh<M> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for poly in &self.polygons {
            let triangles = poly.triangulate();
            let normal = poly
                .plane
                .normal()
                .normalize_checked()
                .unwrap_or_else(|_| Vector3::z());
            for tri in triangles {
                f([
                    Vertex {
                        position: tri[0].position.clone(),
                        normal: normal.clone(),
                    },
                    Vertex {
                        position: tri[1].position.clone(),
                        normal: normal.clone(),
                    },
                    Vertex {
                        position: tri[2].position.clone(),
                        normal: normal.clone(),
                    },
                ]);
            }
        }
    }
}

impl<M: Clone + Send + Sync + std::fmt::Debug> IndexedTriangulated3D for Mesh<M> {}
