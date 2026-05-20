//! [`Triangulated3D`] implementation for [`Mesh`].

use crate::float_types::hunit_vector3;
use crate::mesh::Mesh;
use crate::triangulated::{IndexedTriangulated3D, Triangulated3D};
use crate::vertex::Vertex;
use nalgebra::Vector3;

impl<M: Clone + Send + Sync + std::fmt::Debug> Triangulated3D for Mesh<M> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for poly in &self.polygons {
            let triangles = poly.triangulate();
            let normal = hunit_vector3(&poly.plane.normal()).unwrap_or_else(Vector3::z);
            for tri in triangles {
                f([
                    Vertex {
                        position: tri[0].position,
                        normal,
                    },
                    Vertex {
                        position: tri[1].position,
                        normal,
                    },
                    Vertex {
                        position: tri[2].position,
                        normal,
                    },
                ]);
            }
        }
    }
}

impl<M: Clone + Send + Sync + std::fmt::Debug> IndexedTriangulated3D for Mesh<M> {}
