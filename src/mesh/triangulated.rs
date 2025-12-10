use crate::mesh::Mesh;
use crate::triangulated::{TriVertex, Triangulated3D};

impl<S: Clone + Send + Sync + std::fmt::Debug> Triangulated3D for Mesh<S> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([TriVertex; 3]),
    {
        for poly in &self.polygons {
            let triangles = poly.triangulate();
            let normal = poly.plane.normal().normalize();
            for tri in triangles {
                f([
                    TriVertex { position: tri[0].pos, normal },
                    TriVertex { position: tri[1].pos, normal },
                    TriVertex { position: tri[2].pos, normal },
                ]);
            }
        }
    }
}
