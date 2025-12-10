use crate::mesh::Mesh;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;

impl<S: Clone + Send + Sync + std::fmt::Debug> Triangulated3D for Mesh<S> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for poly in &self.polygons {
            let triangles = poly.triangulate();
            let normal = poly.plane.normal().normalize();
            for tri in triangles {
                f([
                    Vertex { position: tri[0].position, normal },
                    Vertex { position: tri[1].position, normal },
                    Vertex { position: tri[2].position, normal },
                ]);
            }
        }
    }
}
