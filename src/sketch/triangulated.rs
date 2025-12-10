use crate::sketch::Sketch;
use nalgebra::Vector3;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;

impl<S: Clone + Send + Sync + std::fmt::Debug> Triangulated3D for Sketch<S> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        let triangles_2d = self.triangulate(); // Vec<[Point3<Real>; 3]> with z=0
        let normal = Vector3::new(0.0, 0.0, 1.0);

        for tri in triangles_2d {
            f([
                Vertex { position: tri[0], normal },
                Vertex { position: tri[1], normal },
                Vertex { position: tri[2], normal },
            ]);
        }
    }
}
