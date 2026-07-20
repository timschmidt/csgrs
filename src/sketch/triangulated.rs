//! Triangulation support for converting 2D sketches into 3D triangle streams.

use crate::sketch::Profile;
use crate::triangulated::{IndexedTriangulated3D, Triangulated3D};
use crate::vertex::Vertex;
use hyperlattice::Vector3;

impl Triangulated3D for Profile {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        let triangles_2d = self.triangulate(); // Vec<[Point3; 3]> with z=0
        let normal = Vector3::z();

        for tri in triangles_2d {
            f([
                Vertex::new(tri[0].clone(), normal.clone()),
                Vertex::new(tri[1].clone(), normal.clone()),
                Vertex::new(tri[2].clone(), normal.clone()),
            ]);
        }
    }
}

impl IndexedTriangulated3D for Profile {}
