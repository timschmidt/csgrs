use crate::bmesh::BMesh;
use crate::float_types::Real;
use nalgebra::{Point3, Vector3};
use crate::triangulated::{TriVertex, Triangulated3D};

impl<S: Clone + Send + Sync + std::fmt::Debug> Triangulated3D for BMesh<S> {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([TriVertex; 3]),
    {
        let Some(m) = &self.manifold else { return; };

        // Manifold has `ps` (points) and `hs` (half-edges). Triangles are 3 half-edges per face.
        for face_idx in 0..m.nf {
            let base = face_idx * 3;
            let v_idx = [
                m.hs[base].tail,
                m.hs[base + 1].tail,
                m.hs[base + 2].tail,
            ];

            let p: [Point3<Real>; 3] = v_idx.map(|i| {
                let v = &m.ps[i];
                Point3::new(v.x as Real, v.y as Real, v.z as Real)
            });

            let n = {
                let e1 = p[1] - p[0];
                let e2 = p[2] - p[0];
                Vector3::new(e1.x, e1.y, e1.z)
                    .cross(&Vector3::new(e2.x, e2.y, e2.z))
                    .normalize()
            };

            f([
                TriVertex { position: p[0], normal: n },
                TriVertex { position: p[1], normal: n },
                TriVertex { position: p[2], normal: n },
            ]);
        }
    }
}
