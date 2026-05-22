/// Traits for shapes which can be represented by triangles.
use crate::float_types::{Real, hpoints_exactly_equal, hvectors_exactly_equal};
use crate::vertex::Vertex;
use nalgebra::{Point3, Vector3};

/// A triangulated 3D surface.
///
/// Anything that can present itself as a bunch of triangles in 3D
/// can automatically use all the triangle-based IO backends.
pub trait Triangulated3D {
    /// Call `f` for each triangle.
    ///
    /// The triangle is `[v0, v1, v2]` with positions+normals.
    fn visit_triangles<F>(&self, f: F)
    where
        F: FnMut([Vertex; 3]);
}

/// Indexed triangle view with position rows, normal rows, and face rows.
///
/// The default implementation is a compatibility path over [`Triangulated3D`]:
/// it deduplicates finite boundary positions/normals only when promotion to
/// hyperreal-backed vectors proves exact equality. Exact mesh carriers should
/// override this method and preserve their native indexed topology instead.
/// This keeps legacy triangle exporters generic while avoiding tolerance-based
/// topology reconstruction from approximate triangle streams, following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), <https://doi.org/10.1016/0925-7721(95)00040-2>.
#[derive(Clone, Debug, PartialEq)]
pub struct IndexedTriangleMesh3D {
    /// Position rows.
    pub positions: Vec<Point3<Real>>,
    /// Normal rows.
    pub normals: Vec<Vector3<Real>>,
    /// Triangle rows as `(position_index, normal_index)` tuples.
    pub faces: Vec<[(usize, usize); 3]>,
}

/// A shape that can present indexed triangle rows for exporters and adapters.
pub trait IndexedTriangulated3D: Triangulated3D {
    /// Build indexed triangle buffers.
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let mut positions = Vec::<Point3<Real>>::new();
        let mut normals = Vec::<Vector3<Real>>::new();
        let mut faces = Vec::<[(usize, usize); 3]>::new();

        self.visit_triangles(|tri| {
            let mut face = [(0usize, 0usize); 3];
            for (slot, vertex) in tri.into_iter().enumerate() {
                let position = add_unique_position(&mut positions, vertex.position);
                let normal = add_unique_normal(&mut normals, vertex.normal);
                face[slot] = (position, normal);
            }
            faces.push(face);
        });

        IndexedTriangleMesh3D {
            positions,
            normals,
            faces,
        }
    }
}

fn add_unique_position(positions: &mut Vec<Point3<Real>>, position: Point3<Real>) -> usize {
    for (index, existing) in positions.iter().enumerate() {
        if hpoints_exactly_equal(existing, &position) {
            return index;
        }
    }
    positions.push(position);
    positions.len() - 1
}

fn add_unique_normal(normals: &mut Vec<Vector3<Real>>, normal: Vector3<Real>) -> usize {
    for (index, existing) in normals.iter().enumerate() {
        if hvectors_exactly_equal(existing, &normal) {
            return index;
        }
    }
    normals.push(normal);
    normals.len() - 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone)]
    struct TriangleSoup(Vec<[Vertex; 3]>);

    impl Triangulated3D for TriangleSoup {
        fn visit_triangles<F>(&self, mut f: F)
        where
            F: FnMut([Vertex; 3]),
        {
            for triangle in &self.0 {
                f(*triangle);
            }
        }
    }

    impl IndexedTriangulated3D for TriangleSoup {}

    #[test]
    fn indexed_triangles_deduplicate_only_exact_hyperreal_rows() {
        let normal = Vector3::z();
        let origin = Vertex::new(Point3::origin(), normal);
        let exact_repeat = Vertex::new(Point3::origin(), normal);
        let near = Vertex::new(Point3::new(1.0e-12, 0.0, 0.0), normal);
        let top = Vertex::new(Point3::new(0.0, 1.0, 0.0), normal);

        let indexed = TriangleSoup(vec![[origin, exact_repeat, near], [origin, near, top]])
            .indexed_triangles();

        assert_eq!(indexed.positions.len(), 3);
        assert_eq!(indexed.faces[0][0].0, indexed.faces[0][1].0);
        assert_ne!(indexed.faces[0][0].0, indexed.faces[0][2].0);
        assert_eq!(indexed.faces[0][2].0, indexed.faces[1][1].0);
        assert_eq!(indexed.normals.len(), 1);
    }
}
