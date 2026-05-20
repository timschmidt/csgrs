/// Traits for shapes which can be represented by triangles.
use crate::float_types::{Real, hpoints_within_epsilon, hvectors_within_epsilon, tolerance};
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
/// it deduplicates finite boundary positions/normals through the crate's
/// hyperreal-backed tolerance predicates. Exact mesh carriers should override
/// this method and preserve their native indexed topology instead. This keeps
/// legacy triangle exporters generic while allowing hyper geometry types to
/// avoid reconstructing topology from approximate triangle streams, following
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), <https://doi.org/10.1016/0925-7721(95)00040-2>.
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
        if hpoints_within_epsilon(existing, &position, tolerance()) {
            return index;
        }
    }
    positions.push(position);
    positions.len() - 1
}

fn add_unique_normal(normals: &mut Vec<Vector3<Real>>, normal: Vector3<Real>) -> usize {
    for (index, existing) in normals.iter().enumerate() {
        if hvectors_within_epsilon(existing, &normal, tolerance()) {
            return index;
        }
    }
    normals.push(normal);
    normals.len() - 1
}
