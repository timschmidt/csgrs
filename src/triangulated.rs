/// Traits for shapes which can be represented by triangles.
use crate::vertex::Vertex;
use hyperlattice::{Point3, Vector3};

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

    /// Visit triangles together with their exact checked facet normal.
    ///
    /// Implementations with retained support planes can override this to
    /// reuse one certified normal across every triangle of a polygon. The
    /// default keeps triangle-soup implementations correct by deriving the
    /// normal from each triangle's positions.
    fn visit_triangle_facets<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3], Option<Vector3>),
    {
        self.visit_triangles(|triangle| {
            let first = &triangle[1].position - &triangle[0].position;
            let second = &triangle[2].position - &triangle[0].position;
            let normal = first.unit_cross_checked(&second).ok();
            f(triangle, normal);
        });
    }
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
    pub positions: Vec<Point3>,
    /// Normal rows.
    pub normals: Vec<Vector3>,
    /// Triangle rows as `(position_index, normal_index)` tuples.
    pub faces: Vec<[(usize, usize); 3]>,
}

/// A shape that can present indexed triangle rows for exporters and adapters.
pub trait IndexedTriangulated3D: Triangulated3D {
    /// Build indexed triangle buffers.
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let mut positions = Vec::<Point3>::new();
        let mut normals = Vec::<Vector3>::new();
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

fn add_unique_position(positions: &mut Vec<Point3>, position: Point3) -> usize {
    for (index, existing) in positions.iter().enumerate() {
        let existing_point = hyperlimit::Point3::new(
            existing.x.clone(),
            existing.y.clone(),
            existing.z.clone(),
        );
        let position_point = hyperlimit::Point3::new(
            position.x.clone(),
            position.y.clone(),
            position.z.clone(),
        );
        if matches!(
            hyperlimit::point3_equal(&existing_point, &position_point).value(),
            Some(true)
        ) {
            return index;
        }
    }
    positions.push(position);
    positions.len() - 1
}

fn add_unique_normal(normals: &mut Vec<Vector3>, normal: Vector3) -> usize {
    for (index, existing) in normals.iter().enumerate() {
        let existing_normal = hyperlimit::Point3::new(
            existing.0[0].clone(),
            existing.0[1].clone(),
            existing.0[2].clone(),
        );
        let normal_point = hyperlimit::Point3::new(
            normal.0[0].clone(),
            normal.0[1].clone(),
            normal.0[2].clone(),
        );
        if matches!(
            hyperlimit::point3_equal(&existing_normal, &normal_point).value(),
            Some(true)
        ) {
            return index;
        }
    }
    normals.push(normal);
    normals.len() - 1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::{Real, hreal_from_f64};

    #[derive(Clone)]
    struct TriangleSoup(Vec<[Vertex; 3]>);

    impl Triangulated3D for TriangleSoup {
        fn visit_triangles<F>(&self, mut f: F)
        where
            F: FnMut([Vertex; 3]),
        {
            for triangle in &self.0 {
                f(triangle.clone());
            }
        }
    }

    impl IndexedTriangulated3D for TriangleSoup {}

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn indexed_triangles_deduplicate_only_exact_hyperreal_rows() {
        let normal = Vector3::z();
        let origin = Vertex::new(Point3::origin(), normal.clone());
        let exact_repeat = Vertex::new(Point3::origin(), normal.clone());
        let near = Vertex::new(p3(1.0e-12, 0.0, 0.0), normal.clone());
        let top = Vertex::new(p3(0.0, 1.0, 0.0), normal);

        let indexed = TriangleSoup(vec![
            [origin.clone(), exact_repeat, near.clone()],
            [origin, near, top],
        ])
        .indexed_triangles();

        assert_eq!(indexed.positions.len(), 3);
        assert_eq!(indexed.faces[0][0].0, indexed.faces[0][1].0);
        assert_ne!(indexed.faces[0][0].0, indexed.faces[0][2].0);
        assert_eq!(indexed.faces[0][2].0, indexed.faces[1][1].0);
        assert_eq!(indexed.normals.len(), 1);
    }
}
