#![allow(dead_code)]

use std::collections::BTreeMap;

use hypermesh::{
    BooleanMeshBatch, BooleanMeshResult, BooleanOp, HypermeshResult, MeshContext, MeshOutcome,
    Point3, PredicatePolicy, Real, Triangle, TriangleMesh, boolean_mesh_closure_evidence,
};
use hyperreal::{Rational, StructuralKind};

pub const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

pub fn value<T>(result: HypermeshResult<MeshOutcome<T>>) -> HypermeshResult<T> {
    result.map(MeshOutcome::into_value)
}

pub struct Bytes<'a> {
    data: &'a [u8],
    offset: usize,
}

impl<'a> Bytes<'a> {
    pub const fn new(data: &'a [u8]) -> Self {
        Self { data, offset: 0 }
    }

    pub fn next(&mut self) -> u8 {
        let value = self.data.get(self.offset).copied().unwrap_or(0);
        self.offset = self.offset.saturating_add(1);
        value
    }

    pub fn bounded_i64(&mut self, magnitude: u8) -> i64 {
        i64::from(self.next() % (magnitude.saturating_mul(2).saturating_add(1)))
            - i64::from(magnitude)
    }

    pub fn positive_i64(&mut self, maximum: u8) -> i64 {
        i64::from(self.next() % maximum) + 1
    }
}

pub fn r(value: i64) -> Real {
    Real::from(value)
}

pub fn representative_hyperreal_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    let values = vec![
        Real::new(Rational::fraction(3, 2).unwrap()),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2)).sqrt().unwrap(),
        Real::new(Rational::new(3)).ln().unwrap(),
        Real::new(Rational::fraction(1, 5).unwrap()).sin_pi(),
        pi_squared * Real::e(),
        Real::new(Rational::one()).sin(),
    ];
    let expected = [
        StructuralKind::ExactRational,
        StructuralKind::PiLike,
        StructuralKind::ExpLike,
        StructuralKind::SqrtLike,
        StructuralKind::LogLike,
        StructuralKind::TrigExact,
        StructuralKind::ProductConstant,
        StructuralKind::ComputableOpaque,
    ];
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        expected,
    );
    values
}

pub fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

pub fn operation(value: u8) -> BooleanOp {
    match value % 4 {
        0 => BooleanOp::Union,
        1 => BooleanOp::Intersection,
        2 => BooleanOp::Difference,
        _ => BooleanOp::SymmetricDifference,
    }
}

pub fn box_mesh(min: [i64; 3], max: [i64; 3]) -> TriangleMesh {
    TriangleMesh::new(
        vec![
            p(min[0], min[1], min[2]),
            p(max[0], min[1], min[2]),
            p(max[0], max[1], min[2]),
            p(min[0], max[1], min[2]),
            p(min[0], min[1], max[2]),
            p(max[0], min[1], max[2]),
            p(max[0], max[1], max[2]),
            p(min[0], max[1], max[2]),
        ],
        vec![
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            Triangle::new(0, 3, 2),
            Triangle::new(0, 2, 1),
            Triangle::new(1, 2, 6),
            Triangle::new(1, 6, 5),
            Triangle::new(0, 4, 7),
            Triangle::new(0, 7, 3),
            Triangle::new(3, 7, 6),
            Triangle::new(3, 6, 2),
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
        ],
    )
}

pub fn tetrahedron(origin: [i64; 3], extent: [i64; 3]) -> TriangleMesh {
    let [x, y, z] = origin;
    TriangleMesh::new(
        vec![
            p(x, y, z),
            p(x + extent[0], y, z),
            p(x, y + extent[1], z),
            p(x, y, z + extent[2]),
        ],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

pub fn octahedron(center: [i64; 3], radius: [i64; 3]) -> TriangleMesh {
    let [x, y, z] = center;
    TriangleMesh::new(
        vec![
            p(x + radius[0], y, z),
            p(x - radius[0], y, z),
            p(x, y + radius[1], z),
            p(x, y - radius[1], z),
            p(x, y, z + radius[2]),
            p(x, y, z - radius[2]),
        ],
        vec![
            Triangle::new(0, 2, 4),
            Triangle::new(2, 1, 4),
            Triangle::new(1, 3, 4),
            Triangle::new(3, 0, 4),
            Triangle::new(2, 0, 5),
            Triangle::new(1, 2, 5),
            Triangle::new(3, 1, 5),
            Triangle::new(0, 3, 5),
        ],
    )
}

pub fn convex_mesh(bytes: &mut Bytes<'_>) -> TriangleMesh {
    let kind = bytes.next() % 3;
    let origin = [
        bytes.bounded_i64(5),
        bytes.bounded_i64(5),
        bytes.bounded_i64(5),
    ];
    let extent = [
        bytes.positive_i64(4),
        bytes.positive_i64(4),
        bytes.positive_i64(4),
    ];
    let mesh = match kind {
        0 => box_mesh(
            origin,
            [
                origin[0] + extent[0],
                origin[1] + extent[1],
                origin[2] + extent[2],
            ],
        ),
        1 => tetrahedron(origin, extent),
        _ => octahedron(origin, extent),
    };
    if !mesh.triangles.is_empty() {
        let mut triangles = mesh.triangles.to_vec();
        let triangle_count = triangles.len();
        triangles.rotate_left(usize::from(bytes.next()) % triangle_count);
        return TriangleMesh::new(mesh.positions.to_vec(), triangles);
    }
    mesh
}

pub fn combine_meshes(meshes: &[TriangleMesh]) -> TriangleMesh {
    let position_count = meshes.iter().map(|mesh| mesh.positions.len()).sum();
    let triangle_count = meshes.iter().map(|mesh| mesh.triangles.len()).sum();
    let mut positions = Vec::with_capacity(position_count);
    let mut triangles = Vec::with_capacity(triangle_count);
    for mesh in meshes {
        let base = positions.len();
        positions.extend(mesh.positions.iter().cloned());
        triangles.extend(mesh.triangles.iter().map(|triangle| {
            Triangle::new(base + triangle.v0, base + triangle.v1, base + triangle.v2)
        }));
    }
    TriangleMesh::new(positions, triangles)
}

pub fn subdivide_once(mesh: TriangleMesh) -> TriangleMesh {
    let mut positions = mesh.positions.to_vec();
    let mut edge_midpoints = BTreeMap::new();
    let mut triangles = Vec::with_capacity(mesh.triangles.len() * 4);
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        let mut midpoint = |left: usize, right: usize| {
            let key = (left.min(right), left.max(right));
            *edge_midpoints.entry(key).or_insert_with(|| {
                let left = &positions[left];
                let right = &positions[right];
                let two = r(2);
                let point = Point3::new(
                    ((&left.x + &right.x) / &two).expect("two is nonzero"),
                    ((&left.y + &right.y) / &two).expect("two is nonzero"),
                    ((&left.z + &right.z) / &two).expect("two is nonzero"),
                );
                let index = positions.len();
                positions.push(point);
                index
            })
        };
        let ab = midpoint(a, b);
        let bc = midpoint(b, c);
        let ca = midpoint(c, a);
        triangles.extend([
            Triangle::new(a, ab, ca),
            Triangle::new(ab, b, bc),
            Triangle::new(ca, bc, c),
            Triangle::new(ab, bc, ca),
        ]);
    }
    TriangleMesh::new(positions, triangles)
}

pub fn signed_volume_numerator(vertices: &[Point3], result: &BooleanMeshResult) -> Real {
    let mut volume = Real::zero();
    for triangle in &result.triangles {
        let v0 = &vertices[triangle[0] as usize];
        let v1 = &vertices[triangle[1] as usize];
        let v2 = &vertices[triangle[2] as usize];
        volume += &v0.x * &((&v1.y * &v2.z) - (&v1.z * &v2.y))
            + &v0.y * &((&v1.z * &v2.x) - (&v1.x * &v2.z))
            + &v0.z * &((&v1.x * &v2.y) - (&v1.y * &v2.x));
    }
    volume
}

pub fn volume_numerator(vertices: &[Point3], result: &BooleanMeshResult) -> Real {
    signed_volume_numerator(vertices, result).abs()
}

pub fn validate_batch(batch: &BooleanMeshBatch) {
    batch.validate().unwrap();
    for result in &batch.results {
        assert!(
            result
                .sources
                .iter()
                .all(|source| matches!(source.orientation, -1 | 1))
        );
        assert!(boolean_mesh_closure_evidence(result).has_no_boundary());
        let mesh = TriangleMesh::new(
            batch.vertices.clone(),
            result
                .triangles
                .iter()
                .map(|triangle| {
                    Triangle::new(
                        triangle[0] as usize,
                        triangle[1] as usize,
                        triangle[2] as usize,
                    )
                })
                .collect(),
        );
        assert!(
            mesh.has_unique_nondegenerate_triangles(&CONTEXT)
                .unwrap()
                .into_value()
        );
    }
}
