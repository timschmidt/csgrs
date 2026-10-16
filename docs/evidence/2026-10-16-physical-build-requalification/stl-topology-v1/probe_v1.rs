use csgrs::{GeometryContext, Real, TriangleMesh, io::stl, solid};
use hypermesh::Triangle;

fn encode(mesh: &TriangleMesh, ascii: bool) -> Vec<u8> {
    if ascii {
        stl::to_stl_ascii(mesh, "topology").unwrap().into_bytes()
    } else {
        stl::to_stl_binary(mesh, "topology").unwrap()
    }
}

fn duplicate_face_is_rejected(ascii: bool) {
    let cube = solid::cube(Real::one()).unwrap();
    let mut triangles = vec![cube.triangles[0]];
    triangles.extend(cube.triangles.iter().copied());
    let invalid = TriangleMesh::new(cube.positions.to_vec(), triangles);
    assert!(
        invalid
            .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
            .is_err()
    );
    assert!(
        stl::from_stl(&encode(&invalid, ascii)).is_err(),
        "ASCII={ascii}: one face contributes three unmatched directed edges"
    );
}

#[test]
fn reversed_face_remains_invalid() {
    let cube = solid::cube(Real::one()).unwrap();
    let mut triangles = cube.triangles.to_vec();
    let [a, b, c] = triangles[0].indices();
    triangles[0] = Triangle::new(a, c, b);
    let invalid = TriangleMesh::new(cube.positions.to_vec(), triangles);
    assert!(
        invalid
            .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
            .is_err()
    );
    for ascii in [false, true] {
        assert!(stl::from_stl(&encode(&invalid, ascii)).is_err());
    }
}

#[test]
fn balanced_double_winding_preserves_exact_volume() {
    let cube = solid::cube(Real::one()).unwrap();
    let triangles = cube
        .triangles
        .iter()
        .chain(cube.triangles.iter())
        .copied()
        .collect();
    let doubled = TriangleMesh::new(cube.positions.to_vec(), triangles);
    for ascii in [false, true] {
        let restored = stl::from_stl(&encode(&doubled, ascii)).unwrap();
        restored
            .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
            .unwrap();
        let mut volume = Real::zero();
        for triangle in restored.triangles.iter() {
            let points = triangle.indices().map(|i| &restored.positions[i]);
            for p in points {
                for v in [&p.x, &p.y, &p.z] {
                    assert!(v == &Real::zero() || v == &Real::one());
                }
            }
            assert!((0..3).any(|axis| {
                let values = points.map(|p| [&p.x, &p.y, &p.z][axis]);
                values.iter().all(|v| *v == &Real::zero())
                    || values.iter().all(|v| *v == &Real::one())
            }));
            let [a, b, c] = points.map(|p| p.to_vector());
            volume += a.dot(&b.cross(&c));
        }
        assert_eq!(volume, Real::from(12));
    }
}

#[test]
fn binary_unmatched_duplicate_face_is_rejected() {
    duplicate_face_is_rejected(false);
}

#[test]
fn ascii_unmatched_duplicate_face_is_rejected() {
    duplicate_face_is_rejected(true);
}
