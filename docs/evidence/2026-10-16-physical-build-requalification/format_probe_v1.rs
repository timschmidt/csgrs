use csgrs::{
    GeometryContext, Real, TriangleMesh,
    io::{IoError, obj, stl},
    solid::{self, SolidExt},
};
use hyperlattice::Point3;
use hypermesh::{MeshCertainty, Triangle};

fn size(exponent: i32) -> Real {
    Real::try_from(2_f64.powi(exponent)).unwrap()
}

fn assert_box(mesh: &TriangleMesh, side: &Real) {
    let context = GeometryContext::STRICT.mesh_context();
    assert_eq!(
        mesh.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    assert!(mesh.is_closed_manifold_geometry(&context).unwrap().value);
    let zero = Real::zero();
    let mut six_volume = Real::zero();
    for triangle in mesh.triangles.iter() {
        let points = triangle.indices().map(|i| &mesh.positions[i]);
        for point in points {
            for coordinate in [&point.x, &point.y, &point.z] {
                assert!(coordinate >= &zero && coordinate <= side);
            }
        }
        assert!((0..3).any(|axis| {
            let values = points.map(|p| [&p.x, &p.y, &p.z][axis]);
            values.iter().all(|v| *v == &zero) || values.iter().all(|v| *v == side)
        }));
        let [a, b, c] = points.map(|p| p.to_vector());
        six_volume += a.dot(&b.cross(&c));
    }
    let expected = Real::from(6) * side.clone() * side.clone() * side.clone();
    assert_eq!(
        six_volume.certified_eq_until(&expected, -1024).as_bool(),
        Some(true)
    );
}

fn stl_composition(ascii: bool, side: Real) {
    let original = solid::cube(side.clone()).unwrap();
    assert_box(&original, &side);
    let bytes = if ascii {
        stl::to_stl_ascii(&original, "box").unwrap().into_bytes()
    } else {
        stl::to_stl_binary(&original, "box").unwrap()
    };
    let restored = stl::from_stl(&bytes).expect("representable closed cube must reimport");
    assert_box(&restored, &side);
    assert_box(&restored.try_union(&solid::empty()).unwrap(), &side);
}

#[test]
fn binary_stl_small_representable_cube_composes() {
    stl_composition(false, size(-12));
}

#[test]
fn ascii_stl_small_representable_cube_composes() {
    stl_composition(true, size(-12));
}

#[test]
fn binary_stl_unit_control_composes() {
    stl_composition(false, Real::one());
}

#[test]
fn ascii_stl_unit_control_composes() {
    stl_composition(true, Real::one());
}

#[test]
fn obj_unit_control_composes() {
    let original = solid::cube(Real::one()).unwrap();
    let text = obj::to_obj(&original, "unit").unwrap();
    let restored = obj::from_obj(text.as_bytes()).unwrap();
    assert_box(&restored, &Real::one());
    assert_box(&restored.try_union(&solid::empty()).unwrap(), &Real::one());
}

fn malformed_obj(index: usize) -> TriangleMesh {
    TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, index, 0)])
}

#[test]
fn obj_rejects_missing_vertex_index() {
    assert!(matches!(
        obj::to_obj(&malformed_obj(1), "invalid"),
        Err(IoError::Geometry { .. })
    ));
}

#[test]
fn obj_rejects_overflowing_vertex_index() {
    assert!(matches!(
        obj::to_obj(&malformed_obj(usize::MAX), "invalid"),
        Err(IoError::Geometry { .. }) | Err(IoError::SizeOverflow { .. })
    ));
}

#[test]
fn obj_writer_rejects_invalid_input_without_writing() {
    let mut output = b"existing bytes".to_vec();
    let result = obj::write_obj(&malformed_obj(1), &mut output, "invalid");
    assert_eq!(
        (result.is_err(), output.as_slice()),
        (true, b"existing bytes".as_slice())
    );
}

#[test]
fn obj_small_coordinate_projection_survives_text() {
    let original = solid::cube(size(-70)).unwrap();
    let text = obj::to_obj(&original, "tiny").unwrap();
    let projected: Vec<Vec<f64>> = text
        .lines()
        .filter_map(|line| line.strip_prefix("v "))
        .map(|line| line.split_whitespace().map(|v| v.parse().unwrap()).collect())
        .collect();
    assert_eq!(projected.len(), original.positions.len());
    for (row, point) in projected.iter().zip(original.positions.iter()) {
        assert_eq!(
            row,
            &vec![
                point.x.to_f64_lossy().unwrap(),
                point.y.to_f64_lossy().unwrap(),
                point.z.to_f64_lossy().unwrap()
            ]
        );
    }
    let restored = obj::from_obj(text.as_bytes()).unwrap();
    restored
        .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
}

#[test]
fn ascii_stl_small_coordinate_projection_survives_text() {
    let original = solid::cube(size(-70)).unwrap();
    let text = stl::to_stl_ascii(&original, "tiny").unwrap();
    let projected: Vec<Vec<f64>> = text
        .lines()
        .filter_map(|line| line.trim().strip_prefix("vertex "))
        .map(|line| line.split_whitespace().map(|v| v.parse().unwrap()).collect())
        .collect();
    let expected: Vec<Vec<f64>> = original
        .triangles
        .iter()
        .flat_map(|t| t.indices())
        .map(|i| {
            let p = &original.positions[i];
            vec![
                p.x.to_f64_lossy().unwrap(),
                p.y.to_f64_lossy().unwrap(),
                p.z.to_f64_lossy().unwrap(),
            ]
        })
        .collect();
    assert_eq!(projected, expected);
}

#[test]
fn binary_stl_rejects_faces_collapsed_by_quantization() {
    let mesh = solid::cube(Real::one())
        .unwrap()
        .translated(Real::from(1_u32 << 24), Real::zero(), Real::zero())
        .unwrap();
    mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    match stl::to_stl_binary(&mesh, "unrepresentable faces") {
        Err(IoError::Geometry { .. }) => {},
        other => panic!(
            "binary32 face collapse must fail at export; outcome: {:?}",
            other.map(|bytes| (bytes.len(), stl::from_stl(&bytes)))
        ),
    }
}

#[test]
fn canonical_empty_interchange_control() {
    let mesh = solid::empty();
    let obj = obj::from_obj(obj::to_obj(&mesh, "empty").unwrap().as_bytes()).unwrap();
    let ascii = stl::from_stl(stl::to_stl_ascii(&mesh, "empty").unwrap().as_bytes()).unwrap();
    let binary = stl::from_stl(&stl::to_stl_binary(&mesh, "empty").unwrap()).unwrap();
    for restored in [obj, ascii, binary] {
        assert!(restored.positions.is_empty() && restored.triangles.is_empty());
        assert_box(
            &restored
                .try_union(&solid::cube(Real::one()).unwrap())
                .unwrap(),
            &Real::one(),
        );
    }
}
