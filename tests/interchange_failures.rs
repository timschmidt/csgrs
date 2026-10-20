#![cfg(all(feature = "obj-io", feature = "ply-io", feature = "gltf-io"))]

use base64::Engine;
use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::io::{IoError, gltf, obj, ply};
use csgrs::solid::SolidExt;
use csgrs::{GeometryContext, Real, TriangleMesh, solid};
use hyperlattice::Point3;
use hypermesh::{GpuMeshError, HypermeshError, MeshCertainty, Triangle};
use std::error::Error;
use std::io::{self, Write};
use std::sync::Arc;

fn triangle(height: Real) -> TriangleMesh {
    TriangleMesh::new(
        vec![
            Point3::origin(),
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::new(0.into(), height, 0.into()),
        ],
        vec![Triangle::new(0, 1, 2)],
    )
}

fn normal_fixture() -> TriangleMesh {
    TriangleMesh::new(
        [[0, 0, 0], [1, 0, 0], [0, 3, 4]]
            .map(|[x, y, z]| Point3::new(x.into(), y.into(), z.into()))
            .to_vec(),
        vec![Triangle::new(0, 1, 2)],
    )
}

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

fn failures() -> [(TriangleMesh, ValidationErrorKind); 3] {
    [
        (
            TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]),
            ValidationErrorKind::InvalidInput,
        ),
        (triangle(Real::zero()), ValidationErrorKind::InvalidInput),
        (triangle(unresolved_zero()), ValidationErrorKind::Undecided),
    ]
}

fn assert_gpu(
    error: IoError,
    format: &str,
    expected: &GpuMeshError,
    kind: ValidationErrorKind,
) {
    let source = error
        .source()
        .expect("native cause must survive the format boundary");
    let validation = source.downcast_ref::<ValidationError>().unwrap();
    assert_eq!(validation.kind(), kind);
    assert_eq!(validation, &ValidationError::Gpu(expected.clone()));
    assert_eq!(
        source.source().unwrap().downcast_ref::<GpuMeshError>(),
        Some(expected)
    );
    assert_eq!(
        error.to_string(),
        format!("{format} construction failed: {expected}")
    );
    assert!(matches!(error, IoError::Construction { format: actual, .. } if actual == format));
}

#[test]
fn ply_retains_invalid_degenerate_and_undecided_causes() {
    for (mesh, kind) in failures() {
        let native = mesh.to_exact_gpu_mesh_buffers().unwrap_err();
        for _ in 0..2 {
            assert_gpu(
                ply::to_ply(&mesh, "failure").unwrap_err(),
                "PLY",
                &native,
                kind,
            );
            let mut sink = b"unchanged".to_vec();
            assert_gpu(
                ply::write_ply(&mesh, &mut sink, "failure").unwrap_err(),
                "PLY",
                &native,
                kind,
            );
            assert_eq!(sink, b"unchanged");
        }
    }
}

#[test]
fn gltf_exports_and_scene_objects_retain_native_causes() {
    for (mesh, kind) in failures() {
        let native = mesh.to_exact_gpu_mesh_buffers().unwrap_err();
        for _ in 0..2 {
            assert_gpu(
                gltf::to_gltf(&mesh, "failure").unwrap_err(),
                "glTF",
                &native,
                kind,
            );
            assert_gpu(
                gltf::GltfSceneObject::new("failure", &mesh).err().unwrap(),
                "glTF",
                &native,
                kind,
            );
            let mut sink = b"unchanged".to_vec();
            assert_gpu(
                gltf::write_gltf(&mesh, &mut sink, "failure").unwrap_err(),
                "glTF",
                &native,
                kind,
            );
            assert_eq!(sink, b"unchanged");
        }
    }
}

#[test]
fn obj_invalid_index_cause_and_context_survive_export() {
    for index in [3, usize::MAX] {
        let mut mesh = normal_fixture();
        mesh.triangles = vec![Triangle::new(0, 1, index)].into();
        let native = HypermeshError::VertexIndexOutOfBounds {
            index,
            vertex_count: 3,
        };
        let mut sink = b"unchanged".to_vec();
        for error in [
            obj::to_obj(&mesh, "invalid").unwrap_err(),
            obj::write_obj(&mesh, &mut sink, "invalid").unwrap_err(),
        ] {
            let cause = error
                .source()
                .expect("native index cause must survive OBJ export");
            assert_eq!(
                cause.downcast_ref::<ValidationError>(),
                Some(&ValidationError::Mesh(native.clone()))
            );
            assert_eq!(
                cause.source().unwrap().downcast_ref::<HypermeshError>(),
                Some(&native)
            );
            assert!(
                matches!(error, IoError::Construction { format: "OBJ", source } if source.kind() == ValidationErrorKind::InvalidInput)
            );
        }
        assert_eq!(sink, b"unchanged");
    }
}

// Independent format decoding and exact rational geometry, not another call
// through the graphics exporter, supply the success oracle.
fn ply_rows(text: &str) -> Vec<[f64; 6]> {
    text.split_once("end_header\n")
        .unwrap()
        .1
        .lines()
        .filter(|line| !line.starts_with("3 "))
        .map(|line| {
            line.split_whitespace()
                .map(|v| v.parse::<f64>().unwrap())
                .collect::<Vec<_>>()
                .try_into()
                .unwrap()
        })
        .collect()
}

fn gltf_rows(text: &str, view: usize) -> Vec<[f32; 3]> {
    let document: serde_json::Value = serde_json::from_str(text).unwrap();
    let uri = document["buffers"][0]["uri"].as_str().unwrap();
    let bytes = base64::engine::general_purpose::STANDARD
        .decode(uri.split_once(',').unwrap().1)
        .unwrap();
    let entry = &document["bufferViews"][view];
    let start = entry["byteOffset"].as_u64().unwrap() as usize;
    let len = entry["byteLength"].as_u64().unwrap() as usize;
    let (rows, remainder) = bytes[start..start + len].as_chunks::<12>();
    assert!(remainder.is_empty());
    rows.iter()
        .map(|row| {
            std::array::from_fn(|axis| {
                f32::from_le_bytes(row[axis * 4..axis * 4 + 4].try_into().unwrap())
            })
        })
        .collect()
}

#[test]
fn exact_normal_and_position_projection_remain_correct() {
    let mesh = normal_fixture();
    let ply = ply::to_ply(&mesh, "normal").unwrap();
    assert_eq!(
        ply_rows(&ply),
        vec![
            [0., 0., 0., 0., -0.8, 0.6],
            [1., 0., 0., 0., -0.8, 0.6],
            [0., 3., 4., 0., -0.8, 0.6]
        ]
    );
    assert!(ply.ends_with("3 0 1 2\n"));
    let gltf = gltf::to_gltf(&mesh, "normal").unwrap();
    assert_eq!(
        gltf_rows(&gltf, 0),
        vec![[0., 0., 0.], [1., 0., 0.], [0., 3., 4.]]
    );
    assert_eq!(gltf_rows(&gltf, 1), vec![[0., -0.8, 0.6]; 3]);
    let obj = obj::to_obj(&mesh, "normal").unwrap();
    assert_eq!(
        obj::from_obj(obj.as_bytes()).unwrap().positions,
        mesh.positions
    );
    assert!(obj.ends_with("f 1 2 3\n"));
}

fn outputs(mesh: &TriangleMesh, name: &str) -> [String; 3] {
    [
        ply::to_ply(mesh, name).unwrap(),
        gltf::to_gltf(mesh, name).unwrap(),
        obj::to_obj(mesh, name).unwrap(),
    ]
}

#[test]
fn success_caches_keep_owned_geometry_and_returned_text_independent() {
    let source = normal_fixture();
    let expected = outputs(&source, "cache");
    let mut returned = outputs(&source, "cache");
    for value in &mut returned {
        value.clear();
    }
    assert_eq!(outputs(&source, "cache"), expected);
    let mut mutated = source.clone();
    Arc::make_mut(&mut mutated.positions)[2].z = 0.into();
    let altered = outputs(&mutated, "cache");
    assert_ne!(altered, expected);
    assert_eq!(ply_rows(&altered[0])[2], [0., 3., 0., 0., 0., 1.]);
    assert_eq!(gltf_rows(&altered[1], 1), vec![[0., 0., 1.]; 3]);
    Arc::make_mut(&mut mutated.triangles)[0] = Triangle::new(0, 1, 9);
    assert!(ply::to_ply(&mutated, "cache").is_err());
    assert!(gltf::to_gltf(&mutated, "cache").is_err());
    assert!(obj::to_obj(&mutated, "cache").is_err());
    for index in 0..5 {
        outputs(&source, &format!("evict {index}"));
    }
    assert_eq!(outputs(&source, "cache"), expected);
    std::thread::scope(|scope| {
        for _ in 0..3 {
            scope.spawn(|| assert_eq!(outputs(&source, "cache"), expected));
        }
    });
}

#[test]
fn failed_exports_retry_after_a_stronger_proof() {
    let pi = Real::pi();
    let height = &pi - Real::from(pi.certified_rational_interval(-1024).unwrap()[0].clone());
    let mesh = triangle(height.clone());
    let native = mesh.to_exact_gpu_mesh_buffers().unwrap_err();
    assert_eq!(native, GpuMeshError::TriangleNormalUndecided { triangle: 0 });
    assert_gpu(
        ply::to_ply(&mesh, "retry").unwrap_err(),
        "PLY",
        &native,
        ValidationErrorKind::Undecided,
    );
    assert_gpu(
        gltf::to_gltf(&mesh, "retry").unwrap_err(),
        "glTF",
        &native,
        ValidationErrorKind::Undecided,
    );
    assert_eq!(
        height.certified_sign_until(-1100).sign(),
        Some(hyperreal::RealSign::Positive)
    );
    // This establishes normal certification and failure retry only. Finite
    // coordinate quantization is a different claim; this tiny height can vanish.
    let expected = outputs(&mesh, "retry");
    assert_eq!(outputs(&mesh, "retry"), expected);
}

fn assert_unit_cube(mesh: &TriangleMesh) {
    let context = GeometryContext::STRICT.mesh_context();
    assert_eq!(
        mesh.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    assert!(mesh.is_closed_manifold_geometry(&context).unwrap().value);
    let mut six_volume = Real::zero();
    for point in mesh.positions.iter() {
        for coordinate in [&point.x, &point.y, &point.z] {
            assert!(*coordinate == Real::zero() || *coordinate == Real::one());
        }
    }
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        six_volume += a.dot(&b.cross(&c));
    }
    assert_eq!(six_volume, Real::from(6));
}

#[test]
fn representable_closed_exports_reimport_validate_and_compose() {
    let source = solid::cube(Real::one()).unwrap();
    let obj = obj::to_obj(&source, "cube").unwrap();
    let gltf = gltf::to_gltf(&source, "cube").unwrap();
    for restored in [
        obj::from_obj(obj.as_bytes()).unwrap(),
        gltf::from_gltf(gltf.as_bytes()).unwrap().mesh,
    ] {
        assert_unit_cube(&restored);
        assert_unit_cube(&restored.try_union(&solid::empty()).unwrap());
        let removed = restored.try_difference(&restored).unwrap();
        assert!(removed.positions.is_empty() && removed.triangles.is_empty());
        assert_unit_cube(&removed.try_union(&source).unwrap());
    }
}

#[test]
fn supported_empty_formats_remain_successful_and_consumable() {
    let empty = solid::empty();
    let text = ply::to_ply(&empty, "empty").unwrap();
    assert!(text.contains("element vertex 0\n"));
    assert!(text.contains("element face 0\n"));
    assert!(text.ends_with("end_header\n"));
    let obj = obj::to_obj(&empty, "empty").unwrap();
    let restored = obj::from_obj(obj.as_bytes()).unwrap();
    assert!(restored.positions.is_empty() && restored.triangles.is_empty());
    assert_unit_cube(
        &restored
            .try_union(&solid::cube(Real::one()).unwrap())
            .unwrap(),
    );
}

struct BrokenSink;
impl Write for BrokenSink {
    fn write(&mut self, _: &[u8]) -> io::Result<usize> {
        Err(io::Error::from(io::ErrorKind::BrokenPipe))
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn writer_failures_stay_distinct_from_geometry_failures() {
    let source = normal_fixture();
    let scene = gltf::GltfSceneObject::new("normal", &source).unwrap();
    for result in [
        ply::write_ply(&source, &mut BrokenSink, "normal"),
        gltf::write_gltf(&source, &mut BrokenSink, "normal"),
        obj::write_obj(&source, &mut BrokenSink, "normal"),
        gltf::write_gltf_scene(&mut BrokenSink, "scene", &[scene]),
    ] {
        assert!(
            matches!(result, Err(IoError::StdIo(error)) if error.kind() == io::ErrorKind::BrokenPipe)
        );
    }
}
