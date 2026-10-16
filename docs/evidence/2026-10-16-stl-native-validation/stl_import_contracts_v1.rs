#![cfg(feature = "stl-io")]

use csgrs::{
    GeometryContext, Real, TriangleMesh,
    errors::{ValidationError, ValidationErrorKind},
    io::{IoError, stl},
    solid::{self, SolidExt},
};
use hypermesh::{HypermeshError, MeshCertainty, Triangle};
use std::{error::Error, sync::Arc};

type Face = [[f32; 3]; 3];

fn cube_faces(origin: [f32; 3], side: f32) -> Vec<Face> {
    let corners = [
        [0_u8, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [1, 1, 1],
        [0, 1, 1],
    ]
    .map(|p| std::array::from_fn(|axis| origin[axis] + f32::from(p[axis]) * side));
    [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
        [0, 1, 5],
        [0, 5, 4],
        [3, 7, 6],
        [3, 6, 2],
    ]
    .map(|indices| indices.map(|i| corners[i]))
    .to_vec()
}

// Independent format fixtures bypass exporters that reject malformed faces or
// currently round small text coordinates. Numeric expectations are binary32.
fn encode(faces: &[Face], ascii: bool) -> Vec<u8> {
    if ascii {
        let mut text = String::from("solid contract\n");
        for face in faces {
            text.push_str("facet normal 0 0 0\nouter loop\n");
            for [x, y, z] in face {
                text.push_str(&format!("vertex {x:e} {y:e} {z:e}\n"));
            }
            text.push_str("endloop\nendfacet\n");
        }
        text.push_str("endsolid contract\n");
        return text.into_bytes();
    }
    let mut bytes = vec![0; 80];
    bytes.extend_from_slice(&u32::try_from(faces.len()).unwrap().to_le_bytes());
    for face in faces {
        bytes.extend_from_slice(&[0; 12]);
        for value in face.iter().flatten() {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        bytes.extend_from_slice(&[0; 2]);
    }
    bytes
}

fn assert_rows(mesh: &TriangleMesh, expected: &[Face]) {
    assert_eq!(mesh.triangles.len(), expected.len());
    for (triangle, face) in mesh.triangles.iter().zip(expected) {
        for (index, row) in triangle.indices().into_iter().zip(face) {
            let point = &mesh.positions[index];
            for (actual, expected) in [&point.x, &point.y, &point.z].into_iter().zip(row) {
                assert_eq!(actual, &Real::try_from(*expected).unwrap());
            }
        }
    }
}

fn assert_certified(mesh: &TriangleMesh) {
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        assert_eq!(
            mesh.validate_closed_pwn(&context.mesh_context())
                .unwrap()
                .certainty,
            MeshCertainty::Certified
        );
    }
}

fn assert_winding_integral(mesh: &TriangleMesh, six_volume: Real) {
    let actual = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(actual, six_volume);
}

fn assert_cube(mesh: &TriangleMesh, side: &Real) {
    assert_certified(mesh);
    assert!(
        mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
    let zero = Real::zero();
    for triangle in mesh.triangles.iter() {
        let points = triangle.indices().map(|i| &mesh.positions[i]);
        for p in points {
            for value in [&p.x, &p.y, &p.z] {
                assert!(value == &zero || value == side);
            }
        }
        assert!((0..3).any(|axis| {
            let values = points.map(|p| [&p.x, &p.y, &p.z][axis]);
            values.iter().all(|v| *v == &zero) || values.iter().all(|v| *v == side)
        }));
    }
    assert_winding_integral(mesh, Real::from(6) * side * side * side);
}

fn scales_compose(ascii: bool) {
    for side in [
        f32::from_bits(1),
        f32::MIN_POSITIVE,
        2_f32.powi(-12),
        1.0,
        2_f32.powi(60),
        f32::MAX,
    ] {
        let faces = cube_faces([0.0; 3], side);
        let mesh = stl::from_stl(&encode(&faces, ascii)).unwrap();
        assert_rows(&mesh, &faces);
        let side = Real::try_from(side).unwrap();
        assert_cube(&mesh, &side);
        assert_cube(&mesh.try_union(&solid::empty()).unwrap(), &side);
        let empty = mesh.try_difference(&mesh).unwrap();
        assert!(empty.positions.is_empty() && empty.triangles.is_empty());
        let unit = empty.try_union(&solid::cube(Real::one()).unwrap()).unwrap();
        assert_cube(&unit, &Real::one());
    }
}

#[test]
fn binary_scales_reimport_and_compose() {
    scales_compose(false);
}

#[test]
fn ascii_scales_reimport_and_compose() {
    scales_compose(true);
}

fn assert_invalid(faces: &[Face], ascii: bool, expected: HypermeshError) {
    let error = stl::from_stl(&encode(faces, ascii)).unwrap_err();
    let IoError::Construction { format, source } = &error else {
        panic!("expected typed native construction failure, got {error:?}");
    };
    assert_eq!(*format, "STL");
    assert_eq!(source.kind(), ValidationErrorKind::InvalidInput);
    assert_eq!(source, &ValidationError::Mesh(expected.clone()));
    let cause = error.source().unwrap();
    assert!(cause.downcast_ref::<ValidationError>().is_some());
    assert_eq!(
        cause.source().unwrap().downcast_ref::<HypermeshError>(),
        Some(&expected)
    );
}

fn malformed_topology(ascii: bool) {
    let cube = cube_faces([0.0; 3], 1.0);
    let mut open = cube.clone();
    open.remove(0);
    assert_invalid(
        &open,
        ascii,
        HypermeshError::OpenInput {
            mesh_index: 0,
            boundary_edges: 3,
        },
    );
    let mut reversed = cube.clone();
    reversed[0].swap(1, 2);
    let mut duplicated = cube.clone();
    duplicated.insert(0, cube[0]);
    for faces in [reversed, duplicated] {
        assert_invalid(
            &faces,
            ascii,
            HypermeshError::NonPwnInput {
                mesh_index: 0,
                unbalanced_edges: 3,
            },
        );
    }
    assert_invalid(
        &[[[0.0; 3], [1.0; 3], [2.0; 3]]],
        ascii,
        HypermeshError::DegenerateTriangle {
            mesh_index: 0,
            triangle_index: 0,
        },
    );
}

#[test]
fn binary_invalid_topology_retains_native_error_chain() {
    malformed_topology(false);
}

#[test]
fn ascii_invalid_topology_retains_native_error_chain() {
    malformed_topology(true);
}

#[test]
fn canonical_empty_and_parse_failure_remain_distinct() {
    for ascii in [false, true] {
        let empty = stl::from_stl(&encode(&[], ascii)).unwrap();
        assert!(empty.positions.is_empty() && empty.triangles.is_empty());
        assert_cube(
            &empty.try_union(&solid::cube(Real::one()).unwrap()).unwrap(),
            &Real::one(),
        );
    }
    assert!(matches!(stl::from_stl(b"not an STL"), Err(IoError::StdIo(_))));
}

#[test]
fn nonfinite_coordinates_never_become_geometry() {
    for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut faces = cube_faces([0.0; 3], 1.0);
        faces[0][0][0] = value;
        for ascii in [false, true] {
            assert!(stl::from_stl(&encode(&faces, ascii)).is_err());
        }
    }
}

#[test]
fn closed_pwn_variations_preserve_rows_winding_and_later_operations() {
    let cube = cube_faces([0.0; 3], 1.0);
    let mut doubled = cube.clone();
    doubled.extend_from_slice(&cube);
    let mut inverted = cube.clone();
    inverted.iter_mut().for_each(|face| face.swap(1, 2));
    let mut disconnected = cube.clone();
    disconnected.extend(cube_faces([3.0, 0.0, 0.0], 1.0));
    let mut hollow = cube_faces([0.0; 3], 2.0);
    hollow.extend(cube_faces([0.75; 3], 0.5).into_iter().map(|mut face| {
        face.swap(1, 2);
        face
    }));
    let mut signed_zero = cube.clone();
    for face in signed_zero.iter_mut().step_by(2) {
        for coordinate in face.iter_mut().flatten() {
            if *coordinate == 0.0 {
                *coordinate = -0.0;
            }
        }
    }
    for (faces, winding_integral) in [
        (doubled, Real::from(12)),
        (inverted, Real::from(-6)),
        (disconnected, Real::from(12)),
        (hollow, "47.25".parse().unwrap()),
        (signed_zero, Real::from(6)),
    ] {
        for ascii in [false, true] {
            let imported = stl::from_stl(&encode(&faces, ascii)).unwrap();
            assert_rows(&imported, &faces);
            assert_certified(&imported);
            // This integral counts winding multiplicity; it is not a general
            // regularized-set volume assertion for multiply wound surfaces.
            assert_winding_integral(&imported, winding_integral.clone());
            let empty = imported.try_difference(&imported).unwrap();
            assert!(empty.positions.is_empty() && empty.triangles.is_empty());
            assert_cube(
                &empty.try_union(&solid::cube(Real::one()).unwrap()).unwrap(),
                &Real::one(),
            );
        }
    }
}

#[test]
fn mutation_cannot_reuse_an_import_validation_fact() {
    let imported = stl::from_stl(&encode(&cube_faces([0.0; 3], 1.0), false)).unwrap();
    assert_certified(&imported);
    let mut invalid_index = imported.clone();
    Arc::make_mut(&mut invalid_index.triangles)[0] = Triangle::new(0, usize::MAX, 0);
    let mut collapsed = imported.clone();
    let replacement = collapsed.positions[collapsed.triangles[0].indices()[1]].clone();
    let position = collapsed.triangles[0].indices()[0];
    Arc::make_mut(&mut collapsed.positions)[position] = replacement;
    for invalid in [invalid_index, collapsed] {
        assert!(
            invalid
                .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
                .is_err()
        );
        assert!(invalid.try_union(&solid::empty()).is_err());
        assert_cube(&imported, &Real::one());
    }
}
