//! Independent ASCII/binary STL fixtures exercise exact promotion of parsed binary32 coordinates.
//! Row preservation, signed winding integrals, closed-PWN admission and downstream empty reuse
//! concern the imported model; they do not certify fidelity to a pre-serialization source.

#![cfg(feature = "stl-io")]

use csgrs::{
    GeometryContext, Real, TriangleMesh,
    errors::{ValidationError, ValidationErrorKind},
    io::{IoError, stl},
    solid::{self, SolidExt},
};
use hypermesh::{HypermeshError, MeshCertainty, Triangle};
use std::{error::Error, sync::Arc};

/// Three ordered binary32 XYZ corners of one encoded STL triangle.
type Face = [[f32; 3]; 3];

/// Generates twelve outward cube faces using binary32 origin/side arithmetic and a fixed winding.
/// Fixture callers supply representable corners; this helper does not validate overflow or side sign.
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

/// Independently encodes faces with zero file normals, including intentionally malformed geometry.
/// ASCII uses scientific binary32 formatting; binary uses an 80-byte zero header, little-endian
/// count/coordinates and zero attribute words. Does not call the production exporters or validate topology.
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

/// Requires face count/order and every corner value to equal the exact imported binary32 expectation.
/// Vertex indexing may be shared or reordered; comparison follows the stored face references.
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

/// Requires certified closed-PWN validation across strict/approximate/strict passes on the same mesh.
/// This is a local topology receipt, not a source-fidelity certificate.
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

/// Checks the exact sum of oriented scalar triple products against an independent expectation.
/// The integral counts winding multiplicity and orientation rather than general regularized-set volume.
fn assert_winding_integral(mesh: &TriangleMesh, six_volume: Real) {
    let actual = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(actual, six_volume);
}

/// Checks certified PWN, geometric edge pairing, cube corner/support-plane incidence and signed volume.
/// The fixture is anchored at zero with positive side; no independent per-face normal test is performed.
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

/// Imports six scales from the least positive subnormal through binary32 maximum in one encoding.
/// Preserves exact rows and cube invariants, then consumes empty identities/self-removal and restores a unit cube.
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
/// Runs the six-scale exact-import and downstream-composition matrix through binary STL.
fn binary_scales_reimport_and_compose() {
    scales_compose(false);
}

#[test]
/// Runs the same scale matrix through independently formatted ASCII STL parsed as binary32.
fn ascii_scales_reimport_and_compose() {
    scales_compose(true);
}

/// Requires the STL construction label, invalid-input category and exact native topology error chain.
/// Downcasts both modeling and mesh causes instead of relying only on diagnostic text.
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

/// Checks missing, reversed and duplicated cube faces plus a collinear triangle in the selected encoding.
/// Expected open-edge, unbalanced-edge and indexed degeneracy diagnostics are independently specified.
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
/// Runs the malformed-topology and typed-source-chain matrix for binary STL.
fn binary_invalid_topology_retains_native_error_chain() {
    malformed_topology(false);
}

#[test]
/// Runs the same malformed-topology matrix for ASCII STL.
fn ascii_invalid_topology_retains_native_error_chain() {
    malformed_topology(true);
}

#[test]
/// Requires valid zero-face files in either encoding to compose as empty, while junk remains a parse error.
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
/// Requires NaN and either infinity in a face coordinate to fail in both encodings; category is not asserted.
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
/// Imports doubled, inverted, disconnected, hollow and mixed-signed-zero surfaces in both encodings.
/// Checks exact face values, certified PWN and independent winding integrals, then self-subtraction
/// to canonical empty and restoration of a unit cube. Binary sign-bit identity of zero is not asserted.
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
/// Corrupts an index or collapses a cloned imported face after warming validation.
/// Fresh validation and empty union must fail while the original cube still passes its invariants.
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
