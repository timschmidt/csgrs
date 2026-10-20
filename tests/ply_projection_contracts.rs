//! The PLY decimal spelling preserves the selected binary64 projection.
#![cfg(feature = "ply-io")]

use csgrs::{
    ConstructionEvidence, GeometryCertainty, GeometryContext, Real, TriangleMesh,
    errors::ValidationErrorKind,
    io::{IoError, ply},
    solid::{self, SolidExt},
};
use hyperlattice::Point3;
use hypermesh::{BooleanOp, MeshCertainty, Triangle};
use ply_rs::{
    parser::Parser,
    ply::{DefaultElement, Property},
};
use std::{io::Cursor, sync::Arc};

const FIELDS: [&str; 6] = ["x", "y", "z", "nx", "ny", "nz"];

fn real(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

// The plane x_axis=value has a known positive unit normal along that axis.
// No normal evaluation or exporter machinery supplies this expected geometry.
fn plane(value: f64, axis: usize) -> (TriangleMesh, [[f64; 6]; 3]) {
    let value = if value == 0.0 { 0.0 } else { value }; // Real has unsigned zero.
    let mut rows = [[0.0; 6]; 3];
    for row in &mut rows {
        row[axis] = value;
        row[3 + axis] = 1.0;
    }
    rows[1][(axis + 1) % 3] = 1.0;
    rows[2][(axis + 2) % 3] = 1.0;
    let mesh = TriangleMesh::new(
        rows.map(|row| Point3::new(real(row[0]), real(row[1]), real(row[2])))
            .to_vec(),
        vec![Triangle::new(0, 1, 2)],
    );
    (mesh, rows)
}

fn parsed_rows(text: &str) -> (Vec<[f64; 6]>, Vec<[usize; 3]>) {
    let parsed = Parser::<DefaultElement>::new()
        .read_ply(&mut Cursor::new(text.as_bytes()))
        .unwrap();
    let rows = parsed.payload["vertex"]
        .iter()
        .map(|element| {
            FIELDS.map(|field| match element[field] {
                Property::Double(value) => value,
                _ => panic!("expected a declared binary64 field"),
            })
        })
        .collect();
    let faces = parsed.payload["face"]
        .iter()
        .map(|element| match &element["vertex_indices"] {
            Property::ListInt(indices) => indices
                .iter()
                .map(|&index| usize::try_from(index).unwrap())
                .collect::<Vec<_>>()
                .try_into()
                .unwrap(),
            _ => panic!("expected a triangle index list"),
        })
        .collect();
    (rows, faces)
}

fn assert_fields(text: &str, expected: &[[f64; 6]]) {
    let (parsed, _) = parsed_rows(text);
    assert_eq!(parsed.len(), expected.len());
    let lines: Vec<_> = text
        .split_once("end_header\n")
        .unwrap()
        .1
        .lines()
        .take(expected.len())
        .collect();
    assert_eq!(lines.len(), expected.len());
    for ((line, parsed), expected) in lines.iter().zip(parsed).zip(expected) {
        let fields: Vec<_> = line.split_whitespace().collect();
        assert_eq!(fields.len(), 6);
        for axis in 0..6 {
            assert_eq!(
                parsed[axis].to_bits(),
                expected[axis].to_bits(),
                "{}",
                FIELDS[axis]
            );
            assert_eq!(
                fields[axis].parse::<Real>().unwrap(),
                real(expected[axis]),
                "{}",
                FIELDS[axis]
            );
            assert!(fields[axis].len() <= 1077, "binary64 decimal field length");
        }
    }
}

#[test]
fn every_finite_exponent_family_and_boundary_values_preserve_exact_fields() {
    let mut state = 0x3141_5926_5358_9793_u64;
    let mut sample = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    // Every finite exponent field and both signs, with independently sampled
    // significands. This is a finite family matrix, not an exhaustive f64 proof.
    for exponent in 0..2047_u64 {
        for sign in [0, 1_u64 << 63] {
            let value =
                f64::from_bits(sign | (exponent << 52) | (sample() & ((1_u64 << 52) - 1)));
            let (mesh, rows) = plane(value, exponent as usize % 3);
            assert_fields(&ply::to_ply(&mesh, "exponent family").unwrap(), &rows);
        }
    }
    let mut values = vec![
        0.0,
        -0.0,
        f64::from_bits(1),
        f64::from_bits(2),
        f64::from_bits(f64::MIN_POSITIVE.to_bits() - 1),
        f64::MIN_POSITIVE,
        f64::from_bits(f64::MIN_POSITIVE.to_bits() + 1),
        2_f64.powi(-70),
        0.1,
        0.01,
        1.1,
        1.0,
        f64::from_bits(1_f64.to_bits() + 1),
        f64::MAX,
    ];
    for _ in 0..64 {
        let value = f64::from_bits(sample());
        if value.is_finite() {
            values.push(value);
        }
    }
    for value in values {
        for axis in 0..3 {
            for sign in [1.0, -1.0] {
                let (mesh, rows) = plane(sign * value, axis);
                assert_fields(&ply::to_ply(&mesh, "boundary").unwrap(), &rows);
            }
        }
    }
}

#[test]
fn small_and_rounded_normal_components_preserve_selected_values() {
    for (y, z, normal) in [
        (1.0, 2_f64.powi(-70), [0.0, -2_f64.powi(-70), 1.0]),
        (3.0, 4.0, [0.0, -0.8, 0.6]),
    ] {
        let points = [[0.0; 3], [1.0, 0.0, 0.0], [0.0, y, z]];
        let mesh = TriangleMesh::new(
            points
                .map(|[x, y, z]| Point3::new(real(x), real(y), real(z)))
                .to_vec(),
            vec![Triangle::new(0, 1, 2)],
        );
        let expected = points.map(|[x, y, z]| [x, y, z, normal[0], normal[1], normal[2]]);
        // For the first fixture, sqrt(1+2^-140) changes neither selected
        // binary64 component. The second normal follows exactly from 3-4-5.
        assert_fields(&ply::to_ply(&mesh, "normal").unwrap(), &expected);
    }
}

#[test]
fn already_exact_decimal_artifacts_stay_byte_identical() {
    for value in [0.0, -1.5, 2_f64.powi(-17), 2_f64.powi(60), f64::MAX] {
        let (mesh, rows) = plane(value, 0);
        let mut expected = String::from(
            "ply\nformat ascii 1.0\ncomment stable\ncomment Generated by csgrs\nelement vertex 3\nproperty double x\nproperty double y\nproperty double z\nproperty double nx\nproperty double ny\nproperty double nz\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n",
        );
        for row in rows {
            expected.push_str(&format!(
                "{:.17} {:.17} {:.17} {:.17} {:.17} {:.17}\n",
                row[0], row[1], row[2], row[3], row[4], row[5]
            ));
        }
        expected.push_str("3 0 1 2\n");
        assert_eq!(ply::to_ply(&mesh, "stable").unwrap(), expected);
        assert_fields(&expected, &rows);
    }
}

fn reconstruct(text: &str) -> TriangleMesh {
    let (rows, faces) = parsed_rows(text);
    TriangleMesh::new(
        rows.iter()
            .map(|row| Point3::new(real(row[0]), real(row[1]), real(row[2])))
            .collect(),
        faces
            .iter()
            .map(|&[a, b, c]| Triangle::new(a, b, c))
            .collect(),
    )
    .with_inherited_construction(ConstructionEvidence::UNTRACKED.with_coordinate_projection())
}

fn assert_cube(mesh: &TriangleMesh, side: &Real) {
    let zero = Real::zero();
    let mut six_volume = Real::zero();
    for triangle in mesh.triangles.iter() {
        let points = triangle.indices().map(|index| &mesh.positions[index]);
        let rows = points.map(|point| [&point.x, &point.y, &point.z]);
        for row in rows {
            for coordinate in row {
                assert!(coordinate == &zero || coordinate == side);
            }
        }
        assert!((0..3).any(|axis| rows.iter().all(|row| row[axis] == &zero)
            || rows.iter().all(|row| row[axis] == side)));
        let [a, b, c] = points.map(|point| point.to_vector());
        six_volume += a.dot(&b.cross(&c));
    }
    assert_eq!(six_volume, Real::from(6) * side * side * side);
    let context = GeometryContext::STRICT.mesh_context();
    assert_eq!(
        mesh.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    let manifold = mesh.is_closed_manifold_geometry(&context).unwrap();
    assert!(manifold.value);
    assert_eq!(manifold.certainty, MeshCertainty::Certified);
}

#[test]
fn tiny_closed_body_remains_usable_after_external_read_and_strict_composition() {
    let side = real(2_f64.powi(-70));
    let source = solid::cube(side.clone()).unwrap();
    let text = ply::to_ply(&source, "tiny cube").unwrap();
    let restored = reconstruct(&text);
    assert_cube(&restored, &side);
    let evidence = restored.construction_evidence();
    assert!(evidence.source_history_missing() && evidence.coordinates_projected());
    assert_eq!(evidence.predicate_certainty(), None);
    let transformed = solid::try_scale(&restored, 2.into(), 2.into(), 2.into()).unwrap();
    assert_cube(&transformed, &(Real::from(2) * &side));
    let composed = solid::boolean_with_context(
        &restored,
        &solid::empty(),
        BooleanOp::Union,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(composed.certainty, GeometryCertainty::Certified);
    assert_cube(&composed.value, &side);
    assert!(
        composed
            .value
            .construction_evidence()
            .source_history_missing()
    );
    assert_cube(
        &reconstruct(&ply::to_ply(&composed.value, "next").unwrap()),
        &side,
    );
    let removed = restored.try_difference(&composed.value).unwrap();
    assert!(removed.positions.is_empty() && removed.triangles.is_empty());
    let empty = reconstruct(&ply::to_ply(&removed, "removed").unwrap());
    assert!(empty.positions.is_empty() && empty.triangles.is_empty());
    assert_cube(
        &empty.try_union(&solid::cube(1.into()).unwrap()).unwrap(),
        &Real::one(),
    );
    assert_eq!(
        source,
        solid::cube(side).unwrap(),
        "export leaves authoritative source intact"
    );
}

#[test]
fn projected_values_survive_cache_eviction_mutation_and_concurrent_use() {
    let tiny = 2_f64.powi(-70);
    let (source, rows) = plane(tiny, 0);
    let expected = ply::to_ply(&source, "cache").unwrap();
    assert_fields(&expected, &rows);
    let mut returned = ply::to_ply(&source, "cache").unwrap();
    returned.clear();
    assert_eq!(ply::to_ply(&source, "cache").unwrap(), expected);
    for index in 0..5 {
        ply::to_ply(&source, &format!("evict-{index}")).unwrap();
    }
    assert_eq!(ply::to_ply(&source, "cache").unwrap(), expected);
    let mut changed = source.clone();
    for point in Arc::make_mut(&mut changed.positions) {
        point.x = real(-tiny);
    }
    let output = ply::to_ply(&changed, "cache").unwrap();
    assert_ne!(output, expected);
    assert_fields(&output, &plane(-tiny, 0).1);
    Arc::make_mut(&mut changed.triangles)[0] = Triangle::new(0, 1, 9);
    assert!(
        matches!(ply::to_ply(&changed, "cache"), Err(IoError::Construction {format:"PLY",source}) if source.kind() == ValidationErrorKind::InvalidInput)
    );
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                let output = ply::to_ply(&source, "cache").unwrap();
                assert_eq!(output, expected);
                assert_fields(&output, &rows);
            });
        }
    });
}

#[test]
fn failures_remain_explicit_and_conversion_leaves_writers_untouched() {
    let (mut source, _) = plane(1.0, 0);
    let huge = format!("1{}", "0".repeat(1000)).parse::<Real>().unwrap();
    for point in Arc::make_mut(&mut source.positions) {
        point.x = huge.clone();
    }
    let mut sink = b"unchanged".to_vec();
    assert!(matches!(
        ply::write_ply(&source, &mut sink, "overflow"),
        Err(IoError::UnrepresentableCoordinate {
            format: "PLY",
            field: "vertex x",
            target: "f64"
        })
    ));
    assert_eq!(sink, b"unchanged");
    let (source, _) = plane(1.0, 0);
    assert!(matches!(
        ply::write_ply(&source, &mut sink, "bad\nend_header"),
        Err(IoError::InvalidMetadata {
            format: "PLY",
            field: "comment"
        })
    ));
    assert_eq!(sink, b"unchanged");
    let invalid = TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]);
    assert!(
        matches!(ply::write_ply(&invalid, &mut sink, "invalid"), Err(IoError::Construction {format:"PLY",source}) if source.kind() == ValidationErrorKind::InvalidInput)
    );
    assert_eq!(sink, b"unchanged");
    let empty_text = ply::to_ply(&solid::empty(), "empty").unwrap();
    let (vertices, faces) = parsed_rows(&empty_text);
    assert!(vertices.is_empty() && faces.is_empty());
    let mut written = Vec::new();
    ply::write_ply(&source, &mut written, "stream").unwrap();
    assert_eq!(written, ply::to_ply(&source, "stream").unwrap().as_bytes());
}
