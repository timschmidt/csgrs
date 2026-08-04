#![no_main]

mod support;

use hypermesh::{
    BooleanExpression, BooleanMeshBatch, BooleanOp, BooleanProgram, HypermeshError, Point3, Real,
    Triangle, TriangleMesh, TriangleMeshRef, boolean, certify_convex_mesh,
};
use hyperreal::{Rational, StructuralKind};
use libfuzzer_sys::fuzz_target;
use support::{
    Bytes, CONTEXT, operation, r, representative_hyperreal_values, validate_batch, value,
    volume_numerator,
};

fn translated_box(base: &Real, offset: [i64; 3], extent: i64) -> TriangleMesh {
    let coordinate =
        |axis: usize, high: bool| base + r(offset[axis] + if high { extent } else { 0 });
    TriangleMesh::new(
        vec![
            Point3::new(
                coordinate(0, false),
                coordinate(1, false),
                coordinate(2, false),
            ),
            Point3::new(
                coordinate(0, true),
                coordinate(1, false),
                coordinate(2, false),
            ),
            Point3::new(
                coordinate(0, true),
                coordinate(1, true),
                coordinate(2, false),
            ),
            Point3::new(
                coordinate(0, false),
                coordinate(1, true),
                coordinate(2, false),
            ),
            Point3::new(
                coordinate(0, false),
                coordinate(1, false),
                coordinate(2, true),
            ),
            Point3::new(
                coordinate(0, true),
                coordinate(1, false),
                coordinate(2, true),
            ),
            Point3::new(
                coordinate(0, true),
                coordinate(1, true),
                coordinate(2, true),
            ),
            Point3::new(
                coordinate(0, false),
                coordinate(1, true),
                coordinate(2, true),
            ),
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

fn expected_volume(op: hypermesh::BooleanOp, shift: [i64; 3]) -> i64 {
    let overlap = shift
        .into_iter()
        .map(|value| (3 - value.abs()).max(0))
        .product::<i64>();
    match op {
        hypermesh::BooleanOp::Union => 54 - overlap,
        hypermesh::BooleanOp::Intersection => overlap,
        hypermesh::BooleanOp::Difference => 27 - overlap,
        hypermesh::BooleanOp::SymmetricDifference => 54 - 2 * overlap,
    }
}

fn accept_certification_boundary(error: HypermeshError) {
    assert!(
        matches!(
            error,
            HypermeshError::PredicateUndecided { .. }
                | HypermeshError::UnknownClassification
                | HypermeshError::PointAtInfinity
        ),
        "symbolic-coordinate Boolean returned a non-certification error: {error:?}",
    );
}

fn assert_oracle_volume(batch: &BooleanMeshBatch, output: usize, expected: i64) {
    let actual = volume_numerator(&batch.vertices, &batch.results[output]);
    let expected = Rational::new(6 * expected);
    if let Some(actual) = actual.exact_rational() {
        assert_eq!(actual, expected);
        return;
    }
    let [lower, upper] = actual
        .certified_dyadic_interval(-96)
        .expect("non-rational Boolean volume should have a certified enclosure");
    assert!(
        lower <= expected && expected <= upper,
        "exact expected volume lies outside the certified Hyperreal enclosure",
    );
}

fuzz_target!(|data: [u8; 8]| {
    let mut bytes = Bytes::new(&data);
    let values = representative_hyperreal_values();
    let base = &values[usize::from(bytes.next() % values.len() as u8)];
    let kind = base.detailed_facts().symbolic.kind;
    let op = operation(bytes.next());
    let api = bytes.next() % 3;
    let shift = if kind == StructuralKind::ComputableOpaque {
        // Overlapping opaque-computable shells can intentionally exhaust the
        // exact reference search. Keep this representation in every Boolean
        // API without making the baseline fuzz target a known timeout.
        [4, 4, 4]
    } else {
        [
            bytes.bounded_i64(4),
            bytes.bounded_i64(4),
            bytes.bounded_i64(4),
        ]
    };
    let meshes = [
        translated_box(base, [0, 0, 0], 3),
        translated_box(base, shift, 3),
    ];
    for mesh in &meshes {
        if let Err(error) = value(certify_convex_mesh(&CONTEXT, mesh.as_ref())) {
            accept_certification_boundary(error);
            return;
        }
    }
    let refs = [meshes[0].as_ref(), meshes[1].as_ref()];
    let raw_refs = [
        TriangleMeshRef::new(&meshes[0].positions, &meshes[0].triangles),
        TriangleMeshRef::new(&meshes[1].positions, &meshes[1].triangles),
    ];

    let nodes = [
        BooleanExpression::Operation(BooleanOp::Union),
        BooleanExpression::Operation(BooleanOp::Intersection),
        BooleanExpression::Operation(BooleanOp::Difference),
        BooleanExpression::Operation(BooleanOp::SymmetricDifference),
    ];
    let roots = [0, 1, 2, 3];
    let (views, program, output) = match api {
        0 => (
            raw_refs.as_slice(),
            BooleanProgram::Operation(op),
            0,
        ),
        1 => (refs.as_slice(), BooleanProgram::Operation(op), 0),
        _ => (
            refs.as_slice(),
            BooleanProgram::Expressions {
                nodes: &nodes,
                roots: &roots,
            },
            match op {
                BooleanOp::Union => 0,
                BooleanOp::Intersection => 1,
                BooleanOp::Difference => 2,
                BooleanOp::SymmetricDifference => 3,
            },
        ),
    };
    let soup: Result<BooleanMeshBatch, HypermeshError> =
        value(boolean(&CONTEXT, views, program)).inspect(validate_batch);

    match soup {
        Ok(soup) => {
            assert_oracle_volume(&soup, output, expected_volume(op, shift));
        }
        Err(error) => accept_certification_boundary(error),
    }
});
