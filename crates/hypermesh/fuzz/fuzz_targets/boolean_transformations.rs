#![no_main]

mod support;

use hyperlattice::{Matrix4, Matrix4TransformKind};
use hypermesh::{
    BooleanOp, EmberConfig, HypermeshError, TriangleMesh, Real, Triangle, BooleanMesh,
    boolean_operation, boolean_mesh, boolean_mesh_with_certified_convex_inputs,
    certify_convex_mesh, classify_polygon_output,
};
use hyperreal::StructuralKind;
use libfuzzer_sys::fuzz_target;
use support::{
    Bytes, convex_mesh, operation, r, representative_hyperreal_values, validate_result,
    validate_soup, volume_numerator,
};

struct TransformPlan {
    stages: Vec<Matrix4>,
    reverses_orientation: bool,
}

fn matrix(values: [[i64; 4]; 4]) -> Matrix4 {
    Matrix4::new(values.map(|row| row.map(r)))
}

fn assert_transform_kind(matrix: &Matrix4, expected: Matrix4TransformKind) {
    assert_eq!(matrix.structural_facts().transform_kind, expected);
}

fn value_translation(value: Real) -> Matrix4 {
    let translation = Matrix4::affine_translation([value, Real::zero(), Real::zero()]);
    assert_transform_kind(&translation, Matrix4TransformKind::AffineTranslation);
    translation
}

fn transform_plan(family: u8, control: u8, value: &Real) -> TransformPlan {
    let reverses_orientation = control & 1 != 0;
    let (matrix, expected, value_entry): (Matrix4, Matrix4TransformKind, Option<(usize, usize)>) =
        match family % 6 {
            0 => (Matrix4::identity(), Matrix4TransformKind::Identity, None),
            1 => {
                let matrix = if reverses_orientation {
                    matrix([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                } else {
                    matrix([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                };
                (matrix, Matrix4TransformKind::SignedPermutation, None)
            }
            2 => (
                Matrix4::affine_translation([
                    value.clone(),
                    r(i64::from(control % 5) - 2),
                    r(i64::from((control / 5) % 5) - 2),
                ]),
                Matrix4TransformKind::AffineTranslation,
                Some((0, 3)),
            ),
            3 => {
                let sx = if reverses_orientation { -2 } else { 2 };
                (
                    Matrix4::new([
                        [r(sx), Real::zero(), Real::zero(), value.clone()],
                        [Real::zero(), r(2), Real::zero(), Real::zero()],
                        [Real::zero(), Real::zero(), r(3), Real::zero()],
                        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
                    ]),
                    Matrix4TransformKind::AffineDiagonalLinear,
                    Some((0, 3)),
                )
            }
            4 => {
                let sx = if reverses_orientation { -1 } else { 1 };
                (
                    Matrix4::new([
                        [r(sx), value.clone(), Real::zero(), Real::zero()],
                        [Real::zero(), r(2), Real::one(), Real::zero()],
                        [Real::zero(), Real::zero(), Real::one(), Real::zero()],
                        [Real::zero(), Real::zero(), Real::zero(), Real::one()],
                    ]),
                    Matrix4TransformKind::Affine,
                    Some((0, 1)),
                )
            }
            _ => {
                let sx = if reverses_orientation { -1 } else { 1 };
                (
                    Matrix4::new([
                        [r(sx), value.clone(), Real::zero(), Real::zero()],
                        [Real::zero(), Real::one(), Real::zero(), Real::zero()],
                        [Real::zero(), Real::zero(), Real::one(), Real::zero()],
                        [Real::zero(), Real::zero(), Real::one(), r(32)],
                    ]),
                    Matrix4TransformKind::Projective,
                    Some((0, 1)),
                )
            }
        };
    assert_transform_kind(&matrix, expected);

    let kind = value.detailed_facts().symbolic.kind;
    let mut stages = vec![matrix];
    if let Some((row, column)) = value_entry {
        assert_eq!(
            stages[0].0[row][column].detailed_facts().symbolic.kind,
            kind,
        );
    } else {
        // Identity and signed permutations have no free coefficients. Pair
        // them with a value-bearing translation so every transform class is
        // still exercised with every public Hyperreal representation.
        stages.push(value_translation(value.clone()));
        assert_eq!(stages[1].0[0][3].detailed_facts().symbolic.kind, kind,);
    }

    TransformPlan {
        stages,
        reverses_orientation: matches!(expected, Matrix4TransformKind::SignedPermutation)
            && reverses_orientation
            || matches!(
                expected,
                Matrix4TransformKind::AffineDiagonalLinear
                    | Matrix4TransformKind::Affine
                    | Matrix4TransformKind::Projective
            ) && reverses_orientation,
    }
}

fn transform_mesh(mesh: TriangleMesh, plan: &TransformPlan) -> TriangleMesh {
    let mut positions = mesh.positions.to_vec();
    for matrix in &plan.stages {
        let batch = matrix
            .transform_point3_batch(&positions)
            .expect("bounded transform must produce finite points");
        for (source, transformed) in positions.iter().zip(&batch) {
            assert_eq!(
                matrix
                    .transform_point3(source)
                    .expect("single-point transform must produce a finite point"),
                *transformed,
            );
        }
        positions = batch;
    }
    let mut triangles = mesh.triangles.to_vec();
    if plan.reverses_orientation {
        for triangle in &mut triangles {
            *triangle = Triangle::new(triangle.v0, triangle.v2, triangle.v1);
        }
    }
    TriangleMesh::new(positions, triangles)
}

fn accept_certification_boundary(error: HypermeshError) {
    assert!(
        matches!(
            error,
            HypermeshError::UnknownClassification
                | HypermeshError::ReferencePropagationFailed
                | HypermeshError::PointAtInfinity
                | HypermeshError::OpenOutput { .. }
                | HypermeshError::SubdivisionDepthLimit { .. }
        ),
        "symbolic-transform Boolean returned a non-certification error: {error:?}",
    );
}

fn run_boolean(
    meshes: &[TriangleMesh; 2],
    op: BooleanOp,
    api: u8,
) -> Result<BooleanMesh, HypermeshError> {
    let refs = [meshes[0].as_ref(), meshes[1].as_ref()];
    match api % 3 {
        0 => boolean_operation(&refs, op, EmberConfig::default())
            .map(|result| validate_result(&result, op, refs.len())),
        1 => boolean_mesh(&refs, op, EmberConfig::default()).inspect(|soup| {
            validate_soup(soup);
        }),
        _ => boolean_mesh_with_certified_convex_inputs(
            &refs,
            op,
            &[true, true],
            EmberConfig::default(),
        )
        .inspect(|soup| {
            validate_soup(soup);
        }),
    }
}

fn run_symbolic_boolean(meshes: &[TriangleMesh; 2], op: BooleanOp) -> Result<(), HypermeshError> {
    let refs = [meshes[0].as_ref(), meshes[1].as_ref()];
    boolean_operation(&refs, op, EmberConfig::default()).map(|result| {
        assert_eq!(result.output().num_meshes, refs.len());
        assert_eq!(
            result.output().polygons.len(),
            result.classifications().len()
        );
        assert_eq!(result.output().polygons.len(), result.winding_pairs().len());
        assert!(
            result
                .output()
                .polygons
                .iter()
                .all(|polygon| polygon.is_valid())
        );
        for (classification, winding) in result.classifications().iter().zip(result.winding_pairs())
        {
            assert!(matches!(classification, -1 | 1));
            if let Some(winding) = winding {
                assert_eq!(winding.w_front.len(), refs.len());
                assert_eq!(winding.w_back.len(), refs.len());
                assert_eq!(
                    classify_polygon_output(&winding.w_front, &winding.w_back, op),
                    *classification,
                );
            }
        }
    })
}

fuzz_target!(|data: [u8; 48]| {
    let mut bytes = Bytes::new(&data);
    let application = bytes.next() % 4;
    let op = operation(bytes.next());
    let api = bytes.next();
    let values = representative_hyperreal_values();
    let left_value = &values[usize::from(bytes.next() % values.len() as u8)];
    let right_value = &values[usize::from(bytes.next() % values.len() as u8)];
    let left_kind = left_value.detailed_facts().symbolic.kind;
    let right_kind = right_value.detailed_facts().symbolic.kind;
    let plans = [
        transform_plan(bytes.next(), bytes.next(), left_value),
        transform_plan(bytes.next(), bytes.next(), right_value),
    ];
    let mut meshes = [convex_mesh(&mut bytes), convex_mesh(&mut bytes)];
    if application & 1 != 0 {
        meshes[0] = transform_mesh(meshes[0].clone(), &plans[0]);
    }
    if application & 2 != 0 {
        meshes[1] = transform_mesh(meshes[1].clone(), &plans[1]);
    }
    let symbolic_transform = application & 1 != 0 && left_kind != StructuralKind::ExactRational
        || application & 2 != 0 && right_kind != StructuralKind::ExactRational;

    for mesh in &meshes {
        if let Err(error) = certify_convex_mesh(mesh.as_ref()) {
            if symbolic_transform {
                accept_certification_boundary(error);
                return;
            }
            panic!("exact transformed convex mesh failed certification: {error:?}");
        }
    }

    if symbolic_transform {
        if let Err(error) = run_symbolic_boolean(&meshes, op) {
            accept_certification_boundary(error);
        }
        return;
    }

    let result = run_boolean(&meshes, op, api);
    match result {
        Ok(soup) => {
            let alternate = run_boolean(&meshes, op, api.wrapping_add(1))
                .unwrap_or_else(|error| panic!("alternate exact Boolean API failed: {error:?}"));
            assert_eq!(volume_numerator(&soup), volume_numerator(&alternate));
        }
        Err(error) => panic!("exact transformed Boolean failed: {error:?}"),
    }
});
