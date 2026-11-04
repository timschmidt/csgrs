//! A lazy CSG expression evaluates as one Boolean program (U06, P21).
//!
//! The expected sets are built independently, one regularized operation at a
//! time, and compared through an exact empty symmetric difference and the
//! exact enclosed volume. The volume is also checked against an independent
//! closed-form value.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    Csg, Error, GeometryCertainty, GeometryContext, Real, SolidExpr, SolidExprChild,
    TriangleMesh, errors::ValidationErrorKind, solid,
};
use hypermesh::BooleanOp;

fn volume(mesh: &TriangleMesh) -> Real {
    solid::exact_mass_properties(mesh, Real::one())
        .unwrap()
        .volume
}

fn same_set(left: &TriangleMesh, right: &TriangleMesh) {
    let xor = solid::boolean(left, right, BooleanOp::SymmetricDifference).unwrap();
    assert!(
        xor.triangles.is_empty(),
        "sets differ by {} triangles",
        xor.triangles.len()
    );
    assert_eq!(
        volume(left).exact_rational_normal_form(),
        volume(right).exact_rational_normal_form()
    );
}

fn translated(mesh: &TriangleMesh, x: i32, y: i32, z: i32) -> TriangleMesh {
    solid::transform(
        mesh,
        &hyperlattice::Matrix4::affine_translation([
            Real::from(x),
            Real::from(y),
            Real::from(z),
        ]),
    )
    .unwrap()
}

/// `(body - hole) | boss` with a 12-cube body, a 4×4×16 hole through it and a
/// 2-cube boss on top: 12³ - 4·4·12 + 2³ = 1728 - 192 + 8 = 1544.
#[test]
fn an_expression_equals_step_by_step_evaluation() {
    let m = Csg::strict();
    let body = m.cube(12).unwrap();
    let hole = m.cuboid(4, 4, 16).unwrap();
    let boss = m.cube(2).unwrap();
    let expression = (SolidExpr::from(body.clone())
        - SolidExpr::from(hole.clone()).translated(4, 4, -2).unwrap())
        | SolidExpr::from(boss.clone()).translated(5, 5, 12).unwrap();
    let lazy = m.evaluate(&expression).unwrap();

    let stepwise = solid::boolean(
        &solid::boolean(&body, &translated(&hole, 4, 4, -2), BooleanOp::Difference).unwrap(),
        &translated(&boss, 5, 5, 12),
        BooleanOp::Union,
    )
    .unwrap();
    same_set(&lazy, &stepwise);
    assert_eq!(
        volume(&lazy).exact_rational_normal_form(),
        Real::from(1544).exact_rational_normal_form()
    );
    assert_eq!(m.certainty(), GeometryCertainty::Certified);
}

#[test]
fn every_operator_matches_its_eager_operation() {
    let a = solid::cube(Real::from(4)).unwrap();
    let b = translated(&solid::cube(Real::from(4)).unwrap(), 2, 1, 0);
    let (ea, eb) = (SolidExpr::from(a.clone()), SolidExpr::from(b.clone()));
    for (expression, operation) in [
        (&ea | &eb, BooleanOp::Union),
        (&ea & &eb, BooleanOp::Intersection),
        (&ea - &eb, BooleanOp::Difference),
        (&ea ^ &eb, BooleanOp::SymmetricDifference),
    ] {
        let lazy = expression
            .evaluate_with_context(GeometryContext::STRICT)
            .unwrap()
            .value;
        same_set(&lazy, &solid::boolean(&a, &b, operation).unwrap());
    }
}

#[test]
fn transforms_push_down_to_the_leaves() {
    let a = solid::cuboid(Real::from(4), Real::from(2), Real::from(3)).unwrap();
    let b = translated(&solid::cube(Real::from(2)).unwrap(), 1, 1, 1);
    let expression = (SolidExpr::from(a.clone()) - SolidExpr::from(b.clone()))
        .rotated(90, 0, 30)
        .unwrap()
        .scaled(2, 1, 3)
        .unwrap()
        .translated(-7, 0, 5)
        .unwrap();
    let lazy = expression
        .evaluate_with_context(GeometryContext::STRICT)
        .unwrap()
        .value;
    let eager = translated(
        &solid::scale(
            &solid::rotate(
                &solid::boolean(&a, &b, BooleanOp::Difference).unwrap(),
                Real::from(90),
                Real::zero(),
                Real::from(30),
            )
            .unwrap(),
            Real::from(2),
            Real::one(),
            Real::from(3),
        )
        .unwrap(),
        -7,
        0,
        5,
    );
    same_set(&lazy, &eager);
}

#[test]
fn empty_leaves_are_the_empty_set() {
    let cube = SolidExpr::from(solid::cube(Real::from(2)).unwrap());
    let empty = SolidExpr::from(solid::empty());
    let strict = GeometryContext::STRICT;
    let evaluate =
        |expression: &SolidExpr| expression.evaluate_with_context(strict).unwrap().value;
    same_set(&evaluate(&(&cube | &empty)), &evaluate(&cube));
    same_set(&evaluate(&(&cube - &empty)), &evaluate(&cube));
    assert!(evaluate(&(&cube & &empty)).triangles.is_empty());
    assert!(evaluate(&(&empty - &cube)).triangles.is_empty());
    assert!(evaluate(&(&empty | &empty)).triangles.is_empty());
}

#[test]
fn expression_failures_identify_the_failing_node() {
    use SolidExprChild::{Left, Right};
    let cube = SolidExpr::from(solid::cube(2).unwrap());
    let singular = cube.scaled(0, 1, 1).unwrap();
    let expression = &cube | &(&singular - &cube);
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let session = Csg::new(context);
        let error = session.evaluate(&expression).unwrap_err();
        assert_eq!(session.certainty(), GeometryCertainty::Certified);
        assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
        assert!(
            error
                .to_string()
                .contains("CSG expression node [Right, Left]")
        );
        let Error::Expression { node_path, source } = &error else {
            panic!("missing expression context: {error}");
        };
        assert_eq!(node_path, &[Right, Left]);
        assert_eq!(**source, Error::InvalidArguments);
        assert_eq!(
            std::error::Error::source(&error)
                .unwrap()
                .downcast_ref::<Box<Error>>(),
            Some(source)
        );
        // Resolving the path selects the singular transform itself.
        let selected = expression.node_at(node_path).unwrap();
        assert_eq!(
            selected.evaluate_with_context(context).unwrap_err().kind(),
            ValidationErrorKind::InvalidInput
        );
    }

    let mesh = solid::cube(2).unwrap();
    let open = TriangleMesh::from_construction(
        mesh.positions.to_vec(),
        mesh.triangles[1..].to_vec(),
        csgrs::ConstructionEvidence::DECLARED_COORDINATES,
    );
    let expression = &cube | &(&cube - &SolidExpr::from(open));
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let error = expression.evaluate_with_context(context).unwrap_err();
        assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
        assert!(
            error
                .to_string()
                .contains("CSG expression node [Right, Right]")
        );
        let Error::Expression { node_path, source } = &error else {
            panic!("missing expression context: {error}");
        };
        assert_eq!(node_path, &[Right, Right]);
        let cause = std::error::Error::source(source.as_ref()).unwrap();
        assert!(matches!(
            cause.downcast_ref::<hypermesh::HypermeshError>(),
            Some(
                hypermesh::HypermeshError::OpenInput { .. }
                    | hypermesh::HypermeshError::NonPwnInput { .. }
            )
        ));
        assert!(expression.node_at(node_path).is_some());
        assert!(expression.node_at(&[SolidExprChild::Input]).is_none());
        assert!(expression.node_at(&[Left, Left]).is_none());
    }
}

#[test]
fn expression_paths_follow_transforms_and_shared_occurrences() {
    use SolidExprChild::{Input, Left, Right};
    let mesh = solid::cube(2).unwrap();
    let invalid = SolidExpr::from(TriangleMesh::from_construction(
        mesh.positions.to_vec(),
        Vec::new(),
        csgrs::ConstructionEvidence::DECLARED_COORDINATES,
    ));
    let shared = &invalid | &invalid;
    let expression = shared.translated(3, 0, 0).unwrap();
    for _ in 0..2 {
        let error = expression
            .evaluate_with_context(GeometryContext::STRICT)
            .unwrap_err();
        let Error::Expression { node_path, source } = error else {
            panic!("missing expression context");
        };
        // The shared operand is lowered once at its first occurrence.
        assert_eq!(node_path, &[Input, Left]);
        assert!(matches!(
            *source,
            Error::Mesh(hypermesh::HypermeshError::EmptyMesh { mesh_index: 0 })
        ));
    }

    let cube = SolidExpr::from(mesh);
    let singular = shared.scaled(0, 1, 1).unwrap();
    let expression = &cube | &singular;
    let error = expression
        .evaluate_with_context(GeometryContext::STRICT)
        .unwrap_err();
    let Error::Expression { node_path, source } = error else {
        panic!("missing expression context");
    };
    // Pushing an outer transform to leaves still blames that transform.
    assert_eq!(node_path, &[Right]);
    assert_eq!(*source, Error::InvalidArguments);
}

#[test]
fn empty_expression_operands_do_not_hide_located_input_failures() {
    use SolidExprChild::{Left, Right};
    let mesh = solid::cube(2).unwrap();
    let invalid = SolidExpr::from(TriangleMesh::from_construction(
        mesh.positions.to_vec(),
        Vec::new(),
        csgrs::ConstructionEvidence::DECLARED_COORDINATES,
    ));
    let empty = SolidExpr::from(solid::empty());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for (left, right, path) in [(&empty, &invalid, Right), (&invalid, &empty, Left)] {
            for expression in [left | right, left & right, left - right, left ^ right] {
                let error = expression.evaluate_with_context(context).unwrap_err();
                assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
                let Error::Expression { node_path, source } = error else {
                    panic!("missing expression context");
                };
                assert_eq!(node_path, &[path]);
                assert!(matches!(
                    *source,
                    Error::Mesh(hypermesh::HypermeshError::EmptyMesh { mesh_index: 0 })
                ));
            }
        }
    }
}

#[test]
fn unbounded_expression_transforms_keep_their_typed_cause() {
    let mesh = solid::cube(2).unwrap();
    // An invertible projective map undefined on x = 0.
    let reciprocal_x = hyperlattice::Matrix4::from_row_major(
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0].map(Real::from),
    );
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let expected =
            solid::transform_with_context(&mesh, &reciprocal_x, context).unwrap_err();
        let leaf = SolidExpr::from(mesh.clone());
        let expression = &leaf | &leaf.transformed(reciprocal_x.clone());
        let error = expression.evaluate_with_context(context).unwrap_err();
        assert_eq!(error.kind(), ValidationErrorKind::Unbounded);
        let Error::Expression { node_path, source } = error else {
            panic!("missing expression context");
        };
        assert_eq!(node_path, &[SolidExprChild::Right]);
        assert_eq!(*source, expected);
    }
}

#[test]
fn unlocalized_mesh_failures_identify_root_evaluation() {
    let malformed = TriangleMesh::new(
        vec![hyperlattice::Point3::origin()],
        vec![hypermesh::Triangle::new(0, 1, 0)],
    );
    let leaf = SolidExpr::from(malformed);
    let expression = &SolidExpr::from(solid::empty()) | &leaf;
    let error = expression
        .evaluate_with_context(GeometryContext::STRICT)
        .unwrap_err();
    let Error::Expression { node_path, source } = error else {
        panic!("missing expression context");
    };
    // This kernel diagnostic carries no operand index. Do not infer a leaf.
    assert!(node_path.is_empty());
    assert!(expression.node_at(&node_path).is_some());
    assert!(matches!(
        *source,
        Error::Mesh(hypermesh::HypermeshError::VertexIndexOutOfBounds { .. })
    ));
}

/// Regularization can produce an empty mesh while retaining approximate
/// construction ancestry. Lowering that leaf to `false` must keep its receipt.
#[test]
fn empty_leaves_keep_construction_evidence_through_further_operations() {
    let approximate = Csg::approximate_512();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
    let cube = approximate.cube(1).unwrap();
    let touching = approximate
        .translate(&cube, Real::one() + zero, 0, 0)
        .unwrap();
    let empty = approximate.intersection(&cube, &touching).unwrap();
    assert!(empty.positions.is_empty() && empty.triangles.is_empty());
    assert!(empty.construction_evidence().approximate_512_consumed());

    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let leaf = SolidExpr::from(empty.clone());
        let body = SolidExpr::from(cube.clone());
        for expression in [
            leaf.clone(),
            &leaf | &body,
            &body | &leaf,
            &leaf & &body,
            &body - &leaf,
            &leaf - &body,
            &leaf ^ &body,
            &leaf | &leaf,
        ] {
            let outcome = expression.evaluate_with_context(context).unwrap();
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            assert!(
                outcome
                    .value
                    .construction_evidence()
                    .approximate_512_consumed()
            );

            let strict = Csg::strict();
            let moved = strict.translate(&outcome.value, 3, 0, 0).unwrap();
            let next = strict.union(&moved, &cube).unwrap();
            assert_eq!(strict.certainty(), GeometryCertainty::Certified);
            assert!(next.construction_evidence().approximate_512_consumed());
        }
    }
}

#[test]
fn shared_subtrees_and_an_approximate_step_keep_their_meaning() {
    // A strict session cannot decide the touching union; an approximate one
    // can, and its certainty records that.
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
    let cube = SolidExpr::from(solid::cube(Real::one()).unwrap());
    let touching = cube.translated(Real::one() + zero, 0, 0).unwrap();
    let pair = &cube | &touching;
    // The shared subtree appears twice.
    let expression = &pair | &(&pair & &cube);

    let strict = Csg::strict();
    let error = strict.evaluate(&expression).unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
    let Error::Expression { node_path, source } = error else {
        panic!("missing expression context");
    };
    assert!(node_path.is_empty());
    assert!(matches!(
        *source,
        Error::Mesh(
            hypermesh::HypermeshError::PredicateUndecided { .. }
                | hypermesh::HypermeshError::UnknownClassification
        )
    ));
    assert_eq!(strict.certainty(), GeometryCertainty::Certified);

    let approximate = Csg::approximate_512();
    let mesh = approximate.evaluate(&expression).unwrap();
    assert_eq!(
        approximate.certainty(),
        GeometryCertainty::Approximate512Consumed
    );
    assert!(mesh.construction_evidence().approximate_512_consumed());
    // The volume is 2, but it is built over the unprovable zero, so it has
    // no exact rational normal form to compare.
    assert!(!mesh.triangles.is_empty());
}

/// The point of the tree: one surface arrangement for the whole model, where
/// step-by-step evaluation builds one per operation.
#[cfg(feature = "dispatch-trace")]
#[test]
fn an_expression_builds_one_arrangement() {
    let boxes: Vec<TriangleMesh> = (0..5)
        .map(|i| translated(&solid::cube(Real::from(3)).unwrap(), 2 * i, i, 0))
        .collect();
    let arrangements = |run: &dyn Fn()| {
        hyperreal::dispatch_trace::reset();
        hyperreal::dispatch_trace::with_recording(run);
        hyperreal::dispatch_trace::take_trace().path_count("hypermesh", "boolean", "start")
    };
    let lazy = arrangements(&|| {
        let expression = boxes[1..]
            .iter()
            .fold(SolidExpr::from(boxes[0].clone()), |acc, mesh| {
                acc | SolidExpr::from(mesh.clone())
            });
        expression
            .evaluate_with_context(GeometryContext::STRICT)
            .unwrap();
    });
    let stepwise = arrangements(&|| {
        boxes[1..].iter().fold(boxes[0].clone(), |acc, mesh| {
            solid::boolean(&acc, mesh, BooleanOp::Union).unwrap()
        });
    });
    assert_eq!(lazy, 1);
    assert_eq!(stepwise, 4);
}

/// Timing for the program ledger: a 24-cube drilled by eight 2×2 holes,
/// step by step and as one expression. Run with
/// `cargo test --release --test solid_expr -- --ignored --nocapture`.
#[test]
#[ignore = "timing measurement"]
fn drilled_block_timing() {
    let body = solid::cube(Real::from(24)).unwrap();
    let hole = solid::cuboid(Real::from(2), Real::from(2), Real::from(28)).unwrap();
    let holes: Vec<TriangleMesh> = (0..8)
        .map(|i| translated(&hole, 2 + 3 * (i % 4), 4 + 10 * (i / 4), -2))
        .collect();
    let time = |run: &dyn Fn() -> TriangleMesh| {
        let mut samples = Vec::new();
        let mut result = None;
        for _ in 0..5 {
            let start = std::time::Instant::now();
            result = Some(run());
            samples.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        samples.sort_by(f64::total_cmp);
        (samples[2], samples[0], samples[4], result.unwrap())
    };
    let (step_median, step_min, step_max, stepwise) = time(&|| {
        holes.iter().fold(body.clone(), |acc, hole| {
            solid::boolean(&acc, hole, BooleanOp::Difference).unwrap()
        })
    });
    let (lazy_median, lazy_min, lazy_max, lazy) = time(&|| {
        let expression = holes.iter().fold(SolidExpr::from(body.clone()), |acc, hole| {
            acc - SolidExpr::from(hole.clone())
        });
        expression
            .evaluate_with_context(GeometryContext::STRICT)
            .unwrap()
            .value
    });
    same_set(&lazy, &stepwise);
    println!(
        "drilled block, 8 holes: step-by-step median {step_median:.1} ms ({step_min:.1}-{step_max:.1}); \
         one expression median {lazy_median:.1} ms ({lazy_min:.1}-{lazy_max:.1})"
    );
}
