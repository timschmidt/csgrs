use super::*;
use hyperlattice::Matrix4;
use hypersdf::{SdfEvaluationError, SdfTransform};

fn frame() -> Matrix4 {
    // Child-to-world: (a+2b+3, b, 1-2c); inverse: (x-2y-3, y, (1-z)/2).
    Matrix4([
        [1.into(), 2.into(), 0.into(), 3.into()],
        [0.into(), 1.into(), 0.into(), 0.into()],
        [0.into(), 0.into(), (-2).into(), 1.into()],
        [0.into(), 0.into(), 0.into(), 1.into()],
    ])
}
fn field() -> Arc<Sdf> {
    Arc::new(Sdf::new(
        SdfExpr::sphere(point(0, 0, 0), 4.into())
            .affine_transform(frame())
            .unwrap(),
    ))
}
#[test]
fn sheared_reflected_sphere_is_a_native_program_operand() {
    let source = field();
    let p = SdfProgram::compile(source.clone(), &mut graph()).unwrap();
    assert!(Arc::ptr_eq(p.source(), &source));
    assert_eq!(
        p.program()
            .eval_real(&[6.into(), 1.into(), 3.into()], &mut graph())
            .unwrap(),
        vec![Real::from(-1)]
    );
}

#[test]
fn independent_affine_values_and_chain_rule_include_every_coordinate() {
    let p = SdfProgram::compile(field(), &mut graph()).unwrap();
    for x in -2i64..=2 {
        for y in -2i64..=2 {
            for z in -2i64..=2 {
                let a = x - 2 * y - 3;
                let expected = (Real::from(4 * a * a + 4 * y * y + (1 - z) * (1 - z) - 16)
                    / Real::from(4))
                .unwrap();
                assert_eq!(
                    p.source().evaluate_point(&point(x, y, z)).unwrap(),
                    expected
                );
                assert_eq!(
                    p.program()
                        .eval_real(&[x.into(), y.into(), z.into()], &mut graph())
                        .unwrap(),
                    vec![expected]
                );
            }
        }
    }
    // f=a²+b²+c²-4, grad=(2a,-4a+2b,-c); at (6,1,3): a=b=1,c=-1.
    let j = p
        .program()
        .jacobian(&p.program().definition().variables, &mut graph())
        .unwrap();
    assert_eq!(
        j.program()
            .eval_real(&[6.into(), 1.into(), 3.into()], &mut graph())
            .unwrap(),
        vec![Real::from(-1), Real::from(2), Real::from(-2), Real::one()]
    );
}

#[test]
fn transformed_cell_taylor_proof_keeps_correlation_and_reuses_native_source() {
    let expr = logistic()
        .offset((Real::from(3) / Real::from(8)).unwrap())
        .affine_transform(frame())
        .unwrap();
    let p = program(expr);
    // World x=5 and y in [1/2,1] give child x in [0,1]. At y=3/4,
    // the polynomial is -1/8 - 4*(y-3/4)^2: range [-3/8,-1/8].
    let cell = [d(5, 5, 0), d(1, 2, -1), d(1, 1, 0)];
    let natural = p
        .image(cell.clone(), -64, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(natural.image(), &d(-3, 5, -3));
    assert_eq!(natural.location(), Some(Location::Unknown));
    let t = p
        .taylor_cell(
            TaylorRequest {
                center: vec![d(5, 5, 0), d(3, 3, -2), d(1, 1, 0)],
                domain: cell.to_vec(),
                order: 2,
                precision: -64,
            },
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(t.image(), &d(-3, -1, -3));
    assert_eq!(t.location(), Some(Location::ConservativeInside));
    assert_eq!(t.metric_status(), SdfMetricStatus::SignEquivalent);
    let model = t.model().unwrap();
    assert_eq!(model.remainder(), &[zero()]);
    assert_eq!(
        model
            .enclose_on(
                &[d(5, 5, 0), d(3, 3, -2), d(1, 1, 0)],
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap(),
        vec![d(-1, -1, -3)]
    );
    assert!(
        model
            .enclose_on(
                &[d(5, 5, 0), d(2, 2, 0), d(1, 1, 0)],
                &mut graph(),
                &mut arithmetic()
            )
            .is_err()
    );
    let id = hypersolve::SymbolId(78);
    let outer =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(id).powi(2)], &[id], &mut graph())
            .unwrap();
    let next = outer.compose(p.program(), &[0], &mut graph()).unwrap();
    assert_eq!(
        next.program()
            .eval_real(
                &[5.into(), (Real::from(3) / Real::from(4)).unwrap(), 1.into()],
                &mut graph()
            )
            .unwrap(),
        vec![
            (Real::from(-1) / Real::from(8)).unwrap(),
            (Real::one() / Real::from(64)).unwrap()
        ]
    );
}

#[test]
fn native_algebraic_frame_and_raw_inverse_field_semantics_are_preserved() {
    let root = Real::from(2).sqrt().unwrap();
    let mut matrix = frame();
    matrix.0[0] = [root.clone(), 0.into(), 0.into(), 0.into()];
    let p = program(SdfExpr::x().affine_transform(matrix).unwrap());
    let value = p
        .program()
        .eval_real(&[root.clone(), 0.into(), 0.into()], &mut graph())
        .unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&value[0], &Real::one(), hyperlimit::PredicatePolicy::STRICT)
            .value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(
        hyperlimit::compare_reals(
            &p.source()
                .evaluate_point(&Point3::new(root, 0.into(), 0.into()))
                .unwrap(),
            &value[0],
            hyperlimit::PredicatePolicy::STRICT
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    // A public raw enum is not an invertibility certificate. Existing inverse_point
    // semantics uses its first three inverse rows without projective division.
    let mut inverse = frame();
    inverse.0[3] = [1.into(), 2.into(), 3.into(), 0.into()];
    let expr = SdfExpr::Transform {
        child: Box::new(SdfExpr::x()),
        transform: SdfTransform::Affine {
            matrix: frame(),
            inverse,
        },
    };
    let p = program(expr);
    assert_eq!(
        p.source().evaluate_point(&point(2, 3, 4)).unwrap(),
        Real::from(11)
    );
    assert_eq!(
        p.program()
            .eval_real(&[2.into(), 3.into(), 4.into()], &mut graph())
            .unwrap(),
        vec![Real::from(11)]
    );
    assert_eq!(p.source().metric_status(), SdfMetricStatus::SignEquivalent);
}

#[test]
fn nested_and_sibling_frames_keep_local_bindings_and_linear_growth() {
    let child = SdfExpr::x()
        .translate(point(1, 2, 3))
        .affine_transform(frame())
        .unwrap();
    let p = program(
        child
            .clone()
            .add_expr(SdfExpr::x())
            .affine_transform(frame())
            .unwrap()
            .add_expr(child),
    );
    // First frame has a=x-2y-3. Inner translated frame evaluates a-2y-4.
    // Total: (a-2y-4)+a+(x-2y-4) = 3x-8y-14.
    for x in -2i64..=2 {
        for y in -2i64..=2 {
            let expected = Real::from(3 * x - 8 * y - 14);
            assert_eq!(
                p.program()
                    .eval_real(&[x.into(), y.into(), 2.into()], &mut graph())
                    .unwrap(),
                vec![expected.clone()]
            );
            assert_eq!(
                p.source().evaluate_point(&point(x, y, 2)).unwrap(),
                expected
            );
        }
    }
    let mut expr = SdfExpr::x();
    for _ in 0..64 {
        expr = expr.affine_transform(frame()).unwrap();
    }
    let p = program(expr);
    assert_eq!(p.program().definition().nodes.len(), 3 + 64 * 30);
    assert_eq!(
        p.program()
            .eval_real(&[192.into(), 0.into(), 0.into()], &mut graph())
            .unwrap(),
        vec![Real::zero()]
    );
}

#[test]
fn affine_guards_empty_queries_and_typed_retry_survive_composition() {
    for radius in [Real::from(-1), uncertainty::terminal_zero(Real::zero())] {
        let expr = SdfExpr::constant(Real::zero()).mul_expr(
            SdfExpr::sphere(point(0, 0, 0), radius)
                .affine_transform(frame())
                .unwrap(),
        );
        let source = Arc::new(Sdf::new(expr));
        let original = source.evaluate_point(&point(0, 0, 0)).unwrap_err();
        let Err(SdfProgramError::Source(actual)) = SdfProgram::compile(source, &mut graph()) else {
            panic!("lost source admission error")
        };
        assert_eq!(actual, original);
        assert!(matches!(
            actual,
            SdfEvaluationError::InvalidDomain { .. } | SdfEvaluationError::Undecided { .. }
        ));
    }
    let p = program(SdfExpr::x().sqrt().affine_transform(frame()).unwrap());
    assert!(
        p.image(
            [zero(), zero(), zero()],
            -48,
            &mut graph(),
            &mut arithmetic()
        )
        .is_err()
    );
    assert_eq!(
        p.image(
            [DyadicInterval::empty(), zero(), zero()],
            -48,
            &mut graph(),
            &mut arithmetic()
        )
        .unwrap()
        .location(),
        None
    );
    assert!(
        p.image(
            [d(4, 4, 0), zero(), zero()],
            -48,
            &mut graph(),
            &mut arithmetic()
        )
        .is_ok()
    );
    for (source, entries) in [
        (
            Arc::new(Sdf::new(SdfExpr::x().affine_transform(frame()).unwrap())),
            38,
        ),
        (field(), 51),
    ] {
        let mut b = graph();
        let start = b.remaining_work();
        SdfProgram::compile(source.clone(), &mut b).unwrap();
        let used = start - b.remaining_work();
        assert!(
            SdfProgram::compile(source.clone(), &mut ExpressionBudget::new(entries, used)).is_ok()
        );
        assert!(matches!(
            SdfProgram::compile(
                source.clone(),
                &mut ExpressionBudget::new(entries - 1, used)
            ),
            Err(SdfProgramError::Expression(ExpressionError::SizeExhausted))
        ));
        assert!(matches!(
            SdfProgram::compile(
                source.clone(),
                &mut ExpressionBudget::new(entries, used - 1)
            ),
            Err(SdfProgramError::Expression(ExpressionError::WorkExhausted))
        ));
        let token = CancellationToken::new();
        token.cancel();
        let mut b = graph();
        b.cancellation = Some(token);
        assert!(matches!(
            SdfProgram::compile(source.clone(), &mut b),
            Err(SdfProgramError::Expression(ExpressionError::Cancelled))
        ));
        assert!(SdfProgram::compile(source, &mut graph()).is_ok());
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn affine_compilation_reuses_retained_inverse_without_scalar_or_predicate_dispatch() {
    use hyperreal::dispatch_trace;
    let source = Arc::new(Sdf::new(SdfExpr::x().affine_transform(frame()).unwrap()));
    dispatch_trace::reset();
    let p = dispatch_trace::with_recording(|| {
        SdfProgram::compile(source.clone(), &mut graph()).unwrap()
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
    assert!(Arc::ptr_eq(p.source(), &source));
}

#[test]
fn fresh_process_replays_affine_recipe_and_reestablishes_taylor_proof() {
    const KEY: &str = "HYPERSDF_AFFINE_REPLAY";
    if std::env::var(KEY).is_ok() {
        transformed_cell_taylor_proof_keeps_correlation_and_reuses_native_source();
        return;
    }
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "affine_acceptance::fresh_process_replays_affine_recipe_and_reestablishes_taylor_proof",
            "--nocapture",
        ])
        .env(KEY, "1")
        .status()
        .unwrap();
    assert!(status.success());
}
