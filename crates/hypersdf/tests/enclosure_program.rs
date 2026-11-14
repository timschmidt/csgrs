#![cfg(feature = "enclosure-program")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice::Vector3;
use hyperlimit::Point3;
use hyperreal::Real;
use hypersdf::{
    Sdf, SdfCellLocation as Location, SdfExpr, SdfMetricStatus, SdfProgram, SdfProgramError,
};
use hypersolve::{
    Expr,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram, TaylorRequest},
    work_budget::CancellationToken,
};
use std::sync::Arc;
fn graph() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn zero() -> DyadicInterval {
    d(0, 0, 0)
}
fn program(expr: SdfExpr) -> SdfProgram {
    SdfProgram::compile(Arc::new(Sdf::new(expr)), &mut graph()).unwrap()
}
fn point(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn logistic() -> SdfExpr {
    SdfExpr::x().mul_expr(SdfExpr::constant(Real::one()).sub_expr(SdfExpr::x()))
}

#[test]
fn complete_cells_have_certified_signs_without_metric_promotion() {
    let source = Arc::new(Sdf::new(SdfExpr::x().sin().add_expr(SdfExpr::y().cos())));
    assert!(
        source
            .interval_cell(&point(0, 0, 0), &point(1, 0, 0))
            .interval
            .is_none()
    );
    let p = SdfProgram::compile(source.clone(), &mut graph()).unwrap();
    assert!(Arc::ptr_eq(p.source(), &source));
    let cell = [d(0, 1, 0), zero(), d(-20, 20, 0)];
    let result = p
        .image(cell.clone(), -48, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(result.location(), Some(Location::ConservativeOutside));
    assert_eq!(result.metric_status(), SdfMetricStatus::SignEquivalent);
    assert_eq!(result.cell(), &cell);
    assert!(result.model().is_none());
    let negative = program(SdfExpr::x().sin().add_expr(SdfExpr::y().cos()).complement());
    assert_eq!(
        negative
            .image(cell, -48, &mut graph(), &mut arithmetic())
            .unwrap()
            .location(),
        Some(Location::ConservativeInside)
    );
    let crossing = program(SdfExpr::x().sin());
    assert_eq!(
        crossing
            .image(
                [d(-1, 1, 0), zero(), zero()],
                -48,
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap()
            .location(),
        Some(Location::Unknown)
    );
    assert_eq!(
        crossing
            .image(
                [zero(), zero(), zero()],
                -48,
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap()
            .location(),
        Some(Location::Boundary)
    );
    assert_eq!(
        crossing
            .image(
                [DyadicInterval::empty(), zero(), zero()],
                -48,
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap()
            .location(),
        None
    );
}

#[test]
fn exact_translation_linear_and_arithmetic_match_independent_values() {
    let expr = SdfExpr::linear(Vector3([2.into(), (-3).into(), 5.into()]), (-7).into())
        .mul_expr(SdfExpr::x().sub_expr(SdfExpr::y()))
        .offset(4.into())
        .translate(point(1, -2, 3))
        .translate(point(2, 1, -1));
    let p = program(expr);
    for x in -3i64..=3 {
        for y in -3i64..=3 {
            let z = 2;
            let a = x - 3;
            let b = y + 1;
            let c = z - 2;
            let expected = Real::from((2 * a - 3 * b + 5 * c - 7) * (a - b) - 4);
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
    let p = program(
        SdfExpr::x()
            .sqrt()
            .add_expr(SdfExpr::y().tan())
            .add_expr(SdfExpr::z().cos()),
    );
    assert_eq!(
        p.program()
            .eval_real(&[4.into(), 0.into(), 0.into()], &mut graph())
            .unwrap(),
        vec![Real::from(3)]
    );
}

#[test]
fn taylor_dependency_reduction_retains_full_remainder_and_composes() {
    let p = program(logistic().offset((Real::from(3) / Real::from(8)).unwrap()));
    let cell = [d(0, 1, 0), zero(), zero()];
    let natural = p
        .image(cell.clone(), -64, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(natural.image(), &d(-3, 5, -3));
    assert_eq!(natural.location(), Some(Location::Unknown));
    let request = TaylorRequest {
        center: vec![d(1, 1, -1), zero(), zero()],
        domain: cell.to_vec(),
        order: 2,
        precision: -64,
    };
    let t = p
        .taylor_cell(request, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(t.image(), &d(-3, -1, -3));
    assert_eq!(t.location(), Some(Location::ConservativeInside));
    let model = t.model().unwrap();
    assert_eq!(model.remainder(), &[zero()]);
    assert_eq!(
        model
            .enclose_on(
                &[d(1, 1, -1), zero(), zero()],
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap(),
        vec![d(-1, -1, -3)]
    );
    assert!(
        model
            .enclose_on(
                &[d(2, 2, 0), zero(), zero()],
                &mut graph(),
                &mut arithmetic()
            )
            .is_err()
    );
    assert_eq!(
        model
            .enclose_on(
                &[DyadicInterval::empty(), zero(), zero()],
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap(),
        vec![DyadicInterval::empty()]
    );
    let id = hypersolve::SymbolId(55);
    let outer =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(id).powi(2)], &[id], &mut graph())
            .unwrap();
    let composed = outer
        .compose(t.source().program(), &[0], &mut graph())
        .unwrap();
    assert_eq!(
        composed
            .program()
            .eval_real(
                &[(Real::from(1) / Real::from(2)).unwrap(), 0.into(), 0.into()],
                &mut graph()
            )
            .unwrap(),
        vec![
            (Real::from(-1) / Real::from(8)).unwrap(),
            (Real::from(1) / Real::from(64)).unwrap()
        ]
    );
}

#[test]
fn original_guards_and_unsupported_forms_are_not_empty_or_success() {
    let p =
        program(SdfExpr::constant(Real::zero()).mul_expr(SdfExpr::constant(Real::from(-1)).sqrt()));
    assert!(
        p.program()
            .eval_real(&[0.into(), 0.into(), 0.into()], &mut graph())
            .is_err()
    );
    assert!(
        p.image(
            [zero(), zero(), zero()],
            -32,
            &mut graph(),
            &mut arithmetic()
        )
        .is_err()
    );
    assert_eq!(
        p.image(
            [zero(), DyadicInterval::empty(), zero()],
            -32,
            &mut graph(),
            &mut arithmetic()
        )
        .unwrap()
        .location(),
        None
    );
    let sphere = program(SdfExpr::sphere(point(0, 0, 0), Real::one()));
    assert_eq!(
        sphere
            .program()
            .eval_real(&[2.into(), 0.into(), 0.into()], &mut graph())
            .unwrap(),
        vec![Real::from(3)]
    );
    let tangent = program(SdfExpr::x().tan());
    assert!(
        tangent
            .image(
                [d(1, 2, 0), zero(), zero()],
                -48,
                &mut graph(),
                &mut arithmetic()
            )
            .is_err()
    );
    let root = program(SdfExpr::x().sqrt());
    assert!(
        root.image(
            [d(0, 1, 0), zero(), zero()],
            -32,
            &mut graph(),
            &mut arithmetic()
        )
        .is_ok()
    );
    assert!(
        root.taylor_cell(
            TaylorRequest {
                center: vec![d(1, 1, -1), zero(), zero()],
                domain: vec![d(0, 1, 0), zero(), zero()],
                order: 1,
                precision: -32
            },
            &mut graph(),
            &mut arithmetic()
        )
        .is_err()
    );
    for expr in [
        SdfExpr::x().abs(),
        SdfExpr::x().union(SdfExpr::y()),
        SdfExpr::x().intersection(SdfExpr::y()),
        SdfExpr::aabb(point(0, 0, 0), point(1, 1, 1)),
    ] {
        assert!(matches!(
            SdfProgram::compile(Arc::new(Sdf::new(expr)), &mut graph()),
            Err(SdfProgramError::Unsupported(_))
        ));
    }
}

#[test]
fn work_size_cancellation_and_late_publication_failure_retry() {
    let source = Arc::new(Sdf::new(logistic()));
    let mut b = graph();
    let start = b.remaining_work();
    let p = SdfProgram::compile(source.clone(), &mut b).unwrap();
    let used = start - b.remaining_work();
    assert!(
        SdfProgram::compile(source.clone(), &mut ExpressionBudget::new(1_000_000, used)).is_ok()
    );
    assert!(matches!(
        SdfProgram::compile(
            source.clone(),
            &mut ExpressionBudget::new(1_000_000, used - 1)
        ),
        Err(SdfProgramError::Expression(ExpressionError::WorkExhausted))
    ));
    assert!(matches!(
        SdfProgram::compile(source.clone(), &mut ExpressionBudget::new(1, 1000000)),
        Err(SdfProgramError::Expression(ExpressionError::SizeExhausted))
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
    let cell = [d(0, 1, 0), zero(), zero()];
    let mut b = graph();
    let start = b.remaining_work();
    p.image(cell.clone(), -32, &mut b, &mut arithmetic())
        .unwrap();
    let used = start - b.remaining_work();
    assert!(matches!(
        p.image(
            cell.clone(),
            -32,
            &mut ExpressionBudget::new(1_000_000, used - 1),
            &mut arithmetic()
        ),
        Err(SdfProgramError::Expression(ExpressionError::WorkExhausted))
    ));
    assert!(
        p.image(
            cell,
            -32,
            &mut ExpressionBudget::new(1_000_000, used),
            &mut arithmetic()
        )
        .is_ok()
    );
}

#[test]
fn fresh_process_rebuilds_original_source_and_proof() {
    const KEY: &str = "HYPERSDF_ENCLOSURE_REPLAY_OFFSET_EIGHTHS";
    if let Ok(value) = std::env::var(KEY) {
        let amount = value.parse::<i64>().unwrap();
        let p = program(logistic().offset((Real::from(amount) / Real::from(8)).unwrap()));
        let r = p
            .taylor_cell(
                TaylorRequest {
                    center: vec![d(1, 1, -1), zero(), zero()],
                    domain: vec![d(0, 1, 0), zero(), zero()],
                    order: 2,
                    precision: -64,
                },
                &mut graph(),
                &mut arithmetic(),
            )
            .unwrap();
        assert_eq!(r.image(), &d(-3, -1, -3));
        assert_eq!(r.model().unwrap().remainder(), &[zero()]);
        assert_eq!(r.location(), Some(Location::ConservativeInside));
        return;
    }
    // Replay the authored recipe and exact parameter, not generated graph flags.
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_rebuilds_original_source_and_proof",
            "--nocapture",
        ])
        .env(KEY, "3")
        .status()
        .unwrap();
    assert!(status.success());
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn compilation_does_not_run_predicates_or_scalar_dispatch() {
    use hyperreal::dispatch_trace;
    let source = Arc::new(Sdf::new(logistic().translate(point(1, 2, 3))));
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
fn nested_translation_keeps_source_guards_and_linear_graph_growth() {
    let mut expr = logistic();
    for _ in 0..128 {
        expr = expr.translate(point(1, 0, 0));
    }
    let p = program(expr);
    assert_eq!(p.program().definition().nodes.len(), 3 + 128 * 6 + 3);
    assert_eq!(
        p.program()
            .eval_real(&[128.into(), 0.into(), 0.into()], &mut graph())
            .unwrap(),
        vec![Real::zero()]
    );
    let malformed = TaylorRequest {
        center: vec![zero()],
        domain: vec![zero()],
        order: 2,
        precision: -32,
    };
    assert!(
        p.taylor_cell(malformed, &mut graph(), &mut arithmetic())
            .is_err()
    );
}

#[test]
fn constant_compilation_does_not_reserve_unrelated_linear_expansion() {
    let source = Arc::new(Sdf::new(SdfExpr::constant(Real::one())));
    // Three bindings, four nodes, one output, and one retained traversal result.
    assert!(SdfProgram::compile(source, &mut ExpressionBudget::new(9, 1000)).is_ok());
}

#[path = "support/enclosure_primitives.rs"]
mod primitive_acceptance;

#[path = "support/uncertainty.rs"]
mod uncertainty;

#[path = "support/enclosure_affine.rs"]
mod affine_acceptance;
