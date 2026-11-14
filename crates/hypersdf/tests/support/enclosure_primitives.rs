use super::*;
use hyperlimit::{Plane3, PredicatePolicy, compare_reals};
use hypersdf::{SdfCoordinate, SdfEvaluationError};
use std::error::Error;
#[path = "uncertainty.rs"]
mod uncertainty;

#[test]
fn polynomial_primitives_match_independent_integer_geometry_on_every_axis() {
    let plane = program(SdfExpr::plane(Plane3::new(point(2, -3, 5), (-7).into())));
    let sphere = program(SdfExpr::sphere(point(1, -2, 3), 9.into()));
    let tori = [SdfCoordinate::X, SdfCoordinate::Y, SdfCoordinate::Z]
        .map(|axis| program(SdfExpr::torus(axis, point(1, -2, 3), 4.into(), 1.into())));
    for x in -2i64..=2 {
        for y in -2i64..=2 {
            for z in -2i64..=2 {
                let args = [Real::from(x), Real::from(y), Real::from(z)];
                let delta = [x - 1, y + 2, z - 3];
                let squares = delta.map(|v| v * v);
                let sum = squares.iter().sum::<i64>();
                let expectations = [(&plane, 2 * x - 3 * y + 5 * z - 7), (&sphere, sum - 9)];
                for (p, expected) in expectations {
                    assert_eq!(
                        p.program().eval_real(&args, &mut graph()).unwrap(),
                        vec![Real::from(expected)]
                    );
                    assert_eq!(
                        p.source().evaluate_point(&point(x, y, z)).unwrap(),
                        Real::from(expected)
                    );
                }
                for (axis, p) in tori.iter().enumerate() {
                    let radial = sum - squares[axis];
                    let q = sum + 3;
                    let expected = q * q - 16 * radial;
                    assert_eq!(
                        p.program().eval_real(&args, &mut graph()).unwrap(),
                        vec![Real::from(expected)]
                    );
                    assert_eq!(
                        p.source().evaluate_point(&point(x, y, z)).unwrap(),
                        Real::from(expected)
                    );
                }
            }
        }
    }
}

#[test]
fn exact_algebraic_centers_and_degenerate_parameters_keep_their_meaning() {
    let root = Real::from(2).sqrt().unwrap();
    let center = Point3::new(root.clone(), Real::zero(), Real::zero());
    let p = program(SdfExpr::sphere(center.clone(), 2.into()));
    let at_origin = p
        .program()
        .eval_real(&[0.into(), 0.into(), 0.into()], &mut graph())
        .unwrap();
    assert_eq!(
        compare_reals(&at_origin[0], &Real::zero(), PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(
        p.program()
            .eval_real(&[root, 0.into(), 0.into()], &mut graph())
            .unwrap(),
        vec![Real::from(-2)]
    );
    let t = p
        .taylor_cell(
            TaylorRequest {
                center: vec![zero(); 3],
                domain: vec![zero(); 3],
                order: 2,
                precision: -64,
            },
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(t.image(), &zero());
    assert_eq!(t.model().unwrap().remainder(), &[zero()]);
    let zero_radius = program(SdfExpr::sphere(center, Real::zero()));
    assert_eq!(
        zero_radius
            .image(
                [DyadicInterval::empty(), zero(), zero()],
                -32,
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap()
            .location(),
        None
    );
    let collapsed_plane = program(SdfExpr::plane(Plane3::new(point(0, 0, 0), (-2).into())));
    assert_eq!(
        collapsed_plane
            .image(
                core::array::from_fn(|_| d(-1, 1, 0)),
                -32,
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap()
            .image(),
        &d(-2, -2, 0)
    );
    let zero_minor = program(SdfExpr::torus(
        SdfCoordinate::Z,
        point(0, 0, 0),
        4.into(),
        0.into(),
    ));
    assert_eq!(
        zero_minor
            .program()
            .eval_real(&[2.into(), 0.into(), 0.into()], &mut graph())
            .unwrap(),
        vec![Real::zero()]
    );
}

#[test]
fn native_squares_taylor_and_later_composition_preserve_complete_source() {
    let p = program(SdfExpr::sphere(point(0, 0, 0), 4.into()).translate(point(3, 0, 0)));
    let r = p
        .image(
            [d(2, 4, 0), d(-1, 1, 0), d(-1, 1, 0)],
            -64,
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(r.image(), &d(-4, -1, 0));
    assert_eq!(r.location(), Some(Location::ConservativeInside));
    let torus = program(SdfExpr::torus(
        SdfCoordinate::Z,
        point(0, 0, 0),
        4.into(),
        1.into(),
    ));
    let jacobian = torus
        .program()
        .jacobian(&torus.program().definition().variables, &mut graph())
        .unwrap();
    // At (3,0,4), q=28: f=640, df/dx=4*x*q-32*x=240,
    // df/dy=0, and df/dz=4*z*q=448.
    assert_eq!(
        jacobian
            .program()
            .eval_real(&[3.into(), 0.into(), 4.into()], &mut graph())
            .unwrap(),
        vec![
            Real::from(640),
            Real::from(240),
            Real::zero(),
            Real::from(448)
        ]
    );
    let model = torus
        .taylor_cell(
            TaylorRequest {
                center: vec![d(2, 2, 0), zero(), zero()],
                domain: vec![d(1, 3, 0), zero(), zero()],
                order: 4,
                precision: -64,
            },
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(model.model().unwrap().remainder(), &[zero()]);
    assert_eq!(
        model
            .model()
            .unwrap()
            .enclose_on(
                &[d(2, 2, 0), zero(), zero()],
                &mut graph(),
                &mut arithmetic()
            )
            .unwrap(),
        vec![d(-15, -15, 0)]
    );
    let id = hypersolve::SymbolId(77);
    let outer =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(id).powi(2)], &[id], &mut graph())
            .unwrap();
    let next = outer.compose(torus.program(), &[0], &mut graph()).unwrap();
    assert_eq!(
        next.program()
            .eval_real(&[2.into(), 0.into(), 0.into()], &mut graph())
            .unwrap(),
        vec![Real::from(-15), Real::from(225)]
    );
}

#[test]
fn invalid_and_undecided_primitive_parameters_retain_native_causes() {
    let invalid = [
        SdfExpr::sphere(point(0, 0, 0), (-1).into()),
        SdfExpr::torus(SdfCoordinate::Z, point(0, 0, 0), 0.into(), 1.into()),
        SdfExpr::torus(SdfCoordinate::Y, point(0, 0, 0), 4.into(), (-1).into()),
    ];
    for expr in invalid {
        let source = Arc::new(Sdf::new(SdfExpr::constant(Real::zero()).mul_expr(expr)));
        let original = source.evaluate_point(&point(0, 0, 0)).unwrap_err();
        let error = SdfProgram::compile(source, &mut graph()).unwrap_err();
        assert!(error.source().is_some());
        let SdfProgramError::Source(actual) = error else {
            panic!("original source cause lost")
        };
        assert_eq!(actual, original);
        assert!(matches!(actual, SdfEvaluationError::InvalidDomain { .. }));
    }
    let unknown = uncertainty::terminal_zero(Real::zero());
    let source = Arc::new(Sdf::new(SdfExpr::sphere(point(0, 0, 0), unknown)));
    assert!(matches!(
        SdfProgram::compile(source, &mut graph()),
        Err(SdfProgramError::Source(
            SdfEvaluationError::Undecided { .. }
        ))
    ));
}

#[test]
fn primitive_entry_work_limits_cancellation_and_retry_are_exact() {
    for (expr, entries) in [
        (SdfExpr::plane(Plane3::new(point(1, 2, 3), 4.into())), 18),
        (SdfExpr::sphere(point(1, 2, 3), 4.into()), 21),
        (
            SdfExpr::torus(SdfCoordinate::X, point(1, 2, 3), 4.into(), 1.into()),
            28,
        ),
    ] {
        let source = Arc::new(Sdf::new(expr));
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
fn parameter_admission_reuses_the_exact_native_dispatch_counts() {
    use hyperreal::dispatch_trace as trace;
    // The shared linear emitter preserves the original arithmetic-only path.
    let sources = [
        SdfExpr::linear(Vector3([2.into(), (-3).into(), 5.into()]), (-7).into()),
        SdfExpr::plane(Plane3::new(point(2, -3, 5), (-7).into())),
    ];
    for expr in sources {
        let source = Arc::new(Sdf::new(expr));
        trace::reset();
        trace::with_recording(|| SdfProgram::compile(source, &mut graph()).unwrap());
        let actual = trace::take_trace();
        assert!(actual.dispatch.is_empty());
        assert_eq!(actual.rational, Default::default());
    }
    let radius = Real::from(4);
    let minor = Real::one();
    let sphere = Arc::new(Sdf::new(SdfExpr::sphere(point(0, 0, 0), radius.clone())));
    trace::reset();
    trace::with_recording(|| {
        assert_eq!(
            compare_reals(&radius, &Real::zero(), PredicatePolicy::STRICT).value(),
            Some(std::cmp::Ordering::Greater)
        );
    });
    let expected = trace::take_trace();
    trace::reset();
    trace::with_recording(|| SdfProgram::compile(sphere, &mut graph()).unwrap());
    let actual = trace::take_trace();
    assert_eq!(actual, expected);
    let torus = Arc::new(Sdf::new(SdfExpr::torus(
        SdfCoordinate::Z,
        point(0, 0, 0),
        radius.clone(),
        minor.clone(),
    )));
    trace::reset();
    trace::with_recording(|| {
        assert!(
            hyperlimit::classify_real_sign_pair(&radius, &minor, PredicatePolicy::STRICT)
                .value()
                .is_some()
        );
        // The torus polynomial additionally constructs its exact coefficient 4.
        // Compare the full trace, including this constructor and rational count.
        let _four = Real::from(4);
    });
    let expected = trace::take_trace();
    trace::reset();
    trace::with_recording(|| SdfProgram::compile(torus, &mut graph()).unwrap());
    assert_eq!(trace::take_trace(), expected);
}

#[test]
fn fresh_process_recompiles_primitive_recipe_before_using_taylor_proof() {
    const KEY: &str = "HYPERSDF_PRIMITIVE_REPLAY_RADIUS_SQUARED";
    if let Ok(radius) = std::env::var(KEY) {
        let radius = radius.parse::<i64>().unwrap();
        let p = program(SdfExpr::sphere(point(1, 0, 0), radius.into()).translate(point(2, 0, 0)));
        let r = p
            .taylor_cell(
                TaylorRequest {
                    center: vec![d(3, 3, 0), zero(), zero()],
                    domain: vec![d(2, 4, 0), d(-1, 1, 0), d(-1, 1, 0)],
                    order: 2,
                    precision: -64,
                },
                &mut graph(),
                &mut arithmetic(),
            )
            .unwrap();
        assert_eq!(r.model().unwrap().remainder(), &[zero()]);
        assert_eq!(r.image(), &d(-4, -1, 0));
        assert_eq!(r.location(), Some(Location::ConservativeInside));
        return;
    }
    assert!(std::process::Command::new(std::env::current_exe().unwrap()).args(["--exact","primitive_acceptance::fresh_process_recompiles_primitive_recipe_before_using_taylor_proof"]).env(KEY,"4").status().unwrap().success());
}

#[test]
fn quartic_torus_taylor_proves_a_full_cell_that_natural_evaluation_cannot() {
    let p = program(SdfExpr::torus(
        SdfCoordinate::Z,
        point(0, 0, 0),
        4.into(),
        1.into(),
    ));
    let cell = [d(7, 9, -2), d(-1, 1, -2), d(-1, 1, -2)];
    let natural = p
        .image(cell.clone(), -64, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(natural.image(), &d(-11583, 4617, -8));
    assert_eq!(natural.location(), Some(Location::Unknown));
    let proof = p
        .taylor_cell(
            TaylorRequest {
                center: vec![d(2, 2, 0), zero(), zero()],
                domain: cell.to_vec(),
                order: 4,
                precision: -64,
            },
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    // Around (2,0,0), write A=u²+v²+w². The exact polynomial is
    // -15-8u+14u²-2v²+14w²+8u*A+A², for |u|,|v|,|w| <= 1/4.
    // Its separate monomial bounds are [-35/2, -2775/256].
    assert_eq!(proof.model().unwrap().remainder(), &[zero()]);
    assert_eq!(proof.image(), &d(-4480, -2775, -8));
    assert_eq!(proof.location(), Some(Location::ConservativeInside));
}
