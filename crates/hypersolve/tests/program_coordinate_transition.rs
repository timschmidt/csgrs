#![cfg(feature = "coordinate-transition")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, ExpressionBudget, ExpressionProgram},
};
const X: SymbolId = SymbolId(0);
const Y: SymbolId = SymbolId(1);
fn budget() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn chart() -> CoordinateChange {
    CoordinateChange {
        variables: vec![X, Y],
        to_source: vec![vec![1.into(), 128.into()], vec![0.into(), 1.into()]],
        from_source: vec![vec![1.into(), (-128).into()], vec![0.into(), 1.into()]],
        origin: vec![10.into(), (-3).into()],
    }
}
#[test]
fn identical_sheared_charts_compose_exactly_before_enclosure() {
    let p =
        ExpressionProgram::compile(&[Expr::int(2), Expr::int(1)], &[Y, X], &mut budget()).unwrap();
    let a = p.change_coordinates(chart(), &mut budget()).unwrap();
    let b = p.change_coordinates(chart(), &mut budget()).unwrap();
    let map = a.transition_to(&b, &mut budget()).unwrap();
    assert_eq!(
        map.matrix(),
        &[
            vec![Real::one(), Real::zero()],
            vec![Real::zero(), Real::one()]
        ]
    );
    assert_eq!(map.origin(), &[Real::zero(), Real::zero()]);
    assert_eq!(
        map.program()
            .eval_real(&[2.into(), 3.into()], &mut budget())
            .unwrap(),
        vec![Real::from(2), Real::from(3)]
    );
}

fn source() -> ExpressionProgram {
    ExpressionProgram::compile(&[Expr::int(2), Expr::int(1)], &[Y, X], &mut budget()).unwrap()
}
#[test]
fn independent_changed_chart_coefficients_and_point_values() {
    let p = source();
    let a = p.change_coordinates(chart(), &mut budget()).unwrap();
    let b = p
        .change_coordinates(
            CoordinateChange {
                variables: vec![X, Y],
                to_source: vec![vec![2.into(), 0.into()], vec![0.into(), (-1).into()]],
                from_source: vec![
                    vec![(Real::one() / Real::from(2)).unwrap(), 0.into()],
                    vec![0.into(), (-1).into()],
                ],
                origin: vec![6.into(), 2.into()],
            },
            &mut budget(),
        )
        .unwrap();
    let m = a.transition_to(&b, &mut budget()).unwrap();
    assert_eq!(
        m.matrix(),
        &[
            vec![(Real::one() / Real::from(2)).unwrap(), 64.into()],
            vec![0.into(), (-1).into()]
        ]
    );
    assert_eq!(m.origin(), &[Real::from(2), Real::from(5)]);
    for y in -3..=3 {
        for x in -3..=3 {
            let z = m
                .program()
                .eval_real(&[Real::from(2 * x), y.into()], &mut budget())
                .unwrap();
            assert_eq!(z, vec![Real::from(2 + x + 64 * y), Real::from(5 - y)]);
            // Independent original-coordinate equality, not agreement of wrappers.
            assert_eq!(
                &z[0] * Real::from(2) + Real::from(6),
                Real::from(10 + 2 * x + 128 * y)
            );
            assert_eq!(Real::from(2) - &z[1], Real::from(-3 + y));
        }
    }
    assert_eq!(m.from().inverse_evidence().len(), 4);
    assert_eq!(m.to().inverse_evidence().len(), 4);
}
#[test]
fn algebraic_scaling_translation_and_zero_dimension() {
    let s = Real::from(2).sqrt().unwrap();
    let p = ExpressionProgram::compile(&[Expr::int(1)], &[X], &mut budget()).unwrap();
    let c = CoordinateChange {
        variables: vec![X],
        to_source: vec![vec![s.clone()]],
        from_source: vec![vec![s.clone().inverse().unwrap()]],
        origin: vec![s.clone()],
    };
    let a = p.change_coordinates(c.clone(), &mut budget()).unwrap();
    let b = p
        .change_coordinates(
            CoordinateChange {
                origin: vec![&s * Real::from(2)],
                ..c
            },
            &mut budget(),
        )
        .unwrap();
    let m = a.transition_to(&b, &mut budget()).unwrap();
    assert_eq!(m.matrix(), &[vec![Real::one()]]);
    assert_eq!(m.origin(), &[Real::from(-1)]);
    assert_eq!(
        m.program().eval_real(&[3.into()], &mut budget()).unwrap(),
        vec![Real::from(2)]
    );
    let p = ExpressionProgram::compile(&[], &[], &mut budget()).unwrap();
    let a = p
        .change_coordinates(
            CoordinateChange {
                variables: vec![],
                to_source: vec![],
                from_source: vec![],
                origin: vec![],
            },
            &mut budget(),
        )
        .unwrap();
    let m = a.transition_to(&a, &mut budget()).unwrap();
    assert!(m.matrix().is_empty());
    assert!(m.origin().is_empty());
    assert!(
        m.program()
            .eval_real(&[], &mut budget())
            .unwrap()
            .is_empty()
    );
}
#[test]
fn structural_reconstruction_is_admitted_but_foreign_sources_and_orders_are_not() {
    use hypersolve::expression_program::CoordinateTransitionError as E;
    let a = source().change_coordinates(chart(), &mut budget()).unwrap();
    let b = source().change_coordinates(chart(), &mut budget()).unwrap();
    a.transition_to(&b, &mut budget()).unwrap();
    let p =
        ExpressionProgram::compile(&[Expr::int(3), Expr::int(1)], &[Y, X], &mut budget()).unwrap();
    let b = p.change_coordinates(chart(), &mut budget()).unwrap();
    assert!(matches!(
        a.transition_to(&b, &mut budget()),
        Err(E::SourceMismatch)
    ));
    let mut c = chart();
    c.variables.swap(0, 1);
    let b = source().change_coordinates(c, &mut budget()).unwrap();
    assert!(matches!(
        a.transition_to(&b, &mut budget()),
        Err(E::VariableMismatch)
    ));
    let mut invalid = chart();
    invalid.from_source[0][1] = 0.into();
    assert!(source().change_coordinates(invalid, &mut budget()).is_err());
}
#[test]
fn preparation_final_work_and_cancellation_do_not_publish_partial_relations() {
    use hypersolve::expression_program::{
        CoordinateError, CoordinateTransitionError as E, ExpressionError,
    };
    let a = source().change_coordinates(chart(), &mut budget()).unwrap();
    let mut full = budget();
    a.transition_to(&a, &mut full).unwrap();
    let used = budget().remaining_work() - full.remaining_work();
    let mut short = ExpressionBudget::new(1_000_000, used - 1);
    assert!(matches!(
        a.transition_to(&a, &mut short),
        Err(E::Coordinate(CoordinateError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    let mut exact = ExpressionBudget::new(1_000_000, used);
    a.transition_to(&a, &mut exact).unwrap();
    assert_eq!(exact.remaining_work(), 0);
    assert!(
        a.transition_to(&a, &mut ExpressionBudget::new(0, 100_000_000))
            .is_err()
    );
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        a.transition_to(&a, &mut cancelled),
        Err(E::Coordinate(CoordinateError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    a.transition_to(&a, &mut budget()).unwrap();
}
