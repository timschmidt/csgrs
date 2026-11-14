#![cfg(feature = "qr-coordinates")]
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram, QrCoordinateRequest},
};
fn budget() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
#[test]
fn exact_rotated_frame_is_built_without_an_authored_inverse() {
    let [x, y] = [SymbolId(0), SymbolId(1)];
    let source =
        ExpressionProgram::compile(&[Expr::int(1), Expr::int(0)], &[y, x], &mut budget()).unwrap();
    let qr = source
        .qr_coordinates(
            QrCoordinateRequest {
                variables: vec![x, y],
                basis: vec![vec![3.into(), (-4).into()], vec![4.into(), 3.into()]],
                origin: vec![0.into(), 0.into()],
            },
            &mut budget(),
        )
        .unwrap();
    let fifth = (Real::one() / Real::from(5)).unwrap();
    assert_eq!(
        qr.coordinates().change().to_source,
        vec![
            vec![&fifth * Real::from(3), &fifth * Real::from(-4)],
            vec![&fifth * Real::from(4), &fifth * Real::from(3)]
        ]
    );
    assert_eq!(
        qr.upper(),
        &[
            vec![Real::from(5), Real::zero()],
            vec![Real::zero(), Real::from(5)]
        ]
    );
    assert_eq!(qr.reconstruction_evidence().len(), 4);
}
fn source(n: usize) -> ExpressionProgram {
    let variables: Vec<_> = (0..n)
        .map(|i| SymbolId(u32::try_from(i).unwrap()))
        .collect();
    ExpressionProgram::compile(&vec![Expr::int(0); n], &variables, &mut budget()).unwrap()
}
fn request(basis: Vec<Vec<Real>>) -> QrCoordinateRequest {
    let n = basis.len();
    QrCoordinateRequest {
        variables: (0..n)
            .map(|i| SymbolId(u32::try_from(i).unwrap()))
            .collect(),
        basis,
        origin: vec![Real::zero(); n],
    }
}
fn ratio(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
#[test]
fn three_dimensional_decomposition_has_independent_exact_expected_factors() {
    let basis = vec![
        vec![12.into(), (-51).into(), 4.into()],
        vec![6.into(), 167.into(), (-68).into()],
        vec![(-4).into(), 24.into(), (-41).into()],
    ];
    let qr = source(3)
        .qr_coordinates(request(basis), &mut budget())
        .unwrap();
    assert_eq!(
        qr.coordinates().change().to_source,
        vec![
            vec![ratio(6, 7), ratio(-69, 175), ratio(-58, 175)],
            vec![ratio(3, 7), ratio(158, 175), ratio(6, 175)],
            vec![ratio(-2, 7), ratio(6, 35), ratio(-33, 35)]
        ]
    );
    assert_eq!(
        qr.upper(),
        &[[14, 21, -14], [0, 175, -70], [0, 0, 35]].map(|row| row.map(Real::from).to_vec())
    );
    assert_eq!(
        qr.squared_norms(),
        &[Real::from(196), Real::from(30625), Real::from(1225)]
    );
    use hyperlimit::{Certainty, PredicateOutcome, Sign};
    for proof in qr.norm_evidence() {
        assert!(matches!(
            proof,
            PredicateOutcome::Decided {
                value: Sign::Positive,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
    }
    for proof in qr
        .reconstruction_evidence()
        .iter()
        .chain(qr.coordinates().inverse_evidence())
    {
        assert!(matches!(
            proof,
            PredicateOutcome::Decided {
                value: Sign::Zero,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
    }
    assert_eq!(qr.norm_evidence().len(), 3);
    assert_eq!(qr.reconstruction_evidence().len(), 9);
}
#[test]
fn algebraic_reflections_near_dependence_and_unit_dimension_keep_exact_meaning() {
    let qr = source(2)
        .qr_coordinates(
            request(vec![vec![1.into(), 1.into()], vec![1.into(), (-1).into()]]),
            &mut budget(),
        )
        .unwrap();
    let s = Real::from(2).sqrt().unwrap();
    let h = (&s / Real::from(2)).unwrap();
    assert_eq!(
        qr.coordinates().change().to_source,
        vec![vec![h.clone(), h.clone()], vec![h.clone(), -h]]
    );
    assert_eq!(qr.upper(), &[vec![s.clone(), 0.into()], vec![0.into(), s]]);
    let qr = source(1)
        .qr_coordinates(request(vec![vec![(-7).into()]]), &mut budget())
        .unwrap();
    assert_eq!(
        qr.coordinates().change().to_source,
        vec![vec![Real::from(-1)]]
    );
    assert_eq!(qr.upper(), &[vec![Real::from(7)]]);
    let tiny = (Real::one() / Real::from(1_u128 << 100)).unwrap();
    let qr = source(2)
        .qr_coordinates(
            request(vec![vec![1.into(), 1.into()], vec![0.into(), tiny.clone()]]),
            &mut budget(),
        )
        .unwrap();
    assert_eq!(
        qr.coordinates().change().to_source,
        vec![
            vec![Real::one(), Real::zero()],
            vec![Real::zero(), Real::one()]
        ]
    );
    assert_eq!(
        qr.upper(),
        &[vec![Real::one(), Real::one()], vec![Real::zero(), tiny]]
    );
    let qr = source(0)
        .qr_coordinates(request(vec![]), &mut budget())
        .unwrap();
    assert!(qr.upper().is_empty());
    assert!(qr.norm_evidence().is_empty());
    assert!(qr.reconstruction_evidence().is_empty());
    assert!(
        qr.coordinates()
            .program()
            .eval_real(&[], &mut budget())
            .unwrap()
            .is_empty()
    );
}
#[test]
fn dependent_unknown_and_malformed_inputs_have_distinct_outcomes() {
    use hypersolve::expression_program::{
        CoordinateError, ExpressionError, QrCoordinateError as E,
    };
    let p = source(2);
    assert!(matches!(
        p.qr_coordinates(
            request(vec![vec![1.into(), 2.into()], vec![2.into(), 4.into()]]),
            &mut budget()
        ),
        Err(E::DependentColumn { column: 1, .. })
    ));
    assert!(matches!(
        p.qr_coordinates(
            request(vec![vec![0.into(), 2.into()], vec![0.into(), 4.into()]]),
            &mut budget()
        ),
        Err(E::DependentColumn { column: 0, .. })
    ));
    assert!(matches!(
        p.qr_coordinates(
            request(vec![vec![1.into()], vec![0.into(), 1.into()]]),
            &mut budget()
        ),
        Err(E::Coordinate(CoordinateError::Shape {
            field: "basis",
            row: Some(0),
            ..
        }))
    ));
    let mut invalid = request(vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]]);
    invalid.origin.pop();
    assert!(matches!(
        p.qr_coordinates(invalid, &mut budget()),
        Err(E::Coordinate(CoordinateError::Shape {
            field: "origin",
            ..
        }))
    ));
    let mut invalid = request(vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]]);
    invalid.variables[1] = invalid.variables[0];
    assert!(matches!(
        p.qr_coordinates(invalid, &mut budget()),
        Err(E::Coordinate(CoordinateError::Expression(
            ExpressionError::DuplicateVariable(_)
        )))
    ));
    let mut invalid = request(vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]]);
    invalid.variables[1] = SymbolId(99);
    assert!(matches!(
        p.qr_coordinates(invalid, &mut budget()),
        Err(E::Coordinate(CoordinateError::Expression(
            ExpressionError::UnboundVariable(_)
        )))
    ));
    let unresolved = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(matches!(
        source(1).qr_coordinates(request(vec![vec![unresolved]]), &mut budget()),
        Err(E::NormNotCertified {
            outcome: hyperlimit::PredicateOutcome::Unknown { .. },
            ..
        })
    ));
}
#[test]
fn final_work_entry_exhaustion_cancellation_and_retry_preserve_atomicity() {
    use hypersolve::expression_program::{
        CoordinateError, ExpressionError, QrCoordinateError as E,
    };
    let p = source(2);
    let request = request(vec![vec![3.into(), (-4).into()], vec![4.into(), 3.into()]]);
    let mut full = budget();
    p.qr_coordinates(request.clone(), &mut full).unwrap();
    let used = budget().remaining_work() - full.remaining_work();
    let mut short = ExpressionBudget::new(1_000_000, used - 1);
    assert!(matches!(
        p.qr_coordinates(request.clone(), &mut short),
        Err(E::Coordinate(CoordinateError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    let mut exact = ExpressionBudget::new(1_000_000, used);
    p.qr_coordinates(request.clone(), &mut exact).unwrap();
    assert_eq!(exact.remaining_work(), 0);
    assert!(matches!(
        p.qr_coordinates(request.clone(), &mut ExpressionBudget::new(0, 100_000_000)),
        Err(E::Coordinate(CoordinateError::Expression(
            ExpressionError::SizeExhausted
        )))
    ));
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = budget();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        p.qr_coordinates(request.clone(), &mut cancelled),
        Err(E::Coordinate(CoordinateError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    p.qr_coordinates(request, &mut budget()).unwrap();
}
