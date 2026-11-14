#![cfg(feature = "taylor-noise")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorAffineError, TaylorAffineMap, TaylorAffineModel,
        TaylorAffineRow, TaylorNoiseModel, TaylorRequest,
    },
};
use std::sync::Arc;
fn graph() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
fn scalar() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn map(rows: &[&[i32]], inputs: usize) -> TaylorAffineMap {
    TaylorAffineMap {
        input_count: inputs,
        rows: rows
            .iter()
            .map(|r| TaylorAffineRow {
                coefficients: r.iter().map(|&x| Real::from(x)).collect(),
                offset: Real::zero(),
            })
            .collect(),
    }
}
fn source(
    outputs: &[Expr],
    center: i64,
    domain: DyadicInterval,
    order: usize,
) -> Arc<TaylorAffineModel> {
    let p = ExpressionProgram::compile(outputs, &[SymbolId(7)], &mut graph()).unwrap();
    let original = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(center, center, 0)],
                domain: vec![domain],
                order,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap(),
    );
    let rows = (0..outputs.len())
        .map(|i| TaylorAffineRow {
            coefficients: (0..outputs.len())
                .map(|j| Real::from(i32::from(i == j)))
                .collect(),
            offset: Real::zero(),
        })
        .collect();
    Arc::new(
        TaylorAffineModel::new(
            original,
            TaylorAffineMap {
                input_count: outputs.len(),
                rows,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap(),
    )
}
fn polynomial(center: i64, domain: DyadicInterval) -> Arc<TaylorAffineModel> {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let quadratic = Expr::int(16) * x.clone() * x.clone();
    source(&[quadratic.clone() + x, quadratic], center, domain, 2)
}
#[test]
fn dominant_shared_noise_cancels_and_omitted_terms_are_charged() {
    let original = polynomial(0, d(-1, 1, 0));
    let difference = map(&[&[1, -1]], 2);
    let boxed = TaylorNoiseModel::new(original.clone(), 0, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(boxed.columns().len(), 0);
    assert_eq!(
        boxed
            .enclose(&difference, &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        &[d(-17, 17, 0)]
    );
    let retained = TaylorNoiseModel::new(original.clone(), 1, &mut graph(), &mut scalar()).unwrap();
    assert!(Arc::ptr_eq(retained.source(), &original));
    assert_eq!(retained.maximum_noise(), 1);
    assert_eq!(retained.columns().len(), 1);
    let column = &retained.columns()[0];
    assert_eq!(
        original.terms()[column.term_index()].powers(),
        &[(SymbolId(7), 2)]
    );
    assert_eq!(column.domain(), &d(0, 1, 0));
    assert_eq!(column.variation(), &d(16, 16, 0));
    assert_eq!(retained.remainder(), &[d(-1, 1, 0), d(0, 0, 0)]);
    assert_eq!(retained.discarded_terms().len(), 1);
    assert_eq!(
        original.terms()[retained.discarded_terms()[0]].powers(),
        &[(SymbolId(7), 1)]
    );
    let exact = original
        .transform(difference.clone(), &mut graph(), &mut scalar())
        .unwrap();
    let answer = retained
        .enclose(&difference, &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(answer.bounds(), &[d(-1, 1, 0)]);
    assert_eq!(answer.bounds(), exact.range().bounds());
    // Repeated observations preserve the source and return identical exact bounds.
    for _ in 0..8 {
        assert_eq!(
            retained
                .enclose(&difference, &mut graph(), &mut scalar())
                .unwrap()
                .bounds(),
            answer.bounds()
        );
    }
    let all = TaylorNoiseModel::new(original, usize::MAX, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(all.columns().len(), 2);
    assert!(all.discarded_terms().is_empty());
    assert_eq!(
        all.enclose(&difference, &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        answer.bounds()
    );
}
#[test]
fn exact_point_oracles_cover_shifted_asymmetric_domains_and_every_capacity() {
    for center in [0, 1, 2, 3] {
        let original = polynomial(center, d(0, 3, 0));
        for capacity in 0..4 {
            let noise =
                TaylorNoiseModel::new(original.clone(), capacity, &mut graph(), &mut scalar())
                    .unwrap();
            let mut observation = map(&[&[-3, 2], &[1, -1], &[0, 0]], 2);
            for row in &mut observation.rows {
                row.offset = Real::from(5);
            }
            let range = noise
                .enclose(&observation, &mut graph(), &mut scalar())
                .unwrap();
            for i in 0..=24 {
                let x = Rational::from(i) / Rational::from(8);
                let expected = [
                    Rational::from(-16) * &x * &x - Rational::from(3) * &x + Rational::from(5),
                    &x + Rational::from(5),
                    Rational::from(5),
                ];
                for (bound, expected) in range.bounds().iter().zip(expected) {
                    assert!(
                        bound
                            .to_rational(&mut scalar())
                            .unwrap()
                            .contains_point(&expected)
                    );
                }
            }
            assert_eq!(range.bounds()[2], d(5, 5, 0));
        }
    }
}
#[test]
fn equal_residual_bounds_remain_distinct_functions() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let original = source(
        &[
            Expr::int(1) / (Expr::int(1) - x.clone()),
            Expr::int(1) / (Expr::int(1) + x),
        ],
        0,
        d(-1, 1, -2),
        0,
    );
    assert_eq!(original.remainder()[0], original.remainder()[1]);
    let noise = TaylorNoiseModel::new(original, 8, &mut graph(), &mut scalar()).unwrap();
    let range = noise
        .enclose(&map(&[&[1, -1]], 2), &mut graph(), &mut scalar())
        .unwrap();
    assert_ne!(range.bounds(), &[d(0, 0, 0)]);
    // At x=1/4 the exact difference is 8/15, while at -1/4 it is -8/15.
    let bound = range.bounds()[0].to_rational(&mut scalar()).unwrap();
    for sign in [-1, 1] {
        assert!(bound.contains_point(&(Rational::from(sign * 8) / Rational::from(15))));
    }
}
#[test]
fn empty_unit_constants_and_shape_validation_are_distinct() {
    let empty = source(&[Expr::int(3)], 0, DyadicInterval::empty(), 2);
    let model = TaylorNoiseModel::new(empty, 1, &mut graph(), &mut scalar()).unwrap();
    assert!(model.remainder().is_empty());
    let range = model
        .enclose(&map(&[], 1), &mut graph(), &mut scalar())
        .unwrap();
    assert!(range.is_empty());
    assert!(range.bounds().is_empty());
    assert!(matches!(
        model.enclose(&map(&[], 2), &mut graph(), &mut scalar()),
        Err(TaylorAffineError::InputCount { .. })
    ));
    assert!(matches!(
        model.enclose(&map(&[&[]], 1), &mut graph(), &mut scalar()),
        Err(TaylorAffineError::RowWidth { .. })
    ));
    let constant = source(&[Expr::int(3)], 0, d(-1, 1, 0), 0);
    let model = TaylorNoiseModel::new(constant, 0, &mut graph(), &mut scalar()).unwrap();
    let unit = model
        .enclose(&map(&[], 1), &mut graph(), &mut scalar())
        .unwrap();
    assert!(!unit.is_empty());
    assert!(unit.bounds().is_empty());
    assert_eq!(
        model
            .enclose(&map(&[&[2]], 1), &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        &[d(6, 6, 0)]
    );
    let zero_outputs = source(&[], 0, d(-1, 1, 0), 1);
    let model =
        TaylorNoiseModel::new(zero_outputs, usize::MAX, &mut graph(), &mut scalar()).unwrap();
    assert!(
        !model
            .enclose(&map(&[&[]], 0), &mut graph(), &mut scalar())
            .unwrap()
            .is_empty()
    );
}
#[test]
fn tied_variations_preserve_canonical_term_order_and_replay() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let original = source(
        &[x.clone() + Expr::int(2) * x.clone() * x],
        0,
        d(-1, 1, 0),
        2,
    );
    let first = TaylorNoiseModel::new(original.clone(), 1, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(
        original.terms()[first.columns()[0].term_index()].powers(),
        &[(SymbolId(7), 1)]
    );
    let rebuilt = source(
        &[Expr::anonymous_symbol(SymbolId(7))
            + Expr::int(2)
                * Expr::anonymous_symbol(SymbolId(7))
                * Expr::anonymous_symbol(SymbolId(7))],
        0,
        d(-1, 1, 0),
        2,
    );
    let replay = TaylorNoiseModel::new(rebuilt, 1, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(
        first.columns()[0].term_index(),
        replay.columns()[0].term_index()
    );
    assert_eq!(first.remainder(), replay.remainder());
    assert_eq!(
        first
            .enclose(&map(&[&[1]], 1), &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        replay
            .enclose(&map(&[&[1]], 1), &mut graph(), &mut scalar())
            .unwrap()
            .bounds()
    );
}

#[test]
fn resource_boundaries_cancellation_and_retry_preserve_source() {
    use hypersolve::{
        expression_program::{ExpressionError, TaylorError},
        work_budget::CancellationToken,
    };
    let original = polynomial(0, d(-1, 1, 0));
    let mut g = graph();
    let mut a = scalar();
    let gw = g.remaining_work();
    let aw = a.remaining_work();
    let model = TaylorNoiseModel::new(original.clone(), 1, &mut g, &mut a).unwrap();
    let gw = gw - g.remaining_work();
    let aw = aw - a.remaining_work();
    assert!(gw > 0 && aw > 0);
    let rebuilt = TaylorNoiseModel::new(
        original.clone(),
        1,
        &mut ExpressionBudget::new(100_000, gw),
        &mut DyadicBudget::new(16384, aw),
    )
    .unwrap();
    assert_eq!(rebuilt.remainder(), model.remainder());
    assert!(matches!(
        TaylorNoiseModel::new(
            original.clone(),
            1,
            &mut ExpressionBudget::new(100_000, gw - 1),
            &mut scalar()
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert!(matches!(
        TaylorNoiseModel::new(
            original.clone(),
            1,
            &mut graph(),
            &mut DyadicBudget::new(16384, aw - 1)
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Interval(
            hyperinterval::IntervalError::WorkExhausted
        )))
    ));
    assert!(matches!(
        TaylorNoiseModel::new(
            original.clone(),
            1,
            &mut ExpressionBudget::new(1, 1_000_000),
            &mut scalar()
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::SizeExhausted
        )))
    ));
    let observation = map(&[&[1, -1]], 2);
    let mut g = graph();
    let mut a = scalar();
    let gw = g.remaining_work();
    let aw = a.remaining_work();
    let answer = model.enclose(&observation, &mut g, &mut a).unwrap();
    let gw = gw - g.remaining_work();
    let aw = aw - a.remaining_work();
    let exact = model
        .enclose(
            &observation,
            &mut ExpressionBudget::new(100_000, gw),
            &mut DyadicBudget::new(16384, aw),
        )
        .unwrap();
    assert_eq!(exact.bounds(), answer.bounds());
    assert!(matches!(
        model.enclose(
            &observation,
            &mut ExpressionBudget::new(100_000, gw - 1),
            &mut scalar()
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert!(matches!(
        model.enclose(
            &observation,
            &mut graph(),
            &mut DyadicBudget::new(16384, aw - 1)
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Interval(
            hyperinterval::IntervalError::WorkExhausted
        )))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        TaylorNoiseModel::new(original.clone(), 1, &mut cancelled, &mut scalar()),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    assert!(matches!(
        model.enclose(&observation, &mut cancelled, &mut scalar()),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    let retried = TaylorNoiseModel::new(original.clone(), 1, &mut graph(), &mut scalar()).unwrap();
    assert!(Arc::ptr_eq(retried.source(), &original));
    assert_eq!(
        retried
            .enclose(&observation, &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        answer.bounds()
    );
}

#[test]
fn mixed_monomials_use_shared_parameters_with_noncanonical_binding_order() {
    let (x, y) = (
        Expr::anonymous_symbol(SymbolId(7)),
        Expr::anonymous_symbol(SymbolId(3)),
    );
    let shared = Expr::int(32) * x.clone() * y.clone();
    let p = ExpressionProgram::compile(
        &[shared.clone() + x, shared + y],
        &[SymbolId(7), SymbolId(3)],
        &mut graph(),
    )
    .unwrap();
    let taylor = p
        .taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0), d(0, 0, 0)],
                domain: vec![d(-1, 1, 0), d(-2, 2, 0)],
                order: 2,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap();
    let original = Arc::new(
        TaylorAffineModel::new(
            Arc::new(taylor),
            map(&[&[1, 0], &[0, 1]], 2),
            &mut graph(),
            &mut scalar(),
        )
        .unwrap(),
    );
    let model = TaylorNoiseModel::new(original.clone(), 1, &mut graph(), &mut scalar()).unwrap();
    let column = &model.columns()[0];
    assert_eq!(
        original.terms()[column.term_index()].powers(),
        &[(SymbolId(3), 1), (SymbolId(7), 1)]
    );
    assert_eq!(column.domain(), &d(-2, 2, 0));
    let range = model
        .enclose(&map(&[&[1, -1]], 2), &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(range.bounds(), &[d(-3, 3, 0)]);
    for x in -4..=4 {
        for y in -8..=8 {
            let expected = Rational::from(x - y) / Rational::from(4);
            assert!(
                range.bounds()[0]
                    .to_rational(&mut scalar())
                    .unwrap()
                    .contains_point(&expected)
            );
        }
    }
}

#[test]
fn irrational_shared_coefficients_cancel_before_enclosure() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let coefficient = Real::from(16) * Real::from(2).sqrt().unwrap();
    let shared = Expr::real(coefficient.clone()) * x.clone() * x.clone();
    let original = source(&[shared.clone() + x, shared], 0, d(-1, 1, 0), 2);
    let model = TaylorNoiseModel::new(original.clone(), 1, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(
        original.terms()[model.columns()[0].term_index()].coefficients(),
        &[coefficient.clone(), coefficient]
    );
    assert_eq!(
        model
            .enclose(&map(&[&[1, -1]], 2), &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        &[d(-1, 1, 0)]
    );
}

#[test]
fn generated_affine_sources_keep_offsets_and_capacity_tradeoffs_explicit() {
    let original = polynomial(0, d(-1, 1, 0));
    let mut shear = map(&[&[2, 1], &[1, 1]], 2);
    shear.rows[0].offset = Real::from(5);
    shear.rows[1].offset = Real::from(-2);
    let transformed = Arc::new(
        original
            .transform(shear, &mut graph(), &mut scalar())
            .unwrap(),
    );
    let observation = map(&[&[2, -3]], 2);
    // 2*(2*f0+f1+5)-3*(f0+f1-2) = x+16.
    // Dropping the shared linear monomial loses its cross-output correlation,
    // conservatively yielding x in [-7,7]; keeping it recovers [-1,1].
    let one = TaylorNoiseModel::new(transformed.clone(), 1, &mut graph(), &mut scalar()).unwrap();
    let two = TaylorNoiseModel::new(transformed, 2, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(
        one.enclose(&observation, &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        &[d(9, 23, 0)]
    );
    assert_eq!(
        two.enclose(&observation, &mut graph(), &mut scalar())
            .unwrap()
            .bounds(),
        &[d(15, 17, 0)]
    );
}

#[test]
fn duplicated_original_residual_cancels_even_without_retained_monomials() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let original = source(&[Expr::int(1) / (Expr::int(1) - x)], 0, d(-1, 1, -2), 0);
    assert_ne!(original.remainder(), &[d(0, 0, 0)]);
    let duplicate = Arc::new(
        original
            .transform(map(&[&[1], &[1]], 1), &mut graph(), &mut scalar())
            .unwrap(),
    );
    let model = TaylorNoiseModel::new(duplicate, 0, &mut graph(), &mut scalar()).unwrap();
    let difference = model
        .enclose(&map(&[&[1, -1]], 2), &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(difference.bounds(), &[d(0, 0, 0)]);
}
