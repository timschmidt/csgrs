#![cfg(feature = "taylor-affine")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorAffineMap, TaylorAffineModel, TaylorAffineRow,
        TaylorRequest,
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
#[test]
fn shear_inverse_preserves_original_polynomial_correlation() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let program =
        ExpressionProgram::compile(&[x.clone(), x.clone() * x], &[SymbolId(7)], &mut graph())
            .unwrap();
    let source = Arc::new(
        program
            .taylor_model(
                TaylorRequest {
                    center: vec![d(0, 0, 0)],
                    domain: vec![d(-1, 1, 0)],
                    order: 2,
                    precision: -64,
                },
                &mut graph(),
                &mut scalar(),
            )
            .unwrap(),
    );
    let mut image = TaylorAffineModel::new(
        source.clone(),
        map(&[&[1, 1], &[0, 1]], 2),
        &mut graph(),
        &mut scalar(),
    )
    .unwrap();
    assert_eq!(image.range().bounds(), &[d(-1, 2, 0), d(0, 1, 0)]);
    // Projecting this intermediate image would widen x to [-2,2].
    assert_eq!(
        image.range().bounds()[0]
            .sub(&image.range().bounds()[1], &mut scalar())
            .unwrap(),
        d(-2, 2, 0)
    );
    image = image
        .transform(map(&[&[1, -1], &[0, 1]], 2), &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(image.range().bounds(), source.image());
    assert!(Arc::ptr_eq(image.source(), &source));
    for _ in 0..8 {
        image = image
            .transform(map(&[&[1, 1], &[0, 1]], 2), &mut graph(), &mut scalar())
            .unwrap()
            .transform(map(&[&[1, -1], &[0, 1]], 2), &mut graph(), &mut scalar())
            .unwrap();
        assert_eq!(image.range().bounds(), &[d(-1, 1, 0), d(0, 1, 0)]);
        assert_eq!(image.terms().len(), 3);
    }
}

fn reciprocal(outputs: usize) -> Arc<hypersolve::expression_program::TaylorModel> {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let expression = Expr::int(1) / (Expr::int(1) - x);
    let program =
        ExpressionProgram::compile(&vec![expression; outputs], &[SymbolId(7)], &mut graph())
            .unwrap();
    Arc::new(
        program
            .taylor_model(
                TaylorRequest {
                    center: vec![d(0, 0, 0)],
                    domain: vec![d(-1, 1, -2)],
                    order: 0,
                    precision: -64,
                },
                &mut graph(),
                &mut scalar(),
            )
            .unwrap(),
    )
}
#[test]
fn original_remainder_coordinates_survive_duplication_and_cancellation() {
    let source = reciprocal(1);
    assert_ne!(source.remainder(), &[d(0, 0, 0)]);
    let duplicate = TaylorAffineModel::new(
        source.clone(),
        map(&[&[1], &[1]], 1),
        &mut graph(),
        &mut scalar(),
    )
    .unwrap();
    let difference = duplicate
        .transform(map(&[&[1, -1]], 2), &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(difference.map().rows[0].coefficients, vec![Real::zero()]);
    assert_eq!(difference.remainder(), &[d(0, 0, 0)]);
    assert_eq!(difference.range().bounds(), &[d(0, 0, 0)]);
    // Keep the identical-output case, whose exact difference is zero.
    let equal = TaylorAffineModel::new(
        reciprocal(2),
        map(&[&[1, -1]], 2),
        &mut graph(),
        &mut scalar(),
    )
    .unwrap();
    assert!(
        equal.range().bounds()[0]
            .to_rational(&mut scalar())
            .unwrap()
            .contains_point(&hyperreal::Rational::from(0))
    );
    // Distinct f(x)=1/(1-x), g(x)=1/(1+x) have equal order-zero residual
    // intervals on this symmetric box, but f(1/4)-g(1/4)=8/15, not zero.
    let x = Expr::anonymous_symbol(SymbolId(7));
    let p = ExpressionProgram::compile(
        &[
            Expr::int(1) / (Expr::int(1) - x.clone()),
            Expr::int(1) / (Expr::int(1) + x),
        ],
        &[SymbolId(7)],
        &mut graph(),
    )
    .unwrap();
    let reflected = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0)],
                domain: vec![d(-1, 1, -2)],
                order: 0,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap(),
    );
    assert_eq!(reflected.remainder()[0], reflected.remainder()[1]);
    let distinct =
        TaylorAffineModel::new(reflected, map(&[&[1, -1]], 2), &mut graph(), &mut scalar())
            .unwrap();
    assert_ne!(distinct.remainder(), &[d(0, 0, 0)]);
    let value = hyperreal::Rational::from(8) / hyperreal::Rational::from(15);
    assert!(
        distinct
            .enclose_on(&[d(1, 1, -2)], &mut graph(), &mut scalar())
            .unwrap()
            .bounds()[0]
            .to_rational(&mut scalar())
            .unwrap()
            .contains_point(&value)
    );
    assert!(
        d(0, 0, 0)
            .subset_of(&distinct.range().bounds()[0], &mut scalar())
            .unwrap()
    );
    // A nonzero signed combination retains the full original tail.
    let scaled = duplicate
        .transform(map(&[&[-2, 1]], 2), &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(
        scaled.remainder()[0],
        d(-1, -1, 0)
            .mul(&source.remainder()[0], &mut scalar())
            .unwrap()
    );
    assert!(
        d(-4, -4, -2)
            .subset_of(&scaled.range().bounds()[0], &mut scalar())
            .unwrap()
    );
}
#[test]
fn nonrational_translation_cancels_before_interval_projection() {
    use hyperreal::Rational;
    let a = Real::from(Rational::from(1) / Rational::from(2)).sin();
    let program = ExpressionProgram::compile(&[Expr::real(a.clone())], &[], &mut graph()).unwrap();
    let source = Arc::new(
        program
            .taylor_model(
                TaylorRequest {
                    center: vec![],
                    domain: vec![],
                    order: 0,
                    precision: -64,
                },
                &mut graph(),
                &mut scalar(),
            )
            .unwrap(),
    );
    let mut transform = map(&[&[1]], 1);
    transform.rows[0].offset = -a;
    let result = TaylorAffineModel::new(source, transform, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(result.terms()[0].coefficients(), &[Real::zero()]);
    assert_eq!(result.range().bounds(), &[d(0, 0, 0)]);
}
#[test]
fn affine_translation_composes_exactly_and_source_domain_stays_authoritative() {
    use hypersolve::expression_program::{TaylorAffineError, TaylorError};
    let source = reciprocal(1);
    let mut first = map(&[&[2]], 1);
    first.rows[0].offset = Real::from(3);
    let image = TaylorAffineModel::new(source.clone(), first, &mut graph(), &mut scalar()).unwrap();
    let mut second = map(&[&[-3]], 1);
    second.rows[0].offset = Real::from(4);
    let image = image
        .transform(second, &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(image.map().rows[0].coefficients, vec![Real::from(-6)]);
    assert_eq!(image.map().rows[0].offset, Real::from(-5));
    assert!(
        d(-11, -11, 0)
            .subset_of(
                &image
                    .enclose_on(&[d(0, 0, 0)], &mut graph(), &mut scalar())
                    .unwrap()
                    .bounds()[0],
                &mut scalar()
            )
            .unwrap()
    );
    let zero = image
        .transform(map(&[&[0]], 1), &mut graph(), &mut scalar())
        .unwrap();
    assert!(matches!(
        zero.enclose_on(&[d(2, 3, 0)], &mut graph(), &mut scalar()),
        Err(TaylorAffineError::Taylor(TaylorError::OutsideDomain {
            slot: 0
        }))
    ));
    assert!(
        zero.enclose_on(&[DyadicInterval::empty()], &mut graph(), &mut scalar())
            .unwrap()
            .is_empty()
    );
}
#[test]
fn empty_and_unit_products_and_invalid_shapes_stay_distinct() {
    use hypersolve::expression_program::{ExpressionError, TaylorAffineError, TaylorError};
    let source = reciprocal(1);
    assert!(matches!(
        TaylorAffineModel::new(source.clone(), map(&[], 0), &mut graph(), &mut scalar()),
        Err(TaylorAffineError::InputCount {
            expected: 1,
            actual: 0
        })
    ));
    assert!(matches!(
        TaylorAffineModel::new(source.clone(), map(&[&[]], 1), &mut graph(), &mut scalar()),
        Err(TaylorAffineError::RowWidth {
            row: 0,
            expected: 1,
            actual: 0
        })
    ));
    let unit = TaylorAffineModel::new(source, map(&[], 1), &mut graph(), &mut scalar()).unwrap();
    assert!(!unit.range().is_empty());
    assert!(unit.range().bounds().is_empty());
    let mut from_unit = map(&[&[]], 0);
    from_unit.rows[0].offset = Real::from(7);
    let lifted_unit = unit
        .transform(from_unit, &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(lifted_unit.range().bounds(), &[d(7, 7, 0)]);
    assert!(!lifted_unit.range().is_empty());
    assert!(Arc::ptr_eq(unit.source(), lifted_unit.source()));
    let empty = unit
        .enclose_on(&[DyadicInterval::empty()], &mut graph(), &mut scalar())
        .unwrap();
    assert!(empty.is_empty());
    assert!(empty.bounds().is_empty());
    let p = ExpressionProgram::compile(&[], &[], &mut graph()).unwrap();
    let source = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![],
                domain: vec![],
                order: 0,
                precision: -32,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap(),
    );
    let mut constant = map(&[&[]], 0);
    constant.rows[0].offset = Real::from(7);
    let result = TaylorAffineModel::new(source, constant, &mut graph(), &mut scalar()).unwrap();
    assert_eq!(result.range().bounds(), &[d(7, 7, 0)]);
    assert!(!result.range().is_empty());
    let p = ExpressionProgram::compile(&[], &[SymbolId(7)], &mut graph()).unwrap();
    let source = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0)],
                domain: vec![DyadicInterval::empty()],
                order: 0,
                precision: -32,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap(),
    );
    let result =
        TaylorAffineModel::new(source.clone(), map(&[], 0), &mut graph(), &mut scalar()).unwrap();
    assert!(result.range().is_empty());
    assert!(result.range().bounds().is_empty());
    let mut from_empty = map(&[&[]], 0);
    from_empty.rows[0].offset = Real::from(7);
    let lifted_empty = result
        .transform(from_empty, &mut graph(), &mut scalar())
        .unwrap();
    assert!(lifted_empty.range().is_empty());
    assert_eq!(lifted_empty.range().bounds(), &[DyadicInterval::empty()]);
    assert!(matches!(
        TaylorAffineModel::new(source, map(&[&[1]], 0), &mut graph(), &mut scalar()),
        Err(TaylorAffineError::RowWidth { .. })
    ));
    assert!(matches!(
        result.enclose_on(&[], &mut graph(), &mut scalar()),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::BindingCount { .. }
        )))
    ));
}
#[test]
fn final_graph_and_scalar_work_cutoffs_cancel_cleanly_and_retry() {
    use hypersolve::{
        expression_program::{ExpressionError, TaylorAffineError, TaylorError},
        work_budget::CancellationToken,
    };
    let source = reciprocal(1);
    let mut g = graph();
    let mut a = scalar();
    let before_g = g.remaining_work();
    let before_a = a.remaining_work();
    let expected = TaylorAffineModel::new(source.clone(), map(&[&[2]], 1), &mut g, &mut a).unwrap();
    let graph_work = before_g - g.remaining_work();
    let scalar_work = before_a - a.remaining_work();
    assert!(graph_work > 0 && scalar_work > 0);
    let exact = TaylorAffineModel::new(
        source.clone(),
        map(&[&[2]], 1),
        &mut ExpressionBudget::new(100_000, graph_work),
        &mut DyadicBudget::new(16384, scalar_work),
    )
    .unwrap();
    assert_eq!(exact.range().bounds(), expected.range().bounds());
    assert!(matches!(
        TaylorAffineModel::new(
            source.clone(),
            map(&[&[2]], 1),
            &mut ExpressionBudget::new(100_000, graph_work - 1),
            &mut scalar()
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert!(matches!(
        TaylorAffineModel::new(
            source.clone(),
            map(&[&[2]], 1),
            &mut graph(),
            &mut DyadicBudget::new(16384, scalar_work - 1)
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Interval(
            hyperinterval::IntervalError::WorkExhausted
        )))
    ));
    assert!(matches!(
        TaylorAffineModel::new(
            source.clone(),
            map(&[&[2]], 1),
            &mut ExpressionBudget::new(1, 10_000_000),
            &mut scalar()
        ),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::SizeExhausted
        )))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        expected.transform(map(&[&[1]], 1), &mut cancelled, &mut scalar()),
        Err(TaylorAffineError::Taylor(TaylorError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    let retried = expected
        .transform(map(&[&[1]], 1), &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(retried.range().bounds(), expected.range().bounds());
    assert!(Arc::ptr_eq(retried.source(), &source));
}

#[test]
fn mixed_polynomial_coefficients_follow_independent_expansion() {
    let (x, y) = (
        Expr::anonymous_symbol(SymbolId(7)),
        Expr::anonymous_symbol(SymbolId(3)),
    );
    let outputs = [
        x.clone().powi(2) + Expr::int(3) * x.clone() * y.clone() + Expr::int(2) * y.clone().powi(2),
        Expr::int(2) * x.clone().powi(2) - x * y.clone() + Expr::int(5) * y.powi(2),
    ];
    let program =
        ExpressionProgram::compile(&outputs, &[SymbolId(7), SymbolId(3)], &mut graph()).unwrap();
    let source = Arc::new(
        program
            .taylor_model(
                TaylorRequest {
                    center: vec![d(0, 0, 0); 2],
                    domain: vec![d(-1, 1, 0), d(-2, 2, 0)],
                    order: 2,
                    precision: -64,
                },
                &mut graph(),
                &mut scalar(),
            )
            .unwrap(),
    );
    let result =
        TaylorAffineModel::new(source, map(&[&[2, -1]], 2), &mut graph(), &mut scalar()).unwrap();
    // 2*(x²+3xy+2y²) - (2x²-xy+5y²) = 7xy-y².
    for term in result.terms() {
        let expected = match term.powers() {
            [(SymbolId(3), 2)] => -1,
            [(SymbolId(3), 1), (SymbolId(7), 1)] => 7,
            _ => 0,
        };
        assert_eq!(term.coefficients(), &[Real::from(expected)]);
    }
    assert_eq!(result.remainder(), &[d(0, 0, 0)]);
    assert_eq!(result.range().bounds(), &[d(-18, 14, 0)]);
    for a in -1..=1 {
        for b in -2..=2 {
            let point = result
                .enclose_on(&[d(a, a, 0), d(b, b, 0)], &mut graph(), &mut scalar())
                .unwrap();
            assert_eq!(
                point.bounds(),
                &[d(7 * a * b - b * b, 7 * a * b - b * b, 0)]
            );
        }
    }
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_validates_source_and_composed_map() {
    const KEY: &str = "HYPERSOLVE_TAYLOR_AFFINE_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (definition, request, map): (
            hypersolve::expression_program::ExpressionDefinition,
            TaylorRequest,
            TaylorAffineMap,
        ) = serde_json::from_str(&data).unwrap();
        let mut corrupt = definition.clone();
        corrupt.outputs[0] = usize::MAX;
        assert!(ExpressionProgram::from_definition(corrupt, &mut graph()).is_err());
        let source = ExpressionProgram::from_definition(definition, &mut graph()).unwrap();
        let model = Arc::new(
            source
                .taylor_model(request, &mut graph(), &mut scalar())
                .unwrap(),
        );
        assert_ne!(model.remainder(), &[d(0, 0, 0)]);
        let mut invalid = map.clone();
        invalid.rows[0].coefficients.clear();
        assert!(matches!(
            TaylorAffineModel::new(model.clone(), invalid, &mut graph(), &mut scalar()),
            Err(hypersolve::expression_program::TaylorAffineError::RowWidth { .. })
        ));
        let mut g = graph();
        let mut a = scalar();
        let result = TaylorAffineModel::new(model, map, &mut g, &mut a).unwrap();
        assert_eq!(result.range().bounds(), &[d(0, 0, 0)]);
        println!(
            "AFFINE {:?} {:?} {} {}",
            result.map(),
            result.range(),
            g.remaining_work(),
            a.remaining_work()
        );
        return;
    }
    let source = reciprocal(1);
    let result = TaylorAffineModel::new(
        source.clone(),
        map(&[&[1], &[1]], 1),
        &mut graph(),
        &mut scalar(),
    )
    .unwrap()
    .transform(map(&[&[1, -1]], 2), &mut graph(), &mut scalar())
    .unwrap();
    let data =
        serde_json::to_string(&(source.source().definition(), source.request(), result.map()))
            .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(replay_binary::path().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_validates_source_and_composed_map",
                "--nocapture",
            ])
            .env(KEY, &data)
            .output()
            .unwrap();
        assert!(
            child.status.success(),
            "{} {}",
            String::from_utf8_lossy(&child.stdout),
            String::from_utf8_lossy(&child.stderr)
        );
        let text = String::from_utf8(child.stdout).unwrap();
        let row = text
            .lines()
            .find(|s| s.starts_with("AFFINE "))
            .unwrap()
            .to_owned();
        if let Some(expected) = &previous {
            assert_eq!(expected, &row);
        }
        previous = Some(row);
    }
}

#[test]
fn transform_budget_failure_does_not_replace_the_source_or_publish_a_partial_map() {
    use hypersolve::expression_program::{ExpressionError, TaylorAffineError, TaylorError};
    let image = TaylorAffineModel::new(
        reciprocal(1),
        map(&[&[1], &[1]], 1),
        &mut graph(),
        &mut scalar(),
    )
    .unwrap();
    let mut g = graph();
    let before = g.remaining_work();
    let full = image
        .transform(map(&[&[1, -1]], 2), &mut g, &mut scalar())
        .unwrap();
    let work = before - g.remaining_work();
    for limit in [0, work / 2, work - 1] {
        assert!(matches!(
            image.transform(
                map(&[&[1, -1]], 2),
                &mut ExpressionBudget::new(100_000, limit),
                &mut scalar()
            ),
            Err(TaylorAffineError::Taylor(TaylorError::Expression(
                ExpressionError::WorkExhausted
            )))
        ));
        assert_eq!(image.map().rows.len(), 2);
        assert_eq!(image.map().rows[0].coefficients, vec![Real::one()]);
    }
    let retry = image
        .transform(
            map(&[&[1, -1]], 2),
            &mut ExpressionBudget::new(100_000, work),
            &mut scalar(),
        )
        .unwrap();
    assert_eq!(retry.range().bounds(), full.range().bounds());
    assert!(Arc::ptr_eq(image.source(), retry.source()));
}

#[test]
fn query_accounts_for_its_domain_scan_and_shift_construction() {
    // Subtract the unchanged source-owner work to isolate affine transport.
    // This constant fixture still visits each supplied coordinate twice:
    // empty-domain classification and centered-shift construction. The four
    // remaining units cover output storage, output/term visits and its weight.
    for dimensions in [0, 1, 12] {
        let variables: Vec<_> = (0..dimensions).map(|i| SymbolId(i as u32)).collect();
        let program =
            ExpressionProgram::compile(&[Expr::int(1)], &variables, &mut graph()).unwrap();
        let domain = vec![d(-1, 1, 0); dimensions];
        let source = Arc::new(
            program
                .taylor_model(
                    TaylorRequest {
                        center: vec![d(0, 0, 0); dimensions],
                        domain: domain.clone(),
                        order: 0,
                        precision: -64,
                    },
                    &mut graph(),
                    &mut scalar(),
                )
                .unwrap(),
        );
        let image =
            TaylorAffineModel::new(source.clone(), map(&[&[1]], 1), &mut graph(), &mut scalar())
                .unwrap();
        let mut original = graph();
        source
            .enclose_on(&domain, &mut original, &mut scalar())
            .unwrap();
        let mut projected = graph();
        let result = image
            .enclose_on(&domain, &mut projected, &mut scalar())
            .unwrap();
        assert_eq!(result.bounds(), &[d(1, 1, 0)]);
        assert_eq!(
            original.remaining_work() - projected.remaining_work(),
            2 * dimensions + 4
        );
    }
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
