#![cfg(feature = "taylor-model")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram, TaylorRequest},
};
fn graph() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
fn scalar() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}

#[test]
fn nonrational_coefficients_remain_exact_and_invalid_centers_do_not_extrapolate() {
    use hypersolve::expression_program::TaylorError;
    let x = SymbolId(7);
    let v = Expr::anonymous_symbol(x);
    let source = ExpressionProgram::compile(&[v.sin()], &[x], &mut graph()).unwrap();
    let request = TaylorRequest {
        center: vec![d(1, 1, -1)],
        domain: vec![d(0, 1, 0)],
        order: 1,
        precision: -64,
    };
    let model = source
        .taylor_model(request.clone(), &mut graph(), &mut scalar())
        .unwrap();
    let half = Real::from(Rational::from(1) / Rational::from(2));
    assert_eq!(model.terms()[0].coefficient(), &half.clone().sin());
    assert_eq!(model.terms()[1].coefficient(), &half.cos());
    assert!(!model.terms()[0].coefficient().is_rational());
    assert!(!model.terms()[1].coefficient().is_rational());
    let mut invalid = request.clone();
    invalid.center[0] = d(2, 2, 0);
    assert!(matches!(
        source.taylor_model(invalid, &mut graph(), &mut scalar()),
        Err(TaylorError::CenterOutsideDomain { slot: 0 })
    ));
    let mut invalid = request;
    invalid.order = usize::MAX;
    assert!(matches!(
        source.taylor_model(invalid, &mut graph(), &mut scalar()),
        Err(TaylorError::DegreeOverflow)
    ));
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
fn centered_quadratic_has_exact_coefficients_zero_remainder_and_sharper_range() {
    let x = SymbolId(7);
    let v = Expr::anonymous_symbol(x);
    let source =
        ExpressionProgram::compile(&[v.clone() * (Expr::int(1) - v)], &[x], &mut graph()).unwrap();
    let request = TaylorRequest {
        center: vec![d(1, 1, -1)],
        domain: vec![d(0, 1, 0)],
        order: 2,
        precision: -64,
    };
    let natural = source
        .eval_dyadic(&request.domain, -64, &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(natural, vec![d(0, 1, 0)]);
    let model = source
        .taylor_model(request, &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(model.source().definition(), source.definition());
    assert_eq!(model.image(), &[d(0, 1, -2)]);
    assert_eq!(model.remainder(), &[d(0, 0, 0)]);
    assert_eq!(
        model
            .terms()
            .iter()
            .map(|t| t.coefficient().clone())
            .collect::<Vec<_>>(),
        vec![
            Real::from(Rational::from(1) / Rational::from(4)),
            Real::zero(),
            Real::from(-1)
        ]
    );
    assert_eq!(
        model
            .enclose_on(&[d(0, 1, -1)], &mut graph(), &mut scalar())
            .unwrap(),
        vec![d(0, 1, -2)]
    );
}

#[test]
fn mixed_sparse_terms_ignore_unused_bindings_and_reuse_on_points() {
    let (x, y, unused) = (SymbolId(3), SymbolId(9), SymbolId(1));
    let xv = Expr::anonymous_symbol(x);
    let yv = Expr::anonymous_symbol(y);
    let source = ExpressionProgram::compile(
        &[xv.clone().powi(2) + Expr::int(2) * xv * yv],
        &[unused, y, x],
        &mut graph(),
    )
    .unwrap();
    let model = source
        .taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0), d(2, 2, 0), d(1, 1, 0)],
                domain: vec![d(-7, 9, 0), d(1, 3, 0), d(0, 2, 0)],
                order: 2,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap();
    assert_eq!(model.coefficient_program().requests().len(), 10);
    assert!(
        model
            .coefficient_program()
            .requests()
            .iter()
            .flat_map(|r| &r.powers)
            .all(|(s, _)| *s != unused)
    );
    let expected = [
        (vec![], 5),
        (vec![(x, 1)], 6),
        (vec![(y, 1)], 2),
        (vec![(x, 2)], 1),
        (vec![(x, 1), (y, 1)], 2),
        (vec![(y, 2)], 0),
    ];
    for (term, (powers, value)) in model.terms().iter().zip(expected) {
        assert_eq!(
            model.coefficient_program().requests()[term.request_index()].powers,
            powers
        );
        assert_eq!(term.coefficient(), &Real::from(value));
    }
    assert_eq!(model.remainder(), &[d(0, 0, 0)]);
    assert_eq!(
        model
            .enclose_on(
                &[d(-7, 9, 0), d(2, 2, 0), d(1, 1, 0)],
                &mut graph(),
                &mut scalar()
            )
            .unwrap(),
        vec![d(5, 5, 0)]
    );
    assert!(
        model
            .enclose_on(
                &[d(-8, 9, 0), d(2, 2, 0), d(1, 1, 0)],
                &mut graph(),
                &mut scalar()
            )
            .is_err()
    );
}

#[test]
fn rational_function_retains_the_entire_nonzero_tail() {
    let x = SymbolId(0);
    let source = ExpressionProgram::compile(
        &[Expr::int(1) / (Expr::int(1) - Expr::anonymous_symbol(x))],
        &[x],
        &mut graph(),
    )
    .unwrap();
    let model = source
        .taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0)],
                domain: vec![d(0, 1, -2)],
                order: 3,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap();
    assert_eq!(model.terms().len(), 4);
    assert!(
        model
            .terms()
            .iter()
            .all(|t| t.coefficient() == &Real::one())
    );
    assert_eq!(model.remainder_terms().len(), 1);
    let term = &model.remainder_terms()[0];
    assert_eq!(
        model.coefficient_program().requests()[term.request_index()].powers,
        vec![(x, 4)]
    );
    assert_eq!(term.monomial(), Some(&d(0, 1, -8)));
    // Independently, D^4 f / 4! = (1-x)^-5 on [0,1/4].
    let coefficient = term.coefficient_bound().to_rational(&mut scalar()).unwrap();
    assert!(coefficient.lower().unwrap() <= &Rational::from(1));
    assert!(coefficient.upper().unwrap() >= &(Rational::from(1024) / Rational::from(243)));
    // The exact omitted geometric tail x^4/(1-x) has image [0,1/192].
    let remainder = model.remainder()[0].to_rational(&mut scalar()).unwrap();
    assert!(remainder.lower().unwrap() <= &Rational::from(0));
    assert!(remainder.upper().unwrap() >= &(Rational::from(1) / Rational::from(192)));
    assert_ne!(model.remainder()[0], d(0, 0, 0));
    assert_eq!(
        term.contribution(),
        &term
            .coefficient_bound()
            .mul(term.monomial().unwrap(), &mut scalar())
            .unwrap()
    );
    assert_eq!(
        model.image()[0],
        source
            .eval_dyadic(&[d(0, 1, -2)], -64, &mut graph(), &mut scalar())
            .unwrap()[0]
    );
}

#[test]
fn sine_remainder_and_source_regularity_have_independent_controls() {
    let x = SymbolId(0);
    let source =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(x).sin()], &[x], &mut graph()).unwrap();
    let model = source
        .taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0)],
                domain: vec![d(0, 1, -2)],
                order: 2,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap();
    assert_eq!(
        model
            .terms()
            .iter()
            .map(|t| t.coefficient().clone())
            .collect::<Vec<_>>(),
        vec![Real::zero(), Real::one(), Real::zero()]
    );
    // Alternating-series/Taylor bound: -x^3/6 <= sin(x)-x <= 0 on this interval.
    let remainder = model.remainder()[0].to_rational(&mut scalar()).unwrap();
    assert!(remainder.lower().unwrap() <= &(Rational::from(-1) / Rational::from(384)));
    assert!(remainder.upper().unwrap() >= &Rational::from(0));
    let root = ExpressionProgram::compile(&[Expr::anonymous_symbol(x).sqrt()], &[x], &mut graph())
        .unwrap();
    assert!(
        root.eval_dyadic(&[d(0, 1, 0)], -64, &mut graph(), &mut scalar())
            .is_ok()
    );
    assert!(
        root.taylor_model(
            TaylorRequest {
                center: vec![d(1, 1, -1)],
                domain: vec![d(0, 1, 0)],
                order: 2,
                precision: -64
            },
            &mut graph(),
            &mut scalar()
        )
        .is_err()
    );
}

#[test]
fn empty_unit_constant_and_invalid_inputs_keep_distinct_meanings() {
    use hypersolve::expression_program::{ExpressionDefinition, TaylorError};
    let constant = ExpressionProgram::compile(&[Expr::int(5)], &[], &mut graph()).unwrap();
    let model = constant
        .taylor_model(
            TaylorRequest {
                center: vec![],
                domain: vec![],
                order: usize::MAX,
                precision: -64,
            },
            &mut graph(),
            &mut scalar(),
        )
        .unwrap();
    assert!(!model.is_empty());
    assert_eq!(model.image(), &[d(5, 5, 0)]);
    assert_eq!(model.remainder(), &[d(0, 0, 0)]);
    let unit = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![],
            nodes: vec![],
            outputs: vec![],
        },
        &mut graph(),
    )
    .unwrap();
    let model = unit
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
        .unwrap();
    assert!(!model.is_empty());
    assert!(model.image().is_empty());
    let x = SymbolId(0);
    let source = ExpressionProgram::compile(
        &[Expr::int(1) / (Expr::anonymous_symbol(x) - Expr::anonymous_symbol(x))],
        &[x],
        &mut graph(),
    )
    .unwrap();
    let empty = TaylorRequest {
        center: vec![d(0, 0, 0)],
        domain: vec![DyadicInterval::empty()],
        order: 2,
        precision: -64,
    };
    let model = source
        .taylor_model(empty.clone(), &mut graph(), &mut scalar())
        .unwrap();
    assert!(model.is_empty());
    assert_eq!(model.image(), &[DyadicInterval::empty()]);
    assert!(model.regularity().is_none());
    assert!(
        model
            .enclose_on(&[d(0, 0, 0)], &mut graph(), &mut scalar())
            .is_err()
    );
    assert!(matches!(
        source.taylor_model(
            TaylorRequest {
                center: vec![d(0, 1, 0)],
                ..empty.clone()
            },
            &mut graph(),
            &mut scalar()
        ),
        Err(TaylorError::NonPointCenter { .. })
    ));
    assert!(matches!(
        source.taylor_model(
            TaylorRequest {
                center: vec![],
                ..empty
            },
            &mut graph(),
            &mut scalar()
        ),
        Err(TaylorError::CenterCount { .. })
    ));
}

#[test]
fn final_work_scalar_limits_cancellation_and_retry_withhold_incomplete_models() {
    use hypersolve::{
        expression_program::{ExpressionError, TaylorError},
        work_budget::CancellationToken,
    };
    let x = SymbolId(0);
    let source =
        ExpressionProgram::compile(&[Expr::anonymous_symbol(x).powi(2)], &[x], &mut graph())
            .unwrap();
    let request = TaylorRequest {
        center: vec![d(0, 0, 0)],
        domain: vec![d(-1, 1, 0)],
        order: 2,
        precision: -64,
    };
    let mut gb = graph();
    let mut sb = scalar();
    let full = source
        .taylor_model(request.clone(), &mut gb, &mut sb)
        .unwrap();
    let work = 10_000_000 - gb.remaining_work();
    let scalar_work = 1_000_000_000_000 - sb.remaining_work();
    for limit in 0..=work {
        let result = source.taylor_model(
            request.clone(),
            &mut ExpressionBudget::new(100_000, limit),
            &mut scalar(),
        );
        assert_eq!(result.is_ok(), limit == work, "graph {limit}: {result:?}");
    }
    for limit in [0, scalar_work / 2, scalar_work - 1, scalar_work] {
        assert_eq!(
            source
                .taylor_model(
                    request.clone(),
                    &mut graph(),
                    &mut DyadicBudget::new(16384, limit)
                )
                .is_ok(),
            limit == scalar_work,
            "scalar {limit}"
        );
    }
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        source.taylor_model(request.clone(), &mut cancelled, &mut scalar()),
        Err(TaylorError::Expression(ExpressionError::Cancelled))
    ));
    let retry = source
        .taylor_model(request, &mut graph(), &mut scalar())
        .unwrap();
    assert_eq!(format!("{full:?}"), format!("{retry:?}"));
}

#[test]
fn correlated_polynomial_identity_and_empty_subdomain_are_preserved() {
    let (x, y) = (SymbolId(0), SymbolId(1));
    let xv = Expr::anonymous_symbol(x);
    let yv = Expr::anonymous_symbol(y);
    let expression = (xv.clone() + yv.clone()).powi(2)
        - xv.clone().powi(2)
        - Expr::int(2) * xv * yv.clone()
        - yv.powi(2);
    let source = ExpressionProgram::compile(&[expression], &[x, y], &mut graph()).unwrap();
    let model = source
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
    assert_eq!(model.image(), &[d(0, 0, 0)]);
    assert_eq!(model.remainder(), &[d(0, 0, 0)]);
    // An empty Cartesian product is contained even when another lane is outside.
    assert_eq!(
        model
            .enclose_on(
                &[DyadicInterval::empty(), d(10, 11, 0)],
                &mut graph(),
                &mut scalar()
            )
            .unwrap(),
        vec![DyadicInterval::empty()]
    );
    let consumer = ExpressionProgram::compile(
        &[Expr::int(1) / Expr::anonymous_symbol(x)],
        &[x],
        &mut graph(),
    )
    .unwrap();
    assert!(
        consumer
            .eval_dyadic(model.image(), -64, &mut graph(), &mut scalar())
            .is_err()
    );
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_rebuilds_source_coefficients_and_remainder() {
    const KEY: &str = "HYPERSOLVE_TAYLOR_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (source, request): (
            hypersolve::expression_program::ExpressionDefinition,
            TaylorRequest,
        ) = serde_json::from_str(&encoded).unwrap();
        let mut gb = graph();
        let mut sb = scalar();
        let program = ExpressionProgram::from_definition(source, &mut gb).unwrap();
        let model = program.taylor_model(request, &mut gb, &mut sb).unwrap();
        println!(
            "MODEL {model:?} work {} {}",
            gb.remaining_work(),
            sb.remaining_work()
        );
        return;
    }
    let x = SymbolId(0);
    let source = ExpressionProgram::compile(
        &[Expr::int(1) / (Expr::int(1) - Expr::anonymous_symbol(x))],
        &[x],
        &mut graph(),
    )
    .unwrap();
    let request = TaylorRequest {
        center: vec![d(0, 0, 0)],
        domain: vec![d(0, 1, -2)],
        order: 3,
        precision: -64,
    };
    let encoded = serde_json::to_string(&(source.definition(), &request)).unwrap();
    let mut expected = None;
    for _ in 0..2 {
        let result = std::process::Command::new(replay_binary::path().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_rebuilds_source_coefficients_and_remainder",
                "--nocapture",
            ])
            .env(KEY, &encoded)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        let text = String::from_utf8(result.stdout).unwrap();
        let result = text
            .lines()
            .find(|l| l.starts_with("MODEL "))
            .unwrap()
            .to_owned();
        if let Some(expected) = &expected {
            assert_eq!(&result, expected);
        } else {
            expected = Some(result);
        }
    }
    let mut invalid = source.definition().clone();
    invalid.outputs[0] = usize::MAX;
    assert!(ExpressionProgram::from_definition(invalid, &mut graph()).is_err());
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
