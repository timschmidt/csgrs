#![cfg(feature = "taylor-composition")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram, TaylorModel, TaylorRequest},
};
use std::sync::Arc;
fn g() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
fn a() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000)
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn model(
    expr: Expr,
    variable: SymbolId,
    center: DyadicInterval,
    domain: DyadicInterval,
    order: usize,
) -> Arc<TaylorModel> {
    Arc::new(
        ExpressionProgram::compile(&[expr], &[variable], &mut g())
            .unwrap()
            .taylor_model(
                TaylorRequest {
                    center: vec![center],
                    domain: vec![domain],
                    order,
                    precision: -64,
                },
                &mut g(),
                &mut a(),
            )
            .unwrap(),
    )
}
#[test]
fn reciprocal_model_composes_without_redifferentiating_inner_source() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(Expr::int(1) + x, SymbolId(7), d(0, 0, 0), d(-1, 1, -2), 1);
    let u = Expr::anonymous_symbol(SymbolId(20));
    let outer = model(Expr::int(1) / u, SymbolId(20), d(1, 1, 0), d(3, 5, -2), 1);
    let result = inner
        .clone()
        .compose(outer.clone(), &mut g(), &mut a())
        .unwrap();
    assert!(Arc::ptr_eq(result.source(), &inner));
    assert!(Arc::ptr_eq(result.model_map(0).unwrap(), &outer));
    assert_eq!(result.terms(0).unwrap().get(&vec![]), Some(&1.into()));
    assert_eq!(
        result.terms(0).unwrap().get(&vec![(SymbolId(7), 1)]),
        Some(&(-1).into())
    );
    assert_eq!(result.remainder(0), Some(&outer.remainder()[0]));
    assert_eq!(
        result
            .enclose_on(&[d(0, 0, 0)], &mut g(), &mut a())
            .unwrap()
            .bounds(),
        &[d(1, 1, 0)]
    );
    let y = Expr::anonymous_symbol(SymbolId(30));
    let square = ExpressionProgram::compile(&[y.clone() * y], &[SymbolId(30)], &mut g()).unwrap();
    let next = result.transform(square, &mut g(), &mut a()).unwrap();
    assert_eq!(next.stage_count(), 2);
    assert_eq!(
        next.enclose_on(&[d(0, 0, 0)], &mut g(), &mut a())
            .unwrap()
            .bounds(),
        &[d(1, 1, 0)]
    );
}

#[test]
fn higher_outer_degree_is_charged_and_generated_models_compose_again() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(x, SymbolId(7), d(0, 0, 0), d(-1, 1, -1), 1);
    let u = Expr::anonymous_symbol(SymbolId(20));
    let square = model(u.clone() * u, SymbolId(20), d(0, 0, 0), d(-1, 1, -1), 2);
    let squared = inner.compose(square.clone(), &mut g(), &mut a()).unwrap();
    assert_eq!(squared.range(), &[d(0, 1, -2)]);
    assert_eq!(squared.remainder(0), Some(&d(0, 1, -2)));
    let polynomial = squared.polynomial_map(0).unwrap();
    assert!(
        polynomial
            .definition()
            .nodes
            .iter()
            .enumerate()
            .any(|(n, _)| squared
                .product(0, n)
                .is_some_and(
                    |p| p.discarded_terms().get(&vec![(SymbolId(7), 2)]) == Some(&1.into())
                ))
    );
    assert_eq!(
        squared.maps().next().unwrap().definition(),
        square.source().definition()
    );
    let v = Expr::anonymous_symbol(SymbolId(30));
    let reciprocal = model(
        Expr::int(1) / (Expr::int(1) + v),
        SymbolId(30),
        d(0, 0, 0),
        d(0, 1, -2),
        1,
    );
    let final_model = squared
        .compose(reciprocal.clone(), &mut g(), &mut a())
        .unwrap();
    assert_eq!(final_model.stage_count(), 2);
    assert!(Arc::ptr_eq(final_model.model_map(1).unwrap(), &reciprocal));
    for (x, numerator, denominator) in [
        (d(0, 0, 0), 1, 1),
        (d(1, 1, -1), 4, 5),
        (d(-1, -1, -1), 4, 5),
    ] {
        let actual = final_model.enclose_on(&[x], &mut g(), &mut a()).unwrap();
        let rational = actual.bounds()[0].to_rational(&mut a()).unwrap();
        let exact = hyperreal::Rational::from(numerator) / hyperreal::Rational::from(denominator);
        assert!(rational.lower().unwrap() <= &exact && &exact <= rational.upper().unwrap());
    }
}

#[test]
fn nonlinear_inner_residual_and_algebraic_outer_coefficients_survive() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(
        Expr::int(1) / (Expr::int(1) - x),
        SymbolId(7),
        d(0, 0, 0),
        d(-1, 1, -4),
        1,
    );
    assert_ne!(inner.remainder()[0], d(0, 0, 0));
    let u = Expr::anonymous_symbol(SymbolId(20));
    let outer = model(
        (u + Expr::int(1)).sqrt(),
        SymbolId(20),
        d(1, 1, 0),
        d(3, 5, -2),
        1,
    );
    let result = inner.compose(outer.clone(), &mut g(), &mut a()).unwrap();
    assert!(Arc::ptr_eq(result.model_map(0).unwrap(), &outer));
    // At x=0 the exact composite is sqrt(2). Positive endpoint squares are an
    // independent rational check, without a floating-point reference.
    let actual = result
        .enclose_on(&[d(0, 0, 0)], &mut g(), &mut a())
        .unwrap();
    let interval = actual.bounds()[0].to_rational(&mut a()).unwrap();
    let lo = interval.lower().unwrap();
    let hi = interval.upper().unwrap();
    assert!(lo > &0.into());
    assert!(lo * lo <= 2.into());
    assert!(hi * hi >= 2.into());
    let terms = result.terms(0).unwrap();
    let root = hyperreal::Real::from(2).sqrt().unwrap();
    for policy in [
        hyperlimit::PredicatePolicy::STRICT,
        hyperlimit::PredicatePolicy::APPROXIMATE_512,
    ] {
        use hyperlimit::{Certainty, PredicateOutcome, Sign, classify_real_sign};
        let difference = terms.get(&vec![]).unwrap().clone() - root.clone();
        assert!(matches!(
            classify_real_sign(&difference, policy),
            PredicateOutcome::Decided {
                value: Sign::Zero,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
    }
}

#[test]
fn complete_outer_domain_and_dimensions_are_checked_even_for_unused_inputs() {
    use hypersolve::expression_program::{TaylorError, TaylorPolynomialError as E};
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(x, SymbolId(7), d(0, 0, 0), d(-1, 1, 0), 1);
    let narrow = model(Expr::int(3), SymbolId(20), d(0, 0, 0), d(-1, 1, -1), 0);
    assert!(matches!(
        inner.clone().compose(narrow, &mut g(), &mut a()),
        Err(E::Taylor(TaylorError::OutsideDomain { slot: 0 }))
    ));
    let unit = Arc::new(
        ExpressionProgram::compile(&[Expr::int(3)], &[], &mut g())
            .unwrap()
            .taylor_model(
                TaylorRequest {
                    center: vec![],
                    domain: vec![],
                    order: 0,
                    precision: -64,
                },
                &mut g(),
                &mut a(),
            )
            .unwrap(),
    );
    assert!(matches!(
        inner.clone().compose(unit, &mut g(), &mut a()),
        Err(E::InputCount {
            expected: 1,
            actual: 0
        })
    ));
    let outer = model(Expr::int(3), SymbolId(20), d(0, 0, 0), d(-1, 1, 0), 0);
    let result = inner.compose(outer, &mut g(), &mut a()).unwrap();
    assert_eq!(result.range(), &[d(3, 3, 0)]);
    assert!(matches!(
        result.enclose_on(&[d(-2, 2, 0)], &mut g(), &mut a()),
        Err(E::Taylor(TaylorError::OutsideDomain { slot: 0 }))
    ));
}

#[test]
fn empty_unit_and_zero_output_products_keep_their_original_domain() {
    use hypersolve::expression_program::TaylorPolynomialError as E;
    let make = |variables: Vec<SymbolId>, domain: Vec<DyadicInterval>| {
        Arc::new(
            ExpressionProgram::compile(&[], &variables, &mut g())
                .unwrap()
                .taylor_model(
                    TaylorRequest {
                        center: domain
                            .iter()
                            .map(|v| {
                                if v.is_empty() {
                                    d(0, 0, 0)
                                } else {
                                    v.midpoint(&mut a()).unwrap()
                                }
                            })
                            .collect(),
                        domain,
                        order: 1,
                        precision: -64,
                    },
                    &mut g(),
                    &mut a(),
                )
                .unwrap(),
        )
    };
    let empty = make(vec![SymbolId(7)], vec![DyadicInterval::empty()]);
    let unit = make(vec![], vec![]);
    let outer = Arc::new(
        ExpressionProgram::compile(&[Expr::int(4)], &[], &mut g())
            .unwrap()
            .taylor_model(
                TaylorRequest {
                    center: vec![],
                    domain: vec![],
                    order: 1,
                    precision: -64,
                },
                &mut g(),
                &mut a(),
            )
            .unwrap(),
    );
    let a_model = empty
        .clone()
        .compose(outer.clone(), &mut g(), &mut a())
        .unwrap();
    let b_model = unit.compose(outer, &mut g(), &mut a()).unwrap();
    assert!(a_model.is_empty());
    assert_eq!(a_model.range(), &[DyadicInterval::empty()]);
    assert!(!b_model.is_empty());
    assert_eq!(b_model.range(), &[d(4, 4, 0)]);
    let wrong = model(Expr::int(3), SymbolId(20), d(0, 0, 0), d(-1, 1, 0), 0);
    assert!(matches!(
        empty.compose(wrong, &mut g(), &mut a()),
        Err(E::InputCount {
            expected: 0,
            actual: 1
        })
    ));
    let zero = make(vec![SymbolId(20)], vec![d(4, 4, 0)]);
    let result = b_model.compose(zero, &mut g(), &mut a()).unwrap();
    assert!(result.range().is_empty());
    assert!(!result.is_empty());
    assert!(
        !result
            .enclose_on(&[], &mut g(), &mut a())
            .unwrap()
            .is_empty()
    );
}

#[test]
fn work_size_native_cutoffs_and_cancelled_retry_preserve_shared_inputs() {
    use hypersolve::{
        expression_program::{ExpressionError, TaylorError, TaylorPolynomialError as E},
        work_budget::CancellationToken,
    };
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(Expr::int(1) + x, SymbolId(7), d(0, 0, 0), d(-1, 1, -2), 1);
    let u = Expr::anonymous_symbol(SymbolId(20));
    let outer = model(Expr::int(1) / u, SymbolId(20), d(1, 1, 0), d(3, 5, -2), 1);
    let before = format!("{inner:?}{outer:?}");
    let (mut graph, mut arithmetic) = (g(), a());
    let complete = inner
        .clone()
        .compose(outer.clone(), &mut graph, &mut arithmetic)
        .unwrap();
    let graph_spent = g().remaining_work() - graph.remaining_work();
    let native_spent = a().remaining_work() - arithmetic.remaining_work();
    assert!(graph_spent > 0 && native_spent > 0);
    for amount in [0, graph_spent - 1] {
        assert!(matches!(
            inner.clone().compose(
                outer.clone(),
                &mut ExpressionBudget::new(100_000, amount),
                &mut a()
            ),
            Err(E::Taylor(TaylorError::Expression(
                ExpressionError::WorkExhausted
            )))
        ));
    }
    assert!(matches!(
        inner.clone().compose(
            outer.clone(),
            &mut ExpressionBudget::new(0, 10_000_000),
            &mut a()
        ),
        Err(E::Taylor(TaylorError::Expression(
            ExpressionError::SizeExhausted
        )))
    ));
    for amount in [0, native_spent - 1] {
        assert!(matches!(
            inner.clone().compose(
                outer.clone(),
                &mut g(),
                &mut DyadicBudget::new(16384, amount)
            ),
            Err(E::Taylor(TaylorError::Interval(
                hyperinterval::IntervalError::WorkExhausted
            )))
        ));
    }
    let token = CancellationToken::new();
    token.cancel();
    let mut budget = g();
    budget.cancellation = Some(token);
    assert!(matches!(
        inner.clone().compose(outer.clone(), &mut budget, &mut a()),
        Err(E::Taylor(TaylorError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    assert_eq!(format!("{inner:?}{outer:?}"), before);
    let retried = inner.compose(outer, &mut g(), &mut a()).unwrap();
    assert_eq!(format!("{retried:?}"), format!("{complete:?}"));
}

#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_rebuilds_both_certificates_and_generated_chain() {
    use hypersolve::expression_program::ExpressionDefinition;
    const KEY: &str = "HYPERSOLVE_TAYLOR_COMPOSITION_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (inner, inner_request, outer, outer_request, map): (
            ExpressionDefinition,
            TaylorRequest,
            ExpressionDefinition,
            TaylorRequest,
            ExpressionDefinition,
        ) = serde_json::from_str(&encoded).unwrap();
        let (mut graph, mut arithmetic) = (g(), a());
        let inner = Arc::new(
            ExpressionProgram::from_definition(inner, &mut graph)
                .unwrap()
                .taylor_model(inner_request, &mut graph, &mut arithmetic)
                .unwrap(),
        );
        let outer = Arc::new(
            ExpressionProgram::from_definition(outer, &mut graph)
                .unwrap()
                .taylor_model(outer_request, &mut graph, &mut arithmetic)
                .unwrap(),
        );
        let composed = inner.compose(outer, &mut graph, &mut arithmetic).unwrap();
        let model = composed
            .transform(
                ExpressionProgram::from_definition(map, &mut graph).unwrap(),
                &mut graph,
                &mut arithmetic,
            )
            .unwrap();
        assert_eq!(
            model
                .enclose_on(&[d(0, 0, 0)], &mut graph, &mut arithmetic)
                .unwrap()
                .bounds(),
            &[d(1, 1, 0)]
        );
        println!(
            "MODEL {model:?} work {} {}",
            graph.remaining_work(),
            arithmetic.remaining_work()
        );
        return;
    }
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(Expr::int(1) + x, SymbolId(7), d(0, 0, 0), d(-1, 1, -2), 1);
    let u = Expr::anonymous_symbol(SymbolId(20));
    let outer = model(Expr::int(1) / u, SymbolId(20), d(1, 1, 0), d(3, 5, -2), 1);
    let v = Expr::anonymous_symbol(SymbolId(30));
    let square = ExpressionProgram::compile(&[v.clone() * v], &[SymbolId(30)], &mut g()).unwrap();
    let encoded = serde_json::to_string(&(
        inner.source().definition(),
        inner.request(),
        outer.source().definition(),
        outer.request(),
        square.definition(),
    ))
    .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let output = std::process::Command::new(replay_binary::path().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_rebuilds_both_certificates_and_generated_chain",
                "--nocapture",
            ])
            .env(KEY, &encoded)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stdout = std::str::from_utf8(&output.stdout).unwrap();
        let line = stdout
            .lines()
            .find(|x| x.starts_with("MODEL "))
            .unwrap()
            .to_owned();
        if let Some(previous) = previous {
            assert_eq!(previous, line);
        }
        previous = Some(line);
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn getters_preserve_proof_identity_without_new_predicate_dispatch() {
    use hyperreal::dispatch_trace;
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(Expr::int(1) + x, SymbolId(7), d(0, 0, 0), d(-1, 1, -2), 1);
    let u = Expr::anonymous_symbol(SymbolId(20));
    let outer = model(Expr::int(1) / u, SymbolId(20), d(1, 1, 0), d(3, 5, -2), 1);
    let result = inner
        .clone()
        .compose(outer.clone(), &mut g(), &mut a())
        .unwrap();
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        assert!(Arc::ptr_eq(result.source(), &inner));
        assert!(Arc::ptr_eq(result.model_map(0).unwrap(), &outer));
        assert!(result.model_map(1).is_none());
        assert!(result.polynomial_map(1).is_none());
        assert_eq!(result.maps().count(), 1);
        assert_eq!(result.stage_count(), 1);
        assert!(result.terms(0).is_some());
        assert!(result.remainder(0).is_some());
        assert!(!result.is_empty());
        assert_eq!(result.range().len(), 1);
        for (node, _) in result
            .polynomial_map(0)
            .unwrap()
            .definition()
            .nodes
            .iter()
            .enumerate()
        {
            let _ = result.product(0, node);
        }
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}

#[test]
fn mixed_terms_preserve_unsorted_binding_order_and_all_output_residuals() {
    let x = Expr::anonymous_symbol(SymbolId(9));
    let y = Expr::anonymous_symbol(SymbolId(2));
    let inner = Arc::new(
        ExpressionProgram::compile(
            &[Expr::int(1) + x, Expr::int(2) + y],
            &[SymbolId(9), SymbolId(2)],
            &mut g(),
        )
        .unwrap()
        .taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0); 2],
                domain: vec![d(-1, 1, -2); 2],
                order: 1,
                precision: -64,
            },
            &mut g(),
            &mut a(),
        )
        .unwrap(),
    );
    let u = Expr::anonymous_symbol(SymbolId(90));
    let v = Expr::anonymous_symbol(SymbolId(20));
    let product = u * v;
    let outer = Arc::new(
        ExpressionProgram::compile(
            &[product.clone(), -product],
            &[SymbolId(90), SymbolId(20)],
            &mut g(),
        )
        .unwrap()
        .taylor_model(
            TaylorRequest {
                center: vec![d(1, 1, 0), d(2, 2, 0)],
                domain: vec![d(3, 5, -2), d(7, 9, -2)],
                order: 2,
                precision: -64,
            },
            &mut g(),
            &mut a(),
        )
        .unwrap(),
    );
    let result = inner.compose(outer, &mut g(), &mut a()).unwrap();
    for (output, sign) in [(0, 1), (1, -1)] {
        let terms = result.terms(output).unwrap();
        assert_eq!(terms.get(&vec![]), Some(&(2 * sign).into()));
        assert_eq!(terms.get(&vec![(SymbolId(9), 1)]), Some(&(2 * sign).into()));
        assert_eq!(terms.get(&vec![(SymbolId(2), 1)]), Some(&sign.into()));
        assert_eq!(result.remainder(output), Some(&d(-1, 1, -4)));
    }
    assert_eq!(result.range(), &[d(21, 45, -4), d(-45, -21, -4)]);
    assert_eq!(
        result
            .enclose_on(&[d(1, 1, -2), d(-1, -1, -2)], &mut g(), &mut a())
            .unwrap()
            .bounds(),
        &[d(35, 35, -4), d(-35, -35, -4)]
    );
}

#[test]
fn outer_center_need_not_match_or_lie_inside_the_inner_image() {
    use hyperreal::{Rational, Real};
    let x = Expr::anonymous_symbol(SymbolId(7));
    let inner = model(Expr::int(1) + x, SymbolId(7), d(0, 0, 0), d(-1, 1, -2), 1);
    let u = Expr::anonymous_symbol(SymbolId(20));
    // Center 3/2 is outside the input image [3/4,5/4], but inside the certified
    // outer domain. Composition must preserve this supplied expansion.
    for (order, constant, slope) in [
        (0, Rational::from(2) / Rational::from(3), None),
        (
            1,
            Rational::from(8) / Rational::from(9),
            Some(Rational::from(-4) / Rational::from(9)),
        ),
        (
            2,
            Rational::from(26) / Rational::from(27),
            Some(Rational::from(-20) / Rational::from(27)),
        ),
    ] {
        let outer = model(
            Expr::int(1) / u.clone(),
            SymbolId(20),
            d(3, 3, -1),
            d(3, 7, -2),
            order,
        );
        let result = inner
            .clone()
            .compose(outer.clone(), &mut g(), &mut a())
            .unwrap();
        assert!(Arc::ptr_eq(result.model_map(0).unwrap(), &outer));
        assert_eq!(
            result.terms(0).unwrap().get(&vec![]),
            Some(&Real::from(constant))
        );
        assert_eq!(
            result
                .terms(0)
                .unwrap()
                .get(&vec![(SymbolId(7), 1)])
                .cloned(),
            slope.map(Real::from)
        );
        // The exact value at x=0 is one, even though the retained polynomial
        // constant is different. The outer residual must remain authoritative.
        assert_ne!(result.terms(0).unwrap().get(&vec![]), Some(&Real::one()));
        assert_eq!(
            result
                .enclose_on(&[d(0, 0, 0)], &mut g(), &mut a())
                .unwrap()
                .bounds(),
            &[d(1, 1, 0)]
        );
        for (point, exact) in [
            (d(-1, -1, -2), Rational::from(4) / Rational::from(3)),
            (d(1, 1, -2), Rational::from(4) / Rational::from(5)),
        ] {
            let bounds = result
                .enclose_on(&[point], &mut g(), &mut a())
                .unwrap()
                .bounds()[0]
                .to_rational(&mut a())
                .unwrap();
            assert!(bounds.lower().unwrap() <= &exact && &exact <= bounds.upper().unwrap());
        }
    }
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
