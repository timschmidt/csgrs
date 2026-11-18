#![cfg(feature = "taylor-polynomial")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorPolynomialModel, TaylorRequest,
    },
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
#[test]
fn product_accounts_for_discarded_square_and_accepts_generated_input() {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let source = ExpressionProgram::compile(
        &[Expr::int(1) + x.clone(), x - Expr::int(1)],
        &[SymbolId(7)],
        &mut g(),
    )
    .unwrap();
    let source = Arc::new(
        source
            .taylor_model(
                TaylorRequest {
                    center: vec![d(0, 0, 0)],
                    domain: vec![d(-1, 1, 0)],
                    order: 1,
                    precision: -64,
                },
                &mut g(),
                &mut a(),
            )
            .unwrap(),
    );
    let u = Expr::anonymous_symbol(SymbolId(20));
    let v = Expr::anonymous_symbol(SymbolId(21));
    let map =
        ExpressionProgram::compile(&[u * v], &[SymbolId(20), SymbolId(21)], &mut g()).unwrap();
    let result = TaylorPolynomialModel::new(source.clone(), map, &mut g(), &mut a()).unwrap();
    assert_eq!(result.remainder(0), Some(&d(0, 1, 0)));
    assert_eq!(result.range(), &[d(-1, 0, 0)]);
    assert!(Arc::ptr_eq(result.source(), &source));
    let y = Expr::anonymous_symbol(SymbolId(9));
    let square = ExpressionProgram::compile(&[y.clone() * y], &[SymbolId(9)], &mut g()).unwrap();
    let next = result.transform(square, &mut g(), &mut a()).unwrap();
    assert_eq!(next.range(), &[d(0, 1, 0)]);
    assert_eq!(next.stage_count(), 2);
    // A reused input carries its producer's evidence, but is not a multiplication
    // performed at this stage. Evidence lookup must preserve the map/node identity.
    for (stage, map) in next.maps().enumerate() {
        for (node, definition) in map.definition().nodes.iter().enumerate() {
            let multiplication = matches!(
                definition,
                hypersolve::expression_program::ExpressionNode::Binary {
                    operation: hypersolve::expression_program::BinaryOperation::Mul,
                    ..
                }
            );
            assert_eq!(next.product(stage, node).is_some(), multiplication);
        }
    }
    assert_eq!(
        next.enclose_on(&[d(0, 0, 0)], &mut g(), &mut a())
            .unwrap()
            .bounds(),
        &[d(1, 1, 0)]
    );
}
#[test]
fn subdomain_result_distinguishes_empty_from_unit_with_zero_outputs() {
    let source = ExpressionProgram::compile(&[], &[SymbolId(0)], &mut g()).unwrap();
    let source = Arc::new(
        source
            .taylor_model(
                TaylorRequest {
                    center: vec![d(0, 0, 0)],
                    domain: vec![d(-1, 1, 0)],
                    order: 1,
                    precision: -64,
                },
                &mut g(),
                &mut a(),
            )
            .unwrap(),
    );
    let map = ExpressionProgram::compile(&[], &[], &mut g()).unwrap();
    let model = TaylorPolynomialModel::new(source, map, &mut g(), &mut a()).unwrap();
    assert!(
        model
            .enclose_on(&[DyadicInterval::empty()], &mut g(), &mut a())
            .unwrap()
            .is_empty()
    );
    assert!(
        !model
            .enclose_on(&[d(0, 0, 0)], &mut g(), &mut a())
            .unwrap()
            .is_empty()
    );
}
fn reciprocal_pair() -> Arc<hypersolve::expression_program::TaylorModel> {
    let x = Expr::anonymous_symbol(SymbolId(7));
    let p = ExpressionProgram::compile(
        &[
            Expr::int(1) / (Expr::int(1) - x.clone()),
            Expr::int(1) / (Expr::int(1) + x),
        ],
        &[SymbolId(7)],
        &mut g(),
    )
    .unwrap();
    Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0)],
                domain: vec![d(-1, 1, -2)],
                order: 0,
                precision: -64,
            },
            &mut g(),
            &mut a(),
        )
        .unwrap(),
    )
}
#[test]
fn nonlinear_remainders_and_all_cross_terms_cover_independent_rational_values() {
    use hyperreal::{Rational, Real};
    let source = reciprocal_pair();
    assert_ne!(source.remainder()[0], d(0, 0, 0));
    let u = Expr::anonymous_symbol(SymbolId(0));
    let v = Expr::anonymous_symbol(SymbolId(1));
    let map = ExpressionProgram::compile(
        &[u.clone() * v.clone(), u - v],
        &[SymbolId(0), SymbolId(1)],
        &mut g(),
    )
    .unwrap();
    let product_node = map.definition().outputs[0];
    let model = TaylorPolynomialModel::new(source.clone(), map, &mut g(), &mut a()).unwrap();
    let proof = model.product(0, product_node).unwrap();
    assert!(proof.discarded_terms().is_empty());
    assert_eq!(proof.truncation(), &d(0, 0, 0));
    assert_eq!(
        proof.left_polynomial_times_right_remainder(),
        &source.remainder()[1]
    );
    assert_eq!(
        proof.right_polynomial_times_left_remainder(),
        &source.remainder()[0]
    );
    assert_eq!(
        proof.remainder_product(),
        &source.remainder()[0]
            .mul(&source.remainder()[1], &mut a())
            .unwrap()
    );
    assert_eq!(model.terms(0).unwrap().get(&Vec::new()), Some(&Real::one()));
    assert_ne!(model.range()[1], d(0, 0, 0)); // equal interval bounds are not equal functions
    for k in -16..=16 {
        let x = Rational::from(k) / Rational::from(64);
        let f = Rational::from(1) / (Rational::from(1) - &x);
        let h = Rational::from(1) / (Rational::from(1) + &x);
        let expected = [&f * &h, &f - &h];
        let sub = model
            .enclose_on(&[d(k, k, -6)], &mut g(), &mut a())
            .unwrap();
        assert!(!sub.is_empty());
        for (index, value) in expected.iter().enumerate() {
            assert!(
                model.range()[index]
                    .to_rational(&mut a())
                    .unwrap()
                    .contains_point(value)
            );
            assert!(
                sub.bounds()[index]
                    .to_rational(&mut a())
                    .unwrap()
                    .contains_point(value)
            );
        }
    }
}
#[test]
fn multivariate_convolution_combines_powers_and_retains_every_discarded_term() {
    use hyperreal::Real;
    let x = Expr::anonymous_symbol(SymbolId(5));
    let y = Expr::anonymous_symbol(SymbolId(2));
    let p = ExpressionProgram::compile(
        &[x.clone() + y.clone(), x - y],
        &[SymbolId(5), SymbolId(2)],
        &mut g(),
    )
    .unwrap();
    let source = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0), d(0, 0, 0)],
                domain: vec![d(-1, 1, 0), d(-2, 2, 0)],
                order: 1,
                precision: -64,
            },
            &mut g(),
            &mut a(),
        )
        .unwrap(),
    );
    let u = Expr::anonymous_symbol(SymbolId(0));
    let v = Expr::anonymous_symbol(SymbolId(1));
    let map = ExpressionProgram::compile(&[u * v], &[SymbolId(0), SymbolId(1)], &mut g()).unwrap();
    let node = map.definition().outputs[0];
    let model = TaylorPolynomialModel::new(source, map, &mut g(), &mut a()).unwrap();
    let proof = model.product(0, node).unwrap();
    assert_eq!(
        proof.discarded_terms().get(&vec![(SymbolId(5), 2)]),
        Some(&Real::one())
    );
    assert_eq!(
        proof.discarded_terms().get(&vec![(SymbolId(2), 2)]),
        Some(&Real::from(-1))
    );
    assert_eq!(
        proof
            .discarded_terms()
            .get(&vec![(SymbolId(2), 1), (SymbolId(5), 1)]),
        Some(&Real::zero())
    );
    assert_eq!(proof.truncation(), &d(-4, 1, 0));
    assert_eq!(model.range(), &[d(-4, 1, 0)]);
}
#[test]
fn empty_unit_shapes_unsupported_nodes_and_source_domains_are_explicit() {
    use hypersolve::expression_program::{
        ExpressionDefinition, ExpressionNode, TaylorError, TaylorPolynomialError as E,
        UnaryOperation,
    };
    let source = reciprocal_pair();
    let wrong = ExpressionProgram::compile(&[], &[], &mut g()).unwrap();
    assert!(matches!(
        TaylorPolynomialModel::new(source.clone(), wrong, &mut g(), &mut a()),
        Err(E::InputCount {
            expected: 2,
            actual: 0
        })
    ));
    let u = Expr::anonymous_symbol(SymbolId(0));
    let unsupported =
        ExpressionProgram::compile(&[u.sin()], &[SymbolId(0), SymbolId(1)], &mut g()).unwrap();
    assert!(matches!(
        TaylorPolynomialModel::new(source.clone(), unsupported, &mut g(), &mut a()),
        Err(E::UnsupportedOperation { .. })
    ));
    let unused = ExpressionProgram::from_definition(
        ExpressionDefinition {
            variables: vec![SymbolId(0), SymbolId(1)],
            nodes: vec![
                ExpressionNode::Constant(0.into()),
                ExpressionNode::Unary {
                    operation: UnaryOperation::Ln,
                    input: 0,
                },
            ],
            outputs: vec![0],
        },
        &mut g(),
    )
    .unwrap();
    let zero = TaylorPolynomialModel::new(source, unused, &mut g(), &mut a()).unwrap();
    assert_eq!(zero.range(), &[d(0, 0, 0)]);
    assert!(matches!(
        zero.enclose_on(&[d(2, 3, 0)], &mut g(), &mut a()),
        Err(E::Taylor(TaylorError::OutsideDomain { slot: 0 }))
    ));
    let p = ExpressionProgram::compile(&[], &[SymbolId(4)], &mut g()).unwrap();
    let empty = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0)],
                domain: vec![DyadicInterval::empty()],
                order: 1,
                precision: -64,
            },
            &mut g(),
            &mut a(),
        )
        .unwrap(),
    );
    let constant = ExpressionProgram::compile(&[Expr::int(7)], &[], &mut g()).unwrap();
    let empty = TaylorPolynomialModel::new(empty, constant, &mut g(), &mut a()).unwrap();
    assert!(empty.is_empty());
    assert_eq!(empty.range(), &[DyadicInterval::empty()]);
    assert!(empty.terms(0).is_none());
    let p = ExpressionProgram::compile(&[], &[], &mut g()).unwrap();
    let unit = Arc::new(
        p.taylor_model(
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
    let constant = ExpressionProgram::compile(&[Expr::int(7)], &[], &mut g()).unwrap();
    let unit = TaylorPolynomialModel::new(unit, constant, &mut g(), &mut a()).unwrap();
    assert!(!unit.is_empty());
    assert_eq!(unit.range(), &[d(7, 7, 0)]);
    assert!(!unit.enclose_on(&[], &mut g(), &mut a()).unwrap().is_empty());
}
#[test]
fn budgets_cancellation_and_generated_chain_retry_preserve_inputs() {
    use hypersolve::{
        expression_program::{ExpressionError, TaylorError, TaylorPolynomialError as E},
        work_budget::CancellationToken,
    };
    let source = reciprocal_pair();
    let u = Expr::anonymous_symbol(SymbolId(0));
    let v = Expr::anonymous_symbol(SymbolId(1));
    let map = ExpressionProgram::compile(&[u * v], &[SymbolId(0), SymbolId(1)], &mut g()).unwrap();
    assert!(matches!(
        TaylorPolynomialModel::new(
            source.clone(),
            map.clone(),
            &mut ExpressionBudget::new(100_000, 0),
            &mut a()
        ),
        Err(E::Taylor(TaylorError::Expression(
            ExpressionError::WorkExhausted
        )))
    ));
    assert!(matches!(
        TaylorPolynomialModel::new(
            source.clone(),
            map.clone(),
            &mut ExpressionBudget::new(1, 100_000),
            &mut a()
        ),
        Err(E::Taylor(TaylorError::Expression(
            ExpressionError::SizeExhausted
        )))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut budget = g();
    budget.cancellation = Some(token);
    assert!(matches!(
        TaylorPolynomialModel::new(source.clone(), map.clone(), &mut budget, &mut a()),
        Err(E::Taylor(TaylorError::Expression(
            ExpressionError::Cancelled
        )))
    ));
    assert!(
        TaylorPolynomialModel::new(
            source.clone(),
            map.clone(),
            &mut g(),
            &mut DyadicBudget::new(1, 0)
        )
        .is_err()
    );
    let model = TaylorPolynomialModel::new(source.clone(), map, &mut g(), &mut a()).unwrap();
    let before = format!("{model:?}");
    let identity = ExpressionProgram::compile(
        &[Expr::anonymous_symbol(SymbolId(0))],
        &[SymbolId(0)],
        &mut g(),
    )
    .unwrap();
    assert!(
        model
            .transform(
                identity.clone(),
                &mut ExpressionBudget::new(100_000, 0),
                &mut a()
            )
            .is_err()
    );
    assert_eq!(format!("{model:?}"), before);
    let mut chain = model.clone();
    for _ in 0..32 {
        chain = chain
            .transform(identity.clone(), &mut g(), &mut a())
            .unwrap();
    }
    assert_eq!(chain.stage_count(), 33);
    assert_eq!(chain.range(), model.range());
    assert!(Arc::ptr_eq(chain.source(), &source));
}
#[test]
fn final_work_cutoffs_do_not_publish_partial_products() {
    let source = reciprocal_pair();
    let u = Expr::anonymous_symbol(SymbolId(0));
    let v = Expr::anonymous_symbol(SymbolId(1));
    let map = ExpressionProgram::compile(&[u * v], &[SymbolId(0), SymbolId(1)], &mut g()).unwrap();
    let (mut graph, mut arithmetic) = (g(), a());
    let gw = graph.remaining_work();
    let aw = arithmetic.remaining_work();
    let model =
        TaylorPolynomialModel::new(source.clone(), map.clone(), &mut graph, &mut arithmetic)
            .unwrap();
    let needed_graph = gw - graph.remaining_work();
    let needed_arithmetic = aw - arithmetic.remaining_work();
    assert!(needed_graph > 0 && needed_arithmetic > 0);
    assert!(
        TaylorPolynomialModel::new(
            source.clone(),
            map.clone(),
            &mut ExpressionBudget::new(100_000, needed_graph - 1),
            &mut a()
        )
        .is_err()
    );
    assert!(
        TaylorPolynomialModel::new(
            source.clone(),
            map.clone(),
            &mut g(),
            &mut DyadicBudget::new(16384, needed_arithmetic - 1)
        )
        .is_err()
    );
    let exact = TaylorPolynomialModel::new(
        source,
        map,
        &mut ExpressionBudget::new(100_000, needed_graph),
        &mut DyadicBudget::new(16384, needed_arithmetic),
    )
    .unwrap();
    assert_eq!(format!("{exact:?}"), format!("{model:?}"));
}
#[cfg(feature = "dispatch-trace")]
#[test]
fn retained_evidence_access_preserves_zero_dispatch() {
    use hyperreal::dispatch_trace;
    let source = reciprocal_pair();
    let u = Expr::anonymous_symbol(SymbolId(0));
    let v = Expr::anonymous_symbol(SymbolId(1));
    let map = ExpressionProgram::compile(&[u * v], &[SymbolId(0), SymbolId(1)], &mut g()).unwrap();
    let node = map.definition().outputs[0];
    let model = TaylorPolynomialModel::new(source, map, &mut g(), &mut a()).unwrap();
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        assert!(!model.is_empty());
        assert_eq!(model.stage_count(), 1);
        assert_eq!(model.maps().count(), 1);
        assert!(model.terms(0).is_some());
        assert!(model.remainder(0).is_some());
        assert!(model.product(0, node).is_some());
        assert_eq!(model.source().request().order, 0);
        assert_eq!(model.range().len(), 1);
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
#[cfg(feature = "serde")]
#[test]
fn fresh_process_replay_reconstructs_every_source_and_map() {
    use hypersolve::expression_program::ExpressionDefinition;
    const KEY: &str = "HYPERSOLVE_POLYNOMIAL_MODEL_REPLAY";
    if let Ok(encoded) = std::env::var(KEY) {
        let (definition, request, maps): (
            ExpressionDefinition,
            TaylorRequest,
            Vec<ExpressionDefinition>,
        ) = serde_json::from_str(&encoded).unwrap();
        let (mut graph, mut arithmetic) = (g(), a());
        let p = ExpressionProgram::from_definition(definition, &mut graph).unwrap();
        let source = Arc::new(
            p.taylor_model(request, &mut graph, &mut arithmetic)
                .unwrap(),
        );
        let mut maps = maps.into_iter();
        let first = ExpressionProgram::from_definition(maps.next().unwrap(), &mut graph).unwrap();
        let mut model =
            TaylorPolynomialModel::new(source, first, &mut graph, &mut arithmetic).unwrap();
        for definition in maps {
            model = model
                .transform(
                    ExpressionProgram::from_definition(definition, &mut graph).unwrap(),
                    &mut graph,
                    &mut arithmetic,
                )
                .unwrap();
        }
        println!(
            "MODEL {model:?} work {} {}",
            graph.remaining_work(),
            arithmetic.remaining_work()
        );
        return;
    }
    let source = reciprocal_pair();
    let u = Expr::anonymous_symbol(SymbolId(0));
    let v = Expr::anonymous_symbol(SymbolId(1));
    let first =
        ExpressionProgram::compile(&[u * v], &[SymbolId(0), SymbolId(1)], &mut g()).unwrap();
    let y = Expr::anonymous_symbol(SymbolId(8));
    let second =
        ExpressionProgram::compile(&[Expr::int(1) - y.clone() * y], &[SymbolId(8)], &mut g())
            .unwrap();
    let encoded = serde_json::to_string(&(
        source.source().definition(),
        source.request(),
        [first.definition(), second.definition()],
    ))
    .unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let output = std::process::Command::new(replay_binary::path().unwrap())
            .args([
                "--exact",
                "fresh_process_replay_reconstructs_every_source_and_map",
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
        let text = std::str::from_utf8(&output.stdout).unwrap();
        let row = text
            .lines()
            .find(|l| l.starts_with("MODEL "))
            .unwrap()
            .to_owned();
        if let Some(previous) = &previous {
            assert_eq!(&row, previous);
        }
        previous = Some(row);
    }
    let mut invalid = first.definition().clone();
    invalid.outputs[0] = usize::MAX;
    assert!(ExpressionProgram::from_definition(invalid, &mut g()).is_err());
    let mut invalid = source.request().clone();
    invalid.domain = vec![d(-2, 2, 0)];
    assert!(
        source
            .source()
            .taylor_model(invalid, &mut g(), &mut a())
            .is_err()
    );
}
#[test]
fn algebraic_coefficients_remain_exact_through_multiplication_and_negation() {
    use hyperreal::Real;
    let root = Real::from(2).sqrt().unwrap();
    let x = Expr::anonymous_symbol(SymbolId(3));
    let p = ExpressionProgram::compile(&[x + Expr::real(root.clone())], &[SymbolId(3)], &mut g())
        .unwrap();
    let source = Arc::new(
        p.taylor_model(
            TaylorRequest {
                center: vec![d(0, 0, 0)],
                domain: vec![d(-1, 1, 0)],
                order: 1,
                precision: -64,
            },
            &mut g(),
            &mut a(),
        )
        .unwrap(),
    );
    let u = Expr::anonymous_symbol(SymbolId(0));
    let product = u.clone() * (u - Expr::real(Real::from(2) * root));
    let map =
        ExpressionProgram::compile(&[product.clone(), -product], &[SymbolId(0)], &mut g()).unwrap();
    let model = TaylorPolynomialModel::new(source, map, &mut g(), &mut a()).unwrap();
    assert_eq!(
        model.terms(0).unwrap().get(&Vec::new()),
        Some(&Real::from(-2))
    );
    assert_eq!(
        model.terms(0).unwrap().get(&vec![(SymbolId(3), 1)]),
        Some(&Real::zero())
    );
    assert_eq!(model.remainder(0), Some(&d(0, 1, 0)));
    assert_eq!(model.remainder(1), Some(&d(-1, 0, 0)));
    assert_eq!(model.range(), &[d(-2, -1, 0), d(1, 2, 0)]);
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
