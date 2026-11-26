#![cfg(feature = "taylor-combination")]
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionError, ExpressionProgram, TaylorError, TaylorModel,
        TaylorOperand, TaylorPolynomialError as E, TaylorPolynomialModel, TaylorRequest,
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
fn s(id: u32) -> Expr {
    Expr::anonymous_symbol(SymbolId(id))
}
fn model(
    exprs: &[Expr],
    variables: &[u32],
    center: Vec<DyadicInterval>,
    domain: Vec<DyadicInterval>,
    order: usize,
) -> Arc<TaylorModel> {
    let variables: Vec<_> = variables.iter().map(|&v| SymbolId(v)).collect();
    Arc::new(
        ExpressionProgram::compile(exprs, &variables, &mut g())
            .unwrap()
            .taylor_model(
                TaylorRequest {
                    center,
                    domain,
                    order,
                    precision: -64,
                },
                &mut g(),
                &mut a(),
            )
            .unwrap(),
    )
}
fn map(exprs: &[Expr], variables: &[u32]) -> ExpressionProgram {
    let variables: Vec<_> = variables.iter().map(|&v| SymbolId(v)).collect();
    ExpressionProgram::compile(exprs, &variables, &mut g()).unwrap()
}
fn is_zero(value: &Real) -> bool {
    use hyperlimit::{PredicateOutcome, PredicatePolicy, Sign, classify_real_sign};
    matches!(
        classify_real_sign(value, PredicatePolicy::STRICT),
        PredicateOutcome::Decided {
            value: Sign::Zero,
            ..
        }
    )
}
fn zero_in(value: &DyadicInterval) -> bool {
    d(0, 0, 0).subset_of(value, &mut a()).unwrap()
}
fn overlaps(left: &DyadicInterval, right: &DyadicInterval) -> bool {
    !left.intersection(right, &mut a()).unwrap().is_empty()
}

#[test]
fn independent_center_is_reexpanded_exactly_and_truncated_into_the_remainder() {
    // x on [-1/4, 1/4] about 0 at order 1, joined with x^2 about 1 on [0, 2].
    let base = model(&[s(7)], &[7], vec![d(0, 0, 0)], vec![d(-1, 1, -2)], 1);
    let square = model(&[s(7) * s(7)], &[7], vec![d(1, 1, 0)], vec![d(-1, 2, 0)], 2);
    let joined = base
        .clone()
        .join(square.clone(), &mut g(), &mut a())
        .unwrap();
    assert!(Arc::ptr_eq(joined.source(), &base));
    assert_eq!(joined.stage_count(), 1);
    assert!(matches!(joined.joined(0), Some(TaylorOperand::Model(m)) if Arc::ptr_eq(m, &square)));
    assert_eq!(
        joined.maps().next().unwrap().definition(),
        square.source().definition()
    );
    // The receiving row passes through unchanged.
    assert_eq!(
        joined.terms(0).unwrap().get(&vec![(SymbolId(7), 1)]),
        Some(&1.into())
    );
    assert_eq!(joined.remainder(0), Some(&d(0, 0, 0)));
    // 1 + 2(x-1) + (x-1)^2 = x^2: constant and linear parts cancel exactly, and
    // the degree-2 term exceeds the receiving order.
    let terms = joined.terms(1).unwrap();
    assert!(terms.values().all(is_zero), "{terms:?}");
    assert_eq!(joined.remainder(1), Some(&d(0, 1, -4)));
    assert_eq!(joined.range(), &[d(-1, 1, -2), d(0, 1, -4)]);
    let polynomial = joined.polynomial_map(0).unwrap();
    assert!((0..polynomial.definition().nodes.len()).any(|n| {
        joined
            .product(0, n)
            .is_some_and(|p| p.discarded_terms().get(&vec![(SymbolId(7), 2)]) == Some(&1.into()))
    }));
    // Rows of both sources feed later maps: v - u*u encloses zero.
    let difference = joined
        .transform(map(&[s(31) - s(30) * s(30)], &[30, 31]), &mut g(), &mut a())
        .unwrap();
    assert_eq!(difference.stage_count(), 2);
    for x in [d(0, 0, 0), d(1, 1, -3), d(-1, 1, -2)] {
        let range = difference.enclose_on(&[x], &mut g(), &mut a()).unwrap();
        assert!(zero_in(&range.bounds()[0]), "{range:?}");
    }
    // At a sufficient receiving order the expansion is exact.
    let exact = model(&[s(7)], &[7], vec![d(0, 0, 0)], vec![d(-1, 1, -2)], 2)
        .join(square, &mut g(), &mut a())
        .unwrap();
    assert_eq!(exact.remainder(1), Some(&d(0, 0, 0)));
    assert_eq!(
        exact.terms(1).unwrap().get(&vec![(SymbolId(7), 2)]),
        Some(&1.into())
    );
    // About a nonzero receiving center 1/8: x^2 = 1/64 + (x-1/8)/4 + (x-1/8)^2.
    let shifted = model(&[s(7)], &[7], vec![d(1, 1, -3)], vec![d(-1, 1, -2)], 2)
        .join(
            model(&[s(7) * s(7)], &[7], vec![d(1, 1, 0)], vec![d(-1, 2, 0)], 2),
            &mut g(),
            &mut a(),
        )
        .unwrap();
    let terms = shifted.terms(1).unwrap();
    let r =
        |n: i64, m: i64| Real::from(hyperreal::Rational::from(n) / hyperreal::Rational::from(m));
    for (powers, expected) in [
        (vec![], r(1, 64)),
        (vec![(SymbolId(7), 1)], r(1, 4)),
        (vec![(SymbolId(7), 2)], r(1, 1)),
    ] {
        assert!(is_zero(&(terms[&powers].clone() - expected)), "{terms:?}");
    }
    assert_eq!(shifted.remainder(1), Some(&d(0, 0, 0)));
}

#[test]
fn transcendental_models_from_different_sources_multiply_consistently() {
    let domain = d(-1, 1, -3);
    let sine = model(
        &[s(7).sin()],
        &[7],
        vec![d(0, 0, 0)],
        vec![domain.clone()],
        3,
    );
    let cosine = model(
        &[s(7).cos()],
        &[7],
        vec![d(1, 1, -3)],
        vec![d(-1, 2, -2)],
        3,
    );
    let product = sine
        .join(cosine, &mut g(), &mut a())
        .unwrap()
        .transform(map(&[s(30) * s(31)], &[30, 31]), &mut g(), &mut a())
        .unwrap();
    let direct = model(
        &[s(7).sin() * s(7).cos()],
        &[7],
        vec![d(0, 0, 0)],
        vec![domain],
        3,
    );
    for x in [d(0, 0, 0), d(1, 1, -4), d(-1, -1, -3), d(-1, 1, -4)] {
        let joined = product
            .enclose_on(std::slice::from_ref(&x), &mut g(), &mut a())
            .unwrap();
        let reference = direct.enclose_on(&[x], &mut g(), &mut a()).unwrap();
        assert!(overlaps(&joined.bounds()[0], &reference[0]));
    }
    assert!(zero_in(
        &product
            .enclose_on(&[d(0, 0, 0)], &mut g(), &mut a())
            .unwrap()
            .bounds()[0]
    ));
    // sin(x)cos(x) = sin(2x)/2 spans about [-1/8, 1/8] here.
    let width = product.range()[0].width(&mut a()).unwrap();
    assert!(
        width.to_rational(&mut a()).unwrap().upper().unwrap()
            < &(hyperreal::Rational::from(1) / hyperreal::Rational::from(3))
    );
}

#[test]
fn operands_bind_by_symbol_and_must_cover_the_receiving_domain() {
    let base = model(
        &[s(7) * s(8)],
        &[7, 8],
        vec![d(0, 0, 0), d(0, 0, 0)],
        vec![d(-1, 1, -1), d(-1, 1, -1)],
        2,
    );
    // The operand uses only y; x is unconstrained by it.
    let shifted = model(
        &[s(8) + Expr::int(1)],
        &[8],
        vec![d(0, 0, 0)],
        vec![d(-1, 1, 0)],
        1,
    );
    let joined = base.clone().join(shifted, &mut g(), &mut a()).unwrap();
    let terms = joined.terms(1).unwrap();
    assert_eq!(terms.get(&vec![]), Some(&1.into()));
    assert_eq!(terms.get(&vec![(SymbolId(8), 1)]), Some(&1.into()));
    assert_eq!(
        joined
            .enclose_on(&[d(0, 1, -2), d(1, 1, -2)], &mut g(), &mut a())
            .unwrap()
            .bounds()[1],
        d(5, 5, -2)
    );
    let unbound = model(&[s(9)], &[9], vec![d(0, 0, 0)], vec![d(-1, 1, 0)], 1);
    assert!(matches!(
        base.clone().join(unbound, &mut g(), &mut a()),
        Err(E::Taylor(TaylorError::Expression(
            ExpressionError::UnboundVariable(SymbolId(9))
        )))
    ));
    let narrow = model(&[s(8)], &[8], vec![d(0, 0, 0)], vec![d(-1, 1, -2)], 1);
    assert!(matches!(
        base.clone().join(narrow, &mut g(), &mut a()),
        Err(E::Taylor(TaylorError::OutsideDomain { slot: 0 }))
    ));
    // Generated models are operands too, with their whole history.
    let squared = TaylorPolynomialModel::new(
        model(&[s(8)], &[8], vec![d(0, 0, 0)], vec![d(-1, 1, 0)], 1),
        map(&[s(30) * s(30)], &[30]),
        &mut g(),
        &mut a(),
    )
    .unwrap();
    assert_eq!(squared.remainder(0), Some(&d(0, 1, 0)));
    let chained = joined.join(squared.clone(), &mut g(), &mut a()).unwrap();
    assert_eq!(chained.stage_count(), 2);
    assert_eq!(chained.range().len(), 3);
    assert!(matches!(
        chained.joined(1),
        Some(TaylorOperand::Polynomial(_))
    ));
    // The operand's own truncation residual survives the join.
    assert_eq!(chained.remainder(2), Some(&d(0, 1, 0)));
    assert_eq!(chained.range()[2], d(0, 1, -2));
    let reversed = map(&[s(32), s(31), s(30)], &[30, 31, 32]);
    let next = chained.transform(reversed, &mut g(), &mut a()).unwrap();
    assert_eq!(next.range()[2], base.image()[0]);
}

#[test]
fn empty_domains_are_validated_and_publish_empty_rows() {
    let empty = model(
        &[s(7)],
        &[7],
        vec![d(0, 0, 0)],
        vec![DyadicInterval::empty()],
        1,
    );
    let operand = model(&[s(7) + s(7)], &[7], vec![d(0, 0, 0)], vec![d(-1, 1, 0)], 1);
    let joined = empty
        .clone()
        .join(operand.clone(), &mut g(), &mut a())
        .unwrap();
    assert!(joined.is_empty());
    assert_eq!(joined.range().len(), 2);
    assert!(joined.range().iter().all(DyadicInterval::is_empty));
    assert!(joined.terms(1).is_none());
    let base = model(&[s(7)], &[7], vec![d(0, 0, 0)], vec![d(-1, 1, -1)], 1);
    assert!(matches!(
        base.join(empty, &mut g(), &mut a()),
        Err(E::Taylor(TaylorError::OutsideDomain { slot: 0 }))
    ));
    let unbound = model(&[s(9)], &[9], vec![d(0, 0, 0)], vec![d(-1, 1, 0)], 1);
    assert!(matches!(
        model(
            &[s(7)],
            &[7],
            vec![d(0, 0, 0)],
            vec![DyadicInterval::empty()],
            1
        )
        .join(unbound, &mut g(), &mut a()),
        Err(E::Taylor(TaylorError::Expression(
            ExpressionError::UnboundVariable(_)
        )))
    ));
}

#[test]
fn exhausted_budgets_publish_nothing_and_retry_replays_identically() {
    let build = || {
        let base = model(
            &[Expr::int(1) / (Expr::int(1) + s(7))],
            &[7],
            vec![d(0, 0, 0)],
            vec![d(-1, 1, -3)],
            2,
        );
        let operand = model(
            &[(s(7) + Expr::int(1)).sqrt()],
            &[7],
            vec![d(1, 1, -2)],
            vec![d(-1, 1, -1)],
            2,
        );
        (base, operand)
    };
    let (base, operand) = build();
    let mut exhausted = false;
    for work in [1, 16, 64, 256, 1024] {
        match base.clone().join(
            operand.clone(),
            &mut ExpressionBudget::new(100_000, work),
            &mut a(),
        ) {
            Err(E::Taylor(TaylorError::Expression(ExpressionError::WorkExhausted))) => {
                exhausted = true
            }
            Err(other) => panic!("{other:?}"),
            Ok(_) => break,
        }
    }
    assert!(exhausted);
    let first = base.join(operand, &mut g(), &mut a()).unwrap();
    let (base, operand) = build();
    let replay = base.join(operand, &mut g(), &mut a()).unwrap();
    for output in 0..2 {
        assert_eq!(first.terms(output), replay.terms(output));
        assert_eq!(first.remainder(output), replay.remainder(output));
    }
    assert_eq!(first.range(), replay.range());
    assert!(!zero_in(&first.range()[1]));
}

#[test]
fn joined_rows_keep_the_dependency_that_independent_ranges_lose() {
    // sin(x) about 0 against its cubic Taylor polynomial built separately about
    // 1/16. Independent interval ranges cannot cancel; joined rows do.
    let domain = d(-1, 1, -3);
    let sine = model(
        &[s(7).sin()],
        &[7],
        vec![d(0, 0, 0)],
        vec![domain.clone()],
        4,
    );
    let cubic = model(
        &[s(7) - s(7) * s(7) * s(7) / Expr::int(6)],
        &[7],
        vec![d(1, 1, -4)],
        vec![d(-1, 1, -2)],
        3,
    );
    let rational = |value: &DyadicInterval| {
        let width = value.width(&mut a()).unwrap();
        width
            .to_rational(&mut a())
            .unwrap()
            .upper()
            .unwrap()
            .clone()
    };
    let naive = sine.image()[0].sub(&cubic.image()[0], &mut a()).unwrap();
    let joined = sine
        .join(cubic, &mut g(), &mut a())
        .unwrap()
        .transform(map(&[s(30) - s(31)], &[30, 31]), &mut g(), &mut a())
        .unwrap();
    assert!(zero_in(&joined.range()[0]));
    // |sin x - (x - x^3/6)| <= |x|^5/120 < 2^-21 on [-1/8, 1/8].
    let bound = hyperreal::Rational::from(1) / hyperreal::Rational::from(1 << 20);
    assert!(rational(&joined.range()[0]) < bound, "{:?}", joined.range());
    assert!(rational(&naive) > hyperreal::Rational::from(1) / hyperreal::Rational::from(5));
}

fn rational_in(value: &DyadicInterval, numerator: i64, denominator: i64) -> bool {
    let exact = hyperreal::Rational::from(numerator) / hyperreal::Rational::from(denominator);
    let r = value.to_rational(&mut a()).unwrap();
    r.lower().unwrap() <= &exact && &exact <= r.upper().unwrap()
}
fn width_below(value: &DyadicInterval, numerator: i64, denominator: i64) -> bool {
    let w = value
        .width(&mut a())
        .unwrap()
        .to_rational(&mut a())
        .unwrap();
    w.upper().unwrap()
        < &(hyperreal::Rational::from(numerator) / hyperreal::Rational::from(denominator))
}

#[test]
fn reciprocal_and_division_apply_certified_models_over_the_current_range() {
    let domain = || vec![d(-1, 1, -2)];
    let denominator = model(&[s(7) + Expr::int(1)], &[7], vec![d(0, 0, 0)], domain(), 3);
    let reciprocal = denominator
        .clone()
        .apply(&map(&[Expr::int(1) / s(30)], &[30]), 4, &mut g(), &mut a())
        .unwrap();
    // The generated outer certificate covers [3/4, 5/4] about its midpoint 1.
    let outer = reciprocal.model_map(0).unwrap();
    assert_eq!(outer.request().domain, vec![d(3, 5, -2)]);
    assert_eq!(outer.request().center, vec![d(1, 1, 0)]);
    let direct = model(
        &[Expr::int(1) / (s(7) + Expr::int(1))],
        &[7],
        vec![d(0, 0, 0)],
        domain(),
        3,
    );
    for (x, n, m) in [
        (d(0, 0, 0), 1, 1),
        (d(1, 1, -2), 4, 5),
        (d(-1, -1, -2), 4, 3),
        (d(1, 1, -3), 8, 9),
    ] {
        let got = reciprocal
            .enclose_on(std::slice::from_ref(&x), &mut g(), &mut a())
            .unwrap();
        assert!(rational_in(&got.bounds()[0], n, m), "{x:?} {got:?}");
        let reference = direct.enclose_on(&[x], &mut g(), &mut a()).unwrap();
        assert!(overlaps(&got.bounds()[0], &reference[0]));
    }
    // A quotient of rows from different sources and centers: (2+x)/(1+x).
    let numerator = model(&[s(7) + Expr::int(2)], &[7], vec![d(0, 0, 0)], domain(), 3);
    let shifted = model(
        &[s(7) + Expr::int(1)],
        &[7],
        vec![d(1, 1, -3)],
        vec![d(-1, 1, -1)],
        3,
    );
    let quotient = numerator
        .clone()
        .join(shifted.clone(), &mut g(), &mut a())
        .unwrap()
        .apply(&map(&[s(30) / s(31)], &[30, 31]), 4, &mut g(), &mut a())
        .unwrap();
    assert_eq!(quotient.stage_count(), 2);
    for (x, n, m) in [
        (d(0, 0, 0), 2, 1),
        (d(1, 1, -2), 9, 5),
        (d(-1, -1, -2), 7, 3),
    ] {
        let got = quotient.enclose_on(&[x], &mut g(), &mut a()).unwrap();
        assert!(rational_in(&got.bounds()[0], n, m), "{got:?}");
    }
    // (1+x)/(1+x) from two sources and centers stays near 1, where the
    // quotient of the independent ranges does not. Interval Taylor-mode
    // remainders keep higher outer orders useful on the wider domain too
    // (natural bounds on generated partials gave 1.07 there at every order).
    for (exponent, order, denominator) in [(-4, 7, 1_i64 << 28), (-2, 6, 256)] {
        let domain = || vec![d(-1, 1, exponent)];
        let left = model(
            &[s(7) + Expr::int(1)],
            &[7],
            vec![d(0, 0, 0)],
            domain(),
            order,
        );
        let right = model(
            &[s(7) + Expr::int(1)],
            &[7],
            vec![d(1, 1, -5)],
            vec![d(-1, 1, -1)],
            order,
        );
        let unit = left
            .clone()
            .join(right, &mut g(), &mut a())
            .unwrap()
            .apply(&map(&[s(30) / s(31)], &[30, 31]), order, &mut g(), &mut a())
            .unwrap();
        assert!(rational_in(&unit.range()[0], 1, 1));
        assert!(
            width_below(&unit.range()[0], 1, denominator),
            "{:?}",
            unit.range()
        );
        let naive = left.image()[0]
            .div(&left.image()[0], -64, &mut a())
            .unwrap();
        assert!(!width_below(&naive, 1, 8));
    }
    // A reciprocal over a range containing zero has no certified outer model.
    assert!(matches!(
        model(&[s(7)], &[7], vec![d(0, 0, 0)], domain(), 1).apply(
            &map(&[Expr::int(1) / s(30)], &[30]),
            2,
            &mut g(),
            &mut a()
        ),
        Err(E::Taylor(_))
    ));
}

#[test]
fn applied_roots_and_empty_models_keep_their_contracts() {
    let root = model(
        &[s(7) + Expr::int(2)],
        &[7],
        vec![d(0, 0, 0)],
        vec![d(-1, 1, -3)],
        2,
    )
    .apply(&map(&[s(30).sqrt()], &[30]), 3, &mut g(), &mut a())
    .unwrap();
    let at = root.enclose_on(&[d(0, 0, 0)], &mut g(), &mut a()).unwrap();
    let r = at.bounds()[0].to_rational(&mut a()).unwrap();
    let (lo, hi) = (r.lower().unwrap(), r.upper().unwrap());
    assert!(lo * lo <= 2.into() && hi * hi >= 2.into());
    assert!(width_below(&root.range()[0], 1, 8));
    let empty = model(
        &[s(7)],
        &[7],
        vec![d(0, 0, 0)],
        vec![DyadicInterval::empty()],
        1,
    )
    .apply(&map(&[s(30) * s(30)], &[30]), 2, &mut g(), &mut a())
    .unwrap();
    assert!(empty.is_empty());
    assert!(empty.range()[0].is_empty());
}
