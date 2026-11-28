#![cfg(feature = "taylor-model")]
//! The mean-value remainder for order-one Taylor models encloses the function
//! at every sampled point, reuses the regular Jacobian, and is refused at
//! other orders.
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::{Rational, Real};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionProgram, TaylorError, TaylorRemainder, TaylorRequest,
    },
};

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
fn scalar() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}

fn source() -> ExpressionProgram {
    let (x, y) = (SymbolId(1), SymbolId(2));
    let (u, v) = (Expr::anonymous_symbol(x), Expr::anonymous_symbol(y));
    ExpressionProgram::compile(
        &[u.clone() * v.clone() + u.clone().sin(), u.clone() * u - v],
        &[x, y],
        &mut graph(),
    )
    .unwrap()
}

/// x in [0, 1/2], y in [1/4, 3/4], centred at (1/4, 1/2).
fn request(order: usize) -> TaylorRequest {
    TaylorRequest {
        center: vec![d(1, 1, -2), d(1, 1, -1)],
        domain: vec![d(0, 2, -2), d(1, 3, -2)],
        order,
        precision: -64,
    }
}

fn width(value: &DyadicInterval) -> f64 {
    let w = value.width(&mut scalar()).unwrap();
    let (m, e) = w.upper().unwrap();
    m.to_string().parse::<f64>().unwrap() * 2f64.powi(e)
}

#[test]
fn mean_value_models_enclose_sampled_values_and_reuse_the_jacobian() {
    let program = source();
    let model = program
        .taylor_model_with(
            request(1),
            TaylorRemainder::MeanValue,
            &mut graph(),
            &mut scalar(),
        )
        .unwrap();
    let lagrange = program
        .taylor_model(request(1), &mut graph(), &mut scalar())
        .unwrap();
    // Same exact polynomial; only the remainder form differs.
    assert_eq!(model.terms().len(), lagrange.terms().len());
    for (a, b) in model.terms().iter().zip(lagrange.terms()) {
        assert_eq!(a.coefficient(), b.coefficient());
    }
    // One remainder term per first-order coefficient, each keyed by it.
    for term in model.remainder_terms() {
        let request = &model.coefficient_requests()[term.request_index()];
        assert_eq!(request.powers.len(), 1);
        assert_eq!(request.powers[0].1, 1);
    }
    for i in 0..=4_i64 {
        for j in 0..=4_i64 {
            let (px, py) = (d(i, i, -3), d(2 + j, 2 + j, -3));
            let image = model
                .enclose_on(&[px.clone(), py.clone()], &mut graph(), &mut scalar())
                .unwrap();
            let point = [
                Real::new(Rational::fraction(i, 8).unwrap()),
                Real::new(Rational::fraction(2 + j, 8).unwrap()),
            ];
            let exact = program.eval_real(&point, &mut graph()).unwrap();
            for (value, enclosure) in exact.iter().zip(&image) {
                let tight = DyadicInterval::from_real(value, -80, &mut scalar()).unwrap();
                let meet = tight.intersection(enclosure, &mut scalar()).unwrap();
                assert!(!meet.is_empty(), "({i}, {j}) escapes {enclosure:?}");
            }
        }
    }
    // Both remainders are of second order; record their widths.
    for (mean, lagr) in model.remainder().iter().zip(lagrange.remainder()) {
        let (m, l) = (width(mean), width(lagr));
        assert!(m.is_finite() && l.is_finite() && m > 0.0);
        eprintln!("remainder width: mean value {m:.4e}, Lagrange {l:.4e}");
    }
}

#[test]
fn mean_value_remainder_is_refused_outside_order_one() {
    for order in [0, 2] {
        assert!(matches!(
            source().taylor_model_with(
                request(order),
                TaylorRemainder::MeanValue,
                &mut graph(),
                &mut scalar()
            ),
            Err(TaylorError::RemainderOrder { order: o }) if o == order
        ));
    }
}

fn model(
    request: TaylorRequest,
    form: TaylorRemainder,
) -> hypersolve::expression_program::TaylorModel {
    source()
        .taylor_model_with(request, form, &mut graph(), &mut scalar())
        .unwrap()
}

#[test]
fn adaptive_keeps_the_mean_value_form_on_narrow_domains() {
    // x in 1/4 +- 2^-12, y in 1/2 +- 2^-12: the remainder is second order in
    // the width, far below the first-order polynomial range.
    let narrow = TaylorRequest {
        center: vec![d(1, 1, -2), d(1, 1, -1)],
        domain: vec![d(1023, 1025, -12), d(2047, 2049, -12)],
        order: 1,
        precision: -64,
    };
    let adaptive = model(narrow.clone(), TaylorRemainder::Adaptive);
    assert!(!adaptive.remainder_terms().is_empty());
    assert!(
        adaptive
            .remainder_terms()
            .iter()
            .all(|term| term.kind() == TaylorRemainder::MeanValue)
    );
    let mean = model(narrow, TaylorRemainder::MeanValue);
    assert_eq!(adaptive.remainder(), mean.remainder());
}

#[test]
fn adaptive_intersects_both_forms_on_wide_domains_and_is_never_wider_than_lagrange() {
    let adaptive = model(request(1), TaylorRemainder::Adaptive);
    let lagrange = model(request(1), TaylorRemainder::Lagrange);
    let kinds = |m: &hypersolve::expression_program::TaylorModel, k| {
        m.remainder_terms().iter().filter(|t| t.kind() == k).count()
    };
    assert!(kinds(&adaptive, TaylorRemainder::MeanValue) > 0);
    assert_eq!(
        kinds(&adaptive, TaylorRemainder::Lagrange),
        lagrange.remainder_terms().len()
    );
    assert!(
        lagrange
            .remainder_terms()
            .iter()
            .all(|term| term.kind() == TaylorRemainder::Lagrange)
    );
    for (a, l) in adaptive.remainder().iter().zip(lagrange.remainder()) {
        assert!(
            a.subset_of(l, &mut scalar()).unwrap(),
            "{a:?} not within {l:?}"
        );
    }
    // Order two has no mean-value form, so adaptive is exactly Lagrange.
    let second = model(request(2), TaylorRemainder::Adaptive);
    assert_eq!(
        second.remainder(),
        model(request(2), TaylorRemainder::Lagrange).remainder()
    );
}
