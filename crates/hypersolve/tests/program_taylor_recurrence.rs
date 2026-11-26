#![cfg(feature = "taylor-model")]
//! Integral remainders bounded by interval Taylor-mode recurrences.
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        CoefficientProgram, ExpressionBudget, ExpressionProgram, TaylorModel, TaylorRequest,
    },
};
fn g() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn a() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn s(id: u32) -> Expr {
    Expr::anonymous_symbol(SymbolId(id))
}
fn build(
    expr: Expr,
    variables: &[u32],
    center: Vec<DyadicInterval>,
    domain: Vec<DyadicInterval>,
    order: usize,
    precision: i32,
) -> TaylorModel {
    let ids: Vec<_> = variables.iter().map(|&v| SymbolId(v)).collect();
    ExpressionProgram::compile(&[expr], &ids, &mut g())
        .unwrap()
        .taylor_model(
            TaylorRequest {
                center,
                domain,
                order,
                precision,
            },
            &mut g(),
            &mut a(),
        )
        .unwrap()
}
fn quarter(expr: Expr, order: usize) -> TaylorModel {
    build(expr, &[7], vec![d(0, 0, 0)], vec![d(-1, 1, -2)], order, -64)
}
fn within(value: &DyadicInterval, exponent: i32) -> bool {
    value.subset_of(&d(-1, 1, exponent), &mut a()).unwrap()
}
// f(p) lies in P(p) + R at every sample point: the model's point enclosure
// meets a 2^-80-wide natural enclosure of the source at that point.
fn sound_at(model: &TaylorModel, points: &[Vec<DyadicInterval>]) {
    for point in points {
        let got = model.enclose_on(point, &mut g(), &mut a()).unwrap();
        let truth = model
            .source()
            .eval_dyadic(point, -80, &mut g(), &mut a())
            .unwrap();
        for (got, truth) in got.iter().zip(&truth) {
            assert!(
                !got.intersection(truth, &mut a()).unwrap().is_empty(),
                "{point:?}: {got:?} misses {truth:?}"
            );
        }
        // Each degree-(n+1) bound contains the exact normalized partial at xi.
        let partials = formulas(model)
            .program()
            .eval_dyadic(point, -80, &mut g(), &mut a())
            .unwrap();
        let outputs = model.source().definition().outputs.len();
        for term in model.remainder_terms() {
            let exact = &partials[outputs + term.request_index()];
            let bound = term.coefficient_bound();
            assert!(
                !bound.intersection(exact, &mut a()).unwrap().is_empty(),
                "{point:?}: {bound:?} misses {exact:?}"
            );
        }
    }
}
fn formulas(model: &TaylorModel) -> CoefficientProgram {
    let program = model.coefficient_program(&mut g()).unwrap();
    assert_eq!(program.requests(), model.coefficient_requests());
    program
}
// The exact center recurrence agrees with the repeatedly differentiated
// formulas: identical rationals, and 2^-80 enclosures otherwise.
fn exact_at_center(model: &TaylorModel) {
    let program = formulas(model);
    let center: Vec<_> = model
        .request()
        .center
        .iter()
        .map(|c| Real::new(c.to_rational(&mut a()).unwrap().lower().unwrap().clone()))
        .collect();
    let values = program.program().eval_real(&center, &mut g()).unwrap();
    let outputs = model.source().definition().outputs.len();
    for term in model.terms() {
        let formula = &values[outputs + term.request_index()];
        let got = term.coefficient();
        match (got.exact_rational(), formula.exact_rational()) {
            (Some(got), Some(formula)) => assert_eq!(got, formula),
            _ => {
                let enclose = |v: &Real| DyadicInterval::from_real(v, -80, &mut a()).unwrap();
                let (got, formula) = (enclose(got), enclose(formula));
                let difference = got.sub(&formula, &mut a()).unwrap();
                assert!(within(&difference, -70), "{got:?} vs {formula:?}");
            }
        }
    }
}
fn samples() -> Vec<Vec<DyadicInterval>> {
    [-8, -6, -3, -1, 0, 1, 3, 5, 8]
        .iter()
        .map(|&k| vec![d(k, k, -5)])
        .collect()
}

#[test]
fn every_primitive_has_a_tight_and_sound_high_order_remainder() {
    let x = || s(7);
    let one = || Expr::int(1);
    let two = || Expr::int(2);
    // Natural evaluation of the repeatedly differentiated reciprocal partial
    // gave widths near 2e26 at order 6 here; the recurrence gives about 1.2e-3.
    // Each threshold is within a factor two of the attained enclosure.
    let cases = [
        (one() / (one() + x()), -10),
        ((two() + x()).ln(), -22),
        ((two() + x()).log10(), -23),
        ((one() + x()).sqrt(), -17),
        (x().asin(), -16),
        (x().acos(), -16),
        (x().atanh(), -14),
        ((x() + two()).acosh(), -13),
        (x().sin() * x().cos(), -20),
        ((x() * x() + x()).sin(), -15),
        ((two() + x() * x()).ln(), -18),
        (-(x() - x() * x()) / (Expr::int(3) - x()), -22),
        ((one() + x()).powi(5), -60),
        // Composition through certified power images, not a quotient
        // recurrence on the wide cube, keeps this near the natural bound.
        ((one() + x()).powi(-3), -4),
    ];
    for (index, (expr, exponent)) in cases.into_iter().enumerate() {
        let model = quarter(expr, 6);
        assert!(
            within(&model.remainder()[0], exponent),
            "case {index}: {:?}",
            model.remainder()
        );
        sound_at(&model, &samples());
        exact_at_center(&model);
    }
}

#[test]
fn exact_center_coefficients_match_generated_formulas_off_zero() {
    // At center 1/8 most coefficients are irrational (ln(17/8), sin(1/8),...).
    let x = || s(7);
    let one = || Expr::int(1);
    let two = || Expr::int(2);
    let cases = [
        one() / (one() + x()),
        (two() + x()).ln(),
        (two() + x()).log10(),
        (one() + x()).sqrt(),
        x().asin(),
        x().acos(),
        x().atanh(),
        (x() + two()).acosh(),
        x().sin() * x().cos(),
        (x() * x() + x()).sin(),
        (one() + x()).powi(-3),
        (one() + x()).powi(5) - x().cos() / (two() + x().sin()),
    ];
    for expr in cases {
        // Order zero keeps only the center value; its series has no unit slot.
        let constant = build(
            expr.clone(),
            &[7],
            vec![d(1, 1, -3)],
            vec![d(0, 1, -2)],
            0,
            -64,
        );
        exact_at_center(&constant);
        assert_eq!(constant.terms().len(), 1);
        let model = build(expr, &[7], vec![d(1, 1, -3)], vec![d(0, 1, -2)], 5, -64);
        exact_at_center(&model);
        sound_at(
            &model,
            &[vec![d(0, 0, 0)], vec![d(1, 1, -3)], vec![d(1, 1, -2)]],
        );
    }
    let expr = s(7) / (s(8) + Expr::int(2)) + (s(7) * s(8)).sin() + (s(8) + Expr::int(3)).ln();
    let model = build(
        expr,
        &[7, 8],
        vec![d(1, 1, -3), d(-1, 1, -3).midpoint(&mut a()).unwrap()],
        vec![d(0, 1, -2), d(-1, 1, -2)],
        4,
        -64,
    );
    exact_at_center(&model);
}

fn points() -> Vec<Vec<DyadicInterval>> {
    [-8, -3, 0, 5, 8]
        .iter()
        .flat_map(|&i| [-8, 2, 8].map(|j| vec![d(i, i, -5), d(j, j, -5)]))
        .collect()
}
#[test]
fn bivariate_partials_that_natural_evaluation_could_not_bound_are_enclosed() {
    // Generated partials of x/(2+y) previously failed with a zero-containing
    // widened denominator at every order; the model now builds and tightens.
    let expr = || s(7) / (s(8) + Expr::int(2)) + (s(7) * s(8)).sin();
    let square = || vec![d(-1, 1, -2), d(-1, 1, -2)];
    let mut previous = None;
    for order in [2, 4, 6] {
        let model = build(expr(), &[7, 8], vec![d(0, 0, 0); 2], square(), order, -64);
        let remainder = model.remainder()[0].clone();
        if let Some(previous) = previous {
            assert!(remainder.subset_of(&previous, &mut a()).unwrap());
        }
        sound_at(&model, &points());
        exact_at_center(&model);
        previous = Some(remainder);
    }
    let model = build(expr(), &[7, 8], vec![d(0, 0, 0); 2], square(), 6, -64);
    assert!(
        within(&model.remainder()[0], -16),
        "{:?}",
        model.remainder()
    );
    // Mixed chain-rule weights along either axis of a nonlinear argument.
    let log = (Expr::int(2) + s(7) * s(8) + s(8) * s(8)).ln();
    let model = build(log, &[7, 8], vec![d(0, 0, 0); 2], square(), 4, -64);
    assert!(
        within(&model.remainder()[0], -10),
        "{:?}",
        model.remainder()
    );
    sound_at(&model, &points());
}

#[test]
fn coarse_grids_fall_back_to_natural_partial_bounds() {
    // On the quarter grid the recurrence's rounded x*x on [3/8, 1/2] reaches
    // zero, so the square-root recurrence would divide by an enclosure of
    // zero; natural evaluation of the generated partials still applies.
    let root = || (s(7) * s(7)).sqrt();
    let coarse = build(root(), &[7], vec![d(1, 1, -1)], vec![d(3, 4, -3)], 1, -2);
    let fine = build(root(), &[7], vec![d(1, 1, -1)], vec![d(3, 4, -3)], 1, -64);
    assert!(!coarse.remainder_terms().is_empty());
    assert!(!within(&coarse.remainder()[0], -1));
    assert!(within(&fine.remainder()[0], -5), "{:?}", fine.remainder());
    let points: Vec<_> = [6, 7, 8].iter().map(|&k| vec![d(k, k, -4)]).collect();
    sound_at(&coarse, &points);
    sound_at(&fine, &points);
}
