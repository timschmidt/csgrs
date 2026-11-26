#![cfg(feature = "taylor-model")]
//! Taylor-mode ODE coefficients x_(k+1) = [f(t, x)]_k / (k + 1) over boxes.
use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram, OdeBinding, TaylorError},
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
fn program(outputs: &[Expr], variables: &[u32]) -> ExpressionProgram {
    let ids: Vec<_> = variables.iter().map(|&v| SymbolId(v)).collect();
    ExpressionProgram::compile(outputs, &ids, &mut g()).unwrap()
}
fn series(
    program: &ExpressionProgram,
    bindings: &[OdeBinding],
    start: DyadicInterval,
    initial: &[DyadicInterval],
    degree: usize,
) -> Option<Vec<Vec<DyadicInterval>>> {
    program
        .ode_taylor_coefficients(bindings, &start, initial, degree, -64, &mut g(), &mut a())
        .unwrap()
}
// value lies in the enclosure, which is at most 2^-60 wide.
fn tight(enclosure: &DyadicInterval, numerator: i64, denominator: i64) {
    let q = enclosure.to_rational(&mut a()).unwrap();
    let value = hyperreal::Rational::from(numerator) / hyperreal::Rational::from(denominator);
    assert!(q.contains_point(&value), "{enclosure:?} lacks {value}");
    assert!(
        enclosure
            .width(&mut a())
            .unwrap()
            .subset_of(&d(0, 1, -60), &mut a())
            .unwrap(),
        "{enclosure:?}"
    );
}

#[test]
fn point_series_match_closed_form_solutions() {
    use OdeBinding::{State, Time};
    // (x, y)' = (y, -x) from (1, 0): x = cos t, y = -sin t.
    let rotation = program(&[s(2), -s(1)], &[1, 2]);
    let x = series(
        &rotation,
        &[State(0), State(1)],
        d(0, 0, 0),
        &[d(1, 1, 0), d(0, 0, 0)],
        5,
    )
    .unwrap();
    for (got, (n, m)) in x[0]
        .iter()
        .zip([(1, 1), (0, 1), (-1, 2), (0, 1), (1, 24), (0, 1)])
    {
        tight(got, n, m);
    }
    for (got, (n, m)) in x[1]
        .iter()
        .zip([(0, 1), (-1, 1), (0, 1), (1, 6), (0, 1), (-1, 120)])
    {
        tight(got, n, m);
    }
    // x' = x^2 from 1: x = 1/(1 - t), every coefficient one.
    let riccati = program(&[s(1) * s(1)], &[1]);
    for got in &series(&riccati, &[State(0)], d(0, 0, 0), &[d(1, 1, 0)], 6).unwrap()[0] {
        tight(got, 1, 1);
    }
    // x' = t x from x(0) = 1: x = exp(t^2/2). Bindings follow the program's
    // own slot order (state first here), and time starts at a nonzero point.
    let growth = program(&[s(9) * s(1)], &[1, 9]);
    let x = series(&growth, &[State(0), Time], d(0, 0, 0), &[d(1, 1, 0)], 4).unwrap();
    for (got, (n, m)) in x[0].iter().zip([(1, 1), (0, 1), (1, 2), (0, 1), (1, 8)]) {
        tight(got, n, m);
    }
    // From t = 1 with x(1) = 1: x = exp((t^2 - 1)/2), so x' = 1, x''/2 = 1,
    // x'''/6 = 2/3.
    let x = series(&growth, &[State(0), Time], d(1, 1, 0), &[d(1, 1, 0)], 3).unwrap();
    for (got, (n, m)) in x[0].iter().zip([(1, 1), (1, 1), (1, 1), (2, 3)]) {
        tight(got, n, m);
    }
    // Transcendental and guarded primitives: x' = sin(x) + ln(1 + t) at (0, 0)
    // has x_1 = 0 and x_2 = [cos(x) x' + 1/(1+t)]_0 / 2 = 1/2. Then
    // [sin x]_2 = x_2 = 1/2 and [ln(1+t)]_2 = -1/2 cancel, so x_3 = 0, while
    // x_4 = ([sin x]_3 + [ln(1+t)]_3) / 4 = (x_3 + 1/3) / 4 = 1/12.
    let mixed = program(&[s(1).sin() + (Expr::int(1) + s(9)).ln()], &[1, 9]);
    let x = series(&mixed, &[State(0), Time], d(0, 0, 0), &[d(0, 0, 0)], 4).unwrap();
    for (got, (n, m)) in x[0].iter().zip([(0, 1), (0, 1), (1, 2), (0, 1), (1, 12)]) {
        tight(got, n, m);
    }
}

#[test]
fn box_coefficients_contain_every_corner_solution() {
    use OdeBinding::State;
    // x' = x^2: x_k = x0^(k+1) for each initial value x0 in [1/2, 1].
    let riccati = program(&[s(1) * s(1)], &[1]);
    let x = series(&riccati, &[State(0)], d(0, 0, 0), &[d(1, 2, -1)], 4).unwrap();
    for (k, got) in x[0].iter().enumerate() {
        let q = got.to_rational(&mut a()).unwrap();
        for x0 in [1, 2] {
            let base = hyperreal::Rational::from(x0) / hyperreal::Rational::from(2);
            let mut power = base.clone();
            for _ in 0..k {
                power *= base.clone();
            }
            let expected = if k == 0 { base.clone() } else { power };
            assert!(q.contains_point(&expected), "degree {k}: {got:?}");
        }
    }
    // An interval start (a remainder evaluation over a time interval) contains
    // the point-start values of a time-dependent field.
    let growth = program(&[s(9) * s(1)], &[1, 9]);
    let bindings = [State(0), OdeBinding::Time];
    let wide = series(&growth, &bindings, d(0, 1, -2), &[d(1, 2, -1)], 3).unwrap();
    for start in [d(0, 0, 0), d(1, 1, -2)] {
        for initial in [d(1, 1, -1), d(1, 1, 0)] {
            let point = series(&growth, &bindings, start.clone(), &[initial], 3).unwrap();
            for (wide, point) in wide[0].iter().zip(&point[0]) {
                assert!(
                    point.subset_of(wide, &mut a()).unwrap(),
                    "{point:?} {wide:?}"
                );
            }
        }
    }
}

#[test]
fn unavailable_recurrences_and_invalid_shapes_are_distinct() {
    use OdeBinding::{State, Time};
    // sqrt(x) with x reaching zero: the square-root recurrence would divide by
    // an enclosure of zero, so the coefficients are unavailable, not an error.
    let root = program(&[s(1).sqrt()], &[1]);
    assert!(series(&root, &[State(0)], d(0, 0, 0), &[d(0, 1, 0)], 2).is_none());
    assert!(series(&root, &[State(0)], d(0, 0, 0), &[d(1, 2, 0)], 2).is_some());
    // Empty boxes publish nothing.
    let rotation = program(&[s(2), -s(1)], &[1, 2]);
    let empty = DyadicInterval::empty();
    assert!(
        series(
            &rotation,
            &[State(0), State(1)],
            empty.clone(),
            &vec![d(0, 0, 0); 2],
            2
        )
        .is_none()
    );
    assert!(
        series(
            &rotation,
            &[State(0), State(1)],
            d(0, 0, 0),
            &[empty, d(0, 0, 0)],
            2
        )
        .is_none()
    );
    // Degree zero is the initial box itself.
    let zero = series(
        &rotation,
        &[State(0), State(1)],
        d(0, 0, 0),
        &[d(1, 1, 0), d(0, 0, 0)],
        0,
    )
    .unwrap();
    assert_eq!(zero, vec![vec![d(1, 1, 0)], vec![d(0, 0, 0)]]);
    let call = |bindings: &[OdeBinding], initial: &[DyadicInterval]| {
        rotation.ode_taylor_coefficients(bindings, &d(0, 0, 0), initial, 2, -64, &mut g(), &mut a())
    };
    assert!(matches!(
        call(&[State(0)], &vec![d(0, 0, 0); 2]),
        Err(TaylorError::Expression(_))
    ));
    assert!(matches!(
        call(&[State(0), State(1)], &[d(0, 0, 0)]),
        Err(TaylorError::OdeShape {
            outputs: 2,
            state: 1,
            lane: None
        })
    ));
    assert!(matches!(
        call(&[State(0), State(2)], &vec![d(0, 0, 0); 2]),
        Err(TaylorError::OdeShape { lane: Some(2), .. })
    ));
    assert!(
        call(&[Time, State(1)], &vec![d(0, 0, 0); 2])
            .unwrap()
            .is_some()
    );
}
