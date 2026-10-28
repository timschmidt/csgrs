//! Polynomial equations on rational charts over retained algebraic fields.
//! Expectations use explicit coefficients and the independent relation a^3=2.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use hyperreal::{Real, RealSign};
use hypersolve::BivariatePolynomial;
use hypersolve::classification::Classification;
use hypersolve::recursive_quadratic_field::*;
use support::*;

fn cubic_field() -> RecursiveQuadraticField {
    RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap()
}

fn in_cubic(field: &RecursiveQuadraticField, coefficients: &[Real]) -> RecursiveQuadraticValue {
    axis_value(field, coefficients, &[real(0)], &[real(0)], &[real(0)]).unwrap()
}

fn assert_same(actual: &RecursiveQuadraticValue, expected: &RecursiveQuadraticValue) {
    assert!(actual.field().same_field(&expected.field()));
    assert_eq!(
        actual
            .subtract(expected)
            .unwrap()
            .sign(&TestContext::strict())
            .unwrap(),
        Classification::Decided(RealSign::Zero)
    );
}

#[test]
fn recursive_field_circle_substitution_keeps_cubic_center_relation() {
    let field = cubic_field();
    let a = in_cubic(&field, &[real(0), real(1)]);
    let r = fraction(1, 64);
    let r2 = &r * &r;
    let r3 = &r2 * &r;
    // q=(a,1) lies on y^2-x^3+1=0. The rational circle chart is
    // x=a+r(1-t^2)/(1+t^2), y=1+2rt/(1+t^2).
    let x = vec![
        a.add(&field.constant(r.clone()).unwrap()).unwrap(),
        field.constant(real(0)).unwrap(),
        a.subtract(&field.constant(r.clone()).unwrap()).unwrap(),
    ];
    let y = vec![
        field.constant(real(1)).unwrap(),
        field.constant(real(2) * &r).unwrap(),
        field.constant(real(1)).unwrap(),
    ];
    let w = vec![
        field.constant(real(1)).unwrap(),
        field.constant(real(0)).unwrap(),
        field.constant(real(1)).unwrap(),
    ];
    let equation = BivariatePolynomial::new(vec![
        vec![real(1), real(0), real(1)],
        vec![],
        vec![],
        vec![real(-1)],
    ]);
    let actual = compose_bivariate_homogeneous_polynomial(&equation, &x, &y, &w, 3).unwrap();
    // Expansion of Y^2 W - X^3 + W^3, reduced only by a^3=2.
    let expected = [
        vec![-r3.clone(), -real(3) * &r2, -real(3) * &r],
        vec![real(4) * &r],
        vec![real(4) * &r2 + real(3) * &r3, real(3) * &r2, -real(3) * &r],
        vec![real(8) * &r],
        vec![real(4) * &r2 - real(3) * &r3, real(3) * &r2, real(3) * &r],
        vec![real(4) * &r],
        vec![r3, -real(3) * r2, real(3) * r],
    ];
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().zip(&expected) {
        assert_same(actual, &in_cubic(&field, expected));
    }
    // The new polynomial is itself consumed by exact retained-field algebra.
    // Reversing the circle chart t -> -t changes precisely the odd terms.
    let reversed_y = vec![y[0].clone(), y[1].scale(&real(-1)).unwrap(), y[2].clone()];
    let reversed =
        compose_bivariate_homogeneous_polynomial(&equation, &x, &reversed_y, &w, 3).unwrap();
    for (index, (forward, backward)) in actual.iter().zip(&reversed).enumerate() {
        assert_same(
            backward,
            &forward
                .scale(&real(if index % 2 == 0 { 1 } else { -1 }))
                .unwrap(),
        );
    }
}

#[test]
fn recursive_field_substitution_has_no_fixed_polynomial_degree_limit() {
    let field = cubic_field();
    let one = field.constant(real(1)).unwrap();
    let x = vec![in_cubic(&field, &[real(0), real(1)]), one.clone()];
    let y = vec![one.clone(), field.constant(real(-1)).unwrap()];
    let mut rows = vec![vec![real(-3), real(0), real(1)]];
    rows.resize(38, Vec::new());
    rows[37] = vec![real(1)];
    let equation = BivariatePolynomial::new(rows);
    let actual =
        compose_bivariate_homogeneous_polynomial(&equation, &x, &y, std::slice::from_ref(&one), 37)
            .unwrap();
    assert_eq!(actual.len(), 38);
    let mut binomial = 1_i64;
    for (k, coefficient) in actual.iter().enumerate() {
        let exponent = 37 - k;
        let mut expected = vec![real(0); 3];
        expected[exponent % 3] = Real::from(binomial * (1_i64 << (exponent / 3)));
        // (1-t)^2-3 contributes -2-2t+t^2 independently.
        expected[0] += Real::from(match k {
            0 | 1 => -2,
            2 => 1,
            _ => 0,
        });
        assert_same(coefficient, &in_cubic(&field, &expected));
        if k < 37 {
            binomial = binomial * (37 - k as i64) / (k as i64 + 1);
        }
    }
}

#[test]
fn recursive_field_substitution_preserves_homogeneous_factors_and_refuses_invalid_inputs() {
    let field = cubic_field();
    let one = field.constant(real(1)).unwrap();
    let zero = field.constant(real(0)).unwrap();
    let equation = BivariatePolynomial::new(vec![vec![real(0), real(1)], vec![real(1)]]);
    // At degree two, P=x+y becomes XW+YW. With X=Y=1,W=t
    // the factor t must remain; composition does not extend a rational chart
    // to the excluded denominator zero or cancel its authored guard.
    let output = compose_bivariate_homogeneous_polynomial(
        &equation,
        std::slice::from_ref(&one),
        std::slice::from_ref(&one),
        &[zero.clone(), one.clone()],
        2,
    )
    .unwrap();
    assert_eq!(output.len(), 2);
    assert_same(&output[0], &zero);
    assert_same(&output[1], &field.constant(real(2)).unwrap());
    for empty_axis in 0..3 {
        let mut inputs = [vec![one.clone()], vec![one.clone()], vec![one.clone()]];
        inputs[empty_axis].clear();
        assert!(
            compose_bivariate_homogeneous_polynomial(
                &equation, &inputs[0], &inputs[1], &inputs[2], 2
            )
            .is_none()
        );
    }
    assert!(
        compose_bivariate_homogeneous_polynomial(
            &equation,
            std::slice::from_ref(&one),
            std::slice::from_ref(&one),
            std::slice::from_ref(&one),
            0
        )
        .is_none()
    );
    assert!(
        compose_bivariate_homogeneous_polynomial(
            &equation,
            std::slice::from_ref(&one),
            std::slice::from_ref(&one),
            std::slice::from_ref(&one),
            usize::MAX
        )
        .is_none()
    );
    let foreign = cubic_field().constant(real(1)).unwrap();
    assert!(
        compose_bivariate_homogeneous_polynomial(
            &equation,
            std::slice::from_ref(&one),
            std::slice::from_ref(&foreign),
            std::slice::from_ref(&one),
            2
        )
        .is_none()
    );
}

#[test]
fn composed_circle_contact_is_consumed_by_retained_field_root_isolation() {
    use hypersolve::ordered_field_roots::{
        OrderedFieldRootIsolationConfig, OrderedFieldRootIsolationStatus,
        isolate_ordered_field_polynomial_roots,
    };

    let field = cubic_field();
    let a = in_cubic(&field, &[real(0), real(1)]);
    let zero = field.constant(real(0)).unwrap();
    let two = field.constant(real(2)).unwrap();
    // On the radius-two circle about (a,0), x=0 gives
    // (a+2)+(a-2)t^2=0. Since 6/5<a<4/3, its unique positive
    // root lies strictly between 2 and 3. No projected scalar is needed.
    let x = [
        a.add(&two).unwrap(),
        zero.clone(),
        a.subtract(&two).unwrap(),
    ];
    let y = [zero.clone(), field.constant(real(4)).unwrap()];
    let w = [
        field.constant(real(1)).unwrap(),
        zero,
        field.constant(real(1)).unwrap(),
    ];
    let line = BivariatePolynomial::new(vec![vec![], vec![real(1)]]);
    let coefficients = compose_bivariate_homogeneous_polynomial(&line, &x, &y, &w, 1).unwrap();
    let mut context = RecursiveQuadraticOrderedFieldContext {
        field: field.clone(),
        policy: TestContext::strict(),
    };
    let report = isolate_ordered_field_polynomial_roots(
        coefficients.clone(),
        &real(2),
        &real(3),
        OrderedFieldRootIsolationConfig::default(),
        &mut context,
    )
    .unwrap();
    assert_eq!(report.status, OrderedFieldRootIsolationStatus::Isolated);
    assert_eq!(report.intervals.len(), 1);
    let root = &report.intervals[0];
    assert_eq!(root.distinct_root_count, 1);
    assert!(root.lower > real(2) && root.upper < real(3));
    assert!(root.exact_root.is_none());
    // The original distance relation supplies the independent endpoint signs.
    for (t, sign) in [
        (&root.lower, RealSign::Positive),
        (&root.upper, RealSign::Negative),
    ] {
        let t2 = t * t;
        let residual = a
            .scale(&(real(1) + &t2))
            .unwrap()
            .add(&field.constant(real(2) - real(2) * t2).unwrap())
            .unwrap();
        assert_eq!(
            residual.sign(&context.policy).unwrap(),
            Classification::Decided(sign)
        );
    }
    assert_eq!(context.policy.approximations(), 0);
    assert!(
        coefficients
            .iter()
            .all(|value| value.field().same_field(&field))
    );
}

#[test]
fn substitution_retains_nested_radical_relations() {
    let base = cubic_field();
    let a = in_cubic(&base, &[real(0), real(1)]);
    let field = base
        .extension(a.add(&base.constant(real(1)).unwrap()).unwrap())
        .unwrap();
    let b = field
        .element(
            base.constant(real(0)).unwrap(),
            base.constant(real(1)).unwrap(),
        )
        .unwrap();
    // b=sqrt(a+1). Substituting X=b+t, Y=a, W=1 into
    // x^2-y-1 leaves exactly 2bt+t^2 in the same selected extension.
    let one = field.constant(real(1)).unwrap();
    let equation = BivariatePolynomial::new(vec![vec![real(-1), real(-1)], vec![], vec![real(1)]]);
    let coefficients = compose_bivariate_homogeneous_polynomial(
        &equation,
        &[b.clone(), one.clone()],
        &[field.lift(&a).unwrap()],
        std::slice::from_ref(&one),
        2,
    )
    .unwrap();
    assert_eq!(coefficients.len(), 3);
    assert_same(&coefficients[0], &field.constant(real(0)).unwrap());
    assert_same(&coefficients[1], &b.scale(&real(2)).unwrap());
    assert_same(&coefficients[2], &one);
}
