use hyperlimit::{
    PredicateOutcome, RefinementNeed, Sign, affine_independent_d, insphere_d, orient_d,
};
use hyperreal::{Rational, Real};

fn p(values: &[i64]) -> Vec<Real> {
    values.iter().copied().map(Real::from).collect()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).unwrap())
}

fn decided(outcome: PredicateOutcome<Sign>) -> Sign {
    outcome.value().expect("predicate should be decided")
}

#[test]
fn orient_d_decides_exact_3d_and_4d_simplex_signs() {
    let tetra = vec![p(&[0, 0, 0]), p(&[1, 0, 0]), p(&[0, 1, 0]), p(&[0, 0, 1])];
    assert_eq!(decided(orient_d(&tetra)), Sign::Positive);

    let simplex4 = vec![
        p(&[0, 0, 0, 0]),
        p(&[1, 0, 0, 0]),
        p(&[0, 1, 0, 0]),
        p(&[0, 0, 1, 0]),
        p(&[0, 0, 0, 1]),
    ];
    assert_eq!(decided(orient_d(&simplex4)), Sign::Positive);
}

#[test]
fn affine_independence_reports_zero_for_dependent_simplex() {
    let dependent = vec![p(&[0, 0, 0]), p(&[1, 0, 0]), p(&[2, 0, 0]), p(&[3, 0, 0])];
    assert_eq!(
        affine_independent_d(&dependent),
        PredicateOutcome::decided(
            false,
            hyperlimit::Certainty::Exact,
            hyperlimit::Escalation::Structural
        )
    );
}

#[test]
fn insphere_d_decides_exact_inside_and_boundary_cases() {
    let simplex = vec![p(&[0, 0]), p(&[1, 0]), p(&[0, 1])];
    assert_eq!(
        decided(insphere_d(&simplex, &[q(1, 4), q(1, 4)])),
        Sign::Positive
    );
    assert_eq!(
        decided(insphere_d(&simplex, &[Real::from(1), Real::from(1)])),
        Sign::Zero
    );
}

#[test]
fn invalid_nd_predicate_arity_is_explicit_unknown() {
    assert_eq!(
        orient_d(&[p(&[0, 0]), p(&[1, 0])]),
        PredicateOutcome::unknown(RefinementNeed::Unsupported, hyperlimit::Escalation::Exact)
    );
    assert_eq!(
        hyperlimit::orient_d(&[p(&[0, 0]), p(&[1, 0])]),
        PredicateOutcome::unknown(RefinementNeed::Unsupported, hyperlimit::Escalation::Exact)
    );
}
