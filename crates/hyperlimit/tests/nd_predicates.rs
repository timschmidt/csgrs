//! Small rational controls for dimension-generic orientation and lifted sphere signs.
//!
//! These fixtures check selected 2D/3D/4D cases and one invalid arity under an
//! approximation-permitted policy. Most assertions extract only signs; affine
//! dependence and invalid arity additionally require complete receipts. They do
//! not establish arbitrary-dimensional coverage or the behavior of symbolic data.

use hyperlimit::{
    PredicateOutcome, RefinementNeed, Sign, affine_independent_d, insphere_d, orient_d,
};
use hyperreal::{Rational, Real};

/// Approximation-permitted policy for the rational simplex controls.
const APPROX: hyperlimit::PredicatePolicy = hyperlimit::PredicatePolicy::APPROXIMATE_512;

/// Convert an integer coordinate slice to an exact Real point vector.
fn p(values: &[i64]) -> Vec<Real> {
    values.iter().copied().map(Real::from).collect()
}

/// Construct an exact rational fixture scalar, panicking for an invalid denominator.
fn q(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).unwrap())
}

/// Extract a decided sign, panicking on Unknown and discarding certainty/stage.
fn decided(outcome: PredicateOutcome<Sign>) -> Sign {
    outcome.value().expect("predicate should be decided")
}

/// Require positive orientation for the origin followed by the ordered standard basis.
///
/// The two fixtures fix the dimension-generic sign convention in 3D and 4D; this
/// 3D convention differs from orient3's last-point-anchored determinant. Only
/// sign values, not their receipts, are asserted here.
#[test]
fn orient_d_decides_exact_3d_and_4d_simplex_signs() {
    let tetra = vec![p(&[0, 0, 0]), p(&[1, 0, 0]), p(&[0, 1, 0]), p(&[0, 0, 1])];
    assert_eq!(decided(orient_d(&tetra, APPROX)), Sign::Positive);

    let simplex4 = vec![
        p(&[0, 0, 0, 0]),
        p(&[1, 0, 0, 0]),
        p(&[0, 1, 0, 0]),
        p(&[0, 0, 1, 0]),
        p(&[0, 0, 0, 1]),
    ];
    assert_eq!(decided(orient_d(&simplex4, APPROX)), Sign::Positive);
}

/// Require false with an Exact/Structural receipt for four collinear 3D points.
#[test]
fn affine_independence_reports_zero_for_dependent_simplex() {
    let dependent = vec![p(&[0, 0, 0]), p(&[1, 0, 0]), p(&[2, 0, 0]), p(&[3, 0, 0])];
    assert_eq!(
        affine_independent_d(&dependent, APPROX),
        PredicateOutcome::decided(
            false,
            hyperlimit::Certainty::Exact,
            hyperlimit::Escalation::Structural
        )
    );
}

/// Check raw lifted signs for a fixed positively oriented 2D simplex.
///
/// The query `(1/4,1/4)` yields Positive and `(1,1)` yields Zero. This does not
/// assert orientation-independent sphere membership or higher-dimensional cases.
#[test]
fn insphere_d_decides_exact_inside_and_boundary_cases() {
    let simplex = vec![p(&[0, 0]), p(&[1, 0]), p(&[0, 1])];
    assert_eq!(
        decided(insphere_d(&simplex, &[q(1, 4), q(1, 4)], APPROX)),
        Sign::Positive
    );
    assert_eq!(
        decided(insphere_d(
            &simplex,
            &[Real::from(1), Real::from(1)],
            APPROX
        )),
        Sign::Zero
    );
}

/// Require Unknown/Unsupported at Exact stage for two points in two dimensions.
///
/// Imported and fully qualified calls reach the same function; they are not
/// independent implementations or separate invalid-shape cases.
#[test]
fn invalid_nd_predicate_arity_is_explicit_unknown() {
    assert_eq!(
        orient_d(&[p(&[0, 0]), p(&[1, 0])], APPROX),
        PredicateOutcome::unknown(RefinementNeed::Unsupported, hyperlimit::Escalation::Exact)
    );
    assert_eq!(
        hyperlimit::orient_d(&[p(&[0, 0]), p(&[1, 0])], APPROX),
        PredicateOutcome::unknown(RefinementNeed::Unsupported, hyperlimit::Escalation::Exact)
    );
}
