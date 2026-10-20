//! Policy-qualified signs of represented values and dense tensor values at
//! selected root tuples.
//!
//! Expected signs come from exact identities or from certified `hyperreal`
//! signs of the same closed forms. Approximate decisions are exercised only
//! on exact zeroes that STRICT cannot prove, so the approximate answer is
//! also the true one, and every consumed approximation must be recorded.

#[path = "support/represented_roots.rs"]
mod support;

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::radical_expression::TwoSquareRootExpression;
use hypersolve::tensor_support::dense_tensor_with_output_axis;
use hypersolve::*;
use support::*;

const SIGNS: [RealSign; 3] = [RealSign::Negative, RealSign::Zero, RealSign::Positive];

fn signum(sign: RealSign) -> i64 {
    match sign {
        RealSign::Negative => -1,
        RealSign::Zero => 0,
        RealSign::Positive => 1,
    }
}

/// `x` on the single source axis.
fn x() -> DenseTensorPolynomial {
    on_axis(1, 0, ints(&[0, 1]))
}

fn c(value: i64) -> DenseTensorPolynomial {
    constant(1, int(value))
}

fn linear(constant_term: i64, slope: i64) -> DenseTensorPolynomial {
    on_axis(1, 0, ints(&[constant_term, slope]))
}

fn two_root(
    rational: DenseTensorPolynomial,
    first: DenseTensorPolynomial,
    second: DenseTensorPolynomial,
    product: DenseTensorPolynomial,
) -> TwoSquareRootExpression<DenseTensorPolynomial> {
    TwoSquareRootExpression {
        rational,
        first,
        second,
        product,
    }
}

// Sign combinators.

#[test]
fn same_sheet_signs_add_like_signed_magnitudes() {
    for first in SIGNS {
        for second in SIGNS {
            let expected = match (signum(first), signum(second)) {
                (0, s) | (s, 0) => Some(s),
                (a, b) if a == b => Some(a),
                // A positive and a negative term can sum to any sign.
                _ => None,
            };
            assert_eq!(
                same_positive_root_sheet_signs(first, second).map(signum),
                expected,
                "{first:?} + {second:?}"
            );
        }
    }
}

#[test]
fn combined_uncertainty_prefers_budget_then_unsupported_then_the_first_reason() {
    use UncertaintyReason as R;
    let decided = Classification::Decided(RealSign::Positive);
    let u = Classification::<RealSign>::Uncertain;
    for (first, second, expected) in [
        (u(R::BudgetExhausted), decided, R::BudgetExhausted),
        (decided, u(R::BudgetExhausted), R::BudgetExhausted),
        (u(R::Unsupported), u(R::BudgetExhausted), R::BudgetExhausted),
        (u(R::Unsupported), u(R::Predicate), R::Unsupported),
        (u(R::Predicate), u(R::Unsupported), R::Unsupported),
        (u(R::RealSign), u(R::Predicate), R::RealSign),
        (decided, u(R::Boundary), R::Boundary),
        (u(R::Ordering), decided, R::Ordering),
        (decided, decided, R::Predicate),
    ] {
        assert_eq!(combined_sign_uncertainty(&first, &second), expected);
    }
}

#[test]
fn positive_root_sum_components_match_certified_signs_of_a_plus_b_sqrt_s() {
    // Every A + B*sqrt(S) over a small exact grid with S > 0, the documented
    // domain, against hyperreal's sign.
    for a in -3..=3 {
        for b in -3..=3 {
            for s in [1, 2, 4, 9] {
                let rational = certified_sign(&int(a));
                let radical = certified_sign(&int(b));
                let magnitude = certified_sign(&int(a * a - b * b * s));
                let expected = certified_sign(&(int(a) + int(b) * sqrt(s)));
                assert_eq!(
                    positive_root_sum_sign_from_components(
                        Classification::Decided(rational),
                        Classification::Decided(radical),
                        Classification::Decided(magnitude),
                    ),
                    Classification::Decided(expected),
                    "{a} + {b}*sqrt({s})"
                );
            }
        }
    }
    let undecided = Classification::Uncertain(UncertaintyReason::RealSign);
    assert_eq!(
        positive_root_sum_sign_from_components(
            undecided,
            Classification::Decided(RealSign::Positive),
            Classification::Decided(RealSign::Zero),
        ),
        undecided
    );
    assert_eq!(
        positive_root_sum_sign_from_components(
            Classification::Decided(RealSign::Positive),
            Classification::Decided(RealSign::Negative),
            Classification::Uncertain(UncertaintyReason::Predicate),
        ),
        Classification::Uncertain(UncertaintyReason::Predicate)
    );
}

#[test]
fn strict_interval_signs_require_the_whole_interval() {
    let interval = |lower: Real, upper: Real| RealInterval { lower, upper };
    assert_eq!(
        dense_strict_interval_sign(&interval(int(-3), q(-1, 7))),
        Some(RealSign::Negative)
    );
    assert_eq!(
        dense_strict_interval_sign(&interval(q(1, 9), sqrt(2))),
        Some(RealSign::Positive)
    );
    assert_eq!(
        dense_strict_interval_sign(&interval(Real::zero(), Real::zero())),
        Some(RealSign::Zero)
    );
    for (lower, upper) in [
        (int(-1), int(1)),
        (Real::zero(), int(1)),
        (int(-1), Real::zero()),
        (unknown_zero(), unknown_zero()),
    ] {
        assert_eq!(dense_strict_interval_sign(&interval(lower, upper)), None);
    }
}

// Policy signs and orders of represented values.

#[test]
fn policy_signs_consume_approximation_only_when_strict_signs_fail() {
    for mode in [
        Mode::Strict,
        Mode::Approximate,
        Mode::StrictPassOfApproximate,
    ] {
        let policy = Policy::new(mode);
        assert_eq!(
            represented_policy_sign(&sqrt2(), &policy),
            Classification::Decided(RealSign::Positive)
        );
        assert_eq!(
            represented_policy_sign(&negative_sqrt2(), &policy),
            Classification::Decided(RealSign::Negative)
        );
        assert_eq!(policy.observed.get(), 0);
    }

    // sin(1)^2 + cos(1)^2 - 1 is exactly zero, but not provably so.
    let zero = exact(&unknown_zero());
    for mode in [Mode::Strict, Mode::StrictPassOfApproximate] {
        let policy = Policy::new(mode);
        assert_eq!(
            represented_policy_sign(&zero, &policy),
            Classification::Uncertain(UncertaintyReason::Predicate)
        );
        assert_eq!(policy.observed.get(), 0);
    }
    let policy = Policy::new(Mode::Approximate);
    assert_eq!(
        represented_policy_sign(&zero, &policy),
        Classification::Decided(RealSign::Zero)
    );
    assert_eq!(policy.observed.get(), 1);

    // Invalid evidence is never decided, even approximately.
    let policy = Policy::new(Mode::Approximate);
    assert_eq!(
        represented_policy_sign(&non_unit_root(), &policy),
        Classification::Uncertain(UncertaintyReason::Predicate)
    );
    assert_eq!(policy.observed.get(), 0);
}

#[test]
fn orders_against_exact_reals() {
    let policy = Policy::new(Mode::Strict);
    assert_eq!(
        represented_order_to_real(&sqrt2(), &int(1), &policy),
        Classification::Decided(Ordering::Greater)
    );
    assert_eq!(
        represented_order_to_real(&sqrt2(), &q(3, 2), &policy),
        Classification::Decided(Ordering::Less)
    );
    assert_eq!(
        represented_order_to_real(&exact(&int(2)), &int(2), &policy),
        Classification::Decided(Ordering::Equal)
    );
    assert_eq!(
        represented_order_to_real(&sqrt2(), &sqrt(2), &policy),
        Classification::Decided(Ordering::Equal)
    );
    assert_eq!(policy.observed.get(), 0);

    // u = 0 exactly, but STRICT cannot order u against zero.
    for (value, target) in [
        (exact(&unknown_zero()), Real::zero()),
        (exact(&Real::zero()), unknown_zero()),
    ] {
        let strict = Policy::new(Mode::Strict);
        assert_eq!(
            represented_order_to_real(&value, &target, &strict),
            Classification::Uncertain(UncertaintyReason::Predicate)
        );
        let approximate = Policy::new(Mode::Approximate);
        assert_eq!(
            represented_order_to_real(&value, &target, &approximate),
            Classification::Decided(Ordering::Equal)
        );
        assert_eq!(approximate.observed.get(), 1);
    }

    assert_eq!(
        represented_order_to_real(&non_unit_root(), &int(1), &Policy::new(Mode::Approximate)),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

// Dense tuple signs by refinement.

#[test]
fn tuple_signs_by_refinement_decide_strict_signs_and_zeroes() {
    let strict = Policy::new(Mode::Strict);
    let sign = |polynomial: &DenseTensorPolynomial,
                sources: &[AlgebraicRootRepresentation],
                policy: &Policy| {
        dense_tuple_sign_by_refinement(
            polynomial,
            &dense_tensor_with_output_axis(polynomial).unwrap(),
            sources,
            policy,
        )
    };
    // sqrt(2) - 1 > 0 once the isolator excludes 1.
    assert_eq!(
        sign(&linear(-1, 1), &[sqrt2()], &strict),
        Classification::Decided(RealSign::Positive)
    );
    assert_eq!(
        sign(&linear(2, -1), &[cbrt2()], &strict),
        Classification::Decided(RealSign::Positive)
    );
    // sqrt(2)^2 - 2 = 0 at one source.
    let square_minus_two = on_axis(1, 0, ints(&[-2, 0, 1]));
    assert_eq!(
        sign(&square_minus_two, &[sqrt2()], &strict),
        Classification::Decided(RealSign::Zero)
    );
    // x^2 y^2 - 6 = 0 at (sqrt(2), sqrt(3)).
    let mut coefficients = ints(&[0; 9]);
    coefficients[0] = int(-6);
    coefficients[8] = int(1);
    assert_eq!(
        sign(&tensor(&[3, 3], coefficients), &[sqrt2(), sqrt3()], &strict),
        Classification::Decided(RealSign::Zero)
    );
    // x*y - 2 = sqrt(6) - 2 > 0 and 2 - x*y < 0.
    assert_eq!(
        sign(
            &tensor(&[2, 2], ints(&[-2, 0, 0, 1])),
            &[sqrt2(), sqrt3()],
            &strict
        ),
        Classification::Decided(RealSign::Positive)
    );
    assert_eq!(
        sign(
            &tensor(&[2, 2], ints(&[2, 0, 0, -1])),
            &[sqrt2(), sqrt3()],
            &strict
        ),
        Classification::Decided(RealSign::Negative)
    );
    assert_eq!(strict.observed.get(), 0);

    // Selecting the approximate policy does not qualify a certified sign.
    let approximate = Policy::new(Mode::Approximate);
    assert_eq!(
        sign(&linear(-1, 1), &[sqrt2()], &approximate),
        Classification::Decided(RealSign::Positive)
    );
    assert_eq!(approximate.observed.get(), 0);
}

#[test]
fn tuple_signs_by_refinement_stop_at_the_approximate_terminal() {
    let square_minus_two = on_axis(1, 0, ints(&[-2, 0, 1]));
    let value = dense_tensor_with_output_axis(&square_minus_two).unwrap();
    let sources = [sqrt2()];

    // The approximate operation consumes its terminal for the exact zero.
    let approximate = Policy::new(Mode::Approximate);
    assert_eq!(
        dense_tuple_sign_by_refinement(&square_minus_two, &value, &sources, &approximate),
        Classification::Decided(RealSign::Zero)
    );
    assert_eq!(approximate.observed.get(), 1);

    // Its preliminary strict pass leaves the sign undecided for the replay.
    let strict_pass = Policy::new(Mode::StrictPassOfApproximate);
    assert_eq!(
        dense_tuple_sign_by_refinement(&square_minus_two, &value, &sources, &strict_pass),
        Classification::Uncertain(UncertaintyReason::Predicate)
    );
    assert_eq!(strict_pass.observed.get(), 0);
}

#[test]
fn tuple_signs_at_unrefinable_exact_sources() {
    // The exact source u = 0 cannot be refined, and its sign is not provable.
    let sources = [exact(&unknown_zero())];
    let identity = x();
    let value = dense_tensor_with_output_axis(&identity).unwrap();
    for mode in [Mode::Strict, Mode::StrictPassOfApproximate] {
        let policy = Policy::new(mode);
        assert!(
            dense_tuple_sign_by_refinement(&identity, &value, &sources, &policy).is_uncertain()
        );
        assert_eq!(policy.observed.get(), 0);
    }
    let approximate = Policy::new(Mode::Approximate);
    assert_eq!(
        dense_tuple_sign_by_refinement(&identity, &value, &sources, &approximate),
        Classification::Decided(RealSign::Zero)
    );
    assert_eq!(approximate.observed.get(), 1);
}

// Dense polynomial tuple signs through a selected-algebra context.

#[test]
fn tuple_signs_reduce_collapse_and_substitute_before_refinement() {
    let context = Context::strict();
    // x^2 - 2 reduces to zero modulo the defining polynomial of sqrt(2).
    assert_eq!(
        dense_polynomial_tuple_sign(&on_axis(1, 0, ints(&[-2, 0, 1])), &[sqrt2()], &context),
        Ok(Classification::Decided(RealSign::Zero))
    );
    // x - y collapses to zero for two representations of sqrt(2).
    assert_eq!(
        dense_polynomial_tuple_sign(
            &tensor(&[2, 2], ints(&[0, -1, 1, 0])),
            &[sqrt2(), sqrt2_from_cubic()],
            &context
        ),
        Ok(Classification::Decided(RealSign::Zero))
    );
    // Neither zero needed the one-root authority.
    assert_eq!(context.authority_calls.get(), 0);
    // x*y - 2 = sqrt(6) - 2 > 0 and x - y = sqrt(2) - cbrt(2) > 0.
    assert_eq!(
        dense_polynomial_tuple_sign(
            &tensor(&[2, 2], ints(&[-2, 0, 0, 1])),
            &[sqrt2(), sqrt3()],
            &context
        ),
        Ok(Classification::Decided(RealSign::Positive))
    );
    assert_eq!(
        dense_polynomial_tuple_sign_owned(
            tensor(&[2, 2], ints(&[0, -1, 1, 0])),
            &[sqrt2(), cbrt2()],
            &context
        ),
        Ok(Classification::Decided(RealSign::Positive))
    );

    // An exact point source is substituted: x*y - 4 at (sqrt(2), 3) is
    // 3*sqrt(2) - 4 > 0, its negation is negative, and x*y - 3x vanishes.
    let point = [sqrt2(), exact(&int(3))];
    for (coefficients, expected) in [
        ([-4, 0, 0, 1], RealSign::Positive),
        ([4, 0, 0, -1], RealSign::Negative),
        ([0, 0, -3, 1], RealSign::Zero),
    ] {
        assert_eq!(
            dense_polynomial_tuple_sign(&tensor(&[2, 2], ints(&coefficients)), &point, &context),
            Ok(Classification::Decided(expected)),
            "{coefficients:?}"
        );
    }

    // A coefficient that is exactly zero but not provably so.
    assert_eq!(
        dense_polynomial_tuple_sign(&constant(1, unknown_zero()), &[sqrt2()], &context),
        Ok(Classification::Uncertain(UncertaintyReason::RealSign))
    );
    // One axis per source is required.
    assert_eq!(
        dense_polynomial_tuple_sign(&tensor(&[2, 2], ints(&[0, 1, 1, 0])), &[sqrt2()], &context),
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    );
}

#[test]
fn one_source_tuple_signs_consult_the_context_authority() {
    // sqrt(2) - 1 > 0; the deferring authority leaves it to refinement.
    let deferring = Context::strict();
    assert_eq!(
        dense_polynomial_tuple_sign(&linear(-1, 1), &[sqrt2()], &deferring),
        Ok(Classification::Decided(RealSign::Positive))
    );
    assert_eq!(deferring.authority_calls.get(), 1);

    // Inside a bounded exact pass the authority's uncertainty is final.
    let bounded = Context::new(Mode::Strict, Authority::Defer, true);
    assert_eq!(
        dense_polynomial_tuple_sign(&linear(-1, 1), &[sqrt2()], &bounded),
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    );
    assert_eq!(bounded.authority_calls.get(), 1);

    // A failing authority's typed error survives.
    let failing = Context::new(Mode::Strict, Authority::Fail, false);
    assert_eq!(
        dense_polynomial_tuple_sign(&linear(-1, 1), &[sqrt2()], &failing),
        Err(ContextError::Authority)
    );
}

// Signs of A + B*sqrt(S).

#[test]
fn positive_square_root_sums_match_certified_closed_form_signs() {
    let context = Context::strict();
    let sources = [sqrt2()];
    for a0 in -2..=2 {
        for a1 in -1..=1 {
            for b in -1..=1 {
                for s in [1, 2, 3, 8] {
                    let expected =
                        certified_sign(&(int(a0) + int(a1) * sqrt(2) + int(b) * sqrt(s)));
                    assert_eq!(
                        dense_positive_square_root_sum_sign(
                            &linear(a0, a1),
                            &c(b),
                            &c(s),
                            &sources,
                            &context
                        ),
                        Ok(Classification::Decided(expected)),
                        "{a0} + {a1}*sqrt(2) + {b}*sqrt({s})"
                    );
                }
            }
        }
    }
    // A radicand polynomial in the source: sqrt(x^2 + 1) - x = sqrt(3) - sqrt(2).
    assert_eq!(
        dense_positive_square_root_sum_sign(
            &linear(0, -1),
            &c(1),
            &on_axis(1, 0, ints(&[1, 0, 1])),
            &sources,
            &context
        ),
        Ok(Classification::Decided(RealSign::Positive))
    );
}

#[test]
fn positive_square_root_sums_report_blockers() {
    let context = Context::strict();
    // Wrong rank.
    assert_eq!(
        dense_positive_square_root_sum_sign(
            &tensor(&[2, 1], ints(&[0, 1])),
            &c(1),
            &c(2),
            &[sqrt2()],
            &context
        ),
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    );
    // u + 0*sqrt(1) with u = 0 exactly but not provably so.
    assert_eq!(
        dense_positive_square_root_sum_sign(
            &constant(1, unknown_zero()),
            &c(0),
            &c(1),
            &[sqrt2()],
            &context
        ),
        Ok(Classification::Uncertain(UncertaintyReason::RealSign))
    );
    // (x - 1) + sqrt(2): the failing authority is consulted for x - 1.
    let failing = Context::new(Mode::Strict, Authority::Fail, false);
    assert_eq!(
        dense_positive_square_root_sum_sign(&linear(-1, 1), &c(1), &c(2), &[sqrt2()], &failing),
        Err(ContextError::Authority)
    );
}

// Signs of rational + first*sqrt(A) + second*sqrt(B) + product*sqrt(A*B).

#[test]
fn two_square_root_sums_match_certified_closed_form_signs() {
    let context = Context::strict();
    let sources = [sqrt2()];
    for (a, b) in [(2, 3), (3, 2), (2, 8)] {
        for r0 in -1..=1 {
            for r1 in -1..=1 {
                for f in [-1, 1] {
                    for s in [-1, 1] {
                        for p in -1..=1 {
                            let expected = certified_sign(
                                &(int(r0)
                                    + int(r1) * sqrt(2)
                                    + int(f) * sqrt(a)
                                    + int(s) * sqrt(b)
                                    + int(p) * sqrt(a) * sqrt(b)),
                            );
                            let expression = two_root(linear(r0, r1), c(f), c(s), c(p));
                            assert_eq!(
                                dense_two_positive_square_root_sum_sign(
                                    &expression,
                                    &c(a),
                                    &c(b),
                                    &sources,
                                    &context
                                ),
                                Ok(Classification::Decided(expected)),
                                "{r0} + {r1}*sqrt(2) + {f}*sqrt({a}) + {s}*sqrt({b}) + {p}*sqrt({a}*{b})"
                            );
                        }
                    }
                }
            }
        }
    }
}

/// Inputs `u + v*sqrt(B)`, `u = r + f*sqrt(A)` and `v = s + p*sqrt(A)` at
/// `x = sqrt(2)`, each with its exact value. In every case the conjugate
/// `u' + v'*sqrt(B)` under `sqrt(A) -> -sqrt(A)` or the value itself
/// vanishes, so the complete two-radical norm is zero.
fn projected_zero_cases() -> Vec<(
    &'static str,
    TwoSquareRootExpression<DenseTensorPolynomial>,
    DenseTensorPolynomial,
    DenseTensorPolynomial,
    Real,
)> {
    let half_three_minus_two_x = on_axis(1, 0, vec![q(3, 2), int(-1)]);
    vec![
        // sqrt(2) + sqrt(2): the conjugate sqrt(2) - sqrt(2) vanishes.
        (
            "agreeing terms",
            two_root(x(), c(0), c(1), c(0)),
            c(1),
            c(2),
            int(2) * sqrt(2),
        ),
        // (1 + sqrt(2)) - sqrt(2)*sqrt((3 - 2 sqrt(2))/2) = 2.
        (
            "positive norm",
            two_root(c(1), c(1), c(0), c(-1)),
            c(2),
            half_three_minus_two_x,
            int(2),
        ),
        // (1 - sqrt(2)) + sqrt(3 + 2 sqrt(2)) = 2.
        (
            "negative norm",
            two_root(c(1), c(-1), c(1), c(0)),
            c(2),
            linear(3, 2),
            int(2),
        ),
        // (1 + sqrt(2)) - sqrt(3 + 2 sqrt(2)) = 0.
        (
            "cancelling terms",
            two_root(c(1), c(1), c(-1), c(0)),
            c(2),
            linear(3, 2),
            Real::zero(),
        ),
        // sqrt(2) - sqrt(2) = 0.
        (
            "zero norm",
            two_root(x(), c(0), c(-1), c(0)),
            c(1),
            c(2),
            Real::zero(),
        ),
    ]
}

#[test]
fn two_square_root_sums_select_the_authored_sheet() {
    let context = Context::strict();
    for (label, expression, first, second, value) in projected_zero_cases() {
        let expected = certified_sign(&value);
        assert_eq!(
            dense_two_positive_square_root_sum_sign(
                &expression,
                &first,
                &second,
                &[sqrt2()],
                &context
            ),
            Ok(Classification::Decided(expected)),
            "{label}"
        );
        assert_eq!(
            dense_two_positive_square_root_sum_sign_at_projected_zero(
                &expression,
                &first,
                &second,
                &[sqrt2()],
                &context
            ),
            Ok(Classification::Decided(expected)),
            "{label} at a projected zero"
        );
    }
}

#[test]
fn projected_zero_signs_with_at_most_one_component_are_zero() {
    let context = Context::strict();
    // x^2 - 2 reduces to the zero polynomial.
    let expression = two_root(on_axis(1, 0, ints(&[-2, 0, 1])), c(0), c(0), c(0));
    assert_eq!(
        dense_two_positive_square_root_sum_sign_at_projected_zero(
            &expression,
            &c(2),
            &c(3),
            &[sqrt2()],
            &context
        ),
        Ok(Classification::Decided(RealSign::Zero))
    );
}

#[test]
fn two_square_root_sums_report_blockers() {
    let context = Context::strict();
    let wrong_rank = two_root(tensor(&[2, 1], ints(&[0, 1])), c(0), c(0), c(0));
    assert_eq!(
        dense_two_positive_square_root_sum_sign(&wrong_rank, &c(2), &c(3), &[sqrt2()], &context),
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    );
    assert_eq!(
        dense_two_positive_square_root_sum_sign_at_projected_zero(
            &wrong_rank,
            &c(2),
            &c(3),
            &[sqrt2()],
            &context
        ),
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    );
    // (x + u) - sqrt(2) with u = 0 exactly but not provably so.
    let with_unknown = two_root(
        on_axis(1, 0, vec![unknown_zero(), int(1)]),
        c(0),
        c(-1),
        c(0),
    );
    assert_eq!(
        dense_two_positive_square_root_sum_sign_at_projected_zero(
            &with_unknown,
            &c(1),
            &c(2),
            &[sqrt2()],
            &context
        ),
        Ok(Classification::Uncertain(UncertaintyReason::RealSign))
    );
    // u + sqrt(0) with u = 0 exactly but not provably so.
    let unknown_rational = two_root(constant(1, unknown_zero()), c(0), c(1), c(0));
    assert_eq!(
        dense_two_positive_square_root_sum_sign_at_projected_zero(
            &unknown_rational,
            &c(1),
            &c(0),
            &[sqrt2()],
            &context
        ),
        Ok(Classification::Uncertain(UncertaintyReason::RealSign))
    );
    assert!(
        dense_two_positive_square_root_sum_sign(&with_unknown, &c(1), &c(2), &[sqrt2()], &context)
            .unwrap()
            .is_uncertain()
    );
    // (x - sqrt(2)) + u*sqrt(1) = u: the retained part is certified zero,
    // but the candidate u is not.
    let unknown_candidate = two_root(x(), c(-1), constant(1, unknown_zero()), c(0));
    assert_eq!(
        dense_two_positive_square_root_sum_sign_at_projected_zero(
            &unknown_candidate,
            &c(2),
            &c(1),
            &[sqrt2()],
            &context
        ),
        Ok(Classification::Uncertain(UncertaintyReason::RealSign))
    );
}

#[test]
fn two_square_root_sums_propagate_authority_errors() {
    // Any nonzero one-source component consults the one-root authority.
    let failing = Context::new(Mode::Strict, Authority::Fail, false);
    let retained = two_root(linear(-1, 1), c(0), c(1), c(0));
    assert_eq!(
        dense_two_positive_square_root_sum_sign(&retained, &c(2), &c(3), &[sqrt2()], &failing),
        Err(ContextError::Authority)
    );
    assert_eq!(
        dense_two_positive_square_root_sum_sign_at_projected_zero(
            &retained,
            &c(2),
            &c(3),
            &[sqrt2()],
            &failing
        ),
        Err(ContextError::Authority)
    );
    // x^2 - 2 reduces to zero, so only the candidate x - 1 needs the authority.
    let candidate = two_root(on_axis(1, 0, ints(&[-2, 0, 1])), c(0), linear(-1, 1), c(0));
    assert_eq!(
        dense_two_positive_square_root_sum_sign(&candidate, &c(2), &c(3), &[sqrt2()], &failing),
        Err(ContextError::Authority)
    );
}
