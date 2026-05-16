//! Fuzz exact predicate invariants over small rational coordinate sets.
//!
//! The generated inputs stay in `hyperreal::Real` and never use primitive-float
//! topology. The checks focus on metamorphic laws that should survive every
//! exact kernel and fallback route: orientation reversal/cyclicity, batch/scalar
//! agreement, prepared-line and prepared-incircle agreement, and incircle
//! boundary behavior.
//!
//! Run with: `cargo fuzz run predicate_invariants` from `hyperlimit/fuzz/`.

#![no_main]

use arbitrary::Arbitrary;
use hyperlimit::{
    LineSide, Point2, PredicateOutcome, PredicatePolicy, Sign, certified_interval_sign,
    classify_point_line, classify_point_line_batch, incircle2d, orient2d, orient2d_batch,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

#[derive(Clone, Copy, Debug, Arbitrary)]
struct RawPoint {
    x_num: i16,
    x_den: u8,
    y_num: i16,
    y_den: u8,
}

impl RawPoint {
    fn into_point(self) -> Point2 {
        Point2::new(
            rational(self.x_num, self.x_den),
            rational(self.y_num, self.y_den),
        )
    }
}

#[derive(Clone, Copy, Debug, Arbitrary)]
struct Input {
    a: RawPoint,
    b: RawPoint,
    c: RawPoint,
    d: RawPoint,
}

fuzz_target!(|input: Input| {
    predicate_invariants(input);
});

fn predicate_invariants(input: Input) {
    let a = input.a.into_point();
    let b = input.b.into_point();
    let c = input.c.into_point();
    let d = input.d.into_point();

    let abc = orient2d(&a, &b, &c);
    let bca = orient2d(&b, &c, &a);
    let bac = orient2d(&b, &a, &c);

    if let (Some(abc), Some(bca), Some(bac)) = (abc.value(), bca.value(), bac.value()) {
        assert_eq!(abc, bca, "cyclic orientation should preserve sign");
        assert_eq!(
            abc.reversed(),
            bac,
            "swapping two vertices should reverse sign"
        );
    }

    let batch_cases = [
        (a.clone(), b.clone(), c.clone()),
        (b.clone(), a.clone(), c.clone()),
    ];
    let batch = orient2d_batch(&batch_cases);
    assert_eq!(batch[0].value(), orient2d(&a, &b, &c).value());
    assert_eq!(batch[1].value(), orient2d(&b, &a, &c).value());

    let line_side = classify_point_line(&a, &b, &c).value();
    if let Some(sign) = orient2d(&a, &b, &c).value() {
        assert_eq!(line_side, Some(LineSide::from(sign)));
    }

    let session = hyperlimit::ExactGeometrySession::default();
    let prepared = session.prepare_line2(&a, &b);
    assert_eq!(
        session.classify_prepared_line2(&prepared, &c).value(),
        line_side
    );

    let line_batch_cases = [
        (a.clone(), b.clone(), c.clone()),
        (a.clone(), b.clone(), d.clone()),
    ];
    let line_batch = classify_point_line_batch(&line_batch_cases);
    assert_eq!(
        line_batch[0].value(),
        classify_point_line(&a, &b, &c).value()
    );
    assert_eq!(
        line_batch[1].value(),
        classify_point_line(&a, &b, &d).value()
    );

    // Any input site lies exactly on its own circumcircle. Degenerate fixed
    // triples may make the circle predicate zero for broader reasons, but the
    // boundary-site law must always hold when the predicate decides.
    assert_decided_zero(incircle2d(&a, &b, &c, &a));
    assert_decided_zero(incircle2d(&a, &b, &c, &b));
    assert_decided_zero(incircle2d(&a, &b, &c, &c));

    let prepared_incircle = session.prepare_incircle2(&a, &b, &c);
    assert_eq!(
        session
            .test_prepared_incircle2(&prepared_incircle, &d)
            .value(),
        incircle2d(&a, &b, &c, &d).value(),
        "prepared in-circle path must agree with scalar predicate"
    );
    assert!(
        prepared_incircle
            .coefficient_facts()
            .coefficient_exact
            .all_exact_rational,
        "rational fuzz sites must produce exact rational lifted-circle coefficients"
    );
    assert_eq!(
        prepared_incircle
            .coefficient_facts()
            .coefficient_unknown_zero_count(),
        0,
        "rational lifted-circle coefficients should have decidable zero status"
    );

    let interval = certified_interval_sign(&a.x, &b.x);
    let ax_sign = sign_of_rational(&a.x);
    let bx_sign = sign_of_rational(&b.x);
    if ax_sign == bx_sign {
        assert_eq!(
            interval.and_then(PredicateOutcome::value),
            Some(ax_sign),
            "same-sign rational interval endpoints should certify the interval sign"
        );
    }

    let strict_no_refine = PredicatePolicy {
        allow_refinement: false,
        ..PredicatePolicy::STRICT
    };
    let no_refine = hyperlimit::orient2d_with_policy(&a, &b, &c, strict_no_refine);
    if let Some(sign) = orient2d(&a, &b, &c).value() {
        // For exact rational fuzz inputs, disabling refinement must not change
        // decided orientation signs. This is the Yap EGC contract in executable
        // form: exact rational predicates are not primitive-float filters.
        assert_eq!(no_refine.value(), Some(sign));
    }
}

fn rational(numerator: i16, denominator_byte: u8) -> Real {
    let denominator = u64::from(denominator_byte % 16) + 1;
    Rational::fraction(i64::from(numerator), denominator)
        .expect("positive generated denominator")
        .into()
}

fn assert_decided_zero(outcome: PredicateOutcome<Sign>) {
    if let Some(sign) = outcome.value() {
        assert_eq!(sign, Sign::Zero);
    }
}

fn sign_of_rational(value: &Real) -> Sign {
    match value.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => Sign::Negative,
        Some(hyperreal::RealSign::Zero) => Sign::Zero,
        Some(hyperreal::RealSign::Positive) => Sign::Positive,
        None => unreachable!("fuzz inputs are generated as exact rationals"),
    }
}
