//! FOC adapter conformance: complete native results/errors and independent integer enclosures.
//!
//! Q abbreviates Alumina's ordered nonempty Q30 interval. Transform corner checks
//! prove containment of the specified multi-affine expression on each fixture box,
//! not tightness, correlation preservation or correctness of every native failure.
//! Primitive product/division checks additionally compare exact rounded endpoints.
//! Native and shared paths can use the same scalar kernel; their agreement alone
//! is not the independent oracle. Fixed boundary and seeded corpora are finite.
use alumina_foc::{AlphaBeta, DqInterval, FocError, Phase3, Q30, Q30Interval as Q, Rotation};
use interval_foc_conformance as shared;
/// Raw Q30 numerator of one, widened so corner expressions and quotient comparisons fit.
const SCALE: i128 = 1 << 30;

/// Construct an ordered fixture from exact raw numerators; reversed bounds panic.
fn q(a: i32, b: i32) -> Q {
    Q::new(Q30::from_bits(a), Q30::from_bits(b)).unwrap()
}

/// Sign-extend both raw endpoints for exact integer oracle arithmetic without rounding.
fn endpoints(value: Q) -> [i128; 2] {
    [value.lower().bits().into(), value.upper().bits().into()]
}

/// Assert containment of a rational number measured in raw Q30 grid units.
/// Callers supply a positive denominator and cross-products that fit i128, so
/// multiplication preserves inequality direction and no floating division is needed.
fn encloses(value: Q, numerator: i128, denominator: i128) {
    let [lo, hi] = endpoints(value);
    assert!(
        lo * denominator <= numerator && numerator <= hi * denominator,
        "{value:?} misses {numerator}/{denominator} lattice units"
    );
}

/// Enclose all sixteen Cartesian corners of the ideal Park expressions before Q30 rounding.
/// Exact integer formulas check d and q independently. Multi-affinity makes corners
/// sufficient for the box extrema, including component combinations off the unit circle;
/// this checks conservative containment, not the tightest dependency-aware rotation image.
fn park_oracle(value: AlphaBeta, r: Rotation, output: DqInterval) {
    for a in endpoints(value.alpha) {
        for b in endpoints(value.beta) {
            for s in endpoints(r.sine()) {
                for c in endpoints(r.cosine()) {
                    encloses(output.d, a * c + b * s, SCALE);
                    encloses(output.q, b * c - a * s, SCALE);
                }
            }
        }
    }
}

/// Check inverse Park's two ideal multi-affine expressions at all sixteen component corners.
/// The independent i128 formulas precede rounding; returned enclosures may be wider.
fn inverse_park_oracle(value: DqInterval, r: Rotation, output: AlphaBeta) {
    for d in endpoints(value.d) {
        for q in endpoints(value.q) {
            for s in endpoints(r.sine()) {
                for c in endpoints(r.cosine()) {
                    encloses(output.alpha, d * c - q * s, SCALE);
                    encloses(output.beta, d * s + q * c, SCALE);
                }
            }
        }
    }
}

/// Check Clarke's eight phase corners, with both authored radical-bracket endpoints for beta.
/// Alpha uses exact division by three and beta uses a raw coefficient over SCALE.
/// This does not decide balance acceptance; a separate regression validates the radical bracket.
fn clarke_oracle(value: Phase3, output: AlphaBeta) {
    for a in endpoints(value.a) {
        for b in endpoints(value.b) {
            for c in endpoints(value.c) {
                encloses(output.alpha, 2 * a - b - c, 3);
                for k in endpoints(alumina_foc::INV_SQRT_3) {
                    encloses(output.beta, (b - c) * k, SCALE);
                }
            }
        }
    }
}

/// Require exact copying of alpha to phase a and containment of both inverse-Clarke expressions.
/// Enumerate alpha, beta and radical-bracket corners with denominator 2*SCALE;
/// returned b/c may include extra width from their separately rounded intermediate products.
fn inverse_clarke_oracle(value: AlphaBeta, output: Phase3) {
    assert_eq!(output.a, value.alpha);
    for a in endpoints(value.alpha) {
        for b in endpoints(value.beta) {
            for k in endpoints(alumina_foc::SQRT_3_OVER_2) {
                encloses(output.b, -a * SCALE + 2 * b * k, 2 * SCALE);
                encloses(output.c, -a * SCALE - 2 * b * k, 2 * SCALE);
            }
        }
    }
}

/// Enumerate 45 ordered intervals from nine selected extreme, unit and near-zero numerators.
/// Includes point and zero-crossing intervals; this is not all possible i32 endpoint pairs.
fn intervals() -> Vec<Q> {
    let endpoints = [
        i32::MIN,
        i32::MIN + 1,
        -(1 << 30),
        -1,
        0,
        1,
        1 << 30,
        (1 << 30) + 1,
        i32::MAX,
    ];
    endpoints
        .iter()
        .enumerate()
        .flat_map(|(i, &a)| endpoints[i..].iter().map(move |&b| q(a, b)))
        .collect()
}

/// Build eleven validated rotation fixtures: five sample choices and six broad component boxes.
/// The latter pair a signed half/full unit interval with zero and allow norm error one.
/// Their norm enclosures satisfy Rotation's contract; not every enclosed component pair
/// is itself a unit vector. Native validation is reused, not independently reimplemented here.
fn rotations() -> Vec<Rotation> {
    let mut result: Vec<_> = (0..5)
        .map(|i| shared::samples::sample(i).rotation)
        .collect();
    // Continuous rotation families, admitted by the existing norm contract.
    for interval in [q(-(1 << 30), 0), q(0, 1 << 30), q(-(1 << 30), 1 << 30)] {
        result.push(Rotation::new(interval, Q::ZERO, Q30::ONE).unwrap());
        result.push(Rotation::new(Q::ZERO, interval, Q30::ONE).unwrap());
    }
    result
}

/// Count matched outcomes so a comparison corpus cannot silently contain only failures or successes.
#[derive(Default, Debug)]
struct Outcomes {
    /// Fully equal successful results, subsequently available for independent checks.
    success: usize,
    /// Matched representability failures, distinct from completed geometry.
    range: usize,
    /// Matched phase-balance rejections, distinct from arithmetic overflow.
    unbalanced: usize,
}
/// Exact result comparison followed by outcome accounting; unexpected errors fail the test.
impl Outcomes {
    /// Assert full Result equality, count its category, and return only successful values.
    /// Range/UnbalancedPhases return None after accounting; other errors panic rather
    /// than becoming accepted exclusions. Native failure agreement is not an independent proof.
    fn check<T: std::fmt::Debug + PartialEq>(
        &mut self,
        actual: Result<T, FocError>,
        expected: Result<T, FocError>,
    ) -> Option<T> {
        assert_eq!(actual, expected);
        match actual {
            Ok(value) => {
                self.success += 1;
                Some(value)
            }
            Err(FocError::Range) => {
                self.range += 1;
                None
            }
            Err(FocError::UnbalancedPhases) => {
                self.unbalanced += 1;
                None
            }
            Err(error) => panic!("unexpected {error:?}"),
        }
    }
}

#[test]
/// Compare all 45-by-45 source boxes across eleven rotations and check successful corners.
/// Forward and inverse Park each have 22,275 cases; inverse Clarke has 2,025.
/// Require both success and Range populations without claiming exhaustive i32-domain coverage.
fn full_endpoint_transform_matrix_matches_outputs_errors_and_exact_corners() {
    let intervals = intervals();
    let mut park = Outcomes::default();
    let mut inverse = Outcomes::default();
    let mut inverse_clarke = Outcomes::default();
    for &a in &intervals {
        for &b in &intervals {
            let value = AlphaBeta { alpha: a, beta: b };
            if let Some(output) = inverse_clarke.check(
                shared::inverse_clarke(value),
                alumina_foc::inverse_clarke(value),
            ) {
                inverse_clarke_oracle(value, output);
            }
            for r in rotations() {
                if let Some(output) =
                    park.check(shared::park(value, r), alumina_foc::park(value, r))
                {
                    park_oracle(value, r, output);
                }
                let value = DqInterval { d: a, q: b };
                if let Some(output) = inverse.check(
                    shared::inverse_park(value, r),
                    alumina_foc::inverse_park(value, r),
                ) {
                    inverse_park_oracle(value, r, output);
                }
            }
        }
    }
    assert!(park.success > 0 && park.range > 0 && inverse.success > 0 && inverse.range > 0);
    assert!(inverse_clarke.success > 0 && inverse_clarke.range > 0);
    println!("park={park:?}; inverse_park={inverse:?}; inverse_clarke={inverse_clarke:?}");
}

#[test]
/// Compare 273,375 Clarke cases: three selected balance bounds for every triple of 45 intervals.
/// Negative, zero and maximum bounds exercise balance priority and wide numerators.
/// Check independent corners only on success and require success, Range and unbalanced outcomes.
fn clarke_full_endpoint_matrix_preserves_balance_priority_and_widening() {
    let intervals = intervals();
    let mut stats = Outcomes::default();
    for &a in &intervals {
        for &b in &intervals {
            for &c in &intervals {
                let value = Phase3 { a, b, c };
                for bound in [Q30::from_bits(-1), Q30::ZERO, Q30::from_bits(i32::MAX)] {
                    if let Some(output) = stats.check(
                        shared::clarke(value, bound),
                        alumina_foc::clarke(value, bound),
                    ) {
                        clarke_oracle(value, output);
                    }
                }
            }
        }
    }
    assert!(stats.success > 0 && stats.range > 0 && stats.unbalanced > 0);
    println!("clarke={stats:?}");
}

#[test]
/// Retain three hard cases where a plausible matrix rewrite changes the completed contract.
/// A signed dense Park matrix bypasses an existing MIN-negation failure; pre-rounded
/// Clarke coefficients widen an exact alpha; wide b-c can exceed i32 while beta fits.
/// Full expected results and integer square inequalities also validate both radical brackets.
fn original_hard_cases_preclude_dense_or_prerounded_rewrites() {
    use hyperlattice_interval::{IntervalMatrix, IntervalVector, hyperinterval::FixedInterval};
    /// Native Q30 grid used to exhibit the behavior of the alternative dense expressions.
    type I = FixedInterval<i32, 30>;
    let p = |x| I::point(x).unwrap();
    let r = Rotation::new(Q::point(Q30::NEG_ONE), Q::ZERO, Q30::ZERO).unwrap();
    let value = AlphaBeta {
        alpha: q(i32::MIN, i32::MIN),
        beta: Q::ZERO,
    };
    assert_eq!(shared::park(value, r), Err(FocError::Range));
    assert_eq!(alumina_foc::park(value, r), Err(FocError::Range));
    let dense = IntervalMatrix::new([[p(0), p(-(1 << 30))], [p(1 << 30), p(0)]])
        .unwrap()
        .mul_vector(&IntervalVector::new([p(i32::MIN), p(0)]).unwrap(), &mut ())
        .unwrap();
    assert_eq!(dense.lanes().unwrap(), &[p(0), p(i32::MIN)]);
    let phases = Phase3 {
        a: Q::ONE,
        b: Q::ZERO,
        c: Q::point(Q30::NEG_ONE),
    };
    let actual = shared::clarke(phases, Q30::ZERO).unwrap();
    assert_eq!(actual.alpha, Q::ONE);
    let dense = IntervalMatrix::new([[
        I::from_ratio(2, 3).unwrap(),
        I::from_ratio(-1, 3).unwrap(),
        I::from_ratio(-1, 3).unwrap(),
    ]])
    .unwrap()
    .mul_vector(
        &IntervalVector::new([p(1 << 30), p(0), p(-(1 << 30))]).unwrap(),
        &mut (),
    )
    .unwrap();
    assert_eq!(
        dense.lanes().unwrap()[0].bounds(),
        Some(((1 << 30) - 1, (1 << 30) + 1))
    );
    // b-c exceeds i32 while the scaled beta is representable.
    let phases = Phase3 {
        a: Q::ZERO,
        b: q(5 << 28, 5 << 28),
        c: q(-(5 << 28), -(5 << 28)),
    };
    let actual = shared::clarke(phases, Q30::ZERO).unwrap();
    assert_eq!(actual, alumina_foc::clarke(phases, Q30::ZERO).unwrap());
    assert_eq!(actual.beta, q(1_549_812_827, 1_549_812_830));
    clarke_oracle(phases, actual);
    // Validate the authored radical enclosures independently in integers.
    let [lo, hi] = endpoints(alumina_foc::INV_SQRT_3);
    assert!(3 * lo * lo < SCALE * SCALE && 3 * hi * hi > SCALE * SCALE);
    let [lo, hi] = endpoints(alumina_foc::SQRT_3_OVER_2);
    assert!(4 * lo * lo < 3 * SCALE * SCALE && 4 * hi * hi > 3 * SCALE * SCALE);
}

#[test]
/// Consume five transforms for each of the first 64 samples, comparing every intermediate.
/// Require inverse-Clarke phase lanes to contain the source phase lanes. The final
/// Clarke uses the wider residual tolerance; this is fixture containment, not exact inversion.
fn completed_chains_keep_each_intermediate_and_contain_source() {
    for index in 0..64 {
        let sample = shared::samples::sample(index);
        let a = shared::clarke(sample.phases, shared::samples::BALANCE).unwrap();
        assert_eq!(
            a,
            alumina_foc::clarke(sample.phases, shared::samples::BALANCE).unwrap()
        );
        let dq = shared::park(a, sample.rotation).unwrap();
        assert_eq!(dq, alumina_foc::park(a, sample.rotation).unwrap());
        let b = shared::inverse_park(dq, sample.rotation).unwrap();
        assert_eq!(b, alumina_foc::inverse_park(dq, sample.rotation).unwrap());
        let phases = shared::inverse_clarke(b).unwrap();
        assert_eq!(phases, alumina_foc::inverse_clarke(b).unwrap());
        let again = shared::clarke(phases, Q30::from_bits(1 << 20)).unwrap();
        assert_eq!(
            again,
            alumina_foc::clarke(phases, Q30::from_bits(1 << 20)).unwrap()
        );
        for (source, returned) in [
            (sample.phases.a, phases.a),
            (sample.phases.b, phases.b),
            (sample.phases.c, phases.c),
        ] {
            assert!(
                returned.lower().bits() <= source.lower().bits()
                    && returned.upper().bits() >= source.upper().bits()
            );
        }
    }
}

/// Advance a u64 xorshift twice, truncate each draw to signed i32, and order the two endpoints.
/// This deterministic full-width sampler has no rejection loop; it is neither exhaustive
/// nor a uniform sampler of intervals. Zero state remains zero and produces a point fixture.
fn generated(state: &mut u64) -> Q {
    let mut next = || {
        *state ^= *state << 13;
        *state ^= *state >> 7;
        *state ^= *state << 17;
        *state as i32
    };
    let a = next();
    let b = next();
    q(a.min(b), a.max(b))
}

#[test]
/// Run 20,000 seeded interval triples through four transforms and the successful-result oracles.
/// Cycle the eleven validated rotations, use maximum balance tolerance for Clarke,
/// and require all three outcome categories among the 80,000 comparisons.
fn seeded_full_width_intervals_match_native_and_exact_corners() {
    let mut state = 20_261_112_u64;
    let rotations = rotations();
    let mut stats = Outcomes::default();
    for index in 0..20_000 {
        let a = generated(&mut state);
        let b = generated(&mut state);
        let c = generated(&mut state);
        let r = rotations[index % rotations.len()];
        let value = AlphaBeta { alpha: a, beta: b };
        if let Some(out) = stats.check(shared::park(value, r), alumina_foc::park(value, r)) {
            park_oracle(value, r, out);
        }
        let dq = DqInterval { d: a, q: b };
        if let Some(out) = stats.check(
            shared::inverse_park(dq, r),
            alumina_foc::inverse_park(dq, r),
        ) {
            inverse_park_oracle(dq, r, out);
        }
        if let Some(out) = stats.check(
            shared::inverse_clarke(value),
            alumina_foc::inverse_clarke(value),
        ) {
            inverse_clarke_oracle(value, out);
        }
        let phases = Phase3 { a, b, c };
        let bound = Q30::from_bits(i32::MAX);
        if let Some(out) = stats.check(
            shared::clarke(phases, bound),
            alumina_foc::clarke(phases, bound),
        ) {
            clarke_oracle(phases, out);
        }
    }
    assert!(stats.success > 0 && stats.range > 0 && stats.unbalanced > 0);
    println!("seed=20261112; random_transforms={stats:?}");
}

#[test]
/// Compare tight rounded product/quotient hulls for 2,025 boundary and 10,000 seeded pairs.
/// Native, narrow fixed and wide-then-narrow fixed paths must match exact endpoints
/// or the same independently derived Range/DivideByZero failure, not merely contain corners.
fn primitive_and_widened_products_divisions_match_exact_rational_corners() {
    use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
    /// Signed i32 Q30 carrier, matching the domain wrapper's raw endpoint representation.
    type I = FixedInterval<i32, 30>;
    /// Rewrap nonempty fixed results and map only Overflow/ContainsZero to expected FOC errors.
    /// Empty success or any other scalar error violates this test's domain and panics.
    fn fixed_result(result: Result<I, IntervalError>) -> Result<Q, FocError> {
        match result {
            Ok(i) => {
                let (a, b) = i.bounds().unwrap();
                Ok(q(a, b))
            }
            Err(IntervalError::Overflow) => Err(FocError::Range),
            Err(IntervalError::ContainsZero) => Err(FocError::DivideByZero),
            Err(e) => panic!("unexpected {e:?}"),
        }
    }
    /// Floor the lower rational and ceil the upper rational, then check both raw endpoints fit i32.
    /// Callers provide ordered bounds with positive denominators and arithmetic fitting i128.
    fn rounded(lo: (i128, i128), hi: (i128, i128)) -> Result<Q, FocError> {
        let lower = lo.0.div_euclid(lo.1);
        let upper = hi.0.div_euclid(hi.1) + i128::from(hi.0.rem_euclid(hi.1) != 0);
        Ok(q(
            i32::try_from(lower).map_err(|_| FocError::Range)?,
            i32::try_from(upper).map_err(|_| FocError::Range)?,
        ))
    }
    /// Derive product extrema and sign-normalized quotient extrema independently in i128.
    /// A zero-containing divisor predicts DivideByZero before quotient construction.
    /// Otherwise sort four rational corners by cross-products, outward-round, and
    /// compare all three implementations including narrowing after wide arithmetic.
    fn check(a: Q, b: Q) {
        let i = I::new(a.lower().bits(), a.upper().bits()).unwrap();
        let j = I::new(b.lower().bits(), b.upper().bits()).unwrap();
        let wide_i = i.convert::<i64>().unwrap();
        let wide_j = j.convert::<i64>().unwrap();
        let products: Vec<_> = endpoints(a)
            .into_iter()
            .flat_map(|x| endpoints(b).map(|y| x * y))
            .collect();
        let exact = rounded(
            (*products.iter().min().unwrap(), SCALE),
            (*products.iter().max().unwrap(), SCALE),
        );
        assert_eq!(fixed_result(i.checked_mul(&j)), exact);
        assert_eq!(a.checked_mul(b), exact);
        assert_eq!(
            fixed_result(wide_i.checked_mul(&wide_j).and_then(|v| v.convert::<i32>())),
            exact
        );
        let exact = if b.contains(Q30::ZERO) {
            Err(FocError::DivideByZero)
        } else {
            let mut quotients: Vec<_> = endpoints(a)
                .into_iter()
                .flat_map(|x| endpoints(b).map(move |y| (x * SCALE * y.signum(), y.abs())))
                .collect();
            quotients.sort_by(|a, b| (a.0 * b.1).cmp(&(b.0 * a.1)));
            rounded(quotients[0], quotients[3])
        };
        assert_eq!(fixed_result(i.checked_div(&j)), exact);
        assert_eq!(a.checked_div(b), exact);
        assert_eq!(
            fixed_result(wide_i.checked_div(&wide_j).and_then(|v| v.convert::<i32>())),
            exact
        );
    }
    let values = intervals();
    for &a in &values {
        for &b in &values {
            check(a, b);
        }
    }
    let mut state = 20_261_112_u64;
    for _ in 0..10_000 {
        let a = generated(&mut state);
        let b = generated(&mut state);
        check(a, b);
    }
    println!(
        "primitive_pairs=12025; operations=product,division; narrow_and_wide=true; seed=20261112"
    );
}
