#[cfg(test)]
mod magnitude_upper_bound_tests {
    use super::*;
    use crate::Real;

    fn raw(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(kind, BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn fixtures() -> Vec<(String, Computable, Rational)> {
        let mut cases = Vec::new();
        for sign in [-1_i64, 1] {
            let mut sum = Computable::rational(Rational::new(2 * sign));
            for _ in 0..96 {
                sum = raw(Approximation::Add(sum, Computable::rational(Rational::new(sign))));
            }
            cases.push((format!("sum96/{sign}"), sum, Rational::new(98 * sign)));
            for bits in [1, 8, 65, 256] {
                let exact = Rational::from_bigint_fraction(BigInt::from(sign), BigUint::one() << bits).unwrap();
                let difference = raw(Approximation::Add(
                    Computable::rational(Rational::new(2 * sign)),
                    raw(Approximation::Negate(Computable::rational(Rational::new(2 * sign) - exact.clone()))),
                ));
                cases.push((format!("cancel/{sign}/{bits}"), difference.clone(), exact.clone()));
                cases.push((format!("inverse_cancel/{sign}/{bits}"), raw(Approximation::Inverse(difference.clone())), exact.clone().inverse().unwrap()));
                cases.push((format!("square_cancel/{sign}/{bits}"), raw(Approximation::Square(difference.clone())), exact.clone() * exact.clone()));
                for offset in [-65_i32, 65] {
                    let scale = if offset < 0 {
                        Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << offset.unsigned_abs()).unwrap()
                    } else {
                        Rational::from_bigint(BigInt::one() << offset.unsigned_abs())
                    };
                    cases.push((format!("offset_cancel/{sign}/{bits}/{offset}"), raw(Approximation::Offset(difference.clone(), offset)), exact.clone() * scale));
                }
            }
        }
        for numerator in [-7, -3, 3, 7] {
            let left = Rational::fraction(numerator, 4).unwrap();
            let leaf = Computable::rational(left.clone());
            cases.push((format!("square/{numerator}"), raw(Approximation::Square(leaf.clone())), left.clone() * left.clone()));
            cases.push((format!("inverse/{numerator}"), raw(Approximation::Inverse(leaf.clone())), left.clone().inverse().unwrap()));
            for (n, d) in [(17, 16), (7, 4), (-3, 2)] {
                let right = Rational::fraction(n, d).unwrap();
                cases.push((format!("product/{numerator}/{n}/{d}"), raw(Approximation::Multiply(leaf.clone(), Computable::rational(right.clone()))), left.clone() * right));
            }
            if numerator > 0 {
                let square = left.clone() * left.clone();
                cases.push((format!("root/{numerator}"), raw(Approximation::Sqrt(Computable::rational(square))), left));
            }
        }
        cases
    }

    fn check_facts(facts: RealStructuralFacts, expected: &Rational, context: &str, failures: &mut Vec<String>) {
        let exponent = expected.msd_exact().unwrap();
        let valid = facts.sign.is_none_or(|sign| sign == public_sign(expected.sign()))
            && facts.zero != ZeroKnowledge::Zero
            && facts.magnitude.is_none_or(|bits| {
                bits.msd >= exponent && (!bits.exact_msd || bits.msd == exponent)
            });
        if !valid {
            failures.push(format!("{context}: exact_msd={exponent}, facts={facts:?}"));
        }
    }

    fn check_strengthening(before: RealStructuralFacts, after: RealStructuralFacts, context: &str, failures: &mut Vec<String>) {
        let valid = before.sign.is_none_or(|sign| after.sign == Some(sign))
            && (before.zero == ZeroKnowledge::Unknown || after.zero == before.zero)
            && before.magnitude.is_none_or(|old| after.magnitude.is_some_and(|new| {
                if old.exact_msd { new == old } else { new.msd <= old.msd }
            }));
        if !valid {
            failures.push(format!("{context}: weakened {before:?} -> {after:?}"));
        }
    }

    #[test]
    fn public_rational_graph_magnitudes_are_upper_bounds_in_every_query_order() {
        let mut failures = Vec::new();
        for inspect_first in [false, true] {
            for (name, value, exact) in fixtures() {
                let mut previous = if inspect_first {
                    let facts = value.structural_facts();
                    check_facts(facts, &exact, &format!("{name}/cold"), &mut failures);
                    // A reciprocal of a cancellation may have only an upper
                    // input magnitude; that cannot bound its output magnitude.
                    if !name.starts_with("inverse_cancel/") {
                        assert!(facts.magnitude.is_some(), "retain the available magnitude certificate: {name}");
                    }
                    assert!(value.cached().is_none());
                    Some(facts)
                } else { None };
                let shared = value.clone();
                for precision in [-4_i32, -64, -256] {
                    let sample = value.approx(precision).unwrap();
                    let scaled = exact.clone() * Rational::from_bigint(BigInt::one() << precision.unsigned_abs());
                    let error = Rational::from_bigint(sample) - scaled;
                    assert!(error >= -Rational::one() && error <= Rational::one(), "{name}, p={precision}: {error:?}");
                    let context = format!("{name}/inspect={inspect_first}/p={precision}");
                    let facts = shared.structural_facts();
                    check_facts(facts, &exact, &context, &mut failures);
                    if let Some(previous) = previous {
                        check_strengthening(previous, facts, &context, &mut failures);
                    }
                    previous = Some(facts);
                }
                #[cfg(feature = "serde")]
                {
                    let decoded: Computable = serde_json::from_str(&serde_json::to_string(&value).unwrap()).unwrap();
                    assert!(decoded.cached().is_none());
                    check_facts(decoded.structural_facts(), &exact, &format!("{name}/decoded"), &mut failures);
                    decoded.approx(-64).unwrap();
                    check_facts(decoded.structural_facts(), &exact, &format!("{name}/decoded/warm"), &mut failures);
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn scaled_real_magnitudes_include_non_dyadic_carries() {
        let mut failures = Vec::new();
        for radicand in [2, 3, 5, 7, 11] {
            for (numerator, denominator) in [(-7, 4), (-3, 2), (-1, 2), (1, 2), (3, 2), (7, 4)] {
                let scale = Rational::fraction(numerator, denominator).unwrap();
                let square = Rational::new(radicand) * scale.clone() * scale.clone();
                // The value's magnitude is half the logarithm of this exact
                // rational square, independently of the Real magnitude path.
                let exponent = square.msd_exact().unwrap().div_euclid(2);
                let value = Real::new(Rational::new(radicand)).sqrt().unwrap() * Real::new(scale.clone());
                for warm in [false, true] {
                    if warm { assert!(value.certified_rational_interval(-64).is_some()); }
                    let facts = value.structural_facts();
                    assert!(facts.magnitude.is_some());
                    if !facts.magnitude.is_some_and(|bits| bits.msd >= exponent && (!bits.exact_msd || bits.msd == exponent)) {
                        failures.push(format!("radicand={radicand}, scale={scale:?}, warm={warm}, exact_msd={exponent}: {facts:?}"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn numeric_magnitude_contains_the_closed_upper_endpoint() {
        let mut centers: Vec<BigInt> = (-17..=17).map(BigInt::from).collect();
        for bits in [63, 64, 65, 127, 128, 129, 1024] {
            let power = BigInt::one() << bits;
            for offset in -2..=2 {
                let center: BigInt = &power + offset;
                centers.push(center.clone());
                centers.push(-center);
            }
        }
        let mut failures = Vec::new();
        for (case, center) in centers.iter().enumerate() {
            for precision in [i32::MIN, -65, 0, 65, i32::MAX] {
                let actual = Computable::bound_from_approx(precision, center);
                if center.abs() <= BigInt::one() { assert_eq!(actual, BoundInfo::Unknown); continue; }
                let upper = i64::from(Rational::from_bigint(center.abs() + 1_u8).msd_exact().unwrap()) + i64::from(precision);
                let valid = match actual.magnitude_bits() {
                    Some(bits) => i64::from(bits.msd) >= upper,
                    None => i32::try_from(upper).is_err(),
                };
                if !valid { failures.push(format!("case={case}, p={precision}, upper_endpoint_msd={upper}: {actual:?}")); }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn publication_preserves_the_tightest_available_upper_bound() {
        let mut failures = Vec::new();
        for exponent in [-2000, -1, 0, 65, 2000] {
            for sign in [Sign::Minus, Sign::Plus] {
                // All four certificates are valid for sign * 2^exponent.
                // Inspect every prefix of every publication order, not just
                // the final state after the exact certificate has arrived.
                let proofs = [
                    BoundInfo::NonZero { sign: Some(sign), msd: Some(exponent + 3), exact_msd: false },
                    BoundInfo::NonZero { sign: None, msd: Some(exponent + 1), exact_msd: false },
                    BoundInfo::NonZero { sign: None, msd: Some(exponent), exact_msd: false },
                    BoundInfo::NonZero { sign: None, msd: Some(exponent), exact_msd: true },
                ];
                for a in 0..4 { for b in 0..4 { for c in 0..4 { for d in 0..4 {
                    let order = [a, b, c, d];
                    if (0..4).any(|i| (0..i).any(|j| order[i] == order[j])) { continue; }
                    let facts = AtomicFacts::default();
                    let mut tightest = i32::MAX;
                    for (step, &index) in order.iter().enumerate() {
                        let proof = proofs[index];
                        tightest = tightest.min(proof.magnitude_bits().unwrap().msd);
                        facts.publish_bound(proof);
                        facts.publish_bound(BoundInfo::Unknown);
                        let BoundCache::Valid(actual) = facts.bound() else { panic!("published bound missing"); };
                        if !actual.magnitude_bits().is_some_and(|bits| bits.msd <= tightest && bits.msd >= exponent) {
                            failures.push(format!("e={exponent}, sign={sign:?}, order={order:?}, step={step}, tightest={tightest}: {actual:?}"));
                        }
                    }
                    assert_eq!(facts.bound(), BoundCache::Valid(BoundInfo::with_sign(sign, Some(exponent))));
                }}}}
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
