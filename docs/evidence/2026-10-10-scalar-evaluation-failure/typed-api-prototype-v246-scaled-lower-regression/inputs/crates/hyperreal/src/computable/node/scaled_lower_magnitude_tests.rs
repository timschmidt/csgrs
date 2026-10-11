#[cfg(all(test, feature = "serde"))]
mod scaled_lower_magnitude_tests {
    use super::*;
    use crate::Real;

    #[test]
    fn scaled_views_retain_upper_facts_when_a_stronger_exponent_underflows() {
        let mut failures = Vec::new();
        for sign in [-1_i64, 1] {
            for (numerator, denominator, scale_ceiling, loose_delta) in [
                (1, 2, -1, 1),
                (1, 4, -2, 2),
                (3, 8, -1, 1),
                (3, 4, 0, 1),
            ] {
                for exact in [false, true] {
                    // This compact graph is exactly sign * 2^i32::MIN.
                    // Its denotation needs no 2^31-bit materialized integer.
                    let leaf = Computable::rational(Rational::new(sign));
                    let base = Computable {
                        internal: Arc::new(Node::new(
                            Approximation::Offset(leaf, i32::MIN),
                            BoundCache::Invalid,
                            ExactSignCache::Invalid,
                        )),
                        signal: None,
                    };
                    let value: Real = serde_json::from_value(serde_json::json!({
                        "rational": Rational::one(),
                        "class": "Irrational",
                        "computable": base,
                    }))
                    .unwrap();
                    // Folding a unit-scale opaque Real retains its inner node.
                    // The public rational multiplication should share that node.
                    let shared = value.fold_ref();
                    let scale = Rational::fraction(numerator, denominator).unwrap();
                    let scaled = value * Real::new(scale);
                    let known_sign = if sign < 0 { Sign::Minus } else { Sign::Plus };
                    shared.store_bound(&BoundInfo::with_sign_msd(
                        known_sign,
                        Some(i32::MIN + loose_delta),
                        false,
                    ));
                    let before = scaled.structural_facts();
                    assert_eq!(
                        before.magnitude,
                        Some(MagnitudeBits {
                            msd: i32::MIN + loose_delta + scale_ceiling,
                            exact_msd: false,
                        })
                    );
                    // Both incoming certificates are independently valid for
                    // the graph. The second carries a tighter upper bound.
                    shared.store_bound(&BoundInfo::with_sign_msd(
                        known_sign,
                        Some(i32::MIN),
                        exact,
                    ));
                    let after = scaled.structural_facts();
                    assert_eq!(after.sign, before.sign);
                    assert_eq!(after.zero, ZeroKnowledge::NonZero);
                    if !after.magnitude.is_some_and(|bits| {
                        bits.msd <= before.magnitude.unwrap().msd && !bits.exact_msd
                    }) {
                        failures.push(format!("sign={sign}, scale={numerator}/{denominator}, exact={exact}: {before:?} -> {after:?}"));
                    }
                    assert!(shared.cached().is_none());
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
