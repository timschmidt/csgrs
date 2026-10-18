#[cfg(test)]
mod exact_sign_cache_tests {
    use super::*;
    use std::sync::Barrier;

    #[test]
    fn unresolved_publications_preserve_certified_signs_and_other_facts() {
        for sign in [Sign::Minus, Sign::NoSign, Sign::Plus] {
            let certified = ExactSignCache::Valid(sign);
            let bound = if sign == Sign::NoSign {
                BoundCache::Valid(BoundInfo::Zero)
            } else {
                BoundCache::Valid(BoundInfo::NonZero {
                    sign: Some(sign),
                    msd: Some(-1234567),
                    exact_msd: true,
                })
            };
            for bound in [
                BoundCache::Invalid,
                BoundCache::Valid(BoundInfo::Unknown),
                bound,
            ] {
                for inverse in [false, true] {
                    for demand in [i16::MIN, -512, 0, 512, i16::MAX] {
                        let facts = AtomicFacts::new(bound, certified, inverse, demand);
                        assert_eq!(facts.replace_exact_sign(ExactSignCache::Unknown), certified);
                        assert_eq!(facts.snapshot(), (bound, certified));
                        assert_eq!(facts.linear_demand(), demand);
                        assert_eq!(facts.contains_inverse_trig_or_pi(), Some(inverse));
                    }
                }
            }
        }
    }

    #[test]
    fn delayed_unresolved_writer_cannot_erase_a_completed_proof() {
        for sign in [Sign::Minus, Sign::NoSign, Sign::Plus] {
            let facts = AtomicFacts::new(BoundCache::Invalid, ExactSignCache::Invalid, true, 17);
            let ready = Barrier::new(2);
            let proved = Barrier::new(2);
            let observed = std::thread::scope(|scope| {
                let pending = scope.spawn(|| {
                    let observed = facts.exact_sign();
                    ready.wait();
                    proved.wait();
                    facts.replace_exact_sign(ExactSignCache::Unknown);
                    observed
                });
                ready.wait();
                facts.replace_exact_sign(ExactSignCache::Valid(sign));
                proved.wait();
                pending.join().unwrap()
            });
            assert_eq!(observed, ExactSignCache::Invalid);
            assert_eq!(facts.exact_sign(), ExactSignCache::Valid(sign));
            assert_eq!(facts.linear_demand(), 17);
            assert_eq!(facts.contains_inverse_trig_or_pi(), Some(true));
        }
    }

    #[test]
    fn late_unknown_preserves_a_structurally_proved_quadratic_sign() {
        let value = Computable {
            internal: Arc::new(Node::new(
                Approximation::Add(
                    Computable::sqrt_rational(Rational::new(2)),
                    Computable::rational(Rational::new(-1)),
                ),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        };
        let shared = value.clone();
        assert_eq!(value.immediate_sign(), None);
        assert!(value.cached().is_none());
        // Independently, sqrt(2) > 1 because 2 > 1 and both roots are positive.
        assert_eq!(value.exact_sign(), Some(Sign::Plus));
        assert_eq!(value.immediate_sign(), Some(RealSign::Positive));
        assert!(value.cached().is_none());
        // Publish the same unresolved structural result that an older traversal
        // can have computed before the completed quadratic proof.
        shared
            .internal
            .facts
            .replace_exact_sign(ExactSignCache::Unknown);
        assert_eq!(
            (
                shared.immediate_sign(),
                shared.exact_sign(),
                shared.cached().is_none()
            ),
            (Some(RealSign::Positive), Some(Sign::Plus), true),
        );
        assert_eq!(shared.sign_until(0), Some(RealSign::Positive));
        assert!(shared.cached().is_none());
    }

    #[test]
    fn unresolved_sign_can_still_be_strengthened_without_changing_metadata() {
        for sign in [Sign::Minus, Sign::NoSign, Sign::Plus] {
            let facts = AtomicFacts::new(BoundCache::Invalid, ExactSignCache::Invalid, false, -19);
            assert_eq!(
                facts.replace_exact_sign(ExactSignCache::Unknown),
                ExactSignCache::Invalid
            );
            assert_eq!(facts.exact_sign(), ExactSignCache::Unknown);
            assert_eq!(
                facts.replace_exact_sign(ExactSignCache::Valid(sign)),
                ExactSignCache::Unknown
            );
            assert_eq!(
                facts.snapshot(),
                (BoundCache::Invalid, ExactSignCache::Valid(sign))
            );
            assert_eq!(
                facts.replace_exact_sign(ExactSignCache::Valid(sign)),
                ExactSignCache::Valid(sign)
            );
            assert_eq!(facts.linear_demand(), -19);
            assert_eq!(facts.contains_inverse_trig_or_pi(), Some(false));
        }
    }
}
