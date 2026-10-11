#[cfg(test)]
mod cache_publication_tests {
    use super::*;
    use std::cell::RefCell;
    use std::sync::mpsc;
    use std::time::Duration;

    #[derive(Clone, Copy, PartialEq)]
    enum Publication {
        UnknownSign,
        BoundWithoutMagnitude,
        InexactMagnitude,
    }

    struct PausedPublication {
        target: usize,
        kind: Publication,
        entered: mpsc::SyncSender<()>,
        resume: mpsc::Receiver<()>,
    }

    std::thread_local! {
        static PAUSED_PUBLICATION: RefCell<Option<PausedPublication>> = const { RefCell::new(None) };
    }

    pub(super) fn before_sign_publication(facts: &AtomicFacts, value: ExactSignCache) {
        if value == ExactSignCache::Unknown {
            pause_publication(facts, Publication::UnknownSign);
        }
    }

    pub(super) fn before_bound_publication(facts: &AtomicFacts, value: BoundCache) {
        if matches!(
            value,
            BoundCache::Valid(BoundInfo::NonZero { msd: None, .. })
        ) {
            pause_publication(facts, Publication::BoundWithoutMagnitude);
        } else if matches!(
            value,
            BoundCache::Valid(BoundInfo::NonZero {
                msd: Some(_),
                exact_msd: false,
                ..
            })
        ) {
            pause_publication(facts, Publication::InexactMagnitude);
        }
    }

    fn pause_publication(facts: &AtomicFacts, kind: Publication) {
        let paused = PAUSED_PUBLICATION.with(|slot| {
            let mut slot = slot.borrow_mut();
            if slot.as_ref().is_some_and(|paused| {
                paused.target == std::ptr::from_ref(facts) as usize && paused.kind == kind
            }) {
                slot.take()
            } else {
                None
            }
        });
        if let Some(paused) = paused {
            paused.entered.send(()).unwrap();
            paused.resume.recv_timeout(Duration::from_secs(10)).unwrap();
        }
    }

    fn raw(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                kind,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn exact_remainder(expected: &Rational) -> Computable {
        // (pi + 96) - (pi + 96 - expected) = expected. The left sum exceeds
        // the ordinary 64-level quadratic recognizer, while a larger explicit
        // Laurent proof can certify the rational remainder without evaluating.
        let mut left = Computable::pi();
        for _ in 0..96 {
            left = raw(Approximation::Add(left, Computable::one()));
        }
        let right = raw(Approximation::Add(
            Computable::pi(),
            Computable::rational(Rational::new(96) - expected.clone()),
        ));
        raw(Approximation::Add(left, raw(Approximation::Negate(right))))
    }

    #[test]
    fn delayed_unknown_cannot_erase_a_newer_algebraic_sign() {
        let tiny = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 2_000).unwrap();
        let mut failures = Vec::new();
        for expected in [
            Rational::zero(),
            Rational::fraction(7, 9).unwrap(),
            Rational::fraction(-7, 9).unwrap(),
            tiny.clone(),
            -tiny,
        ] {
            let wanted = Some(public_sign(expected.sign()));
            let cold = exact_remainder(&expected);
            assert_eq!(cold.structural_facts().sign, None);
            assert!(cold.cached().is_none());
            assert_eq!(cold.bounded_laurent_rational(128), Some(expected.clone()));
            assert_eq!(cold.immediate_sign(), wanted);
            assert_eq!(cold.sign_until(0), wanted);
            assert!(cold.cached().is_none());

            let value = exact_remainder(&expected);
            let (entered_tx, entered_rx) = mpsc::sync_channel(0);
            let (resume_tx, resume_rx) = mpsc::sync_channel(0);
            let (proof, before, old_answer) = std::thread::scope(|scope| {
                let value = &value;
                let older = scope.spawn(move || {
                    PAUSED_PUBLICATION.with(|slot| {
                        *slot.borrow_mut() = Some(PausedPublication {
                            target: std::ptr::from_ref(&value.internal.facts) as usize,
                            kind: Publication::UnknownSign,
                            entered: entered_tx,
                            resume: resume_rx,
                        });
                    });
                    value.structural_facts()
                });
                entered_rx.recv_timeout(Duration::from_secs(10)).unwrap();
                let proof = value.bounded_laurent_rational(128);
                let before = value.immediate_sign();
                // Always release the old writer before checking its outcome.
                resume_tx.send(()).unwrap();
                (proof, before, older.join().unwrap())
            });
            assert_eq!(proof, Some(expected.clone()));
            assert_eq!(before, wanted);
            assert!(value.cached().is_none());
            let after = value.immediate_sign();
            let later = value.sign_until(0);
            assert!(
                value.cached().is_none(),
                "the retained proof must avoid numeric refinement"
            );
            if after != wanted || later != wanted {
                failures.push(format!(
                    "expected={expected:?}: before={before:?}, old_answer={old_answer:?}, retained={after:?}, later={later:?}"
                ));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    fn signed_half_function(case: usize, sign: i64) -> (Computable, i32) {
        let half = Rational::fraction(sign, 2).unwrap();
        let (kind, msd) = match case {
            0 => (Approximation::AtanRational(half), -2),
            1 => (Approximation::AsinhRational(half), -2),
            2 => (Approximation::AsinRational(half), -1),
            3 => (Approximation::AtanhRational(half), -1),
            _ => unreachable!(),
        };
        // Integral bounds independently give 1/4 < atan(1/2), asinh(1/2)
        // < 1/2, and 1/2 < asin(1/2), atanh(1/2) < 1. All four are odd.
        (raw(kind), msd)
    }

    #[test]
    fn exact_sign_reuses_retained_bound_certificates() {
        let tiny = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 2_000).unwrap();
        let mut failures = Vec::new();
        for expected in [
            Rational::zero(),
            Rational::fraction(7, 9).unwrap(),
            Rational::fraction(-7, 9).unwrap(),
            tiny.clone(),
            -tiny,
        ] {
            let value = exact_remainder(&expected);
            assert_eq!(value.exact_sign(), None);
            // The exact cancellation identity in exact_remainder independently
            // proves this bound, without any numeric evaluation of the graph.
            value.store_bound(&BoundInfo::from_rational(&expected));
            assert_eq!(value.immediate_sign(), Some(public_sign(expected.sign())));
            let actual = value.exact_sign();
            if actual != Some(expected.sign()) {
                failures.push(format!(
                    "expected={expected:?}, bound={:?}, exact_sign={actual:?}",
                    value.cached_bound()
                ));
            }
            assert!(value.cached().is_none());
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn delayed_bound_cannot_erase_an_independent_exact_magnitude_proof() {
        let mut failures = Vec::new();
        for case in 0..4 {
            for sign in [-1_i64, 1] {
                let (value, msd) = signed_half_function(case, sign);
                let wanted_sign = if sign < 0 { Sign::Minus } else { Sign::Plus };
                let (entered_tx, entered_rx) = mpsc::sync_channel(0);
                let (resume_tx, resume_rx) = mpsc::sync_channel(0);
                let (before, old_answer) = std::thread::scope(|scope| {
                    let value = &value;
                    let older = scope.spawn(move || {
                        PAUSED_PUBLICATION.with(|slot| {
                            *slot.borrow_mut() = Some(PausedPublication {
                                target: std::ptr::from_ref(&value.internal.facts) as usize,
                                kind: Publication::BoundWithoutMagnitude,
                                entered: entered_tx,
                                resume: resume_rx,
                            });
                        });
                        value.structural_facts()
                    });
                    entered_rx.recv_timeout(Duration::from_secs(10)).unwrap();
                    value.ensure_finite_real(&None).unwrap();
                    // Publish the independent integral-bound certificate above
                    // through the same path used by structural proof producers.
                    value.store_bound(&BoundInfo::with_sign(wanted_sign, Some(msd)));
                    let before = value.structural_facts();
                    resume_tx.send(()).unwrap();
                    (before, older.join().unwrap())
                });
                assert_eq!(
                    before.magnitude,
                    Some(MagnitudeBits {
                        msd,
                        exact_msd: true
                    })
                );
                let after = value.structural_facts();
                if after != before {
                    failures.push(format!("case={case}, sign={sign}: before={before:?}, old_answer={old_answer:?}, after={after:?}"));
                }
                assert!(value.internal.facts.finite_real());
                assert!(value.cached().is_none());
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn delayed_loose_bound_preserves_a_newer_tighter_upper_bound() {
        for case in 0..4 {
            for sign in [-1_i64, 1] {
                let (value, msd) = signed_half_function(case, sign);
                let wanted_sign = if sign < 0 { Sign::Minus } else { Sign::Plus };
                let (entered_tx, entered_rx) = mpsc::sync_channel(0);
                let (resume_tx, resume_rx) = mpsc::sync_channel(0);
                let before = std::thread::scope(|scope| {
                    let value = &value;
                    let older = scope.spawn(move || {
                        PAUSED_PUBLICATION.with(|slot| {
                            *slot.borrow_mut() = Some(PausedPublication {
                                target: std::ptr::from_ref(&value.internal.facts) as usize,
                                kind: Publication::InexactMagnitude,
                                entered: entered_tx,
                                resume: resume_rx,
                            });
                        });
                        value.store_bound(&BoundInfo::NonZero {
                            sign: None,
                            msd: Some(msd + 3),
                            exact_msd: false,
                        });
                    });
                    entered_rx.recv_timeout(Duration::from_secs(10)).unwrap();
                    value.ensure_finite_real(&None).unwrap();
                    // Both upper bounds follow from the independent integral
                    // bounds in signed_half_function. Only the later writer
                    // supplies the tighter bound and the sign certificate.
                    value.store_bound(&BoundInfo::with_sign_msd(
                        wanted_sign,
                        Some(msd + 1),
                        false,
                    ));
                    let before = value.structural_facts();
                    resume_tx.send(()).unwrap();
                    older.join().unwrap();
                    before
                });
                assert_eq!(before.sign, Some(public_sign(wanted_sign)));
                assert_eq!(before.zero, ZeroKnowledge::NonZero);
                assert_eq!(
                    before.magnitude,
                    Some(MagnitudeBits {
                        msd: msd + 1,
                        exact_msd: false,
                    })
                );
                assert_eq!(value.structural_facts(), before);
                assert!(value.internal.facts.finite_real());
                assert!(value.cached().is_none());
            }
        }
    }

    #[test]
    fn delayed_coarse_evaluation_preserves_a_newer_exact_magnitude_and_sample() {
        for case in 0..2 {
            for sign in [-1_i64, 1] {
                let (value, msd) = signed_half_function(case, sign);
                let (entered_tx, entered_rx) = mpsc::sync_channel(0);
                let (resume_tx, resume_rx) = mpsc::sync_channel(0);
                let (fine, before, coarse) = std::thread::scope(|scope| {
                    let value = &value;
                    let older = scope.spawn(move || {
                        PAUSED_PUBLICATION.with(|slot| {
                            *slot.borrow_mut() = Some(PausedPublication {
                                target: std::ptr::from_ref(&value.internal.facts) as usize,
                                kind: Publication::InexactMagnitude,
                                entered: entered_tx,
                                resume: resume_rx,
                            });
                        });
                        // Integral bounds put both magnitudes strictly between
                        // 7/16 and 1/2. Every legal one-unit sample is 7 or 8,
                        // so its closed error interval crosses an MSD boundary.
                        value.approx(-4)
                    });
                    entered_rx.recv_timeout(Duration::from_secs(10)).unwrap();
                    let fine = value.approx(-128);
                    let before = value.structural_facts();
                    resume_tx.send(()).unwrap();
                    (fine.unwrap(), before, older.join().unwrap().unwrap())
                });
                assert!([BigInt::from(7), BigInt::from(8)].contains(&coarse.abs()));
                assert_eq!(
                    coarse.sign(),
                    if sign < 0 { Sign::Minus } else { Sign::Plus }
                );
                assert_eq!(
                    before.magnitude,
                    Some(MagnitudeBits {
                        msd,
                        exact_msd: true
                    })
                );
                assert_eq!(value.structural_facts(), before);
                assert_eq!(value.internal.cache_snapshot(), Some((-128, fine)));
            }
        }
    }

    #[test]
    fn numeric_magnitude_combines_with_a_structural_sign_certificate() {
        let mut failures = Vec::new();
        for case in 0..4 {
            for sign in [-1_i64, 1] {
                for precision in [-16, -128] {
                    for inspect_first in [false, true] {
                        let (value, msd) = signed_half_function(case, sign);
                        let wanted_sign = if sign < 0 {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        };
                        if inspect_first {
                            let before = value.structural_facts();
                            assert_eq!(before.sign, Some(wanted_sign));
                            assert_eq!(before.magnitude, None);
                        }
                        value.approx(precision).unwrap();
                        let facts = value.structural_facts();
                        if facts.sign != Some(wanted_sign)
                            || facts.zero != ZeroKnowledge::NonZero
                            || facts.magnitude
                                != Some(MagnitudeBits {
                                    msd,
                                    exact_msd: true,
                                })
                        {
                            failures.push(format!("case={case}, sign={sign}, p={precision}, inspect_first={inspect_first}: {facts:?}"));
                        }
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn numeric_magnitude_exactness_respects_closed_error_interval_endpoints() {
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
                if center.abs() <= BigInt::one() {
                    assert_eq!(actual, BoundInfo::Unknown);
                    continue;
                }
                // Compute both endpoint exponents through exact rational
                // arithmetic, independently of the binary pattern shortcut.
                // Add in i64 so extreme precisions need no huge allocation.
                let lower = Rational::from_bigint(center.abs() - 1_u8)
                    .msd_exact()
                    .unwrap();
                let upper = Rational::from_bigint(center.abs() + 1_u8)
                    .msd_exact()
                    .unwrap();
                let expected = (lower == upper)
                    .then(|| i32::try_from(i64::from(lower) + i64::from(precision)).ok())
                    .flatten();
                let valid = match actual {
                    BoundInfo::NonZero {
                        sign,
                        msd,
                        exact_msd,
                    } => {
                        sign == Some(center.sign())
                            && exact_msd == expected.is_some()
                            && expected.is_none_or(|expected| msd == Some(expected))
                    }
                    _ => false,
                };
                if !valid {
                    failures.push(format!(
                        "case={case}, p={precision}, expected_exact_msd={expected:?}: {actual:?}"
                    ));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
