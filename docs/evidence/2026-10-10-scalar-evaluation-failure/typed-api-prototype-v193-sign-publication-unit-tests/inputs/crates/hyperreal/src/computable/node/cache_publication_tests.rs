#[cfg(test)]
mod cache_publication_tests {
    use super::*;
    use std::cell::RefCell;
    use std::sync::mpsc;
    use std::time::Duration;

    struct PausedSign {
        target: usize,
        entered: mpsc::SyncSender<()>,
        resume: mpsc::Receiver<()>,
    }

    std::thread_local! {
        static PAUSED_SIGN: RefCell<Option<PausedSign>> = const { RefCell::new(None) };
    }

    pub(super) fn before_sign_publication(facts: &AtomicFacts, value: ExactSignCache) {
        let paused = PAUSED_SIGN.with(|slot| {
            let mut slot = slot.borrow_mut();
            if value == ExactSignCache::Unknown
                && slot.as_ref().is_some_and(|paused| {
                    paused.target == std::ptr::from_ref(facts) as usize
                })
            {
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
            internal: Arc::new(Node::new(kind, BoundCache::Invalid, ExactSignCache::Invalid)),
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
            Rational::zero(), Rational::fraction(7, 9).unwrap(),
            Rational::fraction(-7, 9).unwrap(), tiny.clone(), -tiny,
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
                    PAUSED_SIGN.with(|slot| {
                        *slot.borrow_mut() = Some(PausedSign {
                            target: std::ptr::from_ref(&value.internal.facts) as usize,
                            entered: entered_tx, resume: resume_rx,
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
            assert!(value.cached().is_none(), "the retained proof must avoid numeric refinement");
            if after != wanted || later != wanted {
                failures.push(format!(
                    "expected={expected:?}: before={before:?}, old_answer={old_answer:?}, retained={after:?}, later={later:?}"
                ));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
