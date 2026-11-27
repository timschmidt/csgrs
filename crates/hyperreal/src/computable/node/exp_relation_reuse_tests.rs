#[cfg(test)]
/// Cache lifetime, collision, fallback and proof-outcome witnesses for exponential atom comparisons.
mod exp_relation_reuse_tests {
    use super::*;

    /// Wrap raw syntax with empty facts and no signal, preserving syntactic fixture shape without domain validation.
    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Build a chosen-depth chain of raw PrescaledSin tags above an integer leaf.
    /// These are structural fixtures and may violate reduced-kernel argument ranges;
    /// tests using this helper do not evaluate them as numerical approximations.
    fn atom(n: i64, depth: usize) -> Computable {
        let mut x = raw(Approximation::Int(n.into()));
        for _ in 0..depth {
            x = raw(Approximation::PrescaledSin(x));
        }
        x
    }

    /// Replace this thread's comparison cache with an empty instance, releasing its weak keys.
    /// Unlike production best-effort access, fixture setup expects available and unborrowed TLS.
    fn reset() {
        EXP_ATOM_CACHE.with(|c| *c.borrow_mut() = ExpAtomCache::new());
    }

    #[test]
    /// Compare equal and unequal independent depth-128 fixtures, then repeat both argument orders one hundred times.
    /// Require expected recognizer outcomes and one retained weak key per operand.
    /// Stable weak counts are not a measurement of comparison-call counts or runtime.
    fn true_false_and_symmetric_hits_do_not_add_weak_references() {
        reset();
        for (left, right, expected) in [(2, 2, true), (2, 3, false)] {
            let a = atom(left, 128);
            let b = atom(right, 128);
            assert_eq!(exp_atom_structural_eq(&a, &b), expected);
            assert_eq!(Arc::weak_count(&a.internal), 1);
            assert_eq!(Arc::weak_count(&b.internal), 1);
            for _ in 0..100 {
                assert_eq!(exp_atom_structural_eq(&b, &a), expected);
                assert_eq!(exp_atom_structural_eq(&a, &b), expected);
            }
            assert_eq!(Arc::weak_count(&a.internal), 1);
            assert_eq!(Arc::weak_count(&b.internal), 1);
        }
        reset();
    }

    #[test]
    /// Compare a depth-128 fixture with itself and require no weak key after the pointer fast path.
    fn pointer_identity_never_populates_cache() {
        reset();
        let a = atom(2, 128);
        assert!(exp_atom_structural_eq(&a, &a));
        assert_eq!(Arc::weak_count(&a.internal), 0);
    }

    #[test]
    /// Traverse 512 distinct pairs three times through sixteen slots, checking alternating match/non-match outcomes.
    /// Bound occupied slots and total weak counts, then reset and require every key
    /// released. The fixture forces replacement without prescribing address hashes
    /// or proving a retained-byte or throughput bound.
    fn collisions_and_eviction_preserve_full_comparison_outcomes() {
        reset();
        let pairs: Vec<_> = (0..512).map(|i| (atom(i, 1), atom(i + i % 2, 1))).collect();
        for _ in 0..3 {
            for (i, (a, b)) in pairs.iter().enumerate() {
                assert_eq!(exp_atom_structural_eq(a, b), i % 2 == 0);
            }
            EXP_ATOM_CACHE.with(|c| {
                assert!(c.borrow().entries.iter().flatten().count() <= EXP_ATOM_CACHE_SIZE);
            });
            assert!(
                pairs
                    .iter()
                    .map(|(a, b)| Arc::weak_count(&a.internal) + Arc::weak_count(&b.internal))
                    .sum::<usize>()
                    <= 2 * EXP_ATOM_CACHE_SIZE
            );
        }
        reset();
        assert!(
            pairs.iter().all(
                |(a, b)| Arc::weak_count(&a.internal) == 0 && Arc::weak_count(&b.internal) == 0
            )
        );
    }

    #[test]
    /// Cache a positive comparison, drop both owners and require root and child weak upgrades to fail.
    /// The retained weak key may still hold allocation storage; this test proves node
    /// values and a child graph are not strongly retained, not immediate deallocation of all bytes.
    fn cache_does_not_keep_nodes_or_child_graphs_alive() {
        reset();
        let a = atom(2, 8);
        let b = atom(2, 8);
        let weak = Arc::downgrade(&a.internal);
        let Approximation::PrescaledSin(child) = &a.internal.approximation else {
            panic!()
        };
        let child_weak = Arc::downgrade(&child.internal);
        assert!(exp_atom_structural_eq(&a, &b));
        assert_eq!(Arc::strong_count(&a.internal), 1);
        drop(a);
        drop(b);
        assert!(weak.upgrade().is_none());
        assert!(child_weak.upgrade().is_none());
        reset();
    }

    #[test]
    /// Compare two borrowed depth-128 fixtures in a scoped worker and observe its weak key.
    /// After joining, require both weak counts to be zero, witnessing TLS destruction.
    fn thread_exit_releases_weak_keys() {
        let a = atom(2, 128);
        let b = atom(2, 128);
        std::thread::scope(|scope| {
            scope
                .spawn(|| {
                    assert!(exp_atom_structural_eq(&a, &b));
                    assert_eq!(Arc::weak_count(&a.internal), 1);
                })
                .join()
                .unwrap();
        });
        assert_eq!(Arc::weak_count(&a.internal), 0);
        assert_eq!(Arc::weak_count(&b.internal), 0);
    }

    #[test]
    /// Hold a mutable TLS-cache borrow while checking equal and unequal structural fixtures.
    /// Require both uncached outcomes and no weak key; this exercises borrow conflict,
    /// not a call made during actual TLS teardown.
    fn borrowed_cache_falls_back_without_losing_true_or_false_results() {
        reset();
        let a = atom(2, 128);
        let b = atom(2, 128);
        let c = atom(3, 128);
        EXP_ATOM_CACHE.with(|cache| {
            let _borrow = cache.borrow_mut();
            assert!(exp_atom_structural_eq(&a, &b));
            assert!(!exp_atom_structural_eq(&a, &c));
        });
        assert_eq!(Arc::weak_count(&a.internal), 0);
    }

    #[test]
    /// Cache a match between independently authored sin(1/3) operands, then refine one at -512.
    /// Require cached and direct structural recognition to remain true; numerical
    /// caches do not participate in the recognized immutable shape.
    fn refining_one_operand_cannot_stale_a_structural_hit() {
        reset();
        let a = Computable::rational(Rational::fraction(1, 3).unwrap()).sin();
        let b = Computable::rational(Rational::fraction(1, 3).unwrap()).sin();
        assert!(exp_atom_structural_eq(&a, &b));
        let _ = a.approx(-512);
        assert!(exp_atom_structural_eq(&a, &b));
        assert!(Computable::internal_structural_eq(&a, &b));
        reset();
    }

    #[test]
    /// Exercise three dyadic widths, three signed numerators and three fresh repetitions with depth-128 atoms.
    /// Require zero/sign certificates inside the coefficient guard and decline for
    /// nonzero 2048-bit perturbations. These 27 symbolic cases do not validate the raw
    /// sine fixtures' numerical domains, measure cache hits or exercise numeric refinement.
    fn deep_independent_atoms_keep_all_proof_outcomes() {
        reset();
        for bits in [7usize, 999, 2048] {
            for numerator in [-1i64, 0, 1] {
                for _ in 0..3 {
                    let a = raw(Approximation::Sqrt(raw(Approximation::PrescaledExp(atom(
                        1, 128,
                    )))));
                    let delta = Computable::rational(
                        Rational::from_bigint_fraction(numerator.into(), BigUint::one() << bits)
                            .unwrap(),
                    );
                    let b = raw(Approximation::PrescaledExp(
                        atom(1, 128).shift_right(1).add(delta),
                    ));
                    let d = raw(Approximation::Add(a, raw(Approximation::Negate(b))));
                    let expected = if numerator == 0 {
                        Some(Sign::NoSign)
                    } else if bits > 1023 {
                        None
                    } else if numerator < 0 {
                        Some(Sign::Plus)
                    } else {
                        Some(Sign::Minus)
                    };
                    assert_eq!(d.exact_positive_exp_difference_sign(), expected);
                }
            }
        }
        reset();
    }
}
