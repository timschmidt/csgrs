#[cfg(test)]
mod exp_relation_reuse_tests {
    use super::*;

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

    fn atom(n: i64, depth: usize) -> Computable {
        let mut x = raw(Approximation::Int(n.into()));
        for _ in 0..depth {
            x = raw(Approximation::PrescaledSin(x));
        }
        x
    }

    fn reset() {
        EXP_ATOM_CACHE.with(|c| *c.borrow_mut() = ExpAtomCache::new());
    }

    #[test]
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
    fn pointer_identity_never_populates_cache() {
        reset();
        let a = atom(2, 128);
        assert!(exp_atom_structural_eq(&a, &a));
        assert_eq!(Arc::weak_count(&a.internal), 0);
    }

    #[test]
    fn collisions_and_eviction_preserve_full_comparison_outcomes() {
        reset();
        let pairs: Vec<_> = (0..512)
            .map(|i| (atom(i, 1), atom(i + i % 2, 1)))
            .collect();
        for _ in 0..3 {
            for (i, (a, b)) in pairs.iter().enumerate() {
                assert_eq!(exp_atom_structural_eq(a, b), i % 2 == 0);
            }
            EXP_ATOM_CACHE.with(|c| {
                assert!(c.borrow().entries.iter().flatten().count() <= EXP_ATOM_CACHE_SIZE);
            });
            assert!(pairs.iter().map(|(a, b)|
                Arc::weak_count(&a.internal) + Arc::weak_count(&b.internal)
            ).sum::<usize>() <= 2 * EXP_ATOM_CACHE_SIZE);
        }
        reset();
        assert!(pairs.iter().all(|(a, b)|
            Arc::weak_count(&a.internal) == 0 && Arc::weak_count(&b.internal) == 0));
    }

    #[test]
    fn cache_does_not_keep_nodes_or_child_graphs_alive() {
        reset();
        let a = atom(2, 8);
        let b = atom(2, 8);
        let weak = Arc::downgrade(&a.internal);
        let Approximation::PrescaledSin(child) = &a.internal.approximation else { panic!() };
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
    fn thread_exit_releases_weak_keys() {
        let a = atom(2, 128);
        let b = atom(2, 128);
        std::thread::scope(|scope| {
            scope.spawn(|| {
                assert!(exp_atom_structural_eq(&a, &b));
                assert_eq!(Arc::weak_count(&a.internal), 1);
            }).join().unwrap();
        });
        assert_eq!(Arc::weak_count(&a.internal), 0);
        assert_eq!(Arc::weak_count(&b.internal), 0);
    }

    #[test]
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
    fn deep_independent_atoms_keep_all_proof_outcomes() {
        reset();
        for bits in [7usize, 999, 2048] {
            for numerator in [-1i64, 0, 1] {
                for _ in 0..3 {
                    let a = raw(Approximation::Sqrt(raw(Approximation::PrescaledExp(atom(1, 128)))));
                    let delta = Computable::rational(Rational::from_bigint_fraction(
                        numerator.into(), BigUint::one() << bits).unwrap());
                    let b = raw(Approximation::PrescaledExp(atom(1, 128).shift_right(1).add(delta)));
                    let d = raw(Approximation::Add(a, raw(Approximation::Negate(b))));
                    let expected = if numerator == 0 { Some(Sign::NoSign) }
                        else if bits > 1023 { None }
                        else if numerator < 0 { Some(Sign::Plus) } else { Some(Sign::Minus) };
                    assert_eq!(d.exact_positive_exp_difference_sign(), expected);
                }
            }
        }
        reset();
    }
}
