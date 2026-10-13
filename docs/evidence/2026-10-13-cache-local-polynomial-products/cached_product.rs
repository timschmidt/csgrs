use super::*;

impl EvaluationCache {
    // This typed entry point admits only the closed-product kernel. It cannot
    // invoke an arbitrary callback while holding the expression cache lock.
    // A miss returns the original owned left polynomial without cloning it.
    fn multiply_pi_laurent_polynomial(
        &self,
        left: PiLaurentPolynomial,
    ) -> Result<Option<PiLaurentPolynomial>, PiLaurentPolynomial> {
        let Some(cell) = self.cell() else {
            return Err(left);
        };
        let guard = cell.read().unwrap_or_else(|error| error.into_inner());
        let Some(right) = guard.pi_laurent_polynomial.as_deref() else {
            return Err(left);
        };
        // Closed coefficients contain no expression owners. The kernel uses
        // rational arithmetic and immutable structural comparison only; it
        // must never evaluate an expression or access an EvaluationCache.
        Ok(left.multiply_closed(right))
    }
}

impl Computable {
    pub(super) fn multiply_pi_laurent_polynomial_retained<const RETAIN: bool>(
        &self,
        left: PiLaurentPolynomial,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
        reused: &mut bool,
    ) -> Option<PiLaurentPolynomial> {
        let left = if RETAIN {
            match self.internal.cache.multiply_pi_laurent_polynomial(left) {
                Ok(product) => {
                    *reused = true;
                    return product;
                }
                Err(left) => left,
            }
        } else {
            left
        };
        // Exactly one cache probe precedes the original cache-miss recognizer.
        // A product decline after a hit is distinct from a cache miss, so the
        // outer query can retain its original reused-certificate retry.
        left.multiply(self.pi_laurent_polynomial_uncached::<RETAIN>(
            budget,
            remaining_work,
            memo,
            reused,
        )?)
    }
}

impl PiLaurentPolynomial {
    fn multiply_closed(self, other: &ClosedPiLaurentPolynomial) -> Option<Self> {
        let mut product = Self { terms: Vec::new() };
        if let [(right_power, right_coefficient)] = other.terms.as_slice() {
            // Match the owned algorithm's consumption and insertion order.
            for (left_power, left_coefficient) in self.terms {
                product.insert(
                    left_power.checked_add(*right_power)?,
                    left_coefficient.multiply_parts(
                        &right_coefficient.rational,
                        &right_coefficient.radicals,
                        &[],
                    )?,
                )?;
            }
            return Some(product);
        }
        for (left_power, left_coefficient) in self.terms {
            for (right_power, right_coefficient) in &other.terms {
                product.insert(
                    left_power.checked_add(*right_power)?,
                    left_coefficient.clone().multiply_parts(
                        &right_coefficient.rational,
                        &right_coefficient.radicals,
                        &[],
                    )?,
                )?;
            }
        }
        Some(product)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        }
    }

    fn rational(value: i64) -> PiLaurentPolynomial {
        PiLaurentPolynomial::monomial(0, RadicalCoefficient::rational(Rational::new(value)))
    }

    fn cache_rational(value: i64) -> Computable {
        let result = raw(Approximation::Ratio(Rational::new(value)));
        result
            .internal
            .cache
            .store_pi_laurent_polynomial(&rational(value));
        result
    }

    fn equal(left: &PiLaurentPolynomial, right: &PiLaurentPolynomial) {
        assert_eq!(left.terms.len(), right.terms.len());
        for ((lp, lc), (rp, rc)) in left.terms.iter().zip(&right.terms) {
            assert_eq!(lp, rp);
            assert_eq!(lc.rational, rc.rational);
            assert_eq!(lc.radicals, rc.radicals);
            assert_eq!(lc.atoms.len(), rc.atoms.len());
            for ((la, lc), (ra, rc)) in lc.atoms.iter().zip(&rc.atoms) {
                assert!(Computable::internal_structural_eq(la, ra));
                assert_eq!(lc, rc);
            }
        }
    }

    fn equal_optional(left: Option<PiLaurentPolynomial>, right: Option<PiLaurentPolynomial>) {
        match (left, right) {
            (Some(left), Some(right)) => equal(&left, &right),
            (None, None) => {}
            _ => panic!("product acceptance differs"),
        }
    }

    #[test]
    fn borrowed_product_preserves_box_buffers_and_releases_the_graph() {
        let value = raw(Approximation::Add(
            raw(Approximation::Ratio(Rational::new(3))),
            raw(Approximation::Multiply(
                raw(Approximation::Ratio(Rational::new(2))),
                raw(Approximation::Sqrt(raw(Approximation::Ratio(
                    Rational::new(13),
                )))),
            )),
        ));
        let mut coefficient = RadicalCoefficient::radical(Rational::new(2), Rational::new(13));
        coefficient.rational = Rational::new(3);
        let polynomial = PiLaurentPolynomial::monomial(0, coefficient);
        value
            .internal
            .cache
            .store_pi_laurent_polynomial(&polynomial);
        let weak = Arc::downgrade(&value.internal);
        let owners = Arc::strong_count(&value.internal);
        let pointers = {
            let guard = value.internal.cache.cell().unwrap().read().unwrap();
            let closed = guard.pi_laurent_polynomial.as_deref().unwrap();
            (
                closed as *const _,
                closed.terms.as_ptr(),
                closed.terms[0].1.radicals.as_ptr(),
            )
        };
        let product = value
            .internal
            .cache
            .multiply_pi_laurent_polynomial(rational(2))
            .ok()
            .flatten()
            .unwrap();
        equal(&product, &polynomial.scaled(Rational::new(2)));
        assert_eq!(product.terms[0].1.rational, Rational::new(6));
        assert_eq!(
            product.terms[0].1.radicals,
            vec![(Rational::new(13), Rational::new(4))]
        );
        {
            let guard = value.internal.cache.cell().unwrap().read().unwrap();
            let closed = guard.pi_laurent_polynomial.as_deref().unwrap();
            assert_eq!(
                pointers,
                (
                    closed as *const _,
                    closed.terms.as_ptr(),
                    closed.terms[0].1.radicals.as_ptr()
                )
            );
        }
        assert!(value.internal.cache.cell().unwrap().try_write().is_ok());
        assert_eq!(Arc::strong_count(&value.internal), owners);
        assert!(value.cached().is_none());
        println!(
            "CACHED_PRODUCT_LAYOUT\t{}\t{}",
            std::mem::size_of::<Result<Option<PiLaurentPolynomial>, PiLaurentPolynomial>>(),
            std::mem::size_of::<CachedEvaluation>()
        );
        drop(value);
        assert!(weak.upgrade().is_none());
        assert_eq!(product.terms[0].1.rational, Rational::new(6));
    }

    #[test]
    fn borrowed_products_match_owned_order_at_term_radical_and_power_frontiers() {
        fn fixture(width: usize, radicals: usize) -> PiLaurentPolynomial {
            let mut terms = Vec::new();
            for power in 0..width {
                let mut coefficient = RadicalCoefficient::rational(Rational::new(3));
                for root in [2, 3, 5, 7, 11, 13, 17, 19].into_iter().take(radicals) {
                    coefficient
                        .insert_radical(Rational::new(root), Rational::fraction(-5, 7).unwrap())
                        .unwrap();
                }
                terms.push((i16::try_from(power).unwrap() - 3, coefficient));
            }
            PiLaurentPolynomial { terms }
        }
        for left_width in [0, 1, 2, 8] {
            for right_width in [0, 1, 2, 8] {
                for left_radicals in [0, 1, 2, 8] {
                    for right_radicals in [0, 1, 2, 8] {
                        let left = fixture(left_width, left_radicals);
                        let right = fixture(right_width, right_radicals);
                        let closed = ClosedPiLaurentPolynomial::from_polynomial(&right).unwrap();
                        equal_optional(left.clone().multiply(right), left.multiply_closed(&closed));
                    }
                }
            }
        }
        for (left_power, right_power) in
            [(i16::MIN, -1), (i16::MAX, 1), (i16::MIN, 1), (i16::MAX, -1)]
        {
            let left = PiLaurentPolynomial::monomial(
                left_power,
                RadicalCoefficient::rational(Rational::new(2)),
            );
            let right = PiLaurentPolynomial::monomial(
                right_power,
                RadicalCoefficient::rational(Rational::new(3)),
            );
            let closed = ClosedPiLaurentPolynomial::from_polynomial(&right).unwrap();
            let expected = left.clone().multiply(right);
            assert_eq!(
                expected.is_some(),
                left_power.checked_add(right_power).is_some()
            );
            equal_optional(expected, left.multiply_closed(&closed));
        }
    }

    #[test]
    fn misses_return_original_left_buffers_without_creating_a_cache_or_owners() {
        let atom = raw(Approximation::AtanRational(
            Rational::fraction(3, 7).unwrap(),
        ));
        let weak = Arc::downgrade(&atom.internal);
        let value = raw(Approximation::Ratio(Rational::new(7)));
        for initialized in [false, true] {
            if initialized {
                value.internal.store_cache_value(-8, BigInt::from(7) << 8);
            }
            let left = PiLaurentPolynomial::monomial(0, RadicalCoefficient::atom(atom.clone()));
            let pointers = (left.terms.as_ptr(), left.terms[0].1.atoms.as_ptr());
            let owners = Arc::strong_count(&atom.internal);
            let returned = value
                .internal
                .cache
                .multiply_pi_laurent_polynomial(left)
                .err()
                .unwrap();
            assert_eq!(
                pointers,
                (returned.terms.as_ptr(), returned.terms[0].1.atoms.as_ptr())
            );
            assert_eq!(Arc::strong_count(&atom.internal), owners);
            assert_eq!(value.internal.cache.cell().is_some(), initialized);
            if initialized {
                assert!(value.internal.cache.cell().unwrap().try_write().is_ok());
            }
            drop(returned);
        }
        assert_eq!(Arc::strong_count(&atom.internal), 1);
        drop(atom);
        assert!(weak.upgrade().is_none());
    }

    #[test]
    fn zero_work_hits_and_product_declines_preserve_reuse_and_memo_accounting() {
        let value = cache_rational(7);
        let mut work = 0;
        let mut memo = None;
        let mut reused = false;
        let product = value
            .multiply_pi_laurent_polynomial_retained::<true>(
                rational(3),
                0,
                &mut work,
                &mut memo,
                &mut reused,
            )
            .unwrap();
        equal(&product, &rational(21));
        assert!(reused && memo.is_none());
        assert_eq!(work, 0);
        assert!(
            value
                .multiply_pi_laurent_polynomial_retained::<false>(
                    rational(3),
                    0,
                    &mut work,
                    &mut memo,
                    &mut false
                )
                .is_none()
        );
        let pi = raw(Approximation::Constant(SharedConstant::Pi));
        pi.internal
            .cache
            .store_pi_laurent_polynomial(&PiLaurentPolynomial::monomial(
                1,
                RadicalCoefficient::rational(Rational::one()),
            ));
        let left =
            PiLaurentPolynomial::monomial(i16::MAX, RadicalCoefficient::rational(Rational::one()));
        reused = false;
        assert!(
            pi.multiply_pi_laurent_polynomial_retained::<true>(
                left,
                0,
                &mut work,
                &mut memo,
                &mut reused
            )
            .is_none()
        );
        assert!(reused && memo.is_none());
        assert_eq!(work, 0);
        assert!(pi.internal.cache.cell().unwrap().try_write().is_ok());
    }

    #[test]
    fn cold_fallback_preserves_recognizer_work_and_memo_frontiers() {
        fn expression() -> Computable {
            raw(Approximation::Add(
                raw(Approximation::Sqrt(raw(Approximation::Ratio(
                    Rational::new(2),
                )))),
                raw(Approximation::Sqrt(raw(Approximation::Ratio(
                    Rational::new(3),
                )))),
            ))
        }
        for budget in [0, 1, 2, 3, 64] {
            for work in [0, 1, 2, 3, 512] {
                let expected_root = expression();
                let actual_root = expression();
                let expected_clone = expected_root.clone();
                let actual_clone = actual_root.clone();
                let (mut expected_work, mut actual_work) = (work, work);
                let (mut expected_memo, mut actual_memo) = (None, None);
                let (mut expected_reused, mut actual_reused) = (false, false);
                let expected = expected_root
                    .pi_laurent_polynomial_retained::<true>(
                        budget,
                        &mut expected_work,
                        &mut expected_memo,
                        &mut expected_reused,
                    )
                    .and_then(|right| rational(3).multiply(right));
                let actual = actual_root.multiply_pi_laurent_polynomial_retained::<true>(
                    rational(3),
                    budget,
                    &mut actual_work,
                    &mut actual_memo,
                    &mut actual_reused,
                );
                equal_optional(expected, actual);
                assert_eq!(expected_work, actual_work);
                assert_eq!(expected_reused, actual_reused);
                let inventory = |memo: &Option<Vec<(usize, Option<PiLaurentPolynomial>)>>| {
                    memo.as_ref()
                        .map(|items| items.iter().map(|(_, p)| p.is_some()).collect::<Vec<_>>())
                };
                assert_eq!(inventory(&expected_memo), inventory(&actual_memo));
                assert_eq!(
                    Arc::strong_count(&expected_clone.internal),
                    Arc::strong_count(&actual_clone.internal)
                );
                assert!(expected_clone.cached().is_none() && actual_clone.cached().is_none());
            }
        }
    }

    #[test]
    fn concurrent_product_readers_and_exact_approximation_writers_complete() {
        let value = cache_rational(7);
        let weak = Arc::downgrade(&value.internal);
        let barrier = std::sync::Barrier::new(9);
        std::thread::scope(|scope| {
            for _ in 0..8 {
                let value = &value;
                let barrier = &barrier;
                scope.spawn(move || {
                    barrier.wait();
                    for _ in 0..64 {
                        let result = value
                            .internal
                            .cache
                            .multiply_pi_laurent_polynomial(rational(3))
                            .ok()
                            .flatten()
                            .unwrap();
                        equal(&result, &rational(21));
                    }
                });
            }
            let value = &value;
            let barrier = &barrier;
            scope.spawn(move || {
                barrier.wait();
                for bits in 64..192 {
                    value.internal.store_cache_value(
                        -bits,
                        BigInt::from(7) << usize::try_from(bits).unwrap(),
                    );
                }
            });
        });
        let (precision, approximation) = value.internal.cache_snapshot().unwrap();
        assert_eq!(precision, -191);
        assert_eq!(approximation, BigInt::from(7) << 191);
        assert!(value.internal.cache.cell().unwrap().try_write().is_ok());
        drop(value);
        assert!(weak.upgrade().is_none());
    }

    #[test]
    fn poisoned_cache_recovers_for_products_and_subsequent_writes() {
        let value = cache_rational(7);
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _guard = value.internal.cache.cell().unwrap().write().unwrap();
            panic!("deliberate poison for recovery control");
        }));
        assert!(panicked.is_err());
        let product = value
            .internal
            .cache
            .multiply_pi_laurent_polynomial(rational(3))
            .ok()
            .flatten()
            .unwrap();
        equal(&product, &rational(21));
        value.internal.store_cache_value(-8, BigInt::from(7) << 8);
        assert_eq!(
            value.internal.cache_snapshot().unwrap(),
            (-8, BigInt::from(7) << 8)
        );
    }

    #[test]
    fn opaque_left_can_reference_the_borrowed_node_without_evaluation() {
        let value = cache_rational(7);
        let weak = Arc::downgrade(&value.internal);
        let left = PiLaurentPolynomial::monomial(0, RadicalCoefficient::atom(value.clone()));
        let owners = Arc::strong_count(&value.internal);
        let product = value
            .internal
            .cache
            .multiply_pi_laurent_polynomial(left)
            .ok()
            .flatten()
            .unwrap();
        assert_eq!(product.terms.len(), 1);
        assert_eq!(product.terms[0].1.atoms.len(), 1);
        assert!(Arc::ptr_eq(
            &product.terms[0].1.atoms[0].0.internal,
            &value.internal
        ));
        assert_eq!(product.terms[0].1.atoms[0].1, Rational::new(7));
        assert_eq!(Arc::strong_count(&value.internal), owners);
        assert!(value.cached().is_none());
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Unknown
        ));
        assert!(value.internal.cache.cell().unwrap().try_write().is_ok());
        drop(value);
        assert!(weak.upgrade().is_some());
        drop(product);
        assert!(weak.upgrade().is_none());

        let surd = raw(Approximation::Sqrt(raw(Approximation::Ratio(
            Rational::new(13),
        ))));
        surd.internal
            .cache
            .store_pi_laurent_polynomial(&PiLaurentPolynomial::monomial(
                0,
                RadicalCoefficient::radical(Rational::one(), Rational::new(13)),
            ));
        let left = PiLaurentPolynomial::monomial(0, RadicalCoefficient::atom(surd.clone()));
        let mut reused = false;
        assert!(
            surd.multiply_pi_laurent_polynomial_retained::<true>(
                left,
                0,
                &mut 0,
                &mut None,
                &mut reused,
            )
            .is_none()
        );
        assert!(reused);
        assert_eq!(Arc::strong_count(&surd.internal), 1);
        assert!(surd.cached().is_none());
        assert!(surd.internal.cache.cell().unwrap().try_write().is_ok());
    }
}
