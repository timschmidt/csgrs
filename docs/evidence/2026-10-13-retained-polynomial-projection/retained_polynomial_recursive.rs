use super::*;

// Only immutable closed certificates share storage. Scratch polynomials keep
// their original ownership, including every opaque Computable reference.
#[derive(Clone)]
pub(super) enum PiLaurentValue {
    Owned(PiLaurentPolynomial),
    Retained(Arc<ClosedPiLaurentPolynomial>),
}

impl PiLaurentValue {
    pub(super) fn into_polynomial(self) -> PiLaurentPolynomial {
        match self {
            Self::Owned(polynomial) => polynomial,
            Self::Retained(polynomial) => polynomial.to_polynomial(),
        }
    }

    pub(super) fn add(self, other: Self) -> Option<PiLaurentPolynomial> {
        self.into_polynomial().add(other.into_polynomial())
    }

    pub(super) fn scaled(self, scale: Rational) -> PiLaurentPolynomial {
        self.into_polynomial().scaled(scale)
    }

    pub(super) fn inverse_monomial(self) -> Option<PiLaurentPolynomial> {
        self.into_polynomial().inverse_monomial()
    }

    pub(super) fn multiply(self, other: Self) -> Option<PiLaurentPolynomial> {
        let left = self.into_polynomial();
        match other {
            Self::Owned(right) => left.multiply(right),
            Self::Retained(right) => left.multiply_closed(&right),
        }
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

    type Snapshot = Vec<(i16, Rational, Vec<(Rational, Rational)>)>;

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

    fn surd() -> PiLaurentPolynomial {
        let mut coefficient = RadicalCoefficient::radical(Rational::new(2), Rational::new(13));
        coefficient.rational = Rational::new(3);
        PiLaurentPolynomial::monomial(0, coefficient)
    }

    fn snapshot(polynomial: &PiLaurentPolynomial) -> Snapshot {
        polynomial
            .terms
            .iter()
            .map(|(power, coefficient)| {
                assert!(coefficient.atoms.is_empty());
                (
                    *power,
                    coefficient.rational.clone(),
                    coefficient.radicals.clone(),
                )
            })
            .collect()
    }

    fn wrap(polynomial: PiLaurentPolynomial, retained: bool) -> PiLaurentValue {
        if retained {
            PiLaurentValue::Retained(Arc::new(
                ClosedPiLaurentPolynomial::from_polynomial(&polynomial).unwrap(),
            ))
        } else {
            PiLaurentValue::Owned(polynomial)
        }
    }

    #[test]
    fn cache_views_share_closed_buffers_and_outlive_the_graph() {
        let value = raw(Approximation::Add(
            Computable::rational(Rational::new(3)),
            Computable::sqrt_rational(Rational::new(13)),
        ));
        let clone = value.clone();
        let weak_node = Arc::downgrade(&value.internal);
        let owners = Arc::strong_count(&value.internal);
        let mut work = 512;
        let mut reused = false;
        let first = value
            .pi_laurent_value_retained::<true>(64, &mut work, &mut None, &mut reused)
            .unwrap();
        assert!(matches!(first, PiLaurentValue::Owned(_)));
        let view = value.internal.cache.pi_laurent_value().unwrap();
        let PiLaurentValue::Retained(closed) = &view else {
            panic!("a successful closed cache read must share its immutable payload");
        };
        let weak_payload = Arc::downgrade(closed);
        let mut zero_work = 0;
        let mut second_reused = false;
        let second = value
            .pi_laurent_value_retained::<true>(0, &mut zero_work, &mut None, &mut second_reused)
            .unwrap();
        let PiLaurentValue::Retained(second_closed) = &second else {
            panic!("a zero-work cache hit must retain the same closed payload");
        };
        assert!(Arc::ptr_eq(closed, second_closed));
        assert_eq!(closed.terms.as_ptr(), second_closed.terms.as_ptr());
        assert_eq!(
            closed.terms[0].1.radicals.as_ptr(),
            second_closed.terms[0].1.radicals.as_ptr()
        );
        assert!(second_reused);
        assert_eq!(zero_work, 0);
        assert_eq!(Arc::strong_count(&value.internal), owners);
        assert!(value.cached().is_none());
        let mut owned = view.clone().into_polynomial();
        assert_ne!(
            owned.terms[0].1.radicals.as_ptr(),
            closed.terms[0].1.radicals.as_ptr()
        );
        owned.terms[0].1.rational = Rational::new(97);
        assert_eq!(closed.terms[0].1.rational, Rational::new(3));
        drop(first);
        drop(clone);
        drop(value);
        assert!(weak_node.upgrade().is_none());
        assert_eq!(
            snapshot(&second.clone().into_polynomial()),
            vec![(
                0,
                Rational::new(3),
                vec![(Rational::new(13), Rational::one())]
            )]
        );
        drop(view);
        assert!(weak_payload.upgrade().is_some());
        drop(second);
        assert!(weak_payload.upgrade().is_none());
        println!(
            "RETAINED_POLYNOMIAL_LAYOUT\t{}\t{}",
            std::mem::size_of::<PiLaurentValue>(),
            std::mem::size_of::<CachedEvaluation>()
        );
    }

    #[test]
    fn all_owned_and_retained_combinations_preserve_exact_arithmetic() {
        let expected = snapshot(&surd());
        for left_retained in [false, true] {
            for right_retained in [false, true] {
                let left = wrap(surd(), left_retained);
                let right = wrap(surd(), right_retained);
                let product = left.clone().multiply(right.clone()).unwrap();
                assert_eq!(
                    snapshot(&product),
                    vec![(
                        0,
                        Rational::new(61),
                        vec![(Rational::new(13), Rational::new(12))]
                    )]
                );
                let inverse = right.clone().inverse_monomial().unwrap();
                assert_eq!(
                    snapshot(&inverse),
                    vec![(
                        0,
                        Rational::fraction(-3, 43).unwrap(),
                        vec![(Rational::new(13), Rational::fraction(2, 43).unwrap())]
                    )]
                );
                let identity = PiLaurentValue::Owned(inverse)
                    .multiply(right.clone())
                    .unwrap();
                assert_eq!(snapshot(&identity), vec![(0, Rational::one(), vec![])]);
                let negative = right.clone().scaled(Rational::new(-1));
                assert!(
                    left.clone()
                        .add(PiLaurentValue::Owned(negative))
                        .unwrap()
                        .terms
                        .is_empty()
                );
                assert!(left.clone().scaled(Rational::zero()).terms.is_empty());
                assert_eq!(snapshot(&left.into_polynomial()), expected);
                assert_eq!(snapshot(&right.into_polynomial()), expected);
            }
        }
    }

    #[test]
    fn borrowed_products_preserve_order_and_both_frontiers() {
        let roots = [2, 3, 5, 7, 11, 13, 17, 19];
        for width in [0, 1, 2, 8] {
            for radical_width in [0, 1, 2, 8] {
                let mut left = PiLaurentPolynomial { terms: Vec::new() };
                for index in 0..width {
                    let mut coefficient = RadicalCoefficient::rational(Rational::new(3));
                    for root in &roots[..radical_width] {
                        coefficient
                            .insert_radical(Rational::new(*root), Rational::new(2))
                            .unwrap();
                    }
                    left.insert(index * 2 - 7, coefficient).unwrap();
                }
                for right in [surd(), left.clone()] {
                    let expected = left.clone().multiply(right.clone()).map(|p| snapshot(&p));
                    for retained_left in [false, true] {
                        let actual = wrap(left.clone(), retained_left)
                            .multiply(wrap(right.clone(), true))
                            .map(|p| snapshot(&p));
                        assert_eq!(
                            actual, expected,
                            "width={width}, radical_width={radical_width}"
                        );
                    }
                }
            }
        }
        for (left, right) in [(i16::MAX, 1), (i16::MIN, -1)] {
            let monomial = |power| {
                PiLaurentPolynomial::monomial(power, RadicalCoefficient::rational(Rational::one()))
            };
            assert!(
                wrap(monomial(left), true)
                    .multiply(wrap(monomial(right), true))
                    .is_none()
            );
        }
        assert!(
            wrap(
                PiLaurentPolynomial::monomial(
                    i16::MIN,
                    RadicalCoefficient::rational(Rational::one())
                ),
                true
            )
            .inverse_monomial()
            .is_none()
        );
        let mut full = PiLaurentPolynomial { terms: Vec::new() };
        for power in 0..8 {
            full.insert(power, RadicalCoefficient::rational(Rational::one()))
                .unwrap();
        }
        assert!(
            wrap(full, true)
                .add(wrap(
                    PiLaurentPolynomial::monomial(8, RadicalCoefficient::rational(Rational::one())),
                    true
                ))
                .is_none()
        );
        let mut coefficient = RadicalCoefficient::rational(Rational::zero());
        for root in roots {
            coefficient
                .insert_radical(Rational::new(root), Rational::one())
                .unwrap();
        }
        assert!(
            wrap(PiLaurentPolynomial::monomial(0, coefficient), true)
                .add(wrap(
                    PiLaurentPolynomial::monomial(
                        0,
                        RadicalCoefficient::radical(Rational::one(), Rational::new(23))
                    ),
                    true
                ))
                .is_none()
        );
    }

    #[test]
    fn concurrent_cache_reads_mutate_only_materialized_owned_values() {
        let value = raw(Approximation::Add(
            Computable::rational(Rational::new(3)),
            Computable::sqrt_rational(Rational::new(13)),
        ));
        let clone = value.clone();
        let mut work = 512;
        value
            .pi_laurent_value_retained::<true>(64, &mut work, &mut None, &mut false)
            .unwrap();
        let retained = value.internal.cache.pi_laurent_value().unwrap();
        std::thread::scope(|scope| {
            for index in 1..=8 {
                let value = &value;
                scope.spawn(move || {
                    for _ in 0..8 {
                        let read = value.internal.cache.pi_laurent_value().unwrap();
                        assert!(matches!(read, PiLaurentValue::Retained(_)));
                        let changed = read.scaled(Rational::new(-index));
                        assert_eq!(changed.terms[0].1.rational, Rational::new(-3 * index));
                        assert_eq!(changed.terms[0].1.radicals[0].1, Rational::new(-index));
                    }
                });
            }
        });
        assert_eq!(
            snapshot(&retained.into_polynomial()),
            vec![(
                0,
                Rational::new(3),
                vec![(Rational::new(13), Rational::one())]
            )]
        );
        assert!(clone.cached().is_none());
    }

    #[test]
    fn closed_right_operands_preserve_opaque_left_owners_and_declines() {
        let atom = raw(Approximation::AtanRational(
            Rational::fraction(3, 7).unwrap(),
        ));
        let weak = Arc::downgrade(&atom.internal);
        let owners = Arc::strong_count(&atom.internal);
        let left = || {
            PiLaurentValue::Owned(PiLaurentPolynomial::monomial(
                0,
                RadicalCoefficient::atom(atom.clone()),
            ))
        };
        let rational = wrap(
            PiLaurentPolynomial::monomial(0, RadicalCoefficient::rational(Rational::new(2))),
            true,
        );
        let product = left().multiply(rational).unwrap();
        assert_eq!(product.terms[0].1.atoms.len(), 1);
        assert!(Computable::internal_structural_eq(
            &product.terms[0].1.atoms[0].0,
            &atom
        ));
        assert_eq!(product.terms[0].1.atoms[0].1, Rational::new(2));
        assert_eq!(Arc::strong_count(&atom.internal), owners + 1);
        assert!(ClosedPiLaurentPolynomial::from_polynomial(&product).is_none());
        drop(product);
        assert_eq!(Arc::strong_count(&atom.internal), owners);
        assert!(left().multiply(wrap(surd(), true)).is_none());
        assert_eq!(Arc::strong_count(&atom.internal), owners);
        drop(atom);
        assert!(weak.upgrade().is_none());
    }
}
