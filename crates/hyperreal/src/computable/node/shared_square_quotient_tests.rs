#[cfg(test)]
mod shared_square_quotient_tests {
    use super::*;
    use std::sync::atomic::Ordering as AtomicOrdering;

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

    fn factors(value: &Computable, square: bool) -> (Computable, Computable) {
        let product = if square {
            Approximation::Square(value.clone())
        } else {
            Approximation::Multiply(value.clone(), value.clone())
        };
        (raw(product), raw(Approximation::Inverse(value.clone())))
    }

    #[test]
    fn both_shared_product_shapes_and_orders_keep_the_entire_value_without_refinement() {
        for value in [Computable::ln2(), Computable::ln2().negate()] {
            assert!(matches!(
                value.immediate_sign(),
                Some(RealSign::Positive | RealSign::Negative)
            ));
            let before = value.cached();
            for square in [false, true] {
                let (product, inverse) = factors(&value, square);
                for (left, right) in [(product.clone(), inverse.clone()), (inverse, product)] {
                    let result = left.multiply(right);
                    assert!(Arc::ptr_eq(&result.internal, &value.internal));
                    assert_eq!(value.cached(), before);
                }
            }
        }
    }

    #[test]
    fn zero_unknown_and_unshared_operands_do_not_supply_a_cancellation_proof() {
        let unknown = raw(Approximation::Add(Computable::pi(), Computable::one()));
        assert_eq!(unknown.immediate_sign(), None);
        for value in [Computable::zero(), unknown] {
            for square in [false, true] {
                let (product, inverse) = factors(&value, square);
                assert!(product.shared_square_quotient(&inverse).is_none());
                assert!(value.cached().is_none());
            }
        }
        let value = Computable::ln2();
        let independently_constructed = raw(Approximation::Constant(SharedConstant::Ln2));
        assert!(Computable::internal_structural_eq(
            &value,
            &independently_constructed
        ));
        assert!(!Arc::ptr_eq(
            &value.internal,
            &independently_constructed.internal
        ));
        let inverse = raw(Approximation::Inverse(value.clone()));
        for product in [
            raw(Approximation::Square(independently_constructed.clone())),
            raw(Approximation::Multiply(
                value.clone(),
                independently_constructed,
            )),
            raw(Approximation::Add(value.clone(), value.clone())),
            value,
        ] {
            assert!(product.shared_square_quotient(&inverse).is_none());
        }
        assert!(
            Computable::one()
                .shared_square_quotient(&Computable::pi())
                .is_none()
        );
    }

    #[test]
    fn every_removed_wrapper_signal_is_retained_or_causes_a_decline() {
        // Different wrappers can share one mathematical node while carrying
        // different cancellation handles. Exercise each position independently.
        for position in 0..5 {
            for conflict in [false, true] {
                let signal = Arc::new(AtomicBool::new(true));
                let alternate = Arc::new(AtomicBool::new(false));
                let value = Computable::ln2();
                let mut first = value.clone();
                let mut second = value.clone();
                let mut divisor = value;
                for (index, child) in [&mut first, &mut second, &mut divisor]
                    .into_iter()
                    .enumerate()
                {
                    if position == index + 2 {
                        child.signal = Some(signal.clone());
                    }
                }
                let mut product = raw(Approximation::Multiply(first, second));
                let mut inverse = raw(Approximation::Inverse(divisor));
                if position == 0 {
                    product.signal = Some(signal.clone());
                }
                if position == 1 {
                    inverse.signal = Some(signal.clone());
                }
                if conflict {
                    if position == 0 {
                        inverse.signal = Some(alternate);
                    } else {
                        product.signal = Some(alternate);
                    }
                }
                let result = product.shared_square_quotient(&inverse);
                if conflict {
                    assert!(
                        result.is_none(),
                        "signal at position {position} was discarded"
                    );
                } else {
                    let result = result.unwrap();
                    assert!(Arc::ptr_eq(result.signal.as_ref().unwrap(), &signal));
                    assert!(should_stop(&result.signal));
                    signal.store(false, AtomicOrdering::Relaxed);
                    assert!(!should_stop(&result.signal));
                    assert!(result.approx(-32) > BigInt::zero());
                }
            }
        }
    }

    #[test]
    fn a_common_signal_and_descendant_graph_survive_square_cancellation() {
        let signal = Arc::new(AtomicBool::new(true));
        let mut child = Computable::ln2();
        child.abort(signal.clone());
        let value = raw(Approximation::Add(child, Computable::one()));
        // ln(2)+1 is strictly positive; attach only that exact root fact.
        value
            .internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(Sign::Plus));
        let (mut product, mut inverse) = factors(&value, true);
        product.abort(signal.clone());
        inverse.abort(signal.clone());
        let result = product.multiply(inverse);
        assert!(Arc::ptr_eq(&result.internal, &value.internal));
        assert!(Arc::ptr_eq(result.signal.as_ref().unwrap(), &signal));
        let Approximation::Add(child, _) = &result.internal.approximation else {
            panic!("descendant graph lost")
        };
        assert!(Arc::ptr_eq(child.signal.as_ref().unwrap(), &signal));
        signal.store(false, AtomicOrdering::Relaxed);
        assert!(result.approx(-32) > (BigInt::one() << 32));
    }
}
