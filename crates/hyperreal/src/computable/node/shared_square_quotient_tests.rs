#[cfg(test)]
/// Shared-operand reciprocal/square cancellation witnesses for nonzero evidence, identity and signal preservation.
mod shared_square_quotient_tests {
    use super::*;
    use std::sync::atomic::Ordering as AtomicOrdering;

    /// Wrap raw syntax with no bound/sign certificate and no signal, preserving the fixture without constructor rewrites.
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

    /// Build either Square(value) or Multiply(value,value), paired with an explicit inverse of the same node.
    /// Clone the wrappers so all children share node identity while their signal handles can be tested separately.
    fn factors(value: &Computable, square: bool) -> (Computable, Computable) {
        let product = if square {
            Approximation::Square(value.clone())
        } else {
            Approximation::Multiply(value.clone(), value.clone())
        };
        (raw(product), raw(Approximation::Inverse(value.clone())))
    }

    #[test]
    /// Cancel positive and negative ln(2) against their shared inverses in both orders, preserving cache snapshots.
    /// Decline zero, a positive expression with no immediate sign, and a separately
    /// allocated equal constant. These refusals test shortcut admission, not numerical inequality.
    fn shared_reciprocal_product_uses_only_the_same_retained_nonzero_value() {
        for value in [Computable::ln2(), Computable::ln2().negate()] {
            let inverse = raw(Approximation::Inverse(value.clone()));
            let before = value.cached();
            for (left, right) in [(value.clone(), inverse.clone()), (inverse, value.clone())] {
                assert_eq!(left.multiply(right).exact_rational(), Some(Rational::one()));
                assert_eq!(value.cached(), before);
            }
        }
        for value in [
            Computable::zero(),
            raw(Approximation::Add(Computable::pi(), Computable::one())),
        ] {
            let inverse = raw(Approximation::Inverse(value.clone()));
            assert!(value.shared_reciprocal_product(&inverse).is_none());
            assert!(value.cached().is_none());
        }
        let value = Computable::ln2();
        let distinct = raw(Approximation::Constant(SharedConstant::Ln2));
        let inverse = raw(Approximation::Inverse(distinct));
        assert!(value.shared_reciprocal_product(&inverse).is_none());
    }

    #[test]
    /// Carry a descendant stop flag through complete cancellation, or decline a conflicting outer flag.
    /// Toggle the retained flag to check identity-based reuse. A 32-negation fixture
    /// also declines the inspection budget without numeric cache creation; the test
    /// does not locate the exact minimal failing graph size or measure cancellation latency.
    fn shared_reciprocal_product_preserves_descendant_controls_or_declines() {
        for conflict in [false, true] {
            let signal = Arc::new(AtomicBool::new(true));
            let alternate = Arc::new(AtomicBool::new(false));
            let mut child = Computable::ln2();
            child.abort(signal.clone());
            let mut value = raw(Approximation::Add(child, Computable::one()));
            value
                .internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(Sign::Plus));
            if conflict {
                value.abort(alternate);
            }
            let inverse = raw(Approximation::Inverse(value.clone()));
            let result = value.shared_reciprocal_product(&inverse);
            if conflict {
                assert!(result.is_none());
            } else {
                let result = result.unwrap();
                assert_eq!(result.exact_rational(), Some(Rational::one()));
                assert!(Arc::ptr_eq(result.signal.as_ref().unwrap(), &signal));
                assert!(should_stop(&result.signal));
                signal.store(false, AtomicOrdering::Relaxed);
                assert!(!should_stop(&result.signal));
            }
        }
        let mut deep = Computable::ln2();
        for _ in 0..32 {
            deep = raw(Approximation::Negate(deep));
        }
        deep.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(Sign::Plus));
        let inverse = raw(Approximation::Inverse(deep.clone()));
        assert!(deep.shared_reciprocal_product(&inverse).is_none());
        assert!(deep.cached().is_none());
    }

    #[test]
    /// Check both signs of ln(2), both square/product shapes and both multiplication orders.
    /// Require the result to retain the original Arc node and unchanged numeric cache
    /// snapshot across all eight cases.
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
    /// Decline zero/unknown signs, separately allocated equal operands and unsupported product shapes.
    /// Explicitly distinguish structural equality from Arc identity and preserve empty
    /// caches in the zero/unknown cases; none of these declines proves inequality.
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
    /// Place a stop flag at each of five removed wrapper positions, with and without an identity conflict.
    /// For all ten cases, require either decline or retention of the exact flag. After
    /// clearing a retained flag, a positive approximation witnesses subsequent consumption.
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
    /// Cancel a shared square of ln(2)+1 while retaining its exact root node and descendant signal.
    /// The fixture explicitly seeds the independently known positive sign. After
    /// clearing the common flag, require an approximation greater than one at scale 2^32.
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
