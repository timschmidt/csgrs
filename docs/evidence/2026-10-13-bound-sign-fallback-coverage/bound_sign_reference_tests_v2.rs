use super::*;

pub(super) fn raw(approximation: Approximation, bound: BoundCache) -> Computable {
    Computable {
        internal: Arc::new(Node::new(approximation, bound, ExactSignCache::Invalid)),
        signal: None,
    }
}

pub(super) fn affine(integer: i64, bound: BoundCache) -> Computable {
    raw(
        Approximation::Add(
            Computable::rational(Rational::new(integer)),
            raw(
                Approximation::Negate(Computable::sqrt_rational(Rational::new(13))),
                BoundCache::Invalid,
            ),
        ),
        bound,
    )
}

pub(super) fn difference(bound: BoundCache) -> Computable {
    raw(
        Approximation::Add(
            Computable::sqrt_rational(Rational::new(13)),
            raw(
                Approximation::Negate(Computable::sqrt_rational(Rational::new(17))),
                BoundCache::Invalid,
            ),
        ),
        bound,
    )
}

pub(super) fn signed_bound(sign: Option<Sign>) -> BoundCache {
    BoundCache::Valid(BoundInfo::NonZero {
        sign,
        msd: None,
        exact_msd: false,
    })
}

#[test]
fn parent_affine_signs_and_publication_are_preserved() {
    for (integer, sign) in [(3, Sign::Minus), (5, Sign::Plus)] {
        for bound in [
            BoundCache::Invalid,
            BoundCache::Valid(BoundInfo::Unknown),
            signed_bound(None),
            signed_bound(Some(sign)),
        ] {
            let value = affine(integer, bound);
            let clone = value.clone();
            let weak = Arc::downgrade(&value.internal);
            assert_eq!(value.exact_sign(), Some(sign));
            assert_eq!(clone.exact_sign(), Some(sign));
            assert_eq!(
                value.internal.facts.exact_sign(),
                ExactSignCache::Valid(sign)
            );
            assert!(value.internal.cache.pi_laurent_polynomial().is_some());
            assert!(value.cached().is_none());
            assert_eq!(Arc::strong_count(&value.internal), 2);
            drop(value);
            drop(clone);
            assert!(weak.upgrade().is_none());
        }
    }
}

#[test]
fn parent_multiradical_decline_retains_an_existing_bound() {
    for bound in [
        BoundCache::Invalid,
        BoundCache::Valid(BoundInfo::Unknown),
        signed_bound(Some(Sign::Minus)),
    ] {
        let value = difference(bound);
        let clone = value.clone();
        assert_eq!(value.exact_sign(), None);
        assert_eq!(clone.exact_sign(), None);
        assert_eq!(value.internal.facts.exact_sign(), ExactSignCache::Unknown);
        assert_eq!(value.internal.facts.bound(), bound);
        assert!(value.internal.cache.pi_laurent_polynomial().is_some());
        assert!(value.cached().is_none());
    }
}

#[test]
fn parent_cached_unknown_and_known_sign_paths_are_preserved() {
    for (cached, expected) in [
        (ExactSignCache::Unknown, None),
        (ExactSignCache::Valid(Sign::Plus), Some(Sign::Plus)),
    ] {
        let value = affine(5, signed_bound(Some(Sign::Plus)));
        value.internal.facts.replace_exact_sign(cached);
        assert_eq!(value.exact_sign(), expected);
        assert!(value.internal.cache.pi_laurent_polynomial().is_none());
        assert!(value.cached().is_none());
    }
}
