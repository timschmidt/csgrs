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

fn closed_expression() -> Computable {
    raw(Approximation::Add(
        raw(Approximation::Ratio(Rational::new(3))),
        raw(Approximation::Sqrt(raw(Approximation::Ratio(
            Rational::new(13),
        )))),
    ))
}

fn assert_closed(value: &PiLaurentPolynomial) {
    assert_eq!(value.terms.len(), 1);
    assert_eq!(value.terms[0].0, 0);
    let coefficient = &value.terms[0].1;
    assert_eq!(coefficient.rational, Rational::new(3));
    assert_eq!(
        coefficient.radicals.as_slice(),
        [(Rational::new(13), Rational::one())]
    );
    assert!(coefficient.atoms.is_empty());
}

#[test]
fn published_closed_value_survives_zero_work_reuse_and_releases_owners() {
    let value = closed_expression();
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    let owners = Arc::strong_count(&value.internal);
    let mut memo = None;
    let mut work = 512;
    let mut reused = false;
    let first = value
        .pi_laurent_polynomial_retained::<true>(64, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_closed(&first);
    assert!(!reused);
    assert_closed(&value.internal.cache.pi_laurent_polynomial().unwrap());
    assert_eq!(Arc::strong_count(&value.internal), owners);
    let entries = memo.as_ref().map_or(0, Vec::len);
    println!("CLOSED_MEMO_INVENTORY\tretained\t{entries}");
    // A fresh zero-work query must use the published certificate, independent
    // of whether the original query kept a redundant local copy.
    drop(memo);
    drop(first);
    let mut empty_memo = None;
    work = 0;
    let warm = clone
        .pi_laurent_polynomial_retained::<true>(0, &mut work, &mut empty_memo, &mut reused)
        .unwrap();
    assert_closed(&warm);
    assert!(reused);
    assert_eq!(work, 0);
    assert!(empty_memo.is_none());
    assert!(value.cached().is_none());
    assert!(clone.cached().is_none());
    drop(warm);
    drop(clone);
    drop(value);
    assert!(weak.upgrade().is_none());
}

#[test]
fn original_recognizer_keeps_zero_work_query_memo_reuse() {
    let value = closed_expression();
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    let mut memo = None;
    let mut work = 512;
    let mut reused = false;
    let first = value
        .pi_laurent_polynomial_retained::<false>(64, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_closed(&first);
    assert!(value.internal.cache.pi_laurent_polynomial().is_none());
    assert_eq!(memo.as_ref().unwrap().len(), 1);
    println!("CLOSED_MEMO_INVENTORY\toriginal\t1");
    work = 0;
    let second = clone
        .pi_laurent_polynomial_retained::<false>(0, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_closed(&second);
    assert!(!reused);
    assert_eq!(work, 0);
    assert!(clone.cached().is_none());
    drop(first);
    drop(second);
    drop(memo);
    drop(clone);
    drop(value);
    assert!(weak.upgrade().is_none());
}

#[test]
fn rejected_opaque_publication_preserves_memo_and_releases_every_owner() {
    let opaque = raw(Approximation::AtanRational(
        Rational::fraction(3, 7).unwrap(),
    ));
    let opaque_weak = Arc::downgrade(&opaque.internal);
    let value = raw(Approximation::Add(
        opaque.clone(),
        raw(Approximation::Ratio(Rational::one())),
    ));
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    let owners = Arc::strong_count(&opaque.internal);
    let mut memo = None;
    let mut work = 512;
    let mut reused = false;
    let first = value
        .pi_laurent_polynomial_retained::<true>(64, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_eq!(first.terms.len(), 1);
    assert_eq!(first.terms[0].1.rational, Rational::one());
    assert_eq!(first.terms[0].1.atoms.len(), 1);
    assert_eq!(first.terms[0].1.atoms[0].1, Rational::one());
    assert!(Computable::internal_structural_eq(
        &first.terms[0].1.atoms[0].0,
        &opaque
    ));
    assert!(value.internal.cache.pi_laurent_polynomial().is_none());
    assert_eq!(memo.as_ref().unwrap().len(), 2);
    assert_eq!(Arc::strong_count(&opaque.internal), owners + 3);
    println!("CLOSED_MEMO_INVENTORY\topaque\t2");
    work = 0;
    let second = clone
        .pi_laurent_polynomial_retained::<true>(0, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_eq!(second.terms[0].1.atoms.len(), 1);
    assert!(!reused);
    assert_eq!(work, 0);
    drop(first);
    drop(second);
    assert_eq!(Arc::strong_count(&opaque.internal), owners + 2);
    drop(memo);
    assert_eq!(Arc::strong_count(&opaque.internal), owners);
    assert!(clone.cached().is_none());
    drop(clone);
    drop(value);
    drop(opaque);
    assert!(weak.upgrade().is_none());
    assert!(opaque_weak.upgrade().is_none());
}
