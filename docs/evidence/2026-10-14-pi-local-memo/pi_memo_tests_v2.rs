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

fn assert_pi(value: &PiLaurentPolynomial) {
    assert_eq!(value.terms.len(), 1);
    let (power, coefficient) = &value.terms[0];
    assert_eq!(*power, 1);
    assert_eq!(coefficient.rational, Rational::one());
    assert!(coefficient.radicals.is_empty());
    assert!(coefficient.atoms.is_empty());
}

fn pi_budgets<const RETAIN: bool>() {
    let value = raw(Approximation::Constant(SharedConstant::Pi));
    let weak = Arc::downgrade(&value.internal);
    let mut memo = None;
    let mut reused = false;
    for (depth, mut work) in [(0, 1), (64, 0)] {
        let before = work;
        assert!(
            value
                .pi_laurent_polynomial_retained::<RETAIN>(depth, &mut work, &mut memo, &mut reused)
                .is_none()
        );
        assert_eq!(work, before);
        assert!(memo.is_none());
    }
    let mut work = 1;
    let unique = value
        .pi_laurent_polynomial_retained::<RETAIN>(1, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_pi(&unique);
    assert_eq!(work, 0);
    assert!(memo.is_none());
    drop(unique);
    let clone = value.clone();
    work = 512;
    let first = value
        .pi_laurent_polynomial_retained::<RETAIN>(64, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_pi(&first);
    assert_eq!(work, 511);
    let entries = memo.as_ref().unwrap();
    assert_eq!(entries.len(), 1);
    assert_eq!(entries[0].0, Arc::as_ptr(&value.internal) as usize);
    assert!(matches!(entries[0].1, PiLaurentMemoValue::Pi));
    work = 0;
    let warm = clone
        .pi_laurent_polynomial_retained::<RETAIN>(0, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_pi(&warm);
    assert_eq!(work, 0);
    assert!(!reused);
    // A process-wide constant cache may already be warm. This raw node's
    // own evaluation cell must remain uninitialized.
    assert!(value.internal.cache.cell().is_none());
    assert_eq!(Arc::strong_count(&value.internal), 2);
    drop(first);
    drop(warm);
    drop(clone);
    assert_eq!(Arc::strong_count(&value.internal), 1);
    // The original sharing guard still applies even when a memo entry exists.
    assert!(
        value
            .pi_laurent_polynomial_retained::<RETAIN>(0, &mut work, &mut memo, &mut reused)
            .is_none()
    );
    assert_eq!(work, 0);
    assert_eq!(memo.as_ref().unwrap().len(), 1);
    drop(memo);
    drop(value);
    assert!(weak.upgrade().is_none());
}

#[test]
fn pi_replay_preserves_budgets_and_sharing_in_both_passes() {
    pi_budgets::<true>();
    pi_budgets::<false>();
    println!(
        "PI_MEMO_LAYOUT\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
        std::mem::size_of::<Rational>(),
        std::mem::size_of::<RadicalCoefficient>(),
        std::mem::size_of::<PiLaurentPolynomial>(),
        std::mem::size_of::<Option<PiLaurentPolynomial>>(),
        std::mem::size_of::<PiLaurentMemoValue>(),
        std::mem::size_of::<(usize, Option<PiLaurentPolynomial>)>(),
        std::mem::size_of::<(usize, PiLaurentMemoValue)>(),
        std::mem::size_of::<ClosedRadicalCoefficient>(),
        std::mem::size_of::<ClosedPiLaurentPolynomial>(),
        std::mem::size_of::<Computable>(),
        std::mem::size_of::<Node>(),
    );
}

fn ordinary(value: Computable, check: impl Fn(&PiLaurentPolynomial)) {
    let weak = Arc::downgrade(&value.internal);
    let clone = value.clone();
    let key = Arc::as_ptr(&value.internal) as usize;
    let mut memo = None;
    let mut work = 512;
    let mut reused = false;
    let first = value
        .pi_laurent_polynomial_retained::<false>(64, &mut work, &mut memo, &mut reused)
        .unwrap();
    check(&first);
    let (_, stored) = memo
        .as_ref()
        .unwrap()
        .iter()
        .find(|(candidate, _)| *candidate == key)
        .unwrap();
    let PiLaurentMemoValue::Polynomial(polynomial) = stored else {
        panic!("a non-Pi node was labeled as Pi");
    };
    check(polynomial);
    work = 0;
    let warm = clone
        .pi_laurent_polynomial_retained::<false>(0, &mut work, &mut memo, &mut reused)
        .unwrap();
    check(&warm);
    assert_eq!(work, 0);
    assert!(!reused);
    assert!(value.internal.cache.cell().is_none());
    drop(first);
    drop(warm);
    drop(memo);
    drop(clone);
    drop(value);
    assert!(weak.upgrade().is_none());
}

#[test]
fn other_leaves_and_pi_valued_composites_keep_polynomial_replay() {
    let wide = (BigInt::one() << 128_usize) + 1_u32;
    for approximation in [
        Approximation::Int(BigInt::zero()),
        Approximation::Ratio(Rational::zero()),
    ] {
        ordinary(raw(approximation), |value| assert!(value.terms.is_empty()));
    }
    for (approximation, expected) in [
        (Approximation::One, Rational::one()),
        (
            Approximation::Int(wide.clone()),
            Rational::from_bigint(wide),
        ),
        (
            Approximation::Ratio(Rational::fraction(3, 7).unwrap()),
            Rational::fraction(3, 7).unwrap(),
        ),
    ] {
        ordinary(raw(approximation), |value| {
            assert_eq!(value.terms.len(), 1);
            assert_eq!(value.terms[0].0, 0);
            assert_eq!(value.terms[0].1.rational, expected);
            assert!(value.terms[0].1.radicals.is_empty());
            assert!(value.terms[0].1.atoms.is_empty());
        });
    }
    for constant in [
        SharedConstant::E,
        SharedConstant::InvPi,
        SharedConstant::Tau,
        SharedConstant::Ln2,
        SharedConstant::Ln3,
        SharedConstant::Ln5,
        SharedConstant::Ln6,
        SharedConstant::Ln7,
        SharedConstant::Ln10,
        SharedConstant::Sqrt2,
        SharedConstant::Sqrt3,
        SharedConstant::Acosh2,
        SharedConstant::Asinh1,
        SharedConstant::AtanInv2,
        SharedConstant::AtanInv5,
        SharedConstant::Atan2,
        SharedConstant::AtanThreeHalves,
    ] {
        ordinary(raw(Approximation::Constant(constant)), |value| {
            assert_eq!(value.terms.len(), 1);
            let (power, coefficient) = &value.terms[0];
            match constant {
                SharedConstant::InvPi | SharedConstant::Tau => {
                    assert_eq!(
                        *power,
                        if constant == SharedConstant::InvPi {
                            -1
                        } else {
                            1
                        }
                    );
                    assert_eq!(
                        coefficient.rational,
                        Rational::new(if constant == SharedConstant::InvPi {
                            1
                        } else {
                            2
                        })
                    );
                    assert!(coefficient.radicals.is_empty());
                    assert!(coefficient.atoms.is_empty());
                }
                SharedConstant::Sqrt2 | SharedConstant::Sqrt3 => {
                    assert_eq!(*power, 0);
                    assert_eq!(coefficient.rational, Rational::zero());
                    assert_eq!(
                        coefficient.radicals.as_slice(),
                        [(
                            Rational::new(if constant == SharedConstant::Sqrt2 {
                                2
                            } else {
                                3
                            }),
                            Rational::one()
                        )]
                    );
                    assert!(coefficient.atoms.is_empty());
                }
                _ => {
                    assert_eq!(*power, 0);
                    assert_eq!(coefficient.rational, Rational::zero());
                    assert!(coefficient.radicals.is_empty());
                    assert_eq!(coefficient.atoms.len(), 1);
                    assert_eq!(coefficient.atoms[0].1, Rational::one());
                    assert!(
                        matches!(coefficient.atoms[0].0.internal.approximation, Approximation::Constant(actual) if actual == constant)
                    );
                }
            }
        });
    }
    ordinary(
        raw(Approximation::Add(
            raw(Approximation::Constant(SharedConstant::Pi)),
            raw(Approximation::Int(BigInt::zero())),
        )),
        assert_pi,
    );
}

fn opaque_owners<const RETAIN: bool>(value: Computable, depth: usize) {
    let weak = Arc::downgrade(&value.internal);
    let clone = value.clone();
    let mut memo = None;
    let mut work = 1;
    let mut reused = false;
    let first = value
        .pi_laurent_polynomial_retained::<RETAIN>(depth, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_eq!(work, 0);
    assert_eq!(memo.as_ref().unwrap().len(), 1);
    assert!(matches!(
        memo.as_ref().unwrap()[0].1,
        PiLaurentMemoValue::Polynomial(_)
    ));
    let warm = clone
        .pi_laurent_polynomial_retained::<RETAIN>(0, &mut work, &mut memo, &mut reused)
        .unwrap();
    for proof in [&first, &warm] {
        assert_eq!(proof.terms.len(), 1);
        assert_eq!(proof.terms[0].0, 0);
        let coefficient = &proof.terms[0].1;
        assert_eq!(coefficient.rational, Rational::zero());
        assert!(coefficient.radicals.is_empty());
        assert_eq!(coefficient.atoms.len(), 1);
        assert!(Arc::ptr_eq(
            &coefficient.atoms[0].0.internal,
            &value.internal
        ));
        assert_eq!(coefficient.atoms[0].1, Rational::one());
    }
    assert_eq!(work, 0);
    assert!(!reused);
    assert!(value.internal.cache.cell().is_none());
    assert_eq!(Arc::strong_count(&value.internal), 5);
    drop(first);
    drop(warm);
    drop(clone);
    assert_eq!(Arc::strong_count(&value.internal), 2);
    let final_replay = value
        .pi_laurent_polynomial_retained::<RETAIN>(0, &mut work, &mut memo, &mut reused)
        .unwrap();
    assert_eq!(Arc::strong_count(&value.internal), 3);
    drop(final_replay);
    drop(memo);
    assert_eq!(Arc::strong_count(&value.internal), 1);
    drop(value);
    assert!(weak.upgrade().is_none());
}

fn opaque_cases<const RETAIN: bool>() {
    opaque_owners::<RETAIN>(
        raw(Approximation::AtanRational(
            Rational::fraction(3, 7).unwrap(),
        )),
        64,
    );
    // The exhausted child expansion must still become the root's opaque atom.
    opaque_owners::<RETAIN>(
        raw(Approximation::Add(
            raw(Approximation::One),
            raw(Approximation::One),
        )),
        1,
    );
}

#[test]
fn opaque_replay_keeps_owners_and_exhausted_expansion_fallback() {
    opaque_cases::<true>();
    opaque_cases::<false>();
}
