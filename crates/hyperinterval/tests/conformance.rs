use hyperinterval::{DomainCoverage, RangeEnclosure, SignSet};

#[test]
fn sign_sets_are_all_subsets_of_three_signs() {
    let sets = [
        SignSet::Empty,
        SignSet::Negative,
        SignSet::Zero,
        SignSet::NegativeOrZero,
        SignSet::Positive,
        SignSet::Nonzero,
        SignSet::ZeroOrPositive,
        SignSet::Mixed,
    ];
    for (bits, set) in sets.iter().copied().enumerate() {
        assert_eq!(set.contains_negative(), bits & 1 != 0);
        assert_eq!(set.contains_zero(), bits & 2 != 0);
        assert_eq!(set.contains_positive(), bits & 4 != 0);
        assert_eq!(set.is_singleton(), bits.count_ones() == 1);
        for (other_bits, other) in sets.iter().copied().enumerate() {
            assert_eq!(set.union(other), sets[bits | other_bits]);
        }
    }
    assert!(!SignSet::Mixed.is_singleton());
    assert!(!SignSet::NegativeOrZero.is_singleton());
    assert!(!SignSet::ZeroOrPositive.is_singleton());
}

#[test]
fn domain_qualification_is_not_erased_by_equal_images() {
    let complete = RangeEnclosure {
        range: (0, 2),
        domain: DomainCoverage::Complete,
    };
    let partial = RangeEnclosure {
        range: (0, 2),
        domain: DomainCoverage::Partial,
    };
    assert_ne!(complete, partial);
    assert_eq!(complete.range, partial.range);
}

#[cfg(feature = "integer")]
mod integer {
    use hyperinterval::{IntegerRange, IntervalError, SignSet};
    use std::collections::BTreeSet;

    #[test]
    fn discrete_sets_match_independent_enumeration() {
        let mut cases = vec![(IntegerRange::<i8>::empty(), BTreeSet::new())];
        for lower in -3..=3 {
            for upper in lower..=3 {
                cases.push((
                    IntegerRange::new(lower, upper).unwrap(),
                    (lower..=upper).collect(),
                ));
            }
        }
        for (interval, points) in &cases {
            assert_eq!(interval.is_empty(), points.is_empty());
            assert_eq!(interval.is_singleton(), points.len() == 1);
            assert_eq!(interval.lower(), points.first().copied());
            assert_eq!(interval.upper(), points.last().copied());
            for point in -4..=4 {
                assert_eq!(interval.contains_point(point), points.contains(&point));
            }
            let signs = points.iter().fold(SignSet::Empty, |set, value| {
                set.union(match value.cmp(&0) {
                    core::cmp::Ordering::Less => SignSet::Negative,
                    core::cmp::Ordering::Equal => SignSet::Zero,
                    core::cmp::Ordering::Greater => SignSet::Positive,
                })
            });
            assert_eq!(interval.signs(), signs);
            for (other, other_points) in &cases {
                assert_eq!(interval.subset_of(other), points.is_subset(other_points));
                assert_eq!(interval.disjoint(other), points.is_disjoint(other_points));
                assert_eq!(interval.overlaps(other), !points.is_disjoint(other_points));
                let common: BTreeSet<_> = points.intersection(other_points).copied().collect();
                let union: BTreeSet<_> = points.union(other_points).copied().collect();
                let intersection = interval.intersection(other);
                let hull = interval.hull(other);
                assert_eq!(intersection.lower(), common.first().copied());
                assert_eq!(intersection.upper(), common.last().copied());
                assert_eq!(hull.lower(), union.first().copied());
                assert_eq!(hull.upper(), union.last().copied());
                assert_eq!(intersection, other.intersection(interval));
                assert_eq!(hull, other.hull(interval));
                assert!(intersection.subset_of(interval));
                assert!(interval.subset_of(&hull));
                let left = !points.is_empty()
                    && !other_points.is_empty()
                    && points.iter().all(|a| other_points.iter().all(|b| a < b));
                assert_eq!(interval.strictly_left_of(other), left);
                let interior: BTreeSet<_> = other_points
                    .iter()
                    .copied()
                    .filter(|v| Some(v) != other_points.first() && Some(v) != other_points.last())
                    .collect();
                assert_eq!(
                    interval.interior_subset_of(other),
                    points.is_subset(&interior)
                );
                for (third, _) in &cases {
                    if interval.subset_of(other) && other.subset_of(third) {
                        assert!(interval.subset_of(third));
                    }
                }
            }
        }
    }

    #[test]
    fn checked_ingress_and_extremes_for_every_width() {
        macro_rules! check {
            ($($t:ty),*) => { $( {
                let full = IntegerRange::<$t>::new(<$t>::MIN, <$t>::MAX).unwrap();
                assert!(full.contains_point(<$t>::MIN));
                assert!(full.contains_point(<$t>::MAX));
                assert_eq!(IntegerRange::<$t>::new(<$t>::MAX, <$t>::MIN), Err(IntervalError::ReversedEndpoints));
                assert!(IntegerRange::<$t>::point(0).is_singleton());
                assert!(IntegerRange::<$t>::empty().is_empty());
            } )* };
        }
        check!(
            i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialization_validates_and_roundtrips() {
        for input in ["null", "[-8,7]", "[0,0]", "[-128,127]"] {
            let value: IntegerRange<i8> = serde_json::from_str(input).unwrap();
            let encoded = serde_json::to_string(&value).unwrap();
            assert_eq!(encoded, input);
            assert_eq!(
                serde_json::from_str::<IntegerRange<i8>>(&encoded).unwrap(),
                value
            );
        }
        for invalid in [
            "[1,0]", "[-129,0]", "[0,128]", "[0]", "[0,1,2]", "[null,1]", "[0.5,1]",
        ] {
            assert!(
                serde_json::from_str::<IntegerRange<i8>>(invalid).is_err(),
                "{invalid}"
            );
        }
    }
}

#[cfg(feature = "fixed")]
mod fixed {
    use hyperinterval::{FixedInterval, IntervalError, SignSet};

    #[test]
    fn continuous_half_lattice_samples_and_composition() {
        type I = FixedInterval<i32, 30>;
        for lower in -3..=3 {
            for upper in lower..=3 {
                let a = I::new(lower, upper).unwrap();
                assert_eq!(a.bounds(), Some((lower, upper)));
                assert_eq!(a.is_singleton(), lower == upper);
                assert_eq!(a.contains_point(0), lower <= 0 && 0 <= upper);
                for c in -3..=3 {
                    for d in c..=3 {
                        let b = I::new(c, d).unwrap();
                        let common = a.intersection(&b);
                        let hull = a.hull(&b);
                        for half_bit in -7..=7 {
                            let in_a = 2 * lower <= half_bit && half_bit <= 2 * upper;
                            let in_b = 2 * c <= half_bit && half_bit <= 2 * d;
                            let in_common = common
                                .bounds()
                                .is_some_and(|(l, u)| 2 * l <= half_bit && half_bit <= 2 * u);
                            assert_eq!(in_common, in_a && in_b);
                            if in_a || in_b {
                                let (l, u) = hull.bounds().unwrap();
                                assert!(2 * l <= half_bit && half_bit <= 2 * u);
                            }
                        }
                        assert_eq!(a.intersection(&b), b.intersection(&a));
                        assert_eq!(a.hull(&b), b.hull(&a));
                        assert!(common.subset_of(&a));
                        assert!(a.subset_of(&hull));
                    }
                }
                assert_eq!(a.hull(&I::empty().unwrap()), a);
                assert!(a.intersection(&I::empty().unwrap()).is_empty());
            }
        }
        assert_eq!(I::new(-1, 1).unwrap().signs(), SignSet::Mixed);
        assert_eq!(I::point(0).unwrap().signs(), SignSet::Zero);
        assert_eq!(I::empty().unwrap().signs(), SignSet::Empty);
    }

    #[test]
    fn every_constructor_checks_scale_and_order() {
        type Invalid = FixedInterval<i32, 32>;
        assert_eq!(Invalid::new(0, 1), Err(IntervalError::InvalidScale));
        assert_eq!(Invalid::point(0), Err(IntervalError::InvalidScale));
        assert_eq!(Invalid::empty(), Err(IntervalError::InvalidScale));
        assert_eq!(
            FixedInterval::<i64, 64>::empty(),
            Err(IntervalError::InvalidScale)
        );
        assert_eq!(
            FixedInterval::<i64, { u32::MAX }>::point(0),
            Err(IntervalError::InvalidScale)
        );
        assert!(FixedInterval::<i64, 63>::new(i64::MIN, i64::MAX).is_ok());
        assert!(FixedInterval::<i32, 31>::new(i32::MIN, i32::MAX).is_ok());
        assert!(FixedInterval::<i32, 0>::new(i32::MIN, i32::MAX).is_ok());
        assert_eq!(
            FixedInterval::<i32, 30>::new(1, 0),
            Err(IntervalError::ReversedEndpoints)
        );
    }

    #[test]
    fn storage_is_copy_without_hidden_allocation() {
        fn is_copy<T: Copy>() {}
        is_copy::<FixedInterval<i32, 30>>();
        is_copy::<FixedInterval<i64, 32>>();
        assert_eq!(core::mem::size_of::<FixedInterval<i32, 30>>(), 12);
        let alignment = core::mem::align_of::<i64>();
        assert_eq!(
            core::mem::size_of::<FixedInterval<i64, 32>>(),
            16 + alignment.max(4)
        );
        let a = FixedInterval::<i32, 30>::new(-2, 3).unwrap();
        let copy = a;
        let changed = copy.intersection(&FixedInterval::point(0).unwrap());
        assert_eq!(a.bounds(), Some((-2, 3)));
        assert_eq!(changed.bounds(), Some((0, 0)));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_cannot_forge_scale_or_bounds() {
        type I = FixedInterval<i32, 30>;
        for input in ["null", "[-2,3]", "[0,0]"] {
            let value: I = serde_json::from_str(input).unwrap();
            assert_eq!(serde_json::to_string(&value).unwrap(), input);
        }
        for invalid in ["[1,0]", "[0]", "[0,1,2]", "[0,2147483648]"] {
            assert!(serde_json::from_str::<I>(invalid).is_err());
        }
        for input in ["null", "[0,0]"] {
            assert!(serde_json::from_str::<FixedInterval<i32, 32>>(input).is_err());
        }
    }
}
