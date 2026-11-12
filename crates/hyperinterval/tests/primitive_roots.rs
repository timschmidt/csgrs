#![cfg(any(feature = "fixed", feature = "float-enclosure"))]
#[cfg(any(feature = "fixed", target_arch = "x86_64"))]
use hyperinterval::{DomainCoverage, IntervalError};

#[cfg(feature = "fixed")]
#[test]
fn fixed_root_domain_and_extreme_scales() {
    use hyperinterval::FixedInterval as F;
    let result = F::<i32, 8>::new(-256, 1024).unwrap().sqrt().unwrap();
    assert_eq!(result.range.bounds(), Some((0, 512)));
    assert_eq!(result.domain, DomainCoverage::Partial);
    assert_eq!(
        F::<i64, 63>::point(i64::MAX).unwrap().sqrt(),
        Err(IntervalError::Overflow)
    );
    assert_eq!(
        F::<i32, 31>::point(i32::MAX).unwrap().sqrt(),
        Err(IntervalError::Overflow)
    );
    assert_eq!(
        F::<i64, 30>::new(-2, -1).unwrap().sqrt(),
        Err(IntervalError::EmptyDomain)
    );
    assert!(
        F::<i32, 30>::empty()
            .unwrap()
            .sqrt()
            .unwrap()
            .range
            .is_empty()
    );
}

#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
#[test]
fn ieee_root_domain_and_subnormals() {
    use hyperinterval::{F32Enclosure as F, F64Enclosure as D};
    let result = D::new(-1.0, 4.0).unwrap().sqrt().unwrap();
    assert_eq!(result.range.bounds(), Some((0.0, 2.0)));
    assert_eq!(result.domain, DomainCoverage::Partial);
    assert_eq!(
        F::new(-2.0, -1.0).unwrap().sqrt(),
        Err(IntervalError::EmptyDomain)
    );
    assert!(D::empty().sqrt().unwrap().range.is_empty());
    let result = D::point(f64::from_bits(1)).unwrap().sqrt().unwrap();
    assert_eq!(result.range.bounds().unwrap().0.to_bits(), (486u64) << 52);
    assert!(result.range.is_singleton());
}

#[cfg(all(
    feature = "float-enclosure",
    target_arch = "x86_64",
    not(target_env = "msvc")
))]
#[path = "support/primitive_sqrt_oracle.rs"]
mod oracle;

#[cfg(all(feature = "fixed", not(target_env = "msvc")))]
#[test]
fn fixed_all_scales_match_independent_gmp_tightness() {
    macro_rules! run {
        ($ty:ty, $($frac:literal),*) => {$({
            for a in -8..=16 {
                for b in a..=16 {
                    let actual = hyperinterval::FixedInterval::<$ty, $frac>::new(a, b).unwrap().sqrt()
                        .map(|x| {let (a,b)=x.range.bounds().unwrap(); ((i64::from(a), i64::from(b)),x.domain)});
                    fixed_oracle::fixed(i64::from(a),i64::from(b),$frac,i64::from(<$ty>::MAX),actual);
                }
            }
            for a in [<$ty>::MIN, -1, 0, 1, <$ty>::MAX / 2, <$ty>::MAX - 1, <$ty>::MAX] {
                let actual = hyperinterval::FixedInterval::<$ty, $frac>::point(a).unwrap().sqrt()
                    .map(|x| {let (a,b)=x.range.bounds().unwrap(); ((i64::from(a), i64::from(b)),x.domain)});
                fixed_oracle::fixed(i64::from(a),i64::from(a),$frac,i64::from(<$ty>::MAX),actual);
            }
        })*};
    }
    run!(
        i32, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29, 30, 31
    );
    run!(
        i64, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46,
        47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
    );
}

#[cfg(all(
    feature = "float-enclosure",
    target_arch = "x86_64",
    not(target_env = "msvc")
))]
#[test]
fn ieee_all_exponents_and_random_images_match_directed_mpfr() {
    macro_rules! run {
        ($ty:ty, $bits:ty, $interval:ident, $root:ident, $f:literal, $maximum:literal) => {{
            use hyperinterval::$interval;
            let check = |value: $ty| {
                let x = $interval::point(value).unwrap().sqrt().unwrap();
                let (a, b) = x.range.bounds().unwrap();
                assert_eq!((a.to_bits(), b.to_bits()), oracle::$root(value).unwrap());
                assert_eq!(x.domain, DomainCoverage::Complete);
            };
            for exponent in 0..$maximum {
                for tail in [
                    0,
                    1,
                    (1 << ($f - 1)) - 1,
                    1 << ($f - 1),
                    (1 << $f) - 2,
                    (1 << $f) - 1,
                ] {
                    check(<$ty>::from_bits((exponent << $f) | tail));
                }
            }
            let mut state = 0x5f84_0c2a_771b_903du64;
            for _ in 0..20_000 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let bits = (state as $bits) & (<$bits>::MAX >> 1);
                if bits >= ($maximum << $f) {
                    continue;
                }
                let value = <$ty>::from_bits(bits);
                check(value);
                let range = $interval::new(-value, value).unwrap().sqrt().unwrap();
                let (lower, upper) = range.range.bounds().unwrap();
                assert_eq!(lower.to_bits(), 0);
                assert_eq!(upper.to_bits(), oracle::$root(value).unwrap().1);
                assert_eq!(
                    range.domain,
                    if bits == 0 {
                        DomainCoverage::Complete
                    } else {
                        DomainCoverage::Partial
                    }
                );
            }
        }};
    }
    run!(f32, u32, F32Enclosure, sqrt_f32, 23, 255u32);
    run!(f64, u64, F64Enclosure, sqrt_f64, 52, 2047u64);
}

#[cfg(all(feature = "fixed", feature = "serde"))]
#[test]
fn fixed_root_replay_and_further_operation_enclose_source() {
    use hyperinterval::FixedInterval;
    let source = FixedInterval::<i64, 30>::new(1, 3 << 30).unwrap();
    let result = source.sqrt().unwrap();
    let bytes = serde_json::to_vec(&result.range).unwrap();
    let restored: FixedInterval<i64, 30> = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(restored, result.range);
    assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
    assert!(source.subset_of(&restored.square().unwrap()));
    let root = restored.sqrt().unwrap();
    assert!(source.subset_of(&root.range.checked_pow(4).unwrap()));
}

#[cfg(all(feature = "fixed", not(target_env = "msvc")))]
#[path = "support/fixed_sqrt_oracle.rs"]
mod fixed_oracle;

#[cfg(all(feature = "float-enclosure", feature = "serde", target_arch = "x86_64"))]
#[test]
fn ieee_root_replay_and_composition_keep_source_enclosed() {
    macro_rules! run {
        ($interval:ident) => {{
            use hyperinterval::$interval;
            let source = $interval::new(0.25, 7.0).unwrap();
            let result = source.sqrt().unwrap();
            let bytes = serde_json::to_vec(&result.range).unwrap();
            let restored: $interval = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(restored, result.range);
            assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
            assert!(source.subset_of(&restored.square().unwrap()));
            let root = restored.sqrt().unwrap().range;
            assert!(source.subset_of(&root.checked_pow(4).unwrap()));
            assert!(
                restored
                    .mul(&restored.reciprocal().unwrap())
                    .unwrap()
                    .contains_point(1.0)
                    .unwrap()
            );
        }};
    }
    run!(F32Enclosure);
    run!(F64Enclosure);
}
