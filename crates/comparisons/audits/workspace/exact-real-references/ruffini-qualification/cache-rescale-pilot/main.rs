use num::{BigInt, Integer, One, Signed, Zero};
use std::{hint::black_box, sync::LazyLock, time::Instant};

#[cfg(feature = "live-allocations")]
mod allocation;

static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);

struct Cached {
    precision: i32,
    value: BigInt,
}

type Rescale = fn(&Cached, i32) -> Option<BigInt>;

// Copies of the current production helper and its scale/shift dependencies.
fn shift(n: BigInt, p: i32) -> BigInt {
    match 0.cmp(&p) {
        std::cmp::Ordering::Greater => n >> -p,
        std::cmp::Ordering::Equal => n,
        std::cmp::Ordering::Less => n << p,
    }
}

fn scale(n: BigInt, p: i32) -> BigInt {
    if p >= 0 {
        n << p
    } else {
        (shift(n, p + 1) + &*ONE) >> 1
    }
}

#[inline(always)]
fn original(c: &Cached, p: i32) -> Option<BigInt> {
    if p < c.precision {
        None
    } else if p == c.precision {
        Some(c.value.clone())
    } else {
        Some(scale(c.value.clone(), c.precision - p))
    }
}

#[inline(always)]
fn wide(c: &Cached, p: i32) -> Option<BigInt> {
    if p < c.precision {
        None
    } else if p == c.precision {
        Some(c.value.clone())
    } else {
        let gap = (i64::from(p) - i64::from(c.precision)) as u32;
        Some(((c.value.clone() >> (gap - 1)) + &*ONE) >> 1)
    }
}

#[inline(always)]
fn borrowed(c: &Cached, p: i32) -> Option<BigInt> {
    if p < c.precision {
        None
    } else if p == c.precision {
        Some(c.value.clone())
    } else {
        let gap = (i64::from(p) - i64::from(c.precision)) as u32;
        Some(((&c.value >> (gap - 1)) + &*ONE) >> 1)
    }
}

#[inline(always)]
fn guarded(c: &Cached, p: i32) -> Option<BigInt> {
    if p < c.precision {
        None
    } else if p == c.precision {
        Some(c.value.clone())
    } else {
        let gap = (i64::from(p) - i64::from(c.precision)) as u32;
        if u64::from(gap) > c.value.bits() {
            Some(BigInt::zero())
        } else {
            Some(((&c.value >> (gap - 1)) + &*ONE) >> 1)
        }
    }
}

fn method(name: &str) -> Rescale {
    match name {
        "original" => original,
        "wide" => wide,
        "borrowed" => borrowed,
        "guarded" => guarded,
        _ => panic!("invalid method"),
    }
}

// Independent quotient/remainder oracle: unlike the implementation, this uses
// signed Euclidean division and an explicit comparison against the halfway point.
fn oracle(n: &BigInt, gap: u32) -> BigInt {
    if gap == 0 {
        return n.clone();
    }
    if gap > 2_000_000 {
        // All inputs are at most 1,048,576 bits: this is a strict
        // magnitude proof for the large precision gaps, not a huge allocation.
        assert!(n.bits() < u64::from(gap));
        return BigInt::zero();
    }
    let divisor = BigInt::one() << gap;
    let (q, r) = n.div_mod_floor(&divisor);
    if &r * 2 >= divisor { q + 1 } else { q }
}

fn check(c: &Cached, p: i32, counts: &mut [u64; 4]) {
    let expected = if p < c.precision {
        None
    } else {
        Some(oracle(&c.value, (i64::from(p) - i64::from(c.precision)) as u32))
    };
    for (i, name) in ["original", "wide", "borrowed", "guarded"].iter().enumerate() {
        // Never execute the release baseline's overflowing subtraction: it can
        // wrap into an enormous left shift. This skip is reported explicitly.
        if i == 0 && c.precision.checked_sub(p).is_none() && p > c.precision {
            continue;
        }
        assert_eq!(method(name)(c, p), expected, "{name}: q={}, p={p}, bits={}", c.precision, c.value.bits());
        counts[i] += 1;
    }
    // Exact enclosure endpoints: input true value is in [n-1,n+1].
    // Returned center m must enclose both after scaling by 2^gap.
    if let Some(m) = expected {
        let gap = (i64::from(p) - i64::from(c.precision)) as u32;
        if gap <= 1_000_000 {
            let divisor = BigInt::one() << gap;
            assert!((&m - 1) * &divisor <= &c.value - 1);
            assert!((&m + 1) * &divisor >= &c.value + 1);
        }
    }
}

fn patterned(bits: u32, negative: bool) -> BigInt {
    assert!(bits > 0);
    let mut state = 0xc408_7c6a_3192_7d5b_u64;
    let mut bytes = Vec::with_capacity(bits.div_ceil(8) as usize);
    for _ in 0..bits.div_ceil(8) {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        bytes.push(state as u8);
    }
    let last = bytes.last_mut().unwrap();
    *last &= (0xff_u16 >> ((8 - bits % 8) % 8)) as u8;
    *last |= 1 << ((bits - 1) % 8);
    BigInt::from_bytes_le(if negative { num::bigint::Sign::Minus } else { num::bigint::Sign::Plus }, &bytes)
}

fn verify() {
    let mut counts = [0_u64; 4];
    for n in -4096..=4096 {
        let c = Cached { precision: -16, value: BigInt::from(n) };
        for p in -17..=0 {
            check(&c, p, &mut counts);
        }
    }
    let mut wide_cases = 0;
    for bits in [1_u32, 2, 31, 32, 33, 63, 64, 65, 127, 128, 129, 1024, 4096, 65536] {
        let unit = BigInt::one() << bits;
        let mut gaps = vec![0, 1, 2, 31, 32, 33, 63, 64, 65, bits - 1, bits, bits + 1, bits + 2];
        gaps.sort_unstable();
        gaps.dedup();
        for n in [&unit - 1, unit.clone(), &unit + 1, &unit >> 1, patterned(bits, false)] {
            for value in [n.clone(), -n] {
                let c = Cached { precision: -100_000, value };
                for &gap in &gaps {
                    check(&c, c.precision + gap as i32, &mut counts);
                    wide_cases += 1;
                }
                for (q, p) in [(i32::MIN, i32::MAX), (-128, i32::MAX), (-1, i32::MAX), (i32::MIN, 0), (i32::MIN, i32::MIN + 1)] {
                    let c = Cached { precision: q, value: c.value.clone() };
                    check(&c, p, &mut counts);
                    wide_cases += 1;
                }
            }
        }
    }
    println!("{{\"mode\":\"verify\",\"counts\":{counts:?},\"wide_cases\":{wide_cases},\"baseline_overflow_skips\":{}}}", counts[1] - counts[0]);
}

#[repr(C)]
struct Timespec { sec: i64, nsec: i64 }
unsafe extern "C" { fn clock_gettime(id: i32, t: *mut Timespec) -> i32; }
fn cpu() -> u64 {
    let mut t = Timespec { sec: 0, nsec: 0 };
    assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0);
    t.sec as u64 * 1_000_000_000 + t.nsec as u64
}

fn bench(args: &[String]) {
    let name = &args[2];
    let bits: u32 = args[3].parse().unwrap();
    let gap: u32 = args[4].parse().unwrap();
    let negative = match args[5].as_str() { "positive" => false, "negative" => true, _ => panic!("sign") };
    let reps: u64 = args[6].parse().unwrap();
    assert!((1..=1_048_576).contains(&bits) && gap <= 2_000_000 && reps > 0);
    let c = Cached { precision: -(bits as i32), value: patterned(bits, negative) };
    let p = c.precision + gap as i32;
    let expected = oracle(&c.value, gap);
    let f = method(name);
    assert_eq!(f(&c, p).unwrap(), expected);
    for _ in 0..100 { black_box(f(black_box(&c), black_box(p))); }
    #[cfg(feature = "live-allocations")]
    let baseline = allocation::reset();
    let wall = Instant::now();
    let begin = cpu();
    let mut checksum = 0_u64;
    for _ in 0..reps {
        let result = black_box(f(black_box(&c), black_box(p)).unwrap());
        checksum = checksum.wrapping_add(result.bits() + u64::from(result.is_negative()));
    }
    let elapsed = cpu() - begin;
    let wall = wall.elapsed().as_nanos();
    #[cfg(feature = "live-allocations")]
    let memory = allocation::measure(baseline);
    assert_eq!(checksum, (expected.bits() + u64::from(expected.is_negative())).wrapping_mul(reps));
    assert_eq!(f(&c, p).unwrap(), expected);
    #[cfg(not(feature = "live-allocations"))]
    let memory = "null";
    println!("{{\"mode\":\"bench\",\"method\":\"{name}\",\"bits\":{bits},\"gap\":{gap},\"negative\":{negative},\"reps\":{reps},\"cpu_ns\":{elapsed},\"wall_ns\":{wall},\"checksum\":{checksum},\"memory\":{memory}}}");
}

fn public_boundary() {
    // A release baseline call with this gap is deliberately not executed.
    assert!(cfg!(debug_assertions), "public pre-repair boundary probe is debug-only");
    let input = hyperreal::Computable::rational(17.into()).sqrt();
    let fine = input.approx(-128);
    assert!(!fine.is_zero());
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| input.approx(i32::MAX)));
    match result {
        Ok(value) => { assert!(value.is_zero()); println!("{{\"mode\":\"public-boundary\",\"result\":\"zero\"}}"); }
        Err(_) => println!("{{\"mode\":\"public-boundary\",\"result\":\"panic\"}}"),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("verify") => verify(),
        Some("bench") => bench(&args),
        Some("public-boundary") => public_boundary(),
        _ => panic!("verify | bench METHOD BITS GAP SIGN REPS | public-boundary"),
    }
}
