use hyperreal::{Computable, Rational};
use num::{BigInt, One, Signed, Zero};
use std::{hint::black_box, time::Instant};

#[cfg(feature = "live-allocations")]
#[path = "../cache-rescale-pilot/allocation.rs"]
mod allocation;

fn input(family: &str, negative: bool) -> Computable {
    let value = match family {
        "root" => Computable::rational(17.into()).sqrt(),
        "rational" => Computable::rational(Rational::fraction(1, 3).unwrap()),
        _ => panic!("family"),
    };
    if negative { value.negate() } else { value }
}

fn check(family: &str, negative: bool, p: i32, result: &BigInt) {
    if p >= 8 {
        // Both input magnitudes are strictly below 256. A zero result encloses
        // either sign for every p>=8 without allocating a giant power of two.
        assert!(result.is_zero());
        return;
    }
    let (mut lo, mut hi, scale): (BigInt, BigInt, BigInt) = if p >= 0 {
        ((result - 1) << p, (result + 1) << p, BigInt::one())
    } else {
        (result - 1, result + 1, BigInt::one() << -p)
    };
    if negative { (lo, hi) = (-hi, -lo); }
    match family {
        "root" => {
            let target = &scale * &scale * 17;
            assert!(hi >= BigInt::zero() && &hi * &hi >= target);
            assert!(lo <= BigInt::zero() || &lo * &lo <= target);
        }
        "rational" => { assert!(&lo * 3 <= scale && &hi * 3 >= scale); }
        _ => panic!("family"),
    }
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
    let family = &args[2];
    let negative = match args[3].as_str() { "positive" => false, "negative" => true, _ => panic!("sign") };
    let mode = &args[4];
    let bits: i32 = args[5].parse().unwrap();
    let gap: i32 = args[6].parse().unwrap();
    let reps: u64 = args[7].parse().unwrap();
    assert!((1..=1_048_576).contains(&bits) && gap >= 0 && gap <= bits + 8 && reps > 0);
    assert!(mode == "warm" || mode == "cold");
    let p = -bits + gap;
    let value = input(family, negative);
    if mode == "warm" { check(family, negative, -bits, &value.approx(-bits)); }
    let expected = value.approx(p);
    check(family, negative, p, &expected);
    for _ in 0..100 { black_box(value.approx(p)); }
    #[cfg(feature = "live-allocations")]
    let baseline = allocation::reset();
    let wall = Instant::now();
    let start = cpu();
    let mut checksum = 0_u64;
    for _ in 0..reps {
        let result = if mode == "warm" { value.approx(black_box(p)) } else { input(black_box(family), black_box(negative)).approx(black_box(p)) };
        let result = black_box(result);
        checksum = checksum.wrapping_add(result.bits() + u64::from(result.is_negative()));
    }
    let elapsed = cpu() - start;
    let wall = wall.elapsed().as_nanos();
    #[cfg(feature = "live-allocations")]
    let memory = allocation::measure(baseline);
    #[cfg(not(feature = "live-allocations"))]
    let memory = "null";
    assert_eq!(checksum, (expected.bits() + u64::from(expected.is_negative())).wrapping_mul(reps));
    check(family, negative, p, &value.approx(p));
    println!("{{\"family\":\"{family}\",\"negative\":{negative},\"mode\":\"{mode}\",\"bits\":{bits},\"gap\":{gap},\"reps\":{reps},\"cpu_ns\":{elapsed},\"wall_ns\":{wall},\"checksum\":{checksum},\"expected\":\"{expected}\",\"memory\":{memory}}}");
}

fn verify(extreme: bool) {
    let mut checked = 0;
    for family in ["root", "rational"] {
        for negative in [false, true] {
            for bits in [16, 64, 128, 4096, 65536] {
                let value = input(family, negative);
                let mut requests = vec![-bits, -bits + 1, -bits / 2, 0, 8, -bits - 32, -bits + 2, -16];
                if extreme { requests.push(i32::MAX); }
                for p in requests { check(family, negative, p, &value.approx(p)); checked += 1; }
            }
        }
    }
    println!("{{\"mode\":\"verify\",\"extreme\":{extreme},\"checked\":{checked}}}");
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("bench") => bench(&args),
        Some("verify") => verify(false),
        Some("verify-extreme") => verify(true),
        _ => panic!("bench FAMILY SIGN MODE BITS GAP REPS | verify | verify-extreme"),
    }
}
