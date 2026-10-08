use hyperreal::{Computable, Rational};
use rug::{Float, Integer, Rational as Q, float::Round};
use std::{alloc::{GlobalAlloc, Layout, System}, sync::atomic::{AtomicU64, Ordering}, time::Instant};

struct Counted;
static CALLS: AtomicU64 = AtomicU64::new(0);
static BYTES: AtomicU64 = AtomicU64::new(0);
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 { CALLS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(l.size() as u64, Ordering::Relaxed); unsafe { System.alloc(l) } }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 { CALLS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(l.size() as u64, Ordering::Relaxed); unsafe { System.alloc_zeroed(l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 { CALLS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(n as u64, Ordering::Relaxed); unsafe { System.realloc(p, l, n) } }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { unsafe { System.dealloc(p, l) } }
}
#[global_allocator] static ALLOCATOR: Counted = Counted;
#[repr(C)] struct Timespec { sec: i64, nsec: i64 }
unsafe extern "C" { fn clock_gettime(id: i32, t: *mut Timespec) -> i32; }
fn cpu() -> u64 { let mut t = Timespec { sec: 0, nsec: 0 }; assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0); t.sec as u64 * 1_000_000_000 + t.nsec as u64 }
fn c(q: &Q) -> Computable { Computable::rational(q.to_string().parse::<Rational>().unwrap()) }
fn make(family: &str, k: usize, precision: u32) -> (Computable, (Q, Q)) {
    let sign = if k % 2 == 0 { 1 } else { -1 };
    match family {
        "tiny" => {
            let q = Q::from((sign, [16, 32, 64, 256][k % 4]));
            (c(&q), (q.clone(), q))
        },
        "large" => {
            let q = Q::from((sign * [1, 2, 3, 7][k % 4], 8));
            (c(&q), (q.clone(), q))
        },
        "radical" => {
            let n = [2, 3, 5, 7][k % 4];
            let mut lo = Float::with_val(precision, n); let mut hi = lo.clone();
            lo.sqrt_round(Round::Down); hi.sqrt_round(Round::Up);
            let (lo, hi) = (lo.to_rational().unwrap(), hi.to_rational().unwrap());
            let bounds = if sign < 0 { (-hi / 64, -lo / 64) } else { (lo / 64, hi / 64) };
            (c(&Q::from(n)).sqrt().multiply(c(&Q::from((sign, 64)))), bounds)
        },
        "opaque" => {
            let n = [4, 6, 10, 100][k % 4];
            let mut lo = Float::with_val(precision, n); let mut hi = lo.clone();
            lo.sin_round(Round::Down); hi.sin_round(Round::Up);
            (c(&Q::from(n)).sin().multiply(c(&Q::from((1, 64)))), (lo.to_rational().unwrap() / 64, hi.to_rational().unwrap() / 64))
        },
        "high" => { let q = Q::from((1, 16)); (c(&q), (q.clone(), q)) },
        _ => panic!("unknown family"),
    }
}
fn truth(op: &str, (lo, hi): (Q, Q), precision: u32) -> (Q, Q) {
    assert!(lo > -1 && hi < 1 && lo <= hi);
    let mut lo = Float::with_val_round(precision, lo, Round::Down).0;
    let mut hi = Float::with_val_round(precision, hi, Round::Up).0;
    match op {
        "asin" => { lo.asin_round(Round::Down); hi.asin_round(Round::Up); },
        "asinh" => { lo.asinh_round(Round::Down); hi.asinh_round(Round::Up); },
        "atanh" => { lo.atanh_round(Round::Down); hi.atanh_round(Round::Up); },
        _ => panic!("op"),
    }
    (lo.to_rational().unwrap(), hi.to_rational().unwrap())
}
fn main() {
    let args: Vec<_> = std::env::args().collect(); let op = &args[1]; let family = &args[2];
    let count = if family == "high" { 1 } else { 256 };
    let bits: Vec<i32> = (0..count).map(|k| if family == "high" { 184_000 } else { [32, 128][k % 2] }).collect();
    let precision = (*bits.iter().max().unwrap() as u32 + 256).max(4096);
    let inputs: Vec<_> = (0..count).map(|k| make(family, k, precision)).collect();
    let refs: Vec<_> = inputs.iter().map(|(_, b)| truth(op, b.clone(), precision)).collect();
    let mut outputs = Vec::with_capacity(inputs.len());
    let calls = CALLS.load(Ordering::Relaxed); let bytes = BYTES.load(Ordering::Relaxed); let start = Instant::now(); let t = cpu();
    for ((input, _), bits) in inputs.iter().zip(&bits) {
        let value = match op.as_str() { "asin" => input.clone().asin(), "asinh" => input.clone().asinh(), "atanh" => input.clone().atanh(), _ => panic!("op") };
        outputs.push(std::hint::black_box(value.approx(-bits)));
    }
    let elapsed = cpu() - t; let wall = start.elapsed().as_nanos();
    let calls = CALLS.load(Ordering::Relaxed) - calls; let bytes = BYTES.load(Ordering::Relaxed) - bytes;
    let mut checksum = 0_u64;
    for (k, ((a, (lo, hi)), bits)) in outputs.iter().zip(refs).zip(&bits).enumerate() {
        let s = a.to_string(); let scale = Integer::from(1) << bits;
        let center = Q::from((Integer::from_str_radix(&s, 10).unwrap(), scale.clone())); let radius = Q::from((1, scale));
        assert!(Q::from(&center - &radius) <= lo && Q::from(&center + &radius) >= hi, "non-enclosing {op} {family} {k}");
        for b in s.bytes() { checksum = checksum.wrapping_mul(131).wrapping_add(u64::from(b)); }
    }
    println!("{op}\t{family}\t{count}\t{elapsed}\t{wall}\t{calls}\t{bytes}\t{checksum}");
}
